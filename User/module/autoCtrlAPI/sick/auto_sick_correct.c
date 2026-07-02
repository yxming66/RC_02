#include "module/autoCtrlAPI/sick/auto_sick_correct.h"

#include <math.h>
#include <string.h>

#define AUTO_SICK_CORRECT_DEFAULT_FINISH_STABLE_MS (120u)
#define AUTO_SICK_CORRECT_DEFAULT_TIMEOUT_MS (5000u)
#define AUTO_SICK_CORRECT_DEFAULT_POLE_TARGET_LIFT (2.0f)
#define AUTO_SICK_CORRECT_ROD_SPEARHEAD_TIMEOUT_SUCCESS_TOLERANCE_ADC (30.0f)
#define AUTO_SICK_CORRECT_ROD_SPEARHEAD_ENABLE_X (0u)
#define AUTO_SICK_CORRECT_ROD_SPEARHEAD_ENABLE_Y (0u)

static float AutoSickCorrect_AbsFloat(float value) {
  return (value >= 0.0f) ? value : -value;
}

static float AutoSickCorrect_Clamp(float value, float limit_abs) {
  if (limit_abs <= 0.0f) {
    return 0.0f;
  }
  if (value > limit_abs) {
    return limit_abs;
  }
  if (value < -limit_abs) {
    return -limit_abs;
  }
  return value;
}

static bool AutoSickCorrect_IndexValid(uint8_t index) {
  return index < AUTO_SICK_CORRECT_SENSOR_COUNT;
}

static bool AutoSickCorrect_ActionUsesSingleXSensor(
    AutoSickCorrect_Action_t action) {
  return action == AUTO_SICK_CORRECT_ACTION_ORE_RELEASE;
}

static bool AutoSickCorrect_ActionUsesOnlyYSensors(
    AutoSickCorrect_Action_t action) {
  return false;
}

static bool AutoSickCorrect_ActionIsRodSpearheadPosition(
    AutoSickCorrect_Action_t action) {
  return action >= AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS1 &&
         action <= AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS6;
}

static bool AutoSickCorrect_ActionUsesRodSpearheadSamples(
    AutoSickCorrect_Action_t action) {
  return AutoSickCorrect_ActionIsRodSpearheadPosition(action);
}

static bool AutoSickCorrect_RodSpearheadXEnabled(void) {
  return AUTO_SICK_CORRECT_ROD_SPEARHEAD_ENABLE_X != 0u;
}

static bool AutoSickCorrect_RodSpearheadYEnabled(void) {
  return AUTO_SICK_CORRECT_ROD_SPEARHEAD_ENABLE_Y != 0u;
}

static bool AutoSickCorrect_RodSpearheadAnyAxisEnabled(void) {
  return AutoSickCorrect_RodSpearheadXEnabled() ||
         AutoSickCorrect_RodSpearheadYEnabled();
}

static uint8_t AutoSickCorrect_RodSpearheadPositionIndex(
    AutoSickCorrect_Action_t action) {
  return (uint8_t)(action - AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS1);
}

static uint32_t AutoSickCorrect_FinishStableMs(
    const AutoSickCorrect_PointParams_t *param) {
  return (param != 0 && param->finish_stable_ms > 0u)
             ? param->finish_stable_ms
             : AUTO_SICK_CORRECT_DEFAULT_FINISH_STABLE_MS;
}

static uint32_t AutoSickCorrect_TimeoutMs(
    const AutoSickCorrect_PointParams_t *param) {
  return (param != 0 && param->timeout_ms > 0u)
             ? param->timeout_ms
             : AUTO_SICK_CORRECT_DEFAULT_TIMEOUT_MS;
}

static float AutoSickCorrect_PoleTargetLift(
    const AutoSickCorrect_PointParams_t *param) {
  return (param != 0 && isfinite(param->pole_target_lift) &&
          param->pole_target_lift > 0.0f)
             ? param->pole_target_lift
             : AUTO_SICK_CORRECT_DEFAULT_POLE_TARGET_LIFT;
}

static bool AutoSickCorrect_PointParamValid(
    const AutoSickCorrect_PointParams_t *param,
    AutoSickCorrect_Action_t action) {
  if (param == 0) {
    return false;
  }

  if (param->valid_adc_min > param->valid_adc_max) {
    return false;
  }

  if (AutoSickCorrect_ActionUsesOnlyYSensors(action)) {
    return AutoSickCorrect_IndexValid(param->rod_rear_index) &&
           isfinite(param->y_target_adc) && param->y_target_adc > 0.0f &&
           isfinite(param->y_tolerance_adc) &&
               param->y_tolerance_adc > 0.0f &&
           isfinite(param->y_kp_mps_per_adc) &&
           isfinite(param->vy_limit_mps) && param->vy_limit_mps > 0.0f &&
           isfinite(param->pole_speed);
  }

  if (AutoSickCorrect_ActionUsesRodSpearheadSamples(action)) {
    const bool x_enabled = AutoSickCorrect_RodSpearheadXEnabled();
    const bool y_enabled = AutoSickCorrect_RodSpearheadYEnabled();
    if (!x_enabled && !y_enabled) {
      return isfinite(param->pole_speed);
    }
    return (!x_enabled ||
            (AutoSickCorrect_IndexValid(param->front_index) &&
             isfinite(param->x_target_adc) && param->x_target_adc > 0.0f &&
             isfinite(param->x_tolerance_adc) &&
                 param->x_tolerance_adc > 0.0f &&
             isfinite(param->x_kp_mps_per_adc) &&
             isfinite(param->vx_limit_mps) && param->vx_limit_mps > 0.0f)) &&
           (!y_enabled ||
            (AutoSickCorrect_IndexValid(param->rod_rear_index) &&
             isfinite(param->y_target_adc) && param->y_target_adc > 0.0f &&
             isfinite(param->y_tolerance_adc) &&
                 param->y_tolerance_adc > 0.0f &&
             isfinite(param->y_kp_mps_per_adc) &&
             isfinite(param->vy_limit_mps) && param->vy_limit_mps > 0.0f)) &&
           isfinite(param->pole_speed);
  }

  if (AutoSickCorrect_ActionUsesSingleXSensor(action)) {
    return AutoSickCorrect_IndexValid(param->front_index) &&
         isfinite(param->x_target_adc) && param->x_target_adc > 0.0f &&
           isfinite(param->x_tolerance_adc) && param->x_tolerance_adc > 0.0f &&
           isfinite(param->x_kp_mps_per_adc) &&
           isfinite(param->vx_limit_mps) && param->vx_limit_mps > 0.0f &&
           isfinite(param->pole_speed);
  }

  if (!AutoSickCorrect_IndexValid(param->front_index) ||
      !AutoSickCorrect_IndexValid(param->rod_front_index) ||
      !AutoSickCorrect_IndexValid(param->rear_index) ||
      !AutoSickCorrect_IndexValid(param->rod_rear_index)) {
    return false;
  }

  return isfinite(param->x_target_adc) && param->x_target_adc > 0.0f &&
         isfinite(param->y_target_adc) && param->y_target_adc > 0.0f &&
         isfinite(param->yaw_target_diff_adc) &&
         isfinite(param->x_tolerance_adc) && param->x_tolerance_adc > 0.0f &&
         isfinite(param->y_tolerance_adc) && param->y_tolerance_adc > 0.0f &&
         isfinite(param->yaw_tolerance_adc) &&
             param->yaw_tolerance_adc > 0.0f &&
         isfinite(param->x_kp_mps_per_adc) &&
         isfinite(param->y_kp_mps_per_adc) &&
         isfinite(param->yaw_kp_rad_s_per_adc) &&
         isfinite(param->vx_limit_mps) && param->vx_limit_mps > 0.0f &&
         isfinite(param->vy_limit_mps) && param->vy_limit_mps > 0.0f &&
         isfinite(param->wz_limit_rad_s) && param->wz_limit_rad_s > 0.0f &&
         isfinite(param->pole_speed);
}

static const AutoSickCorrect_PointParams_t *AutoSickCorrect_ActiveParams(
    const AutoSickCorrect_t *ctrl) {
  if (ctrl == 0) {
    return 0;
  }

  switch (ctrl->action) {
    case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS1:
    case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS2:
    case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS3:
    case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS4:
    case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS5:
    case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS6:
      return &ctrl->param.rod_spearhead_position[
          AutoSickCorrect_RodSpearheadPositionIndex(ctrl->action)];
    case AUTO_SICK_CORRECT_ACTION_ORE_RELEASE:
      return &ctrl->param.ore_release;
    case AUTO_SICK_CORRECT_ACTION_NONE:
    default:
      return 0;
  }
}

static void AutoSickCorrect_ClearOutputs(AutoSickCorrect_t *ctrl) {
  if (ctrl == 0) {
    return;
  }
  memset(&ctrl->chassis_cmd, 0, sizeof(ctrl->chassis_cmd));
  ctrl->chassis_cmd.mode = CHASSIS_MODE_RELAX;
  memset(&ctrl->pole_cmd, 0, sizeof(ctrl->pole_cmd));
  ctrl->pole_cmd.mode = POLE_MODE_RELAX;
  ctrl->chassis_cmd_valid = false;
  ctrl->pole_cmd_valid = false;
}

static void AutoSickCorrect_CommandPole(
    AutoSickCorrect_t *ctrl, const AutoSickCorrect_PointParams_t *param) {
  const float target_lift = AutoSickCorrect_PoleTargetLift(param);

  memset(&ctrl->pole_cmd, 0, sizeof(ctrl->pole_cmd));
  ctrl->pole_cmd.mode = POLE_MODE_ACTIVE;
  ctrl->pole_cmd.auto_target_enable[0] = true;
  ctrl->pole_cmd.auto_target_enable[1] = true;
  ctrl->pole_cmd.auto_target_lift[0] = target_lift;
  ctrl->pole_cmd.auto_target_lift[1] = target_lift;
  ctrl->pole_cmd.auto_lift_speed[0] = param->pole_speed;
  ctrl->pole_cmd.auto_lift_speed[1] = param->pole_speed;
  ctrl->pole_cmd.auto_lift_accel[0] = 0.0f;
  ctrl->pole_cmd.auto_lift_accel[1] = 0.0f;
  ctrl->pole_cmd.disable_lift_accel = false;
  ctrl->pole_cmd_valid = true;
}

static void AutoSickCorrect_CommandChassis(AutoSickCorrect_t *ctrl,
                                           float vx_mps, float vy_mps,
                                           float wz_rad_s) {
  memset(&ctrl->chassis_cmd, 0, sizeof(ctrl->chassis_cmd));
  ctrl->chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
  ctrl->chassis_cmd.ctrl_vec.vx = vx_mps;
  ctrl->chassis_cmd.ctrl_vec.vy = vy_mps;
  ctrl->chassis_cmd.ctrl_vec.wz = wz_rad_s;
  ctrl->chassis_cmd_valid = true;
}

static void AutoSickCorrect_Finish(AutoSickCorrect_t *ctrl,
                                   AutoSickCorrect_Result_t result,
                                   AutoSickCorrect_Fault_t fault,
                                   AutoSickCorrect_State_t state) {
  ctrl->result = result;
  ctrl->fault = fault;
  ctrl->state = state;
  AutoSickCorrect_ClearOutputs(ctrl);
}

static bool AutoSickCorrect_FeedbackSensorValid(
    const AutoSickCorrect_Feedback_t *feedback,
    const AutoSickCorrect_PointParams_t *param, uint8_t index) {
  if (feedback == 0 || param == 0 || !AutoSickCorrect_IndexValid(index) ||
      !feedback->valid[index]) {
    return false;
  }

  return feedback->adc_raw[index] >= param->valid_adc_min &&
         feedback->adc_raw[index] <= param->valid_adc_max;
}

static bool AutoSickCorrect_FeedbackValid(
    const AutoSickCorrect_Feedback_t *feedback,
    const AutoSickCorrect_PointParams_t *param,
    AutoSickCorrect_Action_t action) {
  if (AutoSickCorrect_ActionUsesOnlyYSensors(action)) {
    return AutoSickCorrect_FeedbackSensorValid(feedback, param,
                                               param->rod_rear_index);
  }

  if (AutoSickCorrect_ActionUsesRodSpearheadSamples(action)) {
    if (!AutoSickCorrect_RodSpearheadAnyAxisEnabled()) {
      return true;
    }
    return (!AutoSickCorrect_RodSpearheadXEnabled() ||
            AutoSickCorrect_FeedbackSensorValid(feedback, param,
                                                param->front_index)) &&
           (!AutoSickCorrect_RodSpearheadYEnabled() ||
            AutoSickCorrect_FeedbackSensorValid(feedback, param,
                                                param->rod_rear_index));
  }

  if (AutoSickCorrect_ActionUsesSingleXSensor(action)) {
    return AutoSickCorrect_FeedbackSensorValid(feedback, param,
                                               param->front_index);
  }

  return AutoSickCorrect_FeedbackSensorValid(feedback, param,
                                             param->front_index) &&
         AutoSickCorrect_FeedbackSensorValid(feedback, param,
                                             param->rod_front_index) &&
         AutoSickCorrect_FeedbackSensorValid(feedback, param,
                                             param->rear_index) &&
         AutoSickCorrect_FeedbackSensorValid(feedback, param,
                                             param->rod_rear_index);
}

static bool AutoSickCorrect_Start(AutoSickCorrect_t *ctrl,
                                  AutoSickCorrect_Action_t action,
                                  uint32_t now_ms) {
  if (ctrl == 0 || AutoSickCorrect_IsBusy(ctrl)) {
    return false;
  }

  ctrl->action = action;
  const AutoSickCorrect_PointParams_t *param =
      AutoSickCorrect_ActiveParams(ctrl);
  if (!AutoSickCorrect_PointParamValid(param, action)) {
    AutoSickCorrect_Finish(ctrl, AUTO_SICK_CORRECT_RESULT_FAIL,
                           AUTO_SICK_CORRECT_FAULT_INVALID_PARAM,
                           AUTO_SICK_CORRECT_STATE_FAIL);
    return false;
  }

  ctrl->state = AUTO_SICK_CORRECT_STATE_RUNNING;
  ctrl->result = AUTO_SICK_CORRECT_RESULT_RUNNING;
  ctrl->fault = AUTO_SICK_CORRECT_FAULT_NONE;
  ctrl->step_index = 0u;
  ctrl->start_time_ms = now_ms;
  ctrl->stable_since_ms = 0u;
  ctrl->x_sample_adc = 0.0f;
  ctrl->y_sample_adc = 0.0f;
  ctrl->yaw_sample_diff_adc = 0.0f;
  ctrl->y_target_adc = 0.0f;
  ctrl->x_error_adc = 0.0f;
  ctrl->y_error_adc = 0.0f;
  ctrl->yaw_error_adc = 0.0f;
  ctrl->x_sample_index = 0u;
  AutoSickCorrect_ClearOutputs(ctrl);
  return true;
}

void AutoSickCorrect_Init(AutoSickCorrect_t *ctrl,
                          const AutoSickCorrect_Params_t *param) {
  if (ctrl == 0) {
    return;
  }
  memset(ctrl, 0, sizeof(*ctrl));
  if (param != 0) {
    ctrl->param = *param;
  }
  ctrl->state = AUTO_SICK_CORRECT_STATE_IDLE;
  ctrl->result = AUTO_SICK_CORRECT_RESULT_NONE;
  ctrl->fault = AUTO_SICK_CORRECT_FAULT_NONE;
  ctrl->action = AUTO_SICK_CORRECT_ACTION_NONE;
  AutoSickCorrect_ClearOutputs(ctrl);
}

bool AutoSickCorrect_StartRodSpearheadPosition(AutoSickCorrect_t *ctrl,
                                               uint8_t position_index,
                                               uint32_t now_ms) {
  if (position_index >= AUTO_SICK_CORRECT_ROD_SPEARHEAD_POSITION_COUNT) {
    return false;
  }
  return AutoSickCorrect_Start(
      ctrl,
      (AutoSickCorrect_Action_t)(AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD_POS1 +
                                position_index),
      now_ms);
}

bool AutoSickCorrect_StartOreRelease(AutoSickCorrect_t *ctrl,
                                     uint32_t now_ms) {
  return AutoSickCorrect_Start(ctrl, AUTO_SICK_CORRECT_ACTION_ORE_RELEASE,
                               now_ms);
}

void AutoSickCorrect_Update(AutoSickCorrect_t *ctrl,
                            const AutoSickCorrect_Feedback_t *feedback,
                            uint32_t now_ms) {
  if (ctrl == 0 || ctrl->state != AUTO_SICK_CORRECT_STATE_RUNNING) {
    return;
  }

  const AutoSickCorrect_PointParams_t *param =
      AutoSickCorrect_ActiveParams(ctrl);
  if (!AutoSickCorrect_PointParamValid(param, ctrl->action)) {
    AutoSickCorrect_Finish(ctrl, AUTO_SICK_CORRECT_RESULT_FAIL,
                           AUTO_SICK_CORRECT_FAULT_INVALID_PARAM,
                           AUTO_SICK_CORRECT_STATE_FAIL);
    return;
  }

  const bool rod_spearhead_samples = AutoSickCorrect_ActionUsesRodSpearheadSamples(
      ctrl->action);
  const bool rod_spearhead_x_enabled = AutoSickCorrect_RodSpearheadXEnabled();
  const bool rod_spearhead_y_enabled = AutoSickCorrect_RodSpearheadYEnabled();

  if (rod_spearhead_samples && !rod_spearhead_x_enabled &&
      !rod_spearhead_y_enabled) {
    AutoSickCorrect_CommandChassis(ctrl, 0.0f, 0.0f, 0.0f);
    AutoSickCorrect_Finish(ctrl, AUTO_SICK_CORRECT_RESULT_SUCCESS,
                           AUTO_SICK_CORRECT_FAULT_NONE,
                           AUTO_SICK_CORRECT_STATE_SUCCESS);
    return;
  }

  AutoSickCorrect_CommandPole(ctrl, param);

  if (feedback == 0 || !feedback->pole_all_at_target) {
    ctrl->step_index = 0u;
    AutoSickCorrect_CommandChassis(ctrl, 0.0f, 0.0f, 0.0f);
    ctrl->stable_since_ms = 0u;
    if ((now_ms - ctrl->start_time_ms) >= AutoSickCorrect_TimeoutMs(param)) {
      AutoSickCorrect_Finish(ctrl, AUTO_SICK_CORRECT_RESULT_FAIL,
                             AUTO_SICK_CORRECT_FAULT_TIMEOUT,
                             AUTO_SICK_CORRECT_STATE_FAIL);
    }
    return;
  }

  ctrl->step_index = 1u;

  if (!AutoSickCorrect_FeedbackValid(feedback, param, ctrl->action)) {
    AutoSickCorrect_CommandChassis(ctrl, 0.0f, 0.0f, 0.0f);
    ctrl->stable_since_ms = 0u;
    if ((now_ms - ctrl->start_time_ms) >=
        AutoSickCorrect_TimeoutMs(param)) {
      AutoSickCorrect_Finish(ctrl, AUTO_SICK_CORRECT_RESULT_FAIL,
                             AUTO_SICK_CORRECT_FAULT_SENSOR_INVALID,
                             AUTO_SICK_CORRECT_STATE_FAIL);
    }
    return;
  }

  if (AutoSickCorrect_ActionUsesRodSpearheadSamples(ctrl->action)) {
    if (AutoSickCorrect_RodSpearheadXEnabled()) {
      ctrl->x_sample_adc = (float)feedback->adc_raw[param->front_index];
      ctrl->x_sample_index = param->front_index;
    } else {
      ctrl->x_sample_adc = 0.0f;
      ctrl->x_sample_index = 0u;
    }
    if (rod_spearhead_y_enabled) {
      ctrl->y_sample_adc = (float)feedback->adc_raw[param->rod_rear_index];
      ctrl->y_target_adc = param->y_target_adc;
    } else {
      ctrl->y_sample_adc = 0.0f;
      ctrl->y_target_adc = 0.0f;
    }
    ctrl->yaw_sample_diff_adc = 0.0f;
  } else if (AutoSickCorrect_ActionUsesSingleXSensor(ctrl->action)) {
    ctrl->x_sample_adc = (float)feedback->adc_raw[param->front_index];
    ctrl->x_sample_index = param->front_index;
    ctrl->y_sample_adc = 0.0f;
    ctrl->y_target_adc = 0.0f;
    ctrl->yaw_sample_diff_adc = 0.0f;
  } else if (AutoSickCorrect_ActionUsesOnlyYSensors(ctrl->action)) {
    const float rod_rear_adc = (float)feedback->adc_raw[param->rod_rear_index];

    ctrl->x_sample_adc = 0.0f;
    ctrl->x_sample_index = 0u;
    ctrl->y_sample_adc = rod_rear_adc;
    ctrl->y_target_adc = param->y_target_adc;
    ctrl->yaw_sample_diff_adc = 0.0f;
  } else {
    const float front_adc = (float)feedback->adc_raw[param->front_index];
    const float rear_adc = (float)feedback->adc_raw[param->rear_index];
    const float rod_front_adc = (float)feedback->adc_raw[param->rod_front_index];
    const float rod_rear_adc = (float)feedback->adc_raw[param->rod_rear_index];
    const bool use_front_x = front_adc <= rear_adc;

    ctrl->x_sample_adc = use_front_x ? front_adc : rear_adc;
    ctrl->x_sample_index = use_front_x ? param->front_index : param->rear_index;
    ctrl->y_sample_adc = (rod_front_adc + rod_rear_adc) * 0.5f;
    ctrl->y_target_adc = param->y_target_adc;
    ctrl->yaw_sample_diff_adc = rod_front_adc - rod_rear_adc;
  }

  ctrl->x_error_adc = param->x_target_adc - ctrl->x_sample_adc;
  if (AutoSickCorrect_ActionUsesRodSpearheadSamples(ctrl->action)) {
    if (!rod_spearhead_x_enabled) {
      ctrl->x_error_adc = 0.0f;
    }
    ctrl->y_error_adc = rod_spearhead_y_enabled
                            ? (ctrl->y_target_adc - ctrl->y_sample_adc)
                            : 0.0f;
    ctrl->yaw_error_adc = 0.0f;
  } else if (AutoSickCorrect_ActionUsesOnlyYSensors(ctrl->action)) {
    ctrl->x_error_adc = 0.0f;
    ctrl->y_error_adc = ctrl->y_target_adc - ctrl->y_sample_adc;
    ctrl->yaw_error_adc = 0.0f;
  } else if (AutoSickCorrect_ActionUsesSingleXSensor(ctrl->action)) {
    ctrl->y_error_adc = 0.0f;
    ctrl->yaw_error_adc = 0.0f;
  } else {
    ctrl->y_error_adc = ctrl->y_target_adc - ctrl->y_sample_adc;
    ctrl->yaw_error_adc = param->yaw_target_diff_adc -
                          ctrl->yaw_sample_diff_adc;
  }

  const bool x_done =
      AutoSickCorrect_AbsFloat(ctrl->x_error_adc) <= param->x_tolerance_adc;
  const bool single_x_sensor = AutoSickCorrect_ActionUsesSingleXSensor(
      ctrl->action);
  const bool only_y_sensor = AutoSickCorrect_ActionUsesOnlyYSensors(
      ctrl->action);
  const bool x_axis_enabled = !only_y_sensor &&
      (!rod_spearhead_samples || rod_spearhead_x_enabled);
  const bool yaw_axis_enabled = !single_x_sensor && !only_y_sensor &&
                                !rod_spearhead_samples;
  const bool y_axis_enabled = !single_x_sensor &&
      (!rod_spearhead_samples || rod_spearhead_y_enabled);
  const bool y_done = single_x_sensor ||
      AutoSickCorrect_AbsFloat(ctrl->y_error_adc) <= param->y_tolerance_adc;
  const bool yaw_done = !yaw_axis_enabled ||
      AutoSickCorrect_AbsFloat(ctrl->yaw_error_adc) <= param->yaw_tolerance_adc;

  const float vx_mps =
      (!x_axis_enabled || x_done)
          ? 0.0f
          : AutoSickCorrect_Clamp(ctrl->x_error_adc *
                                      param->x_kp_mps_per_adc,
                                  param->vx_limit_mps);
  const float vy_mps =
      (!y_axis_enabled || y_done)
          ? 0.0f
          : AutoSickCorrect_Clamp(ctrl->y_error_adc *
                                      param->y_kp_mps_per_adc,
                                  param->vy_limit_mps);
  const float wz_rad_s =
      yaw_done ? 0.0f
               : AutoSickCorrect_Clamp(ctrl->yaw_error_adc *
                                           param->yaw_kp_rad_s_per_adc,
                                       param->wz_limit_rad_s);

  AutoSickCorrect_CommandChassis(ctrl, vx_mps, vy_mps, wz_rad_s);

  if ((now_ms - ctrl->start_time_ms) >= AutoSickCorrect_TimeoutMs(param)) {
    if (rod_spearhead_samples &&
        (!x_axis_enabled ||
         AutoSickCorrect_AbsFloat(ctrl->x_error_adc) <=
             AUTO_SICK_CORRECT_ROD_SPEARHEAD_TIMEOUT_SUCCESS_TOLERANCE_ADC) &&
        (!y_axis_enabled ||
         AutoSickCorrect_AbsFloat(ctrl->y_error_adc) <=
             AUTO_SICK_CORRECT_ROD_SPEARHEAD_TIMEOUT_SUCCESS_TOLERANCE_ADC)) {
      AutoSickCorrect_Finish(ctrl, AUTO_SICK_CORRECT_RESULT_SUCCESS,
                             AUTO_SICK_CORRECT_FAULT_NONE,
                             AUTO_SICK_CORRECT_STATE_SUCCESS);
      return;
    }
    AutoSickCorrect_Finish(ctrl, AUTO_SICK_CORRECT_RESULT_FAIL,
                           AUTO_SICK_CORRECT_FAULT_TIMEOUT,
                           AUTO_SICK_CORRECT_STATE_FAIL);
    return;
  }

  if ((x_axis_enabled && !x_done) || (y_axis_enabled && !y_done) ||
      (yaw_axis_enabled && !yaw_done)) {
    ctrl->stable_since_ms = 0u;
    return;
  }

  if (ctrl->stable_since_ms == 0u) {
    ctrl->stable_since_ms = now_ms;
    return;
  }

  if ((now_ms - ctrl->stable_since_ms) >=
      AutoSickCorrect_FinishStableMs(param)) {
    AutoSickCorrect_Finish(ctrl, AUTO_SICK_CORRECT_RESULT_SUCCESS,
                           AUTO_SICK_CORRECT_FAULT_NONE,
                           AUTO_SICK_CORRECT_STATE_SUCCESS);
  }
}

void AutoSickCorrect_Abort(AutoSickCorrect_t *ctrl) {
  if (ctrl == 0) {
    return;
  }
  AutoSickCorrect_Finish(ctrl, AUTO_SICK_CORRECT_RESULT_ABORTED,
                         AUTO_SICK_CORRECT_FAULT_ABORTED,
                         AUTO_SICK_CORRECT_STATE_ABORT);
}

bool AutoSickCorrect_IsBusy(const AutoSickCorrect_t *ctrl) {
  return ctrl != 0 && ctrl->state == AUTO_SICK_CORRECT_STATE_RUNNING;
}

AutoSickCorrect_State_t AutoSickCorrect_GetState(
    const AutoSickCorrect_t *ctrl) {
  return (ctrl == 0) ? AUTO_SICK_CORRECT_STATE_IDLE : ctrl->state;
}

AutoSickCorrect_Result_t AutoSickCorrect_GetResult(
    const AutoSickCorrect_t *ctrl) {
  return (ctrl == 0) ? AUTO_SICK_CORRECT_RESULT_NONE : ctrl->result;
}

AutoSickCorrect_Fault_t AutoSickCorrect_GetFault(
    const AutoSickCorrect_t *ctrl) {
  return (ctrl == 0) ? AUTO_SICK_CORRECT_FAULT_INVALID_PARAM : ctrl->fault;
}

AutoSickCorrect_Action_t AutoSickCorrect_GetAction(
    const AutoSickCorrect_t *ctrl) {
  return (ctrl == 0) ? AUTO_SICK_CORRECT_ACTION_NONE : ctrl->action;
}

uint8_t AutoSickCorrect_GetStepIndex(const AutoSickCorrect_t *ctrl) {
  return (ctrl == 0) ? 0u : ctrl->step_index;
}

const Chassis_CMD_t *AutoSickCorrect_GetChassisCommand(
    const AutoSickCorrect_t *ctrl) {
  return (ctrl != 0 && ctrl->chassis_cmd_valid) ? &ctrl->chassis_cmd : 0;
}

const Pole_CMD_t *AutoSickCorrect_GetPoleCommand(
    const AutoSickCorrect_t *ctrl) {
  return (ctrl != 0 && ctrl->pole_cmd_valid) ? &ctrl->pole_cmd : 0;
}
