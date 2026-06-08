#include "module/autoCtrlAPI/sick/auto_sick_correct.h"

#include <math.h>
#include <string.h>

#define AUTO_SICK_CORRECT_DEFAULT_FINISH_STABLE_MS (120u)
#define AUTO_SICK_CORRECT_DEFAULT_TIMEOUT_MS (5000u)
#define AUTO_SICK_CORRECT_DEFAULT_POLE_TARGET_LIFT (2.0f)

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
    const AutoSickCorrect_PointParams_t *param) {
  if (param == 0) {
    return false;
  }

  if (!AutoSickCorrect_IndexValid(param->left_index) ||
      !AutoSickCorrect_IndexValid(param->front_left_index) ||
      !AutoSickCorrect_IndexValid(param->front_right_index) ||
      !AutoSickCorrect_IndexValid(param->right_index) ||
      param->valid_adc_min > param->valid_adc_max) {
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
    case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD:
      return &ctrl->param.rod_spearhead;
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
    const AutoSickCorrect_PointParams_t *param) {
  return AutoSickCorrect_FeedbackSensorValid(feedback, param,
                                             param->left_index) &&
         AutoSickCorrect_FeedbackSensorValid(feedback, param,
                                             param->front_left_index) &&
         AutoSickCorrect_FeedbackSensorValid(feedback, param,
                                             param->front_right_index) &&
         AutoSickCorrect_FeedbackSensorValid(feedback, param,
                                             param->right_index);
}

static bool AutoSickCorrect_Start(AutoSickCorrect_t *ctrl,
                                  AutoSickCorrect_Action_t action,
                                  uint32_t now_ms) {
  if (ctrl == 0 || AutoSickCorrect_IsBusy(ctrl)) {
    return false;
  }

  if (action == AUTO_SICK_CORRECT_ACTION_ORE_RELEASE) {
    ctrl->action = action;
    AutoSickCorrect_Finish(ctrl, AUTO_SICK_CORRECT_RESULT_FAIL,
                           AUTO_SICK_CORRECT_FAULT_UNSUPPORTED,
                           AUTO_SICK_CORRECT_STATE_FAIL);
    return false;
  }

  ctrl->action = action;
  const AutoSickCorrect_PointParams_t *param =
      AutoSickCorrect_ActiveParams(ctrl);
  if (!AutoSickCorrect_PointParamValid(param)) {
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
  ctrl->x_error_adc = 0.0f;
  ctrl->y_error_adc = 0.0f;
  ctrl->yaw_error_adc = 0.0f;
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

bool AutoSickCorrect_StartRodSpearhead(AutoSickCorrect_t *ctrl,
                                       uint32_t now_ms) {
  return AutoSickCorrect_Start(ctrl, AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD,
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
  if (!AutoSickCorrect_PointParamValid(param)) {
    AutoSickCorrect_Finish(ctrl, AUTO_SICK_CORRECT_RESULT_FAIL,
                           AUTO_SICK_CORRECT_FAULT_INVALID_PARAM,
                           AUTO_SICK_CORRECT_STATE_FAIL);
    return;
  }

  AutoSickCorrect_CommandPole(ctrl, param);

  if (!AutoSickCorrect_FeedbackValid(feedback, param)) {
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

  const float front_left_adc = (float)feedback->adc_raw[param->front_left_index];
  const float front_right_adc =
      (float)feedback->adc_raw[param->front_right_index];
  const float left_adc = (float)feedback->adc_raw[param->left_index];
  const float right_adc = (float)feedback->adc_raw[param->right_index];

  ctrl->x_sample_adc = (front_left_adc + front_right_adc) * 0.5f;
  ctrl->y_sample_adc = (left_adc < right_adc) ? left_adc : right_adc;
  ctrl->yaw_sample_diff_adc = front_left_adc - front_right_adc;

  ctrl->x_error_adc = param->x_target_adc - ctrl->x_sample_adc;
  ctrl->y_error_adc = param->y_target_adc - ctrl->y_sample_adc;
  ctrl->yaw_error_adc = param->yaw_target_diff_adc -
                        ctrl->yaw_sample_diff_adc;

  const bool x_done =
      AutoSickCorrect_AbsFloat(ctrl->x_error_adc) <= param->x_tolerance_adc;
  const bool y_done =
      AutoSickCorrect_AbsFloat(ctrl->y_error_adc) <= param->y_tolerance_adc;
  const bool yaw_done =
      AutoSickCorrect_AbsFloat(ctrl->yaw_error_adc) <= param->yaw_tolerance_adc;

  const float vx_mps =
      x_done ? 0.0f
             : AutoSickCorrect_Clamp(ctrl->x_error_adc *
                                         param->x_kp_mps_per_adc,
                                     param->vx_limit_mps);
  const float vy_mps =
      y_done ? 0.0f
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
    AutoSickCorrect_Finish(ctrl, AUTO_SICK_CORRECT_RESULT_FAIL,
                           AUTO_SICK_CORRECT_FAULT_TIMEOUT,
                           AUTO_SICK_CORRECT_STATE_FAIL);
    return;
  }

  if (!x_done || !y_done || !yaw_done) {
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
