#include "module/autoCtrlAPI/template/auto_ctrl_template_runner.h"

#include <math.h>

#include "module/autoCtrlAPI/primitive/auto_ctrl_primitive.h"
#include "module/config.h"

typedef struct {
  const float *all_extend;
  const float *front_retract;
  const float *all_retract;
  const float *small;
} auto_ctrl_pole_targets_t;

/* Record photo edges near a Pole target, but consume them only after the
 * commanded Pole pose passes the normal at-target debounce. */
#define AUTO_CTRL_ASCEND_200_PHOTO_RECORD_LIFT_RAD (3.5f)
#define AUTO_CTRL_ASCEND_400_PHOTO_RECORD_LIFT_RAD (7.0f)
#define AUTO_CTRL_DESCEND_PHOTO_RECORD_LIFT_RAD (2.5f)

typedef enum {
  AUTO_CTRL_POLE_PROFILE_NONE = 0,
  AUTO_CTRL_POLE_PROFILE_ALL_EXTEND,
  AUTO_CTRL_POLE_PROFILE_ALL_RETRACT,
  AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND,
  AUTO_CTRL_POLE_PROFILE_FRONT_RETRACT,
  AUTO_CTRL_POLE_PROFILE_REAR_EXTEND,
  AUTO_CTRL_POLE_PROFILE_REAR_RETRACT,
} auto_ctrl_pole_profile_e;

#define AUTO_CTRL_PHOTO_STABLE_MS (5u) /* 光电稳定触发时间，单�?ms�?*/
#define AUTO_CTRL_TIMED_MOVE_YAW_TOLERANCE_DEFAULT_RAD (0.3490329252f)
#define AUTO_CTRL_FUSED_CONSTRAINT_SUCCESS_TIMEOUT_MS (5000u)

enum {
  AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_NONE = 0u,
  AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_ASCEND_FRONT = 1u,
  AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_ASCEND_REAR = 2u,
  AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_DESCEND_FIRST_FALLING = 3u,
  AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_DESCEND_SECOND_FALLING = 4u,
  AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_FINAL_RISING = 5u,
};

enum {
  AUTO_CTRL_TEMPLATE_DEBUG_POLE_NONE = 0u,
  AUTO_CTRL_TEMPLATE_DEBUG_POLE_HOLD = 1u,
  AUTO_CTRL_TEMPLATE_DEBUG_POLE_AFTER_PHOTO = 2u,
  AUTO_CTRL_TEMPLATE_DEBUG_POLE_DIRECT = 3u,
};

static void AutoCtrlTemplate_CommandPoleProfile(
    auto_ctrl_t *ctrl, float front_target, float rear_target, float front_speed,
    float rear_speed, auto_ctrl_pole_profile_e front_profile,
    auto_ctrl_pole_profile_e rear_profile);

static void AutoCtrlTemplate_DebugMarkPhotoEvent(auto_ctrl_t *ctrl,
                                                uint32_t now_ms,
                                                uint8_t event_id) {
  uint32_t raw_time_ms;
  uint32_t high_duration_ms = 0u;

  if (ctrl == 0) {
    return;
  }

  if (ctrl->template_ctx.debug_photo_event_time_ms != 0u &&
      ctrl->template_ctx.debug_photo_event_step_index ==
          ctrl->template_ctx.step_index &&
      ctrl->template_ctx.debug_photo_event_id == event_id) {
    return;
  }

  raw_time_ms = now_ms;
  switch (event_id) {
    case AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_ASCEND_FRONT:
      raw_time_ms = ctrl->template_ctx.pe13_photo1_triggered_since_ms;
      break;
    case AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_ASCEND_REAR:
      raw_time_ms = ctrl->template_ctx.pa2_photo3_triggered_since_ms;
      break;
    case AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_DESCEND_FIRST_FALLING:
      raw_time_ms = ctrl->template_ctx.pe9_photo2_released_since_ms;
      if (ctrl->template_ctx.pe9_photo2_released_since_ms >=
          ctrl->template_ctx.pe9_photo2_triggered_since_ms) {
        high_duration_ms = ctrl->template_ctx.pe9_photo2_released_since_ms -
                           ctrl->template_ctx.pe9_photo2_triggered_since_ms;
      }
      break;
    case AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_DESCEND_SECOND_FALLING:
      raw_time_ms = ctrl->template_ctx.pa0_photo4_released_since_ms;
      if (ctrl->template_ctx.pa0_photo4_released_since_ms >=
          ctrl->template_ctx.pa0_photo4_triggered_since_ms) {
        high_duration_ms = ctrl->template_ctx.pa0_photo4_released_since_ms -
                           ctrl->template_ctx.pa0_photo4_triggered_since_ms;
      }
      break;
    case AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_FINAL_RISING:
      raw_time_ms = ctrl->template_ctx.pa0_photo4_triggered_since_ms;
      break;
    case AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_NONE:
    default:
      break;
  }

  ctrl->template_ctx.debug_photo_raw_time_ms =
      (raw_time_ms != 0u) ? raw_time_ms : now_ms;
  ctrl->template_ctx.debug_photo_event_time_ms = now_ms;
    ctrl->template_ctx.debug_photo_high_duration_ms = high_duration_ms;
  ctrl->template_ctx.debug_photo_event_step_index =
      ctrl->template_ctx.step_index;
  ctrl->template_ctx.debug_photo_event_id = event_id;
  ctrl->template_ctx.debug_pole_cmd_time_ms = 0u;
  ctrl->template_ctx.debug_pole_cmd_step_index = 0u;
  ctrl->template_ctx.debug_pole_cmd_kind = AUTO_CTRL_TEMPLATE_DEBUG_POLE_NONE;
}

static void AutoCtrlTemplate_DebugMarkPoleCommand(auto_ctrl_t *ctrl,
                                                 uint32_t now_ms,
                                                 uint8_t command_kind) {
  if (ctrl == 0 || ctrl->template_ctx.debug_photo_event_time_ms == 0u ||
      ctrl->template_ctx.debug_pole_cmd_time_ms != 0u) {
    return;
  }
  ctrl->template_ctx.debug_pole_cmd_time_ms = now_ms;
  ctrl->template_ctx.debug_pole_cmd_step_index = ctrl->template_ctx.step_index;
  ctrl->template_ctx.debug_pole_cmd_kind = command_kind;
}

static void AutoCtrlTemplate_EnterStep(auto_ctrl_t *ctrl, uint32_t now_ms) {
  if (!ctrl->template_ctx.step_entered) {
    ctrl->template_ctx.step_entered = true;
    ctrl->template_ctx.step_enter_time_ms = now_ms;
  }
} 

static uint32_t AutoCtrlTemplate_StepElapsed(const auto_ctrl_t *ctrl,
                                             uint32_t now_ms) {
  return now_ms - ctrl->template_ctx.step_enter_time_ms;
}

static void AutoCtrlTemplate_SetStep(auto_ctrl_t *ctrl, uint8_t step_index) {
  ctrl->template_ctx.step_index = step_index;
  ctrl->template_ctx.step_entered = false;
  ctrl->template_ctx.pole_target_seen_not_ready = false;
  ctrl->template_ctx.photo_stop_entered = false;
  ctrl->template_ctx.photo_stop_enter_time_ms = 0u;
  ctrl->template_ctx.ascend_front_retract_delay_started = false;
  ctrl->template_ctx.ascend_front_retract_delay_start_ms = 0u;
  ctrl->template_ctx.ascend_rear_retract_delay_started = false;
  ctrl->template_ctx.ascend_rear_retract_delay_start_ms = 0u;
  ctrl->template_ctx.final_photo_sprint_started = false;
  ctrl->template_ctx.final_photo_sprint_start_ms = 0u;
  ctrl->template_ctx.descend_start_move_entered = false;
  ctrl->template_ctx.descend_start_lift_ready = false;
  ctrl->template_ctx.descend_landing_approach_done = false;
  ctrl->template_ctx.descend_start_move_time_ms = 0u;
  ctrl->template_ctx.distance_latch_valid = false;
  ctrl->template_ctx.wheel_delta_rad = 0.0f;
}

static void AutoCtrlTemplate_NextStep(auto_ctrl_t *ctrl) {
  AutoCtrlTemplate_SetStep(ctrl, (uint8_t)(ctrl->template_ctx.step_index + 1u));
}

static bool AutoCtrlTemplate_AscendFrontRetractDelayElapsed(
    auto_ctrl_t *ctrl, uint32_t now_ms, uint32_t delay_ms) {
  if (!ctrl->template_ctx.ascend_front_retract_delay_started) {
    ctrl->template_ctx.ascend_front_retract_delay_started = true;
    ctrl->template_ctx.ascend_front_retract_delay_start_ms = now_ms;
  }
  return (uint32_t)(now_ms -
                    ctrl->template_ctx.ascend_front_retract_delay_start_ms) >=
         delay_ms;
}

static bool AutoCtrlTemplate_AscendRearRetractDelayElapsed(
    auto_ctrl_t *ctrl, uint32_t now_ms, uint32_t delay_ms) {
  if (!ctrl->template_ctx.ascend_rear_retract_delay_started) {
    ctrl->template_ctx.ascend_rear_retract_delay_started = true;
    ctrl->template_ctx.ascend_rear_retract_delay_start_ms = now_ms;
  }
  return (uint32_t)(now_ms -
                    ctrl->template_ctx.ascend_rear_retract_delay_start_ms) >=
         delay_ms;
}

static bool AutoCtrlTemplate_IsYawAligned(const auto_ctrl_t *ctrl) {
  return fabsf(ctrl->yaw_error_rad) <= ctrl->yaw_tolerance_rad;
}

static void AutoCtrlTemplate_CommandChassisZeroVector(auto_ctrl_t *ctrl) {
  ctrl->chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
  ctrl->chassis_cmd.ctrl_vec.vx = 0.0f;
  ctrl->chassis_cmd.ctrl_vec.vy = 0.0f;
  ctrl->chassis_cmd.ctrl_vec.wz = 0.0f;
}

static float AutoCtrlTemplate_GetTimedMoveYawTolerance(
    const auto_ctrl_t *ctrl, const AutoCtrl_TemplateParam_t *param) {
  if (param != 0 && isfinite(param->timed_move_yaw_tolerance_rad) &&
      param->timed_move_yaw_tolerance_rad > 0.0f) {
    return param->timed_move_yaw_tolerance_rad;
  }

  if (ctrl != 0 && isfinite(ctrl->yaw_tolerance_rad) &&
      ctrl->yaw_tolerance_rad > 0.0f) {
    return ctrl->yaw_tolerance_rad;
  }

  return AUTO_CTRL_TIMED_MOVE_YAW_TOLERANCE_DEFAULT_RAD;
}

static bool AutoCtrlTemplate_IsTimedMoveYawAligned(
    const auto_ctrl_t *ctrl, const AutoCtrl_TemplateParam_t *param) {
  if (ctrl == 0) {
    return false;
  }

  return fabsf(ctrl->yaw_error_rad) <=
         AutoCtrlTemplate_GetTimedMoveYawTolerance(ctrl, param);
}

static bool AutoCtrlTemplate_TimedMoveReady(auto_ctrl_t *ctrl,
                                            uint32_t now_ms,
                                            uint32_t move_ms,
                                            const AutoCtrl_TemplateParam_t *param) {
  AutoCtrlTemplate_EnterStep(ctrl, now_ms);
  return AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >= move_ms &&
         AutoCtrlTemplate_IsTimedMoveYawAligned(ctrl, param);
}

static uint32_t AutoCtrlTemplate_DescendMidMoveMs(
    const auto_ctrl_t *ctrl, const AutoCtrl_TemplateParam_t *param) {
  if (ctrl != 0 && ctrl->template_ctx.descend_move_override_enabled) {
    return ctrl->template_ctx.descend_mid_move_ms;
  }
  return (param == 0) ? 0u : param->mid_move_ms;
}

static uint32_t AutoCtrlTemplate_DescendRearRetractMoveMs(
    const auto_ctrl_t *ctrl, const AutoCtrl_TemplateParam_t *param) {
  if (ctrl != 0 && ctrl->template_ctx.descend_move_override_enabled) {
    return ctrl->template_ctx.descend_rear_retract_move_ms;
  }
  return (param == 0) ? 0u : param->rear_retract_move_ms;
}

static float AutoCtrlTemplate_DescendRearRetractWheelDeltaRad(
    const auto_ctrl_t *ctrl, const AutoCtrl_TemplateParam_t *param) {
  if (ctrl != 0 && ctrl->template_ctx.descend_move_override_enabled) {
    return ctrl->template_ctx.descend_rear_retract_move_wheel_delta_rad;
  }
  return (param == 0) ? 0.0f : param->rear_retract_move_wheel_delta_rad;
}

static float AutoCtrlTemplate_AbsFloat(float value) {
  return (value >= 0.0f) ? value : -value;
}

/*
 * target_wheel_delta_rad > 0: wheel delta is the primary completion condition,
 * and fallback_move_ms is only a timeout. target_wheel_delta_rad <= 0: use the
 * time gate as a fixed-duration move.
 */
static bool AutoCtrlTemplate_WheelDeltaMoveReady(
    auto_ctrl_t *ctrl, uint32_t now_ms, float target_wheel_delta_rad,
    uint32_t fallback_move_ms, const AutoCtrl_TemplateParam_t *param,
    const Config_RobotParam_t *robot_param) {
  (void)robot_param;
  AutoCtrlTemplate_EnterStep(ctrl, now_ms);

  if (target_wheel_delta_rad <= 0.0f) {
    return AutoCtrlTemplate_TimedMoveReady(ctrl, now_ms, fallback_move_ms,
                                           param);
  }

  if (!ctrl->template_ctx.distance_latch_valid) {
    for (uint8_t i = 0u; i < 4u; ++i) {
      ctrl->template_ctx.distance_start_wheel_rad[i] =
          ctrl->feedback.wheel_position_rad[i];
    }
    ctrl->template_ctx.wheel_delta_rad = 0.0f;
    ctrl->template_ctx.distance_latch_valid = true;
    return false;
  }

  float wheel_delta_abs_sum_rad = 0.0f;
  for (uint8_t i = 0u; i < 4u; ++i) {
    wheel_delta_abs_sum_rad += AutoCtrlTemplate_AbsFloat(
        ctrl->feedback.wheel_position_rad[i] -
        ctrl->template_ctx.distance_start_wheel_rad[i]);
  }
  ctrl->template_ctx.wheel_delta_rad = wheel_delta_abs_sum_rad * 0.25f;

  const bool distance_ready =
      ctrl->template_ctx.wheel_delta_rad >= target_wheel_delta_rad;
  const bool fallback_timeout =
      fallback_move_ms > 0u &&
      AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >= fallback_move_ms;
  if (ctrl->use_fused_template_params) {
    return distance_ready || fallback_timeout;
  }
  return (distance_ready || fallback_timeout) &&
         AutoCtrlTemplate_IsTimedMoveYawAligned(ctrl, param);
}

static float AutoCtrlTemplate_ResolvePoleLiftAccel(
    const Config_RobotParam_t *robot_param,
    const AutoCtrl_TemplateParam_t *template_param,
    auto_ctrl_pole_profile_e action) {
  (void)robot_param;
  if (template_param == 0) {
    return 0.0f;
  }

  switch (action) {
    case AUTO_CTRL_POLE_PROFILE_ALL_EXTEND:
      return template_param->pole_all_extend_accel;
    case AUTO_CTRL_POLE_PROFILE_ALL_RETRACT:
      return template_param->pole_all_retract_accel;
    case AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND:
      return template_param->pole_front_extend_accel;
    case AUTO_CTRL_POLE_PROFILE_FRONT_RETRACT:
      return template_param->pole_front_retract_accel;
    case AUTO_CTRL_POLE_PROFILE_REAR_EXTEND:
      return template_param->pole_rear_extend_accel;
    case AUTO_CTRL_POLE_PROFILE_REAR_RETRACT:
      return template_param->pole_rear_retract_accel;
    case AUTO_CTRL_POLE_PROFILE_NONE:
    default:
      return 0.0f;
  }
}

static bool AutoCtrlTemplate_PoleReadyAfterNewTarget(auto_ctrl_t *ctrl,
                                                     bool pole_ready) {
  if (!pole_ready) {
    ctrl->template_ctx.pole_target_seen_not_ready = true;
    return false;
  }

  return ctrl->template_ctx.pole_target_seen_not_ready;
}

static void AutoCtrlTemplate_CommandPole(auto_ctrl_t *ctrl, float front_target,
                                         float rear_target, float front_speed,
                                         float rear_speed) {
  AutoCtrlTemplate_CommandPoleProfile(
      ctrl, front_target, rear_target, front_speed, rear_speed,
      AUTO_CTRL_POLE_PROFILE_NONE, AUTO_CTRL_POLE_PROFILE_NONE);
}

static float AutoCtrlTemplate_ConfiguredPoleSpeed(
    const AutoCtrl_TemplateParam_t *template_param,
    auto_ctrl_pole_profile_e action) {
  if (template_param == 0) {
    return 0.0f;
  }

  switch (action) {
    case AUTO_CTRL_POLE_PROFILE_ALL_EXTEND:
      return template_param->pole_all_extend_speed;
    case AUTO_CTRL_POLE_PROFILE_ALL_RETRACT:
      return template_param->pole_all_retract_speed;
    case AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND:
      return template_param->pole_front_extend_speed;
    case AUTO_CTRL_POLE_PROFILE_FRONT_RETRACT:
      return template_param->pole_front_retract_speed;
    case AUTO_CTRL_POLE_PROFILE_REAR_EXTEND:
      return template_param->pole_rear_extend_speed;
    case AUTO_CTRL_POLE_PROFILE_REAR_RETRACT:
      return template_param->pole_rear_retract_speed;
    case AUTO_CTRL_POLE_PROFILE_NONE:
    default:
      return 0.0f;
  }
}

static float AutoCtrlTemplate_ResolvePoleSpeed(
    const AutoCtrl_TemplateParam_t *template_param,
    auto_ctrl_pole_profile_e action, float fallback_speed) {
  const float configured_speed =
      AutoCtrlTemplate_ConfiguredPoleSpeed(template_param, action);
  return configured_speed > 0.0f ? configured_speed : fallback_speed;
}

static bool AutoCtrlTemplate_DescendLandingEnabled(
    const AutoCtrl_TemplateParam_t *param) {
  return param != 0 && isfinite(param->pole_extend_landing_zone_rad) &&
         param->pole_extend_landing_zone_rad > 0.0f &&
         isfinite(param->pole_extend_landing_speed) &&
         param->pole_extend_landing_speed > 0.0f;
}

static float AutoCtrlTemplate_DescendLandingApproachTarget(
    const AutoCtrl_TemplateParam_t *param, float hold_target,
    float final_target) {
  if (!AutoCtrlTemplate_DescendLandingEnabled(param) ||
      final_target <= hold_target) {
    return final_target;
  }
  return fmaxf(hold_target,
               final_target - param->pole_extend_landing_zone_rad);
}

static float AutoCtrlTemplate_DescendLandingSpeed(
    const AutoCtrl_TemplateParam_t *param, float normal_speed) {
  if (!AutoCtrlTemplate_DescendLandingEnabled(param)) {
    return normal_speed;
  }
  return fminf(normal_speed, param->pole_extend_landing_speed);
}

static void AutoCtrlTemplate_CommandPoleProfile(
    auto_ctrl_t *ctrl, float front_target, float rear_target, float front_speed,
    float rear_speed, auto_ctrl_pole_profile_e front_profile,
    auto_ctrl_pole_profile_e rear_profile) {
  const Config_RobotParam_t *robot_param = Config_GetRobotParam();
  const AutoCtrl_TemplateParam_t *template_param =
      Config_GetAutoCtrlTemplateParam(ctrl->template_id,
                      ctrl->use_fused_template_params);
  const float front_accel = AutoCtrlTemplate_ResolvePoleLiftAccel(
      robot_param, template_param, front_profile);
  const float rear_accel = AutoCtrlTemplate_ResolvePoleLiftAccel(
      robot_param, template_param, rear_profile);
    const bool front_bypass_limit =
      front_profile != AUTO_CTRL_POLE_PROFILE_NONE && front_accel == 0.0f &&
      AutoCtrlTemplate_ConfiguredPoleSpeed(template_param, front_profile) ==
        0.0f;
    const bool rear_bypass_limit =
      rear_profile != AUTO_CTRL_POLE_PROFILE_NONE && rear_accel == 0.0f &&
      AutoCtrlTemplate_ConfiguredPoleSpeed(template_param, rear_profile) ==
        0.0f;
    front_speed = front_bypass_limit
            ? 0.0f
            : AutoCtrlTemplate_ResolvePoleSpeed(
                template_param, front_profile, front_speed);
    rear_speed = rear_bypass_limit
             ? 0.0f
             : AutoCtrlTemplate_ResolvePoleSpeed(
               template_param, rear_profile, rear_speed);

  AutoCtrlPrimitive_CommandPoleTargetWithSpeed(ctrl, front_target, rear_target,
                                               front_speed, rear_speed);
  /* speed/accel 同为 0 表示直接使用最终目标，不做轨迹或末端制动限速。 */
  ctrl->pole_cmd.auto_lift_accel[0] = front_accel;
  ctrl->pole_cmd.auto_lift_accel[1] = rear_accel;
  ctrl->pole_cmd.disable_lift_accel = front_accel < 0.0f || rear_accel < 0.0f;
}

static void AutoCtrlTemplate_CommandPoleProfileWithLanding(
    auto_ctrl_t *ctrl, const AutoCtrl_TemplateParam_t *param,
    float front_target, float rear_target, float front_speed,
    float rear_speed, auto_ctrl_pole_profile_e front_profile,
    auto_ctrl_pole_profile_e rear_profile, bool front_landing,
    bool rear_landing) {
  AutoCtrlTemplate_CommandPoleProfile(
      ctrl, front_target, rear_target, front_speed, rear_speed,
      front_profile, rear_profile);
  if (front_landing) {
    ctrl->pole_cmd.auto_lift_speed[0] =
        AutoCtrlTemplate_DescendLandingSpeed(
            param, ctrl->pole_cmd.auto_lift_speed[0]);
  }
  if (rear_landing) {
    ctrl->pole_cmd.auto_lift_speed[1] =
        AutoCtrlTemplate_DescendLandingSpeed(
            param, ctrl->pole_cmd.auto_lift_speed[1]);
  }
}

static bool AutoCtrlTemplate_DescendStartPoleLiftReady(
    const auto_ctrl_t *ctrl, const AutoCtrl_TemplateParam_t *param) {
  if (param == 0 || param->descend_start_pole_lift_threshold <= 0.0f) {
    return true;
  }

  const float threshold = param->descend_start_pole_lift_threshold;
  return ctrl->feedback.pole_front_lift_rad >= threshold &&
         ctrl->feedback.pole_rear_lift_rad >= threshold;
}

static bool AutoCtrlTemplate_RunDescendStartSprint(
    auto_ctrl_t *ctrl, uint32_t now_ms, const AutoCtrl_TemplateParam_t *param,
    float vx_mps, float front_target, float rear_target,
    float front_speed, float rear_speed) {
  if (param != 0 && param->descend_start_pole_lift_threshold > 0.0f) {
    const float threshold = param->descend_start_pole_lift_threshold;
    if (front_target < threshold) {
      front_target = threshold;
    }
    if (rear_target < threshold) {
      rear_target = threshold;
    }
  }

  AutoCtrlTemplate_CommandPoleProfile(
      ctrl, front_target, rear_target, front_speed, rear_speed,
      AUTO_CTRL_POLE_PROFILE_ALL_RETRACT,
      AUTO_CTRL_POLE_PROFILE_ALL_RETRACT);

  if (!ctrl->template_ctx.descend_start_lift_ready) {
    ctrl->template_ctx.descend_start_lift_ready =
        AutoCtrlTemplate_DescendStartPoleLiftReady(ctrl, param);
  }

  if (!ctrl->template_ctx.descend_start_lift_ready) {
    AutoCtrlPrimitive_CommandFlatMoveWithYawRate(ctrl, 0.0f);
    return false;
  }

  if (!ctrl->template_ctx.descend_start_move_entered) {
    ctrl->template_ctx.descend_start_move_entered = true;
    ctrl->template_ctx.descend_start_move_time_ms = now_ms;
  }

  const uint32_t move_ms =
      AutoCtrlTemplate_DescendMidMoveMs(ctrl, param);
  if (move_ms > 0u) {
    AutoCtrlPrimitive_ApplyPrealignWithMove(ctrl, vx_mps, 0.0f);
  } else {
    AutoCtrlPrimitive_CommandFlatMoveWithYawRate(ctrl, 0.0f);
  }
  return (now_ms - ctrl->template_ctx.descend_start_move_time_ms) >=
             move_ms &&
         AutoCtrlTemplate_IsTimedMoveYawAligned(ctrl, param);
}

static float AutoCtrlTemplate_SecondPhotoRetractVx(
    const AutoCtrl_TemplateParam_t *param, float fallback_vx) {
  if (param == 0 || param->second_photo_retract_move_speed <= 0.0f) {
    return fallback_vx;
  }

  return fabsf(param->second_photo_retract_move_speed);
}

static void AutoCtrlTemplate_SelectPoleTargets(
    const Config_RobotParam_t *robot_param, bool use_400mm, bool descend,
    auto_ctrl_pole_targets_t *targets) {
  if (use_400mm && descend) {
    targets->all_extend =
        robot_param->pole_param.preset.step_400_descend_all_extend;
    targets->front_retract =
        robot_param->pole_param.preset.step_400_descend_front_retract;
    targets->all_retract =
        robot_param->pole_param.preset.step_400_descend_all_retract;
    targets->small = robot_param->pole_param.preset.step_400_descend_all_retract;
    return;
  }

  if (use_400mm) {
    targets->all_extend = robot_param->pole_param.preset.step_400_all_extend;
    targets->front_retract =
        robot_param->pole_param.preset.step_400_front_retract;
    targets->all_retract = robot_param->pole_param.preset.step_400_all_retract;
    targets->small = robot_param->pole_param.preset.step_200_small;
    return;
  }

  if (descend) {
    targets->all_extend =
        robot_param->pole_param.preset.step_200_descend_all_extend;
    targets->front_retract =
        robot_param->pole_param.preset.step_200_descend_front_retract;
    targets->all_retract =
        robot_param->pole_param.preset.step_200_descend_all_retract;
    targets->small = robot_param->pole_param.preset.step_200_descend_all_retract;
    return;
  }

  targets->all_extend = robot_param->pole_param.preset.step_200_all_extend;
  targets->front_retract =
      robot_param->pole_param.preset.step_200_front_retract;
  targets->all_retract = robot_param->pole_param.preset.step_200_all_retract;
  targets->small = robot_param->pole_param.preset.step_200_small;
}

static void AutoCtrlTemplate_ResetPhotoDetection(auto_ctrl_t *ctrl) {
  ctrl->template_ctx.pe13_photo1_triggered_latched = false;
  ctrl->template_ctx.pe9_photo2_triggered_latched = false;
  ctrl->template_ctx.pa2_photo3_triggered_latched = false;
  ctrl->template_ctx.pa0_photo4_triggered_latched = false;
  ctrl->template_ctx.pe13_photo1_triggered_since_ms = 0u;
  ctrl->template_ctx.pe9_photo2_triggered_since_ms = 0u;
  ctrl->template_ctx.pa2_photo3_triggered_since_ms = 0u;
  ctrl->template_ctx.pa0_photo4_triggered_since_ms = 0u;
  ctrl->template_ctx.pe13_photo1_stable_trigger_seen = false;
  ctrl->template_ctx.pe9_photo2_stable_trigger_seen = false;
  ctrl->template_ctx.pa2_photo3_stable_trigger_seen = false;
  ctrl->template_ctx.pa0_photo4_stable_trigger_seen = false;
  ctrl->template_ctx.pe13_photo1_stable_release_latched = false;
  ctrl->template_ctx.pe9_photo2_stable_release_latched = false;
  ctrl->template_ctx.pa2_photo3_stable_release_latched = false;
  ctrl->template_ctx.pa0_photo4_stable_release_latched = false;
  ctrl->template_ctx.pa0_photo4_stable_low_seen = false;
  ctrl->template_ctx.pe13_photo1_released_since_ms = 0u;
  ctrl->template_ctx.pe9_photo2_released_since_ms = 0u;
  ctrl->template_ctx.pa2_photo3_released_since_ms = 0u;
  ctrl->template_ctx.pa0_photo4_released_since_ms = 0u;
  ctrl->template_ctx.debug_photo_high_duration_ms = 0u;
}

static void AutoCtrlTemplate_PauseStableTimer(bool phase_complete,
                                              uint32_t *since_ms,
                                              uint32_t now_ms) {
  if (!phase_complete && since_ms != 0 && *since_ms != 0u) {
    *since_ms = now_ms;
  }
}

static bool AutoCtrlTemplate_LatchPhotoStable(bool triggered, bool *latched,
                                              uint32_t *triggered_since_ms,
                                              uint32_t now_ms) {
  if (latched == 0 || triggered_since_ms == 0) {
    return false;
  }

  if (*latched) {
    return true;
  }

  if (!triggered) {
    *triggered_since_ms = 0u;
    return false;
  }

  if (*triggered_since_ms == 0u) {
    *triggered_since_ms = now_ms;
    return false;
  }

  if ((now_ms - *triggered_since_ms) >= AUTO_CTRL_PHOTO_STABLE_MS) {
    *latched = true;
  }
  return *latched;
}

static bool AutoCtrlTemplate_LatchPhotoStableFalling(
    bool triggered, bool *stable_trigger_seen, bool *stable_release_latched,
    uint32_t *triggered_since_ms, uint32_t *released_since_ms,
    uint32_t now_ms) {
  if (stable_trigger_seen == 0 || stable_release_latched == 0 ||
      triggered_since_ms == 0 || released_since_ms == 0) {
    return false;
  }

  if (*stable_release_latched) {
    return true;
  }

  if (!*stable_trigger_seen) {
    if (!triggered) {
      *triggered_since_ms = 0u;
      return false;
    }

    if (*triggered_since_ms == 0u) {
      *triggered_since_ms = now_ms;
      return false;
    }

    if ((now_ms - *triggered_since_ms) >= AUTO_CTRL_PHOTO_STABLE_MS) {
      *stable_trigger_seen = true;
      *released_since_ms = 0u;
    }
    return false;
  }

  if (triggered) {
    *released_since_ms = 0u;
    return false;
  }

  if (*released_since_ms == 0u) {
    *released_since_ms = now_ms;
    return false;
  }

  if ((now_ms - *released_since_ms) >= AUTO_CTRL_PHOTO_STABLE_MS) {
    *stable_release_latched = true;
  }
  return *stable_release_latched;
}

static bool AutoCtrlTemplate_LatchPhotoStableRisingAfterLow(
    bool triggered, bool *stable_low_seen, bool *stable_trigger_latched,
    uint32_t *triggered_since_ms, uint32_t *released_since_ms,
    uint32_t now_ms) {
  if (stable_low_seen == 0 || stable_trigger_latched == 0 ||
      triggered_since_ms == 0 || released_since_ms == 0) {
    return false;
  }

  if (*stable_trigger_latched) {
    return true;
  }

  if (!*stable_low_seen) {
    if (triggered) {
      *released_since_ms = 0u;
      return false;
    }

    if (*released_since_ms == 0u) {
      *released_since_ms = now_ms;
      return false;
    }

    if ((now_ms - *released_since_ms) >= AUTO_CTRL_PHOTO_STABLE_MS) {
      *stable_low_seen = true;
      *triggered_since_ms = 0u;
    }
    return false;
  }

  if (!triggered) {
    *triggered_since_ms = 0u;
    return false;
  }

  if (*triggered_since_ms == 0u) {
    *triggered_since_ms = now_ms;
    return false;
  }

  if ((now_ms - *triggered_since_ms) >= AUTO_CTRL_PHOTO_STABLE_MS) {
    *stable_trigger_latched = true;
  }
  return *stable_trigger_latched;
}

static bool AutoCtrlTemplate_LatchFrontPhotoStable(auto_ctrl_t *ctrl,
                                                   uint32_t now_ms) {
  if (!ctrl->feedback.photo_transfer_valid) {
    AutoCtrlTemplate_PauseStableTimer(
        ctrl->template_ctx.pe13_photo1_triggered_latched,
        &ctrl->template_ctx.pe13_photo1_triggered_since_ms, now_ms);
    return false;
  }
  return AutoCtrlTemplate_LatchPhotoStable(
      ctrl->feedback.pe13_photo1_triggered,
      &ctrl->template_ctx.pe13_photo1_triggered_latched,
      &ctrl->template_ctx.pe13_photo1_triggered_since_ms, now_ms);
}

static bool AutoCtrlTemplate_LatchRearPhotoStable(auto_ctrl_t *ctrl,
                                                  uint32_t now_ms) {
  if (!ctrl->feedback.photo_transfer_valid) {
    AutoCtrlTemplate_PauseStableTimer(
        ctrl->template_ctx.pa2_photo3_triggered_latched,
        &ctrl->template_ctx.pa2_photo3_triggered_since_ms, now_ms);
    return false;
  }
  return AutoCtrlTemplate_LatchPhotoStable(
      ctrl->feedback.pa2_photo3_triggered,
      &ctrl->template_ctx.pa2_photo3_triggered_latched,
      &ctrl->template_ctx.pa2_photo3_triggered_since_ms, now_ms);
}

static bool AutoCtrlTemplate_DescendFirstPhotoFallingStable(auto_ctrl_t *ctrl,
                                                            bool use_400mm,
                                                            uint32_t now_ms) {
  (void)use_400mm;
  if (!ctrl->feedback.photo_transfer_valid) {
    AutoCtrlTemplate_PauseStableTimer(
        ctrl->template_ctx.pe9_photo2_stable_trigger_seen,
        &ctrl->template_ctx.pe9_photo2_triggered_since_ms, now_ms);
    AutoCtrlTemplate_PauseStableTimer(
        ctrl->template_ctx.pe9_photo2_stable_release_latched,
        &ctrl->template_ctx.pe9_photo2_released_since_ms, now_ms);
    return false;
  }
  return AutoCtrlTemplate_LatchPhotoStableFalling(
      ctrl->feedback.pe9_photo2_triggered,
      &ctrl->template_ctx.pe9_photo2_stable_trigger_seen,
      &ctrl->template_ctx.pe9_photo2_stable_release_latched,
      &ctrl->template_ctx.pe9_photo2_triggered_since_ms,
      &ctrl->template_ctx.pe9_photo2_released_since_ms, now_ms);
}

static bool AutoCtrlTemplate_DescendSecondPhotoFallingStable(auto_ctrl_t *ctrl,
                                                             bool use_400mm,
                                                             uint32_t now_ms) {
  (void)use_400mm;
  if (!ctrl->feedback.photo_transfer_valid) {
    AutoCtrlTemplate_PauseStableTimer(
        ctrl->template_ctx.pa0_photo4_stable_trigger_seen,
        &ctrl->template_ctx.pa0_photo4_triggered_since_ms, now_ms);
    AutoCtrlTemplate_PauseStableTimer(
        ctrl->template_ctx.pa0_photo4_stable_release_latched,
        &ctrl->template_ctx.pa0_photo4_released_since_ms, now_ms);
    return false;
  }
  return AutoCtrlTemplate_LatchPhotoStableFalling(
      ctrl->feedback.pa0_photo4_triggered,
      &ctrl->template_ctx.pa0_photo4_stable_trigger_seen,
      &ctrl->template_ctx.pa0_photo4_stable_release_latched,
      &ctrl->template_ctx.pa0_photo4_triggered_since_ms,
      &ctrl->template_ctx.pa0_photo4_released_since_ms, now_ms);
}

static bool AutoCtrlTemplate_Photo4RisingAfterLowStable(auto_ctrl_t *ctrl,
                                                        uint32_t now_ms) {
  if (!ctrl->feedback.photo_transfer_valid) {
    AutoCtrlTemplate_PauseStableTimer(
        ctrl->template_ctx.pa0_photo4_stable_low_seen,
        &ctrl->template_ctx.pa0_photo4_released_since_ms, now_ms);
    AutoCtrlTemplate_PauseStableTimer(
        ctrl->template_ctx.pa0_photo4_triggered_latched,
        &ctrl->template_ctx.pa0_photo4_triggered_since_ms, now_ms);
    return false;
  }
  return AutoCtrlTemplate_LatchPhotoStableRisingAfterLow(
      ctrl->feedback.pa0_photo4_triggered,
      &ctrl->template_ctx.pa0_photo4_stable_low_seen,
      &ctrl->template_ctx.pa0_photo4_triggered_latched,
      &ctrl->template_ctx.pa0_photo4_triggered_since_ms,
      &ctrl->template_ctx.pa0_photo4_released_since_ms, now_ms);
}

static void AutoCtrlTemplate_ResetPhoto4RisingAfterLow(auto_ctrl_t *ctrl) {
  ctrl->template_ctx.pa0_photo4_triggered_latched = false;
  ctrl->template_ctx.pa0_photo4_triggered_since_ms = 0u;
  ctrl->template_ctx.pa0_photo4_released_since_ms = 0u;
  ctrl->template_ctx.pa0_photo4_stable_low_seen = false;
}

static bool AutoCtrlTemplate_FinalPhotoSprintReady(
    auto_ctrl_t *ctrl, uint32_t now_ms, const AutoCtrl_TemplateParam_t *param) {
  if (!ctrl->template_ctx.final_photo_sprint_started) {
    if (!AutoCtrlTemplate_Photo4RisingAfterLowStable(ctrl, now_ms)) {
      return false;
    }

    ctrl->template_ctx.final_photo_sprint_started = true;
    ctrl->template_ctx.final_photo_sprint_start_ms = now_ms;
    AutoCtrlTemplate_DebugMarkPhotoEvent(
      ctrl, now_ms, AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_FINAL_RISING);
  }

  const uint32_t sprint_ms = (param == 0) ? 0u : param->final_photo_sprint_ms;
  return (now_ms - ctrl->template_ctx.final_photo_sprint_start_ms) >=
         sprint_ms;
}

static bool AutoCtrlTemplate_PhotoStopSettled(auto_ctrl_t *ctrl,
                                              uint32_t now_ms,
                                              uint32_t settle_ms) {
  if (!ctrl->template_ctx.photo_stop_entered) {
    ctrl->template_ctx.photo_stop_entered = true;
    ctrl->template_ctx.photo_stop_enter_time_ms = now_ms;
  }

  return (now_ms - ctrl->template_ctx.photo_stop_enter_time_ms) >= settle_ms;
}

static bool AutoCtrlTemplate_CommandPhotoStopAndPole(
  auto_ctrl_t *ctrl, uint32_t now_ms, uint32_t settle_ms, bool lock_chassis_zero,
  float settle_vx,
  float hold_front_target, float hold_rear_target, float hold_front_speed,
  float hold_rear_speed, float front_target, float rear_target,
  float front_speed, float rear_speed,
  auto_ctrl_pole_profile_e front_profile,
  auto_ctrl_pole_profile_e rear_profile) {
  if (lock_chassis_zero) {
    AutoCtrlTemplate_CommandChassisZeroVector(ctrl);
  } else {
    AutoCtrlPrimitive_CommandFlatMoveWithYawRate(ctrl, settle_vx);
  }
  if (!AutoCtrlTemplate_PhotoStopSettled(ctrl, now_ms, settle_ms)) {
    AutoCtrlTemplate_CommandPole(ctrl, hold_front_target, hold_rear_target,
                                 hold_front_speed, hold_rear_speed);
    AutoCtrlTemplate_DebugMarkPoleCommand(
        ctrl, now_ms, AUTO_CTRL_TEMPLATE_DEBUG_POLE_HOLD);
    return false;
  }

  AutoCtrlTemplate_CommandPoleProfile(ctrl, front_target, rear_target,
                                      front_speed, rear_speed, front_profile,
                                      rear_profile);
  AutoCtrlTemplate_DebugMarkPoleCommand(
      ctrl, now_ms, AUTO_CTRL_TEMPLATE_DEBUG_POLE_AFTER_PHOTO);
  return true;
}

static bool AutoCtrlTemplate_RunHeadAscendOptimized(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param,
    const AutoCtrl_TemplateParam_t *param, bool use_400mm) {
  auto_ctrl_pole_targets_t pole;
  AutoCtrlTemplate_SelectPoleTargets(robot_param, use_400mm, false, &pole);
  const float photo_record_lift_rad =
      use_400mm ? AUTO_CTRL_ASCEND_400_PHOTO_RECORD_LIFT_RAD
                : AUTO_CTRL_ASCEND_200_PHOTO_RECORD_LIFT_RAD;

  switch (ctrl->template_ctx.step_index) {
    case 0: /* 四杆全伸并前进，等待四杆到位�?*/
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, param->pole_extend_move_speed, 0.0f);
      AutoCtrlTemplate_CommandPoleProfile(
          ctrl, pole.all_extend[0], pole.all_extend[1],
          param->pole_all_extend_speed, param->pole_all_extend_speed,
          AUTO_CTRL_POLE_PROFILE_ALL_EXTEND,
          AUTO_CTRL_POLE_PROFILE_ALL_EXTEND);
      if (ctrl->feedback.pole_front_lift_rad >= photo_record_lift_rad &&
          ctrl->feedback.pole_rear_lift_rad >= photo_record_lift_rad) {
        (void)AutoCtrlTemplate_LatchFrontPhotoStable(ctrl, now_ms);
      }
      if (AutoCtrlTemplate_PoleReadyAfterNewTarget(
              ctrl, ctrl->feedback.pole_all_at_target)) {
        AutoCtrlTemplate_NextStep(ctrl);
      } else if (ctrl->use_fused_template_params &&
                 AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                     AUTO_CTRL_FUSED_CONSTRAINT_SUCCESS_TIMEOUT_MS) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 1: /* 等待前光电连续稳定触发�?*/
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      const bool first_photo_ready =
          ctrl->feedback.pole_all_at_target &&
          AutoCtrlTemplate_LatchFrontPhotoStable(ctrl, now_ms);
      if (!first_photo_ready) {
        if ((ctrl->use_fused_template_params &&
             AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                 AUTO_CTRL_FUSED_CONSTRAINT_SUCCESS_TIMEOUT_MS) ||
            (param->front_photo_timeout_ms > 0u &&
             AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                 param->front_photo_timeout_ms)) {
          if (ctrl->use_fused_template_params) {
            AutoCtrlTemplate_NextStep(ctrl);
          } else {
            ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
          }
        }
        AutoCtrlPrimitive_ApplyPrealignWithMove(
            ctrl, param->pole_extend_move_speed, 0.0f);
        AutoCtrlTemplate_CommandPoleProfile(
          ctrl, pole.all_extend[0], pole.all_extend[1],
          param->pole_front_extend_speed, param->pole_rear_extend_speed,
          AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND,
          AUTO_CTRL_POLE_PROFILE_REAR_EXTEND);
        return false;
      }
      AutoCtrlTemplate_DebugMarkPhotoEvent(
          ctrl, now_ms, AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_ASCEND_FRONT);
        if (!AutoCtrlTemplate_AscendFrontRetractDelayElapsed(
            ctrl, now_ms, param->first_photo_pole_delay_ms)) {
        AutoCtrlPrimitive_ApplyPrealignWithMove(
            ctrl, param->pole_extend_move_speed, 0.0f);
        AutoCtrlTemplate_CommandPoleProfile(
            ctrl, pole.all_extend[0], pole.all_extend[1],
            param->pole_front_extend_speed, param->pole_rear_extend_speed,
            AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND,
            AUTO_CTRL_POLE_PROFILE_REAR_EXTEND);
        return false;
      }
      AutoCtrlTemplate_NextStep(ctrl);
      return false;

    case 2: /* 停车并收前杆，等待前杆到位�?*/
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (use_400mm) {
        AutoCtrlTemplate_CommandChassisZeroVector(ctrl);
      } else {
        AutoCtrlPrimitive_CommandFlatMove(ctrl, 0.4f);
      }
      AutoCtrlTemplate_CommandPoleProfile(
          ctrl, pole.front_retract[0], pole.front_retract[1],
          param->pole_front_retract_speed, param->pole_rear_extend_speed,
          AUTO_CTRL_POLE_PROFILE_FRONT_RETRACT,
          AUTO_CTRL_POLE_PROFILE_REAR_EXTEND);
      if (ctrl->feedback.pole_rear_lift_rad >= photo_record_lift_rad) {
        (void)AutoCtrlTemplate_LatchRearPhotoStable(ctrl, now_ms);
      }
        AutoCtrlTemplate_DebugMarkPoleCommand(
          ctrl, now_ms, AUTO_CTRL_TEMPLATE_DEBUG_POLE_AFTER_PHOTO);
      if (AutoCtrlTemplate_PoleReadyAfterNewTarget(
              ctrl, ctrl->feedback.pole_front_at_target)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      else if ((ctrl->use_fused_template_params &&
                AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                    AUTO_CTRL_FUSED_CONSTRAINT_SUCCESS_TIMEOUT_MS) ||
               (param->front_retract_timeout_ms > 0u &&
                AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                    param->front_retract_timeout_ms)) {
        if (ctrl->use_fused_template_params) {
          AutoCtrlTemplate_NextStep(ctrl);
        } else {
          ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
        }
      }
      return false;

    case 3: /* 前杆收回后的中段定时移动�?*/
      if (ctrl->feedback.pole_rear_lift_rad >= photo_record_lift_rad) {
        (void)AutoCtrlTemplate_LatchRearPhotoStable(ctrl, now_ms);
      }
      if (use_400mm && ctrl->feedback.pole_front_at_target &&
          ctrl->feedback.pole_rear_at_target &&
          AutoCtrlTemplate_LatchRearPhotoStable(ctrl, now_ms)) {
        AutoCtrlTemplate_DebugMarkPhotoEvent(
            ctrl, now_ms, AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_ASCEND_REAR);
        if (!AutoCtrlTemplate_AscendRearRetractDelayElapsed(
          ctrl, now_ms, param->second_photo_pole_delay_ms)) {
          AutoCtrlPrimitive_ApplyPrealignWithMove(
              ctrl, param->mid_move_speed, 0.0f);
          AutoCtrlTemplate_CommandPoleProfile(
              ctrl, pole.front_retract[0], pole.front_retract[1],
              param->pole_front_retract_speed,
              param->pole_rear_extend_speed,
              AUTO_CTRL_POLE_PROFILE_FRONT_RETRACT,
              AUTO_CTRL_POLE_PROFILE_REAR_EXTEND);
          return false;
        }
        AutoCtrlTemplate_CommandChassisZeroVector(ctrl);
        AutoCtrlTemplate_CommandPoleProfile(
            ctrl, pole.all_retract[0], pole.all_retract[1],
            param->pole_front_retract_speed, param->pole_rear_retract_speed,
            AUTO_CTRL_POLE_PROFILE_ALL_RETRACT,
            AUTO_CTRL_POLE_PROFILE_ALL_RETRACT);
        AutoCtrlTemplate_DebugMarkPoleCommand(
          ctrl, now_ms, AUTO_CTRL_TEMPLATE_DEBUG_POLE_AFTER_PHOTO);
        AutoCtrlTemplate_SetStep(ctrl, 5u);
        return false;
      }
      AutoCtrlPrimitive_ApplyPrealignWithMove(ctrl, param->mid_move_speed,
                                              0.0f);
      AutoCtrlTemplate_CommandPoleProfile(
          ctrl, pole.front_retract[0], pole.front_retract[1],
          param->pole_front_retract_speed, param->pole_rear_extend_speed,
          AUTO_CTRL_POLE_PROFILE_FRONT_RETRACT,
          AUTO_CTRL_POLE_PROFILE_REAR_EXTEND);
      if (AutoCtrlTemplate_WheelDeltaMoveReady(
              ctrl, now_ms, param->mid_move_wheel_delta_rad,
              param->mid_move_ms,
              param, robot_param)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 4: /* 等待后光电连续稳定触发�?*/
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      bool second_photo_latched =
          ctrl->template_ctx.pa2_photo3_triggered_latched;
      if (ctrl->feedback.pole_rear_lift_rad >= photo_record_lift_rad) {
        second_photo_latched =
            AutoCtrlTemplate_LatchRearPhotoStable(ctrl, now_ms);
      }
      const bool second_photo_ready = second_photo_latched &&
          ctrl->feedback.pole_front_at_target &&
          ctrl->feedback.pole_rear_at_target;
      if (!second_photo_ready) {
        if ((ctrl->use_fused_template_params &&
             AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                 AUTO_CTRL_FUSED_CONSTRAINT_SUCCESS_TIMEOUT_MS) ||
            (param->rear_photo_timeout_ms > 0u &&
             AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                 param->rear_photo_timeout_ms)) {
          if (ctrl->use_fused_template_params) {
            AutoCtrlTemplate_NextStep(ctrl);
          } else {
            ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
          }
        }
        AutoCtrlPrimitive_ApplyPrealignWithMove(
            ctrl, param->rear_retract_move_speed, 0.0f);
        AutoCtrlTemplate_CommandPoleProfile(
          ctrl, pole.front_retract[0], pole.front_retract[1],
          param->pole_front_retract_speed, param->pole_rear_extend_speed,
          AUTO_CTRL_POLE_PROFILE_FRONT_RETRACT,
          AUTO_CTRL_POLE_PROFILE_REAR_EXTEND);
        return false;
      }
      AutoCtrlTemplate_DebugMarkPhotoEvent(
          ctrl, now_ms, AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_ASCEND_REAR);
      if (!AutoCtrlTemplate_AscendRearRetractDelayElapsed(
              ctrl, now_ms, param->second_photo_pole_delay_ms)) {
        AutoCtrlPrimitive_ApplyPrealignWithMove(
            ctrl, param->rear_retract_move_speed, 0.0f);
        AutoCtrlTemplate_CommandPoleProfile(
            ctrl, pole.front_retract[0], pole.front_retract[1],
            param->pole_front_retract_speed, param->pole_rear_extend_speed,
            AUTO_CTRL_POLE_PROFILE_FRONT_RETRACT,
            AUTO_CTRL_POLE_PROFILE_REAR_EXTEND);
        return false;
      }
      if (use_400mm) {
        AutoCtrlTemplate_CommandChassisZeroVector(ctrl);
      }
      AutoCtrlTemplate_CommandPoleProfile(
          ctrl, pole.all_retract[0], pole.all_retract[1],
          param->pole_front_retract_speed, param->pole_rear_retract_speed,
          AUTO_CTRL_POLE_PROFILE_FRONT_RETRACT,
          AUTO_CTRL_POLE_PROFILE_REAR_RETRACT);
      AutoCtrlTemplate_DebugMarkPoleCommand(
          ctrl, now_ms, AUTO_CTRL_TEMPLATE_DEBUG_POLE_AFTER_PHOTO);
      if (use_400mm) {
        AutoCtrlTemplate_SetStep(ctrl, 5u);
      } else {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 5: /* 停车并四杆全收，等待四杆到位�?*/
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (use_400mm) {
        AutoCtrlTemplate_CommandChassisZeroVector(ctrl);
      } else {
        AutoCtrlPrimitive_CommandFlatMove(ctrl, 0.4f);
      }
      AutoCtrlTemplate_CommandPoleProfile(
          ctrl, pole.all_retract[0], pole.all_retract[1],
          param->pole_front_retract_speed, param->pole_rear_retract_speed,
          use_400mm ? AUTO_CTRL_POLE_PROFILE_ALL_RETRACT
              : AUTO_CTRL_POLE_PROFILE_FRONT_RETRACT,
          use_400mm ? AUTO_CTRL_POLE_PROFILE_ALL_RETRACT
              : AUTO_CTRL_POLE_PROFILE_REAR_RETRACT);
      AutoCtrlTemplate_DebugMarkPoleCommand(
          ctrl, now_ms, AUTO_CTRL_TEMPLATE_DEBUG_POLE_AFTER_PHOTO);
      if (AutoCtrlTemplate_PoleReadyAfterNewTarget(
              ctrl, ctrl->feedback.pole_all_at_target)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      else if ((ctrl->use_fused_template_params &&
                AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                    AUTO_CTRL_FUSED_CONSTRAINT_SUCCESS_TIMEOUT_MS) ||
               (param->rear_retract_timeout_ms > 0u &&
                AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                    param->rear_retract_timeout_ms)) {
        if (ctrl->use_fused_template_params) {
          AutoCtrlTemplate_NextStep(ctrl);
        } else {
          ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
        }
      }
      return false;

    case 6: /* 四杆全收后的离开移动�?*/
      if (!ctrl->template_ctx.step_entered) {
        AutoCtrlTemplate_ResetPhoto4RisingAfterLow(ctrl);
      }
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithMove(ctrl, param->final_move_speed,
                                              0.0f);
      AutoCtrlTemplate_CommandPoleProfile(
          ctrl, pole.all_retract[0], pole.all_retract[1],
          param->pole_front_retract_speed, param->pole_rear_retract_speed,
          use_400mm ? AUTO_CTRL_POLE_PROFILE_ALL_RETRACT
              : AUTO_CTRL_POLE_PROFILE_FRONT_RETRACT,
          use_400mm ? AUTO_CTRL_POLE_PROFILE_ALL_RETRACT
              : AUTO_CTRL_POLE_PROFILE_REAR_RETRACT);
      if (AutoCtrlTemplate_FinalPhotoSprintReady(ctrl, now_ms, param)) {
        AutoCtrlTemplate_NextStep(ctrl);
      } else if (!ctrl->template_ctx.final_photo_sprint_started &&
                 param->final_move_ms > 0u &&
                 AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                     param->final_move_ms) {
        /* The final photo4 rising edge is a preferred clearance marker, not
         * the only safe completion source.  At a boundary position photo4
         * can remain low until the chassis is nudged, leaving an otherwise
         * completed stair action waiting on vehicle motion.  The configured
         * final_move_ms is the bounded travel fallback: after continuously
         * commanding the final escape move for that duration, continue to
         * the final all-poles-retracted verification below. */
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 7: /* 停车并保持四杆全收，全部到位后模板完成。 */
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlTemplate_CommandChassisZeroVector(ctrl);
      AutoCtrlTemplate_CommandPoleProfile(
        ctrl, pole.all_retract[0], pole.all_retract[1],
        param->pole_front_retract_speed, param->pole_rear_retract_speed,
        use_400mm ? AUTO_CTRL_POLE_PROFILE_ALL_RETRACT
          : AUTO_CTRL_POLE_PROFILE_FRONT_RETRACT,
        use_400mm ? AUTO_CTRL_POLE_PROFILE_ALL_RETRACT
          : AUTO_CTRL_POLE_PROFILE_REAR_RETRACT);
      AutoCtrlTemplate_DebugMarkPoleCommand(
        ctrl, now_ms, AUTO_CTRL_TEMPLATE_DEBUG_POLE_AFTER_PHOTO);
      return ctrl->feedback.pole_all_at_target ||
             (ctrl->use_fused_template_params &&
              AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                  AUTO_CTRL_FUSED_CONSTRAINT_SUCCESS_TIMEOUT_MS);

    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}

static bool AutoCtrlTemplate_RunHeadDescendOptimized(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param,
    const AutoCtrl_TemplateParam_t *param, bool use_400mm) {
  auto_ctrl_pole_targets_t pole;
  AutoCtrlTemplate_SelectPoleTargets(robot_param, use_400mm, true, &pole);

  switch (ctrl->template_ctx.step_index) {
    case 0: /* 头向下台阶起步，杆到起步高度后快速接近�?*/
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      /* Down-200: latch a complete first-photo falling edge during the
       * preceding sprint so a pulse cannot disappear across the step change. */
      const bool descend_start_sprint_ready =
          AutoCtrlTemplate_RunDescendStartSprint(
              ctrl, now_ms, param, param->mid_move_speed,
              pole.all_retract[0], pole.all_retract[1],
              param->pole_front_retract_speed,
              param->pole_rear_retract_speed);
      if (ctrl->feedback.pole_front_lift_rad <=
              AUTO_CTRL_DESCEND_PHOTO_RECORD_LIFT_RAD &&
          ctrl->feedback.pole_rear_lift_rad <=
              AUTO_CTRL_DESCEND_PHOTO_RECORD_LIFT_RAD) {
        (void)AutoCtrlTemplate_DescendFirstPhotoFallingStable(
            ctrl, use_400mm, now_ms);
      }
      if (descend_start_sprint_ready) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 1: /* 慢速捕获第一个稳定下降沿，随后伸前杆�?*/
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      bool first_photo_latched =
          ctrl->template_ctx.pe9_photo2_stable_release_latched;
      if (ctrl->feedback.pole_front_lift_rad <=
              AUTO_CTRL_DESCEND_PHOTO_RECORD_LIFT_RAD &&
          ctrl->feedback.pole_rear_lift_rad <=
              AUTO_CTRL_DESCEND_PHOTO_RECORD_LIFT_RAD) {
        first_photo_latched = AutoCtrlTemplate_DescendFirstPhotoFallingStable(
            ctrl, use_400mm, now_ms);
      }
      if (first_photo_latched) {
        AutoCtrlTemplate_DebugMarkPhotoEvent(
          ctrl, now_ms,
          AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_DESCEND_FIRST_FALLING);
        if (AutoCtrlTemplate_CommandPhotoStopAndPole(
          ctrl, now_ms, param->first_photo_pole_delay_ms, use_400mm,
            use_400mm ? 0.0f : param->rear_retract_move_speed,
                pole.all_retract[0], pole.all_retract[1],
                param->pole_front_retract_speed,
                param->pole_rear_retract_speed,
                AutoCtrlTemplate_DescendLandingApproachTarget(
                    param, pole.all_retract[0], pole.all_extend[0]),
                pole.all_retract[1],
                param->pole_front_extend_speed,
                param->pole_rear_retract_speed,
                AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND,
                AUTO_CTRL_POLE_PROFILE_REAR_RETRACT)) {
          AutoCtrlTemplate_NextStep(ctrl);
        }
        return false;
      }
      AutoCtrlPrimitive_CommandFlatMoveWithYawRate(
          ctrl, param->rear_retract_move_speed);
      AutoCtrlTemplate_CommandPoleProfile(
          ctrl, pole.all_retract[0], pole.all_retract[1],
          param->pole_front_retract_speed, param->pole_rear_retract_speed,
          AUTO_CTRL_POLE_PROFILE_ALL_RETRACT,
          AUTO_CTRL_POLE_PROFILE_ALL_RETRACT);
      if (param->rear_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              param->rear_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 2: /* 低速前进并等待前杆支撑到位�?*/
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (use_400mm) {
        AutoCtrlTemplate_CommandChassisZeroVector(ctrl);
      } else {
        AutoCtrlPrimitive_CommandFlatMoveWithYawRate(ctrl, 0.2f);
      }
      const float front_landing_approach =
          AutoCtrlTemplate_DescendLandingApproachTarget(
              param, pole.all_retract[0], pole.all_extend[0]);
      if (front_landing_approach < pole.all_extend[0] &&
          !ctrl->template_ctx.descend_landing_approach_done) {
        AutoCtrlTemplate_CommandPoleProfile(
            ctrl, front_landing_approach, pole.all_retract[1],
            param->pole_front_extend_speed, param->pole_rear_retract_speed,
            AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND,
            AUTO_CTRL_POLE_PROFILE_REAR_RETRACT);
        if (AutoCtrlTemplate_PoleReadyAfterNewTarget(
                ctrl, ctrl->feedback.pole_front_at_target)) {
          ctrl->template_ctx.descend_landing_approach_done = true;
          ctrl->template_ctx.pole_target_seen_not_ready = false;
        }
        return false;
      }
      AutoCtrlTemplate_CommandPoleProfileWithLanding(
          ctrl, param, pole.all_extend[0], pole.all_retract[1],
          param->pole_front_extend_speed, param->pole_rear_retract_speed,
          AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND,
          AUTO_CTRL_POLE_PROFILE_REAR_RETRACT, true, false);
      if (AutoCtrlTemplate_PoleReadyAfterNewTarget(
              ctrl, ctrl->feedback.pole_front_at_target)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 3: /* 前杆支撑后的第二段定时接近�?*/
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (ctrl->feedback.pole_rear_lift_rad <=
          AUTO_CTRL_DESCEND_PHOTO_RECORD_LIFT_RAD) {
        (void)AutoCtrlTemplate_DescendSecondPhotoFallingStable(
            ctrl, use_400mm, now_ms);
      }
      const uint32_t rear_retract_move_ms =
          AutoCtrlTemplate_DescendRearRetractMoveMs(ctrl, param);
      const float rear_retract_wheel_delta_rad =
          AutoCtrlTemplate_DescendRearRetractWheelDeltaRad(ctrl, param);
      if (rear_retract_move_ms > 0u || rear_retract_wheel_delta_rad > 0.0f) {
        AutoCtrlPrimitive_ApplyPrealignWithMove(ctrl, param->mid_move_speed,
                                                0.0f);
      } else {
        AutoCtrlPrimitive_CommandFlatMoveWithYawRate(ctrl, 0.0f);
      }
      AutoCtrlTemplate_CommandPoleProfileWithLanding(
          ctrl, param, pole.all_extend[0], pole.all_retract[1],
          param->pole_front_extend_speed, param->pole_rear_retract_speed,
          AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND,
          AUTO_CTRL_POLE_PROFILE_REAR_RETRACT, true, false);
      if (AutoCtrlTemplate_WheelDeltaMoveReady(
              ctrl, now_ms, rear_retract_wheel_delta_rad,
              rear_retract_move_ms, param, robot_param)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 4: /* 慢速捕获第二个稳定下降沿，随后四杆全伸�?*/
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      bool second_photo_latched =
          ctrl->template_ctx.pa0_photo4_stable_release_latched;
      if (ctrl->feedback.pole_rear_lift_rad <=
          AUTO_CTRL_DESCEND_PHOTO_RECORD_LIFT_RAD) {
        second_photo_latched = AutoCtrlTemplate_DescendSecondPhotoFallingStable(
            ctrl, use_400mm, now_ms);
      }
      if (second_photo_latched) {
        AutoCtrlTemplate_DebugMarkPhotoEvent(
          ctrl, now_ms,
          AUTO_CTRL_TEMPLATE_DEBUG_PHOTO_DESCEND_SECOND_FALLING);
        if (AutoCtrlTemplate_CommandPhotoStopAndPole(
          ctrl, now_ms, param->second_photo_pole_delay_ms, use_400mm,
            use_400mm ? 0.0f : param->front_retract_move_speed,
                pole.all_extend[0], pole.all_retract[1],
                param->pole_front_extend_speed,
                param->pole_rear_retract_speed,
                pole.all_extend[0],
                AutoCtrlTemplate_DescendLandingApproachTarget(
                    param, pole.all_retract[1], pole.all_extend[1]),
                param->pole_front_extend_speed,
                param->pole_rear_extend_speed,
                AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND,
                AUTO_CTRL_POLE_PROFILE_REAR_EXTEND)) {
          AutoCtrlTemplate_NextStep(ctrl);
        }
        return false;
      }
      AutoCtrlPrimitive_CommandFlatMoveWithYawRate(
          ctrl, param->front_retract_move_speed);
      AutoCtrlTemplate_CommandPoleProfileWithLanding(
          ctrl, param, pole.all_extend[0], pole.all_retract[1],
          param->pole_front_extend_speed, param->pole_rear_retract_speed,
          AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND,
          AUTO_CTRL_POLE_PROFILE_REAR_RETRACT, true, false);
      if (param->front_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              param->front_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 5: /* 低速前进并等待四杆支撑到位�?*/
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (use_400mm) {
        AutoCtrlTemplate_CommandChassisZeroVector(ctrl);
      } else {
        AutoCtrlPrimitive_CommandFlatMoveWithYawRate(ctrl, 0.2f);
      }
      const float rear_landing_approach =
          AutoCtrlTemplate_DescendLandingApproachTarget(
              param, pole.all_retract[1], pole.all_extend[1]);
      if (rear_landing_approach < pole.all_extend[1] &&
          !ctrl->template_ctx.descend_landing_approach_done) {
        AutoCtrlTemplate_CommandPoleProfileWithLanding(
            ctrl, param, pole.all_extend[0], rear_landing_approach,
            param->pole_front_extend_speed, param->pole_rear_extend_speed,
            AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND,
            AUTO_CTRL_POLE_PROFILE_REAR_EXTEND, true, false);
        if (AutoCtrlTemplate_PoleReadyAfterNewTarget(
                ctrl, ctrl->feedback.pole_rear_at_target)) {
          ctrl->template_ctx.descend_landing_approach_done = true;
          ctrl->template_ctx.pole_target_seen_not_ready = false;
        }
        return false;
      }
      AutoCtrlTemplate_CommandPoleProfileWithLanding(
          ctrl, param, pole.all_extend[0], pole.all_extend[1],
          param->pole_front_extend_speed, param->pole_rear_extend_speed,
          AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND,
          AUTO_CTRL_POLE_PROFILE_REAR_EXTEND, true, true);
      if (AutoCtrlTemplate_PoleReadyAfterNewTarget(
              ctrl, ctrl->feedback.pole_all_at_target)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 6: /* 四杆全伸支撑通过�?*/
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_CommandFlatMoveWithYawRate(
          ctrl, param->pole_extend_move_speed);
      AutoCtrlTemplate_CommandPoleProfileWithLanding(
          ctrl, param, pole.all_extend[0], pole.all_extend[1],
          param->pole_front_extend_speed, param->pole_rear_extend_speed,
          AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND,
          AUTO_CTRL_POLE_PROFILE_REAR_EXTEND, true, true);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >= param->hold_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 7: /* 四杆保持全伸并离开�?*/
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_CommandFlatMoveWithYawRate(
          ctrl,
          AutoCtrlTemplate_SecondPhotoRetractVx(param, param->final_move_speed));
      AutoCtrlTemplate_CommandPoleProfileWithLanding(
          ctrl, param, pole.all_extend[0], pole.all_extend[1],
          param->pole_front_extend_speed, param->pole_rear_extend_speed,
          AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND,
          AUTO_CTRL_POLE_PROFILE_REAR_EXTEND, true, true);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >= param->final_move_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 8: /* 停车并保持四杆全伸，全部到位后模板完成。 */
      AutoCtrlTemplate_CommandChassisZeroVector(ctrl);
      AutoCtrlTemplate_CommandPoleProfileWithLanding(
          ctrl, param, pole.all_extend[0], pole.all_extend[1],
          param->pole_front_extend_speed, param->pole_rear_extend_speed,
          AUTO_CTRL_POLE_PROFILE_FRONT_EXTEND,
          AUTO_CTRL_POLE_PROFILE_REAR_EXTEND, true, true);
      AutoCtrlTemplate_DebugMarkPoleCommand(
          ctrl, now_ms, AUTO_CTRL_TEMPLATE_DEBUG_POLE_AFTER_PHOTO);
      return ctrl->feedback.pole_all_at_target;

    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}

static bool AutoCtrlTemplate_RunHeadAscend200(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param) {
  const AutoCtrl_TemplateParam_t *param = Config_GetAutoCtrlTemplateParam(
      AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD, ctrl->use_fused_template_params);
  if (param == 0) {
    return false;
  }
  return AutoCtrlTemplate_RunHeadAscendOptimized(
      ctrl, now_ms, robot_param, param, false);
}

static bool AutoCtrlTemplate_RunHeadAscend400(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param) {
  const AutoCtrl_TemplateParam_t *param = Config_GetAutoCtrlTemplateParam(
      AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD, ctrl->use_fused_template_params);
  if (param == 0) {
    return false;
  }
  return AutoCtrlTemplate_RunHeadAscendOptimized(
      ctrl, now_ms, robot_param, param, true);
}

static bool AutoCtrlTemplate_RunHeadDescend200(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param) {
  const AutoCtrl_TemplateParam_t *param = Config_GetAutoCtrlTemplateParam(
      AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD, ctrl->use_fused_template_params);
  if (param == 0) {
    return false;
  }
  return AutoCtrlTemplate_RunHeadDescendOptimized(
      ctrl, now_ms, robot_param, param, false);
}

static bool AutoCtrlTemplate_RunHeadDescend400(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param) {
  const AutoCtrl_TemplateParam_t *param = Config_GetAutoCtrlTemplateParam(
      AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD, ctrl->use_fused_template_params);
  if (param == 0) {
    return false;
  }
  return AutoCtrlTemplate_RunHeadDescendOptimized(
      ctrl, now_ms, robot_param, param, true);
}

bool AutoCtrlTemplate_Run(auto_ctrl_t *ctrl, uint32_t now_ms) {
  const Config_RobotParam_t *robot_param = Config_GetRobotParam();
  if (ctrl == 0 || robot_param == 0) {
    return false;
  }

  if (ctrl->template_ctx.template_start_time_ms == 0u) {
    ctrl->template_ctx.template_start_time_ms = now_ms;
    AutoCtrlTemplate_ResetPhotoDetection(ctrl);
  }

  switch (ctrl->template_id) {
    case AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD:
      return AutoCtrlTemplate_RunHeadAscend200(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD:
      return AutoCtrlTemplate_RunHeadAscend400(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD:
      return AutoCtrlTemplate_RunHeadDescend200(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD:
      return AutoCtrlTemplate_RunHeadDescend400(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_NONE:
    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}
