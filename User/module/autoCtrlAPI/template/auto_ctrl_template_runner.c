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

#define AUTO_CTRL_ASCEND_PHOTO_STABLE_MS (50u)

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

static bool AutoCtrlTemplate_StepTimeout(auto_ctrl_t *ctrl, uint32_t now_ms,
                                         uint32_t timeout_ms) {
  AutoCtrlTemplate_EnterStep(ctrl, now_ms);
  return AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >= timeout_ms;
}

static void AutoCtrlTemplate_NextStep(auto_ctrl_t *ctrl) {
  ctrl->template_ctx.step_index++;
  ctrl->template_ctx.step_entered = false;
  ctrl->template_ctx.pole_target_seen_not_ready = false;
}

static bool AutoCtrlTemplate_IsYawAligned(const auto_ctrl_t *ctrl) {
  return fabsf(ctrl->yaw_error_rad) <= ctrl->yaw_tolerance_rad;
}

static const AutoCtrl_TemplateParam_t *AutoCtrlTemplate_GetTemplateParam(
    const Config_RobotParam_t *robot_param, auto_ctrl_template_e template_id) {
  if (robot_param == 0) {
    return 0;
  }

  switch (template_id) {
    case AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD:
      return &robot_param->auto_ctrl_param.head_ascend_200;
    case AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD:
      return &robot_param->auto_ctrl_param.head_ascend_400;
    case AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD:
      return &robot_param->auto_ctrl_param.head_descend_200;
    case AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD:
      return &robot_param->auto_ctrl_param.head_descend_400;
    case AUTO_CTRL_TEMPLATE_ASCEND_200_TAIL:
      return &robot_param->auto_ctrl_param.tail_ascend_200;
    case AUTO_CTRL_TEMPLATE_ASCEND_400_TAIL:
      return &robot_param->auto_ctrl_param.tail_ascend_400;
    case AUTO_CTRL_TEMPLATE_DESCEND_200_TAIL:
      return &robot_param->auto_ctrl_param.tail_descend_200;
    case AUTO_CTRL_TEMPLATE_DESCEND_400_TAIL:
      return &robot_param->auto_ctrl_param.tail_descend_400;
    case AUTO_CTRL_TEMPLATE_NONE:
    default:
      return 0;
  }
}

static float AutoCtrlTemplate_ResolvePoleLiftAccel(
    const Config_RobotParam_t *robot_param,
    const AutoCtrl_TemplateParam_t *template_param,
    auto_ctrl_template_e template_id) {
  (void)robot_param;
  (void)template_id;

  if (template_param != 0) {
    return template_param->pole_lift_accel;
  }

  return 0.0f;
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
  const Config_RobotParam_t *robot_param = Config_GetRobotParam();
  const AutoCtrl_TemplateParam_t *template_param =
      AutoCtrlTemplate_GetTemplateParam(robot_param, ctrl->template_id);
  const float lift_accel = AutoCtrlTemplate_ResolvePoleLiftAccel(
      robot_param, template_param, ctrl->template_id);

  AutoCtrlPrimitive_CommandPoleTargetWithSpeed(ctrl, front_target, rear_target,
                                               front_speed, rear_speed);
  /* 0 lets Pole use the global default; a negative value disables accel. */
  ctrl->pole_cmd.auto_lift_accel[0] = lift_accel;
  ctrl->pole_cmd.auto_lift_accel[1] = lift_accel;
  ctrl->pole_cmd.disable_lift_accel = lift_accel < 0.0f;
}

static float AutoCtrlTemplate_SecondPhotoRetractVx(
    const AutoCtrl_TemplateParam_t *param, bool tail_side, float fallback_vx) {
  if (param == 0 || param->second_photo_retract_move_speed <= 0.0f) {
    return fallback_vx;
  }

  const float speed = fabsf(param->second_photo_retract_move_speed);
  return tail_side ? -speed : speed;
}

static void AutoCtrlTemplate_SelectPoleTargets(
    const Config_RobotParam_t *robot_param, bool use_400mm,
    auto_ctrl_pole_targets_t *targets) {
  if (use_400mm) {
    targets->all_extend = robot_param->pole_param.preset.step_400_all_extend;
    targets->front_retract = robot_param->pole_param.preset.step_400_front_retract;
    targets->all_retract = robot_param->pole_param.preset.step_400_all_retract;
    targets->small = robot_param->pole_param.preset.step_200_small;
    return;
  }

  targets->all_extend = robot_param->pole_param.preset.step_200_all_extend;
  targets->front_retract = robot_param->pole_param.preset.step_200_front_retract;
  targets->all_retract = robot_param->pole_param.preset.step_200_all_retract;
  targets->small = robot_param->pole_param.preset.step_200_small;
}

static void AutoCtrlTemplate_ResetEdgeDetection(auto_ctrl_t *ctrl) {
  ctrl->template_ctx.pe13_photo1_prev_triggered =
      ctrl->feedback.pe13_photo1_triggered;
  ctrl->template_ctx.pa2_photo3_prev_triggered =
      ctrl->feedback.pa2_photo3_triggered;
  ctrl->template_ctx.pe9_photo2_prev_triggered =
      ctrl->feedback.pe9_photo2_triggered;
  ctrl->template_ctx.pa0_photo4_prev_triggered =
      ctrl->feedback.pa0_photo4_triggered;
  ctrl->template_ctx.pe13_photo1_falling_edge_latched = false;
  ctrl->template_ctx.pa2_photo3_falling_edge_latched = false;
  ctrl->template_ctx.pe9_photo2_falling_edge_latched = false;
  ctrl->template_ctx.pa0_photo4_falling_edge_latched = false;
  ctrl->template_ctx.pe13_photo1_triggered_since_ms = 0u;
  ctrl->template_ctx.pe9_photo2_triggered_since_ms = 0u;
  ctrl->template_ctx.pa2_photo3_triggered_since_ms = 0u;
  ctrl->template_ctx.pa0_photo4_triggered_since_ms = 0u;
}

static void AutoCtrlTemplate_UpdateEdgeDetection(auto_ctrl_t *ctrl) {
  bool curr;

  curr = ctrl->feedback.pe13_photo1_triggered;
  if (ctrl->template_ctx.pe13_photo1_prev_triggered && !curr) {
    ctrl->template_ctx.pe13_photo1_falling_edge_latched = true;
  }
  ctrl->template_ctx.pe13_photo1_prev_triggered = curr;

  curr = ctrl->feedback.pa2_photo3_triggered;
  if (ctrl->template_ctx.pa2_photo3_prev_triggered && !curr) {
    ctrl->template_ctx.pa2_photo3_falling_edge_latched = true;
  }
  ctrl->template_ctx.pa2_photo3_prev_triggered = curr;

  curr = ctrl->feedback.pe9_photo2_triggered;
  if (ctrl->template_ctx.pe9_photo2_prev_triggered && !curr) {
    ctrl->template_ctx.pe9_photo2_falling_edge_latched = true;
  }
  ctrl->template_ctx.pe9_photo2_prev_triggered = curr;

  curr = ctrl->feedback.pa0_photo4_triggered;
  if (ctrl->template_ctx.pa0_photo4_prev_triggered && !curr) {
    ctrl->template_ctx.pa0_photo4_falling_edge_latched = true;
  }
  ctrl->template_ctx.pa0_photo4_prev_triggered = curr;
}

static bool AutoCtrlTemplate_LatchFrontPhoto(auto_ctrl_t *ctrl, bool tail_side) {
  if (tail_side) {
    if (ctrl->feedback.pe9_photo2_triggered) {
      ctrl->template_ctx.pe9_photo2_triggered_latched = true;
    }
    return ctrl->template_ctx.pe9_photo2_triggered_latched;
  }

  if (ctrl->feedback.pe13_photo1_triggered) {
    ctrl->template_ctx.pe13_photo1_triggered_latched = true;
  }
  return ctrl->template_ctx.pe13_photo1_triggered_latched;
}

static bool AutoCtrlTemplate_LatchRearPhoto(auto_ctrl_t *ctrl, bool tail_side) {
  if (tail_side) {
    if (ctrl->feedback.pa0_photo4_triggered) {
      ctrl->template_ctx.pa0_photo4_triggered_latched = true;
    }
    return ctrl->template_ctx.pa0_photo4_triggered_latched;
  }

  if (ctrl->feedback.pa2_photo3_triggered) {
    ctrl->template_ctx.pa2_photo3_triggered_latched = true;
  }
  return ctrl->template_ctx.pa2_photo3_triggered_latched;
}

static bool AutoCtrlTemplate_LatchFrontPhotoStable(auto_ctrl_t *ctrl,
                                                   bool tail_side,
                                                   uint32_t now_ms) {
  bool triggered;
  bool *latched;
  uint32_t *triggered_since_ms;

  if (tail_side) {
    triggered = ctrl->feedback.pe9_photo2_triggered;
    latched = &ctrl->template_ctx.pe9_photo2_triggered_latched;
    triggered_since_ms =
        &ctrl->template_ctx.pe9_photo2_triggered_since_ms;
  } else {
    triggered = ctrl->feedback.pe13_photo1_triggered;
    latched = &ctrl->template_ctx.pe13_photo1_triggered_latched;
    triggered_since_ms =
        &ctrl->template_ctx.pe13_photo1_triggered_since_ms;
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

  if ((now_ms - *triggered_since_ms) >= AUTO_CTRL_ASCEND_PHOTO_STABLE_MS) {
    *latched = true;
  }
  return *latched;
}

static bool AutoCtrlTemplate_LatchRearPhotoStable(auto_ctrl_t *ctrl,
                                                  bool tail_side,
                                                  uint32_t now_ms) {
  bool triggered;
  bool *latched;
  uint32_t *triggered_since_ms;

  if (tail_side) {
    triggered = ctrl->feedback.pa0_photo4_triggered;
    latched = &ctrl->template_ctx.pa0_photo4_triggered_latched;
    triggered_since_ms =
        &ctrl->template_ctx.pa0_photo4_triggered_since_ms;
  } else {
    triggered = ctrl->feedback.pa2_photo3_triggered;
    latched = &ctrl->template_ctx.pa2_photo3_triggered_latched;
    triggered_since_ms =
        &ctrl->template_ctx.pa2_photo3_triggered_since_ms;
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

  if ((now_ms - *triggered_since_ms) >= AUTO_CTRL_ASCEND_PHOTO_STABLE_MS) {
    *latched = true;
  }
  return *latched;
}

static bool AutoCtrlTemplate_DescendFirstPhotoEdge(const auto_ctrl_t *ctrl,
                                                   bool tail_side,
                                                   bool use_400mm) {
  (void)use_400mm;
  if (tail_side) {
    return ctrl->template_ctx.pa2_photo3_falling_edge_latched;
  }
  return ctrl->template_ctx.pe9_photo2_falling_edge_latched;
}

static bool AutoCtrlTemplate_DescendSecondPhotoEdge(const auto_ctrl_t *ctrl,
                                                    bool tail_side,
                                                    bool use_400mm) {
  (void)use_400mm;
  if (tail_side) {
    return ctrl->template_ctx.pe13_photo1_falling_edge_latched;
  }
  return ctrl->template_ctx.pa0_photo4_falling_edge_latched;
}

static bool AutoCtrlTemplate_RunAscend(auto_ctrl_t *ctrl, uint32_t now_ms,
                                       const Config_RobotParam_t *robot_param,
                                       const AutoCtrl_TemplateParam_t *param,
                                       bool tail_side, bool use_400mm) {
  auto_ctrl_pole_targets_t pole;
  AutoCtrlTemplate_SelectPoleTargets(robot_param, use_400mm, &pole);

  const bool first_photo_ready =
      tail_side ? AutoCtrlTemplate_LatchRearPhoto(ctrl, tail_side)
                : AutoCtrlTemplate_LatchFrontPhoto(ctrl, tail_side);
  const bool second_photo_ready =
      tail_side ? AutoCtrlTemplate_LatchFrontPhoto(ctrl, tail_side)
                : AutoCtrlTemplate_LatchRearPhoto(ctrl, tail_side);
  const uint32_t first_photo_timeout_ms =
      tail_side ? param->rear_photo_timeout_ms : param->front_photo_timeout_ms;
  const uint32_t second_photo_timeout_ms =
      tail_side ? param->front_photo_timeout_ms : param->rear_photo_timeout_ms;

  switch (ctrl->template_ctx.step_index) {
    case 0:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithMove(ctrl, param->align_move_speed,
                                              0.0f);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_retract[0],
                                   pole.all_retract[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_retract_speed);
      if (AutoCtrlTemplate_IsYawAligned(ctrl)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 1:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithMove(ctrl,
                                              param->pole_extend_move_speed,
                                              0.0f);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_extend[0],
                                   pole.all_extend[1],
                                   param->pole_all_extend_speed,
                                   param->pole_all_extend_speed);
      const bool all_poles_ready =
          AutoCtrlTemplate_PoleReadyAfterNewTarget(
              ctrl, ctrl->feedback.pole_all_at_target);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              param->pole_extend_settle_ms &&
          all_poles_ready) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 2:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (!first_photo_ready) {
        if (first_photo_timeout_ms > 0u &&
            AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                first_photo_timeout_ms) {
          ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
        }
        AutoCtrlPrimitive_ApplyPrealignWithMove(
            ctrl, tail_side ? 0.0f : param->pole_extend_move_speed,
            tail_side ? param->front_retract_vy : 0.0f);
        AutoCtrlTemplate_CommandPole(ctrl, pole.all_extend[0],
                                     pole.all_extend[1],
                                     param->pole_front_extend_speed,
                                     param->pole_rear_extend_speed);
        return false;
      }
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, param->front_retract_move_speed, 0.0f);
      AutoCtrlTemplate_CommandPole(ctrl, pole.front_retract[0],
                                   pole.front_retract[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_extend_speed);
      if (AutoCtrlTemplate_PoleReadyAfterNewTarget(
              ctrl, ctrl->feedback.pole_front_at_target)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      if (param->front_retract_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              param->front_retract_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 3:
      AutoCtrlPrimitive_CommandFlatMove(ctrl, param->mid_move_speed);
      AutoCtrlTemplate_CommandPole(ctrl, pole.front_retract[0],
                                   pole.front_retract[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_extend_speed);
      if (AutoCtrlTemplate_StepTimeout(ctrl, now_ms, param->mid_move_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 4:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (!second_photo_ready) {
        if (second_photo_timeout_ms > 0u &&
            AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                second_photo_timeout_ms) {
          ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
        }
        AutoCtrlPrimitive_ApplyPrealignWithMove(ctrl, param->mid_move_speed,
                                                0.0f);
        AutoCtrlTemplate_CommandPole(ctrl, pole.front_retract[0],
                                     pole.front_retract[1],
                                     param->pole_front_retract_speed,
                                     param->pole_rear_extend_speed);
        return false;
      }
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl,
          AutoCtrlTemplate_SecondPhotoRetractVx(
              param, tail_side, param->rear_retract_move_speed),
          0.0f);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_retract[0],
                                   pole.all_retract[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_retract_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
          param->rear_retract_move_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      if (param->rear_retract_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              param->rear_retract_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 5:
      AutoCtrlPrimitive_CommandFlatMove(ctrl, param->final_move_speed);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_retract[0],
                                   pole.all_retract[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_retract_speed);
      if (AutoCtrlTemplate_StepTimeout(ctrl, now_ms, param->final_move_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 6:
      return true;

    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}

static bool AutoCtrlTemplate_RunHeadAscendOptimized(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param,
    const AutoCtrl_TemplateParam_t *param, bool use_400mm) {
  auto_ctrl_pole_targets_t pole;
  AutoCtrlTemplate_SelectPoleTargets(robot_param, use_400mm, &pole);

  switch (ctrl->template_ctx.step_index) {
    case 0:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, param->pole_extend_move_speed, 0.0f);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_extend[0],
                                   pole.all_extend[1],
                                   param->pole_all_extend_speed,
                                   param->pole_all_extend_speed);
      if (AutoCtrlTemplate_PoleReadyAfterNewTarget(
              ctrl, ctrl->feedback.pole_all_at_target)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 1:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      const bool first_photo_ready =
          AutoCtrlTemplate_LatchFrontPhotoStable(ctrl, false, now_ms);
      if (!first_photo_ready) {
        if (param->front_photo_timeout_ms > 0u &&
            AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                param->front_photo_timeout_ms) {
          ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
        }
        AutoCtrlPrimitive_ApplyPrealignWithMove(
            ctrl, param->pole_extend_move_speed, 0.0f);
        AutoCtrlTemplate_CommandPole(ctrl, pole.all_extend[0],
                                     pole.all_extend[1],
                                     param->pole_front_extend_speed,
                                     param->pole_rear_extend_speed);
        return false;
      }
      AutoCtrlTemplate_NextStep(ctrl);
      return false;

    case 2:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, param->front_retract_move_speed, 0.0f);
      AutoCtrlTemplate_CommandPole(ctrl, pole.front_retract[0],
                                   pole.front_retract[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_extend_speed);
      if (AutoCtrlTemplate_PoleReadyAfterNewTarget(
              ctrl, ctrl->feedback.pole_front_at_target)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      if (param->front_retract_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              param->front_retract_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 3:
      AutoCtrlPrimitive_CommandFlatMove(ctrl, param->mid_move_speed);
      AutoCtrlTemplate_CommandPole(ctrl, pole.front_retract[0],
                                   pole.front_retract[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_extend_speed);
      if (AutoCtrlTemplate_StepTimeout(ctrl, now_ms, param->mid_move_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 4:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      const bool second_photo_ready =
          AutoCtrlTemplate_LatchRearPhotoStable(ctrl, false, now_ms);
      if (!second_photo_ready) {
        if (param->rear_photo_timeout_ms > 0u &&
            AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                param->rear_photo_timeout_ms) {
          ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
        }
        AutoCtrlPrimitive_ApplyPrealignWithMove(
            ctrl, param->rear_retract_move_speed, 0.0f);
        AutoCtrlTemplate_CommandPole(ctrl, pole.front_retract[0],
                                     pole.front_retract[1],
                                     param->pole_front_retract_speed,
                                     param->pole_rear_extend_speed);
        return false;
      }
      AutoCtrlTemplate_NextStep(ctrl);
      return false;

    case 5:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl,
          AutoCtrlTemplate_SecondPhotoRetractVx(
              param, false, param->rear_retract_move_speed),
          0.0f);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_retract[0],
                                   pole.all_retract[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_retract_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
          param->rear_retract_move_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      if (param->rear_retract_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              param->rear_retract_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 6:
      AutoCtrlPrimitive_ApplyPrealignWithMove(ctrl, param->final_move_speed,
                                              0.0f);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_retract[0],
                                   pole.all_retract[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_retract_speed);
      if (AutoCtrlTemplate_StepTimeout(ctrl, now_ms, param->final_move_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 7:
      return true;

    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}

static bool AutoCtrlTemplate_RunTailDescendOptimized(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param,
    const AutoCtrl_TemplateParam_t *param, bool use_400mm) {
  auto_ctrl_pole_targets_t pole;
  AutoCtrlTemplate_SelectPoleTargets(robot_param, use_400mm, &pole);

  switch (ctrl->template_ctx.step_index) {
    case 0:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_CommandFlatMove(ctrl, 0.0f);
      AutoCtrlTemplate_CommandPole(ctrl, pole.small[0], pole.small[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_retract_speed);
      AutoCtrlTemplate_NextStep(ctrl);
      return false;

    case 1:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (AutoCtrlTemplate_DescendFirstPhotoEdge(ctrl, true, use_400mm)) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      AutoCtrlPrimitive_CommandFlatMove(ctrl, -param->mid_move_speed);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_retract[0],
                                   pole.all_retract[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_retract_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >= param->mid_move_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 2:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (AutoCtrlTemplate_DescendFirstPhotoEdge(ctrl, true, use_400mm)) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      AutoCtrlPrimitive_CommandFlatMove(ctrl, -param->rear_retract_move_speed);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_retract[0],
                                   pole.all_retract[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_retract_speed);
      if (param->rear_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              param->rear_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 3:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_CommandFlatMove(ctrl, 0.0f);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_retract[0],
                                   pole.all_extend[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_extend_speed);
      if (AutoCtrlTemplate_PoleReadyAfterNewTarget(
              ctrl, ctrl->feedback.pole_rear_at_target)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 4:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (AutoCtrlTemplate_DescendSecondPhotoEdge(ctrl, true, use_400mm)) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      AutoCtrlPrimitive_CommandFlatMove(ctrl, -param->mid_move_speed);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_retract[0],
                                   pole.all_extend[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_extend_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
          param->rear_retract_move_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 5:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (AutoCtrlTemplate_DescendSecondPhotoEdge(ctrl, true, use_400mm)) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      AutoCtrlPrimitive_CommandFlatMove(ctrl, -param->front_retract_move_speed);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_retract[0],
                                   pole.all_extend[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_extend_speed);
      if (param->front_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              param->front_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 6:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_CommandFlatMove(ctrl, 0.0f);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_extend[0],
                                   pole.all_extend[1],
                                   param->pole_front_extend_speed,
                                   param->pole_rear_extend_speed);
      if (AutoCtrlTemplate_PoleReadyAfterNewTarget(
              ctrl, ctrl->feedback.pole_all_at_target)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 7:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_CommandFlatMove(ctrl, -param->pole_extend_move_speed);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_extend[0],
                                   pole.all_extend[1],
                                   param->pole_front_extend_speed,
                                   param->pole_rear_extend_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >= param->hold_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 8:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, AutoCtrlTemplate_SecondPhotoRetractVx(
                    param, true, -param->final_move_speed));
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_retract[0],
                                   pole.all_retract[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_retract_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >= param->final_move_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 9:
      return true;

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
  AutoCtrlTemplate_SelectPoleTargets(robot_param, use_400mm, &pole);

  switch (ctrl->template_ctx.step_index) {
    case 0:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_CommandFlatMove(ctrl, param->mid_move_speed);
      // AutoCtrlTemplate_CommandPole(ctrl,
      //                              use_400mm ? pole.all_retract[0]
      //                                        : pole.small[0],
      //                              use_400mm ? pole.all_retract[1]
      //                                        : pole.small[1],
      //                              param->pole_front_retract_speed,
      //                              param->pole_rear_retract_speed);
      AutoCtrlTemplate_CommandPole(ctrl,
                                  pole.small[0],
                                  pole.small[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_retract_speed);
      
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >= param->mid_move_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 1:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (AutoCtrlTemplate_DescendFirstPhotoEdge(ctrl, false, use_400mm)) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      AutoCtrlPrimitive_CommandFlatMove(ctrl, param->rear_retract_move_speed);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_retract[0],
                                   pole.all_retract[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_retract_speed);
      if (param->rear_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              param->rear_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 2:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_CommandFlatMove(ctrl, 0.0f);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_extend[0],
                                   pole.all_retract[1],
                                   param->pole_front_extend_speed,
                                   param->pole_rear_retract_speed);
      if (AutoCtrlTemplate_PoleReadyAfterNewTarget(
              ctrl, ctrl->feedback.pole_front_at_target)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 3:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_CommandFlatMove(ctrl, param->mid_move_speed);
      AutoCtrlTemplate_CommandPole(ctrl,
                                   use_400mm ? pole.all_extend[0]
                                             : pole.all_extend[0] + 1.0f,
                                   use_400mm ? pole.all_retract[1] + 1.0f
                                             : pole.all_retract[1] + 1.0f,
                                   param->pole_front_extend_speed,
                                   param->pole_rear_retract_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
          param->rear_retract_move_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 4:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (AutoCtrlTemplate_DescendSecondPhotoEdge(ctrl, false, use_400mm)) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      AutoCtrlPrimitive_CommandFlatMove(ctrl, param->front_retract_move_speed);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_extend[0] + 1.0f,
                                   pole.all_retract[1] + 1.0f,
                                   param->pole_front_extend_speed,
                                   param->pole_rear_retract_speed);
      if (param->front_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              param->front_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 5:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_CommandFlatMove(ctrl, 0.0f);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_extend[0],
                                   pole.all_extend[1],
                                   param->pole_front_extend_speed,
                                   param->pole_rear_extend_speed);
      if (AutoCtrlTemplate_PoleReadyAfterNewTarget(
              ctrl, ctrl->feedback.pole_all_at_target)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 6:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_CommandFlatMove(ctrl, param->pole_extend_move_speed);
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_extend[0],
                                   pole.all_extend[1],
                                   param->pole_front_extend_speed,
                                   param->pole_rear_extend_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >= param->hold_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 7:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, AutoCtrlTemplate_SecondPhotoRetractVx(
                    param, false, param->final_move_speed));
      AutoCtrlTemplate_CommandPole(ctrl, pole.all_retract[0],
                                   pole.all_retract[1],
                                   param->pole_front_retract_speed,
                                   param->pole_rear_retract_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >= param->final_move_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 8:
      return true;

    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}

static bool AutoCtrlTemplate_RunHeadAscend200(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_RunHeadAscendOptimized(
      ctrl, now_ms, robot_param,
      &robot_param->auto_ctrl_param.head_ascend_200, false);
}

static bool AutoCtrlTemplate_RunHeadAscend400(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_RunHeadAscendOptimized(
      ctrl, now_ms, robot_param,
      &robot_param->auto_ctrl_param.head_ascend_400, true);
}

static bool AutoCtrlTemplate_RunHeadDescend200(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_RunHeadDescendOptimized(
      ctrl, now_ms, robot_param,
      &robot_param->auto_ctrl_param.head_descend_200, false);
}

static bool AutoCtrlTemplate_RunHeadDescend400(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_RunHeadDescendOptimized(
      ctrl, now_ms, robot_param,
      &robot_param->auto_ctrl_param.head_descend_400, true);
}

static bool AutoCtrlTemplate_RunTailAscend200(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_RunAscend(
      ctrl, now_ms, robot_param, &robot_param->auto_ctrl_param.tail_ascend_200,
      true, false);
}

static bool AutoCtrlTemplate_RunTailAscend400(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_RunAscend(
      ctrl, now_ms, robot_param, &robot_param->auto_ctrl_param.tail_ascend_400,
      true, true);
}

static bool AutoCtrlTemplate_RunTailDescend200(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_RunTailDescendOptimized(
      ctrl, now_ms, robot_param,
      &robot_param->auto_ctrl_param.tail_descend_200, false);
}

static bool AutoCtrlTemplate_RunTailDescend400(
    auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_RunTailDescendOptimized(
      ctrl, now_ms, robot_param,
      &robot_param->auto_ctrl_param.tail_descend_400, true);
}

bool AutoCtrlTemplate_Run(auto_ctrl_t *ctrl, uint32_t now_ms) {
  const Config_RobotParam_t *robot_param = Config_GetRobotParam();
  if (ctrl == 0 || robot_param == 0) {
    return false;
  }

  if (ctrl->template_ctx.template_start_time_ms == 0u) {
    ctrl->template_ctx.template_start_time_ms = now_ms;
    AutoCtrlTemplate_ResetEdgeDetection(ctrl);
  }

  AutoCtrlTemplate_UpdateEdgeDetection(ctrl);

  switch (ctrl->template_id) {
    case AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD:
      return AutoCtrlTemplate_RunHeadAscend200(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD:
      return AutoCtrlTemplate_RunHeadAscend400(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD:
      return AutoCtrlTemplate_RunHeadDescend200(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD:
      return AutoCtrlTemplate_RunHeadDescend400(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_ASCEND_200_TAIL:
      return AutoCtrlTemplate_RunTailAscend200(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_ASCEND_400_TAIL:
      return AutoCtrlTemplate_RunTailAscend400(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_DESCEND_200_TAIL:
      return AutoCtrlTemplate_RunTailDescend200(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_DESCEND_400_TAIL:
      return AutoCtrlTemplate_RunTailDescend400(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_NONE:
    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}
