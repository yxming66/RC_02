#include "module/autoCtrlAPI/template/auto_ctrl_template_runner.h"

#include "module/config.h"
#include "module/autoCtrlAPI/primitive/auto_ctrl_primitive.h"

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
}

static bool AutoCtrlTemplate_RunSinglePoleTarget(auto_ctrl_t *ctrl,
                                                 uint32_t now_ms,
                                                 float front_target,
                                                 float rear_target,
                                                 uint32_t hold_ms) {
  switch (ctrl->template_ctx.step_index) {
    case 0:
      AutoCtrlPrimitive_CommandPoleTarget(ctrl, front_target, rear_target);
      if (AutoCtrlTemplate_StepTimeout(ctrl, now_ms, hold_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 1:
      return true;

    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}

static bool AutoCtrlTemplate_RunFlatMove(auto_ctrl_t *ctrl, uint32_t now_ms) {
  switch (ctrl->template_ctx.step_index) {
    case 0:
      AutoCtrlPrimitive_CommandFlatMove(ctrl, 0.25f);
      if (AutoCtrlTemplate_StepTimeout(ctrl, now_ms, 600u)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 1:
      return true;

    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}

static bool AutoCtrlTemplate_RunCross200(auto_ctrl_t *ctrl,
                                         uint32_t now_ms,
                                         const Config_RobotParam_t *robot_param,
                                         float move_sign) {
  switch (ctrl->template_ctx.step_index) {
    case 0:
      AutoCtrlPrimitive_CommandPoleTarget(
          ctrl, robot_param->pole_param.preset.step_200_all_extend[0],
          robot_param->pole_param.preset.step_200_all_extend[1]);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms, robot_param->auto_ctrl_param.pole_extend_settle_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 1:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, move_sign * robot_param->auto_ctrl_param.climb_forward_kick_speed);
      if (AutoCtrlTemplate_StepTimeout(ctrl, now_ms, 80u)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 2:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_CommandPoleTarget(
          ctrl, robot_param->pole_param.preset.step_200_front_retract[0],
          robot_param->pole_param.preset.step_200_front_retract[1]);
      /* PE13 active-high is interpreted only in this cross-200 state machine. */
      if (ctrl->feedback.front_pole_retracted) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (robot_param->auto_ctrl_param.front_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              robot_param->auto_ctrl_param.front_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 3:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, move_sign * robot_param->auto_ctrl_param.climb_forward_speed);
      if (AutoCtrlTemplate_StepTimeout(ctrl, now_ms, 80u)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 4:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_CommandPoleTarget(
          ctrl, robot_param->pole_param.preset.step_200_all_retract[0],
          robot_param->pole_param.preset.step_200_all_retract[1]);
      /* PE9 active-high is interpreted only in this cross-200 state machine. */
      if (ctrl->feedback.rear_pole_retracted) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (robot_param->auto_ctrl_param.rear_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              robot_param->auto_ctrl_param.rear_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 5:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, move_sign * robot_param->auto_ctrl_param.climb_rear_retract_speed);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms, robot_param->auto_ctrl_param.rear_retract_move_ms)) {
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

static bool AutoCtrlTemplate_RunAscend200(auto_ctrl_t *ctrl,
                                          uint32_t now_ms,
                                          const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_RunCross200(ctrl, now_ms, robot_param, 1.0f);
}

static bool AutoCtrlTemplate_RunDescend200(auto_ctrl_t *ctrl,
                                           uint32_t now_ms,
                                           const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_RunCross200(ctrl, now_ms, robot_param, -1.0f);
}

static bool AutoCtrlTemplate_RunAscend400Std(auto_ctrl_t *ctrl,
                                             uint32_t now_ms,
                                             const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_RunSinglePoleTarget(
      ctrl, now_ms, robot_param->pole_param.preset.step_400_all_extend[0],
      robot_param->pole_param.preset.step_400_all_extend[1], 1200u);
}

static bool AutoCtrlTemplate_RunDescend400Std(auto_ctrl_t *ctrl,
                                              uint32_t now_ms,
                                              const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_RunSinglePoleTarget(
      ctrl, now_ms, robot_param->pole_param.preset.step_400_all_extend[0],
      robot_param->pole_param.preset.step_400_all_extend[1], 1200u);
}

bool AutoCtrlTemplate_Run(auto_ctrl_t *ctrl, uint32_t now_ms) {
  const Config_RobotParam_t *robot_param;

  robot_param = Config_GetRobotParam();

  if (ctrl->template_ctx.template_start_time_ms == 0u) {
    ctrl->template_ctx.template_start_time_ms = now_ms;
  }

  switch (ctrl->template_id) {
    case AUTO_CTRL_TEMPLATE_FLAT_MOVE:
      return AutoCtrlTemplate_RunFlatMove(ctrl, now_ms);

    case AUTO_CTRL_TEMPLATE_ASCEND_200:
      return AutoCtrlTemplate_RunAscend200(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_DESCEND_200:
      return AutoCtrlTemplate_RunDescend200(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_ASCEND_400_STD:
      return AutoCtrlTemplate_RunAscend400Std(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_DESCEND_400_STD:
      return AutoCtrlTemplate_RunDescend400Std(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_NONE:
    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}