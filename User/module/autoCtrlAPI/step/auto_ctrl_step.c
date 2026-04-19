#include "module/autoCtrlAPI/step/auto_ctrl_step.h"

#include <string.h>

#include "module/autoCtrlAPI/primitive/auto_ctrl_primitive.h"

void AutoCtrlStep_Reset(auto_ctrl_step_runtime_t *runtime) {
  if (runtime == 0) return;
  memset(runtime, 0, sizeof(*runtime));
}

auto_ctrl_step_status_e AutoCtrlStep_Run(auto_ctrl_t *ctrl,
                                         auto_ctrl_step_runtime_t *runtime,
                                         const auto_ctrl_step_def_t *step,
                                         uint32_t now_ms) {
  if (ctrl == 0 || runtime == 0 || step == 0) {
    return AUTO_CTRL_STEP_STATUS_FAIL;
  }

  if (!runtime->entered || runtime->id != step->id) {
    runtime->id = step->id;
    runtime->enter_time_ms = now_ms;
    runtime->timeout_ms = step->timeout_ms;
    runtime->entered = true;
  }

  switch (step->id) {
    case AUTO_CTRL_STEP_STOP_CHASSIS:
      AutoCtrlPrimitive_ResetChassis(ctrl);
      return AUTO_CTRL_STEP_STATUS_DONE;

    case AUTO_CTRL_STEP_FLAT_MOVE_TIME:
      AutoCtrlPrimitive_CommandFlatMove(ctrl, step->param0);
      if ((now_ms - runtime->enter_time_ms) >= step->timeout_ms) {
        return AUTO_CTRL_STEP_STATUS_DONE;
      }
      return AUTO_CTRL_STEP_STATUS_RUNNING;

    case AUTO_CTRL_STEP_SET_POLE_TARGET:
      AutoCtrlPrimitive_CommandPoleTarget(ctrl, step->param0, step->param1);
      if ((now_ms - runtime->enter_time_ms) >= step->timeout_ms) {
        return AUTO_CTRL_STEP_STATUS_DONE;
      }
      return AUTO_CTRL_STEP_STATUS_RUNNING;

    case AUTO_CTRL_STEP_WAIT_TIMEOUT:
      if ((now_ms - runtime->enter_time_ms) >= step->timeout_ms) {
        return AUTO_CTRL_STEP_STATUS_DONE;
      }
      return AUTO_CTRL_STEP_STATUS_RUNNING;

    case AUTO_CTRL_STEP_WAIT_FRONT_PHOTO:
      if (ctrl->template_id != AUTO_CTRL_TEMPLATE_ASCEND_200 &&
          ctrl->template_id != AUTO_CTRL_TEMPLATE_DESCEND_200) {
        return AUTO_CTRL_STEP_STATUS_RUNNING;
      }
      if (ctrl->feedback.front_pole_retracted) {
        return AUTO_CTRL_STEP_STATUS_DONE;
      }
      if (step->timeout_ms > 0u &&
          (now_ms - runtime->enter_time_ms) >= step->timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
        return AUTO_CTRL_STEP_STATUS_FAIL;
      }
      return AUTO_CTRL_STEP_STATUS_RUNNING;

    case AUTO_CTRL_STEP_WAIT_REAR_PHOTO:
      if (ctrl->template_id != AUTO_CTRL_TEMPLATE_ASCEND_200 &&
          ctrl->template_id != AUTO_CTRL_TEMPLATE_DESCEND_200) {
        return AUTO_CTRL_STEP_STATUS_RUNNING;
      }
      if (ctrl->feedback.rear_pole_retracted) {
        return AUTO_CTRL_STEP_STATUS_DONE;
      }
      if (step->timeout_ms > 0u &&
          (now_ms - runtime->enter_time_ms) >= step->timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
        return AUTO_CTRL_STEP_STATUS_FAIL;
      }
      return AUTO_CTRL_STEP_STATUS_RUNNING;

    case AUTO_CTRL_STEP_NONE:
    default:
      return AUTO_CTRL_STEP_STATUS_FAIL;
  }
}