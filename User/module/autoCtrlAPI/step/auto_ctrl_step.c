#include "module/autoCtrlAPI/step/auto_ctrl_step.h"

/*
 * auto_ctrl_step.c
 *
 * 作用：
 * - 提供可复用的单步动作执行器；
 * - 管理 step 首次进入、计时和超时；
 * - 统一返回 RUNNING / DONE / FAIL，供上层模板编排。
 */

#include <string.h>

#include "module/autoCtrlAPI/primitive/auto_ctrl_primitive.h"

/* 清空 step 运行时状态。 */
void AutoCtrlStep_Reset(auto_ctrl_step_runtime_t *runtime) {
  if (runtime == 0) return;
  memset(runtime, 0, sizeof(*runtime));
}

/*
 * 执行单个 step。
 * - 首次进入会刷新 enter_time 和 timeout；
 * - 根据 step 类型调用对应 primitive；
 * - 对光电等待类 step 内置超时故障上报。
 */
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

    case AUTO_CTRL_STEP_WAIT_HEAD_FRONT_PHOTO:
      if (ctrl->template_id != AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD &&
          ctrl->template_id != AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD) {
        return AUTO_CTRL_STEP_STATUS_RUNNING;
      }
      if (ctrl->feedback.pe13_photo1_triggered) {
        return AUTO_CTRL_STEP_STATUS_DONE;
      }
      if (step->timeout_ms > 0u &&
          (now_ms - runtime->enter_time_ms) >= step->timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
        return AUTO_CTRL_STEP_STATUS_FAIL;
      }
      return AUTO_CTRL_STEP_STATUS_RUNNING;

    case AUTO_CTRL_STEP_WAIT_HEAD_REAR_PHOTO:
      if (ctrl->template_id != AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD &&
          ctrl->template_id != AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD) {
        return AUTO_CTRL_STEP_STATUS_RUNNING;
      }
      if (ctrl->feedback.pa2_photo3_triggered) {
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