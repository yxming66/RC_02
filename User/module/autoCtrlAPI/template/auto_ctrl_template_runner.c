#include "module/autoCtrlAPI/template/auto_ctrl_template_runner.h"

#include <string.h>

#include "module/autoCtrlAPI/step/auto_ctrl_step.h"

static bool AutoCtrlTemplate_RunSequence(auto_ctrl_t *ctrl, uint32_t now_ms,
                                         const auto_ctrl_step_def_t *steps,
                                         uint32_t step_count) {
  auto_ctrl_step_status_e status;
  uint8_t step_index;
  auto_ctrl_step_runtime_t runtime;

  step_index = ctrl->template_ctx.step_index;
  if (step_index >= step_count) {
    return true;
  }

  memset(&runtime, 0, sizeof(runtime));
  runtime.id = (auto_ctrl_step_id_e)ctrl->template_ctx.step_index;
  runtime.entered = ctrl->template_ctx.step_entered;
  runtime.enter_time_ms = ctrl->template_ctx.step_enter_time_ms;

  status = AutoCtrlStep_Run(ctrl, &runtime, &steps[step_index], now_ms);

  ctrl->template_ctx.step_entered = runtime.entered;
  ctrl->template_ctx.step_enter_time_ms = runtime.enter_time_ms;

  if (status == AUTO_CTRL_STEP_STATUS_DONE) {
    ctrl->template_ctx.step_index++;
    ctrl->template_ctx.step_entered = false;
    if (ctrl->template_ctx.step_index >= step_count) {
      return true;
    }
  } else if (status == AUTO_CTRL_STEP_STATUS_FAIL) {
    ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
  }

  return false;
}

bool AutoCtrlTemplate_Run(auto_ctrl_t *ctrl, uint32_t now_ms) {
  static const auto_ctrl_step_def_t k_flat_move_steps[] = {
      {AUTO_CTRL_STEP_FLAT_MOVE_TIME, 0.25f, 0.0f, 600u},
  };

  static const auto_ctrl_step_def_t k_step_200_steps[] = {
      {AUTO_CTRL_STEP_SET_POLE_TARGET, 12.7912035f, 12.7912035f, 900u},
  };

  static const auto_ctrl_step_def_t k_step_400_steps[] = {
      {AUTO_CTRL_STEP_SET_POLE_TARGET, 26.0381184f, 26.0315285f, 1200u},
  };

  if (!ctrl->template_ctx.step_entered && ctrl->template_ctx.template_start_time_ms == 0u) {
    ctrl->template_ctx.template_start_time_ms = now_ms;
  }

  switch (ctrl->template_id) {
    case AUTO_CTRL_TEMPLATE_FLAT_MOVE:
      return AutoCtrlTemplate_RunSequence(ctrl, now_ms, k_flat_move_steps,
                                          sizeof(k_flat_move_steps) / sizeof(k_flat_move_steps[0]));

    case AUTO_CTRL_TEMPLATE_ASCEND_200_STD:
    case AUTO_CTRL_TEMPLATE_DESCEND_200_STD:
    case AUTO_CTRL_TEMPLATE_ASCEND_200_NO_FRONT_SICK:
    case AUTO_CTRL_TEMPLATE_DESCEND_200_NO_FRONT_SICK:
      return AutoCtrlTemplate_RunSequence(ctrl, now_ms, k_step_200_steps,
                                          sizeof(k_step_200_steps) / sizeof(k_step_200_steps[0]));

    case AUTO_CTRL_TEMPLATE_ASCEND_400_STD:
    case AUTO_CTRL_TEMPLATE_DESCEND_400_STD:
      return AutoCtrlTemplate_RunSequence(ctrl, now_ms, k_step_400_steps,
                                          sizeof(k_step_400_steps) / sizeof(k_step_400_steps[0]));

    case AUTO_CTRL_TEMPLATE_NONE:
    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}