#include "module/autoCtrlAPI/template/auto_ctrl_template_runner.h"

#include <string.h>

#include "module/config.h"
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
  const Config_RobotParam_t *robot_param;
  static const auto_ctrl_step_def_t k_flat_move_steps[] = {
      {AUTO_CTRL_STEP_FLAT_MOVE_TIME, 0.25f, 0.0f, 600u},
  };

  static const auto_ctrl_step_def_t k_step_200_legacy_steps[] = {
      {AUTO_CTRL_STEP_SET_POLE_TARGET, 12.7912035f, 12.7912035f, 900u},
  };

  static const auto_ctrl_step_def_t k_step_400_steps[] = {
      {AUTO_CTRL_STEP_SET_POLE_TARGET, 26.0381184f, 26.0315285f, 1200u},
  };

  auto_ctrl_step_def_t k_step_200_ascend_photo_steps[] = {
    {AUTO_CTRL_STEP_SET_POLE_TARGET, 0.0f, 0.0f, 0u},
    {AUTO_CTRL_STEP_FLAT_MOVE_TIME, 0.0f, 0.0f, 80u},
    {AUTO_CTRL_STEP_WAIT_FRONT_PHOTO, 0.0f, 0.0f, 0u},
    {AUTO_CTRL_STEP_STOP_CHASSIS, 0.0f, 0.0f, 0u},
    {AUTO_CTRL_STEP_SET_POLE_TARGET, 0.0f, 0.0f, 0u},
    {AUTO_CTRL_STEP_FLAT_MOVE_TIME, 0.0f, 0.0f, 80u},
    {AUTO_CTRL_STEP_WAIT_REAR_PHOTO, 0.0f, 0.0f, 0u},
    {AUTO_CTRL_STEP_SET_POLE_TARGET, 0.0f, 0.0f, 0u},
    {AUTO_CTRL_STEP_FLAT_MOVE_TIME, 0.0f, 0.0f, 0u},
  };

  robot_param = Config_GetRobotParam();

  k_step_200_ascend_photo_steps[0].param0 =
    robot_param->pole_param.preset.step_200_all_extend[0];
  k_step_200_ascend_photo_steps[0].param1 =
    robot_param->pole_param.preset.step_200_all_extend[1];
  k_step_200_ascend_photo_steps[0].timeout_ms = robot_param->auto_ctrl_param.pole_extend_settle_ms;

  k_step_200_ascend_photo_steps[1].param0 = robot_param->auto_ctrl_param.climb_forward_kick_speed;

  k_step_200_ascend_photo_steps[2].timeout_ms = robot_param->auto_ctrl_param.front_photo_timeout_ms;

  k_step_200_ascend_photo_steps[4].param0 =
    robot_param->pole_param.preset.step_200_front_retract[0];
  k_step_200_ascend_photo_steps[4].param1 =
    robot_param->pole_param.preset.step_200_front_retract[1];
  k_step_200_ascend_photo_steps[4].timeout_ms = robot_param->auto_ctrl_param.front_retract_settle_ms;

  k_step_200_ascend_photo_steps[5].param0 = robot_param->auto_ctrl_param.climb_forward_speed;

  k_step_200_ascend_photo_steps[6].timeout_ms = robot_param->auto_ctrl_param.rear_photo_timeout_ms;

  k_step_200_ascend_photo_steps[7].param0 =
    robot_param->pole_param.preset.step_200_all_retract[0];
  k_step_200_ascend_photo_steps[7].param1 =
    robot_param->pole_param.preset.step_200_all_retract[1];
  k_step_200_ascend_photo_steps[7].timeout_ms = robot_param->auto_ctrl_param.front_retract_settle_ms;

  k_step_200_ascend_photo_steps[8].param0 = robot_param->auto_ctrl_param.climb_rear_retract_speed;
  k_step_200_ascend_photo_steps[8].timeout_ms = robot_param->auto_ctrl_param.rear_retract_move_ms;

  if (!ctrl->template_ctx.step_entered && ctrl->template_ctx.template_start_time_ms == 0u) {
    ctrl->template_ctx.template_start_time_ms = now_ms;
  }

  switch (ctrl->template_id) {
    case AUTO_CTRL_TEMPLATE_FLAT_MOVE:
      return AutoCtrlTemplate_RunSequence(ctrl, now_ms, k_flat_move_steps,
                                          sizeof(k_flat_move_steps) / sizeof(k_flat_move_steps[0]));

    case AUTO_CTRL_TEMPLATE_ASCEND_200_STD:
      return AutoCtrlTemplate_RunSequence(
        ctrl, now_ms, k_step_200_ascend_photo_steps,
        sizeof(k_step_200_ascend_photo_steps) /
          sizeof(k_step_200_ascend_photo_steps[0]));

    case AUTO_CTRL_TEMPLATE_DESCEND_200_STD:
    case AUTO_CTRL_TEMPLATE_ASCEND_200_NO_FRONT_SICK:
    case AUTO_CTRL_TEMPLATE_DESCEND_200_NO_FRONT_SICK:
      return AutoCtrlTemplate_RunSequence(
        ctrl, now_ms, k_step_200_legacy_steps,
        sizeof(k_step_200_legacy_steps) / sizeof(k_step_200_legacy_steps[0]));

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