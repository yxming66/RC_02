#include "module/autoCtrlAPI/primitive/auto_ctrl_primitive.h"

/*
 * auto_ctrl_primitive.cpp
 *
 * 作用：
 * - 封装 AutoCtrl 可复用的底盘/撑杆动作原语；
 * - 作为 API/模板状态机与执行机构命令之间的中间层；
 * - 保持函数短小、单一职责，便于模板组合调用。
 */

#include <string.h>
#include "component/math/scalar.hpp"
#include "module/config.h"

static const AutoCtrl_TemplateParam_t *AutoCtrlPrimitive_GetTemplateParams(
    const Config_RobotParam_t *robot_param, auto_ctrl_template_e template_id) {
  if (robot_param == nullptr) {
    return nullptr;
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
      return nullptr;
  }
}

/* 底盘输出复位为 RELAX，避免上一周期残留控制量。 */
void AutoCtrlPrimitive_ResetChassis(auto_ctrl_t *ctrl) {
  memset(&ctrl->chassis_cmd, 0, sizeof(ctrl->chassis_cmd));
  ctrl->chassis_cmd.mode = CHASSIS_MODE_RELAX;
}

/* 撑杆输出复位为 RELAX，避免上一周期目标位残留。 */
void AutoCtrlPrimitive_ResetPole(auto_ctrl_t *ctrl) {
  memset(&ctrl->pole_cmd, 0, sizeof(ctrl->pole_cmd));
  ctrl->pole_cmd.mode = POLE_MODE_RELAX;
}

/* 同时复位底盘与撑杆输出。 */
void AutoCtrlPrimitive_ResetOutputs(auto_ctrl_t *ctrl) {
  AutoCtrlPrimitive_ResetChassis(ctrl);
  AutoCtrlPrimitive_ResetPole(ctrl);
}

/* 通用限幅函数，防止控制量超界。 */
float AutoCtrlPrimitive_Clamp(float value, float min_value, float max_value) {
  return mr::component::math::clamp_scalar(value, min_value, max_value);
}

/* 仅执行 yaw 对齐控制，不注入平移速度。 */
void AutoCtrlPrimitive_ApplyPrealign(auto_ctrl_t *ctrl) {
  const Config_RobotParam_t *robot_param = Config_GetRobotParam();

  ctrl->chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
  ctrl->chassis_cmd.ctrl_vec.vx = 0.0f;
  ctrl->chassis_cmd.ctrl_vec.vy = 0.0f;
  ctrl->chassis_cmd.ctrl_vec.wz = AutoCtrlPrimitive_Clamp(
      ctrl->yaw_error_rad * robot_param->auto_ctrl_param.common.prealign_kp,
      -robot_param->auto_ctrl_param.common.prealign_wz_limit,
      robot_param->auto_ctrl_param.common.prealign_wz_limit);
}

    /* 在 yaw 对齐基础上叠加 vx/vy。 */
void AutoCtrlPrimitive_ApplyPrealignWithMove(auto_ctrl_t *ctrl, float vx_mps,
                                             float vy_mps) {
  AutoCtrlPrimitive_ApplyPrealign(ctrl);
  ctrl->chassis_cmd.ctrl_vec.vx = vx_mps;
  ctrl->chassis_cmd.ctrl_vec.vy = vy_mps;
}

/* Add forward velocity (+x) while yaw aligning. */
void AutoCtrlPrimitive_ApplyPrealignWithForward(auto_ctrl_t *ctrl,
                                                float vx_mps) {
  AutoCtrlPrimitive_ApplyPrealignWithMove(ctrl, vx_mps, 0.0f);
}

/* Send a pure forward/backward command on vx; vy/wz are cleared. */
void AutoCtrlPrimitive_CommandFlatMove(auto_ctrl_t *ctrl, float vx_mps) {
  ctrl->chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
  ctrl->chassis_cmd.ctrl_vec.vx = vx_mps;
  ctrl->chassis_cmd.ctrl_vec.vy = 0.0f;
  ctrl->chassis_cmd.ctrl_vec.wz = 0.0f;
}

/* 下发撑杆目标位：同时使能前后杆自动目标。 */
void AutoCtrlPrimitive_CommandPoleTarget(auto_ctrl_t *ctrl, float front_target,
                                         float rear_target) {
  if (ctrl == nullptr) {
    return;
  }

  const Config_RobotParam_t *robot_param = Config_GetRobotParam();
  const AutoCtrl_TemplateParam_t *template_param =
      AutoCtrlPrimitive_GetTemplateParams(robot_param, ctrl->template_id);
  const float default_speed =
      (robot_param != nullptr) ? robot_param->pole_param.limit.support_lift_speed
                               : 0.0f;
  const float front_extend_speed =
      (template_param != nullptr) ? template_param->pole_front_extend_speed
                                  : default_speed;
  const float front_retract_speed =
      (template_param != nullptr) ? template_param->pole_front_retract_speed
                                  : default_speed;
  const float rear_extend_speed =
      (template_param != nullptr) ? template_param->pole_rear_extend_speed
                                  : default_speed;
  const float rear_retract_speed =
      (template_param != nullptr) ? template_param->pole_rear_retract_speed
                                  : default_speed;
  const float front_speed =
    (front_target >= 0.0f) ? front_extend_speed : front_retract_speed;
  const float rear_speed =
    (rear_target >= 0.0f) ? rear_extend_speed : rear_retract_speed;

  AutoCtrlPrimitive_CommandPoleTargetWithSpeed(
    ctrl, front_target, rear_target, front_speed, rear_speed);
}

/* 下发撑杆目标位，并指定前后杆目标跟踪速度。 */
void AutoCtrlPrimitive_CommandPoleTargetWithSpeed(auto_ctrl_t *ctrl,
                                                  float front_target,
                                                  float rear_target,
                                                  float front_speed,
                                                  float rear_speed) {
  ctrl->pole_cmd.mode = POLE_MODE_ACTIVE;
  ctrl->pole_cmd.auto_target_enable[0] = true;
  ctrl->pole_cmd.auto_target_enable[1] = true;
  ctrl->pole_cmd.auto_target_lift[0] = front_target;
  ctrl->pole_cmd.auto_target_lift[1] = rear_target;
  ctrl->pole_cmd.auto_lift_speed[0] = front_speed;
  ctrl->pole_cmd.auto_lift_speed[1] = rear_speed;
}

/* 单独控制前后撑杆中的一组。group: 0=前撑杆[0][1], 1=后撑杆[2][3] */
void AutoCtrlPrimitive_CommandPoleGroupWithSpeed(auto_ctrl_t *ctrl,
                                                  uint8_t group,
                                                  float target,
                                                  float speed) {
  ctrl->pole_cmd.mode = POLE_MODE_ACTIVE;
  if (group == 0) {
    ctrl->pole_cmd.auto_target_enable[0] = true;
    ctrl->pole_cmd.auto_target_lift[0] = target;
    ctrl->pole_cmd.auto_lift_speed[0] = speed;
  } else {
    ctrl->pole_cmd.auto_target_enable[1] = true;
    ctrl->pole_cmd.auto_target_lift[1] = target;
    ctrl->pole_cmd.auto_lift_speed[1] = speed;
  }
}
