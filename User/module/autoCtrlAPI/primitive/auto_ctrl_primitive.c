#include "module/autoCtrlAPI/primitive/auto_ctrl_primitive.h"

/*
 * auto_ctrl_primitive.c
 *
 * 作用：
 * - 封装 AutoCtrl 可复用的底盘/撑杆动作原语；
 * - 作为 API/模板状态机与执行机构命令之间的中间层；
 * - 保持函数短小、单一职责，便于模板组合调用。
 */

#include <string.h>
#include "module/config.h"

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
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

/* 仅执行 yaw 对齐控制，不注入平移速度。 */
void AutoCtrlPrimitive_ApplyPrealign(auto_ctrl_t *ctrl) {
  const Config_RobotParam_t *robot_param = Config_GetRobotParam();

  ctrl->chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
  ctrl->chassis_cmd.ctrl_vec.vx = 0.0f;
  ctrl->chassis_cmd.ctrl_vec.vy = 0.0f;
  ctrl->chassis_cmd.ctrl_vec.wz = AutoCtrlPrimitive_Clamp(
      ctrl->yaw_error_deg * robot_param->auto_ctrl_param.prealign_kp,
      -robot_param->auto_ctrl_param.prealign_wz_limit,
      robot_param->auto_ctrl_param.prealign_wz_limit);
}

    /* 在 yaw 对齐基础上叠加 vx/vy。 */
void AutoCtrlPrimitive_ApplyPrealignWithMove(auto_ctrl_t *ctrl, float vx_mps,
                                             float vy_mps) {
  AutoCtrlPrimitive_ApplyPrealign(ctrl);
  ctrl->chassis_cmd.ctrl_vec.vx = vx_mps;
  ctrl->chassis_cmd.ctrl_vec.vy = vy_mps;
}

/* 在 yaw 对齐基础上仅叠加前进速度 vx。 */
void AutoCtrlPrimitive_ApplyPrealignWithForward(auto_ctrl_t *ctrl,
                                                float vx_mps) {
  AutoCtrlPrimitive_ApplyPrealignWithMove(ctrl, vx_mps, 0.0f);
}

/* 下发纯平移命令：匀速 vx，vy/wz 清零。 */
void AutoCtrlPrimitive_CommandFlatMove(auto_ctrl_t *ctrl, float vx_mps) {
  ctrl->chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
  ctrl->chassis_cmd.ctrl_vec.vx = vx_mps;
  ctrl->chassis_cmd.ctrl_vec.vy = 0.0f;
  ctrl->chassis_cmd.ctrl_vec.wz = 0.0f;
}

/* 下发撑杆目标位：同时使能前后杆自动目标。 */
void AutoCtrlPrimitive_CommandPoleTarget(auto_ctrl_t *ctrl, float front_target,
                                         float rear_target) {
  ctrl->pole_cmd.mode = POLE_MODE_ACTIVE;
  ctrl->pole_cmd.auto_target_enable[0] = true;
  ctrl->pole_cmd.auto_target_enable[1] = true;
  ctrl->pole_cmd.auto_target_lift[0] = front_target;
  ctrl->pole_cmd.auto_target_lift[1] = rear_target;
}