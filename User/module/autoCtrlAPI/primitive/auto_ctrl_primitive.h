#pragma once

/**
 * @file auto_ctrl_primitive.h
 * @brief AutoCtrl 底层动作原语接口。
 *
 * 原语层负责把“高层状态机意图”转换为底盘/撑杆命令：
 * - 输出复位；
 * - yaw 修正；
 * - 匀速平移；
 * - 撑杆目标位控制。
 */

#include "module/autoCtrlAPI/api/auto_ctrl_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 清空并复位底盘与撑杆输出。 */
void AutoCtrlPrimitive_ResetOutputs(auto_ctrl_t *ctrl);

/* 清空底盘输出并置为 RELAX。 */
void AutoCtrlPrimitive_ResetChassis(auto_ctrl_t *ctrl);

/* 清空撑杆输出并置为 RELAX。 */
void AutoCtrlPrimitive_ResetPole(auto_ctrl_t *ctrl);

/* 浮点限幅工具。 */
float AutoCtrlPrimitive_Clamp(float value, float min_value, float max_value);

/* 仅执行 yaw 对齐控制（vx/vy 置 0）。 */
void AutoCtrlPrimitive_ApplyPrealign(auto_ctrl_t *ctrl);

/* Add forward velocity (+x) while yaw aligning. */
void AutoCtrlPrimitive_ApplyPrealignWithForward(auto_ctrl_t *ctrl,
                                                float vx_mps);

/* 在 yaw 对齐同时叠加 vx/vy 平移速度。 */
void AutoCtrlPrimitive_ApplyPrealignWithMove(auto_ctrl_t *ctrl, float vx_mps,
                                             float vy_mps);

/* Send a pure forward/backward command on vx; vy/wz are cleared. */
void AutoCtrlPrimitive_CommandFlatMove(auto_ctrl_t *ctrl, float vx_mps);

/* Send a flat move and apply the active external yaw-rate command if enabled. */
void AutoCtrlPrimitive_CommandFlatMoveWithYawRate(auto_ctrl_t *ctrl,
                                                  float vx_mps);

/* 发送前/后撑杆目标位。 */
void AutoCtrlPrimitive_CommandPoleTarget(auto_ctrl_t *ctrl, float front_target,
                                         float rear_target);

/* 发送前/后撑杆目标位，并指定前后杆各自目标跟踪速度。 */
void AutoCtrlPrimitive_CommandPoleTargetWithSpeed(auto_ctrl_t *ctrl,
                                                  float front_target,
                                                  float rear_target,
                                                  float front_speed,
                                                  float rear_speed);

/* 单独控制前撑杆[0][1]或后撑杆[2][3]，不改变另一组状态。group: 0=前, 1=后 */
void AutoCtrlPrimitive_CommandPoleGroupWithSpeed(auto_ctrl_t *ctrl,
                                                  uint8_t group,
                                                  float target,
                                                  float speed);

#ifdef __cplusplus
}
#endif
