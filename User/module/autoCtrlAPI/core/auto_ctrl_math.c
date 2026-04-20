#include "module/autoCtrlAPI/core/auto_ctrl_math.h"

/*
 * auto_ctrl_math.c
 *
 * 作用：
 * - 提供 AutoCtrl 状态机所需的 yaw 角度基础计算；
 * - 所有函数纯计算、无副作用、无硬件依赖；
 * - 可被 api/template/primitive 多处复用。
 */

#include <math.h>

/* 将角度循环折返到 [-180, 180]。 */
float AutoCtrlMath_WrapYawDeg(float yaw_deg) {
  while (yaw_deg > 180.0f) {
    yaw_deg -= 360.0f;
  }
  while (yaw_deg < -180.0f) {
    yaw_deg += 360.0f;
  }
  return yaw_deg;
}

/* 计算目标朝向相对当前朝向的最短有符号误差。 */
float AutoCtrlMath_GetYawErrorDeg(float target_yaw_deg, float current_yaw_deg) {
  return AutoCtrlMath_WrapYawDeg(target_yaw_deg - current_yaw_deg);
}

/* 基于最短误差判断 yaw 是否达标。 */
bool AutoCtrlMath_IsYawAligned(float target_yaw_deg, float current_yaw_deg,
                               float tolerance_deg) {
  return fabsf(AutoCtrlMath_GetYawErrorDeg(target_yaw_deg, current_yaw_deg)) <=
         fabsf(tolerance_deg);
}