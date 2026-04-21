#include "module/autoCtrlAPI/core/auto_ctrl_math.h"

/*
 * auto_ctrl_math.c
 *
 * 作用：
 * - 提供 AutoCtrl 状态机所需的 yaw 弧度基础计算；
 * - 所有函数纯计算、无副作用、无硬件依赖；
 * - 可被 api/template/primitive 多处复用。
 */

#include <math.h>

#define AUTO_CTRL_PI_F (3.14159265358979323846f)
#define AUTO_CTRL_TWO_PI_F (2.0f * AUTO_CTRL_PI_F)

/* 将弧度循环折返到 [-pi, pi]。 */
float AutoCtrlMath_WrapYawRad(float yaw_rad) {
  while (yaw_rad > AUTO_CTRL_PI_F) {
    yaw_rad -= AUTO_CTRL_TWO_PI_F;
  }
  while (yaw_rad < -AUTO_CTRL_PI_F) {
    yaw_rad += AUTO_CTRL_TWO_PI_F;
  }
  return yaw_rad;
}

/* 计算目标朝向相对当前朝向的最短有符号误差（弧度）。 */
float AutoCtrlMath_GetYawErrorRad(float target_yaw_rad, float current_yaw_rad) {
  return AutoCtrlMath_WrapYawRad(target_yaw_rad - current_yaw_rad);
}

/* 基于最短误差判断 yaw 是否达标。 */
bool AutoCtrlMath_IsYawAlignedRad(float target_yaw_rad, float current_yaw_rad,
                                  float tolerance_rad) {
  return fabsf(AutoCtrlMath_GetYawErrorRad(target_yaw_rad, current_yaw_rad)) <=
         fabsf(tolerance_rad);
}