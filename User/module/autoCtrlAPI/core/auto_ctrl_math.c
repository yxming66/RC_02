#include "module/autoCtrlAPI/core/auto_ctrl_math.h"
#include "component/math/scalar.h"

/*
 * auto_ctrl_math.c
 *
 * 作用：
 * - 提供 AutoCtrl 状态机所需的 yaw 弧度基础计算；
 * - 所有函数纯计算、无副作用、无硬件依赖；
 * - 可被 api/template/primitive 多处复用。
 */

#include <math.h>

/* 将弧度循环折返到 [-pi, pi]。 */
float AutoCtrlMath_WrapYawRad(float yaw_rad) {
  return comp_wrap_to_pi_f(yaw_rad);
}

/* 计算目标朝向相对当前朝向的最短有符号误差（弧度）。 */
float AutoCtrlMath_GetYawErrorRad(float target_yaw_rad, float current_yaw_rad) {
  return AutoCtrlMath_WrapYawRad(target_yaw_rad - current_yaw_rad);
}

/* 基于最短误差判断 yaw 是否达标。 */
bool AutoCtrlMath_IsYawAlignedRad(float target_yaw_rad, float current_yaw_rad,
                                  float tolerance_rad) {
  return comp_abs_f(AutoCtrlMath_GetYawErrorRad(target_yaw_rad, current_yaw_rad)) <=
         comp_abs_f(tolerance_rad);
}
