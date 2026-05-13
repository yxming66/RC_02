#include "module/autoCtrlAPI/core/auto_ctrl_math.h"
#include "component/math/scalar.hpp"

/*
 * auto_ctrl_math.cpp
 *
 * 作用：
 * - 提供 AutoCtrl 状态机所需的 yaw 弧度基础计算；
 * - 所有函数纯计算、无副作用、无硬件依赖；
 * - 可被 api/template/primitive 多处复用。
 */

/* 将弧度循环折返到 [-pi, pi)。 */
float AutoCtrlMath_WrapYawRad(float yaw_rad) {
  return mr::component::math::wrap_to_pi(yaw_rad);
}

/* 计算目标朝向相对当前朝向的最短有符号误差（弧度）。 */
float AutoCtrlMath_GetYawErrorRad(float target_yaw_rad, float current_yaw_rad) {
  return AutoCtrlMath_WrapYawRad(target_yaw_rad - current_yaw_rad);
}

/* 基于最短误差判断 yaw 是否达标。 */
bool AutoCtrlMath_IsYawAlignedRad(float target_yaw_rad, float current_yaw_rad,
                                  float tolerance_rad) {
  return mr::component::math::abs_scalar(
             AutoCtrlMath_GetYawErrorRad(target_yaw_rad, current_yaw_rad)) <=
         mr::component::math::abs_scalar(tolerance_rad);
}

float AutoCtrlMath_NearestCardinalYawRad(float current_yaw_rad) {
  const float candidates[4] = {
      0.0f,
      mr::component::math::kHalfPi,
      -mr::component::math::kPi,
      -mr::component::math::kHalfPi,
  };

  float best_yaw = candidates[0];
  float best_error =
      mr::component::math::abs_scalar(AutoCtrlMath_GetYawErrorRad(
          candidates[0], current_yaw_rad));
  for (uint8_t i = 1U; i < 4U; ++i) {
    const float error =
        mr::component::math::abs_scalar(AutoCtrlMath_GetYawErrorRad(
            candidates[i], current_yaw_rad));
    if (error < best_error) {
      best_error = error;
      best_yaw = candidates[i];
    }
  }
  return best_yaw;
}
