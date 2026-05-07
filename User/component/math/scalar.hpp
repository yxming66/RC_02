/**
 ******************************************************************************
 * @file    scalar.hpp
 * @brief   Scalar math utilities for embedded control systems.
 *          标量数学工具函数库
 * @author  XuMing Yuan
 ******************************************************************************
 */

#pragma once

#include "component/math/scalar.h"

namespace mr::component::math {

using Scalar = float;

constexpr Scalar kDefaultEpsilon = 1.0e-6f;
constexpr Scalar kFiniteLimit = 1.0e30f;
constexpr Scalar kPi = 3.14159265358979323846f;
constexpr Scalar kHalfPi = 0.5f * kPi;
constexpr Scalar kTwoPi = 2.0f * kPi;

inline Scalar abs_scalar(Scalar value) {
  return comp_abs_f(value);
}

inline bool is_finite_scalar(Scalar value) {
  return comp_is_finite_f(value);
}

inline bool is_positive_scalar(Scalar value,
                               Scalar epsilon = kDefaultEpsilon) {
  return comp_is_positive_f(value, epsilon);
}

inline Scalar clamp_scalar(Scalar value, Scalar lower, Scalar upper) {
  return comp_clamp_f(value, lower, upper);
}

inline Scalar abs_clip_scalar(Scalar value, Scalar limit) {
  return comp_abs_clip_f(value, limit);
}

inline Scalar positive_or_zero(Scalar value) {
  return comp_positive_or_zero_f(value);
}

inline Scalar positive_or(Scalar value, Scalar fallback) {
  return comp_positive_or_f(value, fallback);
}

inline Scalar square_scalar(Scalar value) {
  return value * value;
}

inline Scalar sanitize_positive(Scalar value,
                                Scalar fallback,
                                Scalar lower = kDefaultEpsilon,
                                Scalar upper = kFiniteLimit) {
  if (!is_finite_scalar(lower) || lower <= 0.0f) {
    lower = kDefaultEpsilon;
  }
  if (lower > kFiniteLimit) {
    lower = kFiniteLimit;
  }
  if (!is_finite_scalar(upper) || upper < lower) {
    upper = kFiniteLimit;
  }
  if (upper < lower) {
    upper = lower;
  }
  if (!is_finite_scalar(value) || value < lower) {
    value = fallback;
  }
  if (!is_finite_scalar(value) || value < lower) {
    value = lower;
  }
  return clamp_scalar(value, lower, upper);
}

inline Scalar sanitize_dt(Scalar dt,
                          Scalar fallback,
                          Scalar min_dt,
                          Scalar max_dt) {
  return comp_sanitize_dt_f(dt, fallback, min_dt, max_dt);
}

inline Scalar wrap_to_range(Scalar value,
                            Scalar lower,
                            Scalar upper,
                            Scalar epsilon = kDefaultEpsilon) {
  return comp_wrap_to_range_f(value, lower, upper, epsilon);
}

inline Scalar wrap_to_pi(Scalar angle) {
  return comp_wrap_to_pi_f(angle);
}

inline Scalar wrap_error(Scalar error, Scalar range) {
  return comp_wrap_error_f(error, range);
}

inline Scalar angle_error(Scalar target, Scalar current) {
  return comp_angle_error_f(target, current);
}

inline Scalar move_towards(Scalar current, Scalar target, Scalar max_step) {
  return comp_move_towards_f(current, target, max_step);
}

inline Scalar apply_delta_limit(Scalar value,
                                Scalar previous,
                                Scalar max_delta) {
  return comp_apply_delta_limit_f(value, previous, max_delta);
}

inline Scalar max_abs_array(const Scalar* values, uint32_t count) {
  return comp_max_abs_f(values, count);
}

inline bool values_are_finite(const Scalar* values, uint32_t count) {
  return comp_values_finite_f(values, count);
}

inline bool scale_to_abs_limit(Scalar* values,
                               uint32_t count,
                               Scalar limit) {
  return comp_scale_to_abs_limit_f(values, count, limit);
}

}  // namespace mr::component::math
