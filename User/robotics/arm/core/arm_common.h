#ifndef ARM_LIB_CORE_ARM_COMMON_H
#define ARM_LIB_CORE_ARM_COMMON_H

#include "component/math/scalar.hpp"
#include "../arm_config.h"
#include "arm_types.h"

namespace mr::robotics::arm {

inline Scalar abs_scalar(Scalar value) {
  return ::mr::component::math::abs_scalar(value);
}

inline bool is_finite_scalar(Scalar value) {
  return ::mr::component::math::is_finite_scalar(value);
}

inline bool is_near_zero(Scalar value, Scalar epsilon = ARM_LIB_EPSILON) {
  return abs_scalar(value) <= epsilon;
}

inline Scalar clamp_scalar(Scalar value, Scalar lower, Scalar upper) {
  return ::mr::component::math::clamp_scalar(value, lower, upper);
}

inline Scalar deg_to_rad(Scalar degrees) {
  return degrees * (ARM_LIB_PI / 180.0f);
}

inline Scalar rad_to_deg(Scalar angle_rad) {
  return angle_rad * (180.0f / ARM_LIB_PI);
}

inline Scalar wrap_to_pi(Scalar angle) {
  return ::mr::component::math::wrap_to_range(
      angle, -ARM_LIB_PI, ARM_LIB_PI, ARM_LIB_EPSILON);
}

inline Scalar angle_distance(Scalar from, Scalar to) {
  return wrap_to_pi(to - from);
}

}  // namespace mr::robotics::arm

#endif
