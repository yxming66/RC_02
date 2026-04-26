#ifndef ARM_LIB_CORE_ARM_COMMON_H
#define ARM_LIB_CORE_ARM_COMMON_H

#include <cmath>

#include "../arm_lib_config.h"
#include "arm_types.h"

namespace arm_lib {

inline Scalar abs_scalar(Scalar value) {
  return fabsf(value);
}

inline bool is_finite_scalar(Scalar value) {
  return (value == value) && (abs_scalar(value) <= 1.0e30f);
}

inline bool is_near_zero(Scalar value, Scalar epsilon = ARM_LIB_EPSILON) {
  return abs_scalar(value) <= epsilon;
}

inline Scalar clamp_scalar(Scalar value, Scalar lower, Scalar upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

inline Scalar deg_to_rad(Scalar degrees) {
  return degrees * (ARM_LIB_PI / 180.0f);
}

inline Scalar rad_to_deg(Scalar angle_rad) {
  return angle_rad * (180.0f / ARM_LIB_PI);
}

inline Scalar wrap_to_pi(Scalar angle) {
  Scalar wrapped = fmodf(angle + ARM_LIB_PI, 2.0f * ARM_LIB_PI);
  if (wrapped < 0.0f) {
    wrapped += 2.0f * ARM_LIB_PI;
  }
  return wrapped - ARM_LIB_PI;
}

inline Scalar angle_distance(Scalar from, Scalar to) {
  return wrap_to_pi(to - from);
}

}  // namespace arm_lib

#endif
