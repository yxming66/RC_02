#pragma once

#include <cmath>
#include <stdint.h>

namespace mr::component::trajectory {

using Scalar = float;

constexpr Scalar kDefaultEpsilon = 1.0e-6f;
constexpr Scalar kDefaultFiniteLimit = 1.0e30f;

inline Scalar abs_scalar(Scalar value) {
  return fabsf(value);
}

inline bool is_finite_scalar(Scalar value) {
  return (value == value) && (abs_scalar(value) <= kDefaultFiniteLimit);
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

struct ScalarMotionSample {
  Scalar position;
  Scalar velocity;
  Scalar acceleration;
  bool valid;
  bool finished;

  ScalarMotionSample()
      : position(0.0f),
        velocity(0.0f),
        acceleration(0.0f),
        valid(false),
        finished(false) {}
};

}  // namespace mr::component::trajectory
