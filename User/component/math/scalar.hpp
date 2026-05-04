#pragma once

#include <cmath>

namespace mr::component::math {

using Scalar = float;

constexpr Scalar kDefaultEpsilon = 1.0e-6f;
constexpr Scalar kFiniteLimit = 1.0e30f;
constexpr Scalar kPi = 3.14159265358979323846f;
constexpr Scalar kHalfPi = 0.5f * kPi;
constexpr Scalar kTwoPi = 2.0f * kPi;

inline Scalar abs_scalar(Scalar value) {
  return fabsf(value);
}

inline bool is_finite_scalar(Scalar value) {
  return (value == value) && (abs_scalar(value) <= kFiniteLimit);
}

inline bool is_positive_scalar(Scalar value,
                               Scalar epsilon = kDefaultEpsilon) {
  return is_finite_scalar(value) && value > epsilon;
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

inline Scalar wrap_to_range(Scalar value,
                            Scalar lower,
                            Scalar upper,
                            Scalar epsilon = kDefaultEpsilon) {
  const Scalar width = upper - lower;
  if (!is_finite_scalar(value) || !is_finite_scalar(lower) ||
      !is_finite_scalar(upper) || width <= epsilon) {
    return value;
  }

  Scalar wrapped = fmodf(value - lower, width);
  if (wrapped < 0.0f) {
    wrapped += width;
  }
  return lower + wrapped;
}

}  // namespace mr::component::math
