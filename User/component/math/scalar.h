#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define COMP_MATH_DEFAULT_EPSILON_F (1.0e-6f)
#define COMP_MATH_FINITE_LIMIT_F (1.0e30f)
#define COMP_MATH_PI_F (3.14159265358979323846f)
#define COMP_MATH_TWO_PI_F (6.28318530717958647692f)

static inline float comp_abs_f(float value) {
  return fabsf(value);
}

static inline bool comp_is_finite_f(float value) {
  return (value == value) && (comp_abs_f(value) <= COMP_MATH_FINITE_LIMIT_F);
}

static inline bool comp_is_positive_f(float value, float epsilon) {
  return comp_is_finite_f(value) && value > epsilon;
}

static inline float comp_clamp_f(float value, float lower, float upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

static inline float comp_abs_clip_f(float value, float limit) {
  const float positive_limit = comp_abs_f(limit);
  return comp_clamp_f(value, -positive_limit, positive_limit);
}

static inline float comp_positive_or_zero_f(float value) {
  return (comp_is_finite_f(value) && value > 0.0f) ? value : 0.0f;
}

static inline float comp_positive_or_f(float value, float fallback) {
  return (comp_is_finite_f(value) && value > 0.0f) ? value : fallback;
}

static inline float comp_sanitize_dt_f(float dt,
                                       float fallback,
                                       float min_dt,
                                       float max_dt) {
  if (!comp_is_finite_f(min_dt) || min_dt <= 0.0f) {
    min_dt = COMP_MATH_DEFAULT_EPSILON_F;
  }
  if (!comp_is_finite_f(max_dt) || max_dt < min_dt) {
    max_dt = COMP_MATH_FINITE_LIMIT_F;
  }
  if (!comp_is_finite_f(fallback) || fallback < min_dt) {
    fallback = min_dt;
  }
  if (!comp_is_finite_f(dt) || dt <= 0.0f) {
    dt = fallback;
  }
  return comp_clamp_f(dt, min_dt, max_dt);
}

static inline float comp_wrap_to_range_f(float value,
                                         float lower,
                                         float upper,
                                         float epsilon) {
  const float width = upper - lower;
  if (!comp_is_finite_f(value) || !comp_is_finite_f(lower) ||
      !comp_is_finite_f(upper) || width <= epsilon) {
    return value;
  }

  float wrapped = fmodf(value - lower, width);
  if (wrapped < 0.0f) {
    wrapped += width;
  }
  return lower + wrapped;
}

static inline float comp_wrap_to_pi_f(float angle) {
  float wrapped = comp_wrap_to_range_f(
      angle, -COMP_MATH_PI_F, COMP_MATH_PI_F, COMP_MATH_DEFAULT_EPSILON_F);
  if (wrapped >= COMP_MATH_PI_F) {
    wrapped -= COMP_MATH_TWO_PI_F;
  }
  return wrapped;
}

static inline float comp_wrap_error_f(float error, float range) {
  if (!comp_is_finite_f(error) || !comp_is_finite_f(range) ||
      range <= COMP_MATH_DEFAULT_EPSILON_F) {
    return error;
  }

  const float half_range = 0.5f * range;
  while (error > half_range) {
    error -= range;
  }
  while (error < -half_range) {
    error += range;
  }
  return error;
}

static inline float comp_angle_error_f(float target, float current) {
  return comp_wrap_to_pi_f(target - current);
}

static inline float comp_move_towards_f(float current,
                                        float target,
                                        float max_step) {
  if (max_step <= 0.0f) {
    return target;
  }

  const float delta = target - current;
  if (delta > max_step) {
    return current + max_step;
  }
  if (delta < -max_step) {
    return current - max_step;
  }
  return target;
}

static inline float comp_apply_delta_limit_f(float value,
                                             float previous,
                                             float max_delta) {
  return previous + comp_abs_clip_f(value - previous, max_delta);
}

static inline float comp_max_abs_f(const float* values, size_t count) {
  if (values == NULL) {
    return 0.0f;
  }

  float max_abs = 0.0f;
  for (size_t i = 0U; i < count; ++i) {
    const float abs_value = comp_abs_f(values[i]);
    if (abs_value > max_abs) {
      max_abs = abs_value;
    }
  }
  return max_abs;
}

static inline bool comp_values_finite_f(const float* values, size_t count) {
  if (values == NULL) {
    return false;
  }

  for (size_t i = 0U; i < count; ++i) {
    if (!comp_is_finite_f(values[i])) {
      return false;
    }
  }
  return true;
}

static inline bool comp_scale_to_abs_limit_f(float* values,
                                             size_t count,
                                             float limit) {
  if (values == NULL || !comp_is_finite_f(limit) || limit <= 0.0f) {
    return false;
  }

  const float max_abs = comp_max_abs_f(values, count);
  if (max_abs <= limit || max_abs <= 0.0f) {
    return false;
  }

  const float scale = limit / max_abs;
  for (size_t i = 0U; i < count; ++i) {
    values[i] *= scale;
  }
  return true;
}

#ifdef __cplusplus
}
#endif
