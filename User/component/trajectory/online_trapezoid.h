#pragma once

#include <math.h>

#include "component/math/scalar.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float position;
  float velocity;
  bool valid;
  bool reached;
} comp_online_trapezoid_axis_sample_t;

static inline comp_online_trapezoid_axis_sample_t
comp_update_trapezoid_axis_f(float current,
                             float target,
                             float* velocity,
                             float max_velocity,
                             float max_acceleration,
                             float dt,
                             float position_epsilon,
                             float velocity_epsilon_floor) {
  comp_online_trapezoid_axis_sample_t out = {current, 0.0f, false, false};
  if (velocity == NULL || !comp_is_finite_f(current) ||
      !comp_is_finite_f(target)) {
    return out;
  }

  float max_vel = comp_abs_f(max_velocity);
  float max_acc = comp_abs_f(max_acceleration);
  if (!comp_is_finite_f(*velocity)) {
    *velocity = 0.0f;
  }
  dt = comp_sanitize_dt_f(dt, 0.0f, COMP_MATH_DEFAULT_EPSILON_F,
                          COMP_MATH_FINITE_LIMIT_F);
  position_epsilon = comp_abs_f(position_epsilon);
  velocity_epsilon_floor = comp_abs_f(velocity_epsilon_floor);

  if (!comp_is_finite_f(max_vel) || !comp_is_finite_f(max_acc) ||
      max_vel <= COMP_MATH_DEFAULT_EPSILON_F ||
      max_acc <= COMP_MATH_DEFAULT_EPSILON_F) {
    out.velocity = *velocity;
    return out;
  }

  const float error = target - current;
  const float stop_vel_epsilon =
      fmaxf(velocity_epsilon_floor, max_acc * dt);

  if (comp_abs_f(error) <= position_epsilon &&
      comp_abs_f(*velocity) <= stop_vel_epsilon) {
    *velocity = 0.0f;
    out.position = target;
    out.velocity = 0.0f;
    out.valid = true;
    out.reached = true;
    return out;
  }

  const float brake_velocity =
      sqrtf(fmaxf(0.0f, 2.0f * max_acc * comp_abs_f(error)));
  float desired_velocity = copysignf(fminf(max_vel, brake_velocity), error);
  if (brake_velocity < stop_vel_epsilon) {
    desired_velocity = 0.0f;
  }

  const float max_delta_velocity = max_acc * dt;
  *velocity = comp_apply_delta_limit_f(desired_velocity, *velocity,
                                       max_delta_velocity);
  *velocity = comp_abs_clip_f(*velocity, max_vel);

  if (comp_abs_f(error) <= position_epsilon &&
      comp_abs_f(*velocity) <= stop_vel_epsilon) {
    *velocity = 0.0f;
    out.position = target;
    out.velocity = 0.0f;
    out.valid = true;
    out.reached = true;
    return out;
  }

  out.position = current + (*velocity) * dt;
  out.velocity = *velocity;
  out.valid = comp_is_finite_f(out.position) && comp_is_finite_f(out.velocity);
  out.reached = false;
  return out;
}

#ifdef __cplusplus
}
#endif
