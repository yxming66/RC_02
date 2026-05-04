#ifndef ARM_LIB_CORE_ARM_LIMITS_H
#define ARM_LIB_CORE_ARM_LIMITS_H

#include "component/math/range_limits.hpp"

#include "arm_common.h"

namespace mr::robotics::arm {

using JointLimitApplicationMode = ::mr::component::math::RangeLimitMode;

template <int N>
struct JointLimits {
  JointVec<N> lower;
  JointVec<N> upper;
  bool enabled[N];

  JointLimits()
      : lower(matrixf::zeros<N, 1>()), upper(matrixf::zeros<N, 1>()) {
    for (uint16_t i = 0; i < N; ++i) {
      enabled[i] = false;
    }
  }

  void enable_all() {
    for (uint16_t i = 0; i < N; ++i) {
      enabled[i] = true;
    }
  }

  void disable_all() {
    for (uint16_t i = 0; i < N; ++i) {
      enabled[i] = false;
    }
  }

  void set(Index index, Scalar lower_bound, Scalar upper_bound) {
    if (index >= N) {
      return;
    }
    lower[index][0] = lower_bound;
    upper[index][0] = upper_bound;
    enabled[index] = true;
  }
};

template <int N>
struct JointRangeProjection {
  JointVec<N> lower;
  JointVec<N> upper;
  bool enabled[N];

  JointRangeProjection()
      : lower(matrixf::zeros<N, 1>()), upper(matrixf::zeros<N, 1>()) {
    for (uint16_t i = 0; i < N; ++i) {
      enabled[i] = false;
    }
  }

  void enable_all() {
    for (uint16_t i = 0; i < N; ++i) {
      enabled[i] = true;
    }
  }

  void disable_all() {
    for (uint16_t i = 0; i < N; ++i) {
      enabled[i] = false;
    }
  }

  void set(Index index, Scalar lower_bound, Scalar upper_bound) {
    if (index >= N) {
      return;
    }
    lower[index][0] = lower_bound;
    upper[index][0] = upper_bound;
    enabled[index] = true;
  }
};

template <int N>
inline bool has_any_limit(const JointLimits<N>& limits) {
  for (uint16_t i = 0; i < N; ++i) {
    if (limits.enabled[i]) {
      return true;
    }
  }
  return false;
}

template <int N>
inline bool is_within_joint_limits(const JointLimits<N>& limits,
                                   const JointVec<N>& q,
                                   Scalar margin = ARM_LIB_EPSILON) {
  for (uint16_t i = 0; i < N; ++i) {
    if (!limits.enabled[i]) {
      continue;
    }
    if (q[i][0] < limits.lower[i][0] - margin) {
      return false;
    }
    if (q[i][0] > limits.upper[i][0] + margin) {
      return false;
    }
  }
  return true;
}

template <int N>
inline JointVec<N> clamp_to_joint_limits(const JointLimits<N>& limits,
                                         const JointVec<N>& q) {
  JointVec<N> clamped = q;
  for (uint16_t i = 0; i < N; ++i) {
    if (!limits.enabled[i]) {
      continue;
    }
    clamped[i][0] = ::mr::component::math::clamp_to_range(
        q[i][0], limits.lower[i][0], limits.upper[i][0]);
  }
  return clamped;
}

template <int N>
inline JointVec<N> project_to_joint_ranges(
    const JointRangeProjection<N>& projection,
    const JointVec<N>& q) {
  JointVec<N> projected = q;
  for (uint16_t i = 0; i < N; ++i) {
    if (!projection.enabled[i]) {
      continue;
    }
    projected[i][0] = ::mr::component::math::wrap_to_range(
        q[i][0], projection.lower[i][0], projection.upper[i][0],
        ARM_LIB_EPSILON);
  }
  return projected;
}

template <int N>
inline JointVec<N> apply_joint_limits(
    const JointLimits<N>& limits,
    const JointVec<N>& q,
    JointLimitApplicationMode mode = JointLimitApplicationMode::kClamp,
    const JointRangeProjection<N>* projection = nullptr) {
  JointVec<N> value = q;
  if (mode == JointLimitApplicationMode::kWrapThenClamp &&
      projection != nullptr) {
    value = project_to_joint_ranges(*projection, value);
  }
  return clamp_to_joint_limits(limits, value);
}

template <int N>
inline Scalar total_limit_violation(const JointLimits<N>& limits,
                                    const JointVec<N>& q) {
  Scalar violation = 0.0f;
  for (uint16_t i = 0; i < N; ++i) {
    if (!limits.enabled[i]) {
      continue;
    }
    if (q[i][0] < limits.lower[i][0]) {
      violation += limits.lower[i][0] - q[i][0];
    } else if (q[i][0] > limits.upper[i][0]) {
      violation += q[i][0] - limits.upper[i][0];
    }
  }
  return violation;
}

}  // namespace mr::robotics::arm

#endif
