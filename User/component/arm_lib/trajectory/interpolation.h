#ifndef ARM_LIB_TRAJECTORY_INTERPOLATION_H
#define ARM_LIB_TRAJECTORY_INTERPOLATION_H

#include "../core/arm_common.h"

namespace arm_lib {
namespace trajectory {

struct CubicBlendSample {
  Scalar s;
  Scalar ds;
  Scalar dds;
  bool finished;

  CubicBlendSample() : s(0.0f), ds(0.0f), dds(0.0f), finished(false) {}
};

inline Scalar clamp_unit_interval(Scalar value) {
  return clamp_scalar(value, 0.0f, 1.0f);
}

inline CubicBlendSample sample_cubic_blend(Scalar elapsed,
                                           Scalar duration) {
  CubicBlendSample sample;
  if (duration <= ARM_LIB_EPSILON) {
    sample.s = (elapsed > 0.0f) ? 1.0f : 0.0f;
    sample.finished = (elapsed >= 0.0f);
    return sample;
  }

  const Scalar u = clamp_unit_interval(elapsed / duration);
  const Scalar uu = u * u;
  const Scalar inv_duration = 1.0f / duration;
  sample.s = (3.0f * uu) - (2.0f * uu * u);
  sample.ds = (6.0f * u * (1.0f - u)) * inv_duration;
  sample.dds = (6.0f - 12.0f * u) * inv_duration * inv_duration;
  sample.finished = (elapsed >= duration);

  if (sample.finished) {
    sample.s = 1.0f;
    sample.ds = 0.0f;
    sample.dds = 0.0f;
  } else if (elapsed <= 0.0f) {
    sample.s = 0.0f;
    sample.ds = 0.0f;
    sample.dds = 0.0f;
  }

  return sample;
}

inline Scalar cubic_duration_from_limits(Scalar distance,
                                         Scalar max_velocity,
                                         Scalar max_acceleration) {
  const Scalar abs_distance = abs_scalar(distance);
  if (abs_distance <= ARM_LIB_EPSILON) {
    return 0.0f;
  }
  if (max_velocity <= ARM_LIB_EPSILON || max_acceleration <= ARM_LIB_EPSILON) {
    return -1.0f;
  }

  const Scalar velocity_bound = 1.5f * abs_distance / max_velocity;
  const Scalar acceleration_bound = sqrtf(6.0f * abs_distance / max_acceleration);
  return (velocity_bound > acceleration_bound) ? velocity_bound
                                               : acceleration_bound;
}

template <int N>
inline JointVec<N> lerp_joint(const JointVec<N>& start,
                              const JointVec<N>& goal,
                              Scalar s) {
  return start + (goal - start) * s;
}

template <int N>
inline JointVec<N> scale_joint_vector(const JointVec<N>& q, Scalar scale) {
  return q * scale;
}

}  // namespace trajectory
}  // namespace arm_lib

#endif
