#ifndef ARM_LIB_TRAJECTORY_INTERPOLATION_H
#define ARM_LIB_TRAJECTORY_INTERPOLATION_H

#include "component/trajectory/cubic_blend.hpp"

#include "../core/arm_common.h"

namespace mr::robotics::arm {
namespace trajectory {

using CubicBlendSample = ::mr::comp::traj::CubicBlendSample;

inline Scalar clamp_unit_interval(Scalar value) {
  return clamp_scalar(value, 0.0f, 1.0f);
}

inline CubicBlendSample sample_cubic_blend(Scalar elapsed,
                                           Scalar duration) {
  return ::mr::comp::traj::sample_cubic_blend(
      elapsed, duration, ARM_LIB_EPSILON);
}

inline Scalar cubic_duration_from_limits(Scalar distance,
                                         Scalar max_velocity,
                                         Scalar max_acceleration) {
  return ::mr::comp::traj::cubic_blend_duration_from_limits(
      distance, max_velocity, max_acceleration, ARM_LIB_EPSILON);
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
}  // namespace mr::robotics::arm

#endif
