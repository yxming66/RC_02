#ifndef ARM_LIB_TRAJECTORY_JOINT_TRAJ_H
#define ARM_LIB_TRAJECTORY_JOINT_TRAJ_H

#include "../adapter/toolbox_adapter.h"
#include "../core/arm_limits.h"
#include "interpolation.h"

namespace mr::robotics::arm {
namespace trajectory {

template <int N>
struct JointTrajectoryRequest {
  JointVec<N> start;
  JointVec<N> goal;
  JointVec<N> max_velocity;
  JointVec<N> max_acceleration;
  JointLimits<N> limits;
  bool enforce_limits;

  JointTrajectoryRequest()
      : start(toolbox_adapter::zero_joint_vec<N>()),
        goal(toolbox_adapter::zero_joint_vec<N>()),
        max_velocity(toolbox_adapter::zero_joint_vec<N>()),
        max_acceleration(toolbox_adapter::zero_joint_vec<N>()),
        limits(),
        enforce_limits(false) {
    for (uint16_t i = 0; i < N; ++i) {
      max_velocity[i][0] = 1.0f;
      max_acceleration[i][0] = 1.0f;
    }
  }
};

template <int N>
struct JointTrajectorySample {
  Scalar time_from_start;
  JointVec<N> position;
  JointVec<N> velocity;
  JointVec<N> acceleration;
  bool valid;
  bool finished;

  JointTrajectorySample()
      : time_from_start(0.0f),
        position(toolbox_adapter::zero_joint_vec<N>()),
        velocity(toolbox_adapter::zero_joint_vec<N>()),
        acceleration(toolbox_adapter::zero_joint_vec<N>()),
        valid(false),
        finished(false) {}
};

template <int N>
class JointTrajectory {
 public:
  JointTrajectory()
      : request_(),
        delta_(toolbox_adapter::zero_joint_vec<N>()),
        duration_(0.0f),
        valid_(false) {}

  bool configure(const JointTrajectoryRequest<N>& request) {
    request_ = request;
    delta_ = request.goal - request.start;
    duration_ = 0.0f;
    valid_ = true;

    if (request.enforce_limits) {
      if (!is_within_joint_limits(request.limits, request.start) ||
          !is_within_joint_limits(request.limits, request.goal)) {
        valid_ = false;
        return false;
      }
    }

    for (uint16_t i = 0; i < N; ++i) {
      const Scalar joint_duration =
          cubic_duration_from_limits(delta_[i][0],
                                     request.max_velocity[i][0],
                                     request.max_acceleration[i][0]);
      if (joint_duration < 0.0f) {
        valid_ = false;
        duration_ = 0.0f;
        return false;
      }
      if (joint_duration > duration_) {
        duration_ = joint_duration;
      }
    }

    return true;
  }

  JointTrajectorySample<N> sample(Scalar elapsed) const {
    JointTrajectorySample<N> out;
    out.time_from_start = elapsed;
    if (!valid_) {
      return out;
    }

    out.valid = true;
    if (duration_ <= ARM_LIB_EPSILON) {
      out.position = request_.goal;
      out.finished = true;
      return out;
    }

    const CubicBlendSample blend = sample_cubic_blend(elapsed, duration_);
    out.position = lerp_joint(request_.start, request_.goal, blend.s);
    out.velocity = scale_joint_vector(delta_, blend.ds);
    out.acceleration = scale_joint_vector(delta_, blend.dds);
    out.finished = blend.finished;

    if (out.finished) {
      out.position = request_.goal;
      out.velocity = toolbox_adapter::zero_joint_vec<N>();
      out.acceleration = toolbox_adapter::zero_joint_vec<N>();
    }

    return out;
  }

  Scalar duration() const { return duration_; }
  bool valid() const { return valid_; }
  const JointTrajectoryRequest<N>& request() const { return request_; }

 private:
  JointTrajectoryRequest<N> request_;
  JointVec<N> delta_;
  Scalar duration_;
  bool valid_;
};

}  // namespace trajectory
}  // namespace mr::robotics::arm

#endif
