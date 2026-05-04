#ifndef ARM_LIB_PLANNING_MOVE_J_PLANNER_H
#define ARM_LIB_PLANNING_MOVE_J_PLANNER_H

#include <cmath>

#include "component/trajectory/synchronized_profile.hpp"

#include "../adapter/toolbox_adapter.h"
#include "../core/arm_limits.h"

namespace mr::robotics::arm {
namespace planning {

template <int N>
struct MoveJRequest {
  JointVec<N> start;
  JointVec<N> goal;
  JointVec<N> max_velocity;
  JointVec<N> max_acceleration;
  JointLimits<N> limits;
  bool enforce_limits;

  MoveJRequest()
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
struct MoveJSample {
  Scalar time_from_start;
  JointVec<N> position;
  JointVec<N> velocity;
  JointVec<N> acceleration;
  bool valid;
  bool finished;

  MoveJSample()
      : time_from_start(0.0f),
        position(toolbox_adapter::zero_joint_vec<N>()),
        velocity(toolbox_adapter::zero_joint_vec<N>()),
        acceleration(toolbox_adapter::zero_joint_vec<N>()),
        valid(false),
        finished(false) {}
};

template <int N>
class MoveJPlanner {
 public:
  MoveJPlanner()
      : request_(),
        delta_(toolbox_adapter::zero_joint_vec<N>()),
        profile_(),
        duration_(0.0f),
        valid_(false) {}

  bool configure(const MoveJRequest<N>& request) {
    request_ = request;
    delta_ = request.goal - request.start;
    duration_ = 0.0f;
    valid_ = false;

    if (!joint_vectors_are_finite(request.start, request.goal,
                                  request.max_velocity,
                                  request.max_acceleration)) {
      return false;
    }

    if (request.enforce_limits) {
      if (!is_within_joint_limits(request.limits, request.start) ||
          !is_within_joint_limits(request.limits, request.goal)) {
        return false;
      }
    }

    Scalar distances[N];
    Scalar max_velocities[N];
    Scalar max_accelerations[N];
    for (uint16_t i = 0; i < N; ++i) {
      distances[i] = delta_[i][0];
      max_velocities[i] = request.max_velocity[i][0];
      max_accelerations[i] = request.max_acceleration[i][0];
    }

    if (!profile_.configure(distances, max_velocities, max_accelerations,
                            ARM_LIB_EPSILON)) {
      return false;
    }

    duration_ = profile_.duration();
    valid_ = true;
    return true;
  }

  MoveJSample<N> sample(Scalar elapsed) const {
    MoveJSample<N> out;
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

    const Scalar t = clamp_scalar(elapsed, 0.0f, duration_);
    out.finished = elapsed >= duration_;

    for (uint16_t i = 0; i < N; ++i) {
      const ::mr::component::trajectory::ScalarMotionSample sample =
          profile_.sample_axis(i, t);

      if (out.finished) {
        out.position[i][0] = request_.goal[i][0];
        out.velocity[i][0] = 0.0f;
        out.acceleration[i][0] = 0.0f;
      } else {
        out.position[i][0] = request_.start[i][0] + sample.position;
        out.velocity[i][0] = sample.velocity;
        out.acceleration[i][0] = sample.acceleration;
      }
    }

    return out;
  }

  Scalar duration() const { return duration_; }
  bool valid() const { return valid_; }
  const MoveJRequest<N>& request() const { return request_; }

 private:
  static bool joint_vector_is_finite(const JointVec<N>& q) {
    for (uint16_t i = 0; i < N; ++i) {
      if (!is_finite_scalar(q[i][0])) {
        return false;
      }
    }
    return true;
  }

  static bool joint_vectors_are_finite(const JointVec<N>& a,
                                       const JointVec<N>& b,
                                       const JointVec<N>& c,
                                       const JointVec<N>& d) {
    return joint_vector_is_finite(a) && joint_vector_is_finite(b) &&
           joint_vector_is_finite(c) && joint_vector_is_finite(d);
  }

  MoveJRequest<N> request_;
  JointVec<N> delta_;
  ::mr::component::trajectory::SynchronizedTrapezoidProfile<N> profile_;
  Scalar duration_;
  bool valid_;
};

}  // namespace planning
}  // namespace mr::robotics::arm

#endif
