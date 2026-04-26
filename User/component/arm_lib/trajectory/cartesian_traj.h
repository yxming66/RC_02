#ifndef ARM_LIB_TRAJECTORY_CARTESIAN_TRAJ_H
#define ARM_LIB_TRAJECTORY_CARTESIAN_TRAJ_H

#include "../adapter/toolbox_adapter.h"
#include "../kinematics/pose_error.h"
#include "interpolation.h"

namespace arm_lib {
namespace trajectory {

struct CartesianTrajectoryRequest {
  Transform start;
  Transform goal;
  Scalar max_linear_velocity;
  Scalar max_angular_velocity;
  Scalar max_linear_acceleration;
  Scalar max_angular_acceleration;

  CartesianTrajectoryRequest()
      : start(toolbox_adapter::identity_transform()),
        goal(toolbox_adapter::identity_transform()),
        max_linear_velocity(0.2f),
        max_angular_velocity(1.0f),
        max_linear_acceleration(0.5f),
        max_angular_acceleration(2.0f) {}
};

struct CartesianTrajectorySample {
  Scalar time_from_start;
  Transform pose;
  Twist6 body_twist;
  Twist6 body_acceleration;
  bool valid;
  bool finished;

  CartesianTrajectorySample()
      : time_from_start(0.0f),
        pose(toolbox_adapter::identity_transform()),
        body_twist(toolbox_adapter::zero_twist()),
        body_acceleration(toolbox_adapter::zero_twist()),
        valid(false),
        finished(false) {}
};

class CartesianTrajectory {
 public:
  CartesianTrajectory()
      : request_(),
        relative_twist_(toolbox_adapter::zero_twist()),
        linear_distance_(0.0f),
        angular_distance_(0.0f),
        duration_(0.0f),
        valid_(false) {}

  bool configure(const CartesianTrajectoryRequest& request) {
    request_ = request;
    valid_ = true;

    const Transform relative =
        toolbox_adapter::inverse_transform(request.start) * request.goal;
    relative_twist_ = toolbox_adapter::twist_from_transform(relative);
    linear_distance_ = relative_twist_.block<3, 1>(0, 0).norm();
    angular_distance_ = relative_twist_.block<3, 1>(3, 0).norm();

    const Scalar linear_duration =
        cubic_duration_from_limits(linear_distance_,
                                   request.max_linear_velocity,
                                   request.max_linear_acceleration);
    const Scalar angular_duration =
        cubic_duration_from_limits(angular_distance_,
                                   request.max_angular_velocity,
                                   request.max_angular_acceleration);

    if (linear_duration < 0.0f || angular_duration < 0.0f) {
      valid_ = false;
      duration_ = 0.0f;
      return false;
    }

    duration_ = linear_duration;
    if (angular_duration > duration_) {
      duration_ = angular_duration;
    }
    return true;
  }

  CartesianTrajectorySample sample(Scalar elapsed) const {
    CartesianTrajectorySample out;
    out.time_from_start = elapsed;
    if (!valid_) {
      return out;
    }

    out.valid = true;
    if (duration_ <= ARM_LIB_EPSILON) {
      out.pose = request_.goal;
      out.finished = true;
      return out;
    }

    const CubicBlendSample blend = sample_cubic_blend(elapsed, duration_);
    out.pose =
        request_.start *
        toolbox_adapter::transform_from_twist(relative_twist_ * blend.s);
    out.body_twist = relative_twist_ * blend.ds;
    out.body_acceleration = relative_twist_ * blend.dds;
    out.finished = blend.finished;

    if (out.finished) {
      out.pose = request_.goal;
      out.body_twist = toolbox_adapter::zero_twist();
      out.body_acceleration = toolbox_adapter::zero_twist();
    }

    return out;
  }

  Scalar duration() const { return duration_; }
  Scalar linear_distance() const { return linear_distance_; }
  Scalar angular_distance() const { return angular_distance_; }
  bool valid() const { return valid_; }
  const CartesianTrajectoryRequest& request() const { return request_; }

 private:
  CartesianTrajectoryRequest request_;
  Twist6 relative_twist_;
  Scalar linear_distance_;
  Scalar angular_distance_;
  Scalar duration_;
  bool valid_;
};

}  // namespace trajectory
}  // namespace arm_lib

#endif
