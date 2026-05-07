#pragma once

#include "../adapter/toolbox_adapter.h"
#include "../trajectory/interpolation.h"

namespace mr::robotics::arm {
namespace planning {

struct CartesianTrajectoryPoint {
  Transform pose;
  Twist6 twist;
  Twist6 acceleration;
  bool finished;
};

class CartesianTrajectoryPlanner {
 public:
  CartesianTrajectoryPlanner()
      : valid_(false),
        duration_(0.0f),
        linear_distance_(0.0f),
        angular_distance_(0.0f) {}

  static CartesianTrajectoryPlanner from_to(const Transform& start,
                                             const Transform& goal,
                                             Scalar max_linear_velocity,
                                             Scalar max_angular_velocity,
                                             Scalar max_linear_acceleration,
                                             Scalar max_angular_acceleration) {
    CartesianTrajectoryPlanner planner;
    planner.configure(start, goal, max_linear_velocity, max_angular_velocity,
                      max_linear_acceleration, max_angular_acceleration);
    return planner;
  }

  bool configure(const Transform& start,
                 const Transform& goal,
                 Scalar max_linear_velocity,
                 Scalar max_angular_velocity,
                 Scalar max_linear_acceleration,
                 Scalar max_angular_acceleration) {
    valid_ = false;

    const Transform relative =
        toolbox_adapter::inverse_transform(start) * goal;
    relative_twist_ = toolbox_adapter::twist_from_transform(relative);
    linear_distance_ = relative_twist_.block<3, 1>(0, 0).norm();
    angular_distance_ = relative_twist_.block<3, 1>(3, 0).norm();

    const Scalar linear_duration = mr::comp::traj::cubic_duration_from_limits(
        linear_distance_, max_linear_velocity, max_linear_acceleration);
    const Scalar angular_duration = mr::comp::traj::cubic_duration_from_limits(
        angular_distance_, max_angular_velocity, max_angular_acceleration);

    if (linear_duration < 0.0f || angular_duration < 0.0f) {
      return false;
    }

    duration_ = (linear_duration > angular_duration) ? linear_duration
                                                     : angular_duration;
    start_ = start;
    goal_ = goal;
    valid_ = true;
    return true;
  }

  CartesianTrajectoryPoint sample(Scalar elapsed) const {
    CartesianTrajectoryPoint out;
    out.finished = false;

    if (!valid_) {
      return out;
    }

    out.finished = elapsed >= duration_;

    if (duration_ <= ARM_LIB_EPSILON) {
      out.pose = goal_;
      out.twist = toolbox_adapter::zero_twist();
      out.acceleration = toolbox_adapter::zero_twist();
      out.finished = true;
      return out;
    }

    const mr::comp::traj::CubicBlendSample blend =
        mr::comp::traj::sample_cubic_blend(elapsed, duration_);

    out.pose = start_ * toolbox_adapter::transform_from_twist(relative_twist_ * blend.s);
    out.twist = relative_twist_ * blend.ds;
    out.acceleration = relative_twist_ * blend.dds;

    if (out.finished) {
      out.pose = goal_;
      out.twist = toolbox_adapter::zero_twist();
      out.acceleration = toolbox_adapter::zero_twist();
    }

    return out;
  }

  Transform sample_pose(Scalar elapsed) const {
    CartesianTrajectoryPoint p = sample(elapsed);
    return p.pose;
  }

  Twist6 sample_twist(Scalar elapsed) const {
    CartesianTrajectoryPoint p = sample(elapsed);
    return p.twist;
  }

  Scalar duration() const { return duration_; }
  Scalar linear_distance() const { return linear_distance_; }
  Scalar angular_distance() const { return angular_distance_; }
  bool valid() const { return valid_; }
  const Transform& goal() const { return goal_; }
  const Transform& start() const { return start_; }

 private:
  Transform start_;
  Transform goal_;
  Twist6 relative_twist_;
  Scalar linear_distance_;
  Scalar angular_distance_;
  Scalar duration_;
  bool valid_;
};

inline CartesianTrajectoryPoint cartesian_traj_point(
    const Transform& start,
    const Transform& goal,
    Scalar max_linear_velocity,
    Scalar max_angular_velocity,
    Scalar max_linear_acceleration,
    Scalar max_angular_acceleration,
    Scalar elapsed) {
  CartesianTrajectoryPlanner planner;
  if (planner.configure(start, goal, max_linear_velocity, max_angular_velocity,
                        max_linear_acceleration, max_angular_acceleration)) {
    return planner.sample(elapsed);
  }
  CartesianTrajectoryPoint invalid;
  return invalid;
}

inline Transform cartesian_traj_pose(
    const Transform& start,
    const Transform& goal,
    Scalar max_linear_velocity,
    Scalar max_angular_velocity,
    Scalar max_linear_acceleration,
    Scalar max_angular_acceleration,
    Scalar elapsed) {
  CartesianTrajectoryPlanner planner;
  if (planner.configure(start, goal, max_linear_velocity, max_angular_velocity,
                        max_linear_acceleration, max_angular_acceleration)) {
    return planner.sample_pose(elapsed);
  }
  return start;
}

inline Twist6 cartesian_traj_twist(
    const Transform& start,
    const Transform& goal,
    Scalar max_linear_velocity,
    Scalar max_angular_velocity,
    Scalar max_linear_acceleration,
    Scalar max_angular_acceleration,
    Scalar elapsed) {
  CartesianTrajectoryPlanner planner;
  if (planner.configure(start, goal, max_linear_velocity, max_angular_velocity,
                        max_linear_acceleration, max_angular_acceleration)) {
    return planner.sample_twist(elapsed);
  }
  return toolbox_adapter::zero_twist();
}

inline CartesianTrajectoryPlanner make_cartesian_trajectory(
    const Transform& start,
    const Transform& goal,
    Scalar max_linear_velocity,
    Scalar max_angular_velocity,
    Scalar max_linear_acceleration,
    Scalar max_angular_acceleration) {
  return CartesianTrajectoryPlanner::from_to(start, goal, max_linear_velocity,
                                             max_angular_velocity,
                                             max_linear_acceleration,
                                             max_angular_acceleration);
}

}  // namespace planning
}  // namespace mr::robotics::arm
