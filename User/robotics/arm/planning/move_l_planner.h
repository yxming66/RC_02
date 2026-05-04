#ifndef ARM_LIB_PLANNING_MOVE_L_PLANNER_H
#define ARM_LIB_PLANNING_MOVE_L_PLANNER_H

#include <cmath>

#include "../adapter/toolbox_adapter.h"
#include "../core/arm_common.h"
#include "../trajectory/interpolation.h"

namespace mr::robotics::arm {
namespace planning {

struct MoveLRequest {
  Transform start;
  Transform goal;
  Scalar max_linear_velocity;
  Scalar max_angular_velocity;
  Scalar max_linear_acceleration;
  Scalar max_angular_acceleration;

  MoveLRequest()
      : start(toolbox_adapter::identity_transform()),
        goal(toolbox_adapter::identity_transform()),
        max_linear_velocity(0.2f),
        max_angular_velocity(1.0f),
        max_linear_acceleration(0.5f),
        max_angular_acceleration(2.0f) {}
};

struct MoveLSample {
  Scalar time_from_start;
  Transform pose;
  Twist6 body_twist;
  bool valid;
  bool finished;

  MoveLSample()
      : time_from_start(0.0f),
        pose(toolbox_adapter::identity_transform()),
        body_twist(toolbox_adapter::zero_twist()),
        valid(false),
        finished(false) {}
};

class MoveLPlanner {
 public:
  MoveLPlanner()
      : request_(),
        relative_twist_(toolbox_adapter::zero_twist()),
        linear_distance_(0.0f),
        angular_distance_(0.0f),
        duration_(0.0f),
        valid_(false) {}

  bool configure(const MoveLRequest& request) {
    request_ = request;
    valid_ = false;
    duration_ = 0.0f;

    const Transform relative =
        toolbox_adapter::inverse_transform(request.start) * request.goal;
    relative_twist_ = toolbox_adapter::twist_from_transform(relative);
    linear_distance_ = relative_twist_.block<3, 1>(0, 0).norm();
    angular_distance_ = relative_twist_.block<3, 1>(3, 0).norm();

    const Scalar linear_duration = trajectory::cubic_duration_from_limits(
        linear_distance_, request.max_linear_velocity,
        request.max_linear_acceleration);
    const Scalar angular_duration = trajectory::cubic_duration_from_limits(
        angular_distance_, request.max_angular_velocity,
        request.max_angular_acceleration);

    if (linear_duration < 0.0f || angular_duration < 0.0f) {
      return false;
    }

    duration_ = (angular_duration > linear_duration) ? angular_duration
                                                     : linear_duration;
    valid_ = true;
    return true;
  }

  MoveLSample sample(Scalar elapsed) const {
    MoveLSample out;
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

    const trajectory::CubicBlendSample blend =
        trajectory::sample_cubic_blend(elapsed, duration_);
    out.pose =
        request_.start *
        toolbox_adapter::transform_from_twist(relative_twist_ * blend.s);
    out.body_twist = relative_twist_ * blend.ds;
    out.finished = blend.finished;

    if (out.finished) {
      out.pose = request_.goal;
      out.body_twist = toolbox_adapter::zero_twist();
    }
    return out;
  }

  Scalar duration() const { return duration_; }
  Scalar linear_distance() const { return linear_distance_; }
  Scalar angular_distance() const { return angular_distance_; }
  bool valid() const { return valid_; }
  const MoveLRequest& request() const { return request_; }

 private:
  MoveLRequest request_;
  Twist6 relative_twist_;
  Scalar linear_distance_;
  Scalar angular_distance_;
  Scalar duration_;
  bool valid_;
};

}  // namespace planning
}  // namespace mr::robotics::arm

#endif
