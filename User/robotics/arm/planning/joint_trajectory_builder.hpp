#pragma once

/*
 * 关节轨迹规划器。
 *
 * 控制律（基于 SynchronizedTrapezoidProfile）：
 *   distance[i] = goal[i] - start[i]
 *   profile.configure(distance[], max_velocity[], max_acceleration[])
 *   elapsed = current_time - start_time
 *   for each axis i:
 *     sample = profile.sample_axis(i, elapsed)
 *     position[i] = start[i] + sample.position
 *     velocity[i] = sample.velocity
 *     acceleration[i] = sample.acceleration
 *
 * 特性：
 *   - 基于 SynchronizedTrapezoidProfile 实现多轴同步
 *   - 同时输出位置、速度、加速度
 *   - 类型适配层：JointVec<N> ↔ Scalar[]
 *
 * 使用示例：
 *   mr::robotics::arm::planning::JointTrajectoryPlanner<3> planner;
 *   planner.configure(start, goal, max_velocity, max_acceleration);
 *
 *   // 控制循环中（每周期执行）：
 *   auto point = planner.sample(elapsed);
 *   // point.position, point.velocity, point.acceleration
 */

#include "component/trajectory/trajectory_types.hpp"
#include "component/trajectory/synchronized_profile.hpp"
#include "../adapter/toolbox_adapter.h"

namespace mr::robotics::arm {
namespace planning {

template <int N>
struct JointTrajectoryPoint {
  JointVec<N> position;
  JointVec<N> velocity;
  JointVec<N> acceleration;
  bool finished;
};

template <int N>
class JointTrajectoryPlanner {
 public:
  JointTrajectoryPlanner()
      : valid_(false), duration_(0.0f) {
    for (int i = 0; i < N; ++i) {
      start_[i] = 0.0f;
      distances_[i] = 0.0f;
      max_velocities_[i] = 1.0f;
      max_accelerations_[i] = 1.0f;
    }
  }

  static JointTrajectoryPlanner<N> from_to(const JointVec<N>& start,
                                           const JointVec<N>& goal,
                                           const JointVec<N>& max_velocity,
                                           const JointVec<N>& max_acceleration) {
    JointTrajectoryPlanner<N> planner;
    planner.configure(start, goal, max_velocity, max_acceleration);
    return planner;
  }

  bool configure(const JointVec<N>& start,
                 const JointVec<N>& goal,
                 const JointVec<N>& max_velocity,
                 const JointVec<N>& max_acceleration) {
    for (int i = 0; i < N; ++i) {
      start_[i] = start[i][0];
      distances_[i] = goal[i][0] - start[i][0];
      max_velocities_[i] = max_velocity[i][0];
      max_accelerations_[i] = max_acceleration[i][0];
    }

    valid_ = profile_.configure(distances_, max_velocities_, max_accelerations_,
                                mr::comp::traj::Scalar(1e-6f));
    if (valid_) {
      duration_ = profile_.duration();
    }
    return valid_;
  }

  JointTrajectoryPoint<N> sample(Scalar elapsed) const {
    JointTrajectoryPoint<N> out;
    out.finished = false;

    if (!valid_) {
      return out;
    }

    if (duration_ <= 1e-6f) {
      for (int i = 0; i < N; ++i) {
        out.position[i][0] = start_[i] + distances_[i];
        out.velocity[i][0] = 0.0f;
        out.acceleration[i][0] = 0.0f;
      }
      out.finished = true;
      return out;
    }

    const Scalar t = mr::comp::traj::clamp_scalar(elapsed, 0.0f, duration_);
    out.finished = elapsed >= duration_;

    for (int i = 0; i < N; ++i) {
      const auto s = profile_.sample_axis(i, t);
      out.position[i][0] = start_[i] + s.position;
      out.velocity[i][0] = s.velocity;
      out.acceleration[i][0] = s.acceleration;
    }

    if (out.finished) {
      for (int i = 0; i < N; ++i) {
        out.velocity[i][0] = 0.0f;
        out.acceleration[i][0] = 0.0f;
      }
    }

    return out;
  }

  JointVec<N> sample_position(Scalar elapsed) const {
    JointTrajectoryPoint<N> p = sample(elapsed);
    return p.position;
  }

  JointVec<N> sample_velocity(Scalar elapsed) const {
    JointTrajectoryPoint<N> p = sample(elapsed);
    return p.velocity;
  }

  Scalar duration() const { return duration_; }
  bool valid() const { return valid_; }

 private:
  mr::comp::traj::SynchronizedTrapezoidProfile<N> profile_;
  Scalar start_[N];
  Scalar distances_[N];
  Scalar max_velocities_[N];
  Scalar max_accelerations_[N];
  Scalar duration_;
  bool valid_;
};

template <int N>
inline JointTrajectoryPoint<N> joint_traj_point(const JointVec<N>& start,
                                                  const JointVec<N>& goal,
                                                  const JointVec<N>& max_velocity,
                                                  const JointVec<N>& max_acceleration,
                                                  Scalar elapsed) {
  JointTrajectoryPlanner<N> planner;
  if (planner.configure(start, goal, max_velocity, max_acceleration)) {
    return planner.sample(elapsed);
  }
  JointTrajectoryPoint<N> invalid;
  return invalid;
}

template <int N>
inline JointVec<N> joint_traj_position(const JointVec<N>& start,
                                         const JointVec<N>& goal,
                                         const JointVec<N>& max_velocity,
                                         const JointVec<N>& max_acceleration,
                                         Scalar elapsed) {
  JointTrajectoryPlanner<N> planner;
  if (planner.configure(start, goal, max_velocity, max_acceleration)) {
    return planner.sample_position(elapsed);
  }
  return start;
}

template <int N>
inline JointVec<N> joint_traj_velocity(const JointVec<N>& start,
                                        const JointVec<N>& goal,
                                        const JointVec<N>& max_velocity,
                                        const JointVec<N>& max_acceleration,
                                        Scalar elapsed) {
  JointTrajectoryPlanner<N> planner;
  if (planner.configure(start, goal, max_velocity, max_acceleration)) {
    return planner.sample_velocity(elapsed);
  }
  return toolbox_adapter::zero_joint_vec<N>();
}

template <int N>
inline JointTrajectoryPlanner<N> make_joint_trajectory(
    const JointVec<N>& start,
    const JointVec<N>& goal,
    const JointVec<N>& max_velocity,
    const JointVec<N>& max_acceleration) {
  return JointTrajectoryPlanner<N>::from_to(start, goal, max_velocity, max_acceleration);
}

}  // namespace planning
}  // namespace mr::robotics::arm
