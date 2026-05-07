#include "robotics/arm/planning/joint_trajectory_builder.hpp"
#include "robotics/arm/planning/cartesian_trajectory_builder.hpp"
#include "robotics/arm/adapter/toolbox_adapter.h"

namespace {

using mr::robotics::arm::planning::JointTrajectoryPlanner;
using mr::robotics::arm::planning::CartesianTrajectoryPlanner;
using mr::robotics::arm::planning::joint_traj_point;
using mr::robotics::arm::planning::joint_traj_position;
using mr::robotics::arm::planning::cartesian_traj_point;
using mr::robotics::arm::planning::cartesian_traj_pose;
using mr::robotics::arm::planning::make_joint_trajectory;
using mr::robotics::arm::planning::make_cartesian_trajectory;
using mr::robotics::arm::toolbox_adapter::zero_joint_vec;
using mr::robotics::arm::toolbox_adapter::identity_transform;
using mr::robotics::arm::Scalar;

}  // namespace

namespace mr::robotics::arm {
namespace planning {

void Example_JointTrajectory_OldAPI() {
  constexpr int N = 3;

  JointVec<N> start = zero_joint_vec<N>();
  JointVec<N> goal = zero_joint_vec<N>();
  goal[0][0] = 1.0f;
  goal[1][0] = 2.0f;
  goal[2][0] = 3.0f;

  JointVec<N> max_vel = zero_joint_vec<N>();
  max_vel[0][0] = 2.0f;
  max_vel[1][0] = 2.0f;
  max_vel[2][0] = 2.0f;

  JointVec<N> max_acc = zero_joint_vec<N>();
  max_acc[0][0] = 5.0f;
  max_acc[1][0] = 5.0f;
  max_acc[2][0] = 5.0f;

  JointTrajectoryPlanner<N> planner;
  (void)planner.configure(start, goal, max_vel, max_acc);

  Scalar time = 0.5f;
  auto point = planner.sample(time);
  (void)point;
}

void Example_JointTrajectory_NewAPI_OneLine() {
  constexpr int N = 3;

  JointVec<N> start = zero_joint_vec<N>();
  JointVec<N> goal = zero_joint_vec<N>();
  goal[0][0] = 1.0f;
  goal[1][0] = 2.0f;
  goal[2][0] = 3.0f;

  JointVec<N> max_vel = zero_joint_vec<N>();
  max_vel[0][0] = 2.0f;
  max_vel[1][0] = 2.0f;
  max_vel[2][0] = 2.0f;

  JointVec<N> max_acc = zero_joint_vec<N>();
  max_acc[0][0] = 5.0f;
  max_acc[1][0] = 5.0f;
  max_acc[2][0] = 5.0f;

  Scalar time = 0.5f;

  auto point = joint_traj_point<N>(start, goal, max_vel, max_acc, time);
  (void)point;

  auto pos = joint_traj_position<N>(start, goal, max_vel, max_acc, time);
  (void)pos;
}

void Example_JointTrajectory_BuilderAPI() {
  constexpr int N = 3;

  auto planner = make_joint_trajectory<N>(
      zero_joint_vec<N>(),
      []()->JointVec<N>{JointVec<N> g; g[0][0]=1; g[1][0]=2; g[2][0]=3; return g;}(),
      []()->JointVec<N>{JointVec<N> v; v[0][0]=2; v[1][0]=2; v[2][0]=2; return v;}(),
      []()->JointVec<N>{JointVec<N> a; a[0][0]=5; a[1][0]=5; a[2][0]=5; return a;}()
  );

  auto sample = planner.sample(0.5f);
  (void)sample;
}

void Example_CartesianTrajectory_OldAPI() {
  Transform start = identity_transform();
  Transform goal = identity_transform();

  CartesianTrajectoryPlanner planner;
  (void)planner.configure(start, goal, 0.2f, 1.0f, 0.5f, 2.0f);

  Scalar time = 0.5f;
  auto point = planner.sample(time);
  (void)point;
}

void Example_CartesianTrajectory_NewAPI_OneLine() {
  Transform start = identity_transform();
  Transform goal = identity_transform();
  Scalar time = 0.5f;

  auto point = cartesian_traj_point(start, goal, 0.2f, 1.0f, 0.5f, 2.0f, time);
  (void)point;

  auto pose = cartesian_traj_pose(start, goal, 0.2f, 1.0f, 0.5f, 2.0f, time);
  (void)pose;
}

void Example_CartesianTrajectory_BuilderAPI() {
  auto planner = make_cartesian_trajectory(
      identity_transform(),
      identity_transform(),
      0.2f, 1.0f, 0.5f, 2.0f
  );

  auto sample = planner.sample(0.5f);
  (void)sample;
}

void RunAllExamples() {
  Example_JointTrajectory_OldAPI();
  Example_JointTrajectory_NewAPI_OneLine();
  Example_JointTrajectory_BuilderAPI();
  Example_CartesianTrajectory_OldAPI();
  Example_CartesianTrajectory_NewAPI_OneLine();
  Example_CartesianTrajectory_BuilderAPI();
}

}  // namespace planning
}  // namespace mr::robotics::arm
