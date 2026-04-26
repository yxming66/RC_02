#ifndef ARM_LIB_DEMO_ARM_LIB_DEMO_3R_H
#define ARM_LIB_DEMO_ARM_LIB_DEMO_3R_H

#include "../arm_lib.h"

namespace arm_lib {
namespace demo {

struct Demo3RReport {
  SerialChain<3> chain;
  JointVec<3> q_nominal;
  JointVec<3> q_seed;
  JointVec<3> q_solution;
  trajectory::JointTrajectory<3> joint_trajectory;
  trajectory::JointTrajectorySample<3> joint_traj_mid;
  trajectory::JointTrajectorySample<3> joint_traj_end;
  trajectory::CartesianTrajectory cartesian_trajectory;
  trajectory::CartesianTrajectorySample cart_traj_mid;
  trajectory::CartesianTrajectorySample cart_traj_end;
  control::JointServo<3> joint_servo;
  control::JointServoResult<3> joint_servo_result;
  control::CartesianServo<3> cartesian_servo;
  control::CartesianServoResult<3> cartesian_servo_result;
  control::CartesianServoResult<3> cartesian_servo_local_result;
  control::CartesianServoResult<3> cartesian_servo_spatial_result;
  control::CartesianServoResult<3> cartesian_servo_tool_z_result;
  control::CartesianServoResult<3> cartesian_servo_yaw_pitch_result;
  control::CartesianServoResult<3> cartesian_servo_yz_pitch_result;
  control::CartesianServoResult<3> cartesian_servo_singular_result;
  kinematics::AnalyticIKPlugin<3> analytic_plugin;
  kinematics::IKResult<3> analytic_ik_result;
  kinematics::IKResult<3> hybrid_ik_result;
  JointVec<3> analytic_q_solution;
  JointVec<3> hybrid_q_solution;
  JointVec<3> gravity_torques;
  Transform target_pose;
  Transform seed_pose;
  Jacobian6xN<3> jacobian;
  kinematics::IKResult<3> ik_result;
  kinematics::IKResult<3> robust_ik_result;
  kinematics::IKResult<3> constrained_ik_result;
  kinematics::IKResult<3> position_relaxed_ik_result;
  kinematics::IKResult<3> yz_pitch_ik_result;
  JointVec<3> robust_q_solution;
  JointVec<3> constrained_q_solution;
  JointVec<3> position_relaxed_q_solution;
  JointVec<3> yz_pitch_q_solution;
  JointVec<3> actuator_coupled_joint_roundtrip;
  JointVec<3> actuator_coupled_motor_position;
  JointVec<3> actuator_coupled_motor_velocity;
  Scalar position_error_norm;
  Scalar orientation_error_norm;
  Scalar robust_position_error_norm;
  Scalar robust_orientation_error_norm;
  Scalar constrained_position_error_norm;
  Scalar constrained_orientation_error_norm;
  Scalar position_relaxed_position_error_norm;
  Scalar position_relaxed_orientation_error_norm;
  Scalar position_relaxed_raw_position_error_norm;
  Scalar actuator_coupling_roundtrip_error_norm;
  Scalar actuator_coupling_velocity_roundtrip_error_norm;
  Scalar robust_singularity_metric;
  Scalar analytic_position_error_norm;
  Scalar analytic_orientation_error_norm;
  Scalar hybrid_position_error_norm;
  Scalar hybrid_orientation_error_norm;
  Scalar gravity_torque_norm;
  Scalar joint_traj_duration;
  Scalar joint_traj_end_error_norm;
  Scalar cart_traj_duration;
  Scalar cart_traj_position_error_norm;
  Scalar cart_traj_orientation_error_norm;
  Scalar joint_servo_error_after_norm;
  Scalar cart_servo_position_improvement;
  Scalar cart_servo_orientation_improvement;
  Scalar cart_servo_local_position_improvement;
  Scalar cart_servo_local_orientation_improvement;
  Scalar cart_servo_spatial_position_improvement;
  Scalar cart_servo_spatial_orientation_improvement;
  Scalar cart_servo_roll_only_full_orientation_error;
  Scalar cart_servo_tool_z_orientation_error;
  Scalar cart_servo_yaw_pitch_orientation_error;
  Scalar yz_pitch_position_error_norm;
  Scalar yz_pitch_orientation_error_norm;
  Scalar yz_pitch_raw_position_error_norm;
  Scalar cart_servo_yz_pitch_position_error;
  Scalar cart_servo_yz_pitch_orientation_error;
  Scalar cart_servo_singular_position_improvement;
  Scalar cart_servo_singular_orientation_weight;
  Scalar cart_servo_singular_velocity_scale;
  bool joint_traj_valid;
  bool cart_traj_valid;
  bool joint_servo_valid;
  bool cart_servo_valid;
  bool cart_servo_local_valid;
  bool cart_servo_spatial_valid;
  bool cart_servo_roll_only_full_reached;
  bool cart_servo_tool_z_valid;
  bool cart_servo_yaw_pitch_valid;
  bool cart_servo_yz_pitch_valid;
  bool analytic_ik_valid;
  bool gravity_valid;
  bool robust_ik_valid;
  bool constrained_ik_valid;
  bool position_relaxed_ik_valid;
  bool yz_pitch_ik_valid;
  bool actuator_coupling_valid;
  bool hybrid_ik_valid;

  Demo3RReport()
      : chain(),
        q_nominal(toolbox_adapter::zero_joint_vec<3>()),
        q_seed(toolbox_adapter::zero_joint_vec<3>()),
        q_solution(toolbox_adapter::zero_joint_vec<3>()),
        joint_trajectory(),
        joint_traj_mid(),
        joint_traj_end(),
        cartesian_trajectory(),
        cart_traj_mid(),
        cart_traj_end(),
        joint_servo(),
        joint_servo_result(),
        cartesian_servo(),
        cartesian_servo_result(),
        cartesian_servo_local_result(),
        cartesian_servo_spatial_result(),
        cartesian_servo_tool_z_result(),
        cartesian_servo_yaw_pitch_result(),
        cartesian_servo_yz_pitch_result(),
        cartesian_servo_singular_result(),
        analytic_plugin(),
        analytic_ik_result(),
        hybrid_ik_result(),
        analytic_q_solution(toolbox_adapter::zero_joint_vec<3>()),
        hybrid_q_solution(toolbox_adapter::zero_joint_vec<3>()),
        gravity_torques(toolbox_adapter::zero_joint_vec<3>()),
        target_pose(toolbox_adapter::identity_transform()),
        seed_pose(toolbox_adapter::identity_transform()),
        jacobian(matrixf::zeros<6, 3>()),
        ik_result(),
        robust_ik_result(),
        constrained_ik_result(),
        position_relaxed_ik_result(),
        yz_pitch_ik_result(),
        robust_q_solution(toolbox_adapter::zero_joint_vec<3>()),
        constrained_q_solution(toolbox_adapter::zero_joint_vec<3>()),
        position_relaxed_q_solution(toolbox_adapter::zero_joint_vec<3>()),
        yz_pitch_q_solution(toolbox_adapter::zero_joint_vec<3>()),
        actuator_coupled_joint_roundtrip(toolbox_adapter::zero_joint_vec<3>()),
        actuator_coupled_motor_position(toolbox_adapter::zero_joint_vec<3>()),
        actuator_coupled_motor_velocity(toolbox_adapter::zero_joint_vec<3>()),
        position_error_norm(0.0f),
        orientation_error_norm(0.0f),
        robust_position_error_norm(0.0f),
        robust_orientation_error_norm(0.0f),
        constrained_position_error_norm(0.0f),
        constrained_orientation_error_norm(0.0f),
        position_relaxed_position_error_norm(0.0f),
        position_relaxed_orientation_error_norm(0.0f),
        position_relaxed_raw_position_error_norm(0.0f),
        actuator_coupling_roundtrip_error_norm(0.0f),
        actuator_coupling_velocity_roundtrip_error_norm(0.0f),
        robust_singularity_metric(0.0f),
        analytic_position_error_norm(0.0f),
        analytic_orientation_error_norm(0.0f),
        hybrid_position_error_norm(0.0f),
        hybrid_orientation_error_norm(0.0f),
        gravity_torque_norm(0.0f),
        joint_traj_duration(0.0f),
        joint_traj_end_error_norm(0.0f),
        cart_traj_duration(0.0f),
        cart_traj_position_error_norm(0.0f),
        cart_traj_orientation_error_norm(0.0f),
        joint_servo_error_after_norm(0.0f),
        cart_servo_position_improvement(0.0f),
        cart_servo_orientation_improvement(0.0f),
        cart_servo_local_position_improvement(0.0f),
        cart_servo_local_orientation_improvement(0.0f),
        cart_servo_spatial_position_improvement(0.0f),
        cart_servo_spatial_orientation_improvement(0.0f),
        cart_servo_roll_only_full_orientation_error(0.0f),
        cart_servo_tool_z_orientation_error(0.0f),
        cart_servo_yaw_pitch_orientation_error(0.0f),
        yz_pitch_position_error_norm(0.0f),
        yz_pitch_orientation_error_norm(0.0f),
        yz_pitch_raw_position_error_norm(0.0f),
        cart_servo_yz_pitch_position_error(0.0f),
        cart_servo_yz_pitch_orientation_error(0.0f),
        cart_servo_singular_position_improvement(0.0f),
        cart_servo_singular_orientation_weight(1.0f),
        cart_servo_singular_velocity_scale(1.0f),
        joint_traj_valid(false),
        cart_traj_valid(false),
        joint_servo_valid(false),
        cart_servo_valid(false),
        cart_servo_local_valid(false),
        cart_servo_spatial_valid(false),
        cart_servo_roll_only_full_reached(false),
        cart_servo_tool_z_valid(false),
        cart_servo_yaw_pitch_valid(false),
        cart_servo_yz_pitch_valid(false),
        analytic_ik_valid(false),
        gravity_valid(false),
        robust_ik_valid(false),
        constrained_ik_valid(false),
        position_relaxed_ik_valid(false),
        yz_pitch_ik_valid(false),
        actuator_coupling_valid(false),
        hybrid_ik_valid(false) {}
};

Demo3RReport run_demo_3r();
bool run_demo_3r_success();

}  // namespace demo
}  // namespace arm_lib

#endif
