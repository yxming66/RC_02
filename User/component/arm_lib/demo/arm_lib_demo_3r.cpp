#include "arm_lib_demo_3r.h"

namespace arm_lib {
namespace demo {
namespace {

SerialChain<3> build_demo_chain() {
  DhParams dh[3];
  dh[0].theta = 0.0f;
  dh[0].d = 0.0f;
  dh[0].a = 0.220f;
  dh[0].alpha = 0.0f;

  dh[1].theta = 0.0f;
  dh[1].d = 0.0f;
  dh[1].a = 0.180f;
  dh[1].alpha = 0.0f;

  dh[2].theta = 0.0f;
  dh[2].d = 0.0f;
  dh[2].a = 0.120f;
  dh[2].alpha = 0.0f;

  ChainJointSpec joints[3];
  for (uint16_t i = 0; i < 3; ++i) {
    joints[i].type = ChainJointType::kRevolute;
    joints[i].offset = 0.0f;
    joints[i].limit_enabled = true;
    joints[i].lower = -ARM_LIB_PI;
    joints[i].upper = ARM_LIB_PI;
    joints[i].participate_in_ik = true;
  }

  SerialChain<3> chain =
      parser::load_chain_from_dh(dh, joints, DHConvention::kStandard);
  Vec3 gravity = toolbox_adapter::zero_vec3();
  gravity[1][0] = -9.81f;
  chain.set_gravity(gravity);

  InertialParams inertial[3];
  inertial[0].mass = 0.6f;
  inertial[0].com[0][0] = 0.11f;
  inertial[1].mass = 0.45f;
  inertial[1].com[0][0] = 0.09f;
  inertial[2].mass = 0.25f;
  inertial[2].com[0][0] = 0.06f;

  for (uint16_t i = 0; i < 3; ++i) {
    Link link = chain.link(i);
    link.set_inertial(inertial[i]);
    chain.set_link(i, link);
  }

  return chain;
}

}  // namespace

Demo3RReport run_demo_3r() {
  Demo3RReport report;
  report.chain = build_demo_chain();

  report.q_nominal[0][0] = 0.35f;
  report.q_nominal[1][0] = -0.55f;
  report.q_nominal[2][0] = 0.45f;

  report.q_seed[0][0] = 0.05f;
  report.q_seed[1][0] = -0.10f;
  report.q_seed[2][0] = 0.10f;

  report.seed_pose = kinematics::fk(report.chain, report.q_seed);
  report.target_pose = kinematics::fk(report.chain, report.q_nominal);
  report.jacobian = kinematics::jacobian(report.chain, report.q_nominal);

  kinematics::IKRequest<3> request;
  request.target = report.target_pose;
  request.seed = report.q_seed;
  request.reference = report.q_nominal;
  request.use_seed = true;
  request.use_reference = true;

  kinematics::IKOptions options;
  options.method = kinematics::IkMethod::kLevenbergMarquardt;
  options.max_iterations = 80;
  options.error_tolerance = 1.0e-5f;
  options.step_tolerance = 1.0e-6f;
  options.damping = 1.0e-3f;
  options.clamp_to_limits_each_step = true;

  report.ik_result = kinematics::solve_ik(report.chain, request, options);
  report.q_solution = report.ik_result.q;

  const Transform solved_pose = kinematics::fk(report.chain, report.q_solution);
  const kinematics::PoseError6 error =
      kinematics::evaluate_pose_error(report.target_pose, solved_pose);
  report.position_error_norm = error.position_norm;
  report.orientation_error_norm = error.orientation_norm;

  kinematics::IKRequest<3> robust_request = request;
  robust_request.seed = toolbox_adapter::zero_joint_vec<3>();
  robust_request.use_seed = false;
  kinematics::IKOptions robust_options = options;
  robust_options.enable_multi_start = true;
  robust_options.max_retries = 6U;
  robust_options.enable_singularity_adaptive_damping = true;
  robust_options.retry_seed_offset = 0.30f;
  robust_options.singularity_threshold = 5.0e-3f;
  robust_options.singularity_damping_scale = 20.0f;
  report.robust_ik_result =
      kinematics::solve_ik_numeric(report.chain, robust_request, robust_options);
  report.robust_q_solution = report.robust_ik_result.q;
  const Transform robust_pose =
      kinematics::fk(report.chain, report.robust_q_solution);
  const kinematics::PoseError6 robust_error =
      kinematics::evaluate_pose_error(report.target_pose, robust_pose);
  report.robust_position_error_norm = robust_error.position_norm;
  report.robust_orientation_error_norm = robust_error.orientation_norm;
  report.robust_singularity_metric = report.robust_ik_result.singularity_metric;
  report.robust_ik_valid = is_success(report.robust_ik_result.status);

  report.analytic_plugin = kinematics::analytic::make_planar_3r_plugin();
  report.analytic_ik_result = kinematics::solve_ik_analytic(
      report.chain, request, report.analytic_plugin, 1.0e-4f);
  report.analytic_q_solution = report.analytic_ik_result.q;
  const Transform analytic_pose =
      kinematics::fk(report.chain, report.analytic_q_solution);
  const kinematics::PoseError6 analytic_error =
      kinematics::evaluate_pose_error(report.target_pose, analytic_pose);
  report.analytic_position_error_norm = analytic_error.position_norm;
  report.analytic_orientation_error_norm = analytic_error.orientation_norm;
  report.analytic_ik_valid = is_success(report.analytic_ik_result.status);
  kinematics::HybridIKOptions hybrid_options;
  hybrid_options.analytic_verification_tolerance = 1.0e-4f;
  hybrid_options.branch_switch_hysteresis = 1.0e-3f;
  hybrid_options.numeric_options = robust_options;
  hybrid_options.numeric_options.enable_multi_start = false;
  hybrid_options.numeric_options.max_retries = 1U;
  report.hybrid_ik_result =
      kinematics::solve_ik_hybrid(report.chain, request, report.analytic_plugin,
                                  hybrid_options);
  report.hybrid_q_solution = report.hybrid_ik_result.q;
  const Transform hybrid_pose =
      kinematics::fk(report.chain, report.hybrid_q_solution);
  const kinematics::PoseError6 hybrid_error =
      kinematics::evaluate_pose_error(report.target_pose, hybrid_pose);
  report.hybrid_position_error_norm = hybrid_error.position_norm;
  report.hybrid_orientation_error_norm = hybrid_error.orientation_norm;
  report.hybrid_ik_valid = is_success(report.hybrid_ik_result.status);
  report.gravity_torques =
      dynamics::gravity_torques(report.chain, report.q_nominal);
  report.gravity_torque_norm = report.gravity_torques.norm();
  report.gravity_valid = report.gravity_torque_norm > 1.0e-4f;

  trajectory::JointTrajectoryRequest<3> traj_request;
  traj_request.start = report.q_seed;
  traj_request.goal = report.q_nominal;
  for (uint16_t i = 0; i < 3; ++i) {
    traj_request.max_velocity[i][0] = 1.2f;
    traj_request.max_acceleration[i][0] = 3.0f;
  }

  report.joint_traj_valid = report.joint_trajectory.configure(traj_request);
  report.joint_traj_duration = report.joint_trajectory.duration();
  report.joint_traj_mid =
      report.joint_trajectory.sample(report.joint_traj_duration * 0.5f);
  report.joint_traj_end =
      report.joint_trajectory.sample(report.joint_traj_duration);
  report.joint_traj_end_error_norm =
      solver::configuration_distance(report.chain,
                                     report.joint_traj_end.position,
                                     report.q_nominal);

  trajectory::CartesianTrajectoryRequest cart_request;
  cart_request.start = report.seed_pose;
  cart_request.goal = report.target_pose;
  cart_request.max_linear_velocity = 0.25f;
  cart_request.max_angular_velocity = 1.0f;
  cart_request.max_linear_acceleration = 0.8f;
  cart_request.max_angular_acceleration = 2.5f;

  report.cart_traj_valid = report.cartesian_trajectory.configure(cart_request);
  report.cart_traj_duration = report.cartesian_trajectory.duration();
  report.cart_traj_mid =
      report.cartesian_trajectory.sample(report.cart_traj_duration * 0.5f);
  report.cart_traj_end =
      report.cartesian_trajectory.sample(report.cart_traj_duration);

  const kinematics::PoseError6 cart_error = kinematics::evaluate_pose_error(
      report.target_pose, report.cart_traj_end.pose);
  report.cart_traj_position_error_norm = cart_error.position_norm;
  report.cart_traj_orientation_error_norm = cart_error.orientation_norm;

  control::JointServoConfig<3> joint_servo_config;
  for (uint16_t i = 0; i < 3; ++i) {
    joint_servo_config.max_velocity[i][0] = 1.5f;
  }
  joint_servo_config.position_tolerance = 1.0e-3f;
  report.joint_servo.set_config(joint_servo_config);
  report.joint_servo_result =
      report.joint_servo.step(report.chain, report.q_seed, report.q_nominal, 0.1f);
  report.joint_servo_valid = report.joint_servo_result.valid;
  report.joint_servo_error_after_norm =
      solver::configuration_distance(report.chain,
                                     report.joint_servo_result.q_command,
                                     report.q_nominal);

  control::CartesianServoConfig<3> cart_servo_config;
  cart_servo_config.max_linear_velocity = 0.30f;
  cart_servo_config.max_angular_velocity = 1.2f;
  cart_servo_config.max_linear_acceleration = 0.8f;
  cart_servo_config.max_angular_acceleration = 2.5f;
  cart_servo_config.position_tolerance = 1.0e-3f;
  cart_servo_config.orientation_tolerance = 1.0e-3f;
  cart_servo_config.ik_options.max_iterations = 60;
  cart_servo_config.ik_options.error_tolerance = 1.0e-5f;
  cart_servo_config.ik_options.step_tolerance = 1.0e-6f;
  cart_servo_config.limit_joint_step = true;
  for (uint16_t i = 0; i < 3; ++i) {
    cart_servo_config.joint_servo.max_velocity[i][0] = 2.0f;
  }
  report.cartesian_servo.set_config(cart_servo_config);
  report.cartesian_servo_result = report.cartesian_servo.step(
      report.chain, report.q_seed, report.target_pose, 0.1f);
  report.cart_servo_valid = report.cartesian_servo_result.valid;

  const kinematics::PoseError6 seed_error =
      kinematics::evaluate_pose_error(report.target_pose, report.seed_pose);
  report.cart_servo_position_improvement =
      seed_error.position_norm - report.cartesian_servo_result.position_error_norm;
  report.cart_servo_orientation_improvement =
      seed_error.orientation_norm -
      report.cartesian_servo_result.orientation_error_norm;

  const Transform local_delta_target =
      toolbox_adapter::inverse_transform(report.seed_pose) * report.target_pose;
  report.cartesian_servo_local_result = report.cartesian_servo.step_delta(
      report.chain, report.q_seed, local_delta_target,
      control::CartesianTargetFrame::kToolLocal, 0.1f);
  report.cart_servo_local_valid = report.cartesian_servo_local_result.valid;
  report.cart_servo_local_position_improvement =
      seed_error.position_norm -
      report.cartesian_servo_local_result.position_error_norm;
  report.cart_servo_local_orientation_improvement =
      seed_error.orientation_norm -
      report.cartesian_servo_local_result.orientation_error_norm;

  control::CartesianServoConfig<3> spatial_servo_config = cart_servo_config;
  spatial_servo_config.error_frame = control::CartesianErrorFrame::kSpatial;
  report.cartesian_servo.set_config(spatial_servo_config);
  report.cartesian_servo_spatial_result = report.cartesian_servo.step(
      report.chain, report.q_seed, report.target_pose, 0.1f);
  report.cart_servo_spatial_valid = report.cartesian_servo_spatial_result.valid;
  report.cart_servo_spatial_position_improvement =
      seed_error.position_norm -
      report.cartesian_servo_spatial_result.position_error_norm;
  report.cart_servo_spatial_orientation_improvement =
      seed_error.orientation_norm -
      report.cartesian_servo_spatial_result.orientation_error_norm;

  Vec3 roll_only_rpy = toolbox_adapter::zero_vec3();
  roll_only_rpy[2][0] = 0.35f;
  const Transform roll_only_delta =
      toolbox_adapter::transform_from_rpy_translation(
          roll_only_rpy, toolbox_adapter::zero_vec3());
  const Transform roll_only_target = report.seed_pose * roll_only_delta;

  kinematics::IKRequest<3> constrained_request;
  constrained_request.target = roll_only_target;
  constrained_request.seed = report.q_seed;
  constrained_request.reference = report.q_seed;
  constrained_request.use_seed = true;
  constrained_request.use_reference = true;
  const kinematics::TaskConstraintProfile yaw_pitch_profile =
      kinematics::make_xyz_yaw_pitch_profile();
  kinematics::IKOptions constrained_options = options;
  kinematics::apply_task_constraint_profile_to_ik_options(
      roll_only_target, yaw_pitch_profile, &constrained_options);
  report.constrained_ik_result =
      kinematics::solve_ik_numeric(report.chain, constrained_request,
                                   constrained_options);
  report.constrained_q_solution = report.constrained_ik_result.q;
  const Transform constrained_pose =
      kinematics::fk(report.chain, report.constrained_q_solution);
  const kinematics::PoseError6 constrained_error =
      kinematics::evaluate_constrained_pose_error(
          roll_only_target, constrained_pose, kinematics::PoseErrorFrame::kBody,
          yaw_pitch_profile);
  report.constrained_position_error_norm = constrained_error.position_norm;
  report.constrained_orientation_error_norm = constrained_error.orientation_norm;
  report.constrained_ik_valid = is_success(report.constrained_ik_result.status);

  Transform x_relaxed_target = report.seed_pose;
  x_relaxed_target[0][3] += 0.05f;
  kinematics::IKRequest<3> position_relaxed_request;
  position_relaxed_request.target = x_relaxed_target;
  position_relaxed_request.seed = report.q_seed;
  position_relaxed_request.reference = report.q_seed;
  position_relaxed_request.use_seed = true;
  position_relaxed_request.use_reference = true;
  const kinematics::TaskConstraintProfile position_relaxed_profile =
      kinematics::make_yz_full_profile();
  kinematics::IKOptions position_relaxed_options = options;
  kinematics::apply_task_constraint_profile_to_ik_options(
      x_relaxed_target, position_relaxed_profile, &position_relaxed_options);
  report.position_relaxed_ik_result = kinematics::solve_ik_numeric(
      report.chain, position_relaxed_request, position_relaxed_options);
  report.position_relaxed_q_solution = report.position_relaxed_ik_result.q;
  const Transform position_relaxed_pose =
      kinematics::fk(report.chain, report.position_relaxed_q_solution);
  const kinematics::PoseError6 position_relaxed_error =
      kinematics::evaluate_constrained_pose_error(
          x_relaxed_target, position_relaxed_pose,
          kinematics::PoseErrorFrame::kBody, position_relaxed_profile);
  report.position_relaxed_position_error_norm =
      position_relaxed_error.position_norm;
  report.position_relaxed_orientation_error_norm =
      position_relaxed_error.orientation_norm;
  report.position_relaxed_raw_position_error_norm =
      kinematics::position_error(x_relaxed_target, position_relaxed_pose,
                                 kinematics::PoseErrorFrame::kBody)
          .norm();
  report.position_relaxed_ik_valid =
      is_success(report.position_relaxed_ik_result.status);

    Transform yz_pitch_target = report.seed_pose;
    yz_pitch_target[1][3] += 0.03f;
    yz_pitch_target[2][3] += 0.04f;
    Vec3 yz_pitch_rpy = toolbox_adapter::rpy_from_rotation(
      toolbox_adapter::rotation_of(yz_pitch_target));
    yz_pitch_rpy[1][0] += 0.20f;
    yz_pitch_target = toolbox_adapter::transform_from_rpy_translation(
      yz_pitch_rpy, toolbox_adapter::translation_of(yz_pitch_target));

    kinematics::IKRequest<3> yz_pitch_request;
    yz_pitch_request.target = yz_pitch_target;
    yz_pitch_request.seed = report.q_seed;
    yz_pitch_request.reference = report.q_seed;
    yz_pitch_request.use_seed = true;
    yz_pitch_request.use_reference = true;
    const kinematics::TaskConstraintProfile yz_pitch_profile =
      kinematics::make_yz_pitch_profile();
    kinematics::IKOptions yz_pitch_options = options;
    kinematics::apply_task_constraint_profile_to_ik_options(
      yz_pitch_target, yz_pitch_profile, &yz_pitch_options);
    report.yz_pitch_ik_result =
      kinematics::solve_ik_numeric(report.chain, yz_pitch_request,
                     yz_pitch_options);
    report.yz_pitch_q_solution = report.yz_pitch_ik_result.q;
    const Transform yz_pitch_pose =
      kinematics::fk(report.chain, report.yz_pitch_q_solution);
    const kinematics::PoseError6 yz_pitch_error =
      kinematics::evaluate_constrained_pose_error(
        yz_pitch_target, yz_pitch_pose, kinematics::PoseErrorFrame::kBody,
        yz_pitch_profile);
    report.yz_pitch_position_error_norm = yz_pitch_error.position_norm;
    report.yz_pitch_orientation_error_norm = yz_pitch_error.orientation_norm;
    report.yz_pitch_raw_position_error_norm =
      kinematics::position_error(yz_pitch_target, yz_pitch_pose,
                   kinematics::PoseErrorFrame::kBody)
        .norm();
    report.yz_pitch_ik_valid = is_success(report.yz_pitch_ik_result.status);

  SerialChain<3> coupled_chain = report.chain;
  ChainTransmission transmissions[3];
  transmissions[0].actuator_scale = 6.0f;
  transmissions[1].actuator_scale = 4.0f;
  transmissions[1].add_coupling(0U, 1.5f);
  transmissions[2].actuator_scale = 3.0f;
  transmissions[2].add_coupling(0U, -0.5f);
  transmissions[2].add_coupling(1U, 0.75f);
  parser::set_joint_transmissions(&coupled_chain, transmissions);
  report.actuator_coupled_motor_position =
      coupled_chain.joint_to_actuator_position(report.q_nominal);
  report.actuator_coupled_joint_roundtrip =
      coupled_chain.actuator_to_joint_position(
          report.actuator_coupled_motor_position);
  report.actuator_coupling_roundtrip_error_norm =
      solver::configuration_distance(coupled_chain,
                                     report.q_nominal,
                                     report.actuator_coupled_joint_roundtrip);
  JointVec<3> joint_velocity = toolbox_adapter::zero_joint_vec<3>();
  joint_velocity[0][0] = 0.4f;
  joint_velocity[1][0] = -0.2f;
  joint_velocity[2][0] = 0.1f;
  report.actuator_coupled_motor_velocity =
      coupled_chain.joint_to_actuator_velocity(joint_velocity);
  const JointVec<3> joint_velocity_roundtrip =
      coupled_chain.actuator_to_joint_velocity(
          report.actuator_coupled_motor_velocity);
  report.actuator_coupling_velocity_roundtrip_error_norm =
      solver::configuration_distance(coupled_chain, joint_velocity,
                                     joint_velocity_roundtrip);
  report.actuator_coupling_valid = !coupled_chain.transmission_is_identity() &&
                                   report.actuator_coupling_roundtrip_error_norm <=
                                       1.0e-6f &&
                                   report.actuator_coupling_velocity_roundtrip_error_norm <=
                                       1.0e-6f;

  report.cartesian_servo.set_config(cart_servo_config);
  const control::CartesianServoResult<3> roll_only_full_result =
      report.cartesian_servo.step(report.chain, report.q_seed, roll_only_target,
                                  0.1f);
  report.cart_servo_roll_only_full_reached = roll_only_full_result.reached;
  report.cart_servo_roll_only_full_orientation_error =
      roll_only_full_result.orientation_error_norm;

  control::CartesianServoConfig<3> tool_z_servo_config = cart_servo_config;
  tool_z_servo_config.error_frame = control::CartesianErrorFrame::kBody;
  tool_z_servo_config.orientation_mode =
      control::CartesianOrientationMode::kToolZAxis;
  report.cartesian_servo.set_config(tool_z_servo_config);
  report.cartesian_servo_tool_z_result = report.cartesian_servo.step(
      report.chain, report.q_seed, roll_only_target, 0.1f);
  report.cart_servo_tool_z_valid =
      report.cartesian_servo_tool_z_result.valid &&
      report.cartesian_servo_tool_z_result.reached;
  report.cart_servo_tool_z_orientation_error =
      report.cartesian_servo_tool_z_result.orientation_error_norm;

  control::CartesianServoConfig<3> yaw_pitch_servo_config = cart_servo_config;
  yaw_pitch_servo_config.error_frame = control::CartesianErrorFrame::kBody;
  yaw_pitch_servo_config.orientation_mode =
      control::CartesianOrientationMode::kYawPitchOnly;
  report.cartesian_servo.set_config(yaw_pitch_servo_config);
  report.cartesian_servo_yaw_pitch_result = report.cartesian_servo.step(
      report.chain, report.q_seed, roll_only_target, 0.1f);
  report.cart_servo_yaw_pitch_valid =
      report.cartesian_servo_yaw_pitch_result.valid &&
      report.cartesian_servo_yaw_pitch_result.reached;
  report.cart_servo_yaw_pitch_orientation_error =
      report.cartesian_servo_yaw_pitch_result.orientation_error_norm;

    control::CartesianServoConfig<3> yz_pitch_servo_config = cart_servo_config;
    yz_pitch_servo_config.error_frame = control::CartesianErrorFrame::kBody;
    yz_pitch_servo_config.orientation_mode =
      control::CartesianOrientationMode::kPitchOnly;
    report.cartesian_servo.set_config(yz_pitch_servo_config);
    report.cartesian_servo_yz_pitch_result = report.cartesian_servo.step(
      report.chain, report.q_seed, yz_pitch_target, 0.1f);
    report.cart_servo_yz_pitch_valid =
      report.cartesian_servo_yz_pitch_result.valid;
    report.cart_servo_yz_pitch_position_error =
      report.cartesian_servo_yz_pitch_result.position_error_norm;
    report.cart_servo_yz_pitch_orientation_error =
      report.cartesian_servo_yz_pitch_result.orientation_error_norm;

  JointVec<3> q_near_singular = toolbox_adapter::zero_joint_vec<3>();
  q_near_singular[0][0] = 0.0f;
  q_near_singular[1][0] = 0.0f;
  q_near_singular[2][0] = 0.0f;
  const Transform singular_pose = kinematics::fk(report.chain, q_near_singular);
  Transform singular_target = singular_pose;
  singular_target[0][3] -= 0.04f;
  control::CartesianServoConfig<3> singular_servo_config = cart_servo_config;
  singular_servo_config.singularity_velocity_threshold = 0.5f;
  singular_servo_config.min_velocity_scale = 0.15f;
  singular_servo_config.min_orientation_weight = 0.10f;
  singular_servo_config.singularity_damping_scale = 25.0f;
  report.cartesian_servo.set_config(singular_servo_config);
  report.cartesian_servo_singular_result =
      report.cartesian_servo.step(report.chain, q_near_singular, singular_target,
                                  0.1f);
  const kinematics::PoseError6 singular_seed_error =
      kinematics::evaluate_pose_error(singular_target, singular_pose);
  report.cart_servo_singular_position_improvement =
      singular_seed_error.position_norm -
      report.cartesian_servo_singular_result.position_error_norm;
  report.cart_servo_singular_orientation_weight =
      report.cartesian_servo_singular_result.orientation_weight;
  report.cart_servo_singular_velocity_scale =
      report.cartesian_servo_singular_result.velocity_scale;

  return report;
}

bool run_demo_3r_success() {
  const Demo3RReport report = run_demo_3r();
  if (!is_success(report.ik_result.status)) {
    return false;
  }
  if (report.position_error_norm > 1.0e-3f) {
    return false;
  }
  if (report.orientation_error_norm > 1.0e-3f) {
    return false;
  }
  if (!report.robust_ik_valid) {
    return false;
  }
  if (report.robust_position_error_norm > 1.0e-3f) {
    return false;
  }
  if (report.robust_orientation_error_norm > 1.0e-3f) {
    return false;
  }
  if (report.robust_ik_result.retry_count == 0U) {
    return false;
  }
  if (!report.analytic_ik_valid) {
    return false;
  }
  if (!report.hybrid_ik_valid) {
    return false;
  }
  if (report.hybrid_position_error_norm > 1.0e-4f) {
    return false;
  }
  if (report.hybrid_orientation_error_norm > 1.0e-4f) {
    return false;
  }
  if (!report.hybrid_ik_result.used_analytic &&
      !report.hybrid_ik_result.used_numeric_refine) {
    return false;
  }
  if (report.analytic_position_error_norm > 1.0e-4f) {
    return false;
  }
  if (report.analytic_orientation_error_norm > 1.0e-4f) {
    return false;
  }
  if (!report.joint_traj_valid) {
    return false;
  }
  if (!report.gravity_valid) {
    return false;
  }
  if (!report.joint_traj_end.finished) {
    return false;
  }
  if (report.joint_traj_end_error_norm > 1.0e-5f) {
    return false;
  }
  if (!report.cart_traj_valid) {
    return false;
  }
  if (!report.cart_traj_end.finished) {
    return false;
  }
  if (report.cart_traj_position_error_norm > 1.0e-5f) {
    return false;
  }
  if (report.cart_traj_orientation_error_norm > 1.0e-5f) {
    return false;
  }
  if (!report.joint_servo_valid) {
    return false;
  }
  if (!(report.joint_servo_error_after_norm <
        solver::configuration_distance(report.chain, report.q_seed,
                                       report.q_nominal))) {
    return false;
  }
  if (!report.cart_servo_valid) {
    return false;
  }
  if (!(report.cart_servo_position_improvement > 0.0f)) {
    return false;
  }
  if (!(report.cart_servo_orientation_improvement >= 0.0f)) {
    return false;
  }
  if (!report.cart_servo_local_valid) {
    return false;
  }
  if (!(report.cart_servo_local_position_improvement > 0.0f)) {
    return false;
  }
  if (!(report.cart_servo_local_orientation_improvement >= 0.0f)) {
    return false;
  }
  if (!report.cart_servo_spatial_valid) {
    return false;
  }
  if (!(report.cart_servo_spatial_position_improvement > 0.0f)) {
    return false;
  }
  if (!(report.cart_servo_spatial_orientation_improvement >= 0.0f)) {
    return false;
  }
  if (report.cart_servo_roll_only_full_reached) {
    return false;
  }
  if (report.cart_servo_roll_only_full_orientation_error <= 1.0e-3f) {
    return false;
  }
  if (!report.constrained_ik_valid) {
    return false;
  }
  if (report.constrained_position_error_norm > 1.0e-4f) {
    return false;
  }
  if (report.constrained_orientation_error_norm > 1.0e-4f) {
    return false;
  }
  if (!report.position_relaxed_ik_valid) {
    return false;
  }
  if (report.position_relaxed_position_error_norm > 1.0e-5f) {
    return false;
  }
  if (report.position_relaxed_orientation_error_norm > 1.0e-5f) {
    return false;
  }
  if (report.position_relaxed_raw_position_error_norm < 1.0e-2f) {
    return false;
  }
  if (!report.yz_pitch_ik_valid) {
    return false;
  }
  if (report.yz_pitch_position_error_norm > 1.0e-4f) {
    return false;
  }
  if (report.yz_pitch_orientation_error_norm > 1.0e-4f) {
    return false;
  }
  if (!report.actuator_coupling_valid) {
    return false;
  }
  if (!report.cart_servo_tool_z_valid) {
    return false;
  }
  if (report.cart_servo_tool_z_orientation_error > 1.0e-5f) {
    return false;
  }
  if (!report.cart_servo_yaw_pitch_valid) {
    return false;
  }
  if (report.cart_servo_yaw_pitch_orientation_error > 1.0e-5f) {
    return false;
  }
  if (!report.cart_servo_yz_pitch_valid) {
    return false;
  }
  if (report.cart_servo_yz_pitch_position_error <= 0.0f) {
    return false;
  }
  if (report.cart_servo_yz_pitch_orientation_error <= 0.0f) {
    return false;
  }
  if (!report.cartesian_servo_singular_result.valid) {
    return false;
  }
  if (!(report.cart_servo_singular_position_improvement > 0.0f)) {
    return false;
  }
  if (!(report.cart_servo_singular_velocity_scale < 1.0f)) {
    return false;
  }
  if (!(report.cart_servo_singular_orientation_weight < 1.0f)) {
    return false;
  }
  return true;
}

}  // namespace demo
}  // namespace arm_lib
