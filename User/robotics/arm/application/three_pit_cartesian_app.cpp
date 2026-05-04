#include "three_pit_cartesian_app.h"

#include "../kinematics/fk.h"
#include "../kinematics/ik_solution_scoring.h"
#include "../kinematics/task_constraints.h"
#include "../servo/servo_l_controller.h"
#include "../solver/solver_common.h"
#include "../trajectory/cartesian_traj.h"

namespace mr::robotics::arm {
namespace application {

namespace {

Rotation rotation_from_axis_angle(const Vec3& axis, Scalar angle) {
  const Scalar norm = axis.norm();
  if (norm <= ARM_LIB_EPSILON) {
    return toolbox_adapter::identity_rotation();
  }

  const Vec3 unit_axis = axis / norm;
  Vec4 angvec = matrixf::zeros<4, 1>();
  angvec[0][0] = unit_axis[0][0];
  angvec[1][0] = unit_axis[1][0];
  angvec[2][0] = unit_axis[2][0];
  angvec[3][0] = angle;
  return ::robotics::angvec2r(angvec);
}

}  // namespace

ThreePitCartesianAppConfig::ThreePitCartesianAppConfig()
    : workspace(),
      input_deadzone(0.05f),
      max_y_velocity(0.30f),
      max_z_velocity(0.30f),
      max_pitch_velocity(1.0f),
      max_linear_velocity(0.20f),
      max_angular_velocity(1.0f),
      max_linear_acceleration(0.50f),
      max_angular_acceleration(2.0f),
      max_joint_velocity(toolbox_adapter::zero_joint_vec<kThreePitCartesianDof>()),
      max_joint_acceleration(
          toolbox_adapter::zero_joint_vec<kThreePitCartesianDof>()),
      max_joint_step(0.04f),
      joint_limits(),
      enforce_joint_limits(true),
      limit_joint_velocity(true),
      limit_joint_acceleration(true),
      limit_joint_step(true),
      ik_options(kinematics::make_ik_options(
          kinematics::IkProfile::kEmbeddedSafe)) {
  for (uint16_t i = 0; i < kThreePitCartesianDof; ++i) {
    max_joint_velocity[i][0] = 2.0f;
    max_joint_acceleration[i][0] = 8.0f;
  }
  ik_options.strategy = kinematics::IkSolveStrategy::kNumericOnly;
  ik_options.analytic_solver_preset =
      kinematics::AnalyticSolverPreset::kDisabled;
  ik_options.error_tolerance = 2.0e-3f;
  ik_options.joint_centering_gain = 0.08f;
  ik_options.reference_bias_gain = 0.08f;
}

ThreePitCartesianAppResult::ThreePitCartesianAppResult()
    : state(ThreePitCartesianAppState::kIdle),
      hold_reason(ThreePitCartesianHoldReason::kNone),
      target_yz_pitch(),
      last_valid_target_yz_pitch(),
      candidate_yz_pitch(),
      command(),
      safety_result(),
      target_ik_result(),
      step_ik_result(),
      candidate_rejected_out_of_range(false),
      target_updated(false),
      target_reachable(false),
      command_updated(false) {}

ThreePitCartesianApp::ThreePitCartesianApp()
    : chain_(),
      config_(),
      configured_(false),
      has_last_valid_(false),
      has_previous_qd_(false),
      state_(ThreePitCartesianAppState::kIdle),
      hold_reason_(ThreePitCartesianHoldReason::kUnconfigured),
      target_yz_pitch_(),
      last_valid_target_yz_pitch_(),
      candidate_yz_pitch_(),
      last_valid_q_(toolbox_adapter::zero_joint_vec<kThreePitCartesianDof>()),
      previous_qd_(toolbox_adapter::zero_joint_vec<kThreePitCartesianDof>()),
      command_(),
      target_transform_(toolbox_adapter::identity_transform()),
      safety_result_(),
      target_ik_result_(),
      step_ik_result_(),
      candidate_rejected_out_of_range_(false),
      target_updated_(false),
      target_reachable_(false),
      command_updated_(false) {}

bool ThreePitCartesianApp::configure(
    const SerialChain<kThreePitCartesianDof>& chain,
    const ThreePitCartesianAppConfig& config) {
  chain_ = chain;
  config_ = config;
  if (!has_any_limit(config_.joint_limits)) {
    config_.joint_limits = chain_.joint_limits();
  }
  configured_ = chain_.validate();
  if (!configured_) {
    enter_hold(ThreePitCartesianAppState::kFaultHold,
               ThreePitCartesianHoldReason::kUnconfigured);
  }
  return configured_;
}

void ThreePitCartesianApp::set_config(
    const ThreePitCartesianAppConfig& config) {
  config_ = config;
  if (configured_ && !has_any_limit(config_.joint_limits)) {
    config_.joint_limits = chain_.joint_limits();
  }
}

bool ThreePitCartesianApp::reset_from_feedback(const JointState3& feedback) {
  candidate_rejected_out_of_range_ = false;
  target_updated_ = false;
  target_reachable_ = false;
  command_updated_ = false;
  target_ik_result_ = kinematics::IKResult<kThreePitCartesianDof>();
  step_ik_result_ = kinematics::IKResult<kThreePitCartesianDof>();
  safety_result_ = safety::JointCommandSafetyResult();

  if (!configured_) {
    enter_hold(ThreePitCartesianAppState::kFaultHold,
               ThreePitCartesianHoldReason::kUnconfigured);
    return false;
  }
  if (!joint_state_is_usable(feedback)) {
    enter_hold(ThreePitCartesianAppState::kFaultHold,
               ThreePitCartesianHoldReason::kInvalidFeedback);
    return false;
  }

  const Transform current_pose = kinematics::fk(chain_, feedback.q);
  target_yz_pitch_ = yz_pitch_from_feedback(current_pose, feedback.q);
  last_valid_target_yz_pitch_ = target_yz_pitch_;
  candidate_yz_pitch_ = target_yz_pitch_;
  target_transform_ = current_pose;
  last_valid_q_ = feedback.q;
  previous_qd_ = toolbox_adapter::zero_joint_vec<kThreePitCartesianDof>();
  has_previous_qd_ = true;
  has_last_valid_ = true;
  make_hold_command(feedback.q);
  state_ = ThreePitCartesianAppState::kIdle;
  hold_reason_ = ThreePitCartesianHoldReason::kNone;
  return true;
}

ThreePitCartesianAppResult ThreePitCartesianApp::update(
    const JointState3& feedback,
    const ThreePitRemoteInput& input,
    Scalar dt) {
  candidate_rejected_out_of_range_ = false;
  target_updated_ = false;
  target_reachable_ = false;
  command_updated_ = false;
  safety_result_ = safety::JointCommandSafetyResult();
  target_ik_result_ = kinematics::IKResult<kThreePitCartesianDof>();
  step_ik_result_ = kinematics::IKResult<kThreePitCartesianDof>();

  ThreePitCartesianAppResult result;
  if (!configured_) {
    enter_hold(ThreePitCartesianAppState::kFaultHold,
               ThreePitCartesianHoldReason::kUnconfigured);
    fill_result(&result);
    return result;
  }
  if (!is_finite_scalar(dt) || dt <= ARM_LIB_EPSILON) {
    enter_hold(ThreePitCartesianAppState::kFaultHold,
               ThreePitCartesianHoldReason::kInvalidDt);
    fill_result(&result);
    return result;
  }
  if (!remote_input_is_finite(input)) {
    enter_hold(ThreePitCartesianAppState::kFaultHold,
               ThreePitCartesianHoldReason::kInvalidInput);
    fill_result(&result);
    return result;
  }
  if (!joint_state_is_usable(feedback)) {
    enter_hold(ThreePitCartesianAppState::kFaultHold,
               ThreePitCartesianHoldReason::kInvalidFeedback);
    fill_result(&result);
    return result;
  }
  if (!has_last_valid_ || input.reset_target_to_feedback) {
    (void)reset_from_feedback(feedback);
    if (input.reset_target_to_feedback) {
      fill_result(&result);
      return result;
    }
  }
  if (!input.enabled) {
    enter_hold(ThreePitCartesianAppState::kIdle,
               ThreePitCartesianHoldReason::kDisabled);
    fill_result(&result);
    return result;
  }

  const Scalar y_axis = apply_deadzone(input.y_axis, config_.input_deadzone);
  const Scalar z_axis = apply_deadzone(input.z_axis, config_.input_deadzone);
  const Scalar pitch_axis =
      apply_deadzone(input.pitch_axis, config_.input_deadzone);

  candidate_yz_pitch_ = target_yz_pitch_;
  candidate_yz_pitch_.y += y_axis * config_.max_y_velocity * dt;
  candidate_yz_pitch_.z += z_axis * config_.max_z_velocity * dt;
  candidate_yz_pitch_.pitch +=
      pitch_axis * config_.max_pitch_velocity * dt;

  if (workspace_contains(config_.workspace, candidate_yz_pitch_)) {
    target_yz_pitch_ = candidate_yz_pitch_;
    target_updated_ = true;
  } else {
    candidate_rejected_out_of_range_ = true;
  }

  const Transform current_pose = kinematics::fk(chain_, feedback.q);
  target_transform_ = make_target_transform(target_yz_pitch_, current_pose);

  target_ik_result_ =
      solve_target_ik(target_transform_, last_valid_q_, feedback.q);
  if (!is_success(target_ik_result_.status)) {
    enter_hold(ThreePitCartesianAppState::kTargetUnreachableHold,
               ThreePitCartesianHoldReason::kTargetIkFailure);
    fill_result(&result);
    return result;
  }
  target_reachable_ = true;

  trajectory::CartesianTrajectoryRequest trajectory_request;
  trajectory_request.start = current_pose;
  trajectory_request.goal = target_transform_;
  trajectory_request.max_linear_velocity = config_.max_linear_velocity;
  trajectory_request.max_angular_velocity = config_.max_angular_velocity;
  trajectory_request.max_linear_acceleration = config_.max_linear_acceleration;
  trajectory_request.max_angular_acceleration =
      config_.max_angular_acceleration;

  trajectory::CartesianTrajectory trajectory;
  if (!trajectory.configure(trajectory_request)) {
    enter_hold(ThreePitCartesianAppState::kFaultHold,
               ThreePitCartesianHoldReason::kTrajectoryFailure);
    fill_result(&result);
    return result;
  }

  const trajectory::CartesianTrajectorySample sample = trajectory.sample(dt);
  if (!sample.valid) {
    enter_hold(ThreePitCartesianAppState::kFaultHold,
               ThreePitCartesianHoldReason::kTrajectoryFailure);
    fill_result(&result);
    return result;
  }

  step_ik_result_ = solve_target_ik(sample.pose, last_valid_q_, feedback.q);
  if (!is_success(step_ik_result_.status)) {
    enter_hold(ThreePitCartesianAppState::kTargetUnreachableHold,
               ThreePitCartesianHoldReason::kStepIkFailure);
    fill_result(&result);
    return result;
  }

  JointVec3 desired_qd =
      joint_delta_from_current(feedback.q, step_ik_result_.q) / dt;
  bool limited = false;
  if (config_.limit_joint_velocity) {
    desired_qd = servo::clamp_joint_velocity(
        desired_qd, config_.max_joint_velocity, &limited);
  }

  const JointVec3 acceleration_reference =
      has_previous_qd_ ? previous_qd_ : feedback.qd;
  if (config_.limit_joint_acceleration) {
    bool accel_limited = false;
    desired_qd = servo::clamp_joint_acceleration(
        desired_qd, acceleration_reference, config_.max_joint_acceleration,
        dt, &accel_limited);
    limited = limited || accel_limited;
  }
  if (config_.limit_joint_step) {
    bool step_limited = false;
    desired_qd = servo::clamp_joint_step(
        desired_qd, config_.max_joint_step, dt, &step_limited);
    limited = limited || step_limited;
  }
  if (config_.enforce_joint_limits) {
    bool position_limited = false;
    desired_qd = servo::clamp_to_position_limits_as_velocity(
        feedback.q, desired_qd, config_.joint_limits, dt, &position_limited);
    limited = limited || position_limited;
  }
  (void)limited;

  JointCommand3 next_command;
  next_command.q = feedback.q + desired_qd * dt;
  next_command.qd = desired_qd;
  next_command.qdd = (desired_qd - acceleration_reference) / dt;
  next_command.mode = JointCommandMode::kPositionVelocityTorque;
  next_command.valid = true;

  const safety::JointCommandSafetyConfig<kThreePitCartesianDof> safety_config =
      make_safety_config(feedback.q);
  safety_result_ = safety::validate_joint_command(next_command, safety_config,
                                                  1.0e-5f);
  if (!safety_result_.ok) {
    enter_hold(ThreePitCartesianAppState::kFaultHold,
               ThreePitCartesianHoldReason::kSafetyFailure);
    fill_result(&result);
    return result;
  }

  command_ = next_command;
  last_valid_q_ = command_.q;
  previous_qd_ = command_.qd;
  has_previous_qd_ = true;
  has_last_valid_ = true;
  last_valid_target_yz_pitch_ = target_yz_pitch_;
  state_ = candidate_rejected_out_of_range_
               ? ThreePitCartesianAppState::kTargetOutOfRangeRejected
               : ThreePitCartesianAppState::kTracking;
  hold_reason_ = ThreePitCartesianHoldReason::kNone;
  command_updated_ = true;

  fill_result(&result);
  return result;
}

bool ThreePitCartesianApp::joint_state_is_usable(const JointState3& state) {
  if (!state.valid || !solver::is_joint_vector_finite(state.q) ||
      !solver::is_joint_vector_finite(state.qd) ||
      !solver::is_joint_vector_finite(state.torque)) {
    return false;
  }
  for (uint16_t i = 0; i < kThreePitCartesianDof; ++i) {
    if (!state.online[i]) {
      return false;
    }
  }
  return true;
}

bool ThreePitCartesianApp::remote_input_is_finite(
    const ThreePitRemoteInput& input) {
  return is_finite_scalar(input.y_axis) &&
         is_finite_scalar(input.z_axis) &&
         is_finite_scalar(input.pitch_axis);
}

Scalar ThreePitCartesianApp::apply_deadzone(Scalar axis, Scalar deadzone) {
  Scalar value = clamp_scalar(axis, -1.0f, 1.0f);
  Scalar dz = deadzone;
  if (!is_finite_scalar(dz) || dz < 0.0f) {
    dz = 0.0f;
  }
  if (dz > 0.99f) {
    dz = 0.99f;
  }
  const Scalar abs_value = abs_scalar(value);
  if (abs_value <= dz) {
    return 0.0f;
  }
  const Scalar scaled = (abs_value - dz) / (1.0f - dz);
  return (value >= 0.0f) ? scaled : -scaled;
}

Scalar ThreePitCartesianApp::planar_pitch_from_joints(
    const JointVec3& q) const {
  Scalar pitch = 0.0f;
  for (uint16_t i = 0; i < kThreePitCartesianDof; ++i) {
    pitch += chain_.link(i).joint_offset() + q[i][0];
  }
  return pitch;
}

YzPitchPose ThreePitCartesianApp::yz_pitch_from_feedback(
    const Transform& transform,
    const JointVec3& q) const {
  const Vec3 translation = toolbox_adapter::translation_of(transform);
  YzPitchPose pose;
  pose.y = translation[1][0];
  pose.z = translation[2][0];
  pose.pitch = planar_pitch_from_joints(q);
  return pose;
}

bool ThreePitCartesianApp::workspace_contains(
    const ThreePitWorkspaceLimits& workspace,
    const YzPitchPose& pose) {
  if (!is_finite_scalar(pose.y) || !is_finite_scalar(pose.z) ||
      !is_finite_scalar(pose.pitch)) {
    return false;
  }
  if (workspace.check_y &&
      (pose.y < workspace.y_min || pose.y > workspace.y_max)) {
    return false;
  }
  if (workspace.check_z &&
      (pose.z < workspace.z_min || pose.z > workspace.z_max)) {
    return false;
  }
  if (workspace.check_pitch &&
      (pose.pitch < workspace.pitch_min ||
       pose.pitch > workspace.pitch_max)) {
    return false;
  }
  return true;
}

Rotation ThreePitCartesianApp::make_planar_pitch_rotation(
    Scalar pitch) const {
  const JointVec3 zero_q =
      toolbox_adapter::zero_joint_vec<kThreePitCartesianDof>();
  const Transform first_joint_frame =
      chain_.base_frame() * chain_.link(0).joint_origin_transform();
  const Vec3 pitch_axis_base =
      toolbox_adapter::rotation_of(first_joint_frame) *
      chain_.link(0).joint_axis_local();
  const Scalar zero_pitch = planar_pitch_from_joints(zero_q);
  const Rotation zero_rotation =
      toolbox_adapter::rotation_of(kinematics::fk(chain_, zero_q));

  return rotation_from_axis_angle(pitch_axis_base, pitch - zero_pitch) *
         zero_rotation;
}

Transform ThreePitCartesianApp::make_target_transform(
    const YzPitchPose& pose,
    const Transform& reference_pose) const {
  Vec3 translation = toolbox_adapter::translation_of(reference_pose);
  translation[1][0] = pose.y;
  translation[2][0] = pose.z;

  return toolbox_adapter::make_transform(
      make_planar_pitch_rotation(pose.pitch), translation);
}

kinematics::IKOptions ThreePitCartesianApp::make_yz_pitch_ik_options(
    const Transform& target_pose) const {
  kinematics::IKOptions options = config_.ik_options;
  kinematics::apply_task_constraint_profile_to_ik_options(
      target_pose, kinematics::make_yz_pitch_profile(1.0f), &options);
  options.enable_solution_step_limit = false;
  return options;
}

kinematics::IKResult<kThreePitCartesianDof>
ThreePitCartesianApp::solve_target_ik(
    const Transform& target_pose,
    const JointVec3& seed,
    const JointVec3& current) const {
  kinematics::IKRequest<kThreePitCartesianDof> request;
  request.target = target_pose;
  request.seed = seed;
  request.reference = seed;
  request.current = current;
  request.use_seed = true;
  request.use_reference = true;
  request.use_current = true;
  return kinematics::solve_ik(chain_, request,
                              make_yz_pitch_ik_options(target_pose));
}

ThreePitCartesianApp::JointVec3
ThreePitCartesianApp::joint_delta_from_current(
    const JointVec3& current,
    const JointVec3& target) const {
  JointVec3 delta = toolbox_adapter::zero_joint_vec<kThreePitCartesianDof>();
  for (uint16_t i = 0; i < kThreePitCartesianDof; ++i) {
    delta[i][0] = kinematics::ik_joint_delta(chain_, current, target, i);
  }
  return delta;
}

safety::JointCommandSafetyConfig<kThreePitCartesianDof>
ThreePitCartesianApp::make_safety_config(const JointVec3& current_q) const {
  safety::JointCommandSafetyConfig<kThreePitCartesianDof> safety_config;
  safety_config.position_limits = config_.joint_limits;
  safety_config.max_velocity = config_.max_joint_velocity;
  safety_config.max_acceleration = config_.max_joint_acceleration;
  safety_config.current_position = current_q;
  safety_config.max_position_step =
      toolbox_adapter::zero_joint_vec<kThreePitCartesianDof>();
  for (uint16_t i = 0; i < kThreePitCartesianDof; ++i) {
    safety_config.max_position_step[i][0] = config_.max_joint_step;
  }
  safety_config.check_position = config_.enforce_joint_limits;
  safety_config.check_velocity = config_.limit_joint_velocity;
  safety_config.check_acceleration = config_.limit_joint_acceleration;
  safety_config.check_joint_step = config_.limit_joint_step;
  return safety_config;
}

void ThreePitCartesianApp::make_hold_command(const JointVec3& q) {
  command_ = JointCommand3();
  command_.q = q;
  command_.qd = toolbox_adapter::zero_joint_vec<kThreePitCartesianDof>();
  command_.qdd = toolbox_adapter::zero_joint_vec<kThreePitCartesianDof>();
  command_.mode = JointCommandMode::kPositionVelocityTorque;
  command_.valid = solver::is_joint_vector_finite(q);
}

void ThreePitCartesianApp::enter_hold(
    ThreePitCartesianAppState state,
    ThreePitCartesianHoldReason reason) {
  const JointVec3 hold_q =
      has_last_valid_ ? last_valid_q_
                      : toolbox_adapter::zero_joint_vec<kThreePitCartesianDof>();
  make_hold_command(hold_q);
  previous_qd_ = toolbox_adapter::zero_joint_vec<kThreePitCartesianDof>();
  has_previous_qd_ = true;
  state_ = state;
  hold_reason_ = reason;
}

void ThreePitCartesianApp::fill_result(
    ThreePitCartesianAppResult* result) const {
  if (result == nullptr) {
    return;
  }
  result->state = state_;
  result->hold_reason = hold_reason_;
  result->target_yz_pitch = target_yz_pitch_;
  result->last_valid_target_yz_pitch = last_valid_target_yz_pitch_;
  result->candidate_yz_pitch = candidate_yz_pitch_;
  result->command = command_;
  result->safety_result = safety_result_;
  result->target_ik_result = target_ik_result_;
  result->step_ik_result = step_ik_result_;
  result->candidate_rejected_out_of_range =
      candidate_rejected_out_of_range_;
  result->target_updated = target_updated_;
  result->target_reachable = target_reachable_;
  result->command_updated = command_updated_;
}

}  // namespace application
}  // namespace mr::robotics::arm
