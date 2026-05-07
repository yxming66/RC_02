#include "three_pit_cartesian_app.h"

#include "../kinematics/fk.h"
#include "../kinematics/ik_solution_scoring.h"
#include "../kinematics/task_constraints.h"
#include "../servo/servo_l_controller.h"
#include "../solver/solver_common.h"

namespace mr::robotics::arm {
namespace application {

namespace {

constexpr Scalar kTaskScaleCandidates[] = {
    1.0f,        0.5f,        0.25f,       0.125f,
    0.0625f,     0.03125f,    0.015625f,   0.0078125f,
    0.00390625f, 0.001953125f, 0.0009765625f};

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
  candidate_yz_pitch_ = make_candidate_pose(
      target_yz_pitch_, y_axis, z_axis, pitch_axis, dt, 1.0f);
  const bool has_task_delta =
      abs_scalar(candidate_yz_pitch_.y - target_yz_pitch_.y) >
          ARM_LIB_EPSILON ||
      abs_scalar(candidate_yz_pitch_.z - target_yz_pitch_.z) >
          ARM_LIB_EPSILON ||
      abs_scalar(candidate_yz_pitch_.pitch - target_yz_pitch_.pitch) >
          ARM_LIB_EPSILON;

  const Transform current_pose = kinematics::fk(chain_, feedback.q);
  target_transform_ = make_target_transform(target_yz_pitch_, current_pose);
  const JointVec3 command_reference =
      has_last_valid_ ? last_valid_q_ : feedback.q;
  const JointVec3 previous_qd =
      has_previous_qd_ ? previous_qd_ : feedback.qd;

  if (!workspace_contains(config_.workspace, candidate_yz_pitch_)) {
    candidate_rejected_out_of_range_ = true;
    make_hold_command(command_reference);
    previous_qd_ = toolbox_adapter::zero_joint_vec<kThreePitCartesianDof>();
    has_previous_qd_ = true;
    state_ = ThreePitCartesianAppState::kTargetOutOfRangeRejected;
    hold_reason_ = ThreePitCartesianHoldReason::kNone;
    fill_result(&result);
    return result;
  }

  const ScaledCandidateSearch search = try_solve_scaled_candidate(
      target_yz_pitch_, current_pose, feedback, y_axis, z_axis,
      pitch_axis, dt, command_reference, previous_qd, has_task_delta);

  if (!search.accepted) {
    if (!is_success(target_ik_result_.status)) {
      const kinematics::IKOptions target_ik_options =
          make_yz_pitch_ik_options(make_target_transform(candidate_yz_pitch_,
                                                         current_pose));
      if (ik_failure_is_singularity(target_ik_result_,
                                    target_ik_options)) {
        enter_hold(ThreePitCartesianAppState::kSingularityHold,
                   ThreePitCartesianHoldReason::kTargetSingularity);
      } else if (ik_failure_is_reachability_boundary(target_ik_result_)) {
        enter_hold(ThreePitCartesianAppState::kTargetUnreachableHold,
                   ThreePitCartesianHoldReason::kTargetUnreachable);
      } else {
        enter_hold(ThreePitCartesianAppState::kFaultHold,
                   ThreePitCartesianHoldReason::kTargetIkFailure);
      }
    } else if (search.saw_safety_failure) {
      enter_hold(ThreePitCartesianAppState::kFaultHold,
                 ThreePitCartesianHoldReason::kSafetyFailure);
    } else if (search.saw_ik_failure) {
      enter_hold(ThreePitCartesianAppState::kFaultHold,
                 ThreePitCartesianHoldReason::kStepIkFailure);
    } else {
      enter_hold(ThreePitCartesianAppState::kTargetUnreachableHold,
                 ThreePitCartesianHoldReason::kTargetUnreachable);
    }
    fill_result(&result);
    return result;
  }

  target_reachable_ = true;
  target_yz_pitch_ = search.accepted_target;
  target_transform_ = search.accepted_transform;
  target_updated_ = has_task_delta;
  safety_result_ = search.accepted_safety;
  command_ = search.accepted_command;
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

bool ThreePitCartesianApp::ik_failure_is_singularity(
    const kinematics::IKResult<kThreePitCartesianDof>& result,
    const kinematics::IKOptions& options) {
  if (is_success(result.status)) {
    return false;
  }
  if (result.status == IkStatus::kSingular ||
      result.diagnostics.reason == IkFailureReason::kSingular) {
    return true;
  }

  const Scalar threshold = options.singularity_threshold;
  if (!is_finite_scalar(threshold) || threshold <= ARM_LIB_EPSILON) {
    return false;
  }

  Scalar metric = result.singularity_metric;
  if (!is_finite_scalar(metric) || metric <= 0.0f) {
    metric = result.diagnostics.singularity_metric;
  }
  if (!is_finite_scalar(metric) || metric <= 0.0f) {
    return false;
  }
  return metric <= threshold;
}

bool ThreePitCartesianApp::ik_failure_is_reachability_boundary(
    const kinematics::IKResult<kThreePitCartesianDof>& result) {
  if (is_success(result.status)) {
    return false;
  }

  if (result.status == IkStatus::kUnreachable ||
      result.status == IkStatus::kLimitViolation ||
      result.diagnostics.reason == IkFailureReason::kUnreachable ||
      result.diagnostics.reason == IkFailureReason::kLimitViolation) {
    return true;
  }

  return result.status == IkStatus::kNoConvergence ||
         result.status == IkStatus::kMaxIterations ||
         result.status == IkStatus::kNumericalFailure;
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

YzPitchPose ThreePitCartesianApp::make_candidate_pose(
    const YzPitchPose& base,
    Scalar y_axis,
    Scalar z_axis,
    Scalar pitch_axis,
    Scalar dt,
    Scalar scale) const {
  YzPitchPose candidate = base;
  candidate.y += y_axis * config_.max_y_velocity * dt * scale;
  candidate.z += z_axis * config_.max_z_velocity * dt * scale;
  candidate.pitch += pitch_axis * config_.max_pitch_velocity * dt * scale;
  return candidate;
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

ThreePitCartesianApp::CandidateSolution
ThreePitCartesianApp::solve_candidate(
    const YzPitchPose& candidate,
    const Transform& current_pose,
    const JointVec3& reference_q,
    const JointState3& feedback) const {
  CandidateSolution result;
  result.pose = candidate;
  result.transform = make_target_transform(candidate, current_pose);
  result.ik_result =
      solve_target_ik(result.transform, reference_q, feedback.q);
  return result;
}

ThreePitCartesianApp::JointCommand3
ThreePitCartesianApp::make_command_from_solution(
    const JointVec3& reference_q,
    const JointVec3& solution_q,
    const JointVec3& previous_qd,
    Scalar dt) const {
  JointCommand3 command;
  command.q = solution_q;
  command.qd = joint_delta_from_current(reference_q, solution_q) / dt;
  command.qdd = (command.qd - previous_qd) / dt;
  command.mode = JointCommandMode::kPositionVelocityTorque;
  command.valid = true;
  return command;
}

safety::JointCommandSafetyResult
ThreePitCartesianApp::validate_candidate_command(
    const JointCommand3& command,
    const JointVec3& reference_q) const {
  return safety::validate_joint_command(
      command, make_safety_config(reference_q), 1.0e-5f);
}

ThreePitCartesianApp::ScaledCandidateSearch
ThreePitCartesianApp::try_solve_scaled_candidate(
    const YzPitchPose& base,
    const Transform& current_pose,
    const JointState3& feedback,
    Scalar y_axis,
    Scalar z_axis,
    Scalar pitch_axis,
    Scalar dt,
    const JointVec3& reference_q,
    const JointVec3& previous_qd,
    bool has_task_delta) {
  ScaledCandidateSearch search;
  search.accepted = false;
  search.saw_ik_failure = false;
  search.saw_safety_failure = false;
  search.accepted_target = base;
  search.accepted_transform = make_target_transform(base, current_pose);
  search.accepted_command = JointCommand3();
  search.accepted_safety = safety::JointCommandSafetyResult();
  search.representative_ik_failure =
      kinematics::IKResult<kThreePitCartesianDof>();

  for (Scalar scale : kTaskScaleCandidates) {
    const YzPitchPose scaled_candidate = make_candidate_pose(
        base, y_axis, z_axis, pitch_axis, dt, scale);
    if (!workspace_contains(config_.workspace, scaled_candidate)) {
      continue;
    }

    const CandidateSolution solution =
        solve_candidate(scaled_candidate, current_pose, reference_q, feedback);
    const bool original_candidate = scale >= 1.0f - ARM_LIB_EPSILON;
    if (original_candidate) {
      target_ik_result_ = solution.ik_result;
    }

    if (!is_success(solution.ik_result.status)) {
      if (!search.saw_ik_failure) {
        search.representative_ik_failure = solution.ik_result;
        if (!original_candidate) {
          step_ik_result_ = solution.ik_result;
        }
      }
      search.saw_ik_failure = true;
      if (original_candidate) {
        return search;
      }
      continue;
    }

    JointCommand3 next_command = make_command_from_solution(
        reference_q, solution.ik_result.q, previous_qd, dt);
    safety::JointCommandSafetyResult safety_result =
        validate_candidate_command(next_command, reference_q);
    if (!safety_result.ok && !has_task_delta) {
      next_command.q = reference_q;
      next_command.qd =
          toolbox_adapter::zero_joint_vec<kThreePitCartesianDof>();
      next_command.qdd =
          toolbox_adapter::zero_joint_vec<kThreePitCartesianDof>();
      safety_result = safety::JointCommandSafetyResult();
    }
    if (!safety_result.ok) {
      safety_result_ = safety_result;
      search.saw_safety_failure = true;
      continue;
    }

    search.accepted = true;
    search.accepted_target = scaled_candidate;
    search.accepted_transform = solution.transform;
    search.accepted_command = next_command;
    search.accepted_safety = safety_result;
    step_ik_result_ = solution.ik_result;
    return search;
  }

  return search;
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
