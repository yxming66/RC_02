#ifndef ARM_LIB_CONTROLLER_ARM_CONTROLLER_H
#define ARM_LIB_CONTROLLER_ARM_CONTROLLER_H

#include "../core/arm_command.h"
#include "../core/arm_status.h"
#include "../dynamics/gravity.h"
#include "../kinematics/fk.h"
#include "../kinematics/ik_dispatch.h"
#include "../kinematics/ik_redundancy.h"
#include "../kinematics/ik_solution_scoring.h"
#include "../kinematics/jacobian.h"
#include "../kinematics/task_constraints.h"
#include "../model/serial_chain.h"
#include "../planning/move_j_planner.h"
#include "../planning/move_l_planner.h"
#include "../safety/joint_command_safety.h"
#include "../servo/servo_l_controller.h"

namespace mr::robotics::arm {
namespace controller {

enum class ArmControlMode : uint8_t {
  kIdle = 0,
  kGravityComp,
  kMoveJ,
  kMoveL,
  kServoL,
  kFault,
};

enum class ArmMotionState : uint8_t {
  kStopped = 0,
  kMoving,
  kReached,
  kFault,
};

enum class ArmFaultReason : uint8_t {
  kNone = 0,
  kUnconfigured,
  kInvalidFeedback,
  kInvalidCommand,
  kLimitViolation,
  kNumericalFailure,
  kUnsupported,
  kEmergencyStop,
};

struct ArmControllerDiagnostics {
  ArmStatus status;
  ArmControlMode mode;
  ArmMotionState motion_state;
  ArmFaultReason fault_reason;
  Transform current_tcp_pose;
  safety::SafetyLimitReason safety_reason;
  Index safety_joint_index;
  IkStatus ik_status;
  servo::ServoLStatus servo_l_status;
  Scalar singularity_metric;
  Scalar dt;
  Scalar motion_time;
  Scalar motion_duration;
  bool command_valid;

  ArmControllerDiagnostics()
      : status(ArmStatus::kUninitialized),
        mode(ArmControlMode::kIdle),
        motion_state(ArmMotionState::kStopped),
        fault_reason(ArmFaultReason::kNone),
        current_tcp_pose(toolbox_adapter::identity_transform()),
        safety_reason(safety::SafetyLimitReason::kNone),
        safety_joint_index(0U),
        ik_status(IkStatus::kSuccess),
        servo_l_status(servo::ServoLStatus::kSuccess),
        singularity_metric(0.0f),
        dt(0.0f),
        motion_time(0.0f),
        motion_duration(0.0f),
        command_valid(false) {}
};

template <int N>
struct MoveLControllerRequest {
  planning::MoveLRequest cartesian;
  JointVec<N> max_velocity;
  JointVec<N> max_acceleration;
  JointLimits<N> limits;
  Scalar max_joint_step;
  Scalar singularity_fault_threshold;
  bool enforce_limits;
  bool limit_joint_step;
  bool fault_on_singularity;
  bool position_priority_only;
  kinematics::IKOptions ik_options;

  MoveLControllerRequest()
      : cartesian(),
        max_velocity(matrixf::zeros<N, 1>()),
        max_acceleration(matrixf::zeros<N, 1>()),
        limits(),
        max_joint_step(0.0f),
        singularity_fault_threshold(0.0f),
        enforce_limits(false),
        limit_joint_step(false),
        fault_on_singularity(false),
        position_priority_only(false),
        ik_options(kinematics::make_ik_options(
            kinematics::IkProfile::kRobust)) {
    for (uint16_t i = 0; i < N; ++i) {
      max_velocity[i][0] = 1.0f;
      max_acceleration[i][0] = 1.0f;
    }
  }
};

template <int N>
struct MoveJToPoseControllerRequest {
  Transform target;
  JointVec<N> max_velocity;
  JointVec<N> max_acceleration;
  JointLimits<N> limits;
  bool enforce_limits;
  bool position_priority_only;
  kinematics::IKOptions ik_options;

  MoveJToPoseControllerRequest()
      : target(toolbox_adapter::identity_transform()),
        max_velocity(matrixf::zeros<N, 1>()),
        max_acceleration(matrixf::zeros<N, 1>()),
        limits(),
        enforce_limits(false),
        position_priority_only(false),
        ik_options(kinematics::make_ik_options(
            kinematics::IkProfile::kRobust)) {
    for (uint16_t i = 0; i < N; ++i) {
      max_velocity[i][0] = 1.0f;
      max_acceleration[i][0] = 1.0f;
    }
  }
};

template <int N>
class ArmController {
 public:
  ArmController()
      : chain_(nullptr),
        configured_(false),
        enabled_(false),
        mode_(ArmControlMode::kIdle),
        motion_state_(ArmMotionState::kStopped),
        state_(),
        command_(),
        move_j_(),
        move_j_elapsed_(0.0f),
        move_l_(),
        move_l_request_(),
        move_l_elapsed_(0.0f),
        move_l_reference_q_(matrixf::zeros<N, 1>()),
        move_l_previous_qd_(matrixf::zeros<N, 1>()),
        servo_l_(),
        servo_l_last_result_(),
        gravity_feedforward_enabled_(false),
        safety_config_(),
        gravity_scale_(matrixf::ones<N, 1>()),
        default_kp_(matrixf::zeros<N, 1>()),
        default_kd_(matrixf::zeros<N, 1>()),
        diagnostics_() {}

  void set_chain(const SerialChain<N>& chain) {
    chain_ = &chain;
    configured_ = chain.validate();
    safety_config_.position_limits = chain.joint_limits();
    safety_config_.check_position = configured_;
    diagnostics_.status =
        configured_ ? ArmStatus::kOk : ArmStatus::kInvalidArgument;
  }

  bool configured() const { return configured_; }

  void set_enabled(bool enabled) {
    enabled_ = enabled;
    if (!enabled_) {
      set_idle();
    }
  }

  bool enabled() const { return enabled_; }

  void set_default_gains(const JointVec<N>& kp, const JointVec<N>& kd) {
    default_kp_ = kp;
    default_kd_ = kd;
  }

  void set_gravity_scales(const JointVec<N>& scales) {
    gravity_scale_ = scales;
  }

  void set_gravity_scale(uint16_t index, Scalar scale) {
    if (index < N) {
      gravity_scale_[index][0] = scale;
    }
  }

  void set_gravity_feedforward_enabled(bool enabled) {
    gravity_feedforward_enabled_ = enabled;
  }

  void set_torque_feedforward_limits(const JointVec<N>& max_torque_ff) {
    safety_config_.max_torque_ff = max_torque_ff;
    safety_config_.check_torque_ff = true;
  }

  void disable_torque_feedforward_limits() {
    safety_config_.check_torque_ff = false;
  }

  void update_feedback(const JointState<N>& state) {
    state_ = state;
    if (!joint_state_is_usable(state_)) {
      enter_fault(ArmFaultReason::kInvalidFeedback,
                  ArmStatus::kInvalidArgument);
      update_diagnostics(0.0f);
      return;
    }
    if (chain_ != nullptr) {
      diagnostics_.current_tcp_pose = kinematics::fk(*chain_, state_.q);
    }
  }

  void set_idle() {
    if (mode_ == ArmControlMode::kFault ||
        motion_state_ == ArmMotionState::kFault) {
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(0.0f);
      return;
    }
    mode_ = ArmControlMode::kIdle;
    motion_state_ = ArmMotionState::kStopped;
    command_ = make_disabled_joint_command<N>();
    move_j_elapsed_ = 0.0f;
    move_l_elapsed_ = 0.0f;
    servo_l_.reset();
    diagnostics_.safety_reason = safety::SafetyLimitReason::kNone;
    update_diagnostics(0.0f);
  }

  void soft_stop() {
    set_idle();
  }

  void request_estop() {
    enter_fault(ArmFaultReason::kEmergencyStop, ArmStatus::kEmergencyStop);
    command_ = make_disabled_joint_command<N>();
    move_j_elapsed_ = 0.0f;
    move_l_elapsed_ = 0.0f;
    servo_l_.reset();
    diagnostics_.command_valid = false;
    update_diagnostics(0.0f);
  }

  ArmStatus start_move_j(const planning::MoveJRequest<N>& request) {
    if (fault_latched()) {
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }
    if (!configured_) {
      enter_fault(ArmFaultReason::kUnconfigured, ArmStatus::kUninitialized);
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }
    if (!joint_state_is_usable(state_)) {
      enter_fault(ArmFaultReason::kInvalidFeedback,
                  ArmStatus::kInvalidArgument);
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }
    if (!move_j_.configure(request)) {
      enter_fault(ArmFaultReason::kLimitViolation,
                  ArmStatus::kInvalidArgument);
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }

    mode_ = ArmControlMode::kMoveJ;
    motion_state_ = ArmMotionState::kMoving;
    move_j_elapsed_ = 0.0f;
    reset_motion_safety_checks();
    safety_config_.max_velocity = request.max_velocity;
    safety_config_.max_acceleration = request.max_acceleration;
    safety_config_.check_velocity = true;
    safety_config_.check_acceleration = true;
    if (request.enforce_limits) {
      safety_config_.position_limits = request.limits;
      safety_config_.check_position = true;
    }
    command_ = make_disabled_joint_command<N>();
    diagnostics_.status = ArmStatus::kOk;
    diagnostics_.fault_reason = ArmFaultReason::kNone;
    diagnostics_.safety_reason = safety::SafetyLimitReason::kNone;
    update_diagnostics(0.0f);
    return ArmStatus::kOk;
  }

  ArmStatus start_move_j(const JointVec<N>& goal,
                         const JointVec<N>& max_velocity,
                         const JointVec<N>& max_acceleration) {
    if (chain_ == nullptr) {
      enter_fault(ArmFaultReason::kUnconfigured, ArmStatus::kUninitialized);
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }

    planning::MoveJRequest<N> request;
    request.start = state_.q;
    request.goal = goal;
    request.max_velocity = max_velocity;
    request.max_acceleration = max_acceleration;
    request.limits = chain_->joint_limits();
    request.enforce_limits = true;
    return start_move_j(request);
  }

  ArmStatus start_move_j_to_pose(
      const MoveJToPoseControllerRequest<N>& request) {
    if (fault_latched()) {
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }
    if (!configured_) {
      enter_fault(ArmFaultReason::kUnconfigured, ArmStatus::kUninitialized);
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }
    if (!joint_state_is_usable(state_) || chain_ == nullptr) {
      enter_fault(ArmFaultReason::kInvalidFeedback,
                  ArmStatus::kInvalidArgument);
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }

    Transform target = request.target;
    kinematics::IKOptions options = request.ik_options;
    if (request.position_priority_only) {
      target = make_position_priority_target(target, state_.q);
      options = kinematics::make_yz_pitch_ik_options(
          target, kinematics::IkProfile::kPositionOnly);
      options.error_tolerance = 2.0e-3f;
      options.joint_centering_gain = 0.08f;
      options.null_space_max_step = 0.08f;
    }

    kinematics::IKRequest<N> ik_request;
    ik_request.target = target;
    ik_request.seed = state_.q;
    ik_request.reference = state_.q;
    ik_request.current = state_.q;
    ik_request.use_seed = true;
    ik_request.use_reference = true;
    ik_request.use_current = true;

    const kinematics::IKResult<N> ik_result =
        kinematics::solve_ik(*chain_, ik_request, options);
    diagnostics_.ik_status = ik_result.status;
    if (!is_success(ik_result.status)) {
      enter_fault(ArmFaultReason::kNumericalFailure,
                  ArmStatus::kNumericalFailure);
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }

    planning::MoveJRequest<N> move_j_request;
    move_j_request.start = state_.q;
    move_j_request.goal = ik_result.q;
    move_j_request.max_velocity = request.max_velocity;
    move_j_request.max_acceleration = request.max_acceleration;
    move_j_request.limits = request.limits;
    move_j_request.enforce_limits = request.enforce_limits;

    const ArmStatus status = start_move_j(move_j_request);
    diagnostics_.ik_status = ik_result.status;
    return status;
  }

  kinematics::IKResult<N> solve_pose_ik(
      const Transform& target,
      const JointVec<N>& seed,
      bool use_seed,
      bool position_priority_only,
      kinematics::IKOptions options) const {
    kinematics::IKResult<N> result;
    if (!configured_ || chain_ == nullptr) {
      result.status = IkStatus::kInvalidModel;
      result.diagnostics.stage = IkFailStage::kModelValidation;
      result.diagnostics.reason = IkFailureReason::kInvalidModel;
      return result;
    }

    JointVec<N> reference = use_seed ? seed : state_.q;
    Transform resolved_target = target;
    if (position_priority_only) {
      resolved_target = make_position_priority_target(target, reference);
      options = kinematics::make_yz_pitch_ik_options(
          resolved_target, kinematics::IkProfile::kPositionOnly);
      options.error_tolerance = 2.0e-3f;
      options.joint_centering_gain = 0.08f;
      options.null_space_max_step = 0.08f;
    }

    kinematics::IKRequest<N> request;
    request.target = resolved_target;
    request.seed = reference;
    request.reference = reference;
    request.current = state_.q;
    request.use_seed = true;
    request.use_reference = true;
    request.use_current = joint_state_is_usable(state_);
    return kinematics::solve_ik(*chain_, request, options);
  }

  kinematics::IKResult<N> solve_pose_ik(
      const Transform& target,
      const JointVec<N>& seed,
      bool use_seed,
      bool position_priority_only) const {
    kinematics::IKOptions options = kinematics::make_ik_options(
        position_priority_only ? kinematics::IkProfile::kPositionOnly
                               : kinematics::IkProfile::kRobust);
    options.error_tolerance = position_priority_only ? 2.0e-3f : 1.0e-3f;
    options.joint_centering_gain = 0.08f;
    options.null_space_max_step = 0.08f;
    return solve_pose_ik(target, seed, use_seed, position_priority_only,
                         options);
  }

  ArmStatus start_move_l(const MoveLControllerRequest<N>& request) {
    if (fault_latched()) {
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }
    if (!configured_) {
      enter_fault(ArmFaultReason::kUnconfigured, ArmStatus::kUninitialized);
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }
    if (!joint_state_is_usable(state_)) {
      enter_fault(ArmFaultReason::kInvalidFeedback,
                  ArmStatus::kInvalidArgument);
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }
    if (!move_l_.configure(request.cartesian)) {
      enter_fault(ArmFaultReason::kInvalidCommand,
                  ArmStatus::kInvalidArgument);
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }

    mode_ = ArmControlMode::kMoveL;
    motion_state_ = ArmMotionState::kMoving;
    move_l_request_ = request;
    move_l_elapsed_ = 0.0f;
    move_l_reference_q_ = state_.q;
    move_l_previous_qd_ = matrixf::zeros<N, 1>();
    reset_motion_safety_checks();
    safety_config_.max_velocity = request.max_velocity;
    safety_config_.max_acceleration = request.max_acceleration;
    safety_config_.check_velocity = true;
    safety_config_.check_acceleration = true;
    if (request.limit_joint_step && request.max_joint_step > ARM_LIB_EPSILON) {
      for (uint16_t i = 0; i < N; ++i) {
        safety_config_.max_position_step[i][0] = request.max_joint_step;
      }
      safety_config_.check_joint_step = true;
    }
    if (request.fault_on_singularity &&
        request.singularity_fault_threshold > ARM_LIB_EPSILON) {
      safety_config_.singularity_threshold =
          request.singularity_fault_threshold;
      safety_config_.check_singularity = true;
    }
    if (request.enforce_limits) {
      safety_config_.position_limits = request.limits;
      safety_config_.check_position = true;
    }
    command_ = make_disabled_joint_command<N>();
    diagnostics_.status = ArmStatus::kOk;
    diagnostics_.fault_reason = ArmFaultReason::kNone;
    diagnostics_.safety_reason = safety::SafetyLimitReason::kNone;
    diagnostics_.ik_status = IkStatus::kSuccess;
    update_diagnostics(0.0f);
    return ArmStatus::kOk;
  }

  ArmStatus start_servo_l(const servo::ServoLRequest<N>& request) {
    if (fault_latched()) {
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }
    if (!configured_) {
      enter_fault(ArmFaultReason::kUnconfigured, ArmStatus::kUninitialized);
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }
    if (!joint_state_is_usable(state_)) {
      enter_fault(ArmFaultReason::kInvalidFeedback,
                  ArmStatus::kInvalidArgument);
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }
    if (!servo_l_.configure(request)) {
      enter_fault(ArmFaultReason::kInvalidCommand,
                  ArmStatus::kInvalidArgument);
      update_diagnostics(0.0f);
      return diagnostics_.status;
    }

    mode_ = ArmControlMode::kServoL;
    motion_state_ = ArmMotionState::kMoving;
    servo_l_last_result_ = servo::ServoLResult<N>();
    reset_motion_safety_checks();
    safety_config_.max_velocity = request.config.max_velocity;
    safety_config_.max_acceleration = request.config.max_acceleration;
    safety_config_.position_limits = request.config.position_limits;
    safety_config_.check_velocity = request.config.limit_joint_velocity;
    safety_config_.check_acceleration =
        request.config.limit_joint_acceleration;
    safety_config_.check_position = request.config.enforce_position_limits;
    if (request.config.limit_joint_step &&
        request.config.max_joint_step > ARM_LIB_EPSILON) {
      for (uint16_t i = 0; i < N; ++i) {
        safety_config_.max_position_step[i][0] =
            request.config.max_joint_step;
      }
      safety_config_.check_joint_step = true;
    }
    if (request.config.fault_on_singularity &&
        request.config.singularity_fault_threshold > ARM_LIB_EPSILON) {
      safety_config_.singularity_threshold =
          request.config.singularity_fault_threshold;
      safety_config_.check_singularity = true;
    }
    command_ = make_disabled_joint_command<N>();
    diagnostics_.status = ArmStatus::kOk;
    diagnostics_.fault_reason = ArmFaultReason::kNone;
    diagnostics_.safety_reason = safety::SafetyLimitReason::kNone;
    diagnostics_.servo_l_status = servo::ServoLStatus::kSuccess;
    diagnostics_.singularity_metric = 0.0f;
    update_diagnostics(0.0f);
    return ArmStatus::kOk;
  }

  void set_servo_l_target_pose(const Transform& target_pose) {
    servo_l_.set_target_pose(target_pose);
  }

  void set_servo_l_twist_ff(const Twist6& twist_ff) {
    servo_l_.set_twist_ff(twist_ff);
  }

  ArmStatus update(Scalar dt) {
    switch (mode_) {
      case ArmControlMode::kMoveJ:
        return update_move_j(dt);
      case ArmControlMode::kMoveL:
        return update_move_l(dt);
      case ArmControlMode::kServoL:
        return update_servo_l(dt);
      case ArmControlMode::kGravityComp:
        return command_gravity_hold(dt);
      case ArmControlMode::kIdle:
        command_ = make_disabled_joint_command<N>();
        motion_state_ = ArmMotionState::kStopped;
        update_diagnostics(dt);
        return configured_ ? ArmStatus::kOk : ArmStatus::kUninitialized;
      case ArmControlMode::kFault:
        command_ = make_disabled_joint_command<N>();
        update_diagnostics(dt);
        return diagnostics_.status;
      default:
        enter_fault(ArmFaultReason::kUnsupported, ArmStatus::kUnsupported);
        command_ = make_disabled_joint_command<N>();
        update_diagnostics(dt);
        return diagnostics_.status;
    }
  }

  ArmStatus update_move_j(Scalar dt) {
    if (!begin_command(dt, ArmControlMode::kMoveJ)) {
      return diagnostics_.status;
    }
    if (!move_j_.valid()) {
      enter_fault(ArmFaultReason::kInvalidCommand,
                  ArmStatus::kInvalidArgument);
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(dt);
      return diagnostics_.status;
    }

    move_j_elapsed_ += dt;
    const planning::MoveJSample<N> sample = move_j_.sample(move_j_elapsed_);
    if (!sample.valid) {
      enter_fault(ArmFaultReason::kNumericalFailure,
                  ArmStatus::kNumericalFailure);
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(dt);
      return diagnostics_.status;
    }

    command_.q = sample.position;
    command_.qd = sample.velocity;
    command_.qdd = sample.acceleration;
    command_.kp = default_kp_;
    command_.kd = default_kd_;
    command_.torque_ff =
        gravity_feedforward_enabled_ ? gravity_torque_scaled()
                                     : matrixf::zeros<N, 1>();
    command_.mode = JointCommandMode::kPositionVelocityTorque;
    command_.valid = true;

    return finish_command(
        dt, sample.finished ? ArmMotionState::kReached
                            : ArmMotionState::kMoving);
  }

  ArmStatus update_move_l(Scalar dt) {
    if (!begin_command(dt, ArmControlMode::kMoveL)) {
      return diagnostics_.status;
    }
    if (!move_l_.valid() || chain_ == nullptr) {
      enter_fault(ArmFaultReason::kInvalidCommand,
                  ArmStatus::kInvalidArgument);
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(dt);
      return diagnostics_.status;
    }
    if (dt <= ARM_LIB_EPSILON) {
      enter_fault(ArmFaultReason::kInvalidCommand,
                  ArmStatus::kInvalidArgument);
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(dt);
      return diagnostics_.status;
    }

    move_l_elapsed_ += dt;
    const planning::MoveLSample sample = move_l_.sample(move_l_elapsed_);
    if (!sample.valid) {
      enter_fault(ArmFaultReason::kNumericalFailure,
                  ArmStatus::kNumericalFailure);
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(dt);
      return diagnostics_.status;
    }

    Transform target = sample.pose;
    kinematics::IKOptions options = move_l_request_.ik_options;
    if (move_l_request_.position_priority_only) {
      target = make_position_priority_target(target, move_l_reference_q_);
      options = kinematics::make_yz_pitch_ik_options(
          target, kinematics::IkProfile::kPositionOnly);
      options.error_tolerance = 2.0e-3f;
      options.joint_centering_gain = 0.08f;
      options.null_space_max_step = 0.08f;
    }
    if (move_l_request_.limit_joint_step &&
        move_l_request_.max_joint_step > ARM_LIB_EPSILON) {
      options.enable_solution_step_limit = true;
      options.max_solution_joint_step = move_l_request_.max_joint_step;
    }

    const JointVec<N> step_reference = move_l_reference_q_;
    JointVec<N> predicted_seed = step_reference + move_l_previous_qd_ * dt;
    if (move_l_request_.enforce_limits) {
      predicted_seed =
          clamp_to_joint_limits(move_l_request_.limits, predicted_seed);
    }

    kinematics::IKRequest<N> ik_request;
    ik_request.target = target;
    ik_request.seed = predicted_seed;
    ik_request.reference = step_reference;
    ik_request.current = state_.q;
    ik_request.use_seed = true;
    ik_request.use_reference = true;
    ik_request.use_current = true;

    const kinematics::IKResult<N> ik_result =
        kinematics::solve_ik(*chain_, ik_request, options);
    diagnostics_.ik_status = ik_result.status;
    if (!is_success(ik_result.status)) {
      enter_fault(ArmFaultReason::kNumericalFailure,
                  ArmStatus::kNumericalFailure);
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(dt);
      return diagnostics_.status;
    }

    const JointVec<N> q_delta =
        joint_delta_from_reference(step_reference, ik_result.q);
    const JointVec<N> qd = q_delta / dt;
    const JointVec<N> qdd = (qd - move_l_previous_qd_) / dt;
    diagnostics_.singularity_metric =
        kinematics::singularity_metric(kinematics::jacobian(*chain_, ik_result.q));
    safety_config_.singularity_metric = diagnostics_.singularity_metric;

    command_.q = ik_result.q;
    command_.qd = qd;
    command_.qdd = qdd;
    command_.kp = default_kp_;
    command_.kd = default_kd_;
    command_.torque_ff =
        gravity_feedforward_enabled_ ? gravity_torque_scaled()
                                     : matrixf::zeros<N, 1>();
    command_.mode = JointCommandMode::kPositionVelocityTorque;
    command_.valid = true;

    const ArmStatus status = finish_command(
        dt, sample.finished ? ArmMotionState::kReached
                            : ArmMotionState::kMoving);
    if (is_ok(status)) {
      move_l_reference_q_ = ik_result.q;
      move_l_previous_qd_ = qd;
    }
    return status;
  }

  ArmStatus update_servo_l(Scalar dt) {
    if (!begin_command(dt, ArmControlMode::kServoL)) {
      return diagnostics_.status;
    }
    if (chain_ == nullptr) {
      enter_fault(ArmFaultReason::kUnconfigured, ArmStatus::kUninitialized);
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(dt);
      return diagnostics_.status;
    }

    servo_l_last_result_ = servo_l_.step(*chain_, state_, dt);
    diagnostics_.servo_l_status = servo_l_last_result_.status;
    diagnostics_.singularity_metric =
        servo_l_last_result_.dls_result.singularity_metric;
    safety_config_.singularity_metric = diagnostics_.singularity_metric;
    if (!servo_l_last_result_.ok) {
      enter_fault(servo_l_last_result_.status == servo::ServoLStatus::kDlsFailure
                      ? ArmFaultReason::kNumericalFailure
                      : ArmFaultReason::kInvalidCommand,
                  servo_l_last_result_.status == servo::ServoLStatus::kDlsFailure
                      ? ArmStatus::kNumericalFailure
                      : ArmStatus::kInvalidArgument);
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(dt);
      return diagnostics_.status;
    }

    command_.q = servo_l_last_result_.q;
    command_.qd = servo_l_last_result_.qd;
    command_.qdd = servo_l_last_result_.qdd;
    command_.kp = default_kp_;
    command_.kd = default_kd_;
    command_.torque_ff =
        gravity_feedforward_enabled_ ? gravity_torque_scaled()
                                     : matrixf::zeros<N, 1>();
    command_.mode = JointCommandMode::kPositionVelocityTorque;
    command_.valid = true;

    return finish_command(
        dt, servo_l_last_result_.reached ? ArmMotionState::kReached
                                         : ArmMotionState::kMoving);
  }

  ArmStatus command_gravity_hold(Scalar dt) {
    if (!begin_command(dt, ArmControlMode::kGravityComp)) {
      return diagnostics_.status;
    }
    reset_motion_safety_checks();

    command_ = make_hold_joint_command(state_);
    command_.mode = JointCommandMode::kPositionVelocityTorque;
    command_.kp = default_kp_;
    command_.kd = default_kd_;
    command_.torque_ff = gravity_torque_scaled();
    command_.valid = true;

    return finish_command(dt);
  }

  ArmStatus command_gravity_torque(Scalar dt) {
    if (!begin_command(dt, ArmControlMode::kGravityComp)) {
      return diagnostics_.status;
    }
    reset_motion_safety_checks();

    command_.q = state_.q;
    command_.qd = matrixf::zeros<N, 1>();
    command_.qdd = matrixf::zeros<N, 1>();
    command_.kp = matrixf::zeros<N, 1>();
    command_.kd = matrixf::zeros<N, 1>();
    command_.torque_ff = gravity_torque_scaled();
    command_.mode = JointCommandMode::kTorque;
    command_.valid = true;

    return finish_command(dt);
  }

  ArmStatus command_hold_position(Scalar dt) {
    if (!begin_command(dt, ArmControlMode::kGravityComp)) {
      return diagnostics_.status;
    }
    reset_motion_safety_checks();

    command_ = make_hold_joint_command(state_);
    command_.mode = JointCommandMode::kPositionVelocityTorque;
    command_.kp = default_kp_;
    command_.kd = default_kd_;
    command_.valid = true;

    return finish_command(dt);
  }

  ArmStatus command_joint_position(const JointVec<N>& target,
                                   Scalar dt,
                                   Scalar position_tolerance = 1.0e-3f) {
    if (!begin_command(dt, ArmControlMode::kMoveJ)) {
      return diagnostics_.status;
    }
    reset_motion_safety_checks();

    command_.q = target;
    command_.qd = matrixf::zeros<N, 1>();
    command_.qdd = matrixf::zeros<N, 1>();
    command_.kp = default_kp_;
    command_.kd = default_kd_;
    command_.torque_ff =
        gravity_feedforward_enabled_ ? gravity_torque_scaled()
                                     : matrixf::zeros<N, 1>();
    command_.mode = JointCommandMode::kPositionVelocityTorque;
    command_.valid = true;

    const JointVec<N> error = joint_delta_from_reference(state_.q, target);
    Scalar max_abs_error = 0.0f;
    for (uint16_t i = 0; i < N; ++i) {
      const Scalar abs_error = abs_scalar(error[i][0]);
      if (abs_error > max_abs_error) {
        max_abs_error = abs_error;
      }
    }

    return finish_command(dt,
                          max_abs_error <= position_tolerance
                              ? ArmMotionState::kReached
                              : ArmMotionState::kMoving);
  }

  const JointCommand<N>& command() const { return command_; }

  ArmControlMode mode() const { return mode_; }
  ArmMotionState motion_state() const { return motion_state_; }
  const ArmControllerDiagnostics& diagnostics() const { return diagnostics_; }
  const planning::MoveJRequest<N>& move_j_request() const {
    return move_j_.request();
  }

  void clear_fault() {
    if (mode_ == ArmControlMode::kFault ||
        motion_state_ == ArmMotionState::kFault) {
      mode_ = ArmControlMode::kIdle;
      motion_state_ = ArmMotionState::kStopped;
      command_ = make_disabled_joint_command<N>();
      servo_l_.reset();
      diagnostics_.fault_reason = ArmFaultReason::kNone;
      diagnostics_.safety_reason = safety::SafetyLimitReason::kNone;
      diagnostics_.status = configured_ ? ArmStatus::kOk : ArmStatus::kUninitialized;
      update_diagnostics(0.0f);
    }
  }

 private:
  static bool joint_state_is_usable(const JointState<N>& state) {
    if (!state.valid) {
      return false;
    }
    for (uint16_t i = 0; i < N; ++i) {
      if (!is_finite_scalar(state.q[i][0]) ||
          !is_finite_scalar(state.qd[i][0]) ||
          !is_finite_scalar(state.torque[i][0])) {
        return false;
      }
    }
    return true;
  }

  bool fault_latched() const {
    return mode_ == ArmControlMode::kFault ||
           motion_state_ == ArmMotionState::kFault;
  }

  void reset_motion_safety_checks() {
    safety_config_.current_position = state_.q;
    safety_config_.max_velocity = matrixf::zeros<N, 1>();
    safety_config_.max_acceleration = matrixf::zeros<N, 1>();
    safety_config_.max_position_step = matrixf::zeros<N, 1>();
    safety_config_.singularity_metric = 0.0f;
    safety_config_.singularity_threshold = 0.0f;
    safety_config_.check_velocity = false;
    safety_config_.check_acceleration = false;
    safety_config_.check_joint_step = false;
    safety_config_.check_singularity = false;
    if (chain_ != nullptr) {
      safety_config_.position_limits = chain_->joint_limits();
      safety_config_.check_position = true;
    } else {
      safety_config_.check_position = false;
    }
  }

  bool begin_command(Scalar dt, ArmControlMode requested_mode) {
    if (!configured_) {
      enter_fault(ArmFaultReason::kUnconfigured, ArmStatus::kUninitialized);
      update_diagnostics(dt);
      return false;
    }
    if (fault_latched()) {
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(dt);
      return false;
    }
    if (!enabled_) {
      set_idle();
      update_diagnostics(dt);
      return false;
    }
    if (!joint_state_is_usable(state_)) {
      enter_fault(ArmFaultReason::kInvalidFeedback,
                  ArmStatus::kInvalidArgument);
      update_diagnostics(dt);
      return false;
    }
    safety_config_.current_position = state_.q;
    mode_ = requested_mode;
    motion_state_ = ArmMotionState::kMoving;
    diagnostics_.status = ArmStatus::kOk;
    diagnostics_.fault_reason = ArmFaultReason::kNone;
    return true;
  }

  ArmStatus finish_command(Scalar dt,
                           ArmMotionState next_motion_state =
                               ArmMotionState::kReached) {
    if (!joint_command_is_finite(command_)) {
      enter_fault(ArmFaultReason::kInvalidCommand,
                  ArmStatus::kNumericalFailure);
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(dt);
      return diagnostics_.status;
    }

    const safety::JointCommandSafetyResult safety_result =
        safety::validate_joint_command(command_, safety_config_);
    if (!safety_result.ok) {
      diagnostics_.safety_reason = safety_result.reason;
      diagnostics_.safety_joint_index = safety_result.joint_index;
      enter_fault(ArmFaultReason::kLimitViolation, safety_status(safety_result));
      command_ = make_disabled_joint_command<N>();
      update_diagnostics(dt);
      return diagnostics_.status;
    }

    motion_state_ = next_motion_state;
    diagnostics_.safety_reason = safety::SafetyLimitReason::kNone;
    update_diagnostics(dt);
    return ArmStatus::kOk;
  }

  static ArmStatus safety_status(
      const safety::JointCommandSafetyResult& safety_result) {
    if (safety_result.reason == safety::SafetyLimitReason::kInvalidCommand) {
      return ArmStatus::kInvalidArgument;
    }
    return ArmStatus::kLimitViolation;
  }

  JointVec<N> joint_delta_from_reference(const JointVec<N>& from,
                                         const JointVec<N>& to) const {
    JointVec<N> delta = to - from;
    if (chain_ == nullptr) {
      return delta;
    }
    for (uint16_t i = 0; i < N; ++i) {
      delta[i][0] = kinematics::ik_joint_delta(*chain_, from, to, i);
    }
    return delta;
  }

  Transform make_position_priority_target(const Transform& target,
                                          const JointVec<N>& reference_q) const {
    if (chain_ == nullptr) {
      return target;
    }

    const Transform reference_pose = kinematics::fk(*chain_, reference_q);
    Vec3 target_rpy = toolbox_adapter::rpy_from_rotation(
        toolbox_adapter::rotation_of(target));
    const Vec3 reference_rpy = toolbox_adapter::rpy_from_rotation(
        toolbox_adapter::rotation_of(reference_pose));
    target_rpy[0][0] = reference_rpy[0][0];
    target_rpy[2][0] = reference_rpy[2][0];
    return toolbox_adapter::transform_from_rpy_translation(
        target_rpy, toolbox_adapter::translation_of(target));
  }

  JointVec<N> gravity_torque_scaled() const {
    JointVec<N> torque = matrixf::zeros<N, 1>();
    if (chain_ == nullptr) {
      return torque;
    }
    torque = dynamics::gravity_torques(*chain_, state_.q);
    for (uint16_t i = 0; i < N; ++i) {
      torque[i][0] *= gravity_scale_[i][0];
    }
    return torque;
  }

  void enter_fault(ArmFaultReason reason, ArmStatus status) {
    mode_ = ArmControlMode::kFault;
    motion_state_ = ArmMotionState::kFault;
    diagnostics_.fault_reason = reason;
    diagnostics_.status = status;
  }

  void update_diagnostics(Scalar dt) {
    diagnostics_.mode = mode_;
    diagnostics_.motion_state = motion_state_;
    diagnostics_.dt = dt;
    if (chain_ != nullptr && joint_state_is_usable(state_)) {
      diagnostics_.current_tcp_pose = kinematics::fk(*chain_, state_.q);
    }
    diagnostics_.motion_time = 0.0f;
    diagnostics_.motion_duration = 0.0f;
    if (mode_ == ArmControlMode::kMoveJ && move_j_.valid()) {
      diagnostics_.motion_time = move_j_elapsed_;
      diagnostics_.motion_duration = move_j_.duration();
    } else if (mode_ == ArmControlMode::kMoveL && move_l_.valid()) {
      diagnostics_.motion_time = move_l_elapsed_;
      diagnostics_.motion_duration = move_l_.duration();
    } else if (mode_ == ArmControlMode::kServoL) {
      diagnostics_.servo_l_status = servo_l_last_result_.status;
      diagnostics_.singularity_metric =
          servo_l_last_result_.dls_result.singularity_metric;
    }
    diagnostics_.command_valid = command_.valid;
  }

  const SerialChain<N>* chain_;
  bool configured_;
  bool enabled_;
  ArmControlMode mode_;
  ArmMotionState motion_state_;
  JointState<N> state_;
  JointCommand<N> command_;
  planning::MoveJPlanner<N> move_j_;
  Scalar move_j_elapsed_;
  planning::MoveLPlanner move_l_;
  MoveLControllerRequest<N> move_l_request_;
  Scalar move_l_elapsed_;
  JointVec<N> move_l_reference_q_;
  JointVec<N> move_l_previous_qd_;
  servo::ServoLController<N> servo_l_;
  servo::ServoLResult<N> servo_l_last_result_;
  bool gravity_feedforward_enabled_;
  safety::JointCommandSafetyConfig<N> safety_config_;
  JointVec<N> gravity_scale_;
  JointVec<N> default_kp_;
  JointVec<N> default_kd_;
  ArmControllerDiagnostics diagnostics_;
};

}  // namespace controller
}  // namespace mr::robotics::arm

#endif
