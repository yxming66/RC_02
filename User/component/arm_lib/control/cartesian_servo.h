#ifndef ARM_LIB_CONTROL_CARTESIAN_SERVO_H
#define ARM_LIB_CONTROL_CARTESIAN_SERVO_H

#include "cartesian_control_types.h"
#include "../kinematics/ik_dispatch.h"
#include "../trajectory/cartesian_traj.h"
#include "joint_servo.h"

namespace arm_lib {
namespace control {

template <int N>
struct CartesianServoConfig {
  Scalar max_linear_velocity;
  Scalar max_angular_velocity;
  Scalar max_linear_acceleration;
  Scalar max_angular_acceleration;
  Scalar position_tolerance;
  Scalar orientation_tolerance;
  Vec3 position_axis_weights;
  CartesianErrorFrame error_frame;
  CartesianOrientationMode orientation_mode;
  Scalar singularity_velocity_threshold;
  Scalar min_velocity_scale;
  Scalar min_orientation_weight;
  Scalar singularity_damping_scale;
  kinematics::IKOptions ik_options;
  bool use_current_as_reference;
  bool limit_joint_step;
  bool enable_singularity_velocity_scaling;
  bool enable_orientation_relaxation;
  bool enable_singularity_damping_boost;
  JointServoConfig<N> joint_servo;

  CartesianServoConfig()
      : max_linear_velocity(0.2f),
        max_angular_velocity(1.0f),
        max_linear_acceleration(0.5f),
        max_angular_acceleration(2.0f),
        position_tolerance(1.0e-3f),
        orientation_tolerance(1.0e-3f),
        position_axis_weights(toolbox_adapter::zero_vec3()),
        error_frame(CartesianErrorFrame::kBody),
        orientation_mode(CartesianOrientationMode::kFull),
        singularity_velocity_threshold(5.0e-3f),
        min_velocity_scale(0.2f),
        min_orientation_weight(0.2f),
        singularity_damping_scale(10.0f),
        ik_options(kinematics::make_ik_options(kinematics::IkProfile::kRobust)),
        use_current_as_reference(true),
        limit_joint_step(false),
        enable_singularity_velocity_scaling(true),
        enable_orientation_relaxation(true),
        enable_singularity_damping_boost(true),
        joint_servo() {
    for (uint16_t i = 0; i < 3U; ++i) {
      position_axis_weights[i][0] = 1.0f;
    }
  }
};

template <int N>
struct CartesianServoResult {
  Transform current_pose;
  Transform commanded_pose;
  Transform achieved_pose;
  Twist6 commanded_twist;
  JointVec<N> q_command;
  Scalar position_error_norm;
  Scalar orientation_error_norm;
  Scalar singularity_metric;
  Scalar velocity_scale;
  Scalar orientation_weight;
  CartesianServoDiagnostics diagnostics;
  bool position_reached;
  bool orientation_reached;
  bool reached;
  bool valid;
  bool pose_step_finished;
  kinematics::IKResult<N> ik_result;
  JointServoResult<N> joint_result;

  CartesianServoResult()
      : current_pose(toolbox_adapter::identity_transform()),
        commanded_pose(toolbox_adapter::identity_transform()),
        achieved_pose(toolbox_adapter::identity_transform()),
        commanded_twist(toolbox_adapter::zero_twist()),
        q_command(toolbox_adapter::zero_joint_vec<N>()),
        position_error_norm(0.0f),
        orientation_error_norm(0.0f),
        singularity_metric(0.0f),
        velocity_scale(1.0f),
        orientation_weight(1.0f),
        diagnostics(),
        position_reached(false),
        orientation_reached(false),
        reached(false),
        valid(false),
        pose_step_finished(false),
        ik_result(),
        joint_result() {}
};

template <int N>
inline void update_cartesian_servo_diagnostics_from_ik(
    const kinematics::IKResult<N>& ik_result,
    CartesianServoDiagnostics* diagnostics) {
  if (diagnostics == nullptr) {
    return;
  }

  diagnostics->ik_status = ik_result.status;
  diagnostics->ik_failed = !is_success(ik_result.status);
  diagnostics->singularity_metric = ik_result.singularity_metric;
  diagnostics->orientation_relaxed = ik_result.diagnostics.orientation_relaxed;

  if (diagnostics->ik_failed) {
    diagnostics->limit_reason = CartesianServoLimitReason::kIkFailure;
  }
}

template <int N>
class CartesianServo {
 public:
  CartesianServo() : config_(), joint_servo_(config_.joint_servo) {}

  explicit CartesianServo(const CartesianServoConfig<N>& config)
      : config_(config), joint_servo_(config.joint_servo) {}

  const CartesianServoConfig<N>& config() const { return config_; }

  void set_config(const CartesianServoConfig<N>& config) {
    config_ = config;
    joint_servo_.set_config(config.joint_servo);
  }

  CartesianServoResult<N> step(const SerialChain<N>& chain,
                               const JointVec<N>& current_q,
                               const Transform& target_pose,
                               Scalar dt) const {
    return step(chain, current_q,
                CartesianTargetRequest::absolute_pose(target_pose), dt);
  }

  CartesianServoResult<N> step(const SerialChain<N>& chain,
                               const JointVec<N>& current_q,
                               const CartesianTargetRequest& target,
                               Scalar dt) const {
    CartesianServoResult<N> out;
    if (dt <= ARM_LIB_EPSILON) {
      return out;
    }

    out.current_pose = kinematics::fk(chain, current_q);
    const Transform target_pose = resolve_target_pose(out.current_pose, target);
    const kinematics::PoseError6 control_error =
        evaluate_control_pose_error(target_pose, out.current_pose);
    out.position_error_norm = control_error.position_norm;
    out.orientation_error_norm = control_error.orientation_norm;
    out.singularity_metric =
        kinematics::singularity_metric(kinematics::jacobian(chain, current_q));
    out.diagnostics.velocity_scale = out.velocity_scale;
    out.diagnostics.singularity_metric = out.singularity_metric;

    if (out.position_error_norm <= config_.position_tolerance &&
        out.orientation_error_norm <= config_.orientation_tolerance) {
      out.position_reached = true;
      out.orientation_reached = true;
      out.commanded_pose = target_pose;
      out.achieved_pose = out.current_pose;
      out.q_command = current_q;
      out.valid = true;
      out.reached = true;
      out.pose_step_finished = true;
      return out;
    }

    if (config_.singularity_velocity_threshold > ARM_LIB_EPSILON &&
        out.singularity_metric < config_.singularity_velocity_threshold) {
      const Scalar ratio =
          clamp_scalar(out.singularity_metric /
                           config_.singularity_velocity_threshold,
                       0.0f, 1.0f);
      if (config_.enable_singularity_velocity_scaling) {
        out.velocity_scale =
            config_.min_velocity_scale +
            (1.0f - config_.min_velocity_scale) * ratio;
      }
      if (config_.enable_orientation_relaxation) {
        out.orientation_weight =
            config_.min_orientation_weight +
            (1.0f - config_.min_orientation_weight) * ratio;
        out.diagnostics.orientation_relaxed =
            out.orientation_weight < (1.0f - ARM_LIB_EPSILON);
      }
      out.diagnostics.limit_reason =
          CartesianServoLimitReason::kSingularityProtection;
    }
    out.diagnostics.velocity_scale = out.velocity_scale;
    out.diagnostics.singularity_metric = out.singularity_metric;

    trajectory::CartesianTrajectory planner;
    trajectory::CartesianTrajectoryRequest request;
    request.start = out.current_pose;
    request.goal = target_pose;
    request.max_linear_velocity = config_.max_linear_velocity * out.velocity_scale;
    request.max_angular_velocity = config_.max_angular_velocity * out.velocity_scale;
    request.max_linear_acceleration =
        config_.max_linear_acceleration * out.velocity_scale;
    request.max_angular_acceleration =
        config_.max_angular_acceleration * out.velocity_scale;
    if (!planner.configure(request)) {
      return out;
    }

    const trajectory::CartesianTrajectorySample pose_sample = planner.sample(dt);
    out.commanded_pose = pose_sample.pose;
    out.commanded_twist = pose_sample.body_twist;
    out.pose_step_finished = pose_sample.finished;

    kinematics::IKRequest<N> ik_request;
    ik_request.target = out.commanded_pose;
    ik_request.seed = current_q;
    ik_request.use_seed = true;
    if (config_.use_current_as_reference) {
      ik_request.reference = current_q;
      ik_request.use_reference = true;
    }

    kinematics::IKOptions effective_ik_options = config_.ik_options;
    if (config_.enable_singularity_damping_boost &&
        config_.singularity_velocity_threshold > ARM_LIB_EPSILON &&
        out.singularity_metric < config_.singularity_velocity_threshold) {
      const Scalar ratio =
          1.0f - clamp_scalar(out.singularity_metric /
                                  config_.singularity_velocity_threshold,
                              0.0f, 1.0f);
      effective_ik_options.damping = clamp_scalar(
          config_.ik_options.damping *
              (1.0f + config_.singularity_damping_scale * ratio),
          config_.ik_options.damping_min, config_.ik_options.damping_max);
    }
    if (config_.orientation_mode != CartesianOrientationMode::kFull ||
        has_relaxed_position_axes() ||
        (config_.enable_orientation_relaxation &&
         out.orientation_weight < (1.0f - ARM_LIB_EPSILON))) {
      kinematics::TaskConstraintProfile profile;
      profile.position_weights = config_.position_axis_weights;
      profile.orientation_mode = config_.orientation_mode;
      profile.orientation_weight = out.orientation_weight;
      const Scalar boosted_damping = effective_ik_options.damping;
      effective_ik_options = kinematics::make_constrained_ik_options(
        target_pose, profile, kinematics::IkProfile::kRobust);
      effective_ik_options.damping = boosted_damping;
    }

    out.ik_result = kinematics::solve_ik(chain, ik_request, effective_ik_options);
    update_cartesian_servo_diagnostics_from_ik(out.ik_result, &out.diagnostics);
    if (!is_success(out.ik_result.status)) {
      return out;
    }

    JointVec<N> q_command = out.ik_result.q;
    if (config_.limit_joint_step) {
      out.joint_result = joint_servo_.step(chain, current_q, q_command, dt);
      if (!out.joint_result.valid) {
        return out;
      }
      q_command = out.joint_result.q_command;
      out.diagnostics.used_joint_servo_limit = true;
      out.diagnostics.limit_reason = CartesianServoLimitReason::kJointStepLimit;
    }

    out.q_command = q_command;
    out.achieved_pose = kinematics::fk(chain, out.q_command);
    const kinematics::PoseError6 final_error =
        evaluate_control_pose_error(target_pose, out.achieved_pose);
    out.position_error_norm = final_error.position_norm;
    out.orientation_error_norm = final_error.orientation_norm;
    out.position_reached =
        out.position_error_norm <= config_.position_tolerance;
    out.orientation_reached =
        out.orientation_error_norm <= config_.orientation_tolerance;
    out.reached = (out.position_error_norm <= config_.position_tolerance) &&
                  (out.orientation_error_norm <= config_.orientation_tolerance);
    out.valid = true;
    return out;
  }

  CartesianServoResult<N> step_delta(const SerialChain<N>& chain,
                                     const JointVec<N>& current_q,
                                     const Transform& delta_pose,
                                     CartesianTargetFrame target_frame,
                                     Scalar dt) const {
    return step(chain, current_q,
                CartesianTargetRequest::delta(delta_pose, target_frame), dt);
  }

 private:
  bool has_relaxed_position_axes() const {
    for (uint16_t i = 0; i < 3U; ++i) {
      if (abs_scalar(config_.position_axis_weights[i][0] - 1.0f) >
          ARM_LIB_EPSILON) {
        return true;
      }
    }
    return false;
  }

  kinematics::PoseError6 evaluate_control_pose_error(
      const Transform& target_pose, const Transform& current_pose) const {
    kinematics::TaskConstraintProfile profile;
    profile.position_weights = config_.position_axis_weights;
    profile.orientation_mode = config_.orientation_mode;
    profile.orientation_weight = 1.0f;
    return kinematics::evaluate_constrained_pose_error(
        target_pose, current_pose, config_.error_frame, profile);
  }

  Transform resolve_target_pose(const Transform& current_pose,
                                const CartesianTargetRequest& target) const {
    if (!target.is_delta) {
      return target.pose;
    }

    if (target.frame == CartesianTargetFrame::kBase) {
      return target.pose * current_pose;
    }

    return current_pose * target.pose;
  }

  CartesianServoConfig<N> config_;
  JointServo<N> joint_servo_;
};

}  // namespace control
}  // namespace arm_lib

#endif
