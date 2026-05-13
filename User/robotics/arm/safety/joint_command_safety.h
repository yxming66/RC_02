#ifndef ARM_LIB_SAFETY_JOINT_COMMAND_SAFETY_H
#define ARM_LIB_SAFETY_JOINT_COMMAND_SAFETY_H

#include "../core/arm_command.h"
#include "../core/arm_limits.h"

namespace mr::robotics::arm {
namespace safety {

enum class SafetyLimitReason : uint8_t {
  kNone = 0,
  kInvalidCommand,
  kJointPosition,
  kJointVelocity,
  kJointAcceleration,
  kTorqueFeedforward,
  kJointStep,
  kSingularity,
};

template <int N>
struct JointCommandSafetyConfig {
  JointLimits<N> position_limits;
  JointVec<N> max_velocity;
  JointVec<N> max_acceleration;
  JointVec<N> max_torque_ff;
  JointVec<N> current_position;
  JointVec<N> max_position_step;
  Scalar singularity_metric;
  Scalar singularity_threshold;
  bool check_position;
  bool check_velocity;
  bool check_acceleration;
  bool check_torque_ff;
  bool check_joint_step;
  bool check_singularity;

  JointCommandSafetyConfig()
      : position_limits(),
        max_velocity(matrixf::zeros<N, 1>()),
        max_acceleration(matrixf::zeros<N, 1>()),
        max_torque_ff(matrixf::zeros<N, 1>()),
        current_position(matrixf::zeros<N, 1>()),
        max_position_step(matrixf::zeros<N, 1>()),
        singularity_metric(0.0f),
        singularity_threshold(0.0f),
        check_position(false),
        check_velocity(false),
        check_acceleration(false),
        check_torque_ff(false),
        check_joint_step(false),
        check_singularity(false) {}
};

struct JointCommandSafetyResult {
  SafetyLimitReason reason;
  Index joint_index;
  Scalar value;
  Scalar limit;
  bool ok;

  JointCommandSafetyResult()
      : reason(SafetyLimitReason::kNone),
        joint_index(0U),
        value(0.0f),
        limit(0.0f),
        ok(true) {}
};

template <int N>
inline JointCommandSafetyResult make_safety_failure(
    SafetyLimitReason reason,
    Index joint_index,
    Scalar value,
    Scalar limit) {
  JointCommandSafetyResult result;
  result.reason = reason;
  result.joint_index = joint_index;
  result.value = value;
  result.limit = limit;
  result.ok = false;
  return result;
}

template <int N>
inline JointCommandSafetyResult validate_joint_command(
    const JointCommand<N>& command,
    const JointCommandSafetyConfig<N>& config,
    Scalar position_margin = ARM_LIB_EPSILON) {
  if (!joint_command_is_finite(command)) {
    return make_safety_failure<N>(SafetyLimitReason::kInvalidCommand, 0U,
                                  0.0f, 0.0f);
  }

  const bool position_mode =
      command.mode == JointCommandMode::kPosition ||
      command.mode == JointCommandMode::kPositionVelocityTorque;

  if (config.check_singularity) {
    if (!is_finite_scalar(config.singularity_metric) ||
        !is_finite_scalar(config.singularity_threshold)) {
      return make_safety_failure<N>(SafetyLimitReason::kInvalidCommand, 0U,
                                    config.singularity_metric,
                                    config.singularity_threshold);
    }
    if (config.singularity_threshold > ARM_LIB_EPSILON &&
        config.singularity_metric < config.singularity_threshold) {
      return make_safety_failure<N>(SafetyLimitReason::kSingularity, 0U,
                                    config.singularity_metric,
                                    config.singularity_threshold);
    }
  }

  for (uint16_t i = 0; i < N; ++i) {
    if (position_mode && config.check_position &&
        config.position_limits.enabled[i]) {
      const Scalar q = command.q[i][0];
      if (q < config.position_limits.lower[i][0] - position_margin) {
        return make_safety_failure<N>(
            SafetyLimitReason::kJointPosition, i, q,
            config.position_limits.lower[i][0]);
      }
      if (q > config.position_limits.upper[i][0] + position_margin) {
        return make_safety_failure<N>(
            SafetyLimitReason::kJointPosition, i, q,
            config.position_limits.upper[i][0]);
      }
    }

    if (position_mode && config.check_velocity) {
      const Scalar limit = abs_scalar(config.max_velocity[i][0]);
      if (limit > ARM_LIB_EPSILON &&
          abs_scalar(command.qd[i][0]) > limit + position_margin) {
        return make_safety_failure<N>(SafetyLimitReason::kJointVelocity, i,
                                      command.qd[i][0], limit);
      }
    }

    if (position_mode && config.check_acceleration) {
      const Scalar limit = abs_scalar(config.max_acceleration[i][0]);
      if (limit > ARM_LIB_EPSILON &&
          abs_scalar(command.qdd[i][0]) > limit + position_margin) {
        return make_safety_failure<N>(SafetyLimitReason::kJointAcceleration, i,
                                      command.qdd[i][0], limit);
      }
    }

    if (position_mode && config.check_joint_step) {
      const Scalar limit = abs_scalar(config.max_position_step[i][0]);
      if (limit > ARM_LIB_EPSILON) {
        const Scalar step = command.q[i][0] - config.current_position[i][0];
        if (abs_scalar(step) > limit + position_margin) {
          return make_safety_failure<N>(SafetyLimitReason::kJointStep, i,
                                        step, limit);
        }
      }
    }

    if (config.check_torque_ff) {
      const Scalar limit = abs_scalar(config.max_torque_ff[i][0]);
      if (limit > ARM_LIB_EPSILON &&
          abs_scalar(command.torque_ff[i][0]) > limit + position_margin) {
        return make_safety_failure<N>(SafetyLimitReason::kTorqueFeedforward, i,
                                      command.torque_ff[i][0], limit);
      }
    }
  }

  return JointCommandSafetyResult();
}

}  // namespace safety
}  // namespace mr::robotics::arm

#endif
