#ifndef ARM_LIB_CONTROL_JOINT_SERVO_H
#define ARM_LIB_CONTROL_JOINT_SERVO_H

#include "../model/serial_chain.h"
#include "../solver/seed_policy.h"

namespace mr::robotics::arm {
namespace control {

template <int N>
struct JointServoConfig {
  JointVec<N> max_velocity;
  Scalar position_tolerance;
  bool clamp_to_limits;

  JointServoConfig()
      : max_velocity(toolbox_adapter::zero_joint_vec<N>()),
        position_tolerance(1.0e-3f),
        clamp_to_limits(true) {
    for (uint16_t i = 0; i < N; ++i) {
      max_velocity[i][0] = 1.0f;
    }
  }
};

template <int N>
struct JointServoResult {
  JointVec<N> q_command;
  JointVec<N> velocity_command;
  Scalar error_norm;
  bool reached;
  bool valid;

  JointServoResult()
      : q_command(toolbox_adapter::zero_joint_vec<N>()),
        velocity_command(toolbox_adapter::zero_joint_vec<N>()),
        error_norm(0.0f),
        reached(false),
        valid(false) {}
};

template <int N>
class JointServo {
 public:
  JointServo() : config_() {}

  explicit JointServo(const JointServoConfig<N>& config) : config_(config) {}

  const JointServoConfig<N>& config() const { return config_; }
  void set_config(const JointServoConfig<N>& config) { config_ = config; }

  JointServoResult<N> step(const SerialChain<N>& chain,
                           const JointVec<N>& current,
                           const JointVec<N>& target,
                           Scalar dt) const {
    JointServoResult<N> out;
    if (dt <= ARM_LIB_EPSILON) {
      return out;
    }

    out.valid = true;
    JointVec<N> q_next = current;
    JointVec<N> qdot = toolbox_adapter::zero_joint_vec<N>();
    Scalar error_sq = 0.0f;

    for (uint16_t i = 0; i < N; ++i) {
      const Link& link = chain.link(i);
      Scalar error = target[i][0] - current[i][0];
      if (link.joint_type() == ChainJointType::kRevolute) {
        const bool allow_wrap =
            !link.limit_enabled() ||
            ((link.upper_limit() - link.lower_limit()) >=
             (2.0f * ARM_LIB_PI - 1.0e-3f));
        if (allow_wrap) {
          error = angle_distance(current[i][0], target[i][0]);
        }
      }

      const Scalar max_velocity = abs_scalar(config_.max_velocity[i][0]);
      if (max_velocity <= ARM_LIB_EPSILON) {
        if (abs_scalar(error) > config_.position_tolerance) {
          out.valid = false;
          return out;
        }
        q_next[i][0] = current[i][0];
        qdot[i][0] = 0.0f;
        continue;
      }

      const Scalar max_step = max_velocity * dt;
      if (abs_scalar(error) <= max_step) {
        q_next[i][0] = current[i][0] + error;
        qdot[i][0] = error / dt;
      } else {
        const Scalar direction = (error >= 0.0f) ? 1.0f : -1.0f;
        q_next[i][0] = current[i][0] + direction * max_step;
        qdot[i][0] = direction * max_velocity;
      }
      error_sq += error * error;
    }

    if (config_.clamp_to_limits) {
      q_next = solver::clamp_to_chain_limits(chain, q_next);
    }

    out.q_command = q_next;
    out.velocity_command = qdot;
    out.error_norm = sqrtf(error_sq);
    out.reached =
        (solver::configuration_distance(chain, q_next, target) <=
         config_.position_tolerance);
    return out;
  }

 private:
  JointServoConfig<N> config_;
};

}  // namespace control
}  // namespace mr::robotics::arm

#endif
