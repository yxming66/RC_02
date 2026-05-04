#ifndef ARM_LIB_CORE_ARM_COMMAND_H
#define ARM_LIB_CORE_ARM_COMMAND_H

#include "arm_common.h"
#include "arm_types.h"

namespace mr::robotics::arm {

enum class JointCommandMode : uint8_t {
  kDisabled = 0,
  kPosition,
  kPositionVelocityTorque,
  kTorque,
};

struct JointSetpoint {
  Scalar position;
  Scalar velocity;
  Scalar acceleration;
  Scalar torque_ff;
  Scalar kp;
  Scalar kd;
  bool has_position;
  bool has_velocity;
  bool has_acceleration;
  bool has_torque_ff;

  JointSetpoint()
      : position(0.0f),
        velocity(0.0f),
        acceleration(0.0f),
        torque_ff(0.0f),
        kp(0.0f),
        kd(0.0f),
        has_position(false),
        has_velocity(false),
        has_acceleration(false),
        has_torque_ff(false) {}

  static JointSetpoint position_torque(Scalar q, Scalar qd, Scalar tau,
                                       Scalar kp_value, Scalar kd_value) {
    JointSetpoint setpoint;
    setpoint.position = q;
    setpoint.velocity = qd;
    setpoint.torque_ff = tau;
    setpoint.kp = kp_value;
    setpoint.kd = kd_value;
    setpoint.has_position = true;
    setpoint.has_velocity = true;
    setpoint.has_torque_ff = true;
    return setpoint;
  }
};

template <int N>
struct JointState {
  JointVec<N> q;
  JointVec<N> qd;
  JointVec<N> torque;
  bool online[N];
  bool valid;

  JointState()
      : q(matrixf::zeros<N, 1>()),
        qd(matrixf::zeros<N, 1>()),
        torque(matrixf::zeros<N, 1>()),
        online{false},
        valid(false) {}
};

template <int N>
struct JointCommand {
  JointVec<N> q;
  JointVec<N> qd;
  JointVec<N> qdd;
  JointVec<N> torque_ff;
  JointVec<N> kp;
  JointVec<N> kd;
  JointCommandMode mode;
  bool valid;

  JointCommand()
      : q(matrixf::zeros<N, 1>()),
        qd(matrixf::zeros<N, 1>()),
        qdd(matrixf::zeros<N, 1>()),
        torque_ff(matrixf::zeros<N, 1>()),
        kp(matrixf::zeros<N, 1>()),
        kd(matrixf::zeros<N, 1>()),
        mode(JointCommandMode::kDisabled),
        valid(false) {}
};

template <int N>
inline JointCommand<N> make_disabled_joint_command() {
  return JointCommand<N>();
}

template <int N>
inline JointCommand<N> make_hold_joint_command(const JointState<N>& state) {
  JointCommand<N> command;
  command.q = state.q;
  command.qd = matrixf::zeros<N, 1>();
  command.qdd = matrixf::zeros<N, 1>();
  command.mode = JointCommandMode::kPositionVelocityTorque;
  command.valid = state.valid;
  return command;
}

template <int N>
inline bool joint_command_is_finite(const JointCommand<N>& command) {
  if (!command.valid) {
    return false;
  }
  for (uint16_t i = 0; i < N; ++i) {
    if (!is_finite_scalar(command.q[i][0]) ||
        !is_finite_scalar(command.qd[i][0]) ||
        !is_finite_scalar(command.qdd[i][0]) ||
        !is_finite_scalar(command.torque_ff[i][0]) ||
        !is_finite_scalar(command.kp[i][0]) ||
        !is_finite_scalar(command.kd[i][0])) {
      return false;
    }
  }
  return true;
}

}  // namespace mr::robotics::arm

#endif
