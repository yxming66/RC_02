#ifndef ARM_LIB_SERVO_SERVO_L_CONTROLLER_H
#define ARM_LIB_SERVO_SERVO_L_CONTROLLER_H

#include "../core/arm_command.h"
#include "../core/arm_limits.h"
#include "../kinematics/fk.h"
#include "../kinematics/jacobian.h"
#include "../kinematics/pose_error.h"
#include "../model/serial_chain.h"
#include "weighted_dls.h"

namespace mr::robotics::arm {
namespace servo {

enum class ServoLStatus : uint8_t {
  kSuccess = 0,
  kReached,
  kInvalidInput,
  kDlsFailure,
};

template <int N>
struct ServoLConfig {
  WeightedDlsConfig<N> dls;
  Twist6 pose_gain;
  JointVec<N> max_velocity;
  JointVec<N> max_acceleration;
  JointLimits<N> position_limits;
  JointVec<N> home;
  Scalar nullspace_gain;
  Scalar max_joint_step;
  Scalar position_tolerance;
  Scalar orientation_tolerance;
  Scalar singularity_slowdown_threshold;
  Scalar singularity_fault_threshold;
  kinematics::PoseErrorFrame error_frame;
  bool enforce_position_limits;
  bool limit_joint_velocity;
  bool limit_joint_acceleration;
  bool limit_joint_step;
  bool fault_on_singularity;
  bool enable_singularity_velocity_scaling;
  bool enable_nullspace_home;
  bool stop_on_target;

  ServoLConfig()
      : dls(),
        pose_gain(matrixf::ones<6, 1>()),
        max_velocity(matrixf::zeros<N, 1>()),
        max_acceleration(matrixf::zeros<N, 1>()),
        position_limits(),
        home(matrixf::zeros<N, 1>()),
        nullspace_gain(0.0f),
        max_joint_step(0.0f),
        position_tolerance(1.0e-3f),
        orientation_tolerance(1.0e-3f),
        singularity_slowdown_threshold(0.0f),
        singularity_fault_threshold(0.0f),
        error_frame(kinematics::PoseErrorFrame::kSpatial),
        enforce_position_limits(false),
        limit_joint_velocity(false),
        limit_joint_acceleration(false),
        limit_joint_step(false),
        fault_on_singularity(false),
        enable_singularity_velocity_scaling(true),
        enable_nullspace_home(false),
        stop_on_target(false) {}
};

template <int N>
struct ServoLRequest {
  Transform target_pose;
  Twist6 twist_ff;
  ServoLConfig<N> config;
  bool use_pose_target;
  bool use_twist_ff;

  ServoLRequest()
      : target_pose(toolbox_adapter::identity_transform()),
        twist_ff(toolbox_adapter::zero_twist()),
        config(),
        use_pose_target(true),
        use_twist_ff(false) {}
};

template <int N>
struct ServoLResult {
  JointVec<N> q;
  JointVec<N> qd;
  JointVec<N> qdd;
  Transform current_pose;
  Twist6 pose_error;
  Twist6 commanded_twist;
  WeightedDlsResult<N> dls_result;
  ServoLStatus status;
  Scalar position_error_norm;
  Scalar orientation_error_norm;
  Scalar singularity_velocity_scale;
  bool ok;
  bool reached;
  bool singularity_velocity_scaled;
  bool velocity_limited;
  bool acceleration_limited;
  bool step_limited;
  bool position_limited;

  ServoLResult()
      : q(matrixf::zeros<N, 1>()),
        qd(matrixf::zeros<N, 1>()),
        qdd(matrixf::zeros<N, 1>()),
        current_pose(toolbox_adapter::identity_transform()),
        pose_error(toolbox_adapter::zero_twist()),
        commanded_twist(toolbox_adapter::zero_twist()),
        dls_result(),
        status(ServoLStatus::kInvalidInput),
        position_error_norm(0.0f),
        orientation_error_norm(0.0f),
        singularity_velocity_scale(1.0f),
        ok(false),
        reached(false),
        singularity_velocity_scaled(false),
        velocity_limited(false),
        acceleration_limited(false),
        step_limited(false),
        position_limited(false) {}
};

template <int N>
inline bool validate_servo_l_config(const ServoLConfig<N>& config) {
  if (!validate_weighted_dls_config(config.dls) ||
      !solver::is_matrix_finite(config.pose_gain) ||
      !solver::is_joint_vector_finite(config.max_velocity) ||
      !solver::is_joint_vector_finite(config.max_acceleration) ||
      !solver::is_joint_vector_finite(config.home) ||
      !is_finite_scalar(config.nullspace_gain) ||
      !is_finite_scalar(config.max_joint_step) ||
      !is_finite_scalar(config.position_tolerance) ||
      !is_finite_scalar(config.orientation_tolerance) ||
      !is_finite_scalar(config.singularity_slowdown_threshold) ||
      !is_finite_scalar(config.singularity_fault_threshold)) {
    return false;
  }
  if (config.nullspace_gain < 0.0f || config.max_joint_step < 0.0f ||
      config.position_tolerance < 0.0f ||
      config.orientation_tolerance < 0.0f ||
      config.singularity_slowdown_threshold < 0.0f ||
      config.singularity_fault_threshold < 0.0f) {
    return false;
  }
  for (uint16_t i = 0; i < 6U; ++i) {
    if (config.pose_gain[i][0] < 0.0f) {
      return false;
    }
  }
  return true;
}

template <int N>
inline bool validate_servo_l_request(const ServoLRequest<N>& request) {
  if (!validate_servo_l_config(request.config) ||
      !solver::is_matrix_finite(request.target_pose) ||
      !solver::is_matrix_finite(request.twist_ff)) {
    return false;
  }
  return request.use_pose_target || request.use_twist_ff;
}

template <int N>
inline JointVec<N> clamp_joint_velocity(
    const JointVec<N>& qd,
    const JointVec<N>& max_velocity,
    bool* limited) {
  JointVec<N> out = qd;
  if (limited != nullptr) {
    *limited = false;
  }
  for (uint16_t i = 0; i < N; ++i) {
    const Scalar limit = abs_scalar(max_velocity[i][0]);
    if (limit <= ARM_LIB_EPSILON) {
      continue;
    }
    if (out[i][0] > limit) {
      out[i][0] = limit;
      if (limited != nullptr) {
        *limited = true;
      }
    } else if (out[i][0] < -limit) {
      out[i][0] = -limit;
      if (limited != nullptr) {
        *limited = true;
      }
    }
  }
  return out;
}

template <int N>
inline JointVec<N> clamp_joint_acceleration(
    const JointVec<N>& qd,
    const JointVec<N>& previous_qd,
    const JointVec<N>& max_acceleration,
    Scalar dt,
    bool* limited) {
  JointVec<N> out = qd;
  if (limited != nullptr) {
    *limited = false;
  }
  if (dt <= ARM_LIB_EPSILON) {
    return out;
  }
  for (uint16_t i = 0; i < N; ++i) {
    const Scalar limit = abs_scalar(max_acceleration[i][0]);
    if (limit <= ARM_LIB_EPSILON) {
      continue;
    }
    const Scalar max_delta = limit * dt;
    const Scalar delta = out[i][0] - previous_qd[i][0];
    if (delta > max_delta) {
      out[i][0] = previous_qd[i][0] + max_delta;
      if (limited != nullptr) {
        *limited = true;
      }
    } else if (delta < -max_delta) {
      out[i][0] = previous_qd[i][0] - max_delta;
      if (limited != nullptr) {
        *limited = true;
      }
    }
  }
  return out;
}

template <int N>
inline JointVec<N> clamp_joint_step(const JointVec<N>& qd,
                                    Scalar max_joint_step,
                                    Scalar dt,
                                    bool* limited) {
  JointVec<N> out = qd;
  if (limited != nullptr) {
    *limited = false;
  }
  if (max_joint_step <= ARM_LIB_EPSILON || dt <= ARM_LIB_EPSILON) {
    return out;
  }
  const Scalar velocity_limit = max_joint_step / dt;
  for (uint16_t i = 0; i < N; ++i) {
    if (out[i][0] > velocity_limit) {
      out[i][0] = velocity_limit;
      if (limited != nullptr) {
        *limited = true;
      }
    } else if (out[i][0] < -velocity_limit) {
      out[i][0] = -velocity_limit;
      if (limited != nullptr) {
        *limited = true;
      }
    }
  }
  return out;
}

template <int N>
inline JointVec<N> clamp_to_position_limits_as_velocity(
    const JointVec<N>& q,
    const JointVec<N>& qd,
    const JointLimits<N>& limits,
    Scalar dt,
    bool* limited) {
  JointVec<N> out = qd;
  if (limited != nullptr) {
    *limited = false;
  }
  if (dt <= ARM_LIB_EPSILON) {
    return out;
  }
  for (uint16_t i = 0; i < N; ++i) {
    if (!limits.enabled[i]) {
      continue;
    }
    const Scalar q_next = q[i][0] + out[i][0] * dt;
    if (q_next < limits.lower[i][0]) {
      out[i][0] = (limits.lower[i][0] - q[i][0]) / dt;
      if (limited != nullptr) {
        *limited = true;
      }
    } else if (q_next > limits.upper[i][0]) {
      out[i][0] = (limits.upper[i][0] - q[i][0]) / dt;
      if (limited != nullptr) {
        *limited = true;
      }
    }
  }
  return out;
}

template <int N>
inline JointVec<N> servo_l_home_gradient(const JointVec<N>& q,
                                         const JointVec<N>& home,
                                         const JointLimits<N>& limits) {
  JointVec<N> gradient = matrixf::zeros<N, 1>();
  for (uint16_t i = 0; i < N; ++i) {
    Scalar delta = home[i][0] - q[i][0];
    if (limits.enabled[i]) {
      const Scalar range = limits.upper[i][0] - limits.lower[i][0];
      if (range > ARM_LIB_EPSILON) {
        delta /= range;
      }
    }
    gradient[i][0] = delta;
  }
  return gradient;
}

template <int N>
inline Scalar servo_l_singularity_velocity_scale(
    Scalar singularity_metric,
    const ServoLConfig<N>& config) {
  if (!config.enable_singularity_velocity_scaling ||
      !is_finite_scalar(singularity_metric)) {
    return 1.0f;
  }

  Scalar slowdown_threshold = config.singularity_slowdown_threshold;
  if (slowdown_threshold <= ARM_LIB_EPSILON) {
    slowdown_threshold = config.dls.singularity_threshold;
  }
  if (slowdown_threshold <= ARM_LIB_EPSILON) {
    return 1.0f;
  }

  Scalar stop_threshold = config.singularity_fault_threshold;
  if (stop_threshold < 0.0f) {
    stop_threshold = 0.0f;
  }
  if (stop_threshold >= slowdown_threshold) {
    stop_threshold = 0.0f;
  }

  if (singularity_metric >= slowdown_threshold) {
    return 1.0f;
  }
  if (singularity_metric <= stop_threshold) {
    return 0.0f;
  }

  const Scalar span = slowdown_threshold - stop_threshold;
  if (span <= ARM_LIB_EPSILON) {
    return 0.0f;
  }
  return clamp_scalar((singularity_metric - stop_threshold) / span,
                      0.0f, 1.0f);
}

template <int N>
class ServoLController {
 public:
  ServoLController()
      : request_(),
        previous_qd_(matrixf::zeros<N, 1>()),
        has_previous_qd_(false) {}

  const ServoLRequest<N>& request() const { return request_; }

  bool configure(const ServoLRequest<N>& request) {
    if (!validate_servo_l_request(request)) {
      return false;
    }
    request_ = request;
    has_previous_qd_ = false;
    previous_qd_ = matrixf::zeros<N, 1>();
    return true;
  }

  void reset() {
    has_previous_qd_ = false;
    previous_qd_ = matrixf::zeros<N, 1>();
  }

  void set_target_pose(const Transform& target_pose) {
    request_.target_pose = target_pose;
    request_.use_pose_target = true;
  }

  void set_twist_ff(const Twist6& twist_ff) {
    request_.twist_ff = twist_ff;
    request_.use_twist_ff = true;
  }

  ServoLResult<N> step(const SerialChain<N>& chain,
                       const JointState<N>& state,
                       Scalar dt) {
    ServoLResult<N> result;
    if (dt <= ARM_LIB_EPSILON || !validate_servo_l_request(request_) ||
        !solver::is_joint_vector_finite(state.q) ||
        !solver::is_joint_vector_finite(state.qd) ||
        !state.valid) {
      result.status = ServoLStatus::kInvalidInput;
      return result;
    }

    result.current_pose = kinematics::fk(chain, state.q);
    result.commanded_twist = toolbox_adapter::zero_twist();

    if (request_.use_pose_target) {
      const kinematics::PoseError6 error = kinematics::evaluate_pose_error(
          request_.target_pose, result.current_pose,
          request_.config.error_frame);
      result.pose_error = error.twist;
      result.position_error_norm = error.position_norm;
      result.orientation_error_norm = error.orientation_norm;
      for (uint16_t i = 0; i < 6U; ++i) {
        result.commanded_twist[i][0] +=
            request_.config.pose_gain[i][0] * result.pose_error[i][0];
      }
    }

    if (request_.use_twist_ff) {
      result.commanded_twist += request_.twist_ff;
    }

    const Jacobian6xN<N> jacobian = kinematics::jacobian(chain, state.q);
    result.singularity_velocity_scale =
        servo_l_singularity_velocity_scale(
            kinematics::singularity_metric(jacobian), request_.config);
    if (result.singularity_velocity_scale < 1.0f - ARM_LIB_EPSILON) {
      result.commanded_twist *= result.singularity_velocity_scale;
      result.singularity_velocity_scaled = true;
    }

    result.dls_result =
        solve_weighted_dls(jacobian, result.commanded_twist,
                           request_.config.dls);
    if (!result.dls_result.ok) {
      result.status = ServoLStatus::kDlsFailure;
      return result;
    }

    JointVec<N> qd = result.dls_result.qdot;
    if (request_.config.enable_nullspace_home &&
        request_.config.nullspace_gain > ARM_LIB_EPSILON) {
      JointDiag<N> projector = matrixf::eye<N, N>();
      if (weighted_dls_nullspace_projector_safe(
              jacobian, request_.config.dls,
              result.dls_result.damping_used, &projector)) {
        const JointVec<N> gradient = servo_l_home_gradient(
            state.q, request_.config.home, request_.config.position_limits);
        qd += projector * (gradient * request_.config.nullspace_gain);
      }
    }

    if (request_.config.limit_joint_velocity) {
      bool limited = false;
      qd = clamp_joint_velocity(qd, request_.config.max_velocity, &limited);
      result.velocity_limited = result.velocity_limited || limited;
    }

    const JointVec<N> acceleration_reference =
        has_previous_qd_ ? previous_qd_ : state.qd;
    if (request_.config.limit_joint_acceleration) {
      bool limited = false;
      qd = clamp_joint_acceleration(qd, acceleration_reference,
                                    request_.config.max_acceleration, dt,
                                    &limited);
      result.acceleration_limited = result.acceleration_limited || limited;
    }

    if (request_.config.limit_joint_step) {
      bool limited = false;
      qd = clamp_joint_step(qd, request_.config.max_joint_step, dt, &limited);
      result.step_limited = result.step_limited || limited;
    }

    if (request_.config.enforce_position_limits) {
      bool limited = false;
      qd = clamp_to_position_limits_as_velocity(
          state.q, qd, request_.config.position_limits, dt, &limited);
      result.position_limited = result.position_limited || limited;
    }

    result.q = state.q + qd * dt;
    result.qd = qd;
    result.qdd = (qd - acceleration_reference) / dt;
    result.ok = solver::is_joint_vector_finite(result.q) &&
                solver::is_joint_vector_finite(result.qd) &&
                solver::is_joint_vector_finite(result.qdd);
    if (!result.ok) {
      result.status = ServoLStatus::kInvalidInput;
      return result;
    }

    result.reached =
        request_.use_pose_target && request_.config.stop_on_target &&
        result.position_error_norm <= request_.config.position_tolerance &&
        result.orientation_error_norm <= request_.config.orientation_tolerance &&
        result.qd.norm() <= request_.config.position_tolerance;
    result.status = result.reached ? ServoLStatus::kReached
                                   : ServoLStatus::kSuccess;
    previous_qd_ = result.qd;
    has_previous_qd_ = true;
    return result;
  }

 private:
  ServoLRequest<N> request_;
  JointVec<N> previous_qd_;
  bool has_previous_qd_;
};

}  // namespace servo
}  // namespace mr::robotics::arm

#endif
