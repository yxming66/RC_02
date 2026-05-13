#ifndef ARM_LIB_SERVO_WEIGHTED_DLS_H
#define ARM_LIB_SERVO_WEIGHTED_DLS_H

#include "../core/arm_common.h"
#include "../core/arm_types.h"
#include "../kinematics/ik_redundancy.h"
#include "../math/linear_solve.h"
#include "../solver/solver_common.h"

namespace mr::robotics::arm {
namespace servo {

enum class WeightedDlsStatus : uint8_t {
  kSuccess = 0,
  kInvalidInput,
  kLinearSolveFailure,
};

template <int N>
struct WeightedDlsConfig {
  Twist6 task_weights;
  JointVec<N> joint_weights;
  Scalar damping;
  Scalar damping_min;
  Scalar damping_max;
  Scalar singularity_threshold;
  Scalar singularity_damping_scale;
  Scalar min_pivot;
  Scalar max_qdot_norm;
  bool enable_singularity_adaptive_damping;
  bool limit_output_norm;

  WeightedDlsConfig()
      : task_weights(matrixf::ones<6, 1>()),
        joint_weights(matrixf::ones<N, 1>()),
        damping(1.0e-3f),
        damping_min(1.0e-6f),
        damping_max(1.0e3f),
        singularity_threshold(1.0e-3f),
        singularity_damping_scale(10.0f),
        min_pivot(1.0e-8f),
        max_qdot_norm(0.0f),
        enable_singularity_adaptive_damping(true),
        limit_output_norm(false) {}
};

template <int N>
struct WeightedDlsResult {
  JointVec<N> qdot;
  Scalar damping_used;
  Scalar singularity_metric;
  Scalar output_norm;
  WeightedDlsStatus status;
  bool ok;
  bool output_limited;

  WeightedDlsResult()
      : qdot(matrixf::zeros<N, 1>()),
        damping_used(0.0f),
        singularity_metric(0.0f),
        output_norm(0.0f),
        status(WeightedDlsStatus::kInvalidInput),
        ok(false),
        output_limited(false) {}
};

template <int N>
inline bool validate_weighted_dls_config(const WeightedDlsConfig<N>& config) {
  if (!solver::is_matrix_finite(config.task_weights) ||
      !solver::is_joint_vector_finite(config.joint_weights) ||
      !is_finite_scalar(config.damping) ||
      !is_finite_scalar(config.damping_min) ||
      !is_finite_scalar(config.damping_max) ||
      !is_finite_scalar(config.singularity_threshold) ||
      !is_finite_scalar(config.singularity_damping_scale) ||
      !is_finite_scalar(config.min_pivot) ||
      !is_finite_scalar(config.max_qdot_norm)) {
    return false;
  }
  if (config.damping < 0.0f || config.damping_min < 0.0f ||
      config.damping_max < config.damping_min || config.min_pivot <= 0.0f ||
      config.singularity_threshold < 0.0f ||
      config.singularity_damping_scale < 1.0f ||
      config.max_qdot_norm < 0.0f) {
    return false;
  }
  for (uint16_t i = 0; i < 6U; ++i) {
    if (config.task_weights[i][0] < 0.0f) {
      return false;
    }
  }
  for (uint16_t i = 0; i < N; ++i) {
    if (config.joint_weights[i][0] <= 0.0f) {
      return false;
    }
  }
  return true;
}

template <int N>
inline Scalar weighted_dls_effective_damping(
    Scalar singularity_metric,
    const WeightedDlsConfig<N>& config) {
  Scalar damping = config.damping;
  if (config.enable_singularity_adaptive_damping &&
      config.singularity_threshold > ARM_LIB_EPSILON &&
      singularity_metric < config.singularity_threshold) {
    const Scalar ratio =
        clamp_scalar((config.singularity_threshold - singularity_metric) /
                         config.singularity_threshold,
                     0.0f, 1.0f);
    const Scalar boost =
        1.0f + ratio * (config.singularity_damping_scale - 1.0f);
    damping *= boost;
  }
  return clamp_scalar(damping, config.damping_min, config.damping_max);
}

template <int N>
inline WeightedDlsResult<N> solve_weighted_dls(
    const Jacobian6xN<N>& jacobian,
    const Twist6& twist,
    const WeightedDlsConfig<N>& config = WeightedDlsConfig<N>()) {
  WeightedDlsResult<N> result;
  if (!solver::is_matrix_finite(jacobian) ||
      !solver::is_matrix_finite(twist) ||
      !validate_weighted_dls_config(config)) {
    result.status = WeightedDlsStatus::kInvalidInput;
    return result;
  }

  result.singularity_metric = kinematics::singularity_metric(jacobian);
  if (!is_finite_scalar(result.singularity_metric)) {
    result.status = WeightedDlsStatus::kInvalidInput;
    return result;
  }
  result.damping_used =
      weighted_dls_effective_damping(result.singularity_metric, config);

  Matrixf<N, N> normal = matrixf::zeros<N, N>();
  JointVec<N> rhs = matrixf::zeros<N, 1>();

  for (uint16_t task = 0; task < 6U; ++task) {
    const Scalar wx = config.task_weights[task][0];
    for (uint16_t col = 0; col < N; ++col) {
      const Scalar weighted_j = jacobian[task][col] * wx;
      rhs[col][0] += weighted_j * twist[task][0];
      for (uint16_t row = 0; row < N; ++row) {
        normal[row][col] += jacobian[task][row] * weighted_j;
      }
    }
  }

  const Scalar damping_squared = result.damping_used * result.damping_used;
  for (uint16_t i = 0; i < N; ++i) {
    normal[i][i] += damping_squared * config.joint_weights[i][0];
  }

  JointVec<N> qdot = matrixf::zeros<N, 1>();
  if (!math::solve_symmetric_ldlt(normal, rhs, &qdot, config.min_pivot)) {
    result.status = WeightedDlsStatus::kLinearSolveFailure;
    return result;
  }
  if (!solver::is_joint_vector_finite(qdot)) {
    result.status = WeightedDlsStatus::kLinearSolveFailure;
    return result;
  }

  result.output_norm = qdot.norm();
  if (!is_finite_scalar(result.output_norm)) {
    result.status = WeightedDlsStatus::kLinearSolveFailure;
    return result;
  }
  if (config.limit_output_norm &&
      config.max_qdot_norm > ARM_LIB_EPSILON &&
      result.output_norm > config.max_qdot_norm) {
    qdot *= config.max_qdot_norm / result.output_norm;
    result.output_norm = config.max_qdot_norm;
    result.output_limited = true;
  }

  result.qdot = qdot;
  result.status = WeightedDlsStatus::kSuccess;
  result.ok = true;
  return result;
}

template <int N>
inline bool weighted_dls_nullspace_projector_safe(
    const Jacobian6xN<N>& jacobian,
    const WeightedDlsConfig<N>& config,
    Scalar damping,
    JointDiag<N>* out_projector) {
  if (out_projector == nullptr) {
    return false;
  }
  if (!solver::is_matrix_finite(jacobian) ||
      !validate_weighted_dls_config(config) ||
      !is_finite_scalar(damping) ||
      damping < 0.0f) {
    *out_projector = matrixf::eye<N, N>();
    return false;
  }

  // P = I - A^-1 B, with B = J^T Wx J and A = B + lambda^2 Wq.
  Matrixf<N, N> task_normal = matrixf::zeros<N, N>();
  for (uint16_t task = 0; task < 6U; ++task) {
    const Scalar wx = config.task_weights[task][0];
    for (uint16_t col = 0; col < N; ++col) {
      const Scalar weighted_j = jacobian[task][col] * wx;
      for (uint16_t row = 0; row < N; ++row) {
        task_normal[row][col] += jacobian[task][row] * weighted_j;
      }
    }
  }

  Matrixf<N, N> regularized = task_normal;
  const Scalar damping_squared = damping * damping;
  for (uint16_t i = 0; i < N; ++i) {
    regularized[i][i] += damping_squared * config.joint_weights[i][0];
  }

  Matrixf<N, N> resolved_primary = matrixf::zeros<N, N>();
  if (!math::solve_symmetric_ldlt(regularized, task_normal,
                                  &resolved_primary, config.min_pivot)) {
    *out_projector = matrixf::eye<N, N>();
    return false;
  }

  *out_projector = matrixf::eye<N, N>() - resolved_primary;
  if (!solver::is_matrix_finite(*out_projector)) {
    *out_projector = matrixf::eye<N, N>();
    return false;
  }
  return true;
}

template <int N>
class WeightedDlsSolver {
 public:
  WeightedDlsSolver() : config_() {}
  explicit WeightedDlsSolver(const WeightedDlsConfig<N>& config)
      : config_(config) {}

  const WeightedDlsConfig<N>& config() const { return config_; }
  void set_config(const WeightedDlsConfig<N>& config) { config_ = config; }

  WeightedDlsResult<N> solve(const Jacobian6xN<N>& jacobian,
                             const Twist6& twist) const {
    return solve_weighted_dls(jacobian, twist, config_);
  }

  bool solve(const Jacobian6xN<N>& jacobian,
             const Twist6& twist,
             JointVec<N>* out_qdot,
             WeightedDlsResult<N>* out_result = nullptr) const {
    if (out_qdot == nullptr) {
      return false;
    }
    const WeightedDlsResult<N> result = solve(jacobian, twist);
    if (out_result != nullptr) {
      *out_result = result;
    }
    *out_qdot = result.qdot;
    return result.ok;
  }

 private:
  WeightedDlsConfig<N> config_;
};

}  // namespace servo
}  // namespace mr::robotics::arm

#endif
