#ifndef ARM_LIB_KINEMATICS_IK_REDUNDANCY_H
#define ARM_LIB_KINEMATICS_IK_REDUNDANCY_H

#include "jacobian.h"
#include "../solver/lm_solver.h"

namespace arm_lib {
namespace kinematics {

template <int M>
inline Scalar symmetric_smallest_eigenvalue(Matrixf<M, M> mat,
                                            uint16_t max_sweeps = 16U) {
  if constexpr (M == 1) {
    return mat[0][0];
  }

  for (uint16_t sweep = 0; sweep < max_sweeps; ++sweep) {
    Scalar max_off_diag = 0.0f;
    uint16_t p = 0U;
    uint16_t q = 1U;

    for (uint16_t row = 0; row < M; ++row) {
      for (uint16_t col = row + 1U; col < M; ++col) {
        const Scalar value = abs_scalar(mat[row][col]);
        if (value > max_off_diag) {
          max_off_diag = value;
          p = row;
          q = col;
        }
      }
    }

    if (max_off_diag <= 1.0e-6f) {
      break;
    }

    const Scalar app = mat[p][p];
    const Scalar aqq = mat[q][q];
    const Scalar apq = mat[p][q];
    const Scalar phi = 0.5f * atan2f(2.0f * apq, aqq - app);
    const Scalar c = cosf(phi);
    const Scalar s = sinf(phi);

    for (uint16_t k = 0; k < M; ++k) {
      if (k == p || k == q) {
        continue;
      }
      const Scalar aik = mat[k][p];
      const Scalar akq = mat[k][q];
      const Scalar new_kp = c * aik - s * akq;
      const Scalar new_kq = s * aik + c * akq;
      mat[k][p] = new_kp;
      mat[p][k] = new_kp;
      mat[k][q] = new_kq;
      mat[q][k] = new_kq;
    }

    mat[p][p] = c * c * app - 2.0f * s * c * apq + s * s * aqq;
    mat[q][q] = s * s * app + 2.0f * s * c * apq + c * c * aqq;
    mat[p][q] = 0.0f;
    mat[q][p] = 0.0f;
  }

  Scalar min_value = mat[0][0];
  for (uint16_t i = 1U; i < M; ++i) {
    if (mat[i][i] < min_value) {
      min_value = mat[i][i];
    }
  }
  return min_value;
}

template <int M>
inline Scalar determinant(Matrixf<M, M> mat) {
  Scalar det = 1.0f;
  int sign = 1;

  for (uint16_t pivot = 0; pivot < M; ++pivot) {
    uint16_t max_row = pivot;
    Scalar max_abs = abs_scalar(mat[pivot][pivot]);
    for (uint16_t row = pivot + 1U; row < M; ++row) {
      const Scalar value = abs_scalar(mat[row][pivot]);
      if (value > max_abs) {
        max_abs = value;
        max_row = row;
      }
    }

    if (max_abs <= 1.0e-8f) {
      return 0.0f;
    }

    if (max_row != pivot) {
      for (uint16_t col = 0; col < M; ++col) {
        const Scalar temp = mat[pivot][col];
        mat[pivot][col] = mat[max_row][col];
        mat[max_row][col] = temp;
      }
      sign = -sign;
    }

    const Scalar pivot_value = mat[pivot][pivot];
    det *= pivot_value;
    for (uint16_t row = pivot + 1U; row < M; ++row) {
      const Scalar factor = mat[row][pivot] / pivot_value;
      for (uint16_t col = pivot + 1U; col < M; ++col) {
        mat[row][col] -= factor * mat[pivot][col];
      }
    }
  }

  return (sign > 0) ? det : -det;
}

template <int N>
inline Scalar singularity_metric(const Jacobian6xN<N>& jacobian) {
  Scalar min_eigen = 0.0f;
  if constexpr (N <= 6) {
    min_eigen = symmetric_smallest_eigenvalue(jacobian.trans() * jacobian);
  } else {
    min_eigen = symmetric_smallest_eigenvalue(jacobian * jacobian.trans());
  }
  return sqrtf((min_eigen > 0.0f) ? min_eigen : 0.0f);
}

template <int N>
inline Scalar manipulability_metric(const Jacobian6xN<N>& jacobian) {
  Scalar det = 0.0f;
  if constexpr (N <= 6) {
    det = determinant(jacobian.trans() * jacobian);
  } else {
    det = determinant(jacobian * jacobian.trans());
  }
  return sqrtf((det > 0.0f) ? det : 0.0f);
}

template <int N>
inline Matrixf<N, 6> damped_task_pseudoinverse(
    const Jacobian6xN<N>& jacobian, Scalar damping) {
  const Matrixf<6, 6> task_identity = matrixf::eye<6, 6>();
  const Matrixf<6, 6> task_matrix =
      jacobian * jacobian.trans() + damping * task_identity;
  return jacobian.trans() * matrixf::inv(task_matrix);
}

template <int N>
inline bool damped_task_pseudoinverse_safe(
    const Jacobian6xN<N>& jacobian, Scalar damping,
    Matrixf<N, 6>* out_pseudoinverse) {
  if (out_pseudoinverse == nullptr) {
    return false;
  }

  const Matrixf<6, 6> task_identity = matrixf::eye<6, 6>();
  const Matrixf<6, 6> task_matrix =
      jacobian * jacobian.trans() + damping * task_identity;
  const Matrixf<6, 6> task_inverse = matrixf::inv(task_matrix);
  if (!solver::is_matrix_finite(task_inverse)) {
    *out_pseudoinverse = matrixf::zeros<N, 6>();
    return false;
  }

  *out_pseudoinverse = jacobian.trans() * task_inverse;
  for (uint16_t row = 0; row < N; ++row) {
    for (uint16_t col = 0; col < 6U; ++col) {
      if (!is_finite_scalar((*out_pseudoinverse)[row][col])) {
        *out_pseudoinverse = matrixf::zeros<N, 6>();
        return false;
      }
    }
  }
  return true;
}

template <int N>
inline JointDiag<N> damped_null_space_projector(
    const Jacobian6xN<N>& jacobian, Scalar damping) {
  const JointDiag<N> identity = matrixf::eye<N, N>();
  const Matrixf<N, 6> pseudoinverse =
      damped_task_pseudoinverse(jacobian, damping);
  return identity - pseudoinverse * jacobian;
}

template <int N>
inline bool damped_null_space_projector_safe(
    const Jacobian6xN<N>& jacobian, Scalar damping,
    JointDiag<N>* out_projector) {
  if (out_projector == nullptr) {
    return false;
  }

  const JointDiag<N> identity = matrixf::eye<N, N>();
  Matrixf<N, 6> pseudoinverse = matrixf::zeros<N, 6>();
  if (!damped_task_pseudoinverse_safe(jacobian, damping, &pseudoinverse)) {
    *out_projector = identity;
    return false;
  }

  *out_projector = identity - pseudoinverse * jacobian;
  return solver::is_matrix_finite(*out_projector);
}

}  // namespace kinematics
}  // namespace arm_lib

#endif
