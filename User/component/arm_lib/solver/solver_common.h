#ifndef ARM_LIB_SOLVER_SOLVER_COMMON_H
#define ARM_LIB_SOLVER_SOLVER_COMMON_H

#include "../core/arm_common.h"
#include "../core/arm_types.h"

namespace arm_lib {
namespace solver {

template <int Rows, int Cols>
inline bool is_matrix_finite(const Matrixf<Rows, Cols>& matrix) {
  for (uint16_t row = 0; row < Rows; ++row) {
    for (uint16_t col = 0; col < Cols; ++col) {
      if (!is_finite_scalar(matrix[row][col])) {
        return false;
      }
    }
  }
  return true;
}

template <int N>
inline bool is_joint_vector_finite(const JointVec<N>& q) {
  for (uint16_t i = 0; i < N; ++i) {
    const Scalar value = q[i][0];
    if ((value != value) || (abs_scalar(value) > 1.0e30f)) {
      return false;
    }
  }
  return true;
}

}  // namespace solver
}  // namespace arm_lib

#endif
