#ifndef ARM_LIB_SOLVER_SOLVER_COMMON_H
#define ARM_LIB_SOLVER_SOLVER_COMMON_H

#include "component/math/linear_solve.hpp"

#include "../core/arm_common.h"
#include "../core/arm_types.h"

namespace mr::robotics::arm {
namespace solver {

template <int Rows, int Cols>
inline bool is_matrix_finite(const Matrixf<Rows, Cols>& matrix) {
  return ::mr::component::math::is_matrix_finite(matrix);
}

template <int N>
inline bool is_joint_vector_finite(const JointVec<N>& q) {
  return ::mr::component::math::is_matrix_finite(q);
}

}  // namespace solver
}  // namespace mr::robotics::arm

#endif
