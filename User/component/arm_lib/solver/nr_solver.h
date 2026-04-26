#ifndef ARM_LIB_SOLVER_NR_SOLVER_H
#define ARM_LIB_SOLVER_NR_SOLVER_H

#include "../adapter/toolbox_adapter.h"
#include "../core/arm_common.h"
#include "../core/arm_types.h"
#include "solver_common.h"

namespace arm_lib {
namespace solver {

template <int N>
inline JointVec<N> nr_delta(const Jacobian6xN<N>& jacobian,
                            const Twist6& error,
                            Scalar damping) {
  const Matrixf<6, 6> identity = matrixf::eye<6, 6>();
  const Matrixf<6, 6> task_space =
      jacobian * jacobian.trans() + damping * identity;
  return jacobian.trans() * (matrixf::inv(task_space) * error);
}

template <int N>
inline bool nr_delta_safe(const Jacobian6xN<N>& jacobian,
                          const Twist6& error,
                          Scalar damping,
                          JointVec<N>* out_delta) {
  if (out_delta == nullptr) {
    return false;
  }

  const Matrixf<6, 6> identity = matrixf::eye<6, 6>();
  const Matrixf<6, 6> task_space =
      jacobian * jacobian.trans() + damping * identity;
  const Matrixf<6, 6> inverse = matrixf::inv(task_space);
  if (!is_matrix_finite(inverse)) {
    *out_delta = toolbox_adapter::zero_joint_vec<N>();
    return false;
  }

  *out_delta = jacobian.trans() * (inverse * error);
  return is_joint_vector_finite(*out_delta);
}

}  // namespace solver
}  // namespace arm_lib

#endif
