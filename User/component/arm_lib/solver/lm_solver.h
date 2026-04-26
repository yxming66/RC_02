#ifndef ARM_LIB_SOLVER_LM_SOLVER_H
#define ARM_LIB_SOLVER_LM_SOLVER_H

#include "../adapter/toolbox_adapter.h"
#include "../core/arm_common.h"
#include "../core/arm_types.h"
#include "solver_common.h"

namespace arm_lib {
namespace solver {

template <int N>
inline JointVec<N> lm_delta(const Jacobian6xN<N>& jacobian,
                            const Twist6& error,
                            Scalar damping) {
  const JointDiag<N> identity = matrixf::eye<N, N>();
  const JointDiag<N> normal =
      jacobian.trans() * jacobian + damping * identity;
  return matrixf::inv(normal) * (jacobian.trans() * error);
}

template <int N>
inline bool lm_delta_safe(const Jacobian6xN<N>& jacobian,
                          const Twist6& error,
                          Scalar damping,
                          JointVec<N>* out_delta) {
  if (out_delta == nullptr) {
    return false;
  }

  const JointDiag<N> identity = matrixf::eye<N, N>();
  const JointDiag<N> normal =
      jacobian.trans() * jacobian + damping * identity;
  const JointDiag<N> inverse = matrixf::inv(normal);
  if (!is_matrix_finite(inverse)) {
    *out_delta = toolbox_adapter::zero_joint_vec<N>();
    return false;
  }

  *out_delta = inverse * (jacobian.trans() * error);
  return is_joint_vector_finite(*out_delta);
}

}  // namespace solver
}  // namespace arm_lib

#endif
