#ifndef ARM_LIB_SOLVER_LM_SOLVER_H
#define ARM_LIB_SOLVER_LM_SOLVER_H

#include "../adapter/toolbox_adapter.h"
#include "../core/arm_common.h"
#include "../core/arm_types.h"
#include "../math/linear_solve.h"
#include "solver_common.h"

namespace mr::robotics::arm {
namespace solver {

template <int N>
inline JointVec<N> lm_delta(const Jacobian6xN<N>& jacobian,
                            const Twist6& error,
                            Scalar damping) {
  const JointDiag<N> identity = matrixf::eye<N, N>();
  const JointDiag<N> normal =
      jacobian.trans() * jacobian + damping * identity;
  JointVec<N> out = toolbox_adapter::zero_joint_vec<N>();
  (void)math::solve_symmetric_ldlt(normal, jacobian.trans() * error, &out);
  return out;
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
  const JointVec<N> rhs = jacobian.trans() * error;
  if (!math::solve_symmetric_ldlt(normal, rhs, out_delta)) {
    *out_delta = toolbox_adapter::zero_joint_vec<N>();
    return false;
  }
  return is_joint_vector_finite(*out_delta);
}

}  // namespace solver
}  // namespace mr::robotics::arm

#endif
