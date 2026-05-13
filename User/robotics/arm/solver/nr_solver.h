#ifndef ARM_LIB_SOLVER_NR_SOLVER_H
#define ARM_LIB_SOLVER_NR_SOLVER_H

#include "../adapter/toolbox_adapter.h"
#include "../core/arm_common.h"
#include "../core/arm_types.h"
#include "../math/linear_solve.h"
#include "solver_common.h"

namespace mr::robotics::arm {
namespace solver {

template <int N>
inline JointVec<N> nr_delta(const Jacobian6xN<N>& jacobian,
                            const Twist6& error,
                            Scalar damping) {
  const Matrixf<6, 6> identity = matrixf::eye<6, 6>();
  const Matrixf<6, 6> task_space =
      jacobian * jacobian.trans() + damping * identity;
  Twist6 task_step = toolbox_adapter::zero_twist();
  (void)math::solve_symmetric_ldlt(task_space, error, &task_step);
  return jacobian.trans() * task_step;
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
  Twist6 task_step = toolbox_adapter::zero_twist();
  if (!math::solve_symmetric_ldlt(task_space, error, &task_step)) {
    *out_delta = toolbox_adapter::zero_joint_vec<N>();
    return false;
  }

  *out_delta = jacobian.trans() * task_step;
  return is_joint_vector_finite(*out_delta);
}

}  // namespace solver
}  // namespace mr::robotics::arm

#endif
