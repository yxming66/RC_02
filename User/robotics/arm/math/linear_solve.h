#ifndef ARM_LIB_MATH_LINEAR_SOLVE_H
#define ARM_LIB_MATH_LINEAR_SOLVE_H

#include "component/math/linear_solve.hpp"

#include "../core/arm_types.h"

namespace mr::robotics::arm {
namespace math {

template <int N, int C>
inline bool solve_symmetric_ldlt(const Matrixf<N, N>& a,
                                 const Matrixf<N, C>& b,
                                 Matrixf<N, C>* x,
                                 Scalar min_pivot = 1.0e-8f) {
  return ::mr::component::math::solve_symmetric_ldlt<N, C>(
      a, b, x, min_pivot);
}

template <int N>
inline bool solve_symmetric_ldlt(const Matrixf<N, N>& a,
                                 const Matrixf<N, 1>& b,
                                 Matrixf<N, 1>* x,
                                 Scalar min_pivot = 1.0e-8f) {
  return solve_symmetric_ldlt<N, 1>(a, b, x, min_pivot);
}

}  // namespace math
}  // namespace mr::robotics::arm

#endif
