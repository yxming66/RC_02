#ifndef ARM_LIB_CORE_ARM_TYPES_H
#define ARM_LIB_CORE_ARM_TYPES_H

#include <stdint.h>

#include "../../toolbox/matrix.h"

namespace arm_lib {

using Scalar = float;
using Index = uint16_t;

using Vec3 = Matrixf<3, 1>;
using Vec4 = Matrixf<4, 1>;
using Rotation = Matrixf<3, 3>;
using Transform = Matrixf<4, 4>;
using Pose = Transform;
using Twist6 = Matrixf<6, 1>;
using TaskMap6 = Matrixf<6, 6>;

template <int N>
using JointVec = Matrixf<N, 1>;

template <int N>
using Jacobian6xN = Matrixf<6, N>;

template <int N>
using JointDiag = Matrixf<N, N>;

}  // namespace arm_lib

#endif
