#ifndef ARM_LIB_KINEMATICS_JACOBIAN_H
#define ARM_LIB_KINEMATICS_JACOBIAN_H

#include "fk.h"

namespace arm_lib {
namespace kinematics {

template <int N>
inline Jacobian6xN<N> jacobian(const SerialChain<N>& chain,
                               const JointVec<N>& q) {
  Jacobian6xN<N> J = matrixf::zeros<6, N>();
  const Transform end_transform = fk(chain, q);
  const Vec3 p_end = toolbox_adapter::translation_of(end_transform);

  Transform prefix = chain.base_frame();

  for (uint16_t i = 0; i < N; ++i) {
    const Link& link = chain.link(i);
    const Transform joint_frame = prefix * link.joint_origin_transform();
    const Vec3 axis =
        toolbox_adapter::rotation_of(joint_frame) * link.joint_axis_local();
    const Vec3 origin = toolbox_adapter::translation_of(joint_frame);

    Vec3 linear = matrixf::zeros<3, 1>();
    Vec3 angular = matrixf::zeros<3, 1>();

    if (link.joint_type() == ChainJointType::kRevolute) {
      linear = vector3f::cross(axis, p_end - origin);
      angular = axis;
    } else if (link.joint_type() == ChainJointType::kPrismatic) {
      linear = axis;
    }

    J[0][i] = linear[0][0];
    J[1][i] = linear[1][0];
    J[2][i] = linear[2][0];
    J[3][i] = angular[0][0];
    J[4][i] = angular[1][0];
    J[5][i] = angular[2][0];

    prefix = prefix * link.transform(q[i][0]);
  }

  return J;
}

template <int N>
inline Jacobian6xN<N> jacobian_to_prefix(const SerialChain<N>& chain,
                                         const JointVec<N>& q,
                                         uint16_t joint_count) {
  Jacobian6xN<N> J = matrixf::zeros<6, N>();
  if (joint_count > N) {
    joint_count = N;
  }

  const Transform end_transform = fk(chain, q, joint_count, false);
  const Vec3 p_end = toolbox_adapter::translation_of(end_transform);

  Transform prefix = chain.base_frame();

  for (uint16_t i = 0; i < joint_count; ++i) {
    const Link& link = chain.link(i);
    const Transform joint_frame = prefix * link.joint_origin_transform();
    const Vec3 axis =
        toolbox_adapter::rotation_of(joint_frame) * link.joint_axis_local();
    const Vec3 origin = toolbox_adapter::translation_of(joint_frame);

    Vec3 linear = matrixf::zeros<3, 1>();
    Vec3 angular = matrixf::zeros<3, 1>();

    if (link.joint_type() == ChainJointType::kRevolute) {
      linear = vector3f::cross(axis, p_end - origin);
      angular = axis;
    } else if (link.joint_type() == ChainJointType::kPrismatic) {
      linear = axis;
    }

    J[0][i] = linear[0][0];
    J[1][i] = linear[1][0];
    J[2][i] = linear[2][0];
    J[3][i] = angular[0][0];
    J[4][i] = angular[1][0];
    J[5][i] = angular[2][0];

    prefix = prefix * link.transform(q[i][0]);
  }

  return J;
}

}  // namespace kinematics
}  // namespace arm_lib

#endif
