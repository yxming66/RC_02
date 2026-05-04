#ifndef ARM_LIB_DYNAMICS_GRAVITY_H
#define ARM_LIB_DYNAMICS_GRAVITY_H

#include "../kinematics/fk.h"

namespace mr::robotics::arm {
namespace dynamics {

inline Vec3 transform_point(const Transform& transform, const Vec3& point) {
  Vec3 out = toolbox_adapter::zero_vec3();
  out[0][0] = transform[0][0] * point[0][0] + transform[0][1] * point[1][0] +
              transform[0][2] * point[2][0] + transform[0][3];
  out[1][0] = transform[1][0] * point[0][0] + transform[1][1] * point[1][0] +
              transform[1][2] * point[2][0] + transform[1][3];
  out[2][0] = transform[2][0] * point[0][0] + transform[2][1] * point[1][0] +
              transform[2][2] * point[2][0] + transform[2][3];
  return out;
}

template <int N>
inline JointVec<N> gravity_torques(const SerialChain<N>& chain,
                                   const JointVec<N>& q) {
  JointVec<N> tau = toolbox_adapter::zero_joint_vec<N>();
  const Vec3 gravity = chain.gravity();

  for (uint16_t link_index = 0; link_index < N; ++link_index) {
    const Link& link = chain.link(link_index);
    if (link.mass() <= 0.0f) {
      continue;
    }

    const Transform link_tf =
        kinematics::fk(chain, q, static_cast<uint16_t>(link_index + 1), false);
    const Vec3 p_com = transform_point(link_tf, link.com());

    Transform prefix = chain.base_frame();
    for (uint16_t joint_index = 0; joint_index <= link_index; ++joint_index) {
      const Link& joint_link = chain.link(joint_index);
      const Transform joint_frame =
          prefix * joint_link.joint_origin_transform();
      const Vec3 axis = toolbox_adapter::rotation_of(joint_frame) *
                        joint_link.joint_axis_local();
      const Vec3 origin = toolbox_adapter::translation_of(joint_frame);

      Scalar contribution = 0.0f;
      if (joint_link.joint_type() == ChainJointType::kRevolute) {
        const Vec3 linear = vector3f::cross(axis, p_com - origin);
        contribution = -(linear[0][0] * gravity[0][0] +
                         linear[1][0] * gravity[1][0] +
                         linear[2][0] * gravity[2][0]) *
                       link.mass();
      } else if (joint_link.joint_type() == ChainJointType::kPrismatic) {
        contribution = -(axis[0][0] * gravity[0][0] +
                         axis[1][0] * gravity[1][0] +
                         axis[2][0] * gravity[2][0]) *
                       link.mass();
      }

      tau[joint_index][0] += contribution;
      prefix = prefix * joint_link.transform(q[joint_index][0]);
    }
  }

  return tau;
}

}  // namespace dynamics
}  // namespace mr::robotics::arm

#endif
