#ifndef ARM_LIB_ADAPTER_TOOLBOX_ADAPTER_H
#define ARM_LIB_ADAPTER_TOOLBOX_ADAPTER_H

#include "../core/arm_types.h"

#include "../../toolbox/robotics.h"

namespace arm_lib {
namespace toolbox_adapter {

Transform identity_transform();
Rotation identity_rotation();
Vec3 zero_vec3();
Twist6 zero_twist();

template <int N>
inline JointVec<N> zero_joint_vec() {
  return matrixf::zeros<N, 1>();
}

Rotation rotation_from_rpy(const Vec3& rpy);
Vec3 rpy_from_rotation(const Rotation& rotation);

Transform make_transform(const Rotation& rotation, const Vec3& translation);
Transform transform_from_rpy_translation(const Vec3& rpy,
                                         const Vec3& translation);
Rotation rotation_of(const Transform& transform);
Vec3 translation_of(const Transform& transform);
Transform inverse_transform(const Transform& transform);

Vec4 quaternion_from_rotation(const Rotation& rotation);
Rotation rotation_from_quaternion(const Vec4& quaternion);

Twist6 twist_from_transform(const Transform& transform);
Transform transform_from_twist(const Twist6& twist);

}  // namespace toolbox_adapter
}  // namespace arm_lib

#endif
