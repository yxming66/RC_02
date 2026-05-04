#include "toolbox_adapter.h"

namespace mr::robotics::arm {
namespace toolbox_adapter {

Transform identity_transform() {
  return matrixf::eye<4, 4>();
}

Rotation identity_rotation() {
  return matrixf::eye<3, 3>();
}

Vec3 zero_vec3() {
  return matrixf::zeros<3, 1>();
}

Twist6 zero_twist() {
  return matrixf::zeros<6, 1>();
}

Rotation rotation_from_rpy(const Vec3& rpy) {
  return ::robotics::rpy2r(rpy);
}

Vec3 rpy_from_rotation(const Rotation& rotation) {
  return ::robotics::r2rpy(rotation);
}

Transform make_transform(const Rotation& rotation, const Vec3& translation) {
  return ::robotics::rp2t(rotation, translation);
}

Transform transform_from_rpy_translation(const Vec3& rpy,
                                         const Vec3& translation) {
  return make_transform(rotation_from_rpy(rpy), translation);
}

Rotation rotation_of(const Transform& transform) {
  return ::robotics::t2r(transform);
}

Vec3 translation_of(const Transform& transform) {
  return ::robotics::t2p(transform);
}

Transform inverse_transform(const Transform& transform) {
  return ::robotics::invT(transform);
}

Vec4 quaternion_from_rotation(const Rotation& rotation) {
  return ::robotics::r2quat(rotation);
}

Rotation rotation_from_quaternion(const Vec4& quaternion) {
  return ::robotics::quat2r(quaternion);
}

Twist6 twist_from_transform(const Transform& transform) {
  return ::robotics::t2twist(transform);
}

Transform transform_from_twist(const Twist6& twist) {
  return ::robotics::twist2t(twist);
}

}  // namespace toolbox_adapter
}  // namespace mr::robotics::arm
