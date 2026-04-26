#include "link.h"

#include <cmath>

#include "../core/arm_common.h"

namespace arm_lib {
namespace {

Vec3 default_joint_axis() {
  Vec3 axis = toolbox_adapter::zero_vec3();
  axis[2][0] = 1.0f;
  return axis;
}

Vec3 normalized_axis(const Vec3& axis) {
  const Scalar norm = axis.norm();
  if (norm <= ARM_LIB_EPSILON) {
    return default_joint_axis();
  }
  return axis / norm;
}

Transform make_axis_rotation_transform(const Vec3& axis, Scalar angle) {
  const Vec3 unit_axis = normalized_axis(axis);
  Vec4 angvec = matrixf::zeros<4, 1>();
  angvec[0][0] = unit_axis[0][0];
  angvec[1][0] = unit_axis[1][0];
  angvec[2][0] = unit_axis[2][0];
  angvec[3][0] = angle;
  return robotics::angvec2t(angvec);
}

Transform make_axis_translation_transform(const Vec3& axis, Scalar distance) {
  return robotics::p2t(normalized_axis(axis) * distance);
}

Transform make_standard_dh_transform(Scalar theta,
                                     Scalar d,
                                     Scalar a,
                                     Scalar alpha) {
  const Scalar ct = cosf(theta);
  const Scalar st = sinf(theta);
  const Scalar ca = cosf(alpha);
  const Scalar sa = sinf(alpha);

  Transform transform = toolbox_adapter::identity_transform();

  transform[0][0] = ct;
  transform[0][1] = -st * ca;
  transform[0][2] = st * sa;
  transform[0][3] = a * ct;

  transform[1][0] = st;
  transform[1][1] = ct * ca;
  transform[1][2] = -ct * sa;
  transform[1][3] = a * st;

  transform[2][0] = 0.0f;
  transform[2][1] = sa;
  transform[2][2] = ca;
  transform[2][3] = d;

  transform[3][0] = 0.0f;
  transform[3][1] = 0.0f;
  transform[3][2] = 0.0f;
  transform[3][3] = 1.0f;

  return transform;
}

Transform make_modified_dh_transform(Scalar theta,
                                     Scalar d,
                                     Scalar a,
                                     Scalar alpha) {
  const Scalar ct = cosf(theta);
  const Scalar st = sinf(theta);
  const Scalar ca = cosf(alpha);
  const Scalar sa = sinf(alpha);

  Transform transform = toolbox_adapter::identity_transform();

  transform[0][0] = ct;
  transform[0][1] = -st;
  transform[0][2] = 0.0f;
  transform[0][3] = a;

  transform[1][0] = st * ca;
  transform[1][1] = ct * ca;
  transform[1][2] = -sa;
  transform[1][3] = -sa * d;

  transform[2][0] = st * sa;
  transform[2][1] = ct * sa;
  transform[2][2] = ca;
  transform[2][3] = ca * d;

  transform[3][0] = 0.0f;
  transform[3][1] = 0.0f;
  transform[3][2] = 0.0f;
  transform[3][3] = 1.0f;

  return transform;
}

}  // namespace

Link::Link()
    : dh_(),
      joint_(),
      inertial_(),
      kinematics_model_(LinkKinematics::kDh),
      joint_origin_(toolbox_adapter::identity_transform()),
      joint_axis_(default_joint_axis()),
      convention_(DHConvention::kStandard) {}

Link::Link(const DhParams& dh,
           const ChainJointSpec& joint,
           DHConvention convention)
    : dh_(dh),
      joint_(joint),
      inertial_(),
      kinematics_model_(LinkKinematics::kDh),
      joint_origin_(toolbox_adapter::identity_transform()),
      joint_axis_(default_joint_axis()),
      convention_(convention) {}

Link::Link(const Transform& joint_origin,
           const Vec3& joint_axis,
           const ChainJointSpec& joint)
    : dh_(),
      joint_(joint),
      inertial_(),
      kinematics_model_(LinkKinematics::kGeneric),
      joint_origin_(joint_origin),
      joint_axis_(normalized_axis(joint_axis)),
      convention_(DHConvention::kStandard) {}

Link::Link(Scalar theta,
           Scalar d,
           Scalar a,
           Scalar alpha,
           ChainJointType type,
           Scalar offset,
           DHConvention convention)
    : dh_(),
      joint_(),
      inertial_(),
      kinematics_model_(LinkKinematics::kDh),
      joint_origin_(toolbox_adapter::identity_transform()),
      joint_axis_(default_joint_axis()),
      convention_(convention) {
  dh_.theta = theta;
  dh_.d = d;
  dh_.a = a;
  dh_.alpha = alpha;
  joint_.type = type;
  joint_.offset = offset;
  joint_.participate_in_ik = (type != ChainJointType::kFixed);
}

Transform Link::transform(Scalar q) const {
  if (kinematics_model_ == LinkKinematics::kGeneric) {
    if (joint_.type == ChainJointType::kRevolute) {
      return joint_origin_ *
             make_axis_rotation_transform(joint_axis_, joint_.offset + q);
    }
    if (joint_.type == ChainJointType::kPrismatic) {
      return joint_origin_ *
             make_axis_translation_transform(joint_axis_, joint_.offset + q);
    }
    return joint_origin_;
  }

  Scalar theta = dh_.theta;
  Scalar d = dh_.d;

  if (joint_.type == ChainJointType::kRevolute) {
    theta += joint_.offset + q;
  } else if (joint_.type == ChainJointType::kPrismatic) {
    d += joint_.offset + q;
  } else {
    theta += joint_.offset;
  }

  if (convention_ == DHConvention::kModified) {
    return make_modified_dh_transform(theta, d, dh_.a, dh_.alpha);
  }
  return make_standard_dh_transform(theta, d, dh_.a, dh_.alpha);
}

Transform Link::zero_transform() const {
  return transform(0.0f);
}

bool Link::is_valid() const {
  if (!chain_joint_spec_is_valid(joint_)) {
    return false;
  }

  if (kinematics_model_ == LinkKinematics::kDh) {
    return dh_params_are_valid(dh_);
  }

  if (!is_finite_scalar(joint_axis_[0][0]) || !is_finite_scalar(joint_axis_[1][0]) ||
      !is_finite_scalar(joint_axis_[2][0])) {
    return false;
  }

  return inertial_params_are_valid(inertial_);
}

bool dh_params_are_valid(const DhParams& dh) {
  return is_finite_scalar(dh.theta) && is_finite_scalar(dh.d) &&
         is_finite_scalar(dh.a) && is_finite_scalar(dh.alpha);
}

bool inertial_params_are_valid(const InertialParams& inertial) {
  if (!is_finite_scalar(inertial.mass) || inertial.mass < 0.0f) {
    return false;
  }

  for (uint16_t i = 0; i < 3U; ++i) {
    if (!is_finite_scalar(inertial.com[i][0])) {
      return false;
    }
    for (uint16_t j = 0; j < 3U; ++j) {
      if (!is_finite_scalar(inertial.inertia[i][j])) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace arm_lib
