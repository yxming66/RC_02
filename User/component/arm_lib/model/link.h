#ifndef ARM_LIB_MODEL_LINK_H
#define ARM_LIB_MODEL_LINK_H

#include "../adapter/toolbox_adapter.h"
#include "../core/arm_status.h"
#include "joint.h"

namespace arm_lib {

enum class LinkKinematics : uint8_t {
  kDh = 0,
  kGeneric = 1,
};

enum class DHConvention : uint8_t {
  kStandard = 0,
  kModified = 1,
};

struct DhParams {
  Scalar theta;
  Scalar d;
  Scalar a;
  Scalar alpha;

  DhParams() : theta(0.0f), d(0.0f), a(0.0f), alpha(0.0f) {}
};

struct InertialParams {
  Scalar mass;
  Vec3 com;
  Rotation inertia;

  InertialParams()
      : mass(0.0f),
        com(toolbox_adapter::zero_vec3()),
        inertia(toolbox_adapter::identity_rotation() * 0.0f) {}
};

class Link {
 public:
  Link();
  Link(const DhParams& dh,
      const ChainJointSpec& joint,
       DHConvention convention = DHConvention::kStandard);
  Link(const Transform& joint_origin,
       const Vec3& joint_axis,
      const ChainJointSpec& joint);
  Link(Scalar theta,
       Scalar d,
       Scalar a,
       Scalar alpha,
      ChainJointType type = ChainJointType::kRevolute,
       Scalar offset = 0.0f,
       DHConvention convention = DHConvention::kStandard);

  const DhParams& dh() const { return dh_; }
    const ChainJointSpec& joint() const { return joint_; }
    const ChainTransmission& transmission() const { return joint_.transmission; }
  const InertialParams& inertial() const { return inertial_; }
  LinkKinematics kinematics_model() const { return kinematics_model_; }
  DHConvention convention() const { return convention_; }
  const Transform& joint_origin_transform() const { return joint_origin_; }
  const Vec3& joint_axis_local() const { return joint_axis_; }

  void set_dh(const DhParams& dh) { dh_ = dh; }
  void set_joint(const ChainJointSpec& joint) { joint_ = joint; }
  void set_transmission(const ChainTransmission& transmission) {
    joint_.transmission = transmission;
  }
  void set_inertial(const InertialParams& inertial) { inertial_ = inertial; }
  void set_joint_origin_transform(const Transform& joint_origin) {
    joint_origin_ = joint_origin;
  }
  void set_joint_axis_local(const Vec3& joint_axis) { joint_axis_ = joint_axis; }
  void set_kinematics_model(LinkKinematics model) { kinematics_model_ = model; }
  void set_convention(DHConvention convention) { convention_ = convention; }

  ChainJointType joint_type() const { return joint_.type; }
  Scalar joint_offset() const { return joint_.offset; }
  bool limit_enabled() const { return joint_.limit_enabled; }
  bool participates_in_ik() const { return joint_.participate_in_ik; }
  bool has_joint_coupling() const { return chain_joint_has_coupling(joint_); }
  Scalar lower_limit() const { return joint_.lower; }
  Scalar upper_limit() const { return joint_.upper; }
  Scalar mass() const { return inertial_.mass; }
  const Vec3& com() const { return inertial_.com; }
  const Rotation& inertia() const { return inertial_.inertia; }

  Transform transform(Scalar q) const;
  Transform zero_transform() const;
  bool is_valid() const;

 private:
  DhParams dh_;
  ChainJointSpec joint_;
  InertialParams inertial_;
  LinkKinematics kinematics_model_;
  Transform joint_origin_;
  Vec3 joint_axis_;
  DHConvention convention_;
};

bool dh_params_are_valid(const DhParams& dh);
bool inertial_params_are_valid(const InertialParams& inertial);

}  // namespace arm_lib

#endif
