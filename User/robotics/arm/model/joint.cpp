#include "joint.h"

#include "../core/arm_common.h"

namespace mr::robotics::arm {

bool chain_joint_is_movable(ChainJointType type) {
  return (type == ChainJointType::kRevolute) ||
         (type == ChainJointType::kPrismatic);
}

bool chain_joint_is_actuated(ChainJointType type) {
  return chain_joint_is_movable(type);
}

const char* chain_joint_type_name(ChainJointType type) {
  switch (type) {
    case ChainJointType::kRevolute:
      return "revolute";
    case ChainJointType::kPrismatic:
      return "prismatic";
    case ChainJointType::kFixed:
      return "fixed";
    default:
      return "unknown";
  }
}

bool chain_joint_has_coupling(const ChainJointSpec& joint) {
  for (uint16_t i = 0; i < joint.transmission.coupling_count; ++i) {
    if (joint.transmission.couplings[i].enabled &&
        abs_scalar(joint.transmission.couplings[i].coefficient) >
            ARM_LIB_EPSILON) {
      return true;
    }
  }
  return false;
}

bool chain_transmission_is_identity(const ChainJointSpec& joint) {
  if (abs_scalar(joint.transmission.actuator_scale - 1.0f) >
      ARM_LIB_EPSILON) {
    return false;
  }
  if (abs_scalar(joint.transmission.actuator_offset) > ARM_LIB_EPSILON) {
    return false;
  }
  return !chain_joint_has_coupling(joint);
}

bool chain_transmission_is_valid(const ChainTransmission& transmission) {
  if (!is_finite_scalar(transmission.actuator_scale) ||
      !is_finite_scalar(transmission.actuator_offset)) {
    return false;
  }

  if (transmission.coupling_count > ARM_LIB_MAX_JOINT_COUPLINGS) {
    return false;
  }

  for (uint16_t i = 0; i < transmission.coupling_count; ++i) {
    const ChainJointCouplingTerm& term = transmission.couplings[i];
    if (!is_finite_scalar(term.coefficient)) {
      return false;
    }
  }

  return true;
}

bool chain_joint_spec_is_valid(const ChainJointSpec& joint) {
  if (!is_finite_scalar(joint.offset) || !is_finite_scalar(joint.lower) ||
      !is_finite_scalar(joint.upper)) {
    return false;
  }

  if (joint.limit_enabled && (joint.lower > joint.upper)) {
    return false;
  }

  if (!chain_transmission_is_valid(joint.transmission)) {
    return false;
  }

  return true;
}

}  // namespace mr::robotics::arm
