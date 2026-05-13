#ifndef ARM_LIB_MODEL_JOINT_MAPPING_H
#define ARM_LIB_MODEL_JOINT_MAPPING_H

#include "joint.h"

namespace mr::robotics::arm {

inline Scalar chain_transmission_joint_to_actuator_position(
    const ChainTransmission& transmission,
    Scalar joint_position,
    Scalar coupled_position,
    Scalar actuator_calibration_offset = 0.0f) {
  return transmission.actuator_scale * joint_position +
         transmission.actuator_offset +
         coupled_position +
         actuator_calibration_offset;
}

inline Scalar chain_transmission_joint_to_actuator_velocity(
    const ChainTransmission& transmission,
    Scalar joint_velocity,
    Scalar coupled_velocity) {
  return transmission.actuator_scale * joint_velocity + coupled_velocity;
}

inline Scalar chain_transmission_actuator_to_joint_position(
    const ChainTransmission& transmission,
    Scalar actuator_position,
    Scalar coupled_position,
    Scalar actuator_calibration_offset = 0.0f) {
  Scalar joint = actuator_position - actuator_calibration_offset -
                 transmission.actuator_offset - coupled_position;
  const Scalar scale = transmission.actuator_scale;
  if ((scale > ARM_LIB_EPSILON) || (scale < -ARM_LIB_EPSILON)) {
    joint /= scale;
  } else {
    joint = 0.0f;
  }
  return joint;
}

inline Scalar chain_transmission_actuator_to_joint_velocity(
    const ChainTransmission& transmission,
    Scalar actuator_velocity,
    Scalar coupled_velocity) {
  Scalar joint = actuator_velocity - coupled_velocity;
  const Scalar scale = transmission.actuator_scale;
  if ((scale > ARM_LIB_EPSILON) || (scale < -ARM_LIB_EPSILON)) {
    joint /= scale;
  } else {
    joint = 0.0f;
  }
  return joint;
}

}  // namespace mr::robotics::arm

#endif
