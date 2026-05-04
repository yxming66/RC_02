#ifndef ARM_LIB_MODEL_JOINT_H
#define ARM_LIB_MODEL_JOINT_H

#include <stdint.h>

#include "../arm_config.h"
#include "../core/arm_types.h"

namespace mr::robotics::arm {

enum class ChainJointType : uint8_t {
  kRevolute = 0,
  kPrismatic = 1,
  kFixed = 2,
};

struct ChainJointCouplingTerm {
  Index source_joint;
  Scalar coefficient;
  bool enabled;

  ChainJointCouplingTerm()
      : source_joint(0U), coefficient(0.0f), enabled(false) {}
};

struct ChainTransmission {
  Scalar actuator_scale;
  Scalar actuator_offset;
  ChainJointCouplingTerm couplings[ARM_LIB_MAX_JOINT_COUPLINGS];
  uint8_t coupling_count;

  ChainTransmission()
      : actuator_scale(1.0f),
        actuator_offset(0.0f),
        couplings(),
        coupling_count(0U) {}

  void clear_couplings() {
    coupling_count = 0U;
    for (uint16_t i = 0; i < ARM_LIB_MAX_JOINT_COUPLINGS; ++i) {
      couplings[i] = ChainJointCouplingTerm();
    }
  }

  bool add_coupling(Index source_joint_index, Scalar coefficient) {
    if (coupling_count >= ARM_LIB_MAX_JOINT_COUPLINGS) {
      return false;
    }
    couplings[coupling_count].source_joint = source_joint_index;
    couplings[coupling_count].coefficient = coefficient;
    couplings[coupling_count].enabled = true;
    ++coupling_count;
    return true;
  }
};

struct ChainJointSpec {
  ChainJointType type;
  Scalar offset;
  bool limit_enabled;
  Scalar lower;
  Scalar upper;
  bool participate_in_ik;
  ChainTransmission transmission;

  ChainJointSpec()
      : type(ChainJointType::kRevolute),
        offset(0.0f),
        limit_enabled(false),
        lower(0.0f),
        upper(0.0f),
        participate_in_ik(true),
        transmission() {}
};

struct ChainJointState {
  Scalar position;
  Scalar velocity;
  Scalar effort;

  ChainJointState() : position(0.0f), velocity(0.0f), effort(0.0f) {}
};

bool chain_joint_is_movable(ChainJointType type);
bool chain_joint_is_actuated(ChainJointType type);
const char* chain_joint_type_name(ChainJointType type);
bool chain_joint_has_coupling(const ChainJointSpec& joint);
bool chain_transmission_is_identity(const ChainJointSpec& joint);
bool chain_joint_spec_is_valid(const ChainJointSpec& joint);
bool chain_transmission_is_valid(const ChainTransmission& transmission);

}  // namespace mr::robotics::arm

#endif
