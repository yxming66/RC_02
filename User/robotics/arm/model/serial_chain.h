#ifndef ARM_LIB_MODEL_SERIAL_CHAIN_H
#define ARM_LIB_MODEL_SERIAL_CHAIN_H

#include "../core/arm_limits.h"
#include "joint_mapping.h"
#include "link.h"
#include "tool_frame.h"

namespace mr::robotics::arm {

template <int N>
class SerialChain {
 public:
  SerialChain()
      : base_frame_(toolbox_adapter::identity_transform()),
        tool_frame_(ToolFrame::identity()),
        gravity_(toolbox_adapter::zero_vec3()) {
    gravity_[2][0] = -9.81f;
  }

  explicit SerialChain(const Link (&links)[N]) : SerialChain() {
    for (uint16_t i = 0; i < N; ++i) {
      links_[i] = links[i];
    }
  }

  SerialChain(const Link (&links)[N],
              const Transform& base_frame,
              const ToolFrame& tool_frame)
      : SerialChain(links) {
    base_frame_ = base_frame;
    tool_frame_ = tool_frame;
  }

  static constexpr int dof() { return N; }

  const Link& link(Index index) const { return links_[index]; }
  Link& link(Index index) { return links_[index]; }

  void set_link(Index index, const Link& link) {
    if (index < N) {
      links_[index] = link;
    }
  }

  bool is_valid_index(Index index) const { return index < N; }

  const Transform& base_frame() const { return base_frame_; }
  void set_base_frame(const Transform& base_frame) { base_frame_ = base_frame; }

  const ToolFrame& tool_frame() const { return tool_frame_; }
  void set_tool_frame(const ToolFrame& tool_frame) { tool_frame_ = tool_frame; }

  const Vec3& gravity() const { return gravity_; }
  void set_gravity(const Vec3& gravity) { gravity_ = gravity; }

  JointVec<N> zero_configuration() const {
    return toolbox_adapter::zero_joint_vec<N>();
  }

  JointVec<N> joint_to_actuator_position(const JointVec<N>& joint_q) const {
    JointVec<N> actuator_q = toolbox_adapter::zero_joint_vec<N>();
    for (uint16_t i = 0; i < N; ++i) {
      const ChainTransmission& transmission = links_[i].transmission();
      Scalar coupled = 0.0f;
      for (uint16_t term = 0; term < transmission.coupling_count; ++term) {
        const ChainJointCouplingTerm& coupling = transmission.couplings[term];
        if (!coupling.enabled || coupling.source_joint >= N) {
          continue;
        }
        coupled += coupling.coefficient * joint_q[coupling.source_joint][0];
      }
      actuator_q[i][0] = chain_transmission_joint_to_actuator_position(
          transmission, joint_q[i][0], coupled);
    }
    return actuator_q;
  }

  JointVec<N> joint_to_actuator_velocity(const JointVec<N>& joint_dq) const {
    JointVec<N> actuator_dq = toolbox_adapter::zero_joint_vec<N>();
    for (uint16_t i = 0; i < N; ++i) {
      const ChainTransmission& transmission = links_[i].transmission();
      Scalar coupled = 0.0f;
      for (uint16_t term = 0; term < transmission.coupling_count; ++term) {
        const ChainJointCouplingTerm& coupling = transmission.couplings[term];
        if (!coupling.enabled || coupling.source_joint >= N) {
          continue;
        }
        coupled += coupling.coefficient * joint_dq[coupling.source_joint][0];
      }
      actuator_dq[i][0] = chain_transmission_joint_to_actuator_velocity(
          transmission, joint_dq[i][0], coupled);
    }
    return actuator_dq;
  }

  JointVec<N> actuator_to_joint_position(const JointVec<N>& actuator_q) const {
    JointVec<N> joint_q = toolbox_adapter::zero_joint_vec<N>();
    for (uint16_t i = 0; i < N; ++i) {
      const ChainTransmission& transmission = links_[i].transmission();
      Scalar coupled = 0.0f;
      for (uint16_t term = 0; term < transmission.coupling_count; ++term) {
        const ChainJointCouplingTerm& coupling = transmission.couplings[term];
        if (!coupling.enabled || coupling.source_joint >= N) {
          continue;
        }
        coupled += coupling.coefficient * joint_q[coupling.source_joint][0];
      }
      joint_q[i][0] = chain_transmission_actuator_to_joint_position(
          transmission, actuator_q[i][0], coupled);
    }
    return joint_q;
  }

  JointVec<N> actuator_to_joint_velocity(const JointVec<N>& actuator_dq) const {
    JointVec<N> joint_dq = toolbox_adapter::zero_joint_vec<N>();
    for (uint16_t i = 0; i < N; ++i) {
      const ChainTransmission& transmission = links_[i].transmission();
      Scalar coupled = 0.0f;
      for (uint16_t term = 0; term < transmission.coupling_count; ++term) {
        const ChainJointCouplingTerm& coupling = transmission.couplings[term];
        if (!coupling.enabled || coupling.source_joint >= N) {
          continue;
        }
        coupled += coupling.coefficient * joint_dq[coupling.source_joint][0];
      }
      joint_dq[i][0] = chain_transmission_actuator_to_joint_velocity(
          transmission, actuator_dq[i][0], coupled);
    }
    return joint_dq;
  }

  bool transmission_is_identity() const {
    for (uint16_t i = 0; i < N; ++i) {
      if (!chain_transmission_is_identity(links_[i].joint())) {
        return false;
      }
    }
    return true;
  }

  JointLimits<N> joint_limits() const {
    JointLimits<N> limits;
    for (uint16_t i = 0; i < N; ++i) {
      if (chain_joint_is_movable(links_[i].joint_type()) &&
          links_[i].limit_enabled()) {
        limits.set(i, links_[i].lower_limit(), links_[i].upper_limit());
      }
    }
    return limits;
  }

  uint16_t active_joint_count() const {
    uint16_t count = 0;
    for (uint16_t i = 0; i < N; ++i) {
      if (chain_joint_is_movable(links_[i].joint_type()) &&
          links_[i].participates_in_ik()) {
        ++count;
      }
    }
    return count;
  }

  bool validate() const {
    for (uint16_t i = 0; i < N; ++i) {
      if (!links_[i].is_valid()) {
        return false;
      }

      const ChainTransmission& transmission = links_[i].transmission();
      for (uint16_t term = 0; term < transmission.coupling_count; ++term) {
        const ChainJointCouplingTerm& coupling = transmission.couplings[term];
        if (coupling.enabled && coupling.source_joint >= N) {
          return false;
        }
      }
    }

    for (uint16_t row = 0; row < 4U; ++row) {
      for (uint16_t col = 0; col < 4U; ++col) {
        if (!is_finite_scalar(base_frame_[row][col]) ||
            !is_finite_scalar(tool_frame_.transform()[row][col])) {
          return false;
        }
      }
    }

    for (uint16_t i = 0; i < 3U; ++i) {
      if (!is_finite_scalar(gravity_[i][0])) {
        return false;
      }
    }

    return true;
  }

 private:
  Link links_[N];
  Transform base_frame_;
  ToolFrame tool_frame_;
  Vec3 gravity_;
};

}  // namespace mr::robotics::arm

#endif
