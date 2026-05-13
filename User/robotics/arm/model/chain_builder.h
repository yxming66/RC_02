#ifndef ARM_LIB_MODEL_CHAIN_BUILDER_H
#define ARM_LIB_MODEL_CHAIN_BUILDER_H

#include "serial_chain.h"

namespace mr::robotics::arm {

template <int N>
inline SerialChain<N> make_serial_chain(const Link (&links)[N]) {
  return SerialChain<N>(links);
}

template <int N>
inline SerialChain<N> make_serial_chain(const Link (&links)[N],
                                        const Transform& base_frame,
                                        const ToolFrame& tool_frame) {
  return SerialChain<N>(links, base_frame, tool_frame);
}

template <int N>
inline void build_links_from_dh(
    const DhParams (&dh)[N],
    Link (&links)[N],
    DHConvention convention = DHConvention::kStandard) {
  for (uint16_t i = 0; i < N; ++i) {
    links[i] = Link(dh[i], ChainJointSpec(), convention);
  }
}

template <int N>
inline void build_links_from_dh(
    const DhParams (&dh)[N],
    const ChainJointSpec (&joints)[N],
    Link (&links)[N],
    DHConvention convention = DHConvention::kStandard) {
  for (uint16_t i = 0; i < N; ++i) {
    links[i] = Link(dh[i], joints[i], convention);
  }
}

template <int N>
inline SerialChain<N> load_chain_from_dh(
    const DhParams (&dh)[N],
    DHConvention convention = DHConvention::kStandard) {
  Link links[N];
  build_links_from_dh(dh, links, convention);
  return make_serial_chain(links);
}

template <int N>
inline SerialChain<N> load_chain_from_dh(
    const DhParams (&dh)[N],
    const ChainJointSpec (&joints)[N],
    DHConvention convention = DHConvention::kStandard) {
  Link links[N];
  build_links_from_dh(dh, joints, links, convention);
  return make_serial_chain(links);
}

template <int N>
inline SerialChain<N> load_chain_from_dh(
    const DhParams (&dh)[N],
    const ChainJointSpec (&joints)[N],
    const Transform& base_frame,
    const ToolFrame& tool_frame,
    DHConvention convention = DHConvention::kStandard) {
  Link links[N];
  build_links_from_dh(dh, joints, links, convention);
  return make_serial_chain(links, base_frame, tool_frame);
}

template <int N>
inline void set_joint_limits(SerialChain<N>* chain,
                             const JointVec<N>& lower,
                             const JointVec<N>& upper) {
  if (chain == nullptr) {
    return;
  }

  for (uint16_t i = 0; i < N; ++i) {
    Link link = chain->link(i);
    ChainJointSpec joint = link.joint();
    if (!chain_joint_is_movable(joint.type)) {
      continue;
    }
    joint.limit_enabled = true;
    joint.lower = lower[i][0];
    joint.upper = upper[i][0];
    link.set_joint(joint);
    chain->set_link(i, link);
  }
}

template <int N>
inline void set_joint_offsets(SerialChain<N>* chain,
                              const JointVec<N>& offset) {
  if (chain == nullptr) {
    return;
  }

  for (uint16_t i = 0; i < N; ++i) {
    Link link = chain->link(i);
    ChainJointSpec joint = link.joint();
    joint.offset = offset[i][0];
    link.set_joint(joint);
    chain->set_link(i, link);
  }
}

template <int N>
inline void set_joint_transmissions(
    SerialChain<N>* chain,
    const ChainTransmission (&transmissions)[N]) {
  if (chain == nullptr) {
    return;
  }

  for (uint16_t i = 0; i < N; ++i) {
    Link link = chain->link(i);
    link.set_transmission(transmissions[i]);
    chain->set_link(i, link);
  }
}

}  // namespace mr::robotics::arm

#endif
