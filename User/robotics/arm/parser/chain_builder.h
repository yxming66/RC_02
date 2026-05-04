#ifndef ARM_LIB_PARSER_CHAIN_BUILDER_H
#define ARM_LIB_PARSER_CHAIN_BUILDER_H

#include "../model/chain_builder.h"

namespace mr::robotics::arm {
namespace parser {

template <int N>
inline SerialChain<N> make_serial_chain(const Link (&links)[N]) {
  return ::mr::robotics::arm::make_serial_chain(links);
}

template <int N>
inline SerialChain<N> make_serial_chain(const Link (&links)[N],
                                        const Transform& base_frame,
                                        const ToolFrame& tool_frame) {
  return ::mr::robotics::arm::make_serial_chain(links, base_frame, tool_frame);
}

template <int N>
inline void set_joint_limits(SerialChain<N>* chain,
                             const JointVec<N>& lower,
                             const JointVec<N>& upper) {
  if (chain == nullptr) {
    return;
  }

  ::mr::robotics::arm::set_joint_limits(chain, lower, upper);
}

template <int N>
inline void set_joint_offsets(SerialChain<N>* chain, const JointVec<N>& offset) {
  if (chain == nullptr) {
    return;
  }

  ::mr::robotics::arm::set_joint_offsets(chain, offset);
}

template <int N>
inline void set_joint_transmissions(
    SerialChain<N>* chain, const ChainTransmission (&transmissions)[N]) {
  if (chain == nullptr) {
    return;
  }

  ::mr::robotics::arm::set_joint_transmissions(chain, transmissions);
}

}  // namespace parser
}  // namespace mr::robotics::arm

#endif
