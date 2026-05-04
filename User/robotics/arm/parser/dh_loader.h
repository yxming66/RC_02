#ifndef ARM_LIB_PARSER_DH_LOADER_H
#define ARM_LIB_PARSER_DH_LOADER_H

#include "chain_builder.h"

namespace mr::robotics::arm {
namespace parser {

template <int N>
inline void build_links_from_dh(const DhParams (&dh)[N],
                                Link (&links)[N],
                                DHConvention convention = DHConvention::kStandard) {
  ::mr::robotics::arm::build_links_from_dh(dh, links, convention);
}

template <int N>
inline void build_links_from_dh(const DhParams (&dh)[N],
                                const ChainJointSpec (&joints)[N],
                                Link (&links)[N],
                                DHConvention convention = DHConvention::kStandard) {
  ::mr::robotics::arm::build_links_from_dh(dh, joints, links, convention);
}

template <int N>
inline SerialChain<N> load_chain_from_dh(
    const DhParams (&dh)[N],
    DHConvention convention = DHConvention::kStandard) {
  return ::mr::robotics::arm::load_chain_from_dh(dh, convention);
}

template <int N>
inline SerialChain<N> load_chain_from_dh(
    const DhParams (&dh)[N],
  const ChainJointSpec (&joints)[N],
    DHConvention convention = DHConvention::kStandard) {
  return ::mr::robotics::arm::load_chain_from_dh(dh, joints, convention);
}

template <int N>
inline SerialChain<N> load_chain_from_dh(
    const DhParams (&dh)[N],
  const ChainJointSpec (&joints)[N],
    const Transform& base_frame,
    const ToolFrame& tool_frame,
    DHConvention convention = DHConvention::kStandard) {
  return ::mr::robotics::arm::load_chain_from_dh(dh, joints, base_frame, tool_frame,
                                     convention);
}

}  // namespace parser
}  // namespace mr::robotics::arm

#endif
