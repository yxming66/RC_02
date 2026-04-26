#ifndef ARM_LIB_PARSER_DH_LOADER_H
#define ARM_LIB_PARSER_DH_LOADER_H

#include "chain_builder.h"

namespace arm_lib {
namespace parser {

template <int N>
inline void build_links_from_dh(const DhParams (&dh)[N],
                                Link (&links)[N],
                                DHConvention convention = DHConvention::kStandard) {
  for (uint16_t i = 0; i < N; ++i) {
    links[i] = Link(dh[i], ChainJointSpec(), convention);
  }
}

template <int N>
inline void build_links_from_dh(const DhParams (&dh)[N],
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

}  // namespace parser
}  // namespace arm_lib

#endif
