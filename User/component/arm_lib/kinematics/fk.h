#ifndef ARM_LIB_KINEMATICS_FK_H
#define ARM_LIB_KINEMATICS_FK_H

#include "../model/serial_chain.h"

namespace arm_lib {
namespace kinematics {

template <int N>
inline Transform fk_prefix(const SerialChain<N>& chain,
                           const JointVec<N>& q,
                           uint16_t joint_count) {
  Transform transform = chain.base_frame();
  if (joint_count > N) {
    joint_count = N;
  }

  for (uint16_t i = 0; i < joint_count; ++i) {
    transform = transform * chain.link(i).transform(q[i][0]);
  }
  return transform;
}

template <int N>
inline Transform fk(const SerialChain<N>& chain, const JointVec<N>& q) {
  return fk_prefix(chain, q, N) * chain.tool_frame().transform();
}

template <int N>
inline Transform fk(const SerialChain<N>& chain,
                    const JointVec<N>& q,
                    uint16_t joint_count,
                    bool include_tool) {
  Transform transform = fk_prefix(chain, q, joint_count);
  if (include_tool && joint_count >= N) {
    transform = transform * chain.tool_frame().transform();
  }
  return transform;
}

template <int N>
inline Transform link_transform(const SerialChain<N>& chain,
                                const JointVec<N>& q,
                                Index index) {
  if (index >= N) {
    return toolbox_adapter::identity_transform();
  }
  return chain.link(index).transform(q[index][0]);
}

}  // namespace kinematics
}  // namespace arm_lib

#endif
