#ifndef ARM_LIB_KINEMATICS_IK_6R_PIEPER_H
#define ARM_LIB_KINEMATICS_IK_6R_PIEPER_H

#include "../ik_analytic.h"

namespace mr::robotics::arm {
namespace kinematics {
namespace analytic {

bool is_pieper_6r_compatible(const SerialChain<6>& chain);
bool generate_pieper_6r_solutions(const SerialChain<6>& chain,
                                  const IKRequest<6>& request,
                                  AnalyticIKSolutionSet<6>* out);

inline AnalyticIKPlugin<6> make_pieper_6r_plugin() {
  AnalyticIKPlugin<6> plugin;
  plugin.info.kind = AnalyticIkPluginKind::kPieper6R;
  plugin.info.name = "pieper_6r";
  plugin.info.priority = 20U;
  plugin.name = "pieper_6r";
  plugin.is_compatible = is_pieper_6r_compatible;
  plugin.generate = generate_pieper_6r_solutions;
  return plugin;
}

}  // namespace analytic
}  // namespace kinematics
}  // namespace mr::robotics::arm

#endif
