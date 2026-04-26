#ifndef ARM_LIB_KINEMATICS_IK_3R_PLANAR_H
#define ARM_LIB_KINEMATICS_IK_3R_PLANAR_H

#include "ik_analytic.h"

namespace arm_lib {
namespace kinematics {
namespace analytic {

bool is_planar_3r_compatible(const SerialChain<3>& chain);
bool generate_planar_3r_solutions(const SerialChain<3>& chain,
                                  const IKRequest<3>& request,
                                  AnalyticIKSolutionSet<3>* out);

inline AnalyticIKPlugin<3> make_planar_3r_plugin() {
  AnalyticIKPlugin<3> plugin;
  plugin.info.kind = AnalyticIkPluginKind::kPlanar3R;
  plugin.info.name = "planar_3r";
  plugin.info.priority = 10U;
  plugin.name = "planar_3r";
  plugin.is_compatible = is_planar_3r_compatible;
  plugin.generate = generate_planar_3r_solutions;
  return plugin;
}

}  // namespace analytic
}  // namespace kinematics
}  // namespace arm_lib

#endif
