#ifndef ARM_LIB_KINEMATICS_ANALYTIC_SOLVERS_SOLVER_REGISTRY_H
#define ARM_LIB_KINEMATICS_ANALYTIC_SOLVERS_SOLVER_REGISTRY_H

#include "../ik.h"
#include "../ik_analytic.h"
#include "../ik_3r_planar.h"

namespace arm_lib {
namespace kinematics {
namespace analytic_solvers {

template <int N, int MaxPlugins = 6>
inline AnalyticIKRegistry<N, MaxPlugins> build_registry(
    AnalyticSolverPreset preset = AnalyticSolverPreset::kAuto) {
  AnalyticIKRegistry<N, MaxPlugins> registry;
  (void)preset;
  return registry;
}

template <>
inline AnalyticIKRegistry<3, 6> build_registry<3, 6>(
    AnalyticSolverPreset preset) {
  AnalyticIKRegistry<3, 6> registry;

  if (preset == AnalyticSolverPreset::kDisabled) {
    return registry;
  }

  if (preset == AnalyticSolverPreset::kAuto ||
      preset == AnalyticSolverPreset::kPlanar3R) {
    (void)register_analytic_plugin(analytic::make_planar_3r_plugin(),
                                   &registry);
  }

  return registry;
}

}  // namespace analytic_solvers
}  // namespace kinematics
}  // namespace arm_lib

#endif
