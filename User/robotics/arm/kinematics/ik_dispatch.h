#ifndef ARM_LIB_KINEMATICS_IK_DISPATCH_H
#define ARM_LIB_KINEMATICS_IK_DISPATCH_H

#include "ik_numeric.h"
#include "ik_analytic.h"
#include "analytic_solvers/solver_registry.h"

namespace mr::robotics::arm {
namespace kinematics {

template <int N>
inline IKResult<N> solve_ik_dispatch(const SerialChain<N>& chain,
                                     const IKRequest<N>& request,
                                     const IKOptions& options = IKOptions()) {
  if (options.strategy == IkSolveStrategy::kNumericOnly) {
    return solve_ik_numeric(chain, request, options);
  }

  const AnalyticIKRegistry<N, 6> registry =
      analytic_solvers::build_registry<N, 6>(options.analytic_solver_preset);
  if (registry.count == 0U) {
    if (options.strategy == IkSolveStrategy::kAnalyticOnly) {
      IKResult<N> result;
      result.status = IkStatus::kUnsupported;
      result.diagnostics.stage = IkFailStage::kAnalyticSolve;
      result.diagnostics.reason = IkFailureReason::kNoAnalyticPlugin;
      return result;
    }
    return solve_ik_numeric(chain, request, options);
  }

  const AnalyticIKPlugin<N>& plugin = registry.plugins[0];

  if (options.strategy == IkSolveStrategy::kAnalyticOnly) {
    return solve_ik_analytic(chain, request, plugin, options.error_tolerance,
                             1.0e-5f, 1.0f, 0.0f, &options);
  }

  HybridIKOptions hybrid_options;
  hybrid_options.numeric_options = options;
  hybrid_options.enable_numeric_fallback =
      options.strategy != IkSolveStrategy::kAnalyticThenNumericRefine;
  hybrid_options.refine_analytic_solution =
      options.strategy != IkSolveStrategy::kAnalyticThenNumericFallback;
  return solve_ik_hybrid(chain, request, plugin, hybrid_options);
}

template <int N>
inline IKResult<N> solve_ik(const SerialChain<N>& chain,
                            const IKRequest<N>& request,
                            const IKOptions& options = IKOptions()) {
  return solve_ik_dispatch(chain, request, options);
}

}  // namespace kinematics
}  // namespace mr::robotics::arm

#endif
