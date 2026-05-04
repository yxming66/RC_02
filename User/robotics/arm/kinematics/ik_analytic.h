#ifndef ARM_LIB_KINEMATICS_IK_ANALYTIC_H
#define ARM_LIB_KINEMATICS_IK_ANALYTIC_H

#include "fk.h"
#include "ik.h"
#include "ik_solution_scoring.h"
#include "ik_numeric.h"
#include "../solver/seed_policy.h"

namespace mr::robotics::arm {
namespace kinematics {

enum class AnalyticIkPluginKind : uint8_t {
  kUnknown = 0,
  kPlanar2R,
  kPlanar3R,
  kScara,
  kPieper6R,
  kCustom,
};

struct AnalyticIkPluginInfo {
  AnalyticIkPluginKind kind;
  const char* name;
  uint8_t priority;

  AnalyticIkPluginInfo()
      : kind(AnalyticIkPluginKind::kUnknown),
        name("unknown"),
        priority(0U) {}
};

template <int N>
struct AnalyticIKSolutionSet {
  JointVec<N> q[ARM_LIB_ANALYTIC_IK_MAX_SOLUTIONS];
  uint16_t count;

  AnalyticIKSolutionSet() : count(0) {}
};

template <int N>
using AnalyticIKGenerateFn = bool (*)(const SerialChain<N>& chain,
                                      const IKRequest<N>& request,
                                      AnalyticIKSolutionSet<N>* out);

template <int N>
using AnalyticIKCompatibleFn = bool (*)(const SerialChain<N>& chain);

template <int N>
struct AnalyticIKPlugin {
  AnalyticIkPluginInfo info;
  const char* name;
  AnalyticIKCompatibleFn<N> is_compatible;
  AnalyticIKGenerateFn<N> generate;

  AnalyticIKPlugin()
      : info(), name(nullptr), is_compatible(nullptr), generate(nullptr) {}
};

template <int N, int MaxPlugins = 6>
struct AnalyticIKRegistry {
  AnalyticIKPlugin<N> plugins[MaxPlugins];
  uint16_t count;

  AnalyticIKRegistry() : count(0U) {}
};

template <int N, int MaxPlugins = 6>
inline bool register_analytic_plugin(const AnalyticIKPlugin<N>& plugin,
                                     AnalyticIKRegistry<N, MaxPlugins>* registry) {
  if (registry == nullptr || registry->count >= MaxPlugins) {
    return false;
  }

  registry->plugins[registry->count] = plugin;
  ++registry->count;
  return true;
}

template <int N, int MaxPlugins = 6>
inline bool solve_with_registered_analytic_plugins(
    const SerialChain<N>& chain,
    const IKRequest<N>& request,
    const AnalyticIKRegistry<N, MaxPlugins>& registry,
    AnalyticIKSolutionSet<N>* out,
    AnalyticIkPluginInfo* selected_plugin_info) {
  if (out == nullptr) {
    return false;
  }

  out->count = 0U;
  for (uint16_t i = 0; i < registry.count; ++i) {
    const AnalyticIKPlugin<N>& plugin = registry.plugins[i];
    if (plugin.is_compatible == nullptr || plugin.generate == nullptr) {
      continue;
    }
    if (!plugin.is_compatible(chain)) {
      continue;
    }
    if (!plugin.generate(chain, request, out)) {
      continue;
    }
    if (selected_plugin_info != nullptr) {
      *selected_plugin_info = plugin.info;
    }
    return out->count > 0U;
  }

  return false;
}

struct HybridIKOptions {
  Scalar analytic_verification_tolerance;
  Scalar analytic_limit_tolerance;
  Scalar analytic_reference_weight;
  Scalar branch_switch_hysteresis;
  bool enable_numeric_fallback;
  bool fallback_on_limit_violation;
  bool refine_analytic_solution;
  IKOptions numeric_options;

  HybridIKOptions()
      : analytic_verification_tolerance(ARM_LIB_IK_ERROR_TOL),
        analytic_limit_tolerance(1.0e-5f),
        analytic_reference_weight(1.0f),
        branch_switch_hysteresis(1.0e-4f),
        enable_numeric_fallback(true),
        fallback_on_limit_violation(true),
        refine_analytic_solution(true),
        numeric_options() {}
};

template <int N>
inline IKResult<N> solve_ik_analytic(
    const SerialChain<N>& chain,
    const IKRequest<N>& request,
    const AnalyticIKPlugin<N>& plugin,
    Scalar verification_tolerance = ARM_LIB_IK_ERROR_TOL,
    Scalar limit_tolerance = 1.0e-5f,
    Scalar reference_weight = 1.0f,
    Scalar branch_switch_hysteresis = 0.0f,
    const IKOptions* verification_options = nullptr) {
  IKResult<N> result;

  auto weighted_pose_error_norm = [](const PoseError6& pose_error,
                                     const IKOptions& ik_options) {
    Twist6 weighted = pose_error.twist;
    if (ik_options.enable_task_projection) {
      weighted = ik_options.task_projection * weighted;
    }
    if (ik_options.enable_task_weighting) {
      for (uint16_t i = 0; i < 6U; ++i) {
        weighted[i][0] *= ik_options.task_weights[i][0];
      }
    }
    return weighted.norm();
  };

  if (plugin.is_compatible == nullptr || plugin.generate == nullptr) {
    result.status = IkStatus::kUnsupported;
    result.diagnostics.stage = IkFailStage::kAnalyticSolve;
    result.diagnostics.reason = IkFailureReason::kNoAnalyticPlugin;
    return result;
  }
  if (!plugin.is_compatible(chain)) {
    result.status = IkStatus::kUnsupported;
    result.diagnostics.stage = IkFailStage::kAnalyticSolve;
    result.diagnostics.reason = IkFailureReason::kAnalyticNotApplicable;
    return result;
  }

  AnalyticIKSolutionSet<N> candidates;
  if (!plugin.generate(chain, request, &candidates) || candidates.count == 0U) {
    result.status = IkStatus::kInvalidTarget;
    result.diagnostics.stage = IkFailStage::kAnalyticSolve;
    result.diagnostics.reason = IkFailureReason::kAnalyticNoSolution;
    return result;
  }
  result.diagnostics.analytic_solution_count = candidates.count;

  const JointVec<N> reference =
      request.use_reference
          ? request.reference
          : (request.use_seed ? request.seed : chain.zero_configuration());
  const bool has_reference = request.use_reference || request.use_seed;
  const JointLimits<N> limits = chain.joint_limits();

  bool found_verified = false;
  bool found_within_limits = false;
  Scalar best_cost = 0.0f;
  Scalar best_error = 0.0f;
  Scalar best_limit_violation = 0.0f;
  Scalar best_singularity_metric = 0.0f;
  const IkCandidateScoreWeights score_weights =
      make_ik_candidate_score_weights(
          reference_weight,
          verification_options != nullptr
              ? verification_options->singularity_threshold
              : IKOptions().singularity_threshold);

  for (uint16_t i = 0; i < candidates.count; ++i) {
    JointVec<N> q_candidate = candidates.q[i];
    if (has_reference) {
      q_candidate = solver::align_revolute_to_reference(chain, q_candidate,
                                                        reference);
    }
    if (verification_options != nullptr &&
        verification_options->enable_solution_step_limit &&
        has_reference &&
        !ik_solution_step_within_limit(
            chain, reference, q_candidate,
            verification_options->max_solution_joint_step)) {
      continue;
    }

    const Scalar limit_violation = total_limit_violation(limits, q_candidate);
    const PoseError6 pose_error =
        evaluate_pose_error(request.target, fk(chain, q_candidate));
    const Scalar error_norm =
        (verification_options != nullptr)
            ? weighted_pose_error_norm(pose_error, *verification_options)
            : pose_error.total_norm();
    if (error_norm > verification_tolerance) {
      continue;
    }

    const bool within_limits = (limit_violation <= limit_tolerance);
    const IkCandidateScore candidate_score = score_ik_candidate(
        chain, q_candidate, reference, has_reference, limits, score_weights);
    const Scalar cost = candidate_score.total;

    if (!found_verified ||
        (within_limits && !found_within_limits) ||
        (within_limits == found_within_limits &&
         cost < (best_cost - branch_switch_hysteresis))) {
      found_verified = true;
      found_within_limits = within_limits;
      best_cost = cost;
      best_error = error_norm;
      best_limit_violation = limit_violation;
      best_singularity_metric = candidate_score.singularity_metric;
      result.q = q_candidate;
    }
  }

  if (!found_verified) {
    result.status = IkStatus::kInvalidTarget;
    result.diagnostics.stage = IkFailStage::kPostCheck;
    result.diagnostics.reason = IkFailureReason::kAnalyticNoSolution;
    return result;
  }

  result.stats.iterations = 1U;
  result.stats.final_error_norm = best_error;
  result.stats.final_step_norm = 0.0f;
  result.diagnostics.iterations = result.stats.iterations;
  result.diagnostics.final_error_norm = result.stats.final_error_norm;
  result.diagnostics.final_step_norm = result.stats.final_step_norm;
  result.limit_violation = best_limit_violation;
  result.singularity_metric = best_singularity_metric;
  result.diagnostics.limit_violation = result.limit_violation;
  result.diagnostics.singularity_metric = result.singularity_metric;
  result.used_analytic = true;
  result.diagnostics.used_analytic = true;
  result.status =
      found_within_limits ? IkStatus::kSuccess : IkStatus::kLimitViolation;
  return result;
}

template <int N>
inline IKResult<N> solve_ik_hybrid(const SerialChain<N>& chain,
                                   const IKRequest<N>& request,
                                   const AnalyticIKPlugin<N>& plugin,
                                   const HybridIKOptions& options =
                                       HybridIKOptions()) {
  IKResult<N> analytic_result = solve_ik_analytic(
      chain, request, plugin, options.analytic_verification_tolerance,
      options.analytic_limit_tolerance, options.analytic_reference_weight,
      options.branch_switch_hysteresis, &options.numeric_options);

  const bool analytic_success = is_success(analytic_result.status);
  const bool analytic_limit_only =
      analytic_result.status == IkStatus::kLimitViolation;

  if (analytic_success && !options.refine_analytic_solution) {
    return analytic_result;
  }

  if (analytic_success || (analytic_limit_only && options.fallback_on_limit_violation)) {
    IKRequest<N> refine_request = request;
    refine_request.seed = analytic_result.q;
    refine_request.use_seed = true;
    if (!refine_request.use_reference) {
      refine_request.reference = analytic_result.q;
      refine_request.use_reference = true;
    }

    IKOptions refine_options = options.numeric_options;
    refine_options.enable_multi_start = false;
    refine_options.max_retries = 1U;
    IKResult<N> refined_result =
        solve_ik_numeric(chain, refine_request, refine_options);
    if (is_success(refined_result.status)) {
      refined_result.used_numeric_refine = true;
      return refined_result;
    }
    if (analytic_success) {
      return analytic_result;
    }
  }

  if (!options.enable_numeric_fallback) {
    return analytic_result;
  }

  IKRequest<N> fallback_request = request;
  if (solver::is_joint_vector_finite(analytic_result.q)) {
    fallback_request.seed = analytic_result.q;
    fallback_request.use_seed = true;
  }

  IKResult<N> fallback_result =
      solve_ik_numeric(chain, fallback_request, options.numeric_options);
  if (is_success(fallback_result.status)) {
    fallback_result.used_numeric_fallback = true;
    return fallback_result;
  }

  return analytic_result;
}

}  // namespace kinematics
}  // namespace mr::robotics::arm

#endif
