#ifndef ARM_LIB_KINEMATICS_IK_NUMERIC_H
#define ARM_LIB_KINEMATICS_IK_NUMERIC_H

#include "../solver/lm_solver.h"
#include "../solver/nr_solver.h"
#include "../solver/seed_policy.h"
#include "fk.h"
#include "ik.h"
#include "jacobian.h"
#include "ik_redundancy.h"

namespace arm_lib {
namespace kinematics {

template <int N>
inline IkStatus validate_ik_request(const SerialChain<N>& chain,
                                    const IKRequest<N>& request,
                                    IkDiagnostics* diagnostics) {
  (void)chain;
  if (diagnostics != nullptr) {
    diagnostics->stage = IkFailStage::kInputValidation;
  }

  const Vec3 target_translation = toolbox_adapter::translation_of(request.target);
  for (uint16_t i = 0; i < 3U; ++i) {
    const Scalar value = target_translation[i][0];
    if ((value != value) || (abs_scalar(value) > 1.0e30f)) {
      if (diagnostics != nullptr) {
        diagnostics->reason = IkFailureReason::kInvalidTarget;
      }
      return IkStatus::kInvalidTarget;
    }
  }

  if (request.use_seed && !solver::is_joint_vector_finite(request.seed)) {
    if (diagnostics != nullptr) {
      diagnostics->reason = IkFailureReason::kInvalidSeed;
    }
    return IkStatus::kInvalidSeed;
  }

  return IkStatus::kSuccess;
}

template <int N>
inline bool is_target_roughly_reachable(const SerialChain<N>& chain,
                                        const Transform& target) {
  Scalar radius_sum = 0.0f;
  for (uint16_t i = 0; i < N; ++i) {
    const Link& link = chain.link(i);
    radius_sum += abs_scalar(link.dh().a) + abs_scalar(link.dh().d);
  }

  const Vec3 p = toolbox_adapter::translation_of(target);
  const Scalar dist =
      sqrtf(p[0][0] * p[0][0] + p[1][0] * p[1][0] + p[2][0] * p[2][0]);
  return dist <= (radius_sum + 0.25f);
}

template <int N, int MaxCandidates = 12>
inline void build_seed_candidates(
    const SerialChain<N>& chain,
    const IKRequest<N>& request,
    const IKOptions& options,
    solver::SeedCandidateSet<N, MaxCandidates>* out) {
  if (out == nullptr) {
    return;
  }

  solver::append_standard_seed_candidates(
      chain,
      request.use_current ? &request.current : nullptr, request.use_current,
      request.use_reference ? &request.reference : nullptr, request.use_reference,
      request.use_seed ? &request.seed : nullptr, request.use_seed,
      out);

  if (options.enable_multi_start) {
    const JointVec<N> base = request.use_seed
                                 ? request.seed
                                 : (request.use_current ? request.current
                                                        : chain.zero_configuration());
    for (uint16_t i = 0;
         i < options.max_retries && out->count < MaxCandidates;
         ++i) {
      (void)solver::append_seed_candidate(
          out, solver::perturb_seed(chain, base, i, options.retry_seed_offset),
          solver::SeedSource::kPerturbation);
    }
  }

  solver::deduplicate_seed_candidates(chain, out);
  solver::rank_seed_candidates(
      chain,
      request.use_current ? &request.current : nullptr, request.use_current,
      request.use_reference ? &request.reference : nullptr, request.use_reference,
      solver::SeedRankingWeights(), out);
}

template <int N>
inline Scalar score_ik_solution(const SerialChain<N>& chain,
                                const JointVec<N>& q,
                                const IKRequest<N>& request,
                                const IKOptions& options,
                                bool from_analytic,
                                bool from_numeric) {
  (void)options;
  Scalar score = 0.0f;
  if (request.use_current) {
    score += solver::configuration_distance(chain, q, request.current);
  }
  if (request.use_reference) {
    score += 0.5f * solver::configuration_distance(chain, q, request.reference);
  }
  if (from_analytic) {
    score -= 0.05f;
  }
  if (from_numeric) {
    score += 0.02f;
  }
  return score;
}

inline Twist6 apply_task_projection(const Twist6& twist,
                                    const IKOptions& options) {
  if (!options.enable_task_projection) {
    return twist;
  }

  return options.task_projection * twist;
}

template <int N>
inline Jacobian6xN<N> apply_task_projection(const Jacobian6xN<N>& jacobian,
                                            const IKOptions& options) {
  if (!options.enable_task_projection) {
    return jacobian;
  }

  return options.task_projection * jacobian;
}

inline Twist6 apply_task_weights(const Twist6& twist,
                                 const IKOptions& options) {
  Twist6 weighted = apply_task_projection(twist, options);
  if (!options.enable_task_weighting) {
    return weighted;
  }

  for (uint16_t i = 0; i < 6U; ++i) {
    weighted[i][0] *= options.task_weights[i][0];
  }
  return weighted;
}

template <int N>
inline Jacobian6xN<N> apply_task_weights(const Jacobian6xN<N>& jacobian,
                                         const IKOptions& options) {
  Jacobian6xN<N> weighted = apply_task_projection(jacobian, options);
  if (!options.enable_task_weighting) {
    return weighted;
  }

  for (uint16_t row = 0; row < 6U; ++row) {
    const Scalar weight = options.task_weights[row][0];
    for (uint16_t col = 0; col < N; ++col) {
      weighted[row][col] *= weight;
    }
  }
  return weighted;
}

inline Scalar task_weighted_error_norm(const PoseError6& error,
                                       const IKOptions& options) {
  return apply_task_weights(error.twist, options).norm();
}

template <int N>
inline JointVec<N> joint_centering_gradient(const SerialChain<N>& chain,
                                            const JointVec<N>& q) {
  JointVec<N> gradient = toolbox_adapter::zero_joint_vec<N>();
  for (uint16_t i = 0; i < N; ++i) {
    const Link& link = chain.link(i);
    if (!chain_joint_is_movable(link.joint_type()) || !link.limit_enabled()) {
      continue;
    }

    const Scalar half_range = 0.5f * (link.upper_limit() - link.lower_limit());
    if (half_range <= ARM_LIB_EPSILON) {
      continue;
    }

    const Scalar midpoint = 0.5f * (link.lower_limit() + link.upper_limit());
    Scalar delta = midpoint - q[i][0];
    if (link.joint_type() == ChainJointType::kRevolute &&
        (link.upper_limit() - link.lower_limit()) >= (2.0f * ARM_LIB_PI)) {
      delta = angle_distance(q[i][0], midpoint);
    }
    gradient[i][0] = delta / (half_range * half_range);
  }
  return gradient;
}

template <int N>
inline JointVec<N> reference_tracking_gradient(const SerialChain<N>& chain,
                                               const JointVec<N>& q,
                                               const JointVec<N>& reference) {
  JointVec<N> gradient = toolbox_adapter::zero_joint_vec<N>();
  for (uint16_t i = 0; i < N; ++i) {
    if (!chain_joint_is_movable(chain.link(i).joint_type())) {
      continue;
    }

    Scalar delta = reference[i][0] - q[i][0];
    if (chain.link(i).joint_type() == ChainJointType::kRevolute) {
      delta = angle_distance(q[i][0], reference[i][0]);
    }
    gradient[i][0] = delta;
  }
  return gradient;
}

enum class SecondaryMetricMode : uint8_t {
  kManipulability = 0,
  kSingularity = 1,
};

template <int N>
inline Scalar evaluate_secondary_metric(const SerialChain<N>& chain,
                                        const JointVec<N>& q,
                                        const IKOptions& options,
                                        SecondaryMetricMode mode) {
  const Jacobian6xN<N> weighted_jacobian =
      apply_task_weights(jacobian(chain, q), options);
  return (mode == SecondaryMetricMode::kManipulability)
             ? manipulability_metric(weighted_jacobian)
             : singularity_metric(weighted_jacobian);
}

template <int N>
inline JointVec<N> numerical_secondary_metric_gradient(
    const SerialChain<N>& chain, const JointVec<N>& q,
    const IKOptions& options, SecondaryMetricMode mode) {
  JointVec<N> gradient = toolbox_adapter::zero_joint_vec<N>();
  const Scalar nominal_step =
      (options.secondary_gradient_step > ARM_LIB_EPSILON)
          ? options.secondary_gradient_step
          : 1.0e-3f;

  for (uint16_t i = 0; i < N; ++i) {
    if (!chain_joint_is_movable(chain.link(i).joint_type())) {
      continue;
    }

    JointVec<N> q_plus = q;
    JointVec<N> q_minus = q;
    q_plus[i][0] += nominal_step;
    q_minus[i][0] -= nominal_step;

    if (chain.link(i).limit_enabled()) {
      q_plus[i][0] =
          clamp_scalar(q_plus[i][0], chain.link(i).lower_limit(),
                       chain.link(i).upper_limit());
      q_minus[i][0] =
          clamp_scalar(q_minus[i][0], chain.link(i).lower_limit(),
                       chain.link(i).upper_limit());
    }

    const Scalar denominator = q_plus[i][0] - q_minus[i][0];
    if (abs_scalar(denominator) <= ARM_LIB_EPSILON) {
      continue;
    }

    const Scalar metric_plus =
        evaluate_secondary_metric(chain, q_plus, options, mode);
    const Scalar metric_minus =
        evaluate_secondary_metric(chain, q_minus, options, mode);
    gradient[i][0] = (metric_plus - metric_minus) / denominator;
  }

  return gradient;
}

template <int N>
inline bool secondary_objectives_enabled(const IKRequest<N>& request,
                                         const IKOptions& options) {
  if (!options.enable_null_space_control) {
    return false;
  }

  return options.enable_joint_centering_objective ||
         (options.enable_reference_objective && request.use_reference) ||
         (options.enable_manipulability_objective &&
          abs_scalar(options.manipulability_gain) > ARM_LIB_EPSILON) ||
         (options.enable_singularity_avoidance_objective &&
          abs_scalar(options.singularity_avoidance_gain) > ARM_LIB_EPSILON);
}

template <int N>
inline JointVec<N> compute_secondary_step(const SerialChain<N>& chain,
                                          const JointVec<N>& q,
                                          const IKRequest<N>& request,
                                          const IKOptions& options,
                                          const Jacobian6xN<N>& weighted_jacobian,
                                          Scalar damping,
                                          Scalar current_singularity,
                                          Scalar* step_norm_out,
                                          bool* used_out) {
  JointVec<N> zero = toolbox_adapter::zero_joint_vec<N>();
  JointDiag<N> null_space_projector = matrixf::eye<N, N>();
  if (step_norm_out != nullptr) {
    *step_norm_out = 0.0f;
  }
  if (used_out != nullptr) {
    *used_out = false;
  }

  if (!secondary_objectives_enabled(request, options)) {
    return zero;
  }

  JointVec<N> gradient = toolbox_adapter::zero_joint_vec<N>();
  if (options.enable_joint_centering_objective &&
      abs_scalar(options.joint_centering_gain) > ARM_LIB_EPSILON) {
    gradient += options.joint_centering_gain * joint_centering_gradient(chain, q);
  }

  if (options.enable_reference_objective && request.use_reference &&
      abs_scalar(options.reference_bias_gain) > ARM_LIB_EPSILON) {
    gradient += options.reference_bias_gain *
                reference_tracking_gradient(chain, q, request.reference);
  }

  if (options.enable_manipulability_objective &&
      abs_scalar(options.manipulability_gain) > ARM_LIB_EPSILON) {
    gradient += options.manipulability_gain *
                numerical_secondary_metric_gradient(
                    chain, q, options, SecondaryMetricMode::kManipulability);
  }

  if (options.enable_singularity_avoidance_objective &&
      abs_scalar(options.singularity_avoidance_gain) > ARM_LIB_EPSILON &&
      current_singularity < options.singularity_threshold) {
    const Scalar ratio =
        (options.singularity_threshold > ARM_LIB_EPSILON)
            ? ((options.singularity_threshold - current_singularity) /
               options.singularity_threshold)
            : 1.0f;
    gradient +=
        (options.singularity_avoidance_gain * (1.0f + ratio)) *
        numerical_secondary_metric_gradient(
            chain, q, options, SecondaryMetricMode::kSingularity);
  }

  if (!solver::is_joint_vector_finite(gradient) ||
      gradient.norm() <= ARM_LIB_EPSILON) {
    return zero;
  }

  if (!damped_null_space_projector_safe(weighted_jacobian, damping,
                                        &null_space_projector)) {
    return zero;
  }

  JointVec<N> step = null_space_projector * (options.null_space_gain * gradient);

  if (!solver::is_joint_vector_finite(step)) {
    return zero;
  }

  const Scalar step_norm = step.norm();
  if (step_norm <= ARM_LIB_EPSILON) {
    return zero;
  }

  if (options.null_space_max_step > ARM_LIB_EPSILON &&
      step_norm > options.null_space_max_step) {
    step *= options.null_space_max_step / step_norm;
  }

  if (step_norm_out != nullptr) {
    *step_norm_out = step.norm();
  }
  if (used_out != nullptr) {
    *used_out = true;
  }
  return step;
}

template <int N>
inline void finalize_ik_metrics(const SerialChain<N>& chain,
                                const JointVec<N>& q,
                                const IKOptions& options,
                                IKResult<N>* result) {
  if (result == nullptr) {
    return;
  }

  const Jacobian6xN<N> weighted_jacobian =
      apply_task_weights(jacobian(chain, q), options);
  result->singularity_metric = singularity_metric(weighted_jacobian);
  result->manipulability_metric = manipulability_metric(weighted_jacobian);
}

template <int N>
struct NumericIkState {
  Transform current_pose;
  PoseError6 pose_error;
  Twist6 weighted_error;
  Scalar weighted_error_norm;
  Jacobian6xN<N> raw_jacobian;
  Jacobian6xN<N> weighted_jacobian;
  Scalar singularity;
  Scalar manipulability;

  NumericIkState()
      : current_pose(toolbox_adapter::identity_transform()),
        pose_error(),
        weighted_error(toolbox_adapter::zero_twist()),
        weighted_error_norm(0.0f),
        raw_jacobian(matrixf::zeros<6, N>()),
        weighted_jacobian(matrixf::zeros<6, N>()),
        singularity(0.0f),
        manipulability(0.0f) {}
};

template <int N>
inline NumericIkState<N> evaluate_numeric_ik_state(
    const SerialChain<N>& chain, const Transform& target,
    const JointVec<N>& q, const IKOptions& options) {
  NumericIkState<N> state;
  state.current_pose = fk(chain, q);
  state.pose_error = evaluate_pose_error(target, state.current_pose);
  state.weighted_error = apply_task_weights(state.pose_error.twist, options);
  state.weighted_error_norm = state.weighted_error.norm();
  state.raw_jacobian = jacobian(chain, q);
  state.weighted_jacobian = apply_task_weights(state.raw_jacobian, options);
  state.singularity = singularity_metric(state.weighted_jacobian);
  state.manipulability = manipulability_metric(state.weighted_jacobian);
  return state;
}

template <int N>
inline Scalar evaluate_numeric_ik_error(const SerialChain<N>& chain,
                                        const Transform& target,
                                        const JointVec<N>& q,
                                        const IKOptions& options) {
  return evaluate_numeric_ik_state(chain, target, q, options).weighted_error_norm;
}

template <int N>
inline void fill_numeric_ik_diagnostics(const IKOptions& options,
                                        IKResult<N>* result) {
  if (result == nullptr) {
    return;
  }

  result->diagnostics.iterations = result->stats.iterations;
  result->diagnostics.final_error_norm = result->stats.final_error_norm;
  result->diagnostics.final_step_norm = result->stats.final_step_norm;
  result->diagnostics.singularity_metric = result->singularity_metric;
  result->diagnostics.manipulability_metric = result->manipulability_metric;
  result->diagnostics.limit_violation = result->limit_violation;
  result->diagnostics.used_analytic = result->used_analytic;
  result->diagnostics.used_numeric_fallback = result->used_numeric_fallback;
  result->diagnostics.used_numeric_refine = result->used_numeric_refine;
  result->diagnostics.task_projection_applied = options.enable_task_projection;
  result->diagnostics.task_weighting_applied = options.enable_task_weighting;
  result->diagnostics.clamped_to_limits = options.clamp_to_limits_each_step;
}

template <int N>
inline IKResult<N> solve_ik_numeric_once(const SerialChain<N>& chain,
                                         const IKRequest<N>& request,
                                         const IKOptions& options = IKOptions()) {
  IKResult<N> result;
  JointVec<N> q = solver::select_initial_seed(chain, request.seed,
                                              request.use_seed);
  if (request.use_reference) {
    q = solver::align_revolute_to_reference(chain, q, request.reference);
  }
  if (options.clamp_to_limits_each_step) {
    q = solver::clamp_to_chain_limits(chain, q);
  }

  Scalar damping = options.damping;

  for (uint16_t iter = 0; iter < options.max_iterations; ++iter) {
    const NumericIkState<N> state =
        evaluate_numeric_ik_state(chain, request.target, q, options);

    result.stats.iterations = static_cast<uint16_t>(iter + 1U);
    result.stats.final_error_norm = state.weighted_error_norm;

    if (state.weighted_error_norm <= options.error_tolerance) {
      result.status = IkStatus::kSuccess;
      result.q = q;
      result.limit_violation = total_limit_violation(chain.joint_limits(), q);
      finalize_ik_metrics(chain, q, options, &result);
      return result;
    }

    result.singularity_metric = state.singularity;
    result.manipulability_metric = state.manipulability;

    JointVec<N> dq = toolbox_adapter::zero_joint_vec<N>();
    Scalar effective_damping = damping;
    if (options.enable_singularity_adaptive_damping &&
        state.singularity < options.singularity_threshold) {
      const Scalar ratio =
          (options.singularity_threshold > ARM_LIB_EPSILON)
              ? ((options.singularity_threshold - state.singularity) /
                 options.singularity_threshold)
              : 1.0f;
      effective_damping =
          clamp_scalar(damping * (1.0f + options.singularity_damping_scale * ratio),
                       options.damping_min, options.damping_max);
    }

    if (options.method == IkMethod::kNewtonRaphson) {
      if (!solver::nr_delta_safe<N>(state.weighted_jacobian,
                                    state.weighted_error,
                                    effective_damping,
                                    &dq)) {
        result.status = IkStatus::kNumericalFailure;
        result.diagnostics.stage = IkFailStage::kNumericSolve;
        result.diagnostics.reason = IkFailureReason::kNumericalFailure;
        result.q = q;
        result.limit_violation = total_limit_violation(chain.joint_limits(), q);
        finalize_ik_metrics(chain, q, options, &result);
        return result;
      }
    } else {
      if (!solver::lm_delta_safe<N>(state.weighted_jacobian,
                                    state.weighted_error,
                                    effective_damping,
                                    &dq)) {
        result.status = IkStatus::kNumericalFailure;
        result.diagnostics.stage = IkFailStage::kNumericSolve;
        result.diagnostics.reason = IkFailureReason::kNumericalFailure;
        result.q = q;
        result.limit_violation = total_limit_violation(chain.joint_limits(), q);
        finalize_ik_metrics(chain, q, options, &result);
        return result;
      }
    }

    Scalar secondary_step_norm = 0.0f;
    bool used_secondary = false;
    const JointVec<N> dq_secondary = compute_secondary_step(
        chain, q, request, options, state.weighted_jacobian, effective_damping,
        state.singularity, &secondary_step_norm, &used_secondary);
    result.null_space_step_norm = secondary_step_norm;
    result.used_null_space_objective =
        result.used_null_space_objective || used_secondary;

    const Scalar step_norm = (dq + dq_secondary).norm();
    result.stats.final_step_norm = step_norm;

    if (step_norm <= options.step_tolerance) {
      result.status = IkStatus::kNoConvergence;
      result.diagnostics.stage = IkFailStage::kNumericSolve;
      result.diagnostics.reason = IkFailureReason::kNoConvergence;
      result.q = q;
      result.limit_violation = total_limit_violation(chain.joint_limits(), q);
      finalize_ik_metrics(chain, q, options, &result);
      return result;
    }

    JointVec<N> q_candidate = q + dq + dq_secondary;
    q_candidate = solver::align_revolute_to_reference(chain, q_candidate, q);
    if (request.use_reference) {
      q_candidate = solver::align_revolute_to_reference(
          chain, q_candidate, request.reference);
    }
    if (options.clamp_to_limits_each_step) {
      q_candidate = solver::clamp_to_chain_limits(chain, q_candidate);
    }

    if (options.method == IkMethod::kLevenbergMarquardt) {
      const Scalar candidate_error =
          evaluate_numeric_ik_error(chain, request.target, q_candidate, options);
      const Scalar acceptance_slack = 0.25f * options.error_tolerance;

      JointVec<N> accepted_candidate = q_candidate;
      Scalar accepted_error = candidate_error;
      if (candidate_error > (state.weighted_error_norm + acceptance_slack) &&
          used_secondary) {
        JointVec<N> primary_only_candidate = q + dq;
        primary_only_candidate =
            solver::align_revolute_to_reference(chain, primary_only_candidate, q);
        if (request.use_reference) {
          primary_only_candidate = solver::align_revolute_to_reference(
              chain, primary_only_candidate, request.reference);
        }
        if (options.clamp_to_limits_each_step) {
          primary_only_candidate =
              solver::clamp_to_chain_limits(chain, primary_only_candidate);
        }

        const Scalar primary_only_error = evaluate_numeric_ik_error(
            chain, request.target, primary_only_candidate, options);
        if (primary_only_error <= (state.weighted_error_norm + acceptance_slack)) {
          accepted_candidate = primary_only_candidate;
          accepted_error = primary_only_error;
          result.null_space_step_norm = 0.0f;
        }
      }

      if (accepted_error <= (state.weighted_error_norm + acceptance_slack)) {
        q = accepted_candidate;
        damping =
            clamp_scalar(damping * options.damping_downscale,
                         options.damping_min,
                         options.damping_max);
      } else {
        damping =
            clamp_scalar(damping * options.damping_upscale,
                         options.damping_min,
                         options.damping_max);
      }
    } else {
      q = q_candidate;
    }
  }

  result.status = IkStatus::kMaxIterations;
  result.diagnostics.stage = IkFailStage::kNumericSolve;
  result.diagnostics.reason = IkFailureReason::kMaxIterations;
  result.q = q;
  result.limit_violation = total_limit_violation(chain.joint_limits(), q);
  finalize_ik_metrics(chain, q, options, &result);
  return result;
}

template <int N>
inline IKResult<N> solve_ik_numeric(const SerialChain<N>& chain,
                                    const IKRequest<N>& request,
                                    const IKOptions& options = IKOptions()) {
  IKResult<N> validation_result;
  validation_result.status =
      validate_ik_request(chain, request, &validation_result.diagnostics);
  if (!is_success(validation_result.status)) {
    return validation_result;
  }

  if (options.enable_workspace_precheck &&
      !is_target_roughly_reachable(chain, request.target)) {
    validation_result.status = IkStatus::kUnreachable;
    validation_result.diagnostics.stage = IkFailStage::kWorkspacePrecheck;
    validation_result.diagnostics.reason = IkFailureReason::kUnreachable;
    return validation_result;
  }

  if (options.enable_model_validation && !chain.validate()) {
    validation_result.status = IkStatus::kInvalidModel;
    validation_result.diagnostics.stage = IkFailStage::kModelValidation;
    validation_result.diagnostics.reason = IkFailureReason::kInvalidModel;
    return validation_result;
  }

  if (!options.enable_multi_start || options.max_retries == 0U) {
    IKResult<N> result = solve_ik_numeric_once(chain, request, options);
    fill_numeric_ik_diagnostics(options, &result);
    return result;
  }

  IKResult<N> best_result;
  bool found_any = false;
  bool found_success = false;
  Scalar best_cost = 0.0f;

  solver::SeedCandidateSet<N, 12> seeds;
  build_seed_candidates(chain, request, options, &seeds);
  best_result.diagnostics.stage = IkFailStage::kSeedSelection;
  if (seeds.count == 0U) {
    best_result.status = IkStatus::kNoConvergence;
    best_result.diagnostics.reason = IkFailureReason::kNoCandidateSeed;
    return best_result;
  }

  for (uint16_t attempt = 0; attempt < seeds.count; ++attempt) {
    if (!seeds.items[attempt].valid) {
      continue;
    }

    IKRequest<N> attempt_request = request;
    attempt_request.seed = seeds.items[attempt].q;
    attempt_request.use_seed = true;

    IKResult<N> attempt_result =
        solve_ik_numeric_once(chain, attempt_request, options);
    attempt_result.retry_count = static_cast<uint16_t>(attempt + 1U);
    attempt_result.candidate_solution_count = seeds.count;
    attempt_result.selected_solution_index = attempt;
    attempt_result.diagnostics.stage = IkFailStage::kNumericSolve;
    attempt_result.diagnostics.seed_count_tried = static_cast<uint16_t>(attempt + 1U);
    fill_numeric_ik_diagnostics(options, &attempt_result);

    const bool success = is_success(attempt_result.status);
    const Scalar cost =
        attempt_result.stats.final_error_norm +
        attempt_result.limit_violation * 1000.0f +
        score_ik_solution(chain, attempt_result.q, request, options, false, true) *
            1.0e-2f -
        attempt_result.manipulability_metric * 1.0e-3f;

    if (!found_any || (success && !found_success) ||
        (success == found_success && cost < best_cost)) {
      found_any = true;
      found_success = success;
      best_cost = cost;
      best_result = attempt_result;
    }

    if (success) {
      break;
    }
  }

  if (!found_any) {
    best_result.status = IkStatus::kNoConvergence;
    best_result.diagnostics.stage = IkFailStage::kNumericSolve;
    best_result.diagnostics.reason = IkFailureReason::kNoConvergence;
  }

  if (!is_success(best_result.status) &&
      best_result.diagnostics.reason == IkFailureReason::kNone) {
    best_result.diagnostics.reason =
        (best_result.status == IkStatus::kNumericalFailure)
            ? IkFailureReason::kNumericalFailure
            : (best_result.status == IkStatus::kMaxIterations)
                  ? IkFailureReason::kMaxIterations
                  : IkFailureReason::kNoConvergence;
  }

  return best_result;
}

}  // namespace kinematics
}  // namespace arm_lib

#endif
