#ifndef ARM_LIB_KINEMATICS_IK_H
#define ARM_LIB_KINEMATICS_IK_H

#include "../arm_lib_config.h"
#include "../adapter/toolbox_adapter.h"
#include "../core/arm_status.h"
#include "../model/serial_chain.h"
#include "pose_error.h"

namespace arm_lib {
namespace kinematics {

enum class IkMethod : uint8_t {
  kLevenbergMarquardt = 0,
  kNewtonRaphson = 1,
};

enum class IkSolveStrategy : uint8_t {
  kNumericOnly = 0,
  kAnalyticOnly,
  kAnalyticThenNumericRefine,
  kAnalyticThenNumericFallback,
  kHybridPreferred,
};

enum class AnalyticSolverPreset : uint8_t {
  kAuto = 0,
  kDisabled,
  kPlanar3R,
};

enum class IkSolutionPreference : uint8_t {
  kClosestToCurrent = 0,
  kClosestToReference,
  kMaxLimitMargin,
  kBestManipulability,
};

enum class IkProfile : uint8_t {
  kDefault = 0,
  kRobust,
  kFast,
  kPositionOnly,
  kEmbeddedSafe,
};

struct IKOptions {
  IkMethod method;
  IkSolveStrategy strategy;
  AnalyticSolverPreset analytic_solver_preset;
  IkSolutionPreference preference;
  uint16_t max_iterations;
  uint16_t max_retries;
  uint16_t max_seed_candidates;
  Scalar error_tolerance;
  Scalar step_tolerance;
  Scalar damping;
  Scalar damping_min;
  Scalar damping_max;
  Scalar damping_upscale;
  Scalar damping_downscale;
  Scalar retry_seed_offset;
  Scalar singularity_threshold;
  Scalar singularity_damping_scale;
  Scalar null_space_gain;
  Scalar null_space_max_step;
  Scalar secondary_gradient_step;
  Scalar joint_centering_gain;
  Scalar reference_bias_gain;
  Scalar manipulability_gain;
  Scalar singularity_avoidance_gain;
  TaskMap6 task_projection;
  Twist6 task_weights;
  bool enable_multi_start;
  bool enable_singularity_adaptive_damping;
  bool enable_null_space_control;
  bool enable_joint_centering_objective;
  bool enable_reference_objective;
  bool enable_manipulability_objective;
  bool enable_singularity_avoidance_objective;
  bool enable_task_projection;
  bool enable_task_weighting;
  bool clamp_to_limits_each_step;
  bool enable_solution_ranking;
  bool enable_workspace_precheck;
  bool enable_model_validation;
  bool enable_post_check;

  IKOptions()
      : method(IkMethod::kLevenbergMarquardt),
        strategy(IkSolveStrategy::kHybridPreferred),
        analytic_solver_preset(AnalyticSolverPreset::kAuto),
        preference(IkSolutionPreference::kClosestToCurrent),
        max_iterations(ARM_LIB_IK_MAX_ITER),
        max_retries(4U),
        max_seed_candidates(8U),
        error_tolerance(ARM_LIB_IK_ERROR_TOL),
        step_tolerance(ARM_LIB_IK_STEP_TOL),
        damping(ARM_LIB_LM_DAMPING),
        damping_min(1.0e-6f),
        damping_max(1.0e6f),
        damping_upscale(10.0f),
        damping_downscale(0.5f),
        retry_seed_offset(0.35f),
        singularity_threshold(1.0e-3f),
        singularity_damping_scale(10.0f),
        null_space_gain(1.0f),
        null_space_max_step(0.15f),
        secondary_gradient_step(1.0e-3f),
        joint_centering_gain(0.10f),
        reference_bias_gain(0.05f),
        manipulability_gain(0.0f),
        singularity_avoidance_gain(0.0f),
        task_projection(matrixf::eye<6, 6>()),
        task_weights(matrixf::zeros<6, 1>()),
        enable_multi_start(true),
        enable_singularity_adaptive_damping(true),
        enable_null_space_control(false),
        enable_joint_centering_objective(true),
        enable_reference_objective(false),
        enable_manipulability_objective(false),
        enable_singularity_avoidance_objective(false),
        enable_task_projection(false),
        enable_task_weighting(false),
        clamp_to_limits_each_step(true),
        enable_solution_ranking(true),
        enable_workspace_precheck(false),
        enable_model_validation(true),
        enable_post_check(true) {
    for (uint16_t i = 0; i < 6U; ++i) {
      task_weights[i][0] = 1.0f;
    }
  }
};

inline IKOptions make_ik_options(IkProfile profile = IkProfile::kDefault) {
  IKOptions options;

  switch (profile) {
    case IkProfile::kRobust:
      options.method = IkMethod::kLevenbergMarquardt;
      options.strategy = IkSolveStrategy::kHybridPreferred;
      options.max_iterations = 100U;
      options.max_retries = 6U;
      options.enable_multi_start = true;
      options.damping = 3.0e-3f;
      options.damping_min = 1.0e-6f;
      options.damping_max = 1.0e6f;
      options.enable_singularity_adaptive_damping = true;
      options.singularity_threshold = 5.0e-3f;
      options.singularity_damping_scale = 20.0f;
      options.enable_null_space_control = true;
      options.enable_joint_centering_objective = true;
      options.joint_centering_gain = 0.10f;
      options.enable_reference_objective = true;
      options.reference_bias_gain = 0.05f;
      options.enable_manipulability_objective = true;
      options.manipulability_gain = 0.01f;
      options.enable_singularity_avoidance_objective = true;
      options.singularity_avoidance_gain = 0.04f;
      break;

    case IkProfile::kFast:
      options.method = IkMethod::kLevenbergMarquardt;
      options.strategy = IkSolveStrategy::kNumericOnly;
      options.max_iterations = 32U;
      options.max_retries = 1U;
      options.max_seed_candidates = 3U;
      options.enable_multi_start = false;
      options.enable_null_space_control = false;
      options.enable_joint_centering_objective = false;
      options.enable_reference_objective = false;
      options.enable_manipulability_objective = false;
      options.enable_singularity_avoidance_objective = false;
      options.enable_solution_ranking = false;
      options.damping = 1.0e-3f;
      break;

    case IkProfile::kPositionOnly:
      options.method = IkMethod::kLevenbergMarquardt;
      options.strategy = IkSolveStrategy::kHybridPreferred;
      options.max_iterations = 60U;
      options.max_retries = 4U;
      options.enable_multi_start = true;
      options.enable_null_space_control = true;
      options.enable_joint_centering_objective = true;
      options.enable_reference_objective = true;
      options.reference_bias_gain = 0.05f;
      break;

    case IkProfile::kEmbeddedSafe:
      options.method = IkMethod::kLevenbergMarquardt;
      options.strategy = IkSolveStrategy::kNumericOnly;
      options.max_iterations = 48U;
      options.max_retries = 2U;
      options.max_seed_candidates = 4U;
      options.enable_multi_start = false;
      options.enable_solution_ranking = false;
      options.enable_null_space_control = true;
      options.enable_joint_centering_objective = true;
      options.enable_reference_objective = false;
      options.enable_manipulability_objective = false;
      options.enable_singularity_avoidance_objective = true;
      options.singularity_avoidance_gain = 0.02f;
      options.null_space_max_step = 0.08f;
      options.damping = 2.0e-3f;
      break;

    case IkProfile::kDefault:
    default:
      break;
  }

  return options;
}

template <int N>
struct IKRequest {
  Transform target;
  JointVec<N> seed;
  JointVec<N> current;
  JointVec<N> reference;
  bool use_seed;
  bool use_current;
  bool use_reference;

  IKRequest()
      : target(toolbox_adapter::identity_transform()),
        seed(toolbox_adapter::zero_joint_vec<N>()),
        current(toolbox_adapter::zero_joint_vec<N>()),
        reference(toolbox_adapter::zero_joint_vec<N>()),
        use_seed(false),
        use_current(false),
        use_reference(false) {}
};

template <int N>
struct IKResult {
  IkStatus status;
  JointVec<N> q;
  SolverStats stats;
  IkDiagnostics diagnostics;
  Scalar limit_violation;
  uint16_t retry_count;
  uint16_t candidate_solution_count;
  uint16_t selected_solution_index;
  Scalar singularity_metric;
  Scalar manipulability_metric;
  Scalar null_space_step_norm;
  bool used_analytic;
  bool used_numeric_fallback;
  bool used_numeric_refine;
  bool used_null_space_objective;

  IKResult()
      : status(IkStatus::kNoConvergence),
        q(toolbox_adapter::zero_joint_vec<N>()),
        stats(),
        diagnostics(),
        limit_violation(0.0f),
        retry_count(0U),
        candidate_solution_count(0U),
        selected_solution_index(0U),
        singularity_metric(0.0f),
        manipulability_metric(0.0f),
        null_space_step_norm(0.0f),
        used_analytic(false),
        used_numeric_fallback(false),
        used_numeric_refine(false),
        used_null_space_objective(false) {}
};

}  // namespace kinematics
}  // namespace arm_lib

#endif
