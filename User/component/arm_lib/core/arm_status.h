#ifndef ARM_LIB_CORE_ARM_STATUS_H
#define ARM_LIB_CORE_ARM_STATUS_H

#include <stdint.h>

#include "arm_types.h"

namespace arm_lib {

enum class ArmStatus : uint8_t {
  kOk = 0,
  kInvalidArgument,
  kOutOfRange,
  kLimitViolation,
  kSingular,
  kMaxIterations,
  kNumericalFailure,
  kUnsupported,
  kUninitialized,
};

enum class IkStatus : uint8_t {
  kSuccess = 0,
  kInvalidTarget,
  kInvalidSeed,
  kInvalidModel,
  kUnreachable,
  kConstraintConflict,
  kLimitViolation,
  kSingular,
  kMaxIterations,
  kNumericalFailure,
  kNoConvergence,
  kUnsupported,
};

enum class IkFailStage : uint8_t {
  kNone = 0,
  kInputValidation,
  kModelValidation,
  kWorkspacePrecheck,
  kSeedSelection,
  kAnalyticSolve,
  kNumericSolve,
  kSolutionRanking,
  kPostCheck,
};

enum class IkFailureReason : uint8_t {
  kNone = 0,
  kInvalidTarget,
  kInvalidSeed,
  kInvalidModel,
  kUnreachable,
  kConstraintConflict,
  kLimitViolation,
  kSingular,
  kMaxIterations,
  kNumericalFailure,
  kNoConvergence,
  kNoCandidateSeed,
  kNoAnalyticPlugin,
  kAnalyticNotApplicable,
  kAnalyticNoSolution,
};

struct SolverStats {
  uint16_t iterations;
  Scalar final_error_norm;
  Scalar final_step_norm;

  SolverStats() : iterations(0), final_error_norm(0.0f), final_step_norm(0.0f) {}
};

struct IkDiagnostics {
  IkFailStage stage;
  IkFailureReason reason;
  uint16_t iterations;
  uint16_t seed_count_tried;
  uint16_t analytic_solution_count;
  Scalar final_error_norm;
  Scalar final_step_norm;
  Scalar singularity_metric;
  Scalar manipulability_metric;
  Scalar limit_violation;
  bool used_analytic;
  bool used_numeric_fallback;
  bool used_numeric_refine;
  bool task_projection_applied;
  bool task_weighting_applied;
  bool orientation_relaxed;
  bool clamped_to_limits;

  IkDiagnostics()
      : stage(IkFailStage::kNone),
        reason(IkFailureReason::kNone),
        iterations(0),
        seed_count_tried(0),
        analytic_solution_count(0),
        final_error_norm(0.0f),
        final_step_norm(0.0f),
        singularity_metric(0.0f),
        manipulability_metric(0.0f),
        limit_violation(0.0f),
        used_analytic(false),
        used_numeric_fallback(false),
        used_numeric_refine(false),
        task_projection_applied(false),
        task_weighting_applied(false),
        orientation_relaxed(false),
        clamped_to_limits(false) {}
};

inline bool is_ok(ArmStatus status) {
  return status == ArmStatus::kOk;
}

inline bool is_success(IkStatus status) {
  return status == IkStatus::kSuccess;
}

inline bool is_retryable_ik_failure(IkStatus status) {
  return status == IkStatus::kSingular ||
         status == IkStatus::kMaxIterations ||
         status == IkStatus::kNumericalFailure ||
         status == IkStatus::kNoConvergence;
}

inline bool is_model_related_ik_failure(IkStatus status) {
  return status == IkStatus::kInvalidModel ||
         status == IkStatus::kUnsupported;
}

}  // namespace arm_lib

#endif
