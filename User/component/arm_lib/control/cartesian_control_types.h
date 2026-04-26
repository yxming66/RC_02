#ifndef ARM_LIB_CONTROL_CARTESIAN_CONTROL_TYPES_H
#define ARM_LIB_CONTROL_CARTESIAN_CONTROL_TYPES_H

#include "../adapter/toolbox_adapter.h"
#include "../core/arm_status.h"
#include "../kinematics/task_constraints.h"

namespace arm_lib {
namespace control {

enum class CartesianServoLimitReason : uint8_t {
  kNone = 0,
  kLinearVelocity,
  kAngularVelocity,
  kLinearAcceleration,
  kAngularAcceleration,
  kSingularityProtection,
  kJointStepLimit,
  kIkFailure,
};

struct CartesianServoDiagnostics {
  CartesianServoLimitReason limit_reason;
  Scalar velocity_scale;
  Scalar singularity_metric;
  bool orientation_relaxed;
  bool used_joint_servo_limit;
  bool ik_failed;
  IkStatus ik_status;

  CartesianServoDiagnostics()
      : limit_reason(CartesianServoLimitReason::kNone),
        velocity_scale(1.0f),
        singularity_metric(0.0f),
        orientation_relaxed(false),
        used_joint_servo_limit(false),
        ik_failed(false),
        ik_status(IkStatus::kSuccess) {}
};

// Target frame answers: "How should the commanded delta be interpreted?"
enum class CartesianTargetFrame : uint8_t {
  kBase = 0,
  kToolLocal = 1,
};

// Error frame answers: "In which frame should tracking error be measured?"
using CartesianErrorFrame = kinematics::PoseErrorFrame;

// Orientation mode answers: "Which parts of attitude must actually be tracked?"
using CartesianOrientationMode = kinematics::OrientationConstraintMode;

struct CartesianTargetRequest {
  Transform pose;
  bool is_delta;
  CartesianTargetFrame frame;

  CartesianTargetRequest()
      : pose(toolbox_adapter::identity_transform()),
        is_delta(false),
        frame(CartesianTargetFrame::kBase) {}

  static CartesianTargetRequest absolute_pose(const Transform& target_pose) {
    CartesianTargetRequest request;
    request.pose = target_pose;
    request.is_delta = false;
    request.frame = CartesianTargetFrame::kBase;
    return request;
  }

  static CartesianTargetRequest base_delta(const Transform& delta_pose) {
    CartesianTargetRequest request;
    request.pose = delta_pose;
    request.is_delta = true;
    request.frame = CartesianTargetFrame::kBase;
    return request;
  }

  static CartesianTargetRequest tool_delta(const Transform& delta_pose) {
    CartesianTargetRequest request;
    request.pose = delta_pose;
    request.is_delta = true;
    request.frame = CartesianTargetFrame::kToolLocal;
    return request;
  }

  static CartesianTargetRequest delta(const Transform& delta_pose,
                                      CartesianTargetFrame target_frame) {
    CartesianTargetRequest request;
    request.pose = delta_pose;
    request.is_delta = true;
    request.frame = target_frame;
    return request;
  }
};

}  // namespace control
}  // namespace arm_lib

#endif
