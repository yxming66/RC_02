#ifndef ARM_LIB_APPLICATION_THREE_PIT_CARTESIAN_APP_H
#define ARM_LIB_APPLICATION_THREE_PIT_CARTESIAN_APP_H

#include <stdint.h>

#include "../adapter/toolbox_adapter.h"
#include "../core/arm_command.h"
#include "../core/arm_limits.h"
#include "../kinematics/ik.h"
#include "../kinematics/ik_dispatch.h"
#include "../safety/joint_command_safety.h"

namespace mr::robotics::arm {
namespace application {

constexpr int kThreePitCartesianDof = 3;

struct YzPitchPose {
  Scalar y;
  Scalar z;
  Scalar pitch;

  YzPitchPose() : y(0.0f), z(0.0f), pitch(0.0f) {}
};

struct ThreePitWorkspaceLimits {
  Scalar y_min;
  Scalar y_max;
  Scalar z_min;
  Scalar z_max;
  Scalar pitch_min;
  Scalar pitch_max;
  bool check_y;
  bool check_z;
  bool check_pitch;

  ThreePitWorkspaceLimits()
      : y_min(-1.0f),
        y_max(1.0f),
        z_min(-0.8f),
        z_max(1.2f),
        pitch_min(-ARM_LIB_PI),
        pitch_max(ARM_LIB_PI),
        check_y(true),
        check_z(true),
        check_pitch(true) {}
};

struct ThreePitRemoteInput {
  Scalar y_axis;
  Scalar z_axis;
  Scalar pitch_axis;
  bool enabled;
  bool reset_target_to_feedback;

  ThreePitRemoteInput()
      : y_axis(0.0f),
        z_axis(0.0f),
        pitch_axis(0.0f),
        enabled(false),
        reset_target_to_feedback(false) {}
};

struct ThreePitCartesianAppConfig {
  ThreePitWorkspaceLimits workspace;
  Scalar input_deadzone;
  Scalar max_y_velocity;
  Scalar max_z_velocity;
  Scalar max_pitch_velocity;
  Scalar max_linear_velocity;
  Scalar max_angular_velocity;
  Scalar max_linear_acceleration;
  Scalar max_angular_acceleration;
  JointVec<kThreePitCartesianDof> max_joint_velocity;
  JointVec<kThreePitCartesianDof> max_joint_acceleration;
  Scalar max_joint_step;
  JointLimits<kThreePitCartesianDof> joint_limits;
  bool enforce_joint_limits;
  bool limit_joint_velocity;
  bool limit_joint_acceleration;
  bool limit_joint_step;
  kinematics::IKOptions ik_options;

  ThreePitCartesianAppConfig();
};

enum class ThreePitCartesianAppState : uint8_t {
  kIdle = 0,
  kTracking,
  kTargetUnreachableHold,
  kTargetOutOfRangeRejected,
  kFaultHold,
  kSingularityHold,
};

enum class ThreePitCartesianHoldReason : uint8_t {
  kNone = 0,
  kDisabled,
  kUnconfigured,
  kInvalidDt,
  kInvalidFeedback,
  kInvalidInput,
  kTargetIkFailure,
  kTrajectoryFailure,
  kStepIkFailure,
  kSafetyFailure,
  kTargetUnreachable,
  kTargetSingularity,
};

struct ThreePitCartesianAppResult {
  ThreePitCartesianAppState state;
  ThreePitCartesianHoldReason hold_reason;
  YzPitchPose target_yz_pitch;
  YzPitchPose last_valid_target_yz_pitch;
  YzPitchPose candidate_yz_pitch;
  JointCommand<kThreePitCartesianDof> command;
  safety::JointCommandSafetyResult safety_result;
  kinematics::IKResult<kThreePitCartesianDof> target_ik_result;
  kinematics::IKResult<kThreePitCartesianDof> step_ik_result;
  bool candidate_rejected_out_of_range;
  bool target_updated;
  bool target_reachable;
  bool command_updated;

  ThreePitCartesianAppResult();
};

class ThreePitCartesianApp {
 public:
  ThreePitCartesianApp();

  bool configure(const SerialChain<kThreePitCartesianDof>& chain,
                 const ThreePitCartesianAppConfig& config);
  void set_config(const ThreePitCartesianAppConfig& config);
  const ThreePitCartesianAppConfig& config() const { return config_; }

  bool reset_from_feedback(
      const JointState<kThreePitCartesianDof>& feedback);

  ThreePitCartesianAppResult update(
      const JointState<kThreePitCartesianDof>& feedback,
      const ThreePitRemoteInput& input,
      Scalar dt);

  const JointCommand<kThreePitCartesianDof>& command() const {
    return command_;
  }
  ThreePitCartesianAppState state() const { return state_; }
  ThreePitCartesianHoldReason hold_reason() const { return hold_reason_; }
  const YzPitchPose& target_yz_pitch() const { return target_yz_pitch_; }
  const YzPitchPose& last_valid_target_yz_pitch() const {
    return last_valid_target_yz_pitch_;
  }
  const JointVec<kThreePitCartesianDof>& last_valid_q() const {
    return last_valid_q_;
  }
  const Transform& target_transform() const { return target_transform_; }
  bool configured() const { return configured_; }
  bool has_last_valid_command() const { return has_last_valid_; }

 private:
  using JointVec3 = JointVec<kThreePitCartesianDof>;
  using JointCommand3 = JointCommand<kThreePitCartesianDof>;
  using JointState3 = JointState<kThreePitCartesianDof>;

  struct CandidateSolution {
    YzPitchPose pose;
    Transform transform;
    kinematics::IKResult<kThreePitCartesianDof> ik_result;
  };

  struct ScaledCandidateSearch {
    bool accepted;
    bool saw_ik_failure;
    bool saw_safety_failure;
    YzPitchPose accepted_target;
    Transform accepted_transform;
    JointCommand3 accepted_command;
    safety::JointCommandSafetyResult accepted_safety;
    kinematics::IKResult<kThreePitCartesianDof> representative_ik_failure;
  };

  static bool joint_state_is_usable(const JointState3& state);
  static bool remote_input_is_finite(const ThreePitRemoteInput& input);
  static Scalar apply_deadzone(Scalar axis, Scalar deadzone);
  static bool workspace_contains(const ThreePitWorkspaceLimits& workspace,
                                 const YzPitchPose& pose);
  static bool ik_failure_is_singularity(
      const kinematics::IKResult<kThreePitCartesianDof>& result,
      const kinematics::IKOptions& options);
  static bool ik_failure_is_reachability_boundary(
      const kinematics::IKResult<kThreePitCartesianDof>& result);

  Scalar planar_pitch_from_joints(const JointVec3& q) const;
  YzPitchPose yz_pitch_from_feedback(const Transform& transform,
                                     const JointVec3& q) const;
  Rotation make_planar_pitch_rotation(Scalar pitch) const;
  Transform make_target_transform(const YzPitchPose& pose,
                                  const Transform& reference_pose) const;
  YzPitchPose make_candidate_pose(const YzPitchPose& base,
                                  Scalar y_axis,
                                  Scalar z_axis,
                                  Scalar pitch_axis,
                                  Scalar dt,
                                  Scalar scale) const;
  kinematics::IKOptions make_yz_pitch_ik_options(
      const Transform& target_pose) const;
  kinematics::IKResult<kThreePitCartesianDof> solve_target_ik(
      const Transform& target_pose,
      const JointVec3& seed,
      const JointVec3& current) const;
  CandidateSolution solve_candidate(const YzPitchPose& candidate,
                                    const Transform& current_pose,
                                    const JointVec3& reference_q,
                                    const JointState3& feedback) const;
  JointCommand3 make_command_from_solution(const JointVec3& reference_q,
                                           const JointVec3& solution_q,
                                           const JointVec3& previous_qd,
                                           Scalar dt) const;
  safety::JointCommandSafetyResult validate_candidate_command(
      const JointCommand3& command,
      const JointVec3& reference_q) const;
  ScaledCandidateSearch try_solve_scaled_candidate(
      const YzPitchPose& base,
      const Transform& current_pose,
      const JointState3& feedback,
      Scalar y_axis,
      Scalar z_axis,
      Scalar pitch_axis,
      Scalar dt,
      const JointVec3& reference_q,
      const JointVec3& previous_qd,
      bool has_task_delta);
  JointVec3 joint_delta_from_current(const JointVec3& current,
                                     const JointVec3& target) const;
  safety::JointCommandSafetyConfig<kThreePitCartesianDof>
  make_safety_config(const JointVec3& current_q) const;

  void make_hold_command(const JointVec3& q);
  void enter_hold(ThreePitCartesianAppState state,
                  ThreePitCartesianHoldReason reason);
  void fill_result(ThreePitCartesianAppResult* result) const;

  SerialChain<kThreePitCartesianDof> chain_;
  ThreePitCartesianAppConfig config_;
  bool configured_;
  bool has_last_valid_;
  bool has_previous_qd_;
  ThreePitCartesianAppState state_;
  ThreePitCartesianHoldReason hold_reason_;
  YzPitchPose target_yz_pitch_;
  YzPitchPose last_valid_target_yz_pitch_;
  YzPitchPose candidate_yz_pitch_;
  JointVec3 last_valid_q_;
  JointVec3 previous_qd_;
  JointCommand3 command_;
  Transform target_transform_;
  safety::JointCommandSafetyResult safety_result_;
  kinematics::IKResult<kThreePitCartesianDof> target_ik_result_;
  kinematics::IKResult<kThreePitCartesianDof> step_ik_result_;
  bool candidate_rejected_out_of_range_;
  bool target_updated_;
  bool target_reachable_;
  bool command_updated_;
};

}  // namespace application
}  // namespace mr::robotics::arm

#endif
