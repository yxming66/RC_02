#pragma once

#include <array>
#include <stddef.h>
#include <stdint.h>

#include "cmsis_os2.h"

#ifdef __cplusplus
#include "device/joint/joint.hpp"
#include "device/motor/motor.hpp"
#include "module/arm/arm_control_types.h"
#include "module/arm/robotic_arm.hpp"
#include "robotics/arm/application/three_pit_cartesian_app.h"
#include "robotics/arm/core/arm_command.h"
#include "robotics/arm/model/serial_chain.h"

namespace mr::arm {

struct RuntimeIkDebugData {
  mr::robotics::arm::IkStatus status;
  mr::robotics::arm::IkFailStage stage;
  mr::robotics::arm::IkFailureReason reason;
  uint16_t iterations;
  uint16_t seed_count_tried;
  uint16_t retry_count;
  uint16_t candidate_solution_count;
  uint16_t selected_solution_index;
  float final_error_norm;
  float final_step_norm;
  float singularity_metric;
  float manipulability_metric;
  float limit_violation;
  float null_space_step_norm;
  float max_abs_delta_from_current;
  float q[ARM_JOINT_COUNT];
  float delta_from_current[ARM_JOINT_COUNT];
  bool used_analytic;
  bool used_numeric_fallback;
  bool used_numeric_refine;
  bool used_null_space_objective;
  bool task_projection_applied;
  bool task_weighting_applied;
  bool orientation_relaxed;
  bool clamped_to_limits;

  RuntimeIkDebugData();
};

struct RuntimeRemoteCartesianDebugData {
  bool active;
  bool feedback_valid;
  bool input_enabled;
  bool reset_target_to_feedback;
  bool candidate_rejected_out_of_range;
  bool target_updated;
  bool target_reachable;
  bool command_updated;
  float dt_s;
  float y_axis;
  float z_axis;
  float pitch_axis;
  mr::robotics::arm::application::YzPitchPose target_yz_pitch;
  mr::robotics::arm::application::YzPitchPose last_valid_target_yz_pitch;
  mr::robotics::arm::application::YzPitchPose candidate_yz_pitch;
  mr::robotics::arm::safety::JointCommandSafetyResult safety_result;
  RuntimeIkDebugData target_ik;
  RuntimeIkDebugData step_ik;

  RuntimeRemoteCartesianDebugData();
};

struct RuntimeDebugData {
  bool initialized;
  bool enabled;
  bool cartesian_app_ready;
  bool last_commit_ok;
  Arm_CtrlType_t ctrl_type;
  ArmControlFrame_t frame;
  ArmPose_t current_pose;
  ArmPose_t target_pose;
  ArmJointAngles_t current_joints;
  ArmJointAngles_t target_joints;
  float joint_velocity[ARM_JOINT_COUNT];
  float feedforward_torque[ARM_JOINT_COUNT];
  mr::robotics::arm::JointCommandMode control_command_mode;
  bool control_command_valid;
  float control_position[ARM_JOINT_COUNT];
  float control_velocity[ARM_JOINT_COUNT];
  float control_acceleration[ARM_JOINT_COUNT];
  float control_torque_ff[ARM_JOINT_COUNT];
  float gravity_torque[ARM_JOINT_COUNT];
  float control_total_torque_ff[ARM_JOINT_COUNT];
  bool joint_online[ARM_JOINT_COUNT];
  bool joint_pending[ARM_JOINT_COUNT];
  mr::robotics::arm::application::ThreePitCartesianAppState cartesian_state;
  mr::robotics::arm::application::ThreePitCartesianHoldReason
      cartesian_hold_reason;
  RuntimeRemoteCartesianDebugData remote_cartesian;

  RuntimeDebugData();
};

class Runtime {
 public:
  Runtime();

  bool Init(float control_freq_hz);
  void Update();
  void PollCommand(osMessageQueueId_t cmd_queue);
  void Control(float dt_s);
  void Commit();
  int8_t RelaxJoint(size_t index);
  int8_t RelaxAll();
  int8_t SetMotorZero(size_t index);

  const RuntimeDebugData& debug() const { return debug_; }
  RuntimeDebugData& debug() { return debug_; }

  RoboticArm& robot() { return robot_; }
  const RoboticArm& robot() const { return robot_; }
  const arm_lib::SerialChain<ARM_JOINT_COUNT>& chain() const {
    return chain_;
  }
  IJoint* joint(size_t index) {
    return (index < ARM_JOINT_COUNT) ? joints_[index] : nullptr;
  }
  const IJoint* joint(size_t index) const {
    return (index < ARM_JOINT_COUNT) ? joints_[index] : nullptr;
  }
  mr::motor::LzRso3Motor* joint1_motor() { return joint1_motor_; }
  mr::motor::DmJ4340Motor* joint2_motor() { return joint2_motor_; }
  mr::motor::DmJ4310PMotor* joint3_motor() { return joint3_motor_; }
  const mr::robotics::arm::application::ThreePitCartesianApp&
  cartesian_app() const {
    return cartesian_app_;
  }

 private:
  bool InitActuatorsAndJoints();
  bool RunGravityHoldCycle();
  bool ShouldSyncTargets() const;
  void SyncTargetsFromCurrent();
  void ApplyCommand(float dt_s);
  bool BuildPoseCandidate(float dt_s, ArmPose_t* candidate_pose) const;
  bool ApplyPoseTarget(const ArmPose_t& candidate_pose);
  bool ApplyJointTarget(const ArmJointAngles_t& candidate_joints);
  void CaptureJointPositionControlDebug(bool command_valid);
  void ConfigureCartesianApp();
  arm_lib::JointState<ARM_JOINT_COUNT> ReadCurrentJointState() const;
  mr::robotics::arm::application::ThreePitRemoteInput
  BuildRemoteCartesianInput() const;
  bool RunRemoteCartesianCycle(float dt_s);
  void CaptureRemoteCartesianDebug(
      const mr::robotics::arm::application::ThreePitCartesianAppResult& result,
      const mr::robotics::arm::application::ThreePitRemoteInput& input,
      const arm_lib::JointState<ARM_JOINT_COUNT>& feedback,
      float dt_s);
  bool StageJointCommand(
      const arm_lib::JointCommand<ARM_JOINT_COUNT>& command,
      float dt_s);
  void RunDisabledCycle();
  void RememberLastCommandState();
  void RefreshDebugData();

  arm_lib::SerialChain<ARM_JOINT_COUNT> chain_{};
  mr::motor::LzRso3Motor* joint1_motor_ = nullptr;
  mr::motor::DmJ4340Motor* joint2_motor_ = nullptr;
  mr::motor::DmJ4310PMotor* joint3_motor_ = nullptr;
  LzActuator joint1_actuator_{};
  DmJ4340Actuator joint2_actuator_{};
  DmJ4310PActuator joint3_actuator_{};
  std::array<IJoint*, ARM_JOINT_COUNT> joints_{};
  RoboticArm robot_{};
  mr::robotics::arm::application::ThreePitCartesianApp cartesian_app_{};
  Arm_CMD_t arm_cmd_{};
  ArmPose_t target_pose_{};
  ArmJointAngles_t target_joints_{};
  ArmControlFrame_t last_frame_ = ARM_CTRL_FRAME_WORLD;
  Arm_CtrlType_t last_ctrl_type_ = ARM_CTRL_REMOTE_CARTESIAN;
  bool last_enable_ = false;
  bool cartesian_app_ready_ = false;
  float control_freq_hz_ = 500.0f;
  const Arm_Params_t* param_ = nullptr;
  RuntimeDebugData debug_{};
};

void SetDefaultRuntime(Runtime* runtime);
bool Init(float control_freq_hz);
void Update();
void PollCommand(osMessageQueueId_t cmd_queue);
void Control(float dt_s);
void Commit();
int8_t RelaxJoint(size_t index);
int8_t RelaxAll();
int8_t SetMotorZero(size_t index);

}  // namespace mr::arm
#endif
