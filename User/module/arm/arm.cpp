#include "module/arm/arm.hpp"

#include <array>
#include <cmath>
#include <cstring>

#include "bsp/can.h"
#include "component/arm_lib/model/serial_chain.h"
#include "component/arm_lib/parser/urdf_loader.h"
#include "device/joint/joint.hpp"
#include "device/motor/factory/motor_factory.hpp"
#include "device/motor/motor.hpp"
#include "device/motor_dm.h"
#include "device/motor_lz.h"
#include "module/arm/arm_control_types.h"
#include "module/arm/robotic_arm.hpp"

namespace {

static_assert(ARM_JOINT_COUNT == 3, "3pit runtime expects exactly 3 joints");

constexpr float kMinLoopDt = 0.0005f;
constexpr float kMaxLoopDt = 0.05f;
constexpr float kDebugTorqueEpsilon = 1.0e-4f;
constexpr float kGravityCompScales[ARM_JOINT_COUNT] = {1.0f, 1.0f, 1.0f};
constexpr float kJointKp[ARM_JOINT_COUNT] = {6.0f, 8.0f, 10.0f};
constexpr float kJointKd[ARM_JOINT_COUNT] = {0.20f, 0.20f, 0.15f};
constexpr const char* kJointNames[ARM_JOINT_COUNT] = {"armj1", "armj2", "armj3"};

}  // namespace

extern "C" {

volatile float g_arm_j1_test_torque_nm = 0.0f;
volatile float g_arm_j2_test_torque_nm = 0.0f;
volatile float g_arm_j3_test_torque_nm = 0.0f;

}

namespace {

using mrobot::ControlMode;
using mrobot::DmActuator;
using mrobot::DmJoint;
using mrobot::IJoint;
using mrobot::JointControlParams;
using mrobot::JointT;
using mrobot::LzActuator;
using mrobot::LzJoint;
using mrobot::RoboticArm;
using mrobot::arm_project::apply_pose_delta;
using mrobot::motor::MotorFactory;
using mrobot::motor::MotorInstanceConfig;
using mrobot::motor::MotorKind;
using mrobot::motor::MotorModel;
using mrobot::motor::kDirectDriveInstall;

static const MOTOR_LZ_Param_t kJoint1MotorParam = {
    .can = BSP_CAN_1,
    .motor_id = 127,
    .host_id = 0xff,
    .module = MOTOR_LZ_RSO3,
    .reverse = true,
    .mode = MOTOR_LZ_MODE_MOTION,
};

static const MOTOR_DM_Param_t kJoint2MotorParam = {
    .can = BSP_CAN_1,
    .master_id = 0x15,
    .can_id = 0x05,
    .module = MOTOR_DM_J4310,
    .reverse = true,
};

static const MOTOR_DM_Param_t kJoint3MotorParam = {
    .can = BSP_CAN_1,
    .master_id = 0x16,
    .can_id = 0x06,
    .module = MOTOR_DM_J4310,
    .reverse = false,
};

static const char kThreePitUrdf[] =
    "<robot name=\"armurdf_standard_motor\">"
    "<link name=\"base_link\">"
    "<inertial>"
    "<origin xyz=\"0.06401797 0.00093364 0.06018482\" rpy=\"0 0 0\"/>"
    "<mass value=\"1.19404\"/>"
    "<inertia ixx=\"0.00054518\" ixy=\"-8.8651E-07\" ixz=\"0\" iyy=\"0.00099237\" iyz=\"3.2702E-08\" izz=\"0.0011813\"/>"
    "</inertial>"
    "</link>"
    "<link name=\"Link1\">"
    "<inertial>"
    "<origin xyz=\"-0.00047343 0.28954791 0.0005662\" rpy=\"0 0 0\"/>"
    "<mass value=\"0.54717\"/>"
    "<inertia ixx=\"0.0023759\" ixy=\"3.3021E-06\" ixz=\"0\" iyy=\"6.4486E-05\" iyz=\"0\" izz=\"0.0024336\"/>"
    "</inertial>"
    "</link>"
    "<link name=\"Link2\">"
    "<inertial>"
    "<origin xyz=\"-0.0003464 0.2219719 0.00658453\" rpy=\"0 0 0\"/>"
    "<mass value=\"0.5273\"/>"
    "<inertia ixx=\"0.00075992\" ixy=\"1.0161E-06\" ixz=\"0\" iyy=\"4.9865E-05\" iyz=\"0\" izz=\"0.00078302\"/>"
    "</inertial>"
    "</link>"
    "<link name=\"Link3\">"
    "<inertial>"
    "<origin xyz=\"0.002693 0.11635 0.0019852\" rpy=\"0 0 0\"/>"
    "<mass value=\"0.24563\"/>"
    "<inertia ixx=\"0.00013404\" ixy=\"1.2628E-06\" ixz=\"6.3612E-09\" iyy=\"0.0001778\" iyz=\"-1.8653E-10\" izz=\"0.00013348\"/>"
    "</inertial>"
    "</link>"
    "<link name=\"tool_Link\">"
    "<inertial>"
    "<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
    "<mass value=\"0.0001\"/>"
    "<inertia ixx=\"1E-06\" ixy=\"1E-06\" ixz=\"0\" iyy=\"1E-06\" iyz=\"0\" izz=\"0\"/>"
    "</inertial>"
    "</link>"
    "<joint name=\"j1\" type=\"revolute\">"
    "<origin xyz=\"0 0 0.07\" rpy=\"1.5708 0 -1.5708\"/>"
    "<parent link=\"base_link\"/>"
    "<child link=\"Link1\"/>"
    "<axis xyz=\"0 0 -1\"/>"
    "<limit lower=\"-1.57\" upper=\"1.57\"/>"
    "</joint>"
    "<joint name=\"j2\" type=\"revolute\">"
    "<origin xyz=\"-0.0005 0.35 0\" rpy=\"0 0 0\"/>"
    "<parent link=\"Link1\"/>"
    "<child link=\"Link2\"/>"
    "<axis xyz=\"0 0 -1\"/>"
    "<limit lower=\"-2.32\" upper=\"2.32\"/>"
    "</joint>"
    "<joint name=\"j3\" type=\"revolute\">"
    "<origin xyz=\"-0.000357142857063231 0.24999974489785 -0.00262500000000032\" rpy=\"0 0 0\"/>"
    "<parent link=\"Link2\"/>"
    "<child link=\"Link3\"/>"
    "<axis xyz=\"0 0 -1\"/>"
    "<limit lower=\"-1.95\" upper=\"1.95\"/>"
    "</joint>"
    "<joint name=\"j_tool\" type=\"fixed\">"
    "<origin xyz=\"0.0034791 0.14696 -2.5E-05\" rpy=\"0 0 0\"/>"
    "<parent link=\"Link3\"/>"
    "<child link=\"tool_Link\"/>"
    "</joint>"
    "</robot>";

float ClampDt(float dt_s) {
  if (dt_s < kMinLoopDt) {
    return kMinLoopDt;
  }
  if (dt_s > kMaxLoopDt) {
    return kMaxLoopDt;
  }
  return dt_s;
}

void ClearTransientCommand(Arm_CMD_t* cmd) {
  if (cmd == nullptr) {
    return;
  }
  cmd->set_target_as_current = false;
  cmd->gripper_toggle = false;
  std::memset(&cmd->joy_vel, 0, sizeof(cmd->joy_vel));
  std::memset(&cmd->target_delta, 0, sizeof(cmd->target_delta));
}

ArmPoseDelta_t ScalePoseDelta(const ArmPoseDelta_t& velocity, float dt_s) {
  ArmPoseDelta_t delta{};
  delta.x = velocity.x * dt_s;
  delta.y = velocity.y * dt_s;
  delta.z = velocity.z * dt_s;
  delta.roll = velocity.roll * dt_s;
  delta.pitch = velocity.pitch * dt_s;
  delta.yaw = velocity.yaw * dt_s;
  return delta;
}

arm_lib::SerialChain<ARM_JOINT_COUNT> BuildThreePitChain(bool* ok) {
  arm_lib::SerialChain<ARM_JOINT_COUNT> chain;
  const bool loaded =
      arm_lib::parser::load_movable_chain_from_urdf<ARM_JOINT_COUNT>(
          kThreePitUrdf, "base_link", "tool_Link", &chain);
  if (ok != nullptr) {
    *ok = loaded && chain.validate();
  }
  return chain;
}

class ArmRuntime {
 public:
  bool Init(float control_freq_hz);
  void Update();
  void PollCommand(osMessageQueueId_t cmd_queue);
  void Control(float dt_s);

 private:
  bool InitActuatorsAndJoints();
  bool HasDebugTorqueOverride() const;
  void RunDebugTorqueCycle();
  bool ShouldSyncTargets() const;
  void SyncTargetsFromCurrent();
  void ApplyCommand(float dt_s);
  bool BuildPoseCandidate(float dt_s, ArmPose_t* candidate_pose) const;
  bool ApplyPoseTarget(const ArmPose_t& candidate_pose);
  bool ApplyJointTarget(const ArmJointAngles_t& candidate_joints);
  void RunDisabledCycle();
  void RememberLastCommandState();

  arm_lib::SerialChain<ARM_JOINT_COUNT> chain_{};
  mrobot::motor::LzRso3Motor* joint1_motor_ = nullptr;
  mrobot::motor::DmJ4310Motor* joint2_motor_ = nullptr;
  mrobot::motor::DmJ4310Motor* joint3_motor_ = nullptr;
  LzActuator joint1_actuator_{};
  DmActuator joint2_actuator_{};
  DmActuator joint3_actuator_{};
  std::array<IJoint*, ARM_JOINT_COUNT> joints_{};
  RoboticArm robot_{};
  Arm_CMD_t arm_cmd_{};
  ArmPose_t target_pose_{};
  ArmJointAngles_t target_joints_{};
  ArmControlFrame_t last_frame_ = ARM_CTRL_FRAME_WORLD;
  Arm_CtrlType_t last_ctrl_type_ = ARM_CTRL_REMOTE_CARTESIAN;
  bool last_enable_ = false;
  float control_freq_hz_ = 500.0f;
};

ArmRuntime g_arm_runtime;

bool ArmRuntime::Init(float control_freq_hz) {
  control_freq_hz_ = (control_freq_hz > 0.0f) ? control_freq_hz : 500.0f;
  BSP_CAN_Init();
  (void)MOTOR_LZ_Init();

  bool chain_ok = false;
  chain_ = BuildThreePitChain(&chain_ok);
  if (!chain_ok) {
    return false;
  }

  if (!InitActuatorsAndJoints()) {
    return false;
  }

  robot_.SetChain(chain_);
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    robot_.AddJoint(i, joints_[i]);
  }
  if (robot_.Init() != 0) {
    return false;
  }

  robot_.EnableGravityCompensation(true);
  robot_.SetGravityCompScales(kGravityCompScales);
  robot_.SetMode(ControlMode::JOINT_POSITION);

  osDelay(50);
  (void)robot_.Update();

  arm_cmd_.ctrl_type = ARM_CTRL_REMOTE_CARTESIAN;
  arm_cmd_.frame = ARM_CTRL_FRAME_WORLD;
  SyncTargetsFromCurrent();
  return true;
}

void ArmRuntime::Update() {
  (void)robot_.Update();
  robot_.SetGravityCompScales(kGravityCompScales);
}

void ArmRuntime::PollCommand(osMessageQueueId_t cmd_queue) {
  if (cmd_queue == nullptr ||
      osMessageQueueGet(cmd_queue, &arm_cmd_, NULL, 0) != osOK) {
    ClearTransientCommand(&arm_cmd_);
  }
}

void ArmRuntime::Control(float dt_s) {
  const float clamped_dt = ClampDt(dt_s);

  if (HasDebugTorqueOverride()) {
    RunDebugTorqueCycle();
    return;
  }

  robot_.Enable(arm_cmd_.enable);
  if (!arm_cmd_.enable) {
    RunDisabledCycle();
    return;
  }

  if (ShouldSyncTargets()) {
    SyncTargetsFromCurrent();
  }

  robot_.SetMode(ControlMode::JOINT_POSITION);
  ApplyCommand(clamped_dt);
  (void)robot_.Control();
  RememberLastCommandState();
}

bool ArmRuntime::InitActuatorsAndJoints() {
  const auto lz_cfg =
      MotorInstanceConfig<MotorKind::LZ>::FromVendorParam(kJoint1MotorParam);
  const auto dm2_cfg =
      MotorInstanceConfig<MotorKind::DM>::FromVendorParam(kJoint2MotorParam);
  const auto dm3_cfg =
      MotorInstanceConfig<MotorKind::DM>::FromVendorParam(kJoint3MotorParam);

  joint1_motor_ =
      MotorFactory::Create<MotorKind::LZ, MotorModel::RSO3>(lz_cfg,
                                                            kDirectDriveInstall);
  joint2_motor_ =
      MotorFactory::Create<MotorKind::DM, MotorModel::J4310>(dm2_cfg,
                                                             kDirectDriveInstall);
  joint3_motor_ =
      MotorFactory::Create<MotorKind::DM, MotorModel::J4310>(dm3_cfg,
                                                             kDirectDriveInstall);
  if (joint1_motor_ == nullptr || joint2_motor_ == nullptr ||
      joint3_motor_ == nullptr) {
    return false;
  }

  joint1_actuator_.SetMotor(joint1_motor_);
  joint2_actuator_.SetMotor(joint2_motor_);
  joint3_actuator_.SetMotor(joint3_motor_);

  std::array<JointControlParams, ARM_JOINT_COUNT> joint_params{};
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    joint_params[i].qmin = chain_.link(i).lower_limit();
    joint_params[i].qmax = chain_.link(i).upper_limit();
    joint_params[i].kp = kJointKp[i];
    joint_params[i].kd = kJointKd[i];
  }

  joints_[0] =
      new LzJoint(0, &joint1_actuator_, joint_params[0], 0.0f, control_freq_hz_,
                  kJointNames[0]);
  joints_[1] =
      new DmJoint(1, &joint2_actuator_, joint_params[1], 0.0f, control_freq_hz_,
                  kJointNames[1]);
  joints_[2] =
      new DmJoint(2, &joint3_actuator_, joint_params[2], 0.0f, control_freq_hz_,
                  kJointNames[2]);
  if (joints_[0] == nullptr || joints_[1] == nullptr || joints_[2] == nullptr) {
    return false;
  }

  for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    static_cast<JointT*>(joints_[i])->SetTransmission(
        chain_.link(i).transmission());
  }
  return true;
}

bool ArmRuntime::HasDebugTorqueOverride() const {
  return std::fabs(g_arm_j1_test_torque_nm) > kDebugTorqueEpsilon ||
         std::fabs(g_arm_j2_test_torque_nm) > kDebugTorqueEpsilon ||
         std::fabs(g_arm_j3_test_torque_nm) > kDebugTorqueEpsilon;
}

void ArmRuntime::RunDebugTorqueCycle() {
  const float torques[ARM_JOINT_COUNT] = {
      g_arm_j1_test_torque_nm,
      g_arm_j2_test_torque_nm,
      g_arm_j3_test_torque_nm,
  };
  robot_.Enable(true);
  mrobot::arm::drive_torque_targets_and_commit(joints_, torques);
  last_enable_ = false;
  last_frame_ = arm_cmd_.frame;
  last_ctrl_type_ = arm_cmd_.ctrl_type;
  ClearTransientCommand(&arm_cmd_);
}

bool ArmRuntime::ShouldSyncTargets() const {
  return !last_enable_ || arm_cmd_.set_target_as_current ||
         arm_cmd_.frame != last_frame_ ||
         arm_cmd_.ctrl_type != last_ctrl_type_;
}

void ArmRuntime::SyncTargetsFromCurrent() {
  target_pose_ = robot_.GetEndPose();
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    target_joints_.q[i] =
        (joints_[i] != nullptr) ? joints_[i]->GetCurrentAngle() : 0.0f;
  }
  (void)robot_.SetTargetPose(target_pose_);
}

void ArmRuntime::ApplyCommand(float dt_s) {
  if (arm_cmd_.ctrl_type == ARM_CTRL_CUSTOM_JOINT) {
    (void)ApplyJointTarget(arm_cmd_.target_joints);
    return;
  }

  ArmPose_t candidate_pose{};
  if (BuildPoseCandidate(dt_s, &candidate_pose)) {
    (void)ApplyPoseTarget(candidate_pose);
  }
}

bool ArmRuntime::BuildPoseCandidate(float dt_s, ArmPose_t* candidate_pose) const {
  if (candidate_pose == nullptr) {
    return false;
  }

  switch (arm_cmd_.ctrl_type) {
    case ARM_CTRL_REMOTE_CARTESIAN:
      *candidate_pose = apply_pose_delta(
          target_pose_, ScalePoseDelta(arm_cmd_.joy_vel, dt_s), arm_cmd_.frame);
      return true;
    case ARM_CTRL_POSE_ABSOLUTE:
      *candidate_pose = arm_cmd_.target_pose;
      return true;
    case ARM_CTRL_POSE_DELTA:
      *candidate_pose =
          apply_pose_delta(target_pose_, arm_cmd_.target_delta, arm_cmd_.frame);
      return true;
    default:
      return false;
  }
}

bool ArmRuntime::ApplyPoseTarget(const ArmPose_t& candidate_pose) {
  ArmJointAngles_t q_solution{};
  if (robot_.InverseKinematicsAnalytical(&candidate_pose, &q_solution,
                                         &target_joints_) != 0) {
    return false;
  }
  if (!mrobot::arm_project::joint_step_is_reasonable(
          q_solution, target_joints_,
          mrobot::arm_project::kTrajectoryMaxJointDelta)) {
    return false;
  }
  if (robot_.MoveJoint(q_solution.q) != 0) {
    return false;
  }

  target_joints_ = q_solution;
  target_pose_ = candidate_pose;
  (void)robot_.SetTargetPose(candidate_pose);
  return true;
}

bool ArmRuntime::ApplyJointTarget(const ArmJointAngles_t& candidate_joints) {
  if (robot_.MoveJoint(candidate_joints.q) != 0) {
    return false;
  }

  target_joints_ = candidate_joints;
  target_pose_ = mrobot::arm::forward_pose(chain_, candidate_joints);
  (void)robot_.SetTargetPose(target_pose_);
  return true;
}

void ArmRuntime::RunDisabledCycle() {
  (void)robot_.Control();
  last_enable_ = false;
  last_frame_ = arm_cmd_.frame;
  last_ctrl_type_ = arm_cmd_.ctrl_type;
  ClearTransientCommand(&arm_cmd_);
}

void ArmRuntime::RememberLastCommandState() {
  ClearTransientCommand(&arm_cmd_);
  last_enable_ = arm_cmd_.enable;
  last_frame_ = arm_cmd_.frame;
  last_ctrl_type_ = arm_cmd_.ctrl_type;
}

}  // namespace

namespace mrobot::arm {

bool Init(float control_freq_hz) {
  return g_arm_runtime.Init(control_freq_hz);
}

void Update() {
  g_arm_runtime.Update();
}

void PollCommand(osMessageQueueId_t cmd_queue) {
  g_arm_runtime.PollCommand(cmd_queue);
}

void Control(float dt_s) {
  g_arm_runtime.Control(dt_s);
}

}  // namespace mrobot::arm
