#include "module/arm/arm.hpp"

#include <array>
#include <cstring>

#include "bsp/can.h"
#include "component/math/scalar.hpp"
#include "module/arm/detail/utils.hpp"
#include "robotics/arm/adapter/toolbox_adapter.h"
#include "robotics/arm/application/three_pit_cartesian_app.h"
#include "robotics/arm/kinematics/fk.h"
#include "robotics/arm/model/serial_chain.h"
#include "device/joint/joint.hpp"
#include "device/motor/factory/motor_factory.hpp"
#include "device/motor/motor.hpp"
#include "device/motor_dm.h"
#include "device/motor_lz.h"
#include "module/arm/arm_control_types.h"
#include "module/arm/robotic_arm.hpp"
#include "module/config.h"

namespace {

static_assert(ARM_JOINT_COUNT == 3, "3pit runtime expects exactly 3 joints");

constexpr float kMinLoopDt = 0.0005f;
constexpr float kMaxLoopDt = 0.05f;
constexpr const char* kJointNames[ARM_JOINT_COUNT] = {"armj1", "armj2", "armj3"};

}  // namespace

namespace {

using mr::ControlMode;
using mr::DmJoint;
using mr::IJoint;
using mr::JointControlParams;
using mr::JointT;
using mr::LzActuator;
using mr::LzJoint;
using mr::RoboticArm;
using mr::arm_project::apply_pose_delta;
using mr::motor::MotorFactory;
using mr::motor::MotorInstanceConfig;
using mr::motor::MotorKind;
using mr::motor::MotorModel;
using mr::motor::kDirectDriveInstall;
namespace arm_app = mr::robotics::arm::application;

arm_lib::Vec3 MakeVec3(float x, float y, float z) {
  arm_lib::Vec3 v = arm_lib::toolbox_adapter::zero_vec3();
  v[0][0] = x;
  v[1][0] = y;
  v[2][0] = z;
  return v;
}

mr::motor::MotorTemperatureProtectionConfig MakeTemperatureProtection(
    const MOTOR_TemperatureProtectionConfig_t& param) {
  mr::motor::MotorTemperatureProtectionConfig config{};
  config.warning_c = param.warning_c;
  config.limit_c = param.limit_c;
  config.auto_relax_on_limit = param.auto_relax_on_limit;
  return config;
}

arm_lib::Transform MakeUrdfTransform(float x,
                                     float y,
                                     float z,
                                     float roll,
                                     float pitch,
                                     float yaw) {
  const arm_lib::Vec3 xyz = MakeVec3(x, y, z);
  const arm_lib::Vec3 ypr = MakeVec3(yaw, pitch, roll);
  return arm_lib::toolbox_adapter::transform_from_rpy_translation(ypr, xyz);
}

arm_lib::Rotation MakeInertia(float ixx,
                              float ixy,
                              float ixz,
                              float iyy,
                              float iyz,
                              float izz) {
  arm_lib::Rotation inertia =
      arm_lib::toolbox_adapter::identity_rotation() * 0.0f;
  inertia[0][0] = ixx;
  inertia[0][1] = ixy;
  inertia[0][2] = ixz;
  inertia[1][0] = ixy;
  inertia[1][1] = iyy;
  inertia[1][2] = iyz;
  inertia[2][0] = ixz;
  inertia[2][1] = iyz;
  inertia[2][2] = izz;
  return inertia;
}

arm_lib::InertialParams MakeInertial(float mass,
                                     const arm_lib::Vec3& com,
                                     const arm_lib::Rotation& inertia) {
  arm_lib::InertialParams params;
  params.mass = mass;
  params.com = com;
  params.inertia = inertia;
  return params;
}

arm_lib::ChainJointSpec MakeRevoluteJoint(float lower, float upper) {
  arm_lib::ChainJointSpec joint;
  joint.type = arm_lib::ChainJointType::kRevolute;
  joint.limit_enabled = true;
  joint.lower = lower;
  joint.upper = upper;
  joint.participate_in_ik = true;
  return joint;
}

arm_lib::Link MakeArmLink(const arm_lib::Transform& joint_origin,
                          float lower,
                          float upper,
                          const arm_lib::InertialParams& inertial) {
  arm_lib::Link link(joint_origin, MakeVec3(0.0f, 0.0f, -1.0f),
                     MakeRevoluteJoint(lower, upper));
  link.set_inertial(inertial);
  return link;
}

float ClampDt(float dt_s) {
  return mr::component::math::sanitize_dt(dt_s, kMinLoopDt, kMinLoopDt,
                                          kMaxLoopDt);
}

void ClearTransientCommand(Arm_CMD_t* cmd) {
  if (cmd == nullptr) {
    return;
  }
  cmd->set_target_as_current = false;
  cmd->gripper_toggle = false;
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

float NormalizeRemoteAxis(float velocity, float max_velocity) {
  if (max_velocity <= 1.0e-6f) {
    return 0.0f;
  }
  return mr::component::math::clamp_scalar(velocity / max_velocity, -1.0f,
                                           1.0f);
}

ArmPose_t ForwardPose(const arm_lib::SerialChain<ARM_JOINT_COUNT>& chain,
                      const ArmJointAngles_t& joints) {
  const arm_lib::JointVec<ARM_JOINT_COUNT> q =
      mr::arm_project::angles_to_joint_vec(joints);
  return mr::arm_project::transform_to_three_pit_pose(
      arm_lib::kinematics::fk(chain, q), q);
}

void CopyIkDebugResult(
    const arm_lib::kinematics::IKResult<ARM_JOINT_COUNT>& result,
    const arm_lib::JointVec<ARM_JOINT_COUNT>& current,
    mr::arm::RuntimeIkDebugData* debug) {
  if (debug == nullptr) {
    return;
  }

  debug->status = result.status;
  debug->stage = result.diagnostics.stage;
  debug->reason = result.diagnostics.reason;
  debug->iterations = result.stats.iterations;
  debug->seed_count_tried = result.diagnostics.seed_count_tried;
  debug->retry_count = result.retry_count;
  debug->candidate_solution_count = result.candidate_solution_count;
  debug->selected_solution_index = result.selected_solution_index;
  debug->final_error_norm = result.stats.final_error_norm;
  debug->final_step_norm = result.stats.final_step_norm;
  debug->singularity_metric = result.singularity_metric;
  debug->manipulability_metric = result.manipulability_metric;
  debug->limit_violation = result.limit_violation;
  debug->null_space_step_norm = result.null_space_step_norm;
  debug->used_analytic = result.used_analytic;
  debug->used_numeric_fallback = result.used_numeric_fallback;
  debug->used_numeric_refine = result.used_numeric_refine;
  debug->used_null_space_objective = result.used_null_space_objective;
  debug->task_projection_applied =
      result.diagnostics.task_projection_applied;
  debug->task_weighting_applied =
      result.diagnostics.task_weighting_applied;
  debug->orientation_relaxed = result.diagnostics.orientation_relaxed;
  debug->clamped_to_limits = result.diagnostics.clamped_to_limits;

  float max_abs_delta = 0.0f;
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    const float q = result.q[i][0];
    const float delta = q - current[i][0];
    debug->q[i] = q;
    debug->delta_from_current[i] = delta;
    const float abs_delta = arm_lib::abs_scalar(delta);
    if (abs_delta > max_abs_delta) {
      max_abs_delta = abs_delta;
    }
  }
  debug->max_abs_delta_from_current = max_abs_delta;
}

arm_lib::SerialChain<ARM_JOINT_COUNT> BuildThreePitChain(bool* ok) {
  arm_lib::Link links[ARM_JOINT_COUNT] = {
      MakeArmLink(
          MakeUrdfTransform(0.0f, 0.0f, 0.07f, 1.5708f, 0.0f, 1.57079265f),
          -1.57f, 1.57f,
          MakeInertial(
              0.54717f, MakeVec3(-0.00047343f, 0.28954791f, 0.0005662f),
              MakeInertia(0.0023759f, 3.3021E-06f, 0.0f, 6.4486E-05f, 0.0f,
                          0.0024336f))),
      MakeArmLink(
          MakeUrdfTransform(-0.0005f, 0.35f, 0.0f, 0.0f, 0.0f, 0.0f),
          -2.32f, 2.32f,
          MakeInertial(
              0.5643f, MakeVec3(-0.00034549f, 0.22380246f, 0.00724804f),
              MakeInertia(0.00075992f, 1.0161E-06f, 0.0f, 4.9865E-05f, 0.0f,
                          0.00078302f))),
      MakeArmLink(
          MakeUrdfTransform(-0.000357142857063231f, 0.24999974489785f,
                            -0.00262500000000032f, 0.0f, 0.0f, 0.0f),
          -1.95f, 1.95f,
          MakeInertial(
              0.256f, MakeVec3(0.002693f, 0.11635f, 0.0019852f),
              MakeInertia(0.00013404f, 1.2628E-06f, 6.3612E-09f,
                          0.0001778f, -1.8653E-10f, 0.00013348f))),
  };

  const arm_lib::Transform base_frame =
      arm_lib::toolbox_adapter::identity_transform();
  const arm_lib::ToolFrame tool_frame(MakeUrdfTransform(
      0.0034791f, 0.14696f, -2.5E-05f, 0.0f, 0.0f, 0.0f));
  arm_lib::SerialChain<ARM_JOINT_COUNT> chain(links, base_frame, tool_frame);
  if (ok != nullptr) {
    *ok = chain.validate();
  }
  return chain;
}

}  // namespace

namespace mr::arm {

namespace arm_app = mr::robotics::arm::application;

RuntimeIkDebugData::RuntimeIkDebugData()
    : status(arm_lib::IkStatus::kNoConvergence),
      stage(arm_lib::IkFailStage::kNone),
      reason(arm_lib::IkFailureReason::kNone),
      iterations(0U),
      seed_count_tried(0U),
      retry_count(0U),
      candidate_solution_count(0U),
      selected_solution_index(0U),
      final_error_norm(0.0f),
      final_step_norm(0.0f),
      singularity_metric(0.0f),
      manipulability_metric(0.0f),
      limit_violation(0.0f),
      null_space_step_norm(0.0f),
      max_abs_delta_from_current(0.0f),
      q{0.0f},
      delta_from_current{0.0f},
      used_analytic(false),
      used_numeric_fallback(false),
      used_numeric_refine(false),
      used_null_space_objective(false),
      task_projection_applied(false),
      task_weighting_applied(false),
      orientation_relaxed(false),
      clamped_to_limits(false) {}

RuntimeRemoteCartesianDebugData::RuntimeRemoteCartesianDebugData()
    : active(false),
      feedback_valid(false),
      input_enabled(false),
      reset_target_to_feedback(false),
      candidate_rejected_out_of_range(false),
      target_updated(false),
      target_reachable(false),
      command_updated(false),
      dt_s(0.0f),
      y_axis(0.0f),
      z_axis(0.0f),
      pitch_axis(0.0f),
      target_yz_pitch(),
      last_valid_target_yz_pitch(),
      candidate_yz_pitch(),
      safety_result(),
      target_ik(),
      step_ik() {}

RuntimeDebugData::RuntimeDebugData()
    : initialized(false),
      enabled(false),
      cartesian_app_ready(false),
      last_commit_ok(false),
      ctrl_type(ARM_CTRL_REMOTE_CARTESIAN),
      frame(ARM_CTRL_FRAME_WORLD),
      current_pose(),
      target_pose(),
      current_joints(),
      target_joints(),
      joint_velocity{0.0f},
      feedforward_torque{0.0f},
      control_command_mode(arm_lib::JointCommandMode::kDisabled),
      control_command_valid(false),
      control_position{0.0f},
      control_velocity{0.0f},
      control_acceleration{0.0f},
      control_torque_ff{0.0f},
      gravity_torque{0.0f},
      control_total_torque_ff{0.0f},
      joint_online{false},
      joint_pending{false},
      joint_temperature_c{0.0f},
      joint_temperature_warning{false},
      joint_temperature_over_limit{false},
      joint_temperature_limit_latched{false},
      cartesian_state(arm_app::ThreePitCartesianAppState::kIdle),
      cartesian_hold_reason(arm_app::ThreePitCartesianHoldReason::kNone),
      remote_cartesian() {}

Runtime::Runtime() {
  joints_.fill(nullptr);
  RefreshDebugData();
}

bool Runtime::Init(float control_freq_hz) {
  control_freq_hz_ = (control_freq_hz > 0.0f) ? control_freq_hz : 500.0f;
  const Config_RobotParam_t* robot_param = Config_GetRobotParam();
  param_ = (robot_param != nullptr) ? &robot_param->arm_param : nullptr;
  if (param_ == nullptr) {
    return false;
  }
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
  robot_.SetGravityCompScales(param_->gravity_comp_scale);
  robot_.SetMode(ControlMode::JOINT_POSITION);

  osDelay(50);
  (void)robot_.Update();

  arm_cmd_.ctrl_type = ARM_CTRL_REMOTE_CARTESIAN;
  arm_cmd_.frame = ARM_CTRL_FRAME_WORLD;
  ConfigureCartesianApp();
  SyncTargetsFromCurrent();
  debug_.initialized = true;
  RefreshDebugData();
  return true;
}

void Runtime::Update() {
  (void)robot_.Update();
  if (param_ != nullptr) {
    robot_.EnableGravityCompensation(true);
    robot_.SetGravityCompScales(param_->gravity_comp_scale);
  }
  RefreshDebugData();
}

void Runtime::PollCommand(osMessageQueueId_t cmd_queue) {
  if (cmd_queue == nullptr ||
      osMessageQueueGet(cmd_queue, &arm_cmd_, NULL, 0) != osOK) {
    ClearTransientCommand(&arm_cmd_);
  }
}

void Runtime::Control(float dt_s) {
  const float clamped_dt = ClampDt(dt_s);

  robot_.Enable(arm_cmd_.enable);
  if (!arm_cmd_.enable) {
    RunDisabledCycle();
    RefreshDebugData();
    return;
  }

  if (ShouldSyncTargets()) {
    SyncTargetsFromCurrent();
  }

  if (arm_cmd_.ctrl_type == ARM_CTRL_REMOTE_CARTESIAN &&
      arm_cmd_.set_target_as_current) {
    target_pose_ = robot_.GetEndPose();
    for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
      target_joints_.q[i] =
          (joints_[i] != nullptr) ? joints_[i]->GetCurrentAngle() : 0.0f;
    }
    if (!RunGravityHoldCycle()) {
      RunDisabledCycle();
    }
    RememberLastCommandState();
    RefreshDebugData();
    return;
  }

  if (arm_cmd_.ctrl_type == ARM_CTRL_REMOTE_CARTESIAN) {
    if (!RunRemoteCartesianCycle(clamped_dt)) {
      RunDisabledCycle();
    }
    RememberLastCommandState();
    RefreshDebugData();
    return;
  }

  robot_.SetMode(ControlMode::JOINT_POSITION);
  ApplyCommand(clamped_dt);
  (void)robot_.Control();
  CaptureJointPositionControlDebug(true);
  RememberLastCommandState();
  RefreshDebugData();
}

bool Runtime::InitActuatorsAndJoints() {
  if (param_ == nullptr) {
    return false;
  }
  const auto lz_cfg =
      MotorInstanceConfig<MotorKind::LZ>::FromVendorParam(
          param_->joint1_motor_param);
  const auto dm2_cfg =
      MotorInstanceConfig<MotorKind::DM>::FromVendorParam(
          param_->joint2_motor_param);
  const auto dm3_cfg =
      MotorInstanceConfig<MotorKind::DM>::FromVendorParam(
          param_->joint3_motor_param);

  joint1_motor_ =
      MotorFactory::Create<MotorKind::LZ, MotorModel::RSO3>(lz_cfg,
                                                            kDirectDriveInstall,
                                                            MakeTemperatureProtection(
                                                                param_->joint_temperature_protection[0]));
  joint2_motor_ =
      MotorFactory::Create<MotorKind::DM, MotorModel::J4340>(dm2_cfg,
                                                             kDirectDriveInstall,
                                                             MakeTemperatureProtection(
                                                                 param_->joint_temperature_protection[1]));
  joint3_motor_ =
      MotorFactory::Create<MotorKind::DM, MotorModel::J4310P>(dm3_cfg,
                                                             kDirectDriveInstall,
                                                             MakeTemperatureProtection(
                                                                 param_->joint_temperature_protection[2]));
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
    joint_params[i].kp = param_->joint_kp[i];
    joint_params[i].kd = param_->joint_kd[i];
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

bool Runtime::RunGravityHoldCycle() {
  debug_.control_command_mode =
      arm_lib::JointCommandMode::kPositionVelocityTorque;
  debug_.control_command_valid = true;

  bool ok = true;
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    IJoint* joint = joints_[i];
    const float target_angle =
        (joint != nullptr) ? joint->GetCurrentAngle() : 0.0f;
    const float gravity_torque = robot_.GetGravityTorque(i);
    debug_.control_position[i] =
        target_angle;
    debug_.control_velocity[i] = 0.0f;
    debug_.control_acceleration[i] = 0.0f;
    debug_.control_torque_ff[i] = 0.0f;
    debug_.gravity_torque[i] = gravity_torque;
    debug_.control_total_torque_ff[i] = gravity_torque;

    if (joint == nullptr || !joint->IsOnline()) {
      ok = false;
      continue;
    }
    joint->SetTargetAngle(target_angle);
    joint->SetFeedforwardTorque(gravity_torque);
    if (joint->PositionControl(target_angle, 0.0f) != 0) {
      ok = false;
    }
  }
  return ok;
}

bool Runtime::ShouldSyncTargets() const {
  return !last_enable_ || arm_cmd_.set_target_as_current ||
         arm_cmd_.frame != last_frame_ ||
         arm_cmd_.ctrl_type != last_ctrl_type_;
}

void Runtime::SyncTargetsFromCurrent() {
  target_pose_ = robot_.GetEndPose();
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    target_joints_.q[i] =
        (joints_[i] != nullptr) ? joints_[i]->GetCurrentAngle() : 0.0f;
  }
  (void)robot_.SetTargetPose(target_pose_);
  if (cartesian_app_ready_) {
    (void)cartesian_app_.reset_from_feedback(ReadCurrentJointState());
  }
}

void Runtime::ApplyCommand(float dt_s) {
  if (arm_cmd_.ctrl_type == ARM_CTRL_CUSTOM_JOINT) {
    (void)ApplyJointTarget(arm_cmd_.target_joints);
    return;
  }

  ArmPose_t candidate_pose{};
  if (BuildPoseCandidate(dt_s, &candidate_pose)) {
    (void)ApplyPoseTarget(candidate_pose);
  }
}

bool Runtime::BuildPoseCandidate(float dt_s, ArmPose_t* candidate_pose) const {
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

bool Runtime::ApplyPoseTarget(const ArmPose_t& candidate_pose) {
  ArmJointAngles_t q_solution{};
  if (robot_.InverseKinematicsAnalytical(&candidate_pose, &q_solution,
                                         &target_joints_) != 0) {
    return false;
  }
  if (!mr::arm_project::joint_step_is_reasonable(
          q_solution, target_joints_,
          mr::arm_project::kTrajectoryMaxJointDelta)) {
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

bool Runtime::ApplyJointTarget(const ArmJointAngles_t& candidate_joints) {
  if (robot_.MoveJoint(candidate_joints.q) != 0) {
    return false;
  }

  target_joints_ = candidate_joints;
  target_pose_ = ForwardPose(chain_, candidate_joints);
  (void)robot_.SetTargetPose(target_pose_);
  return true;
}

void Runtime::CaptureJointPositionControlDebug(bool command_valid) {
  debug_.control_command_mode =
      arm_lib::JointCommandMode::kPositionVelocityTorque;
  debug_.control_command_valid = command_valid;
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    IJoint* joint = joints_[i];
    debug_.control_position[i] =
        (joint != nullptr) ? joint->GetTargetAngle() : target_joints_.q[i];
    debug_.control_velocity[i] = 0.0f;
    debug_.control_acceleration[i] = 0.0f;
    debug_.control_torque_ff[i] =
        (joint != nullptr) ? joint->GetFeedforwardTorque() : 0.0f;
    debug_.gravity_torque[i] = robot_.GetGravityTorque(i);
    debug_.control_total_torque_ff[i] = debug_.control_torque_ff[i];
  }
}

void Runtime::ConfigureCartesianApp() {
  if (param_ == nullptr) {
    cartesian_app_ready_ = false;
    return;
  }
  const ArmCartesianRemoteParam_t& remote = param_->remote_cartesian;
  arm_app::ThreePitCartesianAppConfig config;
  config.workspace.y_min = remote.workspace.y_min;
  config.workspace.y_max = remote.workspace.y_max;
  config.workspace.z_min = remote.workspace.z_min;
  config.workspace.z_max = remote.workspace.z_max;
  config.workspace.pitch_min = remote.workspace.pitch_min;
  config.workspace.pitch_max = remote.workspace.pitch_max;
  config.input_deadzone = remote.input_deadzone;
  config.max_y_velocity = remote.max_y_velocity;
  config.max_z_velocity = remote.max_z_velocity;
  config.max_pitch_velocity = remote.max_pitch_velocity;
  config.max_linear_velocity = remote.max_linear_velocity;
  config.max_angular_velocity = remote.max_angular_velocity;
  config.max_linear_acceleration = remote.max_linear_acceleration;
  config.max_angular_acceleration = remote.max_angular_acceleration;
  config.joint_limits = chain_.joint_limits();
  config.max_joint_step = remote.max_joint_step;
  config.enforce_joint_limits = true;
  config.limit_joint_velocity = true;
  config.limit_joint_acceleration = true;
  config.limit_joint_step = true;
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    config.max_joint_velocity[i][0] = remote.joint_max_velocity[i];
    config.max_joint_acceleration[i][0] =
        remote.joint_max_acceleration[i];
  }
  config.ik_options = arm_lib::kinematics::make_ik_options(
      arm_lib::kinematics::IkProfile::kEmbeddedSafe);
  config.ik_options.strategy = arm_lib::kinematics::IkSolveStrategy::kNumericOnly;
  config.ik_options.analytic_solver_preset =
      arm_lib::kinematics::AnalyticSolverPreset::kDisabled;
  config.ik_options.error_tolerance = 2.0e-3f;
  config.ik_options.joint_centering_gain = 0.08f;
  config.ik_options.reference_bias_gain = 0.08f;

  cartesian_app_ready_ = cartesian_app_.configure(chain_, config);
}

arm_lib::JointState<ARM_JOINT_COUNT> Runtime::ReadCurrentJointState() const {
  arm_lib::JointState<ARM_JOINT_COUNT> state;
  state.valid = true;
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    const IJoint* joint = joints_[i];
    if (joint == nullptr) {
      state.valid = false;
      state.online[i] = false;
      continue;
    }
    state.q[i][0] = joint->GetCurrentAngle();
    state.qd[i][0] = joint->GetCurrentVelocity();
    state.torque[i][0] = 0.0f;
    state.online[i] = joint->IsOnline();
    state.valid = state.valid && state.online[i];
  }
  return state;
}

arm_app::ThreePitRemoteInput Runtime::BuildRemoteCartesianInput() const {
  arm_app::ThreePitRemoteInput input;
  input.enabled = arm_cmd_.enable;
  input.reset_target_to_feedback = arm_cmd_.set_target_as_current;
  const ArmCartesianRemoteParam_t* remote =
      (param_ != nullptr) ? &param_->remote_cartesian : nullptr;
  const float max_y_velocity =
      (remote != nullptr) ? remote->max_y_velocity : 1.0f;
  const float max_z_velocity =
      (remote != nullptr) ? remote->max_z_velocity : 1.0f;
  const float max_pitch_velocity =
      (remote != nullptr) ? remote->max_pitch_velocity : 1.0f;
  input.y_axis =
      NormalizeRemoteAxis(arm_cmd_.joy_vel.y, max_y_velocity);
  input.z_axis =
      NormalizeRemoteAxis(arm_cmd_.joy_vel.z, max_z_velocity);
  input.pitch_axis =
      NormalizeRemoteAxis(arm_cmd_.joy_vel.pitch, max_pitch_velocity);
  return input;
}

bool Runtime::RunRemoteCartesianCycle(float dt_s) {
  if (!cartesian_app_ready_) {
    ConfigureCartesianApp();
  }
  if (!cartesian_app_ready_) {
    return false;
  }

  const arm_lib::JointState<ARM_JOINT_COUNT> feedback =
      ReadCurrentJointState();
  const arm_app::ThreePitRemoteInput input = BuildRemoteCartesianInput();
  const arm_app::ThreePitCartesianAppResult result =
      cartesian_app_.update(feedback, input, dt_s);
  CaptureRemoteCartesianDebug(result, input, feedback, dt_s);

  target_pose_ = mr::arm_project::transform_to_pose_with_planar_pitch(
      cartesian_app_.target_transform(),
      cartesian_app_.target_yz_pitch().pitch);
  if (cartesian_app_.has_last_valid_command()) {
    target_joints_ =
        mr::arm_project::joint_vec_to_angles(cartesian_app_.last_valid_q());
  }

  return StageJointCommand(result.command, dt_s);
}

void Runtime::CaptureRemoteCartesianDebug(
    const arm_app::ThreePitCartesianAppResult& result,
    const arm_app::ThreePitRemoteInput& input,
    const arm_lib::JointState<ARM_JOINT_COUNT>& feedback,
    float dt_s) {
  RuntimeRemoteCartesianDebugData& remote_debug = debug_.remote_cartesian;
  remote_debug.active = true;
  remote_debug.feedback_valid = feedback.valid;
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    remote_debug.feedback_valid =
        remote_debug.feedback_valid && feedback.online[i];
  }
  remote_debug.input_enabled = input.enabled;
  remote_debug.reset_target_to_feedback = input.reset_target_to_feedback;
  remote_debug.candidate_rejected_out_of_range =
      result.candidate_rejected_out_of_range;
  remote_debug.target_updated = result.target_updated;
  remote_debug.target_reachable = result.target_reachable;
  remote_debug.command_updated = result.command_updated;
  remote_debug.dt_s = dt_s;
  remote_debug.y_axis = input.y_axis;
  remote_debug.z_axis = input.z_axis;
  remote_debug.pitch_axis = input.pitch_axis;
  remote_debug.target_yz_pitch = result.target_yz_pitch;
  remote_debug.last_valid_target_yz_pitch =
      result.last_valid_target_yz_pitch;
  remote_debug.candidate_yz_pitch = result.candidate_yz_pitch;
  remote_debug.safety_result = result.safety_result;
  CopyIkDebugResult(result.target_ik_result, feedback.q,
                    &remote_debug.target_ik);
  CopyIkDebugResult(result.step_ik_result, feedback.q,
                    &remote_debug.step_ik);
}

bool Runtime::StageJointCommand(
    const arm_lib::JointCommand<ARM_JOINT_COUNT>& command,
    float dt_s) {
  debug_.control_command_mode = command.mode;
  debug_.control_command_valid = command.valid;
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    const float gravity_torque =
        robot_.IsGravityCompEnabled() ? robot_.GetGravityTorque(i) : 0.0f;
    debug_.control_position[i] = command.q[i][0];
    debug_.control_velocity[i] = command.qd[i][0];
    debug_.control_acceleration[i] = command.qdd[i][0];
    debug_.control_torque_ff[i] = command.torque_ff[i][0];
    debug_.gravity_torque[i] = gravity_torque;
    debug_.control_total_torque_ff[i] =
        (command.mode == arm_lib::JointCommandMode::kTorque)
            ? command.torque_ff[i][0]
            : command.torque_ff[i][0] + gravity_torque;
  }

  if (command.mode == arm_lib::JointCommandMode::kDisabled) {
    for (IJoint* joint : joints_) {
      if (joint != nullptr) {
        joint->ClearPendingControl();
      }
    }
    return true;
  }
  if (!command.valid) {
    return false;
  }

  bool ok = true;
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    IJoint* joint = joints_[i];
    if (joint == nullptr || !joint->IsOnline()) {
      ok = false;
      continue;
    }
    if (command.mode == arm_lib::JointCommandMode::kTorque) {
      ok = (joint->TorqueControl(command.torque_ff[i][0]) == 0) && ok;
      continue;
    }
    const float torque_ff = debug_.control_total_torque_ff[i];
    joint->SetTargetAngle(command.q[i][0]);
    joint->SetFeedforwardTorque(torque_ff);
    ok = (joint->PositionControl(command.q[i][0], dt_s) == 0) && ok;
  }
  return ok;
}

void Runtime::Commit() {
  debug_.last_commit_ok = (robot_.Commit() == 0);
  RefreshDebugData();
}

int8_t Runtime::RelaxJoint(size_t index) {
  IJoint* selected_joint = joint(index);
  if (selected_joint == nullptr) {
    return -1;
  }
  selected_joint->ClearPendingControl();
  const int8_t ret = selected_joint->Relax();
  RefreshDebugData();
  return ret;
}

int8_t Runtime::RelaxAll() {
  int8_t ret = 0;
  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    IJoint* selected_joint = joint(i);
    if (selected_joint == nullptr) {
      ret = -1;
      continue;
    }
    selected_joint->ClearPendingControl();
    if (selected_joint->Relax() != 0) {
      ret = -1;
    }
  }
  RefreshDebugData();
  return ret;
}

int8_t Runtime::SetMotorZero(size_t index) {
  IJoint* selected_joint = joint(index);
  if (selected_joint == nullptr) {
    return -1;
  }
  selected_joint->ClearPendingControl();
  const int8_t ret = selected_joint->SetZero();
  if (ret == 0) {
    for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
      target_joints_.q[i] =
          (joints_[i] != nullptr) ? joints_[i]->GetCurrentAngle() : 0.0f;
    }
    target_pose_ = ForwardPose(chain_, target_joints_);
    (void)robot_.SetTargetPose(target_pose_);
    if (cartesian_app_ready_) {
      (void)cartesian_app_.reset_from_feedback(ReadCurrentJointState());
    }
  }
  RefreshDebugData();
  return ret;
}

void Runtime::RunDisabledCycle() {
  debug_.control_command_mode = arm_lib::JointCommandMode::kDisabled;
  debug_.control_command_valid = false;
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    debug_.control_position[i] =
        (joints_[i] != nullptr) ? joints_[i]->GetCurrentAngle() : 0.0f;
    debug_.control_velocity[i] = 0.0f;
    debug_.control_acceleration[i] = 0.0f;
    debug_.control_torque_ff[i] = 0.0f;
    debug_.gravity_torque[i] = robot_.GetGravityTorque(i);
    debug_.control_total_torque_ff[i] = 0.0f;
  }
  (void)robot_.Control();
  last_enable_ = false;
  last_frame_ = arm_cmd_.frame;
  last_ctrl_type_ = arm_cmd_.ctrl_type;
  ClearTransientCommand(&arm_cmd_);
}

void Runtime::RememberLastCommandState() {
  ClearTransientCommand(&arm_cmd_);
  last_enable_ = arm_cmd_.enable;
  last_frame_ = arm_cmd_.frame;
  last_ctrl_type_ = arm_cmd_.ctrl_type;
}

void Runtime::RefreshDebugData() {
  debug_.enabled = arm_cmd_.enable;
  debug_.cartesian_app_ready = cartesian_app_ready_;
  debug_.ctrl_type = arm_cmd_.ctrl_type;
  debug_.frame = arm_cmd_.frame;
  debug_.current_pose = robot_.GetEndPose();
  debug_.target_pose = target_pose_;
  debug_.target_joints = target_joints_;
  debug_.cartesian_state = cartesian_app_.state();
  debug_.cartesian_hold_reason = cartesian_app_.hold_reason();

  for (uint8_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    IJoint* joint = joints_[i];
    debug_.current_joints.q[i] =
        (joint != nullptr) ? joint->GetCurrentAngle() : 0.0f;
    debug_.joint_velocity[i] =
        (joint != nullptr) ? joint->GetCurrentVelocity() : 0.0f;
    debug_.target_joints.q[i] =
        (joint != nullptr) ? joint->GetTargetAngle() : target_joints_.q[i];
    debug_.feedforward_torque[i] =
        (joint != nullptr) ? joint->GetFeedforwardTorque() : 0.0f;
    debug_.gravity_torque[i] = robot_.GetGravityTorque(i);
    debug_.joint_online[i] = (joint != nullptr) && joint->IsOnline();
    debug_.joint_pending[i] = (joint != nullptr) && joint->HasPendingControl();

    mr::motor::MotorState motor_state{};
    if (i == 0U && joint1_motor_ != nullptr) {
      motor_state = joint1_motor_->GetState();
    } else if (i == 1U && joint2_motor_ != nullptr) {
      motor_state = joint2_motor_->GetState();
    } else if (i == 2U && joint3_motor_ != nullptr) {
      motor_state = joint3_motor_->GetState();
    }
    debug_.joint_temperature_c[i] = motor_state.temperature_c;
    debug_.joint_temperature_warning[i] = motor_state.temperature_warning;
    debug_.joint_temperature_over_limit[i] = motor_state.temperature_over_limit;
    debug_.joint_temperature_limit_latched[i] =
        motor_state.temperature_limit_latched;
  }
}

namespace {

Runtime* g_default_runtime = nullptr;

}  // namespace

void SetDefaultRuntime(Runtime* runtime) {
  g_default_runtime = runtime;
}

bool Init(float control_freq_hz) {
  return (g_default_runtime != nullptr) &&
         g_default_runtime->Init(control_freq_hz);
}

void Update() {
  if (g_default_runtime != nullptr) {
    g_default_runtime->Update();
  }
}

void PollCommand(osMessageQueueId_t cmd_queue) {
  if (g_default_runtime != nullptr) {
    g_default_runtime->PollCommand(cmd_queue);
  }
}

void Control(float dt_s) {
  if (g_default_runtime != nullptr) {
    g_default_runtime->Control(dt_s);
  }
}

void Commit() {
  if (g_default_runtime != nullptr) {
    g_default_runtime->Commit();
  }
}

int8_t RelaxJoint(size_t index) {
  return (g_default_runtime != nullptr) ? g_default_runtime->RelaxJoint(index)
                                        : -1;
}

int8_t RelaxAll() {
  return (g_default_runtime != nullptr) ? g_default_runtime->RelaxAll() : -1;
}

int8_t SetMotorZero(size_t index) {
  return (g_default_runtime != nullptr) ? g_default_runtime->SetMotorZero(index)
                                        : -1;
}

}  // namespace mr::arm
