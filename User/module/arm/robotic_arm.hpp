#pragma once

#include <array>
#include <cmath>
#include <cstring>
#include <stdint.h>

#include "component/math/scalar.hpp"
#include "device/joint/joint.hpp"
#include "module/arm/arm_control_types.h"
#include "module/arm/detail/utils.hpp"
#include "robotics/arm/controller/arm_controller.h"
#include "robotics/arm/dynamics/gravity.h"
#include "robotics/arm/kinematics/fk.h"
#include "robotics/arm/model/serial_chain.h"

namespace mr {

enum class ControlMode {
    IDLE = 0,
    JOINT_POSITION,
    CARTESIAN_POSITION,
    CARTESIAN_ANALYTICAL,
    TEACH,
    GRAVITY_COMP,
    JOINT_TRAJECTORY,
};

enum class MotionState {
    STOPPED = 0,
    MOVING,
    REACHED,
    ERROR,
};

class RoboticArm {
private:
    static constexpr size_t JOINT_NUM = ARM_JOINT_COUNT;
    using Controller = arm_lib::controller::ArmController<ARM_JOINT_COUNT>;

public:
    RoboticArm()
        : joints_{},
          chain_(),
          controller_(),
          chain_configured_(false),
          mode_(ControlMode::IDLE),
          state_(MotionState::STOPPED),
          position_tolerance_(0.02f),
          enable_(false),
          gravity_comp_enable_(false),
          current_pose_{},
          target_pose_{},
          target_joints_{},
          ik_angles_{},
          ik_valid_(false),
          target_joints_valid_(false),
          max_lin_vel_(0.15f),
          max_ang_vel_(1.0f),
          max_lin_acc_(0.40f),
          max_ang_acc_(2.0f),
          pending_relax_(false) {
        joints_.fill(nullptr);
        for (size_t i = 0; i < JOINT_NUM; ++i) {
            gravity_torques_[i] = 0.0f;
            gravity_scales_[i] = 1.0f;
            joint_traj_max_velocity_[i] =
                arm_project::kDefaultJointTrajMaxVelocity;
            joint_traj_max_acceleration_[i] =
                arm_project::kDefaultJointTrajMaxAcceleration;
        }
    }

    RoboticArm(const RoboticArm&) = delete;
    RoboticArm& operator=(const RoboticArm&) = delete;

    void SetChain(const arm_lib::SerialChain<ARM_JOINT_COUNT>& chain) {
        chain_ = chain;
        chain_configured_ = chain_.validate();
        controller_.set_chain(chain_);
        ConfigureControllerDefaults();
    }

    void AddJoint(size_t index, IJoint* joint) {
        if (index < JOINT_NUM) {
            joints_[index] = joint;
            ConfigureControllerDefaults();
        }
    }

    IJoint* GetJoint(size_t index) {
        return (index < JOINT_NUM) ? joints_[index] : nullptr;
    }

    const IJoint* GetJoint(size_t index) const {
        return (index < JOINT_NUM) ? joints_[index] : nullptr;
    }

    int8_t Init() {
        if (!chain_configured_) {
            return -1;
        }

        for (IJoint* joint : joints_) {
            if (joint == nullptr) {
                return -1;
            }
            joint->Register();
            joint->Enable();
        }

        controller_.set_chain(chain_);
        ConfigureControllerDefaults();
        if (!RefreshControllerFeedback()) {
            return -1;
        }
        SyncTargetsFromCurrent();
        return 0;
    }

    int8_t Update() {
        for (IJoint* joint : joints_) {
            if (joint == nullptr) {
                return -1;
            }
            joint->Enable();
            joint->Update();
        }

        if (!RefreshControllerFeedback()) {
            return -1;
        }

        if (gravity_comp_enable_) {
            CalcGravityTorques(CurrentJointAngles());
        }
        return 0;
    }

    int8_t CalcGravityTorques(const ArmJointAngles_t& q) {
        if (!chain_configured_) {
            return -1;
        }

        const arm_lib::JointVec<ARM_JOINT_COUNT> tau =
            arm_lib::dynamics::gravity_torques(
                chain_, arm_project::angles_to_joint_vec(q));
        for (size_t i = 0; i < JOINT_NUM; ++i) {
            gravity_torques_[i] = tau[i][0] * gravity_scales_[i];
        }
        return 0;
    }

    int8_t InverseKinematicsAnalytical(
        const ArmPose_t* pose,
        ArmJointAngles_t* q_out,
        const ArmJointAngles_t* q_seed = nullptr) {
        if (pose == nullptr || q_out == nullptr || !chain_configured_) {
            return -1;
        }

        const ArmJointAngles_t seed =
            (q_seed != nullptr) ? *q_seed : CurrentJointAngles();
        const arm_lib::kinematics::IKResult<ARM_JOINT_COUNT> result =
            controller_.solve_pose_ik(
                arm_project::pose_to_transform(*pose),
                arm_project::angles_to_joint_vec(seed),
                true,
                true);

        if (!arm_lib::is_success(result.status)) {
            ik_valid_ = false;
            return -1;
        }

        ik_angles_ = arm_project::joint_vec_to_angles(result.q);
        *q_out = ik_angles_;
        ik_valid_ = true;
        return 0;
    }

    int8_t GetPose(ArmPose_t* pose) const {
        if (pose == nullptr) {
            return -1;
        }
        *pose = current_pose_;
        return 0;
    }

    int8_t SetTargetPose(const ArmPose_t pose) {
        target_pose_ = pose;
        return 0;
    }

    int8_t GetTargetPose(ArmPose_t* pose) const {
        if (pose == nullptr) {
            return -1;
        }
        *pose = target_pose_;
        return 0;
    }

    int8_t Control() {
        return Control(0.001f);
    }

    int8_t Control(float dt) {
        if (!chain_configured_) {
            state_ = MotionState::ERROR;
            return -1;
        }

        const float safe_dt = SanitizeDt(dt);
        if (!RefreshControllerFeedback()) {
            state_ = MotionState::ERROR;
            return -1;
        }
        controller_.set_enabled(enable_);
        controller_.set_gravity_feedforward_enabled(gravity_comp_enable_);

        if (!enable_) {
            controller_.set_idle();
            StageRelaxAllJoints();
            state_ = MotionState::STOPPED;
            return 0;
        }

        arm_lib::ArmStatus status = arm_lib::ArmStatus::kOk;
        bool has_controller_command = true;

        switch (mode_) {
        case ControlMode::IDLE:
            controller_.set_idle();
            StageRelaxAllJoints();
            state_ = MotionState::STOPPED;
            return 0;

        case ControlMode::JOINT_POSITION:
            if (!target_joints_valid_) {
                SyncTargetsFromCurrent();
            }
            status = controller_.command_joint_position(
                arm_project::angles_to_joint_vec(target_joints_),
                safe_dt,
                position_tolerance_);
            break;

        case ControlMode::CARTESIAN_POSITION:
        case ControlMode::CARTESIAN_ANALYTICAL:
        case ControlMode::JOINT_TRAJECTORY:
            status = controller_.update(safe_dt);
            break;

        case ControlMode::GRAVITY_COMP:
            status = controller_.command_gravity_hold(safe_dt);
            break;

        case ControlMode::TEACH:
            if (gravity_comp_enable_) {
                status = controller_.command_gravity_torque(safe_dt);
            } else {
                StageRelaxAllJoints();
                state_ = MotionState::MOVING;
                has_controller_command = false;
            }
            break;

        default:
            state_ = MotionState::ERROR;
            return -1;
        }

        if (!arm_lib::is_ok(status)) {
            state_ = MotionState::ERROR;
            StageRelaxAllJoints();
            return -1;
        }

        if (has_controller_command && StageControllerCommand(safe_dt) != 0) {
            state_ = MotionState::ERROR;
            StageRelaxAllJoints();
            return -1;
        }

        state_ = MapMotionState(controller_.motion_state());
        return 0;
    }

    int8_t Commit() {
        int8_t ret = 0;
        bool has_pending_control = false;
        for (IJoint* joint : joints_) {
            if (joint != nullptr && joint->HasPendingControl()) {
                has_pending_control = true;
                break;
            }
        }

        if (pending_relax_ && !has_pending_control) {
            for (IJoint* joint : joints_) {
                if (joint != nullptr && joint->Relax() != 0) {
                    ret = -1;
                }
            }
            pending_relax_ = false;
            return ret;
        }
        pending_relax_ = false;

        for (IJoint* joint : joints_) {
            if (joint != nullptr && joint->HasPendingControl() &&
                joint->CommitControl() != 0) {
                ret = -1;
            }
        }
        return ret;
    }

    void Enable(bool enable) {
        const bool was_enabled = enable_;
        enable_ = enable;
        controller_.set_enabled(enable);
        if (enable && !was_enabled) {
            for (IJoint* joint : joints_) {
                if (joint != nullptr) {
                    joint->Enable();
                }
            }
        }
        if (!enable) {
            mode_ = ControlMode::IDLE;
        }
    }

    void SetMode(ControlMode mode) {
        mode_ = mode;
    }

    MotionState GetState() const {
        return state_;
    }

    void EnableGravityCompensation(bool enable) {
        gravity_comp_enable_ = enable;
        controller_.set_gravity_feedforward_enabled(enable);
        if (!enable) {
            for (size_t i = 0; i < JOINT_NUM; ++i) {
                gravity_torques_[i] = 0.0f;
            }
            ClearFeedforwardTorques();
        }
    }

    void SetGravityCompScale(float scale) {
        for (size_t i = 0; i < JOINT_NUM; ++i) {
            gravity_scales_[i] = scale;
        }
        ConfigureControllerDefaults();
    }

    void SetGravityCompScale(size_t index, float scale) {
        if (index < JOINT_NUM) {
            gravity_scales_[index] = scale;
            ConfigureControllerDefaults();
        }
    }

    void SetGravityCompScales(const float scales[JOINT_NUM]) {
        if (scales == nullptr) {
            return;
        }
        for (size_t i = 0; i < JOINT_NUM; ++i) {
            gravity_scales_[i] = scales[i];
        }
        ConfigureControllerDefaults();
    }

    float GetGravityCompScale(size_t index) const {
        return (index < JOINT_NUM) ? gravity_scales_[index] : 1.0f;
    }

    float GetGravityTorque(size_t index) const {
        return (index < JOINT_NUM) ? gravity_torques_[index] : 0.0f;
    }

    bool IsGravityCompEnabled() const {
        return gravity_comp_enable_;
    }

    int8_t MoveJoint(const float target_angles[JOINT_NUM]) {
        if (target_angles == nullptr) {
            return -1;
        }

        ArmJointAngles_t candidate{};
        for (size_t i = 0; i < JOINT_NUM; ++i) {
            if (!std::isfinite(target_angles[i])) {
                return -1;
            }
            if (joints_[i] != nullptr) {
                const auto& params = joints_[i]->GetParams();
                if (target_angles[i] < params.qmin ||
                    target_angles[i] > params.qmax) {
                    return -1;
                }
            }
            candidate.q[i] = target_angles[i];
        }

        target_joints_ = candidate;
        target_joints_valid_ = true;
        ik_angles_ = candidate;
        ik_valid_ = true;
        for (size_t i = 0; i < JOINT_NUM; ++i) {
            if (joints_[i] != nullptr) {
                joints_[i]->SetTargetAngle(target_joints_.q[i]);
            }
        }
        state_ = MotionState::MOVING;
        return 0;
    }

    int8_t MoveCartesian(const ArmPose_t& goal) {
        if (!chain_configured_) {
            return -1;
        }

        if (!RefreshControllerFeedback()) {
            state_ = MotionState::ERROR;
            return -1;
        }

        arm_lib::controller::MoveLControllerRequest<ARM_JOINT_COUNT> request;
        request.cartesian.start = arm_project::pose_to_transform(current_pose_);
        request.cartesian.goal = arm_project::pose_to_transform(goal);
        request.cartesian.max_linear_velocity = max_lin_vel_;
        request.cartesian.max_angular_velocity = max_ang_vel_;
        request.cartesian.max_linear_acceleration = max_lin_acc_;
        request.cartesian.max_angular_acceleration = max_ang_acc_;
        request.limits = chain_.joint_limits();
        request.enforce_limits = true;
        request.limit_joint_step = true;
        request.max_joint_step = arm_project::kTrajectoryMaxJointDelta;
        request.position_priority_only =
            (mode_ == ControlMode::CARTESIAN_ANALYTICAL);
        FillJointLimitVectors(request.max_velocity, request.max_acceleration);

        const arm_lib::ArmStatus status = controller_.start_move_l(request);
        if (!arm_lib::is_ok(status)) {
            state_ = MotionState::ERROR;
            return -1;
        }

        target_pose_ = goal;
        target_joints_valid_ = false;
        state_ = MotionState::MOVING;
        return 0;
    }

    int8_t MoveJointTrajectory(const ArmPose_t& goal) {
        if (!chain_configured_) {
            return -1;
        }

        if (!RefreshControllerFeedback()) {
            state_ = MotionState::ERROR;
            return -1;
        }

        arm_lib::controller::MoveJToPoseControllerRequest<ARM_JOINT_COUNT>
            request;
        request.target = arm_project::pose_to_transform(goal);
        request.limits = chain_.joint_limits();
        request.enforce_limits = true;
        request.position_priority_only = false;
        FillJointLimitVectors(request.max_velocity, request.max_acceleration);

        const arm_lib::ArmStatus status =
            controller_.start_move_j_to_pose(request);
        if (!arm_lib::is_ok(status)) {
            state_ = MotionState::ERROR;
            return -1;
        }

        target_pose_ = goal;
        target_joints_valid_ = false;
        state_ = MotionState::MOVING;
        return 0;
    }

    void Stop() {
        controller_.set_idle();
        state_ = MotionState::STOPPED;
    }

    void MoveCartesianDelta(const ArmPoseDelta_t& delta,
                            ArmControlFrame_t frame = ARM_CTRL_FRAME_WORLD) {
        const ArmPose_t goal =
            arm_project::apply_pose_delta(target_pose_, delta, frame);
        MoveCartesian(goal);
    }

    void MoveCartesianTool(const ArmPose_t& target_tool_delta) {
        ArmPoseDelta_t delta{};
        delta.x = target_tool_delta.x;
        delta.y = target_tool_delta.y;
        delta.z = target_tool_delta.z;
        delta.roll = target_tool_delta.roll;
        delta.pitch = target_tool_delta.pitch;
        delta.yaw = target_tool_delta.yaw;
        MoveCartesianDelta(delta, ARM_CTRL_FRAME_TOOL);
    }

    void MoveCartesianHeading(const ArmPose_t& target_heading_delta) {
        ArmPoseDelta_t delta{};
        delta.x = target_heading_delta.x;
        delta.y = target_heading_delta.y;
        delta.z = target_heading_delta.z;
        delta.roll = target_heading_delta.roll;
        delta.pitch = target_heading_delta.pitch;
        delta.yaw = target_heading_delta.yaw;
        MoveCartesianDelta(delta, ARM_CTRL_FRAME_HEADING);
    }

    const ArmPose_t& GetEndPose() const {
        return current_pose_;
    }

    const ArmJointAngles_t& GetIkAngles() const {
        return ik_angles_;
    }

    bool IsIkValid() const {
        return ik_valid_;
    }

    void SetLinVelLimit(float vel) {
        if (vel > 0.0f) {
            max_lin_vel_ = vel;
        }
    }

    void SetAngVelLimit(float vel) {
        if (vel > 0.0f) {
            max_ang_vel_ = vel;
        }
    }

    void SetJointTrajectoryVelocity(size_t joint_index, float max_vel) {
        if (joint_index < JOINT_NUM && max_vel > 0.0f) {
            joint_traj_max_velocity_[joint_index] = max_vel;
        }
    }

    void SetJointTrajectoryAcceleration(size_t joint_index, float max_accel) {
        if (joint_index < JOINT_NUM && max_accel > 0.0f) {
            joint_traj_max_acceleration_[joint_index] = max_accel;
        }
    }

    float GetTrajProgress() const {
        if (!IsTrajActive()) {
            return 1.0f;
        }
        return MotionProgress();
    }

    bool IsTrajActive() const {
        return (mode_ == ControlMode::CARTESIAN_POSITION ||
                mode_ == ControlMode::CARTESIAN_ANALYTICAL) &&
               controller_.motion_state() ==
                   arm_lib::controller::ArmMotionState::kMoving;
    }

    float GetJointTrajProgress() const {
        if (!IsJointTrajActive()) {
            return 1.0f;
        }
        return MotionProgress();
    }

    bool IsJointTrajActive() const {
        return mode_ == ControlMode::JOINT_TRAJECTORY &&
               controller_.motion_state() ==
                   arm_lib::controller::ArmMotionState::kMoving;
    }

    void ResetError() {
        controller_.clear_fault();
        state_ = MotionState::REACHED;
        enable_ = true;
        controller_.set_enabled(true);
        SyncTargetsFromCurrent();
        SetJointTargetsToCurrent();
    }

private:
    static float SanitizeDt(float dt) {
        return mr::component::math::sanitize_dt(dt, 0.0002f, 0.0002f, 0.02f);
    }

    static MotionState MapMotionState(
        arm_lib::controller::ArmMotionState state) {
        switch (state) {
        case arm_lib::controller::ArmMotionState::kStopped:
            return MotionState::STOPPED;
        case arm_lib::controller::ArmMotionState::kMoving:
            return MotionState::MOVING;
        case arm_lib::controller::ArmMotionState::kReached:
            return MotionState::REACHED;
        case arm_lib::controller::ArmMotionState::kFault:
        default:
            return MotionState::ERROR;
        }
    }

    void ConfigureControllerDefaults() {
        arm_lib::JointVec<ARM_JOINT_COUNT> kp =
            arm_lib::toolbox_adapter::zero_joint_vec<ARM_JOINT_COUNT>();
        arm_lib::JointVec<ARM_JOINT_COUNT> kd =
            arm_lib::toolbox_adapter::zero_joint_vec<ARM_JOINT_COUNT>();
        arm_lib::JointVec<ARM_JOINT_COUNT> gravity_scales =
            arm_lib::toolbox_adapter::zero_joint_vec<ARM_JOINT_COUNT>();

        for (size_t i = 0; i < JOINT_NUM; ++i) {
            if (joints_[i] != nullptr) {
                const auto& params = joints_[i]->GetParams();
                kp[i][0] = params.kp;
                kd[i][0] = params.kd;
            }
            gravity_scales[i][0] = gravity_scales_[i];
        }

        controller_.set_default_gains(kp, kd);
        controller_.set_gravity_scales(gravity_scales);
        controller_.set_gravity_feedforward_enabled(gravity_comp_enable_);
    }

    void FillJointLimitVectors(
        arm_lib::JointVec<ARM_JOINT_COUNT>& max_velocity,
        arm_lib::JointVec<ARM_JOINT_COUNT>& max_acceleration) const {
        for (size_t i = 0; i < JOINT_NUM; ++i) {
            max_velocity[i][0] = joint_traj_max_velocity_[i];
            max_acceleration[i][0] = joint_traj_max_acceleration_[i];
        }
    }

    ArmJointAngles_t CurrentJointAngles() const {
        ArmJointAngles_t q{};
        for (size_t i = 0; i < JOINT_NUM; ++i) {
            q.q[i] = (joints_[i] != nullptr) ? joints_[i]->GetCurrentAngle()
                                             : 0.0f;
        }
        return q;
    }

    arm_lib::JointState<ARM_JOINT_COUNT> CurrentJointState() const {
        arm_lib::JointState<ARM_JOINT_COUNT> state;
        state.valid = true;
        for (size_t i = 0; i < JOINT_NUM; ++i) {
            if (joints_[i] == nullptr) {
                state.valid = false;
                continue;
            }
            state.q[i][0] = joints_[i]->GetCurrentAngle();
            state.qd[i][0] = joints_[i]->GetCurrentVelocity();
            state.torque[i][0] = 0.0f;
            state.online[i] = joints_[i]->IsOnline();
        }
        return state;
    }

    bool RefreshControllerFeedback() {
        if (!chain_configured_) {
            return false;
        }

        const arm_lib::JointState<ARM_JOINT_COUNT> state =
            CurrentJointState();
        if (!state.valid) {
            return false;
        }

        controller_.update_feedback(state);
        current_pose_ = arm_project::transform_to_three_pit_pose(
            arm_lib::kinematics::fk(chain_, state.q), state.q);
        return true;
    }

    void SyncTargetsFromCurrent() {
        const ArmJointAngles_t q = CurrentJointAngles();
        target_joints_ = q;
        target_joints_valid_ = true;
        ik_angles_ = q;
        ik_valid_ = true;
        if (chain_configured_) {
            const arm_lib::JointVec<ARM_JOINT_COUNT> q_vec =
                arm_project::angles_to_joint_vec(q);
            target_pose_ = arm_project::transform_to_three_pit_pose(
                arm_lib::kinematics::fk(chain_, q_vec), q_vec);
            current_pose_ = target_pose_;
        } else {
            std::memset(&target_pose_, 0, sizeof(target_pose_));
            std::memset(&current_pose_, 0, sizeof(current_pose_));
        }
    }

    void SetJointTargetsToCurrent() {
        for (IJoint* joint : joints_) {
            if (joint != nullptr) {
                joint->SetTargetAngle(joint->GetCurrentAngle());
            }
        }
    }

    void ClearFeedforwardTorques() {
        for (IJoint* joint : joints_) {
            if (joint != nullptr) {
                joint->SetFeedforwardTorque(0.0f);
            }
        }
    }

    void StageRelaxAllJoints() {
        for (IJoint* joint : joints_) {
            if (joint != nullptr) {
                joint->ClearPendingControl();
            }
        }
        pending_relax_ = true;
    }

    int8_t StageControllerCommand(float dt) {
        const arm_lib::JointCommand<ARM_JOINT_COUNT>& command =
            controller_.command();

        if (command.mode == arm_lib::JointCommandMode::kDisabled) {
            StageRelaxAllJoints();
            return 0;
        }
        if (!command.valid) {
            return -1;
        }

        int8_t ret = 0;
        for (size_t i = 0; i < JOINT_NUM; ++i) {
            IJoint* joint = joints_[i];
            if (joint == nullptr || !joint->IsOnline()) {
                ret = -1;
                continue;
            }

            if (command.mode == arm_lib::JointCommandMode::kTorque) {
                if (joint->TorqueControl(command.torque_ff[i][0]) != 0) {
                    ret = -1;
                }
                continue;
            }

            joint->SetTargetAngle(command.q[i][0]);
            joint->SetFeedforwardTorque(command.torque_ff[i][0]);
            if (joint->PositionControl(command.q[i][0], dt) != 0) {
                ret = -1;
            }
        }
        pending_relax_ = false;
        return ret;
    }

    float MotionProgress() const {
        const auto& diagnostics = controller_.diagnostics();
        if (diagnostics.motion_duration <= 1.0e-6f) {
            return 1.0f;
        }
        const float progress =
            diagnostics.motion_time / diagnostics.motion_duration;
        if (progress < 0.0f) {
            return 0.0f;
        }
        if (progress > 1.0f) {
            return 1.0f;
        }
        return progress;
    }

    std::array<IJoint*, JOINT_NUM> joints_;
    arm_lib::SerialChain<ARM_JOINT_COUNT> chain_;
    Controller controller_;
    bool chain_configured_;
    ControlMode mode_;
    MotionState state_;
    float position_tolerance_;
    bool enable_;
    bool gravity_comp_enable_;
    float gravity_torques_[JOINT_NUM];
    float gravity_scales_[JOINT_NUM];
    ArmPose_t current_pose_;
    ArmPose_t target_pose_;
    ArmJointAngles_t target_joints_;
    ArmJointAngles_t ik_angles_;
    bool ik_valid_;
    bool target_joints_valid_;
    float max_lin_vel_;
    float max_ang_vel_;
    float max_lin_acc_;
    float max_ang_acc_;
    float joint_traj_max_velocity_[JOINT_NUM];
    float joint_traj_max_acceleration_[JOINT_NUM];
    bool pending_relax_;
};

}  // namespace mr
