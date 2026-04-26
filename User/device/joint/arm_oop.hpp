#pragma once

#include <array>
#include <cmath>
#include <cstring>
#include <stdint.h>

#include "bsp/time.h"
#include "arm_control_types.h"
#include "arm_runtime_control_ops.hpp"
#include "arm_runtime_kinematics.hpp"
#include "arm_runtime_trajectory.hpp"
#include "arm_runtime_utils.hpp"
#include "component/arm_lib/model/serial_chain.h"
#include "component/arm_lib/dynamics/gravity.h"
#include "joint.hpp"

namespace mrobot {

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

    std::array<IJoint*, JOINT_NUM> joints_;
    arm_lib::SerialChain<ARM_JOINT_COUNT> chain_;
    bool chain_configured_;

    struct {
        ArmPose_t current;
        ArmPose_t target;
    } end_effector_;

    struct {
        ControlMode mode;
        MotionState state;
        float position_tolerance;
        bool enable;
        bool gravity_comp_enable;
    } control_;

    struct {
        ArmJointAngles_t angles;
        bool valid;
    } inverse_kinematics_;

    struct {
        float torques[JOINT_NUM];
        float scales[JOINT_NUM];
    } gravity_comp_;

    struct {
        ArmPose_t start;
        ArmPose_t goal;
        float elapsed;
        bool active;
        bool valid;
        float max_lin_vel;
        float max_ang_vel;
        float max_lin_acc;
        float max_ang_acc;
        float max_joint_delta;
        ArmJointAngles_t angles;
        arm_lib::trajectory::CartesianTrajectory trajectory;
    } traj_;

    struct {
        ArmJointAngles_t start;
        ArmJointAngles_t goal;
        float elapsed;
        bool active;
        bool valid;
        float max_velocity[JOINT_NUM];
        float max_acceleration[JOINT_NUM];
        ArmJointAngles_t angles;
        arm_lib::trajectory::JointTrajectory<ARM_JOINT_COUNT> trajectory;
    } joint_traj_;

    struct {
        float now;
        uint64_t last_wakeup;
        float dt;
    } timer_;

    void StepTrajectory(bool position_priority_only) {
        if (!traj_.active || !traj_.trajectory.valid()) {
            return;
        }

        const arm_runtime::CartesianTrajectoryStepResult step =
            arm_runtime::step_cartesian_trajectory(
                &traj_.trajectory,
                &traj_.elapsed,
                timer_.dt,
                joints_,
                chain_,
                chain_configured_,
                traj_.valid,
                traj_.angles,
                traj_.max_joint_delta,
                position_priority_only);
        if (!step.valid) {
            traj_.valid = false;
            traj_.active = false;
            control_.state = MotionState::ERROR;
            return;
        }

        traj_.angles = step.angles;
        traj_.valid = true;
        traj_.active = step.active;
        end_effector_.target = step.pose;
        inverse_kinematics_.angles = step.angles;
        inverse_kinematics_.valid = true;
    }

    void StepJointTrajectory() {
        if (!joint_traj_.active || !joint_traj_.trajectory.valid()) {
            return;
        }

        const arm_runtime::JointTrajectoryStepResult step =
            arm_runtime::step_joint_trajectory(
                &joint_traj_.trajectory,
                &joint_traj_.elapsed,
                timer_.dt,
                chain_);
        if (!step.valid) {
            joint_traj_.valid = false;
            joint_traj_.active = false;
            control_.state = MotionState::ERROR;
            return;
        }

        joint_traj_.angles = step.angles;
        joint_traj_.valid = true;
        joint_traj_.active = step.active;
        end_effector_.target = step.pose;
        inverse_kinematics_.angles = joint_traj_.angles;
        inverse_kinematics_.valid = true;
    }

public:
    RoboticArm()
        : joints_{},
          chain_(),
          chain_configured_(false) {
        joints_.fill(nullptr);
        std::memset(&end_effector_, 0, sizeof(end_effector_));
        std::memset(&inverse_kinematics_, 0, sizeof(inverse_kinematics_));

        control_.mode = ControlMode::IDLE;
        control_.state = MotionState::STOPPED;
        control_.position_tolerance = 0.02f;
        control_.enable = false;
        control_.gravity_comp_enable = false;

        for (size_t i = 0; i < JOINT_NUM; ++i) {
            gravity_comp_.torques[i] = 0.0f;
            gravity_comp_.scales[i] = 1.0f;
            joint_traj_.max_velocity[i] =
                arm_project::kDefaultJointTrajMaxVelocity;
            joint_traj_.max_acceleration[i] =
                arm_project::kDefaultJointTrajMaxAcceleration;
        }

        traj_.elapsed = 0.0f;
        traj_.active = false;
        traj_.valid = false;
        traj_.max_lin_vel = 0.15f;
        traj_.max_ang_vel = 1.0f;
        traj_.max_lin_acc = 0.40f;
        traj_.max_ang_acc = 2.0f;
        traj_.max_joint_delta = arm_project::kTrajectoryMaxJointDelta;

        joint_traj_.elapsed = 0.0f;
        joint_traj_.active = false;
        joint_traj_.valid = false;

        timer_.now = 0.0f;
        timer_.last_wakeup = 0U;
        timer_.dt = 0.001f;
    }

    RoboticArm(const RoboticArm&) = delete;
    RoboticArm& operator=(const RoboticArm&) = delete;

    void SetChain(const arm_lib::SerialChain<ARM_JOINT_COUNT>& chain) {
        chain_ = chain;
        chain_configured_ = true;
    }

    void AddJoint(size_t index, IJoint* joint) {
        if (index < JOINT_NUM) {
            joints_[index] = joint;
        }
    }

    IJoint* GetJoint(size_t index) {
        return (index < JOINT_NUM) ? joints_[index] : nullptr;
    }

    const IJoint* GetJoint(size_t index) const {
        return (index < JOINT_NUM) ? joints_[index] : nullptr;
    }

    int8_t Init() {
        for (IJoint* joint : joints_) {
            if (joint != nullptr) {
                joint->Register();
                joint->Enable();
            }
        }
        std::memset(&end_effector_.target, 0, sizeof(end_effector_.target));
        return chain_configured_ ? 0 : -1;
    }

    int8_t Update() {
        for (IJoint* joint : joints_) {
            if (joint != nullptr) {
                joint->Enable();
                joint->Update();
            }
        }

        if (!chain_configured_) {
            return -1;
        }

        ArmJointAngles_t q{};
        arm_runtime::fill_current_joint_angles(joints_, &q);
        end_effector_.current = arm_runtime::forward_pose(chain_, q);

        if (control_.gravity_comp_enable) {
            CalcGravityTorques(q);
        }
        return 0;
    }

    int8_t CalcGravityTorques(const ArmJointAngles_t& q) {
        if (!chain_configured_) {
            return -1;
        }

        const auto tau = arm_lib::dynamics::gravity_torques(
            chain_, arm_project::angles_to_joint_vec(q));
        for (size_t i = 0; i < JOINT_NUM; ++i) {
            gravity_comp_.torques[i] = tau[i][0] * gravity_comp_.scales[i];
        }
        return 0;
    }

    int8_t InverseKinematicsAnalytical(const ArmPose_t* pose,
                                       ArmJointAngles_t* q_out,
                                       const ArmJointAngles_t* q_seed = nullptr) {
        if (pose == nullptr || q_out == nullptr) {
            return -1;
        }
        return arm_runtime::solve_ik(
            chain_, chain_configured_, joints_, *pose, q_out, q_seed, true);
    }

    int8_t GetPose(ArmPose_t* pose) const {
        if (pose == nullptr) {
            return -1;
        }
        *pose = end_effector_.current;
        return 0;
    }

    int8_t SetTargetPose(const ArmPose_t pose) {
        end_effector_.target = pose;
        return 0;
    }

    int8_t GetTargetPose(ArmPose_t* pose) const {
        if (pose == nullptr) {
            return -1;
        }
        *pose = end_effector_.target;
        return 0;
    }

    int8_t Control() {
        const uint64_t now_us = BSP_TIME_Get_us();
        if (timer_.last_wakeup == 0U || now_us <= timer_.last_wakeup) {
            timer_.dt = 0.001f;
        } else {
            timer_.dt = (now_us - timer_.last_wakeup) / 1000000.0f;
            if (timer_.dt < 0.0002f) {
                timer_.dt = 0.0002f;
            }
            if (timer_.dt > 0.02f) {
                timer_.dt = 0.02f;
            }
        }
        timer_.last_wakeup = now_us;
        timer_.now = now_us / 1000000.0f;

        if (!control_.enable) {
            arm_runtime::relax_all_joints(joints_);
            control_.state = MotionState::STOPPED;
            return 0;
        }

        switch (control_.mode) {
        case ControlMode::IDLE:
            arm_runtime::relax_all_joints(joints_);
            control_.state = MotionState::STOPPED;
            break;

        case ControlMode::JOINT_POSITION:
        case ControlMode::CARTESIAN_POSITION:
        case ControlMode::CARTESIAN_ANALYTICAL:
        case ControlMode::JOINT_TRAJECTORY:
        case ControlMode::GRAVITY_COMP: {
            if (control_.state == MotionState::ERROR) {
                arm_runtime::relax_all_joints(joints_);
                control_.enable = false;
                return -1;
            }

            if (control_.gravity_comp_enable) {
                arm_runtime::apply_feedforward_torques(joints_, gravity_comp_.torques);
            }

            if (control_.mode == ControlMode::JOINT_POSITION) {
                if (!arm_runtime::validate_and_copy_joint_targets(
                        joints_, &inverse_kinematics_.angles)) {
                    arm_runtime::relax_all_joints(joints_);
                    control_.state = MotionState::ERROR;
                    return -1;
                }
                inverse_kinematics_.valid = true;
                traj_.valid = true;
                traj_.active = false;
            }

            if (control_.mode == ControlMode::GRAVITY_COMP) {
                arm_runtime::set_joint_targets_to_current(joints_);
            }

            if (control_.mode == ControlMode::CARTESIAN_POSITION ||
                control_.mode == ControlMode::CARTESIAN_ANALYTICAL) {
                StepTrajectory(control_.mode == ControlMode::CARTESIAN_ANALYTICAL);
                if (!traj_.valid) {
                    arm_runtime::relax_all_joints(joints_);
                    control_.state = MotionState::ERROR;
                    return -1;
                }

                arm_runtime::apply_joint_targets(joints_, traj_.angles);
            }

            if (control_.mode == ControlMode::JOINT_TRAJECTORY) {
                StepJointTrajectory();
                if (!joint_traj_.valid) {
                    arm_runtime::relax_all_joints(joints_);
                    control_.state = MotionState::ERROR;
                    return -1;
                }

                arm_runtime::apply_joint_targets(joints_, joint_traj_.angles);
            }

            const bool all_reached = arm_runtime::drive_position_targets_and_commit(
                joints_, timer_.dt, control_.position_tolerance);
            control_.state = all_reached ? MotionState::REACHED : MotionState::MOVING;
            break;
        }

        case ControlMode::TEACH:
            if (control_.gravity_comp_enable) {
                arm_runtime::drive_torque_targets_and_commit(joints_, gravity_comp_.torques);
            } else {
                arm_runtime::relax_all_joints(joints_);
            }
            control_.state = MotionState::MOVING;
            break;

        default:
            control_.state = MotionState::ERROR;
            break;
        }

        return 0;
    }

    void Enable(bool enable) {
        control_.enable = enable;
        if (!enable) {
            control_.mode = ControlMode::IDLE;
        }
    }

    void SetMode(ControlMode mode) {
        control_.mode = mode;
    }

    MotionState GetState() const {
        return control_.state;
    }

    void EnableGravityCompensation(bool enable) {
        control_.gravity_comp_enable = enable;
        if (!enable) {
            for (size_t i = 0; i < JOINT_NUM; ++i) {
                gravity_comp_.torques[i] = 0.0f;
            }
            arm_runtime::clear_feedforward_torques(joints_);
        }
    }

    void SetGravityCompScale(float scale) {
        for (size_t i = 0; i < JOINT_NUM; ++i) {
            gravity_comp_.scales[i] = scale;
        }
    }

    void SetGravityCompScale(size_t index, float scale) {
        if (index < JOINT_NUM) {
            gravity_comp_.scales[index] = scale;
        }
    }

    void SetGravityCompScales(const float scales[JOINT_NUM]) {
        if (scales == nullptr) {
            return;
        }
        for (size_t i = 0; i < JOINT_NUM; ++i) {
            gravity_comp_.scales[i] = scales[i];
        }
    }

    float GetGravityCompScale(size_t index) const {
        return (index < JOINT_NUM) ? gravity_comp_.scales[index] : 1.0f;
    }

    float GetGravityTorque(size_t index) const {
        return (index < JOINT_NUM) ? gravity_comp_.torques[index] : 0.0f;
    }

    bool IsGravityCompEnabled() const {
        return control_.gravity_comp_enable;
    }

    int8_t MoveJoint(const float target_angles[JOINT_NUM]) {
        for (size_t i = 0; i < JOINT_NUM; ++i) {
            if (joints_[i] != nullptr) {
                const auto& params = joints_[i]->GetParams();
                if (target_angles[i] < params.qmin || target_angles[i] > params.qmax) {
                    return -1;
                }
            }
        }

        for (size_t i = 0; i < JOINT_NUM; ++i) {
            if (joints_[i] != nullptr) {
                joints_[i]->SetTargetAngle(target_angles[i]);
            }
        }
        control_.state = MotionState::MOVING;
        return 0;
    }

    int8_t MoveCartesian(const ArmPose_t& goal) {
        const bool goal_changed = arm_project::is_pose_changed(
            goal,
            traj_.goal,
            arm_project::kTrajectoryPosThreshold,
            arm_project::kTrajectoryAngThreshold);

        if (!goal_changed && traj_.active) {
            return 0;
        }

        arm_lib::trajectory::CartesianTrajectoryRequest request;
        request.start = arm_project::pose_to_transform(end_effector_.current);
        request.goal = arm_project::pose_to_transform(goal);
        request.max_linear_velocity = traj_.max_lin_vel;
        request.max_angular_velocity = traj_.max_ang_vel;
        request.max_linear_acceleration = traj_.max_lin_acc;
        request.max_angular_acceleration = traj_.max_ang_acc;

        if (!traj_.trajectory.configure(request)) {
            control_.state = MotionState::ERROR;
            return -1;
        }

        traj_.start = end_effector_.current;
        traj_.goal = goal;
        traj_.elapsed = 0.0f;
        traj_.active = true;
        traj_.valid = false;
        end_effector_.target = goal;
        control_.state = MotionState::MOVING;
        return 0;
    }

    int8_t MoveJointTrajectory(const ArmPose_t& goal) {
        if (!chain_configured_) {
            return -1;
        }

        ArmJointAngles_t q_seed{};
        arm_runtime::fill_current_joint_angles(joints_, &q_seed);
        ArmJointAngles_t q_goal{};
        if (arm_runtime::solve_ik(
                chain_, chain_configured_, joints_, goal, &q_goal, &q_seed, false) != 0) {
            control_.state = MotionState::ERROR;
            return -1;
        }

        arm_lib::trajectory::JointTrajectoryRequest<ARM_JOINT_COUNT> request;
        request.start = arm_project::angles_to_joint_vec(q_seed);
        request.goal = arm_project::angles_to_joint_vec(q_goal);

        for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) {
            request.max_velocity[i][0] = joint_traj_.max_velocity[i];
            request.max_acceleration[i][0] = joint_traj_.max_acceleration[i];
        }
        request.enforce_limits = false;

        if (!joint_traj_.trajectory.configure(request)) {
            control_.state = MotionState::ERROR;
            return -1;
        }

        joint_traj_.start = q_seed;
        joint_traj_.goal = q_goal;
        joint_traj_.elapsed = 0.0f;
        joint_traj_.active = true;
        joint_traj_.valid = false;
        end_effector_.target = goal;
        control_.state = MotionState::MOVING;
        return 0;
    }

    void Stop() {
        traj_.active = false;
        joint_traj_.active = false;
        control_.state = MotionState::STOPPED;
    }

    void MoveCartesianDelta(const ArmPoseDelta_t& delta,
                            ArmControlFrame_t frame = ARM_CTRL_FRAME_WORLD) {
        const ArmPose_t goal =
            arm_project::apply_pose_delta(end_effector_.target, delta, frame);
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
        return end_effector_.current;
    }

    const ArmJointAngles_t& GetIkAngles() const {
        return inverse_kinematics_.angles;
    }

    bool IsIkValid() const {
        return inverse_kinematics_.valid;
    }

    void SetLinVelLimit(float vel) {
        traj_.max_lin_vel = vel;
    }

    void SetAngVelLimit(float vel) {
        traj_.max_ang_vel = vel;
    }

    void SetJointTrajectoryVelocity(size_t joint_index, float max_vel) {
        if (joint_index < ARM_JOINT_COUNT && max_vel > 0.0f) {
            joint_traj_.max_velocity[joint_index] = max_vel;
        }
    }

    void SetJointTrajectoryAcceleration(size_t joint_index, float max_accel) {
        if (joint_index < ARM_JOINT_COUNT && max_accel > 0.0f) {
            joint_traj_.max_acceleration[joint_index] = max_accel;
        }
    }

    float GetTrajProgress() const {
        if (!traj_.trajectory.valid()) {
            return traj_.active ? 0.0f : 1.0f;
        }
        const float duration = traj_.trajectory.duration();
        if (duration <= 1.0e-6f) {
            return 1.0f;
        }
        const float progress = traj_.elapsed / duration;
        if (progress < 0.0f) {
            return 0.0f;
        }
        if (progress > 1.0f) {
            return 1.0f;
        }
        return progress;
    }

    bool IsTrajActive() const {
        return traj_.active;
    }

    float GetJointTrajProgress() const {
        if (!joint_traj_.trajectory.valid()) {
            return joint_traj_.active ? 0.0f : 1.0f;
        }
        const float duration = joint_traj_.trajectory.duration();
        if (duration <= 1.0e-6f) {
            return 1.0f;
        }
        const float progress = joint_traj_.elapsed / duration;
        if (progress < 0.0f) {
            return 0.0f;
        }
        if (progress > 1.0f) {
            return 1.0f;
        }
        return progress;
    }

    bool IsJointTrajActive() const {
        return joint_traj_.active;
    }

    void ResetError() {
        control_.state = MotionState::REACHED;
        control_.enable = true;
        inverse_kinematics_.valid = true;
        traj_.valid = true;
        traj_.active = false;
        traj_.elapsed = 0.0f;

        ArmJointAngles_t q{};
        arm_runtime::fill_current_joint_angles(joints_, &q);
        arm_runtime::set_joint_targets_to_current(joints_);
        inverse_kinematics_.angles = q;
        traj_.angles = q;
        traj_.goal = end_effector_.current;
        traj_.start = end_effector_.current;
        end_effector_.target = end_effector_.current;
    }
};

}  // namespace mrobot
