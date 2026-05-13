#pragma once

#include <array>

#include "module/arm/arm_control_types.h"
#include "module/arm/detail/kinematics.hpp"
#include "module/arm/detail/utils.hpp"
#include "robotics/arm/trajectory/cartesian_traj.h"
#include "robotics/arm/trajectory/joint_traj.h"
#include "device/joint/joint.hpp"

namespace mr {
namespace arm {

struct CartesianTrajectoryStepResult {
    ArmPose_t pose;
    ArmJointAngles_t angles;
    bool valid;
    bool active;

    CartesianTrajectoryStepResult()
        : pose(), angles(), valid(false), active(false) {}
};

inline CartesianTrajectoryStepResult step_cartesian_trajectory(
    arm_lib::trajectory::CartesianTrajectory* trajectory,
    float* elapsed,
    float dt,
    const std::array<IJoint*, ARM_JOINT_COUNT>& joints,
    const arm_lib::SerialChain<ARM_JOINT_COUNT>& chain,
    bool chain_configured,
    bool previous_valid,
    const ArmJointAngles_t& previous_angles,
    float max_joint_delta,
    bool position_priority_only) {
    CartesianTrajectoryStepResult out;
    if (trajectory == nullptr || elapsed == nullptr || !trajectory->valid()) {
        return out;
    }

    *elapsed += dt;
    const auto sample = trajectory->sample(*elapsed);
    if (!sample.valid) {
        return out;
    }

    const ArmPose_t interp = arm_project::transform_to_pose(sample.pose);
    ArmJointAngles_t q_seed{};
    if (previous_valid) {
        q_seed = previous_angles;
    } else {
        fill_current_joint_angles(joints, &q_seed);
    }

    ArmJointAngles_t q_next{};
    if (solve_ik(
            chain,
            chain_configured,
            joints,
            interp,
            &q_next,
            &q_seed,
            position_priority_only) != 0) {
        return out;
    }

    if (!arm_project::joint_step_is_reasonable(q_next, q_seed, max_joint_delta)) {
        return out;
    }

    out.pose = interp;
    out.angles = q_next;
    out.valid = true;
    out.active = !sample.finished;
    return out;
}

struct JointTrajectoryStepResult {
    ArmPose_t pose;
    ArmJointAngles_t angles;
    bool valid;
    bool active;

    JointTrajectoryStepResult()
        : pose(), angles(), valid(false), active(false) {}
};

inline JointTrajectoryStepResult step_joint_trajectory(
    arm_lib::trajectory::JointTrajectory<ARM_JOINT_COUNT>* trajectory,
    float* elapsed,
    float dt,
    const arm_lib::SerialChain<ARM_JOINT_COUNT>& chain) {
    JointTrajectoryStepResult out;
    if (trajectory == nullptr || elapsed == nullptr || !trajectory->valid()) {
        return out;
    }

    *elapsed += dt;
    const auto sample = trajectory->sample(*elapsed);
    if (!sample.valid) {
        return out;
    }

    out.angles = arm_project::joint_vec_to_angles(sample.position);
    out.pose = arm_project::transform_to_pose(
        arm_lib::kinematics::fk(chain, sample.position));
    out.valid = true;
    out.active = !sample.finished;
    return out;
}

}  // namespace arm
}  // namespace mr
