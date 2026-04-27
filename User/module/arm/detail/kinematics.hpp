#pragma once

#include <array>

#include "module/arm/arm_control_types.h"
#include "module/arm/detail/utils.hpp"
#include "component/arm_lib/kinematics/fk.h"
#include "component/arm_lib/kinematics/ik_dispatch.h"
#include "component/arm_lib/kinematics/task_constraints.h"
#include "component/arm_lib/model/serial_chain.h"
#include "device/joint/joint.hpp"

namespace mrobot {
namespace arm {

inline void fill_current_joint_angles(
    const std::array<IJoint*, ARM_JOINT_COUNT>& joints,
    ArmJointAngles_t* q) {
    if (q == nullptr) {
        return;
    }
    for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) {
        q->q[i] = (joints[i] != nullptr) ? joints[i]->GetCurrentAngle() : 0.0f;
    }
}

inline ArmPose_t forward_pose(
    const arm_lib::SerialChain<ARM_JOINT_COUNT>& chain,
    const ArmJointAngles_t& q) {
    return arm_project::transform_to_pose(
        arm_lib::kinematics::fk(chain, arm_project::angles_to_joint_vec(q)));
}

inline int8_t solve_ik(
    const arm_lib::SerialChain<ARM_JOINT_COUNT>& chain,
    bool chain_configured,
    const std::array<IJoint*, ARM_JOINT_COUNT>& joints,
    const ArmPose_t& pose,
    ArmJointAngles_t* q_out,
    const ArmJointAngles_t* q_seed,
    bool position_priority_only) {
    if (!chain_configured || q_out == nullptr) {
        return -1;
    }

    ArmPose_t target_pose = pose;
    ArmJointAngles_t seed{};
    if (q_seed != nullptr) {
        seed = *q_seed;
    } else {
        fill_current_joint_angles(joints, &seed);
    }

    arm_lib::kinematics::IKRequest<ARM_JOINT_COUNT> request;
    request.target = arm_project::pose_to_transform(target_pose);
    request.seed = arm_project::angles_to_joint_vec(seed);
    request.reference = request.seed;
    request.use_seed = true;
    request.use_reference = true;

    arm_lib::kinematics::IKOptions options =
        arm_lib::kinematics::make_ik_options(
            position_priority_only
                ? arm_lib::kinematics::IkProfile::kPositionOnly
                : arm_lib::kinematics::IkProfile::kRobust);
    options.error_tolerance = position_priority_only ? 2.0e-3f : 1.0e-3f;
    options.joint_centering_gain = 0.08f;
    options.null_space_max_step = 0.08f;

    if (position_priority_only) {
        const ArmPose_t seed_pose =
            arm_project::transform_to_pose(
                arm_lib::kinematics::fk(chain, request.seed));
        target_pose.roll = seed_pose.roll;
        target_pose.yaw = seed_pose.yaw;
        request.target = arm_project::pose_to_transform(target_pose);

        options = arm_lib::kinematics::make_yz_pitch_ik_options(
            request.target,
            arm_lib::kinematics::IkProfile::kPositionOnly);
        options.error_tolerance = 2.0e-3f;
        options.joint_centering_gain = 0.08f;
        options.null_space_max_step = 0.08f;
    }

    const auto result = arm_lib::kinematics::solve_ik(chain, request, options);
    if (!arm_lib::is_success(result.status)) {
        return -1;
    }

    *q_out = arm_project::joint_vec_to_angles(result.q);
    return 0;
}

}  // namespace arm
}  // namespace mrobot
