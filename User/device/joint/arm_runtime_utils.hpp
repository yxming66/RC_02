#pragma once

#include <cmath>
#include <stddef.h>
#include <stdint.h>

#include "arm_control_types.h"
#include "component/arm_lib/adapter/toolbox_adapter.h"
#include "component/arm_lib/core/arm_types.h"

namespace mrobot {
namespace arm_project {

static constexpr size_t kJointNum = ARM_JOINT_COUNT;
static constexpr float kTrajectoryPosThreshold = 0.001f;
static constexpr float kTrajectoryAngThreshold = 0.01f;
static constexpr float kTrajectoryMaxJointDelta = 0.45f;
static constexpr float kDefaultJointTrajMaxVelocity = 2.0f;
static constexpr float kDefaultJointTrajMaxAcceleration = 5.0f;

inline arm_lib::Vec3 pose_to_rpy(const ArmPose_t& pose) {
    arm_lib::Vec3 rpy = arm_lib::toolbox_adapter::zero_vec3();
    rpy[0][0] = pose.yaw;
    rpy[1][0] = pose.pitch;
    rpy[2][0] = pose.roll;
    return rpy;
}

inline arm_lib::Vec3 pose_to_translation(const ArmPose_t& pose) {
    arm_lib::Vec3 translation = arm_lib::toolbox_adapter::zero_vec3();
    translation[0][0] = pose.x;
    translation[1][0] = pose.y;
    translation[2][0] = pose.z;
    return translation;
}

inline arm_lib::Transform pose_to_transform(const ArmPose_t& pose) {
    return arm_lib::toolbox_adapter::transform_from_rpy_translation(
        pose_to_rpy(pose), pose_to_translation(pose));
}

inline ArmPose_t transform_to_pose(const arm_lib::Transform& transform) {
    const arm_lib::Vec3 translation =
        arm_lib::toolbox_adapter::translation_of(transform);
    const arm_lib::Vec3 rpy =
        arm_lib::toolbox_adapter::rpy_from_rotation(
            arm_lib::toolbox_adapter::rotation_of(transform));

    ArmPose_t pose{};
    pose.x = translation[0][0];
    pose.y = translation[1][0];
    pose.z = translation[2][0];
    pose.roll = rpy[2][0];
    pose.pitch = rpy[1][0];
    pose.yaw = rpy[0][0];
    return pose;
}

inline arm_lib::JointVec<ARM_JOINT_COUNT> angles_to_joint_vec(
    const ArmJointAngles_t& angles) {
    arm_lib::JointVec<ARM_JOINT_COUNT> q =
        arm_lib::toolbox_adapter::zero_joint_vec<ARM_JOINT_COUNT>();
    for (size_t i = 0; i < kJointNum; ++i) {
        q[i][0] = angles.q[i];
    }
    return q;
}

inline ArmJointAngles_t joint_vec_to_angles(
    const arm_lib::JointVec<ARM_JOINT_COUNT>& q) {
    ArmJointAngles_t angles{};
    for (size_t i = 0; i < kJointNum; ++i) {
        angles.q[i] = q[i][0];
    }
    return angles;
}

inline ArmPose_t apply_pose_delta(const ArmPose_t& base,
                                  const ArmPoseDelta_t& delta,
                                  ArmControlFrame_t frame) {
    ArmPose_t result = base;
    if (frame == ARM_CTRL_FRAME_TOOL) {
        ArmPose_t delta_pose{};
        delta_pose.x = delta.x;
        delta_pose.y = delta.y;
        delta_pose.z = delta.z;
        delta_pose.roll = delta.roll;
        delta_pose.pitch = delta.pitch;
        delta_pose.yaw = delta.yaw;
        return transform_to_pose(
            pose_to_transform(base) * pose_to_transform(delta_pose));
    }

    if (frame == ARM_CTRL_FRAME_HEADING) {
        const float cy = cosf(base.yaw);
        const float sy = sinf(base.yaw);
        result.x += cy * delta.x - sy * delta.y;
        result.y += sy * delta.x + cy * delta.y;
        result.z += delta.z;
        result.roll += delta.roll;
        result.pitch += delta.pitch;
        result.yaw += delta.yaw;
        return result;
    }

    result.x += delta.x;
    result.y += delta.y;
    result.z += delta.z;
    result.roll += delta.roll;
    result.pitch += delta.pitch;
    result.yaw += delta.yaw;
    return result;
}

inline bool is_pose_changed(const ArmPose_t& lhs,
                            const ArmPose_t& rhs,
                            float pos_eps,
                            float ang_eps) {
    return fabsf(lhs.x - rhs.x) > pos_eps ||
           fabsf(lhs.y - rhs.y) > pos_eps ||
           fabsf(lhs.z - rhs.z) > pos_eps ||
           fabsf(lhs.roll - rhs.roll) > ang_eps ||
           fabsf(lhs.pitch - rhs.pitch) > ang_eps ||
           fabsf(lhs.yaw - rhs.yaw) > ang_eps;
}

inline bool joint_step_is_reasonable(const ArmJointAngles_t& next_q,
                                     const ArmJointAngles_t& prev_q,
                                     float max_joint_delta) {
    if (max_joint_delta <= 0.0f) {
        return true;
    }
    for (size_t i = 0; i < kJointNum; ++i) {
        if (fabsf(next_q.q[i] - prev_q.q[i]) > max_joint_delta) {
            return false;
        }
    }
    return true;
}

}  // namespace arm_project
}  // namespace mrobot
