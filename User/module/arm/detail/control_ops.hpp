#pragma once

#include <array>
#include <cmath>

#include "module/arm/arm_control_types.h"
#include "device/joint/joint.hpp"

namespace mr {
namespace arm {

struct PositionDriveResult {
    bool all_reached;
    bool all_online;
    bool command_ok;

    PositionDriveResult()
        : all_reached(true), all_online(true), command_ok(true) {}
};

inline void relax_all_joints(
    const std::array<IJoint*, ARM_JOINT_COUNT>& joints) {
    for (IJoint* joint : joints) {
        if (joint != nullptr) {
            joint->ClearPendingControl();
            joint->Relax();
        }
    }
}

inline void set_joint_targets_to_current(
    const std::array<IJoint*, ARM_JOINT_COUNT>& joints) {
    for (IJoint* joint : joints) {
        if (joint != nullptr) {
            joint->SetTargetAngle(joint->GetCurrentAngle());
        }
    }
}

inline void apply_feedforward_torques(
    const std::array<IJoint*, ARM_JOINT_COUNT>& joints,
    const float torques[ARM_JOINT_COUNT]) {
    if (torques == nullptr) {
        return;
    }
    for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) {
        if (joints[i] != nullptr) {
            joints[i]->SetFeedforwardTorque(torques[i]);
        }
    }
}

inline void clear_feedforward_torques(
    const std::array<IJoint*, ARM_JOINT_COUNT>& joints) {
    for (IJoint* joint : joints) {
        if (joint != nullptr) {
            joint->SetFeedforwardTorque(0.0f);
        }
    }
}

inline bool validate_and_copy_joint_targets(
    const std::array<IJoint*, ARM_JOINT_COUNT>& joints,
    ArmJointAngles_t* out_targets) {
    if (out_targets == nullptr) {
        return false;
    }

    for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) {
        out_targets->q[i] = 0.0f;
        if (joints[i] == nullptr) {
            continue;
        }

        const float target = joints[i]->GetTargetAngle();
        const auto& params = joints[i]->GetParams();
        if (!std::isfinite(target) || target < params.qmin || target > params.qmax) {
            return false;
        }
        out_targets->q[i] = target;
    }
    return true;
}

inline void apply_joint_targets(
    const std::array<IJoint*, ARM_JOINT_COUNT>& joints,
    const ArmJointAngles_t& targets) {
    for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) {
        if (joints[i] != nullptr) {
            joints[i]->SetTargetAngle(targets.q[i]);
        }
    }
}

inline PositionDriveResult drive_position_targets_and_commit(
    const std::array<IJoint*, ARM_JOINT_COUNT>& joints,
    float dt,
    float position_tolerance) {
    PositionDriveResult result;
    for (IJoint* joint : joints) {
        if (joint != nullptr) {
            if (!joint->IsOnline()) {
                result.all_online = false;
                result.command_ok = false;
                continue;
            }

            if (joint->PositionControl(joint->GetTargetAngle(), dt) != 0) {
                result.command_ok = false;
            }
            if (!joint->IsReached(position_tolerance)) {
                result.all_reached = false;
            }
        }
    }
    for (IJoint* joint : joints) {
        if (joint != nullptr) {
            if (joint->CommitControl() != 0) {
                result.command_ok = false;
            }
        }
    }
    return result;
}

inline void drive_torque_targets_and_commit(
    const std::array<IJoint*, ARM_JOINT_COUNT>& joints,
    const float torques[ARM_JOINT_COUNT]) {
    if (torques == nullptr) {
        return;
    }
    for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) {
        if (joints[i] != nullptr) {
            joints[i]->TorqueControl(torques[i]);
        }
    }
    for (IJoint* joint : joints) {
        if (joint != nullptr) {
            joint->CommitControl();
        }
    }
}

}  // namespace arm
}  // namespace mr
