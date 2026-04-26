#pragma once

#include <array>
#include <cmath>

#include "arm_control_types.h"
#include "joint.hpp"

namespace mrobot {
namespace arm_runtime {

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

inline bool drive_position_targets_and_commit(
    const std::array<IJoint*, ARM_JOINT_COUNT>& joints,
    float dt,
    float position_tolerance) {
    bool all_reached = true;
    for (IJoint* joint : joints) {
        if (joint != nullptr) {
            joint->PositionControl(joint->GetTargetAngle(), dt);
            if (!joint->IsReached(position_tolerance)) {
                all_reached = false;
            }
        }
    }
    for (IJoint* joint : joints) {
        if (joint != nullptr) {
            joint->CommitControl();
        }
    }
    return all_reached;
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

}  // namespace arm_runtime
}  // namespace mrobot
