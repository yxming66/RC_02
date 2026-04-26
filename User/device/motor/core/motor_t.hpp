#pragma once

// 强类型电机对象封装。
// 对外提供统一的 Register / Enable / Update / SetXXX 接口，内部委托给对应 Protocol。

#include <type_traits>

#include "device/device.h"
#include "device/motor/core/motor_instance_config.hpp"
#include "device/motor/core/motor_install_spec.hpp"
#include "device/motor/core/motor_kind.hpp"
#include "device/motor/core/motor_model.hpp"
#include "device/motor/core/motor_state.hpp"
#include "device/motor/core/motor_traits.hpp"
#include "device/motor/protocol/dm_protocol.hpp"
#include "device/motor/protocol/lz_protocol.hpp"
#include "device/motor/protocol/rm_protocol.hpp"

namespace mrobot::motor {

template <MotorKind Kind, MotorModel Model>
class MotorT final {
public:
    static_assert(MotorModelValidV<Kind, Model>, "Invalid motor kind/model combination");

    using Traits = MotorTraits<Kind, Model>;
    using Config = MotorInstanceConfig<Kind>;
    using Protocol = MotorProtocol<Kind, Model>;

    explicit MotorT(const Config& config, const MotorInstallSpec& install = kDirectDriveInstall)
        : config_(config), install_(install), protocol_(config, install_, state_, Traits::kVendorModule) {}

    int8_t Register() { return protocol_.Register(); }
    int8_t Enable() { return protocol_.Enable(); }
    int8_t Disable() { return protocol_.Disable(); }
    int8_t Relax() { return protocol_.Relax(); }
    int8_t Update() { return protocol_.Update(); }
    int8_t CommitCommand() {
        int8_t ret = protocol_.CommitCommand();
        state_.command_pending = protocol_.HasPendingCommand();
        return ret;
    }
    bool HasPendingCommand() const { return state_.command_pending; }
    void ClearPendingCommand() {
        protocol_.ClearPendingCommand();
        state_.command_pending = false;
    }
    const auto& ProtocolDebug() const { return protocol_.GetDebugSnapshot(); }

    int8_t SetTorque(float torque_nm) {
        if constexpr (Traits::kSupportsTorque) {
            return protocol_.SetTorque(torque_nm);
        } else {
            return DEVICE_ERR_UNSUPPORTED;
        }
    }

    int8_t SetVelocity(float velocity) {
        if constexpr (Traits::kSupportsVelocity) {
            return protocol_.SetVelocity(velocity);
        } else {
            return DEVICE_ERR_UNSUPPORTED;
        }
    }

    int8_t SetPosition(float position, float max_velocity = 0.0f) {
        if constexpr (Traits::kSupportsPosition) {
            return protocol_.SetPosition(position, max_velocity);
        } else {
            return DEVICE_ERR_UNSUPPORTED;
        }
    }

    int8_t SetMIT(float position, float velocity, float kp, float kd, float torque_ff) {
        if constexpr (Traits::kSupportsMit) {
            return protocol_.SetMIT(position, velocity, kp, kd, torque_ff);
        } else {
            return DEVICE_ERR_UNSUPPORTED;
        }
    }

    MotorState GetState() const { return state_; }
    const MotorInstallSpec& GetInstallConfig() const { return install_; }

    static constexpr MotorKind kind() { return Kind; }
    static constexpr MotorModel model() { return Model; }
    static constexpr const char* name() { return Traits::kName; }
    static constexpr float gear_ratio() { return Traits::kGearRatio; }
    static constexpr float max_position() { return Traits::kMaxPosition; }
    static constexpr float max_velocity() { return Traits::kMaxVelocity; }
    static constexpr float recommended_velocity() { return Traits::kRecommendedVelocity; }
    static constexpr float rated_velocity() { return Traits::kRatedVelocity; }
    static constexpr float recommended_current() { return Traits::kRecommendedCurrent; }
    static constexpr float rated_current() { return Traits::kRatedCurrent; }
    static constexpr float peak_current() { return Traits::kPeakCurrent; }
    static constexpr float torque_constant() { return Traits::kTorqueConstant; }
    static constexpr float rated_torque() { return Traits::kRatedTorque; }
    static constexpr float peak_torque() { return Traits::kPeakTorque; }
    static constexpr MotorCapability capabilities() { return Traits::kCapabilities; }

private:
    Config config_;
    MotorInstallSpec install_;
    MotorState state_{};
    Protocol protocol_;
};

} // namespace mrobot::motor
