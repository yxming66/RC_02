#pragma once

#include <stdint.h>

#include "device/device.h"
#include "device/motor/core/motor_capability.hpp"
#include "device/motor/core/motor_spec.hpp"
#include "device/motor/core/motor_state.hpp"

namespace mrobot::motor {

#if defined(__GNUC__) || defined(__clang__)
#define MROBOT_MOTOR_DEPRECATED(msg) __attribute__((deprecated(msg)))
#elif defined(_MSC_VER)
#define MROBOT_MOTOR_DEPRECATED(msg) __declspec(deprecated(msg))
#else
#define MROBOT_MOTOR_DEPRECATED(msg)
#endif

class IMotor {
public:
    virtual ~IMotor() = default;

    virtual int8_t Register() = 0;
    virtual int8_t Enable() = 0;
    virtual int8_t Disable() = 0;
    virtual int8_t Update() = 0;
    virtual int8_t CommitCommand() = 0;
    virtual bool HasPendingCommand() const = 0;
    virtual void ClearPendingCommand() = 0;

    // 统一为输出轴前馈力矩，单位 N·m。
    virtual int8_t SetTorque(float torque_nm) = 0;
    // 统一为输出轴角速度，单位 rad/s。
    virtual int8_t SetVelocity(float velocity) { (void)velocity; return DEVICE_ERR; }
    // 统一为输出轴角度与输出轴最大角速度，单位 rad / rad/s。
    virtual int8_t SetPosition(float position, float max_velocity = 0.0f) {
        (void)position;
        (void)max_velocity;
        return DEVICE_ERR;
    }
    // 统一为输出轴角度、输出轴角速度、输出轴前馈力矩。
    virtual int8_t SetMIT(float position, float velocity, float kp, float kd, float torque_ff) {
        (void)position;
        (void)velocity;
        (void)kp;
        (void)kd;
        (void)torque_ff;
        return DEVICE_ERR;
    }

    virtual MotorState GetState() const = 0;
    virtual const MotorSpec& GetSpec() const = 0;
    virtual const MotorInstallSpec& GetInstallSpec() const = 0;

    // 兼容旧关节层接口，后续可逐步移除。
    MROBOT_MOTOR_DEPRECATED("Use GetState().position_rad instead of GetAngle()")
    float GetAngle() const { return GetState().position_rad; }
    MROBOT_MOTOR_DEPRECATED("Use GetState().velocity_rad_s instead of GetVelocity()")
    float GetVelocity() const { return GetState().velocity_rad_s; }
    MROBOT_MOTOR_DEPRECATED("Use GetState().torque_nm instead of GetTorque()")
    float GetTorque() const { return GetState().torque_nm; }
    MROBOT_MOTOR_DEPRECATED("Use GetState().online instead of IsOnline()")
    bool IsOnline() const { return GetState().online; }
    MROBOT_MOTOR_DEPRECATED("Use Disable() instead of Relax()")
    int8_t Relax() { return Disable(); }
    MROBOT_MOTOR_DEPRECATED("Use SetMIT() + CommitCommand() instead of MITControl()")
    int8_t MITControl(float position, float velocity, float kp, float kd, float torque_ff) {
        const int8_t ret = SetMIT(position, velocity, kp, kd, torque_ff);
        if (ret != DEVICE_OK) {
            return ret;
        }
        return CommitCommand();
    }

    MotorCapability GetCapabilities() const { return GetSpec().capabilities; }
    bool Supports(MotorCapability capability) const {
        return HasCapability(GetCapabilities(), capability);
    }
};

} // namespace mrobot::motor

#undef MROBOT_MOTOR_DEPRECATED
