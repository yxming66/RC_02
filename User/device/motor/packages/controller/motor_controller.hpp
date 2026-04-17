#pragma once

// 电机控制器包装层。
// 在单个 MotorT 之上叠加位置/速度 PID 串级能力，提供更高层的闭环控制接口。

#include <stdint.h>

#include "component/pid.h"
#include "device/motor/core/motor_install_spec.hpp"
#include "device/motor/motor.hpp"

namespace mrobot::motor {

struct MotorControllerConfig {
    const KPID_Params_t* velocity_pid;
    const KPID_Params_t* position_pid;
    float sample_freq;

    float position_to_velocity_limit;
    float velocity_to_torque_limit;
};

template <typename MotorType>
class MotorControllerT final {
public:
    MotorControllerT(MotorType& motor, const MotorControllerConfig& config);

    int8_t Register();
    int8_t Enable();
    int8_t Disable();
    int8_t Relax();
    int8_t Update();
    int8_t CommitCommand();
    bool HasPendingCommand() const;
    void ClearPendingCommand();

    int8_t SetTorque(float torque_nm);
    int8_t SetVelocity(float velocity);
    int8_t SetPosition(float position, float max_velocity = 0.0f);
    int8_t SetMIT(float position, float velocity, float kp, float kd, float torque_ff);

    MotorState GetState() const;
    const MotorInstallSpec& GetInstallConfig() const;

private:
    enum class ControlMode : uint8_t {
        Torque = 0,
        Velocity,
        Position,
    };

    int8_t ResetControllers();
    float ResolveVelocityLimit(float request_limit) const;
    float ResolveTorqueLimit(float request_limit) const;

    MotorType& motor_;
    MotorControllerConfig config_;

    KPID_t velocity_pid_;
    KPID_t position_pid_;

    ControlMode mode_;
    float target_torque_;
    float target_velocity_;
    float target_position_;
    float position_velocity_limit_;
    float velocity_torque_limit_;
};

using MotorController = MotorControllerT<mrobot::motor::RmM3508Motor>;

} // namespace mrobot::motor