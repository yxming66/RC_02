#pragma once

#include <stdint.h>

#include "component/pid.h"
#include "device/motor/core/imotor.hpp"

namespace mrobot::motor {

struct MotorControllerConfig {
    const KPID_Params_t* velocity_pid;
    const KPID_Params_t* position_pid;
    float sample_freq;

    float position_to_velocity_limit;
    float velocity_to_torque_limit;
};

class MotorController final : public IMotor {
public:
    MotorController(IMotor& motor, const MotorControllerConfig& config);

    int8_t Register() override;
    int8_t Enable() override;
    int8_t Disable() override;
    int8_t Update() override;
    int8_t CommitCommand() override;
    bool HasPendingCommand() const override;
    void ClearPendingCommand() override;

    int8_t SetTorque(float torque_nm) override;
    int8_t SetVelocity(float velocity) override;
    int8_t SetPosition(float position, float max_velocity = 0.0f) override;
    int8_t SetMIT(float position, float velocity, float kp, float kd, float torque_ff) override;

    MotorState GetState() const override;
    const MotorSpec& GetSpec() const override;
    const MotorInstallSpec& GetInstallSpec() const override;

private:
    enum class ControlMode : uint8_t {
        Torque = 0,
        Velocity,
        Position,
    };

    int8_t ResetControllers();
    float ResolveVelocityLimit(float request_limit) const;
    float ResolveTorqueLimit(float request_limit) const;

    IMotor& motor_;
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

} // namespace mrobot::motor