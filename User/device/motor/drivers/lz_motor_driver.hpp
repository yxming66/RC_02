#pragma once

#include "device/motor/drivers/motor_driver_base.hpp"

namespace mrobot::motor {

class LzMotorDriver final : public MotorDriverBase {
public:
    LzMotorDriver(const MOTOR_LZ_Param_t& param, const MotorSpec& spec, const MotorInstallSpec& install);

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

    const MOTOR_t* RawMotor() const override;
    bool TryGetRotorFeedback(float& rotor_position_rad,
                             float& rotor_velocity_rad_s,
                             float& torque_current,
                             float& temperature_c) const override;

private:
    PendingCommandType pending_type_ = PendingCommandType::None;
    MOTOR_LZ_MotionParam_t pending_motion_output_{};
    float pending_velocity_ = 0.0f;
    float pending_position_ = 0.0f;
    float pending_position_velocity_limit_ = 0.0f;

    MOTOR_LZ_Param_t param_;
    MOTOR_LZ_t* instance_;
};

} // namespace mrobot::motor
