#pragma once

#include "device/motor/drivers/motor_driver_base.hpp"

namespace mrobot::motor {

class RmMotorDriver final : public MotorDriverBase {
public:
    RmMotorDriver(const MOTOR_RM_Param_t& param, const MotorSpec& spec, const MotorInstallSpec& install);

    int8_t Register() override;
    int8_t Enable() override;
    int8_t Disable() override;
    int8_t Update() override;
    int8_t CommitCommand() override;
    bool HasPendingCommand() const override;
    void ClearPendingCommand() override;

    int8_t SetTorque(float torque_nm) override;

    const MOTOR_t* RawMotor() const override;
    bool TryGetRotorFeedback(float& rotor_position_rad,
                             float& rotor_velocity_rad_s,
                             float& torque_current,
                             float& temperature_c) const override;

private:
    float pending_torque_current_ = 0.0f;
    bool pending_valid_ = false;

    MOTOR_RM_Param_t param_;
    MOTOR_RM_t* instance_;
};

} // namespace mrobot::motor
