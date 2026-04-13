#include "device/motor/drivers/lz_motor_driver.hpp"

#include "device/device.h"
#include "device/motor_lz.h"
#include "component/user_math.h"

namespace mrobot::motor {

LzMotorDriver::LzMotorDriver(const MOTOR_LZ_Param_t& param, const MotorSpec& spec, const MotorInstallSpec& install)
    : MotorDriverBase(spec, install), param_(param), instance_(nullptr) {
}

int8_t LzMotorDriver::Register() {
    int8_t ret = MOTOR_LZ_Register(&param_);
    if (ret == DEVICE_OK) {
        instance_ = MOTOR_LZ_GetMotor(&param_);
    }
    return ret;
}

int8_t LzMotorDriver::Enable() {
    return MOTOR_LZ_Enable(&param_);
}

int8_t LzMotorDriver::Disable() {
    ClearPendingCommand();
    return MOTOR_LZ_Relax(&param_);
}

int8_t LzMotorDriver::Update() {
    int8_t ret = MOTOR_LZ_Update(&param_);
    if (ret == DEVICE_OK) {
        instance_ = MOTOR_LZ_GetMotor(&param_);
        RefreshStateCache();
    }
    return ret;
}

const MOTOR_t* LzMotorDriver::RawMotor() const {
    return instance_ ? &instance_->motor : nullptr;
}

bool LzMotorDriver::TryGetRotorFeedback(float& rotor_position_rad,
                                         float& rotor_velocity_rad_s,
                                         float& torque_current,
                                         float& temperature_c) const {
    if (MOTOR_LZ_GetRawFeedback(const_cast<MOTOR_LZ_Param_t*>(&param_)) == nullptr) {
        return false;
    }

    rotor_position_rad = MOTOR_LZ_GetRotorPositionRad(const_cast<MOTOR_LZ_Param_t*>(&param_));
    rotor_velocity_rad_s = MOTOR_LZ_GetRotorVelocityRadS(const_cast<MOTOR_LZ_Param_t*>(&param_));
    torque_current = MOTOR_LZ_GetTorqueCurrent(const_cast<MOTOR_LZ_Param_t*>(&param_));
    temperature_c = MOTOR_LZ_GetMotorTemperatureC(const_cast<MOTOR_LZ_Param_t*>(&param_));
    return true;
}

int8_t LzMotorDriver::SetTorque(float torque_nm) {
    const float max_torque_nm = (spec_.peak_torque > 0.0f)
        ? spec_.peak_torque
        : ((spec_.rated_torque > 0.0f) ? spec_.rated_torque : 0.0f);
    if (max_torque_nm <= 0.0f) {
        return DEVICE_ERR;
    }
    return SetMIT(0.0f, 0.0f, 0.0f, 0.0f, AbsClip(torque_nm, max_torque_nm));
}

int8_t LzMotorDriver::SetVelocity(float velocity) {
    const float rotor_velocity = ToRotorVelocity(velocity);
    const float rotor_velocity_limit = ToRotorVelocity(spec_.max_velocity);
    pending_velocity_ = AbsClip(rotor_velocity, rotor_velocity_limit);
    pending_type_ = PendingCommandType::Velocity;
    SetPendingStatus(true);
    return DEVICE_OK;
}

int8_t LzMotorDriver::SetPosition(float position, float max_velocity) {
    const float output_velocity_limit = (max_velocity > 0.0f) ? AbsClip(max_velocity, spec_.max_velocity) : spec_.max_velocity;
    const float rotor_position = ToRotorPosition(position);
    const float rotor_position_limit = (spec_.max_position > 0.0f) ? ToRotorPosition(spec_.max_position) : 0.0f;
    const float rotor_velocity_limit = ToRotorVelocity(output_velocity_limit);
    const float clipped_rotor_position = (rotor_position_limit > 0.0f)
        ? AbsClip(rotor_position, rotor_position_limit)
        : rotor_position;
    pending_position_ = clipped_rotor_position;
    pending_position_velocity_limit_ = rotor_velocity_limit;
    pending_type_ = PendingCommandType::Position;
    SetPendingStatus(true);
    return DEVICE_OK;
}

int8_t LzMotorDriver::SetMIT(float position, float velocity, float kp, float kd, float torque_ff) {
    const float rotor_position = ToRotorPosition(position);
    const float rotor_velocity = ToRotorVelocity(velocity);
    const float rotor_position_limit = (spec_.max_position > 0.0f) ? ToRotorPosition(spec_.max_position) : 0.0f;
    const float rotor_velocity_limit = ToRotorVelocity(spec_.max_velocity);

    pending_motion_output_.target_angle = (rotor_position_limit > 0.0f) ? AbsClip(rotor_position, rotor_position_limit) : rotor_position;
    pending_motion_output_.target_velocity = AbsClip(rotor_velocity, rotor_velocity_limit);
    pending_motion_output_.kp = kp;
    pending_motion_output_.kd = kd;
    const float max_torque_nm = (spec_.peak_torque > 0.0f)
        ? spec_.peak_torque
        : ((spec_.rated_torque > 0.0f) ? spec_.rated_torque : 0.0f);
    pending_motion_output_.torque = (max_torque_nm > 0.0f)
        ? AbsClip(torque_ff, max_torque_nm)
        : torque_ff;
    pending_type_ = PendingCommandType::Mit;
    SetPendingStatus(true);
    return DEVICE_OK;
}

int8_t LzMotorDriver::CommitCommand() {
    int8_t ret = DEVICE_OK;
    switch (pending_type_) {
        case PendingCommandType::Velocity:
            ret = MOTOR_LZ_VelocityControl(&param_, pending_velocity_);
            break;
        case PendingCommandType::Position:
            ret = MOTOR_LZ_PositionControl(&param_, pending_position_, pending_position_velocity_limit_);
            break;
        case PendingCommandType::Mit:
        case PendingCommandType::Torque:
            ret = MOTOR_LZ_MotionControl(&param_, &pending_motion_output_);
            break;
        case PendingCommandType::None:
        default:
            SetLastCommitStatus(true);
            return DEVICE_OK;
    }

    SetLastCommitStatus(ret == DEVICE_OK);
    if (ret == DEVICE_OK) {
        ClearPendingCommand();
    }
    return ret;
}

bool LzMotorDriver::HasPendingCommand() const {
    return pending_type_ != PendingCommandType::None;
}

void LzMotorDriver::ClearPendingCommand() {
    pending_type_ = PendingCommandType::None;
    pending_velocity_ = 0.0f;
    pending_position_ = 0.0f;
    pending_position_velocity_limit_ = 0.0f;
    pending_motion_output_ = {};
    SetPendingStatus(false);
}

} // namespace mrobot::motor
