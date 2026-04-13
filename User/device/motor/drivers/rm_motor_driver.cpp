#include "device/motor/drivers/rm_motor_driver.hpp"

#include "device/device.h"
#include "device/motor_rm.h"
#include "component/user_math.h"

namespace mrobot::motor {

RmMotorDriver::RmMotorDriver(const MOTOR_RM_Param_t& param, const MotorSpec& spec, const MotorInstallSpec& install)
    : MotorDriverBase(spec, install), param_(param), instance_(nullptr) {
}

int8_t RmMotorDriver::Register() {
    int8_t ret = MOTOR_RM_Register(&param_);
    if (ret == DEVICE_OK) {
        instance_ = MOTOR_RM_GetMotor(&param_);
    }
    return ret;
}

int8_t RmMotorDriver::Enable() {
    return DEVICE_OK;
}

int8_t RmMotorDriver::Disable() {
    ClearPendingCommand();
    int8_t ret = MOTOR_RM_Relax(&param_);
    if (ret != DEVICE_OK) {
        return ret;
    }
    return MOTOR_RM_Ctrl(&param_);
}

int8_t RmMotorDriver::Update() {
    int8_t ret = MOTOR_RM_Update(&param_);
    if (ret == DEVICE_OK) {
        instance_ = MOTOR_RM_GetMotor(&param_);
        RefreshStateCache();
    }
    return ret;
}

const MOTOR_t* RmMotorDriver::RawMotor() const {
    return instance_ ? &instance_->motor : nullptr;
}

bool RmMotorDriver::TryGetRotorFeedback(float& rotor_position_rad,
                                         float& rotor_velocity_rad_s,
                                         float& torque_current,
                                         float& temperature_c) const {
    if (instance_ == nullptr || MOTOR_RM_GetRawFeedback(const_cast<MOTOR_RM_Param_t*>(&param_)) == nullptr) {
        return false;
    }

    rotor_position_rad = MOTOR_RM_GetRotorPositionRad(const_cast<MOTOR_RM_Param_t*>(&param_));
    rotor_velocity_rad_s = MOTOR_RM_GetRotorVelocityRadS(const_cast<MOTOR_RM_Param_t*>(&param_));
    torque_current = MOTOR_RM_GetTorqueCurrent(const_cast<MOTOR_RM_Param_t*>(&param_));
    temperature_c = MOTOR_RM_GetMotorTemperatureC(const_cast<MOTOR_RM_Param_t*>(&param_));
    return true;
}

int8_t RmMotorDriver::CommitCommand() {
    if (!pending_valid_) {
        SetLastCommitStatus(true);
        return DEVICE_OK;
    }

    int8_t ret = MOTOR_RM_SetTorqueCurrent(&param_, pending_torque_current_);
    if (ret != DEVICE_OK) {
        SetLastCommitStatus(false);
        return ret;
    }
    ret = MOTOR_RM_Ctrl(&param_);
    SetLastCommitStatus(ret == DEVICE_OK);
    if (ret == DEVICE_OK) {
        ClearPendingCommand();
    }
    return ret;
}

bool RmMotorDriver::HasPendingCommand() const {
    return pending_valid_;
}

void RmMotorDriver::ClearPendingCommand() {
    pending_valid_ = false;
    pending_torque_current_ = 0.0f;
    SetPendingStatus(false);
}

int8_t RmMotorDriver::SetTorque(float torque_nm) {
    const float max_current = (spec_.max_current > 0.0f) ? spec_.max_current : 0.0f;
    if (max_current <= 0.0f) {
        return DEVICE_ERR;
    }
    const float torque_current = ToTorqueCurrent(torque_nm);
    if (spec_.torque_constant <= 0.0f || TotalRatio() <= 0.0f) {
        return DEVICE_ERR;
    }
    pending_torque_current_ = AbsClip(torque_current, max_current);
    pending_valid_ = true;
    SetPendingStatus(true);
    return DEVICE_OK;
}

} // namespace mrobot::motor
