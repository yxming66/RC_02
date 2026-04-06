#pragma once

namespace mrobot {

inline int8_t Motor::DMRegister(Motor* self) {
    int8_t ret = MOTOR_DM_Register(&self->dm_param_);
    if (ret == DEVICE_OK) {
        self->dm_instance_ = MOTOR_DM_GetMotor(&self->dm_param_);
    }
    return ret;
}

inline int8_t Motor::DMEnable(Motor* self) {
    return MOTOR_DM_Enable(&self->dm_param_);
}

inline int8_t Motor::DMUpdate(Motor* self) {
    int8_t ret = MOTOR_DM_Update(&self->dm_param_);
    if (ret == DEVICE_OK) {
        self->dm_instance_ = MOTOR_DM_GetMotor(&self->dm_param_);
    }
    return ret;
}

inline int8_t Motor::DMRelax(Motor* self) {
    return MOTOR_DM_Relax(&self->dm_param_);
}

inline int8_t Motor::DMCurrent(Motor* self, float current) {
    return self->MITControl(self->GetAngle(), 0.0f, 0.0f, 0.0f, current);
}

inline int8_t Motor::DMVelocity(Motor* self, float velocity) {
    return self->MITControl(self->GetAngle(), velocity, 0.0f, 0.0f, 0.0f);
}

inline int8_t Motor::DMPosition(Motor* self, float angle, float max_velocity) {
    return self->MITControl(angle, max_velocity, self->mit_default_kp_, self->mit_default_kd_, 0.0f);
}

inline int8_t Motor::DMMIT(Motor* self, float angle, float velocity, float kp, float kd, float torque) {
    MOTOR_MIT_Output_t output{};
    output.angle = angle;
    output.velocity = velocity;
    output.kp = kp;
    output.kd = kd;
    output.torque = torque;
    return MOTOR_DM_MITCtrl(&self->dm_param_, &output);
}

inline const MOTOR_t* Motor::DMRawMotor(const Motor* self) {
    return self->dm_instance_ ? &self->dm_instance_->motor : nullptr;
}

} // namespace mrobot
