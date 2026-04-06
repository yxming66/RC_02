#pragma once

namespace mrobot {

inline int8_t Motor::LZRegister(Motor* self) {
    int8_t ret = MOTOR_LZ_Register(&self->lz_param_);
    if (ret == DEVICE_OK) {
        self->lz_instance_ = MOTOR_LZ_GetMotor(&self->lz_param_);
    }
    return ret;
}

inline int8_t Motor::LZEnable(Motor* self) {
    return MOTOR_LZ_Enable(&self->lz_param_);
}

inline int8_t Motor::LZUpdate(Motor* self) {
    int8_t ret = MOTOR_LZ_Update(&self->lz_param_);
    if (ret == DEVICE_OK) {
        self->lz_instance_ = MOTOR_LZ_GetMotor(&self->lz_param_);
    }
    return ret;
}

inline int8_t Motor::LZRelax(Motor* self) {
    return MOTOR_LZ_Relax(&self->lz_param_);
}

inline int8_t Motor::LZCurrent(Motor* self, float current) {
    return self->MITControl(self->GetAngle(), 0.0f, 0.0f, 0.0f, current);
}

inline int8_t Motor::LZVelocity(Motor* self, float velocity) {
    return self->MITControl(self->GetAngle(), velocity, 0.0f, 0.0f, 0.0f);
}

inline int8_t Motor::LZPosition(Motor* self, float angle, float max_velocity) {
    return self->MITControl(angle, max_velocity, self->mit_default_kp_, self->mit_default_kd_, 0.0f);
}

inline int8_t Motor::LZMIT(Motor* self, float angle, float velocity, float kp, float kd, float torque) {
    MOTOR_LZ_MotionParam_t output{};
    output.target_angle = angle;
    output.target_velocity = velocity;
    output.kp = kp;
    output.kd = kd;
    output.torque = torque;
    return MOTOR_LZ_MotionControl(&self->lz_param_, &output);
}

inline const MOTOR_t* Motor::LZRawMotor(const Motor* self) {
    return self->lz_instance_ ? &self->lz_instance_->motor : nullptr;
}

} // namespace mrobot
