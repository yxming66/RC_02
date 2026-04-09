#pragma once

namespace mrobot {

inline int8_t Motor::RMRegister(Motor* self) {
    int8_t ret = MOTOR_RM_Register(&self->rm_param_);
    if (ret == DEVICE_OK) {
        self->rm_instance_ = MOTOR_RM_GetMotor(&self->rm_param_);
    }
    return ret;
}

inline int8_t Motor::RMEnable(Motor* self) {
    (void)self;
    return DEVICE_OK;
}

inline int8_t Motor::RMUpdate(Motor* self) {
    int8_t ret = MOTOR_RM_Update(&self->rm_param_);
    if (ret == DEVICE_OK) {
        self->rm_instance_ = MOTOR_RM_GetMotor(&self->rm_param_);
    }
    return ret;
}

inline int8_t Motor::RMRelax(Motor* self) {
    int8_t ret = MOTOR_RM_Relax(&self->rm_param_);
    if (ret != DEVICE_OK) return ret;
    return MOTOR_RM_Ctrl(&self->rm_param_);
}

inline int8_t Motor::RMSetZero(Motor* self) {
    const MOTOR_t* motor = self->RMRawMotor(self);
    if (motor == nullptr) {
        return DEVICE_ERR;
    }
    self->zero_position_offset_rad_ = self->feedback_.rotor_abs_angle;
    return DEVICE_OK;
}

inline int8_t Motor::RMCurrent(Motor* self, float current) {
    return self->RMCurrentControl(current);
}

inline int8_t Motor::RMVelocity(Motor* self, float velocity) {
    if (!self->rm_pid_enabled_ || !self->rm_vel_pid_inited_) {
        return DEVICE_ERR;
    }
    return self->RMCurrentControl(PID_Calc(&self->rm_vel_pid_, velocity, self->GetVelocity(), 0.0f, self->rm_control_period_s_));
}

inline int8_t Motor::RMPosition(Motor* self, float angle, float max_velocity) {
    if (!self->rm_pid_enabled_ || !self->rm_pos_pid_inited_) {
        return DEVICE_ERR;
    }
    float vel_sp = PID_Calc(&self->rm_pos_pid_, angle, self->GetAngle(), 0.0f, self->rm_control_period_s_);
    if (max_velocity > 0.0f) {
        if (vel_sp > max_velocity) vel_sp = max_velocity;
        if (vel_sp < -max_velocity) vel_sp = -max_velocity;
    }
    return RMVelocity(self, vel_sp);
}

inline int8_t Motor::RMMIT(Motor* self, float angle, float velocity, float kp, float kd, float torque) {
    if (!self->rm_pid_enabled_) {
        return DEVICE_ERR;
    }
    const float vel_cmd = velocity + kp * (angle - self->GetAngle()) - kd * self->GetVelocity();
    if (self->rm_vel_pid_inited_) {
        float current_cmd = PID_Calc(&self->rm_vel_pid_, vel_cmd, self->GetVelocity(), 0.0f, self->rm_control_period_s_);
        current_cmd += torque;
        return self->RMCurrentControl(current_cmd);
    }
    return RMVelocity(self, vel_cmd);
}

inline const MOTOR_t* Motor::RMRawMotor(const Motor* self) {
    return self->rm_instance_ ? &self->rm_instance_->motor : nullptr;
}

} // namespace mrobot
