/**
 * @file motor_base.cpp
 * @brief Unified motor facade implementation
 */

#include "device/motors/motor.hpp"

#include <string.h>

namespace mrobot {

Motor::Motor(const MotorConfig& cfg)
    : name_("motor"),
      type_(cfg.type),
      ops_(ResolveOps(cfg.type)),
      reverse_(cfg.reverse),
      lz_param_(cfg.lz_param),
      dm_param_(cfg.dm_param),
      rm_param_(cfg.rm_param),
      lz_instance_(nullptr),
      dm_instance_(nullptr),
      rm_instance_(nullptr),
      mit_default_kp_(cfg.mit_default_kp),
      mit_default_kd_(cfg.mit_default_kd),
      rm_pid_enabled_(false),
      rm_control_period_s_(0.001f),
      rm_current_limit_a_(0.0f),
      zero_position_offset_rad_(0.0f),
      rm_pos_pid_inited_(false),
      rm_vel_pid_inited_(false) {
    if (cfg.name != nullptr && cfg.name[0] != '\0') {
        name_ = cfg.name;
    }

    lz_param_.reverse = reverse_;
    dm_param_.reverse = reverse_;
    rm_param_.reverse = reverse_;

    ConfigurePid(cfg.pid);
    ClearFeedback();
}

int8_t Motor::ConfigurePid(const MotorPidConfig& cfg) {
    rm_pid_enabled_ = false;
    rm_pos_pid_inited_ = false;
    rm_vel_pid_inited_ = false;
    rm_control_period_s_ = (cfg.control_hz > 1.0f) ? (1.0f / cfg.control_hz) : 0.001f;
    rm_current_limit_a_ = cfg.current_limit;
    rm_pos_pid_param_ = cfg.pos_to_vel_pid;
    rm_vel_pid_param_ = cfg.vel_to_cur_pid;

    if (type_ != MotorType::RM || !cfg.enabled) {
        return DEVICE_OK;
    }

    if (cfg.control_hz <= 1.0f || cfg.current_limit <= 0.0f) {
        return DEVICE_ERR;
    }

    rm_pos_pid_inited_ =
        (PID_Init(&rm_pos_pid_, KPID_MODE_CALC_D, cfg.control_hz, &rm_pos_pid_param_) == 0);
    rm_vel_pid_inited_ =
        (PID_Init(&rm_vel_pid_, KPID_MODE_CALC_D, cfg.control_hz, &rm_vel_pid_param_) == 0);
    rm_pid_enabled_ = rm_pos_pid_inited_ && rm_vel_pid_inited_;
    return rm_pid_enabled_ ? DEVICE_OK : DEVICE_ERR;
}

int8_t Motor::Register() {
    if (!ops_ || !ops_->register_fn) return DEVICE_ERR;
    return ops_->register_fn(this);
}

int8_t Motor::Enable() {
    if (!ops_ || !ops_->enable_fn) return DEVICE_ERR;
    return ops_->enable_fn(this);
}

int8_t Motor::Update() {
    if (!ops_ || !ops_->update_fn) return DEVICE_ERR;
    int8_t ret = ops_->update_fn(this);
    RefreshFeedbackCache();
    return ret;
}

int8_t Motor::Relax() {
    if (!ops_ || !ops_->relax_fn) return DEVICE_ERR;
    return ops_->relax_fn(this);
}

bool Motor::GetLZExtraFeedback(MotorLZExtraFeedback* out) const {
    if (out == nullptr || type_ != MotorType::LZ || lz_instance_ == nullptr) {
        return false;
    }
    *out = lz_extra_;
    return true;
}

int8_t Motor::SetZeroPoint() {
    if (!ops_ || !ops_->set_zero_fn) {
        return DEVICE_ERR;
    }
    return ops_->set_zero_fn(this);
}

int8_t Motor::CurrentControl(float current) {
    if (!ops_ || !ops_->current_fn) return DEVICE_ERR;
    return ops_->current_fn(this, current);
}

int8_t Motor::VelocityControl(float velocity) {
    if (!ops_ || !ops_->velocity_fn) return DEVICE_ERR;
    return ops_->velocity_fn(this, velocity);
}

int8_t Motor::PositionControl(float angle, float max_velocity) {
    if (!ops_ || !ops_->position_fn) return DEVICE_ERR;
    return ops_->position_fn(this, angle, max_velocity);
}

int8_t Motor::MITControl(float angle, float velocity, float kp, float kd, float torque) {
    if (!ops_ || !ops_->mit_fn) return DEVICE_ERR;
    if (kp < 0.0f) kp = mit_default_kp_;
    if (kd < 0.0f) kd = mit_default_kd_;
    return ops_->mit_fn(this, angle, velocity, kp, kd, torque);
}

int8_t Motor::RMCurrentControl(float current) {
    if (rm_current_limit_a_ <= 0.0f) {
        return DEVICE_ERR;
    }

    float output = current / rm_current_limit_a_;
    if (output > 1.0f) output = 1.0f;
    if (output < -1.0f) output = -1.0f;

    int8_t ret = MOTOR_RM_SetOutput(&rm_param_, output);
    if (ret != DEVICE_OK) {
        return ret;
    }
    return MOTOR_RM_Ctrl(&rm_param_);
}

void Motor::ClearFeedback() {
    feedback_.rotor_abs_angle = 0.0f;
    feedback_.rotor_speed = 0.0f;
    feedback_.torque_current = 0.0f;
    feedback_.temp = 0.0f;
    feedback_.online = false;
    memset(&lz_extra_, 0, sizeof(lz_extra_));
}

void Motor::RefreshFeedbackCache() {
    const MOTOR_t* motor = RawMotor();
    if (motor == nullptr) {
        ClearFeedback();
        return;
    }

    feedback_.rotor_abs_angle = MOTOR_GetRotorAbsAngle(motor);
    feedback_.rotor_speed = MOTOR_GetRotorSpeed(motor);
    feedback_.torque_current = MOTOR_GetTorqueCurrent(motor);
    feedback_.temp = MOTOR_GetTemp(motor);
    feedback_.online = motor->header.online;

    if (type_ == MotorType::LZ && lz_instance_ != nullptr) {
        lz_extra_.state = lz_instance_->lz_feedback.state;
        lz_extra_.fault = lz_instance_->lz_feedback.fault;
        lz_extra_.motor_can_id = lz_instance_->lz_feedback.motor_can_id;
    }
}

} // namespace mrobot

#include "device/motors/motor_ops_lz.hpp"
#include "device/motors/motor_ops_dm.hpp"
#include "device/motors/motor_ops_rm.hpp"
#include "device/motors/motor_ops_registry.hpp"
