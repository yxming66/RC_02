/**
 * @file motor_controller.hpp
 * @brief 电机控制适配层：在不改动底层能力的前提下提供可选的PID降级控制
 */

#pragma once

#include "device/motors/motor_base.hpp"

namespace mrobot {

struct MotorFallbackConfig {
    bool enable_fallback;
    float control_hz;

    // 位置降级：position -> velocity
    KPID_Params_t pos_to_vel_pid;

    // 速度降级：velocity -> current
    KPID_Params_t vel_to_cur_pid;
};

class MotorController {
public:
    explicit MotorController(Motor* motor) : motor_(motor) {}

    int8_t InitFallback(const MotorFallbackConfig& cfg) {
        if (motor_ == nullptr) {
            return DEVICE_ERR_NULL;
        }

        MotorPidConfig pid_cfg{};
        pid_cfg.enabled = cfg.enable_fallback;
        pid_cfg.control_hz = cfg.control_hz;
        pid_cfg.current_limit = 10.0f;
        pid_cfg.pos_to_vel_pid = cfg.pos_to_vel_pid;
        pid_cfg.vel_to_cur_pid = cfg.vel_to_cur_pid;
        return motor_->ConfigurePid(pid_cfg);
    }

    int8_t Register() { return motor_ ? motor_->Register() : DEVICE_ERR_NULL; }
    int8_t Enable() { return motor_ ? motor_->Enable() : DEVICE_ERR_NULL; }
    int8_t Update() { return motor_ ? motor_->Update() : DEVICE_ERR_NULL; }
    int8_t Relax() { return motor_ ? motor_->Relax() : DEVICE_ERR_NULL; }

    float GetAngle() const { return motor_ ? motor_->GetAngle() : 0.0f; }
    float GetVelocity() const { return motor_ ? motor_->GetVelocity() : 0.0f; }
    float GetTorque() const { return motor_ ? motor_->GetTorque() : 0.0f; }
    bool IsOnline() const { return motor_ ? motor_->IsOnline() : false; }

    int8_t CurrentControl(float current) {
        if (motor_ == nullptr) {
            return DEVICE_ERR_NULL;
        }
        return motor_->CurrentControl(current);
    }

    int8_t VelocityControl(float velocity) {
        if (motor_ == nullptr) {
            return DEVICE_ERR_NULL;
        }
        return motor_->VelocityControl(velocity);
    }

    int8_t PositionControl(float angle, float max_velocity) {
        if (motor_ == nullptr) {
            return DEVICE_ERR_NULL;
        }
        return motor_->PositionControl(angle, max_velocity);
    }

    int8_t MITControl(float angle, float velocity, float kp, float kd, float torque_ff) {
        if (motor_ == nullptr) {
            return DEVICE_ERR_NULL;
        }

        return motor_->MITControl(angle, velocity, kp, kd, torque_ff);

        // 近似MIT降级：
        // vel_cmd = velocity + kp*(pos_err) - kd*(vel_fb)
        // 若可用电流环，则叠加 torque_ff；否则降级到速度命令。
    }

private:
    Motor* motor_;

    bool fallback_enabled_;
    float dt_;

    KPID_t pos_pid_;
    KPID_t vel_pid_;
    KPID_Params_t pos_param_;
    KPID_Params_t vel_param_;

    bool pos_pid_inited_;
    bool vel_pid_inited_;
};

} // namespace mrobot
