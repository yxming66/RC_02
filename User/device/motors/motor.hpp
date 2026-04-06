/**
 * @file motor_base.hpp
 * @brief 统一电机门面（策略分发）
 */

#pragma once

#include <stdint.h>
#include "component/pid.h"
#include "device/motor.h"
#include "device/motor_dm.h"
#include "device/motor_lz.h"
#include "device/motor_rm.h"

namespace mrobot {

enum class MotorType : uint8_t {
    LZ = 0,
    DM = 1,
    RM = 2,
};

struct MotorFeedback {
    float rotor_abs_angle;
    float rotor_speed;
    float torque_current;
    float temp;
    bool online;
};

struct MotorLZExtraFeedback {
    MOTOR_LZ_State_t state;
    MOTOR_LZ_Fault_t fault;
    uint8_t motor_can_id;
};

inline KPID_Params_t MakePidParams(float k, float p, float i, float d,
                                   float i_limit, float out_limit,
                                   float d_cutoff_freq, float range) {
    KPID_Params_t pid{};
    pid.k = k;
    pid.p = p;
    pid.i = i;
    pid.d = d;
    pid.i_limit = i_limit;
    pid.out_limit = out_limit;
    pid.d_cutoff_freq = d_cutoff_freq;
    pid.range = range;
    return pid;
}

struct MotorPidConfig {
    bool enabled;
    float control_hz;
    float current_limit;
    KPID_Params_t pos_to_vel_pid;
    KPID_Params_t vel_to_cur_pid;

    static MotorPidConfig Disabled() {
        MotorPidConfig cfg{};
        cfg.enabled = false;
        cfg.control_hz = 1000.0f;
        cfg.current_limit = 10.0f;
        cfg.pos_to_vel_pid = MakePidParams(1.0f, 0.0f, 0.0f, 0.0f,
                                           0.0f, 0.0f, 50.0f, 0.0f);
        cfg.vel_to_cur_pid = MakePidParams(1.0f, 0.0f, 0.0f, 0.0f,
                                           0.0f, 0.0f, 50.0f, 0.0f);
        return cfg;
    }

    static MotorPidConfig DefaultRM() {
        MotorPidConfig cfg = Disabled();
        cfg.enabled = true;
        cfg.control_hz = 1000.0f;
        cfg.current_limit = 10.0f;
        cfg.pos_to_vel_pid = MakePidParams(1.0f, 5.0f, 0.0f, 0.1f,
                                           5.0f, 30.0f, 50.0f, 0.0f);
        cfg.vel_to_cur_pid = MakePidParams(1.0f, 0.05f, 0.0f, 0.0f,
                                           10.0f, 10.0f, 50.0f, 0.0f);
        return cfg;
    }
};

struct MotorConfig {
    const char* name;
    MotorType type;
    bool reverse;

    MOTOR_LZ_Param_t lz_param;
    MOTOR_DM_Param_t dm_param;
    MOTOR_RM_Param_t rm_param;

    float mit_default_kp;
    float mit_default_kd;
    MotorPidConfig pid;

    static MotorConfig FromLZ(const char* motor_name,
                              MOTOR_LZ_Param_t param,
                              float mit_kp = 10.0f,
                              float mit_kd = 0.5f) {
        MotorConfig cfg{};
        cfg.name = motor_name;
        cfg.type = MotorType::LZ;
        cfg.reverse = param.reverse;
        cfg.lz_param = param;
        cfg.mit_default_kp = mit_kp;
        cfg.mit_default_kd = mit_kd;
        cfg.pid = MotorPidConfig::Disabled();
        return cfg;
    }

    static MotorConfig FromDM(const char* motor_name,
                              MOTOR_DM_Param_t param,
                              float mit_kp = 50.0f,
                              float mit_kd = 3.0f) {
        MotorConfig cfg{};
        cfg.name = motor_name;
        cfg.type = MotorType::DM;
        cfg.reverse = param.reverse;
        cfg.dm_param = param;
        cfg.mit_default_kp = mit_kp;
        cfg.mit_default_kd = mit_kd;
        cfg.pid = MotorPidConfig::Disabled();
        return cfg;
    }

    static MotorConfig FromRM(const char* motor_name,
                              MOTOR_RM_Param_t param,
                              const MotorPidConfig* pid_cfg = nullptr,
                              float mit_kp = 5.0f,
                              float mit_kd = 0.5f) {
        MotorConfig cfg{};
        cfg.name = motor_name;
        cfg.type = MotorType::RM;
        cfg.reverse = param.reverse;
        cfg.rm_param = param;
        cfg.mit_default_kp = mit_kp;
        cfg.mit_default_kd = mit_kd;
        cfg.pid = (pid_cfg != nullptr) ? *pid_cfg : MotorPidConfig::DefaultRM();
        return cfg;
    }
};

class Motor {
public:
    explicit Motor(const MotorConfig& cfg);

    const char* GetName() const { return name_; }
    MotorType GetType() const { return type_; }
    bool IsReversed() const { return reverse_; }

    int8_t ConfigurePid(const MotorPidConfig& cfg);

    int8_t Register();

    int8_t Enable();

    int8_t Update();

    int8_t Relax();

    // New naming style (clear units/semantics)
    float GetRawAngleRad() const { return feedback_.rotor_abs_angle; }
    float GetPositionRad() const { return feedback_.rotor_abs_angle - zero_position_offset_rad_; }
    float GetVelocityRadPerSec() const { return feedback_.rotor_speed; }
    float GetTorqueCurrentAmp() const { return feedback_.torque_current; }
    float GetTemperatureCelsius() const { return feedback_.temp; }
    bool IsMotorOnline() const { return feedback_.online; }

    // Backward-compatible APIs
    float GetAngle() const { return GetPositionRad(); }
    float GetVelocity() const { return GetVelocityRadPerSec(); }
    float GetTorque() const { return GetTorqueCurrentAmp(); }
    float GetTemp() const { return GetTemperatureCelsius(); }
    bool IsOnline() const { return IsMotorOnline(); }

    const MotorFeedback& GetFeedback() const { return feedback_; }

    bool GetLZExtraFeedback(MotorLZExtraFeedback* out) const;

    // Set the current position as logical zero point (software offset).
    int8_t SetZeroPoint();
    void ClearZeroPoint() { zero_position_offset_rad_ = 0.0f; }
    float GetZeroPointOffsetRad() const { return zero_position_offset_rad_; }

    int8_t CurrentControl(float current);

    int8_t VelocityControl(float velocity);

    int8_t PositionControl(float angle, float max_velocity = 0.0f);

    int8_t MITControl(float angle, float velocity = 0.0f,
                      float kp = -1.0f, float kd = -1.0f, float torque = 0.0f);

private:
    struct MotorOps {
        int8_t (*register_fn)(Motor* self);
        int8_t (*enable_fn)(Motor* self);
        int8_t (*update_fn)(Motor* self);
        int8_t (*relax_fn)(Motor* self);
        int8_t (*current_fn)(Motor* self, float current);
        int8_t (*velocity_fn)(Motor* self, float velocity);
        int8_t (*position_fn)(Motor* self, float angle, float max_velocity);
        int8_t (*mit_fn)(Motor* self, float angle, float velocity, float kp, float kd, float torque);
        const MOTOR_t* (*raw_motor_fn)(const Motor* self);
    };

    static const MotorOps* ResolveOps(MotorType type);

    static int8_t LZRegister(Motor* self);
    static int8_t DMRegister(Motor* self);
    static int8_t RMRegister(Motor* self);

    static int8_t LZEnable(Motor* self);
    static int8_t DMEnable(Motor* self);
    static int8_t RMEnable(Motor* self);

    static int8_t LZUpdate(Motor* self);
    static int8_t DMUpdate(Motor* self);
    static int8_t RMUpdate(Motor* self);

    static int8_t LZRelax(Motor* self);
    static int8_t DMRelax(Motor* self);
    static int8_t RMRelax(Motor* self);

    static int8_t LZCurrent(Motor* self, float current);
    static int8_t DMCurrent(Motor* self, float current);
    static int8_t RMCurrent(Motor* self, float current);

    static int8_t LZVelocity(Motor* self, float velocity);
    static int8_t DMVelocity(Motor* self, float velocity);
    static int8_t RMVelocity(Motor* self, float velocity);

    static int8_t LZPosition(Motor* self, float angle, float max_velocity);
    static int8_t DMPosition(Motor* self, float angle, float max_velocity);
    static int8_t RMPosition(Motor* self, float angle, float max_velocity);

    static int8_t LZMIT(Motor* self, float angle, float velocity, float kp, float kd, float torque);
    static int8_t DMMIT(Motor* self, float angle, float velocity, float kp, float kd, float torque);
    static int8_t RMMIT(Motor* self, float angle, float velocity, float kp, float kd, float torque);

    static const MOTOR_t* LZRawMotor(const Motor* self);
    static const MOTOR_t* DMRawMotor(const Motor* self);
    static const MOTOR_t* RMRawMotor(const Motor* self);

    static const MotorOps kLZOps;
    static const MotorOps kDMOps;
    static const MotorOps kRMOps;

    const MOTOR_t* RawMotor() const {
        if (!ops_ || !ops_->raw_motor_fn) return nullptr;
        return ops_->raw_motor_fn(this);
    }

    int8_t RMCurrentControl(float current);

    void ClearFeedback();

    void RefreshFeedbackCache();

    const char* name_;
    MotorType type_;
    const MotorOps* ops_;
    bool reverse_;

    MOTOR_LZ_Param_t lz_param_;
    MOTOR_DM_Param_t dm_param_;
    MOTOR_RM_Param_t rm_param_;

    MOTOR_LZ_t* lz_instance_;
    MOTOR_DM_t* dm_instance_;
    MOTOR_RM_t* rm_instance_;

    MotorFeedback feedback_;
    MotorLZExtraFeedback lz_extra_;

    float mit_default_kp_;
    float mit_default_kd_;

    bool rm_pid_enabled_;
    float rm_control_period_s_;
    float rm_current_limit_a_;
    float zero_position_offset_rad_;

    KPID_t rm_pos_pid_;
    KPID_t rm_vel_pid_;
    KPID_Params_t rm_pos_pid_param_;
    KPID_Params_t rm_vel_pid_param_;
    bool rm_pos_pid_inited_;
    bool rm_vel_pid_inited_;
};

} // namespace mrobot
