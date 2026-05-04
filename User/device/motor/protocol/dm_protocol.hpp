#pragma once

// DM 电机协议层接口定义�?
// 负责�?DM C 驱动 raw 数据转换为统一状态，并封�?MIT/速度/位置/力矩控制接口�?

#include "device/motor/protocol/motor_protocol_fwd.hpp"
#include "device/motor/core/motor_install_spec.hpp"
#include "device/motor/core/motor_instance_config.hpp"
#include "device/motor/core/motor_state.hpp"
#include "device/motor/core/motor_traits.hpp"
#include "device/motor/protocol/pending_command_type.hpp"
#include "device/motor_dm.h"

namespace mr::motor {

template <MotorModel Model>
class MotorProtocol<MotorKind::DM, Model> {
public:
    MotorProtocol(const MotorInstanceConfig<MotorKind::DM>& config,
                  const MotorInstallSpec& install,
                  MotorState& state,
                  MOTOR_DM_Module_t module);

    int8_t Register();
    int8_t Enable();
    int8_t Disable();
    int8_t Relax();
    int8_t SetZero();
    int8_t Update();
    int8_t CommitCommand();
    bool HasPendingCommand() const { return pending_type_ != PendingCommandType::None; }
    void ClearPendingCommand();

    int8_t SetTorque(float torque_nm);
    int8_t SetVelocity(float velocity);
    int8_t SetPosition(float position, float max_velocity);
    int8_t SetMIT(float position, float velocity, float kp, float kd, float torque_ff);

private:
    float TotalRatio() const;
    float ToRotorPosition(float output_position_rad) const;
    float ToRotorVelocity(float output_velocity_rad_s) const;
    float ToOutputPosition(float rotor_position_rad) const;
    float ToOutputVelocity(float rotor_velocity_rad_s) const;
    float ToOutputTorque(float torque_current) const;
    void ResetPositionTracker();
    void SyncRotorPosition(float rotor_position_rad);
    float AccumulateRotorPosition(float rotor_position_rad, float rotor_velocity_rad_s);
    void RefreshStateCache();
    bool TryGetRotorFeedback(float& rotor_position_rad,
                             float& rotor_velocity_rad_s,
                             float& torque_current,
                             float& temperature_c) const;

    MotorInstanceConfig<MotorKind::DM> config_;
    MotorInstallSpec install_;
    MotorState& state_;
    PendingCommandType pending_type_ = PendingCommandType::None;
    MOTOR_MIT_Output_t pending_mit_output_{};
    float pending_velocity_ = 0.0f;
    float pending_position_ = 0.0f;
    float pending_position_velocity_limit_ = 0.0f;
    MOTOR_DM_Param_t param_{};
    MOTOR_DM_t* instance_ = nullptr;
    MOTOR_DM_t vendor_instance_{};
    bool rotor_position_initialized_ = false;
    float last_rotor_position_rad_ = 0.0f;
    float accumulated_rotor_position_rad_ = 0.0f;
    bool last_online_ = false;
};

} // namespace mr::motor
