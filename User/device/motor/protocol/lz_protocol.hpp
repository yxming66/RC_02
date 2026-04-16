#pragma once

// LZ 电机协议层接口定义。
// 负责基于 LZ C 驱动 raw 数据完成状态解释、故障位映射与统一控制命令封装。

#include "device/motor/protocol/motor_protocol_fwd.hpp"
#include "device/motor/core/motor_install_spec.hpp"
#include "device/motor/core/motor_instance_config.hpp"
#include "device/motor/core/motor_state.hpp"
#include "device/motor/core/motor_traits.hpp"
#include "device/motor/protocol/pending_command_type.hpp"
#include "device/motor_lz.h"

namespace mrobot::motor {

template <MotorModel Model>
class MotorProtocol<MotorKind::LZ, Model> {
public:
    MotorProtocol(const MotorInstanceConfig<MotorKind::LZ>& config,
                  const MotorInstallSpec& install,
                  MotorState& state,
                  MOTOR_LZ_Module_t module);

    int8_t Register();
    int8_t Enable();
    int8_t Disable();
    int8_t Relax();
    int8_t Update();
    int8_t CommitCommand();
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
    void RefreshStateCache();
    bool TryGetRotorFeedback(float& rotor_position_rad,
                             float& rotor_velocity_rad_s,
                             float& torque_current,
                             float& temperature_c) const;

    MotorInstanceConfig<MotorKind::LZ> config_;
    MotorInstallSpec install_;
    MotorState& state_;
    PendingCommandType pending_type_ = PendingCommandType::None;
    MOTOR_LZ_MotionParam_t pending_motion_output_{};
    float pending_velocity_ = 0.0f;
    float pending_position_ = 0.0f;
    float pending_position_velocity_limit_ = 0.0f;
    MOTOR_LZ_Param_t param_{};
    MOTOR_LZ_t* instance_ = nullptr;
    MOTOR_LZ_t vendor_instance_{};
};

} // namespace mrobot::motor