#pragma once

#include "device/motor.h"
#include "device/motor_dm.h"
#include "device/motor_lz.h"
#include "device/motor_rm.h"
#include "device/motor/core/imotor.hpp"

namespace mrobot::motor {

class MotorDriverBase : public IMotor {
public:
    MotorDriverBase(const MotorSpec& spec, const MotorInstallSpec& install)
        : spec_(spec), install_(install) {}

    MotorState GetState() const override;
    const MotorSpec& GetSpec() const override { return spec_; }
    const MotorInstallSpec& GetInstallSpec() const override { return install_; }

    virtual const MOTOR_t* RawMotor() const = 0;
    virtual bool TryGetRotorFeedback(float& rotor_position_rad,
                                     float& rotor_velocity_rad_s,
                                     float& torque_current,
                                     float& temperature_c) const = 0;

protected:
    // 读取“电机转子侧/本体侧”单圈反馈；成功时由 V2 统一累计多圈并换算到最终输出侧。
    void RefreshStateCache();
    void SetPendingStatus(bool pending) const;
    void SetLastCommitStatus(bool ok) const;
    float TotalRatio() const;
    void ResetPositionTracker() const;
    void SyncRotorPosition(float single_turn_rotor_position_rad) const;
    float AccumulateRotorPosition(float single_turn_rotor_position_rad,
                                  float rotor_velocity_rad_s,
                                  bool* resynced = nullptr) const;
    float ToOutputPosition(float rotor_position_rad) const;
    float ToOutputVelocity(float rotor_velocity_rad_s) const;
    float ToOutputTorque(float torque_current) const;
    float ToRotorPosition(float output_position_rad) const;
    float ToRotorVelocity(float output_velocity_rad_s) const;
    float ToTorqueCurrent(float output_torque_nm) const;

    MotorSpec spec_;
    MotorInstallSpec install_;

private:
    void ResetStateCache() const;

    mutable MotorState cached_state_{};
    mutable bool rotor_position_initialized_ = false;
    mutable bool last_online_ = false;
    mutable float last_single_turn_rotor_position_rad_ = 0.0f;
    mutable float accumulated_rotor_position_rad_ = 0.0f;
};

} // namespace mrobot::motor
