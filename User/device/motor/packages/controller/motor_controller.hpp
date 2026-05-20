#pragma once

// Motor controller wrapper.
// Native motor capabilities are used first; unsupported velocity/position
// modes fall back to software PID loops.

#include <stdint.h>

#include "component/controller/pid.hpp"
#include "component/pid.h"
#include "component/timer.hpp"
#include "device/motor/core/motor_install_spec.hpp"
#include "device/motor/motor.hpp"

namespace mr::motor {

struct MotorControllerConfig {
    const KPID_Params_t* velocity_pid;
    const KPID_Params_t* position_pid;
    float sample_freq;

    float position_to_velocity_limit;
    float velocity_to_torque_limit;
    float feedback_lowpass_cutoff_hz;
    float output_lowpass_cutoff_hz;
};

template <typename MotorType>
class MotorControllerT final {
public:
    MotorControllerT(MotorType& motor, const MotorControllerConfig& config);

    int8_t Register();
    int8_t Enable();
    int8_t Disable();
    int8_t Relax();
    int8_t Update();
    int8_t CommitCommand();
    bool HasPendingCommand() const;
    void ClearPendingCommand();

    int8_t SetTorque(float torque_nm);
    int8_t SetVelocity(float velocity);
    int8_t SetPosition(float position, float max_velocity = 0.0f);
    int8_t SetPositionTorque(float position, float torque_limit = 0.0f);
    int8_t SetMIT(float position, float velocity, float kp, float kd, float torque_ff);

    MotorState GetState() const;
    const MotorInstallSpec& GetInstallConfig() const;
    const MotorType& GetMotor() const;
    float GetLastFeedbackPosition() const;
    float GetLastFeedbackVelocity() const;
    float GetLastOutputTorque() const;
    float CalculatePositionOutput(float target, float feedback, float dt_s);
    float CalculateVelocityOutput(float target, float feedback, float dt_s);

private:
    enum class ControlMode : uint8_t {
        Torque = 0,
        NativeVelocity,
        EmulatedVelocity,
        NativePosition,
        EmulatedPosition,
        EmulatedPositionTorque,
        NativeMit,
    };

    int8_t ResetControllers();
    float UpdateControlDt();
    float ResolveVelocityLimit(float request_limit) const;
    float ResolveTorqueLimit(float request_limit) const;
    float LowpassAlpha(float cutoff_hz, float dt_s) const;
    void ResetFiltersToState();
    MotorState FilterFeedback(const MotorState& state, float dt_s);
    float FilterOutputTorque(float torque_nm, float dt_s);

    MotorType& motor_;
    MotorControllerConfig config_;

    mr::comp::cntlr::pid velocity_pid_;
    mr::comp::cntlr::pid position_pid_;
    mr::comp::timer control_timer_;
    float control_dt_fallback_s_;
    bool velocity_pid_ready_;
    bool position_pid_ready_;
    float feedback_lowpass_cutoff_hz_;
    float output_lowpass_cutoff_hz_;
    bool feedback_filter_initialized_;
    bool output_filter_initialized_;
    float filtered_position_;
    float filtered_velocity_;
    float filtered_output_torque_;

    ControlMode mode_;
    float target_torque_;
    float target_velocity_;
    float target_position_;
    float position_velocity_limit_;
    float velocity_torque_limit_;
    float target_mit_kp_;
    float target_mit_kd_;
    float target_mit_torque_ff_;
    float last_feedback_position_;
    float last_feedback_velocity_;
    float last_output_torque_;
};

using MotorController = MotorControllerT<mr::motor::RmM3508Motor>;

} // namespace mr::motor
