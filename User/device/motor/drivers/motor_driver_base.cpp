#include "device/motor/drivers/motor_driver_base.hpp"

#include <math.h>

namespace mrobot::motor {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kDefaultResyncDeltaRad = 1.75f * kPi;

float ResolvePositiveRatio(float ratio) {
    return (ratio > 0.0f) ? ratio : 1.0f;
}

float WrapAngleDiff(float diff_rad) {
    while (diff_rad > kPi) {
        diff_rad -= kTwoPi;
    }
    while (diff_rad < -kPi) {
        diff_rad += kTwoPi;
    }
    return diff_rad;
}

} // namespace

float MotorDriverBase::TotalRatio() const {
    return ResolvePositiveRatio(spec_.reducer_ratio) * ResolvePositiveRatio(install_.external_ratio);
}

MotorState MotorDriverBase::GetState() const {
    return cached_state_;
}

void MotorDriverBase::ResetStateCache() const {
    cached_state_ = {};
}

void MotorDriverBase::SetPendingStatus(bool pending) const {
    cached_state_.command_pending = pending;
}

void MotorDriverBase::SetLastCommitStatus(bool ok) const {
    cached_state_.last_commit_ok = ok;
}

void MotorDriverBase::RefreshStateCache() {
    MotorState state{};
    state.command_pending = cached_state_.command_pending;
    state.last_commit_ok = cached_state_.last_commit_ok;
    const MOTOR_t* motor = RawMotor();
    if (motor == nullptr) {
        ResetPositionTracker();
        last_online_ = false;
        ResetStateCache();
        return;
    }

    state.online = motor->header.online;
    if (!state.online) {
        ResetPositionTracker();
        last_online_ = false;
        state.temperature_c = MOTOR_GetTemp(motor);
        cached_state_ = state;
        return;
    }

    float rotor_position_rad = 0.0f;
    float rotor_velocity_rad_s = 0.0f;
    float torque_current = 0.0f;
    float temperature_c = 0.0f;

    if (!TryGetRotorFeedback(rotor_position_rad, rotor_velocity_rad_s, torque_current, temperature_c)) {
        ResetPositionTracker();
        last_online_ = state.online;
        state.temperature_c = MOTOR_GetTemp(motor);
        cached_state_ = state;
        return;
    }

    if (!last_online_) {
        SyncRotorPosition(rotor_position_rad);
    }

    state.position_rad = ToOutputPosition(AccumulateRotorPosition(rotor_position_rad, rotor_velocity_rad_s));
    state.velocity_rad_s = ToOutputVelocity(rotor_velocity_rad_s);
    state.torque_nm = ToOutputTorque(torque_current);
    state.temperature_c = temperature_c;

    if (install_.reverse_output) {
        state.position_rad = -state.position_rad;
        state.velocity_rad_s = -state.velocity_rad_s;
        state.torque_nm = -state.torque_nm;
    }
    last_online_ = state.online;
    cached_state_ = state;
}

void MotorDriverBase::ResetPositionTracker() const {
    rotor_position_initialized_ = false;
    last_single_turn_rotor_position_rad_ = 0.0f;
    accumulated_rotor_position_rad_ = 0.0f;
}

void MotorDriverBase::SyncRotorPosition(float single_turn_rotor_position_rad) const {
    rotor_position_initialized_ = true;
    last_single_turn_rotor_position_rad_ = single_turn_rotor_position_rad;
    accumulated_rotor_position_rad_ = single_turn_rotor_position_rad;
}

float MotorDriverBase::AccumulateRotorPosition(float single_turn_rotor_position_rad,
                                                float rotor_velocity_rad_s,
                                                bool* resynced) const {
    if (resynced != nullptr) {
        *resynced = false;
    }

    if (!rotor_position_initialized_) {
        SyncRotorPosition(single_turn_rotor_position_rad);
        return accumulated_rotor_position_rad_;
    }

    const float delta = WrapAngleDiff(single_turn_rotor_position_rad - last_single_turn_rotor_position_rad_);
    const float delta_limit = (fabsf(rotor_velocity_rad_s) > 0.0f)
        ? fabsf(rotor_velocity_rad_s) + kPi
        : kDefaultResyncDeltaRad;

    if (fabsf(delta) > delta_limit) {
        SyncRotorPosition(single_turn_rotor_position_rad);
        if (resynced != nullptr) {
            *resynced = true;
        }
        return accumulated_rotor_position_rad_;
    }

    accumulated_rotor_position_rad_ += delta;
    last_single_turn_rotor_position_rad_ = single_turn_rotor_position_rad;
    return accumulated_rotor_position_rad_;
}

float MotorDriverBase::ToOutputPosition(float rotor_position_rad) const {
    return rotor_position_rad / TotalRatio();
}

float MotorDriverBase::ToOutputVelocity(float rotor_velocity_rad_s) const {
    return rotor_velocity_rad_s / TotalRatio();
}

float MotorDriverBase::ToOutputTorque(float torque_current) const {
    const float ratio = TotalRatio();
    const float torque_constant = (spec_.torque_constant > 0.0f) ? spec_.torque_constant : 0.0f;
    return torque_current * torque_constant * ratio;
}

float MotorDriverBase::ToRotorPosition(float output_position_rad) const {
    const float signed_position = install_.reverse_output ? -output_position_rad : output_position_rad;
    return signed_position * TotalRatio();
}

float MotorDriverBase::ToRotorVelocity(float output_velocity_rad_s) const {
    const float signed_velocity = install_.reverse_output ? -output_velocity_rad_s : output_velocity_rad_s;
    return signed_velocity * TotalRatio();
}

float MotorDriverBase::ToTorqueCurrent(float output_torque_nm) const {
    const float signed_torque = install_.reverse_output ? -output_torque_nm : output_torque_nm;
    const float ratio = TotalRatio();
    const float torque_constant = (spec_.torque_constant > 0.0f) ? spec_.torque_constant : 0.0f;
    if (torque_constant <= 0.0f || ratio <= 0.0f) {
        return 0.0f;
    }
    return signed_torque / (torque_constant * ratio);
}

} // namespace mrobot::motor
