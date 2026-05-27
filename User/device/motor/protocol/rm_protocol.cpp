#include "device/motor/protocol/rm_protocol.hpp"

#include <math.h>

#include "component/math/scalar.hpp"
#include "component/user_math.h"
#include "device/device.h"
#include "device/motor.h"

namespace mr::motor {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kDefaultResyncDeltaRad = 1.75f * kPi;
constexpr float kMotorEncoderResolution = 8192.0f;
constexpr float kSecondsPerMinute = 60.0f;

float ResolvePositiveRatio(float ratio) {
    return mr::component::math::positive_or(ratio, 1.0f);
}

float WrapAngleDiff(float diff_rad) {
    return mr::component::math::wrap_error(diff_rad, kTwoPi);
}

float WrapToPi(float angle_rad) {
    return mr::component::math::wrap_to_pi(angle_rad);
}

float RpmToRadPerSec(float rpm) {
    return rpm * kTwoPi / kSecondsPerMinute;
}

uint32_t EncodeRmC610Fault(uint8_t error_code) {
    if (error_code == MOTOR_RM_C610_ERROR_NONE) {
        return 0u;
    }
    if (error_code < 32u) {
        return (1u << error_code);
    }
    return (1u << 31);
}

} // namespace

template <MotorModel Model>
MotorProtocol<MotorKind::RM, Model>::MotorProtocol(const MotorInstanceConfig<MotorKind::RM>& config,
                                            const MotorInstallSpec& install,
                                            MotorState& state,
                                            MOTOR_RM_Module_t module)
    : config_(config), install_(install), state_(state), param_(config.ToVendorParam(module)) {
}

template <MotorModel Model>
float MotorProtocol<MotorKind::RM, Model>::TotalRatio() const {
    return ResolvePositiveRatio(MotorTraits<MotorKind::RM, Model>::kGearRatio) * ResolvePositiveRatio(install_.external_ratio);
}

template <MotorModel Model>
float MotorProtocol<MotorKind::RM, Model>::ToTorqueCurrent(float output_torque_nm) const {
    const float signed_torque = install_.reverse_output ? -output_torque_nm : output_torque_nm;
    const float torque_constant = (MotorTraits<MotorKind::RM, Model>::kTorqueConstant > 0.0f) ? MotorTraits<MotorKind::RM, Model>::kTorqueConstant : 0.0f;
    const float total_ratio = TotalRatio();
    if (torque_constant <= 0.0f || total_ratio <= 0.0f) {
        return 0.0f;
    }
    return signed_torque / (torque_constant * total_ratio);
}

template <MotorModel Model>
void MotorProtocol<MotorKind::RM, Model>::ResetStateCache() {
    state_ = {};
}

template <MotorModel Model>
void MotorProtocol<MotorKind::RM, Model>::ResetPositionTracker() {
    rotor_position_initialized_ = false;
    last_single_turn_rotor_position_rad_ = 0.0f;
    accumulated_rotor_position_rad_ = 0.0f;
    rotor_zero_offset_rad_ = 0.0f;
}

template <MotorModel Model>
void MotorProtocol<MotorKind::RM, Model>::SyncRotorPosition(float single_turn_rotor_position_rad) {
    rotor_position_initialized_ = true;
    last_single_turn_rotor_position_rad_ = single_turn_rotor_position_rad;
    accumulated_rotor_position_rad_ = single_turn_rotor_position_rad;
}

template <MotorModel Model>
void MotorProtocol<MotorKind::RM, Model>::SetRotorPositionZero(float single_turn_rotor_position_rad) {
    rotor_position_initialized_ = true;
    last_single_turn_rotor_position_rad_ = single_turn_rotor_position_rad;
    accumulated_rotor_position_rad_ = single_turn_rotor_position_rad;
    rotor_zero_offset_rad_ = single_turn_rotor_position_rad;
}

template <MotorModel Model>
float MotorProtocol<MotorKind::RM, Model>::AccumulateRotorPosition(float single_turn_rotor_position_rad, float rotor_velocity_rad_s) {
    if (!rotor_position_initialized_) {
        SyncRotorPosition(single_turn_rotor_position_rad);
        return accumulated_rotor_position_rad_;
    }

    const float delta = WrapAngleDiff(single_turn_rotor_position_rad - last_single_turn_rotor_position_rad_);
    const float delta_limit = (fabsf(rotor_velocity_rad_s) > 0.0f)
        ? fabsf(rotor_velocity_rad_s) + kPi
        : kDefaultResyncDeltaRad;

    if (fabsf(delta) > delta_limit) {
        last_single_turn_rotor_position_rad_ = single_turn_rotor_position_rad;
        return accumulated_rotor_position_rad_;
    }

    accumulated_rotor_position_rad_ += delta;
    last_single_turn_rotor_position_rad_ = single_turn_rotor_position_rad;
    return accumulated_rotor_position_rad_;
}

template <MotorModel Model>
float MotorProtocol<MotorKind::RM, Model>::ApplyRotorPositionOffset(float rotor_position_rad) const {
    return rotor_position_rad - rotor_zero_offset_rad_;
}

template <MotorModel Model>
float MotorProtocol<MotorKind::RM, Model>::ToOutputPosition(float rotor_position_rad) const {
    return rotor_position_rad / TotalRatio();
}

template <MotorModel Model>
float MotorProtocol<MotorKind::RM, Model>::ToOutputVelocity(float rotor_velocity_rad_s) const {
    return rotor_velocity_rad_s / TotalRatio();
}

template <MotorModel Model>
float MotorProtocol<MotorKind::RM, Model>::ToOutputTorque(float torque_current) const {
    return torque_current * MotorTraits<MotorKind::RM, Model>::kTorqueConstant * TotalRatio();
}

template <MotorModel Model>
static float RawCurrentToAmp(int16_t raw_current) {
    constexpr float kRawCurrentRange = MotorTraits<MotorKind::RM, Model>::kRawCurrentRange;
    constexpr float kCurrentRangeAmp = MotorTraits<MotorKind::RM, Model>::kCurrentRangeAmp;
    if (kRawCurrentRange <= 0.0f || kCurrentRangeAmp <= 0.0f) {
        return 0.0f;
    }
    return static_cast<float>(raw_current) * kCurrentRangeAmp / kRawCurrentRange;
}

template <MotorModel Model>
bool MotorProtocol<MotorKind::RM, Model>::TryGetRotorFeedback(float& rotor_position_rad,
                                                       float& rotor_velocity_rad_s,
                                                       float& torque_current,
                                                       float& temperature_c,
                                                       uint8_t& error_code) const {
    const MOTOR_RM_RawFeedback_t* raw = MOTOR_RM_GetRawFeedback(const_cast<MOTOR_RM_Param_t*>(&param_));
    if (instance_ == nullptr || raw == nullptr) {
        return false;
    }

    rotor_position_rad = (static_cast<float>(raw->raw_angle) / kMotorEncoderResolution) * kTwoPi;
    while (rotor_position_rad < 0.0f) {
        rotor_position_rad += kTwoPi;
    }
    while (rotor_position_rad >= kTwoPi) {
        rotor_position_rad -= kTwoPi;
    }
    if (param_.reverse) {
        rotor_position_rad = kTwoPi - rotor_position_rad;
        if (rotor_position_rad >= kTwoPi) {
            rotor_position_rad -= kTwoPi;
        }
    }

    rotor_velocity_rad_s = RpmToRadPerSec(static_cast<float>(raw->raw_speed));
    if (param_.reverse) {
        rotor_velocity_rad_s = -rotor_velocity_rad_s;
    }

    torque_current = RawCurrentToAmp<Model>(raw->raw_current);
    if (param_.reverse) {
        torque_current = -torque_current;
    }

    temperature_c = static_cast<float>(raw->raw_temp);
    error_code = raw->raw_error_code;
    return true;
}

template <MotorModel Model>
void MotorProtocol<MotorKind::RM, Model>::RefreshStateCache() {
    MotorState next = state_;
    const MOTOR_t* motor = instance_ ? &instance_->motor : nullptr;
    if (motor == nullptr) {
        ResetPositionTracker();
        last_online_ = false;
        ResetStateCache();
        return;
    }

    next.online = motor->header.online;
    if (!next.online) {
        ResetPositionTracker();
        last_online_ = false;
        next.temperature_c = MOTOR_GetTemp(motor);
        state_ = next;
        return;
    }

    float rotor_position_rad = 0.0f;
    float rotor_velocity_rad_s = 0.0f;
    float torque_current = 0.0f;
    float temperature_c = 0.0f;
    uint8_t error_code = 0u;
    if (!TryGetRotorFeedback(rotor_position_rad, rotor_velocity_rad_s, torque_current, temperature_c, error_code)) {
        ResetPositionTracker();
        last_online_ = next.online;
        next.temperature_c = MOTOR_GetTemp(motor);
        state_ = next;
        return;
    }

    if (!last_online_) {
        if constexpr (Model == MotorModel::M2006 || Model == MotorModel::M3508) {
            SetRotorPositionZero(rotor_position_rad);
        } else {
            SyncRotorPosition(rotor_position_rad);
        }
    }

    next.position_rad = ToOutputPosition(ApplyRotorPositionOffset(
        AccumulateRotorPosition(rotor_position_rad, rotor_velocity_rad_s)));
    next.position_single_turn_rad = WrapToPi(next.position_rad);
    next.velocity_rad_s = ToOutputVelocity(rotor_velocity_rad_s);
    next.torque_nm = ToOutputTorque(torque_current);
    next.temperature_c = temperature_c;
    next.protocol_status_code = error_code;
    next.protocol_fault = EncodeRmC610Fault(error_code);
    if (next.protocol_fault != 0u) {
        next.protocol_state = MotorProtocolState::Fault;
    } else if (next.protocol_state == MotorProtocolState::Fault) {
        next.protocol_state = last_non_fault_protocol_state_;
    }
    debug_.last_error_code = error_code;

    if (install_.reverse_output) {
        next.position_rad = -next.position_rad;
        next.position_single_turn_rad = WrapToPi(-next.position_single_turn_rad);
        next.velocity_rad_s = -next.velocity_rad_s;
        next.torque_nm = -next.torque_nm;
    }
    last_online_ = next.online;
    state_ = next;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::RM, Model>::Register() {
    if (instance_ != nullptr) {
        last_non_fault_protocol_state_ = MotorProtocolState::Registered;
        state_.protocol_state = last_non_fault_protocol_state_;
        return DEVICE_OK;
    }
    const int8_t ret = MOTOR_RM_AttachExternal(&param_, &vendor_instance_);
    if (ret == DEVICE_OK) {
        instance_ = &vendor_instance_;
        last_non_fault_protocol_state_ = MotorProtocolState::Registered;
        state_.protocol_state = last_non_fault_protocol_state_;
    }
    return ret;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::RM, Model>::Enable() {
    last_non_fault_protocol_state_ = MotorProtocolState::Enabled;
    state_.protocol_state = last_non_fault_protocol_state_;
    return DEVICE_OK;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::RM, Model>::Disable() {
    ClearPendingCommand();
    const int8_t ret = MOTOR_RM_Relax(&param_);
    if (ret != DEVICE_OK) {
        return ret;
    }
    last_non_fault_protocol_state_ = MotorProtocolState::Disabled;
    state_.protocol_state = last_non_fault_protocol_state_;
    return DEVICE_OK;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::RM, Model>::Relax() {
    ClearPendingCommand();
    const int8_t ret = MOTOR_RM_Relax(&param_);
    if (ret != DEVICE_OK) {
        return ret;
    }
    last_non_fault_protocol_state_ = MotorProtocolState::Enabled;
    state_.protocol_state = last_non_fault_protocol_state_;
    return DEVICE_OK;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::RM, Model>::Update() {
    const int8_t ret = MOTOR_RM_Update(&param_);
    if (ret == DEVICE_OK) {
        RefreshStateCache();
    }
    return ret;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::RM, Model>::CommitCommand() {
    if (!pending_valid_) {
        debug_.pending_valid = pending_valid_;
        debug_.pending_torque_current = pending_torque_current_;
        debug_.last_commit_ret = DEVICE_OK;
        debug_.last_commit_skipped = true;
        state_.last_commit_ok = true;
        return DEVICE_OK;
    }

    int8_t ret = MOTOR_RM_SetTorqueCurrent(&param_, pending_torque_current_);
    if (ret != DEVICE_OK) {
        debug_.pending_valid = pending_valid_;
        debug_.pending_torque_current = pending_torque_current_;
        debug_.last_commit_ret = ret;
        debug_.last_commit_skipped = false;
        state_.last_commit_ok = false;
        return ret;
    }
    debug_.pending_valid = pending_valid_;
    debug_.pending_torque_current = pending_torque_current_;
    debug_.last_commit_ret = DEVICE_OK;
    debug_.last_commit_skipped = false;
    state_.last_commit_ok = true;
    ClearPendingCommand();
    return DEVICE_OK;
}

template <MotorModel Model>
void MotorProtocol<MotorKind::RM, Model>::ClearPendingCommand() {
    pending_valid_ = false;
    pending_torque_current_ = 0.0f;
    debug_.pending_valid = pending_valid_;
    debug_.pending_torque_current = pending_torque_current_;
    state_.command_pending = false;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::RM, Model>::SetTorque(float torque_nm) {
    const float max_current = (MotorTraits<MotorKind::RM, Model>::kPeakCurrent > 0.0f) ? MotorTraits<MotorKind::RM, Model>::kPeakCurrent : 0.0f;
    if (max_current <= 0.0f) {
        debug_.last_set_torque_nm = torque_nm;
        debug_.last_set_torque_ret = DEVICE_ERR;
        return DEVICE_ERR;
    }
    pending_torque_current_ = AbsClip(ToTorqueCurrent(torque_nm), max_current);
    pending_valid_ = true;
    debug_.pending_valid = pending_valid_;
    debug_.pending_torque_current = pending_torque_current_;
    debug_.last_set_torque_nm = torque_nm;
    debug_.last_set_torque_ret = DEVICE_OK;
    state_.command_pending = true;
    return DEVICE_OK;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::RM, Model>::SetZero() {
    if constexpr (!(Model == MotorModel::M2006 || Model == MotorModel::M3508)) {
        return DEVICE_ERR_UNSUPPORTED;
    } else {
        float rotor_position_rad = 0.0f;
        float rotor_velocity_rad_s = 0.0f;
        float torque_current = 0.0f;
        float temperature_c = 0.0f;
        uint8_t error_code = 0u;
        if (!TryGetRotorFeedback(rotor_position_rad, rotor_velocity_rad_s, torque_current, temperature_c, error_code)) {
            return DEVICE_ERR;
        }

        SetRotorPositionZero(rotor_position_rad);
        state_.position_rad = 0.0f;
        state_.position_single_turn_rad = 0.0f;
        state_.velocity_rad_s = ToOutputVelocity(rotor_velocity_rad_s);
        state_.torque_nm = ToOutputTorque(torque_current);
        state_.temperature_c = temperature_c;
        state_.protocol_status_code = error_code;
        state_.protocol_fault = EncodeRmC610Fault(error_code);
        return DEVICE_OK;
    }
}

template <MotorModel Model>
const RmProtocolDebugSnapshot& MotorProtocol<MotorKind::RM, Model>::GetDebugSnapshot() const {
    return debug_;
}

template class MotorProtocol<MotorKind::RM, MotorModel::M2006>;
template class MotorProtocol<MotorKind::RM, MotorModel::M3508>;
template class MotorProtocol<MotorKind::RM, MotorModel::M6020>;

} // namespace mr::motor
