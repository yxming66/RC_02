#include "device/motor/protocol/rm_protocol.hpp"

#include <math.h>

#include "bsp/time.h"
#include "component/math/scalar.hpp"
#include "component/user_math.h"
#include "device/device.h"
#include "device/motor.h"

namespace mr::motor {

namespace {

constexpr float kSuspiciousDeltaRad = 0.90f * mr::component::math::kPi;
constexpr float kVelocityCorrectionThresholdRad = mr::component::math::kPi;
constexpr float kVelocityMismatchWarnRad = 0.75f * mr::component::math::kPi;
constexpr float kMotorEncoderResolution = 8192.0f;
constexpr uint32_t kFeedbackLostUs = 100000u;

constexpr uint32_t kRmPositionFaultNone = 0u;
constexpr uint32_t kRmPositionFaultLargeDelta = (1u << 0);
constexpr uint32_t kRmPositionFaultFeedbackLost = (1u << 1);

float CorrectRmDeltaByVelocity(float wrapped_delta_rad, float rotor_velocity_rad_s, float dt_s) {
    if (dt_s <= 0.0f || !isfinite(rotor_velocity_rad_s)) {
        return wrapped_delta_rad;
    }
    const float expected_delta = rotor_velocity_rad_s * dt_s;
    if (fabsf(expected_delta) <= kVelocityCorrectionThresholdRad) {
        return wrapped_delta_rad;
    }
    const float turn_count = roundf((expected_delta - wrapped_delta_rad) /
                                    mr::component::math::kTwoPi);
    return wrapped_delta_rad + turn_count * mr::component::math::kTwoPi;
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

float TimeDeltaSeconds(uint32_t newer_us, uint32_t older_us) {
    if (older_us == 0u) {
        return 0.0f;
    }
    return static_cast<float>((uint32_t)(newer_us - older_us)) * 1.0e-6f;
}

} // namespace

template <MotorModel Model>
MotorProtocol<MotorKind::RM, Model>::MotorProtocol(const MotorInstanceConfig<MotorKind::RM>& config,
                                            const MotorInstallSpec& install,
                                            MotorState& state,
                                            MOTOR_RM_Module_t module)
    : config_(config),
      install_(install),
      state_(state),
      mapper_(TransmissionMapper::FromTotalRatio(
          ResolvePositiveRatio(MotorTraits<MotorKind::RM, Model>::kGearRatio) *
              ResolvePositiveRatio(install.external_ratio),
          MotorTraits<MotorKind::RM, Model>::kTorqueConstant)),
      param_(config.ToVendorParam(module)) {
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
    angle_valid_ = false;
    position_fault_ = kRmPositionFaultNone;
    last_feedback_tick_ = 0u;
}

template <MotorModel Model>
void MotorProtocol<MotorKind::RM, Model>::SyncRotorPosition(float single_turn_rotor_position_rad) {
    rotor_position_initialized_ = true;
    angle_valid_ = true;
    position_fault_ = kRmPositionFaultNone;
    last_single_turn_rotor_position_rad_ = single_turn_rotor_position_rad;
    accumulated_rotor_position_rad_ = single_turn_rotor_position_rad;
}

template <MotorModel Model>
void MotorProtocol<MotorKind::RM, Model>::SetRotorPositionZero(float single_turn_rotor_position_rad) {
    rotor_position_initialized_ = true;
    angle_valid_ = true;
    position_fault_ = kRmPositionFaultNone;
    last_single_turn_rotor_position_rad_ = single_turn_rotor_position_rad;
    accumulated_rotor_position_rad_ = single_turn_rotor_position_rad;
    rotor_zero_offset_rad_ = single_turn_rotor_position_rad;
}

template <MotorModel Model>
float MotorProtocol<MotorKind::RM, Model>::AccumulateRotorPosition(float single_turn_rotor_position_rad, float rotor_velocity_rad_s, float dt_s) {
    if (!rotor_position_initialized_) {
        SyncRotorPosition(single_turn_rotor_position_rad);
        return accumulated_rotor_position_rad_;
    }

    const float wrapped_delta = mr::component::math::wrap_error(
        single_turn_rotor_position_rad - last_single_turn_rotor_position_rad_,
        mr::component::math::kTwoPi);
    const float delta = CorrectRmDeltaByVelocity(wrapped_delta, rotor_velocity_rad_s, dt_s);
    const float abs_delta = fabsf(delta);
    if (abs_delta > max_abs_delta_rad_) {
        max_abs_delta_rad_ = abs_delta;
    }
    const float expected_delta = rotor_velocity_rad_s * dt_s;
    if (fabsf(expected_delta) > kVelocityCorrectionThresholdRad &&
        fabsf(delta - expected_delta) > kVelocityMismatchWarnRad) {
        ++large_delta_count_;
        MarkPositionInvalid(kRmPositionFaultLargeDelta);
    } else if (fabsf(wrapped_delta) >= kSuspiciousDeltaRad) {
        ++large_delta_count_;
        MarkPositionInvalid(kRmPositionFaultLargeDelta);
    }
    const float delta_limit = (fabsf(expected_delta) > 0.0f)
        ? fabsf(expected_delta) + mr::component::math::kPi
        : kDefaultRotorResyncDeltaRad;

    if (fabsf(delta) > delta_limit) {
        last_single_turn_rotor_position_rad_ = single_turn_rotor_position_rad;
        MarkPositionInvalid(kRmPositionFaultLargeDelta);
        return accumulated_rotor_position_rad_;
    }

    accumulated_rotor_position_rad_ += delta;
    last_single_turn_rotor_position_rad_ = single_turn_rotor_position_rad;
    return accumulated_rotor_position_rad_;
}

template <MotorModel Model>
void MotorProtocol<MotorKind::RM, Model>::MarkPositionInvalid(uint32_t fault_mask) {
    angle_valid_ = false;
    position_fault_ |= fault_mask;
}

template <MotorModel Model>
void MotorProtocol<MotorKind::RM, Model>::RefreshPositionDiagnostics(MotorState& next) {
    next.position_valid = angle_valid_;
    next.position_fault = position_fault_;
    next.position_large_delta_count = large_delta_count_;
    next.position_max_delta_rad = max_abs_delta_rad_;
    next.feedback_lost_count = feedback_lost_count_;
    next.last_feedback_tick = last_feedback_tick_;

    debug_.rotor_position_initialized = rotor_position_initialized_;
    debug_.angle_valid = angle_valid_;
    debug_.position_fault = position_fault_;
    debug_.large_delta_count = large_delta_count_;
    debug_.max_abs_delta_rad = max_abs_delta_rad_;
    debug_.feedback_lost_count = feedback_lost_count_;
    debug_.last_feedback_tick = last_feedback_tick_;
}

template <MotorModel Model>
float MotorProtocol<MotorKind::RM, Model>::ApplyRotorPositionOffset(float rotor_position_rad) const {
    return rotor_position_rad - rotor_zero_offset_rad_;
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

    rotor_position_rad = (static_cast<float>(raw->raw_angle) / kMotorEncoderResolution) *
                         mr::component::math::kTwoPi;
    while (rotor_position_rad < 0.0f) {
        rotor_position_rad += mr::component::math::kTwoPi;
    }
    while (rotor_position_rad >= mr::component::math::kTwoPi) {
        rotor_position_rad -= mr::component::math::kTwoPi;
    }
    if (param_.reverse) {
        rotor_position_rad = mr::component::math::kTwoPi - rotor_position_rad;
        if (rotor_position_rad >= mr::component::math::kTwoPi) {
            rotor_position_rad -= mr::component::math::kTwoPi;
        }
    }

    rotor_velocity_rad_s = MotorTraitsRpmToRadPerSec(static_cast<float>(raw->raw_speed));
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
        RefreshPositionDiagnostics(state_);
        return;
    }

    next.online = motor->header.online;
    if (!next.online) {
        ResetPositionTracker();
        last_online_ = false;
        next.temperature_c = MOTOR_GetTemp(motor);
        RefreshPositionDiagnostics(next);
        state_ = next;
        return;
    }

    const uint32_t now_us = static_cast<uint32_t>(BSP_TIME_Get_us());
    const float dt_s = TimeDeltaSeconds(now_us, last_feedback_tick_);
    if (last_feedback_tick_ != 0u &&
        (uint32_t)(now_us - last_feedback_tick_) > kFeedbackLostUs) {
        ++feedback_lost_count_;
        position_fault_ |= kRmPositionFaultFeedbackLost;
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
        RefreshPositionDiagnostics(next);
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

    next.position_rad = mapper_.ToOutputPosition(ApplyRotorPositionOffset(
        AccumulateRotorPosition(rotor_position_rad, rotor_velocity_rad_s, dt_s)));
    last_feedback_tick_ = now_us;
    next.position_single_turn_rad = mr::component::math::wrap_to_pi(next.position_rad);
    next.velocity_rad_s = mapper_.ToOutputVelocity(rotor_velocity_rad_s);
    next.torque_nm = mapper_.ToOutputTorque(torque_current);
    next.temperature_c = temperature_c;
    next.protocol_status_code = error_code;
    next.protocol_fault = EncodeRmC610Fault(error_code);
    if (next.protocol_fault != 0u) {
        next.protocol_state = MotorProtocolState::Fault;
    } else if (next.protocol_state == MotorProtocolState::Fault) {
        next.protocol_state = last_non_fault_protocol_state_;
    }
    debug_.last_error_code = error_code;
    RefreshPositionDiagnostics(next);

    if (install_.reverse_output) {
        next.position_rad = -next.position_rad;
        next.position_single_turn_rad = mr::component::math::wrap_to_pi(-next.position_single_turn_rad);
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
    MarkNoPendingCommand(state_);
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::RM, Model>::SetTorque(float torque_nm) {
    const float max_current = (MotorTraits<MotorKind::RM, Model>::kPeakCurrent > 0.0f) ? MotorTraits<MotorKind::RM, Model>::kPeakCurrent : 0.0f;
    if (max_current <= 0.0f) {
        debug_.last_set_torque_nm = torque_nm;
        debug_.last_set_torque_ret = DEVICE_ERR;
        return DEVICE_ERR;
    }
    pending_torque_current_ = AbsClip(mapper_.ToTorqueCurrent(torque_nm, install_.reverse_output), max_current);
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
        state_.position_valid = angle_valid_;
        state_.position_fault = position_fault_;
        state_.position_large_delta_count = large_delta_count_;
        state_.position_max_delta_rad = max_abs_delta_rad_;
        state_.feedback_lost_count = feedback_lost_count_;
        state_.last_feedback_tick = last_feedback_tick_;
        state_.velocity_rad_s = mapper_.ToOutputVelocity(rotor_velocity_rad_s);
        state_.torque_nm = mapper_.ToOutputTorque(torque_current);
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
