#include "device/motor/protocol/lz_protocol.hpp"

#include "component/math/scalar.hpp"
#include "component/user_math.h"
#include "device/device.h"

namespace mr::motor {

namespace {

float ResolvePositiveRatio(float ratio) {
    return mr::component::math::positive_or(ratio, 1.0f);
}

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kLzAngleRangeRad = 12.57f;
constexpr float kLzVelocityRangeRadS = 20.0f;
constexpr float kLzRawValueMax = 65535.0f;
constexpr float kLzTempScale = 10.0f;

float RawToFloat(uint16_t raw_value, float max_value) {
    return (static_cast<float>(raw_value) / kLzRawValueMax) * (2.0f * max_value) - max_value;
}

float WrapToPi(float angle_rad) {
    return mr::component::math::wrap_to_pi(angle_rad);
}

MotorProtocolState DecodeLzProtocolState(uint8_t state_bits, uint32_t fault_code) {
    if (fault_code != 0u) {
        return MotorProtocolState::Fault;
    }
    switch (state_bits) {
        case MOTOR_LZ_STATE_MOTOR:
            return MotorProtocolState::Enabled;
        case MOTOR_LZ_STATE_RESET:
            return MotorProtocolState::Disabled;
        case MOTOR_LZ_STATE_CALI:
            return MotorProtocolState::Enabled;
        default:
            return MotorProtocolState::Registered;
    }
}

uint32_t EncodeFaultBits(uint8_t fault_bits) {
    return (fault_bits & 0x20 ? (1u << 21) : 0u)
         | (fault_bits & 0x10 ? (1u << 20) : 0u)
         | (fault_bits & 0x08 ? (1u << 19) : 0u)
         | (fault_bits & 0x04 ? (1u << 18) : 0u)
         | (fault_bits & 0x02 ? (1u << 17) : 0u)
         | (fault_bits & 0x01 ? (1u << 16) : 0u);
}

} // namespace

template <MotorModel Model>
MotorProtocol<MotorKind::LZ, Model>::MotorProtocol(const MotorInstanceConfig<MotorKind::LZ>& config,
                                            const MotorInstallSpec& install,
                                            MotorState& state,
                                            MOTOR_LZ_Module_t module)
    : config_(config), install_(install), state_(state), param_(config.ToVendorParam(module)) {
}

template <MotorModel Model>
float MotorProtocol<MotorKind::LZ, Model>::TotalRatio() const {
    return ResolvePositiveRatio(install_.external_ratio);
}

template <MotorModel Model>
float MotorProtocol<MotorKind::LZ, Model>::ToRotorPosition(float output_position_rad) const {
    const float signed_position = install_.reverse_output ? -output_position_rad : output_position_rad;
    return signed_position * TotalRatio();
}

template <MotorModel Model>
float MotorProtocol<MotorKind::LZ, Model>::ToRotorVelocity(float output_velocity_rad_s) const {
    const float signed_velocity = install_.reverse_output ? -output_velocity_rad_s : output_velocity_rad_s;
    return signed_velocity * TotalRatio();
}

template <MotorModel Model>
float MotorProtocol<MotorKind::LZ, Model>::ToOutputPosition(float rotor_position_rad) const {
    return rotor_position_rad / TotalRatio();
}

template <MotorModel Model>
float MotorProtocol<MotorKind::LZ, Model>::ToOutputVelocity(float rotor_velocity_rad_s) const {
    return rotor_velocity_rad_s / TotalRatio();
}

template <MotorModel Model>
float MotorProtocol<MotorKind::LZ, Model>::ToOutputTorque(float torque_current) const {
    float torque = torque_current * MotorTraits<MotorKind::LZ, Model>::kTorqueConstant * TotalRatio();
    return install_.reverse_output ? -torque : torque;
}

template <MotorModel Model>
bool MotorProtocol<MotorKind::LZ, Model>::TryGetRotorFeedback(float& rotor_position_rad,
                                                       float& rotor_velocity_rad_s,
                                                       float& torque_current,
                                                       float& temperature_c) const {
    const MOTOR_LZ_RawFeedback_t* raw = MOTOR_LZ_GetRawFeedback(const_cast<MOTOR_LZ_Param_t*>(&param_));
    if (raw == nullptr) {
        return false;
    }

    rotor_position_rad = RawToFloat(raw->raw_angle, kLzAngleRangeRad);
    if (param_.reverse) {
        rotor_position_rad = -rotor_position_rad;
    }

    rotor_velocity_rad_s = RawToFloat(static_cast<uint16_t>(raw->raw_speed), kLzVelocityRangeRadS);
    if (param_.reverse) {
        rotor_velocity_rad_s = -rotor_velocity_rad_s;
    }

    torque_current = RawToFloat(static_cast<uint16_t>(raw->raw_current),
                                MotorTraits<MotorKind::LZ, Model>::kPeakTorque);
    if (param_.reverse) {
        torque_current = -torque_current;
    }

    temperature_c = static_cast<float>(raw->raw_temp) / kLzTempScale;
    return true;
}

template <MotorModel Model>
void MotorProtocol<MotorKind::LZ, Model>::RefreshStateCache() {
    if (instance_ == nullptr) {
        state_ = {};
        return;
    }

    float rotor_position_rad = 0.0f;
    float rotor_velocity_rad_s = 0.0f;
    float torque_current = 0.0f;
    float temperature_c = 0.0f;
    if (!TryGetRotorFeedback(rotor_position_rad, rotor_velocity_rad_s, torque_current, temperature_c)) {
        state_.online = false;
        return;
    }

    state_.position_rad = ToOutputPosition(rotor_position_rad);
    state_.position_single_turn_rad = WrapToPi(state_.position_rad);
    state_.velocity_rad_s = ToOutputVelocity(rotor_velocity_rad_s);
    state_.torque_nm = ToOutputTorque(torque_current);
    state_.temperature_c = temperature_c;
    state_.online = instance_->motor.header.online;
    state_.protocol_status_code = instance_->lz_feedback.state_bits;
    state_.protocol_fault = EncodeFaultBits(instance_->lz_feedback.fault_bits);
    state_.protocol_state = DecodeLzProtocolState(instance_->lz_feedback.state_bits, state_.protocol_fault);
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::LZ, Model>::Register() {
    if (instance_ != nullptr) {
        state_.protocol_state = MotorProtocolState::Registered;
        return DEVICE_OK;
    }
    const int8_t ret = MOTOR_LZ_AttachExternal(&param_, &vendor_instance_);
    if (ret == DEVICE_OK) {
        instance_ = &vendor_instance_;
        state_.protocol_state = MotorProtocolState::Registered;
    }
    return ret;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::LZ, Model>::Enable() {
    if (instance_ != nullptr && instance_->motor.header.online) {
        const uint32_t fault_code = EncodeFaultBits(instance_->lz_feedback.fault_bits);
        if (fault_code == 0u && instance_->lz_feedback.state_bits == MOTOR_LZ_STATE_MOTOR) {
            state_.protocol_status_code = instance_->lz_feedback.state_bits;
            state_.protocol_fault = fault_code;
            return DEVICE_OK;
        }
    }
    return MOTOR_LZ_Enable(&param_);
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::LZ, Model>::Disable() {
    ClearPendingCommand();
    if (instance_ != nullptr && instance_->motor.header.online) {
        const uint32_t fault_code = EncodeFaultBits(instance_->lz_feedback.fault_bits);
        if (fault_code == 0u && instance_->lz_feedback.state_bits == MOTOR_LZ_STATE_RESET) {
            state_.protocol_status_code = instance_->lz_feedback.state_bits;
            state_.protocol_fault = fault_code;
            return DEVICE_OK;
        }
    }
    return MOTOR_LZ_Disable(&param_, false);
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::LZ, Model>::Relax() {
    ClearPendingCommand();
    return MOTOR_LZ_Relax(&param_);
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::LZ, Model>::SetZero() {
    ClearPendingCommand();
    const int8_t ret = MOTOR_LZ_SetZero(&param_);
    if (ret == DEVICE_OK) {
        state_.position_rad = 0.0f;
        state_.position_single_turn_rad = 0.0f;
        state_.last_commit_ok = true;
    } else {
        state_.last_commit_ok = false;
    }
    return ret;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::LZ, Model>::Update() {
    const int8_t ret = MOTOR_LZ_Update(&param_);
    RefreshStateCache();
    return ret;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::LZ, Model>::CommitCommand() {
    int8_t ret = DEVICE_OK;
    switch (pending_type_) {
        case PendingCommandType::Velocity:
            ret = MOTOR_LZ_VelocityControl(&param_, pending_velocity_);
            break;
        case PendingCommandType::Position:
            ret = MOTOR_LZ_PositionControl(&param_, pending_position_, pending_position_velocity_limit_);
            break;
        case PendingCommandType::Mit:
            ret = MOTOR_LZ_MotionControl(&param_, &pending_motion_output_);
            break;
        case PendingCommandType::None:
        default:
            state_.last_commit_ok = true;
            return DEVICE_OK;
    }

    state_.last_commit_ok = (ret == DEVICE_OK);
    if (ret == DEVICE_OK) {
        ClearPendingCommand();
    }
    return ret;
}

template <MotorModel Model>
void MotorProtocol<MotorKind::LZ, Model>::ClearPendingCommand() {
    pending_type_ = PendingCommandType::None;
    pending_velocity_ = 0.0f;
    pending_position_ = 0.0f;
    pending_position_velocity_limit_ = 0.0f;
    pending_motion_output_ = {};
    state_.command_pending = false;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::LZ, Model>::SetTorque(float torque_nm) {
    constexpr float peak_torque = MotorTraits<MotorKind::LZ, Model>::kPeakTorque;
    constexpr float rated_torque = MotorTraits<MotorKind::LZ, Model>::kRatedTorque;
    const float max_torque_nm = (peak_torque > 0.0f) ? peak_torque : rated_torque;
    if (max_torque_nm <= 0.0f) {
        return DEVICE_ERR;
    }

    const float hold_position = state_.online ? state_.position_rad : 0.0f;
    return SetMIT(hold_position,
                  0.0f,
                  0.0f,
                  0.0f,
                  AbsClip(torque_nm, max_torque_nm));
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::LZ, Model>::SetVelocity(float velocity) {
    pending_velocity_ = AbsClip(ToRotorVelocity(velocity), ToRotorVelocity(MotorTraits<MotorKind::LZ, Model>::kMaxVelocity));
    pending_type_ = PendingCommandType::Velocity;
    state_.command_pending = true;
    return DEVICE_OK;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::LZ, Model>::SetPosition(float position, float max_velocity) {
    constexpr float max_velocity_limit = MotorTraits<MotorKind::LZ, Model>::kMaxVelocity;
    constexpr float max_position_limit = MotorTraits<MotorKind::LZ, Model>::kMaxPosition;
    const float output_velocity_limit = (max_velocity > 0.0f) ? AbsClip(max_velocity, max_velocity_limit) : max_velocity_limit;
    const float rotor_position = ToRotorPosition(position);
    const float rotor_position_limit = (max_position_limit > 0.0f) ? ToRotorPosition(max_position_limit) : 0.0f;
    pending_position_ = (rotor_position_limit > 0.0f) ? AbsClip(rotor_position, rotor_position_limit) : rotor_position;
    pending_position_velocity_limit_ = ToRotorVelocity(output_velocity_limit);
    pending_type_ = PendingCommandType::Position;
    state_.command_pending = true;
    return DEVICE_OK;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::LZ, Model>::SetMIT(float position, float velocity, float kp, float kd, float torque_ff) {
    constexpr float max_position_limit = MotorTraits<MotorKind::LZ, Model>::kMaxPosition;
    constexpr float max_velocity_limit = MotorTraits<MotorKind::LZ, Model>::kMaxVelocity;
    constexpr float peak_torque = MotorTraits<MotorKind::LZ, Model>::kPeakTorque;
    const float rotor_position = ToRotorPosition(position);
    const float rotor_velocity = ToRotorVelocity(velocity);
    const float rotor_position_limit = (max_position_limit > 0.0f) ? ToRotorPosition(max_position_limit) : 0.0f;
    const float rotor_velocity_limit = ToRotorVelocity(max_velocity_limit);
    pending_motion_output_.target_angle = (rotor_position_limit > 0.0f) ? AbsClip(rotor_position, rotor_position_limit) : rotor_position;
    pending_motion_output_.target_velocity = AbsClip(rotor_velocity, rotor_velocity_limit);
    pending_motion_output_.kp = kp;
    pending_motion_output_.kd = kd;
    pending_motion_output_.torque = (peak_torque > 0.0f) ? AbsClip(torque_ff, peak_torque) : torque_ff;
    pending_type_ = PendingCommandType::Mit;
    state_.command_pending = true;
    return DEVICE_OK;
}

template class MotorProtocol<MotorKind::LZ, MotorModel::RSO0>;
template class MotorProtocol<MotorKind::LZ, MotorModel::RSO1>;
template class MotorProtocol<MotorKind::LZ, MotorModel::RSO2>;
template class MotorProtocol<MotorKind::LZ, MotorModel::RSO3>;
template class MotorProtocol<MotorKind::LZ, MotorModel::RSO4>;
template class MotorProtocol<MotorKind::LZ, MotorModel::RSO5>;
template class MotorProtocol<MotorKind::LZ, MotorModel::RSO6>;

} // namespace mr::motor
