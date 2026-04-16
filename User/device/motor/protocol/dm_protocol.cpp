#include "device/motor/protocol/dm_protocol.hpp"

#include "component/user_math.h"
#include "device/device.h"

namespace mrobot::motor {

namespace {

float ResolvePositiveRatio(float ratio) {
    return (ratio > 0.0f) ? ratio : 1.0f;
}

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kDmPositionMin = -12.56637f;
constexpr float kDmPositionMax = 12.56637f;
constexpr float kDmVelocityMin = -30.0f;
constexpr float kDmVelocityMax = 30.0f;
constexpr float kDmTorqueMin = -12.0f;
constexpr float kDmTorqueMax = 12.0f;

float UintToFloat(uint16_t raw, float min_value, float max_value, int bits) {
    const float span = max_value - min_value;
    return (static_cast<float>(raw) * span / static_cast<float>((1 << bits) - 1)) + min_value;
}

float WrapToPi(float angle_rad) {
    while (angle_rad >= kPi) {
        angle_rad -= kTwoPi;
    }
    while (angle_rad < -kPi) {
        angle_rad += kTwoPi;
    }
    return angle_rad;
}

uint32_t EncodeDmFault(MOTOR_DM_Status_t status) {
    switch (status) {
        case MOTOR_DM_STATUS_OVERVOLTAGE: return (1u << 0);
        case MOTOR_DM_STATUS_UNDERVOLTAGE: return (1u << 1);
        case MOTOR_DM_STATUS_OVERCURRENT: return (1u << 2);
        case MOTOR_DM_STATUS_MOS_OVERTEMP: return (1u << 3);
        case MOTOR_DM_STATUS_COIL_OVERTEMP: return (1u << 4);
        case MOTOR_DM_STATUS_COMM_LOST: return (1u << 5);
        case MOTOR_DM_STATUS_OVERLOAD: return (1u << 6);
        default: return 0u;
    }
}

MotorProtocolState DecodeDmProtocolState(MOTOR_DM_Status_t status) {
    switch (status) {
        case MOTOR_DM_STATUS_ENABLED:
            return MotorProtocolState::Enabled;
        case MOTOR_DM_STATUS_DISABLED:
            return MotorProtocolState::Disabled;
        case MOTOR_DM_STATUS_OVERVOLTAGE:
        case MOTOR_DM_STATUS_UNDERVOLTAGE:
        case MOTOR_DM_STATUS_OVERCURRENT:
        case MOTOR_DM_STATUS_MOS_OVERTEMP:
        case MOTOR_DM_STATUS_COIL_OVERTEMP:
        case MOTOR_DM_STATUS_COMM_LOST:
        case MOTOR_DM_STATUS_OVERLOAD:
            return MotorProtocolState::Fault;
        default:
            return MotorProtocolState::Registered;
    }
}

} // namespace

template <MotorModel Model>
MotorProtocol<MotorKind::DM, Model>::MotorProtocol(const MotorInstanceConfig<MotorKind::DM>& config,
                                            const MotorInstallSpec& install,
                                            MotorState& state,
                                            MOTOR_DM_Module_t module)
    : config_(config), install_(install), state_(state), param_(config.ToVendorParam(module)) {
}

template <MotorModel Model>
float MotorProtocol<MotorKind::DM, Model>::TotalRatio() const {
    return ResolvePositiveRatio(MotorTraits<MotorKind::DM, Model>::kGearRatio) * ResolvePositiveRatio(install_.external_ratio);
}

template <MotorModel Model>
float MotorProtocol<MotorKind::DM, Model>::ToRotorPosition(float output_position_rad) const {
    const float signed_position = install_.reverse_output ? -output_position_rad : output_position_rad;
    return signed_position * TotalRatio();
}

template <MotorModel Model>
float MotorProtocol<MotorKind::DM, Model>::ToRotorVelocity(float output_velocity_rad_s) const {
    const float signed_velocity = install_.reverse_output ? -output_velocity_rad_s : output_velocity_rad_s;
    return signed_velocity * TotalRatio();
}

template <MotorModel Model>
float MotorProtocol<MotorKind::DM, Model>::ToOutputPosition(float rotor_position_rad) const {
    return rotor_position_rad / TotalRatio();
}

template <MotorModel Model>
float MotorProtocol<MotorKind::DM, Model>::ToOutputVelocity(float rotor_velocity_rad_s) const {
    return rotor_velocity_rad_s / TotalRatio();
}

template <MotorModel Model>
float MotorProtocol<MotorKind::DM, Model>::ToOutputTorque(float torque_current) const {
    float torque = torque_current * MotorTraits<MotorKind::DM, Model>::kTorqueConstant * TotalRatio();
    return install_.reverse_output ? -torque : torque;
}

template <MotorModel Model>
bool MotorProtocol<MotorKind::DM, Model>::TryGetRotorFeedback(float& rotor_position_rad,
                                                       float& rotor_velocity_rad_s,
                                                       float& torque_current,
                                                       float& temperature_c) const {
    const MOTOR_DM_RawFeedback_t* raw = MOTOR_DM_GetRawFeedback(const_cast<MOTOR_DM_Param_t*>(&param_));
    if (raw == nullptr) {
        return false;
    }

    rotor_position_rad = UintToFloat(raw->raw_angle, kDmPositionMin, kDmPositionMax, 16);
    if (param_.reverse) {
        rotor_position_rad = -rotor_position_rad;
    }

    rotor_velocity_rad_s = UintToFloat(static_cast<uint16_t>(raw->raw_speed), kDmVelocityMin, kDmVelocityMax, 12);
    if (param_.reverse) {
        rotor_velocity_rad_s = -rotor_velocity_rad_s;
    }

    torque_current = UintToFloat(static_cast<uint16_t>(raw->raw_current), kDmTorqueMin, kDmTorqueMax, 12);
    if (param_.reverse) {
        torque_current = -torque_current;
    }

    temperature_c = static_cast<float>(raw->raw_temp);
    return true;
}

template <MotorModel Model>
void MotorProtocol<MotorKind::DM, Model>::RefreshStateCache() {
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
    state_.device_temperature_c = static_cast<float>(instance_->rotor_temp);
    state_.online = instance_->motor.header.online;
    state_.protocol_status_code = instance_->status_raw;
    state_.protocol_fault = EncodeDmFault(instance_->status);
    state_.protocol_state = DecodeDmProtocolState(instance_->status);
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::Register() {
    const int8_t ret = MOTOR_DM_AttachExternal(&param_, &vendor_instance_);
    if (ret == DEVICE_OK) {
        instance_ = &vendor_instance_;
        state_.protocol_state = MotorProtocolState::Registered;
    }
    return ret;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::Enable() {
    if (instance_ != nullptr && instance_->motor.header.online && instance_->status == MOTOR_DM_STATUS_ENABLED) {
        state_.protocol_state = MotorProtocolState::Enabled;
        state_.protocol_status_code = instance_->status_raw;
        state_.protocol_fault = EncodeDmFault(instance_->status);
        return DEVICE_OK;
    }
    const int8_t ret = MOTOR_DM_Enable(&param_);
    if (ret == DEVICE_OK) {
        state_.protocol_state = MotorProtocolState::Enabled;
    }
    return ret;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::Disable() {
    ClearPendingCommand();
    if (instance_ != nullptr && instance_->motor.header.online && instance_->status == MOTOR_DM_STATUS_DISABLED) {
        state_.protocol_state = MotorProtocolState::Disabled;
        state_.protocol_status_code = instance_->status_raw;
        state_.protocol_fault = EncodeDmFault(instance_->status);
        return DEVICE_OK;
    }
    const int8_t ret = MOTOR_DM_Disable(&param_);
    if (ret == DEVICE_OK) {
        state_.protocol_state = MotorProtocolState::Disabled;
    }
    return ret;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::Relax() {
    ClearPendingCommand();
    if (instance_ != nullptr && instance_->motor.header.online && instance_->status == MOTOR_DM_STATUS_DISABLED) {
        state_.protocol_state = MotorProtocolState::Relaxed;
        state_.protocol_status_code = instance_->status_raw;
        state_.protocol_fault = EncodeDmFault(instance_->status);
        return DEVICE_OK;
    }
    const int8_t ret = MOTOR_DM_Relax(&param_);
    if (ret == DEVICE_OK) {
        state_.protocol_state = MotorProtocolState::Relaxed;
    }
    return ret;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::Update() {
    const int8_t ret = MOTOR_DM_Update(&param_);
    if (ret == DEVICE_OK) {
        RefreshStateCache();
    }
    return ret;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::CommitCommand() {
    int8_t ret = DEVICE_OK;
    switch (pending_type_) {
        case PendingCommandType::Velocity:
            ret = MOTOR_DM_VelCtrl(&param_, pending_velocity_);
            break;
        case PendingCommandType::Position:
            ret = MOTOR_DM_PosVelCtrl(&param_, pending_position_, pending_position_velocity_limit_);
            break;
        case PendingCommandType::Mit:
            ret = MOTOR_DM_MITCtrl(&param_, &pending_mit_output_);
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
void MotorProtocol<MotorKind::DM, Model>::ClearPendingCommand() {
    pending_type_ = PendingCommandType::None;
    pending_velocity_ = 0.0f;
    pending_position_ = 0.0f;
    pending_position_velocity_limit_ = 0.0f;
    pending_mit_output_ = {};
    state_.command_pending = false;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::SetTorque(float torque_nm) {
    constexpr float peak_torque = MotorTraits<MotorKind::DM, Model>::kPeakTorque;
    constexpr float rated_torque = MotorTraits<MotorKind::DM, Model>::kRatedTorque;
    const float max_torque_nm = (peak_torque > 0.0f) ? peak_torque : rated_torque;
    if (max_torque_nm <= 0.0f) {
        return DEVICE_ERR;
    }
    return SetMIT(0.0f, 0.0f, 0.0f, 0.0f, AbsClip(torque_nm, max_torque_nm));
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::SetVelocity(float velocity) {
    pending_velocity_ = AbsClip(ToRotorVelocity(velocity), ToRotorVelocity(MotorTraits<MotorKind::DM, Model>::kMaxVelocity));
    pending_type_ = PendingCommandType::Velocity;
    state_.command_pending = true;
    return DEVICE_OK;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::SetPosition(float position, float max_velocity) {
    constexpr float max_velocity_limit = MotorTraits<MotorKind::DM, Model>::kMaxVelocity;
    constexpr float max_position_limit = MotorTraits<MotorKind::DM, Model>::kMaxPosition;
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
int8_t MotorProtocol<MotorKind::DM, Model>::SetMIT(float position, float velocity, float kp, float kd, float torque_ff) {
    constexpr float max_position_limit = MotorTraits<MotorKind::DM, Model>::kMaxPosition;
    constexpr float max_velocity_limit = MotorTraits<MotorKind::DM, Model>::kMaxVelocity;
    constexpr float peak_torque = MotorTraits<MotorKind::DM, Model>::kPeakTorque;
    const float rotor_position = ToRotorPosition(position);
    const float rotor_velocity = ToRotorVelocity(velocity);
    const float rotor_position_limit = (max_position_limit > 0.0f) ? ToRotorPosition(max_position_limit) : 0.0f;
    const float rotor_velocity_limit = ToRotorVelocity(max_velocity_limit);
    pending_mit_output_.angle = (rotor_position_limit > 0.0f) ? AbsClip(rotor_position, rotor_position_limit) : rotor_position;
    pending_mit_output_.velocity = AbsClip(rotor_velocity, rotor_velocity_limit);
    pending_mit_output_.kp = kp;
    pending_mit_output_.kd = kd;
    pending_mit_output_.torque = (peak_torque > 0.0f) ? AbsClip(torque_ff, peak_torque) : torque_ff;
    pending_type_ = PendingCommandType::Mit;
    state_.command_pending = true;
    return DEVICE_OK;
}

template class MotorProtocol<MotorKind::DM, MotorModel::J4310>;

} // namespace mrobot::motor