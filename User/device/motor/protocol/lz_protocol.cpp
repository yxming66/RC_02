#include "device/motor/protocol/lz_protocol.hpp"

#include "component/math/scalar.hpp"
#include "device/device.h"

namespace mr::motor {

namespace {

constexpr float kLzAngleRangeRad = 12.57f;
constexpr float kLzVelocityRangeRadS = 20.0f;

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
    : config_(config),
      install_(install),
      state_(state),
      mapper_(TransmissionMapper::FromExternalRatio(install.external_ratio,
                                                    MotorTraits<MotorKind::LZ, Model>::kTorqueConstant)),
      param_(config.ToVendorParam(module)) {
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

    rotor_position_rad = mr::component::math::uint_to_float(raw->raw_angle,
                                                            -kLzAngleRangeRad,
                                                            kLzAngleRangeRad,
                                                            16U);
    if (param_.reverse) {
        rotor_position_rad = -rotor_position_rad;
    }

    rotor_velocity_rad_s = mr::component::math::uint_to_float(static_cast<uint16_t>(raw->raw_speed),
                                                              -kLzVelocityRangeRadS,
                                                              kLzVelocityRangeRadS,
                                                              16U);
    if (param_.reverse) {
        rotor_velocity_rad_s = -rotor_velocity_rad_s;
    }

    torque_current = mr::component::math::uint_to_float(static_cast<uint16_t>(raw->raw_current),
                                                        -MotorTraits<MotorKind::LZ, Model>::kPeakTorque,
                                                        MotorTraits<MotorKind::LZ, Model>::kPeakTorque,
                                                        16U);
    if (param_.reverse) {
        torque_current = -torque_current;
    }

    temperature_c = static_cast<float>(raw->raw_temp);
    return true;
}

template <MotorModel Model>
void MotorProtocol<MotorKind::LZ, Model>::RefreshStateCache() {
    MotorState next = state_;
    if (instance_ == nullptr) {
        position_tracker_.Reset();
        last_online_ = false;
        state_ = {};
        return;
    }

    next.online = instance_->motor.header.online;
    if (!next.online) {
        position_tracker_.Reset();
        last_online_ = false;
        state_ = next;
        return;
    }

    float rotor_position_rad = 0.0f;
    float rotor_velocity_rad_s = 0.0f;
    float torque_current = 0.0f;
    float temperature_c = 0.0f;
    if (!TryGetRotorFeedback(rotor_position_rad, rotor_velocity_rad_s, torque_current, temperature_c)) {
        position_tracker_.Reset();
        last_online_ = false;
        next.online = false;
        state_ = next;
        return;
    }

    if (!last_online_) {
        position_tracker_.Sync(rotor_position_rad);
    }

    next.position_rad = mapper_.ToOutputPosition(
        position_tracker_.Accumulate(rotor_position_rad,
                                     rotor_velocity_rad_s,
                                     2.0f * kLzAngleRangeRad,
                                     kDefaultRotorResyncDeltaRad));
    next.position_single_turn_rad = mr::component::math::wrap_to_pi(next.position_rad);
    next.velocity_rad_s = mapper_.ToOutputVelocity(rotor_velocity_rad_s);
    next.torque_nm = mapper_.ToOutputTorque(torque_current);
    next.temperature_c = temperature_c;
    next.protocol_status_code = instance_->lz_feedback.state_bits;
    next.protocol_fault = EncodeFaultBits(instance_->lz_feedback.fault_bits);
    next.protocol_state = DecodeLzProtocolState(instance_->lz_feedback.state_bits, next.protocol_fault);

    last_online_ = true;
    state_ = next;
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
        position_tracker_.Reset();
        ResetZeroedState(state_);
    } else {
        MarkCommandFailed(state_);
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

#if MOTOR_PROTOCOL_DEBUG_ENABLE
    debug_.last_commit_ret = ret;
#endif
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
#if MOTOR_PROTOCOL_DEBUG_ENABLE
    debug_.pending_type = pending_type_;
    debug_.pending_velocity = pending_velocity_;
    debug_.pending_position = pending_position_;
    debug_.pending_position_velocity_limit = pending_position_velocity_limit_;
    debug_.pending_motion_torque = pending_motion_output_.torque;
#endif
    MarkNoPendingCommand(state_);
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
                  mr::component::math::abs_clip_scalar(torque_nm,
                                                       max_torque_nm));
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::LZ, Model>::SetVelocity(float velocity) {
    pending_velocity_ = PrepareVelocityCommand(mapper_, velocity, MotorTraits<MotorKind::LZ, Model>::kMaxVelocity);
    pending_type_ = PendingCommandType::Velocity;
#if MOTOR_PROTOCOL_DEBUG_ENABLE
    debug_.pending_type = pending_type_;
    debug_.pending_velocity = pending_velocity_;
#endif
    state_.command_pending = true;
    return DEVICE_OK;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::LZ, Model>::SetPosition(float position, float max_velocity) {
    constexpr float max_velocity_limit = MotorTraits<MotorKind::LZ, Model>::kMaxVelocity;
    constexpr float max_position_limit = MotorTraits<MotorKind::LZ, Model>::kMaxPosition;
    const PositionCommandValues command = PreparePositionCommand(mapper_,
                                                                 position,
                                                                 max_velocity,
                                                                 max_position_limit,
                                                                 max_velocity_limit);
    pending_position_ = command.rotor_position;
    pending_position_velocity_limit_ = command.rotor_velocity_limit;
    pending_type_ = PendingCommandType::Position;
#if MOTOR_PROTOCOL_DEBUG_ENABLE
    debug_.pending_type = pending_type_;
    debug_.pending_position = pending_position_;
    debug_.pending_position_velocity_limit = pending_position_velocity_limit_;
#endif
    state_.command_pending = true;
    return DEVICE_OK;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::LZ, Model>::SetMIT(float position, float velocity, float kp, float kd, float torque_ff) {
    constexpr float max_position_limit = MotorTraits<MotorKind::LZ, Model>::kMaxPosition;
    constexpr float max_velocity_limit = MotorTraits<MotorKind::LZ, Model>::kMaxVelocity;
    constexpr float peak_torque = MotorTraits<MotorKind::LZ, Model>::kPeakTorque;
    const MitCommandValues command = PrepareMitCommand(mapper_,
                                                       position,
                                                       velocity,
                                                       torque_ff,
                                                       max_position_limit,
                                                       max_velocity_limit,
                                                       peak_torque);
    pending_motion_output_.target_angle = command.rotor_position;
    pending_motion_output_.target_velocity = command.rotor_velocity;
    pending_motion_output_.kp = kp;
    pending_motion_output_.kd = kd;
    pending_motion_output_.torque = command.torque_ff;
    pending_type_ = PendingCommandType::Mit;
#if MOTOR_PROTOCOL_DEBUG_ENABLE
    debug_.pending_type = pending_type_;
    debug_.pending_position = pending_motion_output_.target_angle;
    debug_.pending_velocity = pending_motion_output_.target_velocity;
    debug_.pending_motion_torque = pending_motion_output_.torque;
#endif
    state_.command_pending = true;
    return DEVICE_OK;
}

#if MOTOR_PROTOCOL_DEBUG_ENABLE
template <MotorModel Model>
const LzProtocolDebugSnapshot& MotorProtocol<MotorKind::LZ, Model>::GetDebugSnapshot() const {
    return debug_;
}
#endif

template class MotorProtocol<MotorKind::LZ, MotorModel::RSO0>;
template class MotorProtocol<MotorKind::LZ, MotorModel::RSO1>;
template class MotorProtocol<MotorKind::LZ, MotorModel::RSO2>;
template class MotorProtocol<MotorKind::LZ, MotorModel::RSO3>;
template class MotorProtocol<MotorKind::LZ, MotorModel::RSO4>;
template class MotorProtocol<MotorKind::LZ, MotorModel::RSO5>;
template class MotorProtocol<MotorKind::LZ, MotorModel::RSO6>;

} // namespace mr::motor
