#include "device/motor/protocol/dm_protocol.hpp"

#include "component/math/scalar.hpp"
#include "device/device.h"

namespace mr::motor {

namespace {

constexpr float kDmPositionMin = -12.56637f;
constexpr float kDmPositionMax = 12.56637f;
constexpr float kDmPositionSpan = kDmPositionMax - kDmPositionMin;
constexpr float kDmVelocityMin = -30.0f;
constexpr float kDmVelocityMax = 30.0f;
constexpr float kDmTorqueMin = -12.0f;
constexpr float kDmTorqueMax = 12.0f;

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
    : config_(config),
      install_(install),
      state_(state),
      mapper_(TransmissionMapper::FromExternalRatio(install.external_ratio,
                                                    MotorTraits<MotorKind::DM, Model>::kTorqueConstant)),
      param_(config.ToVendorParam(module)) {
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

    rotor_position_rad = mr::component::math::uint_to_float(raw->raw_angle,
                                                            kDmPositionMin,
                                                            kDmPositionMax,
                                                            16U);
    if (param_.reverse) {
        rotor_position_rad = -rotor_position_rad;
    }

    rotor_velocity_rad_s = mr::component::math::uint_to_float(static_cast<uint16_t>(raw->raw_speed),
                                                              kDmVelocityMin,
                                                              kDmVelocityMax,
                                                              12U);
    if (param_.reverse) {
        rotor_velocity_rad_s = -rotor_velocity_rad_s;
    }

    torque_current = mr::component::math::uint_to_float(static_cast<uint16_t>(raw->raw_current),
                                                        kDmTorqueMin,
                                                        kDmTorqueMax,
                                                        12U);
    if (param_.reverse) {
        torque_current = -torque_current;
    }

    temperature_c = static_cast<float>(raw->raw_temp);
    return true;
}

template <MotorModel Model>
void MotorProtocol<MotorKind::DM, Model>::RefreshStateCache() {
    MotorState next = state_;
    const MOTOR_t* motor = instance_ ? &instance_->motor : nullptr;
    if (motor == nullptr) {
        position_tracker_.Reset();
        last_online_ = false;
        state_ = {};
        return;
    }

    next.online = motor->header.online;
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
                                     kDmPositionSpan,
                                     kDefaultRotorResyncDeltaRad));
    next.position_single_turn_rad = mr::component::math::wrap_to_pi(next.position_rad);
    next.velocity_rad_s = mapper_.ToOutputVelocity(rotor_velocity_rad_s);
    next.torque_nm = mapper_.ToOutputTorque(torque_current);
    next.temperature_c = temperature_c;
    next.device_temperature_c = static_cast<float>(instance_->rotor_temp);
    next.protocol_status_code = instance_->status_raw;
    next.protocol_fault = EncodeDmFault(instance_->status);
    next.protocol_state = DecodeDmProtocolState(instance_->status);

    last_online_ = true;
    state_ = next;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::Register() {
    if (instance_ != nullptr) {
        state_.protocol_state = MotorProtocolState::Registered;
        return DEVICE_OK;
    }
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
        state_.protocol_status_code = instance_->status_raw;
        state_.protocol_fault = EncodeDmFault(instance_->status);
        return DEVICE_OK;
    }
    return MOTOR_DM_Enable(&param_);
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::Disable() {
    ClearPendingCommand();
    if (instance_ != nullptr && instance_->motor.header.online && instance_->status == MOTOR_DM_STATUS_DISABLED) {
        state_.protocol_status_code = instance_->status_raw;
        state_.protocol_fault = EncodeDmFault(instance_->status);
        return DEVICE_OK;
    }
    return MOTOR_DM_Disable(&param_);
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::Relax() {
    ClearPendingCommand();
    return MOTOR_DM_Relax(&param_);
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::SetZero() {
    ClearPendingCommand();
    const int8_t ret = MOTOR_DM_SetZero(&param_);
    if (ret == DEVICE_OK) {
        position_tracker_.Reset();
        ResetZeroedState(state_);
    } else {
        MarkCommandFailed(state_);
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

    debug_.last_commit_ret = ret;
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
    debug_.pending_type = pending_type_;
    debug_.pending_velocity = pending_velocity_;
    debug_.pending_position = pending_position_;
    debug_.pending_position_velocity_limit = pending_position_velocity_limit_;
    debug_.pending_mit_torque = pending_mit_output_.torque;
    MarkNoPendingCommand(state_);
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::SetTorque(float torque_nm) {
    constexpr float peak_torque = MotorTraits<MotorKind::DM, Model>::kPeakTorque;
    constexpr float rated_torque = MotorTraits<MotorKind::DM, Model>::kRatedTorque;
    const float max_torque_nm = (peak_torque > 0.0f) ? peak_torque : rated_torque;
    if (max_torque_nm <= 0.0f) {
        return DEVICE_ERR;
    }
    return SetMIT(0.0f,
                  0.0f,
                  0.0f,
                  0.0f,
                  mr::component::math::abs_clip_scalar(torque_nm,
                                                       max_torque_nm));
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::SetVelocity(float velocity) {
    pending_velocity_ = PrepareVelocityCommand(mapper_, velocity, MotorTraits<MotorKind::DM, Model>::kMaxVelocity);
    pending_type_ = PendingCommandType::Velocity;
    debug_.pending_type = pending_type_;
    debug_.pending_velocity = pending_velocity_;
    state_.command_pending = true;
    return DEVICE_OK;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::SetPosition(float position, float max_velocity) {
    constexpr float max_velocity_limit = MotorTraits<MotorKind::DM, Model>::kMaxVelocity;
    constexpr float max_position_limit = MotorTraits<MotorKind::DM, Model>::kMaxPosition;
    const PositionCommandValues command = PreparePositionCommand(mapper_,
                                                                 position,
                                                                 max_velocity,
                                                                 max_position_limit,
                                                                 max_velocity_limit);
    pending_position_ = command.rotor_position;
    pending_position_velocity_limit_ = command.rotor_velocity_limit;
    pending_type_ = PendingCommandType::Position;
    debug_.pending_type = pending_type_;
    debug_.pending_position = pending_position_;
    debug_.pending_position_velocity_limit = pending_position_velocity_limit_;
    state_.command_pending = true;
    return DEVICE_OK;
}

template <MotorModel Model>
int8_t MotorProtocol<MotorKind::DM, Model>::SetMIT(float position, float velocity, float kp, float kd, float torque_ff) {
    constexpr float max_position_limit = MotorTraits<MotorKind::DM, Model>::kMaxPosition;
    constexpr float max_velocity_limit = MotorTraits<MotorKind::DM, Model>::kMaxVelocity;
    constexpr float peak_torque = MotorTraits<MotorKind::DM, Model>::kPeakTorque;
    const MitCommandValues command = PrepareMitCommand(mapper_,
                                                       position,
                                                       velocity,
                                                       torque_ff,
                                                       max_position_limit,
                                                       max_velocity_limit,
                                                       peak_torque);
    pending_mit_output_.angle = command.rotor_position;
    pending_mit_output_.velocity = command.rotor_velocity;
    pending_mit_output_.kp = kp;
    pending_mit_output_.kd = kd;
    pending_mit_output_.torque = command.torque_ff;
    pending_type_ = PendingCommandType::Mit;
    debug_.pending_type = pending_type_;
    debug_.pending_position = pending_mit_output_.angle;
    debug_.pending_velocity = pending_mit_output_.velocity;
    debug_.pending_mit_torque = pending_mit_output_.torque;
    state_.command_pending = true;
    return DEVICE_OK;
}

template <MotorModel Model>
const DmProtocolDebugSnapshot& MotorProtocol<MotorKind::DM, Model>::GetDebugSnapshot() const {
    return debug_;
}

template class MotorProtocol<MotorKind::DM, MotorModel::J10010>;
template class MotorProtocol<MotorKind::DM, MotorModel::J10010L>;
template class MotorProtocol<MotorKind::DM, MotorModel::J10422P>;
template class MotorProtocol<MotorKind::DM, MotorModel::J3507>;
template class MotorProtocol<MotorKind::DM, MotorModel::J4310>;
template class MotorProtocol<MotorKind::DM, MotorModel::J4310P>;
template class MotorProtocol<MotorKind::DM, MotorModel::J4340>;
template class MotorProtocol<MotorKind::DM, MotorModel::J4340P>;
template class MotorProtocol<MotorKind::DM, MotorModel::J6006>;
template class MotorProtocol<MotorKind::DM, MotorModel::J6248P>;
template class MotorProtocol<MotorKind::DM, MotorModel::J8006>;
template class MotorProtocol<MotorKind::DM, MotorModel::J8009>;
template class MotorProtocol<MotorKind::DM, MotorModel::J8009P>;
template class MotorProtocol<MotorKind::DM, MotorModel::H3510>;

} // namespace mr::motor
