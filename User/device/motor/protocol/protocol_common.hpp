#pragma once

// 电机协议层通用组件。
// 仅承载协议无关的单位换算、位置累计和命令准备逻辑；厂商驱动调用与反馈解码仍保留在各协议实现中。

#include "component/math/cyclic_tracker.hpp"
#include "component/math/scalar.hpp"
#include "component/math/transmission_mapper.hpp"
#include "device/motor/core/motor_state.hpp"

namespace mr::motor {

inline constexpr float kDefaultRotorResyncDeltaRad =
    1.75f * mr::component::math::kPi;

struct TransmissionMapper : mr::component::math::TransmissionRatioMapper {
    float torque_constant = 0.0f;

    static TransmissionMapper FromExternalRatio(float external_ratio, float torque_constant) {
        return {mr::component::math::TransmissionRatioMapper::FromRatio(external_ratio),
                torque_constant};
    }

    static TransmissionMapper FromTotalRatio(float total_ratio, float torque_constant) {
        return {mr::component::math::TransmissionRatioMapper::FromRatio(total_ratio),
                torque_constant};
    }

    float ToRotorPosition(float output_position_rad) const {
        return ToInput(output_position_rad);
    }

    float ToRotorVelocity(float output_velocity_rad_s) const {
        return ToInput(output_velocity_rad_s);
    }

    float ToRotorLimit(float output_limit) const {
        return ToInputLimit(output_limit);
    }

    float ToOutputPosition(float rotor_position_rad) const {
        return ToOutput(rotor_position_rad);
    }

    float ToOutputVelocity(float rotor_velocity_rad_s) const {
        return ToOutput(rotor_velocity_rad_s);
    }

    float ToOutputTorque(float torque_current) const {
        return torque_current * torque_constant * ratio;
    }

    float ToTorqueCurrent(float output_torque_nm, bool reverse_output = false) const {
        const float signed_torque = reverse_output ? -output_torque_nm : output_torque_nm;
        if (torque_constant <= 0.0f || ratio <= 0.0f) {
            return 0.0f;
        }
        return signed_torque / (torque_constant * ratio);
    }
};

using RotorPositionTracker = mr::component::math::CyclicPositionTracker;

struct PositionCommandValues {
    float rotor_position = 0.0f;
    float rotor_velocity_limit = 0.0f;
};

struct MitCommandValues {
    float rotor_position = 0.0f;
    float rotor_velocity = 0.0f;
    float torque_ff = 0.0f;
};

inline float PrepareVelocityCommand(const TransmissionMapper& mapper,
                                    float output_velocity,
                                    float max_output_velocity) {
    return mr::component::math::abs_clip_scalar(
        mapper.ToRotorVelocity(output_velocity),
        mapper.ToRotorLimit(max_output_velocity));
}

inline PositionCommandValues PreparePositionCommand(const TransmissionMapper& mapper,
                                                    float output_position,
                                                    float max_output_velocity,
                                                    float max_position_limit,
                                                    float max_velocity_limit) {
    const float output_velocity_limit = (max_output_velocity > 0.0f)
        ? mr::component::math::abs_clip_scalar(max_output_velocity, max_velocity_limit)
        : max_velocity_limit;
    const float rotor_position = mapper.ToRotorPosition(output_position);
    const float rotor_position_limit = (max_position_limit > 0.0f)
        ? mapper.ToRotorLimit(max_position_limit)
        : 0.0f;

    PositionCommandValues values {};
    values.rotor_position = (rotor_position_limit > 0.0f)
        ? mr::component::math::abs_clip_scalar(rotor_position, rotor_position_limit)
        : rotor_position;
    values.rotor_velocity_limit = mapper.ToRotorVelocity(output_velocity_limit);
    return values;
}

inline MitCommandValues PrepareMitCommand(const TransmissionMapper& mapper,
                                          float output_position,
                                          float output_velocity,
                                          float torque_ff,
                                          float max_position_limit,
                                          float max_velocity_limit,
                                          float peak_torque) {
    const float rotor_position = mapper.ToRotorPosition(output_position);
    const float rotor_velocity = mapper.ToRotorVelocity(output_velocity);
    const float rotor_position_limit = (max_position_limit > 0.0f)
        ? mapper.ToRotorLimit(max_position_limit)
        : 0.0f;
    const float rotor_velocity_limit = mapper.ToRotorLimit(max_velocity_limit);

    MitCommandValues values {};
    values.rotor_position = (rotor_position_limit > 0.0f)
        ? mr::component::math::abs_clip_scalar(rotor_position, rotor_position_limit)
        : rotor_position;
    values.rotor_velocity = mr::component::math::abs_clip_scalar(
        rotor_velocity,
        rotor_velocity_limit);
    values.torque_ff = (peak_torque > 0.0f)
        ? mr::component::math::abs_clip_scalar(torque_ff, peak_torque)
        : torque_ff;
    return values;
}

inline void ResetZeroedState(MotorState& state) {
    state.position_rad = 0.0f;
    state.position_single_turn_rad = 0.0f;
    state.last_commit_ok = true;
}

inline void MarkCommandFailed(MotorState& state) {
    state.last_commit_ok = false;
}

inline void MarkNoPendingCommand(MotorState& state) {
    state.command_pending = false;
}

} // namespace mr::motor
