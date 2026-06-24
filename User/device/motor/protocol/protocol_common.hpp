#pragma once

// 电机协议层通用组件。
// 仅承载协议无关的单位换算、位置累计和命令准备逻辑；厂商驱动调用与反馈解码仍保留在各协议实现中。

#include <math.h>

#include "component/math/scalar.hpp"
#include "device/motor/core/motor_state.hpp"

namespace mr::motor {

inline constexpr float kDefaultRotorResyncDeltaRad =
    1.75f * mr::component::math::kPi;

inline float ResolvePositiveRatio(float ratio) {
    return mr::component::math::positive_or(ratio, 1.0f);
}

struct TransmissionMapper {
    float total_ratio = 1.0f;
    float torque_constant = 0.0f;

    static TransmissionMapper FromExternalRatio(float external_ratio, float torque_constant) {
        return {ResolvePositiveRatio(external_ratio), torque_constant};
    }

    static TransmissionMapper FromTotalRatio(float total_ratio, float torque_constant) {
        return {ResolvePositiveRatio(total_ratio), torque_constant};
    }

    float ToRotorPosition(float output_position_rad) const {
        return output_position_rad * total_ratio;
    }

    float ToRotorVelocity(float output_velocity_rad_s) const {
        return output_velocity_rad_s * total_ratio;
    }

    float ToRotorLimit(float output_limit) const {
        return fabsf(output_limit) * total_ratio;
    }

    float ToOutputPosition(float rotor_position_rad) const {
        return rotor_position_rad / total_ratio;
    }

    float ToOutputVelocity(float rotor_velocity_rad_s) const {
        return rotor_velocity_rad_s / total_ratio;
    }

    float ToOutputTorque(float torque_current) const {
        return torque_current * torque_constant * total_ratio;
    }

    float ToTorqueCurrent(float output_torque_nm, bool reverse_output = false) const {
        const float signed_torque = reverse_output ? -output_torque_nm : output_torque_nm;
        if (torque_constant <= 0.0f || total_ratio <= 0.0f) {
            return 0.0f;
        }
        return signed_torque / (torque_constant * total_ratio);
    }
};

class RotorPositionTracker {
public:
    void Reset() {
        initialized_ = false;
        last_rotor_position_rad_ = 0.0f;
        accumulated_rotor_position_rad_ = 0.0f;
    }

    void Sync(float rotor_position_rad) {
        initialized_ = true;
        last_rotor_position_rad_ = rotor_position_rad;
        accumulated_rotor_position_rad_ = rotor_position_rad;
    }

    float Accumulate(float rotor_position_rad,
                     float rotor_velocity_rad_s,
                     float angle_span_rad,
                     float default_resync_delta_rad) {
        if (!initialized_) {
            Sync(rotor_position_rad);
            return accumulated_rotor_position_rad_;
        }

        const float delta = mr::component::math::wrap_error(
            rotor_position_rad - last_rotor_position_rad_,
            angle_span_rad);
        const float delta_limit = (fabsf(rotor_velocity_rad_s) > 0.0f)
            ? fabsf(rotor_velocity_rad_s) + mr::component::math::kPi
            : default_resync_delta_rad;

        if (fabsf(delta) > delta_limit) {
            Sync(rotor_position_rad);
            return accumulated_rotor_position_rad_;
        }

        accumulated_rotor_position_rad_ += delta;
        last_rotor_position_rad_ = rotor_position_rad;
        return accumulated_rotor_position_rad_;
    }

private:
    bool initialized_ = false;
    float last_rotor_position_rad_ = 0.0f;
    float accumulated_rotor_position_rad_ = 0.0f;
};

struct PositionCommandValues {
    float rotor_position = 0.0f;
    float rotor_velocity_limit = 0.0f;
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
