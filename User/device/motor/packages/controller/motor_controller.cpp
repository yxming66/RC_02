#include "device/motor/packages/controller/motor_controller.hpp"

#include <math.h>

#include "component/user_math.h"

namespace mr::motor {

namespace {

float ClampLimit(float value) {
    return (value > 0.0f) ? value : 0.0f;
}

float ResolvePositiveRatio(float ratio) {
    return (ratio > 0.0f) ? ratio : 1.0f;
}

} // namespace

template <typename MotorType>
MotorControllerT<MotorType>::MotorControllerT(MotorType& motor,
                                              const MotorControllerConfig& config)
    : motor_(motor),
      config_(config),
      velocity_pid_ {},
      position_pid_ {},
      mode_(ControlMode::Torque),
      target_torque_(0.0f),
      target_velocity_(0.0f),
      target_position_(0.0f),
      position_velocity_limit_(config.position_to_velocity_limit),
      velocity_torque_limit_(config.velocity_to_torque_limit),
      target_mit_kp_(0.0f),
      target_mit_kd_(0.0f),
      target_mit_torque_ff_(0.0f) {
    if (config_.velocity_pid != nullptr && config_.sample_freq > 0.0f) {
        PID_Init(&velocity_pid_, KPID_MODE_CALC_D, config_.sample_freq,
                 config_.velocity_pid);
    }
    if (config_.position_pid != nullptr && config_.sample_freq > 0.0f) {
        PID_Init(&position_pid_, KPID_MODE_CALC_D, config_.sample_freq,
                 config_.position_pid);
    }
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::Register() {
    return motor_.Register();
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::Enable() {
    mode_ = ControlMode::Torque;
    target_torque_ = 0.0f;
    target_velocity_ = motor_.GetState().velocity_rad_s;
    target_position_ = motor_.GetState().position_rad;
    position_velocity_limit_ = config_.position_to_velocity_limit;
    velocity_torque_limit_ = config_.velocity_to_torque_limit;
    target_mit_kp_ = 0.0f;
    target_mit_kd_ = 0.0f;
    target_mit_torque_ff_ = 0.0f;
    ResetControllers();
    return motor_.Enable();
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::Disable() {
    mode_ = ControlMode::Torque;
    target_torque_ = 0.0f;
    ResetControllers();
    return motor_.Disable();
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::Relax() {
    mode_ = ControlMode::Torque;
    target_torque_ = 0.0f;
    ResetControllers();
    return motor_.Relax();
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::Update() {
    int8_t ret = motor_.Update();
    if (ret != DEVICE_OK) {
        return ret;
    }

    const MotorState state = motor_.GetState();
    const float dt =
        (config_.sample_freq > 0.0f) ? (1.0f / config_.sample_freq) : 0.0f;

    switch (mode_) {
    case ControlMode::Torque:
        return motor_.SetTorque(target_torque_);

    case ControlMode::NativeVelocity:
        return motor_.SetVelocity(target_velocity_);

    case ControlMode::EmulatedVelocity: {
        if (config_.velocity_pid == nullptr || dt <= 0.0f) {
            return DEVICE_ERR_UNSUPPORTED;
        }
        float torque_cmd = PID_Calc(&velocity_pid_, target_velocity_,
                                    state.velocity_rad_s, 0.0f, dt);
        torque_cmd =
            AbsClip(torque_cmd, ResolveTorqueLimit(velocity_torque_limit_));
        return motor_.SetTorque(torque_cmd);
    }

    case ControlMode::NativePosition:
        return motor_.SetPosition(target_position_, position_velocity_limit_);

    case ControlMode::EmulatedPosition: {
        if (config_.position_pid == nullptr || config_.velocity_pid == nullptr ||
            dt <= 0.0f) {
            return DEVICE_ERR_UNSUPPORTED;
        }
        float velocity_cmd = PID_Calc(&position_pid_, target_position_,
                                      state.position_rad, 0.0f, dt);
        velocity_cmd =
            AbsClip(velocity_cmd, ResolveVelocityLimit(position_velocity_limit_));
        float torque_cmd =
            PID_Calc(&velocity_pid_, velocity_cmd, state.velocity_rad_s, 0.0f, dt);
        torque_cmd =
            AbsClip(torque_cmd, ResolveTorqueLimit(velocity_torque_limit_));
        return motor_.SetTorque(torque_cmd);
    }

    case ControlMode::NativeMit:
        return motor_.SetMIT(target_position_, target_velocity_, target_mit_kp_,
                             target_mit_kd_, target_mit_torque_ff_);
    }

    return DEVICE_ERR;
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::CommitCommand() {
    return motor_.CommitCommand();
}

template <typename MotorType>
bool MotorControllerT<MotorType>::HasPendingCommand() const {
    return motor_.HasPendingCommand();
}

template <typename MotorType>
void MotorControllerT<MotorType>::ClearPendingCommand() {
    motor_.ClearPendingCommand();
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::SetTorque(float torque_nm) {
    if (mode_ != ControlMode::Torque) {
        ResetControllers();
    }
    mode_ = ControlMode::Torque;
    target_torque_ = torque_nm;
    position_velocity_limit_ = config_.position_to_velocity_limit;
    velocity_torque_limit_ = config_.velocity_to_torque_limit;
    return DEVICE_OK;
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::SetVelocity(float velocity) {
    target_velocity_ = velocity;
    target_position_ = motor_.GetState().position_rad;
    velocity_torque_limit_ = config_.velocity_to_torque_limit;
    
    if constexpr (MotorType::Traits::kSupportsVelocity) {
        if (mode_ != ControlMode::NativeVelocity) {
            ResetControllers();
        }
        mode_ = ControlMode::NativeVelocity;
        return DEVICE_OK;
    }

    if (config_.velocity_pid == nullptr) {
        return DEVICE_ERR_UNSUPPORTED;
    }

    if (mode_ != ControlMode::EmulatedVelocity) {
        ResetControllers();
    }
    mode_ = ControlMode::EmulatedVelocity;
    return DEVICE_OK;
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::SetPosition(float position,
                                                float max_velocity) {
    target_position_ = position;
    position_velocity_limit_ =
        (max_velocity > 0.0f) ? max_velocity : config_.position_to_velocity_limit;
    velocity_torque_limit_ = config_.velocity_to_torque_limit;

    if constexpr (MotorType::Traits::kSupportsPosition) {
        if (mode_ != ControlMode::NativePosition) {
            ResetControllers();
        }
        mode_ = ControlMode::NativePosition;
        return DEVICE_OK;
    }

    if (config_.position_pid == nullptr || config_.velocity_pid == nullptr) {
        return DEVICE_ERR_UNSUPPORTED;
    }

    if (mode_ != ControlMode::EmulatedPosition) {
        ResetControllers();
    }
    mode_ = ControlMode::EmulatedPosition;
    return DEVICE_OK;
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::SetMIT(float position, float velocity,
                                           float kp, float kd,
                                           float torque_ff) {
    if constexpr (!MotorType::Traits::kSupportsMit) {
        return DEVICE_ERR_UNSUPPORTED;
    }

    if (mode_ != ControlMode::NativeMit) {
        ResetControllers();
    }
    mode_ = ControlMode::NativeMit;
    target_position_ = position;
    target_velocity_ = velocity;
    target_mit_kp_ = kp;
    target_mit_kd_ = kd;
    target_mit_torque_ff_ = torque_ff;
    return DEVICE_OK;
}

template <typename MotorType>
MotorState MotorControllerT<MotorType>::GetState() const {
    return motor_.GetState();
}

template <typename MotorType>
const MotorInstallSpec& MotorControllerT<MotorType>::GetInstallConfig() const {
    return motor_.GetInstallConfig();
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::ResetControllers() {
    if (config_.velocity_pid != nullptr) {
        PID_Reset(&velocity_pid_);
    }
    if (config_.position_pid != nullptr) {
        PID_Reset(&position_pid_);
    }
    return DEVICE_OK;
}

template <typename MotorType>
float MotorControllerT<MotorType>::ResolveVelocityLimit(
    float request_limit) const {
    const float limit = ClampLimit(request_limit);
    if (limit > 0.0f) {
        return limit;
    }

    const float recommended_limit = ClampLimit(MotorType::recommended_velocity());
    if (recommended_limit > 0.0f) {
        return recommended_limit;
    }

    const float rated_limit = ClampLimit(MotorType::rated_velocity());
    if (rated_limit > 0.0f) {
        return rated_limit;
    }

    return ClampLimit(MotorType::max_velocity());
}

template <typename MotorType>
float MotorControllerT<MotorType>::ResolveTorqueLimit(float request_limit) const {
    float limit = ClampLimit(request_limit);
    if (limit > 0.0f) {
        return limit;
    }

    if (mode_ == ControlMode::EmulatedVelocity ||
        mode_ == ControlMode::EmulatedPosition) {
        const float ratio = ResolvePositiveRatio(MotorType::gear_ratio());
        const float output_ratio =
            ratio * ResolvePositiveRatio(motor_.GetInstallConfig().external_ratio);
        const float recommended_limit =
            ClampLimit(MotorType::recommended_current());
        if (recommended_limit > 0.0f) {
            return MotorType::torque_constant() > 0.0f
                       ? recommended_limit * MotorType::torque_constant() *
                             output_ratio
                       : recommended_limit;
        }
        const float rated_limit = ClampLimit(MotorType::rated_current());
        if (rated_limit > 0.0f) {
            return MotorType::torque_constant() > 0.0f
                       ? rated_limit * MotorType::torque_constant() * output_ratio
                       : rated_limit;
        }
        const float legacy_limit = ClampLimit(MotorType::peak_current());
        if (legacy_limit > 0.0f) {
            return MotorType::torque_constant() > 0.0f
                       ? legacy_limit * MotorType::torque_constant() *
                             output_ratio
                       : legacy_limit;
        }
    }

    return 0.0f;
}

template class MotorControllerT<mr::motor::DmJ4310Motor>;
template class MotorControllerT<mr::motor::RmM3508Motor>;

} // namespace mr::motor
