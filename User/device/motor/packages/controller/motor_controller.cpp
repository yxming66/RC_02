#include "device/motor/packages/controller/motor_controller.hpp"

namespace mr::motor {

namespace {

namespace cntlr = mr::comp::cntlr;
namespace scalar = mr::component::math;

constexpr float kControllerMaxDtS = 0.05f;

float ClampLimit(float value) {
    return (scalar::is_finite_scalar(value) && value > 0.0f) ? value : 0.0f;
}

float ResolvePositiveRatio(float ratio) {
    return (ratio > 0.0f) ? ratio : 1.0f;
}

mr::comp::timer::Config BuildControlTimerConfig() {
    mr::comp::timer::Config timer_config{};
    timer_config.max_dt_us = mr::comp::timer::SecondsToUs(kControllerMaxDtS);
    return timer_config;
}

bool PidParamsUsable(const KPID_Params_t* params) {
    return params != nullptr &&
           scalar::is_finite_scalar(params->k) &&
           scalar::is_finite_scalar(params->p) &&
           scalar::is_finite_scalar(params->i) &&
           scalar::is_finite_scalar(params->d) &&
           scalar::is_finite_scalar(params->i_limit) &&
           scalar::is_finite_scalar(params->out_limit) &&
           scalar::is_finite_scalar(params->d_cutoff_freq) &&
           scalar::is_finite_scalar(params->range);
}

cntlr::pid::Config BuildPidConfig(const KPID_Params_t* params) {
    cntlr::pid::Config pid_config{};
    pid_config.derivative_mode = cntlr::DerivativeMode::kOnMeasurement;
    pid_config.hold_integral_on_saturation = true;

    if (!PidParamsUsable(params)) {
        return pid_config;
    }

    pid_config.gains.k = params->k;
    pid_config.gains.p = params->p;
    pid_config.gains.i = params->i;
    pid_config.gains.d = params->d;
    pid_config.output_limit =
        cntlr::SymmetricLimit::Abs(ClampLimit(params->out_limit));

    const float integral_output_limit =
        (params->i > 0.0f && params->i_limit > 0.0f)
            ? params->i_limit * params->i
            : cntlr::kUnlimited;
    pid_config.integral_output_limit =
        cntlr::SymmetricLimit::Abs(ClampLimit(integral_output_limit));
    pid_config.error_wrap =
        (params->range > 0.0f) ? cntlr::RangeWrap::Periodic(params->range)
                               : cntlr::RangeWrap::Disabled();
    pid_config.derivative_cutoff_hz = ClampLimit(params->d_cutoff_freq);
    return pid_config;
}

float ApplyRequiredAbsLimit(float value, float limit) {
    const float positive_limit = ClampLimit(limit);
    if (positive_limit <= 0.0f) {
        return 0.0f;
    }
    return cntlr::SymmetricLimit::Abs(positive_limit).Apply(value);
}

} // namespace

template <typename MotorType>
MotorControllerT<MotorType>::MotorControllerT(MotorType& motor,
                                              const MotorControllerConfig& config)
    : motor_(motor),
      config_(config),
      velocity_pid_(BuildPidConfig(config.velocity_pid)),
      position_pid_(BuildPidConfig(config.position_pid)),
      control_timer_(mr::comp::timer::Build(BuildControlTimerConfig())),
      velocity_pid_ready_(PidParamsUsable(config.velocity_pid)),
      position_pid_ready_(PidParamsUsable(config.position_pid)),
      mode_(ControlMode::Torque),
      target_torque_(0.0f),
      target_velocity_(0.0f),
      target_position_(0.0f),
      position_velocity_limit_(config.position_to_velocity_limit),
      velocity_torque_limit_(config.velocity_to_torque_limit),
      target_mit_kp_(0.0f),
      target_mit_kd_(0.0f),
      target_mit_torque_ff_(0.0f) {
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
int8_t MotorControllerT<MotorType>::UpdateFeedback() {
    return motor_.Update();
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::UpdateControl() {
    const float dt = UpdateControlDt();

    const MotorState state = motor_.GetState();

    switch (mode_) {
    case ControlMode::Torque:
        return motor_.SetTorque(target_torque_);

    case ControlMode::NativeVelocity:
        return motor_.SetVelocity(target_velocity_);

    case ControlMode::EmulatedVelocity: {
        if (!velocity_pid_ready_) {
            return DEVICE_ERR_UNSUPPORTED;
        }
        float torque_cmd =
            velocity_pid_.Update(target_velocity_, state.velocity_rad_s, dt);
        torque_cmd = ApplyRequiredAbsLimit(
            torque_cmd, ResolveTorqueLimit(velocity_torque_limit_));
        return motor_.SetTorque(torque_cmd);
    }

    case ControlMode::NativePosition:
        return motor_.SetPosition(target_position_, position_velocity_limit_);

    case ControlMode::EmulatedPosition: {
        if (!position_pid_ready_ || !velocity_pid_ready_) {
            return DEVICE_ERR_UNSUPPORTED;
        }
        float velocity_cmd =
            position_pid_.Update(target_position_, state.position_rad, dt);
        velocity_cmd = ApplyRequiredAbsLimit(
            velocity_cmd, ResolveVelocityLimit(position_velocity_limit_));
        float torque_cmd =
            velocity_pid_.Update(velocity_cmd, state.velocity_rad_s, dt);
        torque_cmd = ApplyRequiredAbsLimit(
            torque_cmd, ResolveTorqueLimit(velocity_torque_limit_));
        return motor_.SetTorque(torque_cmd);
    }

    case ControlMode::NativeMit:
        return motor_.SetMIT(target_position_, target_velocity_, target_mit_kp_,
                             target_mit_kd_, target_mit_torque_ff_);
    }

    return DEVICE_ERR;
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::Update() {
    const int8_t ret = UpdateFeedback();
    if (ret != DEVICE_OK) {
        return ret;
    }
    return UpdateControl();
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

    if (!velocity_pid_ready_) {
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

    if (!position_pid_ready_ || !velocity_pid_ready_) {
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
    if (velocity_pid_ready_) {
        velocity_pid_.Reset();
    }
    if (position_pid_ready_) {
        position_pid_.Reset();
    }
    control_timer_.Reset();
    return DEVICE_OK;
}

template <typename MotorType>
float MotorControllerT<MotorType>::UpdateControlDt() {
    return cntlr::SanitizeDt(control_timer_.Update());
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
template class MotorControllerT<mr::motor::RmM2006Motor>;
template class MotorControllerT<mr::motor::RmM3508Motor>;

} // namespace mr::motor
