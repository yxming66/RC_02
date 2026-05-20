#include "device/motor/packages/controller/motor_controller.hpp"

namespace mr::motor {

namespace {

namespace cntlr = mr::comp::cntlr;
namespace scalar = mr::component::math;

constexpr float kControllerMaxDtS = 0.05f;
constexpr float kTwoPi = 6.28318530717958647692f;

float ClampLimit(float value) {
    return (scalar::is_finite_scalar(value) && value > 0.0f) ? value : 0.0f;
}

float ResolvePositiveRatio(float ratio) {
    return (ratio > 0.0f) ? ratio : 1.0f;
}

float Maxf(float lhs, float rhs) {
    return (lhs > rhs) ? lhs : rhs;
}

float ResolveControlFreqHz(float sample_freq) {
    return (scalar::is_finite_scalar(sample_freq) && sample_freq > 0.0f)
               ? sample_freq
               : cntlr::kDefaultSampleFreqHz;
}

float ResolveDefaultDtS(float sample_freq) {
    return 1.0f / ResolveControlFreqHz(sample_freq);
}

mr::comp::timer::Config BuildControlTimerConfig(float sample_freq) {
    const float default_dt_s = ResolveDefaultDtS(sample_freq);
    mr::comp::timer::Config timer_config{};
    timer_config.first_dt_us = mr::comp::timer::SecondsToUs(default_dt_s);
    timer_config.min_dt_us = mr::comp::timer::SecondsToUs(default_dt_s);
    timer_config.max_dt_us =
        mr::comp::timer::SecondsToUs(Maxf(default_dt_s, kControllerMaxDtS));
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

cntlr::pid::Config BuildPidConfig(const KPID_Params_t* params,
                                  float sample_freq) {
    cntlr::pid::Config pid_config{};
    pid_config.control_freq_hz = ResolveControlFreqHz(sample_freq);
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

float ResolveCutoffHz(float cutoff_hz) {
    return (scalar::is_finite_scalar(cutoff_hz) && cutoff_hz > 0.0f)
               ? cutoff_hz
               : 0.0f;
}

} // namespace

template <typename MotorType>
MotorControllerT<MotorType>::MotorControllerT(MotorType& motor,
                                              const MotorControllerConfig& config)
    : motor_(motor),
      config_(config),
      velocity_pid_(BuildPidConfig(config.velocity_pid, config.sample_freq)),
      position_pid_(BuildPidConfig(config.position_pid, config.sample_freq)),
      control_timer_(mr::comp::timer::Build(BuildControlTimerConfig(config.sample_freq))),
      control_dt_fallback_s_(ResolveDefaultDtS(config.sample_freq)),
      velocity_pid_ready_(PidParamsUsable(config.velocity_pid)),
      position_pid_ready_(PidParamsUsable(config.position_pid)),
    feedback_lowpass_cutoff_hz_(ResolveCutoffHz(config.feedback_lowpass_cutoff_hz)),
    output_lowpass_cutoff_hz_(ResolveCutoffHz(config.output_lowpass_cutoff_hz)),
    feedback_filter_initialized_(false),
    output_filter_initialized_(false),
    filtered_position_(0.0f),
    filtered_velocity_(0.0f),
    filtered_output_torque_(0.0f),
      mode_(ControlMode::Torque),
      target_torque_(0.0f),
      target_velocity_(0.0f),
      target_position_(0.0f),
      position_velocity_limit_(config.position_to_velocity_limit),
      velocity_torque_limit_(config.velocity_to_torque_limit),
      target_mit_kp_(0.0f),
      target_mit_kd_(0.0f),
    target_mit_torque_ff_(0.0f),
    last_feedback_position_(0.0f),
    last_feedback_velocity_(0.0f),
    last_output_torque_(0.0f) {
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
    ResetFiltersToState();
    return motor_.Enable();
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::Disable() {
    mode_ = ControlMode::Torque;
    target_torque_ = 0.0f;
    ResetControllers();
    ResetFiltersToState();
    return motor_.Disable();
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::Relax() {
    mode_ = ControlMode::Torque;
    target_torque_ = 0.0f;
    ResetControllers();
    ResetFiltersToState();
    return motor_.Relax();
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::Update() {
    const float dt = UpdateControlDt();
    int8_t ret = motor_.Update();
    if (ret != DEVICE_OK) {
        return ret;
    }

    const MotorState state = FilterFeedback(motor_.GetState(), dt);

    switch (mode_) {
    case ControlMode::Torque:
        return motor_.SetTorque(FilterOutputTorque(target_torque_, dt));

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
        return motor_.SetTorque(FilterOutputTorque(torque_cmd, dt));
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
        return motor_.SetTorque(FilterOutputTorque(torque_cmd, dt));
    }

    case ControlMode::EmulatedPositionTorque: {
        if (!position_pid_ready_) {
            return DEVICE_ERR_UNSUPPORTED;
        }
        float torque_cmd =
            position_pid_.Update(target_position_, state.position_rad, dt);
        torque_cmd = ApplyRequiredAbsLimit(
            torque_cmd, ResolveTorqueLimit(velocity_torque_limit_));
        return motor_.SetTorque(FilterOutputTorque(torque_cmd, dt));
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
int8_t MotorControllerT<MotorType>::SetPositionTorque(float position,
                                                      float torque_limit) {
    target_position_ = position;
    target_velocity_ = 0.0f;
    position_velocity_limit_ = config_.position_to_velocity_limit;
    velocity_torque_limit_ =
        (torque_limit > 0.0f) ? torque_limit : config_.velocity_to_torque_limit;

    if (!position_pid_ready_) {
        return DEVICE_ERR_UNSUPPORTED;
    }

    if (mode_ != ControlMode::EmulatedPositionTorque) {
        ResetControllers();
    }
    mode_ = ControlMode::EmulatedPositionTorque;
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
const MotorType& MotorControllerT<MotorType>::GetMotor() const {
    return motor_;
}

template <typename MotorType>
float MotorControllerT<MotorType>::GetLastFeedbackPosition() const {
    return last_feedback_position_;
}

template <typename MotorType>
float MotorControllerT<MotorType>::GetLastFeedbackVelocity() const {
    return last_feedback_velocity_;
}

template <typename MotorType>
float MotorControllerT<MotorType>::GetLastOutputTorque() const {
    return last_output_torque_;
}

template <typename MotorType>
float MotorControllerT<MotorType>::CalculatePositionOutput(float target,
                                                           float feedback,
                                                           float dt_s) {
    return position_pid_ready_ ? position_pid_.Update(target, feedback, dt_s)
                               : 0.0f;
}

template <typename MotorType>
float MotorControllerT<MotorType>::CalculateVelocityOutput(float target,
                                                           float feedback,
                                                           float dt_s) {
    return velocity_pid_ready_ ? velocity_pid_.Update(target, feedback, dt_s)
                               : 0.0f;
}

template <typename MotorType>
float MotorControllerT<MotorType>::LowpassAlpha(float cutoff_hz,
                                                float dt_s) const {
    const float cutoff = ResolveCutoffHz(cutoff_hz);
    if (cutoff <= 0.0f || dt_s <= 0.0f) {
        return 1.0f;
    }
    const float rc = 1.0f / (kTwoPi * cutoff);
    return dt_s / (rc + dt_s);
}

template <typename MotorType>
void MotorControllerT<MotorType>::ResetFiltersToState() {
    const MotorState state = motor_.GetState();
    filtered_position_ = state.position_rad;
    filtered_velocity_ = state.velocity_rad_s;
    filtered_output_torque_ = 0.0f;
    last_feedback_position_ = filtered_position_;
    last_feedback_velocity_ = filtered_velocity_;
    last_output_torque_ = filtered_output_torque_;
    feedback_filter_initialized_ = true;
    output_filter_initialized_ = false;
}

template <typename MotorType>
MotorState MotorControllerT<MotorType>::FilterFeedback(const MotorState& state,
                                                       float dt_s) {
    if (!feedback_filter_initialized_ || feedback_lowpass_cutoff_hz_ <= 0.0f) {
        filtered_position_ = state.position_rad;
        filtered_velocity_ = state.velocity_rad_s;
        feedback_filter_initialized_ = true;
    } else {
        const float alpha = LowpassAlpha(feedback_lowpass_cutoff_hz_, dt_s);
        filtered_position_ += alpha * (state.position_rad - filtered_position_);
        filtered_velocity_ += alpha * (state.velocity_rad_s - filtered_velocity_);
    }

    last_feedback_position_ = filtered_position_;
    last_feedback_velocity_ = filtered_velocity_;

    MotorState filtered_state = state;
    filtered_state.position_rad = filtered_position_;
    filtered_state.velocity_rad_s = filtered_velocity_;
    return filtered_state;
}

template <typename MotorType>
float MotorControllerT<MotorType>::FilterOutputTorque(float torque_nm,
                                                      float dt_s) {
    if (!output_filter_initialized_ || output_lowpass_cutoff_hz_ <= 0.0f) {
        filtered_output_torque_ = torque_nm;
        output_filter_initialized_ = true;
    } else {
        const float alpha = LowpassAlpha(output_lowpass_cutoff_hz_, dt_s);
        filtered_output_torque_ += alpha * (torque_nm - filtered_output_torque_);
    }
    last_output_torque_ = filtered_output_torque_;
    return filtered_output_torque_;
}

template <typename MotorType>
int8_t MotorControllerT<MotorType>::ResetControllers() {
    if (velocity_pid_ready_) {
        velocity_pid_.Reset();
    }
    if (position_pid_ready_) {
        position_pid_.Reset();
    }
    control_timer_.Configure(BuildControlTimerConfig(config_.sample_freq));
    return DEVICE_OK;
}

template <typename MotorType>
float MotorControllerT<MotorType>::UpdateControlDt() {
    return cntlr::SanitizeDt(control_timer_.Update(), control_dt_fallback_s_);
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
