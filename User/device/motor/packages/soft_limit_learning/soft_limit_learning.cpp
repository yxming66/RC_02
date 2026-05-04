#include "device/motor/packages/soft_limit_learning/soft_limit_learning.hpp"

#include <math.h>

namespace mr::motor {

namespace {

float ClampNonNegative(float value) {
    return (value > 0.0f) ? value : 0.0f;
}

float ClampFiniteNonNegative(float value, float fallback) {
    if (!isfinite(value) || value < 0.0f) {
        return fallback;
    }
    return value;
}

float AbsFloat(float value) {
    return (value >= 0.0f) ? value : -value;
}

} // namespace

SoftLimitLearning::SoftLimitLearning() = default;

void SoftLimitLearning::Configure(const SoftLimitLearningConfig& config) {
    config_ = config;
    config_.stall_velocity_threshold_rad_s =
        ClampFiniteNonNegative(config_.stall_velocity_threshold_rad_s, 0.03f);
    config_.stall_position_window_rad =
        ClampFiniteNonNegative(config_.stall_position_window_rad, 0.0015f);
    config_.stall_cycles_required =
        (config_.stall_cycles_required > 0u) ? config_.stall_cycles_required : 1u;
    config_.seek_timeout_s = ClampFiniteNonNegative(config_.seek_timeout_s, 8.0f);
}

const SoftLimitLearningConfig& SoftLimitLearning::GetConfig() const {
    return config_;
}

void SoftLimitLearning::Reset() {
    range_ = {};
    observed_state_ = {};
    has_observed_state_ = false;
    ResetSeekTracking();
    state_ = SoftLimitLearningState::Idle;
}

void SoftLimitLearning::ResetLearningState() {
    ResetSeekTracking();
    state_ = ResolveIdleState();
}

void SoftLimitLearning::ClearRange() {
    range_ = {};
    ResetLearningState();
}

void SoftLimitLearning::Observe(const MotorState& state, float dt_s) {
    observed_state_ = state;
    has_observed_state_ = true;

    if (!IsSeeking()) {
        return;
    }

    const float dt = ClampFiniteNonNegative(dt_s, 0.0f);
    elapsed_seek_s_ += dt;
    if (config_.seek_timeout_s > 0.0f && elapsed_seek_s_ >= config_.seek_timeout_s) {
        MarkFailed();
        return;
    }

    if (!state.online) {
        stall_cycles_ = 0u;
        last_seek_position_rad_ = state.position_rad;
        seek_position_initialized_ = true;
        return;
    }

    if (!seek_position_initialized_) {
        last_seek_position_rad_ = state.position_rad;
        seek_position_initialized_ = true;
        return;
    }

    const float position_delta = AbsFloat(state.position_rad - last_seek_position_rad_);
    last_seek_position_rad_ = state.position_rad;

    const bool velocity_small =
        AbsFloat(state.velocity_rad_s) <= config_.stall_velocity_threshold_rad_s;
    const bool position_small =
        position_delta <= config_.stall_position_window_rad;

    if (velocity_small && position_small) {
        if (stall_cycles_ < UINT16_MAX) {
            ++stall_cycles_;
        }
    } else {
        stall_cycles_ = 0u;
    }

    if (stall_cycles_ >= config_.stall_cycles_required) {
        FinishSeekAt(state.position_rad);
    }
}

bool SoftLimitLearning::StartSeekLower(float seek_velocity_rad_s) {
    const float seek_speed = ClampNonNegative(AbsFloat(seek_velocity_rad_s));
    if (seek_speed <= 0.0f) {
        return false;
    }

    ResetSeekTracking();
    seek_velocity_rad_s_ = -seek_speed;
    state_ = SoftLimitLearningState::SeekingLower;
    return true;
}

bool SoftLimitLearning::StartSeekUpper(float seek_velocity_rad_s) {
    const float seek_speed = ClampNonNegative(AbsFloat(seek_velocity_rad_s));
    if (seek_speed <= 0.0f) {
        return false;
    }

    ResetSeekTracking();
    seek_velocity_rad_s_ = seek_speed;
    state_ = SoftLimitLearningState::SeekingUpper;
    return true;
}

void SoftLimitLearning::CancelSeek() {
    ResetLearningState();
}

void SoftLimitLearning::MarkFailed() {
    ResetSeekTracking();
    state_ = SoftLimitLearningState::Failed;
}

bool SoftLimitLearning::CaptureLower(float position_rad) {
    if (!isfinite(position_rad)) {
        return false;
    }
    range_.lower_rad = position_rad;
    range_.lower_valid = true;
    NormalizeRange();
    state_ = ResolveIdleState();
    return true;
}

bool SoftLimitLearning::CaptureUpper(float position_rad) {
    if (!isfinite(position_rad)) {
        return false;
    }
    range_.upper_rad = position_rad;
    range_.upper_valid = true;
    NormalizeRange();
    state_ = ResolveIdleState();
    return true;
}

bool SoftLimitLearning::CaptureLowerFromObserved() {
    if (!has_observed_state_) {
        return false;
    }
    return CaptureLower(observed_state_.position_rad);
}

bool SoftLimitLearning::CaptureUpperFromObserved() {
    if (!has_observed_state_) {
        return false;
    }
    return CaptureUpper(observed_state_.position_rad);
}

bool SoftLimitLearning::SetRange(float lower_rad, float upper_rad) {
    if (!isfinite(lower_rad) || !isfinite(upper_rad)) {
        return false;
    }
    range_.lower_rad = lower_rad;
    range_.upper_rad = upper_rad;
    range_.lower_valid = true;
    range_.upper_valid = true;
    NormalizeRange();
    state_ = ResolveIdleState();
    return true;
}

bool SoftLimitLearning::SetRangeByLowerAndTravel(float lower_rad, float travel_rad) {
    if (!isfinite(lower_rad) || !isfinite(travel_rad) || travel_rad < 0.0f) {
        return false;
    }
    return SetRange(lower_rad, lower_rad + travel_rad);
}

bool SoftLimitLearning::SetRangeByUpperAndTravel(float upper_rad, float travel_rad) {
    if (!isfinite(upper_rad) || !isfinite(travel_rad) || travel_rad < 0.0f) {
        return false;
    }
    return SetRange(upper_rad - travel_rad, upper_rad);
}

bool SoftLimitLearning::HasObservedState() const {
    return has_observed_state_;
}

const MotorState& SoftLimitLearning::GetObservedState() const {
    return observed_state_;
}

bool SoftLimitLearning::HasLower() const {
    return range_.lower_valid;
}

bool SoftLimitLearning::HasUpper() const {
    return range_.upper_valid;
}

bool SoftLimitLearning::HasRange() const {
    return range_.lower_valid && range_.upper_valid;
}

bool SoftLimitLearning::IsSeeking() const {
    return state_ == SoftLimitLearningState::SeekingLower ||
           state_ == SoftLimitLearningState::SeekingUpper;
}

bool SoftLimitLearning::IsReady() const {
    return state_ == SoftLimitLearningState::Ready;
}

const SoftLimitRange& SoftLimitLearning::GetRange() const {
    return range_;
}

SoftLimitLearningState SoftLimitLearning::GetState() const {
    return state_;
}

float SoftLimitLearning::GetSeekVelocity() const {
    return IsSeeking() ? seek_velocity_rad_s_ : 0.0f;
}

void SoftLimitLearning::NormalizeRange() {
    if (!range_.lower_valid || !range_.upper_valid) {
        return;
    }

    if (range_.lower_rad > range_.upper_rad) {
        const float tmp = range_.lower_rad;
        range_.lower_rad = range_.upper_rad;
        range_.upper_rad = tmp;
    }
}

void SoftLimitLearning::FinishSeekAt(float position_rad) {
    if (state_ == SoftLimitLearningState::SeekingLower) {
        range_.lower_rad = position_rad;
        range_.lower_valid = true;
    } else if (state_ == SoftLimitLearningState::SeekingUpper) {
        range_.upper_rad = position_rad;
        range_.upper_valid = true;
    }

    NormalizeRange();
    ResetSeekTracking();
    state_ = ResolveIdleState();
}

void SoftLimitLearning::ResetSeekTracking() {
    seek_velocity_rad_s_ = 0.0f;
    last_seek_position_rad_ = 0.0f;
    seek_position_initialized_ = false;
    stall_cycles_ = 0u;
    elapsed_seek_s_ = 0.0f;
}

SoftLimitLearningState SoftLimitLearning::ResolveIdleState() const {
    return HasRange() ? SoftLimitLearningState::Ready : SoftLimitLearningState::Idle;
}

} // namespace mr::motor
