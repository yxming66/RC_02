#include "device/motor/packages/soft_limit_learning/soft_limit_learning.hpp"

#include "component/math/range_limits.hpp"
#include "component/math/scalar.hpp"

namespace mr::motor {

namespace {

namespace scalar = mr::component::math;

constexpr float kDefaultStallVelocityThresholdRadS = 0.03f;
constexpr float kDefaultStallPositionWindowRad = 0.0015f;
constexpr float kDefaultSeekTimeoutS = 8.0f;

float ClampNonNegative(float value) {
    return (scalar::is_finite_scalar(value) && value > 0.0f) ? value : 0.0f;
}

float ClampFiniteNonNegative(float value, float fallback) {
    if (!scalar::is_finite_scalar(value) || value < 0.0f) {
        return fallback;
    }
    return value;
}

float AbsFloat(float value) {
    return scalar::abs_scalar(value);
}

bool IsFinitePosition(float position_rad) {
    return scalar::is_finite_scalar(position_rad);
}

bool IsFiniteMotorState(const MotorState& state) {
    return scalar::is_finite_scalar(state.position_rad) &&
           scalar::is_finite_scalar(state.velocity_rad_s);
}

bool RangeBoundValuesFinite(const SoftLimitRange& range) {
    if (range.lower_valid && !IsFinitePosition(range.lower_rad)) {
        return false;
    }
    if (range.upper_valid && !IsFinitePosition(range.upper_rad)) {
        return false;
    }
    return true;
}

bool NormalizeRangeCandidate(SoftLimitRange& range, float min_range_rad) {
    if (!RangeBoundValuesFinite(range)) {
        return false;
    }

    if (range.lower_valid && range.upper_valid) {
        if (range.lower_rad > range.upper_rad) {
            const float tmp = range.lower_rad;
            range.lower_rad = range.upper_rad;
            range.upper_rad = tmp;
        }

        const float min_range = ClampNonNegative(min_range_rad);
        if ((range.upper_rad - range.lower_rad) < min_range) {
            return false;
        }
    }
    return true;
}

mr::component::math::RangeLimits<1> BuildRangeLimits(float lower_rad,
                                                     float upper_rad) {
    mr::component::math::RangeLimits<1> limits;
    limits.set(0u, lower_rad, upper_rad);
    return limits;
}

mr::comp::timer::Config BuildObserveTimerConfig(const SoftLimitLearningConfig& config) {
    mr::comp::timer::Config timer_config{};
    if (config.seek_timeout_s > 0.0f) {
        timer_config.max_dt_us = mr::comp::timer::SecondsToUs(config.seek_timeout_s);
    }
    return timer_config;
}

} // namespace

SoftLimitLearning::SoftLimitLearning() {
    Configure(config_);
}

void SoftLimitLearning::Configure(const SoftLimitLearningConfig& config) {
    config_ = config;
    config_.stall_velocity_threshold_rad_s =
        ClampFiniteNonNegative(config_.stall_velocity_threshold_rad_s,
                               kDefaultStallVelocityThresholdRadS);
    config_.stall_position_window_rad =
        ClampFiniteNonNegative(config_.stall_position_window_rad,
                               kDefaultStallPositionWindowRad);
    config_.stall_cycles_required =
        (config_.stall_cycles_required > 0u) ? config_.stall_cycles_required : 1u;
    config_.seek_timeout_s =
        ClampFiniteNonNegative(config_.seek_timeout_s, kDefaultSeekTimeoutS);
    config_.limit_margin_rad =
        ClampFiniteNonNegative(config_.limit_margin_rad, 0.0f);
    config_.learned_limit_margin_rad =
        ClampFiniteNonNegative(config_.learned_limit_margin_rad,
                               config_.limit_margin_rad);
    config_.min_range_rad =
        ClampFiniteNonNegative(config_.min_range_rad, 0.0f);
    observe_timer_.Configure(BuildObserveTimerConfig(config_));
    if (!NormalizeRangeCandidate(range_, config_.min_range_rad)) {
        range_ = {};
        if (!IsSeeking()) {
            state_ = SoftLimitLearningState::Idle;
        }
    } else if (!IsSeeking()) {
        state_ = ResolveIdleState();
    }
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

void SoftLimitLearning::Observe(const MotorState& state) {
    Observe(state, observe_timer_.Update());
}

void SoftLimitLearning::Observe(const MotorState& state, float dt_s) {
    observed_state_ = state;
    has_observed_state_ = IsFiniteMotorState(state);

    if (!IsSeeking()) {
        return;
    }

    if (!state.online || !IsFiniteMotorState(state)) {
        stall_cycles_ = 0u;
        seek_position_initialized_ = false;
        return;
    }

    const float dt = ClampFiniteNonNegative(dt_s, 0.0f);
    elapsed_seek_s_ += dt;
    if (config_.seek_timeout_s > 0.0f && elapsed_seek_s_ >= config_.seek_timeout_s) {
        MarkFailed();
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

void SoftLimitLearning::ResetObserverTimer() {
    observe_timer_.Reset();
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
    if (!IsFinitePosition(position_rad)) {
        return false;
    }
    SoftLimitRange candidate = range_;
    candidate.lower_rad = position_rad;
    candidate.lower_valid = true;
    return CommitRange(candidate);
}

bool SoftLimitLearning::CaptureUpper(float position_rad) {
    if (!IsFinitePosition(position_rad)) {
        return false;
    }
    SoftLimitRange candidate = range_;
    candidate.upper_rad = position_rad;
    candidate.upper_valid = true;
    return CommitRange(candidate);
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
    if (!IsFinitePosition(lower_rad) || !IsFinitePosition(upper_rad)) {
        return false;
    }
    SoftLimitRange candidate{};
    candidate.lower_rad = lower_rad;
    candidate.upper_rad = upper_rad;
    candidate.lower_valid = true;
    candidate.upper_valid = true;
    return CommitRange(candidate);
}

bool SoftLimitLearning::SetRangeByLowerAndTravel(float lower_rad, float travel_rad) {
    if (!IsFinitePosition(lower_rad) || !scalar::is_finite_scalar(travel_rad) ||
        travel_rad < 0.0f) {
        return false;
    }
    const float margin = ClampNonNegative(config_.learned_limit_margin_rad);
    if (travel_rad < 2.0f * margin) {
        return false;
    }
    return SetRange(lower_rad + margin, lower_rad + travel_rad - margin);
}

bool SoftLimitLearning::SetRangeByUpperAndTravel(float upper_rad, float travel_rad) {
    if (!IsFinitePosition(upper_rad) || !scalar::is_finite_scalar(travel_rad) ||
        travel_rad < 0.0f) {
        return false;
    }
    const float margin = ClampNonNegative(config_.learned_limit_margin_rad);
    if (travel_rad < 2.0f * margin) {
        return false;
    }
    return SetRange(upper_rad - travel_rad + margin, upper_rad - margin);
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

bool SoftLimitLearning::IsFailed() const {
    return state_ == SoftLimitLearningState::Failed;
}

float SoftLimitLearning::GetTravelRad() const {
    return HasRange() ? (range_.upper_rad - range_.lower_rad) : 0.0f;
}

float SoftLimitLearning::GetCenterRad() const {
    if (HasRange()) {
        return 0.5f * (range_.lower_rad + range_.upper_rad);
    }
    if (range_.lower_valid) {
        return range_.lower_rad;
    }
    if (range_.upper_valid) {
        return range_.upper_rad;
    }
    return has_observed_state_ ? observed_state_.position_rad : 0.0f;
}

float SoftLimitLearning::ClampPosition(float position_rad) const {
    return ClampPosition(position_rad, config_.limit_margin_rad);
}

float SoftLimitLearning::ClampPosition(float position_rad, float margin_rad) const {
    float value = IsFinitePosition(position_rad) ? position_rad : GetCenterRad();
    const float margin = ResolveMargin(margin_rad);

    if (range_.lower_valid && range_.upper_valid) {
        float lower = range_.lower_rad + margin;
        float upper = range_.upper_rad - margin;
        if (lower > upper) {
            const float center = 0.5f * (range_.lower_rad + range_.upper_rad);
            lower = center;
            upper = center;
        }
        const float in[1] = {value};
        float out[1] = {value};
        const auto limits = BuildRangeLimits(lower, upper);
        scalar::apply_range_limits(limits, in, out);
        return out[0];
    }

    if (range_.lower_valid && value < range_.lower_rad + margin) {
        value = range_.lower_rad + margin;
    }
    if (range_.upper_valid && value > range_.upper_rad - margin) {
        value = range_.upper_rad - margin;
    }
    return value;
}

float SoftLimitLearning::LimitVelocity(float position_rad,
                                       float velocity_rad_s) const {
    return LimitVelocity(position_rad, velocity_rad_s, config_.limit_margin_rad);
}

float SoftLimitLearning::LimitVelocity(float position_rad,
                                       float velocity_rad_s,
                                       float margin_rad) const {
    if (!IsFinitePosition(position_rad) ||
        !scalar::is_finite_scalar(velocity_rad_s)) {
        return 0.0f;
    }

    const float margin = ResolveMargin(margin_rad);
    if (range_.lower_valid &&
        position_rad <= range_.lower_rad + margin &&
        velocity_rad_s < 0.0f) {
        return 0.0f;
    }
    if (range_.upper_valid &&
        position_rad >= range_.upper_rad - margin &&
        velocity_rad_s > 0.0f) {
        return 0.0f;
    }
    return velocity_rad_s;
}

bool SoftLimitLearning::IsPositionWithinRange(float position_rad) const {
    return IsPositionWithinRange(position_rad, config_.limit_margin_rad);
}

bool SoftLimitLearning::IsPositionWithinRange(float position_rad,
                                              float margin_rad) const {
    if (!IsFinitePosition(position_rad)) {
        return false;
    }
    const float margin = ResolveMargin(margin_rad);
    if (range_.lower_valid && position_rad < range_.lower_rad + margin) {
        return false;
    }
    if (range_.upper_valid && position_rad > range_.upper_rad - margin) {
        return false;
    }
    return true;
}

bool SoftLimitLearning::IsAtLowerLimit(float position_rad) const {
    return IsAtLowerLimit(position_rad, config_.limit_margin_rad);
}

bool SoftLimitLearning::IsAtLowerLimit(float position_rad,
                                       float margin_rad) const {
    return range_.lower_valid && IsFinitePosition(position_rad) &&
           position_rad <= range_.lower_rad + ResolveMargin(margin_rad);
}

bool SoftLimitLearning::IsAtUpperLimit(float position_rad) const {
    return IsAtUpperLimit(position_rad, config_.limit_margin_rad);
}

bool SoftLimitLearning::IsAtUpperLimit(float position_rad,
                                       float margin_rad) const {
    return range_.upper_valid && IsFinitePosition(position_rad) &&
           position_rad >= range_.upper_rad - ResolveMargin(margin_rad);
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

uint16_t SoftLimitLearning::GetStallCycles() const {
    return stall_cycles_;
}

float SoftLimitLearning::GetElapsedSeekS() const {
    return elapsed_seek_s_;
}

bool SoftLimitLearning::CommitRange(SoftLimitRange range) {
    if (!NormalizeRangeCandidate(range, config_.min_range_rad)) {
        return false;
    }
    if (IsSeeking()) {
        ResetSeekTracking();
    }
    range_ = range;
    state_ = ResolveIdleState();
    return true;
}

void SoftLimitLearning::FinishSeekAt(float position_rad) {
    SoftLimitRange candidate = range_;
    if (state_ == SoftLimitLearningState::SeekingLower) {
        candidate.lower_rad = position_rad;
        candidate.lower_valid = true;
    } else if (state_ == SoftLimitLearningState::SeekingUpper) {
        candidate.upper_rad = position_rad;
        candidate.upper_valid = true;
    }

    ResetSeekTracking();
    if (!CommitRange(candidate)) {
        state_ = SoftLimitLearningState::Failed;
    }
}

void SoftLimitLearning::ResetSeekTracking() {
    seek_velocity_rad_s_ = 0.0f;
    last_seek_position_rad_ = 0.0f;
    seek_position_initialized_ = false;
    stall_cycles_ = 0u;
    elapsed_seek_s_ = 0.0f;
    ResetObserverTimer();
}

SoftLimitLearningState SoftLimitLearning::ResolveIdleState() const {
    return HasRange() ? SoftLimitLearningState::Ready : SoftLimitLearningState::Idle;
}

float SoftLimitLearning::ResolveMargin(float margin_rad) const {
    return ClampFiniteNonNegative(margin_rad, config_.limit_margin_rad);
}

} // namespace mr::motor
