#pragma once

#include <stdint.h>

#include "component/timer.hpp"
#include "device/motor/core/motor_state.hpp"

namespace mr::motor {

struct SoftLimitRange {
    bool lower_valid = false;
    bool upper_valid = false;
    float lower_rad = 0.0f;
    float upper_rad = 0.0f;
};

struct SoftLimitLearningConfig {
    float stall_velocity_threshold_rad_s = 0.03f;
    float stall_position_window_rad = 0.0015f;
    uint16_t stall_cycles_required = 20u;
    float seek_timeout_s = 8.0f;
    float limit_margin_rad = 0.0f;
    float learned_limit_margin_rad = 0.0f;
    float min_range_rad = 0.0f;
};

enum class SoftLimitLearningState : uint8_t {
    Idle = 0,
    SeekingLower,
    SeekingUpper,
    Ready,
    Failed,
};

class SoftLimitLearning final {
public:
    SoftLimitLearning();

    void Configure(const SoftLimitLearningConfig& config);
    const SoftLimitLearningConfig& GetConfig() const;

    void Reset();
    void ResetLearningState();
    void ClearRange();

    void Observe(const MotorState& state);
    void Observe(const MotorState& state, float dt_s);
    void ResetObserverTimer();

    bool StartSeekLower(float seek_velocity_rad_s);
    bool StartSeekUpper(float seek_velocity_rad_s);
    void CancelSeek();
    void MarkFailed();

    bool CaptureLower(float position_rad);
    bool CaptureUpper(float position_rad);
    bool CaptureLowerFromObserved();
    bool CaptureUpperFromObserved();
    bool SetRange(float lower_rad, float upper_rad);
    bool SetRangeByLowerAndTravel(float lower_rad, float travel_rad);
    bool SetRangeByUpperAndTravel(float upper_rad, float travel_rad);

    bool HasObservedState() const;
    const MotorState& GetObservedState() const;

    bool HasLower() const;
    bool HasUpper() const;
    bool HasRange() const;
    bool IsSeeking() const;
    bool IsReady() const;
    bool IsFailed() const;

    float GetTravelRad() const;
    float GetCenterRad() const;
    float ClampPosition(float position_rad) const;
    float ClampPosition(float position_rad, float margin_rad) const;
    float LimitVelocity(float position_rad, float velocity_rad_s) const;
    float LimitVelocity(float position_rad, float velocity_rad_s, float margin_rad) const;
    bool IsPositionWithinRange(float position_rad) const;
    bool IsPositionWithinRange(float position_rad, float margin_rad) const;
    bool IsAtLowerLimit(float position_rad) const;
    bool IsAtLowerLimit(float position_rad, float margin_rad) const;
    bool IsAtUpperLimit(float position_rad) const;
    bool IsAtUpperLimit(float position_rad, float margin_rad) const;

    const SoftLimitRange& GetRange() const;
    SoftLimitLearningState GetState() const;
    float GetSeekVelocity() const;
    uint16_t GetStallCycles() const;
    float GetElapsedSeekS() const;

private:
    bool CommitRange(SoftLimitRange range);
    void FinishSeekAt(float position_rad);
    void ResetSeekTracking();
    SoftLimitLearningState ResolveIdleState() const;
    float ResolveMargin(float margin_rad) const;

    SoftLimitLearningConfig config_ {};
    SoftLimitRange range_ {};
    SoftLimitLearningState state_ = SoftLimitLearningState::Idle;
    mr::comp::timer observe_timer_ {};
    MotorState observed_state_ {};
    bool has_observed_state_ = false;
    float seek_velocity_rad_s_ = 0.0f;
    float last_seek_position_rad_ = 0.0f;
    bool seek_position_initialized_ = false;
    uint16_t stall_cycles_ = 0u;
    float elapsed_seek_s_ = 0.0f;
};

} // namespace mr::motor
