/**
 * @file motor_limit_common.hpp
 * @brief 限位学习公共参数与检测辅助
 */

#pragma once

#include <stdint.h>

#include <cmath>

#include "device/motors/motor.hpp"

namespace mrobot {

struct MotorLimitSeekParams {
    float seek_velocity_rad_per_sec;
    float stall_velocity_threshold;
    float stall_current_threshold;
    uint32_t stall_confirm_ms;
    uint32_t seek_timeout_ms;
    float settle_velocity_threshold;
    uint32_t settle_time_ms;

    static MotorLimitSeekParams Default() {
        MotorLimitSeekParams p{};
        p.seek_velocity_rad_per_sec = 0.8f;
        p.stall_velocity_threshold = 0.08f;
        p.stall_current_threshold = 0.6f;
        p.stall_confirm_ms = 120;
        p.seek_timeout_ms = 4000;
        p.settle_velocity_threshold = 0.15f;
        p.settle_time_ms = 80;
        return p;
    }
};

struct MotorLimitFeedback {
    float position_rad;
    float velocity_rad_per_sec;
    float torque_current_amp;
};

inline uint32_t MotorLimitElapsedMs(uint32_t now_ms, uint32_t start_ms) {
    return now_ms - start_ms;
}

inline bool MotorLimitIsFinite(float value) {
    return std::isfinite(value);
}

inline bool MotorLimitFeedbackLooksValid(const Motor* motor) {
    return motor != nullptr &&
           MotorLimitIsFinite(motor->GetPositionRad()) &&
           MotorLimitIsFinite(motor->GetVelocityRadPerSec()) &&
           MotorLimitIsFinite(motor->GetTorqueCurrentAmp());
}

inline bool MotorLimitSeekParamsValid(const MotorLimitSeekParams& params) {
    return MotorLimitIsFinite(params.seek_velocity_rad_per_sec) &&
           params.seek_velocity_rad_per_sec > 0.0f &&
           MotorLimitIsFinite(params.stall_velocity_threshold) &&
           params.stall_velocity_threshold >= 0.0f &&
           MotorLimitIsFinite(params.stall_current_threshold) &&
           params.stall_current_threshold >= 0.0f &&
           params.stall_confirm_ms > 0U &&
           params.seek_timeout_ms > 0U &&
           MotorLimitIsFinite(params.settle_velocity_threshold) &&
           params.settle_velocity_threshold >= 0.0f &&
           params.settle_time_ms > 0U;
}

class MotorLimitDetector {
public:
    MotorLimitDetector() : stall_start_ms_(0), settle_start_ms_(0) {}

    void Reset() {
        stall_start_ms_ = 0;
        settle_start_ms_ = 0;
    }

    void ResetStall() { stall_start_ms_ = 0; }
    void ResetSettle() { settle_start_ms_ = 0; }

    bool StallDetected(const Motor* motor, const MotorLimitSeekParams& params, uint32_t now_ms) {
        const bool low_velocity = std::fabs(motor->GetVelocityRadPerSec()) <= params.stall_velocity_threshold;
        const bool high_current = std::fabs(motor->GetTorqueCurrentAmp()) >= params.stall_current_threshold;

        if (low_velocity && high_current) {
            if (stall_start_ms_ == 0U) {
                stall_start_ms_ = now_ms;
            }
            return MotorLimitElapsedMs(now_ms, stall_start_ms_) >= params.stall_confirm_ms;
        }

        stall_start_ms_ = 0U;
        return false;
    }

    bool Settled(const Motor* motor, const MotorLimitSeekParams& params, uint32_t now_ms) {
        if (std::fabs(motor->GetVelocityRadPerSec()) <= params.settle_velocity_threshold) {
            if (settle_start_ms_ == 0U) {
                settle_start_ms_ = now_ms;
            }
            return MotorLimitElapsedMs(now_ms, settle_start_ms_) >= params.settle_time_ms;
        }

        settle_start_ms_ = 0U;
        return false;
    }

private:
    uint32_t stall_start_ms_;
    uint32_t settle_start_ms_;
};

} // namespace mrobot
