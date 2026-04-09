/**
 * @file motor_dual_limit_calibration.hpp
 * @brief 双边限位学习标定状态机
 */

#pragma once

#include <stdint.h>

#include "device/motors/motor_packages/limit_selfLearning/common/motor_limit_common.hpp"
#include "device/motors/motor.hpp"

namespace mrobot {

class MotorDualLimitCalibration {
public:
    enum class Result : uint8_t {
        RUNNING = 0,
        PASS,
        FAIL_NULL,
        FAIL_OFFLINE,
        FAIL_FEEDBACK_INVALID,
        FAIL_SEEK_TIMEOUT,
        FAIL_INVALID_RANGE,
    };

    struct Params {
        MotorLimitSeekParams seek;
        float min_valid_range_rad;
        bool return_to_center_enable;
        float return_position_tolerance_rad;
        float return_velocity_limit_rad_per_sec;
        uint32_t return_timeout_ms;

        static Params Default() {
            Params p{};
            p.seek = MotorLimitSeekParams::Default();
            p.min_valid_range_rad = 0.3f;
            p.return_to_center_enable = true;
            p.return_position_tolerance_rad = 0.05f;
            p.return_velocity_limit_rad_per_sec = 0.6f;
            p.return_timeout_ms = 3000;
            return p;
        }
    };

    struct LearnedRange {
        float min_position_rad;
        float max_position_rad;
        bool valid;
    };

    explicit MotorDualLimitCalibration(Motor* motor,
                                       const Params& params = Params::Default())
        : motor_(motor),
          params_(params),
          state_(State::IDLE),
          result_(motor ? Result::RUNNING : Result::FAIL_NULL),
          state_enter_ms_(0),
          detector_(),
          min_position_rad_(0.0f),
          max_position_rad_(0.0f),
          learned_(false) {}

    void Start(uint32_t now_ms);
    Result Update(uint32_t now_ms);
    void Cancel();

    Result GetResult() const { return result_; }
    bool IsRunning() const { return result_ == Result::RUNNING; }
    bool Passed() const { return result_ == Result::PASS; }
    float GetCenterPositionRad() const { return 0.5f * (min_position_rad_ + max_position_rad_); }
    LearnedRange GetLearnedRange() const {
        LearnedRange range{};
        range.min_position_rad = min_position_rad_;
        range.max_position_rad = max_position_rad_;
        range.valid = learned_;
        return range;
    }

private:
    enum class State : uint8_t {
        IDLE = 0,
        SEEK_MIN,
        SETTLE_AFTER_MIN,
        SEEK_MAX,
        RETURN_TO_CENTER,
        DONE,
        ERROR,
    };

    void EnterState(State state, uint32_t now_ms);
    void Finish(Result result);
    bool ParamsValid() const;
    bool FeedbackLooksValid() const;
    bool StallDetected(uint32_t now_ms);
    bool Settled(uint32_t now_ms);
    bool ReachedCenter() const;

    Motor* motor_;
    Params params_;
    State state_;
    Result result_;
    uint32_t state_enter_ms_;
    MotorLimitDetector detector_;
    float min_position_rad_;
    float max_position_rad_;
    bool learned_;
};

} // namespace mrobot
