/**
 * @file motor_single_limit_calibration.hpp
 * @brief 单边限位归零 + 固定行程标定状态机
 */

#pragma once

#include <stdint.h>

#include "device/motors/motor_packages/limit_selfLearning/common/motor_limit_common.hpp"
#include "device/motors/motor.hpp"

namespace mrobot {

class MotorSingleLimitCalibration {
public:
    enum class Result : uint8_t {
        RUNNING = 0,
        PASS,
        FAIL_NULL,
        FAIL_OFFLINE,
        FAIL_FEEDBACK_INVALID,
        FAIL_SEEK_TIMEOUT,
        FAIL_INVALID_TRAVEL,
    };

    struct Params {
        MotorLimitSeekParams seek;
        float user_travel_rad;
        bool seek_positive_direction;
        bool settle_after_zero_enable;

        static Params Default() {
            Params p{};
            p.seek = MotorLimitSeekParams::Default();
            p.user_travel_rad = 1.0f;
            p.seek_positive_direction = false;
            p.settle_after_zero_enable = true;
            return p;
        }
    };

    struct LearnedRange {
        float min_position_rad;
        float max_position_rad;
        bool valid;
    };

    explicit MotorSingleLimitCalibration(Motor* motor,
                                         const Params& params = Params::Default())
        : motor_(motor),
          params_(params),
          state_(State::IDLE),
          result_(motor ? Result::RUNNING : Result::FAIL_NULL),
          state_enter_ms_(0),
          detector_(),
          learned_(false),
          min_position_rad_(0.0f),
          max_position_rad_(0.0f) {}

    void Start(uint32_t now_ms);
    Result Update(uint32_t now_ms);
    void Cancel();

    Result GetResult() const { return result_; }
    bool IsRunning() const { return result_ == Result::RUNNING; }
    bool Passed() const { return result_ == Result::PASS; }
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
        SEEK_ZERO_LIMIT,
        SETTLE_AFTER_ZERO,
        DONE,
        ERROR,
    };

    void EnterState(State state, uint32_t now_ms);
    void Finish(Result result);
    bool ParamsValid() const;
    bool FeedbackLooksValid() const;
    bool StallDetected(uint32_t now_ms);
    bool Settled(uint32_t now_ms);

    Motor* motor_;
    Params params_;
    State state_;
    Result result_;
    uint32_t state_enter_ms_;
    MotorLimitDetector detector_;
    bool learned_;
    float min_position_rad_;
    float max_position_rad_;
};

} // namespace mrobot
