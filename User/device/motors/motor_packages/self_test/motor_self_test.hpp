/**
 * @file motor_self_test.hpp
 * @brief 电机启动自检状态机
 */

#pragma once

#include <stdint.h>

#include "device/motors/motor.hpp"

namespace mrobot {

class MotorSelfTest {
public:
    enum class Result : uint8_t {
        RUNNING = 0,
        PASS,
        FAIL_NULL,
        FAIL_REGISTER,
        FAIL_ENABLE,
        FAIL_OFFLINE,
        FAIL_FEEDBACK_INVALID,
        FAIL_OVERTEMP,
        FAIL_UNSTABLE,
        FAIL_NO_RESPONSE,
    };

    struct Params {
        uint32_t online_timeout_ms;
        uint32_t stable_check_ms;
        float max_abs_velocity;
        float max_abs_current;
        float max_temperature_c;
        bool active_probe_enable;
        float probe_current;
        uint32_t probe_duration_ms;
        float probe_velocity_threshold;

        static Params Default() {
            Params p{};
            p.online_timeout_ms = 300;
            p.stable_check_ms = 200;
            p.max_abs_velocity = 5.0f;
            p.max_abs_current = 3.0f;
            p.max_temperature_c = 80.0f;
            p.active_probe_enable = false;
            p.probe_current = 0.8f;
            p.probe_duration_ms = 120;
            p.probe_velocity_threshold = 0.3f;
            return p;
        }
    };

    explicit MotorSelfTest(Motor* motor,
                           const Params& params = Params::Default())
        : motor_(motor),
          params_(params),
          state_(State::IDLE),
          result_(motor ? Result::RUNNING : Result::FAIL_NULL),
          state_enter_ms_(0),
          stable_elapsed_ms_(0),
          probe_detected_motion_(false) {}

    void Start(uint32_t now_ms);
    Result Update(uint32_t now_ms);
    void Cancel();

    Result GetResult() const { return result_; }
    bool IsRunning() const { return result_ == Result::RUNNING; }
    bool Passed() const { return result_ == Result::PASS; }

private:
    enum class State : uint8_t {
        IDLE = 0,
        WAIT_ONLINE,
        STABLE_CHECK,
        PROBE_FORWARD,
        PROBE_REVERSE,
        DONE,
        ERROR,
    };

    void EnterState(State state, uint32_t now_ms);
    bool FeedbackLooksValid() const;
    bool PassiveChecksPassed() const;
    void Finish(Result result);

    Motor* motor_;
    Params params_;
    State state_;
    Result result_;
    uint32_t state_enter_ms_;
    uint32_t stable_elapsed_ms_;
    bool probe_detected_motion_;
};

} // namespace mrobot