/**
 * @file motor_self_test.cpp
 * @brief 电机启动自检状态机实现
 */

#include "device/motors/motor_packages/self_test/motor_self_test.hpp"

#include <cmath>

namespace mrobot {

namespace {

static uint32_t ElapsedMs(uint32_t now_ms, uint32_t start_ms) {
    return now_ms - start_ms;
}

static bool IsFinite(float value) {
    return std::isfinite(value);
}

} // namespace

void MotorSelfTest::Start(uint32_t now_ms) {
    stable_elapsed_ms_ = 0;
    probe_detected_motion_ = false;

    if (motor_ == nullptr) {
        Finish(Result::FAIL_NULL);
        return;
    }

    if (motor_->Register() != DEVICE_OK) {
        Finish(Result::FAIL_REGISTER);
        return;
    }

    if (motor_->Enable() != DEVICE_OK) {
        Finish(Result::FAIL_ENABLE);
        return;
    }

    result_ = Result::RUNNING;
    EnterState(State::WAIT_ONLINE, now_ms);
}

MotorSelfTest::Result MotorSelfTest::Update(uint32_t now_ms) {
    if (result_ != Result::RUNNING) {
        return result_;
    }

    if (motor_ == nullptr) {
        Finish(Result::FAIL_NULL);
        return result_;
    }

    if (motor_->Update() != DEVICE_OK) {
        Finish(Result::FAIL_FEEDBACK_INVALID);
        return result_;
    }

    switch (state_) {
    case State::WAIT_ONLINE:
        if (motor_->IsMotorOnline()) {
            EnterState(State::STABLE_CHECK, now_ms);
        } else if (ElapsedMs(now_ms, state_enter_ms_) >= params_.online_timeout_ms) {
            Finish(Result::FAIL_OFFLINE);
        }
        break;

    case State::STABLE_CHECK:
        if (!FeedbackLooksValid()) {
            Finish(Result::FAIL_FEEDBACK_INVALID);
            break;
        }
        if (motor_->GetTemperatureCelsius() > params_.max_temperature_c) {
            Finish(Result::FAIL_OVERTEMP);
            break;
        }
        if (!PassiveChecksPassed()) {
            Finish(Result::FAIL_UNSTABLE);
            break;
        }

        stable_elapsed_ms_ = ElapsedMs(now_ms, state_enter_ms_);
        if (stable_elapsed_ms_ >= params_.stable_check_ms) {
            if (!params_.active_probe_enable) {
                Finish(Result::PASS);
            } else {
                probe_detected_motion_ = false;
                EnterState(State::PROBE_FORWARD, now_ms);
            }
        }
        break;

    case State::PROBE_FORWARD:
        motor_->CurrentControl(params_.probe_current);
        if (std::fabs(motor_->GetVelocityRadPerSec()) >= params_.probe_velocity_threshold) {
            probe_detected_motion_ = true;
        }
        if (ElapsedMs(now_ms, state_enter_ms_) >= params_.probe_duration_ms) {
            motor_->Relax();
            if (!probe_detected_motion_) {
                Finish(Result::FAIL_NO_RESPONSE);
            } else {
                probe_detected_motion_ = false;
                EnterState(State::PROBE_REVERSE, now_ms);
            }
        }
        break;

    case State::PROBE_REVERSE:
        motor_->CurrentControl(-params_.probe_current);
        if (std::fabs(motor_->GetVelocityRadPerSec()) >= params_.probe_velocity_threshold) {
            probe_detected_motion_ = true;
        }
        if (ElapsedMs(now_ms, state_enter_ms_) >= params_.probe_duration_ms) {
            motor_->Relax();
            Finish(probe_detected_motion_ ? Result::PASS : Result::FAIL_NO_RESPONSE);
        }
        break;

    case State::DONE:
    case State::ERROR:
    case State::IDLE:
    default:
        break;
    }

    return result_;
}

void MotorSelfTest::Cancel() {
    if (motor_ != nullptr) {
        motor_->Relax();
    }
    state_ = State::IDLE;
    result_ = Result::FAIL_UNSTABLE;
}

void MotorSelfTest::EnterState(State state, uint32_t now_ms) {
    state_ = state;
    state_enter_ms_ = now_ms;
}

bool MotorSelfTest::FeedbackLooksValid() const {
    return IsFinite(motor_->GetRawAngleRad()) &&
           IsFinite(motor_->GetVelocityRadPerSec()) &&
           IsFinite(motor_->GetTorqueCurrentAmp()) &&
           IsFinite(motor_->GetTemperatureCelsius());
}

bool MotorSelfTest::PassiveChecksPassed() const {
    return std::fabs(motor_->GetVelocityRadPerSec()) <= params_.max_abs_velocity &&
           std::fabs(motor_->GetTorqueCurrentAmp()) <= params_.max_abs_current;
}

void MotorSelfTest::Finish(Result result) {
    if (motor_ != nullptr) {
        motor_->Relax();
    }
    result_ = result;
    state_ = (result == Result::PASS) ? State::DONE : State::ERROR;
}

} // namespace mrobot