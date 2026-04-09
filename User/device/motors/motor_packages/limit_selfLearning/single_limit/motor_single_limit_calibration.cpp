/**
 * @file motor_single_limit_calibration.cpp
 * @brief 单边限位归零 + 固定行程标定状态机实现
 */

#include "device/motors/motor_packages/limit_selfLearning/single_limit/motor_single_limit_calibration.hpp"

#include <cmath>

namespace mrobot {

void MotorSingleLimitCalibration::Start(uint32_t now_ms) {
    learned_ = false;
    min_position_rad_ = 0.0f;
    max_position_rad_ = 0.0f;
    detector_.Reset();

    if (motor_ == nullptr) {
        Finish(Result::FAIL_NULL);
        return;
    }

    if (!ParamsValid()) {
        Finish(Result::FAIL_INVALID_TRAVEL);
        return;
    }

    result_ = Result::RUNNING;
    EnterState(State::SEEK_ZERO_LIMIT, now_ms);
}

MotorSingleLimitCalibration::Result MotorSingleLimitCalibration::Update(uint32_t now_ms) {
    if (result_ != Result::RUNNING) {
        return result_;
    }

    if (motor_ == nullptr) {
        Finish(Result::FAIL_NULL);
        return result_;
    }

    if (motor_->Update() != DEVICE_OK || !motor_->IsMotorOnline()) {
        Finish(Result::FAIL_OFFLINE);
        return result_;
    }

    if (!FeedbackLooksValid()) {
        Finish(Result::FAIL_FEEDBACK_INVALID);
        return result_;
    }

    switch (state_) {
    case State::SEEK_ZERO_LIMIT: {
        const float velocity_cmd = params_.seek_positive_direction ?
                                       params_.seek.seek_velocity_rad_per_sec :
                                       -params_.seek.seek_velocity_rad_per_sec;
        motor_->VelocityControl(velocity_cmd);
        if (StallDetected(now_ms)) {
            motor_->Relax();
            motor_->SetZeroPoint();
            if (params_.seek_positive_direction) {
                min_position_rad_ = -params_.user_travel_rad;
                max_position_rad_ = 0.0f;
            } else {
                min_position_rad_ = 0.0f;
                max_position_rad_ = params_.user_travel_rad;
            }
            learned_ = true;
            if (params_.settle_after_zero_enable) {
                EnterState(State::SETTLE_AFTER_ZERO, now_ms);
            } else {
                Finish(Result::PASS);
            }
        } else if (MotorLimitElapsedMs(now_ms, state_enter_ms_) >= params_.seek.seek_timeout_ms) {
            Finish(Result::FAIL_SEEK_TIMEOUT);
        }
        break;
    }

    case State::SETTLE_AFTER_ZERO:
        if (Settled(now_ms)) {
            Finish(Result::PASS);
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

void MotorSingleLimitCalibration::Cancel() {
    if (motor_ != nullptr) {
        motor_->Relax();
    }
    detector_.Reset();
    state_ = State::IDLE;
    result_ = Result::FAIL_SEEK_TIMEOUT;
}

void MotorSingleLimitCalibration::EnterState(State state, uint32_t now_ms) {
    state_ = state;
    state_enter_ms_ = now_ms;
    detector_.Reset();
}

void MotorSingleLimitCalibration::Finish(Result result) {
    if (motor_ != nullptr) {
        motor_->Relax();
    }
    result_ = result;
    state_ = (result == Result::PASS) ? State::DONE : State::ERROR;
}

bool MotorSingleLimitCalibration::ParamsValid() const {
    return MotorLimitSeekParamsValid(params_.seek) &&
           MotorLimitIsFinite(params_.user_travel_rad) &&
           params_.user_travel_rad > 0.0f &&
           (!params_.settle_after_zero_enable || params_.seek.settle_time_ms > 0U);
}

bool MotorSingleLimitCalibration::FeedbackLooksValid() const {
    return MotorLimitFeedbackLooksValid(motor_);
}

bool MotorSingleLimitCalibration::StallDetected(uint32_t now_ms) {
    return detector_.StallDetected(motor_, params_.seek, now_ms);
}

bool MotorSingleLimitCalibration::Settled(uint32_t now_ms) {
    return detector_.Settled(motor_, params_.seek, now_ms);
}

} // namespace mrobot
