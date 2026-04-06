/**
 * @file motor_soft_limit_learning.cpp
 * @brief 电机软限位学习状态机实现
 */

#include "device/motors/motor_packages/soft_limit_learning/motor_soft_limit_learning.hpp"

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

void MotorSoftLimitLearning::Start(uint32_t now_ms) {
    min_position_rad_ = 0.0f;
    max_position_rad_ = 0.0f;
    learned_ = false;
    stall_start_ms_ = 0;
    settle_start_ms_ = 0;

    if (motor_ == nullptr) {
        Finish(Result::FAIL_NULL);
        return;
    }

    result_ = Result::RUNNING;
    EnterState(State::SEEK_MIN, now_ms);
}

MotorSoftLimitLearning::Result MotorSoftLimitLearning::Update(uint32_t now_ms) {
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
    case State::SEEK_MIN:
        motor_->CurrentControl(-params_.seek_current);
        if (StallDetected(now_ms)) {
            min_position_rad_ = motor_->GetPositionRad();
            motor_->Relax();
            EnterState(State::SETTLE_AFTER_MIN, now_ms);
        } else if (ElapsedMs(now_ms, state_enter_ms_) >= params_.seek_timeout_ms) {
            Finish(Result::FAIL_SEEK_TIMEOUT);
        }
        break;

    case State::SETTLE_AFTER_MIN:
        if (Settled(now_ms)) {
            stall_start_ms_ = 0;
            EnterState(State::SEEK_MAX, now_ms);
        }
        break;

    case State::SEEK_MAX:
        motor_->CurrentControl(params_.seek_current);
        if (StallDetected(now_ms)) {
            max_position_rad_ = motor_->GetPositionRad();
            motor_->Relax();
            if ((max_position_rad_ - min_position_rad_) < params_.min_valid_range_rad) {
                Finish(Result::FAIL_INVALID_RANGE);
            } else {
                learned_ = true;
                if (params_.return_to_center_enable) {
                    EnterState(State::RETURN_TO_CENTER, now_ms);
                } else {
                    Finish(Result::PASS);
                }
            }
        } else if (ElapsedMs(now_ms, state_enter_ms_) >= params_.seek_timeout_ms) {
            Finish(Result::FAIL_SEEK_TIMEOUT);
        }
        break;

    case State::RETURN_TO_CENTER:
        motor_->PositionControl(GetCenterPositionRad(), params_.return_velocity_limit_rad_per_sec);
        if (ReachedCenter()) {
            motor_->Relax();
            Finish(Result::PASS);
        } else if (ElapsedMs(now_ms, state_enter_ms_) >= params_.return_timeout_ms) {
            Finish(Result::FAIL_SEEK_TIMEOUT);
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

void MotorSoftLimitLearning::Cancel() {
    if (motor_ != nullptr) {
        motor_->Relax();
    }
    state_ = State::IDLE;
    result_ = Result::FAIL_INVALID_RANGE;
}

void MotorSoftLimitLearning::EnterState(State state, uint32_t now_ms) {
    state_ = state;
    state_enter_ms_ = now_ms;
    stall_start_ms_ = 0;
    settle_start_ms_ = 0;
}

void MotorSoftLimitLearning::Finish(Result result) {
    if (motor_ != nullptr) {
        motor_->Relax();
    }
    result_ = result;
    state_ = (result == Result::PASS) ? State::DONE : State::ERROR;
}

bool MotorSoftLimitLearning::FeedbackLooksValid() const {
    return IsFinite(motor_->GetPositionRad()) &&
           IsFinite(motor_->GetVelocityRadPerSec()) &&
           IsFinite(motor_->GetTorqueCurrentAmp());
}

bool MotorSoftLimitLearning::StallDetected(uint32_t now_ms) {
    const bool low_velocity = std::fabs(motor_->GetVelocityRadPerSec()) <= params_.stall_velocity_threshold;
    const bool high_current = std::fabs(motor_->GetTorqueCurrentAmp()) >= params_.stall_current_threshold;

    if (low_velocity && high_current) {
        if (stall_start_ms_ == 0) {
            stall_start_ms_ = now_ms;
        }
        return ElapsedMs(now_ms, stall_start_ms_) >= params_.stall_confirm_ms;
    }

    stall_start_ms_ = 0;
    return false;
}

bool MotorSoftLimitLearning::Settled(uint32_t now_ms) {
    if (std::fabs(motor_->GetVelocityRadPerSec()) <= params_.settle_velocity_threshold) {
        if (settle_start_ms_ == 0) {
            settle_start_ms_ = now_ms;
        }
        return ElapsedMs(now_ms, settle_start_ms_) >= params_.settle_time_ms;
    }

    settle_start_ms_ = 0;
    return false;
}

bool MotorSoftLimitLearning::ReachedCenter() const {
    return std::fabs(motor_->GetPositionRad() - GetCenterPositionRad()) <=
           params_.return_position_tolerance_rad;
}

} // namespace mrobot