/**
 * @file motor_dual_limit_calibration.cpp
 * @brief 双边限位学习标定状态机实现
 */

#include "device/motors/motor_packages/limit_selfLearning/dual_limit/motor_dual_limit_calibration.hpp"

#include <cmath>

namespace mrobot {

void MotorDualLimitCalibration::Start(uint32_t now_ms) {
    min_position_rad_ = 0.0f;
    max_position_rad_ = 0.0f;
    learned_ = false;
    detector_.Reset();

    if (motor_ == nullptr) {
        Finish(Result::FAIL_NULL);
        return;
    }

    if (!ParamsValid()) {
        Finish(Result::FAIL_INVALID_RANGE);
        return;
    }

    result_ = Result::RUNNING;
    EnterState(State::SEEK_MIN, now_ms);
}

MotorDualLimitCalibration::Result MotorDualLimitCalibration::Update(uint32_t now_ms) {
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
        motor_->VelocityControl(-params_.seek.seek_velocity_rad_per_sec);
        if (StallDetected(now_ms)) {
            min_position_rad_ = motor_->GetPositionRad();
            motor_->Relax();
            EnterState(State::SETTLE_AFTER_MIN, now_ms);
        } else if (MotorLimitElapsedMs(now_ms, state_enter_ms_) >= params_.seek.seek_timeout_ms) {
            Finish(Result::FAIL_SEEK_TIMEOUT);
        }
        break;

    case State::SETTLE_AFTER_MIN:
        if (Settled(now_ms)) {
            detector_.ResetStall();
            EnterState(State::SEEK_MAX, now_ms);
        }
        break;

    case State::SEEK_MAX:
        motor_->VelocityControl(params_.seek.seek_velocity_rad_per_sec);
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
        } else if (MotorLimitElapsedMs(now_ms, state_enter_ms_) >= params_.seek.seek_timeout_ms) {
            Finish(Result::FAIL_SEEK_TIMEOUT);
        }
        break;

    case State::RETURN_TO_CENTER:
        motor_->PositionControl(GetCenterPositionRad(), params_.return_velocity_limit_rad_per_sec);
        if (ReachedCenter()) {
            motor_->Relax();
            Finish(Result::PASS);
        } else if (MotorLimitElapsedMs(now_ms, state_enter_ms_) >= params_.return_timeout_ms) {
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

void MotorDualLimitCalibration::Cancel() {
    if (motor_ != nullptr) {
        motor_->Relax();
    }
    detector_.Reset();
    state_ = State::IDLE;
    result_ = Result::FAIL_INVALID_RANGE;
}

void MotorDualLimitCalibration::EnterState(State state, uint32_t now_ms) {
    state_ = state;
    state_enter_ms_ = now_ms;
    detector_.Reset();
}

void MotorDualLimitCalibration::Finish(Result result) {
    if (motor_ != nullptr) {
        motor_->Relax();
    }
    result_ = result;
    state_ = (result == Result::PASS) ? State::DONE : State::ERROR;
}

bool MotorDualLimitCalibration::ParamsValid() const {
    return MotorLimitSeekParamsValid(params_.seek) &&
           MotorLimitIsFinite(params_.min_valid_range_rad) &&
           params_.min_valid_range_rad > 0.0f &&
           MotorLimitIsFinite(params_.return_position_tolerance_rad) &&
           params_.return_position_tolerance_rad >= 0.0f &&
           MotorLimitIsFinite(params_.return_velocity_limit_rad_per_sec) &&
           params_.return_velocity_limit_rad_per_sec > 0.0f &&
           params_.return_timeout_ms > 0U;
}

bool MotorDualLimitCalibration::FeedbackLooksValid() const {
    return MotorLimitFeedbackLooksValid(motor_);
}

bool MotorDualLimitCalibration::StallDetected(uint32_t now_ms) {
    return detector_.StallDetected(motor_, params_.seek, now_ms);
}

bool MotorDualLimitCalibration::Settled(uint32_t now_ms) {
    return detector_.Settled(motor_, params_.seek, now_ms);
}

bool MotorDualLimitCalibration::ReachedCenter() const {
    return std::fabs(motor_->GetPositionRad() - GetCenterPositionRad()) <=
           params_.return_position_tolerance_rad;
}

} // namespace mrobot
