#include "module/chassis/front_omni_rear_mecanum.hpp"

#include <cmath>
#include <cstdlib>
#include <new>

#include "bsp/can.h"
#include "component/math/scalar.hpp"
#include "device/device.h"
#include "device/motor_rm.h"

namespace mr::module::chassis {

namespace {

using PlanarVelocity = mr::robotics::chassis::PlanarVelocity;
namespace scalar = mr::component::math;

constexpr float kRotorWzMin = 0.6f;
constexpr float kRotorWzMax = 0.8f;
constexpr float kRotorOmega = 0.001f;
constexpr float kFollowOffset35DegRad = M_2PI * 7.0f / 72.0f;
constexpr float kDefaultDtS = 0.001f;
constexpr float kMinDtS = 0.0005f;
constexpr float kMaxDtS = 0.050f;
constexpr float kMinWheelSpeedMps = 1e-4f;
constexpr float kMinWheelRadiusM = 1e-4f;
constexpr float kHoldCommandDeadband = 1e-4f;

mr::robotics::chassis::FrontOmniRearMecanumGeometry
MakeGeometry(const Chassis_Params_t *param) {
  mr::robotics::chassis::FrontOmniRearMecanumGeometry geometry{};
  if (param != nullptr) {
    geometry.wheelbase_m = param->physical.wheelbase_m;
    geometry.trackwidth_m = param->physical.trackwidth_m;
  }
  return geometry;
}

mr::motor::MotorInstallSpec MakeInstallSpec(const Chassis_Params_t *param,
                                            uint8_t idx) {
  mr::motor::MotorInstallSpec spec{};
  if (param != nullptr && idx < FrontOmniRearMecanumController::kWheelCount) {
    spec.external_ratio = param->motor_install[idx].external_ratio;
    spec.reverse_output = param->motor_install[idx].reverse_output;
  }
  return spec;
}

mr::motor::MotorTemperatureProtectionConfig MakeTemperatureProtection(
    const Chassis_Params_t *param) {
  mr::motor::MotorTemperatureProtectionConfig config{};
  if (param == nullptr) {
    return config;
  }
  config.warning_c = param->motor_temperature_protection.warning_c;
  config.limit_c = param->motor_temperature_protection.limit_c;
  config.auto_relax_on_limit =
      param->motor_temperature_protection.auto_relax_on_limit;
  return config;
}

float ResolveWheelRadius(const Chassis_Params_t *param) {
  if (param == nullptr || param->physical.wheel_radius_m <= 0.0f) {
    return kMinWheelRadiusM;
  }
  return param->physical.wheel_radius_m;
}

float WheelSpeedLimit(const Chassis_Params_t *param) {
  if (param == nullptr) {
    return 0.0f;
  }
  const float limit = param->physical.wheel_output_max_speed;
  return scalar::is_positive_scalar(limit) ? limit : 0.0f;
}

float ClampSymmetric(float value, float limit) {
  if (!scalar::is_finite_scalar(value)) {
    return 0.0f;
  }
  if (!scalar::is_positive_scalar(limit)) {
    return value;
  }
  return scalar::abs_clip_scalar(value, limit);
}

float RotorWz(float min_wz, float max_wz, uint32_t now) {
  const float wz_vary = std::fabs(0.2f * std::sin(kRotorOmega * now)) + min_wz;
  return (wz_vary > max_wz) ? max_wz : wz_vary;
}

float RotorSign(float stored_sign, Chassis_RotorMode_t mode) {
  switch (mode) {
    case ROTOR_MODE_CW:
      return -1.0f;
    case ROTOR_MODE_CCW:
      return 1.0f;
    case ROTOR_MODE_RAND:
    default:
      return (stored_sign != 0.0f) ? stored_sign : 1.0f;
  }
}

bool LateralHeadingHoldEnabled(const Chassis_Params_t *param) {
  return param != nullptr &&
         param->front_omni_rear_mecanum.lateral_heading_hold_enable;
}

float PositiveOr(float value, float fallback) {
  return scalar::is_positive_scalar(value) ? value : fallback;
}

}  // namespace

int8_t FrontOmniRearMecanumController::Init(const Chassis_Params_t *param,
                                            float target_freq) {
  if (param == nullptr) {
    return CHASSIS_ERR_NULL;
  }
  if (param->type != CHASSIS_TYPE_FRONT_OMNI_REAR_MECANUM) {
    return CHASSIS_ERR_TYPE;
  }

  Kinematics kinematics(MakeGeometry(param));
  if (!kinematics.IsValid()) {
    return CHASSIS_ERR_TYPE;
  }

  BSP_CAN_Init();
  ResetRuntime();
  param_ = param;
  kinematics_ = kinematics;
  InitFiltersAndPid(target_freq);

  const int8_t ret = RegisterWheels(target_freq);
  if (ret != CHASSIS_OK) {
    return ret;
  }

  UpdateBodyVelocityFeedback();
  return CHASSIS_OK;
}

int8_t FrontOmniRearMecanumController::UpdateFeedback() {
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    Wheel *wheel = wheels_[i];
    if (wheel == nullptr) {
      return CHASSIS_ERR_NULL;
    }
    if (wheel->Update() != DEVICE_OK) {
      return CHASSIS_ERR;
    }
    StoreWheelState(i, wheel->State());
  }

  UpdateBodyVelocityFeedback();
  return CHASSIS_OK;
}

int8_t FrontOmniRearMecanumController::Control(const Chassis_CMD_t &cmd,
                                               uint32_t now) {
  if (param_ == nullptr) {
    return CHASSIS_ERR_NULL;
  }

  dt_ = CalcDt(now);
  debug_.dt_s = dt_;
  debug_.cmd_vec_raw = cmd.ctrl_vec;

  const int8_t mode_ret = SetMode(cmd.mode, now);
  if (mode_ret != CHASSIS_OK) {
    return mode_ret;
  }

  switch (mode_) {
    case CHASSIS_MODE_BREAK:
      move_vec_.vx = 0.0f;
      move_vec_.vy = 0.0f;
      break;
    case CHASSIS_MODE_INDEPENDENT:
      move_vec_.vx = cmd.ctrl_vec.vx;
      move_vec_.vy = cmd.ctrl_vec.vy;
      debug_.gimbal_beta_rad = 0.0f;
      break;
    default: {
      float beta = feedback_.encoder_gimbalYawMotor - mech_zero_;
      if (param_->reverse.yaw) {
        beta = -beta;
      }
      const float cos_beta = std::cos(beta);
      const float sin_beta = std::sin(beta);
      debug_.gimbal_beta_rad = beta;
      move_vec_.vx = cos_beta * cmd.ctrl_vec.vx - sin_beta * cmd.ctrl_vec.vy;
      move_vec_.vy = sin_beta * cmd.ctrl_vec.vx + cos_beta * cmd.ctrl_vec.vy;
      break;
    }
  }

  debug_.cmd_vec_body.vx = move_vec_.vx;
  debug_.cmd_vec_body.vy = move_vec_.vy;

  switch (mode_) {
    case CHASSIS_MODE_RELAX:
    case CHASSIS_MODE_BREAK:
      move_vec_.wz = 0.0f;
      break;
    case CHASSIS_MODE_INDEPENDENT:
    case CHASSIS_MODE_OPEN:
      move_vec_.wz = cmd.ctrl_vec.wz;
      break;
    case CHASSIS_MODE_FOLLOW_GIMBAL:
      move_vec_.wz =
          PID_Calc(&follow_pid_, mech_zero_, feedback_.encoder_gimbalYawMotor,
                   0.0f, dt_);
      break;
    case CHASSIS_MODE_FOLLOW_GIMBAL_35:
      move_vec_.wz =
          PID_Calc(&follow_pid_, mech_zero_ + kFollowOffset35DegRad,
                   feedback_.encoder_gimbalYawMotor, 0.0f, dt_);
      break;
    case CHASSIS_MODE_ROTOR:
      move_vec_.wz =
          RotorSign(wz_multi_, cmd.mode_rotor) *
          RotorWz(kRotorWzMin, kRotorWzMax, now);
      break;
  }

  debug_.lateral_wz_feedforward = 0.0f;
  debug_.lateral_heading_hold_enabled = LateralHeadingHoldEnabled(param_);
  debug_.lateral_heading_hold_active = lateral_heading_hold_active_;
  debug_.lateral_heading_error_rad = 0.0f;
  debug_.lateral_yaw_rate_rad_s =
      scalar::is_finite_scalar(yaw_rate_rad_s_) ? yaw_rate_rad_s_ : 0.0f;
  debug_.lateral_heading_wz_correction = 0.0f;

  if (ShouldUseLateralYawCorrection(cmd.ctrl_vec.wz)) {
    debug_.lateral_wz_feedforward = CalcLateralWzFeedforward();
    move_vec_.wz += debug_.lateral_wz_feedforward;
  }

  if (ShouldUseLateralHeadingHold(cmd.ctrl_vec.wz)) {
    if (!lateral_heading_hold_active_) {
      EnterLateralHeadingHold();
    }
    debug_.lateral_heading_wz_correction = CalcLateralHeadingHoldWz();
    move_vec_.wz += debug_.lateral_heading_wz_correction;
  } else if (lateral_heading_hold_active_) {
    ExitLateralHeadingHold();
  }
  debug_.lateral_heading_hold_active = lateral_heading_hold_active_;

  debug_.cmd_vec_body.wz = move_vec_.wz;
  debug_.rotor_wz_cmd = (mode_ == CHASSIS_MODE_ROTOR) ? move_vec_.wz : 0.0f;
  LimitMoveVector();
  debug_.cmd_vec_limited = move_vec_;

  if (ShouldHoldZeroCommand()) {
    return ControlWheelHold();
  }
  if (wheel_hold_active_) {
    ExitWheelHold();
  }

  const int8_t ik_ret = ComputeWheelSpeeds();
  if (ik_ret != CHASSIS_OK) {
    return ik_ret;
  }

  for (uint8_t i = 0; i < kWheelCount; ++i) {
    const float ref_speed = wheel_speed_ref_[i];
    const float raw_feedback = WheelSpeedFeedback(i);
    const float feedback = FilteredWheelSpeedFeedback(i);
    float torque_cmd = 0.0f;

    debug_.wheel_speed_ref_mps[i] = ref_speed;
    debug_.wheel_speed_fdb_mps[i] = raw_feedback;
    debug_.wheel_speed_fdb_filtered_mps[i] = feedback;

    switch (mode_) {
      case CHASSIS_MODE_BREAK:
      case CHASSIS_MODE_FOLLOW_GIMBAL:
      case CHASSIS_MODE_FOLLOW_GIMBAL_35:
      case CHASSIS_MODE_ROTOR:
      case CHASSIS_MODE_INDEPENDENT:
        torque_cmd =
            PID_Calc(&wheel_pid_[i], ref_speed, feedback, 0.0f, dt_);
        break;
      case CHASSIS_MODE_OPEN:
        torque_cmd = ref_speed /
                     std::fmax(WheelSpeedLimit(param_), kMinWheelSpeedMps);
        break;
      case CHASSIS_MODE_RELAX:
        torque_cmd = 0.0f;
        break;
    }

    debug_.wheel_torque_pid_out[i] = torque_cmd;
    out_.motor[i] = LowPassFilter2p_Apply(&output_filter_[i], torque_cmd);
    out_.motor[i] = ClampSymmetric(out_.motor[i], param_->limit.max_torque_cmd);
    debug_.wheel_torque_cmd_nm[i] = out_.motor[i];
  }

  for (uint8_t i = 0; i < kWheelCount; ++i) {
    Wheel *wheel = wheels_[i];
    if (wheel == nullptr) {
      return CHASSIS_ERR_NULL;
    }

    if (mode_ == CHASSIS_MODE_RELAX) {
      out_.set_torque_ret[i] = wheel->Relax();
      StoreWheelDebug(i);
      out_.controller_update_ret[i] = DEVICE_OK;
      out_.motor[i] = 0.0f;
      out_.command_pending[i] = false;
      continue;
    }

    out_.set_torque_ret[i] = wheel->SetTorque(out_.motor[i]);
    StoreWheelDebug(i);
    if (out_.set_torque_ret[i] != DEVICE_OK) {
      return CHASSIS_ERR;
    }
    out_.controller_update_ret[i] = DEVICE_OK;
    out_.command_pending[i] = wheel->HasPendingCommand();
  }

  return CHASSIS_OK;
}

void FrontOmniRearMecanumController::Output() {
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    Wheel *wheel = wheels_[i];
    if (wheel == nullptr) {
      out_.commit_ret[i] = DEVICE_ERR_NULL;
      out_.command_pending[i] = false;
      out_.last_commit_ok[i] = false;
      continue;
    }

    out_.commit_ret[i] = wheel->CommitCommand();
    out_.command_pending[i] = wheel->HasPendingCommand();
    out_.last_commit_ok[i] = (out_.commit_ret[i] == DEVICE_OK);
    StoreWheelDebug(i);
  }

  if (param_ != nullptr) {
    (void)MOTOR_RM_Ctrl(const_cast<MOTOR_RM_Param_t *>(&param_->motor_param[0]));
  }
}

void FrontOmniRearMecanumController::ResetOutput() {
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    Wheel *wheel = wheels_[i];
    if (wheel != nullptr) {
      wheel->Relax();
      out_.motor[i] = 0.0f;
      out_.command_pending[i] = false;
    }
  }

  if (param_ != nullptr) {
    (void)MOTOR_RM_Ctrl(const_cast<MOTOR_RM_Param_t *>(&param_->motor_param[0]));
  }
}

void FrontOmniRearMecanumController::ResetRuntime() {
  param_ = nullptr;
  kinematics_ = Kinematics{};
  mode_ = CHASSIS_MODE_RELAX;
  wheels_ = {};
  wheel_speed_ref_ = {};
  move_vec_ = {};
  feedback_ = {};
  debug_ = {};
  out_ = {};
  last_wakeup_ = 0U;
  dt_ = 0.0f;
  mech_zero_ = 0.0f;
  wz_multi_ = 1.0f;
  yaw_rate_rad_s_ = 0.0f;
  wheel_hold_active_ = false;
  lateral_heading_hold_active_ = false;
  lateral_heading_target_rad_ = 0.0f;
  wheel_hold_position_rad_ = {};

  for (uint8_t i = 0; i < kWheelCount; ++i) {
    out_.set_torque_ret[i] = DEVICE_ERR;
    out_.controller_update_ret[i] = DEVICE_ERR;
    out_.commit_ret[i] = DEVICE_ERR;
    debug_.wheel_last_set_torque_ret[i] = DEVICE_ERR;
    debug_.wheel_last_commit_ret[i] = DEVICE_ERR;
  }
}

void FrontOmniRearMecanumController::InitFiltersAndPid(float target_freq) {
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    PID_Init(&wheel_pid_[i], KPID_MODE_NO_D, target_freq,
             &param_->pid.motor_pid_param);
    PID_Init(&wheel_hold_pid_[i], KPID_MODE_CALC_D, target_freq,
             &param_->pid.motor_pos_pid_param);
    LowPassFilter2p_Init(&wheel_speed_filter_[i], target_freq,
                         param_->low_pass_cutoff_freq.in);
    LowPassFilter2p_Init(&output_filter_[i], target_freq,
                         param_->low_pass_cutoff_freq.out);
    LowPassFilter2p_Init(&torque_filter_[i], target_freq,
                         param_->low_pass_cutoff_freq.in);
    LowPassFilter2p_Reset(&torque_filter_[i], 0.0f);
  }

  for (uint8_t i = 0; i < 3U; ++i) {
    LowPassFilter2p_Init(&body_velocity_filter_[i], target_freq,
                         param_->low_pass_cutoff_freq.in);
  }

  PID_Init(&follow_pid_, KPID_MODE_CALC_D, target_freq,
           &param_->pid.follow_pid_param);
}

void FrontOmniRearMecanumController::ResetControlStateOnModeChange() {
  ResetWheelVelocityControlState();
  ResetWheelHoldControlState();
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    LowPassFilter2p_Reset(&torque_filter_[i], feedback_.motor[i].torque_nm);
  }
  for (uint8_t i = 0; i < 3U; ++i) {
    LowPassFilter2p_Reset(&body_velocity_filter_[i], 0.0f);
  }
  wheel_hold_active_ = false;
  lateral_heading_hold_active_ = false;
  lateral_heading_target_rad_ = feedback_.encoder_gimbalYawMotor;
}

void FrontOmniRearMecanumController::ResetWheelVelocityControlState() {
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    PID_Reset(&wheel_pid_[i]);
    LowPassFilter2p_Reset(&wheel_speed_filter_[i], 0.0f);
    LowPassFilter2p_Reset(&output_filter_[i], 0.0f);
  }
}

void FrontOmniRearMecanumController::ResetWheelHoldControlState() {
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    PID_Reset(&wheel_hold_pid_[i]);
  }
}

int8_t FrontOmniRearMecanumController::SetMode(Chassis_Mode_t mode,
                                               uint32_t now) {
  if (mode < CHASSIS_MODE_RELAX || mode > CHASSIS_MODE_OPEN) {
    return CHASSIS_ERR_MODE;
  }
  if (mode == mode_) {
    return CHASSIS_OK;
  }

  if (mode == CHASSIS_MODE_ROTOR && mode_ != CHASSIS_MODE_ROTOR) {
    std::srand(now);
    wz_multi_ = (std::rand() % 2) ? -1.0f : 1.0f;
  }

  ResetControlStateOnModeChange();
  mode_ = mode;
  return CHASSIS_OK;
}

void FrontOmniRearMecanumController::LimitMoveVector() {
  move_vec_.vx = ClampSymmetric(move_vec_.vx, param_->limit.max_vx);
  move_vec_.vy = ClampSymmetric(move_vec_.vy, param_->limit.max_vy);
  move_vec_.wz = ClampSymmetric(move_vec_.wz, param_->limit.max_wz);
}

void FrontOmniRearMecanumController::UpdateBodyVelocityFeedback() {
  ResetMoveVector(&feedback_.chassis_vel);

  WheelSpeeds wheel_speeds{};
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    wheel_speeds[i] = WheelSpeedFeedback(i);
  }

  PlanarVelocity raw_velocity{};
  if (!kinematics_.ForwardKinematics(wheel_speeds, raw_velocity)) {
    return;
  }

  debug_.body_vel_raw_vx = raw_velocity.vx_mps;
  debug_.body_vel_raw_vy = raw_velocity.vy_mps;
  debug_.body_vel_raw_wz = raw_velocity.wz_rad_s;
  feedback_.chassis_vel.vx =
      LowPassFilter2p_Apply(&body_velocity_filter_[0], raw_velocity.vx_mps);
  feedback_.chassis_vel.vy =
      LowPassFilter2p_Apply(&body_velocity_filter_[1], raw_velocity.vy_mps);
  feedback_.chassis_vel.wz =
      LowPassFilter2p_Apply(&body_velocity_filter_[2], raw_velocity.wz_rad_s);
}

int8_t FrontOmniRearMecanumController::ComputeWheelSpeeds() {
  PlanarVelocity body_velocity{};
  body_velocity.vx_mps = move_vec_.vx;
  body_velocity.vy_mps = move_vec_.vy;
  body_velocity.wz_rad_s = move_vec_.wz;

  WheelSpeeds wheel_speeds{};
  if (!kinematics_.InverseKinematics(body_velocity, wheel_speeds)) {
    return CHASSIS_ERR_TYPE;
  }

  Kinematics::ScaleWheelSpeedsToLimit(wheel_speeds, WheelSpeedLimit(param_));
  wheel_speed_ref_ = wheel_speeds;
  return CHASSIS_OK;
}

bool FrontOmniRearMecanumController::ShouldHoldZeroCommand() const {
  if (mode_ == CHASSIS_MODE_RELAX || mode_ == CHASSIS_MODE_BREAK) {
    return false;
  }

  return std::fabs(move_vec_.vx) <= kHoldCommandDeadband &&
         std::fabs(move_vec_.vy) <= kHoldCommandDeadband &&
         std::fabs(move_vec_.wz) <= kHoldCommandDeadband;
}

bool FrontOmniRearMecanumController::ShouldUseLateralYawCorrection(
    float raw_wz_cmd) const {
  if (mode_ != CHASSIS_MODE_INDEPENDENT && mode_ != CHASSIS_MODE_OPEN) {
    return false;
  }

  const float raw_wz_deadband = PositiveOr(
      param_->front_omni_rear_mecanum.lateral_heading_hold_wz_deadband,
      kHoldCommandDeadband);
  return std::fabs(move_vec_.vy) > kHoldCommandDeadband &&
         std::fabs(raw_wz_cmd) <= raw_wz_deadband;
}

bool FrontOmniRearMecanumController::ShouldUseLateralHeadingHold(
    float raw_wz_cmd) const {
  return LateralHeadingHoldEnabled(param_) &&
         ShouldUseLateralYawCorrection(raw_wz_cmd);
}

float FrontOmniRearMecanumController::CalcLateralWzFeedforward() const {
  const float gain =
      param_->front_omni_rear_mecanum.lateral_vy_to_wz_feedforward;
  if (!scalar::is_finite_scalar(gain)) {
    return 0.0f;
  }
  return ClampSymmetric(gain * move_vec_.vy, param_->limit.max_wz);
}

void FrontOmniRearMecanumController::EnterWheelHold() {
  lateral_heading_hold_active_ = false;
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    wheel_hold_position_rad_[i] =
        (wheels_[i] != nullptr) ? wheels_[i]->State().position_rad : 0.0f;
    if (wheels_[i] != nullptr) {
      wheels_[i]->ClearPendingCommand();
    }
  }
  wheel_speed_ref_ = {};
  ResetWheelVelocityControlState();
  ResetWheelHoldControlState();
  wheel_hold_active_ = true;
}

void FrontOmniRearMecanumController::ExitWheelHold() {
  ResetWheelVelocityControlState();
  ResetWheelHoldControlState();
  wheel_hold_active_ = false;
}

void FrontOmniRearMecanumController::EnterLateralHeadingHold() {
  lateral_heading_target_rad_ = feedback_.encoder_gimbalYawMotor;
  PID_Reset(&follow_pid_);
  lateral_heading_hold_active_ = true;
}

void FrontOmniRearMecanumController::ExitLateralHeadingHold() {
  PID_Reset(&follow_pid_);
  lateral_heading_hold_active_ = false;
}

float FrontOmniRearMecanumController::CalcLateralHeadingHoldWz() {
  float yaw_error = scalar::angle_error(
      lateral_heading_target_rad_, feedback_.encoder_gimbalYawMotor);
  const float error_deadband =
      PositiveOr(param_->front_omni_rear_mecanum.lateral_heading_hold_error_deadband,
             0.0f);
  if (std::fabs(yaw_error) <= error_deadband) {
    yaw_error = 0.0f;
  }

  const float yaw_rate =
      scalar::is_finite_scalar(yaw_rate_rad_s_) ? yaw_rate_rad_s_ : 0.0f;
  const float kp = scalar::is_finite_scalar(
                       param_->front_omni_rear_mecanum.lateral_heading_hold_kp)
                       ? param_->front_omni_rear_mecanum.lateral_heading_hold_kp
                       : 0.0f;
  const float kd = scalar::is_finite_scalar(
                       param_->front_omni_rear_mecanum.lateral_heading_hold_kd)
                       ? param_->front_omni_rear_mecanum.lateral_heading_hold_kd
                       : 0.0f;
  const float correction_wz = kp * yaw_error - kd * yaw_rate;
  const float limit = PositiveOr(
      param_->front_omni_rear_mecanum.lateral_heading_hold_max_wz,
      param_->limit.max_wz);

  debug_.lateral_heading_error_rad = yaw_error;
  debug_.lateral_yaw_rate_rad_s = yaw_rate;
  return ClampSymmetric(correction_wz, limit);
}

int8_t FrontOmniRearMecanumController::ControlWheelHold() {
  if (!wheel_hold_active_) {
    EnterWheelHold();
  }

  wheel_speed_ref_ = {};
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    Wheel *wheel = wheels_[i];
    if (wheel == nullptr) {
      return CHASSIS_ERR_NULL;
    }

    debug_.wheel_speed_ref_mps[i] = 0.0f;
    debug_.wheel_speed_fdb_mps[i] = WheelSpeedFeedback(i);
    debug_.wheel_speed_fdb_filtered_mps[i] = FilteredWheelSpeedFeedback(i);
    const float torque_cmd = PID_Calc(&wheel_hold_pid_[i],
                                      wheel_hold_position_rad_[i],
                                      wheel->State().position_rad, 0.0f, dt_);
    debug_.wheel_torque_pid_out[i] = torque_cmd;
    out_.motor[i] = ClampSymmetric(torque_cmd, param_->limit.max_torque_cmd);
    debug_.wheel_torque_cmd_nm[i] = out_.motor[i];

    out_.set_torque_ret[i] = wheel->SetTorque(out_.motor[i]);
    StoreWheelDebug(i);
    if (out_.set_torque_ret[i] != DEVICE_OK) {
      return CHASSIS_ERR;
    }
    out_.controller_update_ret[i] = DEVICE_OK;
    out_.command_pending[i] = wheel->HasPendingCommand();
  }

  return CHASSIS_OK;
}

int8_t FrontOmniRearMecanumController::RegisterWheels(float target_freq) {
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    mr::motor::MotorControllerConfig controller_config{};
    controller_config.velocity_pid = &param_->pid.motor_pid_param;
    controller_config.position_pid = &param_->pid.motor_pos_pid_param;
    controller_config.sample_freq =
        (param_->controller.sample_freq > 0.0f) ? param_->controller.sample_freq
                                                : target_freq;

    mr::wheel::RmM3508WheelConfig wheel_config{};
    wheel_config.motor_param = param_->motor_param[i];
    wheel_config.install = MakeInstallSpec(param_, i);
    wheel_config.temperature_protection = MakeTemperatureProtection(param_);
    wheel_config.controller = controller_config;
    wheel_config.radius_m = ResolveWheelRadius(param_);

    auto *wheel = new (wheel_storage_[i]) Wheel(wheel_config);
    wheels_[i] = wheel;
    if (!wheel->IsValid()) {
      return CHASSIS_ERR_NULL;
    }
    if (wheel->Register() != DEVICE_OK) {
      return CHASSIS_ERR;
    }
    if (wheel->Enable() != DEVICE_OK) {
      return CHASSIS_ERR;
    }
  }
  return CHASSIS_OK;
}

float FrontOmniRearMecanumController::WheelSpeedFeedback(uint8_t idx) const {
  if (idx >= kWheelCount || wheels_[idx] == nullptr) {
    return 0.0f;
  }
  return wheels_[idx]->State().linear_velocity_mps;
}

float FrontOmniRearMecanumController::FilteredWheelSpeedFeedback(uint8_t idx) {
  if (idx >= kWheelCount) {
    return 0.0f;
  }
  return LowPassFilter2p_Apply(&wheel_speed_filter_[idx],
                               WheelSpeedFeedback(idx));
}

float FrontOmniRearMecanumController::FilteredObservedTorque(uint8_t idx,
                                                            float torque_nm) {
  if (idx >= kWheelCount) {
    return torque_nm;
  }
  return LowPassFilter2p_Apply(&torque_filter_[idx], torque_nm);
}

void FrontOmniRearMecanumController::StoreWheelState(
    uint8_t idx,
    const WheelState &state) {
  feedback_.motor[idx].position_rad = state.position_rad;
  feedback_.motor[idx].velocity_rad_s = state.angular_velocity_rad_s;
  feedback_.motor[idx].torque_nm = state.torque_nm;
  feedback_.motor[idx].temperature_c = state.temperature_c;
  feedback_.motor[idx].temperature_warning = state.temperature_warning;
  feedback_.motor[idx].temperature_over_limit = state.temperature_over_limit;
  feedback_.motor[idx].temperature_limit_latched =
      state.temperature_limit_latched;
  feedback_.motor[idx].online = state.online;
  debug_.wheel_motor_velocity_rad_s[idx] = state.angular_velocity_rad_s;
  debug_.wheel_motor_torque_nm[idx] =
      FilteredObservedTorque(idx, state.torque_nm);
}

void FrontOmniRearMecanumController::StoreWheelDebug(uint8_t idx) {
  if (idx >= kWheelCount || wheels_[idx] == nullptr) {
    return;
  }

  const auto &debug = wheels_[idx]->Debug();
  debug_.wheel_pending_valid[idx] = debug.pending_valid;
  debug_.wheel_pending_torque_current[idx] = debug.pending_torque_current;
  debug_.wheel_last_set_torque_nm[idx] = debug.last_set_torque_nm;
  debug_.wheel_last_set_torque_ret[idx] = debug.last_set_torque_ret;
  debug_.wheel_last_commit_ret[idx] = debug.last_commit_ret;
  debug_.wheel_last_commit_skipped[idx] = debug.last_commit_skipped;
}

float FrontOmniRearMecanumController::CalcDt(uint32_t now) {
  float dt = static_cast<float>(now - last_wakeup_) / 1000.0f;
  last_wakeup_ = now;
  return scalar::sanitize_dt(dt, kDefaultDtS, kMinDtS, kMaxDtS);
}

}  // namespace mr::module::chassis
