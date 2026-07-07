#include "module/chassis/mecanum.hpp"

#include <cmath>
#include <cstdlib>
#include <new>

#include "debug_config.h"
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
constexpr float kDefaultDtS = 0.001f;
constexpr float kMinDtS = 0.0005f;
constexpr float kMaxDtS = 0.050f;
constexpr float kMinWheelRadiusM = 1e-4f;
constexpr float kHoldCommandDeadband = 1e-4f;
constexpr bool kZeroCommandWheelPositionHoldEnabled = true;//零向量锁死
constexpr float kWheelHoldEnterSpeedMps = 0.8f;
constexpr float kWheelHoldEnterStillTimeS = 0.030f;
constexpr float kWheelHoldMaxPositionStepRad = 0.35f;
constexpr float kDefaultStaticFrictionDeadbandMps = 0.01f;

mr::robotics::chassis::MecanumGeometry MakeMecanumGeometry(
    const Chassis_Params_t *param) {
  mr::robotics::chassis::MecanumGeometry geometry{};
  if (param != nullptr) {
    geometry.wheelbase_m = param->physical.wheelbase_m;
    geometry.trackwidth_m = param->physical.trackwidth_m;
  }
  return geometry;
}

mr::motor::MotorInstallSpec MakeInstallSpec(const Chassis_Params_t *param,
                                            uint8_t idx) {
  mr::motor::MotorInstallSpec spec{};
  if (param != nullptr && idx < MecanumController::kWheelCount) {
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
  const MOTOR_TemperatureProtectionConfig_t temperature_protection =
      MOTOR_NormalizeTemperatureProtection(param->motor_temperature_protection);
  config.warning_c = temperature_protection.warning_c;
  config.limit_c = temperature_protection.limit_c;
  config.auto_relax_on_limit =
      temperature_protection.auto_relax_on_limit;
  return config;
}

float ResolveWheelRadius(const Chassis_Params_t *param) {
  if (param == nullptr || param->physical.wheel_radius_m <= 0.0f) {
    return kMinWheelRadiusM;
  }
  return param->physical.wheel_radius_m;
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

bool IsSameSign(float a, float b) {
  return (a >= 0.0f && b >= 0.0f) || (a <= 0.0f && b <= 0.0f);
}

float StaticFrictionFeedforward(const Chassis_Params_t *param,
                                float ref_speed_mps) {
  if (param == nullptr || !scalar::is_finite_scalar(ref_speed_mps)) {
    return 0.0f;
  }

  const float compensation_nm = param->controller.wheel_static_friction_nm;
  if (!scalar::is_positive_scalar(compensation_nm)) {
    return 0.0f;
  }

  float deadband_mps = param->controller.wheel_static_friction_deadband_mps;
  if (!scalar::is_positive_scalar(deadband_mps)) {
    deadband_mps = kDefaultStaticFrictionDeadbandMps;
  }

  if (std::fabs(ref_speed_mps) <= deadband_mps) {
    return 0.0f;
  }

  return (ref_speed_mps > 0.0f) ? compensation_nm : -compensation_nm;
}

bool ShouldUseHighPoleWheelPid(const Chassis_Params_t *param,
                               float pole_lift_max_rad) {
  if (param == nullptr || !scalar::is_finite_scalar(pole_lift_max_rad)) {
    return false;
  }
  const float switch_lift = param->controller.wheel_high_pole_pid_switch_lift;
  return scalar::is_positive_scalar(switch_lift) &&
         pole_lift_max_rad > switch_lift;
}

}  // namespace

int8_t MecanumController::Init(const Chassis_Params_t *param,
                               float target_freq) {
  if (param == nullptr) {
    return CHASSIS_ERR_NULL;
  }
  if (param->type != CHASSIS_TYPE_MECANUM) {
    return CHASSIS_ERR_TYPE;
  }

  const MecanumGeometry geometry = MakeMecanumGeometry(param);
  MecanumKinematics kinematics(geometry);
  if (!kinematics.IsValid()) {
    return CHASSIS_ERR_TYPE;
  }

  BSP_CAN_Init();
  ResetRuntime();
  param_ = param;
  kinematics_ = kinematics;
  nominal_dt_ = scalar::sanitize_dt(1.0f / target_freq, kDefaultDtS, kMinDtS,
                                    kMaxDtS);
  InitFiltersAndPid(target_freq);

  const int8_t ret = RegisterWheels(target_freq);
  if (ret != CHASSIS_OK) {
    return ret;
  }

  UpdateBodyVelocityFeedback();
  return CHASSIS_OK;
}

void MecanumController::SetPoleLift(float front_lift_rad,
                                    float rear_lift_rad) {
  float max_lift = 0.0f;
  if (scalar::is_finite_scalar(front_lift_rad) && front_lift_rad > max_lift) {
    max_lift = front_lift_rad;
  }
  if (scalar::is_finite_scalar(rear_lift_rad) && rear_lift_rad > max_lift) {
    max_lift = rear_lift_rad;
  }
  pole_lift_max_rad_ = max_lift;
}

int8_t MecanumController::UpdateFeedback() {
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    Wheel *wheel = wheels_[i];
    if (wheel == nullptr) {
      return CHASSIS_ERR_NULL;
    }
    if (wheel->UpdateFeedback() != DEVICE_OK) {
      return CHASSIS_ERR;
    }
    StoreWheelState(i, wheel->State());
  }

  UpdateBodyVelocityFeedback();
  return CHASSIS_OK;
}

int8_t MecanumController::Control(const Chassis_CMD_t &cmd, uint32_t now) {
  if (param_ == nullptr) {
    return CHASSIS_ERR_NULL;
  }

  dt_ = CalcDt(now);
#if CHASSIS_RUNTIME_DEBUG_ENABLE
  debug_.dt_s = dt_;
  debug_.target_vec = cmd.ctrl_vec;
#endif

  const int8_t mode_ret = SetMode(cmd.mode, now);
  if (mode_ret != CHASSIS_OK) {
    return mode_ret;
  }
  UpdateWheelPidSelection();
#if CHASSIS_RUNTIME_DEBUG_ENABLE
  debug_.mode = mode_;
#endif

  switch (mode_) {
    case CHASSIS_MODE_BREAK:
      move_vec_.vx = 0.0f;
      move_vec_.vy = 0.0f;
      break;
    case CHASSIS_MODE_INDEPENDENT:
    case CHASSIS_MODE_OPEN:
    case CHASSIS_MODE_FOLLOW_GIMBAL:
    case CHASSIS_MODE_FOLLOW_GIMBAL_35:
    case CHASSIS_MODE_ROTOR:
    case CHASSIS_MODE_RELAX:
      move_vec_.vx = cmd.ctrl_vec.vx;
      move_vec_.vy = cmd.ctrl_vec.vy;
      break;
  }

  switch (mode_) {
    case CHASSIS_MODE_RELAX:
    case CHASSIS_MODE_BREAK:
      move_vec_.wz = 0.0f;
      break;
    case CHASSIS_MODE_INDEPENDENT:
    case CHASSIS_MODE_OPEN:
    case CHASSIS_MODE_FOLLOW_GIMBAL:
    case CHASSIS_MODE_FOLLOW_GIMBAL_35:
      move_vec_.wz = cmd.ctrl_vec.wz;
      break;
    case CHASSIS_MODE_ROTOR:
      move_vec_.wz =
          RotorSign(wz_multi_, cmd.mode_rotor) *
          RotorWz(kRotorWzMin, kRotorWzMax, now);
      break;
  }

  LimitMoveVector();
#if CHASSIS_RUNTIME_DEBUG_ENABLE
  debug_.output_vec = move_vec_;
#endif

  if (ShouldHoldZeroCommand()) {
    const bool hold_ready =
        wheel_hold_active_ || UpdateWheelHoldEntryReady();
    if (hold_ready) {
      const int8_t ret = ControlWheelHold();
      return ret;
    }
  } else {
    ResetWheelHoldEntryState();
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
    const float feedback = FilteredWheelSpeedFeedback(i);
    float torque_cmd = 0.0f;
    float pid_out = 0.0f;
    KPID_t *active_pid = ActiveWheelSpeedPid(i);

  #if CHASSIS_RUNTIME_DEBUG_ENABLE
    debug_.wheel_speed_ref_mps[i] = ref_speed;
    debug_.wheel_speed_fdb_mps[i] = feedback;
  #endif

    switch (mode_) {
      case CHASSIS_MODE_BREAK:
      case CHASSIS_MODE_FOLLOW_GIMBAL:
      case CHASSIS_MODE_FOLLOW_GIMBAL_35:
      case CHASSIS_MODE_ROTOR:
      case CHASSIS_MODE_INDEPENDENT:
        pid_out = PID_Calc(active_pid, ref_speed, feedback, 0.0f, dt_);
        torque_cmd = pid_out + StaticFrictionFeedforward(param_, ref_speed);
        break;
      case CHASSIS_MODE_OPEN:
        torque_cmd = ref_speed;
        break;
      case CHASSIS_MODE_RELAX:
        torque_cmd = 0.0f;
        break;
    }

#if CHASSIS_RUNTIME_DEBUG_ENABLE
    if (active_pid != nullptr) {
      debug_.wheel_pid_error[i] = active_pid->last.err;
      debug_.wheel_pid_integral[i] = active_pid->i;
    }
    debug_.wheel_torque_pid_out[i] = pid_out;
#endif
    out_.motor[i] = LowPassFilter2p_Apply(&output_filter_[i], torque_cmd);
    out_.motor[i] = ClampSymmetric(out_.motor[i], param_->limit.max_torque_cmd);
#if CHASSIS_RUNTIME_DEBUG_ENABLE
    debug_.wheel_torque_cmd_nm[i] = out_.motor[i];
#endif
  }

  for (uint8_t i = 0; i < kWheelCount; ++i) {
    Wheel *wheel = wheels_[i];
    if (wheel == nullptr) {
      return CHASSIS_ERR_NULL;
    }

    if (mode_ == CHASSIS_MODE_RELAX) {
      out_.set_torque_ret[i] = wheel->Relax();
      out_.controller_update_ret[i] = DEVICE_OK;
      out_.motor[i] = 0.0f;
      out_.command_pending[i] = false;
      continue;
    }

    out_.set_torque_ret[i] = wheel->SetTorque(out_.motor[i]);
    if (out_.set_torque_ret[i] == DEVICE_OK) {
      out_.controller_update_ret[i] = wheel->UpdateCommand();
    } else {
      out_.controller_update_ret[i] = out_.set_torque_ret[i];
    }
    if (out_.set_torque_ret[i] != DEVICE_OK ||
        out_.controller_update_ret[i] != DEVICE_OK) {
      return CHASSIS_ERR;
    }
    out_.command_pending[i] = wheel->HasPendingCommand();
  }

  return CHASSIS_OK;
}

void MecanumController::Output() {
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
  }

  if (param_ != nullptr) {
    (void)MOTOR_RM_Ctrl(
        const_cast<MOTOR_RM_Param_t *>(&param_->motor_param[0]));
  }
}

void MecanumController::ResetOutput() {
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    Wheel *wheel = wheels_[i];
    if (wheel != nullptr) {
      wheel->Relax();
      out_.motor[i] = 0.0f;
      out_.command_pending[i] = false;
    }
  }

  if (param_ != nullptr) {
    (void)MOTOR_RM_Ctrl(
        const_cast<MOTOR_RM_Param_t *>(&param_->motor_param[0]));
  }
}

void MecanumController::ResetRuntime() {
  param_ = nullptr;
  kinematics_ = MecanumKinematics{};
  mode_ = CHASSIS_MODE_RELAX;
  wheels_ = {};
  wheel_speed_ref_ = {};
  wheel_speed_ref_planned_ = {};
  move_vec_ = {};
  feedback_ = {};
#if CHASSIS_RUNTIME_DEBUG_ENABLE
  debug_ = {};
#endif
  out_ = {};
  last_wakeup_us_ = 0U;
  dt_ = 0.0f;
  nominal_dt_ = kDefaultDtS;
  mech_zero_ = 0.0f;
  wz_multi_ = 1.0f;
  pole_lift_max_rad_ = 0.0f;
  wheel_high_pole_pid_active_ = false;
  wheel_hold_active_ = false;
  wheel_hold_still_time_s_ = 0.0f;
  wheel_hold_position_rad_ = {};
  wheel_hold_last_position_rad_ = {};

  for (uint8_t i = 0; i < kWheelCount; ++i) {
    out_.set_torque_ret[i] = DEVICE_ERR;
    out_.controller_update_ret[i] = DEVICE_ERR;
    out_.commit_ret[i] = DEVICE_ERR;
  }
}

void MecanumController::InitFiltersAndPid(float target_freq) {
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    PID_Init(&wheel_pid_[i], KPID_MODE_NO_D, target_freq,
             &param_->pid.motor_pid_param);
    PID_Init(&wheel_high_pole_pid_[i], KPID_MODE_NO_D, target_freq,
             &param_->pid.motor_high_pole_pid_param);
    PID_Init(&wheel_hold_pid_[i], KPID_MODE_CALC_D, target_freq,
             &param_->pid.motor_pos_pid_param);
    LowPassFilter2p_Init(&wheel_speed_filter_[i], target_freq,
                         param_->low_pass_cutoff_freq.in);
    LowPassFilter2p_Init(&output_filter_[i], target_freq,
                         param_->low_pass_cutoff_freq.out);
  }

  for (uint8_t i = 0; i < 3U; ++i) {
    LowPassFilter2p_Init(&body_velocity_filter_[i], target_freq,
                         param_->low_pass_cutoff_freq.in);
  }

  PID_Init(&follow_pid_, KPID_MODE_CALC_D, target_freq,
           &param_->pid.follow_pid_param);
}

void MecanumController::ResetControlStateOnModeChange() {
  ResetWheelVelocityControlState();
  ResetWheelHoldControlState();
  for (uint8_t i = 0; i < 3U; ++i) {
    LowPassFilter2p_Reset(&body_velocity_filter_[i], 0.0f);
  }
  wheel_hold_active_ = false;
  ResetWheelHoldEntryState();
  ResetWheelSpeedPlanner();
}

void MecanumController::ResetWheelVelocityControlState() {
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    PID_Reset(&wheel_pid_[i]);
    PID_Reset(&wheel_high_pole_pid_[i]);
    LowPassFilter2p_Reset(&wheel_speed_filter_[i], 0.0f);
    LowPassFilter2p_Reset(&output_filter_[i], 0.0f);
  }
}

void MecanumController::ResetWheelHoldControlState() {
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    PID_Reset(&wheel_hold_pid_[i]);
  }
}

void MecanumController::ResetWheelSpeedPlanner() {
  wheel_speed_ref_planned_ = {};
}

void MecanumController::UpdateWheelPidSelection() {
  const bool use_high_pole_pid =
      ShouldUseHighPoleWheelPid(param_, pole_lift_max_rad_);
#if CHASSIS_RUNTIME_DEBUG_ENABLE
  debug_.wheel_high_pole_pid_active = wheel_high_pole_pid_active_;
#endif
  if (use_high_pole_pid == wheel_high_pole_pid_active_) {
    return;
  }

  wheel_high_pole_pid_active_ = use_high_pole_pid;
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    PID_Reset(&wheel_pid_[i]);
    PID_Reset(&wheel_high_pole_pid_[i]);
  }
#if CHASSIS_RUNTIME_DEBUG_ENABLE
  debug_.wheel_high_pole_pid_active = wheel_high_pole_pid_active_;
#endif
}

KPID_t *MecanumController::ActiveWheelSpeedPid(uint8_t idx) {
  if (idx >= kWheelCount) {
    return nullptr;
  }
  return wheel_high_pole_pid_active_ ? &wheel_high_pole_pid_[idx]
                                     : &wheel_pid_[idx];
}

int8_t MecanumController::SetMode(Chassis_Mode_t mode, uint32_t now) {
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

void MecanumController::LimitMoveVector() {
  move_vec_.vx = ClampSymmetric(move_vec_.vx, param_->limit.max_vx);
  move_vec_.vy = ClampSymmetric(move_vec_.vy, param_->limit.max_vy);
  move_vec_.wz = ClampSymmetric(move_vec_.wz, param_->limit.max_wz);
}

void MecanumController::UpdateBodyVelocityFeedback() {
  ResetMoveVector(&feedback_.chassis_vel);

  WheelSpeeds wheel_speeds{};
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    wheel_speeds[i] = WheelSpeedFeedback(i);
  }

  PlanarVelocity raw_velocity{};
  if (!kinematics_.ForwardKinematics(wheel_speeds, raw_velocity)) {
    return;
  }

  feedback_.chassis_vel.vx =
      LowPassFilter2p_Apply(&body_velocity_filter_[0], raw_velocity.vx_mps);
  feedback_.chassis_vel.vy =
      LowPassFilter2p_Apply(&body_velocity_filter_[1], raw_velocity.vy_mps);
  feedback_.chassis_vel.wz =
      LowPassFilter2p_Apply(&body_velocity_filter_[2], raw_velocity.wz_rad_s);
#if CHASSIS_RUNTIME_DEBUG_ENABLE
  debug_.feedback_vec = feedback_.chassis_vel;
#endif
}

int8_t MecanumController::ComputeWheelSpeeds() {
  PlanarVelocity body_velocity{};
  body_velocity.vx_mps = move_vec_.vx;
  body_velocity.vy_mps = move_vec_.vy;
  body_velocity.wz_rad_s = move_vec_.wz;

  WheelSpeeds wheel_speeds{};
  if (!kinematics_.InverseKinematics(body_velocity, wheel_speeds)) {
    return CHASSIS_ERR_TYPE;
  }

  for (uint8_t i = 0; i < kWheelCount; ++i) {
    wheel_speeds[i] = PlanWheelStartSpeed(i, wheel_speeds[i]);
  }
  wheel_speed_ref_ = wheel_speeds;
  return CHASSIS_OK;
}

float MecanumController::PlanWheelStartSpeed(uint8_t idx,
                                             float target_speed_mps) {
  if (idx >= kWheelCount || !scalar::is_finite_scalar(target_speed_mps)) {
    return 0.0f;
  }

  const float accel_limit =
      (param_ != nullptr) ? param_->controller.wheel_start_accel_mps2 : 0.0f;
  if (!scalar::is_positive_scalar(accel_limit)) {
    wheel_speed_ref_planned_[idx] = target_speed_mps;
    return target_speed_mps;
  }

  float previous = wheel_speed_ref_planned_[idx];
  if (!scalar::is_finite_scalar(previous)) {
    previous = 0.0f;
  }

  const float abs_target = std::fabs(target_speed_mps);
  const float abs_previous = std::fabs(previous);
  if (abs_target <= kHoldCommandDeadband) {
    wheel_speed_ref_planned_[idx] = 0.0f;
    return 0.0f;
  }

  if (abs_previous > kHoldCommandDeadband &&
      IsSameSign(previous, target_speed_mps) &&
      abs_target <= abs_previous) {
    wheel_speed_ref_planned_[idx] = target_speed_mps;
    return target_speed_mps;
  }

  const float planner_start =
      IsSameSign(previous, target_speed_mps) ? previous : 0.0f;
  const float max_delta = accel_limit * dt_;
  const float planned =
      scalar::move_towards(planner_start, target_speed_mps, max_delta);
  wheel_speed_ref_planned_[idx] = planned;
  return planned;
}

bool MecanumController::ShouldHoldZeroCommand() const {
  if (!kZeroCommandWheelPositionHoldEnabled) {
    return false;
  }

  if (mode_ == CHASSIS_MODE_RELAX || mode_ == CHASSIS_MODE_BREAK) {
    return false;
  }

  return std::fabs(move_vec_.vx) <= kHoldCommandDeadband &&
         std::fabs(move_vec_.vy) <= kHoldCommandDeadband &&
         std::fabs(move_vec_.wz) <= kHoldCommandDeadband;
}

bool MecanumController::UpdateWheelHoldEntryReady() {
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    const float speed_mps = WheelSpeedFeedback(i);
    if (!scalar::is_finite_scalar(speed_mps) ||
        std::fabs(speed_mps) > kWheelHoldEnterSpeedMps) {
      ResetWheelHoldEntryState();
      return false;
    }
  }

  wheel_hold_still_time_s_ += dt_;
  return wheel_hold_still_time_s_ >= kWheelHoldEnterStillTimeS;
}

void MecanumController::ResetWheelHoldEntryState() {
  wheel_hold_still_time_s_ = 0.0f;
}

void MecanumController::EnterWheelHold() {
  ResetWheelSpeedPlanner();
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    wheel_hold_position_rad_[i] =
        (wheels_[i] != nullptr) ? wheels_[i]->State().position_rad : 0.0f;
    wheel_hold_last_position_rad_[i] = wheel_hold_position_rad_[i];
    if (wheels_[i] != nullptr) {
      wheels_[i]->ClearPendingCommand();
    }
  }
  wheel_speed_ref_ = {};
  ResetWheelVelocityControlState();
  ResetWheelHoldControlState();
  wheel_hold_active_ = true;
}

void MecanumController::ExitWheelHold() {
  ResetWheelVelocityControlState();
  ResetWheelHoldControlState();
  ResetWheelHoldEntryState();
  wheel_hold_active_ = false;
}

bool MecanumController::WheelHoldPositionJumped(uint8_t idx,
                                                float position_rad) const {
  if (idx >= kWheelCount || !scalar::is_finite_scalar(position_rad)) {
    return true;
  }

  const float last_position = wheel_hold_last_position_rad_[idx];
  return scalar::is_finite_scalar(last_position) &&
         std::fabs(position_rad - last_position) >
             kWheelHoldMaxPositionStepRad;
}

int8_t MecanumController::ControlWheelHold() {
  if (!wheel_hold_active_) {
    EnterWheelHold();
  }

  wheel_speed_ref_ = {};
  for (uint8_t i = 0; i < kWheelCount; ++i) {
    Wheel *wheel = wheels_[i];
    if (wheel == nullptr) {
      return CHASSIS_ERR_NULL;
    }

    const float position_rad = wheel->State().position_rad;
    float torque_cmd = 0.0f;
    if (!scalar::is_finite_scalar(position_rad)) {
      PID_Reset(&wheel_hold_pid_[i]);
    } else {
      if (WheelHoldPositionJumped(i, position_rad)) {
        wheel_hold_position_rad_[i] = position_rad;
        PID_Reset(&wheel_hold_pid_[i]);
      }
      torque_cmd = PID_Calc(&wheel_hold_pid_[i],
                            wheel_hold_position_rad_[i],
                            position_rad, 0.0f, dt_);
      wheel_hold_last_position_rad_[i] = position_rad;
    }
#if CHASSIS_RUNTIME_DEBUG_ENABLE
    debug_.wheel_speed_ref_mps[i] = 0.0f;
    debug_.wheel_speed_fdb_mps[i] = WheelSpeedFeedback(i);
    debug_.wheel_pid_error[i] = wheel_hold_pid_[i].last.err;
    debug_.wheel_pid_integral[i] = wheel_hold_pid_[i].i;
    debug_.wheel_torque_pid_out[i] = torque_cmd;
#endif
    out_.motor[i] = ClampSymmetric(torque_cmd, param_->limit.max_torque_cmd);
#if CHASSIS_RUNTIME_DEBUG_ENABLE
    debug_.wheel_torque_cmd_nm[i] = out_.motor[i];
#endif

    out_.set_torque_ret[i] = wheel->SetTorque(out_.motor[i]);
    if (out_.set_torque_ret[i] == DEVICE_OK) {
      out_.controller_update_ret[i] = wheel->UpdateCommand();
    } else {
      out_.controller_update_ret[i] = out_.set_torque_ret[i];
    }
    if (out_.set_torque_ret[i] != DEVICE_OK ||
        out_.controller_update_ret[i] != DEVICE_OK) {
      return CHASSIS_ERR;
    }
    out_.command_pending[i] = wheel->HasPendingCommand();
  }

  return CHASSIS_OK;
}

int8_t MecanumController::RegisterWheels(float target_freq) {
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

float MecanumController::WheelSpeedFeedback(uint8_t idx) const {
  if (idx >= kWheelCount || wheels_[idx] == nullptr) {
    return 0.0f;
  }
  return wheels_[idx]->State().linear_velocity_mps;
}

float MecanumController::FilteredWheelSpeedFeedback(uint8_t idx) {
  if (idx >= kWheelCount) {
    return 0.0f;
  }
  return LowPassFilter2p_Apply(&wheel_speed_filter_[idx],
                               WheelSpeedFeedback(idx));
}

void MecanumController::StoreWheelState(uint8_t idx, const WheelState &state) {
  feedback_.motor[idx].position_rad = state.position_rad;
  feedback_.motor[idx].velocity_rad_s = state.angular_velocity_rad_s;
  feedback_.motor[idx].torque_nm = state.torque_nm;
  feedback_.motor[idx].temperature_c = state.temperature_c;
  feedback_.motor[idx].temperature_warning = state.temperature_warning;
  feedback_.motor[idx].temperature_over_limit = state.temperature_over_limit;
  feedback_.motor[idx].temperature_limit_latched =
      state.temperature_limit_latched;
  feedback_.motor[idx].online = state.online;
}

float MecanumController::CalcDt(uint32_t now_ms) {
  float dt = nominal_dt_;
  if (last_wakeup_us_ != 0U) {
    dt = static_cast<float>(
             (uint32_t)(now_ms - static_cast<uint32_t>(last_wakeup_us_))) *
         1.0e-3f;
  }
  last_wakeup_us_ = now_ms;
  return scalar::sanitize_dt(dt, nominal_dt_, nominal_dt_ * 0.5f,
                             nominal_dt_ * 3.0f);
}

}  // namespace mr::module::chassis
