#include "module/ore_store.h"

#include <math.h>
#include <new>
#include <string.h>

#include "bsp/can.h"
#include "component/math/scalar.hpp"
#include "device/device.h"
#include "device/motor/factory/motor_factory.hpp"
#include "device/motor/packages/controller/motor_controller.hpp"
#include "device/motor/packages/soft_limit_learning/soft_limit_learning.hpp"
#include "device/motor_rm.h"

namespace {

namespace scalar = mr::component::math;

using PlatformMotor = mr::motor::RmM3508Motor;
using PlatformController = mr::motor::MotorControllerT<PlatformMotor>;
using SmallMotor = mr::motor::RmM2006Motor;
using SmallController = mr::motor::MotorControllerT<SmallMotor>;

constexpr float kDefaultDtS = 0.001f;
constexpr float kMinDtS = 0.0005f;
constexpr float kMaxDtS = 0.050f;
constexpr float kDefaultSampleFreqHz = 500.0f;
constexpr float kDefaultMoveVelocityRadS = 1.0f;
constexpr float kDefaultMoveAccelRadS2 = 30.0f;
constexpr float kDefaultArriveThresholdRad = 0.03f;
constexpr float kDefaultOnlineWaitTimeoutS = 1.5f;

alignas(PlatformController) unsigned char
    g_platform_controller_storage[sizeof(PlatformController)];
alignas(SmallController) unsigned char
    g_small_controller_storage[ORE_STORE_AXIS_NUM - 1u][sizeof(SmallController)];
alignas(mr::motor::SoftLimitLearning) unsigned char
    g_soft_limit_storage[ORE_STORE_AXIS_NUM][sizeof(mr::motor::SoftLimitLearning)];

bool AxisValid(uint8_t axis) {
  return axis < ORE_STORE_AXIS_NUM;
}

bool IsPlatformAxis(uint8_t axis) {
  return axis == ORE_STORE_AXIS_PLATFORM;
}

uint8_t SmallAxisIndex(uint8_t axis) {
  return (axis > 0u) ? static_cast<uint8_t>(axis - 1u) : 0u;
}

float PositiveOr(float value, float fallback) {
  return scalar::positive_or(value, fallback);
}

mr::motor::MotorInstallSpec BuildInstall(const OreStore_Params_t *param,
                                         uint8_t axis) {
  mr::motor::MotorInstallSpec install{};
  install.external_ratio =
      PositiveOr(param->motor_install[axis].external_ratio, 1.0f);
  install.reverse_output = param->motor_install[axis].reverse_output;
  return install;
}

mr::motor::MotorControllerConfig BuildControllerConfig(
    const OreStore_Params_t *param, uint8_t axis, float target_freq) {
  mr::motor::MotorControllerConfig config{};
  config.velocity_pid = &param->pid.velocity_pid[axis];
  config.position_pid = &param->pid.position_pid[axis];
  config.sample_freq =
      PositiveOr(param->controller.sample_freq,
                 PositiveOr(target_freq, kDefaultSampleFreqHz));
  config.position_to_velocity_limit =
      scalar::positive_or_zero(param->controller.position_to_velocity_limit[axis]);
  config.velocity_to_torque_limit =
      scalar::positive_or_zero(param->controller.velocity_to_torque_limit[axis]);
  config.feedback_lowpass_cutoff_hz =
      scalar::positive_or_zero(param->controller.feedback_lowpass_cutoff_hz[axis]);
  config.output_lowpass_cutoff_hz =
      scalar::positive_or_zero(param->controller.output_lowpass_cutoff_hz[axis]);
  return config;
}

mr::motor::MotorTemperatureProtectionConfig BuildTemperatureProtectionConfig(
    const OreStore_Params_t *param) {
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

mr::motor::SoftLimitLearningConfig BuildSoftLimitConfig(
    const OreStore_SoftLimitConfig_t &config) {
  mr::motor::SoftLimitLearningConfig out{};
  out.seek_velocity_rad_s = 0.0f;
  out.known_travel_rad = 0.0f;
  out.limit_margin_rad = config.limit_margin_rad;
  out.min_range_rad = config.min_range_rad;
  return out;
}

PlatformController *PlatformControllerPtr(const OreStore_t *store) {
  return reinterpret_cast<PlatformController *>(
      store->controller[ORE_STORE_AXIS_PLATFORM]);
}

SmallController *SmallControllerPtr(const OreStore_t *store, uint8_t axis) {
  return reinterpret_cast<SmallController *>(store->controller[axis]);
}

mr::motor::SoftLimitLearning *SoftLimitPtr(const OreStore_t *store,
                                           uint8_t axis) {
  return reinterpret_cast<mr::motor::SoftLimitLearning *>(
      store->soft_limit[axis]);
}

int8_t ControllerRegister(OreStore_t *store, uint8_t axis) {
  return IsPlatformAxis(axis) ? PlatformControllerPtr(store)->Register()
                              : SmallControllerPtr(store, axis)->Register();
}

int8_t ControllerEnable(OreStore_t *store, uint8_t axis) {
  return IsPlatformAxis(axis) ? PlatformControllerPtr(store)->Enable()
                              : SmallControllerPtr(store, axis)->Enable();
}

int8_t ControllerRelax(OreStore_t *store, uint8_t axis) {
  return IsPlatformAxis(axis) ? PlatformControllerPtr(store)->Relax()
                              : SmallControllerPtr(store, axis)->Relax();
}

int8_t ControllerUpdate(OreStore_t *store, uint8_t axis) {
  return IsPlatformAxis(axis) ? PlatformControllerPtr(store)->Update()
                              : SmallControllerPtr(store, axis)->Update();
}

int8_t ControllerCommit(OreStore_t *store, uint8_t axis) {
  return IsPlatformAxis(axis) ? PlatformControllerPtr(store)->CommitCommand()
                              : SmallControllerPtr(store, axis)->CommitCommand();
}

int8_t ControllerSetVelocity(OreStore_t *store, uint8_t axis,
                             float velocity_rad_s) {
  return IsPlatformAxis(axis)
             ? PlatformControllerPtr(store)->SetVelocity(velocity_rad_s)
             : SmallControllerPtr(store, axis)->SetVelocity(velocity_rad_s);
}

int8_t ControllerSetPosition(OreStore_t *store, uint8_t axis,
                             float raw_position_rad,
                             float max_velocity_rad_s) {
  return IsPlatformAxis(axis)
             ? PlatformControllerPtr(store)->SetPosition(raw_position_rad,
                                                         max_velocity_rad_s)
             : SmallControllerPtr(store, axis)->SetPosition(raw_position_rad,
                                                            max_velocity_rad_s);
}

int8_t ControllerSetMIT(OreStore_t *store, uint8_t axis,
                       float target_position_rad, float target_velocity_rad_s,
                       float kp, float kd, float torque_ff) {
  return IsPlatformAxis(axis)
             ? PlatformControllerPtr(store)->SetMIT(target_position_rad,
                                                    target_velocity_rad_s,
                                                    kp, kd, torque_ff)
             : SmallControllerPtr(store, axis)->SetMIT(target_position_rad,
                                                        target_velocity_rad_s,
                                                        kp, kd, torque_ff);
}

void ResetAxisPid(OreStore_t *store, uint8_t axis) {
  if (store == nullptr || !AxisValid(axis)) {
    return;
  }
  if (IsPlatformAxis(axis)) {
    (void)PlatformControllerPtr(store)->ResetControllers();
  } else {
    (void)SmallControllerPtr(store, axis)->ResetControllers();
  }
}

bool ControllerHasPendingCommand(const OreStore_t *store, uint8_t axis) {
  return IsPlatformAxis(axis)
             ? PlatformControllerPtr(store)->HasPendingCommand()
             : SmallControllerPtr(store, axis)->HasPendingCommand();
}

float ControllerLastSetTorqueNm(const OreStore_t *store, uint8_t axis) {
  return IsPlatformAxis(axis)
     ? PlatformControllerPtr(store)
       ->GetMotor()
       .ProtocolDebug()
       .last_set_torque_nm
     : SmallControllerPtr(store, axis)
       ->GetMotor()
       .ProtocolDebug()
       .last_set_torque_nm;
}

float ControllerPendingCurrentA(const OreStore_t *store, uint8_t axis) {
  return IsPlatformAxis(axis)
     ? PlatformControllerPtr(store)
       ->GetMotor()
       .ProtocolDebug()
       .pending_torque_current
     : SmallControllerPtr(store, axis)
       ->GetMotor()
       .ProtocolDebug()
       .pending_torque_current;
}

float ControllerFilteredPosition(const OreStore_t *store, uint8_t axis) {
  return IsPlatformAxis(axis) ? PlatformControllerPtr(store)->GetLastFeedbackPosition()
                              : SmallControllerPtr(store, axis)->GetLastFeedbackPosition();
}

float ControllerFilteredVelocity(const OreStore_t *store, uint8_t axis) {
  return IsPlatformAxis(axis) ? PlatformControllerPtr(store)->GetLastFeedbackVelocity()
                              : SmallControllerPtr(store, axis)->GetLastFeedbackVelocity();
}

float ControllerFilteredOutputTorque(const OreStore_t *store, uint8_t axis) {
  return IsPlatformAxis(axis) ? PlatformControllerPtr(store)->GetLastOutputTorque()
                              : SmallControllerPtr(store, axis)->GetLastOutputTorque();
}

mr::motor::MotorState ControllerState(const OreStore_t *store, uint8_t axis) {
  return IsPlatformAxis(axis) ? PlatformControllerPtr(store)->GetState()
                              : SmallControllerPtr(store, axis)->GetState();
}

void StoreFeedback(MOTOR_Feedback_t *feedback,
                   const mr::motor::MotorState &state,
                   float local_position_rad) {
  if (feedback == nullptr) {
    return;
  }

  feedback->rotor_abs_angle = local_position_rad;
  feedback->rotor_speed = state.velocity_rad_s;
  feedback->torque_current = state.torque_nm;
  feedback->temp = state.temperature_c;
}

float AxisTravel(const OreStore_Params_t *param, uint8_t axis) {
  return scalar::positive_or_zero(param->limit.travel_rad[axis]);
}

float AxisMoveVelocity(const OreStore_Params_t *param, uint8_t axis) {
  return PositiveOr(param->limit.move_velocity_rad_s[axis],
                    kDefaultMoveVelocityRadS);
}

float AxisMoveAccel(const OreStore_Params_t *param, uint8_t axis) {
  return PositiveOr(param->limit.move_accel_rad_s2[axis],
                    kDefaultMoveAccelRadS2);
}

float UpdateAxisMotionProfile(OreStore_t *store, uint8_t axis,
                              float target_position_rad) {
  const float dt_s = scalar::positive_or_zero(store->dt);
  const float max_velocity = AxisMoveVelocity(store->param, axis);
  const float max_accel = AxisMoveAccel(store->param, axis);
  if (dt_s <= 0.0f || max_velocity <= 0.0f || max_accel <= 0.0f) {
    store->command_velocity_rad_s[axis] = 0.0f;
    store->tracked_position_rad[axis] = target_position_rad;
    return target_position_rad;
  }

  const float position_error = target_position_rad - store->tracked_position_rad[axis];
  const float stop_distance =
      (store->command_velocity_rad_s[axis] * store->command_velocity_rad_s[axis]) /
      (2.0f * max_accel);
  float desired_velocity = 0.0f;
  if (fabsf(position_error) > 0.0001f) {
    desired_velocity = (position_error > 0.0f) ? max_velocity : -max_velocity;
    if (fabsf(position_error) <= stop_distance) {
      desired_velocity = 0.0f;
    }
  }

  const float max_delta_velocity = max_accel * dt_s;
  store->command_velocity_rad_s[axis] = scalar::move_towards(
      store->command_velocity_rad_s[axis], desired_velocity, max_delta_velocity);

  float next_position =
      store->tracked_position_rad[axis] + store->command_velocity_rad_s[axis] * dt_s;
  if ((position_error > 0.0f && next_position > target_position_rad) ||
      (position_error < 0.0f && next_position < target_position_rad)) {
    next_position = target_position_rad;
    store->command_velocity_rad_s[axis] = 0.0f;
  }

  store->tracked_position_rad[axis] = next_position;
  return next_position;
}

float AxisSeekVelocity(const OreStore_Params_t *param, uint8_t axis) {
  return PositiveOr(param->limit.lower_seek_velocity_rad_s[axis],
                    kDefaultMoveVelocityRadS);
}

float AxisArriveThreshold(const OreStore_Params_t *param, uint8_t axis,
                          float fallback) {
  return PositiveOr(param->limit.arrive_threshold_rad[axis], fallback);
}

float AxisOnlineWaitTimeout(const OreStore_Params_t *param, uint8_t axis) {
  return PositiveOr(param->limit.config[axis].online_wait_timeout_s,
                    kDefaultOnlineWaitTimeoutS);
}

bool AxisFixedPowerOnEnabled(const OreStore_t *store, uint8_t axis) {
  return store != nullptr && store->param != nullptr && AxisValid(axis) &&
         store->param->power_on.fixed_position_enable[axis];
}

float AxisFixedPowerOnPosition(const OreStore_t *store, uint8_t axis) {
  if (!AxisFixedPowerOnEnabled(store, axis)) {
    return 0.0f;
  }
  const float position = store->param->power_on.fixed_position_rad[axis];
  return scalar::is_finite_scalar(position) ? position : 0.0f;
}

void SyncAxisTargetFromPosition(OreStore_t *store, uint8_t axis,
                                float position) {
  if (store == nullptr || !AxisValid(axis)) {
    return;
  }
  store->target_position_rad[axis] = position;
  store->tracked_position_rad[axis] = position;
  store->command_position_rad[axis] = position;
}

bool AxisCanUseFixedAssume(const OreStore_t *store, uint8_t axis) {
  return AxisFixedPowerOnEnabled(store, axis) &&
         store->feedback.online[axis] &&
         store->debug.controller_update_ret[axis] == DEVICE_OK &&
         !store->axis_failed[axis];
}

bool AxisCanUseFixedHome(const OreStore_t *store, uint8_t axis) {
  return AxisFixedPowerOnEnabled(store, axis) &&
         store->param->power_on.home_mode_uses_fixed_position &&
         store->feedback.online[axis] &&
         store->debug.controller_update_ret[axis] == DEVICE_OK &&
         !store->axis_failed[axis];
}

int8_t ApplyFixedPowerOnAssume(OreStore_t *store, uint8_t axis) {
  if (!AxisCanUseFixedAssume(store, axis)) {
    return ORE_STORE_ERR;
  }

  const float fixed_position = AxisFixedPowerOnPosition(store, axis);
  const int8_t ret = OreStore_AssumeAxisHomedAtCurrent(store, axis, fixed_position);
  store->power_on_assume_ret[axis] = ret;
  if (ret == ORE_STORE_OK) {
    SyncAxisTargetFromPosition(store, axis, fixed_position);
  }
  return ret;
}

bool AxisFeedbackReady(const OreStore_t *store, uint8_t axis) {
  return store != nullptr && AxisValid(axis) && store->feedback.online[axis] &&
         store->debug.controller_update_ret[axis] == DEVICE_OK;
}

float CommandTarget(const OreStore_CMD_t *cmd, uint8_t axis) {
  switch (axis) {
    case ORE_STORE_AXIS_PLATFORM:
      return cmd->platform_target_rad;
    case ORE_STORE_AXIS_GATE_LEFT:
      return cmd->gate_target_rad[0];
    case ORE_STORE_AXIS_GATE_RIGHT:
      return cmd->gate_target_rad[1];
    case ORE_STORE_AXIS_TRACK_LEFT:
      return cmd->track_target_rad[0];
    case ORE_STORE_AXIS_TRACK_RIGHT:
      return cmd->track_target_rad[1];
    default:
      return 0.0f;
  }
}

void RefreshDebugAxis(OreStore_t *store, uint8_t axis) {
  if (store == nullptr || !AxisValid(axis)) {
    return;
  }

  mr::motor::SoftLimitLearning *limit = SoftLimitPtr(store, axis);
  store->debug.target_position_rad[axis] = store->target_position_rad[axis];
  store->debug.command_position_rad[axis] = store->command_position_rad[axis];
  store->debug.zero_offset_rad[axis] = store->zero_offset_rad[axis];
  store->debug.learned_lower_raw_rad[axis] = store->learned_lower_raw_rad[axis];
  store->debug.travel_rad[axis] = AxisTravel(store->param, axis);
  store->debug.velocity_setpoint_rad_s[axis] =
      store->velocity_setpoint_rad_s[axis];
  store->debug.online_wait_s[axis] = store->homing_online_wait_s[axis];
  store->debug.seek_velocity_rad_s[axis] =
      (limit != nullptr) ? limit->GetSeekVelocity() : 0.0f;
  store->debug.soft_limit_state[axis] =
      (limit != nullptr)
          ? static_cast<uint8_t>(limit->GetState())
          : static_cast<uint8_t>(mr::motor::SoftLimitLearningState::Failed);
  store->debug.stall_cycles[axis] =
      (limit != nullptr) ? limit->GetStallCycles() : 0u;
  store->debug.homing_started[axis] = store->homing_started[axis];
  store->debug.axis_failed[axis] = store->axis_failed[axis];
  store->debug.command_pending[axis] =
      (store->controller[axis] != nullptr)
          ? ControllerHasPendingCommand(store, axis)
          : false;
  if (store->controller[axis] != nullptr) {
    store->debug.filtered_position_rad[axis] =
        ControllerFilteredPosition(store, axis);
    store->debug.filtered_velocity_rad_s[axis] =
        ControllerFilteredVelocity(store, axis);
    store->debug.filtered_output_torque_nm[axis] =
        ControllerFilteredOutputTorque(store, axis);
  } else {
    store->debug.filtered_position_rad[axis] = store->feedback.position_rad[axis];
    store->debug.filtered_velocity_rad_s[axis] = store->feedback.velocity_rad_s[axis];
    store->debug.filtered_output_torque_nm[axis] = 0.0f;
  }
  store->debug.motor_torque_nm[axis] = store->feedback.motor[axis].torque_current;
  if (store->controller[axis] != nullptr) {
    store->debug.rm_last_set_torque_nm[axis] =
        ControllerLastSetTorqueNm(store, axis);
    store->debug.rm_pending_current_a[axis] =
        ControllerPendingCurrentA(store, axis);
  } else {
    store->debug.rm_last_set_torque_nm[axis] = 0.0f;
    store->debug.rm_pending_current_a[axis] = 0.0f;
  }
  store->debug.power_on_fixed_position_rad[axis] =
      AxisFixedPowerOnPosition(store, axis);
  store->debug.power_on_assume_ret[axis] = store->power_on_assume_ret[axis];
  store->debug.power_on_assume_count[axis] = store->power_on_assume_count[axis];
  const MOTOR_RM_Param_t *motor_param = &store->param->motor_param[axis];
  const int8_t logical_index = (motor_param->id >= 0x201u &&
                               motor_param->id <= 0x208u)
                                  ? (int8_t)(motor_param->id - 0x201u)
                                  : -1;
  if (logical_index >= 0) {
    const MOTOR_RM_SlotTxDebug_t *tx_debug =
        MOTOR_RM_GetSlotTxDebug((uint8_t)logical_index);
    if (tx_debug != nullptr && tx_debug->valid && tx_debug->can == motor_param->can) {
      store->debug.rm_output_raw[axis] = tx_debug->output_value;
      store->debug.rm_tx_frame_id[axis] = tx_debug->tx_frame_id;
    }
  }
}

void ResetAxisHoming(OreStore_t *store, uint8_t axis) {
  if (store == nullptr || !AxisValid(axis)) {
    return;
  }

  mr::motor::SoftLimitLearning *limit = SoftLimitPtr(store, axis);
  if (limit != nullptr) {
    limit->Reset();
    limit->Configure(BuildSoftLimitConfig(store->param->limit.config[axis]));
  }
  store->axis_homed[axis] = false;
  store->homing_started[axis] = false;
  store->axis_failed[axis] = false;
  store->zero_offset_rad[axis] = 0.0f;
  store->learned_lower_raw_rad[axis] = 0.0f;
  store->homing_online_wait_s[axis] = 0.0f;
  store->power_on_assume_ret[axis] = ORE_STORE_ERR_NULL;
  store->target_position_rad[axis] = 0.0f;
  store->tracked_position_rad[axis] = 0.0f;
  store->command_position_rad[axis] = 0.0f;
  store->velocity_setpoint_rad_s[axis] = 0.0f;
  ResetAxisPid(store, axis);
  RefreshDebugAxis(store, axis);
}

void ResetAllHoming(OreStore_t *store) {
  for (uint8_t axis = 0; axis < ORE_STORE_AXIS_NUM; ++axis) {
    ResetAxisHoming(store, axis);
  }
}

bool AxisHomed(const OreStore_t *store, uint8_t axis) {
  if (store == nullptr || !AxisValid(axis)) {
    return false;
  }
  const mr::motor::SoftLimitLearning *limit = SoftLimitPtr(store, axis);
  return store->axis_homed[axis] && limit != nullptr && limit->IsReady();
}

bool AllHomed(const OreStore_t *store) {
  if (store == nullptr) {
    return false;
  }
  for (uint8_t axis = 0; axis < ORE_STORE_AXIS_NUM; ++axis) {
    if (!AxisHomed(store, axis)) {
      return false;
    }
  }
  return true;
}

void SyncAxisTargetToFeedback(OreStore_t *store, uint8_t axis) {
  if (store == nullptr || !AxisValid(axis) || !AxisHomed(store, axis)) {
    return;
  }

  const float position = store->feedback.position_rad[axis];
  store->target_position_rad[axis] = position;
  store->tracked_position_rad[axis] = position;
  store->command_position_rad[axis] = position;
  store->velocity_setpoint_rad_s[axis] = 0.0f;
  ResetAxisPid(store, axis);
}

int8_t FinishAxisHome(OreStore_t *store, uint8_t axis, float raw_lower_rad) {
  mr::motor::SoftLimitLearning *limit = SoftLimitPtr(store, axis);
  if (limit == nullptr) {
    return ORE_STORE_ERR_NULL;
  }

  const float travel_rad = AxisTravel(store->param, axis);
  limit->Reset();
  limit->Configure(BuildSoftLimitConfig(store->param->limit.config[axis]));
  if (!limit->SetRangeByLowerAndTravel(0.0f, travel_rad)) {
    store->axis_failed[axis] = true;
    limit->MarkFailed();
    return ORE_STORE_ERR;
  }

  const float learned_lower = limit->GetRange().lower_rad;
  store->learned_lower_raw_rad[axis] = raw_lower_rad;
  store->zero_offset_rad[axis] = raw_lower_rad - learned_lower;
  store->axis_homed[axis] = true;
  store->homing_started[axis] = false;
  store->axis_failed[axis] = false;
  store->homing_online_wait_s[axis] = 0.0f;
  store->target_position_rad[axis] = learned_lower;
  store->tracked_position_rad[axis] = learned_lower;
  store->command_position_rad[axis] = learned_lower;
  return ControllerSetPosition(store, axis, store->zero_offset_rad[axis] + learned_lower,
                               AxisMoveVelocity(store->param, axis));
}

int8_t HomeAxis(OreStore_t *store, uint8_t axis) {
  mr::motor::SoftLimitLearning *limit = SoftLimitPtr(store, axis);
  if (limit == nullptr) {
    return ORE_STORE_ERR_NULL;
  }

  if (AxisCanUseFixedHome(store, axis)) {
    return ApplyFixedPowerOnAssume(store, axis);
  }

  if (store->axis_failed[axis] || limit->IsFailed()) {
    store->axis_failed[axis] = true;
    (void)ControllerRelax(store, axis);
    return ORE_STORE_ERR;
  }

  if (!AxisFeedbackReady(store, axis)) {
    (void)ControllerRelax(store, axis);
    store->target_position_rad[axis] = store->feedback.position_rad[axis];
    store->tracked_position_rad[axis] = store->feedback.position_rad[axis];
    store->command_position_rad[axis] = store->feedback.position_rad[axis];
    store->homing_online_wait_s[axis] += store->dt;
    if (store->homing_online_wait_s[axis] >=
        AxisOnlineWaitTimeout(store->param, axis)) {
      store->axis_failed[axis] = true;
      store->homing_started[axis] = false;
      limit->MarkFailed();
      return ORE_STORE_ERR;
    }
    return ORE_STORE_OK;
  }
  store->homing_online_wait_s[axis] = 0.0f;

  if (AxisHomed(store, axis)) {
    const float home_position = limit->GetRange().lower_rad;
    store->target_position_rad[axis] = home_position;
    store->tracked_position_rad[axis] = home_position;
    store->command_position_rad[axis] = home_position;
    return ControllerSetPosition(store, axis,
                                 store->zero_offset_rad[axis] + home_position,
                                 AxisMoveVelocity(store->param, axis));
  }

  if (!store->homing_started[axis]) {
    limit->ClearRange();
    mr::motor::SoftLimitLearningConfig config = limit->GetConfig();
    config.seek_velocity_rad_s = AxisSeekVelocity(store->param, axis);
    limit->Configure(config);
    if (!limit->StartSeekLower()) {
      store->axis_failed[axis] = true;
      limit->MarkFailed();
      return ORE_STORE_ERR;
    }
    store->homing_started[axis] = true;
  }

  if (limit->HasLower() && !limit->IsSeeking()) {
    return FinishAxisHome(store, axis, limit->GetRange().lower_rad);
  }

  if (!limit->IsSeeking()) {
    store->homing_started[axis] = false;
    return ORE_STORE_OK;
  }

  store->target_position_rad[axis] = 0.0f;
  store->tracked_position_rad[axis] = 0.0f;
  store->command_position_rad[axis] = 0.0f;
  return ControllerSetVelocity(store, axis, limit->GetSeekVelocity());
}

int8_t ActiveAxis(OreStore_t *store, const OreStore_CMD_t *cmd, uint8_t axis) {
  mr::motor::SoftLimitLearning *limit = SoftLimitPtr(store, axis);
  if (limit == nullptr) {
    return ORE_STORE_ERR_NULL;
  }

  if (!AxisHomed(store, axis)) {
    if (store->param->power_on.auto_assume_on_active &&
      AxisCanUseFixedAssume(store, axis)) {
      const int8_t assume_ret = ApplyFixedPowerOnAssume(store, axis);
      if (assume_ret != ORE_STORE_OK) {
        (void)ControllerRelax(store, axis);
        SyncAxisTargetFromPosition(store, axis, store->feedback.position_rad[axis]);
        return assume_ret;
      }
    } else {
      (void)ControllerRelax(store, axis);
      SyncAxisTargetFromPosition(store, axis, store->feedback.position_rad[axis]);
      return ORE_STORE_ERR;
    }
  }

  const float requested_target = CommandTarget(cmd, axis);
  store->target_position_rad[axis] = requested_target;
  const float limited_target = limit->ClampPosition(requested_target);
  (void)UpdateAxisMotionProfile(store, axis, limited_target);
  store->command_position_rad[axis] = limit->ClampPosition(store->tracked_position_rad[axis]);
  const float raw_target = store->zero_offset_rad[axis] + store->command_position_rad[axis];

  if (store->control_mode == ORE_STORE_CONTROL_MIT_STYLE) {
    // MIT风格控制: 直接设置位置/速度目标，由controller计算力矩
    const float kp = store->param->pid.mit_kp[axis];
    const float kd = store->param->pid.mit_kd[axis];
    const float target_velocity = AxisMoveVelocity(store->param, axis);
    return ControllerSetMIT(store, axis, raw_target, target_velocity, kp, kd, 0.0f);
  }

  // 默认双环PID控制
  if (IsPlatformAxis(axis)) {
    // Use proper cascade: SetPosition sets the target and velocity limit,
    // Update() runs position PID + velocity PID in series.
    return PlatformControllerPtr(store)->SetPosition(
        raw_target, AxisMoveVelocity(store->param, axis));
  }
  return ControllerSetPosition(store, axis, raw_target,
                               AxisMoveVelocity(store->param, axis));
}

int8_t ConstructAxis(OreStore_t *store, uint8_t axis, float target_freq) {
  const auto motor_config =
      mr::motor::MotorInstanceConfig<mr::motor::MotorKind::RM>::
          FromVendorParam(store->param->motor_param[axis]);
  const mr::motor::MotorInstallSpec install = BuildInstall(store->param, axis);
  const mr::motor::MotorControllerConfig controller_config =
      BuildControllerConfig(store->param, axis, target_freq);
  const mr::motor::MotorTemperatureProtectionConfig temperature_config =
      BuildTemperatureProtectionConfig(store->param);

  if (IsPlatformAxis(axis)) {
    PlatformMotor *motor =
        mr::motor::MotorFactory::Create<mr::motor::MotorKind::RM,
                                        mr::motor::MotorModel::M3508>(
            motor_config, install, temperature_config);
    if (motor == nullptr) {
      return ORE_STORE_ERR;
    }
    PlatformController *controller =
        new (g_platform_controller_storage)
            PlatformController(*motor, controller_config);
    store->motor[axis] = motor;
    store->controller[axis] = controller;
  } else {
    const uint8_t small_idx = SmallAxisIndex(axis);
    SmallMotor *motor =
        mr::motor::MotorFactory::Create<mr::motor::MotorKind::RM,
                                        mr::motor::MotorModel::M2006>(
        motor_config, install, temperature_config);
    if (motor == nullptr) {
      return ORE_STORE_ERR;
    }
    SmallController *controller =
        new (g_small_controller_storage[small_idx])
            SmallController(*motor, controller_config);
    store->motor[axis] = motor;
    store->controller[axis] = controller;
  }

  mr::motor::SoftLimitLearning *limit =
      new (g_soft_limit_storage[axis]) mr::motor::SoftLimitLearning();
  limit->Configure(BuildSoftLimitConfig(store->param->limit.config[axis]));
  store->soft_limit[axis] = limit;

  if (ControllerRegister(store, axis) != DEVICE_OK ||
      ControllerEnable(store, axis) != DEVICE_OK) {
    return ORE_STORE_ERR;
  }
  return ORE_STORE_OK;
}

}  // namespace

extern "C" {

extern volatile OreStore_DebugCommand_t g_ore_store_debug_command;

int8_t OreStore_Init(OreStore_t *store, const OreStore_Params_t *param,
                     float target_freq) {
  if (store == nullptr || param == nullptr) {
    return ORE_STORE_ERR_NULL;
  }

  memset(store, 0, sizeof(OreStore_t));
  BSP_CAN_Init();

  store->param = param;
  store->mode = ORE_STORE_MODE_RELAX;
  store->control_mode = ORE_STORE_CONTROL_PID_DUAL;

  for (uint8_t axis = 0; axis < ORE_STORE_AXIS_NUM; ++axis) {
    const int8_t ret = ConstructAxis(store, axis, target_freq);
    if (ret != ORE_STORE_OK) {
      return ret;
    }
  }

  ResetAllHoming(store);
  return ORE_STORE_OK;
}

int8_t OreStore_UpdateFeedback(OreStore_t *store) {
  if (store == nullptr || store->param == nullptr) {
    return ORE_STORE_ERR_NULL;
  }

  bool all_homed = true;
  int8_t result = ORE_STORE_OK;
  for (uint8_t axis = 0; axis < ORE_STORE_AXIS_NUM; ++axis) {
    if (store->controller[axis] == nullptr ||
        store->soft_limit[axis] == nullptr) {
      result = ORE_STORE_ERR_NULL;
      continue;
    }

    const int8_t update_ret = ControllerUpdate(store, axis);
    store->debug.controller_update_ret[axis] = update_ret;
    if (update_ret != DEVICE_OK) {
      result = ORE_STORE_ERR;
    }

    const mr::motor::MotorState raw_state = ControllerState(store, axis);
    const float local_position =
        store->axis_homed[axis]
            ? raw_state.position_rad - store->zero_offset_rad[axis]
            : raw_state.position_rad;

    store->feedback.raw_position_rad[axis] = raw_state.position_rad;
    store->feedback.position_rad[axis] = local_position;
    store->feedback.velocity_rad_s[axis] = raw_state.velocity_rad_s;
    store->feedback.temperature_warning[axis] = raw_state.temperature_warning;
    store->feedback.temperature_over_limit[axis] =
      raw_state.temperature_over_limit;
    store->feedback.online[axis] = raw_state.online;
    store->feedback.homed[axis] = AxisHomed(store, axis);
    StoreFeedback(&store->feedback.motor[axis], raw_state, local_position);

    mr::motor::MotorState observed_state = raw_state;
    observed_state.position_rad = local_position;
    observed_state.online = raw_state.online && update_ret == DEVICE_OK;
    SoftLimitPtr(store, axis)->Observe(observed_state);

    if (!store->feedback.homed[axis]) {
      all_homed = false;
    }
    RefreshDebugAxis(store, axis);
  }

  store->feedback.all_homed = all_homed;
  return result;
}

int8_t OreStore_Control(OreStore_t *store, const OreStore_CMD_t *cmd,
                        uint32_t now) {
  if (store == nullptr || store->param == nullptr || cmd == nullptr) {
    return ORE_STORE_ERR_NULL;
  }

  store->dt = scalar::sanitize_dt((now - store->last_wakeup) * 0.001f,
                                  kDefaultDtS, kMinDtS, kMaxDtS);
  store->last_wakeup = now;
  store->debug.dt_s = store->dt;
  const OreStore_Mode_t previous_mode = store->mode;
  store->mode = cmd->mode;

  if (previous_mode != ORE_STORE_MODE_ACTIVE &&
      store->mode == ORE_STORE_MODE_ACTIVE) {
    for (uint8_t axis = 0; axis < ORE_STORE_AXIS_NUM; ++axis) {
      if (!AxisHomed(store, axis) &&
          store->param->power_on.auto_assume_on_active &&
          AxisCanUseFixedAssume(store, axis)) {
        (void)ApplyFixedPowerOnAssume(store, axis);
      }
      SyncAxisTargetToFeedback(store, axis);
    }
  }

  if (cmd->force_rehome && !store->rehome_latched) {
    ResetAllHoming(store);
  }
  store->rehome_latched = cmd->force_rehome;

  int8_t result = ORE_STORE_OK;
  for (uint8_t axis = 0; axis < ORE_STORE_AXIS_NUM; ++axis) {
    int8_t ret = ORE_STORE_OK;
    switch (store->mode) {
      case ORE_STORE_MODE_RELAX:
        store->target_position_rad[axis] = store->feedback.position_rad[axis];
        store->command_position_rad[axis] = store->feedback.position_rad[axis];
        ret = ControllerRelax(store, axis);
        break;
      case ORE_STORE_MODE_HOME:
        ret = HomeAxis(store, axis);
        break;
      case ORE_STORE_MODE_ACTIVE:
        ret = ActiveAxis(store, cmd, axis);
        break;
      default:
        ret = ORE_STORE_ERR;
        break;
    }

    store->debug.set_command_ret[axis] = ret;
    if (ret != DEVICE_OK && ret != ORE_STORE_OK) {
      result = ORE_STORE_ERR;
    }
    RefreshDebugAxis(store, axis);
  }

  return result;
}

void OreStore_Output(OreStore_t *store) {
  if (store == nullptr || store->param == nullptr) {
    return;
  }

  for (uint8_t axis = 0; axis < ORE_STORE_AXIS_NUM; ++axis) {
    if (store->controller[axis] == nullptr) {
      store->debug.commit_ret[axis] = DEVICE_ERR_NULL;
    } else {
      store->debug.commit_ret[axis] = ControllerCommit(store, axis);
    }
    RefreshDebugAxis(store, axis);
  }
}

void OreStore_ResetOutput(OreStore_t *store) {
  if (store == nullptr || store->param == nullptr) {
    return;
  }

  for (uint8_t axis = 0; axis < ORE_STORE_AXIS_NUM; ++axis) {
    if (store->controller[axis] != nullptr) {
      (void)ControllerRelax(store, axis);
    }
    store->debug.set_command_ret[axis] = DEVICE_OK;
    store->debug.commit_ret[axis] = ORE_STORE_ERR_NULL;
    RefreshDebugAxis(store, axis);
  }
  OreStore_Output(store);
}

void OreStore_RequestRehome(OreStore_t *store) {
  if (store == nullptr || store->param == nullptr) {
    return;
  }
  ResetAllHoming(store);
}

int8_t OreStore_AssumeAxisHomedAtCurrent(OreStore_t *store, uint8_t axis,
                                         float position_rad) {
  if (store == nullptr || store->param == nullptr || !AxisValid(axis)) {
    return ORE_STORE_ERR_NULL;
  }

  mr::motor::SoftLimitLearning *limit = SoftLimitPtr(store, axis);
  if (limit == nullptr) {
    return ORE_STORE_ERR_NULL;
  }

  const float travel_rad = AxisTravel(store->param, axis);
  if (!scalar::is_finite_scalar(position_rad) || position_rad < 0.0f ||
      position_rad > travel_rad) {
    return ORE_STORE_ERR;
  }

  limit->Reset();
  limit->Configure(BuildSoftLimitConfig(store->param->limit.config[axis]));
  if (!limit->SetRangeByLowerAndTravel(0.0f, travel_rad)) {
    store->axis_failed[axis] = true;
    limit->MarkFailed();
    RefreshDebugAxis(store, axis);
    return ORE_STORE_ERR;
  }

  store->learned_lower_raw_rad[axis] =
      store->feedback.raw_position_rad[axis] - position_rad;
  store->zero_offset_rad[axis] =
      store->feedback.raw_position_rad[axis] - position_rad;
  store->axis_homed[axis] = true;
  store->homing_started[axis] = false;
  store->axis_failed[axis] = false;
  store->homing_online_wait_s[axis] = 0.0f;
  store->power_on_assume_ret[axis] = ORE_STORE_OK;
  ++store->power_on_assume_count[axis];
  store->target_position_rad[axis] = position_rad;
  store->tracked_position_rad[axis] = position_rad;
  store->command_position_rad[axis] = position_rad;
  RefreshDebugAxis(store, axis);
  return ORE_STORE_OK;
}

bool OreStore_IsAxisHomed(const OreStore_t *store, uint8_t axis) {
  return AxisHomed(store, axis);
}

bool OreStore_IsAllHomed(const OreStore_t *store) {
  return AllHomed(store);
}

bool OreStore_IsAxisAtTarget(const OreStore_t *store, uint8_t axis,
                             float threshold_rad) {
  if (store == nullptr || !AxisValid(axis) || !AxisHomed(store, axis)) {
    return false;
  }

  const float threshold =
      AxisArriveThreshold(store->param, axis,
                          PositiveOr(fabsf(threshold_rad),
                                     kDefaultArriveThresholdRad));
  return fabsf(store->feedback.position_rad[axis] -
               store->command_position_rad[axis]) <= threshold;
}

bool OreStore_IsAllAtTarget(const OreStore_t *store, float threshold_rad) {
  if (store == nullptr) {
    return false;
  }

  for (uint8_t axis = 0; axis < ORE_STORE_AXIS_NUM; ++axis) {
    if (!OreStore_IsAxisAtTarget(store, axis, threshold_rad)) {
      return false;
    }
  }
  return true;
}

const OreStore_Debug_t *OreStore_GetDebug(const OreStore_t *store) {
  return (store != nullptr) ? &store->debug : nullptr;
}

void OreStore_SetControlMode(OreStore_t *store, OreStore_ControlMode_t mode) {
  if (store != nullptr) {
    store->control_mode = mode;
  }
}

OreStore_ControlMode_t OreStore_GetControlMode(const OreStore_t *store) {
  return (store != nullptr) ? store->control_mode : ORE_STORE_CONTROL_PID_DUAL;
}

void OreStore_SetDebugCommand(bool enable, OreStore_Mode_t mode,
                              float platform_target, const float gate_target[2],
                              const float track_target[2]) {
  g_ore_store_debug_command.enable = enable;
  g_ore_store_debug_command.mode = mode;
  g_ore_store_debug_command.platform_target_rad = platform_target;
  if (gate_target != nullptr) {
    for (int i = 0; i < 2; ++i) {
      g_ore_store_debug_command.gate_target_rad[i] = gate_target[i];
    }
  }
  if (track_target != nullptr) {
    for (int i = 0; i < 2; ++i) {
      g_ore_store_debug_command.track_target_rad[i] = track_target[i];
    }
  }
}

void OreStore_DisableDebugCommand(void) {
  g_ore_store_debug_command.enable = false;
}

}  // extern "C"
