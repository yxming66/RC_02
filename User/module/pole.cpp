/*
 * Pole module: 4x3508 support motors.
 */

#include "module/pole.h"

#include <math.h>
#include <string.h>

#include "bsp/can.h"
#include "component/math/scalar.hpp"
#include "component/trajectory/online_trapezoid.hpp"
#include "device/motor_rm.h"

namespace {

constexpr float kPoleManualLiftDeadband = 1.0e-4f;
constexpr float kPoleLiftLimitEpsilon = 1.0e-4f;
constexpr float kPoleManualOffsetRecoverySpeedRadS = 0.5f;

float Pole_PositiveOrZero(float value) {
  return mr::component::math::is_finite_scalar(value) && value > 0.0f
             ? value
             : 0.0f;
}

float Pole_ClampTrackedLiftAtLimit(Pole_t *c, uint8_t motor, float lift) {
  if (c == NULL || c->param == NULL || motor >= POLE_SUPPORT_MOTOR_NUM) {
    return lift;
  }

  const float travel = Pole_PositiveOrZero(c->param->limit.support_total_travel);
  const float min_target = mr::component::math::clamp_scalar(
      Pole_PositiveOrZero(c->param->limit.support_min_target_lift), 0.0f,
      travel);
  const float clamped_lift =
      mr::component::math::clamp_scalar(lift, min_target, travel);
  float *velocity = &c->support_angle.tracked_target_velocity[motor];

  if ((clamped_lift <= min_target + kPoleLiftLimitEpsilon && *velocity < 0.0f) ||
      (clamped_lift >= travel - kPoleLiftLimitEpsilon && *velocity > 0.0f)) {
    *velocity = 0.0f;
  }

  return clamped_lift;
}

void Pole_ResetSideTargetVelocity(Pole_t *c, uint8_t side) {
  if (c == NULL || side >= 2u) return;
  const uint8_t start = (side == 0u) ? 0u : 2u;
  for (uint8_t i = start; i < start + 2u; i++) {
    c->support_angle.tracked_target_velocity[i] = 0.0f;
  }
}

static void Pole_ClearSideTargetOffset(Pole_t *c, uint8_t side) {
  if (c == NULL || side >= 2u) return;

  const uint8_t start = (side == 0u) ? 0u : 2u;
  for (uint8_t i = start; i < start + 2u; i++) {
    c->support_angle.target_offset[i] = 0.0f;
  }
}

float Pole_UpdateTrackedLift(Pole_t *c, uint8_t motor, float speed_limit) {
  if (c == NULL || c->param == NULL || motor >= POLE_SUPPORT_MOTOR_NUM) {
    return 0.0f;
  }

  const uint8_t side = (motor < 2u) ? 0u : 1u;
  const float final_lift = mr::component::math::clamp_scalar(
      c->support_angle.final_target_lift[side] +
          c->support_angle.target_offset[motor],
      0.0f, c->param->limit.support_total_travel);

  const float max_velocity = Pole_PositiveOrZero(speed_limit);
  const float max_acceleration = c->setpoint.disable_lift_accel
      ? 0.0f
      : Pole_PositiveOrZero(c->setpoint.lift_accel[side]);

  if (max_velocity <= 0.0f) {
    c->support_angle.tracked_target_velocity[motor] = 0.0f;
    return Pole_ClampTrackedLiftAtLimit(c, motor, final_lift);
  }

  if (max_acceleration <= 0.0f) {
    c->support_angle.tracked_target_velocity[motor] = 0.0f;
    const float next_lift = mr::component::math::move_towards(
        c->support_angle.tracked_target_lift[motor], final_lift,
        max_velocity * c->dt);
    return Pole_ClampTrackedLiftAtLimit(c, motor, next_lift);
  }

  const mr::comp::traj::OnlineTrapezoidAxisSample sample =
      mr::comp::traj::update_trapezoid_axis(
          c->support_angle.tracked_target_lift[motor], final_lift,
          &c->support_angle.tracked_target_velocity[motor], max_velocity,
          max_acceleration, c->dt);

  const float next_lift =
      sample.valid ? sample.position : c->support_angle.tracked_target_lift[motor];
  return Pole_ClampTrackedLiftAtLimit(c, motor, next_lift);
}

}  // namespace

static MOTOR_RM_t *Pole_GetMotorHandle(const Pole_t *c, uint8_t idx) {
  return (MOTOR_RM_t *)c->motors[idx];
}

static float Pole_GetSupportAngle(const Pole_t *c, uint8_t idx) {
  MOTOR_RM_t *motor = Pole_GetMotorHandle(c, idx);
  if (motor == NULL) return 0.0f;

  return motor->feedback.rotor_total_angle;
}

static void Pole_ResetControllers(Pole_t *c) {
  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    PID_Reset(&c->pid.support_pos[i]);
    PID_Reset(&c->pid.support_vel[i]);
  }
}

static void Pole_RecoverSideTargetOffset(Pole_t *c, uint8_t side,
                                         float manual_target_delta) {
  if (c == NULL || side >= 2u || c->dt <= 0.0f) return;

  /* 校平修正不超过本周期手动目标移动量，避免任一 Pole 目标反向。 */
  const float recovery_step = fminf(
      kPoleManualOffsetRecoverySpeedRadS * c->dt,
      fabsf(manual_target_delta));
  if (recovery_step <= 0.0f) return;

  const uint8_t start = (side == 0u) ? 0u : 2u;
  for (uint8_t i = start; i < start + 2u; i++) {
    c->support_angle.target_offset[i] =
        mr::component::math::move_towards(
            c->support_angle.target_offset[i], 0.0f, recovery_step);
  }
}

static bool Pole_TemperatureThresholdEnabled(float threshold_c) {
  return isfinite(threshold_c) && threshold_c > 0.0f;
}

static void Pole_UpdateMotorTemperatureFlags(Pole_t *c, uint8_t idx) {
  if (c == NULL || c->param == NULL || idx >= POLE_MOTOR_NUM) return;

  const float temp_c = c->feedback.motor[idx].temp;
  const MOTOR_TemperatureProtectionConfig_t temperature_protection =
      MOTOR_NormalizeTemperatureProtection(c->param->motor_temperature_protection);
  const float warning_c = temperature_protection.warning_c;
  const float limit_c = temperature_protection.limit_c;
  const bool temp_valid = isfinite(temp_c);

  c->feedback.temperature_warning[idx] =
      temp_valid && Pole_TemperatureThresholdEnabled(warning_c) &&
      temp_c >= warning_c;
  c->feedback.temperature_over_limit[idx] =
      temp_valid && Pole_TemperatureThresholdEnabled(limit_c) &&
      temp_c >= limit_c;
}

bool Pole_HasTemperatureWarning(const Pole_t *c) {
  if (c == NULL) return false;
  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    if (c->feedback.temperature_warning[i]) return true;
  }
  return false;
}

bool Pole_HasTemperatureOverLimit(const Pole_t *c) {
  if (c == NULL) return false;
  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    if (c->feedback.temperature_over_limit[i]) return true;
  }
  return false;
}

float Pole_GetMaxTemperature(const Pole_t *c) {
  if (c == NULL) return 0.0f;
  float max_temp = 0.0f;
  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    const float temp_c = c->feedback.motor[i].temp;
    if (i == 0u || temp_c > max_temp) {
      max_temp = temp_c;
    }
  }
  return max_temp;
}

static float Pole_GetCurrentSideLift(const Pole_t *c, uint8_t side) {
  if (c == NULL || c->param == NULL || side >= 2u ||
      !c->support_angle.calibrated) {
    return 0.0f;
  }

  const uint8_t start = (side == 0u) ? 0u : 2u;
  float lift_sum = 0.0f;
  for (uint8_t i = start; i < start + 2u; i++) {
    lift_sum += Pole_GetSupportAngle(c, i) - c->support_angle.lower[i];
  }

  return mr::component::math::clamp_scalar(
      lift_sum * 0.5f, 0.0f, c->param->limit.support_total_travel);
}

static float Pole_GetTrackedSideBaseLift(const Pole_t *c, uint8_t side) {
  if (c == NULL || c->param == NULL || side >= 2u) {
    return 0.0f;
  }

  const uint8_t start = (side == 0u) ? 0u : 2u;
  float lift_sum = 0.0f;
  for (uint8_t i = start; i < start + 2u; i++) {
    lift_sum += c->support_angle.tracked_target_lift[i] -
                c->support_angle.target_offset[i];
  }
  return mr::component::math::clamp_scalar(
      lift_sum * 0.5f, 0.0f, c->param->limit.support_total_travel);
}

static void Pole_SyncSideTargetToFeedback(Pole_t *c, uint8_t side) {
  if (c == NULL || c->param == NULL || side >= 2u ||
      !c->support_angle.calibrated) {
    return;
  }

  const uint8_t start = (side == 0u) ? 0u : 2u;
  const float current_lift = Pole_GetCurrentSideLift(c, side);
  c->support_angle.final_target_lift[side] = current_lift;
  for (uint8_t i = start; i < start + 2u; i++) {
    const float motor_lift = mr::component::math::clamp_scalar(
        Pole_GetSupportAngle(c, i) - c->support_angle.lower[i], 0.0f,
        c->param->limit.support_total_travel);
    c->support_angle.tracked_target_lift[i] = motor_lift;
    c->support_angle.target_offset[i] = motor_lift - current_lift;
  }
  Pole_ResetSideTargetVelocity(c, side);
}

static void Pole_SyncTargetsToFeedback(Pole_t *c) {
  if (c == NULL || c->param == NULL || !c->support_angle.calibrated) {
    return;
  }

  float motor_lift[POLE_SUPPORT_MOTOR_NUM] = {0.0f};
  float lift_sum = 0.0f;
  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    motor_lift[i] = mr::component::math::clamp_scalar(
        Pole_GetSupportAngle(c, i) - c->support_angle.lower[i], 0.0f,
        c->param->limit.support_total_travel);
    lift_sum += motor_lift[i];
  }

  const float common_lift = mr::component::math::clamp_scalar(
      lift_sum / (float)POLE_SUPPORT_MOTOR_NUM, 0.0f,
      c->param->limit.support_total_travel);
  c->support_angle.final_target_lift[0] = common_lift;
  c->support_angle.final_target_lift[1] = common_lift;

  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    c->support_angle.tracked_target_lift[i] = motor_lift[i];
    c->support_angle.target_offset[i] = motor_lift[i] - common_lift;
    c->support_angle.tracked_target_velocity[i] = 0.0f;
  }
}

static int8_t Pole_SetMode(Pole_t *c, Pole_Mode_t mode) {
  if (c == NULL) return POLE_ERR_NULL;
  if (c->mode == mode) return POLE_OK;

  if (mode == POLE_MODE_ACTIVE) {
    Pole_SyncTargetsToFeedback(c);
    c->support_angle.manual_target_was_moving[0] = false;
    c->support_angle.manual_target_was_moving[1] = false;
  }
  Pole_ResetControllers(c);
  c->mode = mode;
  return POLE_OK;
}

int8_t Pole_Init(Pole_t *c, const Pole_Params_t *param, float target_freq) {
  if (c == NULL || param == NULL) return POLE_ERR_NULL;

  memset(c, 0, sizeof(Pole_t));
  BSP_CAN_Init();

  c->param = param;
  c->mode = POLE_MODE_RELAX;
  c->nominal_dt =
      mr::component::math::sanitize_dt(1.0f / target_freq, 0.001f, 0.0005f,
                                       0.010f);
  c->dt = c->nominal_dt;

  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    PID_Init(&c->pid.support_pos[i], KPID_MODE_CALC_D, target_freq,
             &param->pid.support_pos_pid);
    PID_Init(&c->pid.support_vel[i], KPID_MODE_NO_D, target_freq,
             &param->pid.support_vel_pid);
  }

  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    MOTOR_RM_Register((MOTOR_RM_Param_t *)&param->motor_param[i]);
  }

  return POLE_OK;
}

int8_t Pole_UpdateFeedback(Pole_t *c) {
  if (c == NULL || c->param == NULL) return POLE_ERR_NULL;

  float sum = 0.0f;
  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    MOTOR_RM_Update((MOTOR_RM_Param_t *)&c->param->motor_param[i]);
    c->motors[i] = MOTOR_RM_GetMotor((MOTOR_RM_Param_t *)&c->param->motor_param[i]);
    MOTOR_RM_t *motor = Pole_GetMotorHandle(c, i);
    if (motor == NULL) return POLE_ERR;

    c->feedback.motor[i] = motor->feedback;
    Pole_UpdateMotorTemperatureFlags(c, i);
    if (i < POLE_SUPPORT_MOTOR_NUM) {
      sum += Pole_GetSupportAngle(c, i);
    }
  }

  c->feedback.support_angle_avg = sum / (float)POLE_SUPPORT_MOTOR_NUM;
  c->feedback.support_lift[0] = Pole_GetCurrentSideLift(c, 0u);
  c->feedback.support_lift[1] = Pole_GetCurrentSideLift(c, 1u);
  return POLE_OK;
}

int8_t Pole_Control(Pole_t *c, const Pole_CMD_t *c_cmd, uint32_t now) {
  if (c == NULL || c_cmd == NULL || c->param == NULL) return POLE_ERR_NULL;

  const float raw_dt = (c->last_wakeup == 0U)
                           ? c->nominal_dt
                           : (float)(now - c->last_wakeup) / 1000.0f;
  c->last_wakeup = now;
  c->dt = mr::component::math::sanitize_dt(
      raw_dt, c->nominal_dt, c->nominal_dt * 0.5f, c->nominal_dt * 3.0f);

  c->setpoint.disable_lift_accel = c_cmd->disable_lift_accel;

  Pole_SetMode(c, c_cmd->mode);

  if (c->mode == POLE_MODE_RELAX) {
    for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
      c->out.motor[i] = 0.0f;
    }
    return POLE_OK;
  }

  if (!c->support_angle.calibrated) {
    for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
      float now_angle = Pole_GetSupportAngle(c, i);
      c->support_angle.lower[i] = now_angle;
      c->support_angle.upper[i] = now_angle + c->param->limit.support_total_travel;
    }
    c->support_angle.final_target_lift[0] = 0.0f;
    c->support_angle.final_target_lift[1] = 0.0f;
    for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
      c->support_angle.tracked_target_lift[i] = 0.0f;
      c->support_angle.tracked_target_velocity[i] = 0.0f;
      c->support_angle.target_offset[i] = 0.0f;
    }
    c->support_angle.auto_target_was_enabled[0] = false;
    c->support_angle.auto_target_was_enabled[1] = false;
    c->support_angle.manual_target_was_moving[0] = false;
    c->support_angle.manual_target_was_moving[1] = false;
    c->support_angle.calibrated = true;
    Pole_ResetControllers(c);
  }

  const bool synchronized_manual_cmd =
      !c_cmd->auto_target_enable[0] && !c_cmd->auto_target_enable[1] &&
      fabsf(c_cmd->lift[0] - c_cmd->lift[1]) <= kPoleManualLiftDeadband;
  const bool synchronized_manual_moving =
      synchronized_manual_cmd &&
      fabsf(c_cmd->lift[0]) > kPoleManualLiftDeadband;
  const bool synchronized_manual_start =
      synchronized_manual_moving &&
      !(c->support_angle.manual_target_was_moving[0] &&
        c->support_angle.manual_target_was_moving[1]);
  const bool synchronized_manual_stop =
      synchronized_manual_cmd &&
      fabsf(c_cmd->lift[0]) <= kPoleManualLiftDeadband &&
      (c->support_angle.manual_target_was_moving[0] ||
       c->support_angle.manual_target_was_moving[1]);
  const bool synchronized_manual_from_auto =
      synchronized_manual_cmd &&
      (c->support_angle.auto_target_was_enabled[0] ||
       c->support_angle.auto_target_was_enabled[1]);
  if (synchronized_manual_start || synchronized_manual_stop ||
      synchronized_manual_from_auto) {
    Pole_SyncTargetsToFeedback(c);
    c->support_angle.auto_target_was_enabled[0] = false;
    c->support_angle.auto_target_was_enabled[1] = false;
    c->support_angle.manual_target_was_moving[0] = false;
    c->support_angle.manual_target_was_moving[1] = false;
    Pole_ResetControllers(c);
  }

  for (uint8_t side = 0; side < 2u; side++) {
    float default_speed = c->param->limit.support_lift_speed;
    float auto_speed = c_cmd->auto_lift_speed[side];
    float default_accel = c->param->limit.support_lift_accel;
    float auto_accel = c_cmd->auto_lift_accel[side];
    const bool auto_target_enabled = c_cmd->auto_target_enable[side];
    float manual_target_delta = 0.0f;
    const bool bypass_target_limit =
        auto_target_enabled && auto_speed == 0.0f && auto_accel == 0.0f;
    float speed_limit =
        bypass_target_limit ? 0.0f
                            : ((auto_speed > 0.0f) ? auto_speed : default_speed);
    c->setpoint.lift_accel[side] =
        bypass_target_limit ? 0.0f
                            : ((auto_accel > 0.0f) ? auto_accel : default_accel);

    if (auto_target_enabled) {
      Pole_ClearSideTargetOffset(c, side);
      c->support_angle.final_target_lift[side] = c_cmd->auto_target_lift[side];
      c->support_angle.manual_target_was_moving[side] = false;
    } else {
      if (c->support_angle.auto_target_was_enabled[side]) {
        Pole_SyncSideTargetToFeedback(c, side);
      }
      const bool manual_target_moving =
          fabsf(c_cmd->lift[side]) > kPoleManualLiftDeadband;
      if (!manual_target_moving) {
        if (c->support_angle.manual_target_was_moving[side]) {
          Pole_SyncSideTargetToFeedback(c, side);
          Pole_ResetControllers(c);
        }
        c->support_angle.final_target_lift[side] =
            Pole_GetTrackedSideBaseLift(c, side);
      } else {
        const float manual_speed =
            (auto_speed > 0.0f) ? auto_speed : default_speed;
        if (manual_speed > 0.0f) {
          manual_target_delta = c_cmd->lift[side] * manual_speed * c->dt;
        } else {
          manual_target_delta = c_cmd->lift[side] *
                                c->param->limit.support_total_travel * c->dt;
        }
        c->support_angle.final_target_lift[side] += manual_target_delta;
        Pole_RecoverSideTargetOffset(c, side, manual_target_delta);
      }
      c->support_angle.manual_target_was_moving[side] = manual_target_moving;
    }
    c->support_angle.auto_target_was_enabled[side] = auto_target_enabled;

    const float min_target = mr::component::math::clamp_scalar(
      Pole_PositiveOrZero(c->param->limit.support_min_target_lift), 0.0f,
      c->param->limit.support_total_travel);
    c->support_angle.final_target_lift[side] = mr::component::math::clamp_scalar(
      c->support_angle.final_target_lift[side], min_target, c->param->limit.support_total_travel);
    const uint8_t start = (side == 0u) ? 0u : 2u;
    float tracked_lift_sum = 0.0f;
    float tracked_velocity_sum = 0.0f;
    for (uint8_t i = start; i < start + 2u; i++) {
      c->support_angle.tracked_target_lift[i] =
          Pole_UpdateTrackedLift(c, i, speed_limit);
      c->debug.tracked_motor_target_lift[i] =
          c->support_angle.tracked_target_lift[i];
      c->debug.tracked_motor_target_velocity[i] =
          c->support_angle.tracked_target_velocity[i];
      tracked_lift_sum += c->support_angle.tracked_target_lift[i] -
                          c->support_angle.target_offset[i];
      tracked_velocity_sum += c->support_angle.tracked_target_velocity[i];
    }
    c->debug.final_target_lift[side] = c->support_angle.final_target_lift[side];
    c->debug.tracked_target_lift[side] = tracked_lift_sum * 0.5f;
    c->debug.tracked_target_velocity[side] = tracked_velocity_sum * 0.5f;
  }

  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    float target = c->support_angle.lower[i] +
                   c->support_angle.tracked_target_lift[i];
    c->setpoint.support_target_angle[i] = mr::component::math::clamp_scalar(target, c->support_angle.lower[i],
                                                     c->support_angle.upper[i]);
  }

  float vel[4];
  float out[4];
  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    float fb_angle = Pole_GetSupportAngle(c, i);
    vel[i] = PID_Calc(&c->pid.support_pos[i],
                      c->setpoint.support_target_angle[i], fb_angle, 0.0f,
                      c->dt);
    out[i] = PID_Calc(&c->pid.support_vel[i], vel[i],
                        c->feedback.motor[i].rotor_speed, 0.0f, c->dt);
    c->out.motor[i] = mr::component::math::clamp_scalar(out[i], -c->param->limit.max_current,
                                 c->param->limit.max_current);
    if ((fb_angle <= c->support_angle.lower[i] + kPoleLiftLimitEpsilon &&
         c->out.motor[i] < 0.0f) ||
        (fb_angle >= c->support_angle.upper[i] - kPoleLiftLimitEpsilon &&
         c->out.motor[i] > 0.0f)) {
      c->out.motor[i] = 0.0f;
      PID_Reset(&c->pid.support_vel[i]);
    }
    c->debug.target_angle_rad[i] = c->setpoint.support_target_angle[i];
    c->debug.feedback_angle_rad[i] = fb_angle;
    c->debug.feedback_speed_rad_s[i] = c->feedback.motor[i].rotor_speed;
    c->debug.torque_cmd_nm[i] = c->out.motor[i];
  }

  return POLE_OK;
}

bool Pole_IsGroupAtTarget(const Pole_t *c, uint8_t group, float threshold_rad) {
  if (c == NULL || c->param == NULL) return false;
  if (group >= 2u) return false;
  if (!c->support_angle.calibrated) return false;

  float threshold = fabsf(threshold_rad);
  uint8_t start = (group == 0u) ? 0u : 2u;
  uint8_t end = start + 2u;
  for (uint8_t i = start; i < end; i++) {
    float fb_angle = Pole_GetSupportAngle(c, i);
    float err = c->setpoint.support_target_angle[i] - fb_angle;
    if (fabsf(err) > threshold) return false;
  }
  return true;
}

bool Pole_IsAllAtTarget(const Pole_t *c, float threshold_rad) {
  return Pole_IsGroupAtTarget(c, 0u, threshold_rad) &&
         Pole_IsGroupAtTarget(c, 1u, threshold_rad);
}

bool Pole_IsGroupAtFinalTarget(const Pole_t *c, uint8_t group,
                               float threshold_rad) {
  if (c == NULL || c->param == NULL) return false;
  if (group >= 2u) return false;
  if (!c->support_angle.calibrated) return false;

  float threshold = fabsf(threshold_rad);
  uint8_t start = (group == 0u) ? 0u : 2u;
  uint8_t end = start + 2u;
  for (uint8_t i = start; i < end; i++) {
    const uint8_t side = (i < 2u) ? 0u : 1u;
    const float target_lift = mr::component::math::clamp_scalar(
        c->support_angle.final_target_lift[side], 0.0f,
        c->param->limit.support_total_travel);
    const float target = c->support_angle.lower[i] + target_lift +
                         c->support_angle.target_offset[i];
    const float fb_angle = Pole_GetSupportAngle(c, i);
    if (fabsf(target - fb_angle) > threshold) return false;
  }
  return true;
}

bool Pole_IsAllAtFinalTarget(const Pole_t *c, float threshold_rad) {
  return Pole_IsGroupAtFinalTarget(c, 0u, threshold_rad) &&
         Pole_IsGroupAtFinalTarget(c, 1u, threshold_rad);
}

void Pole_Output(Pole_t *c) {
  if (c == NULL || c->param == NULL) return;

  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    MOTOR_RM_SetOutput((MOTOR_RM_Param_t *)&c->param->motor_param[i], c->out.motor[i]);
  }

  MOTOR_RM_Ctrl((MOTOR_RM_Param_t *)&c->param->motor_param[0]);
}

void Pole_ResetOutput(Pole_t *c) {
  if (c == NULL || c->param == NULL) return;

  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    MOTOR_RM_Relax((MOTOR_RM_Param_t *)&c->param->motor_param[i]);
    c->out.motor[i] = 0.0f;
  }

  MOTOR_RM_Ctrl((MOTOR_RM_Param_t *)&c->param->motor_param[0]);
}

void Pole_Power_Control(Pole_t *c, float max_power) {
  if (c == NULL || c->param == NULL) return;

  float limit = mr::component::math::clamp_scalar(max_power, 0.0f, c->param->limit.max_current);
  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    c->out.motor[i] = mr::component::math::clamp_scalar(c->out.motor[i], -limit, limit);
  }
}

