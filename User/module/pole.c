/*
 * Pole module: 4x3508 support motors.
 */

#include "module/pole.h"

#include <math.h>
#include <string.h>

#include "bsp/can.h"
#include "device/motor_rm.h"

static float Pole_GetSupportAngle(const Pole_t *c, uint8_t idx) {
  float angle = c->motors[idx]->gearbox_total_angle;
  if (c->param->motor_param[idx].reverse) {
    angle = -angle;
  }
  return angle;
}

static float Pole_Clipf(float val, float min, float max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

static float Pole_MoveTowards(float current, float target, float max_step) {
  if (max_step <= 0.0f) return target;

  float delta = target - current;
  if (delta > max_step) return current + max_step;
  if (delta < -max_step) return current - max_step;
  return target;
}

static void Pole_ResetControllers(Pole_t *c) {
  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    PID_Reset(&c->pid.support_pos[i]);
    PID_Reset(&c->pid.support_vel[i]);
  }
}

static int8_t Pole_SetMode(Pole_t *c, Pole_Mode_t mode) {
  if (c == NULL) return POLE_ERR_NULL;
  if (c->mode == mode) return POLE_OK;

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
    if (c->motors[i] == NULL) return POLE_ERR;

    c->feedback.motor[i] = c->motors[i]->feedback;
    if (i < POLE_SUPPORT_MOTOR_NUM) {
      sum += Pole_GetSupportAngle(c, i);
    }
  }

  c->feedback.support_angle_avg = sum / (float)POLE_SUPPORT_MOTOR_NUM;
  return POLE_OK;
}

int8_t Pole_Control(Pole_t *c, const Pole_CMD_t *c_cmd, uint32_t now) {
  if (c == NULL || c_cmd == NULL || c->param == NULL) return POLE_ERR_NULL;

  c->dt = (float)(now - c->last_wakeup) / 1000.0f;
  c->last_wakeup = now;
  if (!isfinite(c->dt) || c->dt <= 0.0f) c->dt = 0.001f;
  if (c->dt < 0.0005f) c->dt = 0.0005f;
  if (c->dt > 0.050f) c->dt = 0.050f;

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
    c->support_angle.tracked_target_lift[0] = 0.0f;
    c->support_angle.tracked_target_lift[1] = 0.0f;
    c->support_angle.calibrated = true;
    Pole_ResetControllers(c);
  }

  for (uint8_t side = 0; side < 2u; side++) {
    float default_speed = c->param->limit.support_lift_speed;
    float auto_speed = c_cmd->auto_lift_speed[side];
    float speed_limit = (auto_speed > 0.0f) ? auto_speed : default_speed;

    if (c_cmd->auto_target_enable[side]) {
      c->support_angle.final_target_lift[side] = c_cmd->auto_target_lift[side];
    } else {
      c->support_angle.final_target_lift[side] += c_cmd->lift[side] * default_speed * c->dt;
    }

    c->support_angle.final_target_lift[side] = Pole_Clipf(
        c->support_angle.final_target_lift[side], 0.0f, c->param->limit.support_total_travel);
    c->support_angle.tracked_target_lift[side] = Pole_MoveTowards(
        c->support_angle.tracked_target_lift[side], c->support_angle.final_target_lift[side],
        speed_limit * c->dt);
    c->support_angle.tracked_target_lift[side] = Pole_Clipf(
        c->support_angle.tracked_target_lift[side], 0.0f, c->param->limit.support_total_travel);
  }

  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    uint8_t side = (i < 2u) ? 0u : 1u;
    float target = c->support_angle.lower[i] + c->support_angle.tracked_target_lift[side];
    c->setpoint.support_target_angle[i] = Pole_Clipf(target, c->support_angle.lower[i],
                                                     c->support_angle.upper[i]);
  }

  float vel[4];
  float out[4];
  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    float fb_angle = Pole_GetSupportAngle(c, i);
     vel[i] = PID_Calc(&c->pid.support_pos[i], c->setpoint.support_target_angle[i],
                         fb_angle, 0.0f, c->dt);
     out[i] = PID_Calc(&c->pid.support_vel[i], vel[i], 
                        c->feedback.motor[i].rotor_speed, 0.0f, c->dt);
    c->out.motor[i] = Pole_Clipf(out[i], -c->param->limit.max_current,
                                 c->param->limit.max_current);
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

void Pole_Output(Pole_t *c) {
  if (c == NULL || c->param == NULL) return;

  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    MOTOR_RM_SetOutput((MOTOR_RM_Param_t *)&c->param->motor_param[i], c->out.motor[i]);
  }

  MOTOR_RM_Ctrl(&c->param->motor_param[0]);
}

void Pole_ResetOutput(Pole_t *c) {
  if (c == NULL || c->param == NULL) return;

  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    MOTOR_RM_Relax((MOTOR_RM_Param_t *)&c->param->motor_param[i]);
    c->out.motor[i] = 0.0f;
  }
}

void Pole_Power_Control(Pole_t *c, float max_power) {
  if (c == NULL || c->param == NULL) return;

  float limit = Pole_Clipf(max_power, 0.0f, c->param->limit.max_current);
  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    c->out.motor[i] = Pole_Clipf(c->out.motor[i], -limit, limit);
  }
}

