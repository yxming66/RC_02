/*
 * Pole module: 4x3508 support motors + 2x2006 drive-wheel motors.
 */

#include "module/pole.h"

#include <math.h>
#include <string.h>

#include "bsp/can.h"

#define POLE_DRIVE_ATTACH_SUPPORT_0 (2u) /* 2006[0] on support #3 */
#define POLE_DRIVE_ATTACH_SUPPORT_1 (3u) /* 2006[1] on support #4 */

static float Pole_Clipf(float val, float min, float max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

static void Pole_ResetControllers(Pole_t *c) {
  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    PID_Reset(&c->pid.support_pos[i]);
  }
  for (uint8_t i = 0; i < POLE_DRIVE_MOTOR_NUM; i++) {
    PID_Reset(&c->pid.drive_spd[i]);
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
    PID_Init(&c->pid.support_pos[i], KPID_MODE_NO_D, target_freq,
             &param->pid.support_pos_pid);
  }

  for (uint8_t i = 0; i < POLE_DRIVE_MOTOR_NUM; i++) {
    PID_Init(&c->pid.drive_spd[i], KPID_MODE_NO_D, target_freq,
             &param->pid.drive_spd_pid);
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
      sum += c->motors[i]->gearbox_total_angle;
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
    c->support_angle.lower = c->feedback.support_angle_avg;
    c->support_angle.upper = c->support_angle.lower + c->param->limit.support_total_travel;
    c->support_angle.target = c->support_angle.lower;
    c->support_angle.calibrated = true;
    Pole_ResetControllers(c);
  }

  c->support_angle.target += c_cmd->lift * c->param->limit.support_lift_speed * c->dt;
  c->support_angle.target = Pole_Clipf(c->support_angle.target, c->support_angle.lower,
                                       c->support_angle.upper);
  c->setpoint.support_target_angle = c->support_angle.target;

  const uint8_t attach_support_idx[POLE_DRIVE_MOTOR_NUM] = {
      POLE_DRIVE_ATTACH_SUPPORT_0,
      POLE_DRIVE_ATTACH_SUPPORT_1,
  };
  for (uint8_t i = 0; i < POLE_DRIVE_MOTOR_NUM; i++) {
    uint8_t support_idx = attach_support_idx[i];
    float lifted = c->motors[support_idx]->gearbox_total_angle - c->support_angle.lower;
    if (lifted >= c->param->limit.drive_enable_angle) {
      c->setpoint.drive_target_rpm[i] = c_cmd->drive * c->param->limit.drive_max_rpm;
    } else {
      c->setpoint.drive_target_rpm[i] = 0.0f;
    }
  }

  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    float fb_angle = c->motors[i]->gearbox_total_angle;
    float out = PID_Calc(&c->pid.support_pos[i], c->setpoint.support_target_angle,
                         fb_angle, 0.0f, c->dt);
    c->out.motor[i] = Pole_Clipf(out, -c->param->limit.max_current,
                                 c->param->limit.max_current);
  }

  for (uint8_t i = 0; i < POLE_DRIVE_MOTOR_NUM; i++) {
    uint8_t idx = (uint8_t)(POLE_SUPPORT_MOTOR_NUM + i);
    float fb_rpm = c->feedback.motor[idx].rotor_speed;
    float out = PID_Calc(&c->pid.drive_spd[i], c->setpoint.drive_target_rpm[i],
                         fb_rpm, 0.0f, c->dt);
    c->out.motor[idx] = Pole_Clipf(out, -c->param->limit.max_current,
                                   c->param->limit.max_current);
  }

  return POLE_OK;
}

void Pole_Output(Pole_t *c) {
  if (c == NULL || c->param == NULL) return;

  bool sent_can1 = false;
  bool sent_can2 = false;

  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    MOTOR_RM_SetOutput((MOTOR_RM_Param_t *)&c->param->motor_param[i], c->out.motor[i]);
  }

  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    BSP_CAN_t can = c->param->motor_param[i].can;
    if (can == BSP_CAN_1) {
      if (!sent_can1) {
        MOTOR_RM_Ctrl((MOTOR_RM_Param_t *)&c->param->motor_param[i]);
        sent_can1 = true;
      }
    } else if (can == BSP_CAN_2) {
      if (!sent_can2) {
        MOTOR_RM_Ctrl((MOTOR_RM_Param_t *)&c->param->motor_param[i]);
        sent_can2 = true;
      }
    }
  }
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

