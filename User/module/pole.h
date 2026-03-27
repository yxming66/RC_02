/*
 * Pole module: 4x3508 support motors + 2x2006 drive-wheel motors.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "component/pid.h"
#include "device/motor_rm.h"

#define POLE_OK (0)
#define POLE_ERR (-1)
#define POLE_ERR_NULL (-2)

#define POLE_SUPPORT_MOTOR_NUM (4)
#define POLE_DRIVE_MOTOR_NUM (2)
#define POLE_MOTOR_NUM (POLE_SUPPORT_MOTOR_NUM + POLE_DRIVE_MOTOR_NUM)

typedef enum {
  POLE_MODE_RELAX = 0,
  POLE_MODE_ACTIVE,
} Pole_Mode_t;

typedef struct {
  Pole_Mode_t mode;
  float lift[2];   /* [-1, 1], >0 means up */
  float drive;  /* [-1, 1], drive wheel speed command */
} Pole_CMD_t;

typedef struct {
  MOTOR_RM_Param_t motor_param[POLE_MOTOR_NUM];
  struct {
    KPID_Params_t support_pos_pid;
    KPID_Params_t support_vel_pid;
    KPID_Params_t drive_spd_pid;
  } pid;
  struct {
    float max_current;
    float support_total_travel;   /* rad */
    float support_lift_speed;     /* rad/s */
    float drive_enable_angle;     /* rad, from lower limit */
    float drive_max_rpm;
  } limit;
} Pole_Params_t;

typedef struct {
  MOTOR_Feedback_t motor[POLE_MOTOR_NUM];
  float support_angle_avg;
} Pole_Feedback_t;

typedef struct {
  float motor[POLE_MOTOR_NUM];
} Pole_Output_t;

typedef struct {
  uint32_t last_wakeup;
  float dt;

  const Pole_Params_t *param;
  Pole_Mode_t mode;
  MOTOR_RM_t *motors[POLE_MOTOR_NUM];

  struct {
    bool calibrated;
    float lower[POLE_SUPPORT_MOTOR_NUM];
    float upper[POLE_SUPPORT_MOTOR_NUM];
    float target_lift[2];
  } support_angle;

  struct {
    float support_target_angle[POLE_SUPPORT_MOTOR_NUM];
    float drive_target_rpm[POLE_DRIVE_MOTOR_NUM];
  } setpoint;

  struct {
    KPID_t support_pos[POLE_SUPPORT_MOTOR_NUM];
    KPID_t support_vel[POLE_SUPPORT_MOTOR_NUM];
    KPID_t drive_spd[POLE_DRIVE_MOTOR_NUM];
  } pid;

  Pole_Output_t out;
  Pole_Feedback_t feedback;
} Pole_t;

int8_t Pole_Init(Pole_t *c, const Pole_Params_t *param, float target_freq);
int8_t Pole_UpdateFeedback(Pole_t *c);
int8_t Pole_Control(Pole_t *c, const Pole_CMD_t *c_cmd, uint32_t now);
void Pole_Output(Pole_t *c);
void Pole_ResetOutput(Pole_t *c);
void Pole_Power_Control(Pole_t *c, float max_power);

#ifdef __cplusplus
}
#endif




