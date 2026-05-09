/*
 * Robot configuration.
 */

#pragma once

#include <stdint.h>

#include "component/pid.h"
#include "device/motor.h"
#include "device/motor_rm.h"
#include "module/arm/arm_control_types.h"
#include "module/chassis.h"
#include "module/pole.h"
#include "module/rod.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float prealign_kp;          /* yaw error to wz gain */
  float prealign_wz_limit;    /* yaw correction limit, rad/s */
  float flat_move_speed;      /* default simple flat move speed, m/s */
  uint32_t flat_move_hold_ms; /* default simple flat move hold time, ms */

  float sick_valid_min_cm;       /* valid SICK distance lower bound, cm */
  float sick_valid_max_cm;       /* valid SICK distance upper bound, cm */
  float sick_norm_err_deadband;  /* SICK normalized left-right deadband */
  float sick_norm_err_to_rad;    /* SICK normalized error to yaw assist, rad */
  float sick_assist_max_rad;     /* SICK assist clamp, rad */
  float sick_assist_gain;        /* SICK assist gain */
} AutoCtrl_CommonParam_t;

typedef struct {
  float prealign_move_speed;     /* PREALIGN added vx, m/s */
  float align_move_speed;        /* template align step vx, m/s */
  float pole_extend_move_speed;  /* move speed while extending poles, m/s */
  uint32_t pole_extend_settle_ms;/* wait after all poles extend, ms */

  float front_retract_move_speed;     /* move speed while front poles retract/extend, m/s */
  float front_retract_vy;             /* optional vy while waiting front photo, m/s */
  uint32_t front_retract_settle_ms;   /* front pole action settle time, ms */
  uint32_t front_retract_timeout_ms;  /* front pole/photo timeout, ms */

  float mid_move_speed;       /* middle move speed, m/s */
  uint32_t mid_move_ms;       /* middle move time, ms */

  float rear_retract_move_speed;     /* move speed while rear poles retract/extend, m/s */
  float rear_retract_vy;             /* optional vy while waiting rear photo, m/s */
  uint32_t rear_retract_timeout_ms;  /* rear pole/photo timeout, ms */
  uint32_t rear_retract_move_ms;     /* rear action move time, ms */

  float final_move_speed;     /* final leave/flat move speed, m/s */
  uint32_t final_move_ms;     /* final leave/flat move time, ms */

  float pole_all_extend_speed;    /* all poles extend speed, rad/s */
  float pole_front_extend_speed;  /* front poles extend speed, rad/s */
  float pole_front_retract_speed; /* front poles retract speed, rad/s */
  float pole_rear_extend_speed;   /* rear poles extend speed, rad/s */
  float pole_rear_retract_speed;  /* rear poles retract speed, rad/s */

  uint32_t front_photo_timeout_ms; /* current forward photo timeout, ms */
  uint32_t rear_photo_timeout_ms;  /* current rear photo timeout, ms */
  uint32_t hold_ms;                /* simple fallback hold time, ms */
} AutoCtrl_TemplateParam_t;

typedef struct {
  AutoCtrl_CommonParam_t common;

  AutoCtrl_TemplateParam_t head_ascend_200;
  AutoCtrl_TemplateParam_t head_ascend_400;
  AutoCtrl_TemplateParam_t head_descend_200;
  AutoCtrl_TemplateParam_t head_descend_400;

  AutoCtrl_TemplateParam_t tail_ascend_200;
  AutoCtrl_TemplateParam_t tail_ascend_400;
  AutoCtrl_TemplateParam_t tail_descend_200;
  AutoCtrl_TemplateParam_t tail_descend_400;
} AutoCtrl_Params_t;

typedef struct {
  Chassis_Params_t chassis_param;
  Pole_Params_t pole_param;
  Arm_Params_t arm_param;
  AutoCtrl_Params_t auto_ctrl_param;
  Rod_Params_t rod_param;
} Config_RobotParam_t;

Config_RobotParam_t *Config_GetRobotParam(void);

#ifdef __cplusplus
}
#endif
