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
#include "module/ore_store.h"
#include "module/pole.h"
#include "module/rod_new.h"
#include "module/camera_yaw.h"
#include "module/arm_simple.h"
#include "module/autoCtrlAPI/ore_store/auto_ore_store.h"
#include "module/autoCtrlAPI/rod/auto_rod_spearhead.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float prealign_kp;          /* yaw error to wz gain */
  float prealign_wz_limit;    /* yaw correction limit, rad/s */
 
  float sick_valid_min_cm;       /* valid SICK distance lower bound, cm */
  float sick_valid_max_cm;       /* valid SICK distance upper bound, cm */
  float sick_norm_err_deadband;  /* SICK normalized left-right deadband */
  float sick_norm_err_to_rad;    /* SICK normalized error to yaw assist, rad */
  float sick_assist_max_rad;     /* SICK assist clamp, rad */
  float sick_assist_gain;        /* SICK assist gain */
} AutoCtrl_CommonParam_t;

typedef struct {
  float prealign_move_speed;     /* PREALIGN added vx, m/s */
  float align_move_speed;        /* ascend/template alignment vx, m/s */
  float pole_extend_move_speed;  /* vx for all-pole extend/support-pass stages, m/s */
  uint32_t pole_extend_settle_ms;/* ascend all-pole target minimum settle time, ms */

  float front_retract_move_speed;     /* vx for front-side action or second-edge slow capture, m/s */
  float front_retract_vy;             /* optional vy while waiting front-side photo, m/s */
  uint32_t front_retract_timeout_ms;  /* front-side action/photo timeout, ms */

  float mid_move_speed;       /* fixed fast-approach/middle move speed, m/s */
  uint32_t mid_move_ms;       /* first fixed fast-approach/middle move time, ms */

  float rear_retract_move_speed;     /* vx for rear-side action or first-edge slow capture, m/s */
  uint32_t rear_retract_timeout_ms;  /* rear-side action/photo timeout, ms */
  uint32_t rear_retract_move_ms;     /* second fixed fast-approach/rear action move time, ms */

  float second_photo_retract_move_speed; /* optional vx magnitude when retracting after the second photo, m/s */
  float final_move_speed;     /* final leave speed, or fallback for second-photo retract vx, m/s */
  uint32_t final_move_ms;     /* final/all-retract leave time, ms */

  float pole_all_extend_speed;    /* all poles extend speed, rad/s */
  float pole_front_extend_speed;  /* front poles extend speed, rad/s */
  float pole_front_retract_speed; /* front poles retract speed, rad/s */
  float pole_rear_extend_speed;   /* rear poles extend speed, rad/s */
  float pole_rear_retract_speed;  /* rear poles retract speed, rad/s */
  float pole_lift_accel;          /* rad/s^2; >0 limit, 0 Pole default, <0 disable */

  uint32_t front_photo_timeout_ms; /* current forward photo timeout, ms */
  uint32_t rear_photo_timeout_ms;  /* current rear photo timeout, ms */
  uint32_t hold_ms;                /* descend support-pass hold time, ms */
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
  OreStore_Params_t ore_store_param;
  Arm_Params_t arm_param;
  ArmSimple_Params_t arm_simple_param;
  AutoOre_Params_t auto_ore_param;
  AutoRodSpearhead_Params_t auto_rod_spearhead_param;
  AutoCtrl_Params_t auto_ctrl_param;
  RodNew_Params_t rod_new_param;
  CameraYaw_Params_t camera_yaw_param;
} Config_RobotParam_t;

Config_RobotParam_t *Config_GetRobotParam(void);

#ifdef __cplusplus
}
#endif
