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
#include "module/autoCtrlAPI/sick/auto_sick_correct.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float prealign_kp;          /* yaw 误差到 wz 的增益。 */
  float prealign_wz_limit;    /* yaw 修正角速度限幅，单位 rad/s。 */
 
  float sick_valid_min_cm;       /* SICK 有效距离下限，单位 cm。 */
  float sick_valid_max_cm;       /* SICK 有效距离上限，单位 cm。 */
  float sick_norm_err_deadband;  /* SICK 左右归一化误差死区。 */
  float sick_norm_err_to_rad;    /* SICK 归一化误差到 yaw 辅助量的比例，单位 rad。 */
  float sick_assist_max_rad;     /* SICK yaw 辅助量限幅，单位 rad。 */
  float sick_assist_gain;        /* SICK 辅助修正增益。 */
  uint16_t sick_front_ore_adc_min; /* 前 SICK 矿检测 ADC 下限。 */
  uint16_t sick_front_ore_adc_max; /* 前 SICK 矿检测 ADC 上限。 */
  uint32_t sick_front_ore_stable_ms; /* 前 SICK 矿检测稳定确认时间，单位 ms。 */
} AutoCtrl_CommonParam_t;

#define AUTO_CTRL_POLE_SPEED_SEGMENT_COUNT (3u)

typedef struct {
  float height_rad;   /* 本段切换高度阈值，单位 rad；<=0 表示该段关闭。 */
  float speed_rad_s;  /* 本段撑杆速度，单位 rad/s；<=0 表示沿用旧单段速度。 */
} AutoCtrl_PoleSpeedSegment_t;

typedef struct {
  AutoCtrl_PoleSpeedSegment_t front[AUTO_CTRL_POLE_SPEED_SEGMENT_COUNT];
  AutoCtrl_PoleSpeedSegment_t rear[AUTO_CTRL_POLE_SPEED_SEGMENT_COUNT];
} AutoCtrl_PoleSpeedProfile_t;

typedef struct {
  float prealign_move_speed;     /* PREALIGN 阶段叠加 vx，单位 m/s。 */
  float align_move_speed;        /* 上台阶/模板对正阶段 vx，单位 m/s。 */
  float pole_extend_move_speed;  /* 四杆全伸/支撑通过阶段 vx，单位 m/s。 */
  uint32_t pole_extend_settle_ms;/* 上台阶四杆全伸到位后的最小稳定时间，单位 ms。 */
  uint32_t photo_stop_settle_ms; /* 下台阶光电触发停车后、伸杆前的保持时间，单位 ms。 */
  float descend_start_pole_lift_threshold; /* 下台阶首次冲刺前的最小撑杆高度，单位 rad；<=0 禁用。 */

  float front_retract_move_speed;     /* 前侧动作或第二个边沿慢速捕获 vx，单位 m/s。 */
  float front_retract_vy;             /* 等待前侧光电时可选的 vy，单位 m/s。 */
  uint32_t front_retract_timeout_ms;  /* 前侧动作/光电等待超时，单位 ms。 */

  float mid_move_speed;       /* 固定快速靠近/中段移动速度，单位 m/s。 */
  uint32_t mid_move_ms;       /* 第一段固定快速靠近/中段移动时间，单位 ms；角度阈值>0时作为兜底超时。 */
  float mid_move_wheel_delta_rad;  /* 第一段固定快速靠近/中段移动轮转角阈值，单位 rad；>0 优先按角度切步，<=0 使用 mid_move_ms 定时切步。 */
  float timed_move_yaw_tolerance_rad; /* 中段移动切步的 yaw 容差，单位 rad。 */

  float rear_retract_move_speed;     /* 后侧动作或第一个边沿慢速捕获 vx，单位 m/s。 */
  uint32_t rear_retract_timeout_ms;  /* 后侧动作/光电等待超时，单位 ms。 */
  uint32_t rear_retract_move_ms;     /* 第二段固定快速靠近/后侧动作移动时间，单位 ms；角度阈值>0时作为兜底超时。 */
  float rear_retract_move_wheel_delta_rad; /* 第二段固定快速靠近/后侧动作移动轮转角阈值，单位 rad；>0 优先按角度切步，<=0 使用 rear_retract_move_ms 定时切步。 */

  float second_photo_retract_move_speed; /* 第二个光电触发后收尾移动的可选 vx 幅值，单位 m/s。 */
  float final_move_speed;     /* 收尾离开速度，或第二光电后收尾 vx 的备用值，单位 m/s。 */
  uint32_t final_move_ms;     /* 收尾离开时间，单位 ms；角度阈值>0时作为兜底超时。 */
  uint32_t final_photo_sprint_ms; /* 上台阶末尾光电稳定触发后的额外冲刺时间，单位 ms；0 表示触发即结束。 */
  float final_move_wheel_delta_rad; /* 收尾离开轮转角阈值，单位 rad；>0 优先按角度切步，<=0 使用 final_move_ms 定时切步。 */

  float pole_all_extend_speed;    /* 四杆全伸速度，单位 rad/s。 */
  float pole_front_extend_speed;  /* 前杆伸出速度，单位 rad/s。 */
  float pole_front_retract_speed; /* 前杆回收速度，单位 rad/s。 */
  float pole_rear_extend_speed;   /* 后杆伸出速度，单位 rad/s。 */
  float pole_rear_retract_speed;  /* 后杆回收速度，单位 rad/s。 */
  float pole_lift_accel;          /* 撑杆加速度限制，单位 rad/s^2；>0 限幅，0 使用 Pole 默认值，<0 禁用。 */

  AutoCtrl_PoleSpeedProfile_t pole_all_extend_profile;    /* 四杆全伸三段速度。 */
  AutoCtrl_PoleSpeedProfile_t pole_front_extend_profile;  /* 前杆伸出三段速度。 */
  AutoCtrl_PoleSpeedProfile_t pole_front_retract_profile; /* 前杆回收三段速度。 */
  AutoCtrl_PoleSpeedProfile_t pole_rear_extend_profile;   /* 后杆伸出三段速度。 */
  AutoCtrl_PoleSpeedProfile_t pole_rear_retract_profile;  /* 后杆回收三段速度。 */

  uint32_t front_photo_timeout_ms; /* 当前前侧光电等待超时，单位 ms。 */
  uint32_t rear_photo_timeout_ms;  /* 当前后侧光电等待超时，单位 ms。 */
  uint32_t hold_ms;                /* 下台阶支撑通过保持时间，单位 ms。 */
} AutoCtrl_TemplateParam_t;

typedef struct {
  AutoCtrl_CommonParam_t common;
  AutoSickCorrect_Params_t sick_correct;

  AutoCtrl_TemplateParam_t head_ascend_200;
  AutoCtrl_TemplateParam_t head_ascend_400;
  AutoCtrl_TemplateParam_t head_descend_200;
  AutoCtrl_TemplateParam_t head_descend_400;
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
  CameraYaw_Params_t camera_yaw_param[CAMERA_YAW_NUM];
} Config_RobotParam_t;

Config_RobotParam_t *Config_GetRobotParam(void);

#ifdef __cplusplus
}
#endif
