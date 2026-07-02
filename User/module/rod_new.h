/*
 * RodNew module: Servo pitch + pneumatic gripper.
 * @note 新版取矛头机构：舵机控制pitch角度，IO控制气动夹爪
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "bsp/pwm.h"
#include "bsp/gpio.h"
#include "component/pid.h"
#include "component/filter.h"

#define ROD_NEW_OK (0)
#define ROD_NEW_ERR (-1)
#define ROD_NEW_ERR_NULL (-2)

/* 舵机PWM参数 ----------------------------------------------------------- */
#define ROD_NEW_SERVO_PULSE_MIN_US 766U    /* 水平位脉宽 */
#define ROD_NEW_SERVO_PULSE_MAX_US 1544U   /* 等待位脉宽 */
#define ROD_NEW_SERVO_PULSE_NEUTRAL_US ROD_NEW_SERVO_PULSE_MIN_US
#define ROD_NEW_SERVO_DEADBAND_US 1U        /* 死区 */
#define ROD_NEW_SERVO_DEFAULT_FREQ_HZ 50U   /* 默认频率 50Hz */

#ifndef ROD_NEW_GRIPPER_OPEN_LEVEL
#define ROD_NEW_GRIPPER_OPEN_LEVEL (0u)
#endif

typedef enum {
  ROD_NEW_MODE_RELAX = 0,
  ROD_NEW_MODE_ACTIVE,
} RodNew_Mode_t;

typedef enum {
  ROD_NEW_GRIP_RELEASE = 0, /* 打开 */
  ROD_NEW_GRIP_GRAB,        /* 关闭 */
} RodNew_GripState_t;

typedef enum {
  ROD_NEW_POSE_STANDBY = 0,  /* 放平位 */
  ROD_NEW_POSE_GRAB_HIGH,     /* 高位夹取 */
  ROD_NEW_POSE_DETECT,        /* 识别检测位置 */
  ROD_NEW_POSE_DOCK_WAIT,     /* 等待对接位置 */
  ROD_NEW_POSE_MANUAL,        /* 手动角度 */
} RodNew_Pose_t;

typedef struct {
  /* PWM输出 */
  BSP_PWM_Channel_t pwm_channel;
  float freq_hz;
  uint32_t zero_pulse_us; /* 保留字段，当前端点线性映射不使用 */

  /* 角度参数（弧度） */
  float angle_standby_rad;
  float angle_grab_high_rad;
  float angle_detect_rad;
  float angle_dock_wait_rad;

  /* 角度限幅 */
  float angle_min_rad;
  float angle_max_rad;

  /* 角度阈值（到位判定） */
  float arrive_threshold_rad;

  /* 速度/加速度限制 */
  float max_vel_rad_s;
  float max_acc_rad_s;
} RodNew_ServoParams_t;

typedef struct {
  /* IO引脚（后续在gpio.h中配置具体引脚） */
  BSP_GPIO_t gripper_gpio;

  /* 夹取超时 */
  uint32_t grip_timeout_ms;
} RodNew_GripperParams_t;

typedef struct {
  RodNew_ServoParams_t servo;
  RodNew_GripperParams_t gripper;
} RodNew_Params_t;

typedef struct {
  RodNew_Mode_t mode;
  RodNew_Pose_t pose;
  RodNew_GripState_t grip;
  float target_angle_rad;
} RodNew_CMD_t;

typedef struct {
  volatile bool enable;
  volatile bool direct_pulse_enable;
  volatile BSP_PWM_Channel_t pwm_channel;
  volatile float target_angle_rad;
  volatile uint32_t pulse_us;
  volatile RodNew_GripState_t grip;
} RodNew_DebugControl_t;

typedef struct {
  RodNew_Mode_t mode;
  RodNew_Pose_t pose;
  RodNew_GripState_t grip;
  float target_angle_rad;
  float tracked_angle_rad;
  float tracked_velocity_rad_s;
  float feedback_angle_rad;
  bool at_target;
} RodNew_Feedback_t;

typedef struct {
  float target_angle_rad;   /* 目标角度（弧度） */
  float tracked_angle_rad;   /* 跟踪目标（带速度限幅） */
  float tracked_vel_rad_s;   /* 跟踪速度 */
  float feedback_angle_rad; /* 反馈角度 */
  bool at_target;
} RodNew_ServoState_t;

typedef struct {
  RodNew_GripState_t state;
  bool grip_done;
  uint32_t grip_start_tick;
  bool timed_out;
} RodNew_GripperState_t;

typedef struct {
  uint32_t last_wakeup;
  float dt;
  float nominal_dt;

  const RodNew_Params_t *param;
  RodNew_Mode_t mode;

  RodNew_ServoState_t servo;
  RodNew_GripperState_t gripper;

  uint32_t now_tick;
} RodNew_t;

/* Exported functions prototypes -------------------------------------------- */
int8_t RodNew_Init(RodNew_t *r, const RodNew_Params_t *param,
                   float control_freq_hz);
int8_t RodNew_Control(RodNew_t *r, RodNew_Mode_t mode, RodNew_Pose_t pose,
                      RodNew_GripState_t grip, float target_angle_rad,
                      uint32_t now);
void RodNew_Output(RodNew_t *r);
void RodNew_Reset(RodNew_t *r);

/* 辅助函数 */
float RodNew_AngleToPulseUs(float angle_rad, const RodNew_ServoParams_t *param);
bool RodNew_IsAtTarget(const RodNew_t *r);
bool RodNew_IsGripDone(const RodNew_t *r);

extern volatile RodNew_DebugControl_t g_rod_new_debug;

#ifdef __cplusplus
}
#endif
