/*
 * Pole auto step-climb state machine.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "module/pole.h"

#define POLE_AUTO_OK (0)
#define POLE_AUTO_ERR (-1)
#define POLE_AUTO_ERR_NULL (-2)

typedef enum {
  POLE_AUTO_STEP_NONE = 0,
  POLE_AUTO_STEP_200MM,
  POLE_AUTO_STEP_400MM,
} Pole_AutoStepType_t;

typedef enum {
  POLE_AUTO_STAGE_IDLE = 0,
  POLE_AUTO_STAGE_LIFT_ALL,
  POLE_AUTO_STAGE_WAIT_FRONT_TRIGGER,
  POLE_AUTO_STAGE_RETRACT_FRONT,
  POLE_AUTO_STAGE_WAIT_REAR_TRIGGER,
  POLE_AUTO_STAGE_RETRACT_REAR,
  POLE_AUTO_STAGE_FINISHED,
  POLE_AUTO_STAGE_ERROR,
} Pole_AutoStage_t;

typedef struct {
  float distance_m[4];   /* 0:front-left 1:front-right 2:rear-left 3:rear-right */
  bool valid[4];
} Pole_AutoSensor_t;

typedef struct {
  bool enable;
  bool restart;
  Pole_AutoStepType_t step_type;
} Pole_AutoCmd_t;

typedef struct {
  float lift_target_front;         /* 前撑杆撑起目标角，单位 rad，相对下限零位 */
  float lift_target_rear;          /* 后撑杆撑起目标角，单位 rad，相对下限零位 */
  float retract_front_target;      /* 前撑杆收回目标角，单位 rad，相对下限零位 */
  float retract_rear_target;       /* 后撑杆收回目标角，单位 rad，相对下限零位 */
  float lift_speed_front;          /* 前撑杆撑起过程的目标变化速度上限，单位 rad/s */
  float lift_speed_rear;           /* 后撑杆撑起过程的目标变化速度上限，单位 rad/s */
  float retract_speed_front;       /* 前撑杆收回过程的目标变化速度上限，单位 rad/s */
  float retract_speed_rear;        /* 后撑杆收回过程的目标变化速度上限，单位 rad/s */
  float front_trigger_threshold_m; /* 前侧两路测距触发阈值，单位 m，满足后允许收前撑杆 */
  float rear_trigger_threshold_m;  /* 后侧两路测距触发阈值，单位 m，满足后允许收后撑杆 */
  float trigger_hold_time_s;       /* 测距连续满足触发阈值的保持时间，单位 s，用于防抖 */
  float stage_timeout_s;           /* 单个状态阶段允许的最长持续时间，单位 s，超时进入错误态 */
  float arrive_threshold_rad;      /* 撑杆判定到位的角度误差阈值，单位 rad */
} Pole_AutoProfile_t;

typedef struct {
  Pole_AutoStage_t stage;
  Pole_AutoStepType_t step_type;
  bool running;
  bool done;
  bool error;
  uint32_t stage_start_tick;
  uint32_t trigger_start_tick;
} Pole_Auto_t;

void Pole_AutoInit(Pole_Auto_t *a);
void Pole_AutoReset(Pole_Auto_t *a);
const Pole_AutoProfile_t *Pole_AutoGetProfile(Pole_AutoStepType_t step_type);
int8_t Pole_AutoUpdate(Pole_Auto_t *a, const Pole_t *pole, const Pole_AutoCmd_t *cmd,
                       const Pole_AutoSensor_t *sensor, Pole_CMD_t *pole_cmd, uint32_t now);

#ifdef __cplusplus
}
#endif