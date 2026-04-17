/*
 * Pole module: 4x3508 support motors.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "component/pid.h"
#include "component/filter.h"
#include "device/motor.h"
#include "device/motor_rm.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include "device/motor/packages/controller/motor_controller.hpp"
#endif

#define POLE_OK (0)
#define POLE_ERR (-1)
#define POLE_ERR_NULL (-2)

#define POLE_SUPPORT_MOTOR_NUM (4)
#define POLE_MOTOR_NUM (POLE_SUPPORT_MOTOR_NUM)

typedef enum {
  POLE_MODE_RELAX = 0,
  POLE_MODE_ACTIVE,
} Pole_Mode_t;

typedef struct {
  Pole_Mode_t mode;
  float lift[2];   /* [-1, 1], >0 means up */
  bool auto_target_enable[2];
  float auto_target_lift[2];   /* rad, relative to calibrated lower limit */
  float auto_lift_speed[2];    /* rad/s, <=0 means use default support_lift_speed */
} Pole_CMD_t;

typedef struct {
  MOTOR_RM_Param_t motor_param[POLE_MOTOR_NUM];
  struct {
    float external_ratio;
    bool reverse_output;
  } motor_install[POLE_MOTOR_NUM];
  struct {
    KPID_Params_t support_pos_pid;
    KPID_Params_t support_vel_pid;
  } pid;
  struct {
    float support_vel_feedback_cutoff_hz;   /* Hz, <=0 means fallback */
  } filter;
  struct {
    float step_200_all_extend[2];      /* 200mm台阶-四撑杆全伸: [0]前两杆, [1]后两杆 */
    float step_200_front_retract[2];   /* 200mm台阶-前两撑杆收: [0]前两杆, [1]后两杆 */
    float step_200_all_retract[2];     /* 200mm台阶-四撑杆全收: [0]前两杆, [1]后两杆 */
    float step_400_all_extend[2];      /* 400mm台阶-四撑杆全伸: [0]前两杆, [1]后两杆 */
    float step_400_front_retract[2];   /* 400mm台阶-前两撑杆收: [0]前两杆, [1]后两杆 */
    float step_400_all_retract[2];     /* 400mm台阶-四撑杆全收: [0]前两杆, [1]后两杆 */
  } preset;
  struct {
    float max_current;
    float support_total_travel;   /* rad */
    float support_lift_speed;     /* rad/s */
  } limit;
} Pole_Params_t;

typedef struct {
  MOTOR_Feedback_t motor[POLE_MOTOR_NUM];
  float support_vel_filtered[POLE_SUPPORT_MOTOR_NUM];
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
  void *motors[POLE_MOTOR_NUM];
  void *controllers[POLE_MOTOR_NUM];

  struct {
    bool calibrated;
    float lower[POLE_SUPPORT_MOTOR_NUM];
    float upper[POLE_SUPPORT_MOTOR_NUM];
    float final_target_lift[2];
    float tracked_target_lift[2];
  } support_angle;

  struct {
    float support_target_angle[POLE_SUPPORT_MOTOR_NUM];
  } setpoint;

  struct {
    KPID_t support_pos[POLE_SUPPORT_MOTOR_NUM];
    KPID_t support_vel[POLE_SUPPORT_MOTOR_NUM];
  } pid;

  struct {
    LowPassFilter2p_t support_vel_in[POLE_SUPPORT_MOTOR_NUM];
  } filter;

  Pole_Output_t out;
  Pole_Feedback_t feedback;
} Pole_t;

#ifdef __cplusplus
extern "C" {
#endif

int8_t Pole_Init(Pole_t *c, const Pole_Params_t *param, float target_freq);
int8_t Pole_UpdateFeedback(Pole_t *c);
int8_t Pole_Control(Pole_t *c, const Pole_CMD_t *c_cmd, uint32_t now);
bool Pole_IsGroupAtTarget(const Pole_t *c, uint8_t group, float threshold_rad);
bool Pole_IsAllAtTarget(const Pole_t *c, float threshold_rad);
void Pole_Output(Pole_t *c);
void Pole_ResetOutput(Pole_t *c);
void Pole_Power_Control(Pole_t *c, float max_power);

#ifdef __cplusplus
}
#endif




