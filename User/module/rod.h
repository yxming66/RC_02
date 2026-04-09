/*
 * Rod module: 2x DM4310 pick-rod mechanism.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "device/motor_dm.h"

#define ROD_OK (0)
#define ROD_ERR (-1)
#define ROD_ERR_NULL (-2)

#define ROD_GRAVITY_G (9.80665f)
#define ROD_PIT_GRAVITY_MASS_KG (0.10f)
#define ROD_PIT_COM_DISTANCE_M (0.10f)
#define ROD_PIT_GRAVITY_DIFF_RAD (0.001f)
#define ROD_PIT_GRAVITY_SIGN (1.0f)

typedef enum {
  ROD_MODE_RELAX = 0,
  ROD_MODE_ACTIVE,
  ROD_MODE_SEQUENCE,
} Rod_Mode_t;

typedef enum {
  ROD_POSE_DOWN = 0,
  ROD_POSE_UP,
  ROD_POSE_FLIP,
  ROD_POSE_NONE,
} Rod_Pose_t;

typedef struct {
  Rod_Mode_t mode;
  Rod_Pose_t pose;
  bool sequence_trigger;
  bool grip_done;
} Rod_CMD_t;

typedef struct {
  MOTOR_DM_Param_t pit_motor_param;
  MOTOR_DM_Param_t rol_motor_param;
  struct {
    float pit_down_angle;
    float pit_up_angle;
    float rol_home_angle;
    float rol_flip_angle;
  } pose;
  struct {
    float pit_arrive_threshold;
    float rol_arrive_threshold;
    float sequence_timeout;
    float grip_wait_time;
    float pit_kp;
    float pit_kd;
    float rol_kp;
    float rol_kd;
    float pit_max_vel;
    float pit_max_acc;
    float rol_max_vel;
    float rol_max_acc;
  } limit;
} Rod_Params_t;

typedef struct {
  MOTOR_Feedback_t pit_motor;
  MOTOR_Feedback_t rol_motor;
} Rod_Feedback_t;

typedef struct {
  MOTOR_MIT_Output_t pit_motor;
  MOTOR_MIT_Output_t rol_motor;
} Rod_Output_t;

typedef struct {
  uint32_t last_wakeup;
  float dt;

  const Rod_Params_t *param;
  Rod_Mode_t mode;
  Rod_Pose_t pose;
  Rod_Pose_t setpoint_pose;

  struct {
    float pit_angle;
    float rol_angle;
  } setpoint;

  struct {
    float pit_angle;
    float rol_angle;
    float pit_vel;
    float rol_vel;
    bool initialized;
  } traj;

  struct {
    bool initialized;
    bool done;
    uint8_t step;
    uint32_t step_start_tick;
  } sequence;

  struct {
    MOTOR_DM_t *pit_motor;
    MOTOR_DM_t *rol_motor;
  } motor;

  Rod_Feedback_t feedback;
  Rod_Output_t out;
} Rod_t;

int8_t Rod_Init(Rod_t *r, const Rod_Params_t *param, float target_freq);
int8_t Rod_UpdateFeedback(Rod_t *r);
int8_t Rod_Control(Rod_t *r, const Rod_CMD_t *cmd, uint32_t now);
void Rod_Output(Rod_t *r);
void Rod_ResetOutput(Rod_t *r);

#ifdef __cplusplus
}
#endif
