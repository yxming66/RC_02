#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "component/pid.h"
#include "device/dr16.h"
#include "device/motor_dm.h"
#include "device/motor_lz.h"
#include "device/motor_rm.h"

#define ARMPOS_OK (0)
#define ARMPOS_ERR (-1)
#define ARMPOS_ERR_NULL (-2)

typedef enum {
  ARMPOS_MODE_RELAX = 0,
  ARMPOS_MODE_ACTIVE,
} ArmPos_Mode_t;

typedef struct {
  MOTOR_LZ_Param_t lzmotor_param;
  MOTOR_DM_Param_t dmmotor_param;
  MOTOR_RM_Param_t rmmotor_param;

  KPID_Params_t pid_dmmotor_pos;
  KPID_Params_t pid_dmmotor_vel;
  KPID_Params_t pid_rmmotor_pos;
  KPID_Params_t pid_rmmotor_vel;

  struct {
    float lz_min;
    float lz_max;
    float dm_min;
    float dm_max;
    float rm_min;
    float rm_max;
  } joint_limit;

  struct {
    float lz_speed;
    float dm_speed;
    float rm_speed;
  } input_scale;

  struct {
    float rm_output_limit;
    float rm_velocity_limit;
    float dm_velocity_ref_limit;
    float dm_kd;
    float dm_hold_torque;
    float dm_hold_velocity_threshold;
  } limit;
} ArmPos_Params_t;

typedef struct {
  ArmPos_Mode_t mode;
  float lz_delta;
  float dm_delta;
  float rm_delta;
} ArmPos_CMD_t;

typedef struct {
  MOTOR_LZ_Feedback_t lzmotor_feedback;
  MOTOR_Feedback_t dmmotor_feedback;
  MOTOR_RM_Feedback_t rmmotor_feedback;
} ArmPos_Feedback_t;

typedef struct {
  MOTOR_LZ_MotionParam_t lzmotor;
  MOTOR_MIT_Output_t dmmotor;
  float rmmotor;
} ArmPos_Output_t;

typedef struct {
  ArmPos_Params_t *param;
  MOTOR_LZ_Param_t *lzmotor_param;
  MOTOR_DM_Param_t *dmmotor_param;
  MOTOR_RM_Param_t *rmmotor_param;
  ArmPos_Mode_t mode;

  struct {
    float now;
    uint64_t lask_wakeup;
    float dt;
  } timer;

  struct {
    KPID_t dmmotor_pos;
    KPID_t dmmotor_vel;
    KPID_t rmmotor_pos;
    KPID_t rmmotor_vel;
  } pid;

  struct {
    float lz;
    float dm;
    float rm;
  } target;

  ArmPos_Feedback_t feedback;
  struct {
    MOTOR_RM_t *rmmotor;
    MOTOR_DM_t *dmmotor;
    MOTOR_LZ_t *lzmotor;
  } motor;
  ArmPos_Output_t out;
} ArmPos_t;

int8_t ArmPos_Init(ArmPos_t *a, ArmPos_Params_t *param, float target_freq);
int8_t ArmPos_UpdateFeedback(ArmPos_t *a);
int8_t ArmPos_Control(ArmPos_t *a, const ArmPos_CMD_t *cmd);
int8_t ArmPos_Output(ArmPos_t *a);
void ArmPos_Relax(ArmPos_t *a);
void ArmPos_ResetOutput(ArmPos_t *a);
void ArmPos_SetCmdFromRc(ArmPos_CMD_t *cmd, const DR16_t *rc);

#ifdef __cplusplus
}
#endif