/*
 * Camera yaw module: DM-H3510 native MIT control in chassis body frame.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "device/motor_dm.h"

#define CAMERA_YAW_OK (0)
#define CAMERA_YAW_ERR (-1)
#define CAMERA_YAW_ERR_NULL (-2)

typedef enum {
  CAMERA_YAW_MODE_RELAX = 0,
  CAMERA_YAW_MODE_ACTIVE = 1,
} CameraYaw_Mode_t;

typedef enum {
  CAMERA_YAW_LEFT = 0,
  CAMERA_YAW_RIGHT = 1,
  CAMERA_YAW_NUM = 2,
} CameraYaw_Channel_t;

typedef struct {
  MOTOR_DM_Param_t motor_param;
  float encoder_zero_offset_rad;
  struct {
    float kp;
    float kd;
    float target_velocity_rad_s;
    float torque_ff_nm;
    float max_position_error_rad;
  } mit;
  struct {
    float max_torque_nm;
    float arrive_threshold_rad;
    uint32_t feedback_timeout_ms;
  } limit;
} CameraYaw_Params_t;

typedef struct {
  CameraYaw_Mode_t mode;
  float target_yaw_rad;
  float feedback_yaw_rad;
  uint32_t feedback_tick_ms;
  bool feedback_valid;
} CameraYaw_CMD_t;

typedef struct {
  CameraYaw_Mode_t mode;
  bool motor_online;
  bool feedback_valid;
  bool at_target;
  float target_yaw_rad;
  float feedback_yaw_rad;
  float error_yaw_rad;
  float motor_angle_rad;
  float motor_velocity_rad_s;
  float output;
  float temperature_c;
  uint32_t feedback_age_ms;
} CameraYaw_Feedback_t;

typedef struct {
  CameraYaw_CMD_t yaw[CAMERA_YAW_NUM];
} CameraYaw_GroupCMD_t;

typedef struct {
  CameraYaw_Feedback_t yaw[CAMERA_YAW_NUM];
} CameraYaw_GroupFeedback_t;

typedef struct {
  volatile bool enable;
  volatile bool direct_output_enable;
  volatile CameraYaw_Mode_t mode;
  volatile float target_yaw_rad;
  volatile float feedback_yaw_rad;
  volatile float direct_output;
} CameraYaw_DebugControl_t;

typedef struct {
  void *motor_protocol;
  void *motor_state;
  uint32_t last_wakeup;
  float dt;
  float nominal_dt;
  const CameraYaw_Params_t *param;
  CameraYaw_Mode_t mode;
  CameraYaw_CMD_t cmd;
  CameraYaw_Feedback_t feedback;
  float mit_target_motor_angle_rad;
  float output;
} CameraYaw_t;

int8_t CameraYaw_Init(CameraYaw_t *c, const CameraYaw_Params_t *param,
                      float target_freq);
int8_t CameraYaw_UpdateFeedback(CameraYaw_t *c);
int8_t CameraYaw_Control(CameraYaw_t *c, const CameraYaw_CMD_t *cmd,
                         uint32_t now_ms);
void CameraYaw_SetOutput(CameraYaw_t *c);
void CameraYaw_FlushOutput(CameraYaw_t *c);
void CameraYaw_Output(CameraYaw_t *c);
void CameraYaw_ResetOutput(CameraYaw_t *c);
bool CameraYaw_IsAtTarget(const CameraYaw_t *c);

extern volatile CameraYaw_DebugControl_t g_camera_yaw_debug;

#ifdef __cplusplus
}
#endif
