/*
 * Camera yaw module: 6020 yaw motor closed loop in chassis body frame.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "component/pid.h"
#include "device/motor.h"
#include "device/motor_rm.h"

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
  MOTOR_RM_Param_t motor_param;
  float encoder_zero_offset_rad;
  struct {
    KPID_Params_t yaw_pid;
  } pid;
  struct {
    float max_output;
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
  uint32_t last_wakeup;
  float dt;
  const CameraYaw_Params_t *param;
  CameraYaw_Mode_t mode;
  MOTOR_RM_t *motor;
  KPID_t yaw_pid;
  CameraYaw_CMD_t cmd;
  CameraYaw_Feedback_t feedback;
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
