/*
 * Camera yaw module: 6020 yaw motor closed loop in world frame.
 */

#include "module/camera_yaw.h"

#include <math.h>
#include <string.h>

#include "bsp/can.h"
#include "component/user_math.h"

static float CameraYaw_ClampSymmetric(float value, float limit) {
  if (!isfinite(value)) {
    return 0.0f;
  }
  if (limit <= 0.0f || !isfinite(limit)) {
    limit = 1.0f;
  }
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

static bool CameraYaw_ModeIsValid(CameraYaw_Mode_t mode) {
  return mode == CAMERA_YAW_MODE_RELAX || mode == CAMERA_YAW_MODE_ACTIVE;
}

static float CameraYaw_CalcDt(CameraYaw_t *c, uint32_t now_ms) {
  if (c == NULL) {
    return 0.001f;
  }

  float dt = 0.001f;
  if (c->last_wakeup != 0u) {
    dt = (float)(now_ms - c->last_wakeup) * 0.001f;
  }
  c->last_wakeup = now_ms;

  if (!isfinite(dt) || dt < 0.0005f || dt > 0.050f) {
    dt = 0.001f;
  }
  return dt;
}

static bool CameraYaw_CommandFresh(const CameraYaw_t *c,
                                   const CameraYaw_CMD_t *cmd,
                                   uint32_t now_ms,
                                   uint32_t *age_ms) {
  if (age_ms != NULL) {
    *age_ms = 0u;
  }
  if (c == NULL || c->param == NULL || cmd == NULL || !cmd->feedback_valid) {
    return false;
  }
  if (!isfinite(cmd->target_yaw_rad) || !isfinite(cmd->feedback_yaw_rad)) {
    return false;
  }

  const uint32_t age = now_ms - cmd->feedback_tick_ms;
  if (age_ms != NULL) {
    *age_ms = age;
  }

  const uint32_t timeout_ms = c->param->limit.feedback_timeout_ms;
  return timeout_ms == 0u || age <= timeout_ms;
}

int8_t CameraYaw_Init(CameraYaw_t *c, const CameraYaw_Params_t *param,
                      float target_freq) {
  if (c == NULL || param == NULL) {
    return CAMERA_YAW_ERR_NULL;
  }
  if (target_freq <= 0.0f || !isfinite(target_freq)) {
    return CAMERA_YAW_ERR;
  }

  memset(c, 0, sizeof(*c));
  BSP_CAN_Init();

  c->param = param;
  c->mode = CAMERA_YAW_MODE_RELAX;
  c->cmd.mode = CAMERA_YAW_MODE_RELAX;

  if (PID_Init(&c->yaw_pid, KPID_MODE_NO_D, target_freq,
               &param->pid.yaw_pid) != 0) {
    return CAMERA_YAW_ERR;
  }

  if (MOTOR_RM_Register((MOTOR_RM_Param_t *)&param->motor_param) !=
      DEVICE_OK) {
    return CAMERA_YAW_ERR;
  }

  c->motor = MOTOR_RM_GetMotor((MOTOR_RM_Param_t *)&param->motor_param);
  return c->motor != NULL ? CAMERA_YAW_OK : CAMERA_YAW_ERR;
}

int8_t CameraYaw_UpdateFeedback(CameraYaw_t *c) {
  if (c == NULL || c->param == NULL) {
    return CAMERA_YAW_ERR_NULL;
  }

  const int8_t update_ret =
      MOTOR_RM_Update((MOTOR_RM_Param_t *)&c->param->motor_param);
  c->motor = MOTOR_RM_GetMotor((MOTOR_RM_Param_t *)&c->param->motor_param);
  if (c->motor == NULL) {
    c->feedback.motor_online = false;
    return CAMERA_YAW_ERR;
  }

  c->feedback.motor_online = c->motor->motor.header.online;
  c->feedback.motor_angle_rad = c->motor->feedback.rotor_total_angle;
  c->feedback.motor_velocity_rad_s = c->motor->feedback.rotor_speed;
  c->feedback.temperature_c = c->motor->feedback.temp;

  return update_ret == DEVICE_OK ? CAMERA_YAW_OK : CAMERA_YAW_ERR;
}

int8_t CameraYaw_Control(CameraYaw_t *c, const CameraYaw_CMD_t *cmd,
                         uint32_t now_ms) {
  if (c == NULL || c->param == NULL || cmd == NULL) {
    return CAMERA_YAW_ERR_NULL;
  }

  c->dt = CameraYaw_CalcDt(c, now_ms);
  c->cmd = *cmd;

  if (!CameraYaw_ModeIsValid(cmd->mode)) {
    c->cmd.mode = CAMERA_YAW_MODE_RELAX;
  }

  uint32_t feedback_age_ms = 0u;
  const bool feedback_valid =
      CameraYaw_CommandFresh(c, &c->cmd, now_ms, &feedback_age_ms);

  c->feedback.feedback_age_ms = feedback_age_ms;
  c->feedback.feedback_valid = feedback_valid;
  c->feedback.target_yaw_rad = c->cmd.target_yaw_rad;
  c->feedback.feedback_yaw_rad = c->cmd.feedback_yaw_rad;
  c->mode = c->cmd.mode;
  c->feedback.mode = c->mode;

  if (c->mode == CAMERA_YAW_MODE_RELAX || !feedback_valid ||
      !c->feedback.motor_online) {
    c->output = 0.0f;
    c->feedback.error_yaw_rad = 0.0f;
    c->feedback.at_target = false;
    PID_Reset(&c->yaw_pid);
    return CAMERA_YAW_OK;
  }

  c->feedback.error_yaw_rad =
      CircleError(c->cmd.target_yaw_rad, c->cmd.feedback_yaw_rad, M_2PI);
  c->output =
      PID_Calc(&c->yaw_pid, c->cmd.target_yaw_rad, c->cmd.feedback_yaw_rad,
               0.0f, c->dt);
  c->output = CameraYaw_ClampSymmetric(c->output, c->param->limit.max_output);
  c->feedback.output = c->output;
  c->feedback.at_target =
      fabsf(c->feedback.error_yaw_rad) <=
      fabsf(c->param->limit.arrive_threshold_rad);
  return CAMERA_YAW_OK;
}

void CameraYaw_Output(CameraYaw_t *c) {
  if (c == NULL || c->param == NULL) {
    return;
  }

  float output = c->output;
  if (g_camera_yaw_debug.enable && g_camera_yaw_debug.direct_output_enable) {
    output = g_camera_yaw_debug.direct_output;
  }
  output = CameraYaw_ClampSymmetric(output, c->param->limit.max_output);
  c->feedback.output = output;

  (void)MOTOR_RM_SetOutput((MOTOR_RM_Param_t *)&c->param->motor_param, output);
  (void)MOTOR_RM_Ctrl((MOTOR_RM_Param_t *)&c->param->motor_param);
}

void CameraYaw_ResetOutput(CameraYaw_t *c) {
  if (c == NULL || c->param == NULL) {
    return;
  }

  c->output = 0.0f;
  c->feedback.output = 0.0f;
  PID_Reset(&c->yaw_pid);
  (void)MOTOR_RM_Relax((MOTOR_RM_Param_t *)&c->param->motor_param);
  (void)MOTOR_RM_Ctrl((MOTOR_RM_Param_t *)&c->param->motor_param);
}

bool CameraYaw_IsAtTarget(const CameraYaw_t *c) {
  return c != NULL && c->feedback.at_target;
}
