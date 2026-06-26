/*
 * Camera yaw module: DM-H3510 yaw motor closed loop in chassis body frame.
 */

#include "module/camera_yaw.h"

#include <math.h>
#include <new>
#include <string.h>

#include "bsp/can.h"
#include "component/user_math.h"
#include "device/motor/core/motor_install_spec.hpp"
#include "device/motor/core/motor_instance_config.hpp"
#include "device/motor/core/motor_model.hpp"
#include "device/motor/core/motor_state.hpp"
#include "device/motor/protocol/dm_protocol.hpp"

using CameraYawMotorProtocol =
    mr::motor::MotorProtocol<mr::motor::MotorKind::DM,
                             mr::motor::MotorModel::H3510>;

struct CameraYawMotorSlot {
  bool used;
  CameraYaw_t *owner;
  mr::motor::MotorState state;
  alignas(CameraYawMotorProtocol) unsigned char protocol_storage[sizeof(CameraYawMotorProtocol)];
};

static CameraYawMotorSlot camera_yaw_motor_slots[CAMERA_YAW_NUM];

static CameraYawMotorProtocol *CameraYaw_GetMotorProtocol(CameraYaw_t *c) {
  return (c == NULL) ? NULL : (CameraYawMotorProtocol *)c->motor_protocol;
}

static const mr::motor::MotorState *CameraYaw_GetMotorStateConst(
    const CameraYaw_t *c) {
  return (c == NULL) ? NULL : (const mr::motor::MotorState *)c->motor_state;
}

static CameraYawMotorSlot *CameraYaw_FindSlot(CameraYaw_t *c) {
  if (c == NULL) {
    return NULL;
  }
  for (uint8_t i = 0u; i < CAMERA_YAW_NUM; ++i) {
    if (camera_yaw_motor_slots[i].used &&
        camera_yaw_motor_slots[i].owner == c) {
      return &camera_yaw_motor_slots[i];
    }
  }
  for (uint8_t i = 0u; i < CAMERA_YAW_NUM; ++i) {
    if (!camera_yaw_motor_slots[i].used) {
      camera_yaw_motor_slots[i].used = true;
      camera_yaw_motor_slots[i].owner = c;
      camera_yaw_motor_slots[i].state = {};
      return &camera_yaw_motor_slots[i];
    }
  }
  return NULL;
}

static CameraYawMotorProtocol *CameraYaw_ConstructMotorProtocol(
    CameraYaw_t *c, const MOTOR_DM_Param_t *param) {
  if (c == NULL || param == NULL) {
    return NULL;
  }
  CameraYawMotorSlot *slot = CameraYaw_FindSlot(c);
  if (slot == NULL) {
    return NULL;
  }

  const mr::motor::MotorInstanceConfig<mr::motor::MotorKind::DM> config =
      mr::motor::MotorInstanceConfig<mr::motor::MotorKind::DM>::FromVendorParam(
          *param);
  CameraYawMotorProtocol *protocol = new (slot->protocol_storage)
      CameraYawMotorProtocol(config, mr::motor::kDirectDriveInstall,
                             slot->state, param->module);
  c->motor_protocol = protocol;
  c->motor_state = &slot->state;
  return protocol;
}

#define CAMERA_YAW_PID_EPSILON (1.0e-6f)

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

static float CameraYaw_WrapAngleRad(float angle_rad) {
  if (!isfinite(angle_rad)) {
    return 0.0f;
  }
  angle_rad = fmodf(angle_rad, M_2PI);
  if (angle_rad < 0.0f) {
    angle_rad += M_2PI;
  }
  return angle_rad;
}

static float CameraYaw_EncoderZeroOffsetRad(const CameraYaw_t *c) {
  if (c == NULL || c->param == NULL ||
      !isfinite(c->param->encoder_zero_offset_rad)) {
    return 0.0f;
  }
  return c->param->encoder_zero_offset_rad;
}

static float CameraYaw_BodyYawFromEncoder(const CameraYaw_t *c,
                                          float encoder_angle_rad) {
  return CameraYaw_WrapAngleRad(encoder_angle_rad -
                                CameraYaw_EncoderZeroOffsetRad(c));
}

static bool CameraYaw_ModeIsValid(CameraYaw_Mode_t mode) {
  return mode == CAMERA_YAW_MODE_RELAX || mode == CAMERA_YAW_MODE_ACTIVE;
}

static float CameraYaw_CalcMitCurrentFromError(CameraYaw_t *c,
                                               float error_yaw_rad) {
  if (c == NULL || c->yaw_pid.param == NULL || !isfinite(error_yaw_rad)) {
    return 0.0f;
  }

  const KPID_Params_t *param = c->yaw_pid.param;
  const float dt = fmaxf(c->dt, c->yaw_pid.dt_min);
  const float k_error = error_yaw_rad * param->k;
  float current = k_error * param->p;

  if (param->d > CAMERA_YAW_PID_EPSILON && isfinite(dt)) {
    const float d_error = (k_error - c->yaw_pid.last.err) / dt;
    if (isfinite(d_error)) {
      current += d_error * param->d;
    }
  }

  c->yaw_pid.last.err = k_error;

  if (param->i > CAMERA_YAW_PID_EPSILON && isfinite(dt)) {
    const float candidate_i = c->yaw_pid.i + k_error * dt;
    const float candidate_i_out = candidate_i * param->i;
    const float out_limit = fabsf(param->out_limit);
    const float i_limit = fabsf(param->i_limit);
    const bool output_ok = out_limit <= CAMERA_YAW_PID_EPSILON ||
                           fabsf(current + candidate_i_out) <= out_limit;
    const bool integral_ok = i_limit <= CAMERA_YAW_PID_EPSILON ||
                             fabsf(candidate_i) <= i_limit;
    if (isfinite(candidate_i) && output_ok && integral_ok) {
      c->yaw_pid.i = candidate_i;
    }
    current += c->yaw_pid.i * param->i;
  }

  if (isfinite(current)) {
    if (param->out_limit > CAMERA_YAW_PID_EPSILON) {
      current = CameraYaw_ClampSymmetric(current, param->out_limit);
    }
    c->yaw_pid.last.out = current;
  }
  return c->yaw_pid.last.out;
}

static float CameraYaw_CalcDt(CameraYaw_t *c, uint32_t now_ms) {
  if (c == NULL) {
    return 0.001f;
  }

  const float nominal_dt =
      (isfinite(c->nominal_dt) && c->nominal_dt > 0.0f) ? c->nominal_dt
                                                        : 0.001f;
  float dt = nominal_dt;
  if (c->last_wakeup != 0u) {
    dt = (float)(now_ms - c->last_wakeup) * 0.001f;
  }
  c->last_wakeup = now_ms;

  if (!isfinite(dt) || dt < nominal_dt * 0.5f || dt > nominal_dt * 3.0f) {
    dt = nominal_dt;
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
  if (!isfinite(cmd->target_yaw_rad)) {
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
  c->nominal_dt = 1.0f / target_freq;
  c->dt = c->nominal_dt;
  c->cmd.mode = CAMERA_YAW_MODE_RELAX;

  if (PID_Init(&c->yaw_pid, KPID_MODE_NO_D, target_freq,
               &param->pid.yaw_pid) != 0) {
    return CAMERA_YAW_ERR;
  }

  CameraYawMotorProtocol *protocol =
      CameraYaw_ConstructMotorProtocol(c, &param->motor_param);
  if (protocol == NULL) {
    return CAMERA_YAW_ERR;
  }
  if (protocol->Register() != DEVICE_OK) {
    return CAMERA_YAW_ERR;
  }

  (void)protocol->Enable();
  return CAMERA_YAW_OK;
}

int8_t CameraYaw_UpdateFeedback(CameraYaw_t *c) {
  if (c == NULL || c->param == NULL) {
    return CAMERA_YAW_ERR_NULL;
  }

  CameraYawMotorProtocol *protocol = CameraYaw_GetMotorProtocol(c);
  const mr::motor::MotorState *state = CameraYaw_GetMotorStateConst(c);
  if (protocol == NULL || state == NULL) {
    c->feedback.motor_online = false;
    return CAMERA_YAW_ERR;
  }

  const int8_t update_ret = protocol->Update();

  c->feedback.motor_online = state->online;
  c->feedback.motor_angle_rad = state->position_rad;
  c->feedback.motor_velocity_rad_s = state->velocity_rad_s;
  c->feedback.temperature_c = state->device_temperature_c;
  c->feedback.feedback_yaw_rad =
      CameraYaw_BodyYawFromEncoder(c, c->feedback.motor_angle_rad);

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
  const float target_yaw_rad = CameraYaw_WrapAngleRad(c->cmd.target_yaw_rad);
  const float feedback_yaw_rad = c->feedback.feedback_yaw_rad;
  c->feedback.target_yaw_rad = target_yaw_rad;
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
      CircleError(target_yaw_rad, feedback_yaw_rad, M_2PI);
  c->output = CameraYaw_CalcMitCurrentFromError(
      c, c->feedback.error_yaw_rad);
  c->output = CameraYaw_ClampSymmetric(c->output, c->param->limit.max_output);
  c->feedback.output = c->output;
  c->feedback.at_target =
      fabsf(c->feedback.error_yaw_rad) <=
      fabsf(c->param->limit.arrive_threshold_rad);
  return CAMERA_YAW_OK;
}

void CameraYaw_SetOutput(CameraYaw_t *c) {
  if (c == NULL || c->param == NULL) {
    return;
  }

  float output = c->output;
  if (g_camera_yaw_debug.enable && g_camera_yaw_debug.direct_output_enable) {
    output = g_camera_yaw_debug.direct_output;
  }
  output = CameraYaw_ClampSymmetric(output, c->param->limit.max_output);
  c->feedback.output = output;

  CameraYawMotorProtocol *protocol = CameraYaw_GetMotorProtocol(c);
  if (protocol != NULL) {
    (void)protocol->SetMIT(0.0f, 0.0f, 0.0f, 0.0f, output);
  }
}

void CameraYaw_FlushOutput(CameraYaw_t *c) {
  if (c == NULL || c->param == NULL) {
    return;
  }

  CameraYawMotorProtocol *protocol = CameraYaw_GetMotorProtocol(c);
  if (protocol != NULL) {
    (void)protocol->CommitCommand();
  }
}

void CameraYaw_Output(CameraYaw_t *c) {
  CameraYaw_SetOutput(c);
  CameraYaw_FlushOutput(c);
}

void CameraYaw_ResetOutput(CameraYaw_t *c) {
  if (c == NULL || c->param == NULL) {
    return;
  }

  c->output = 0.0f;
  c->feedback.output = 0.0f;
  PID_Reset(&c->yaw_pid);
  CameraYawMotorProtocol *protocol = CameraYaw_GetMotorProtocol(c);
  if (protocol != NULL) {
    (void)protocol->Relax();
  }
}

bool CameraYaw_IsAtTarget(const CameraYaw_t *c) {
  return c != NULL && c->feedback.at_target;
}
