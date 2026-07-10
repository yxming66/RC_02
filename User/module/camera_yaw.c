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

static constexpr float kCameraYawMitKpMax = 500.0f;
static constexpr float kCameraYawMitKdMax = 5.0f;
static constexpr float kCameraYawMitPositionMinRad = -12.56637f;
static constexpr float kCameraYawMitPositionMaxRad = 12.56637f;

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

static float CameraYaw_WrapMitPositionRad(float angle_rad) {
  if (!isfinite(angle_rad)) {
    return 0.0f;
  }
  const float range =
      kCameraYawMitPositionMaxRad - kCameraYawMitPositionMinRad;
  angle_rad = fmodf(angle_rad - kCameraYawMitPositionMinRad, range);
  if (angle_rad < 0.0f) {
    angle_rad += range;
  }
  return angle_rad + kCameraYawMitPositionMinRad;
}

static float CameraYaw_EncoderZeroOffsetRad(const CameraYaw_t *c) {
  if (c == NULL || !isfinite(c->encoder_zero_offset_rad)) {
    return 0.0f;
  }
  return c->encoder_zero_offset_rad;
}

static float CameraYaw_BodyYawFromEncoder(const CameraYaw_t *c,
                                          float encoder_angle_rad) {
  return CameraYaw_WrapAngleRad(encoder_angle_rad -
                                CameraYaw_EncoderZeroOffsetRad(c));
}

static bool CameraYaw_ModeIsValid(CameraYaw_Mode_t mode) {
  return mode == CAMERA_YAW_MODE_RELAX || mode == CAMERA_YAW_MODE_ACTIVE;
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
  c->encoder_zero_offset_rad =
      isfinite(param->encoder_zero_offset_rad)
          ? param->encoder_zero_offset_rad
          : 0.0f;
  c->mode = CAMERA_YAW_MODE_RELAX;
  c->nominal_dt = 1.0f / target_freq;
  c->dt = c->nominal_dt;
  c->cmd.mode = CAMERA_YAW_MODE_RELAX;
    if (!isfinite(param->mit.kp) || param->mit.kp < 0.0f ||
      param->mit.kp > kCameraYawMitKpMax ||
      !isfinite(param->mit.kd) || param->mit.kd < 0.0f ||
      param->mit.kd > kCameraYawMitKdMax ||
      !isfinite(param->mit.target_velocity_rad_s) ||
      !isfinite(param->mit.torque_ff_nm) ||
      !isfinite(param->mit.max_position_error_rad) ||
      param->mit.max_position_error_rad <= 0.0f ||
      !isfinite(param->limit.max_torque_nm) ||
      param->limit.max_torque_nm <= 0.0f ||
      fabsf(param->mit.torque_ff_nm) > param->limit.max_torque_nm) {
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
  if (c->mode != CAMERA_YAW_MODE_RELAX) {
    (void)protocol->Enable();
  }

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
    c->feedback.output = 0.0f;
    c->feedback.error_yaw_rad = 0.0f;
    c->feedback.at_target = false;
    c->mit_target_motor_angle_rad = c->feedback.motor_angle_rad;
    c->mit_kd_cmd = 0.0f;
    return CAMERA_YAW_OK;
  }

  c->feedback.error_yaw_rad =
      CircleError(target_yaw_rad, feedback_yaw_rad, M_2PI);
  float limited_position_error = CameraYaw_ClampSymmetric(
      c->feedback.error_yaw_rad, c->param->mit.max_position_error_rad);
  const float requested_position_torque =
      c->param->mit.kp * limited_position_error +
      c->param->mit.torque_ff_nm;
  const float limited_position_torque = CameraYaw_ClampSymmetric(
      requested_position_torque, c->param->limit.max_torque_nm);
  if (c->param->mit.kp > 0.0f) {
    limited_position_error =
        (limited_position_torque - c->param->mit.torque_ff_nm) /
        c->param->mit.kp;
  }

  const float velocity_error = c->param->mit.target_velocity_rad_s -
                               c->feedback.motor_velocity_rad_s;
  const float damping_budget =
      fmaxf(c->param->limit.max_torque_nm - fabsf(limited_position_torque),
            0.0f);
  c->mit_kd_cmd = c->param->mit.kd;
  if (fabsf(velocity_error) * c->mit_kd_cmd > damping_budget) {
    c->mit_kd_cmd = damping_budget / fabsf(velocity_error);
  }

  c->mit_target_motor_angle_rad = CameraYaw_WrapMitPositionRad(
      c->feedback.motor_angle_rad + limited_position_error);
  c->output = limited_position_torque + c->mit_kd_cmd * velocity_error;
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

  CameraYawMotorProtocol *protocol = CameraYaw_GetMotorProtocol(c);
  if (protocol == NULL) {
    return;
  }

  if (g_camera_yaw_debug.enable && g_camera_yaw_debug.direct_output_enable) {
    const float torque_ff = CameraYaw_ClampSymmetric(
        g_camera_yaw_debug.direct_output, c->param->limit.max_torque_nm);
    c->feedback.output = torque_ff;
    (void)protocol->SetMIT(0.0f, 0.0f, 0.0f, 0.0f, torque_ff);
    return;
  }

  if (c->mode != CAMERA_YAW_MODE_ACTIVE || !c->feedback.feedback_valid ||
      !c->feedback.motor_online) {
    (void)protocol->SetMIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    return;
  }

  (void)protocol->SetMIT(c->mit_target_motor_angle_rad,
                         c->param->mit.target_velocity_rad_s,
                         c->param->mit.kp, c->mit_kd_cmd,
                         c->param->mit.torque_ff_nm);
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
  c->mit_target_motor_angle_rad = c->feedback.motor_angle_rad;
  c->mit_kd_cmd = 0.0f;
  CameraYawMotorProtocol *protocol = CameraYaw_GetMotorProtocol(c);
  if (protocol != NULL) {
    (void)protocol->Relax();
  }
}

int8_t CameraYaw_SetZero(CameraYaw_t *c) {
  if (c == NULL || c->param == NULL) {
    return CAMERA_YAW_ERR_NULL;
  }

  CameraYawMotorProtocol *protocol = CameraYaw_GetMotorProtocol(c);
  if (protocol == NULL) {
    return CAMERA_YAW_ERR;
  }

  const int8_t ret = protocol->SetZero();
  if (ret != DEVICE_OK) {
    return CAMERA_YAW_ERR;
  }

  c->encoder_zero_offset_rad = 0.0f;
  c->output = 0.0f;
  c->feedback.output = 0.0f;
  c->feedback.motor_angle_rad = 0.0f;
  c->feedback.feedback_yaw_rad = 0.0f;
  c->feedback.error_yaw_rad = 0.0f;
  c->feedback.at_target = false;
  c->mit_target_motor_angle_rad = 0.0f;
  c->mit_kd_cmd = 0.0f;
  return CAMERA_YAW_OK;
}

bool CameraYaw_IsAtTarget(const CameraYaw_t *c) {
  return c != NULL && c->feedback.at_target;
}
