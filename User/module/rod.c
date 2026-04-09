/*
 * Rod module: 2x DM4310 pick-rod mechanism.
 */

#include "module/rod.h"

#include <math.h>
#include <string.h>

static float Rod_Clipf(float val, float min, float max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

static float Rod_WrapAngle(float angle) {
  while (angle > (float)M_PI) angle -= 2.0f * (float)M_PI;
  while (angle < (float)-M_PI) angle += 2.0f * (float)M_PI;
  return angle;
}

static float Rod_AngleError(float target, float feedback) {
  return Rod_WrapAngle(target - feedback);
}

static bool Rod_Arrived(float target, float feedback, float threshold) {
  return fabsf(Rod_AngleError(target, feedback)) <= threshold;
}

static float Rod_GetPitAngle(const Rod_t *r) {
  float angle = r->feedback.pit_motor.rotor_abs_angle;
  if (r->param->pit_motor_param.reverse) {
    angle = -angle;
  }
  return angle;
}

static float Rod_GetRolAngle(const Rod_t *r) {
  float angle = r->feedback.rol_motor.rotor_abs_angle;
  if (r->param->rol_motor_param.reverse) {
    angle = -angle;
  }
  return angle;
}

static float Rod_GetPotentialEnergy(float pit_angle) {
  return ROD_PIT_GRAVITY_MASS_KG * ROD_GRAVITY_G *
         ROD_PIT_COM_DISTANCE_M * (1.0f - cosf(pit_angle));
}

static float Rod_GetPitGravityComp(float pit_angle) {
  const float h = ROD_PIT_GRAVITY_DIFF_RAD;
  const float grad = (Rod_GetPotentialEnergy(pit_angle + h) -
                      Rod_GetPotentialEnergy(pit_angle - h)) /
                     (2.0f * h);
  return ROD_PIT_GRAVITY_SIGN * grad;
}

static void Rod_SetPoseSetpoint(Rod_t *r, Rod_Pose_t pose) {
  switch (pose) {
    case ROD_POSE_DOWN:
      r->setpoint.pit_angle = r->param->pose.pit_down_angle;
      r->setpoint.rol_angle = r->param->pose.rol_home_angle;
      break;
    case ROD_POSE_UP:
      r->setpoint.pit_angle = r->param->pose.pit_up_angle;
      r->setpoint.rol_angle = r->param->pose.rol_home_angle;
      break;
    case ROD_POSE_FLIP:
      r->setpoint.pit_angle = r->param->pose.pit_up_angle;
      r->setpoint.rol_angle = r->param->pose.rol_flip_angle;
      break;
    default:
      break;
  }
}

static void Rod_ResetSequence(Rod_t *r, uint32_t now) {
  r->sequence.initialized = true;
  r->sequence.done = false;
  r->sequence.step = 0u;
  r->sequence.step_start_tick = now;
  Rod_SetPoseSetpoint(r, ROD_POSE_UP);
}

static void Rod_FinishSequence(Rod_t *r) {
  r->sequence.done = true;
  r->sequence.step = 0u;
  Rod_SetPoseSetpoint(r, ROD_POSE_DOWN);
}
  
static void Rod_UpdateSequence(Rod_t *r, const Rod_CMD_t *cmd, uint32_t now) {
  if (!r->sequence.initialized) {
    Rod_ResetSequence(r, now);
  }

  if (r->sequence.done) {
    Rod_SetPoseSetpoint(r, ROD_POSE_DOWN);
    return;
  }

  if ((float)(now - r->sequence.step_start_tick) * 0.001f >
      r->param->limit.sequence_timeout) {
    Rod_FinishSequence(r);
    return;
  }

  switch (r->sequence.step) {
    case 0u:
      Rod_SetPoseSetpoint(r, ROD_POSE_UP);
      if (Rod_Arrived(r->setpoint.pit_angle, Rod_GetPitAngle(r),
                      r->param->limit.pit_arrive_threshold)) {
        r->sequence.step = 1u;
        r->sequence.step_start_tick = now;
      }
      break;
    case 1u:
      Rod_SetPoseSetpoint(r, ROD_POSE_UP);
      if (cmd->grip_done ||
          ((float)(now - r->sequence.step_start_tick) * 0.001f >=
           r->param->limit.grip_wait_time)) {
        r->sequence.step = 2u;
        r->sequence.step_start_tick = now;
      }
      break;
    case 2u:
      Rod_SetPoseSetpoint(r, ROD_POSE_FLIP);
      if (Rod_Arrived(r->setpoint.rol_angle, Rod_GetRolAngle(r),
                      r->param->limit.rol_arrive_threshold)) {
        r->sequence.step = 3u;
        r->sequence.step_start_tick = now;
      }
      break;
    default:
      Rod_SetPoseSetpoint(r, ROD_POSE_DOWN);
      if (Rod_Arrived(r->setpoint.pit_angle, Rod_GetPitAngle(r),
                      r->param->limit.pit_arrive_threshold)) {
        Rod_FinishSequence(r);
      }
      break;
  }
}

int8_t Rod_Init(Rod_t *r, const Rod_Params_t *param, float target_freq) {
  (void)target_freq;
  if (r == NULL || param == NULL) return ROD_ERR_NULL;

  memset(r, 0, sizeof(Rod_t));
  r->param = param;
  r->mode = ROD_MODE_RELAX;
  r->pose = ROD_POSE_DOWN;
  Rod_SetPoseSetpoint(r, ROD_POSE_DOWN);

  MOTOR_DM_Register((MOTOR_DM_Param_t *)&param->pit_motor_param);
  MOTOR_DM_Register((MOTOR_DM_Param_t *)&param->rol_motor_param);
  MOTOR_DM_Enable((MOTOR_DM_Param_t *)&param->pit_motor_param);
  MOTOR_DM_Enable((MOTOR_DM_Param_t *)&param->rol_motor_param);

  return ROD_OK;
}

int8_t Rod_UpdateFeedback(Rod_t *r) {
  if (r == NULL || r->param == NULL) return ROD_ERR_NULL;

  MOTOR_DM_Update((MOTOR_DM_Param_t *)&r->param->pit_motor_param);
  MOTOR_DM_Update((MOTOR_DM_Param_t *)&r->param->rol_motor_param);

  r->motor.pit_motor = MOTOR_DM_GetMotor((MOTOR_DM_Param_t *)&r->param->pit_motor_param);
  r->motor.rol_motor = MOTOR_DM_GetMotor((MOTOR_DM_Param_t *)&r->param->rol_motor_param);
  if (r->motor.pit_motor == NULL || r->motor.rol_motor == NULL) return ROD_ERR;

  r->feedback.pit_motor = r->motor.pit_motor->motor.feedback;
  r->feedback.rol_motor = r->motor.rol_motor->motor.feedback;
  return ROD_OK;
}

int8_t Rod_Control(Rod_t *r, const Rod_CMD_t *cmd, uint32_t now) {
  if (r == NULL || cmd == NULL || r->param == NULL) return ROD_ERR_NULL;

  r->dt = (float)(now - r->last_wakeup) / 1000.0f;
  r->last_wakeup = now;
  if (!isfinite(r->dt) || r->dt <= 0.0f) r->dt = 0.001f;
  if (r->dt < 0.0005f) r->dt = 0.0005f;
  if (r->dt > 0.050f) r->dt = 0.050f;

  r->mode = cmd->mode;
  r->pose = cmd->pose;

  if (r->mode == ROD_MODE_RELAX) {
    Rod_ResetOutput(r);
    r->sequence.initialized = false;
    r->sequence.done = false;
    return ROD_OK;
  }

  if (r->mode == ROD_MODE_SEQUENCE) {
    if (cmd->sequence_trigger && (r->sequence.done || !r->sequence.initialized)) {
      Rod_ResetSequence(r, now);
    }
    Rod_UpdateSequence(r, cmd, now);
  } else {
    r->sequence.initialized = false;
    r->sequence.done = false;
    Rod_SetPoseSetpoint(r, r->pose);
  }

  r->out.pit_motor.angle = r->setpoint.pit_angle;
  r->out.pit_motor.velocity = Rod_Clipf(Rod_AngleError(r->setpoint.pit_angle, Rod_GetPitAngle(r)) / r->dt,
                                        -r->param->limit.max_vel, r->param->limit.max_vel);
  r->out.pit_motor.kp = r->param->limit.pit_kp;
  r->out.pit_motor.kd = r->param->limit.pit_kd;
  r->out.pit_motor.torque = Rod_GetPitGravityComp(Rod_GetPitAngle(r));

  r->out.rol_motor.angle = r->setpoint.rol_angle;
  r->out.rol_motor.velocity = Rod_Clipf(Rod_AngleError(r->setpoint.rol_angle, Rod_GetRolAngle(r)) / r->dt,
                                        -r->param->limit.max_vel, r->param->limit.max_vel);
  r->out.rol_motor.kp = r->param->limit.rol_kp;
  r->out.rol_motor.kd = r->param->limit.rol_kd;
  r->out.rol_motor.torque = 0.0f;

  return ROD_OK;
}

void Rod_Output(Rod_t *r) {
  if (r == NULL || r->param == NULL) return;

  MOTOR_DM_MITCtrl((MOTOR_DM_Param_t *)&r->param->pit_motor_param, &r->out.pit_motor);
  MOTOR_DM_MITCtrl((MOTOR_DM_Param_t *)&r->param->rol_motor_param, &r->out.rol_motor);
}

void Rod_ResetOutput(Rod_t *r) {
  if (r == NULL || r->param == NULL) return;

  memset(&r->out, 0, sizeof(r->out));
  MOTOR_DM_Relax((MOTOR_DM_Param_t *)&r->param->pit_motor_param);
  MOTOR_DM_Relax((MOTOR_DM_Param_t *)&r->param->rol_motor_param);
}
