/*
 * Rod module: 2x DM4310 pick-rod mechanism.
 */

#include "module/rod.h"

#include <math.h>
#include <string.h>

static void Rod_SetGrip(bool open) {
  HAL_GPIO_WritePin(rod_GPIO_Port, rod_Pin,
                    open ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Rod_SetMotorEnable(Rod_t *r, bool enable) {
  if (r == NULL || r->param == NULL) return;

  if (enable) {
    MOTOR_DM_Enable((MOTOR_DM_Param_t *)&r->param->pit_motor_param);
    MOTOR_DM_Enable((MOTOR_DM_Param_t *)&r->param->rol_motor_param);
  } else {
    MOTOR_DM_Disable((MOTOR_DM_Param_t *)&r->param->pit_motor_param);
    MOTOR_DM_Disable((MOTOR_DM_Param_t *)&r->param->rol_motor_param);
  }
}

static float Rod_Clipf(float val, float min, float max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

static float Rod_AngleError(float target, float feedback) {
  return target - feedback;
}

static float Rod_UpdateTrapezoidAxis(float current, float target, float *velocity,
                                     float max_vel, float max_acc, float dt) {
  float error = Rod_AngleError(target, current);
  const float stop_pos_eps = 0.002f;
  const float stop_vel_eps = fmaxf(0.02f, max_acc * dt);

  if (fabsf(error) <= stop_pos_eps && fabsf(*velocity) <= stop_vel_eps) {
    *velocity = 0.0f;
    return target;
  }

  float brake_vel = sqrtf(fmaxf(0.0f, 2.0f * max_acc * fabsf(error)));
  float desired_vel = copysignf(fminf(max_vel, brake_vel), error);

  if (brake_vel < stop_vel_eps) {
    desired_vel = 0.0f;
  }

  if (fabsf(desired_vel) > brake_vel) {
    desired_vel = copysignf(brake_vel, desired_vel);
  }

  float dv = desired_vel - *velocity;
  float max_dv = max_acc * dt;
  dv = Rod_Clipf(dv, -max_dv, max_dv);
  *velocity = Rod_Clipf(*velocity + dv, -max_vel, max_vel);

  if (fabsf(error) <= stop_pos_eps && fabsf(*velocity) <= stop_vel_eps) {
    *velocity = 0.0f;
    return target;
  }

  return current + (*velocity) * dt;
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
  const Rod_Pose_t prev_pose = r->setpoint_pose;

  switch (pose) {
    case ROD_POSE_DOWN:
      r->setpoint.pit_angle = r->param->pose.pit_down_angle;
      r->setpoint.rol_angle = r->param->pose.rol_home_angle;
      break;
    case ROD_POSE_UP:
      r->setpoint.pit_angle = r->param->pose.pit_up_angle;
      r->setpoint.rol_angle = r->param->pose.rol_home_angle;
      break;
    case ROD_POSE_GRIP:
      r->setpoint.pit_angle = r->param->pose.pit_grip_angle;
      r->setpoint.rol_angle = r->param->pose.rol_home_angle;
      break;
    case ROD_POSE_LIFT:
      r->setpoint.pit_angle = r->param->pose.pit_lift_angle;
      r->setpoint.rol_angle = r->param->pose.rol_home_angle;
      break;
    case ROD_POSE_FLIP:
      r->setpoint.pit_angle = r->param->pose.pit_lift_angle;
      r->setpoint.rol_angle = r->param->pose.rol_flip_angle;
      break;
    default:
      break;
  }

  r->setpoint_pose = pose;
}

static void Rod_SyncTrajectoryToFeedback(Rod_t *r) {
  r->traj.pit_angle = Rod_GetPitAngle(r);
  r->traj.rol_angle = Rod_GetRolAngle(r);
  r->traj.pit_vel = 0.0f;
  r->traj.rol_vel = 0.0f;
  r->traj.initialized = true;
}

static void Rod_UpdateTrajectory(Rod_t *r) {
  if (!r->traj.initialized) {
    Rod_SyncTrajectoryToFeedback(r);
  }

  const float pit_max_vel = fmaxf(r->param->limit.pit_max_vel, 0.01f);
  const float pit_max_acc = fmaxf(r->param->limit.pit_max_acc, 0.01f);
  const float rol_max_vel = fmaxf(r->param->limit.rol_max_vel, 0.01f);
  const float rol_max_acc = fmaxf(r->param->limit.rol_max_acc, 0.01f);

  r->traj.pit_angle = Rod_UpdateTrapezoidAxis(
      r->traj.pit_angle, r->setpoint.pit_angle, &r->traj.pit_vel,
      pit_max_vel, pit_max_acc, r->dt);
  r->traj.rol_angle = Rod_UpdateTrapezoidAxis(
      r->traj.rol_angle, r->setpoint.rol_angle, &r->traj.rol_vel,
      rol_max_vel, rol_max_acc, r->dt);
}

static void Rod_ResetSequence(Rod_t *r, uint32_t now) {
  r->sequence.initialized = true;
  r->sequence.done = false;
  r->sequence.step = 0u;
  r->sequence.step_start_tick = now;
  Rod_SetPoseSetpoint(r, ROD_POSE_UP);
  Rod_SetGrip(true);
}

static void Rod_FinishSequence(Rod_t *r) {
  r->sequence.done = true;
  r->sequence.step = 0u;
  Rod_SetPoseSetpoint(r, ROD_POSE_DOWN);
  Rod_SetGrip(false);
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
      Rod_SetGrip(true);
      if (Rod_Arrived(r->setpoint.pit_angle, Rod_GetPitAngle(r),
                      r->param->limit.pit_arrive_threshold)) {
        r->sequence.step = 1u;
        r->sequence.step_start_tick = now;
      }
      break;
    case 1u:
      Rod_SetPoseSetpoint(r, ROD_POSE_GRIP);
      Rod_SetGrip(false);
      if (cmd->grip_done ||
          ((float)(now - r->sequence.step_start_tick) * 0.001f >=
           r->param->limit.grip_wait_time)) {
        r->sequence.step = 2u;
        r->sequence.step_start_tick = now;
      }
      break;
    case 2u:
      Rod_SetPoseSetpoint(r, ROD_POSE_LIFT);
      Rod_SetGrip(false);
      if (Rod_Arrived(r->setpoint.pit_angle, Rod_GetPitAngle(r),
                      r->param->limit.pit_arrive_threshold)) {
        r->sequence.step = 3u;
        r->sequence.step_start_tick = now;
      }
      break;
    case 3u:
      Rod_SetPoseSetpoint(r, ROD_POSE_FLIP);
      Rod_SetGrip(false);
      if (Rod_Arrived(r->setpoint.rol_angle, Rod_GetRolAngle(r),
                      r->param->limit.rol_arrive_threshold)) {
        r->sequence.step = 4u;
        r->sequence.step_start_tick = now;
      }
      break;
    default:
      Rod_SetPoseSetpoint(r, ROD_POSE_DOWN);
      Rod_SetGrip(false);
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
  r->setpoint_pose = ROD_POSE_DOWN;
  r->setpoint.rol_angle = param->pose.rol_home_angle;
  Rod_SetPoseSetpoint(r, ROD_POSE_DOWN);
  r->traj.pit_angle = param->pose.pit_down_angle;
  r->traj.rol_angle = param->pose.rol_home_angle;
  r->traj.pit_vel = 0.0f;
  r->traj.rol_vel = 0.0f;
  r->traj.initialized = false;
	
  BSP_CAN_Init();
  
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
  if (r == NULL || r->param == NULL || cmd == NULL) return ROD_ERR_NULL;

  r->dt = (float)(now - r->last_wakeup) / 1000.0f;
  r->last_wakeup = now;
  if (!isfinite(r->dt) || r->dt <= 0.0f) r->dt = 0.001f;
  if (r->dt < 0.0005f) r->dt = 0.0005f;
  if (r->dt > 0.050f) r->dt = 0.050f;

  r->mode = cmd->mode;
  r->pose = cmd->pose;

  if (r->mode == ROD_MODE_RELAX) {
    Rod_SetGrip(false);
    Rod_ResetOutput(r);
    r->sequence.initialized = false;
    r->sequence.done = false;
    r->traj.initialized = false;
    return ROD_OK;
  }

  Rod_SetMotorEnable(r, true);

  if (r->mode == ROD_MODE_SEQUENCE) {
    if (cmd->sequence_trigger && (r->sequence.done || !r->sequence.initialized)) {
      Rod_ResetSequence(r, now);
    }
    Rod_UpdateSequence(r, cmd, now);
  } else {
    r->sequence.initialized = false;
    r->sequence.done = false;
    Rod_SetGrip(r->pose == ROD_POSE_UP);
    Rod_SetPoseSetpoint(r, r->pose);
  }

  Rod_UpdateTrajectory(r);

  r->out.pit_motor.angle = r->traj.pit_angle;
  r->out.pit_motor.velocity = r->traj.pit_vel;
  r->out.pit_motor.kp = r->param->limit.pit_kp;
  r->out.pit_motor.kd = r->param->limit.pit_kd;
  r->out.pit_motor.torque = Rod_GetPitGravityComp(Rod_GetPitAngle(r));
  // r->out.pit_motor.torque = 0.0f;

  r->out.rol_motor.angle = r->traj.rol_angle;
  r->out.rol_motor.velocity = r->traj.rol_vel;
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
  MOTOR_DM_Disable((MOTOR_DM_Param_t *)&r->param->pit_motor_param);
  MOTOR_DM_Disable((MOTOR_DM_Param_t *)&r->param->rol_motor_param);
}
