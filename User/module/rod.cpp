/*
 * Rod module: 2x DM4310 pick-rod mechanism.
 */

#include "module/rod.h"

#include <math.h>
#include <string.h>

#include "bsp/can.h"
#include "device/motor/factory/motor_factory.hpp"

using mr::motor::DmJ4310Motor;
using mr::motor::MotorFactory;
using mr::motor::MotorInstanceConfig;
using mr::motor::MotorKind;
using mr::motor::MotorState;

namespace {

using RodMotor = DmJ4310Motor;

static RodMotor *&RodPitMotor(Rod_t *r) {
  return *reinterpret_cast<RodMotor **>(&r->motor.pit_motor);
}

static RodMotor *&RodRolMotor(Rod_t *r) {
  return *reinterpret_cast<RodMotor **>(&r->motor.rol_motor);
}

static void Rod_StoreMotorState(MOTOR_Feedback_t *feedback,
                                const MotorState &state) {
  if (feedback == NULL) {
    return;
  }

  feedback->rotor_abs_angle = state.position_rad;
  feedback->rotor_speed = state.velocity_rad_s;
  feedback->torque_current = state.torque_nm;
  feedback->temp = (uint8_t)state.temperature_c;
}

static void Rod_SetGrip(bool open) {
  HAL_GPIO_WritePin(rod_GPIO_Port, rod_Pin,
                    open ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Rod_SetMotorEnable(Rod_t *r, bool enable) {
  if (r == NULL || r->param == NULL) return;

  RodMotor *pit_motor = RodPitMotor(r);
  RodMotor *rol_motor = RodRolMotor(r);
  if (pit_motor == NULL || rol_motor == NULL) {
    return;
  }

  if (enable) {
    (void)pit_motor->Enable();
    (void)rol_motor->Enable();
  } else {
    (void)pit_motor->Disable();
    (void)rol_motor->Disable();
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
  return r->feedback.pit_motor.rotor_abs_angle;
}

static float Rod_GetRolAngle(const Rod_t *r) {
  return r->feedback.rol_motor.rotor_abs_angle;
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

static void Rod_ResetComp(Rod_t *r) {
  r->comp.pit_err_i = 0.0f;
}

static float Rod_GetPitIntegralComp(Rod_t *r) {
  const float err = Rod_AngleError(r->traj.pit_angle, Rod_GetPitAngle(r));

  if (fabsf(err) <= ROD_PIT_I_ACTIVE_ERR_RAD) {
    r->comp.pit_err_i += err * r->dt;
    r->comp.pit_err_i = Rod_Clipf(r->comp.pit_err_i, -ROD_PIT_I_LIMIT,
                                  ROD_PIT_I_LIMIT);
  } else {
    r->comp.pit_err_i = 0.0f;
  }

  return ROD_PIT_I_GAIN * r->comp.pit_err_i;
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
    case ROD_POSE_READY:
      r->setpoint.pit_angle = r->param->pose.pit_down_angle;
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
    // Rod_FinishSequence(r);
    return;
  }

  switch (r->sequence.step) {
    case 0u:
      Rod_SetPoseSetpoint(r, ROD_POSE_UP);
      Rod_SetGrip(true);
      if ((float)(now - r->sequence.step_start_tick) * 0.001f >=
          r->param->limit.grip_open_wait_time) {
        r->sequence.step = 1u;
        r->sequence.step_start_tick = now;
      }
      break;
    case 1u:
      Rod_SetPoseSetpoint(r, ROD_POSE_UP);
      Rod_SetGrip(true);
      if (Rod_Arrived(r->setpoint.pit_angle, Rod_GetPitAngle(r),
                      r->param->limit.pit_arrive_threshold)) {
        r->sequence.step = 2u;
        r->sequence.step_start_tick = now;
      }
      break;
    case 2u:
      Rod_SetPoseSetpoint(r, ROD_POSE_GRIP);
      Rod_SetGrip(false);
      if (cmd->grip_done ||
          ((float)(now - r->sequence.step_start_tick) * 0.001f >=
           r->param->limit.grip_wait_time)) {
        r->sequence.step = 3u;
        r->sequence.step_start_tick = now;
      }
      break;
    case 3u:
      Rod_SetPoseSetpoint(r, ROD_POSE_LIFT);
      Rod_SetGrip(false);
      if (Rod_Arrived(r->setpoint.pit_angle, Rod_GetPitAngle(r),
                      r->param->limit.pit_arrive_threshold)) {
        r->sequence.step = 4u;
        r->sequence.step_start_tick = now;
      }
      break;
    case 4u:
      Rod_SetPoseSetpoint(r, ROD_POSE_LIFT);
      Rod_SetGrip(false);
      if ((float)(now - r->sequence.step_start_tick) * 0.001f >=
          r->param->limit.rol_flip_wait_time) {
        r->sequence.step = 5u;
        r->sequence.step_start_tick = now;
      }
      break;
    case 5u:
      Rod_SetPoseSetpoint(r, ROD_POSE_FLIP);
      Rod_SetGrip(false);
      if (Rod_Arrived(r->setpoint.rol_angle, Rod_GetRolAngle(r),
                      r->param->limit.rol_arrive_threshold)) {
        r->sequence.step = 6u;
        r->sequence.step_start_tick = now;
      }
      break;
    case 6u:
      r->mode = ROD_MODE_ACTIVE;
      Rod_SetPoseSetpoint(r, ROD_POSE_READY);
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

}  // namespace

extern "C" {

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

  const auto pit_cfg =
      MotorInstanceConfig<MotorKind::DM>::FromVendorParam(param->pit_motor_param);
  const auto rol_cfg =
      MotorInstanceConfig<MotorKind::DM>::FromVendorParam(param->rol_motor_param);

  RodPitMotor(r) = MotorFactory::Create<MotorKind::DM, mr::motor::MotorModel::J4310>(pit_cfg);
  RodRolMotor(r) = MotorFactory::Create<MotorKind::DM, mr::motor::MotorModel::J4310>(rol_cfg);
  if (RodPitMotor(r) == NULL || RodRolMotor(r) == NULL) {
    return ROD_ERR;
  }

  if (RodPitMotor(r)->Register() != DEVICE_OK ||
      RodRolMotor(r)->Register() != DEVICE_OK) {
    return ROD_ERR;
  }

  Rod_SetMotorEnable(r, true);

  return ROD_OK;
}

int8_t Rod_UpdateFeedback(Rod_t *r) {
  if (r == NULL || r->param == NULL) return ROD_ERR_NULL;

  RodMotor *pit_motor = RodPitMotor(r);
  RodMotor *rol_motor = RodRolMotor(r);
  if (pit_motor == NULL || rol_motor == NULL) return ROD_ERR;

  (void)pit_motor->Update();
  (void)rol_motor->Update();

  Rod_StoreMotorState(&r->feedback.pit_motor, pit_motor->GetState());
  Rod_StoreMotorState(&r->feedback.rol_motor, rol_motor->GetState());
  return ROD_OK;
}

int8_t Rod_Control(Rod_t *r, const Rod_CMD_t *cmd, uint32_t now) {
  if (r == NULL || r->param == NULL || cmd == NULL) return ROD_ERR_NULL;

  const Rod_Mode_t prev_mode = r->mode;

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
    Rod_ResetComp(r);
    return ROD_OK;
  }

  Rod_SetMotorEnable(r, true);

  if (prev_mode != r->mode) {
    Rod_ResetComp(r);
  }

  if (r->mode == ROD_MODE_SEQUENCE) {
    if (!r->sequence.initialized ||
        (cmd->sequence_trigger && r->sequence.done)) {
      Rod_ResetSequence(r, now);
    }
    Rod_UpdateSequence(r, cmd, now);
  } else {
    r->sequence.initialized = false;
    r->sequence.done = false;
    Rod_SetGrip((r->pose == ROD_POSE_UP || r->pose == ROD_POSE_READY) &&
                fabsf(Rod_GetPitAngle(r) - r->setpoint.pit_angle) < 0.5f);
    Rod_SetPoseSetpoint(r, r->pose);
  }

  Rod_UpdateTrajectory(r);

  r->out.pit_motor.angle = r->traj.pit_angle;
  r->out.pit_motor.velocity = r->traj.pit_vel;
  r->out.pit_motor.kp = r->param->limit.pit_kp;
  r->out.pit_motor.kd = r->param->limit.pit_kd;
  r->out.pit_motor.torque = Rod_GetPitGravityComp(Rod_GetPitAngle(r)) +
                            Rod_GetPitIntegralComp(r);

  r->out.rol_motor.angle = r->traj.rol_angle;
  r->out.rol_motor.velocity = r->traj.rol_vel;
  r->out.rol_motor.kp = r->param->limit.rol_kp;
  r->out.rol_motor.kd = r->param->limit.rol_kd;
  r->out.rol_motor.torque = 0.0f;

  return ROD_OK;
}

void Rod_Output(Rod_t *r) {
  if (r == NULL || r->param == NULL) return;

  RodMotor *pit_motor = RodPitMotor(r);
  RodMotor *rol_motor = RodRolMotor(r);
  if (pit_motor == NULL || rol_motor == NULL) return;

  (void)pit_motor->SetMIT(r->out.pit_motor.angle, r->out.pit_motor.velocity,
                          r->out.pit_motor.kp, r->out.pit_motor.kd,
                          r->out.pit_motor.torque);
  (void)pit_motor->CommitCommand();

  (void)rol_motor->SetMIT(r->out.rol_motor.angle, r->out.rol_motor.velocity,
                          r->out.rol_motor.kp, r->out.rol_motor.kd,
                          r->out.rol_motor.torque);
  (void)rol_motor->CommitCommand();
}

void Rod_ResetOutput(Rod_t *r) {
  if (r == NULL || r->param == NULL) return;

  memset(&r->out, 0, sizeof(r->out));
  Rod_ResetComp(r);
  Rod_SetMotorEnable(r, false);
}

}  // extern "C"
