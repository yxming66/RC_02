#include "module/armpos.h"

#include <math.h>
#include <string.h>

#include "bsp/can.h"
#include "bsp/time.h"

namespace {

constexpr float ARMPOS_TWO_PI = 2.0f * (float)M_PI;
constexpr float ARMPOS_CMD_DEADBAND = 0.05f;

static float ArmPos_Clamp(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static void ArmPos_ResetPid(ArmPos_t *a) {
  if (a == NULL) {
    return;
  }

  (void)PID_Reset(&a->pid.rmmotor_pos);
  (void)PID_Reset(&a->pid.rmmotor_vel);
}

static void ArmPos_SetTargetsToCurrent(ArmPos_t *a) {
  if (a == NULL) {
    return;
  }

  a->target.lz = a->feedback.lzmotor_feedback.current_angle;
  a->target.dm = 0.0f;
  if (a->motor.rmmotor != NULL) {
    a->target.rm = a->motor.rmmotor->gearbox_total_angle;
  } else {
    a->target.rm = 0.0f;
  }
}

static int8_t ArmPos_SetMode(ArmPos_t *a, ArmPos_Mode_t mode) {
  if (a == NULL) {
    return ARMPOS_ERR_NULL;
  }
  if (a->mode == mode) {
    return ARMPOS_OK;
  }

  ArmPos_ResetOutput(a);
  ArmPos_ResetPid(a);
  ArmPos_SetTargetsToCurrent(a);
  a->mode = mode;
  return ARMPOS_OK;
}

}  // namespace

extern "C" {

int8_t ArmPos_Init(ArmPos_t *a, ArmPos_Params_t *param, float target_freq) {
  if (a == NULL || param == NULL) {
    return ARMPOS_ERR_NULL;
  }

  memset(a, 0, sizeof(ArmPos_t));
  a->param = param;
  a->lzmotor_param = &param->lzmotor_param;
  a->dmmotor_param = &param->dmmotor_param;
  a->rmmotor_param = &param->rmmotor_param;
  a->mode = ARMPOS_MODE_RELAX;

  BSP_CAN_Init();
  MOTOR_LZ_Init();

  MOTOR_RM_Register(a->rmmotor_param);
  MOTOR_DM_Register(a->dmmotor_param);
  MOTOR_LZ_Register(a->lzmotor_param);

  PID_Init(&a->pid.rmmotor_pos, KPID_MODE_CALC_D, target_freq, &param->pid_rmmotor_pos);
  PID_Init(&a->pid.rmmotor_vel, KPID_MODE_CALC_D, target_freq, &param->pid_rmmotor_vel);

  a->motor.rmmotor = MOTOR_RM_GetMotor(a->rmmotor_param);
  MOTOR_DM_Enable(a->dmmotor_param);
  MOTOR_LZ_Enable(a->lzmotor_param);

  (void)ArmPos_UpdateFeedback(a);
  ArmPos_SetTargetsToCurrent(a);
  return ARMPOS_OK;
}

int8_t ArmPos_UpdateFeedback(ArmPos_t *a) {
  if (a == NULL) {
    return ARMPOS_ERR_NULL;
  }

  MOTOR_RM_Update(a->rmmotor_param);
  MOTOR_DM_Update(a->dmmotor_param);
  MOTOR_LZ_Update(a->lzmotor_param);

  a->motor.rmmotor = MOTOR_RM_GetMotor(a->rmmotor_param);
  a->motor.dmmotor = MOTOR_DM_GetMotor(a->dmmotor_param);
  a->motor.lzmotor = MOTOR_LZ_GetMotor(a->lzmotor_param);
  if (a->motor.rmmotor == NULL || a->motor.dmmotor == NULL || a->motor.lzmotor == NULL) {
    return ARMPOS_ERR_NULL;
  }

  a->feedback.rmmotor_feedback = a->motor.rmmotor->feedback;
  a->feedback.dmmotor_feedback = a->motor.dmmotor->motor.feedback;
  a->feedback.lzmotor_feedback = a->motor.lzmotor->lz_feedback;
  return ARMPOS_OK;
}
float rm_output;
float rm_velocity_ref;
int8_t ArmPos_Control(ArmPos_t *a, const ArmPos_CMD_t *cmd) {
  if (a == NULL || cmd == NULL || a->param == NULL) {
    return ARMPOS_ERR_NULL;
  }

  a->timer.now = BSP_TIME_Get_us() / 1000000.0f;
  a->timer.dt = (BSP_TIME_Get_us() - a->timer.lask_wakeup) / 1000000.0f;
  a->timer.lask_wakeup = BSP_TIME_Get_us();
  if (a->timer.dt <= 0.0f || a->timer.dt > 0.1f) {
    a->timer.dt = 0.001f;
  }

  ArmPos_Mode_t request_mode = cmd->mode;
  if (request_mode != ARMPOS_MODE_ACTIVE) {
    request_mode = ARMPOS_MODE_RELAX;
  }
  (void)ArmPos_SetMode(a, request_mode);

  if (a->mode == ARMPOS_MODE_RELAX) {
    ArmPos_Relax(a);
    return ARMPOS_OK;
  }

  MOTOR_DM_Enable(a->dmmotor_param);
  MOTOR_LZ_Enable(a->lzmotor_param);

  a->target.lz += cmd->lz_delta * a->param->input_scale.lz_speed * a->timer.dt;
  a->target.dm = cmd->dm_delta * a->param->input_scale.dm_speed;
  a->target.rm += cmd->rm_delta * a->param->input_scale.rm_speed * a->timer.dt;

  a->target.lz = ArmPos_Clamp(a->target.lz,
                              a->param->joint_limit.lz_min,
                              a->param->joint_limit.lz_max);
  a->target.dm = ArmPos_Clamp(a->target.dm,
                              -a->param->limit.dm_velocity_ref_limit,
                              a->param->limit.dm_velocity_ref_limit);
  a->target.rm = ArmPos_Clamp(a->target.rm,
                              a->param->joint_limit.rm_min,
                              a->param->joint_limit.rm_max);

  a->out.lzmotor.target_angle = a->target.lz;
  a->out.lzmotor.target_velocity = 0.0f;
  a->out.lzmotor.kp = 25.0f;
  a->out.lzmotor.kd = 0.05f;
  a->out.lzmotor.torque = 0.0f;

  const bool dm_hold = fabsf(cmd->dm_delta) < ARMPOS_CMD_DEADBAND;
  a->out.dmmotor.angle = 0.0f;
  a->out.dmmotor.velocity = 1.8f*a->target.dm;
  a->out.dmmotor.kp = 0.0f;
  a->out.dmmotor.kd = 10.0f*a->param->limit.dm_kd;
  a->out.dmmotor.torque = 0.0f;
  if (dm_hold && fabsf(a->feedback.dmmotor_feedback.rotor_speed) < a->param->limit.dm_hold_velocity_threshold) {
    a->out.dmmotor.velocity = 0.0f;
    a->out.dmmotor.torque = a->param->limit.dm_hold_torque;
  }

  rm_velocity_ref = PID_Calc(&a->pid.rmmotor_pos,
                                   a->target.rm,
                                   a->motor.rmmotor->gearbox_total_angle,
                                   0.0f,
                                   a->timer.dt);
  // rm_velocity_ref = ArmPos_Clamp(rm_velocity_ref,
  //                                -a->param->limit.rm_velocity_limit,
  //                                a->param->limit.rm_velocity_limit);

  float vel = a->feedback.rmmotor_feedback.rotor_speed;
  rm_output = PID_Calc(&a->pid.rmmotor_vel,
                             rm_velocity_ref,
                             vel,
                             0.0f,
                             a->timer.dt);
        a->out.rmmotor=rm_output;
  // a->out.rmmotor = ArmPos_Clamp(rm_output,
  //                               -a->param->limit.rm_output_limit,
  //                               a->param->limit.rm_output_limit);
  return ARMPOS_OK;
}

int8_t ArmPos_Output(ArmPos_t *a) {
  if (a == NULL) {
    return ARMPOS_ERR_NULL;
  }

  MOTOR_LZ_MotionControl(a->lzmotor_param, &a->out.lzmotor);
  MOTOR_DM_MITCtrl(a->dmmotor_param, &a->out.dmmotor);
  if (a->motor.rmmotor != NULL) {
    MOTOR_RM_SetOutput(a->rmmotor_param, a->out.rmmotor);
    MOTOR_RM_Ctrl(a->rmmotor_param);
  }
  return ARMPOS_OK;
}

void ArmPos_Relax(ArmPos_t *a) {
  if (a == NULL) {
    return;
  }

  ArmPos_SetTargetsToCurrent(a);
  MOTOR_LZ_Relax(a->lzmotor_param);
  MOTOR_DM_Relax(a->dmmotor_param);
  if (a->motor.rmmotor != NULL) {
    MOTOR_RM_Relax(a->rmmotor_param);
    MOTOR_RM_Ctrl(a->rmmotor_param);
  }
}

void ArmPos_ResetOutput(ArmPos_t *a) {
  if (a == NULL) {
    return;
  }
  memset(&a->out, 0, sizeof(a->out));
}

void ArmPos_SetCmdFromRc(ArmPos_CMD_t *cmd, const DR16_t *rc) {
  if (cmd == NULL) {
    return;
  }

  if (rc == NULL || !rc->header.online || rc->data.sw_l != DR16_SW_DOWN) {
    cmd->mode = ARMPOS_MODE_RELAX;
    cmd->lz_delta = 0.0f;
    cmd->dm_delta = 0.0f;
    cmd->rm_delta = 0.0f;
    return;
  }

  cmd->mode = ARMPOS_MODE_ACTIVE;
  cmd->lz_delta = rc->data.ch_r_y;
  cmd->dm_delta = -rc->data.ch_l_y;
  cmd->rm_delta = rc->data.ch_l_x;
}

}  // extern "C"