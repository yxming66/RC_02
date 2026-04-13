/*
 * Arm module: 4x3508 support motors + 2x2006 drive-wheel motors.
 */

#include "module/arm.h"

#include <math.h>
#include <string.h>

#include "bsp/can.h"
#include "bsp/time.h"

namespace {

constexpr float ARM_INTERP_MAX_SPEED_LZ = 0.4f;
constexpr float ARM_INTERP_MAX_SPEED_DM = 0.4f;
constexpr float ARM_INTERP_MAX_SPEED_RM = 1.0f;
constexpr float ARM_RM_OUTPUT_LIMIT = 0.2f;

static float Arm_Clamp(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void Arm_SetJoint3SetpointToCurrent(Arm_t *a);
static float Arm_GetJoint3RelativeAngle(const Arm_t *a);

static void Arm_InterpAxis(float target, float max_speed, float dt,
                           float *setpoint, float *interp_target, bool *active) {
  if (*interp_target != target) {
    *interp_target = target;
    *active = true;
  }

  if (!*active) {
    return;
  }

  float err = *interp_target - *setpoint;
  float step = max_speed * dt;
  if (fabsf(err) <= step) {
    *setpoint = *interp_target;
    *active = false;
  } else {
    *setpoint += step * (err > 0.0f ? 1.0f : -1.0f);
  }
}

static void Arm_GetPoint2PointTargets(const Arm_t *a, Arm_Point2PointMode_t mode,
                                      float *lz_target, float *dm_target, float *rm_target) {
  *lz_target = a->point2point[mode].lzmotor_pos;
  *dm_target = a->point2point[mode].dmmotor_pos - a->joint_angle.joint1;
  *rm_target = a->point2point[mode].rmmotor_pos - a->joint_angle.joint2;
  if (a->joint3cil.cilibrate) {
    *rm_target = Arm_Clamp(*rm_target, a->joint3cil.rmmotor_min, a->joint3cil.rmmotor_max);
  }
}

static void Arm_ApplyPoint2PointInterp(Arm_t *a,
                                       float lz_target,
                                       float dm_target,
                                       float rm_target,
                                       bool update_rm,
                                       float dt,
                                       float speed_lz,
                                       float speed_dm,
                                       float speed_rm) {
  Arm_InterpAxis(lz_target, speed_lz, dt,
                 &a->setpoint.lzmotor.target_angle,
                 &a->interp.lzmotor_target,
                 &a->interp.lzmotor_active);

  Arm_InterpAxis(dm_target, speed_dm, dt,
                 &a->setpoint.dmmotor.angle,
                 &a->interp.dmmotor_target,
                 &a->interp.dmmotor_active);

  if (update_rm) {
    Arm_InterpAxis(rm_target, speed_rm, dt,
                   &a->setpoint.rmmotor,
                   &a->interp.rmmotor_target,
                   &a->interp.rmmotor_active);
  }
}

static void Arm_ResetControllers(Arm_t *a) {
  if (a == NULL) return;

  Arm_ResetOutput(a);
  a->interp.lzmotor_active = false;
  a->interp.dmmotor_active = false;
  a->interp.rmmotor_active = false;
}

static int8_t Arm_SetMode(Arm_t *a, Arm_Mode_t mode) {
  if (a == NULL) return ARM_ERR_NULL;
  if (a->mode == mode) return ARM_OK;

  Arm_ResetControllers(a);
  a->setpoint.lzmotor.target_angle = a->joint_angle.joint1;
  a->setpoint.dmmotor.angle = a->joint_angle.joint2 - a->joint_angle.joint1;
  Arm_SetJoint3SetpointToCurrent(a);
  a->interp.lzmotor_target = a->setpoint.lzmotor.target_angle;
  a->interp.dmmotor_target = a->setpoint.dmmotor.angle;
  a->mode = mode;
  return ARM_OK;
}

static void Arm_ResetCalibrationState(Arm_t *a) {
  if (a == NULL) {
    return;
  }

  a->joint3cil.cilibrate = true;
  a->joint3cil.rmmotor_min = -a->joint3cil.user_travel_rad;
  a->joint3cil.rmmotor_max = a->joint3cil.user_travel_rad;
  Arm_SetJoint3SetpointToCurrent(a);
}

static float Arm_GetJoint3RelativeAngle(const Arm_t *a) {
  if (a == NULL || a->motor.rmmotor == NULL) {
    return 0.0f;
  }

  return a->motor.rmmotor->feedback.rotor_abs_angle - a->joint3cil.zero_offset;
}

static void Arm_SetJoint3SetpointToCurrent(Arm_t *a) {
  if (a == NULL) {
    return;
  }

  a->setpoint.rmmotor = Arm_GetJoint3RelativeAngle(a);
  a->interp.rmmotor_target = a->setpoint.rmmotor;
  a->interp.rmmotor_active = false;
}

static bool Arm_RunCalibration(Arm_t *a) {
  if (a == NULL) {
    return false;
  }

  // 简化校准：第一次联调不做自学习，直接锁定当前姿态并进入点位控制。
  a->setpoint.lzmotor.target_angle = a->joint_angle.joint1;
  a->setpoint.dmmotor.angle = a->joint_angle.joint2 - a->joint_angle.joint1;
  Arm_SetJoint3SetpointToCurrent(a);
  a->out.lzmotor = a->setpoint.lzmotor;
  a->out.dmmotor = a->setpoint.dmmotor;
  a->out.rmmotor = 0.0f;
  Arm_ResetCalibrationState(a);
  Arm_SetMode(a, ARM_MODE_POINT2POINT);
  return true;
}

}  // namespace

extern "C" {

int8_t Arm_Init(Arm_t *a, Arm_Params_t *param, float target_freq) {
  if (a == NULL || param == NULL) return ARM_ERR_NULL;

  memset(a, 0, sizeof(Arm_t));

  a->param = param;
  a->lzmotor_param = &param->lzmotor_param;
  a->dmmotor_param = &param->dmmotor_param;
  a->rmmotor_param = &param->rmmotor_param;
  a->mode = ARM_MODE_CILIBRATE;

  BSP_CAN_Init();
  MOTOR_LZ_Init();

  MOTOR_RM_Register(a->rmmotor_param);
  MOTOR_DM_Register(a->dmmotor_param);
  MOTOR_LZ_Register(a->lzmotor_param);

  PID_Init(&a->pid.rmmotor_pos, KPID_MODE_CALC_D, target_freq, &param->pid_rmmotor_pos);
  PID_Init(&a->pid.rmmotor_vel, KPID_MODE_CALC_D, target_freq, &param->pid_rmmotor_vel);

  MOTOR_RM_t *rmmotor = MOTOR_RM_GetMotor(a->rmmotor_param);
  a->motor.rmmotor = rmmotor;

  MOTOR_DM_Enable(a->dmmotor_param);
  MOTOR_LZ_Enable(a->lzmotor_param);
  a->joint3cil.cali_mode = a->param->joint3_cali.mode;
  a->joint3cil.user_travel_rad = a->param->joint3_cali.user_travel_rad;
  a->joint3cil.zero_offset = 0.0f;

  if (a->joint3cil.user_travel_rad <= 0.0f) {
    a->joint3cil.user_travel_rad = (float)M_PI;
  }

  if (rmmotor != NULL) {
    a->joint3cil.zero_offset = rmmotor->feedback.rotor_abs_angle;
  }

  Arm_ResetCalibrationState(a);
  Arm_SetJoint3SetpointToCurrent(a);

  return ARM_OK;
}

int8_t Arm_UpdateFeedback(Arm_t *a) {
  if (a == NULL) return ARM_ERR_NULL;

  MOTOR_RM_Update(a->rmmotor_param);
  MOTOR_DM_Update(a->dmmotor_param);
  MOTOR_LZ_Update(a->lzmotor_param);

  MOTOR_RM_t *rmmotor = MOTOR_RM_GetMotor(a->rmmotor_param);
  MOTOR_DM_t *dmmotor = MOTOR_DM_GetMotor(a->dmmotor_param);
  MOTOR_LZ_t *lzmotor = MOTOR_LZ_GetMotor(a->lzmotor_param);
  if (rmmotor == NULL || dmmotor == NULL || lzmotor == NULL) {
    return ARM_ERR_NULL;
  }
  a->motor.rmmotor = rmmotor;
  a->motor.dmmotor = dmmotor;
  a->motor.lzmotor = lzmotor;

  a->feedback.rmmotor_feedback = rmmotor->feedback;
  a->feedback.dmmotor_feedback = dmmotor->motor.feedback;
  a->feedback.lzmotor_feedback = lzmotor->lz_feedback;

  a->joint_angle.joint1 = a->feedback.lzmotor_feedback.current_angle;
  a->joint_angle.joint2 = a->feedback.dmmotor_feedback.rotor_abs_angle + a->joint_angle.joint1;

  float j3_rel = Arm_GetJoint3RelativeAngle(a);
  if (a->joint3cil.cilibrate) {
    j3_rel = Arm_Clamp(j3_rel, a->joint3cil.rmmotor_min, a->joint3cil.rmmotor_max);
  }
  a->joint_angle.joint3 = j3_rel + a->joint_angle.joint2;

  return ARM_OK;
}

int8_t Arm_Control(Arm_t *a, const Arm_CMD_t *a_cmd) {
  if (a == NULL || a_cmd == NULL) return ARM_ERR_NULL;

  Arm_Mode_t req_mode = a_cmd->mode;
  if ((req_mode < ARM_MODE_RELAX) || (req_mode > ARM_MODE_POINT2POINT)) {
    req_mode = ARM_MODE_RELAX;
  }
  Arm_Point2PointMode_t req_point_mode = a_cmd->point2point_mode;
  if ((req_point_mode < ARM_POINT_SLEEP) || (req_point_mode >= ARM_POINT_NONE)) {
    req_point_mode = ARM_POINT_SLEEP;
  }
  a->point2point_mode = req_point_mode;

  a->timer.now = BSP_TIME_Get_us() / 1000000.0f;
  a->timer.dt = (BSP_TIME_Get_us() - a->timer.lask_wakeup) / 1000000.0f;
  a->timer.lask_wakeup = BSP_TIME_Get_us();

  if (a->timer.dt <= 0.0f || a->timer.dt > 0.1f) {
    a->timer.dt = 0.001f;
  }

  if (req_mode == ARM_MODE_RELAX) {
    (void)Arm_SetMode(a, ARM_MODE_RELAX);
  } else if (req_mode == ARM_MODE_CILIBRATE) {
    (void)Arm_SetMode(a, ARM_MODE_CILIBRATE);
  } else {
    (void)Arm_SetMode(a, req_mode);
  }

  float j3vel, j3out;
  float j3_rel = Arm_GetJoint3RelativeAngle(a);
  if (a->joint3cil.cilibrate) {
    j3_rel = Arm_Clamp(j3_rel, a->joint3cil.rmmotor_min, a->joint3cil.rmmotor_max);
  }
  switch (a->mode) {
    case ARM_MODE_RELAX:
      Arm_Relax(a);
      break;

    case ARM_MODE_CILIBRATE:
      MOTOR_DM_Enable(a->dmmotor_param);
      MOTOR_LZ_Enable(a->lzmotor_param);
      (void)Arm_RunCalibration(a);
      break;

    case ARM_MODE_ACTIVE:
      MOTOR_DM_Enable(a->dmmotor_param);
      MOTOR_LZ_Enable(a->lzmotor_param);
      break;

    case ARM_MODE_POINT2POINT: {
      MOTOR_DM_Enable(a->dmmotor_param);
      MOTOR_LZ_Enable(a->lzmotor_param);
      float lz_target, dm_target, rm_target;
      Arm_GetPoint2PointTargets(a, a->point2point_mode, &lz_target, &dm_target, &rm_target);

      Arm_ApplyPoint2PointInterp(a, lz_target, dm_target, rm_target, true, a->timer.dt,
                                 ARM_INTERP_MAX_SPEED_LZ, ARM_INTERP_MAX_SPEED_DM,
                                 ARM_INTERP_MAX_SPEED_RM);

      a->out.lzmotor = a->setpoint.lzmotor;
      a->out.dmmotor = a->setpoint.dmmotor;

      // 关节3目标为相对 joint2 的绝对角度差，反馈同样使用解耦后的相对角。
      j3vel = PID_Calc(&a->pid.rmmotor_pos, a->setpoint.rmmotor, j3_rel, 0.0f, a->timer.dt);
      j3out = PID_Calc(&a->pid.rmmotor_vel, j3vel, a->feedback.rmmotor_feedback.rotor_speed, 0.0f, a->timer.dt);
      a->out.rmmotor = Arm_Clamp(j3out, -ARM_RM_OUTPUT_LIMIT, ARM_RM_OUTPUT_LIMIT);
      break;
    }

    case ARM_MODE_TEACH:
      break;
    case ARM_MODE_STUDY:
      break;
  }

  return ARM_OK;
}

void Arm_Teach(Arm_t *a, Arm_StudyMode_t study_mode) {
  if (a == NULL) return;
  if (study_mode == ARM_STUDY_NONE) return;
}

int8_t Arm_Output(Arm_t *a) {
  if (a == NULL) return ARM_ERR_NULL;

  a->out.lzmotor.kp = 15.0f;
  a->out.lzmotor.kd = 0.0f;

  a->out.dmmotor.kp = 25.0f;
  a->out.dmmotor.kd = 0.0f;

  MOTOR_LZ_MotionControl(a->lzmotor_param, &a->out.lzmotor);
  MOTOR_DM_MITCtrl(a->dmmotor_param, &a->out.dmmotor);

  if (a->motor.rmmotor != NULL) {
    MOTOR_RM_SetOutput(a->rmmotor_param, a->out.rmmotor);
    MOTOR_RM_Ctrl(a->rmmotor_param);
  }

  return ARM_OK;
}

void Arm_ResetOutput(Arm_t *a) {
  if (a == NULL) {
    return;
  }

  memset(&a->out, 0, sizeof(a->out));
}

void Arm_Relax(Arm_t *a) {
  if (a == NULL) return;

  a->setpoint.lzmotor.target_angle = a->joint_angle.joint1;
  a->setpoint.dmmotor.angle = a->joint_angle.joint2 - a->joint_angle.joint1;
  a->setpoint.rmmotor = a->joint_angle.joint3 - a->joint_angle.joint2;

  a->out.lzmotor.target_angle = a->joint_angle.joint1;
  a->out.dmmotor.angle = a->joint_angle.joint2 - a->joint_angle.joint1;
  a->out.rmmotor = a->joint_angle.joint3 - a->joint_angle.joint2;

  MOTOR_LZ_Relax(a->lzmotor_param);
  MOTOR_LZ_Disable(a->lzmotor_param, true);
  MOTOR_DM_Relax(a->dmmotor_param);
  if (a->motor.rmmotor != NULL) {
    MOTOR_RM_Relax(a->rmmotor_param);
    MOTOR_RM_Ctrl(a->rmmotor_param);
  }
}

}  // extern "C"