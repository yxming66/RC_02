/*
 * Arm module: 4x3508 support motors + 2x2006 drive-wheel motors.
 */

#include "module/arm.h"

#include <math.h>
#include <string.h>

#include "bsp/can.h"
#include "bsp/time.h"

#include <new>

namespace {

constexpr float ARM_J12_SLEEP_ARRIVE_TOL = 0.04f;
constexpr float ARM_INTERP_MAX_SPEED_LZ = 0.4f;
constexpr float ARM_INTERP_MAX_SPEED_DM = 0.4f;
constexpr float ARM_INTERP_MAX_SPEED_RM = 1.0f;

static float Arm_Clamp(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static uint32_t Arm_NowMs(void) {
  return (uint32_t)(BSP_TIME_Get_us() / 1000ULL);
}

static void Arm_SetJoint3SetpointToCurrent(Arm_t *a);

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

  if (a->mode == ARM_MODE_CILIBRATE && a->joint3cil.soft_limit_learning != NULL &&
      a->joint3cil.soft_limit_learning->IsRunning()) {
    a->joint3cil.soft_limit_learning->Cancel();
  }
  if (a->mode == ARM_MODE_CILIBRATE && a->joint3cil.self_test != NULL &&
      a->joint3cil.self_test->IsRunning()) {
    a->joint3cil.self_test->Cancel();
  }

  Arm_ResetControllers(a);
  a->mode = mode;
  return ARM_OK;
}

static void Arm_InitJoint3Packages(Arm_t *a) {
  if (a == NULL || a->joint3cil.motor == NULL) {
    return;
  }

  mrobot::MotorSelfTest::Params self_test_param = mrobot::MotorSelfTest::Params::Default();
  self_test_param.active_probe_enable = true;
  self_test_param.probe_current = 0.6f;
  self_test_param.probe_duration_ms = 100;

  mrobot::MotorSoftLimitLearning::Params soft_limit_param = mrobot::MotorSoftLimitLearning::Params::Default();
  soft_limit_param.seek_current = 0.8f;
  soft_limit_param.stall_velocity_threshold = 0.1f;
  soft_limit_param.stall_current_threshold = 0.3f;
  soft_limit_param.min_valid_range_rad = 0.3f;
  soft_limit_param.return_velocity_limit_rad_per_sec = 0.8f;

  if (a->joint3cil.self_test_storage == NULL || a->joint3cil.soft_limit_learning_storage == NULL) {
    return;
  }

  a->joint3cil.self_test = new (a->joint3cil.self_test_storage)
      mrobot::MotorSelfTest(a->joint3cil.motor, self_test_param);
  a->joint3cil.soft_limit_learning = new (a->joint3cil.soft_limit_learning_storage)
      mrobot::MotorSoftLimitLearning(a->joint3cil.motor, soft_limit_param);
  a->joint3cil.self_test_started = false;
  a->joint3cil.soft_limit_started = false;
}

static void Arm_ResetCalibrationState(Arm_t *a) {
  if (a == NULL) {
    return;
  }

  a->joint3cil.cilibrate = false;
  a->joint3cil.self_test_started = false;
  a->joint3cil.soft_limit_started = false;
  a->joint3cil.rmmotor_min = 0.0f;
  a->joint3cil.rmmotor_max = 0.0f;

  if (a->joint3cil.motor != NULL) {
    a->joint3cil.motor->ClearZeroPoint();
  }

  Arm_SetJoint3SetpointToCurrent(a);
  Arm_InitJoint3Packages(a);
}

static void Arm_SetJoint3SetpointToCurrent(Arm_t *a) {
  if (a == NULL || a->joint3cil.motor == NULL) {
    return;
  }

  a->setpoint.rmmotor = a->joint3cil.motor->GetPositionRad();
  a->interp.rmmotor_target = a->setpoint.rmmotor;
  a->interp.rmmotor_active = false;
}

static bool Arm_RunCalibration(Arm_t *a) {
  if (a == NULL || a->joint3cil.motor == NULL ||
      a->joint3cil.self_test == NULL || a->joint3cil.soft_limit_learning == NULL) {
    return false;
  }

  float sleep_joint1_target = a->point2point[ARM_POINT_SLEEP].lzmotor_pos;
  float sleep_joint2_target = a->point2point[ARM_POINT_SLEEP].dmmotor_pos;
  bool j12_sleep_arrived = false;
  {
    float sleep_lz_target, sleep_dm_target, sleep_rm_target;
    Arm_GetPoint2PointTargets(a, ARM_POINT_SLEEP,
                              &sleep_lz_target,
                              &sleep_dm_target,
                              &sleep_rm_target);
    Arm_ApplyPoint2PointInterp(a,
                               sleep_lz_target,
                               sleep_dm_target,
                               sleep_rm_target,
                               false,
                               a->timer.dt,
                               ARM_INTERP_MAX_SPEED_LZ,
                               ARM_INTERP_MAX_SPEED_DM,
                               ARM_INTERP_MAX_SPEED_RM);
    a->out.lzmotor = a->setpoint.lzmotor;
    a->out.dmmotor = a->setpoint.dmmotor;

    j12_sleep_arrived = (fabsf(sleep_joint1_target - a->joint_angle.joint1) <= ARM_J12_SLEEP_ARRIVE_TOL) &&
                        (fabsf(sleep_joint2_target - a->joint_angle.joint2) <= ARM_J12_SLEEP_ARRIVE_TOL);
  }

  if (!j12_sleep_arrived) {
    a->out.rmmotor = 0.0f;
    return false;
  }

  const uint32_t now_ms = Arm_NowMs();
  mrobot::MotorSelfTest::Result self_test_result = a->joint3cil.self_test->GetResult();
  if (!a->joint3cil.self_test_started) {
    a->joint3cil.self_test->Start(now_ms);
    a->joint3cil.self_test_started = true;
    self_test_result = a->joint3cil.self_test->GetResult();
  }
  if (a->joint3cil.self_test->IsRunning()) {
    self_test_result = a->joint3cil.self_test->Update(now_ms);
  }

  if (self_test_result != mrobot::MotorSelfTest::Result::PASS) {
    if (self_test_result != mrobot::MotorSelfTest::Result::RUNNING) {
      Arm_ResetCalibrationState(a);
    }
    a->out.rmmotor = 0.0f;
    return false;
  }

  mrobot::MotorSoftLimitLearning::Result learn_result = a->joint3cil.soft_limit_learning->GetResult();
  if (!a->joint3cil.soft_limit_started) {
    a->joint3cil.soft_limit_learning->Start(now_ms);
    a->joint3cil.soft_limit_started = true;
    learn_result = a->joint3cil.soft_limit_learning->GetResult();
  }
  if (a->joint3cil.soft_limit_learning->IsRunning()) {
    learn_result = a->joint3cil.soft_limit_learning->Update(now_ms);
  }

  if (learn_result == mrobot::MotorSoftLimitLearning::Result::PASS) {
    mrobot::MotorSoftLimitLearning::LearnedRange range = a->joint3cil.soft_limit_learning->GetLearnedRange();
    if (range.valid) {
      a->joint3cil.rmmotor_min = range.min_position_rad;
      a->joint3cil.rmmotor_max = range.max_position_rad;
      a->joint3cil.motor->SetZeroPoint();
      a->joint3cil.cilibrate = true;
      a->joint3cil.self_test_started = false;
      a->joint3cil.soft_limit_started = false;
      Arm_SetJoint3SetpointToCurrent(a);
      Arm_SetMode(a, ARM_MODE_POINT2POINT);
      a->out.rmmotor = 0.0f;
      return true;
    }
  }

  if (learn_result != mrobot::MotorSoftLimitLearning::Result::RUNNING) {
    Arm_ResetCalibrationState(a);
  }

  a->out.rmmotor = 0.0f;
  return false;
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

  MOTOR_DM_Register(a->dmmotor_param);
  MOTOR_LZ_Register(a->lzmotor_param);

  PID_Init(&a->pid.rmmotor_pos, KPID_MODE_CALC_D, target_freq, &param->pid_rmmotor_pos);
  PID_Init(&a->pid.rmmotor_vel, KPID_MODE_CALC_D, target_freq, &param->pid_rmmotor_vel);

  MOTOR_DM_Enable(a->dmmotor_param);
  MOTOR_LZ_Enable(a->lzmotor_param);

  static mrobot::Motor joint3_motor(mrobot::MotorConfig::FromRM("arm_joint3", *a->rmmotor_param));
  alignas(mrobot::MotorSelfTest) static unsigned char joint3_self_test_storage[sizeof(mrobot::MotorSelfTest)];
  alignas(mrobot::MotorSoftLimitLearning) static unsigned char joint3_soft_limit_storage[sizeof(mrobot::MotorSoftLimitLearning)];
  a->joint3cil.motor = &joint3_motor;
  a->joint3cil.self_test_storage = joint3_self_test_storage;
  a->joint3cil.soft_limit_learning_storage = joint3_soft_limit_storage;
  (void)a->joint3cil.motor->Register();
  (void)a->joint3cil.motor->Enable();
  Arm_ResetCalibrationState(a);
  Arm_SetJoint3SetpointToCurrent(a);

  return ARM_OK;
}

int8_t Arm_UpdateFeedback(Arm_t *a) {
  if (a == NULL) return ARM_ERR_NULL;

  MOTOR_DM_Update(a->dmmotor_param);
  MOTOR_LZ_Update(a->lzmotor_param);

  MOTOR_DM_t *dmmotor = MOTOR_DM_GetMotor(a->dmmotor_param);
  MOTOR_LZ_t *lzmotor = MOTOR_LZ_GetMotor(a->lzmotor_param);
  if (dmmotor == NULL || lzmotor == NULL) {
    return ARM_ERR_NULL;
  }
  a->motor.rmmotor = NULL;
  a->motor.dmmotor = dmmotor;
  a->motor.lzmotor = lzmotor;

  memset(&a->feedback.rmmotor_feedback, 0, sizeof(a->feedback.rmmotor_feedback));
  a->feedback.dmmotor_feedback = dmmotor->motor.feedback;
  a->feedback.lzmotor_feedback = lzmotor->lz_feedback;

  a->joint_angle.joint1 = a->feedback.lzmotor_feedback.current_angle;
  a->joint_angle.joint2 = a->feedback.dmmotor_feedback.rotor_abs_angle + a->joint_angle.joint1;

  if (a->joint3cil.motor != NULL) {
    (void)a->joint3cil.motor->Update();
    a->feedback.rmmotor_feedback.rotor_abs_angle = a->joint3cil.motor->GetRawAngleRad();
    a->feedback.rmmotor_feedback.rotor_speed = a->joint3cil.motor->GetVelocityRadPerSec();
    a->feedback.rmmotor_feedback.torque_current = a->joint3cil.motor->GetTorqueCurrentAmp();
    a->feedback.rmmotor_feedback.temp = (uint8_t)a->joint3cil.motor->GetTemperatureCelsius();
  }

  float j3_rel = (a->joint3cil.motor != NULL) ? a->joint3cil.motor->GetPositionRad()
                                              : 0.0f;
  if (a->joint3cil.cilibrate) {
    j3_rel = Arm_Clamp(j3_rel, a->joint3cil.rmmotor_min, a->joint3cil.rmmotor_max);
  }
  a->joint_angle.joint3 = j3_rel + a->joint_angle.joint2;

  return ARM_OK;
}

int8_t Arm_Control(Arm_t *a, const Arm_CMD_t *a_cmd) {
  if (a == NULL || a_cmd == NULL) return ARM_ERR_NULL;

  a->timer.now = BSP_TIME_Get_us() / 1000000.0f;
  a->timer.dt = (BSP_TIME_Get_us() - a->timer.lask_wakeup) / 1000000.0f;
  a->timer.lask_wakeup = BSP_TIME_Get_us();

  if (a->timer.dt <= 0.0f || a->timer.dt > 0.1f) {
    a->timer.dt = 0.001f;
  }

  if (!a->joint3cil.cilibrate) {
    if (a_cmd->mode == ARM_MODE_RELAX) {
      a->mode = ARM_MODE_RELAX;
    } else {
      a->mode = ARM_MODE_CILIBRATE;
    }
  } else {
    a->mode = a_cmd->mode;
  }

  float j3vel, j3out;
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

      j3vel = PID_Calc(&a->pid.rmmotor_pos, a->setpoint.rmmotor, a->joint_angle.joint3, 0.0f, a->timer.dt);
      j3out = PID_Calc(&a->pid.rmmotor_vel, j3vel, a->feedback.rmmotor_feedback.rotor_speed, 0.0f, a->timer.dt);
      a->out.rmmotor = j3out;
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

  if (a->mode != ARM_MODE_CILIBRATE) {
    if (a->joint3cil.motor != NULL) {
      (void)a->joint3cil.motor->CurrentControl(a->out.rmmotor);
    }
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
  if (a->joint3cil.motor != NULL) {
    (void)a->joint3cil.motor->Relax();
  }
}

}  // extern "C"