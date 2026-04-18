/*
 * Pole module: 4x3508 support motors.
 */

#include "module/pole.h"

#include <math.h>
#include <new>
#include <string.h>

#include "bsp/can.h"
#include "device/motor.h"
#include "device/motor/factory/motor_factory.hpp"
#include "device/motor/packages/controller/motor_controller.hpp"

using mrobot::motor::MotorControllerConfig;
using mrobot::motor::MotorControllerT;
using mrobot::motor::MotorFactory;
using mrobot::motor::MotorInstallSpec;
using mrobot::motor::MotorInstanceConfig;
using mrobot::motor::MotorState;
using mrobot::motor::RmM3508Motor;

using PoleMotor = RmM3508Motor;
using PoleMotorController = MotorControllerT<PoleMotor>;

namespace {

constexpr float kPoleVelFeedbackLpfDefaultHz = 10.0f;
constexpr float kPoleOutputLpfDefaultHz = 18.0f;

alignas(PoleMotorController) static unsigned char g_pole_controller_storage[POLE_MOTOR_NUM][sizeof(PoleMotorController)];

static PoleMotor *& PoleMotorHandle(Pole_t *c, uint8_t idx) {
  return *reinterpret_cast<PoleMotor **>(&c->motors[idx]);
}

static PoleMotorController *& PoleController(Pole_t *c, uint8_t idx) {
  return *reinterpret_cast<PoleMotorController **>(&c->controllers[idx]);
}

static MotorInstallSpec PoleInstallSpec(const Pole_t *c, uint8_t idx) {
  MotorInstallSpec spec {};
  spec.external_ratio = c->param->motor_install[idx].external_ratio;
  spec.reverse_output = c->param->motor_install[idx].reverse_output;
  return spec;
}

static void PoleStoreMotorState(Pole_t *c, uint8_t idx, const MotorState& state) {
  c->feedback.motor[idx].rotor_abs_angle = state.position_single_turn_rad;
  c->feedback.motor[idx].rotor_speed = state.velocity_rad_s;
  c->feedback.motor[idx].torque_current = state.torque_nm;
  c->feedback.motor[idx].temp = state.temperature_c;
}

}

static float Pole_GetSupportAngle(const Pole_t *c, uint8_t idx) {
  return PoleController(const_cast<Pole_t *>(c), idx)->GetState().position_rad;
}

static float Pole_Clipf(float val, float min, float max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

static float Pole_MoveTowards(float current, float target, float max_step) {
  if (max_step <= 0.0f) return target;

  float delta = target - current;
  if (delta > max_step) return current + max_step;
  if (delta < -max_step) return current - max_step;
  return target;
}

static float Pole_GetSoftLimitedSpeed(const Pole_t *c, uint8_t idx, float desired_speed, float fb_angle) {
  if (c == NULL || c->param == NULL) return desired_speed;

  const float base_limit = fmaxf(c->param->limit.support_lift_speed, 0.0f);
  const float soft_zone = fmaxf(c->param->limit.support_limit_soft_zone, 0.0f);
  if (soft_zone <= 0.0f || base_limit <= 0.0f) {
    return Pole_Clipf(desired_speed, -base_limit, base_limit);
  }

  const float dist_to_lower = fb_angle - c->support_angle.lower[idx];
  const float dist_to_upper = c->support_angle.upper[idx] - fb_angle;

  float allow_up = base_limit;
  float allow_down = base_limit;

  if (dist_to_upper <= soft_zone) {
    const float ratio = Pole_Clipf(dist_to_upper / soft_zone, 0.12f, 1.0f);
    allow_up *= ratio;
  }
  if (dist_to_lower <= soft_zone) {
    const float ratio = Pole_Clipf(dist_to_lower / soft_zone, 0.12f, 1.0f);
    allow_down *= ratio;
  }

  if (desired_speed >= 0.0f) {
    return Pole_Clipf(desired_speed, -base_limit, allow_up);
  }
  return Pole_Clipf(desired_speed, -allow_down, base_limit);
}

static bool Pole_ShouldHoldLowerLimit(const Pole_t *c, uint8_t idx, float fb_angle, float fb_speed) {
  if (c == NULL || c->param == NULL) return false;

  const float hold_zone = fmaxf(c->param->limit.support_hold_zone, 0.0f);
  const float speed_threshold = fmaxf(c->param->limit.support_hold_speed_threshold, 0.0f);
  if (hold_zone <= 0.0f) return false;

  const float dist_to_lower = fb_angle - c->support_angle.lower[idx];
  if (dist_to_lower > hold_zone) return false;
  if (fabsf(fb_speed) > speed_threshold) return false;
  return true;
}

static float Pole_GetVelFeedbackLpfCutoffHz(const Pole_t *c) {
  if (c == NULL || c->param == NULL) return kPoleVelFeedbackLpfDefaultHz;
  const float dedicated_cutoff_hz = c->param->filter.support_vel_feedback_cutoff_hz;
  if (dedicated_cutoff_hz > 0.0f) {
    return dedicated_cutoff_hz;
  }

  const float legacy_cutoff_hz = c->param->pid.support_vel_pid.d_cutoff_freq;
  return (legacy_cutoff_hz > 0.0f) ? legacy_cutoff_hz : kPoleVelFeedbackLpfDefaultHz;
}

static float Pole_GetOutputLpfCutoffHz(const Pole_t *c) {
  if (c == NULL || c->param == NULL) return kPoleOutputLpfDefaultHz;
  const float dedicated_cutoff_hz = c->param->filter.support_output_cutoff_hz;
  return (dedicated_cutoff_hz > 0.0f) ? dedicated_cutoff_hz : kPoleOutputLpfDefaultHz;
}

static float Pole_GetSupportSpeedFeedbackFiltered(Pole_t *c, uint8_t idx) {
  if (c == NULL || idx >= POLE_SUPPORT_MOTOR_NUM) return 0.0f;
  c->feedback.support_vel_filtered[idx] = LowPassFilter2p_Apply(
      &c->filter.support_vel_in[idx], c->feedback.motor[idx].rotor_speed);
  return c->feedback.support_vel_filtered[idx];
}

static bool Pole_UseLegacyNormalizedOutput(const Pole_t *c) {
  return (c != NULL && c->param != NULL && c->param->output.use_legacy_normalized_output);
}

static float Pole_ToLegacyNormalizedOutput(const Pole_t *c, float output_value) {
  if (c == NULL || c->param == NULL) return 0.0f;

  const float full_scale = fmaxf(c->param->limit.max_current, 1e-6f);
  return Pole_Clipf(output_value / full_scale, -1.0f, 1.0f);
}

static void Pole_ResetControllers(Pole_t *c) {
  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    PID_Reset(&c->pid.support_pos[i]);
    PID_Reset(&c->pid.support_vel[i]);
    LowPassFilter2p_Reset(&c->filter.support_vel_in[i], c->feedback.motor[i].rotor_speed);
    LowPassFilter2p_Reset(&c->filter.support_out[i], 0.0f);
    c->feedback.support_vel_filtered[i] = c->feedback.motor[i].rotor_speed;
  }
}

static int8_t Pole_SetMode(Pole_t *c, Pole_Mode_t mode) {
  if (c == NULL) return POLE_ERR_NULL;
  if (c->mode == mode) return POLE_OK;

  Pole_ResetControllers(c);
  c->mode = mode;
  return POLE_OK;
}

extern "C" {

int8_t Pole_Init(Pole_t *c, const Pole_Params_t *param, float target_freq) {
  if (c == NULL || param == NULL) return POLE_ERR_NULL;

  memset(c, 0, sizeof(Pole_t));
  BSP_CAN_Init();

  c->param = param;
  c->mode = POLE_MODE_RELAX;

  const float vel_fb_cutoff_hz = Pole_GetVelFeedbackLpfCutoffHz(c);
  const float output_cutoff_hz = Pole_GetOutputLpfCutoffHz(c);

  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    PoleMotorHandle(c, i) = nullptr;
    PoleController(c, i) = nullptr;
    c->out.motor[i] = 0.0f;
    c->feedback.motor[i] = {};

    PID_Init(&c->pid.support_pos[i], KPID_MODE_NO_D, target_freq,
             &param->pid.support_pos_pid);
    PID_Init(&c->pid.support_vel[i], KPID_MODE_NO_D, target_freq,
             &param->pid.support_vel_pid);
    LowPassFilter2p_Init(&c->filter.support_vel_in[i], target_freq, vel_fb_cutoff_hz);
    LowPassFilter2p_Reset(&c->filter.support_vel_in[i], 0.0f);
    LowPassFilter2p_Init(&c->filter.support_out[i], target_freq, output_cutoff_hz);
    LowPassFilter2p_Reset(&c->filter.support_out[i], 0.0f);

    const auto motor_config = MotorInstanceConfig<mrobot::motor::MotorKind::RM>::FromVendorParam(c->param->motor_param[i]);
    PoleMotorHandle(c, i) = MotorFactory::Create<mrobot::motor::MotorKind::RM, mrobot::motor::MotorModel::M3508>(motor_config, PoleInstallSpec(c, i));
    if (PoleMotorHandle(c, i) == nullptr) {
      return POLE_ERR;
    }

    const MotorControllerConfig controller_config = {
      .velocity_pid = &param->pid.support_vel_pid,
      .position_pid = &param->pid.support_pos_pid,
      .sample_freq = target_freq,
      .position_to_velocity_limit = param->limit.support_lift_speed,
      .velocity_to_torque_limit = param->limit.max_current,
    };

    PoleController(c, i) = new (g_pole_controller_storage[i]) PoleMotorController(*PoleMotorHandle(c, i), controller_config);
    if (PoleController(c, i)->Register() != DEVICE_OK) {
      return POLE_ERR;
    }
    if (PoleController(c, i)->Enable() != DEVICE_OK) {
      return POLE_ERR;
    }
  }

  return POLE_OK;
}

int8_t Pole_UpdateFeedback(Pole_t *c) {
  if (c == NULL || c->param == NULL) return POLE_ERR_NULL;

  float sum = 0.0f;
  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    if (PoleController(c, i) == nullptr) return POLE_ERR;
    if (PoleController(c, i)->Update() != DEVICE_OK) return POLE_ERR;

    PoleStoreMotorState(c, i, PoleController(c, i)->GetState());
    if (i < POLE_SUPPORT_MOTOR_NUM) {
      sum += Pole_GetSupportAngle(c, i);
    }
  }

  c->feedback.support_angle_avg = sum / (float)POLE_SUPPORT_MOTOR_NUM;
  return POLE_OK;
}

int8_t Pole_Control(Pole_t *c, const Pole_CMD_t *c_cmd, uint32_t now) {
  if (c == NULL || c_cmd == NULL || c->param == NULL) return POLE_ERR_NULL;

  c->dt = (float)(now - c->last_wakeup) / 1000.0f;
  c->last_wakeup = now;
  if (!isfinite(c->dt) || c->dt <= 0.0f) c->dt = 0.001f;
  if (c->dt < 0.0005f) c->dt = 0.0005f;
  if (c->dt > 0.050f) c->dt = 0.050f;

  Pole_SetMode(c, c_cmd->mode);

  if (c->mode == POLE_MODE_RELAX) {
    for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
      c->out.motor[i] = 0.0f;
    }
    return POLE_OK;
  }

  if (!c->support_angle.calibrated) {
    for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
      float now_angle = Pole_GetSupportAngle(c, i);
      c->support_angle.lower[i] = now_angle;
      c->support_angle.upper[i] = now_angle + c->param->limit.support_total_travel;
    }
    c->support_angle.final_target_lift[0] = 0.0f;
    c->support_angle.final_target_lift[1] = 0.0f;
    c->support_angle.tracked_target_lift[0] = 0.0f;
    c->support_angle.tracked_target_lift[1] = 0.0f;
    c->support_angle.calibrated = true;
    Pole_ResetControllers(c);
  }

  for (uint8_t side = 0; side < 2u; side++) {
    float default_speed = c->param->limit.support_lift_speed;
    float auto_speed = c_cmd->auto_lift_speed[side];
    float speed_limit = (auto_speed > 0.0f) ? auto_speed : default_speed;

    if (c_cmd->auto_target_enable[side]) {
      c->support_angle.final_target_lift[side] = c_cmd->auto_target_lift[side];
    } else {
      c->support_angle.final_target_lift[side] += c_cmd->lift[side] * default_speed * c->dt;
    }

    c->support_angle.final_target_lift[side] = Pole_Clipf(
        c->support_angle.final_target_lift[side], 0.0f, c->param->limit.support_total_travel);
    c->support_angle.tracked_target_lift[side] = Pole_MoveTowards(
        c->support_angle.tracked_target_lift[side], c->support_angle.final_target_lift[side],
        speed_limit * c->dt);
    c->support_angle.tracked_target_lift[side] = Pole_Clipf(
        c->support_angle.tracked_target_lift[side], 0.0f, c->param->limit.support_total_travel);
  }

  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    uint8_t side = (i < 2u) ? 0u : 1u;
    float target = c->support_angle.lower[i] + c->support_angle.tracked_target_lift[side];
    c->setpoint.support_target_angle[i] = Pole_Clipf(target, c->support_angle.lower[i],
                                                     c->support_angle.upper[i]);
  }
  float vel[4];
  float out[4];
  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    float fb_angle = Pole_GetSupportAngle(c, i);
    float fb_speed = Pole_GetSupportSpeedFeedbackFiltered(c, i);

    if (Pole_ShouldHoldLowerLimit(c, i, fb_angle, fb_speed) &&
        c->setpoint.support_target_angle[i] <= (c->support_angle.lower[i] + c->param->limit.support_hold_zone)) {
      c->setpoint.support_target_angle[i] = c->support_angle.lower[i];
      PID_Reset(&c->pid.support_pos[i]);
    }

     vel[i] = PID_Calc(&c->pid.support_pos[i], c->setpoint.support_target_angle[i],
                         fb_angle, 0.0f, c->dt);
     vel[i] = Pole_GetSoftLimitedSpeed(c, i, vel[i], fb_angle);
     out[i] = PID_Calc(&c->pid.support_vel[i], vel[i],
                        fb_speed, 0.0f, c->dt);
    c->out.motor[i] = LowPassFilter2p_Apply(&c->filter.support_out[i], out[i]);
    c->out.motor[i] = Pole_Clipf(c->out.motor[i], -c->param->limit.max_current,
                                 c->param->limit.max_current);
  }

  return POLE_OK;
}

bool Pole_IsGroupAtTarget(const Pole_t *c, uint8_t group, float threshold_rad) {
  if (c == NULL || c->param == NULL) return false;
  if (group >= 2u) return false;
  if (!c->support_angle.calibrated) return false;

  float threshold = fabsf(threshold_rad);
  uint8_t start = (group == 0u) ? 0u : 2u;
  uint8_t end = start + 2u;
  for (uint8_t i = start; i < end; i++) {
    float fb_angle = Pole_GetSupportAngle(c, i);
    float err = c->setpoint.support_target_angle[i] - fb_angle;
    if (fabsf(err) > threshold) return false;
  }
  return true;
}

bool Pole_IsAllAtTarget(const Pole_t *c, float threshold_rad) {
  return Pole_IsGroupAtTarget(c, 0u, threshold_rad) &&
         Pole_IsGroupAtTarget(c, 1u, threshold_rad);
}

void Pole_Output(Pole_t *c) {
  if (c == NULL || c->param == NULL) return;

  const bool use_legacy_output = Pole_UseLegacyNormalizedOutput(c);

  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    if (use_legacy_output) {
      MOTOR_RM_Param_t *rm_param = const_cast<MOTOR_RM_Param_t *>(&c->param->motor_param[i]);
      const float normalized = Pole_ToLegacyNormalizedOutput(c, c->out.motor[i]);
      (void)MOTOR_RM_SetOutput(rm_param, normalized);
      (void)MOTOR_RM_Ctrl(rm_param);
      continue;
    }

    if (PoleController(c, i) == nullptr) continue;
    PoleController(c, i)->SetTorque(c->out.motor[i]);
    PoleController(c, i)->CommitCommand();
  }
}

void Pole_ResetOutput(Pole_t *c) {
  if (c == NULL || c->param == NULL) return;

  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    if (PoleController(c, i) != nullptr) {
      PoleController(c, i)->Relax();
      PoleController(c, i)->CommitCommand();
    }
    c->out.motor[i] = 0.0f;
  }
}

void Pole_Power_Control(Pole_t *c, float max_power) {
  if (c == NULL || c->param == NULL) return;

  float limit = Pole_Clipf(max_power, 0.0f, c->param->limit.max_current);
  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    c->out.motor[i] = Pole_Clipf(c->out.motor[i], -limit, limit);
  }
}

}

