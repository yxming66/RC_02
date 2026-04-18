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
constexpr float kPoleOffGroundLoadLpfDefaultHz = 10.0f;
constexpr float kPoleOffGroundSpeedGateDefaultRadS = 3.0f;
constexpr uint8_t kPoleOffGroundDebounceDefaultCycles = 5u;

constexpr float kPoleGravityDescendingScaleDefault = 0.25f;
constexpr float kPoleGravityErrDeadbandDefaultRad = 0.03f;
constexpr float kPoleGravityRiseRateDefault = 6.0f;
constexpr float kPoleGravityFallRateDefault = 10.0f;

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

static float Pole_GetLowerHoldReleaseZone(const Pole_t *c) {
  if (c == NULL || c->param == NULL) return 0.0f;

  const float hold_zone = fmaxf(c->param->limit.support_hold_zone, 0.0f);
  const float configured_release = fmaxf(c->param->limit.support_hold_release_zone, 0.0f);
  if (configured_release > hold_zone) return configured_release;

  return fmaxf(hold_zone * 1.8f, hold_zone + 0.08f);
}

static bool Pole_UpdateLowerHoldLatch(Pole_t *c, uint8_t idx, bool request_lower, float fb_angle, float fb_speed) {
  if (c == NULL || c->param == NULL || idx >= POLE_SUPPORT_MOTOR_NUM) return false;

  const float hold_zone = fmaxf(c->param->limit.support_hold_zone, 0.0f);
  if (hold_zone <= 0.0f) {
    c->support_angle.lower_hold_latched[idx] = false;
    return false;
  }

  const float speed_threshold = fmaxf(c->param->limit.support_hold_speed_threshold, 0.0f);
  const float release_zone = Pole_GetLowerHoldReleaseZone(c);
  const float dist_to_lower = fb_angle - c->support_angle.lower[idx];

  bool latched = c->support_angle.lower_hold_latched[idx];
  if (!request_lower) {
    latched = false;
  } else if (!latched) {
    if (dist_to_lower <= hold_zone && fabsf(fb_speed) <= speed_threshold) {
      latched = true;
    }
  } else if (dist_to_lower > release_zone) {
    latched = false;
  }

  c->support_angle.lower_hold_latched[idx] = latched;
  return latched;
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

static float Pole_GetMaxOutputTorqueNm(const Pole_t *c) {
  if (c == NULL || c->param == NULL) return 0.0f;
  if (c->param->limit.max_torque_nm > 0.0f) return c->param->limit.max_torque_nm;
  return fmaxf(c->param->limit.max_current, 0.0f);
}

static uint8_t Pole_GetOffGroundDebounceCycles(const Pole_t *c) {
  if (c == NULL || c->param == NULL) return kPoleOffGroundDebounceDefaultCycles;

  const uint8_t configured = c->param->off_ground.debounce_cycles;
  return (configured > 0u) ? configured : kPoleOffGroundDebounceDefaultCycles;
}

static float Pole_GetOffGroundLoadLpfHz(const Pole_t *c) {
  if (c == NULL || c->param == NULL) return kPoleOffGroundLoadLpfDefaultHz;

  const float configured = c->param->off_ground.load_lpf_hz;
  return (configured > 0.0f) ? configured : kPoleOffGroundLoadLpfDefaultHz;
}

static float Pole_GetOffGroundLoadEnterNm(const Pole_t *c) {
  if (c == NULL || c->param == NULL) return 0.8f;

  if (c->param->off_ground.load_enter_nm > 0.0f) {
    return c->param->off_ground.load_enter_nm;
  }

  return fmaxf(Pole_GetMaxOutputTorqueNm(c) * 0.22f, 0.8f);
}

static float Pole_GetOffGroundLoadExitNm(const Pole_t *c) {
  if (c == NULL || c->param == NULL) return 0.5f;

  const float enter_nm = Pole_GetOffGroundLoadEnterNm(c);
  if (c->param->off_ground.load_exit_nm > 0.0f) {
    return Pole_Clipf(c->param->off_ground.load_exit_nm, 0.0f, enter_nm * 0.95f);
  }

  return enter_nm * 0.65f;
}

static float Pole_GetOffGroundSpeedGateRadS(const Pole_t *c) {
  if (c == NULL || c->param == NULL) return kPoleOffGroundSpeedGateDefaultRadS;

  const float configured = c->param->off_ground.speed_gate_rad_s;
  return (configured > 0.0f) ? configured : kPoleOffGroundSpeedGateDefaultRadS;
}

static bool Pole_UseGravityComp(const Pole_t *c) {
  return (c != NULL && c->param != NULL && c->param->gravity_comp.enable);
}

static float Pole_GetGravityDescendingScale(const Pole_t *c) {
  if (c == NULL || c->param == NULL) return kPoleGravityDescendingScaleDefault;

  const float configured = c->param->gravity_comp.descending_scale;
  if (configured >= 0.0f && configured <= 1.0f) return configured;
  return kPoleGravityDescendingScaleDefault;
}

static float Pole_GetGravityErrDeadbandRad(const Pole_t *c) {
  if (c == NULL || c->param == NULL) return kPoleGravityErrDeadbandDefaultRad;

  const float configured = c->param->gravity_comp.err_deadband_rad;
  return (configured > 0.0f) ? configured : kPoleGravityErrDeadbandDefaultRad;
}

static float Pole_GetGravityRiseRate(const Pole_t *c) {
  if (c == NULL || c->param == NULL) return kPoleGravityRiseRateDefault;

  const float configured = c->param->gravity_comp.rise_rate;
  return (configured > 0.0f) ? configured : kPoleGravityRiseRateDefault;
}

static float Pole_GetGravityFallRate(const Pole_t *c) {
  if (c == NULL || c->param == NULL) return kPoleGravityFallRateDefault;

  const float configured = c->param->gravity_comp.fall_rate;
  return (configured > 0.0f) ? configured : kPoleGravityFallRateDefault;
}

static void Pole_ResetOffGroundState(Pole_t *c) {
  if (c == NULL) return;

  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    c->off_ground.motor_load_nm[i] = fabsf(c->feedback.motor[i].torque_current);
  }

  for (uint8_t side = 0; side < 2u; side++) {
    const uint8_t i0 = (side == 0u) ? 0u : 2u;
    const uint8_t i1 = i0 + 1u;
    c->off_ground.side_load_nm[side] =
        0.5f * (c->off_ground.motor_load_nm[i0] + c->off_ground.motor_load_nm[i1]);
    c->off_ground.gravity_scale[side] = 0.0f;
    c->off_ground.side_off_ground[side] = false;
    c->off_ground.side_enter_count[side] = 0u;
    c->off_ground.side_exit_count[side] = 0u;
  }
}

static void Pole_SyncOffGroundDebug(Pole_t *c) {
  if (c == NULL) return;

  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    c->debug.motor_load_nm[i] = c->off_ground.motor_load_nm[i];
  }

  for (uint8_t side = 0; side < 2u; side++) {
    c->debug.side_load_nm[side] = c->off_ground.side_load_nm[side];
    c->debug.side_off_ground[side] = c->off_ground.side_off_ground[side] ? 1u : 0u;
    c->debug.gravity_scale[side] = c->off_ground.gravity_scale[side];
  }
}

static void Pole_UpdateOffGroundState(Pole_t *c, const float fb_speed[POLE_SUPPORT_MOTOR_NUM]) {
  if (c == NULL || c->param == NULL) return;

  const float dt = fmaxf(c->dt, 0.0005f);
  const float load_lpf_hz = Pole_GetOffGroundLoadLpfHz(c);
  const float alpha = Pole_Clipf(6.2831853f * load_lpf_hz * dt, 0.0f, 1.0f);
  const float speed_gate = Pole_GetOffGroundSpeedGateRadS(c);
  const float load_enter_nm = Pole_GetOffGroundLoadEnterNm(c);
  const float load_exit_nm = Pole_GetOffGroundLoadExitNm(c);
  const uint8_t debounce_cycles = Pole_GetOffGroundDebounceCycles(c);

  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    const float speed_abs = fabsf(fb_speed[i]);
    const float measured_load_nm = fabsf(c->feedback.motor[i].torque_current);
    float sampled_load_nm = measured_load_nm;

    if (speed_gate > 0.0f && speed_abs > speed_gate) {
      sampled_load_nm = c->off_ground.motor_load_nm[i];
    }

    c->off_ground.motor_load_nm[i] += alpha * (sampled_load_nm - c->off_ground.motor_load_nm[i]);
  }

  for (uint8_t side = 0; side < 2u; side++) {
    const uint8_t i0 = (side == 0u) ? 0u : 2u;
    const uint8_t i1 = i0 + 1u;

    c->off_ground.side_load_nm[side] =
        0.5f * (c->off_ground.motor_load_nm[i0] + c->off_ground.motor_load_nm[i1]);

    const bool exceed_enter = (c->off_ground.side_load_nm[side] >= load_enter_nm);
    const bool below_exit = (c->off_ground.side_load_nm[side] <= load_exit_nm);

    if (!c->off_ground.side_off_ground[side]) {
      if (below_exit) {
        if (c->off_ground.side_enter_count[side] < debounce_cycles) {
          c->off_ground.side_enter_count[side]++;
        }
      } else {
        c->off_ground.side_enter_count[side] = 0u;
      }
      c->off_ground.side_exit_count[side] = 0u;

      if (c->off_ground.side_enter_count[side] >= debounce_cycles) {
        c->off_ground.side_off_ground[side] = true;
        c->off_ground.side_enter_count[side] = 0u;
      }
    } else {
      if (exceed_enter) {
        if (c->off_ground.side_exit_count[side] < debounce_cycles) {
          c->off_ground.side_exit_count[side]++;
        }
      } else {
        c->off_ground.side_exit_count[side] = 0u;
      }
      c->off_ground.side_enter_count[side] = 0u;

      if (c->off_ground.side_exit_count[side] >= debounce_cycles) {
        c->off_ground.side_off_ground[side] = false;
        c->off_ground.side_exit_count[side] = 0u;
      }
    }

    float target_scale = c->off_ground.side_off_ground[side] ? 0.0f : 1.0f;
    if (!Pole_UseGravityComp(c)) {
      target_scale = 0.0f;
    }

    const float rate = target_scale > c->off_ground.gravity_scale[side]
                           ? Pole_GetGravityRiseRate(c)
                           : Pole_GetGravityFallRate(c);
    c->off_ground.gravity_scale[side] = Pole_MoveTowards(
        c->off_ground.gravity_scale[side], target_scale, fmaxf(rate, 0.0f) * dt);
    c->off_ground.gravity_scale[side] = Pole_Clipf(c->off_ground.gravity_scale[side], 0.0f, 1.0f);
  }

  Pole_SyncOffGroundDebug(c);
}

static float Pole_GetGravityFeedforwardNm(const Pole_t *c, uint8_t idx,
                                          float target_angle, float fb_angle) {
  if (c == NULL || c->param == NULL) return 0.0f;
  if (idx >= POLE_SUPPORT_MOTOR_NUM) return 0.0f;
  if (!Pole_UseGravityComp(c)) return 0.0f;

  const uint8_t side = (idx < 2u) ? 0u : 1u;
  const float side_scale = Pole_Clipf(c->off_ground.gravity_scale[side], 0.0f, 1.0f);
  if (side_scale <= 0.0f) return 0.0f;

  const float ff_nominal = c->param->gravity_comp.torque_ff_nm[idx];
  float motion_scale = 1.0f;
  const float err = target_angle - fb_angle;
  const float deadband = Pole_GetGravityErrDeadbandRad(c);
  if (err < -deadband) {
    motion_scale = Pole_GetGravityDescendingScale(c);
  }

  return ff_nominal * side_scale * motion_scale;
}

static float Pole_GetSupportSpeedFeedbackFiltered(Pole_t *c, uint8_t idx) {
  if (c == NULL || idx >= POLE_SUPPORT_MOTOR_NUM) return 0.0f;

  float raw_speed = c->feedback.motor[idx].rotor_speed;
  const float spike_limit = fmaxf(c->param->filter.support_vel_feedback_spike_rad_s, 0.0f);
  if (spike_limit > 0.0f) {
    const float filtered_prev = c->feedback.support_vel_filtered[idx];
    raw_speed = Pole_Clipf(raw_speed, filtered_prev - spike_limit, filtered_prev + spike_limit);
  }

  c->feedback.support_vel_filtered[idx] = LowPassFilter2p_Apply(
      &c->filter.support_vel_in[idx], raw_speed);
  return c->feedback.support_vel_filtered[idx];
}

static bool Pole_UseLegacyNormalizedOutput(const Pole_t *c) {
  return (c != NULL && c->param != NULL && c->param->output.use_legacy_normalized_output);
}

static float Pole_ToLegacyNormalizedOutput(const Pole_t *c, float output_value) {
  if (c == NULL || c->param == NULL) return 0.0f;

  const float full_scale = fmaxf(Pole_GetMaxOutputTorqueNm(c), 1e-6f);
  return Pole_Clipf(output_value / full_scale, -1.0f, 1.0f);
}

static void Pole_ResetControllers(Pole_t *c) {
  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    PID_Reset(&c->pid.support_pos[i]);
    PID_Reset(&c->pid.support_vel[i]);
    LowPassFilter2p_Reset(&c->filter.support_vel_in[i], c->feedback.motor[i].rotor_speed);
    LowPassFilter2p_Reset(&c->filter.support_out[i], 0.0f);
    c->feedback.support_vel_filtered[i] = c->feedback.motor[i].rotor_speed;
    c->support_angle.lower_hold_latched[i] = false;
    c->debug.gravity_ff_nm[i] = 0.0f;
    c->debug.torque_pid_nm[i] = 0.0f;
    c->debug.torque_cmd_nm[i] = 0.0f;
    c->debug.target_angle_rad[i] = 0.0f;
    c->debug.feedback_angle_rad[i] = 0.0f;
    c->debug.feedback_speed_rad_s[i] = 0.0f;
  }

  Pole_ResetOffGroundState(c);
  Pole_SyncOffGroundDebug(c);
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
      .velocity_to_torque_limit = Pole_GetMaxOutputTorqueNm(c),
    };

    PoleController(c, i) = new (g_pole_controller_storage[i]) PoleMotorController(*PoleMotorHandle(c, i), controller_config);
    if (PoleController(c, i)->Register() != DEVICE_OK) {
      return POLE_ERR;
    }
    if (PoleController(c, i)->Enable() != DEVICE_OK) {
      return POLE_ERR;
    }
  }

  Pole_ResetOffGroundState(c);
  Pole_SyncOffGroundDebug(c);

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
    Pole_ResetOffGroundState(c);
    Pole_SyncOffGroundDebug(c);
    for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
      c->out.motor[i] = 0.0f;
      c->debug.gravity_ff_nm[i] = 0.0f;
      c->debug.torque_pid_nm[i] = 0.0f;
      c->debug.torque_cmd_nm[i] = 0.0f;
      c->debug.target_angle_rad[i] = 0.0f;
      c->debug.feedback_angle_rad[i] = Pole_GetSupportAngle(c, i);
      c->debug.feedback_speed_rad_s[i] = c->feedback.motor[i].rotor_speed;
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
    c->debug.target_angle_rad[i] = c->setpoint.support_target_angle[i];
  }

  float fb_speed_buf[POLE_SUPPORT_MOTOR_NUM] = {};
  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    fb_speed_buf[i] = Pole_GetSupportSpeedFeedbackFiltered(c, i);
  }
  Pole_UpdateOffGroundState(c, fb_speed_buf);

  float vel[4];
  float out[4];
  const float max_torque_nm = Pole_GetMaxOutputTorqueNm(c);
  for (uint8_t i = 0; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    const float out_prev = c->out.motor[i];
    float fb_angle = Pole_GetSupportAngle(c, i);
    float fb_speed = fb_speed_buf[i];
    c->debug.feedback_angle_rad[i] = fb_angle;
    c->debug.feedback_speed_rad_s[i] = fb_speed;

    const bool request_lower =
        (c->setpoint.support_target_angle[i] <= (c->support_angle.lower[i] + c->param->limit.support_hold_zone));
    const bool was_latched = c->support_angle.lower_hold_latched[i];
    const bool hold_latched = Pole_UpdateLowerHoldLatch(c, i, request_lower, fb_angle, fb_speed);
    if (hold_latched) {
      c->setpoint.support_target_angle[i] = fb_angle;
      if (!was_latched) {
        PID_Reset(&c->pid.support_vel[i]);
      }
      PID_Reset(&c->pid.support_pos[i]);
      LowPassFilter2p_Reset(&c->filter.support_out[i], 0.0f);
      vel[i] = 0.0f;
      out[i] = 0.0f;
      c->out.motor[i] = 0.0f;
      c->debug.gravity_ff_nm[i] = 0.0f;
      c->debug.torque_pid_nm[i] = 0.0f;
      c->debug.torque_cmd_nm[i] = 0.0f;
      continue;
    }

    vel[i] = PID_Calc(&c->pid.support_pos[i], c->setpoint.support_target_angle[i],
                      fb_angle, 0.0f, c->dt);
    vel[i] = Pole_GetSoftLimitedSpeed(c, i, vel[i], fb_angle);
    out[i] = PID_Calc(&c->pid.support_vel[i], vel[i],
                      fb_speed, 0.0f, c->dt);

    float out_cmd_pid = LowPassFilter2p_Apply(&c->filter.support_out[i], out[i]);
    const float gravity_ff_nm = Pole_GetGravityFeedforwardNm(c, i, c->setpoint.support_target_angle[i], fb_angle);
    float out_cmd = out_cmd_pid + gravity_ff_nm;
    out_cmd = Pole_Clipf(out_cmd, -max_torque_nm, max_torque_nm);

    const float slew_rate = fmaxf(c->param->limit.support_output_slew_rate, 0.0f);
    if (slew_rate > 0.0f) {
      out_cmd = Pole_MoveTowards(out_prev, out_cmd, slew_rate * c->dt);
    }
    c->out.motor[i] = out_cmd;
    c->debug.gravity_ff_nm[i] = gravity_ff_nm;
    c->debug.torque_pid_nm[i] = out_cmd_pid;
    c->debug.torque_cmd_nm[i] = out_cmd;
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

bool Pole_IsSideOffGround(const Pole_t *c, uint8_t side) {
  if (c == NULL || c->param == NULL) return false;
  if (side >= 2u) return false;
  return c->off_ground.side_off_ground[side];
}

bool Pole_IsAnyOffGround(const Pole_t *c) {
  return Pole_IsSideOffGround(c, 0u) || Pole_IsSideOffGround(c, 1u);
}

const Pole_Debug_t *Pole_GetDebug(const Pole_t *c) {
  if (c == NULL) return NULL;
  return &c->debug;
}

void Pole_Output(Pole_t *c) {
  if (c == NULL || c->param == NULL) return;

  const bool use_legacy_output = Pole_UseLegacyNormalizedOutput(c);

  if (use_legacy_output) {
    bool ctrl_sent[BSP_CAN_NUM][2] = {};
    for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
      MOTOR_RM_Param_t *rm_param = const_cast<MOTOR_RM_Param_t *>(&c->param->motor_param[i]);
      const float normalized = Pole_ToLegacyNormalizedOutput(c, c->out.motor[i]);
      (void)MOTOR_RM_SetOutput(rm_param, normalized);
    }

    for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
      MOTOR_RM_Param_t *rm_param = const_cast<MOTOR_RM_Param_t *>(&c->param->motor_param[i]);
      const uint8_t can = (uint8_t)rm_param->can;
      const uint8_t group = (rm_param->id >= 0x205u && rm_param->id <= 0x208u) ? 1u : 0u;
      if (can >= BSP_CAN_NUM) continue;
      if (ctrl_sent[can][group]) continue;
      (void)MOTOR_RM_Ctrl(rm_param);
      ctrl_sent[can][group] = true;
    }
    return;
  }

  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
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
    c->debug.gravity_ff_nm[i] = 0.0f;
    c->debug.torque_pid_nm[i] = 0.0f;
    c->debug.torque_cmd_nm[i] = 0.0f;
  }
}

void Pole_Power_Control(Pole_t *c, float max_power) {
  if (c == NULL || c->param == NULL) return;

  float limit = Pole_Clipf(max_power, 0.0f, Pole_GetMaxOutputTorqueNm(c));
  for (uint8_t i = 0; i < POLE_MOTOR_NUM; i++) {
    c->out.motor[i] = Pole_Clipf(c->out.motor[i], -limit, limit);
    c->debug.torque_cmd_nm[i] = c->out.motor[i];
  }
}

}

