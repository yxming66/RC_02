/*
 * RodNew module: Servo pitch + pneumatic gripper.
 */

#include "module/rod_new.h"

#include <math.h>
#include <string.h>

#include "bsp/pwm.h"
#include "component/math/scalar.hpp"
#include "module/shared_valve.h"

namespace {

float Clamp(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

float MoveTowards(float current, float target, float max_delta) {
  float delta = target - current;
  if (delta > max_delta) return current + max_delta;
  if (delta < -max_delta) return current - max_delta;
  return target;
}

bool RodNew_ParamsValid(const RodNew_Params_t *param) {
  return param != nullptr && param->servo.angle_max_rad > param->servo.angle_min_rad &&
         isfinite(param->servo.angle_min_rad) && isfinite(param->servo.angle_max_rad) &&
         isfinite(param->servo.angle_standby_rad) && isfinite(param->servo.max_vel_rad_s) &&
         param->servo.pwm_channel < BSP_PWM_NUM;
}

}  // namespace

extern "C" {

int8_t RodNew_Init(RodNew_t *r, const RodNew_Params_t *param) {
  if (r == nullptr || param == nullptr) {
    return ROD_NEW_ERR_NULL;
  }
  if (!RodNew_ParamsValid(param)) {
    return ROD_NEW_ERR;
  }

  memset(r, 0, sizeof(RodNew_t));
  r->param = param;
  r->mode = ROD_NEW_MODE_RELAX;
  r->servo.target_angle_rad = param->servo.angle_standby_rad;
  r->servo.tracked_angle_rad = param->servo.angle_standby_rad;
  r->servo.tracked_vel_rad_s = 0.0f;
  r->servo.feedback_angle_rad = param->servo.angle_standby_rad;
  r->servo.at_target = true;
  r->gripper.state = ROD_NEW_GRIP_RELEASE;
  r->gripper.grip_done = false;
  r->gripper.grip_start_tick = 0U;
  r->gripper.timed_out = false;

  const float freq_hz = (param->servo.freq_hz > 0.0f && isfinite(param->servo.freq_hz))
                            ? param->servo.freq_hz
                            : (float)ROD_NEW_SERVO_DEFAULT_FREQ_HZ;
  if (BSP_PWM_SetFreq(param->servo.pwm_channel, freq_hz) != BSP_OK ||
      BSP_PWM_Start(param->servo.pwm_channel) != BSP_OK) {
    return ROD_NEW_ERR;
  }

  BSP_PWM_SetPulseUs(param->servo.pwm_channel,
                     (uint32_t)RodNew_AngleToPulseUs(r->servo.tracked_angle_rad,
                                                      &param->servo));
  SharedValve_SetRodRequest(false);

  return ROD_NEW_OK;
}

int8_t RodNew_Control(RodNew_t *r, RodNew_Mode_t mode, RodNew_Pose_t pose,
                      RodNew_GripState_t grip, float target_angle_rad,
                      uint32_t now) {
  if (r == nullptr || r->param == nullptr) {
    return ROD_NEW_ERR_NULL;
  }

  r->now_tick = now;
  r->dt = (r->last_wakeup == 0U) ? 0.001f : (float)(now - r->last_wakeup) / 1000.0f;
  r->last_wakeup = now;
  r->dt = mr::component::math::sanitize_dt(r->dt, 0.001f, 0.0005f, 0.050f);

  r->mode = mode;

  if (r->mode == ROD_NEW_MODE_RELAX) {
    r->servo.target_angle_rad = r->param->servo.angle_standby_rad;
    r->servo.tracked_angle_rad = r->param->servo.angle_standby_rad;
    r->servo.tracked_vel_rad_s = 0.0f;
    r->servo.feedback_angle_rad = r->servo.tracked_angle_rad;
    r->servo.at_target = true;
    r->gripper.state = ROD_NEW_GRIP_RELEASE;
    r->gripper.grip_done = false;
    r->gripper.timed_out = false;
    return ROD_NEW_OK;
  }

  /* 目标角度设置 */
  switch (pose) {
    case ROD_NEW_POSE_STANDBY:
      r->servo.target_angle_rad = r->param->servo.angle_standby_rad;
      break;
    case ROD_NEW_POSE_GRAB_HIGH:
      r->servo.target_angle_rad = r->param->servo.angle_grab_high_rad;
      break;
    case ROD_NEW_POSE_DOCK_WAIT:
      r->servo.target_angle_rad = r->param->servo.angle_dock_wait_rad;
      break;
    case ROD_NEW_POSE_MANUAL:
      r->servo.target_angle_rad = target_angle_rad;
      break;
    default:
      r->servo.target_angle_rad = r->param->servo.angle_standby_rad;
      break;
  }
  r->servo.target_angle_rad = Clamp(r->servo.target_angle_rad,
                                    r->param->servo.angle_min_rad,
                                    r->param->servo.angle_max_rad);

  /* 夹爪状态机 */
  if (grip != r->gripper.state) {
    r->gripper.state = grip;
    if (grip == ROD_NEW_GRIP_GRAB) {
      r->gripper.grip_start_tick = now;
      r->gripper.grip_done = false;
      r->gripper.timed_out = false;
    } else {
      r->gripper.grip_done = false;
      r->gripper.timed_out = false;
    }
  }

  /* 夹取超时检测 */
  if (r->gripper.state == ROD_NEW_GRIP_GRAB && !r->gripper.grip_done) {
    uint32_t elapsed_ms = (now >= r->gripper.grip_start_tick)
                              ? (now - r->gripper.grip_start_tick)
                              : 0U;
    if (elapsed_ms >= r->param->gripper.grip_timeout_ms) {
      r->gripper.grip_done = true;
    }
  }

  return ROD_NEW_OK;
}

void RodNew_Output(RodNew_t *r) {
  if (r == nullptr || r->param == nullptr) {
    return;
  }

  if (g_rod_new_debug.enable && g_rod_new_debug.direct_pulse_enable) {
    uint32_t pulse_us = g_rod_new_debug.pulse_us;
    BSP_PWM_Channel_t pwm_channel = g_rod_new_debug.pwm_channel;
    if (pwm_channel >= BSP_PWM_NUM) {
      pwm_channel = r->param->servo.pwm_channel;
    }
    if (pulse_us < ROD_NEW_SERVO_PULSE_MIN_US) {
      pulse_us = ROD_NEW_SERVO_PULSE_MIN_US;
    }
    if (pulse_us > ROD_NEW_SERVO_PULSE_MAX_US) {
      pulse_us = ROD_NEW_SERVO_PULSE_MAX_US;
    }
    r->servo.target_angle_rad = r->servo.tracked_angle_rad;
    r->servo.tracked_vel_rad_s = 0.0f;
    r->servo.at_target = true;
    BSP_PWM_SetPulseUs(pwm_channel, pulse_us);
    SharedValve_SetRodRequest(g_rod_new_debug.grip == ROD_NEW_GRIP_GRAB);
    return;
  }

  if (g_rod_new_debug.enable) {
    r->servo.target_angle_rad = Clamp(g_rod_new_debug.target_angle_rad,
                                      r->param->servo.angle_min_rad,
                                      r->param->servo.angle_max_rad);
    r->gripper.state = g_rod_new_debug.grip;
  }

  /* 速度限幅跟踪目标 */
  const float max_vel = fmaxf(r->param->servo.max_vel_rad_s, 0.01f);
  const float max_acc = fmaxf(r->param->servo.max_acc_rad_s, 0.0f);
  const float error = r->servo.target_angle_rad - r->servo.tracked_angle_rad;
  const float desired_vel = Clamp(error / r->dt, -max_vel, max_vel);

  if (max_acc > 0.0f) {
    r->servo.tracked_vel_rad_s =
        MoveTowards(r->servo.tracked_vel_rad_s, desired_vel, max_acc * r->dt);
  } else {
    r->servo.tracked_vel_rad_s = desired_vel;
  }

  const float step = r->servo.tracked_vel_rad_s * r->dt;
  if (fabsf(step) >= fabsf(error)) {
    r->servo.tracked_angle_rad = r->servo.target_angle_rad;
    r->servo.tracked_vel_rad_s = 0.0f;
  } else {
    r->servo.tracked_angle_rad += step;
  }
  r->servo.tracked_angle_rad =
      Clamp(r->servo.tracked_angle_rad, r->param->servo.angle_min_rad,
            r->param->servo.angle_max_rad);
  r->servo.feedback_angle_rad = r->servo.tracked_angle_rad;
  r->servo.at_target =
      fabsf(r->servo.target_angle_rad - r->servo.tracked_angle_rad) <=
      r->param->servo.arrive_threshold_rad;

  const uint32_t pulse_us =
      (uint32_t)RodNew_AngleToPulseUs(r->servo.tracked_angle_rad, &r->param->servo);
  BSP_PWM_SetPulseUs(r->param->servo.pwm_channel, pulse_us);

  /* 夹爪IO输出 */
  SharedValve_SetRodRequest(r->gripper.state == ROD_NEW_GRIP_GRAB);
}

float RodNew_AngleToPulseUs(float angle_rad,
                              const RodNew_ServoParams_t *param) {
  if (param == nullptr) {
    return ROD_NEW_SERVO_PULSE_NEUTRAL_US;
  }

  float clamped = Clamp(-angle_rad, param->angle_min_rad, param->angle_max_rad);

  uint32_t zero_pulse_us = param->zero_pulse_us;
  if (zero_pulse_us < ROD_NEW_SERVO_PULSE_MIN_US ||
      zero_pulse_us > ROD_NEW_SERVO_PULSE_MAX_US) {
    zero_pulse_us = ROD_NEW_SERVO_PULSE_NEUTRAL_US;
  }

  float pulse_us = (float)zero_pulse_us;
  if (clamped >= 0.0f) {
    const float pos_range = param->angle_max_rad;
    if (fabsf(pos_range) > 1e-6f) {
      pulse_us += (clamped / pos_range) *
                  ((float)ROD_NEW_SERVO_PULSE_MAX_US - (float)zero_pulse_us);
    }
  } else {
    const float neg_range = -param->angle_min_rad;
    if (fabsf(neg_range) > 1e-6f) {
      pulse_us -= ((-clamped) / neg_range) *
                  ((float)zero_pulse_us - (float)ROD_NEW_SERVO_PULSE_MIN_US);
    }
  }

  return pulse_us;
}

bool RodNew_IsAtTarget(const RodNew_t *r) {
  if (r == nullptr || r->param == nullptr) {
    return false;
  }
  float err = fabsf(r->servo.target_angle_rad - r->servo.tracked_angle_rad);
  return err <= r->param->servo.arrive_threshold_rad;
}

bool RodNew_IsGripDone(const RodNew_t *r) {
  if (r == nullptr) {
    return false;
  }
  return r->gripper.grip_done || r->gripper.timed_out;
}

void RodNew_Reset(RodNew_t *r) {
  if (r == nullptr) {
    return;
  }
  r->mode = ROD_NEW_MODE_RELAX;
  r->servo.target_angle_rad = r->param != nullptr
                                  ? r->param->servo.angle_standby_rad
                                  : 0.0f;
  r->servo.tracked_angle_rad = r->servo.target_angle_rad;
  r->servo.tracked_vel_rad_s = 0.0f;
  r->servo.feedback_angle_rad = r->servo.tracked_angle_rad;
  r->servo.at_target = true;
  r->gripper.state = ROD_NEW_GRIP_RELEASE;
  r->gripper.grip_done = false;
  r->gripper.timed_out = false;
  r->last_wakeup = 0U;
}

}  // extern "C"
