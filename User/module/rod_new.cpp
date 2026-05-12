/*
 * RodNew module: Servo pitch + pneumatic gripper.
 */

#include "module/rod_new.h"

#include <math.h>
#include <string.h>

#include "bsp/pwm.h"
#include "component/math/scalar.hpp"

namespace {

constexpr float kServoPeriodUs = 20000.0f; /* 50Hz -> 20000us per cycle */

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

}  // namespace

extern "C" {

int8_t RodNew_Init(RodNew_t *r, const RodNew_Params_t *param) {
  if (r == nullptr || param == nullptr) {
    return ROD_NEW_ERR_NULL;
  }

  memset(r, 0, sizeof(RodNew_t));
  r->param = param;
  r->mode = ROD_NEW_MODE_RELAX;
  r->servo.target_angle_rad = param->servo.angle_standby_rad;
  r->servo.tracked_angle_rad = param->servo.angle_standby_rad;
  r->servo.feedback_angle_rad = 0.0f;
  r->servo.at_target = true;
  r->gripper.state = ROD_NEW_GRIP_RELEASE;
  r->gripper.grip_done = false;
  r->gripper.grip_start_tick = 0U;
  r->gripper.timed_out = false;

  return ROD_NEW_OK;
}

int8_t RodNew_Control(RodNew_t *r, RodNew_Mode_t mode, RodNew_Pose_t pose,
                      RodNew_GripState_t grip, uint32_t now) {
  if (r == nullptr || r->param == nullptr) {
    return ROD_NEW_ERR_NULL;
  }

  r->now_tick = now;
  r->dt = (float)(now - r->last_wakeup) / 1000.0f;
  r->last_wakeup = now;
  r->dt = mr::component::math::sanitize_dt(r->dt, 0.001f, 0.0005f, 0.050f);

  r->mode = mode;

  if (r->mode == ROD_NEW_MODE_RELAX) {
    r->servo.target_angle_rad = r->param->servo.angle_standby_rad;
    r->servo.tracked_angle_rad = r->param->servo.angle_standby_rad;
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
    case ROD_NEW_POSE_READY:
      r->servo.target_angle_rad = r->param->servo.angle_ready_rad;
      break;
    case ROD_NEW_POSE_GRAB_LOW:
      r->servo.target_angle_rad = r->param->servo.angle_grab_low_rad;
      break;
    case ROD_NEW_POSE_GRAB_HIGH:
      r->servo.target_angle_rad = r->param->servo.angle_grab_high_rad;
      break;
    case ROD_NEW_POSE_LIFT:
      r->servo.target_angle_rad = r->param->servo.angle_lift_rad;
      break;
    default:
      r->servo.target_angle_rad = r->param->servo.angle_standby_rad;
      break;
  }

  /* 夹爪状态机 */
  if (grip != r->gripper.state) {
    r->gripper.state = grip;
    if (grip == ROD_NEW_GRIP_GRAB) {
      r->gripper.grip_start_tick = now;
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
      r->gripper.timed_out = true;
      r->gripper.grip_done = true;
    }
  }

  return ROD_NEW_OK;
}

void RodNew_Output(RodNew_t *r) {
  if (r == nullptr || r->param == nullptr) {
    return;
  }

  /* 速度限幅跟踪目标 */
  const float max_delta = r->param->servo.max_vel_rad_s * r->dt;
  r->servo.tracked_angle_rad =
      MoveTowards(r->servo.tracked_angle_rad, r->servo.target_angle_rad, max_delta);
  r->servo.tracked_angle_rad =
      Clamp(r->servo.tracked_angle_rad, r->param->servo.angle_min_rad,
            r->param->servo.angle_max_rad);

  /* 夹爪IO输出 */
  BSP_GPIO_WritePin(r->param->gripper.gripper_gpio,
                   (r->gripper.state == ROD_NEW_GRIP_GRAB));
}

float RodNew_AngleToPulseUs(float angle_rad,
                              const RodNew_ServoParams_t *param) {
  if (param == nullptr) {
    return ROD_NEW_SERVO_PULSE_NEUTRAL_US;
  }

  /* 映射角度到脉宽 */
  float clamped = Clamp(angle_rad, param->angle_min_rad, param->angle_max_rad);

  /* 线性映射：angle_min -> 500us, angle_max -> 2500us */
  float range = param->angle_max_rad - param->angle_min_rad;
  if (fabsf(range) < 1e-6f) {
    return ROD_NEW_SERVO_PULSE_NEUTRAL_US;
  }

  float t = (clamped - param->angle_min_rad) / range;
  float pulse_us = (float)ROD_NEW_SERVO_PULSE_MIN_US +
                   t * ((float)ROD_NEW_SERVO_PULSE_MAX_US -
                        (float)ROD_NEW_SERVO_PULSE_MIN_US);

  return pulse_us;
}

bool RodNew_IsAtTarget(const RodNew_t *r) {
  if (r == nullptr || r->param == nullptr) {
    return false;
  }
  float err = fabsf(r->servo.tracked_angle_rad - r->servo.feedback_angle_rad);
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
  r->gripper.state = ROD_NEW_GRIP_RELEASE;
  r->gripper.grip_done = false;
  r->gripper.timed_out = false;
  r->last_wakeup = 0U;
}

}  // extern "C"
