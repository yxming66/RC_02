#include "module/autoCtrlAPI/core/auto_ctrl_math.h"

#include <math.h>

float AutoCtrlMath_WrapYawDeg(float yaw_deg) {
  while (yaw_deg > 180.0f) {
    yaw_deg -= 360.0f;
  }
  while (yaw_deg < -180.0f) {
    yaw_deg += 360.0f;
  }
  return yaw_deg;
}

float AutoCtrlMath_GetYawErrorDeg(float target_yaw_deg, float current_yaw_deg) {
  return AutoCtrlMath_WrapYawDeg(target_yaw_deg - current_yaw_deg);
}

bool AutoCtrlMath_IsYawAligned(float target_yaw_deg, float current_yaw_deg,
                               float tolerance_deg) {
  return fabsf(AutoCtrlMath_GetYawErrorDeg(target_yaw_deg, current_yaw_deg)) <=
         fabsf(tolerance_deg);
}