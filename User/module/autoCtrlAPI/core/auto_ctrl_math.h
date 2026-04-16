#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

float AutoCtrlMath_WrapYawDeg(float yaw_deg);
float AutoCtrlMath_GetYawErrorDeg(float target_yaw_deg, float current_yaw_deg);
bool AutoCtrlMath_IsYawAligned(float target_yaw_deg, float current_yaw_deg,
                               float tolerance_deg);

#ifdef __cplusplus
}
#endif