#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "module/autoCtrlAPI/api/auto_ctrl_api.h"

void AutoCtrlPrimitive_ResetOutputs(auto_ctrl_t *ctrl);
void AutoCtrlPrimitive_ResetChassis(auto_ctrl_t *ctrl);
void AutoCtrlPrimitive_ResetPole(auto_ctrl_t *ctrl);
float AutoCtrlPrimitive_Clamp(float value, float min_value, float max_value);
void AutoCtrlPrimitive_ApplyPrealign(auto_ctrl_t *ctrl);
void AutoCtrlPrimitive_CommandFlatMove(auto_ctrl_t *ctrl, float vx_mps);
void AutoCtrlPrimitive_CommandPoleTarget(auto_ctrl_t *ctrl, float front_target,
                                         float rear_target);

#ifdef __cplusplus
}
#endif