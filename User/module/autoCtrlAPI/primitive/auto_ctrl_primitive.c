#include "module/autoCtrlAPI/primitive/auto_ctrl_primitive.h"

#include <string.h>

#define AUTO_CTRL_PREALIGN_KP (0.03f)
#define AUTO_CTRL_PREALIGN_WZ_LIMIT (1.5f)

void AutoCtrlPrimitive_ResetChassis(auto_ctrl_t *ctrl) {
  memset(&ctrl->chassis_cmd, 0, sizeof(ctrl->chassis_cmd));
  ctrl->chassis_cmd.mode = CHASSIS_MODE_RELAX;
}

void AutoCtrlPrimitive_ResetPole(auto_ctrl_t *ctrl) {
  memset(&ctrl->pole_cmd, 0, sizeof(ctrl->pole_cmd));
  ctrl->pole_cmd.mode = POLE_MODE_RELAX;
}

void AutoCtrlPrimitive_ResetOutputs(auto_ctrl_t *ctrl) {
  AutoCtrlPrimitive_ResetChassis(ctrl);
  AutoCtrlPrimitive_ResetPole(ctrl);
}

float AutoCtrlPrimitive_Clamp(float value, float min_value, float max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

void AutoCtrlPrimitive_ApplyPrealign(auto_ctrl_t *ctrl) {
  ctrl->chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
  ctrl->chassis_cmd.ctrl_vec.vx = 0.0f;
  ctrl->chassis_cmd.ctrl_vec.vy = 0.0f;
  ctrl->chassis_cmd.ctrl_vec.wz = AutoCtrlPrimitive_Clamp(
      ctrl->yaw_error_deg * AUTO_CTRL_PREALIGN_KP,
      -AUTO_CTRL_PREALIGN_WZ_LIMIT, AUTO_CTRL_PREALIGN_WZ_LIMIT);
}

void AutoCtrlPrimitive_CommandFlatMove(auto_ctrl_t *ctrl, float vx_mps) {
  ctrl->chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
  ctrl->chassis_cmd.ctrl_vec.vx = vx_mps;
  ctrl->chassis_cmd.ctrl_vec.vy = 0.0f;
  ctrl->chassis_cmd.ctrl_vec.wz = 0.0f;
}

void AutoCtrlPrimitive_CommandPoleTarget(auto_ctrl_t *ctrl, float front_target,
                                         float rear_target) {
  ctrl->pole_cmd.mode = POLE_MODE_ACTIVE;
  ctrl->pole_cmd.auto_target_enable[0] = true;
  ctrl->pole_cmd.auto_target_enable[1] = true;
  ctrl->pole_cmd.auto_target_lift[0] = front_target;
  ctrl->pole_cmd.auto_target_lift[1] = rear_target;
}