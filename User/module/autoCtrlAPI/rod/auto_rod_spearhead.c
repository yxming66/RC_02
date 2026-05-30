#include "module/autoCtrlAPI/rod/auto_rod_spearhead.h"

#include <string.h>

#define AUTO_ROD_SPEARHEAD_DEFAULT_OPEN_DELAY_MS (20u)
#define AUTO_ROD_SPEARHEAD_DEFAULT_GRAB_HIGH_DELAY_MS (500u)
#define AUTO_ROD_SPEARHEAD_DEFAULT_DOCK_WAIT_DELAY_MS (500u)

static uint32_t AutoRodSpearhead_OpenDelayMs(
    const AutoRodSpearhead_t *ctrl) {
  return ctrl->param.open_delay_ms > 0u
             ? ctrl->param.open_delay_ms
             : AUTO_ROD_SPEARHEAD_DEFAULT_OPEN_DELAY_MS;
}

static uint32_t AutoRodSpearhead_GrabHighDelayMs(
    const AutoRodSpearhead_t *ctrl) {
  return ctrl->param.grab_high_delay_ms > 0u
             ? ctrl->param.grab_high_delay_ms
             : AUTO_ROD_SPEARHEAD_DEFAULT_GRAB_HIGH_DELAY_MS;
}

static uint32_t AutoRodSpearhead_DockWaitDelayMs(
    const AutoRodSpearhead_t *ctrl) {
  return ctrl->param.dock_wait_delay_ms > 0u
             ? ctrl->param.dock_wait_delay_ms
             : AUTO_ROD_SPEARHEAD_DEFAULT_DOCK_WAIT_DELAY_MS;
}

static uint32_t AutoRodSpearhead_StepElapsed(
    const AutoRodSpearhead_t *ctrl, uint32_t now_ms) {
  return now_ms - ctrl->step_enter_time_ms;
}

static void AutoRodSpearhead_EnterStep(AutoRodSpearhead_t *ctrl,
                                       uint32_t now_ms) {
  if (!ctrl->step_entered) {
    ctrl->step_entered = true;
    ctrl->step_enter_time_ms = now_ms;
  }
}

static void AutoRodSpearhead_NextStep(AutoRodSpearhead_t *ctrl) {
  ctrl->step_index++;
  ctrl->step_entered = false;
}

static bool AutoRodSpearhead_CommandRod(AutoRodSpearhead_t *ctrl,
                                        RodNew_Pose_t pose,
                                        RodNew_GripState_t grip) {
  if (ctrl->param.rod_param == 0) {
    ctrl->rod_cmd_valid = false;
    ctrl->state = AUTO_ROD_SPEARHEAD_STATE_FAIL;
    ctrl->result = AUTO_ROD_SPEARHEAD_RESULT_FAIL;
    ctrl->fault = AUTO_ROD_SPEARHEAD_FAULT_INVALID_PARAM;
    return false;
  }

  memset(&ctrl->rod_cmd, 0, sizeof(ctrl->rod_cmd));
  ctrl->rod_cmd.mode = ROD_NEW_MODE_ACTIVE;
  ctrl->rod_cmd.pose = pose;
  ctrl->rod_cmd.grip = grip;
  ctrl->rod_cmd.target_angle_rad = 0.0f;
  ctrl->rod_cmd_valid = true;
  return true;
}

static void AutoRodSpearhead_FinishSuccess(AutoRodSpearhead_t *ctrl) {
  ctrl->state = AUTO_ROD_SPEARHEAD_STATE_SUCCESS;
  ctrl->result = AUTO_ROD_SPEARHEAD_RESULT_SUCCESS;
  ctrl->fault = AUTO_ROD_SPEARHEAD_FAULT_NONE;
}

void AutoRodSpearhead_Init(AutoRodSpearhead_t *ctrl,
                           const AutoRodSpearhead_Params_t *param) {
  if (ctrl == 0) {
    return;
  }
  memset(ctrl, 0, sizeof(*ctrl));
  if (param != 0) {
    ctrl->param = *param;
  }
  ctrl->state = AUTO_ROD_SPEARHEAD_STATE_IDLE;
  ctrl->result = AUTO_ROD_SPEARHEAD_RESULT_NONE;
  ctrl->fault = AUTO_ROD_SPEARHEAD_FAULT_NONE;
}

bool AutoRodSpearhead_Start(AutoRodSpearhead_t *ctrl, uint32_t now_ms) {
  if (ctrl == 0 || AutoRodSpearhead_IsBusy(ctrl)) {
    return false;
  }
  if (ctrl->param.rod_param == 0) {
    ctrl->state = AUTO_ROD_SPEARHEAD_STATE_FAIL;
    ctrl->result = AUTO_ROD_SPEARHEAD_RESULT_FAIL;
    ctrl->fault = AUTO_ROD_SPEARHEAD_FAULT_INVALID_PARAM;
    return false;
  }
  ctrl->state = AUTO_ROD_SPEARHEAD_STATE_RUNNING;
  ctrl->result = AUTO_ROD_SPEARHEAD_RESULT_RUNNING;
  ctrl->fault = AUTO_ROD_SPEARHEAD_FAULT_NONE;
  ctrl->step_index = 0u;
  ctrl->step_entered = false;
  ctrl->step_enter_time_ms = now_ms;
  ctrl->rod_cmd_valid = false;
  return true;
}

void AutoRodSpearhead_Update(AutoRodSpearhead_t *ctrl, uint32_t now_ms) {
  if (ctrl == 0 || ctrl->state != AUTO_ROD_SPEARHEAD_STATE_RUNNING) {
    return;
  }

  ctrl->rod_cmd_valid = false;
  switch (ctrl->step_index) {
    case 0:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_STANDBY,
                                       ROD_NEW_GRIP_RELEASE)) {
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_OpenDelayMs(ctrl)) {
        AutoRodSpearhead_NextStep(ctrl);
      }
      return;
    case 1:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_GRAB_HIGH,
                                       ROD_NEW_GRIP_RELEASE)) {
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_GrabHighDelayMs(ctrl)) {
        AutoRodSpearhead_NextStep(ctrl);
      }
      return;
    case 2:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_DOCK_WAIT,
                                       ROD_NEW_GRIP_RELEASE)) {
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_DockWaitDelayMs(ctrl)) {
        AutoRodSpearhead_FinishSuccess(ctrl);
      }
      return;
    default:
      AutoRodSpearhead_FinishSuccess(ctrl);
      return;
  }
}

void AutoRodSpearhead_Abort(AutoRodSpearhead_t *ctrl) {
  if (ctrl == 0) {
    return;
  }
  ctrl->state = AUTO_ROD_SPEARHEAD_STATE_ABORT;
  ctrl->result = AUTO_ROD_SPEARHEAD_RESULT_ABORTED;
  ctrl->fault = AUTO_ROD_SPEARHEAD_FAULT_ABORTED;
  ctrl->rod_cmd_valid = false;
}

bool AutoRodSpearhead_IsBusy(const AutoRodSpearhead_t *ctrl) {
  return ctrl != 0 && ctrl->state == AUTO_ROD_SPEARHEAD_STATE_RUNNING;
}

AutoRodSpearhead_State_t AutoRodSpearhead_GetState(
    const AutoRodSpearhead_t *ctrl) {
  return ctrl == 0 ? AUTO_ROD_SPEARHEAD_STATE_IDLE : ctrl->state;
}

AutoRodSpearhead_Result_t AutoRodSpearhead_GetResult(
    const AutoRodSpearhead_t *ctrl) {
  return ctrl == 0 ? AUTO_ROD_SPEARHEAD_RESULT_NONE : ctrl->result;
}

AutoRodSpearhead_Fault_t AutoRodSpearhead_GetFault(
    const AutoRodSpearhead_t *ctrl) {
  return ctrl == 0 ? AUTO_ROD_SPEARHEAD_FAULT_INVALID_PARAM : ctrl->fault;
}

uint8_t AutoRodSpearhead_GetStepIndex(const AutoRodSpearhead_t *ctrl) {
  return ctrl == 0 ? 0u : ctrl->step_index;
}

const RodNew_CMD_t *AutoRodSpearhead_GetRodCommand(
    const AutoRodSpearhead_t *ctrl) {
  return (ctrl != 0 && ctrl->rod_cmd_valid) ? &ctrl->rod_cmd : 0;
}
