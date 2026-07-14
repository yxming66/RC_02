#include "module/autoCtrlAPI/rod/auto_rod_spearhead.h"

#include <string.h>

#define AUTO_ROD_SPEARHEAD_DEFAULT_OPEN_DELAY_MS (20u)
#define AUTO_ROD_SPEARHEAD_DEFAULT_DETECT_GRIP_DELAY_MS (100u)
#define AUTO_ROD_SPEARHEAD_DEFAULT_DETECT_POSE_DELAY_MS (2000u)
#define AUTO_ROD_SPEARHEAD_DEFAULT_DETECT_SUCCESS_HOLD_MS (500u)
#define AUTO_ROD_SPEARHEAD_DEFAULT_DOCK_WAIT_DELAY_MS (500u)
#define AUTO_ROD_SPEARHEAD_DEFAULT_PHOTO_CHECK_MS (1000u)
#define AUTO_ROD_SPEARHEAD_DEFAULT_PHOTO_TIMEOUT_MS (2000u)
#define AUTO_ROD_SPEARHEAD_DOCK_WAIT_POSE_DELAY_MS (300u)
#define AUTO_ROD_SPEARHEAD_DOCK_WAIT_STANDBY_THRESHOLD_RAD (18.0f)
#define AUTO_ROD_SPEARHEAD_STEP1_PICKUP_OFFSET_RAD (1.0f)

static uint32_t AutoRodSpearhead_OpenDelayMs(
    const AutoRodSpearhead_t *ctrl) {
  return ctrl->param.open_delay_ms > 0u
             ? ctrl->param.open_delay_ms
             : AUTO_ROD_SPEARHEAD_DEFAULT_OPEN_DELAY_MS;
}

static uint32_t AutoRodSpearhead_DetectGripDelayMs(
    const AutoRodSpearhead_t *ctrl) {
  return ctrl->param.detect_grip_delay_ms > 0u
             ? ctrl->param.detect_grip_delay_ms
             : AUTO_ROD_SPEARHEAD_DEFAULT_DETECT_GRIP_DELAY_MS;
}

static uint32_t AutoRodSpearhead_GrabHighDelayMs(
    const AutoRodSpearhead_t *ctrl) {
  return ctrl->param.grab_high_delay_ms > 0u
             ? ctrl->param.grab_high_delay_ms
             : AUTO_ROD_SPEARHEAD_DEFAULT_DETECT_SUCCESS_HOLD_MS;
}

static uint32_t AutoRodSpearhead_DetectPoseDelayMs(
    const AutoRodSpearhead_t *ctrl) {
  return ctrl->param.detect_pose_delay_ms > 0u
             ? ctrl->param.detect_pose_delay_ms
             : AUTO_ROD_SPEARHEAD_DEFAULT_DETECT_POSE_DELAY_MS;
}

static uint32_t AutoRodSpearhead_DetectSuccessHoldMs(
    const AutoRodSpearhead_t *ctrl) {
  return ctrl->param.detect_success_hold_ms > 0u
             ? ctrl->param.detect_success_hold_ms
             : AUTO_ROD_SPEARHEAD_DEFAULT_DETECT_SUCCESS_HOLD_MS;
}

static uint32_t AutoRodSpearhead_DockWaitDelayMs(
    const AutoRodSpearhead_t *ctrl) {
  return ctrl->param.dock_wait_delay_ms > 0u
             ? ctrl->param.dock_wait_delay_ms
             : AUTO_ROD_SPEARHEAD_DEFAULT_DOCK_WAIT_DELAY_MS;
}

static uint32_t AutoRodSpearhead_PhotoCheckMs(
    const AutoRodSpearhead_t *ctrl) {
  return ctrl->param.photo_check_ms > 0u
             ? ctrl->param.photo_check_ms
             : AUTO_ROD_SPEARHEAD_DEFAULT_PHOTO_CHECK_MS;
}

static uint32_t AutoRodSpearhead_PhotoTimeoutMs(
    const AutoRodSpearhead_t *ctrl) {
  const uint32_t configured_timeout =
      ctrl->param.photo_timeout_ms > 0u
          ? ctrl->param.photo_timeout_ms
          : AUTO_ROD_SPEARHEAD_DEFAULT_PHOTO_TIMEOUT_MS;
  const uint32_t stable_ms = AutoRodSpearhead_PhotoCheckMs(ctrl);
  return configured_timeout > stable_ms ? configured_timeout : stable_ms;
}

static OreStore_TransformPoint_t AutoRodSpearhead_DockWaitTransform(
    const AutoRodSpearhead_t *ctrl) {
  if (ctrl->param.dock_wait_transform < ORE_STORE_TRANSFORM_POINT_NUM) {
    return ctrl->param.dock_wait_transform;
  }
  return ORE_STORE_TRANSFORM_SPEARHEAD_DOCK_WAIT;
}

static uint32_t AutoRodSpearhead_StepElapsed(
    const AutoRodSpearhead_t *ctrl, uint32_t now_ms) {
  return now_ms - ctrl->step_enter_time_ms;
}

static float AutoRodSpearhead_RodPoseTargetRad(
    const AutoRodSpearhead_t *ctrl,
    RodNew_Pose_t pose) {
  if (ctrl == 0 || ctrl->param.rod_param == 0) {
    return 0.0f;
  }

  switch (pose) {
    case ROD_NEW_POSE_STANDBY:
      return ctrl->param.rod_param->servo.angle_standby_rad;
    case ROD_NEW_POSE_GRAB_HIGH:
      return ctrl->param.rod_param->servo.angle_grab_high_rad;
    case ROD_NEW_POSE_DETECT:
      return ctrl->param.rod_param->servo.angle_detect_rad;
    case ROD_NEW_POSE_DOCK_WAIT:
      return ctrl->param.rod_param->servo.angle_dock_wait_rad;
    case ROD_NEW_POSE_MANUAL:
    default:
      return 0.0f;
  }
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
    ctrl->ore_store_cmd_valid = false;
    ctrl->state = AUTO_ROD_SPEARHEAD_STATE_FAIL;
    ctrl->result = AUTO_ROD_SPEARHEAD_RESULT_FAIL;
    ctrl->fault = AUTO_ROD_SPEARHEAD_FAULT_INVALID_PARAM;
    return false;
  }

  memset(&ctrl->rod_cmd, 0, sizeof(ctrl->rod_cmd));
  ctrl->rod_cmd.mode = ROD_NEW_MODE_ACTIVE;
  ctrl->rod_cmd.pose = pose;
  ctrl->rod_cmd.grip = grip;
  ctrl->rod_cmd.target_angle_rad =
      AutoRodSpearhead_RodPoseTargetRad(ctrl, pose);
  ctrl->rod_cmd_valid = true;
  return true;
}

static bool AutoRodSpearhead_CommandOreStore(
    AutoRodSpearhead_t *ctrl,
    OreStore_TransformPoint_t transform,
    bool cylinder_closed) {
  ctrl->ore_store_cmd_valid = OreStore_MakePresetCommand(
      ctrl->param.ore_store_param, transform, cylinder_closed,
      &ctrl->ore_store_cmd);
  if (!ctrl->ore_store_cmd_valid) {
    ctrl->rod_cmd_valid = false;
    ctrl->state = AUTO_ROD_SPEARHEAD_STATE_FAIL;
    ctrl->result = AUTO_ROD_SPEARHEAD_RESULT_FAIL;
    ctrl->fault = AUTO_ROD_SPEARHEAD_FAULT_INVALID_PARAM;
  }
  return ctrl->ore_store_cmd_valid;
}

static bool AutoRodSpearhead_CommandOreStoreOffset(
    AutoRodSpearhead_t *ctrl,
    OreStore_TransformPoint_t transform,
    bool cylinder_closed,
    float offset_rad) {
  if (!AutoRodSpearhead_CommandOreStore(ctrl, transform, cylinder_closed)) {
    return false;
  }
  ctrl->ore_store_cmd.platform_target_rad += offset_rad;
  return true;
}

static void AutoRodSpearhead_ClearOutputs(AutoRodSpearhead_t *ctrl) {
  ctrl->rod_cmd_valid = false;
  ctrl->ore_store_cmd_valid = false;
}

static bool AutoRodSpearhead_PhotoStateStable(AutoRodSpearhead_t *ctrl,
                                              bool state,
                                              uint32_t now_ms) {
  if (!ctrl->photo_stable_started ||
      ctrl->photo_stable_state != state) {
    ctrl->photo_stable_started = true;
    ctrl->photo_stable_state = state;
    ctrl->photo_stable_start_time_ms = now_ms;
    return false;
  }

  return (now_ms - ctrl->photo_stable_start_time_ms) >=
         AutoRodSpearhead_PhotoCheckMs(ctrl);
}

static void AutoRodSpearhead_FinishSuccess(AutoRodSpearhead_t *ctrl) {
  ctrl->state = AUTO_ROD_SPEARHEAD_STATE_SUCCESS;
  ctrl->result = AUTO_ROD_SPEARHEAD_RESULT_SUCCESS;
  ctrl->fault = AUTO_ROD_SPEARHEAD_FAULT_NONE;
  AutoRodSpearhead_ClearOutputs(ctrl);
}

static void AutoRodSpearhead_FinishNoSpearhead(AutoRodSpearhead_t *ctrl) {
  ctrl->state = AUTO_ROD_SPEARHEAD_STATE_FAIL;
  ctrl->result = AUTO_ROD_SPEARHEAD_RESULT_FAIL;
  ctrl->fault = AUTO_ROD_SPEARHEAD_FAULT_NO_SPEARHEAD;
  AutoRodSpearhead_ClearOutputs(ctrl);
}

static void AutoRodSpearhead_FinishTimeout(AutoRodSpearhead_t *ctrl) {
  ctrl->state = AUTO_ROD_SPEARHEAD_STATE_FAIL;
  ctrl->result = AUTO_ROD_SPEARHEAD_RESULT_FAIL;
  ctrl->fault = AUTO_ROD_SPEARHEAD_FAULT_TIMEOUT;
  AutoRodSpearhead_ClearOutputs(ctrl);
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

static bool AutoRodSpearhead_StartAction(AutoRodSpearhead_t *ctrl,
                                         AutoRodSpearhead_Action_t action,
                                         uint32_t now_ms) {
  if (ctrl == 0 || AutoRodSpearhead_IsBusy(ctrl)) {
    return false;
  }
  if (ctrl->param.rod_param == 0 || ctrl->param.ore_store_param == 0 ||
      action == AUTO_ROD_SPEARHEAD_ACTION_NONE) {
    ctrl->state = AUTO_ROD_SPEARHEAD_STATE_FAIL;
    ctrl->result = AUTO_ROD_SPEARHEAD_RESULT_FAIL;
    ctrl->fault = AUTO_ROD_SPEARHEAD_FAULT_INVALID_PARAM;
    return false;
  }
  ctrl->state = AUTO_ROD_SPEARHEAD_STATE_RUNNING;
  ctrl->result = AUTO_ROD_SPEARHEAD_RESULT_RUNNING;
  ctrl->fault = AUTO_ROD_SPEARHEAD_FAULT_NONE;
  ctrl->action = action;
  ctrl->step_index = 0u;
  ctrl->step_entered = false;
  ctrl->step_enter_time_ms = now_ms;
  ctrl->photo_stable_started = false;
  ctrl->photo_stable_state = false;
  ctrl->photo_stable_start_time_ms = now_ms;
  ctrl->dock_complete_latched = false;
  ctrl->dock_wait_local_ready = false;
  ctrl->dock_wait_ready_time_ms = 0u;
  AutoRodSpearhead_ClearOutputs(ctrl);
  return true;
}

bool AutoRodSpearhead_Start(AutoRodSpearhead_t *ctrl, uint32_t now_ms) {
  return AutoRodSpearhead_StartPickup(ctrl, now_ms);
}

bool AutoRodSpearhead_StartPickup(AutoRodSpearhead_t *ctrl,
                                  uint32_t now_ms) {
  return AutoRodSpearhead_StartAction(
      ctrl, AUTO_ROD_SPEARHEAD_ACTION_PICKUP, now_ms);
}

bool AutoRodSpearhead_StartPickupStep1(AutoRodSpearhead_t *ctrl,
                                       uint32_t now_ms) {
  return AutoRodSpearhead_StartAction(
      ctrl, AUTO_ROD_SPEARHEAD_ACTION_PICKUP_STEP1, now_ms);
}

bool AutoRodSpearhead_StartPickupStep2(AutoRodSpearhead_t *ctrl,
                                       uint32_t now_ms) {
  return AutoRodSpearhead_StartAction(
      ctrl, AUTO_ROD_SPEARHEAD_ACTION_PICKUP_STEP2, now_ms);
}

bool AutoRodSpearhead_StartDockWait(AutoRodSpearhead_t *ctrl,
                                    uint32_t now_ms) {
  return AutoRodSpearhead_StartAction(
      ctrl, AUTO_ROD_SPEARHEAD_ACTION_DOCK_WAIT, now_ms);
}

static void AutoRodSpearhead_RunPickupStep1(
    AutoRodSpearhead_t *ctrl,
    const AutoRodSpearhead_Feedback_t *feedback,
    uint32_t now_ms) {
  const bool ore_store_at_target =
      feedback != 0 && feedback->ore_store_at_target;

  switch (ctrl->step_index) {
    case 0:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStoreOffset(
              ctrl, ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP, false,
              AUTO_ROD_SPEARHEAD_STEP1_PICKUP_OFFSET_RAD)) {
        return;
      }
      if (ore_store_at_target) {
        AutoRodSpearhead_NextStep(ctrl);
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_DockWaitDelayMs(ctrl)) {
        AutoRodSpearhead_FinishTimeout(ctrl);
      }
      return;
    case 1:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStoreOffset(
              ctrl, ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP, false,
              AUTO_ROD_SPEARHEAD_STEP1_PICKUP_OFFSET_RAD) ||
          !AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_STANDBY,
                                       ROD_NEW_GRIP_RELEASE)) {
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_DetectPoseDelayMs(ctrl)) {
        AutoRodSpearhead_FinishSuccess(ctrl);
      }
      return;
    default:
      AutoRodSpearhead_FinishSuccess(ctrl);
      return;
  }
}

static void AutoRodSpearhead_RunPickupStep2(
    AutoRodSpearhead_t *ctrl,
    const AutoRodSpearhead_Feedback_t *feedback,
    uint32_t now_ms) {
  const bool rod_photo_triggered =
      feedback != 0 && feedback->rod_photo_triggered;
  const bool ore_store_at_target =
      feedback != 0 && feedback->ore_store_at_target;

  switch (ctrl->step_index) {
    case 0:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStore(
              ctrl, ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP, false) ||
          !AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_STANDBY,
                                       ROD_NEW_GRIP_RELEASE)) {
        return;
      }
      if (ore_store_at_target) {
        AutoRodSpearhead_NextStep(ctrl);
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_DockWaitDelayMs(ctrl)) {
        AutoRodSpearhead_FinishTimeout(ctrl);
      }
      return;
    case 1:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStore(
              ctrl, ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP, false) ||
          !AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_STANDBY,
                                       ROD_NEW_GRIP_RELEASE)) {
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_DetectGripDelayMs(ctrl)) {
        AutoRodSpearhead_NextStep(ctrl);
      }
      return;
    case 2:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStore(
              ctrl, ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP, false) ||
          !AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_STANDBY,
                                       ROD_NEW_GRIP_GRAB)) {
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_GrabHighDelayMs(ctrl)) {
        AutoRodSpearhead_NextStep(ctrl);
      }
      return;
    case 3:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStore(
              ctrl, ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP, false) ||
          !AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_DETECT,
                                       ROD_NEW_GRIP_GRAB)) {
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_DetectPoseDelayMs(ctrl)) {
        ctrl->photo_stable_started = false;
        ctrl->photo_stable_state = false;
        ctrl->photo_stable_start_time_ms = now_ms;
        AutoRodSpearhead_NextStep(ctrl);
      }
      return;
    case 4:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStore(
              ctrl, ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP, false) ||
          !AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_DETECT,
                                       ROD_NEW_GRIP_GRAB)) {
        return;
      }
      if (!ctrl->param.use_photo_check) {
        AutoRodSpearhead_NextStep(ctrl);
        return;
      }
      if (!rod_photo_triggered) {
        ctrl->photo_stable_started = false;
        ctrl->photo_stable_state = false;
        ctrl->photo_stable_start_time_ms = now_ms;
        if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
            AutoRodSpearhead_PhotoTimeoutMs(ctrl)) {
          AutoRodSpearhead_FinishNoSpearhead(ctrl);
        }
        return;
      }
      if (!AutoRodSpearhead_PhotoStateStable(ctrl, rod_photo_triggered,
                                             now_ms)) {
        return;
      }
      AutoRodSpearhead_NextStep(ctrl);
      return;
    case 5:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStore(
              ctrl, ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP, false) ||
          !AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_GRAB_HIGH,
                                       ROD_NEW_GRIP_GRAB)) {
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_DetectSuccessHoldMs(ctrl)) {
        AutoRodSpearhead_FinishSuccess(ctrl);
      }
      return;
    default:
      AutoRodSpearhead_FinishSuccess(ctrl);
      return;
  }
}

static void AutoRodSpearhead_RunPickup(
    AutoRodSpearhead_t *ctrl,
    const AutoRodSpearhead_Feedback_t *feedback,
    uint32_t now_ms) {
  const bool rod_photo_triggered =
      feedback != 0 && feedback->rod_photo_triggered;
  const bool ore_store_at_target =
      feedback != 0 && feedback->ore_store_at_target;

  switch (ctrl->step_index) {
    case 0:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStore(
              ctrl, ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP, false)) {
        return;
      }
      if (ore_store_at_target) {
        AutoRodSpearhead_NextStep(ctrl);
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_DockWaitDelayMs(ctrl)) {
        AutoRodSpearhead_FinishTimeout(ctrl);
      }
      return;
    case 1:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStore(
              ctrl, ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP, false) ||
          !AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_STANDBY,
                                       ROD_NEW_GRIP_RELEASE)) {
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_OpenDelayMs(ctrl)) {
        AutoRodSpearhead_NextStep(ctrl);
      }
      return;
    case 2:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStore(
              ctrl, ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP, false) ||
          !AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_DOCK_WAIT,
                                       ROD_NEW_GRIP_GRAB)) {
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_DetectGripDelayMs(ctrl)) {
        AutoRodSpearhead_NextStep(ctrl);
      }
      return;
    case 3:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStore(
              ctrl, ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP, false) ||
          !AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_DETECT,
                                       ROD_NEW_GRIP_GRAB)) {
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_DetectPoseDelayMs(ctrl)) {
        ctrl->photo_stable_started = false;
        ctrl->photo_stable_state = false;
        ctrl->photo_stable_start_time_ms = now_ms;
        AutoRodSpearhead_NextStep(ctrl);
      }
      return;
    case 4:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStore(
              ctrl, ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP, false) ||
          !AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_DETECT,
                                       ROD_NEW_GRIP_GRAB)) {
        return;
      }
      if (!ctrl->param.use_photo_check) {
        AutoRodSpearhead_NextStep(ctrl);
        return;
      }
      if (!rod_photo_triggered) {
        ctrl->photo_stable_started = false;
        ctrl->photo_stable_state = false;
        ctrl->photo_stable_start_time_ms = now_ms;
        if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
            AutoRodSpearhead_PhotoTimeoutMs(ctrl)) {
          AutoRodSpearhead_FinishNoSpearhead(ctrl);
        }
        return;
      }
      if (!AutoRodSpearhead_PhotoStateStable(ctrl, rod_photo_triggered,
                                             now_ms)) {
        return;
      }
      AutoRodSpearhead_NextStep(ctrl);
      return;
    case 5:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStore(
              ctrl, ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP, false) ||
          !AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_GRAB_HIGH,
                                       ROD_NEW_GRIP_GRAB)) {
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_DetectSuccessHoldMs(ctrl)) {
        AutoRodSpearhead_FinishSuccess(ctrl);
      }
      return;
    default:
      AutoRodSpearhead_FinishSuccess(ctrl);
      return;
  }
}

static void AutoRodSpearhead_RunDockWait(
    AutoRodSpearhead_t *ctrl,
    const AutoRodSpearhead_Feedback_t *feedback,
    uint32_t now_ms) {
  const bool transform_position_high =
      feedback == 0 || !feedback->ore_store_position_valid ||
      feedback->ore_store_platform_position_rad >
          AUTO_ROD_SPEARHEAD_DOCK_WAIT_STANDBY_THRESHOLD_RAD;

  switch (ctrl->step_index) {
    case 0:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStore(
              ctrl, AutoRodSpearhead_DockWaitTransform(ctrl), false) ||
          !AutoRodSpearhead_CommandRod(
              ctrl,
              transform_position_high ? ROD_NEW_POSE_STANDBY
                                      : ROD_NEW_POSE_DOCK_WAIT,
              ROD_NEW_GRIP_GRAB)) {
        return;
      }
      if (!transform_position_high) {
        AutoRodSpearhead_NextStep(ctrl);
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AUTO_ROD_SPEARHEAD_DOCK_WAIT_POSE_DELAY_MS) {
        AutoRodSpearhead_NextStep(ctrl);
        return;
      }
      return;
    case 1:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStore(
              ctrl, AutoRodSpearhead_DockWaitTransform(ctrl), false) ||
          !AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_DOCK_WAIT,
                                       ROD_NEW_GRIP_GRAB)) {
        return;
      }
      const bool local_ready =
          feedback != 0 && feedback->ore_store_at_target &&
          feedback->rod_dock_wait_at_target;
      if (!local_ready) {
        ctrl->dock_wait_local_ready = false;
        ctrl->dock_wait_ready_time_ms = 0u;
      } else {
        if (!ctrl->dock_wait_local_ready) {
          ctrl->dock_wait_local_ready = true;
          ctrl->dock_wait_ready_time_ms = now_ms;
        }
        if (feedback->dock_complete_received &&
            feedback->dock_complete_rx_ms >=
                ctrl->dock_wait_ready_time_ms) {
          ctrl->dock_complete_latched = true;
        }
      }
      if (ctrl->dock_complete_latched) {
        AutoRodSpearhead_NextStep(ctrl);
        return;
      }
      /* No docking timeout: keep the platform and servo at their verified
       * local-ready poses until a fresh completion signal or an abort. */
      return;
    case 2:
      AutoRodSpearhead_EnterStep(ctrl, now_ms);
      if (!AutoRodSpearhead_CommandOreStore(
              ctrl, AutoRodSpearhead_DockWaitTransform(ctrl), false) ||
          !AutoRodSpearhead_CommandRod(ctrl, ROD_NEW_POSE_DOCK_WAIT,
                                       ROD_NEW_GRIP_RELEASE)) {
        return;
      }
      if (AutoRodSpearhead_StepElapsed(ctrl, now_ms) >=
          AutoRodSpearhead_OpenDelayMs(ctrl)) {
        AutoRodSpearhead_FinishSuccess(ctrl);
      }
      return;
    default:
      AutoRodSpearhead_FinishSuccess(ctrl);
      return;
  }
}

void AutoRodSpearhead_Update(AutoRodSpearhead_t *ctrl,
                             const AutoRodSpearhead_Feedback_t *feedback,
                             uint32_t now_ms) {
  if (ctrl == 0 || ctrl->state != AUTO_ROD_SPEARHEAD_STATE_RUNNING) {
    return;
  }

  switch (ctrl->action) {
    case AUTO_ROD_SPEARHEAD_ACTION_PICKUP:
      AutoRodSpearhead_RunPickup(ctrl, feedback, now_ms);
      return;
    case AUTO_ROD_SPEARHEAD_ACTION_PICKUP_STEP1:
      AutoRodSpearhead_RunPickupStep1(ctrl, feedback, now_ms);
      return;
    case AUTO_ROD_SPEARHEAD_ACTION_PICKUP_STEP2:
      AutoRodSpearhead_RunPickupStep2(ctrl, feedback, now_ms);
      return;
    case AUTO_ROD_SPEARHEAD_ACTION_DOCK_WAIT:
      AutoRodSpearhead_RunDockWait(ctrl, feedback, now_ms);
      return;
    case AUTO_ROD_SPEARHEAD_ACTION_NONE:
    default:
      ctrl->state = AUTO_ROD_SPEARHEAD_STATE_FAIL;
      ctrl->result = AUTO_ROD_SPEARHEAD_RESULT_FAIL;
      ctrl->fault = AUTO_ROD_SPEARHEAD_FAULT_INVALID_PARAM;
      AutoRodSpearhead_ClearOutputs(ctrl);
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
  ctrl->action = AUTO_ROD_SPEARHEAD_ACTION_NONE;
  AutoRodSpearhead_ClearOutputs(ctrl);
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

AutoRodSpearhead_Action_t AutoRodSpearhead_GetAction(
    const AutoRodSpearhead_t *ctrl) {
  return ctrl == 0 ? AUTO_ROD_SPEARHEAD_ACTION_NONE : ctrl->action;
}

uint8_t AutoRodSpearhead_GetStepIndex(const AutoRodSpearhead_t *ctrl) {
  return ctrl == 0 ? 0u : ctrl->step_index;
}

const RodNew_CMD_t *AutoRodSpearhead_GetRodCommand(
    const AutoRodSpearhead_t *ctrl) {
  return (ctrl != 0 && ctrl->rod_cmd_valid) ? &ctrl->rod_cmd : 0;
}

const OreStore_CMD_t *AutoRodSpearhead_GetOreStoreCommand(
    const AutoRodSpearhead_t *ctrl) {
  return (ctrl != 0 && ctrl->ore_store_cmd_valid) ? &ctrl->ore_store_cmd : 0;
}
