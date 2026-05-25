#include "module/autoCtrlAPI/ore_store/auto_ore_store.h"

#include <string.h>

#define AUTO_ORE_DELAY_STORE_SUCTION_OFF_MS (50u)
#define AUTO_ORE_DELAY_RELEASE_ARM_SETTLE_MS (10u)
#define AUTO_ORE_DELAY_RELEASE_SUCTION_OFF_MS (10u)
#define AUTO_ORE_DELAY_RELEASE_TRACK_MS (20u)
#define AUTO_ORE_DELAY_RELEASE_GATE_MS (10u)
#define AUTO_ORE_DELAY_RELEASE_TRANSFORM_MS (10u)
#define AUTO_ORE_DEFAULT_STEP_TIMEOUT_MS (5000u)

static uint32_t AutoOre_StepElapsed(const AutoOre_t *ctrl, uint32_t now_ms) {
  return now_ms - ctrl->step_enter_time_ms;
}

static void AutoOre_EnterStep(AutoOre_t *ctrl, uint32_t now_ms) {
  if (!ctrl->step_entered) {
    ctrl->step_entered = true;
    ctrl->step_enter_time_ms = now_ms;
  }
}

static void AutoOre_NextStep(AutoOre_t *ctrl) {
  ctrl->step_index++;
  ctrl->step_entered = false;
}

static uint32_t AutoOre_StepTimeoutMs(const AutoOre_t *ctrl) {
  return (ctrl->param.default_step_timeout_ms > 0u)
             ? ctrl->param.default_step_timeout_ms
             : AUTO_ORE_DEFAULT_STEP_TIMEOUT_MS;
}

static bool AutoOre_CheckTimeout(AutoOre_t *ctrl, uint32_t now_ms) {
  if (AutoOre_StepElapsed(ctrl, now_ms) < AutoOre_StepTimeoutMs(ctrl)) {
    return false;
  }
  ctrl->state = AUTO_ORE_STATE_FAIL;
  ctrl->result = AUTO_ORE_RESULT_FAIL;
  ctrl->fault = AUTO_ORE_FAULT_TIMEOUT;
  return true;
}

static void AutoOre_ClearOutputs(AutoOre_t *ctrl) {
  ctrl->arm_cmd_valid = false;
  ctrl->ore_store_cmd_valid = false;
  ctrl->pole_cmd_valid = false;
}

static bool AutoOre_CommandArm(AutoOre_t *ctrl, ArmSimple_BehaviorPoint_t point,
                               Suction_State_t suction) {
  ctrl->arm_cmd_valid = ArmSimple_MakeBehaviorCommand(
      ctrl->param.arm_param, point, suction, &ctrl->arm_cmd);
  return ctrl->arm_cmd_valid;
}

static bool AutoOre_CommandOreStore(AutoOre_t *ctrl, OreStore_TrackPoint_t track,
                                    OreStore_TransformPoint_t transform,
                                    OreStore_GatePoint_t gate,
                                    bool cylinder_closed) {
  ctrl->ore_store_cmd_valid = OreStore_MakePresetCommand(
      ctrl->param.ore_store_param, track, transform, gate, cylinder_closed,
      &ctrl->ore_store_cmd);
  return ctrl->ore_store_cmd_valid;
}

static bool AutoOre_CommandReleasePole(AutoOre_t *ctrl) {
  if (ctrl->param.pole_param == 0) {
    ctrl->pole_cmd_valid = false;
    return false;
  }

  memset(&ctrl->pole_cmd, 0, sizeof(ctrl->pole_cmd));
  ctrl->pole_cmd.mode = POLE_MODE_ACTIVE;
  ctrl->pole_cmd.lift[0] = 0.0f;
  ctrl->pole_cmd.lift[1] = 0.0f;
  ctrl->pole_cmd.auto_target_enable[0] = true;
  ctrl->pole_cmd.auto_target_enable[1] = true;
  ctrl->pole_cmd.auto_target_lift[0] =
      ctrl->param.pole_param->preset.ore_release_target[0];
  ctrl->pole_cmd.auto_target_lift[1] =
      ctrl->param.pole_param->preset.ore_release_target[1];
  ctrl->pole_cmd.auto_lift_speed[0] =
      ctrl->param.pole_param->preset.ore_release_speed;
  ctrl->pole_cmd.auto_lift_speed[1] =
      ctrl->param.pole_param->preset.ore_release_speed;
  ctrl->pole_cmd_valid = true;
  return true;
}

static void AutoOre_FailInvalidParam(AutoOre_t *ctrl) {
  ctrl->state = AUTO_ORE_STATE_FAIL;
  ctrl->result = AUTO_ORE_RESULT_FAIL;
  ctrl->fault = AUTO_ORE_FAULT_INVALID_PARAM;
}

static void AutoOre_FinishSuccess(AutoOre_t *ctrl) {
  if (ctrl->action == AUTO_ORE_ACTION_STORE && ctrl->held_ore_count < 3u) {
    ctrl->held_ore_count++;
  } else if (ctrl->action == AUTO_ORE_ACTION_RELEASE &&
             ctrl->held_ore_count > 0u) {
    ctrl->held_ore_count--;
  }
  ctrl->state = AUTO_ORE_STATE_SUCCESS;
  ctrl->result = AUTO_ORE_RESULT_SUCCESS;
  ctrl->fault = AUTO_ORE_FAULT_NONE;
  ctrl->action = AUTO_ORE_ACTION_NONE;
}

static bool AutoOre_WaitAll(AutoOre_t *ctrl, uint32_t now_ms) {
  if (ctrl->feedback.arm_at_target && ctrl->feedback.ore_store_all_at_target) {
    AutoOre_NextStep(ctrl);
    return true;
  }
  return AutoOre_CheckTimeout(ctrl, now_ms);
}

static void AutoOre_RunStore0(AutoOre_t *ctrl, uint32_t now_ms) {
  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE, SUCTION_ON) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_STANDBY,
                                   ORE_STORE_TRANSFORM_MID_WAIT,
                                   ORE_STORE_GATE_CLOSED, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      (void)AutoOre_WaitAll(ctrl, now_ms);
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      (void)AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
                               SUCTION_OFF);
      if (AutoOre_StepElapsed(ctrl, now_ms) >=
          AUTO_ORE_DELAY_STORE_SUCTION_OFF_MS) {
        AutoOre_NextStep(ctrl);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_STANDBY,
                                   ORE_STORE_TRANSFORM_LIFT,
                                   ORE_STORE_GATE_CLOSED, true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.ore_store_all_at_target) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 3:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_STANDBY,
                                   ORE_STORE_TRANSFORM_LIFT,
                                   ORE_STORE_GATE_OPEN, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.ore_store_all_at_target) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 4:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_STANDBY,
                                   ORE_STORE_TRANSFORM_STANDBY,
                                   ORE_STORE_GATE_OPEN, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.ore_store_all_at_target) {
        AutoOre_FinishSuccess(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    default:
      AutoOre_FinishSuccess(ctrl);
      return;
  }
}

static void AutoOre_RunStore1(AutoOre_t *ctrl, uint32_t now_ms) {
  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE, SUCTION_ON) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_STANDBY,
                                   ORE_STORE_TRANSFORM_STANDBY,
                                   ORE_STORE_GATE_CLOSED, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      (void)AutoOre_WaitAll(ctrl, now_ms);
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      (void)AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
                               SUCTION_OFF);
      if (AutoOre_StepElapsed(ctrl, now_ms) >=
          AUTO_ORE_DELAY_STORE_SUCTION_OFF_MS) {
        AutoOre_NextStep(ctrl);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_STANDBY,
                                   ORE_STORE_TRANSFORM_STANDBY,
                                   ORE_STORE_GATE_CLOSED, true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      AutoOre_FinishSuccess(ctrl);
      return;
    default:
      AutoOre_FinishSuccess(ctrl);
      return;
  }
}

static void AutoOre_RunStore2(AutoOre_t *ctrl, uint32_t now_ms) {
  AutoOre_EnterStep(ctrl, now_ms);
  if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY, SUCTION_ON)) {
    AutoOre_FailInvalidParam(ctrl);
    return;
  }
  if (ctrl->feedback.arm_at_target) {
    AutoOre_FinishSuccess(ctrl);
  } else {
    (void)AutoOre_CheckTimeout(ctrl, now_ms);
  }
}

static void AutoOre_RunReleaseCommonTrackGate(AutoOre_t *ctrl,
                                              uint32_t now_ms) {
  switch (ctrl->step_index) {
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandReleasePole(ctrl) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_RELEASE,
                                   ORE_STORE_TRANSFORM_STANDBY,
                                   ORE_STORE_GATE_CLOSED, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.pole_all_at_target &&
          ctrl->feedback.ore_store_all_at_target &&
          AutoOre_StepElapsed(ctrl, now_ms) >=
              AUTO_ORE_DELAY_RELEASE_TRACK_MS) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 3:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_RELEASE,
                                   ORE_STORE_TRANSFORM_STANDBY,
                                   ORE_STORE_GATE_OPEN, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.ore_store_all_at_target &&
          AutoOre_StepElapsed(ctrl, now_ms) >= AUTO_ORE_DELAY_RELEASE_GATE_MS) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 4:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_STANDBY,
                                   ORE_STORE_TRANSFORM_STANDBY,
                                   ORE_STORE_GATE_OPEN, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.ore_store_all_at_target) {
        AutoOre_FinishSuccess(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    default:
      AutoOre_FinishSuccess(ctrl);
      return;
  }
}

static void AutoOre_RunRelease3(AutoOre_t *ctrl, uint32_t now_ms) {
  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandReleasePole(ctrl) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_STANDBY,
                                   ORE_STORE_TRANSFORM_STANDBY,
                                   ORE_STORE_GATE_CLOSED, true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.pole_all_at_target &&
          ctrl->feedback.ore_store_all_at_target) {
        ctrl->step_index = 2u;
        ctrl->step_entered = false;
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    default:
      AutoOre_RunReleaseCommonTrackGate(ctrl, now_ms);
      return;
  }
}

static void AutoOre_RunRelease2(AutoOre_t *ctrl, uint32_t now_ms) {
  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_STANDBY,
                                   ORE_STORE_TRANSFORM_STANDBY,
                                   ORE_STORE_GATE_CLOSED, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      AutoOre_NextStep(ctrl);
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
                              AutoOre_StepElapsed(ctrl, now_ms) >=
                                      AUTO_ORE_DELAY_RELEASE_ARM_SETTLE_MS
                                  ? SUCTION_OFF
                                  : SUCTION_ON)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.arm_at_target &&
          AutoOre_StepElapsed(ctrl, now_ms) >=
              (AUTO_ORE_DELAY_RELEASE_ARM_SETTLE_MS +
               AUTO_ORE_DELAY_RELEASE_SUCTION_OFF_MS)) {
        if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_STANDBY,
                                     ORE_STORE_TRANSFORM_STANDBY,
                                     ORE_STORE_GATE_CLOSED, true)) {
          AutoOre_FailInvalidParam(ctrl);
          return;
        }
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    default:
      AutoOre_RunReleaseCommonTrackGate(ctrl, now_ms);
      return;
  }
}

static void AutoOre_RunRelease1(AutoOre_t *ctrl, uint32_t now_ms) {
  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandReleasePole(ctrl) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_STANDBY,
                                   ORE_STORE_TRANSFORM_LIFT,
                                   ORE_STORE_GATE_OPEN, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.pole_all_at_target &&
          ctrl->feedback.ore_store_all_at_target) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_STANDBY,
                                   ORE_STORE_TRANSFORM_STANDBY,
                                   ORE_STORE_GATE_CLOSED, true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.ore_store_all_at_target &&
          AutoOre_StepElapsed(ctrl, now_ms) >=
              (AUTO_ORE_DELAY_RELEASE_GATE_MS +
               AUTO_ORE_DELAY_RELEASE_TRANSFORM_MS)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_RELEASE,
                                   ORE_STORE_TRANSFORM_STANDBY,
                                   ORE_STORE_GATE_CLOSED, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.ore_store_all_at_target &&
          AutoOre_StepElapsed(ctrl, now_ms) >=
              AUTO_ORE_DELAY_RELEASE_TRACK_MS) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 3:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_RELEASE,
                                   ORE_STORE_TRANSFORM_STANDBY,
                                   ORE_STORE_GATE_OPEN, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.ore_store_all_at_target &&
          AutoOre_StepElapsed(ctrl, now_ms) >= AUTO_ORE_DELAY_RELEASE_GATE_MS) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 4:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRACK_STANDBY,
                                   ORE_STORE_TRANSFORM_STANDBY,
                                   ORE_STORE_GATE_OPEN, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.ore_store_all_at_target) {
        AutoOre_FinishSuccess(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    default:
      AutoOre_FinishSuccess(ctrl);
      return;
  }
}

void AutoOre_Init(AutoOre_t *ctrl, const AutoOre_Params_t *param,
                  uint8_t held_ore_count) {
  if (ctrl == 0) {
    return;
  }
  memset(ctrl, 0, sizeof(*ctrl));
  if (param != 0) {
    ctrl->param = *param;
  }
  ctrl->held_ore_count = (held_ore_count > 3u) ? 3u : held_ore_count;
  ctrl->state = AUTO_ORE_STATE_IDLE;
  ctrl->result = AUTO_ORE_RESULT_NONE;
  ctrl->fault = AUTO_ORE_FAULT_NONE;
}

void AutoOre_SetHeldOreCount(AutoOre_t *ctrl, uint8_t held_ore_count) {
  if (ctrl != 0 && !AutoOre_IsBusy(ctrl)) {
    ctrl->held_ore_count = (held_ore_count > 3u) ? 3u : held_ore_count;
  }
}

uint8_t AutoOre_GetHeldOreCount(const AutoOre_t *ctrl) {
  return (ctrl == 0) ? 0u : ctrl->held_ore_count;
}

static bool AutoOre_Start(AutoOre_t *ctrl, AutoOre_Action_t action,
                          uint32_t now_ms) {
  if (ctrl == 0 || AutoOre_IsBusy(ctrl)) {
    return false;
  }
  if (ctrl->held_ore_count > 3u ||
      (action == AUTO_ORE_ACTION_STORE && ctrl->held_ore_count >= 3u) ||
      (action == AUTO_ORE_ACTION_RELEASE && ctrl->held_ore_count == 0u)) {
    ctrl->state = AUTO_ORE_STATE_FAIL;
    ctrl->result = AUTO_ORE_RESULT_FAIL;
    ctrl->fault = AUTO_ORE_FAULT_INVALID_COUNT;
    return false;
  }
  ctrl->state = AUTO_ORE_STATE_RUNNING;
  ctrl->result = AUTO_ORE_RESULT_RUNNING;
  ctrl->fault = AUTO_ORE_FAULT_NONE;
  ctrl->action = action;
  ctrl->step_index = 0u;
  ctrl->step_entered = false;
  ctrl->step_enter_time_ms = now_ms;
  AutoOre_ClearOutputs(ctrl);
  return true;
}

bool AutoOre_StartStore(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_STORE, now_ms);
}

bool AutoOre_StartRelease(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_RELEASE, now_ms);
}

void AutoOre_Update(AutoOre_t *ctrl, const AutoOre_Feedback_t *feedback,
                    uint32_t now_ms) {
  if (ctrl == 0) {
    return;
  }
  if (feedback != 0) {
    ctrl->feedback = *feedback;
  }
  if (ctrl->state != AUTO_ORE_STATE_RUNNING) {
    return;
  }
  if (!ctrl->feedback.ore_store_all_homed) {
    ctrl->state = AUTO_ORE_STATE_FAIL;
    ctrl->result = AUTO_ORE_RESULT_FAIL;
    ctrl->fault = AUTO_ORE_FAULT_NOT_HOMED;
    return;
  }

  AutoOre_ClearOutputs(ctrl);
  if (ctrl->action == AUTO_ORE_ACTION_STORE) {
    if (ctrl->held_ore_count == 0u) {
      AutoOre_RunStore0(ctrl, now_ms);
    } else if (ctrl->held_ore_count == 1u) {
      AutoOre_RunStore1(ctrl, now_ms);
    } else if (ctrl->held_ore_count == 2u) {
      AutoOre_RunStore2(ctrl, now_ms);
    } else {
      ctrl->state = AUTO_ORE_STATE_FAIL;
      ctrl->result = AUTO_ORE_RESULT_FAIL;
      ctrl->fault = AUTO_ORE_FAULT_INVALID_COUNT;
    }
  } else if (ctrl->action == AUTO_ORE_ACTION_RELEASE) {
    if (ctrl->held_ore_count == 3u) {
      AutoOre_RunRelease3(ctrl, now_ms);
    } else if (ctrl->held_ore_count == 2u) {
      AutoOre_RunRelease2(ctrl, now_ms);
    } else if (ctrl->held_ore_count == 1u) {
      AutoOre_RunRelease1(ctrl, now_ms);
    } else {
      ctrl->state = AUTO_ORE_STATE_FAIL;
      ctrl->result = AUTO_ORE_RESULT_FAIL;
      ctrl->fault = AUTO_ORE_FAULT_INVALID_COUNT;
    }
  }
}

void AutoOre_Abort(AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return;
  }
  ctrl->state = AUTO_ORE_STATE_ABORT;
  ctrl->result = AUTO_ORE_RESULT_ABORTED;
  ctrl->fault = AUTO_ORE_FAULT_ABORTED;
  ctrl->action = AUTO_ORE_ACTION_NONE;
  AutoOre_ClearOutputs(ctrl);
}

bool AutoOre_IsBusy(const AutoOre_t *ctrl) {
  return ctrl != 0 && ctrl->state == AUTO_ORE_STATE_RUNNING;
}

AutoOre_State_t AutoOre_GetState(const AutoOre_t *ctrl) {
  return (ctrl == 0) ? AUTO_ORE_STATE_IDLE : ctrl->state;
}

AutoOre_Result_t AutoOre_GetResult(const AutoOre_t *ctrl) {
  return (ctrl == 0) ? AUTO_ORE_RESULT_NONE : ctrl->result;
}

AutoOre_Fault_t AutoOre_GetFault(const AutoOre_t *ctrl) {
  return (ctrl == 0) ? AUTO_ORE_FAULT_INVALID_PARAM : ctrl->fault;
}

uint8_t AutoOre_GetStepIndex(const AutoOre_t *ctrl) {
  return (ctrl == 0) ? 0u : ctrl->step_index;
}

const ArmSimple_CMD_t *AutoOre_GetArmCommand(const AutoOre_t *ctrl) {
  return (ctrl != 0 && ctrl->arm_cmd_valid) ? &ctrl->arm_cmd : 0;
}

const OreStore_CMD_t *AutoOre_GetOreStoreCommand(const AutoOre_t *ctrl) {
  return (ctrl != 0 && ctrl->ore_store_cmd_valid) ? &ctrl->ore_store_cmd : 0;
}

const Pole_CMD_t *AutoOre_GetPoleCommand(const AutoOre_t *ctrl) {
  return (ctrl != 0 && ctrl->pole_cmd_valid) ? &ctrl->pole_cmd : 0;
}
