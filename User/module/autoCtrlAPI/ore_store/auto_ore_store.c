#include "module/autoCtrlAPI/ore_store/auto_ore_store.h"

#include <string.h>

#define AUTO_ORE_DELAY_STORE_ARM_SETTLE_MS (100u)
#define AUTO_ORE_DELAY_STORE_CYLINDER_CLOSE_MS (50u)
#define AUTO_ORE_DELAY_STORE_CYLINDER_OPEN_MS (50u)
#define AUTO_ORE_DELAY_RELEASE_WAIT_MS (50u)
#define AUTO_ORE_DELAY_RELEASE_SUCTION_OFF_MS (100u)
#define AUTO_ORE_DELAY_CHAMBER_LOW_CLAMP_MS (50u)
#define AUTO_ORE_DELAY_CHAMBER_ARM_SETTLE_MS (100u)
#define AUTO_ORE_DELAY_CHAMBER_CYLINDER_OPEN_MS (50u)
#define AUTO_ORE_DEFAULT_STEP_TIMEOUT_MS (5000u)

#ifndef AUTO_ORE_LOW_OCCUPANCY_SOURCE
#define AUTO_ORE_LOW_OCCUPANCY_SOURCE AUTO_ORE_OCCUPANCY_SOURCE_STATE_MACHINE
#endif

#ifndef AUTO_ORE_HIGH_OCCUPANCY_SOURCE
#define AUTO_ORE_HIGH_OCCUPANCY_SOURCE AUTO_ORE_OCCUPANCY_SOURCE_STATE_MACHINE
#endif

#ifndef AUTO_ORE_ARM_OCCUPANCY_SOURCE
#define AUTO_ORE_ARM_OCCUPANCY_SOURCE AUTO_ORE_OCCUPANCY_SOURCE_STATE_MACHINE
#endif

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
  ctrl->step_phase = 0u;
  ctrl->step_entered = false;
  ctrl->step_condition_met = false;
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

static bool AutoOre_WaitConditionThenDelay(AutoOre_t *ctrl, uint32_t now_ms,
                                           bool condition,
                                           uint32_t delay_ms) {
  if (!condition) {
    ctrl->step_condition_met = false;
    return false;
  }
  if (!ctrl->step_condition_met) {
    ctrl->step_condition_met = true;
    ctrl->step_condition_time_ms = now_ms;
  }
  return (now_ms - ctrl->step_condition_time_ms) >= delay_ms;
}

static void AutoOre_SetStepPhase(AutoOre_t *ctrl, uint8_t phase,
                                 uint32_t now_ms) {
  ctrl->step_phase = phase;
  ctrl->step_condition_met = false;
  ctrl->step_condition_time_ms = now_ms;
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

static bool AutoOre_CommandOreStore(AutoOre_t *ctrl,
                                    OreStore_TransformPoint_t transform,
                                    bool cylinder_closed) {
  ctrl->ore_store_cmd_valid = OreStore_MakePresetCommand(
      ctrl->param.ore_store_param, transform, cylinder_closed,
      &ctrl->ore_store_cmd);
  return ctrl->ore_store_cmd_valid;
}

static void AutoOre_FailInvalidParam(AutoOre_t *ctrl) {
  ctrl->state = AUTO_ORE_STATE_FAIL;
  ctrl->result = AUTO_ORE_RESULT_FAIL;
  ctrl->fault = AUTO_ORE_FAULT_INVALID_PARAM;
}

static bool AutoOre_IsPositionValid(AutoOre_Position_t position) {
  return position == AUTO_ORE_POSITION_TRANSFORM_LOW ||
         position == AUTO_ORE_POSITION_TRANSFORM_HIGH ||
         position == AUTO_ORE_POSITION_ARM;
}

static bool AutoOre_OccupancyIsFull(const AutoOre_Occupancy_t *occupancy) {
  return occupancy != 0 && occupancy->transform_low_has_ore &&
         occupancy->transform_high_has_ore && occupancy->arm_has_ore;
}

static void AutoOre_ApplyFeedbackOccupancy(AutoOre_t *ctrl) {
#if AUTO_ORE_LOW_OCCUPANCY_SOURCE == AUTO_ORE_OCCUPANCY_SOURCE_PHOTOELECTRIC
  ctrl->occupancy.transform_low_has_ore =
      ctrl->feedback.photoelectric_occupancy.transform_low_has_ore;
#endif
#if AUTO_ORE_HIGH_OCCUPANCY_SOURCE == AUTO_ORE_OCCUPANCY_SOURCE_PHOTOELECTRIC
  ctrl->occupancy.transform_high_has_ore =
      ctrl->feedback.photoelectric_occupancy.transform_high_has_ore;
#endif
#if AUTO_ORE_ARM_OCCUPANCY_SOURCE == AUTO_ORE_OCCUPANCY_SOURCE_PHOTOELECTRIC
  ctrl->occupancy.arm_has_ore = ctrl->feedback.photoelectric_occupancy.arm_has_ore;
#endif
}

static AutoOre_Position_t AutoOre_SelectStorePosition(
    const AutoOre_Occupancy_t *occupancy) {
  if (occupancy == 0) {
    return AUTO_ORE_POSITION_NONE;
  }
  if (!occupancy->transform_low_has_ore) {
    return AUTO_ORE_POSITION_TRANSFORM_LOW;
  }
  if (!occupancy->transform_high_has_ore) {
    return AUTO_ORE_POSITION_TRANSFORM_HIGH;
  }
  if (!occupancy->arm_has_ore) {
    return AUTO_ORE_POSITION_ARM;
  }
  return AUTO_ORE_POSITION_NONE;
}

static AutoOre_Position_t AutoOre_SelectChamberPosition(
    const AutoOre_Occupancy_t *occupancy) {
  if (occupancy == 0 || occupancy->arm_has_ore) {
    return AUTO_ORE_POSITION_NONE;
  }
  if (occupancy->transform_high_has_ore) {
    return AUTO_ORE_POSITION_TRANSFORM_HIGH;
  }
  if (occupancy->transform_low_has_ore) {
    return AUTO_ORE_POSITION_TRANSFORM_LOW;
  }
  return AUTO_ORE_POSITION_NONE;
}

static void AutoOre_SetActivePositionHasOre(AutoOre_t *ctrl, bool has_ore) {
#if AUTO_ORE_LOW_OCCUPANCY_SOURCE == AUTO_ORE_OCCUPANCY_SOURCE_STATE_MACHINE
  if (ctrl->active_position == AUTO_ORE_POSITION_TRANSFORM_LOW) {
    ctrl->occupancy.transform_low_has_ore = has_ore;
  }
#endif
#if AUTO_ORE_HIGH_OCCUPANCY_SOURCE == AUTO_ORE_OCCUPANCY_SOURCE_STATE_MACHINE
  if (ctrl->active_position == AUTO_ORE_POSITION_TRANSFORM_HIGH) {
    ctrl->occupancy.transform_high_has_ore = has_ore;
  }
#endif
#if AUTO_ORE_ARM_OCCUPANCY_SOURCE == AUTO_ORE_OCCUPANCY_SOURCE_STATE_MACHINE
  if (ctrl->active_position == AUTO_ORE_POSITION_ARM) {
    ctrl->occupancy.arm_has_ore = has_ore;
  }
#endif
}

static void AutoOre_SetArmHasOre(AutoOre_t *ctrl, bool has_ore) {
#if AUTO_ORE_ARM_OCCUPANCY_SOURCE == AUTO_ORE_OCCUPANCY_SOURCE_STATE_MACHINE
  ctrl->occupancy.arm_has_ore = has_ore;
#else
  (void)ctrl;
  (void)has_ore;
#endif
}

static void AutoOre_FinishChamberSuccess(AutoOre_t *ctrl) {
  AutoOre_SetActivePositionHasOre(ctrl, false);
  AutoOre_SetArmHasOre(ctrl, true);
}

static void AutoOre_FinishSuccess(AutoOre_t *ctrl) {
  const AutoOre_Fault_t fault_before_finish = ctrl->fault;
  if (ctrl->action == AUTO_ORE_ACTION_STORE) {
    AutoOre_SetActivePositionHasOre(ctrl, true);
  } else if (ctrl->action == AUTO_ORE_ACTION_RELEASE) {
    AutoOre_SetActivePositionHasOre(ctrl, false);
  } else if (ctrl->action == AUTO_ORE_ACTION_CHAMBER) {
    AutoOre_FinishChamberSuccess(ctrl);
  }
  ctrl->state = AUTO_ORE_STATE_SUCCESS;
  ctrl->result = AUTO_ORE_RESULT_SUCCESS;
  ctrl->fault = (fault_before_finish == AUTO_ORE_FAULT_INVALID_OCCUPANCY)
                    ? fault_before_finish
                    : AUTO_ORE_FAULT_NONE;
  ctrl->action = AUTO_ORE_ACTION_NONE;
  ctrl->active_position = AUTO_ORE_POSITION_NONE;
}

static bool AutoOre_WaitAll(AutoOre_t *ctrl, uint32_t now_ms) {
  if (ctrl->feedback.arm_at_target && ctrl->feedback.ore_store_all_at_target) {
    AutoOre_NextStep(ctrl);
    return true;
  }
  return AutoOre_CheckTimeout(ctrl, now_ms);
}

static void AutoOre_RunStoreLow(AutoOre_t *ctrl, uint32_t now_ms) {
  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_WAIT_STORE_ORE,
                              SUCTION_ON) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      (void)AutoOre_WaitAll(ctrl, now_ms);
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
                              SUCTION_OFF) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT,
                                   ctrl->step_phase != 0u)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->step_phase == 0u) {
        if (AutoOre_WaitConditionThenDelay(
                ctrl, now_ms, ctrl->feedback.arm_at_target,
                AUTO_ORE_DELAY_STORE_ARM_SETTLE_MS)) {
          AutoOre_SetStepPhase(ctrl, 1u, now_ms);
        } else {
          (void)AutoOre_CheckTimeout(ctrl, now_ms);
        }
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AUTO_ORE_DELAY_STORE_CYLINDER_CLOSE_MS)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_StepElapsed(ctrl, now_ms) >=
          AUTO_ORE_DELAY_STORE_CYLINDER_OPEN_MS) {
        AutoOre_NextStep(ctrl);
      }
      return;
    case 3:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, false)) {
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

static void AutoOre_RunStoreHigh(AutoOre_t *ctrl, uint32_t now_ms) {
  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_WAIT_STORE_ORE,
                              SUCTION_ON) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_MID_WAIT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      (void)AutoOre_WaitAll(ctrl, now_ms);
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
                              SUCTION_OFF) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_MID_WAIT,
                                   ctrl->step_phase != 0u)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->step_phase == 0u) {
        if (AutoOre_WaitConditionThenDelay(
                ctrl, now_ms, ctrl->feedback.arm_at_target,
                AUTO_ORE_DELAY_STORE_ARM_SETTLE_MS)) {
          AutoOre_SetStepPhase(ctrl, 1u, now_ms);
        } else {
          (void)AutoOre_CheckTimeout(ctrl, now_ms);
        }
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AUTO_ORE_DELAY_STORE_CYLINDER_CLOSE_MS)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_MID_WAIT, true)) {
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

static void AutoOre_RunStoreArm(AutoOre_t *ctrl, uint32_t now_ms) {
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

static bool AutoOre_CommandReleaseOreStoreHold(
    AutoOre_t *ctrl, OreStore_TransformPoint_t transform,
    bool cylinder_closed) {
  return AutoOre_CommandOreStore(ctrl, transform, cylinder_closed);
}

static void AutoOre_RunReleaseArm(AutoOre_t *ctrl, uint32_t now_ms) {
  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_WAIT_RELEASE_ORE,
                              SUCTION_ON) ||
          !AutoOre_CommandReleaseOreStoreHold(ctrl,
                                              ORE_STORE_TRANSFORM_STANDBY,
                                              true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(
              ctrl, now_ms,
              ctrl->feedback.arm_at_target &&
                  ctrl->feedback.ore_store_all_at_target,
              AUTO_ORE_DELAY_RELEASE_WAIT_MS)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_RELEASE_ORE,
                              SUCTION_OFF) ||
          !AutoOre_CommandReleaseOreStoreHold(ctrl,
                                              ORE_STORE_TRANSFORM_STANDBY,
                                              true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms,
                                         ctrl->feedback.arm_at_target,
                                         AUTO_ORE_DELAY_RELEASE_SUCTION_OFF_MS)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY,
                              SUCTION_OFF) ||
          !AutoOre_CommandReleaseOreStoreHold(ctrl,
                                              ORE_STORE_TRANSFORM_STANDBY,
                                              true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.arm_at_target) {
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

static void AutoOre_RunChamberHigh(AutoOre_t *ctrl, uint32_t now_ms) {
  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_WAIT_STORE_ORE,
                              SUCTION_ON) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      (void)AutoOre_WaitAll(ctrl, now_ms);
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
                              SUCTION_ON) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY,
                                   ctrl->step_phase == 0u)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->step_phase == 0u) {
        if (AutoOre_WaitConditionThenDelay(
                ctrl, now_ms, ctrl->feedback.arm_at_target,
                AUTO_ORE_DELAY_CHAMBER_ARM_SETTLE_MS)) {
          AutoOre_SetStepPhase(ctrl, 1u, now_ms);
        } else {
          (void)AutoOre_CheckTimeout(ctrl, now_ms);
        }
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AUTO_ORE_DELAY_CHAMBER_CYLINDER_OPEN_MS)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY,
                              SUCTION_ON) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.arm_at_target) {
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

static void AutoOre_RunChamberLow(AutoOre_t *ctrl, uint32_t now_ms) {
  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_WAIT_STORE_ORE,
                              SUCTION_ON) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT,
                                   ctrl->step_phase != 0u)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->step_phase == 0u) {
        if (AutoOre_WaitConditionThenDelay(
                ctrl, now_ms,
                ctrl->feedback.arm_at_target &&
                    ctrl->feedback.ore_store_all_at_target,
                AUTO_ORE_DELAY_CHAMBER_LOW_CLAMP_MS)) {
          AutoOre_SetStepPhase(ctrl, 1u, now_ms);
        } else {
          (void)AutoOre_CheckTimeout(ctrl, now_ms);
        }
        return;
      }
      AutoOre_NextStep(ctrl);
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
                              SUCTION_ON) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT,
                                   ctrl->step_phase == 0u)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->step_phase == 0u) {
        if (AutoOre_WaitConditionThenDelay(
                ctrl, now_ms, ctrl->feedback.arm_at_target,
                AUTO_ORE_DELAY_CHAMBER_ARM_SETTLE_MS)) {
          AutoOre_SetStepPhase(ctrl, 1u, now_ms);
        } else {
          (void)AutoOre_CheckTimeout(ctrl, now_ms);
        }
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AUTO_ORE_DELAY_CHAMBER_CYLINDER_OPEN_MS)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY,
                              SUCTION_ON) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.arm_at_target) {
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
                  const AutoOre_Occupancy_t *initial_occupancy) {
  if (ctrl == 0) {
    return;
  }
  memset(ctrl, 0, sizeof(*ctrl));
  if (param != 0) {
    ctrl->param = *param;
  }
  if (initial_occupancy != 0) {
    ctrl->occupancy = *initial_occupancy;
  }
  ctrl->state = AUTO_ORE_STATE_IDLE;
  ctrl->result = AUTO_ORE_RESULT_NONE;
  ctrl->fault = AUTO_ORE_FAULT_NONE;
  ctrl->active_position = AUTO_ORE_POSITION_NONE;
}

AutoOre_Occupancy_t AutoOre_GetOccupancy(const AutoOre_t *ctrl) {
  AutoOre_Occupancy_t empty = {0};
  return (ctrl == 0) ? empty : ctrl->occupancy;
}

uint8_t AutoOre_GetOccupancyMask(const AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return 0u;
  }
  return (uint8_t)((ctrl->occupancy.transform_low_has_ore ? 0x01u : 0u) |
                   (ctrl->occupancy.transform_high_has_ore ? 0x02u : 0u) |
                   (ctrl->occupancy.arm_has_ore ? 0x04u : 0u));
}

static bool AutoOre_Start(AutoOre_t *ctrl, AutoOre_Action_t action,
                          uint32_t now_ms) {
  bool occupancy_suspect = false;
  if (ctrl == 0 || AutoOre_IsBusy(ctrl)) {
    return false;
  }
  AutoOre_ApplyFeedbackOccupancy(ctrl);
  AutoOre_Position_t position = AUTO_ORE_POSITION_NONE;
  if (action == AUTO_ORE_ACTION_STORE) {
    if (AutoOre_OccupancyIsFull(&ctrl->occupancy)) {
      ctrl->fault = AUTO_ORE_FAULT_INVALID_OCCUPANCY;
      occupancy_suspect = true;
    }
    position = AutoOre_SelectStorePosition(&ctrl->occupancy);
    if (!AutoOre_IsPositionValid(position)) {
      position = AUTO_ORE_POSITION_TRANSFORM_LOW;
    }
  } else if (action == AUTO_ORE_ACTION_RELEASE) {
    if (!ctrl->occupancy.arm_has_ore) {
      ctrl->fault = AUTO_ORE_FAULT_INVALID_OCCUPANCY;
      occupancy_suspect = true;
    }
    position = AUTO_ORE_POSITION_ARM;
  } else if (action == AUTO_ORE_ACTION_CHAMBER) {
    if (ctrl->occupancy.arm_has_ore ||
        (!ctrl->occupancy.transform_high_has_ore &&
         !ctrl->occupancy.transform_low_has_ore)) {
      ctrl->fault = AUTO_ORE_FAULT_INVALID_OCCUPANCY;
      occupancy_suspect = true;
    }
    position = AutoOre_SelectChamberPosition(&ctrl->occupancy);
    if (!AutoOre_IsPositionValid(position)) {
      position = AUTO_ORE_POSITION_TRANSFORM_HIGH;
    }
  }
  if (!AutoOre_IsPositionValid(position)) {
    ctrl->state = AUTO_ORE_STATE_FAIL;
    ctrl->result = AUTO_ORE_RESULT_FAIL;
    ctrl->fault = AUTO_ORE_FAULT_INVALID_OCCUPANCY;
    return false;
  }
  ctrl->state = AUTO_ORE_STATE_RUNNING;
  ctrl->result = AUTO_ORE_RESULT_RUNNING;
  if (!occupancy_suspect) {
    ctrl->fault = AUTO_ORE_FAULT_NONE;
  }
  ctrl->action = action;
  ctrl->active_position = position;
  ctrl->step_index = 0u;
  ctrl->step_phase = 0u;
  ctrl->step_entered = false;
  ctrl->step_condition_met = false;
  ctrl->step_condition_time_ms = now_ms;
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

bool AutoOre_StartChamber(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_CHAMBER, now_ms);
}

void AutoOre_Update(AutoOre_t *ctrl, const AutoOre_Feedback_t *feedback,
                    uint32_t now_ms) {
  if (ctrl == 0) {
    return;
  }
  if (feedback != 0) {
    ctrl->feedback = *feedback;
  }
  AutoOre_ApplyFeedbackOccupancy(ctrl);
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
    switch (ctrl->active_position) {
      case AUTO_ORE_POSITION_TRANSFORM_LOW:
        AutoOre_RunStoreLow(ctrl, now_ms);
        break;
      case AUTO_ORE_POSITION_TRANSFORM_HIGH:
        AutoOre_RunStoreHigh(ctrl, now_ms);
        break;
      case AUTO_ORE_POSITION_ARM:
        AutoOre_RunStoreArm(ctrl, now_ms);
        break;
      default:
        ctrl->state = AUTO_ORE_STATE_FAIL;
        ctrl->result = AUTO_ORE_RESULT_FAIL;
        ctrl->fault = AUTO_ORE_FAULT_INVALID_OCCUPANCY;
        break;
    }
  } else if (ctrl->action == AUTO_ORE_ACTION_RELEASE) {
    if (ctrl->active_position == AUTO_ORE_POSITION_ARM) {
      AutoOre_RunReleaseArm(ctrl, now_ms);
    } else {
      ctrl->state = AUTO_ORE_STATE_FAIL;
      ctrl->result = AUTO_ORE_RESULT_FAIL;
      ctrl->fault = AUTO_ORE_FAULT_INVALID_OCCUPANCY;
    }
  } else if (ctrl->action == AUTO_ORE_ACTION_CHAMBER) {
    switch (ctrl->active_position) {
      case AUTO_ORE_POSITION_TRANSFORM_HIGH:
        AutoOre_RunChamberHigh(ctrl, now_ms);
        break;
      case AUTO_ORE_POSITION_TRANSFORM_LOW:
        AutoOre_RunChamberLow(ctrl, now_ms);
        break;
      default:
        ctrl->state = AUTO_ORE_STATE_FAIL;
        ctrl->result = AUTO_ORE_RESULT_FAIL;
        ctrl->fault = AUTO_ORE_FAULT_INVALID_OCCUPANCY;
        break;
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
  ctrl->active_position = AUTO_ORE_POSITION_NONE;
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

AutoOre_Position_t AutoOre_GetActivePosition(const AutoOre_t *ctrl) {
  return (ctrl == 0) ? AUTO_ORE_POSITION_NONE : ctrl->active_position;
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
