#include "module/autoCtrlAPI/ore_store/auto_ore_store.h"

#include <string.h>

#define AUTO_ORE_DEFAULT_STORE_ARM_SETTLE_MS (100u)
#define AUTO_ORE_DEFAULT_STORE_CYLINDER_CLOSE_MS (50u)
#define AUTO_ORE_DEFAULT_STORE_CYLINDER_OPEN_MS (50u)
#define AUTO_ORE_DEFAULT_RELEASE_WAIT_MS (50u)
#define AUTO_ORE_DEFAULT_RELEASE_ARM_SETTLE_MS (10u)
#define AUTO_ORE_DEFAULT_RELEASE_SUCTION_OFF_MS (100u)
#define AUTO_ORE_DEFAULT_CHAMBER_LOW_CLAMP_MS (50u)
#define AUTO_ORE_DEFAULT_CHAMBER_ARM_SETTLE_MS (100u)
#define AUTO_ORE_DEFAULT_CHAMBER_CYLINDER_OPEN_MS (50u)
#define AUTO_ORE_DEFAULT_FETCH_CHASSIS_MOVE_MS (300u)
#define AUTO_ORE_DEFAULT_STEP_TIMEOUT_MS (5000u)
#define AUTO_ORE_DEFAULT_FETCH_CHASSIS_VX_MPS (0.20f)
#define AUTO_ORE_DEFAULT_FUSED_PREALIGN_STABLE_MS (100u)
#define AUTO_ORE_DEFAULT_FUSED_PICK_PRECONTACT_TIMEOUT_MS (2000u)
#define AUTO_ORE_DEFAULT_FUSED_PICK_LIFT_DETECT_MS (200u)
#define AUTO_ORE_DEFAULT_FUSED_ARM_PHOTO_STABLE_MS (120u)
#define AUTO_ORE_DEFAULT_WHEEL_RADIUS_M (0.076f)

#define AUTO_ORE_OCCUPANCY_SOURCE_STATE_MACHINE_VALUE (0u)
#define AUTO_ORE_OCCUPANCY_SOURCE_PHOTOELECTRIC_VALUE (1u)

#if !defined(AUTO_ORE_LOW_OCCUPANCY_SOURCE) || \
  AUTO_ORE_LOW_OCCUPANCY_SOURCE != AUTO_ORE_OCCUPANCY_SOURCE_PHOTOELECTRIC_VALUE
#undef AUTO_ORE_LOW_OCCUPANCY_SOURCE
#define AUTO_ORE_LOW_OCCUPANCY_SOURCE AUTO_ORE_OCCUPANCY_SOURCE_PHOTOELECTRIC_VALUE
#endif

#if !defined(AUTO_ORE_HIGH_OCCUPANCY_SOURCE) || \
  AUTO_ORE_HIGH_OCCUPANCY_SOURCE != AUTO_ORE_OCCUPANCY_SOURCE_PHOTOELECTRIC_VALUE
#undef AUTO_ORE_HIGH_OCCUPANCY_SOURCE
#define AUTO_ORE_HIGH_OCCUPANCY_SOURCE AUTO_ORE_OCCUPANCY_SOURCE_PHOTOELECTRIC_VALUE
#endif

#if !defined(AUTO_ORE_ARM_OCCUPANCY_SOURCE) || \
  AUTO_ORE_ARM_OCCUPANCY_SOURCE != AUTO_ORE_OCCUPANCY_SOURCE_PHOTOELECTRIC_VALUE
#undef AUTO_ORE_ARM_OCCUPANCY_SOURCE
#define AUTO_ORE_ARM_OCCUPANCY_SOURCE AUTO_ORE_OCCUPANCY_SOURCE_STATE_MACHINE_VALUE
#endif

static uint32_t AutoOre_StepElapsed(const AutoOre_t *ctrl, uint32_t now_ms) {
  return now_ms - ctrl->step_enter_time_ms;
}

static float AutoOre_AbsFloat(float value) {
  return (value >= 0.0f) ? value : -value;
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
  ctrl->distance_latch_valid = false;
  ctrl->distance_travel_m = 0.0f;
  ctrl->distance_target_m = 0.0f;
}

static uint32_t AutoOre_StepTimeoutMs(const AutoOre_t *ctrl) {
  return (ctrl->param.default_step_timeout_ms > 0u)
             ? ctrl->param.default_step_timeout_ms
             : AUTO_ORE_DEFAULT_STEP_TIMEOUT_MS;
}

static uint32_t AutoOre_TimingValue(uint32_t configured_ms,
                                    uint32_t default_ms) {
  return (configured_ms > 0u) ? configured_ms : default_ms;
}

static uint32_t AutoOre_StoreArmSettleMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.store_arm_settle_ms,
                             AUTO_ORE_DEFAULT_STORE_ARM_SETTLE_MS);
}

static uint32_t AutoOre_StoreCylinderCloseMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.store_cylinder_close_ms,
                             AUTO_ORE_DEFAULT_STORE_CYLINDER_CLOSE_MS);
}

static uint32_t AutoOre_StoreCylinderOpenMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.store_cylinder_open_ms,
                             AUTO_ORE_DEFAULT_STORE_CYLINDER_OPEN_MS);
}

static uint32_t AutoOre_ReleaseWaitMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.release_wait_ms,
                             AUTO_ORE_DEFAULT_RELEASE_WAIT_MS);
}

static uint32_t AutoOre_ReleaseArmSettleMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.release_arm_settle_ms,
                             AUTO_ORE_DEFAULT_RELEASE_ARM_SETTLE_MS);
}

static uint32_t AutoOre_ReleaseSuctionOffMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.release_suction_off_ms,
                             AUTO_ORE_DEFAULT_RELEASE_SUCTION_OFF_MS);
}

static uint32_t AutoOre_ChamberLowClampMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.chamber_low_clamp_ms,
                             AUTO_ORE_DEFAULT_CHAMBER_LOW_CLAMP_MS);
}

static uint32_t AutoOre_ChamberArmSettleMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.chamber_arm_settle_ms,
                             AUTO_ORE_DEFAULT_CHAMBER_ARM_SETTLE_MS);
}

static uint32_t AutoOre_ChamberCylinderOpenMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.chamber_cylinder_open_ms,
                             AUTO_ORE_DEFAULT_CHAMBER_CYLINDER_OPEN_MS);
}

static uint32_t AutoOre_FetchChassisMoveMs(const AutoOre_t *ctrl) {
  if (ctrl->action == AUTO_ORE_ACTION_PICK_NEG_200) {
    return ctrl->param.timing.fetch_neg_200_chassis_move_ms;
  }
  return AutoOre_TimingValue(ctrl->param.timing.fetch_chassis_move_ms,
                             AUTO_ORE_DEFAULT_FETCH_CHASSIS_MOVE_MS);
}

static uint32_t AutoOre_FusedPrealignStableMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.fused_prealign_stable_ms,
                             AUTO_ORE_DEFAULT_FUSED_PREALIGN_STABLE_MS);
}

static uint32_t AutoOre_FusedPickPrecontactTimeoutMs(
    const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(
      ctrl->param.timing.fused_pick_precontact_timeout_ms,
      AUTO_ORE_DEFAULT_FUSED_PICK_PRECONTACT_TIMEOUT_MS);
}

static uint32_t AutoOre_FusedPickLiftDetectMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.fused_pick_lift_detect_ms,
                             AUTO_ORE_DEFAULT_FUSED_PICK_LIFT_DETECT_MS);
}

static uint32_t AutoOre_FusedArmPhotoStableMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.fused_arm_photo_stable_ms,
                             AUTO_ORE_DEFAULT_FUSED_ARM_PHOTO_STABLE_MS);
}

static float AutoOre_OreStoreArriveThresholdRad(const AutoOre_t *ctrl) {
  return (ctrl->param.ore_store_arrive_threshold_rad > 0.0f)
             ? ctrl->param.ore_store_arrive_threshold_rad
             : 0.05f;
}

static float AutoOre_ChamberLowPlatformThresholdRad(const AutoOre_t *ctrl) {
  const float base_threshold = AutoOre_OreStoreArriveThresholdRad(ctrl);
  return (base_threshold < 0.20f) ? 0.20f : base_threshold;
}

static float AutoOre_FetchChassisVxMps(const AutoOre_t *ctrl) {
  if (ctrl->action == AUTO_ORE_ACTION_PICK_NEG_200) {
    return ctrl->param.fetch_neg_200_chassis_vx_mps;
  }
  return (ctrl->param.fetch_chassis_vx_mps > 0.0f)
             ? ctrl->param.fetch_chassis_vx_mps
             : AUTO_ORE_DEFAULT_FETCH_CHASSIS_VX_MPS;
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

static bool AutoOre_WaitArmCommandTarget(AutoOre_t *ctrl, uint32_t now_ms) {
  if (!ctrl->step_condition_met) {
    ctrl->step_condition_met = true;
    ctrl->step_condition_time_ms = now_ms;
    return false;
  }
  if (now_ms == ctrl->step_condition_time_ms) {
    return false;
  }
  return ctrl->feedback.arm_at_target;
}

static bool AutoOre_WaitOreStoreCommandTarget(AutoOre_t *ctrl,
                                              uint32_t now_ms) {
  if (!ctrl->step_condition_met) {
    ctrl->step_condition_met = true;
    ctrl->step_condition_time_ms = now_ms;
    return false;
  }
  if (now_ms == ctrl->step_condition_time_ms) {
    return false;
  }
  return ctrl->feedback.ore_store_all_at_target;
}

static bool AutoOre_OreStorePlatformAtTarget(const AutoOre_t *ctrl,
                                             float threshold_rad) {
  return ctrl != 0 &&
         AutoOre_AbsFloat(ctrl->feedback.ore_store_platform_error_rad) <=
             threshold_rad;
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
  ctrl->chassis_cmd_valid = false;
}

static bool AutoOre_CommandArm(AutoOre_t *ctrl, ArmSimple_BehaviorPoint_t point,
                 Suction_State_t suction,
                 const AutoOre_ArmSpeedLimit_t *speed) {
  const float joint1_max_vel_rad_s =
    (speed != 0) ? speed->joint1_max_vel_rad_s : 0.0f;
  const float joint2_max_vel_rad_s =
    (speed != 0) ? speed->joint2_max_vel_rad_s : 0.0f;
  ctrl->arm_cmd_valid = ArmSimple_MakeBehaviorCommandWithSpeed(
    ctrl->param.arm_param, point, suction, joint1_max_vel_rad_s,
    joint2_max_vel_rad_s, &ctrl->arm_cmd);
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

static bool AutoOre_CommandPoleTarget(AutoOre_t *ctrl,
                                      const float target_lift[2]) {
  if (target_lift == 0) {
    ctrl->pole_cmd_valid = false;
    return false;
  }
  memset(&ctrl->pole_cmd, 0, sizeof(ctrl->pole_cmd));
  ctrl->pole_cmd.mode = POLE_MODE_ACTIVE;
  ctrl->pole_cmd.lift[0] = 0.0f;
  ctrl->pole_cmd.lift[1] = 0.0f;
  ctrl->pole_cmd.auto_target_enable[0] = true;
  ctrl->pole_cmd.auto_target_enable[1] = true;
  ctrl->pole_cmd.auto_target_lift[0] = target_lift[0];
  ctrl->pole_cmd.auto_target_lift[1] = target_lift[1];
  ctrl->pole_cmd.auto_lift_speed[0] = 0.0f;
  ctrl->pole_cmd.auto_lift_speed[1] = 0.0f;
  ctrl->pole_cmd.auto_lift_accel[0] = 0.0f;
  ctrl->pole_cmd.auto_lift_accel[1] = 0.0f;
  ctrl->pole_cmd.disable_lift_accel = false;
  ctrl->pole_cmd_valid = true;
  return true;
}

static void AutoOre_CommandChassisMove(AutoOre_t *ctrl, float vx_mps) {
  memset(&ctrl->chassis_cmd, 0, sizeof(ctrl->chassis_cmd));
  ctrl->chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
  ctrl->chassis_cmd.ctrl_vec.vx = vx_mps;
  ctrl->chassis_cmd.ctrl_vec.vy = 0.0f;
  ctrl->chassis_cmd.ctrl_vec.wz = 0.0f;
  ctrl->chassis_cmd_valid = true;
}

static void AutoOre_CommandChassisMoveYawRate(AutoOre_t *ctrl, float vx_mps,
                                              float wz_rad_s) {
  AutoOre_CommandChassisMove(ctrl, vx_mps);
  ctrl->chassis_cmd.ctrl_vec.wz = wz_rad_s;
}

static float AutoOre_WheelRadiusM(const AutoOre_t *ctrl) {
  if (ctrl != 0 && ctrl->param.wheel_radius_m > 0.0f) {
    return ctrl->param.wheel_radius_m;
  }
  return AUTO_ORE_DEFAULT_WHEEL_RADIUS_M;
}

static void AutoOre_CommandChassisHold(AutoOre_t *ctrl) {
  AutoOre_CommandChassisMove(ctrl, 0.0f);
}

static void AutoOre_ResetDistanceGate(AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return;
  }
  ctrl->distance_latch_valid = false;
  ctrl->distance_travel_m = 0.0f;
  ctrl->distance_target_m = 0.0f;
}

static bool AutoOre_DistanceMoveReached(AutoOre_t *ctrl,
                                        float target_distance_m) {
  if (ctrl == 0 || target_distance_m <= 0.0f) {
    return false;
  }
  ctrl->distance_target_m = target_distance_m;

  if (!ctrl->distance_latch_valid) {
    for (uint8_t i = 0u; i < 4u; ++i) {
      ctrl->distance_start_wheel_rad[i] = ctrl->feedback.wheel_position_rad[i];
    }
    ctrl->distance_travel_m = 0.0f;
    ctrl->distance_latch_valid = true;
    return false;
  }

  float wheel_delta_abs_sum_rad = 0.0f;
  for (uint8_t i = 0u; i < 4u; ++i) {
    wheel_delta_abs_sum_rad += AutoOre_AbsFloat(
        ctrl->feedback.wheel_position_rad[i] -
        ctrl->distance_start_wheel_rad[i]);
  }
  ctrl->distance_travel_m =
      (wheel_delta_abs_sum_rad * 0.25f) * AutoOre_WheelRadiusM(ctrl);
  return ctrl->distance_travel_m >= target_distance_m;
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
#if AUTO_ORE_LOW_OCCUPANCY_SOURCE == AUTO_ORE_OCCUPANCY_SOURCE_PHOTOELECTRIC_VALUE
  ctrl->occupancy.transform_low_has_ore =
      ctrl->feedback.photoelectric_occupancy.transform_low_has_ore;
#endif
#if AUTO_ORE_HIGH_OCCUPANCY_SOURCE == AUTO_ORE_OCCUPANCY_SOURCE_PHOTOELECTRIC_VALUE
  ctrl->occupancy.transform_high_has_ore =
      ctrl->feedback.photoelectric_occupancy.transform_high_has_ore;
#endif
#if AUTO_ORE_ARM_OCCUPANCY_SOURCE == AUTO_ORE_OCCUPANCY_SOURCE_PHOTOELECTRIC_VALUE
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
#if AUTO_ORE_LOW_OCCUPANCY_SOURCE == AUTO_ORE_OCCUPANCY_SOURCE_STATE_MACHINE_VALUE
  if (ctrl->active_position == AUTO_ORE_POSITION_TRANSFORM_LOW) {
    ctrl->occupancy.transform_low_has_ore = has_ore;
  }
#endif
#if AUTO_ORE_HIGH_OCCUPANCY_SOURCE == AUTO_ORE_OCCUPANCY_SOURCE_STATE_MACHINE_VALUE
  if (ctrl->active_position == AUTO_ORE_POSITION_TRANSFORM_HIGH) {
    ctrl->occupancy.transform_high_has_ore = has_ore;
  }
#endif
#if AUTO_ORE_ARM_OCCUPANCY_SOURCE == AUTO_ORE_OCCUPANCY_SOURCE_STATE_MACHINE_VALUE
  if (ctrl->active_position == AUTO_ORE_POSITION_ARM) {
    ctrl->occupancy.arm_has_ore = has_ore;
  }
#endif
}

static void AutoOre_SetArmHasOre(AutoOre_t *ctrl, bool has_ore) {
#if AUTO_ORE_ARM_OCCUPANCY_SOURCE == AUTO_ORE_OCCUPANCY_SOURCE_STATE_MACHINE_VALUE
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
  } else if (ctrl->action == AUTO_ORE_ACTION_PICK_POS_400 ||
             ctrl->action == AUTO_ORE_ACTION_PICK_POS_200 ||
             ctrl->action == AUTO_ORE_ACTION_PICK_NEG_200) {
    AutoOre_SetArmHasOre(ctrl, true);
  } else if (ctrl->action == AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD) {
    ctrl->step_ctrl_active = false;
    ctrl->step_ctrl_started = false;
  }
  ctrl->state = AUTO_ORE_STATE_SUCCESS;
  ctrl->result = AUTO_ORE_RESULT_SUCCESS;
  ctrl->fault = (fault_before_finish == AUTO_ORE_FAULT_INVALID_OCCUPANCY)
                    ? fault_before_finish
                    : AUTO_ORE_FAULT_NONE;
  ctrl->action = AUTO_ORE_ACTION_NONE;
  ctrl->active_position = AUTO_ORE_POSITION_NONE;
  ctrl->step_ctrl_active = false;
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
              SUCTION_ON, &ctrl->param.arm_speed.store_wait) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      (void)AutoOre_WaitAll(ctrl, now_ms);
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      const Suction_State_t suction =
        (ctrl->step_phase == 0u) ? SUCTION_ON : SUCTION_OFF;
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
          suction, &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT,
                                   ctrl->step_phase != 0u)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->step_phase == 0u) {
        if (AutoOre_WaitConditionThenDelay(
                ctrl, now_ms, ctrl->feedback.arm_at_target,
                AutoOre_StoreArmSettleMs(ctrl))) {
          AutoOre_SetStepPhase(ctrl, 1u, now_ms);
        } else {
          (void)AutoOre_CheckTimeout(ctrl, now_ms);
        }
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AutoOre_StoreCylinderCloseMs(ctrl))) {
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
          AutoOre_StoreCylinderOpenMs(ctrl)) {
        AutoOre_NextStep(ctrl);
      }
      return;
    case 3:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitOreStoreCommandTarget(ctrl, now_ms)) {
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
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
          SUCTION_ON, &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(
              ctrl, now_ms, ctrl->feedback.arm_at_target,
              AutoOre_StoreArmSettleMs(ctrl))) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
          SUCTION_OFF, &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AutoOre_StoreCylinderCloseMs(ctrl))) {
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

static void AutoOre_RunStoreArm(AutoOre_t *ctrl, uint32_t now_ms) {
  AutoOre_EnterStep(ctrl, now_ms);
  if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY, SUCTION_ON,
                          &ctrl->param.arm_speed.store_standby)) {
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
  AutoOre_CommandChassisHold(ctrl);

  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_WAIT_RELEASE_ORE,
              SUCTION_ON,
              &ctrl->param.arm_speed.release_wait) ||
          !AutoOre_CommandReleaseOreStoreHold(ctrl,
                                              ORE_STORE_TRANSFORM_STANDBY,
                                              true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(
              ctrl, now_ms, ctrl->feedback.arm_at_target,
              AutoOre_ReleaseWaitMs(ctrl))) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_RELEASE_ORE,
              SUCTION_ON,
              &ctrl->param.arm_speed.release_place) ||
          !AutoOre_CommandReleaseOreStoreHold(ctrl,
                                              ORE_STORE_TRANSFORM_STANDBY,
                                              true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms,
                                         ctrl->feedback.arm_at_target,
                                         AutoOre_ReleaseArmSettleMs(ctrl))) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_RELEASE_ORE,
              SUCTION_OFF,
              &ctrl->param.arm_speed.release_place) ||
          !AutoOre_CommandReleaseOreStoreHold(ctrl,
                                              ORE_STORE_TRANSFORM_STANDBY,
                                              true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AutoOre_ReleaseSuctionOffMs(ctrl))) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 3:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY,
              SUCTION_OFF,
              &ctrl->param.arm_speed.release_standby) ||
          !AutoOre_CommandReleaseOreStoreHold(ctrl,
                                              ORE_STORE_TRANSFORM_STANDBY,
                                              true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
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
              SUCTION_ON,
              &ctrl->param.arm_speed.chamber_wait) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      (void)AutoOre_WaitAll(ctrl, now_ms);
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_CHAMBER_ORE,
              SUCTION_ON,
              &ctrl->param.arm_speed.chamber_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY,
                                   ctrl->step_phase == 0u)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->step_phase == 0u) {
        if (AutoOre_WaitConditionThenDelay(
                ctrl, now_ms, ctrl->feedback.arm_at_target,
                AutoOre_ChamberArmSettleMs(ctrl))) {
          AutoOre_SetStepPhase(ctrl, 1u, now_ms);
        } else {
          (void)AutoOre_CheckTimeout(ctrl, now_ms);
        }
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AutoOre_ChamberCylinderOpenMs(ctrl))) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY,
              SUCTION_ON,
              &ctrl->param.arm_speed.chamber_standby) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
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
              SUCTION_ON,
              &ctrl->param.arm_speed.chamber_wait) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(
            ctrl, now_ms,
            AutoOre_OreStorePlatformAtTarget(
              ctrl, AutoOre_ChamberLowPlatformThresholdRad(ctrl)),
              AutoOre_ChamberLowClampMs(ctrl))) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_CHAMBER_ORE,
              SUCTION_ON,
              &ctrl->param.arm_speed.chamber_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(
              ctrl, now_ms, ctrl->feedback.arm_at_target,
              AutoOre_ChamberArmSettleMs(ctrl))) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_CHAMBER_ORE,
              SUCTION_ON,
              &ctrl->param.arm_speed.chamber_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AutoOre_ChamberCylinderOpenMs(ctrl))) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 3:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY,
              SUCTION_ON,
              &ctrl->param.arm_speed.chamber_standby) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
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

static bool AutoOre_ActionIsPick(AutoOre_Action_t action) {
  return action == AUTO_ORE_ACTION_PICK_POS_400 ||
         action == AUTO_ORE_ACTION_PICK_POS_200 ||
         action == AUTO_ORE_ACTION_PICK_NEG_200;
}

static bool AutoOre_ActionIsFused(AutoOre_Action_t action) {
  return action == AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD;
}

static const float *AutoOre_PickPoleTarget(const AutoOre_t *ctrl) {
  if (ctrl == 0 || ctrl->param.pole_param == 0) {
    return 0;
  }
  switch (ctrl->action) {
    case AUTO_ORE_ACTION_PICK_POS_400:
      return ctrl->param.pole_param->preset.step_400_all_extend;
    case AUTO_ORE_ACTION_PICK_POS_200:
      return ctrl->param.pole_param->preset.step_200_all_extend;
    case AUTO_ORE_ACTION_PICK_NEG_200:
      return ctrl->param.pole_param->preset.step_200_small;
    default:
      return 0;
  }
}

static ArmSimple_BehaviorPoint_t AutoOre_PickArmPoint(
    AutoOre_Action_t action) {
  switch (action) {
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
      return ARM_SIMPLE_BEHAVIOR_PICK_POS_200;
    case AUTO_ORE_ACTION_PICK_POS_400:
      return ARM_SIMPLE_BEHAVIOR_PICK_POS_400;
    case AUTO_ORE_ACTION_PICK_POS_200:
      return ARM_SIMPLE_BEHAVIOR_PICK_POS_200;
    case AUTO_ORE_ACTION_PICK_NEG_200:
      return ARM_SIMPLE_BEHAVIOR_PICK_NEG_200;
    default:
      return ARM_SIMPLE_BEHAVIOR_STANDBY;
  }
}

static const float *AutoOre_FusedPickPoleTarget(const AutoOre_t *ctrl) {
  if (ctrl == 0 || ctrl->param.pole_param == 0) {
    return 0;
  }
  AutoOre_Action_t pick_action =
      ctrl->param.fused_step_pick_store_ascend_200_head.pick_action;
  if (!AutoOre_ActionIsPick(pick_action)) {
    pick_action = AUTO_ORE_ACTION_PICK_POS_200;
  }
  AutoOre_t local = *ctrl;
  local.action = pick_action;
  return AutoOre_PickPoleTarget(&local);
}

static ArmSimple_BehaviorPoint_t AutoOre_FusedPickArmPoint(
    const AutoOre_t *ctrl) {
  AutoOre_Action_t pick_action =
      ctrl->param.fused_step_pick_store_ascend_200_head.pick_action;
  if (!AutoOre_ActionIsPick(pick_action)) {
    pick_action = AUTO_ORE_ACTION_PICK_POS_200;
  }
  return AutoOre_PickArmPoint(pick_action);
}

static bool AutoOre_FusedArmPhotoConfirmed(AutoOre_t *ctrl,
                                           uint32_t now_ms) {
  const AutoOre_FusedParam_t *param =
      &ctrl->param.fused_step_pick_store_ascend_200_head;
  if (!param->use_arm_photo_confirm) {
    ctrl->pick_lift_confirmed = true;
    return true;
  }

  if (!ctrl->feedback.arm_photo_has_ore) {
    ctrl->fused_arm_photo_since_ms = 0u;
    return false;
  }

  if (ctrl->fused_arm_photo_since_ms == 0u) {
    ctrl->fused_arm_photo_since_ms = now_ms;
    return false;
  }

  if ((now_ms - ctrl->fused_arm_photo_since_ms) >=
      AutoOre_FusedArmPhotoStableMs(ctrl)) {
    ctrl->pick_lift_confirmed = true;
  }
  return ctrl->pick_lift_confirmed;
}

static AutoOre_Position_t AutoOre_FusedSelectStorePosition(AutoOre_t *ctrl) {
  AutoOre_ApplyFeedbackOccupancy(ctrl);
  AutoOre_Position_t position = AutoOre_SelectStorePosition(&ctrl->occupancy);
  if (!AutoOre_IsPositionValid(position)) {
    position = AUTO_ORE_POSITION_TRANSFORM_LOW;
  }
  return position;
}

static void AutoOre_FusedStoreNextStep(AutoOre_t *ctrl) {
  ctrl->fused_store_step_index++;
  ctrl->fused_store_step_phase = 0u;
  ctrl->step_condition_met = false;
  ctrl->step_condition_time_ms = 0u;
}

static void AutoOre_FusedStoreMarkDone(AutoOre_t *ctrl) {
  ctrl->fused_store_done = true;
  ctrl->fused_store_step_index = 0u;
  ctrl->fused_store_step_phase = 0u;
  if (AutoOre_IsPositionValid(ctrl->fused_store_position)) {
    ctrl->active_position = ctrl->fused_store_position;
    AutoOre_SetActivePositionHasOre(ctrl, true);
  }
  AutoOre_SetArmHasOre(ctrl, false);
  ctrl->fused_store_position = AUTO_ORE_POSITION_NONE;
}

static void AutoOre_RunFusedStoreLow(AutoOre_t *ctrl, uint32_t now_ms) {
  switch (ctrl->fused_store_step_index) {
    case 0:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_WAIT_STORE_ORE,
                              SUCTION_ON,
                              &ctrl->param.arm_speed.store_wait) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.arm_at_target && ctrl->feedback.ore_store_all_at_target) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 1: {
      const Suction_State_t suction =
          (ctrl->fused_store_step_phase == 0u) ? SUCTION_ON : SUCTION_OFF;
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE, suction,
                              &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(
              ctrl, ORE_STORE_TRANSFORM_LIFT,
              ctrl->fused_store_step_phase != 0u)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->fused_store_step_phase == 0u) {
        if (AutoOre_WaitConditionThenDelay(
                ctrl, now_ms, ctrl->feedback.arm_at_target,
                AutoOre_StoreArmSettleMs(ctrl))) {
          ctrl->fused_store_step_phase = 1u;
          ctrl->step_condition_met = false;
        }
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AutoOre_StoreCylinderCloseMs(ctrl))) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    }
    case 2:
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AutoOre_StoreCylinderOpenMs(ctrl))) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 3:
      if (!AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitOreStoreCommandTarget(ctrl, now_ms)) {
        AutoOre_FusedStoreMarkDone(ctrl);
      }
      return;
    default:
      AutoOre_FusedStoreMarkDone(ctrl);
      return;
  }
}

static void AutoOre_RunFusedStoreHigh(AutoOre_t *ctrl, uint32_t now_ms) {
  switch (ctrl->fused_store_step_index) {
    case 0:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
                              SUCTION_ON,
                              &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(
              ctrl, now_ms, ctrl->feedback.arm_at_target,
              AutoOre_StoreArmSettleMs(ctrl))) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 1:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
                              SUCTION_OFF,
                              &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AutoOre_StoreCylinderCloseMs(ctrl))) {
        AutoOre_FusedStoreMarkDone(ctrl);
      }
      return;
    default:
      AutoOre_FusedStoreMarkDone(ctrl);
      return;
  }
}

static void AutoOre_RunFusedStoreArm(AutoOre_t *ctrl, uint32_t now_ms) {
  (void)now_ms;
  if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY, SUCTION_ON,
                          &ctrl->param.arm_speed.store_standby)) {
    AutoOre_FailInvalidParam(ctrl);
    return;
  }
  if (ctrl->feedback.arm_at_target) {
    AutoOre_FusedStoreMarkDone(ctrl);
  }
}

static void AutoOre_RunFusedStore(AutoOre_t *ctrl, uint32_t now_ms) {
  if (!AutoOre_IsPositionValid(ctrl->fused_store_position)) {
    ctrl->fused_store_position = AutoOre_FusedSelectStorePosition(ctrl);
    ctrl->fused_store_step_index = 0u;
    ctrl->fused_store_step_phase = 0u;
    ctrl->step_condition_met = false;
  }

  switch (ctrl->fused_store_position) {
    case AUTO_ORE_POSITION_TRANSFORM_LOW:
      AutoOre_RunFusedStoreLow(ctrl, now_ms);
      return;
    case AUTO_ORE_POSITION_TRANSFORM_HIGH:
      AutoOre_RunFusedStoreHigh(ctrl, now_ms);
      return;
    case AUTO_ORE_POSITION_ARM:
      AutoOre_RunFusedStoreArm(ctrl, now_ms);
      return;
    default:
      AutoOre_FailInvalidParam(ctrl);
      return;
  }
}

static void AutoOre_CopyFeedbackToStepCtrl(AutoOre_t *ctrl) {
  auto_ctrl_feedback_t step_feedback = {0};
  step_feedback.yaw_auto_rad = ctrl->feedback.yaw_auto_rad;
  step_feedback.pe13_photo1_triggered = ctrl->feedback.pe13_photo1_triggered;
  step_feedback.pe9_photo2_triggered = ctrl->feedback.pe9_photo2_triggered;
  step_feedback.pa2_photo3_triggered = ctrl->feedback.pa2_photo3_triggered;
  step_feedback.pa0_photo4_triggered = ctrl->feedback.pa0_photo4_triggered;
  step_feedback.pole_front_lift_rad = ctrl->feedback.pole_front_lift_rad;
  step_feedback.pole_rear_lift_rad = ctrl->feedback.pole_rear_lift_rad;
  step_feedback.pole_front_at_target = ctrl->feedback.pole_front_at_target;
  step_feedback.pole_rear_at_target = ctrl->feedback.pole_rear_at_target;
  step_feedback.pole_all_at_target = ctrl->feedback.pole_all_at_target;
  for (uint8_t i = 0u; i < 4u; ++i) {
    step_feedback.wheel_position_rad[i] = ctrl->feedback.wheel_position_rad[i];
  }
  AutoCtrl_SetFeedback(&ctrl->step_ctrl, &step_feedback);
  AutoCtrl_SetYawRateCommand(&ctrl->step_ctrl,
                             ctrl->feedback.yaw_rate_cmd_rad_s);
}

static void AutoOre_CopyStepCtrlOutputs(AutoOre_t *ctrl) {
  ctrl->chassis_cmd = ctrl->step_ctrl.chassis_cmd;
  ctrl->pole_cmd = ctrl->step_ctrl.pole_cmd;
  ctrl->chassis_cmd_valid =
      ctrl->step_ctrl.chassis_cmd.mode != CHASSIS_MODE_RELAX;
  ctrl->pole_cmd_valid = ctrl->step_ctrl.pole_cmd.mode != POLE_MODE_RELAX;
}

static void AutoOre_RunFusedStepTemplate(AutoOre_t *ctrl, uint32_t now_ms) {
  const AutoOre_FusedParam_t *param =
      &ctrl->param.fused_step_pick_store_ascend_200_head;

  if (!ctrl->step_ctrl_started) {
    AutoCtrl_Init(&ctrl->step_ctrl);
    AutoCtrl_SetYawSource(&ctrl->step_ctrl, AUTO_CTRL_YAW_SOURCE_STM32);
    AutoCtrl_SetYawZeroOffset(&ctrl->step_ctrl, 0.0f);
    AutoOre_CopyFeedbackToStepCtrl(ctrl);
    const auto_ctrl_template_e template_id =
        (param->step_template != AUTO_CTRL_TEMPLATE_NONE)
            ? param->step_template
            : AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD;
    const float yaw_tolerance =
        (param->yaw_tolerance_rad > 0.0f) ? param->yaw_tolerance_rad : 0.35f;
    if (!AutoCtrl_StartTemplate(&ctrl->step_ctrl, template_id,
                                AUTO_CTRL_TRAVEL_DIR_HEAD_FORWARD,
                                param->target_yaw_rad, yaw_tolerance,
                                AUTO_CTRL_SENSOR_MODE_YAW_ONLY, now_ms)) {
      AutoOre_FailInvalidParam(ctrl);
      return;
    }
    ctrl->step_ctrl_started = true;
    ctrl->step_ctrl_active = true;
  }

  AutoOre_CopyFeedbackToStepCtrl(ctrl);
  AutoCtrl_Update(&ctrl->step_ctrl, now_ms);
  AutoOre_CopyStepCtrlOutputs(ctrl);

  if (AutoCtrl_GetState(&ctrl->step_ctrl) == AUTO_CTRL_STATE_SUCCESS) {
    ctrl->fused_step_done = true;
    ctrl->step_ctrl_active = false;
    AutoOre_NextStep(ctrl);
    return;
  }

  if (AutoCtrl_GetState(&ctrl->step_ctrl) == AUTO_CTRL_STATE_FAIL ||
      AutoCtrl_GetState(&ctrl->step_ctrl) == AUTO_CTRL_STATE_ABORT) {
    ctrl->state = AUTO_ORE_STATE_FAIL;
    ctrl->result = AUTO_ORE_RESULT_FAIL;
    ctrl->fault = (AutoCtrl_GetFault(&ctrl->step_ctrl) ==
                   AUTO_CTRL_FAULT_TEMPLATE_TIMEOUT)
                      ? AUTO_ORE_FAULT_TIMEOUT
                      : AUTO_ORE_FAULT_SENSOR_INVALID;
  }
}

static void AutoOre_RunStepPickStoreAscend200Head(AutoOre_t *ctrl,
                                                  uint32_t now_ms) {
  const AutoOre_FusedParam_t *fused =
      &ctrl->param.fused_step_pick_store_ascend_200_head;
  const float *pole_target = AutoOre_FusedPickPoleTarget(ctrl);
  if (pole_target == 0) {
    AutoOre_FailInvalidParam(ctrl);
    return;
  }

  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      AutoOre_CommandChassisHold(ctrl);
      const float yaw_tolerance =
          (fused->yaw_tolerance_rad > 0.0f) ? fused->yaw_tolerance_rad : 0.35f;
      if (AutoOre_AbsFloat(AutoCtrlMath_GetYawErrorRad(
              fused->target_yaw_rad, ctrl->feedback.yaw_auto_rad)) <=
          yaw_tolerance) {
        if (ctrl->fused_yaw_stable_since_ms == 0u) {
          ctrl->fused_yaw_stable_since_ms = now_ms;
        }
      } else {
        ctrl->fused_yaw_stable_since_ms = 0u;
      }
      if (ctrl->fused_yaw_stable_since_ms != 0u &&
          (now_ms - ctrl->fused_yaw_stable_since_ms) >=
              AutoOre_FusedPrealignStableMs(ctrl)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      AutoOre_CommandChassisHold(ctrl);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY, SUCTION_ON,
                              &ctrl->param.arm_speed.pick_standby) ||
          !AutoOre_CommandPoleTarget(ctrl, pole_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.pole_all_at_target) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      AutoOre_CommandChassisHold(ctrl);
      if (!AutoOre_CommandArm(ctrl, AutoOre_FusedPickArmPoint(ctrl),
                              SUCTION_ON,
                              &ctrl->param.arm_speed.pick_place) ||
          !AutoOre_CommandPoleTarget(ctrl, pole_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 3:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, AutoOre_FusedPickArmPoint(ctrl),
                              SUCTION_ON,
                              &ctrl->param.arm_speed.pick_fetch) ||
          !AutoOre_CommandPoleTarget(ctrl, pole_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMoveYawRate(ctrl, fused->precontact_vx_mps,
                                        ctrl->feedback.yaw_rate_cmd_rad_s);
      if (fused->fail_on_precontact_front_photo &&
          ctrl->feedback.precontact_front_photo_triggered) {
        ctrl->state = AUTO_ORE_STATE_FAIL;
        ctrl->result = AUTO_ORE_RESULT_FAIL;
        ctrl->fault = AUTO_ORE_FAULT_SENSOR_INVALID;
        return;
      }
      if (AutoOre_DistanceMoveReached(ctrl, fused->precontact_distance_m) ||
          AutoOre_StepElapsed(ctrl, now_ms) >=
              AutoOre_FusedPickPrecontactTimeoutMs(ctrl)) {
        AutoOre_CommandChassisHold(ctrl);
        AutoOre_NextStep(ctrl);
      }
      return;
    case 4:
      AutoOre_EnterStep(ctrl, now_ms);
      AutoOre_CommandChassisHold(ctrl);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_PICK_LIFT_DETECT,
                              SUCTION_ON,
                              &ctrl->param.arm_speed.pick_lift_detect) ||
          !AutoOre_CommandPoleTarget(ctrl, pole_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(
              ctrl, now_ms, ctrl->feedback.arm_at_target,
              AutoOre_FusedPickLiftDetectMs(ctrl)) &&
          AutoOre_FusedArmPhotoConfirmed(ctrl, now_ms)) {
        AutoOre_SetArmHasOre(ctrl, true);
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 5:
      AutoOre_EnterStep(ctrl, now_ms);
      AutoOre_CommandChassisHold(ctrl);
      AutoOre_RunFusedStore(ctrl, now_ms);
      if (ctrl->fused_store_done) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 6:
      AutoOre_EnterStep(ctrl, now_ms);
      if (fused->step_start_distance_m <= 0.0f) {
        AutoOre_NextStep(ctrl);
        return;
      }
      if (!AutoOre_CommandPoleTarget(
              ctrl, ctrl->param.pole_param->preset.step_200_all_extend)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMoveYawRate(ctrl, fused->step_start_vx_mps,
                                        ctrl->feedback.yaw_rate_cmd_rad_s);
      if (AutoOre_DistanceMoveReached(ctrl, fused->step_start_distance_m)) {
        AutoOre_CommandChassisHold(ctrl);
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 7:
      AutoOre_EnterStep(ctrl, now_ms);
      AutoOre_RunFusedStepTemplate(ctrl, now_ms);
      return;
    case 8:
      AutoOre_FinishSuccess(ctrl);
      return;
    default:
      AutoOre_FinishSuccess(ctrl);
      return;
  }
}

static void AutoOre_RunPickOre(AutoOre_t *ctrl, uint32_t now_ms) {
  const float *pole_target = AutoOre_PickPoleTarget(ctrl);
  if (pole_target == 0) {
    AutoOre_FailInvalidParam(ctrl);
    return;
  }

  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY,
                              SUCTION_ON,
                              &ctrl->param.arm_speed.pick_standby) ||
          !AutoOre_CommandPoleTarget(ctrl, pole_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.pole_all_at_target) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, AutoOre_PickArmPoint(ctrl->action),
                              SUCTION_ON,
                              &ctrl->param.arm_speed.pick_place) ||
          !AutoOre_CommandPoleTarget(ctrl, pole_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, AutoOre_PickArmPoint(ctrl->action),
                              SUCTION_ON,
                              &ctrl->param.arm_speed.pick_fetch) ||
          !AutoOre_CommandPoleTarget(ctrl, pole_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMove(ctrl, AutoOre_FetchChassisVxMps(ctrl));
      if (AutoOre_StepElapsed(ctrl, now_ms) >=
          AutoOre_FetchChassisMoveMs(ctrl)) {
        AutoOre_NextStep(ctrl);
      }
      return;
    case 3:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY,
                              SUCTION_ON,
                              &ctrl->param.arm_speed.pick_standby) ||
          !AutoOre_CommandPoleTarget(ctrl, pole_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
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
  } else if (AutoOre_ActionIsPick(action)) {
    position = AUTO_ORE_POSITION_ARM;
  } else if (AutoOre_ActionIsFused(action)) {
    position = AUTO_ORE_POSITION_ARM;
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
  ctrl->step_ctrl_active = false;
  ctrl->step_ctrl_started = false;
  ctrl->fused_step_done = false;
  ctrl->fused_store_done = false;
  ctrl->pick_lift_confirmed = false;
  ctrl->fused_yaw_stable_since_ms = 0u;
  ctrl->fused_arm_photo_since_ms = 0u;
  ctrl->fused_store_step_index = 0u;
  ctrl->fused_store_step_phase = 0u;
  ctrl->fused_store_position = AUTO_ORE_POSITION_NONE;
  ctrl->fused_target_yaw_rad =
      ctrl->param.fused_step_pick_store_ascend_200_head.target_yaw_rad;
  AutoOre_ResetDistanceGate(ctrl);
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

bool AutoOre_StartPickPos400(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_PICK_POS_400, now_ms);
}

bool AutoOre_StartPickPos200(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_PICK_POS_200, now_ms);
}

bool AutoOre_StartPickNeg200(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_PICK_NEG_200, now_ms);
}

bool AutoOre_StartStepPickStoreAscend200Head(AutoOre_t *ctrl,
                                             uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD,
                       now_ms);
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
  if ((!AutoOre_ActionIsPick(ctrl->action) ||
       AutoOre_ActionIsFused(ctrl->action)) &&
      !ctrl->feedback.ore_store_all_homed) {
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
  } else if (AutoOre_ActionIsPick(ctrl->action)) {
    if (ctrl->active_position == AUTO_ORE_POSITION_ARM) {
      AutoOre_RunPickOre(ctrl, now_ms);
    } else {
      ctrl->state = AUTO_ORE_STATE_FAIL;
      ctrl->result = AUTO_ORE_RESULT_FAIL;
      ctrl->fault = AUTO_ORE_FAULT_INVALID_OCCUPANCY;
    }
  } else if (ctrl->action == AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD) {
    AutoOre_RunStepPickStoreAscend200Head(ctrl, now_ms);
  }
}

void AutoOre_Abort(AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return;
  }
  ctrl->state = AUTO_ORE_STATE_ABORT;
  ctrl->result = AUTO_ORE_RESULT_ABORTED;
  ctrl->fault = AUTO_ORE_FAULT_ABORTED;
  if (ctrl->step_ctrl_active || ctrl->step_ctrl_started) {
    AutoCtrl_Abort(&ctrl->step_ctrl);
  }
  ctrl->action = AUTO_ORE_ACTION_NONE;
  ctrl->active_position = AUTO_ORE_POSITION_NONE;
  ctrl->step_ctrl_active = false;
  ctrl->step_ctrl_started = false;
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

const Chassis_CMD_t *AutoOre_GetChassisCommand(const AutoOre_t *ctrl) {
  return (ctrl != 0 && ctrl->chassis_cmd_valid) ? &ctrl->chassis_cmd : 0;
}
