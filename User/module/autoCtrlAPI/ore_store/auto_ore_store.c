#include "module/autoCtrlAPI/ore_store/auto_ore_store.h"

#include "device/sick.h"
#include "module/autoCtrlAPI/core/auto_ctrl_math.h"
#include "module/config.h"

#include <math.h>
#include <string.h>

#define AUTO_ORE_DEFAULT_STORE_ARM_SETTLE_MS (100u)
#define AUTO_ORE_DEFAULT_STORE_CYLINDER_CLOSE_MS (50u)
#define AUTO_ORE_DEFAULT_STORE_ARM_SUCTION_OFF_MS (300u)
#define AUTO_ORE_DEFAULT_STORE_CYLINDER_OPEN_MS (50u)
#define AUTO_ORE_DEFAULT_STORE_LOW_RETURN_VELOCITY_RAD_S (80.0f)
#define AUTO_ORE_DEFAULT_STORE_LOW_RETURN_ACCEL_RAD_S2 (80.0f)
#define AUTO_ORE_DEFAULT_STORE_LOW_RETURN_DECEL_RAD_S2 (80.0f)
#define AUTO_ORE_STORE_LOW_RETURN_MIN_VELOCITY_RAD_S (0.05f)
#define AUTO_ORE_DEFAULT_STORE_LOW_SHAKE_AMPLITUDE_RAD (1.0f)
#define AUTO_ORE_DEFAULT_STORE_LOW_SHAKE_VELOCITY_RAD_S (50.0f)
#define AUTO_ORE_STORE_LOW_SHAKE_MAX_CYCLES (16u)
#define AUTO_ORE_DEFAULT_RELEASE_WAIT_MS (50u)
#define AUTO_ORE_RELEASE_GRID_CHECK_MS (500u)
#define AUTO_ORE_DEFAULT_RELEASE_LIFT_DETECT_TIMEOUT_MS (10000u)
#define AUTO_ORE_DEFAULT_RELEASE_LIFT_DETECT_SETTLE_MS (500u)
#define AUTO_ORE_DEFAULT_RELEASE_LIFT_DETECT_SICK_ADC_THRESHOLD (3200u)
#define AUTO_ORE_DEFAULT_RELEASE_ARM_SETTLE_MS (10u)
#define AUTO_ORE_DEFAULT_RELEASE_SUCTION_OFF_MS (100u)
#define AUTO_ORE_DEFAULT_CHAMBER_LOW_CLAMP_MS (50u)
#define AUTO_ORE_DEFAULT_CHAMBER_ARM_SETTLE_MS (100u)
#define AUTO_ORE_DEFAULT_CHAMBER_CYLINDER_OPEN_MS (50u)
#define AUTO_ORE_DEFAULT_FETCH_CHASSIS_MOVE_MS (300u)
#define AUTO_ORE_DEFAULT_RECOVER_FRONT_SICK_DELAY_MS (50u)
#define AUTO_ORE_DEFAULT_RECOVER_FRONT_SICK_ADC_THRESHOLD (2800u)
#define AUTO_ORE_DEFAULT_RECOVER_SUCTION_SETTLE_MS (200u)
#define AUTO_ORE_DEFAULT_RECOVER_CHASSIS_RETREAT_MS (500u)
#define AUTO_ORE_DEFAULT_STEP_TIMEOUT_MS (5000u)
#define AUTO_ORE_DEFAULT_FETCH_CHASSIS_VX_MPS (0.20f)
#define AUTO_ORE_DEFAULT_FUSED_PREALIGN_STABLE_MS (100u)
#define AUTO_ORE_DEFAULT_FUSED_PICK_PRECONTACT_TIMEOUT_MS (2000u)
#define AUTO_ORE_DEFAULT_FUSED_PICK_LIFT_DETECT_MS (200u)
#define AUTO_ORE_DEFAULT_FUSED_ARM_PHOTO_STABLE_MS (120u)
#define AUTO_ORE_DEFAULT_FUSED_PHOTO1_LIFT_DELAY_MS (200u)
#define AUTO_ORE_FUSED_STEP_PHOTO_STABLE_MS (20u)
#define AUTO_ORE_PICK_STORE_FETCH_PHOTO_DELAY_MS (100u)
#define AUTO_ORE_PICK_STORE_RETREAT_PHOTO_DELAY_MS (100u)
#define AUTO_ORE_FUSED_HEAD_ASCEND_FRONT_RETRACT_STEP_INDEX (2u)
#define AUTO_ORE_FUSED_HEAD_DESCEND_FIRST_EDGE_STEP_INDEX (1u)
#define AUTO_ORE_FUSED_ARM_PHOTO_ENABLE_JOINT1_RAD (0.6981317f)

#ifndef AUTO_CTRL_STM32_YAW_WZ_ENABLE
#define AUTO_CTRL_STM32_YAW_WZ_ENABLE (0u)
#endif

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
#define AUTO_ORE_ARM_OCCUPANCY_SOURCE AUTO_ORE_OCCUPANCY_SOURCE_PHOTOELECTRIC_VALUE
#endif

static uint32_t AutoOre_StepElapsed(const AutoOre_t *ctrl, uint32_t now_ms) {
  return now_ms - ctrl->step_enter_time_ms;
}

static float AutoOre_AbsFloat(float value) {
  return (value >= 0.0f) ? value : -value;
}

static bool AutoOre_ActionIsFused(AutoOre_Action_t action);
static auto_ctrl_template_e AutoOre_FusedStepTemplateId(
  const AutoOre_t *ctrl);
static float AutoOre_SelectPrealignWz(AutoOre_t *ctrl);
static float AutoOre_OreStoreArriveThresholdRad(const AutoOre_t *ctrl);
static void AutoOre_FinishSuccess(AutoOre_t *ctrl);
static void AutoOre_SetActivePositionHasOre(AutoOre_t *ctrl, bool has_ore);
static bool AutoOre_CommandArm(AutoOre_t *ctrl,
                 ArmSimple_BehaviorPoint_t point,
                 Suction_State_t suction,
                 const AutoOre_ArmSpeedLimit_t *speed);
static bool AutoOre_CommandReleasePoleTarget(AutoOre_t *ctrl);
static bool AutoOre_CommandReleaseOreStoreHold(
  AutoOre_t *ctrl, OreStore_TransformPoint_t transform,
  bool cylinder_closed);
static void AutoOre_CommandChassisMoveYawRate(AutoOre_t *ctrl, float vx_mps,
                                               float wz_rad_s);
static void AutoOre_CommandChassisHold(AutoOre_t *ctrl);
static void AutoOre_CommandChassisZeroVector(AutoOre_t *ctrl);
static bool AutoOre_WaitArmCommandTarget(AutoOre_t *ctrl, uint32_t now_ms);
static bool AutoOre_WaitPickArmCommandTargetStable(AutoOre_t *ctrl,
                                                    uint32_t now_ms);
static bool AutoOre_CheckTimeout(AutoOre_t *ctrl, uint32_t now_ms);
static void AutoOre_FailInvalidParam(AutoOre_t *ctrl);

static bool AutoOre_ActionIsReleaseLike(AutoOre_Action_t action) {
  return action == AUTO_ORE_ACTION_RELEASE ||
         action == AUTO_ORE_ACTION_RELEASE_LIFT_DETECT ||
      action == AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT ||
         action == AUTO_ORE_ACTION_RELEASE_STEP1 ||
         action == AUTO_ORE_ACTION_RELEASE_STEP2 ||
         action == AUTO_ORE_ACTION_RELEASE_LIFT_DETECT_STEP1 ||
      action == AUTO_ORE_ACTION_RELEASE_LIFT_DETECT_STEP2 ||
      action == AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT_STEP1 ||
      action == AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT_STEP2;
}

static bool AutoOre_ActionUsesReleaseLiftDetect(AutoOre_Action_t action) {
  return action == AUTO_ORE_ACTION_RELEASE_LIFT_DETECT ||
         action == AUTO_ORE_ACTION_RELEASE_LIFT_DETECT_STEP1 ||
      action == AUTO_ORE_ACTION_RELEASE_LIFT_DETECT_STEP2 ||
      action == AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT ||
      action == AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT_STEP1 ||
      action == AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT_STEP2;
}

static bool AutoOre_ActionUsesIrReleaseLiftDetect(AutoOre_Action_t action) {
  return action == AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT ||
      action == AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT_STEP1 ||
      action == AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT_STEP2;
}

static bool AutoOre_ActionIsZone3IrRelease(AutoOre_Action_t action) {
  return action == AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT;
}

static bool AutoOre_ActionIsReleaseStep1(AutoOre_Action_t action) {
  return action == AUTO_ORE_ACTION_RELEASE_STEP1 ||
      action == AUTO_ORE_ACTION_RELEASE_LIFT_DETECT_STEP1 ||
      action == AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT_STEP1;
}

static bool AutoOre_ActionIsReleaseStep2(AutoOre_Action_t action) {
  return action == AUTO_ORE_ACTION_RELEASE_STEP2 ||
      action == AUTO_ORE_ACTION_RELEASE_LIFT_DETECT_STEP2 ||
      action == AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT_STEP2;
}

static bool AutoOre_ActionIsPickStoreFused(AutoOre_Action_t action) {
  return action == AUTO_ORE_ACTION_PICK_STORE_POS_400 ||
         action == AUTO_ORE_ACTION_PICK_STORE_POS_200 ||
         action == AUTO_ORE_ACTION_PICK_STORE_NEG_200;
}

static bool AutoOre_ActionIsDropStoreFused(AutoOre_Action_t action) {
  return action == AUTO_ORE_ACTION_STEP_DROP_STORE_ASCEND_200_HEAD ||
         action == AUTO_ORE_ACTION_STEP_DROP_STORE_DESCEND_200_HEAD ||
         action == AUTO_ORE_ACTION_STEP_DROP_STORE_ASCEND_400_HEAD;
}

static AutoOre_Action_t AutoOre_PickStoreFusedPickAction(
    AutoOre_Action_t action) {
  switch (action) {
    case AUTO_ORE_ACTION_PICK_STORE_POS_400:
      return AUTO_ORE_ACTION_PICK_POS_400;
    case AUTO_ORE_ACTION_PICK_STORE_POS_200:
      return AUTO_ORE_ACTION_PICK_POS_200;
    case AUTO_ORE_ACTION_PICK_STORE_NEG_200:
      return AUTO_ORE_ACTION_PICK_NEG_200;
    default:
      return action;
  }
}

static bool AutoOre_PickActionUsesPhoto1Falling(AutoOre_Action_t action) {
  return AutoOre_PickStoreFusedPickAction(action) ==
         AUTO_ORE_ACTION_PICK_NEG_200;
}

static void AutoOre_ResetPhoto1Latch(AutoOre_t *ctrl) {
  ctrl->fused_photo1_stable_trigger_seen = false;
  ctrl->fused_photo1_stable_release_latched = false;
  ctrl->fused_photo1_arm_protection_latched = false;
  ctrl->fused_photo1_triggered_since_ms = 0u;
  ctrl->fused_photo1_released_since_ms = 0u;
}

static void AutoOre_PausePhoto1Latch(AutoOre_t *ctrl, uint32_t now_ms) {
  if (ctrl == 0) {
    return;
  }
  if (!ctrl->fused_photo1_stable_trigger_seen) {
    if (ctrl->fused_photo1_triggered_since_ms != 0u) {
      ctrl->fused_photo1_triggered_since_ms = now_ms;
    }
    return;
  }
  if (!ctrl->fused_photo1_stable_release_latched &&
      ctrl->fused_photo1_released_since_ms != 0u) {
    ctrl->fused_photo1_released_since_ms = now_ms;
  }
}

static void AutoOre_ResetPhoto2Latch(AutoOre_t *ctrl) {
  ctrl->fused_photo2_stable_trigger_seen = false;
  ctrl->fused_photo2_stable_release_latched = false;
  ctrl->fused_photo2_handoff_applied = false;
  ctrl->fused_photo2_triggered_since_ms = 0u;
  ctrl->fused_photo2_released_since_ms = 0u;
}

static void AutoOre_PausePhoto2Latch(AutoOre_t *ctrl, uint32_t now_ms) {
  if (ctrl == 0) {
    return;
  }
  if (!ctrl->fused_photo2_stable_trigger_seen) {
    if (ctrl->fused_photo2_triggered_since_ms != 0u) {
      ctrl->fused_photo2_triggered_since_ms = now_ms;
    }
    return;
  }
  if (!ctrl->fused_photo2_stable_release_latched &&
      ctrl->fused_photo2_released_since_ms != 0u) {
    ctrl->fused_photo2_released_since_ms = now_ms;
  }
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
  ctrl->arm_target_stable_since_ms = 0u;
  ctrl->distance_latch_valid = false;
  ctrl->wheel_delta_rad = 0.0f;
  ctrl->target_wheel_delta_rad = 0.0f;
}

static void AutoOre_JumpToStep(AutoOre_t *ctrl, uint8_t step_index) {
  ctrl->step_index = step_index;
  ctrl->step_phase = 0u;
  ctrl->step_entered = false;
  ctrl->step_condition_met = false;
  ctrl->arm_target_stable_since_ms = 0u;
  ctrl->distance_latch_valid = false;
  ctrl->wheel_delta_rad = 0.0f;
  ctrl->target_wheel_delta_rad = 0.0f;
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

static uint32_t AutoOre_StoreArmSuctionOffMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.store_arm_suction_off_ms,
                             AUTO_ORE_DEFAULT_STORE_ARM_SUCTION_OFF_MS);
}

static uint32_t AutoOre_StoreCylinderOpenMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.store_cylinder_open_ms,
                             AUTO_ORE_DEFAULT_STORE_CYLINDER_OPEN_MS);
}

static float AutoOre_StoreLowReturnVelocityRadS(const AutoOre_t *ctrl) {
  return (ctrl->param.store_low_return_velocity_rad_s > 0.0f)
             ? ctrl->param.store_low_return_velocity_rad_s
             : AUTO_ORE_DEFAULT_STORE_LOW_RETURN_VELOCITY_RAD_S;
}

static float AutoOre_StoreLowReturnAccelRadS2(const AutoOre_t *ctrl) {
  return (ctrl->param.store_low_return_accel_rad_s2 > 0.0f)
             ? ctrl->param.store_low_return_accel_rad_s2
             : AUTO_ORE_DEFAULT_STORE_LOW_RETURN_ACCEL_RAD_S2;
}

static float AutoOre_StoreLowReturnDecelRadS2(const AutoOre_t *ctrl) {
  return (ctrl->param.store_low_return_decel_rad_s2 > 0.0f)
             ? ctrl->param.store_low_return_decel_rad_s2
             : AUTO_ORE_DEFAULT_STORE_LOW_RETURN_DECEL_RAD_S2;
}

static float AutoOre_StoreLowShakeAmplitudeRad(const AutoOre_t *ctrl) {
  return (ctrl->param.store_low_shake_amplitude_rad > 0.0f &&
          isfinite(ctrl->param.store_low_shake_amplitude_rad))
             ? ctrl->param.store_low_shake_amplitude_rad
             : AUTO_ORE_DEFAULT_STORE_LOW_SHAKE_AMPLITUDE_RAD;
}

static float AutoOre_StoreLowShakeVelocityRadS(const AutoOre_t *ctrl) {
  return (ctrl->param.store_low_shake_velocity_rad_s > 0.0f &&
          isfinite(ctrl->param.store_low_shake_velocity_rad_s))
             ? ctrl->param.store_low_shake_velocity_rad_s
             : AUTO_ORE_DEFAULT_STORE_LOW_SHAKE_VELOCITY_RAD_S;
}

static uint8_t AutoOre_StoreLowShakeCycles(const AutoOre_t *ctrl) {
  uint8_t cycles = ctrl->param.store_low_shake_cycles;
  if (cycles > AUTO_ORE_STORE_LOW_SHAKE_MAX_CYCLES) {
    cycles = AUTO_ORE_STORE_LOW_SHAKE_MAX_CYCLES;
  }
  return cycles;
}

static bool AutoOre_StoreLowReturnSegmentsConfigured(const AutoOre_t *ctrl) {
  for (uint8_t i = 0u; i < AUTO_ORE_STORE_LOW_RETURN_SEGMENT_COUNT; ++i) {
    const AutoOre_StoreLowReturnSegment_t *segment =
        &ctrl->param.store_low_return_segments[i];
    if (segment->end_ratio > 0.0f && segment->velocity_rad_s > 0.0f) {
      return true;
    }
  }
  return false;
}

static float AutoOre_MinPositiveFloat(float a, float b) {
  if (a <= 0.0f) {
    return b;
  }
  if (b <= 0.0f) {
    return a;
  }
  return (a < b) ? a : b;
}

static float AutoOre_StoreLowReturnSegmentVelocityRadS(const AutoOre_t *ctrl) {
  const float start_rad =
      ctrl->param.ore_store_param->preset
          .transform_position_rad[ORE_STORE_TRANSFORM_LIFT];
  const float target_rad =
      ctrl->param.ore_store_param->preset
          .transform_position_rad[ORE_STORE_TRANSFORM_STANDBY];
  const float total_rad = AutoOre_AbsFloat(start_rad - target_rad);
  float progress = 1.0f;
  if (total_rad > 0.0001f) {
    const float remaining_rad =
        AutoOre_AbsFloat(ctrl->feedback.ore_store_platform_position_rad -
                         target_rad);
    progress = 1.0f - (remaining_rad / total_rad);
    if (progress < 0.0f) {
      progress = 0.0f;
    } else if (progress > 1.0f) {
      progress = 1.0f;
    }
  }

  float fallback_velocity = AutoOre_StoreLowReturnVelocityRadS(ctrl);
  for (uint8_t i = 0u; i < AUTO_ORE_STORE_LOW_RETURN_SEGMENT_COUNT; ++i) {
    const AutoOre_StoreLowReturnSegment_t *segment =
        &ctrl->param.store_low_return_segments[i];
    if (segment->velocity_rad_s <= 0.0f) {
      continue;
    }
    fallback_velocity = segment->velocity_rad_s;
    float end_ratio = segment->end_ratio;
    if (end_ratio <= 0.0f) {
      continue;
    }
    if (end_ratio > 1.0f) {
      end_ratio = 1.0f;
    }
    if (progress <= end_ratio ||
        i == AUTO_ORE_STORE_LOW_RETURN_SEGMENT_COUNT - 1u) {
      return segment->velocity_rad_s;
    }
  }
  return fallback_velocity;
}

static float AutoOre_StoreLowReturnPlannedVelocityRadS(
    const AutoOre_t *ctrl, uint32_t now_ms) {
  if (AutoOre_StoreLowReturnSegmentsConfigured(ctrl)) {
    float velocity = AutoOre_StoreLowReturnSegmentVelocityRadS(ctrl);
    if (velocity < AUTO_ORE_STORE_LOW_RETURN_MIN_VELOCITY_RAD_S) {
      velocity = AUTO_ORE_STORE_LOW_RETURN_MIN_VELOCITY_RAD_S;
    }
    return velocity;
  }

  const float max_velocity = AutoOre_StoreLowReturnVelocityRadS(ctrl);
  const float accel = AutoOre_StoreLowReturnAccelRadS2(ctrl);
  const float decel = AutoOre_StoreLowReturnDecelRadS2(ctrl);
  const float elapsed_s = (float)AutoOre_StepElapsed(ctrl, now_ms) * 0.001f;
  const float target_rad =
      ctrl->param.ore_store_param->preset
          .transform_position_rad[ORE_STORE_TRANSFORM_STANDBY];
  const float remaining_rad =
      AutoOre_AbsFloat(ctrl->feedback.ore_store_platform_position_rad -
                       target_rad);
  const float accel_limited_velocity = accel * elapsed_s;
  const float decel_limited_velocity = sqrtf(2.0f * decel * remaining_rad);
  float velocity = max_velocity;
  velocity = AutoOre_MinPositiveFloat(velocity, accel_limited_velocity);
  velocity = AutoOre_MinPositiveFloat(velocity, decel_limited_velocity);
  if (velocity < AUTO_ORE_STORE_LOW_RETURN_MIN_VELOCITY_RAD_S) {
    velocity = AUTO_ORE_STORE_LOW_RETURN_MIN_VELOCITY_RAD_S;
  }
  return velocity;
}

static uint32_t AutoOre_ReleaseWaitMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.release_wait_ms,
                             AUTO_ORE_DEFAULT_RELEASE_WAIT_MS);
}

static uint32_t AutoOre_ReleaseLiftDetectTimeoutMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(
      ctrl->param.timing.release_lift_detect_timeout_ms,
      AUTO_ORE_DEFAULT_RELEASE_LIFT_DETECT_TIMEOUT_MS);
}

static uint32_t AutoOre_ReleaseLiftDetectSettleMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(
      ctrl->param.timing.release_lift_detect_settle_ms,
      AUTO_ORE_DEFAULT_RELEASE_LIFT_DETECT_SETTLE_MS);
}

static uint8_t AutoOre_ReleaseLiftDetectSickIndex(const AutoOre_t *ctrl) {
  return (ctrl->param.release_lift_detect_sick_index <
          SICK_OUTPUT_CHANNEL_COUNT)
             ? ctrl->param.release_lift_detect_sick_index
             : SICK_BOTTOM_PHOTO_INDEX;
}

static uint16_t AutoOre_ReleaseLiftDetectSickAdcThreshold(
    const AutoOre_t *ctrl) {
  return (ctrl->param.release_lift_detect_sick_adc_threshold > 0u)
             ? ctrl->param.release_lift_detect_sick_adc_threshold
             : AUTO_ORE_DEFAULT_RELEASE_LIFT_DETECT_SICK_ADC_THRESHOLD;
}

static void AutoOre_ResetReleaseLiftObserver(AutoOre_t *ctrl) {
  ctrl->release_lift_observer_active = false;
  ctrl->release_lift_detected = false;
  ctrl->release_grid_check_active = false;
  ctrl->release_grid_check_done = false;
  ctrl->release_grid_has_ore = false;
  ctrl->release_lift_observer_start_ms = 0u;
  ctrl->release_lift_observer_last_ms = 0u;
  ctrl->release_lift_detect_time_ms = 0u;
  ctrl->release_grid_check_start_ms = 0u;
  ctrl->release_grid_check_last_ms = 0u;
  ctrl->release_lift_sick_index = AutoOre_ReleaseLiftDetectSickIndex(ctrl);
  ctrl->release_lift_sick_adc_raw = 0u;
  ctrl->release_lift_sick_adc_threshold =
      AutoOre_ReleaseLiftDetectSickAdcThreshold(ctrl);
  ctrl->release_lift_sick_valid = false;
}

static bool AutoOre_UpdateReleaseGridCheck(AutoOre_t *ctrl, uint32_t now_ms,
                                           bool wait_until_empty) {
  if (wait_until_empty && ctrl->feedback.release_grid_has_ore) {
    ctrl->release_grid_check_active = true;
    ctrl->release_grid_check_done = false;
    ctrl->release_grid_has_ore = true;
    ctrl->release_grid_check_start_ms = now_ms;
    ctrl->release_grid_check_last_ms = now_ms;
    return false;
  }

  if (!ctrl->release_grid_check_active) {
    ctrl->release_grid_check_active = true;
    ctrl->release_grid_check_done = false;
    ctrl->release_grid_has_ore = false;
    ctrl->release_grid_check_start_ms = now_ms;
    ctrl->release_grid_check_last_ms = now_ms;
  }

  if (wait_until_empty && ctrl->release_grid_has_ore) {
    ctrl->release_grid_check_done = false;
    ctrl->release_grid_has_ore = false;
    ctrl->release_grid_check_start_ms = now_ms;
  }

  ctrl->release_grid_check_last_ms = now_ms;
  if (ctrl->feedback.release_grid_has_ore) {
    ctrl->release_grid_has_ore = true;
  }
  if ((now_ms - ctrl->release_grid_check_start_ms) >=
      AUTO_ORE_RELEASE_GRID_CHECK_MS) {
    ctrl->release_grid_check_done = true;
  }
  return ctrl->release_grid_check_done;
}

static void AutoOre_FinishIrReleaseAbort(AutoOre_t *ctrl) {
  if (AutoOre_ActionIsZone3IrRelease(ctrl->action) &&
      ctrl->release_ir_suction_released) {
    AutoOre_SetActivePositionHasOre(ctrl, false);
  }
  ctrl->state = AUTO_ORE_STATE_ABORT;
  ctrl->result = AUTO_ORE_RESULT_ABORTED;
  ctrl->fault = AUTO_ORE_FAULT_ABORTED;
  ctrl->failure_mask = AUTO_ORE_FAILURE_ABORTED;
  ctrl->action = AUTO_ORE_ACTION_NONE;
  ctrl->active_position = AUTO_ORE_POSITION_NONE;
  ctrl->release_ir_abort_pending = false;
  ctrl->release_ir_abort_recovery_active = false;
}

static void AutoOre_RunIrReleaseAbortRecovery(AutoOre_t *ctrl,
                                               uint32_t now_ms) {
  if (AutoOre_ActionIsZone3IrRelease(ctrl->action)) {
    if (!ctrl->release_ir_abort_recovery_active) {
      ctrl->release_ir_abort_recovery_active = true;
      ctrl->release_ir_abort_source_step = ctrl->step_index;
      ctrl->release_ir_abort_suction_on =
          !ctrl->release_ir_suction_released;
      ctrl->step_entered = false;
      ctrl->step_condition_met = false;
    }

    AutoOre_CommandChassisZeroVector(ctrl);
    AutoOre_EnterStep(ctrl, now_ms);
    const Suction_State_t suction_state = ctrl->release_ir_abort_suction_on
                                              ? SUCTION_ON
                                              : SUCTION_OFF;
    if (!AutoOre_CommandReleasePoleTarget(ctrl) ||
        !AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_WAIT_RELEASE_ORE,
                            suction_state,
                            &ctrl->param.arm_speed.release_wait) ||
        !AutoOre_CommandReleaseOreStoreHold(ctrl,
                                            ORE_STORE_TRANSFORM_STANDBY,
                                            true)) {
      AutoOre_FailInvalidParam(ctrl);
      return;
    }
    if (!AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
      (void)AutoOre_CheckTimeout(ctrl, now_ms);
      return;
    }
    ctrl->release_ir_abort_pending = false;
    ctrl->release_ir_abort_recovery_active = false;
    ctrl->release_ir_abort_recovered = true;
    ctrl->step_entered = false;
    ctrl->step_condition_met = false;
    return;
  }

  if (!ctrl->release_ir_abort_recovery_active) {
    ctrl->release_ir_abort_recovery_active = true;
    ctrl->release_ir_abort_source_step = ctrl->step_index;
    if (ctrl->active_position != AUTO_ORE_POSITION_ARM) {
      ctrl->release_ir_abort_recovery_phase = 2u;
      ctrl->release_ir_abort_suction_on = true;
    } else {
      ctrl->release_ir_abort_recovery_phase =
          (ctrl->step_index >= 4u) ? 0u :
          ((ctrl->step_index >= 3u) ? 1u : 2u);
      ctrl->release_ir_abort_suction_on = ctrl->step_index < 5u;
    }
    ctrl->step_entered = false;
    ctrl->step_condition_met = false;
  }

  AutoOre_CommandChassisZeroVector(ctrl);
  AutoOre_EnterStep(ctrl, now_ms);
  const Suction_State_t suction_state = ctrl->release_ir_abort_suction_on
                                            ? SUCTION_ON
                                            : SUCTION_OFF;
  ArmSimple_BehaviorPoint_t arm_behavior = ARM_SIMPLE_BEHAVIOR_VERTICAL;
  const AutoOre_ArmSpeedLimit_t *speed = &ctrl->param.arm_speed.release_wait;
  if (ctrl->release_ir_abort_recovery_phase == 0u) {
    arm_behavior = ARM_SIMPLE_BEHAVIOR_RELEASE_ORE_ASSIST;
    speed = &ctrl->param.arm_speed.release_assist;
  } else if (ctrl->release_ir_abort_recovery_phase == 1u) {
    arm_behavior = ARM_SIMPLE_BEHAVIOR_WAIT_RELEASE_ORE;
  }

  if (!AutoOre_CommandReleasePoleTarget(ctrl) ||
      !AutoOre_CommandArm(ctrl, arm_behavior, suction_state, speed) ||
      !AutoOre_CommandReleaseOreStoreHold(ctrl, ORE_STORE_TRANSFORM_STANDBY,
                                          true)) {
    AutoOre_FailInvalidParam(ctrl);
    return;
  }

  if (!AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
    (void)AutoOre_CheckTimeout(ctrl, now_ms);
    return;
  }
  if (ctrl->release_ir_abort_recovery_phase < 2u) {
    ctrl->release_ir_abort_recovery_phase++;
    ctrl->step_entered = false;
    ctrl->step_condition_met = false;
    return;
  }
  AutoOre_FinishIrReleaseAbort(ctrl);
}

static void AutoOre_RunZone3AbortHold(AutoOre_t *ctrl, uint32_t now_ms) {
  AutoOre_CommandChassisZeroVector(ctrl);
  const Suction_State_t suction_state = ctrl->release_ir_abort_suction_on
                                            ? SUCTION_ON
                                            : SUCTION_OFF;
  if (!AutoOre_CommandReleasePoleTarget(ctrl) ||
      !AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_WAIT_RELEASE_ORE,
                          suction_state,
                          &ctrl->param.arm_speed.release_wait) ||
      !AutoOre_CommandReleaseOreStoreHold(ctrl,
                                          ORE_STORE_TRANSFORM_STANDBY,
                                          true)) {
    AutoOre_FailInvalidParam(ctrl);
    return;
  }
  (void)now_ms;
}

static void AutoOre_RunZone3FinishReturn(AutoOre_t *ctrl,
                                         uint32_t now_ms) {
  if (!ctrl->release_ir_finish_return_active) {
    ctrl->release_ir_finish_return_active = true;
    ctrl->step_entered = false;
    ctrl->step_condition_met = false;
  }

  AutoOre_CommandChassisZeroVector(ctrl);
  AutoOre_EnterStep(ctrl, now_ms);
  const Suction_State_t suction_state = ctrl->release_ir_suction_released
                                            ? SUCTION_OFF
                                            : SUCTION_ON;
  if (!AutoOre_CommandReleasePoleTarget(ctrl) ||
      !AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY,
                          suction_state,
                          &ctrl->param.arm_speed.release_standby) ||
      !AutoOre_CommandReleaseOreStoreHold(ctrl,
                                          ORE_STORE_TRANSFORM_STANDBY,
                                          true)) {
    AutoOre_FailInvalidParam(ctrl);
    return;
  }
  if (!AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
    (void)AutoOre_CheckTimeout(ctrl, now_ms);
    return;
  }
  if (ctrl->release_ir_abort_recovered) {
    AutoOre_FinishIrReleaseAbort(ctrl);
  } else {
    AutoOre_FinishSuccess(ctrl);
  }
}

static bool AutoOre_UpdateReleaseLiftObserver(AutoOre_t *ctrl,
                                              uint32_t now_ms) {
  if (!ctrl->release_lift_observer_active) {
    ctrl->release_lift_observer_active = true;
    ctrl->release_lift_detected = false;
    ctrl->release_lift_observer_start_ms = now_ms;
    ctrl->release_lift_observer_last_ms = now_ms;
    ctrl->release_lift_detect_time_ms = 0u;
    ctrl->release_lift_sick_index = AutoOre_ReleaseLiftDetectSickIndex(ctrl);
    ctrl->release_lift_sick_adc_threshold =
        AutoOre_ReleaseLiftDetectSickAdcThreshold(ctrl);
  }

  ctrl->release_lift_observer_last_ms = now_ms;
  ctrl->release_lift_sick_adc_raw = ctrl->feedback.release_lift_sick_adc_raw;
  ctrl->release_lift_sick_valid = ctrl->feedback.release_lift_sick_valid;
  if (!ctrl->release_lift_detected &&
      ctrl->release_lift_sick_valid &&
      (ctrl->param.release_lift_detect_sick_greater_than_threshold
           ? (ctrl->release_lift_sick_adc_raw >=
              ctrl->release_lift_sick_adc_threshold)
           : (ctrl->release_lift_sick_adc_raw <=
              ctrl->release_lift_sick_adc_threshold))) {
    ctrl->release_lift_detected = true;
    ctrl->release_lift_detect_time_ms = now_ms;
  }
  return ctrl->release_lift_detected;
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

static uint32_t AutoOre_FusedPrecontactTimeoutMs(
    const AutoOre_t *ctrl, const AutoOre_FusedParam_t *fused) {
  if (fused != 0 && fused->precontact_timeout_ms > 0u) {
    return fused->precontact_timeout_ms;
  }
  return AutoOre_FusedPickPrecontactTimeoutMs(ctrl);
}

static uint32_t AutoOre_FusedPickLiftDetectMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.fused_pick_lift_detect_ms,
                             AUTO_ORE_DEFAULT_FUSED_PICK_LIFT_DETECT_MS);
}

static uint32_t AutoOre_FusedArmPhotoStableMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.fused_arm_photo_stable_ms,
                             AUTO_ORE_DEFAULT_FUSED_ARM_PHOTO_STABLE_MS);
}

static uint32_t AutoOre_FusedPhoto1LiftDelayMs(const AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return AUTO_ORE_DEFAULT_FUSED_PHOTO1_LIFT_DELAY_MS;
  }
  return ctrl->param.timing.fused_photo1_lift_delay_ms;
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
  const AutoOre_Action_t pick_action =
      AutoOre_PickStoreFusedPickAction(ctrl->action);
  if (pick_action == AUTO_ORE_ACTION_PICK_NEG_200) {
    return ctrl->param.fetch_neg_200_chassis_vx_mps;
  }
  return (ctrl->param.fetch_chassis_vx_mps > 0.0f)
             ? ctrl->param.fetch_chassis_vx_mps
             : AUTO_ORE_DEFAULT_FETCH_CHASSIS_VX_MPS;
}

static uint32_t AutoOre_RecoverForwardMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.recover_chassis_forward_ms,
                             AutoOre_TimingValue(
                                 ctrl->param.timing.fetch_chassis_move_ms,
                                 AUTO_ORE_DEFAULT_FETCH_CHASSIS_MOVE_MS));
}

static uint32_t AutoOre_RecoverFrontSickDelayMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(
      ctrl->param.timing.recover_front_sick_delay_ms,
      AUTO_ORE_DEFAULT_RECOVER_FRONT_SICK_DELAY_MS);
}

static uint16_t AutoOre_RecoverFrontSickAdcThreshold(
    const AutoOre_t *ctrl) {
  return (ctrl->param.recover_front_sick_adc_threshold > 0u)
             ? ctrl->param.recover_front_sick_adc_threshold
             : AUTO_ORE_DEFAULT_RECOVER_FRONT_SICK_ADC_THRESHOLD;
}

static bool AutoOre_RecoverFrontSickReached(const AutoOre_t *ctrl) {
  return ctrl != 0 && ctrl->feedback.recover_front_sick_valid &&
         ctrl->feedback.recover_front_sick_adc_raw <=
             AutoOre_RecoverFrontSickAdcThreshold(ctrl);
}

static uint32_t AutoOre_RecoverSuctionSettleMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.recover_suction_settle_ms,
                             AUTO_ORE_DEFAULT_RECOVER_SUCTION_SETTLE_MS);
}

static uint32_t AutoOre_RecoverRetreatMs(const AutoOre_t *ctrl) {
  return AutoOre_TimingValue(ctrl->param.timing.recover_chassis_retreat_ms,
                             AUTO_ORE_DEFAULT_RECOVER_CHASSIS_RETREAT_MS);
}

static float AutoOre_RecoverForwardVxMps(const AutoOre_t *ctrl) {
  return (ctrl != 0 && ctrl->param.recover_chassis_forward_vx_mps > 0.0f)
             ? ctrl->param.recover_chassis_forward_vx_mps
             : AutoOre_FetchChassisVxMps(ctrl);
}

static float AutoOre_RecoverRetreatVxMps(const AutoOre_t *ctrl) {
  return (ctrl != 0 && ctrl->param.recover_chassis_retreat_vx_mps > 0.0f)
             ? ctrl->param.recover_chassis_retreat_vx_mps
             : AutoOre_RecoverForwardVxMps(ctrl);
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

static void AutoOre_AddFailureMask(AutoOre_t *ctrl, uint16_t failure_mask) {
  if (ctrl != 0) {
    ctrl->failure_mask |= failure_mask;
  }
}

static bool AutoOre_CheckTimeoutWithFailure(AutoOre_t *ctrl, uint32_t now_ms,
                                            uint16_t failure_mask) {
  if (AutoOre_StepElapsed(ctrl, now_ms) < AutoOre_StepTimeoutMs(ctrl)) {
    return false;
  }
  AutoOre_AddFailureMask(ctrl, failure_mask);
  ctrl->state = AUTO_ORE_STATE_FAIL;
  ctrl->result = AUTO_ORE_RESULT_FAIL;
  ctrl->fault = AUTO_ORE_FAULT_TIMEOUT;
  return true;
}

static uint32_t AutoOre_FusedParallelTimeoutMs(const AutoOre_t *ctrl) {
  const uint32_t timeout_ms = AutoOre_StepTimeoutMs(ctrl) * 4u;
  return (timeout_ms < 15000u) ? 15000u : timeout_ms;
}

static bool AutoOre_CheckFusedParallelTimeout(AutoOre_t *ctrl,
                                              uint32_t now_ms) {
  if (AutoOre_StepElapsed(ctrl, now_ms) <
      AutoOre_FusedParallelTimeoutMs(ctrl)) {
    return false;
  }
  if (!ctrl->fused_store_done) {
    AutoOre_AddFailureMask(ctrl, AUTO_ORE_FAILURE_STORE_ORE);
  }
  if (!ctrl->fused_step_done) {
    AutoOre_AddFailureMask(ctrl, AUTO_ORE_FAILURE_STEP);
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

static bool AutoOre_WaitLatchedConditionThenDelay(AutoOre_t *ctrl,
                                                  uint32_t now_ms,
                                                  bool condition,
                                                  uint32_t delay_ms) {
  if (condition && !ctrl->step_condition_met) {
    ctrl->step_condition_met = true;
    ctrl->step_condition_time_ms = now_ms;
  }
  return ctrl->step_condition_met &&
         (now_ms - ctrl->step_condition_time_ms) >= delay_ms;
}

static bool AutoOre_LatchPhotoStableFalling(
    bool triggered, bool *stable_trigger_seen, bool *stable_release_latched,
    uint32_t *triggered_since_ms, uint32_t *released_since_ms,
    uint32_t now_ms) {
  if (stable_trigger_seen == 0 || stable_release_latched == 0 ||
      triggered_since_ms == 0 || released_since_ms == 0) {
    return false;
  }

  if (*stable_release_latched) {
    return true;
  }

  if (!*stable_trigger_seen) {
    if (!triggered) {
      *triggered_since_ms = 0u;
      return false;
    }

    if (*triggered_since_ms == 0u) {
      *triggered_since_ms = now_ms;
      return false;
    }

    if ((now_ms - *triggered_since_ms) >=
        AUTO_ORE_FUSED_STEP_PHOTO_STABLE_MS) {
      *stable_trigger_seen = true;
      *released_since_ms = 0u;
    }
    return false;
  }

  if (triggered) {
    *released_since_ms = 0u;
    return false;
  }

  if (*released_since_ms == 0u) {
    *released_since_ms = now_ms;
    return false;
  }

  if ((now_ms - *released_since_ms) >=
      AUTO_ORE_FUSED_STEP_PHOTO_STABLE_MS) {
    *stable_release_latched = true;
  }
  return *stable_release_latched;
}

static bool AutoOre_LatchPhotoStableTriggered(
    bool triggered, bool *stable_trigger_latched,
    uint32_t *triggered_since_ms, uint32_t now_ms) {
  if (stable_trigger_latched == 0 || triggered_since_ms == 0) {
    return false;
  }
  if (*stable_trigger_latched) {
    return true;
  }
  if (!triggered) {
    *triggered_since_ms = 0u;
    return false;
  }
  if (*triggered_since_ms == 0u) {
    *triggered_since_ms = now_ms;
    return false;
  }
  if ((now_ms - *triggered_since_ms) >=
      AUTO_ORE_FUSED_STEP_PHOTO_STABLE_MS) {
    *stable_trigger_latched = true;
  }
  return *stable_trigger_latched;
}

static bool AutoOre_ArmCommandAtTarget(const AutoOre_t *ctrl) {
  const float threshold =
      (ctrl != 0 && ctrl->param.arm_arrive_threshold_rad > 0.0f)
          ? ctrl->param.arm_arrive_threshold_rad
          : 0.05f;
  return ctrl != 0 && ctrl->arm_cmd_valid &&
         AutoOre_AbsFloat(ctrl->arm_cmd.target_joint.joint1 -
                          ctrl->feedback.arm_joint1_rad) <= threshold &&
         AutoOre_AbsFloat(ctrl->arm_cmd.target_joint.joint2 -
                          ctrl->feedback.arm_joint2_rad) <= threshold;
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
  return AutoOre_ArmCommandAtTarget(ctrl);
}

static bool AutoOre_WaitPickArmCommandTargetStable(AutoOre_t *ctrl,
                                                    uint32_t now_ms) {
  if (!AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
    ctrl->arm_target_stable_since_ms = 0u;
    return false;
  }

  const float velocity_threshold =
      (ctrl->param.arm_arrive_velocity_threshold_rad_s > 0.0f)
          ? ctrl->param.arm_arrive_velocity_threshold_rad_s
          : 0.25f;
  if (!isfinite(ctrl->feedback.arm_joint1_velocity_rad_s) ||
      AutoOre_AbsFloat(ctrl->feedback.arm_joint1_velocity_rad_s) >
      velocity_threshold) {
    ctrl->arm_target_stable_since_ms = 0u;
    return false;
  }

  const uint32_t stable_ms =
      (ctrl->param.pick_arm_arrive_stable_ms > 0u)
          ? ctrl->param.pick_arm_arrive_stable_ms
          : 100u;
  if (ctrl->arm_target_stable_since_ms == 0u) {
    ctrl->arm_target_stable_since_ms = now_ms;
    return stable_ms == 0u;
  }
  return (now_ms - ctrl->arm_target_stable_since_ms) >= stable_ms;
}

static bool AutoOre_FusedPhoto1ProtectsUnstableArm(
    AutoOre_t *ctrl, bool arm_stable) {
  if (ctrl == 0) {
    return false;
  }
  if (arm_stable) {
    ctrl->fused_photo1_arm_protection_latched = false;
    return false;
  }
  if (ctrl->feedback.photo_transfer_valid &&
      ctrl->feedback.pe13_photo1_triggered) {
    ctrl->fused_photo1_arm_protection_latched = true;
  }
  return ctrl->fused_photo1_arm_protection_latched;
}

static void AutoOre_CommandFusedApproachWithArmProtection(
    AutoOre_t *ctrl, const AutoOre_FusedParam_t *fused, bool arm_stable) {
  if (ctrl == 0 || fused == 0) {
    return;
  }
  if (AutoOre_FusedPhoto1ProtectsUnstableArm(ctrl, arm_stable)) {
    AutoOre_CommandChassisHold(ctrl);
    return;
  }
  AutoOre_CommandChassisMoveYawRate(ctrl, fused->precontact_vx_mps,
                                    ctrl->feedback.yaw_rate_cmd_rad_s);
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
  return ctrl != 0 && ctrl->ore_store_cmd_valid &&
         AutoOre_AbsFloat(ctrl->ore_store_cmd.platform_target_rad -
                          ctrl->feedback.ore_store_platform_position_rad) <=
             threshold_rad;
}

static bool AutoOre_ChamberWaitPlatformReady(const AutoOre_t *ctrl,
                                             float threshold_rad) {
  if (ctrl != 0 && AutoOre_ActionIsReleaseLike(ctrl->action)) {
    return true;
  }
  return AutoOre_OreStorePlatformAtTarget(ctrl, threshold_rad);
}

static void AutoOre_SetStepPhase(AutoOre_t *ctrl, uint8_t phase,
                                 uint32_t now_ms) {
  ctrl->step_phase = phase;
  ctrl->step_condition_met = false;
  ctrl->step_condition_time_ms = now_ms;
}

static void AutoOre_SetFusedStepSidePhase(AutoOre_t *ctrl, uint8_t phase) {
  ctrl->step_phase = phase;
  ctrl->distance_latch_valid = false;
  ctrl->wheel_delta_rad = 0.0f;
  ctrl->target_wheel_delta_rad = 0.0f;
}

static void AutoOre_ClearOutputs(AutoOre_t *ctrl) {
  ctrl->arm_cmd_valid = false;
  ctrl->ore_store_cmd_valid = false;
  ctrl->pole_cmd_valid = false;
  ctrl->chassis_cmd_valid = false;
}

static uint8_t AutoOre_BuildOwnedResourceMask(const AutoOre_t *ctrl) {
  if (ctrl == 0 || ctrl->state != AUTO_ORE_STATE_RUNNING) {
    return AUTO_ORE_RESOURCE_NONE;
  }

  uint8_t mask = ctrl->reserved_resource_mask;
  if (ctrl->chassis_cmd_valid) {
    mask |= AUTO_ORE_RESOURCE_CHASSIS;
  }
  if (ctrl->pole_cmd_valid) {
    mask |= AUTO_ORE_RESOURCE_POLE;
  }
  if (ctrl->arm_cmd_valid) {
    mask |= AUTO_ORE_RESOURCE_ARM | AUTO_ORE_RESOURCE_SHARED_VALVE;
  }
  if (ctrl->ore_store_cmd_valid) {
    mask |= AUTO_ORE_RESOURCE_STORE | AUTO_ORE_RESOURCE_SHARED_VALVE;
  }
  return mask;
}

static void AutoOre_PublishOutputSnapshot(AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return;
  }

  const uint8_t current = (uint8_t)(ctrl->output_snapshot_index & 1u);
  const uint8_t next = (uint8_t)(current ^ 1u);
  AutoOre_OutputSnapshot_t *snapshot = &ctrl->output_snapshot[next];
  const uint32_t generation =
      ctrl->output_snapshot[current].generation + 1u;

  memset(snapshot, 0, sizeof(*snapshot));
  snapshot->generation = generation;
  snapshot->owned_resource_mask = AutoOre_BuildOwnedResourceMask(ctrl);
  snapshot->valid_resource_mask = AUTO_ORE_RESOURCE_NONE;
  if (ctrl->arm_cmd_valid) {
    snapshot->valid_resource_mask |=
        AUTO_ORE_RESOURCE_ARM | AUTO_ORE_RESOURCE_SHARED_VALVE;
    snapshot->arm_cmd = ctrl->arm_cmd;
  }
  if (ctrl->ore_store_cmd_valid) {
    snapshot->valid_resource_mask |=
        AUTO_ORE_RESOURCE_STORE | AUTO_ORE_RESOURCE_SHARED_VALVE;
    snapshot->ore_store_cmd = ctrl->ore_store_cmd;
  }
  if (ctrl->pole_cmd_valid) {
    snapshot->valid_resource_mask |= AUTO_ORE_RESOURCE_POLE;
    snapshot->pole_cmd = ctrl->pole_cmd;
  }
  if (ctrl->chassis_cmd_valid) {
    snapshot->valid_resource_mask |= AUTO_ORE_RESOURCE_CHASSIS;
    snapshot->chassis_cmd = ctrl->chassis_cmd;
  }

  __atomic_thread_fence(__ATOMIC_RELEASE);
  ctrl->output_snapshot_index = next;
}

static bool AutoOre_CommandArm(AutoOre_t *ctrl, ArmSimple_BehaviorPoint_t point,
                 Suction_State_t suction,
                 const AutoOre_ArmSpeedLimit_t *speed) {
  const float joint1_max_vel_rad_s =
    (speed != 0) ? speed->joint1_max_vel_rad_s : 0.0f;
  const float joint2_max_vel_rad_s =
    (speed != 0) ? speed->joint2_max_vel_rad_s : 0.0f;
  const float joint1_max_accel_rad_s2 =
    (speed != 0) ? speed->joint1_max_accel_rad_s2 : 0.0f;
  const float joint2_max_accel_rad_s2 =
    (speed != 0) ? speed->joint2_max_accel_rad_s2 : 0.0f;
  ctrl->arm_cmd_valid = ArmSimple_MakeBehaviorCommandWithSpeed(
    ctrl->param.arm_param, point, suction, joint1_max_vel_rad_s,
    joint2_max_vel_rad_s, joint1_max_accel_rad_s2,
    joint2_max_accel_rad_s2, &ctrl->arm_cmd);
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

static bool AutoOre_CommandOreStoreWithVelocity(
    AutoOre_t *ctrl, OreStore_TransformPoint_t transform,
    bool cylinder_closed, float platform_velocity_rad_s) {
  if (!AutoOre_CommandOreStore(ctrl, transform, cylinder_closed)) {
    return false;
  }
  ctrl->ore_store_cmd.platform_velocity_rad_s = platform_velocity_rad_s;
  return true;
}

static bool AutoOre_UpdateStoreLowShake(AutoOre_t *ctrl, uint8_t *phase,
                                        bool *done) {
  if (ctrl == 0 || phase == 0 || done == 0 ||
      ctrl->param.ore_store_param == 0) {
    return false;
  }

  *done = false;
  const uint8_t cycles = AutoOre_StoreLowShakeCycles(ctrl);
  const uint8_t phase_count = (uint8_t)(cycles * 2u);
  const float standby_rad =
      ctrl->param.ore_store_param->preset
          .transform_position_rad[ORE_STORE_TRANSFORM_STANDBY];
  const float lift_rad =
      ctrl->param.ore_store_param->preset
          .transform_position_rad[ORE_STORE_TRANSFORM_LIFT];
  const float travel_rad = AutoOre_AbsFloat(lift_rad - standby_rad);

  if (*phase >= phase_count || travel_rad <= 0.0001f) {
    if (!AutoOre_CommandOreStoreWithVelocity(
            ctrl, ORE_STORE_TRANSFORM_STANDBY, false,
            AutoOre_StoreLowShakeVelocityRadS(ctrl))) {
      return false;
    }
    *done = AutoOre_OreStorePlatformAtTarget(
        ctrl, ctrl->param.ore_store_arrive_threshold_rad);
    return true;
  }

  float amplitude_rad = AutoOre_StoreLowShakeAmplitudeRad(ctrl);
  if (amplitude_rad > travel_rad) {
    amplitude_rad = travel_rad;
  }
  const bool move_up = ((*phase & 1u) == 0u);
  const float direction = (lift_rad >= standby_rad) ? 1.0f : -1.0f;
  const float target_rad =
      move_up ? standby_rad + direction * amplitude_rad : standby_rad;

  if (!AutoOre_CommandOreStoreWithVelocity(
          ctrl, ORE_STORE_TRANSFORM_STANDBY, false,
          AutoOre_StoreLowShakeVelocityRadS(ctrl))) {
    return false;
  }
  ctrl->ore_store_cmd.platform_target_rad = target_rad;

  if (AutoOre_OreStorePlatformAtTarget(
          ctrl, ctrl->param.ore_store_arrive_threshold_rad)) {
    (*phase)++;
    *done = (*phase >= phase_count);
  }
  return true;
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

static void AutoOre_ApplyPoleCommandLimitsFromTemplate(
    AutoOre_t *ctrl, const AutoCtrl_TemplateParam_t *template_param) {
  if (ctrl == 0 || template_param == 0) {
    return;
  }

  ctrl->pole_cmd.auto_lift_speed[0] = template_param->pole_all_extend_speed;
  ctrl->pole_cmd.auto_lift_speed[1] = template_param->pole_all_extend_speed;
  ctrl->pole_cmd.auto_lift_accel[0] = template_param->pole_all_extend_accel;
  ctrl->pole_cmd.auto_lift_accel[1] = template_param->pole_all_extend_accel;
  ctrl->pole_cmd.disable_lift_accel =
      template_param->pole_all_extend_accel < 0.0f;
}

static bool AutoOre_CommandReleasePoleTarget(AutoOre_t *ctrl) {
  if (ctrl == 0 || ctrl->param.pole_param == 0) {
    return false;
  }

  if (!AutoOre_CommandPoleTarget(
          ctrl, ctrl->param.pole_param->preset.ore_release_target)) {
    return false;
  }

  const float speed = ctrl->param.pole_param->preset.ore_release_speed;
  if (speed > 0.0f) {
    ctrl->pole_cmd.auto_lift_speed[0] = speed;
    ctrl->pole_cmd.auto_lift_speed[1] = speed;
  }
  const float accel = ctrl->param.pole_param->preset.ore_release_accel;
  if (accel > 0.0f) {
    ctrl->pole_cmd.auto_lift_accel[0] = accel;
    ctrl->pole_cmd.auto_lift_accel[1] = accel;
  }
  return true;
}

static bool AutoOre_ReleasePoleAtTarget(const AutoOre_t *ctrl) {
  if (ctrl == 0 || ctrl->param.pole_param == 0) {
    return false;
  }

  const float *target = ctrl->param.pole_param->preset.ore_release_target;
  const float threshold = (ctrl->param.pole_arrive_threshold_rad > 0.0f)
                              ? ctrl->param.pole_arrive_threshold_rad
                              : 0.30f;
  return AutoOre_AbsFloat(ctrl->feedback.pole_front_lift_rad - target[0]) <=
             threshold &&
         AutoOre_AbsFloat(ctrl->feedback.pole_rear_lift_rad - target[1]) <=
             threshold;
}

static void AutoOre_CommandChassisMove(AutoOre_t *ctrl, float vx_mps) {
  memset(&ctrl->chassis_cmd, 0, sizeof(ctrl->chassis_cmd));
  ctrl->chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
  ctrl->chassis_cmd.ctrl_vec.vx = vx_mps;
  ctrl->chassis_cmd.ctrl_vec.vy =
      (ctrl->feedback.yaw_source == AUTO_CTRL_YAW_SOURCE_PC)
          ? ctrl->feedback.lateral_velocity_cmd_mps
          : 0.0f;
  ctrl->chassis_cmd.ctrl_vec.wz = AutoOre_SelectPrealignWz(ctrl);
  ctrl->chassis_cmd_valid = true;
}

static void AutoOre_CommandChassisMoveYawRate(AutoOre_t *ctrl, float vx_mps,
                                              float wz_rad_s) {
  AutoOre_CommandChassisMove(ctrl, vx_mps);
  if (!ctrl->prealign_yaw_target_valid) {
    ctrl->chassis_cmd.ctrl_vec.wz = wz_rad_s;
  }
}

static void AutoOre_CommandChassisHold(AutoOre_t *ctrl) {
  AutoOre_CommandChassisMove(ctrl, 0.0f);
}

static void AutoOre_CommandChassisZeroVector(AutoOre_t *ctrl) {
  memset(&ctrl->chassis_cmd, 0, sizeof(ctrl->chassis_cmd));
  ctrl->chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
  ctrl->chassis_cmd_valid = true;
}

static void AutoOre_ReleaseChassisCommand(AutoOre_t *ctrl) {
  memset(&ctrl->chassis_cmd, 0, sizeof(ctrl->chassis_cmd));
  ctrl->chassis_cmd.mode = CHASSIS_MODE_RELAX;
  ctrl->chassis_cmd_valid = false;
}

static void AutoOre_ReleasePoleCommand(AutoOre_t *ctrl) {
  memset(&ctrl->pole_cmd, 0, sizeof(ctrl->pole_cmd));
  ctrl->pole_cmd.mode = POLE_MODE_RELAX;
  ctrl->pole_cmd_valid = false;
}

static float AutoOre_PrealignYawToleranceRad(const AutoOre_t *ctrl) {
  return (ctrl != 0 && ctrl->param.prealign_yaw_tolerance_rad > 0.0f)
             ? ctrl->param.prealign_yaw_tolerance_rad
             : 0.0f;
}

static void AutoOre_UpdatePrealignYawError(AutoOre_t *ctrl) {
  if (ctrl == 0 || !ctrl->prealign_yaw_target_valid) {
    return;
  }
  ctrl->prealign_yaw_error_rad =
      AutoCtrlMath_GetYawErrorRad(ctrl->prealign_target_yaw_rad,
                                  ctrl->feedback.yaw_auto_rad);
}

static float AutoOre_SelectPrealignWz(AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return 0.0f;
  }
  if (!ctrl->prealign_yaw_target_valid) {
    ctrl->prealign_yaw_error_rad = 0.0f;
    return 0.0f;
  }
  AutoOre_UpdatePrealignYawError(ctrl);
  if (ctrl->feedback.yaw_source == AUTO_CTRL_YAW_SOURCE_PC) {
    return ctrl->feedback.yaw_rate_cmd_rad_s;
  }

#if !AUTO_CTRL_STM32_YAW_WZ_ENABLE
  return 0.0f;
#endif

  Config_RobotParam_t *robot_param = Config_GetRobotParam();
  if (robot_param == 0) {
    return 0.0f;
  }

  const float kp = robot_param->auto_ctrl_param.common.prealign_kp;
  const float limit = robot_param->auto_ctrl_param.common.prealign_wz_limit;
  float wz = ctrl->prealign_yaw_error_rad * kp;
  if (limit > 0.0f) {
    if (wz > limit) {
      wz = limit;
    } else if (wz < -limit) {
      wz = -limit;
    }
  }
  return wz;
}

static bool AutoOre_RunPickPrealign(AutoOre_t *ctrl, uint32_t now_ms) {
  if (ctrl == 0) {
    return false;
  }
  if (!ctrl->prealign_yaw_target_valid) {
    ctrl->prealign_target_yaw_rad =
        (ctrl->feedback.yaw_source == AUTO_CTRL_YAW_SOURCE_PC)
            ? ctrl->feedback.yaw_auto_rad
            : AutoCtrlMath_NearestCardinalYawRad(ctrl->feedback.yaw_auto_rad);
    ctrl->prealign_yaw_target_valid = true;
  }

  AutoOre_CommandChassisMoveYawRate(ctrl, 0.0f, AutoOre_SelectPrealignWz(ctrl));

  const bool yaw_ready =
      AutoOre_AbsFloat(ctrl->prealign_yaw_error_rad) <=
      AutoOre_PrealignYawToleranceRad(ctrl);
  return AutoOre_WaitConditionThenDelay(
      ctrl, now_ms, yaw_ready, AutoOre_FusedPrealignStableMs(ctrl));
}

static void AutoOre_ResetDistanceGate(AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return;
  }
  ctrl->distance_latch_valid = false;
  ctrl->wheel_delta_rad = 0.0f;
  ctrl->target_wheel_delta_rad = 0.0f;
}

static bool AutoOre_WheelDeltaMoveReached(AutoOre_t *ctrl,
                                          float target_wheel_delta_rad) {
  if (ctrl == 0 || target_wheel_delta_rad <= 0.0f) {
    return false;
  }
  ctrl->target_wheel_delta_rad = target_wheel_delta_rad;

  if (!ctrl->distance_latch_valid) {
    for (uint8_t i = 0u; i < 4u; ++i) {
      ctrl->distance_start_wheel_rad[i] = ctrl->feedback.wheel_position_rad[i];
    }
    ctrl->wheel_delta_rad = 0.0f;
    ctrl->distance_latch_valid = true;
    return false;
  }

  float wheel_delta_abs_sum_rad = 0.0f;
  for (uint8_t i = 0u; i < 4u; ++i) {
    wheel_delta_abs_sum_rad += AutoOre_AbsFloat(
        ctrl->feedback.wheel_position_rad[i] -
        ctrl->distance_start_wheel_rad[i]);
  }
  ctrl->wheel_delta_rad = wheel_delta_abs_sum_rad * 0.25f;
  return ctrl->wheel_delta_rad >= target_wheel_delta_rad;
}

static void AutoOre_FailInvalidParam(AutoOre_t *ctrl) {
  ctrl->state = AUTO_ORE_STATE_FAIL;
  ctrl->result = AUTO_ORE_RESULT_FAIL;
  ctrl->fault = AUTO_ORE_FAULT_INVALID_PARAM;
}

static void AutoOre_FailInvalidOccupancy(AutoOre_t *ctrl) {
  ctrl->state = AUTO_ORE_STATE_FAIL;
  ctrl->result = AUTO_ORE_RESULT_FAIL;
  ctrl->fault = AUTO_ORE_FAULT_INVALID_OCCUPANCY;
}

static void AutoOre_FailSetupInvalidParam(AutoOre_t *ctrl) {
  AutoOre_AddFailureMask(ctrl, AUTO_ORE_FAILURE_SETUP);
  AutoOre_FailInvalidParam(ctrl);
}

static void AutoOre_FailPickInvalidParam(AutoOre_t *ctrl) {
  AutoOre_AddFailureMask(ctrl, AUTO_ORE_FAILURE_PICK_ORE);
  AutoOre_FailInvalidParam(ctrl);
}

static void AutoOre_FailStoreInvalidParam(AutoOre_t *ctrl) {
  AutoOre_AddFailureMask(ctrl, AUTO_ORE_FAILURE_STORE_ORE);
  AutoOre_FailInvalidParam(ctrl);
}

static void AutoOre_FailStoreInvalidOccupancy(AutoOre_t *ctrl) {
  AutoOre_AddFailureMask(ctrl, AUTO_ORE_FAILURE_STORE_ORE);
  AutoOre_FailInvalidOccupancy(ctrl);
}

static void AutoOre_FailReleaseInvalidOccupancy(AutoOre_t *ctrl) {
  AutoOre_AddFailureMask(ctrl, AUTO_ORE_FAILURE_RELEASE_ORE);
  AutoOre_FailInvalidOccupancy(ctrl);
}

static void AutoOre_FailStepInvalidParam(AutoOre_t *ctrl) {
  AutoOre_AddFailureMask(ctrl, AUTO_ORE_FAILURE_STEP);
  AutoOre_FailInvalidParam(ctrl);
}

static bool AutoOre_IsPositionValid(AutoOre_Position_t position) {
  return position == AUTO_ORE_POSITION_TRANSFORM_LOW ||
         position == AUTO_ORE_POSITION_TRANSFORM_HIGH ||
         position == AUTO_ORE_POSITION_ARM;
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

static void AutoOre_FinishChamberSegment(AutoOre_t *ctrl, uint32_t now_ms) {
  AutoOre_FinishChamberSuccess(ctrl);
  if (AutoOre_ActionIsReleaseLike(ctrl->action)) {
    ctrl->active_position = AUTO_ORE_POSITION_ARM;
    ctrl->step_index = 0u;
    ctrl->step_phase = 0u;
    ctrl->step_entered = false;
    ctrl->step_condition_met = false;
    ctrl->step_condition_time_ms = now_ms;
    ctrl->step_enter_time_ms = now_ms;
    return;
  }
  AutoOre_FinishSuccess(ctrl);
}

static void AutoOre_FinishSuccess(AutoOre_t *ctrl) {
  const AutoOre_Fault_t fault_before_finish = ctrl->fault;
  if (ctrl->action == AUTO_ORE_ACTION_STORE) {
    AutoOre_SetActivePositionHasOre(ctrl, true);
  } else if (AutoOre_ActionIsReleaseLike(ctrl->action) &&
             (!AutoOre_ActionIsZone3IrRelease(ctrl->action) ||
              ctrl->release_ir_release_started)) {
    AutoOre_SetActivePositionHasOre(ctrl, false);
  } else if (ctrl->action == AUTO_ORE_ACTION_CHAMBER) {
    AutoOre_FinishChamberSuccess(ctrl);
  } else if (ctrl->action == AUTO_ORE_ACTION_PICK_POS_400 ||
             ctrl->action == AUTO_ORE_ACTION_PICK_POS_200 ||
             ctrl->action == AUTO_ORE_ACTION_PICK_NEG_200) {
    AutoOre_SetArmHasOre(ctrl, true);
  } else if (AutoOre_ActionIsPickStoreFused(ctrl->action)) {
    /* Preserve per-branch completion events until the next action starts. */
  } else if (AutoOre_ActionIsFused(ctrl->action)) {
    ctrl->step_ctrl_active = false;
    ctrl->step_ctrl_started = false;
  }
  ctrl->state = AUTO_ORE_STATE_SUCCESS;
  ctrl->result = AUTO_ORE_RESULT_SUCCESS;
  ctrl->fault = (fault_before_finish == AUTO_ORE_FAULT_INVALID_OCCUPANCY)
                    ? fault_before_finish
                    : AUTO_ORE_FAULT_NONE;
  ctrl->failure_mask = AUTO_ORE_FAILURE_NONE;
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
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_MID_WAIT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      (void)AutoOre_WaitAll(ctrl, now_ms);
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
          SUCTION_ON, &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_MID_WAIT,
                                   false)) {
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
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
          SUCTION_OFF, &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_MID_WAIT, true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AutoOre_StoreCylinderCloseMs(ctrl))) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 3:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
          SUCTION_OFF, &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitOreStoreCommandTarget(ctrl, now_ms) &&
          AutoOre_StepElapsed(ctrl, now_ms) >=
              AutoOre_StoreArmSuctionOffMs(ctrl)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 4:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
          SUCTION_OFF, &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AutoOre_StoreCylinderOpenMs(ctrl))) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 5:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_WAIT_STORE_ORE,
          SUCTION_OFF, &ctrl->param.arm_speed.store_wait) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 6:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_WAIT_STORE_ORE,
          SUCTION_OFF, &ctrl->param.arm_speed.store_wait) ||
          !AutoOre_CommandOreStoreWithVelocity(
              ctrl, ORE_STORE_TRANSFORM_STANDBY, false,
              AutoOre_StoreLowReturnPlannedVelocityRadS(ctrl, now_ms))) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitOreStoreCommandTarget(ctrl, now_ms)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 7:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY,
          SUCTION_OFF, &ctrl->param.arm_speed.store_standby) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 8: {
      AutoOre_EnterStep(ctrl, now_ms);
      bool shake_done = false;
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY, SUCTION_OFF,
                              &ctrl->param.arm_speed.store_standby) ||
          !AutoOre_UpdateStoreLowShake(ctrl, &ctrl->step_phase,
                                       &shake_done)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (shake_done) {
        AutoOre_FinishSuccess(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    }
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
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY,
          SUCTION_OFF, &ctrl->param.arm_speed.store_standby) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, true)) {
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

static bool AutoOre_CommandReleasePoleDuringChamber(AutoOre_t *ctrl) {
  if (!AutoOre_ActionIsReleaseLike(ctrl->action)) {
    return true;
  }
  if (AutoOre_ActionIsZone3IrRelease(ctrl->action)) {
    AutoOre_CommandChassisZeroVector(ctrl);
  }
  return AutoOre_CommandReleasePoleTarget(ctrl);
}

static void AutoOre_RunReleaseArm(AutoOre_t *ctrl, uint32_t now_ms) {
  AutoOre_CommandChassisZeroVector(ctrl);

  if (ctrl->release_ir_abort_pending) {
    AutoOre_RunIrReleaseAbortRecovery(ctrl, now_ms);
    return;
  }

  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      const bool zone3_ir_release_step0 =
          AutoOre_ActionIsZone3IrRelease(ctrl->action);
      if (!AutoOre_CommandReleasePoleTarget(ctrl) ||
          !AutoOre_CommandArm(ctrl,
              zone3_ir_release_step0 ? ARM_SIMPLE_BEHAVIOR_WAIT_RELEASE_ORE
                                     : ARM_SIMPLE_BEHAVIOR_VERTICAL,
              SUCTION_ON,
              &ctrl->param.arm_speed.release_wait) ||
          !AutoOre_CommandReleaseOreStoreHold(ctrl,
                                              ORE_STORE_TRANSFORM_STANDBY,
                                              true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_StepElapsed(ctrl, now_ms) > 0u &&
          AutoOre_ReleasePoleAtTarget(ctrl) &&
          AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      const bool zone3_ir_release_step1 =
          AutoOre_ActionIsZone3IrRelease(ctrl->action);
      if (!AutoOre_CommandReleasePoleTarget(ctrl) ||
          !AutoOre_CommandArm(ctrl,
              zone3_ir_release_step1 ? ARM_SIMPLE_BEHAVIOR_WAIT_RELEASE_ORE
                                     : ARM_SIMPLE_BEHAVIOR_VERTICAL,
              SUCTION_ON,
              &ctrl->param.arm_speed.release_wait) ||
          !AutoOre_CommandReleaseOreStoreHold(ctrl,
                                              ORE_STORE_TRANSFORM_STANDBY,
                                              true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      const bool arm_ready = AutoOre_WaitConditionThenDelay(
          ctrl, now_ms, ctrl->feedback.arm_at_target,
          AutoOre_ReleaseWaitMs(ctrl));
      const bool ir_release =
          AutoOre_ActionUsesIrReleaseLiftDetect(ctrl->action);
      if (AutoOre_ActionIsZone3IrRelease(ctrl->action)) {
        const bool local_ready = arm_ready &&
                                 AutoOre_ReleasePoleAtTarget(ctrl) &&
                                 ctrl->feedback.ore_store_all_at_target;
        if (!local_ready) {
          ctrl->release_ir_wait_ready = false;
          ctrl->release_ir_wait_ready_time_ms = 0u;
          (void)AutoOre_CheckTimeout(ctrl, now_ms);
          return;
        }
        if (!ctrl->release_ir_wait_ready) {
          ctrl->release_ir_wait_ready = true;
          ctrl->release_ir_wait_ready_time_ms = now_ms;
        }
        const bool allow_is_new =
            (int32_t)(ctrl->feedback.release_lift_ir_allow_rx_ms -
                      ctrl->release_ir_wait_ready_time_ms) >= 0;
        if (ctrl->feedback.release_lift_ir_claw_open && allow_is_new) {
          ctrl->release_ir_allow_latched = true;
        }
        if (ctrl->release_ir_allow_latched) {
          ctrl->release_ir_release_started = true;
          AutoOre_NextStep(ctrl);
        }
        return;
      }
      const bool grid_check_done = arm_ready && AutoOre_ReleasePoleAtTarget(ctrl)
                         ? AutoOre_UpdateReleaseGridCheck(
                           ctrl, now_ms, ir_release)
                                       : false;
      if (!AutoOre_ActionUsesReleaseLiftDetect(ctrl->action)) {
        if (grid_check_done) {
          if (ctrl->release_grid_has_ore) {
            AutoOre_FailReleaseInvalidOccupancy(ctrl);
            return;
          }
          if (AutoOre_ActionIsReleaseStep1(ctrl->action)) {
            ctrl->fused_step_done = true;
            ctrl->fused_store_done = true;
            AutoOre_CommandChassisZeroVector(ctrl);
            return;
          }
          AutoOre_NextStep(ctrl);
        } else {
          (void)AutoOre_CheckTimeout(ctrl, now_ms);
        }
        return;
      }
      bool lift_ready = false;
      if (ir_release) {
        if (ctrl->feedback.release_lift_ir_claw_open) {
          ctrl->release_ir_allow_latched = true;
        }
        lift_ready = ctrl->release_ir_allow_latched;
        if (lift_ready && !ctrl->release_lift_detected) {
          ctrl->release_lift_observer_active = true;
          ctrl->release_lift_detected = true;
          ctrl->release_lift_detect_time_ms = now_ms;
        }
      } else {
        lift_ready = AutoOre_UpdateReleaseLiftObserver(ctrl, now_ms);
      }
      const bool lift_settled = lift_ready &&
          (now_ms - ctrl->release_lift_detect_time_ms) >=
              AutoOre_ReleaseLiftDetectSettleMs(ctrl);
      if (lift_settled) {
        if (grid_check_done) {
          if (!ir_release && ctrl->release_grid_has_ore) {
            AutoOre_FailReleaseInvalidOccupancy(ctrl);
            return;
          }
          if (AutoOre_ActionIsReleaseStep1(ctrl->action)) {
            ctrl->fused_step_done = true;
            ctrl->fused_store_done = true;
            AutoOre_CommandChassisZeroVector(ctrl);
            return;
          }
          AutoOre_NextStep(ctrl);
        } else {
          if (!ir_release) {
            (void)AutoOre_CheckTimeout(ctrl, now_ms);
          }
        }
      } else if (ctrl->release_lift_observer_active &&
                 (now_ms - ctrl->release_lift_observer_start_ms) >=
                     AutoOre_ReleaseLiftDetectTimeoutMs(ctrl)) {
        ctrl->state = AUTO_ORE_STATE_FAIL;
        ctrl->result = AUTO_ORE_RESULT_FAIL;
        ctrl->fault = AUTO_ORE_FAULT_TIMEOUT;
        ctrl->failure_mask |= AUTO_ORE_FAILURE_RELEASE_ORE;
      } else if (!arm_ready) {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandReleasePoleTarget(ctrl) ||
          !AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_WAIT_RELEASE_ORE,
              SUCTION_ON,
              &ctrl->param.arm_speed.release_wait) ||
          !AutoOre_CommandReleaseOreStoreHold(ctrl,
                                              ORE_STORE_TRANSFORM_STANDBY,
                                              true)) {
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
      if (!AutoOre_CommandReleasePoleTarget(ctrl) ||
          !AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_RELEASE_ORE_ASSIST,
              SUCTION_ON,
              &ctrl->param.arm_speed.release_assist) ||
          !AutoOre_CommandReleaseOreStoreHold(ctrl,
                                              ORE_STORE_TRANSFORM_STANDBY,
                                              true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 4:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandReleasePoleTarget(ctrl) ||
          !AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_RELEASE_ORE,
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
    case 5:
      AutoOre_EnterStep(ctrl, now_ms);
      if (AutoOre_ActionIsZone3IrRelease(ctrl->action)) {
        ctrl->release_ir_suction_released = true;
      }
      if (!AutoOre_CommandReleasePoleTarget(ctrl) ||
          !AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_RELEASE_ORE,
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
    case 6:
      AutoOre_EnterStep(ctrl, now_ms);
      const bool zone3_ir_release_step6 =
          AutoOre_ActionIsZone3IrRelease(ctrl->action);
      if (!AutoOre_CommandReleasePoleTarget(ctrl) ||
          !AutoOre_CommandArm(ctrl,
              zone3_ir_release_step6 ? ARM_SIMPLE_BEHAVIOR_WAIT_RELEASE_ORE
                                     : ARM_SIMPLE_BEHAVIOR_STANDBY,
              SUCTION_OFF,
              zone3_ir_release_step6 ? &ctrl->param.arm_speed.release_wait
                                     : &ctrl->param.arm_speed.release_standby) ||
          !AutoOre_CommandReleaseOreStoreHold(ctrl,
                                              ORE_STORE_TRANSFORM_STANDBY,
                                              true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
        if (!zone3_ir_release_step6) {
          AutoOre_FinishSuccess(ctrl);
        }
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
  if (ctrl->release_ir_abort_pending) {
    AutoOre_RunIrReleaseAbortRecovery(ctrl, now_ms);
    return;
  }
  if (!AutoOre_CommandReleasePoleDuringChamber(ctrl)) {
    AutoOre_FailInvalidParam(ctrl);
    return;
  }

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
      if (AutoOre_WaitConditionThenDelay(
              ctrl, now_ms,
              AutoOre_ArmCommandAtTarget(ctrl) &&
              AutoOre_ChamberWaitPlatformReady(
                      ctrl, AutoOre_OreStoreArriveThresholdRad(ctrl)),
              AutoOre_ChamberArmSettleMs(ctrl))) {
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
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY,
                                   ctrl->step_phase == 0u)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->step_phase == 0u) {
        if (AutoOre_WaitConditionThenDelay(
                ctrl, now_ms, AutoOre_ArmCommandAtTarget(ctrl),
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
        AutoOre_FinishChamberSegment(ctrl, now_ms);
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
  if (ctrl->release_ir_abort_pending) {
    AutoOre_RunIrReleaseAbortRecovery(ctrl, now_ms);
    return;
  }
  if (!AutoOre_CommandReleasePoleDuringChamber(ctrl)) {
    AutoOre_FailInvalidParam(ctrl);
    return;
  }

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
            AutoOre_ArmCommandAtTarget(ctrl) &&
            AutoOre_ChamberWaitPlatformReady(
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
              ctrl, now_ms, AutoOre_ArmCommandAtTarget(ctrl),
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
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_CHAMBER_ORE,
              SUCTION_ON,
              &ctrl->param.arm_speed.chamber_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitOreStoreCommandTarget(ctrl, now_ms)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 4:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY,
              SUCTION_ON,
              &ctrl->param.arm_speed.chamber_standby) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
        AutoOre_FinishChamberSegment(ctrl, now_ms);
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

static bool AutoOre_ActionIsRecoverStore(AutoOre_Action_t action) {
  return action == AUTO_ORE_ACTION_RECOVER_STORE;
}

static bool AutoOre_ActionIsFused(AutoOre_Action_t action) {
  return action == AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD ||
         action == AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD ||
         action == AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD ||
         AutoOre_ActionIsDropStoreFused(action);
}

static void AutoOre_InitBranch(AutoOre_BranchContext_t *branch,
                               uint8_t segment_mask) {
  memset(branch, 0, sizeof(*branch));
  branch->state = AUTO_ORE_BRANCH_PENDING;
  branch->segment_mask = segment_mask;
}

static void AutoOre_ResetParallelContext(AutoOre_t *ctrl) {
  memset(&ctrl->parallel, 0, sizeof(ctrl->parallel));
  if (!AutoOre_ActionIsFused(ctrl->action) &&
      !AutoOre_ActionIsPickStoreFused(ctrl->action)) {
    return;
  }
  AutoOre_InitBranch(&ctrl->parallel.handoff, AUTO_ORE_SEGMENT_HANDOFF);
  AutoOre_InitBranch(&ctrl->parallel.store, AUTO_ORE_SEGMENT_STORE);
  AutoOre_InitBranch(&ctrl->parallel.step, AUTO_ORE_SEGMENT_STEP);
}

static void AutoOre_BranchStart(AutoOre_BranchContext_t *branch,
                                uint8_t resources, uint32_t now_ms) {
  if (branch->state == AUTO_ORE_BRANCH_PENDING) {
    branch->state = AUTO_ORE_BRANCH_RUNNING;
    branch->enter_time_ms = now_ms;
  }
  if (branch->state == AUTO_ORE_BRANCH_RUNNING) {
    branch->resource_mask = resources;
  }
}

static void AutoOre_BranchSucceed(AutoOre_BranchContext_t *branch,
                                  uint32_t now_ms) {
  if (branch->state == AUTO_ORE_BRANCH_PENDING ||
      branch->state == AUTO_ORE_BRANCH_RUNNING) {
    branch->state = AUTO_ORE_BRANCH_SUCCEEDED;
    branch->resource_mask = AUTO_ORE_RESOURCE_NONE;
    branch->finish_time_ms = now_ms;
  }
}

static void AutoOre_BranchFail(AutoOre_BranchContext_t *branch,
                               uint16_t failure_mask, uint32_t now_ms) {
  if (branch->state == AUTO_ORE_BRANCH_SUCCEEDED) {
    return;
  }
  branch->state = AUTO_ORE_BRANCH_FAILED;
  branch->failure_mask |= failure_mask;
  branch->resource_mask = AUTO_ORE_RESOURCE_NONE;
  branch->finish_time_ms = now_ms;
}

static void AutoOre_BranchCancel(AutoOre_BranchContext_t *branch,
                                 uint32_t now_ms) {
  if (branch->state == AUTO_ORE_BRANCH_PENDING ||
      branch->state == AUTO_ORE_BRANCH_RUNNING) {
    branch->state = AUTO_ORE_BRANCH_CANCELLED;
    branch->resource_mask = AUTO_ORE_RESOURCE_NONE;
    branch->finish_time_ms = now_ms;
  }
}

static void AutoOre_SyncParallelContext(AutoOre_t *ctrl, uint32_t now_ms) {
  if (!AutoOre_ActionIsFused(ctrl->action) &&
      !AutoOre_ActionIsPickStoreFused(ctrl->action)) {
    return;
  }

  const uint8_t parallel_step = AutoOre_ActionIsFused(ctrl->action) ? 5u : 4u;
  if (ctrl->step_index >= parallel_step &&
      ctrl->state == AUTO_ORE_STATE_RUNNING) {
    AutoOre_BranchStart(&ctrl->parallel.handoff,
                        AUTO_ORE_RESOURCE_ARM |
                            AUTO_ORE_RESOURCE_SHARED_VALVE,
                        now_ms);
    AutoOre_BranchStart(&ctrl->parallel.store,
                        AUTO_ORE_RESOURCE_ARM | AUTO_ORE_RESOURCE_STORE |
                            AUTO_ORE_RESOURCE_SHARED_VALVE,
                        now_ms);
    AutoOre_BranchStart(&ctrl->parallel.step,
                        AUTO_ORE_RESOURCE_CHASSIS | AUTO_ORE_RESOURCE_POLE,
                        now_ms);
  }

  if (ctrl->fused_pick_done) {
    AutoOre_BranchSucceed(&ctrl->parallel.handoff, now_ms);
  }
  if (ctrl->fused_store_done) {
    AutoOre_BranchSucceed(&ctrl->parallel.store, now_ms);
  }
  if (ctrl->fused_step_done) {
    AutoOre_BranchSucceed(&ctrl->parallel.step, now_ms);
  }

  if (ctrl->state == AUTO_ORE_STATE_FAIL) {
    if ((ctrl->failure_mask & AUTO_ORE_FAILURE_PICK_ORE) != 0u) {
      AutoOre_BranchFail(&ctrl->parallel.handoff,
                         AUTO_ORE_FAILURE_PICK_ORE, now_ms);
    }
    if ((ctrl->failure_mask & AUTO_ORE_FAILURE_STORE_ORE) != 0u) {
      AutoOre_BranchFail(&ctrl->parallel.store,
                         AUTO_ORE_FAILURE_STORE_ORE, now_ms);
    }
    if ((ctrl->failure_mask & AUTO_ORE_FAILURE_STEP) != 0u) {
      AutoOre_BranchFail(&ctrl->parallel.step, AUTO_ORE_FAILURE_STEP, now_ms);
    }
    AutoOre_BranchCancel(&ctrl->parallel.handoff, now_ms);
    AutoOre_BranchCancel(&ctrl->parallel.store, now_ms);
    AutoOre_BranchCancel(&ctrl->parallel.step, now_ms);
  }

  ctrl->parallel.acquired_resource_mask =
      ctrl->parallel.handoff.resource_mask |
      ctrl->parallel.store.resource_mask | ctrl->parallel.step.resource_mask;
}

static const AutoOre_FusedParam_t *AutoOre_FusedParam(const AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return 0;
  }
  switch (ctrl->action) {
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
    case AUTO_ORE_ACTION_STEP_DROP_STORE_ASCEND_200_HEAD:
      return &ctrl->param.fused_step_pick_store_ascend_200_head;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
    case AUTO_ORE_ACTION_STEP_DROP_STORE_DESCEND_200_HEAD:
      return &ctrl->param.fused_step_pick_store_descend_200_head;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
    case AUTO_ORE_ACTION_STEP_DROP_STORE_ASCEND_400_HEAD:
      return &ctrl->param.fused_step_pick_store_ascend_400_head;
    default:
      return 0;
  }
}

static const float *AutoOre_PickPoleTarget(const AutoOre_t *ctrl) {
  if (ctrl == 0 || ctrl->param.pole_param == 0) {
    return 0;
  }
  switch (AutoOre_PickStoreFusedPickAction(ctrl->action)) {
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
  switch (AutoOre_PickStoreFusedPickAction(action)) {
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
      return ARM_SIMPLE_BEHAVIOR_PICK_POS_200;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
      return ARM_SIMPLE_BEHAVIOR_PICK_POS_400;
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

static ArmSimple_BehaviorPoint_t AutoOre_RecoverArmPoint(void) {
  return ARM_SIMPLE_BEHAVIOR_PICK_POS_200;
}

static const float *AutoOre_FusedStepStartPoleTarget(const AutoOre_t *ctrl) {
  if (ctrl == 0 || ctrl->param.pole_param == 0) {
    return 0;
  }
  switch (ctrl->action) {
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
    case AUTO_ORE_ACTION_STEP_DROP_STORE_ASCEND_400_HEAD:
      return ctrl->param.pole_param->preset.step_400_all_extend;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
    case AUTO_ORE_ACTION_STEP_DROP_STORE_ASCEND_200_HEAD:
      return ctrl->param.pole_param->preset.step_200_all_extend;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
    case AUTO_ORE_ACTION_STEP_DROP_STORE_DESCEND_200_HEAD:
      return ctrl->param.pole_param->preset.step_200_descend_all_retract;
    default:
      return 0;
  }
}

static const AutoCtrl_TemplateParam_t *AutoOre_FusedStepTemplateParam(
    const AutoOre_t *ctrl) {
  const Config_RobotParam_t *robot_param = Config_GetRobotParam();
  if (robot_param == 0) {
    return 0;
  }

  switch (AutoOre_FusedStepTemplateId(ctrl)) {
    case AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD:
      return &robot_param->auto_ctrl_param.head_ascend_200;
    case AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD:
      return &robot_param->auto_ctrl_param.head_ascend_400;
    case AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD:
      return &robot_param->auto_ctrl_param.head_descend_200;
    case AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD:
      return &robot_param->auto_ctrl_param.head_descend_400;
    case AUTO_CTRL_TEMPLATE_NONE:
    default:
      return 0;
  }
}

static bool AutoOre_CommandFusedStepStartPoleTarget(AutoOre_t *ctrl) {
  if (!AutoOre_CommandPoleTarget(ctrl,
                                 AutoOre_FusedStepStartPoleTarget(ctrl))) {
    return false;
  }

  AutoOre_ApplyPoleCommandLimitsFromTemplate(
      ctrl, AutoOre_FusedStepTemplateParam(ctrl));
  return true;
}

static auto_ctrl_template_e AutoOre_FusedDefaultTemplate(
    AutoOre_Action_t action) {
  switch (action) {
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
    case AUTO_ORE_ACTION_STEP_DROP_STORE_ASCEND_200_HEAD:
      return AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
    case AUTO_ORE_ACTION_STEP_DROP_STORE_DESCEND_200_HEAD:
      return AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
    case AUTO_ORE_ACTION_STEP_DROP_STORE_ASCEND_400_HEAD:
      return AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD;
    default:
      return AUTO_CTRL_TEMPLATE_NONE;
  }
}

static auto_ctrl_template_e AutoOre_FusedStepTemplateId(
    const AutoOre_t *ctrl) {
  const AutoOre_FusedParam_t *param = AutoOre_FusedParam(ctrl);
  if (param != 0 && param->step_template != AUTO_CTRL_TEMPLATE_NONE) {
    return param->step_template;
  }
  return (ctrl == 0) ? AUTO_CTRL_TEMPLATE_NONE
                     : AutoOre_FusedDefaultTemplate(ctrl->action);
}

static bool AutoOre_FusedTemplateStartsAtHeldPoleTarget(
    auto_ctrl_template_e template_id) {
  return template_id == AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD ||
         template_id == AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD;
}

/* photo1 belongs to the fused pick handoff: it confirms that the ore has
 * cleared the arm-side sensor and the arm may lift.  It is not a descend
 * stair-edge input. */
static bool AutoOre_FusedArmLiftPhotoReached(AutoOre_t *ctrl,
                                             uint32_t now_ms) {
  if (ctrl == 0) {
    return false;
  }
  if (!ctrl->feedback.photo_transfer_valid) {
    AutoOre_PausePhoto1Latch(ctrl, now_ms);
    return false;
  }

  switch (AutoOre_FusedStepTemplateId(ctrl)) {
    case AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD:
    case AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD:
      return ctrl->fused_photo1_stable_trigger_seen;
    case AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD:
    case AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD:
      return AutoOre_LatchPhotoStableFalling(
          ctrl->feedback.pe13_photo1_triggered,
          &ctrl->fused_photo1_stable_trigger_seen,
          &ctrl->fused_photo1_stable_release_latched,
          &ctrl->fused_photo1_triggered_since_ms,
          &ctrl->fused_photo1_released_since_ms, now_ms);
    default:
      return false;
  }
}

/* The optional pre-template shortcut is only valid for head-ascend.  A head-
 * descend template must enter its normal flow and detect the first stair edge
 * from photo2 (pe9) inside AutoCtrlTemplate_DescendFirstPhotoFallingStable(). */
static bool AutoOre_FusedStepStartShortcutReached(const AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return false;
  }

  switch (AutoOre_FusedStepTemplateId(ctrl)) {
    case AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD:
    case AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD:
      return ctrl->fused_photo1_stable_trigger_seen;
    case AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD:
    case AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD:
    case AUTO_CTRL_TEMPLATE_NONE:
    default:
      return false;
  }
}

static bool AutoOre_UsesFusedAscendTemplate(const AutoOre_t *ctrl) {
  const auto_ctrl_template_e template_id = AutoOre_FusedStepTemplateId(ctrl);
  return ctrl != 0 && AutoOre_ActionIsFused(ctrl->action) &&
         (template_id == AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD ||
          template_id == AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD);
}

/* The first stair edge for fused ascend is photo1, but Arm/Pole setup can
 * delay creation of the embedded step template.  Latch a stable photo1 high
 * from action start so the event remains available after the raw pulse ends. */
static void AutoOre_UpdateFusedAscendPhoto1Latch(AutoOre_t *ctrl,
                                                 uint32_t now_ms) {
  if (!AutoOre_UsesFusedAscendTemplate(ctrl) || ctrl->step_ctrl_started) {
    return;
  }
  if (!ctrl->feedback.photo_transfer_valid) {
    AutoOre_PausePhoto1Latch(ctrl, now_ms);
    return;
  }
  (void)AutoOre_LatchPhotoStableTriggered(
      ctrl->feedback.pe13_photo1_triggered,
      &ctrl->fused_photo1_stable_trigger_seen,
      &ctrl->fused_photo1_triggered_since_ms, now_ms);
}

static bool AutoOre_UsesFusedDescendTemplate(const AutoOre_t *ctrl) {
  const auto_ctrl_template_e template_id = AutoOre_FusedStepTemplateId(ctrl);
  return ctrl != 0 && AutoOre_ActionIsFused(ctrl->action) &&
         (template_id == AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD ||
          template_id == AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD);
}

/* A fused descend can spend time waiting for the photo1/Arm handoff before
 * its embedded stair template exists.  Sample photo2 during that wait so a
 * complete stair-edge falling pulse cannot disappear before the template is
 * allowed to start. */
static void AutoOre_UpdateFusedDescendPhoto2Latch(AutoOre_t *ctrl,
                                                  uint32_t now_ms) {
  if (!AutoOre_UsesFusedDescendTemplate(ctrl) ||
      ctrl->fused_photo2_handoff_applied) {
    return;
  }
  if (!ctrl->feedback.photo_transfer_valid) {
    AutoOre_PausePhoto2Latch(ctrl, now_ms);
    return;
  }
  (void)AutoOre_LatchPhotoStableFalling(
      ctrl->feedback.pe9_photo2_triggered,
      &ctrl->fused_photo2_stable_trigger_seen,
      &ctrl->fused_photo2_stable_release_latched,
      &ctrl->fused_photo2_triggered_since_ms,
      &ctrl->fused_photo2_released_since_ms, now_ms);
}

/* AutoCtrl resets its photo detector on the first RUN_TEMPLATE update.  Merge
 * the early photo2 state only after that reset has happened, then let the
 * normal descend template consume it on the next update. */
static void AutoOre_ApplyFusedDescendPhoto2Handoff(AutoOre_t *ctrl) {
  auto_ctrl_template_ctx_t *step_ctx;

  if (!AutoOre_UsesFusedDescendTemplate(ctrl) ||
      ctrl->fused_photo2_handoff_applied || !ctrl->step_ctrl_started ||
      AutoCtrl_GetState(&ctrl->step_ctrl) != AUTO_CTRL_STATE_RUN_TEMPLATE) {
    return;
  }

  step_ctx = &ctrl->step_ctrl.template_ctx;
  if (!step_ctx->step_entered) {
    return;
  }

  if (ctrl->fused_photo2_stable_trigger_seen) {
    step_ctx->pe9_photo2_stable_trigger_seen = true;
    step_ctx->pe9_photo2_triggered_since_ms =
        ctrl->fused_photo2_triggered_since_ms;
  }
  if (ctrl->fused_photo2_stable_release_latched) {
    step_ctx->pe9_photo2_stable_release_latched = true;
    step_ctx->pe9_photo2_released_since_ms =
        ctrl->fused_photo2_released_since_ms;
  }
  ctrl->fused_photo2_handoff_applied = true;
}

static bool AutoOre_PickPhoto1LiftReached(AutoOre_t *ctrl, uint32_t now_ms) {
  if (ctrl == 0) {
    return false;
  }
  if (!ctrl->feedback.photo_transfer_valid) {
    AutoOre_PausePhoto1Latch(ctrl, now_ms);
    return false;
  }
  if (!AutoOre_PickActionUsesPhoto1Falling(ctrl->action)) {
    return ctrl->feedback.pe13_photo1_triggered;
  }
  return AutoOre_LatchPhotoStableFalling(
      ctrl->feedback.pe13_photo1_triggered,
      &ctrl->fused_photo1_stable_trigger_seen,
      &ctrl->fused_photo1_stable_release_latched,
      &ctrl->fused_photo1_triggered_since_ms,
      &ctrl->fused_photo1_released_since_ms, now_ms);
}

static bool AutoOre_PickStoreRetreatPhotoReached(AutoOre_t *ctrl,
                                                 uint32_t now_ms) {
  if (ctrl == 0) {
    return false;
  }
  if (!ctrl->feedback.photo_transfer_valid) {
    AutoOre_PausePhoto1Latch(ctrl, now_ms);
    return false;
  }
  if (AutoOre_PickActionUsesPhoto1Falling(ctrl->action)) {
    return ctrl->feedback.pe13_photo1_triggered;
  }
  return AutoOre_LatchPhotoStableFalling(
      ctrl->feedback.pe13_photo1_triggered,
      &ctrl->fused_photo1_stable_trigger_seen,
      &ctrl->fused_photo1_stable_release_latched,
      &ctrl->fused_photo1_triggered_since_ms,
      &ctrl->fused_photo1_released_since_ms, now_ms);
}

static bool AutoOre_PickStoreRetreatReady(AutoOre_t *ctrl, uint32_t now_ms) {
  if (!AutoOre_PickStoreRetreatPhotoReached(ctrl, now_ms)) {
    ctrl->fused_arm_photo_since_ms = 0u;
    return false;
  }
  if (ctrl->fused_arm_photo_since_ms == 0u) {
    ctrl->fused_arm_photo_since_ms = now_ms;
    return false;
  }
  return (now_ms - ctrl->fused_arm_photo_since_ms) >=
         AUTO_ORE_PICK_STORE_RETREAT_PHOTO_DELAY_MS;
}

static uint8_t AutoOre_FusedFastPickTemplateStartStepIndex(
    const AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return 0u;
  }

  switch (AutoOre_FusedStepTemplateId(ctrl)) {
    case AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD:
    case AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD:
      return AUTO_ORE_FUSED_HEAD_ASCEND_FRONT_RETRACT_STEP_INDEX;
    case AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD:
    case AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD:
      return AUTO_ORE_FUSED_HEAD_DESCEND_FIRST_EDGE_STEP_INDEX;
    default:
      return 0u;
  }
}

static bool AutoOre_FusedFastPickOnFrontPhotoEnabled(
    const AutoOre_t *ctrl, const AutoOre_FusedParam_t *fused) {
  if (ctrl == 0 || fused == 0 || !fused->fast_pick_on_front_photo) {
    return false;
  }

  switch (AutoOre_FusedStepTemplateId(ctrl)) {
    case AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD:
    case AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD:
    case AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD:
    case AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD:
      return true;
    default:
      return false;
  }
}

static AutoOre_Action_t AutoOre_FusedDefaultPickAction(
    AutoOre_Action_t action) {
  switch (action) {
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
    case AUTO_ORE_ACTION_STEP_DROP_STORE_ASCEND_400_HEAD:
      return AUTO_ORE_ACTION_PICK_POS_400;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
    case AUTO_ORE_ACTION_STEP_DROP_STORE_ASCEND_200_HEAD:
      return AUTO_ORE_ACTION_PICK_POS_200;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
    case AUTO_ORE_ACTION_STEP_DROP_STORE_DESCEND_200_HEAD:
      return AUTO_ORE_ACTION_PICK_NEG_200;
    default:
      return AUTO_ORE_ACTION_PICK_POS_200;
  }
}

static const float *AutoOre_FusedPickPoleTarget(const AutoOre_t *ctrl) {
  if (ctrl == 0 || ctrl->param.pole_param == 0) {
    return 0;
  }
  if (ctrl->action == AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD ||
      ctrl->action == AUTO_ORE_ACTION_STEP_DROP_STORE_DESCEND_200_HEAD) {
    return ctrl->param.pole_param->preset.step_200_descend_all_retract;
  }
  const AutoOre_FusedParam_t *param = AutoOre_FusedParam(ctrl);
  AutoOre_Action_t pick_action = (param != 0)
                                     ? param->pick_action
                                     : AUTO_ORE_ACTION_NONE;
  if (!AutoOre_ActionIsPick(pick_action)) {
    pick_action = AutoOre_FusedDefaultPickAction(ctrl->action);
  }

  switch (pick_action) {
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

static bool AutoOre_CommandFusedPickPoleTarget(AutoOre_t *ctrl,
                                               const float target_lift[2]) {
  if (!AutoOre_CommandPoleTarget(ctrl, target_lift)) {
    return false;
  }

  AutoOre_ApplyPoleCommandLimitsFromTemplate(
      ctrl, AutoOre_FusedStepTemplateParam(ctrl));
  return true;
}

static ArmSimple_BehaviorPoint_t AutoOre_FusedPickArmPoint(
    const AutoOre_t *ctrl) {
  const AutoOre_FusedParam_t *param = AutoOre_FusedParam(ctrl);
  AutoOre_Action_t pick_action = (param != 0)
                                     ? param->pick_action
                                     : AUTO_ORE_ACTION_NONE;
  if (!AutoOre_ActionIsPick(pick_action)) {
    pick_action = AutoOre_FusedDefaultPickAction(ctrl->action);
  }
  return AutoOre_PickArmPoint(pick_action);
}

static bool AutoOre_ArmPhotoConfirmed(AutoOre_t *ctrl, uint32_t now_ms,
                                      bool use_arm_photo_confirm) {
  if (!use_arm_photo_confirm) {
    ctrl->pick_lift_confirmed = true;
    return true;
  }

  if (ctrl->feedback.arm_joint1_rad <
      AUTO_ORE_FUSED_ARM_PHOTO_ENABLE_JOINT1_RAD) {
    ctrl->fused_arm_photo_since_ms = 0u;
    return false;
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

static bool AutoOre_FusedArmPhotoConfirmed(AutoOre_t *ctrl,
                                           uint32_t now_ms) {
  const AutoOre_FusedParam_t *param = AutoOre_FusedParam(ctrl);
  return param != 0 &&
         AutoOre_ArmPhotoConfirmed(ctrl, now_ms,
                                   param->use_arm_photo_confirm);
}

static AutoOre_Position_t AutoOre_FusedSelectStorePosition(AutoOre_t *ctrl) {
  if (AutoOre_ActionIsDropStoreFused(ctrl->action)) {
    return AUTO_ORE_POSITION_TRANSFORM_HIGH;
  }

  AutoOre_Occupancy_t occupancy = ctrl->occupancy;
  if (ctrl->feedback.ore_store_fixed_ore_cylinder_closed) {
    occupancy.transform_low_has_ore = true;
    occupancy.transform_high_has_ore = true;
  }
  const AutoOre_Position_t position = AutoOre_SelectStorePosition(&occupancy);
  return AutoOre_IsPositionValid(position) ? position : AUTO_ORE_POSITION_ARM;
}

static void AutoOre_FusedStoreNextStep(AutoOre_t *ctrl) {
  ctrl->fused_store_step_index++;
  ctrl->fused_store_step_phase = 0u;
  ctrl->step_condition_met = false;
  ctrl->step_condition_time_ms = 0u;
}

static void AutoOre_FusedStoreMarkDone(AutoOre_t *ctrl) {
  ctrl->fused_store_done = true;
  ctrl->fused_pick_done = true;
  ctrl->fused_store_step_index = 0u;
  ctrl->fused_store_step_phase = 0u;
  if (AutoOre_IsPositionValid(ctrl->fused_store_position)) {
    ctrl->active_position = ctrl->fused_store_position;
    AutoOre_SetActivePositionHasOre(ctrl, true);
  }
  if (ctrl->fused_store_position != AUTO_ORE_POSITION_ARM) {
    AutoOre_SetArmHasOre(ctrl, false);
  }
  ctrl->fused_store_position = AUTO_ORE_POSITION_NONE;
}

static void AutoOre_RunFusedStoreLow(AutoOre_t *ctrl, uint32_t now_ms) {
  switch (ctrl->fused_store_step_index) {
    case 0:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_WAIT_STORE_ORE,
                              SUCTION_ON,
                              &ctrl->param.arm_speed.store_wait) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_MID_WAIT, false)) {
        AutoOre_FailStoreInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.arm_at_target && ctrl->feedback.ore_store_all_at_target) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 1:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE, SUCTION_ON,
                              &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_MID_WAIT, false)) {
        AutoOre_FailStoreInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(
              ctrl, now_ms, ctrl->feedback.arm_at_target,
              AutoOre_StoreArmSettleMs(ctrl))) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 2:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE, SUCTION_OFF,
                              &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_MID_WAIT, true)) {
        AutoOre_FailStoreInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AutoOre_StoreCylinderCloseMs(ctrl))) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 3:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE, SUCTION_OFF,
                              &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, true)) {
        AutoOre_FailStoreInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitOreStoreCommandTarget(ctrl, now_ms) &&
          AutoOre_StepElapsed(ctrl, now_ms) >=
              AutoOre_StoreArmSuctionOffMs(ctrl)) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 4:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE, SUCTION_OFF,
                              &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, false)) {
        AutoOre_FailStoreInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AutoOre_StoreCylinderOpenMs(ctrl))) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 5:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_WAIT_STORE_ORE,
                              SUCTION_OFF,
                              &ctrl->param.arm_speed.store_wait) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, false)) {
        AutoOre_FailStoreInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
        ctrl->fused_pick_done = true;
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 6:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_WAIT_STORE_ORE,
                              SUCTION_OFF,
                              &ctrl->param.arm_speed.store_wait) ||
          !AutoOre_CommandOreStoreWithVelocity(
              ctrl, ORE_STORE_TRANSFORM_STANDBY, false,
              AutoOre_StoreLowReturnPlannedVelocityRadS(ctrl, now_ms))) {
        AutoOre_FailStoreInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitOreStoreCommandTarget(ctrl, now_ms)) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 7:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY,
                              SUCTION_OFF,
                              &ctrl->param.arm_speed.store_standby) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, false)) {
        AutoOre_FailStoreInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 8: {
      bool shake_done = false;
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY, SUCTION_OFF,
                              &ctrl->param.arm_speed.store_standby) ||
          !AutoOre_UpdateStoreLowShake(ctrl, &ctrl->fused_store_step_phase,
                                       &shake_done)) {
        AutoOre_FailStoreInvalidParam(ctrl);
        return;
      }
      if (shake_done) {
        AutoOre_FusedStoreMarkDone(ctrl);
      }
      return;
    }
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
        AutoOre_FailStoreInvalidParam(ctrl);
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
        AutoOre_FailStoreInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(ctrl, now_ms, true,
                                         AutoOre_StoreCylinderCloseMs(ctrl))) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 2:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY,
                              SUCTION_OFF,
                              &ctrl->param.arm_speed.store_standby) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, true)) {
        AutoOre_FailStoreInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitArmCommandTarget(ctrl, now_ms)) {
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
    AutoOre_FailStoreInvalidParam(ctrl);
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
      AutoOre_FailStoreInvalidOccupancy(ctrl);
      return;
  }
}

static void AutoOre_CopyFeedbackToStepCtrl(AutoOre_t *ctrl) {
  auto_ctrl_feedback_t step_feedback = {0};
  step_feedback.yaw_auto_rad = ctrl->feedback.yaw_auto_rad;
  step_feedback.photo_transfer_valid = ctrl->feedback.photo_transfer_valid;
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
  AutoCtrl_SetLateralVelocityCommand(&ctrl->step_ctrl,
                                     ctrl->feedback.lateral_velocity_cmd_mps);
  AutoCtrl_SetYawRateCommand(&ctrl->step_ctrl,
                             ctrl->feedback.yaw_rate_cmd_rad_s);
}

static bool AutoOre_CopyStepCtrlOutputs(AutoOre_t *ctrl,
                                        bool keep_fused_pole_target) {
  ctrl->chassis_cmd = ctrl->step_ctrl.chassis_cmd;
  ctrl->chassis_cmd_valid =
      ctrl->step_ctrl.chassis_cmd.mode != CHASSIS_MODE_RELAX;

  if (keep_fused_pole_target) {
    return AutoOre_CommandFusedStepStartPoleTarget(ctrl);
  }

  ctrl->pole_cmd = ctrl->step_ctrl.pole_cmd;
  ctrl->pole_cmd_valid = ctrl->step_ctrl.pole_cmd.mode != POLE_MODE_RELAX;
  return true;
}

static bool AutoOre_ShouldKeepFusedPoleDuringStepCtrl(
    auto_ctrl_run_state_e state_before_update,
    auto_ctrl_run_state_e state_after_update) {
  return state_before_update == AUTO_CTRL_STATE_PREALIGN &&
         (state_after_update == AUTO_CTRL_STATE_PREALIGN ||
          state_after_update == AUTO_CTRL_STATE_RUN_TEMPLATE);
}

static void AutoOre_ResetFusedStepTemplateCtx(AutoOre_t *ctrl,
                                              uint8_t step_index) {
  memset(&ctrl->step_ctrl.template_ctx, 0,
         sizeof(ctrl->step_ctrl.template_ctx));
  ctrl->step_ctrl.template_ctx.step_index = step_index;
}

static void AutoOre_SkipFusedStepPrealign(AutoOre_t *ctrl,
                                          uint32_t now_ms,
                                          uint8_t step_index) {
  ctrl->step_ctrl.state = AUTO_CTRL_STATE_RUN_TEMPLATE;
  ctrl->step_ctrl.state_enter_time_ms = now_ms;
  ctrl->step_ctrl.yaw_zero_offset_rad = ctrl->step_ctrl.yaw_raw_rad;
  ctrl->step_ctrl.feedback.yaw_auto_rad = 0.0f;
  ctrl->step_ctrl.target_yaw_rad = 0.0f;
  ctrl->step_ctrl.yaw_error_rad = 0.0f;
  AutoOre_ResetFusedStepTemplateCtx(ctrl, step_index);
  ctrl->fused_step_template_start_step_index = 0u;
}

static void AutoOre_ApplyFusedStepTemplateOverrides(
    AutoOre_t *ctrl, const AutoOre_FusedParam_t *param) {
  if (ctrl == 0 || param == 0) {
    return;
  }

  ctrl->step_ctrl.template_ctx.descend_move_override_enabled =
      param->override_descend_move;
  ctrl->step_ctrl.template_ctx.descend_mid_move_ms =
      param->descend_mid_move_ms;
  ctrl->step_ctrl.template_ctx.descend_rear_retract_move_ms =
      param->descend_rear_retract_move_ms;
  ctrl->step_ctrl.template_ctx.descend_rear_retract_move_wheel_delta_rad =
      param->descend_rear_retract_move_wheel_delta_rad;
}

static void AutoOre_RunFusedStepTemplate(AutoOre_t *ctrl, uint32_t now_ms) {
  const AutoOre_FusedParam_t *param = AutoOre_FusedParam(ctrl);
  if (param == 0) {
    AutoOre_FailStepInvalidParam(ctrl);
    return;
  }

  if (!ctrl->step_ctrl_started) {
    AutoCtrl_Init(&ctrl->step_ctrl);
    const auto_ctrl_yaw_source_e yaw_source =
        (ctrl->feedback.yaw_source == AUTO_CTRL_YAW_SOURCE_PC)
            ? AUTO_CTRL_YAW_SOURCE_PC
            : AUTO_CTRL_YAW_SOURCE_STM32;
    AutoCtrl_SetYawSource(&ctrl->step_ctrl, yaw_source);
    AutoCtrl_SetYawZeroOffset(&ctrl->step_ctrl, 0.0f);
    AutoOre_CopyFeedbackToStepCtrl(ctrl);
    const auto_ctrl_template_e template_id = AutoOre_FusedStepTemplateId(ctrl);
    const float target_yaw_rad = ctrl->prealign_yaw_target_valid
                                     ? ctrl->prealign_target_yaw_rad
                                     : ctrl->feedback.yaw_auto_rad;
    const float yaw_tolerance = AutoOre_PrealignYawToleranceRad(ctrl);
    const auto_ctrl_sensor_mode_e sensor_mode =
        (yaw_source == AUTO_CTRL_YAW_SOURCE_PC) ? AUTO_CTRL_SENSOR_MODE_NONE
                                                : AUTO_CTRL_SENSOR_MODE_YAW_ONLY;
    if (!AutoCtrl_StartTemplate(&ctrl->step_ctrl, template_id,
                                AUTO_CTRL_TRAVEL_DIR_HEAD_FORWARD,
                                target_yaw_rad, yaw_tolerance,
                                sensor_mode, now_ms)) {
      AutoOre_FailStepInvalidParam(ctrl);
      return;
    }
    if (ctrl->fused_step_template_start_step_index != 0u &&
        AutoOre_FusedTemplateStartsAtHeldPoleTarget(template_id)) {
      AutoOre_SkipFusedStepPrealign(
          ctrl, now_ms, ctrl->fused_step_template_start_step_index);
    }
    ctrl->step_ctrl_started = true;
    ctrl->step_ctrl_active = true;
  }

  AutoOre_CopyFeedbackToStepCtrl(ctrl);
  const auto_ctrl_run_state_e state_before_update =
      AutoCtrl_GetState(&ctrl->step_ctrl);
  AutoOre_ApplyFusedStepTemplateOverrides(ctrl, param);
  AutoCtrl_Update(&ctrl->step_ctrl, now_ms);
  const auto_ctrl_run_state_e state_after_update =
      AutoCtrl_GetState(&ctrl->step_ctrl);
  bool template_skip_applied = false;
  if (state_after_update == AUTO_CTRL_STATE_RUN_TEMPLATE &&
      ctrl->fused_step_template_start_step_index != 0u) {
    AutoOre_ResetFusedStepTemplateCtx(
        ctrl, ctrl->fused_step_template_start_step_index);
    AutoOre_ApplyFusedStepTemplateOverrides(ctrl, param);
    ctrl->fused_step_template_start_step_index = 0u;
    template_skip_applied = true;
  }
  AutoOre_ApplyFusedDescendPhoto2Handoff(ctrl);
  AutoOre_ApplyFusedStepTemplateOverrides(ctrl, param);
  if (state_before_update == AUTO_CTRL_STATE_PREALIGN &&
      state_after_update == AUTO_CTRL_STATE_RUN_TEMPLATE &&
      !template_skip_applied &&
      AutoOre_FusedTemplateStartsAtHeldPoleTarget(
          AutoCtrl_GetTemplate(&ctrl->step_ctrl))) {
    ctrl->step_ctrl.template_ctx.pole_target_seen_not_ready = true;
  }
  if (!AutoOre_CopyStepCtrlOutputs(
          ctrl, AutoOre_ShouldKeepFusedPoleDuringStepCtrl(
                    state_before_update, state_after_update))) {
    AutoOre_FailStepInvalidParam(ctrl);
    return;
  }

  if (state_after_update == AUTO_CTRL_STATE_SUCCESS) {
    ctrl->fused_step_done = true;
    ctrl->step_ctrl_active = false;
    return;
  }

  if (state_after_update == AUTO_CTRL_STATE_FAIL ||
      state_after_update == AUTO_CTRL_STATE_ABORT) {
    AutoOre_AddFailureMask(ctrl, AUTO_ORE_FAILURE_STEP);
    ctrl->state = AUTO_ORE_STATE_FAIL;
    ctrl->result = AUTO_ORE_RESULT_FAIL;
    ctrl->fault = (AutoCtrl_GetFault(&ctrl->step_ctrl) ==
                   AUTO_CTRL_FAULT_TEMPLATE_TIMEOUT)
                      ? AUTO_ORE_FAULT_TIMEOUT
                      : AUTO_ORE_FAULT_SENSOR_INVALID;
  }
}

static void AutoOre_FinishFusedStepSide(AutoOre_t *ctrl) {
  ctrl->fused_step_done = true;
  ctrl->step_ctrl_active = false;
  ctrl->fused_step_template_start_step_index = 0u;
  AutoOre_ReleaseChassisCommand(ctrl);
  AutoOre_ReleasePoleCommand(ctrl);
}

static bool AutoOre_FusedStepStartPoleReady(AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return false;
  }
  if (!ctrl->feedback.pole_all_at_target) {
    ctrl->fused_step_start_pole_seen_not_ready = true;
    return false;
  }
  return ctrl->fused_step_start_pole_seen_not_ready;
}

static void AutoOre_RunFusedStepSide(AutoOre_t *ctrl, uint32_t now_ms,
                                     const AutoOre_FusedParam_t *fused) {
  if (ctrl == 0 || fused == 0 || ctrl->fused_step_done) {
    return;
  }

  switch (ctrl->step_phase) {
    case 0:
      if (!AutoOre_CommandFusedStepStartPoleTarget(ctrl)) {
        AutoOre_FailStepInvalidParam(ctrl);
        return;
      }

      const bool start_pole_ready = AutoOre_FusedStepStartPoleReady(ctrl);
      if (start_pole_ready &&
          AutoOre_FusedStepStartShortcutReached(ctrl)) {
        AutoOre_CommandChassisHold(ctrl);
        ctrl->fused_step_template_start_step_index =
            AutoOre_FusedFastPickTemplateStartStepIndex(ctrl);
        AutoOre_SetFusedStepSidePhase(ctrl, 1u);
        return;
      }

      if (fused->step_start_wheel_delta_rad <= 0.0f) {
        AutoOre_CommandChassisHold(ctrl);
        AutoOre_SetFusedStepSidePhase(ctrl, 1u);
        return;
      }

      AutoOre_CommandChassisMoveYawRate(ctrl, fused->step_start_vx_mps,
                                        ctrl->feedback.yaw_rate_cmd_rad_s);
      const bool step_start_distance_ready =
          AutoOre_WheelDeltaMoveReached(ctrl, fused->step_start_wheel_delta_rad);
      const bool step_start_timeout_ready =
          AutoOre_StepElapsed(ctrl, now_ms) >=
          AutoOre_FusedPrecontactTimeoutMs(ctrl, fused);
      if (step_start_distance_ready || step_start_timeout_ready) {
        AutoOre_CommandChassisHold(ctrl);
        AutoOre_SetFusedStepSidePhase(ctrl, 1u);
      }
      return;

    case 1:
      AutoOre_RunFusedStepTemplate(ctrl, now_ms);
      if (ctrl->fused_step_done) {
        AutoOre_FinishFusedStepSide(ctrl);
      }
      return;

    default:
      AutoOre_FinishFusedStepSide(ctrl);
      return;
  }
}

static void AutoOre_RunFusedStoreAndStepParallel(AutoOre_t *ctrl,
                                                uint32_t now_ms,
                                                const AutoOre_FusedParam_t *fused) {
  AutoOre_EnterStep(ctrl, now_ms);

  if (!ctrl->fused_store_done) {
    AutoOre_RunFusedStore(ctrl, now_ms);
  }

  if (ctrl->state != AUTO_ORE_STATE_RUNNING) {
    return;
  }

  if (!ctrl->fused_step_done) {
    AutoOre_RunFusedStepSide(ctrl, now_ms, fused);
  } else {
    AutoOre_ReleaseChassisCommand(ctrl);
  }

  if (ctrl->state != AUTO_ORE_STATE_RUNNING) {
    return;
  }

  if (ctrl->fused_store_done && ctrl->fused_step_done) {
    AutoOre_NextStep(ctrl);
  } else {
    (void)AutoOre_CheckFusedParallelTimeout(ctrl, now_ms);
  }
}

static void AutoOre_RunPickStoreFusedParallel(AutoOre_t *ctrl,
                                              uint32_t now_ms) {
  AutoOre_EnterStep(ctrl, now_ms);

  if (!ctrl->fused_store_done) {
    AutoOre_RunFusedStore(ctrl, now_ms);
  }

  if (ctrl->state != AUTO_ORE_STATE_RUNNING) {
    return;
  }

  if (!ctrl->fused_step_done) {
    if (!AutoOre_PickStoreRetreatReady(ctrl, now_ms)) {
      AutoOre_CommandChassisMove(ctrl, -AutoOre_FetchChassisVxMps(ctrl));
    } else {
      ctrl->fused_step_done = true;
      AutoOre_ReleaseChassisCommand(ctrl);
    }
  } else {
    AutoOre_ReleaseChassisCommand(ctrl);
  }

  if (ctrl->fused_store_done && ctrl->fused_step_done) {
    AutoOre_NextStep(ctrl);
  } else {
    (void)AutoOre_CheckFusedParallelTimeout(ctrl, now_ms);
  }
}

static void AutoOre_RunPickStoreFused(AutoOre_t *ctrl, uint32_t now_ms) {
  const float *pole_target = AutoOre_PickPoleTarget(ctrl);
  if (pole_target == 0) {
    AutoOre_FailPickInvalidParam(ctrl);
    return;
  }

  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      const bool prealign_ready = AutoOre_RunPickPrealign(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY, SUCTION_ON,
                              &ctrl->param.arm_speed.pick_standby) ||
          !AutoOre_CommandPoleTarget(ctrl, pole_target)) {
        AutoOre_FailPickInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.pole_all_at_target && prealign_ready) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeoutWithFailure(
            ctrl, now_ms, AUTO_ORE_FAILURE_PICK_ORE);
      }
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, AutoOre_PickArmPoint(ctrl->action),
                              SUCTION_ON, &ctrl->param.arm_speed.pick_place) ||
          !AutoOre_CommandPoleTarget(ctrl, pole_target)) {
        AutoOre_FailPickInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitPickArmCommandTargetStable(ctrl, now_ms)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeoutWithFailure(
            ctrl, now_ms, AUTO_ORE_FAILURE_PICK_ORE);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, AutoOre_PickArmPoint(ctrl->action),
                              SUCTION_ON, &ctrl->param.arm_speed.pick_fetch) ||
          !AutoOre_CommandPoleTarget(ctrl, pole_target)) {
        AutoOre_FailPickInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMove(ctrl, AutoOre_FetchChassisVxMps(ctrl));
      if (AutoOre_WaitLatchedConditionThenDelay(
              ctrl, now_ms, AutoOre_PickPhoto1LiftReached(ctrl, now_ms),
              AUTO_ORE_PICK_STORE_FETCH_PHOTO_DELAY_MS)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeoutWithFailure(
            ctrl, now_ms, AUTO_ORE_FAILURE_PICK_ORE);
      }
      return;
    case 3:
      AutoOre_EnterStep(ctrl, now_ms);
      AutoOre_CommandChassisHold(ctrl);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_PICK_LIFT_DETECT,
                              SUCTION_ON,
                              &ctrl->param.arm_speed.pick_lift_detect) ||
          !AutoOre_CommandPoleTarget(ctrl, pole_target)) {
        AutoOre_FailPickInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(
              ctrl, now_ms, ctrl->feedback.arm_at_target,
              AutoOre_FusedPickLiftDetectMs(ctrl)) &&
          AutoOre_ArmPhotoConfirmed(ctrl, now_ms, false)) {
        AutoOre_SetArmHasOre(ctrl, true);
        AutoOre_ResetPhoto1Latch(ctrl);
        ctrl->fused_arm_photo_since_ms = 0u;
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeoutWithFailure(
            ctrl, now_ms, AUTO_ORE_FAILURE_PICK_ORE);
      }
      return;
    case 4:
      AutoOre_RunPickStoreFusedParallel(ctrl, now_ms);
      return;
    case 5:
      AutoOre_FinishSuccess(ctrl);
      return;
    default:
      AutoOre_FinishSuccess(ctrl);
      return;
  }
}

static void AutoOre_RunStepPickStoreFused(AutoOre_t *ctrl, uint32_t now_ms) {
  const AutoOre_FusedParam_t *fused = AutoOre_FusedParam(ctrl);
  if (fused == 0) {
    AutoOre_FailSetupInvalidParam(ctrl);
    return;
  }
  const float *pole_target = AutoOre_FusedPickPoleTarget(ctrl);
  if (pole_target == 0) {
    AutoOre_FailSetupInvalidParam(ctrl);
    return;
  }

  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      if (AutoOre_RunPickPrealign(ctrl, now_ms)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeoutWithFailure(
            ctrl, now_ms, AUTO_ORE_FAILURE_PICK_ORE);
      }
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY, SUCTION_ON,
                              &ctrl->param.arm_speed.pick_standby) ||
          !AutoOre_CommandFusedPickPoleTarget(ctrl, pole_target)) {
        AutoOre_FailPickInvalidParam(ctrl);
        return;
      }
      /* STANDBY is only a transit target.  For photo1 protection the Arm is
       * not considered safe until the fused pick target in case 2 is stable. */
      AutoOre_CommandFusedApproachWithArmProtection(
          ctrl, fused, false);
      if (ctrl->feedback.pole_all_at_target) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeoutWithFailure(
            ctrl, now_ms, AUTO_ORE_FAILURE_PICK_ORE);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, AutoOre_FusedPickArmPoint(ctrl),
                              SUCTION_ON,
                              &ctrl->param.arm_speed.pick_place) ||
          !AutoOre_CommandFusedPickPoleTarget(ctrl, pole_target)) {
        AutoOre_FailPickInvalidParam(ctrl);
        return;
      }
      const bool pick_arm_stable =
          AutoOre_WaitPickArmCommandTargetStable(ctrl, now_ms);
      AutoOre_CommandFusedApproachWithArmProtection(
          ctrl, fused, pick_arm_stable);
      if (pick_arm_stable) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeoutWithFailure(
            ctrl, now_ms, AUTO_ORE_FAILURE_PICK_ORE);
      }
      return;
    case 3:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, AutoOre_FusedPickArmPoint(ctrl),
                              SUCTION_ON,
                              &ctrl->param.arm_speed.pick_fetch) ||
          !AutoOre_CommandFusedPickPoleTarget(ctrl, pole_target)) {
        AutoOre_FailPickInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMoveYawRate(ctrl, fused->precontact_vx_mps,
                                        ctrl->feedback.yaw_rate_cmd_rad_s);
      if (AutoOre_FusedFastPickOnFrontPhotoEnabled(ctrl, fused)) {
        const bool start_pole_ready = AutoOre_FusedStepStartPoleReady(ctrl);
        /* Latch the falling edge even before Pole reports ready.  The old
         * short-circuit expression skipped sampling during that interval and
         * made a short photo1 pulse intermittently disappear. */
        const bool arm_lift_photo_reached =
            AutoOre_FusedArmLiftPhotoReached(ctrl, now_ms);
        if (start_pole_ready && arm_lift_photo_reached) {
          if (!AutoOre_WaitConditionThenDelay(
                  ctrl, now_ms, true,
              AutoOre_FusedPhoto1LiftDelayMs(ctrl))) {
            return;
          }
          ctrl->pick_lift_confirmed = true;
          AutoOre_SetArmHasOre(ctrl, true);
          ctrl->fused_step_template_start_step_index =
              AutoOre_FusedFastPickTemplateStartStepIndex(ctrl);
          AutoOre_JumpToStep(ctrl, 5u);
          ctrl->step_phase = 1u;
          AutoOre_RunFusedStoreAndStepParallel(ctrl, now_ms, fused);
          return;
        }

        ctrl->step_condition_met = false;

        if (AutoOre_StepElapsed(ctrl, now_ms) >=
            AutoOre_FusedPrecontactTimeoutMs(ctrl, fused)) {
          AutoOre_CommandChassisHold(ctrl);
          AutoOre_NextStep(ctrl);
        }
        return;
      }
      const bool precontact_distance_ready =
          AutoOre_WheelDeltaMoveReached(ctrl, fused->precontact_wheel_delta_rad);
      const bool precontact_timeout_ready =
          AutoOre_StepElapsed(ctrl, now_ms) >=
          AutoOre_FusedPrecontactTimeoutMs(ctrl, fused);
      if (precontact_distance_ready || precontact_timeout_ready) {
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
          !AutoOre_CommandFusedPickPoleTarget(ctrl, pole_target)) {
        AutoOre_FailPickInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(
              ctrl, now_ms, ctrl->feedback.arm_at_target,
              AutoOre_FusedPickLiftDetectMs(ctrl)) &&
          AutoOre_FusedArmPhotoConfirmed(ctrl, now_ms)) {
        AutoOre_SetArmHasOre(ctrl, true);
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeoutWithFailure(
            ctrl, now_ms, AUTO_ORE_FAILURE_PICK_ORE);
      }
      return;
    case 5:
      AutoOre_RunFusedStoreAndStepParallel(ctrl, now_ms, fused);
      return;
    case 6:
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
      const bool prealign_ready = AutoOre_RunPickPrealign(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY,
                              SUCTION_ON,
                              &ctrl->param.arm_speed.pick_standby) ||
          !AutoOre_CommandPoleTarget(ctrl, pole_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (ctrl->feedback.pole_all_at_target && prealign_ready) {
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
      if (AutoOre_WaitPickArmCommandTargetStable(ctrl, now_ms)) {
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
      if (AutoOre_PickPhoto1LiftReached(ctrl, now_ms)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 3:
      AutoOre_EnterStep(ctrl, now_ms);
      /* The short fetch motion has ended. Keep the arm/pole lease, but return
       * the idle chassis to PC while lift detection finishes. */
      AutoOre_ReleaseChassisCommand(ctrl);
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
          AutoOre_ArmPhotoConfirmed(ctrl, now_ms, false)) {
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

static void AutoOre_RunRecoverStore(AutoOre_t *ctrl, uint32_t now_ms) {
  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      if (ctrl->occupancy.arm_has_ore) {
        AutoOre_AddFailureMask(ctrl, AUTO_ORE_FAILURE_SETUP);
        ctrl->state = AUTO_ORE_STATE_FAIL;
        ctrl->result = AUTO_ORE_RESULT_FAIL;
        ctrl->fault = AUTO_ORE_FAULT_INVALID_OCCUPANCY;
        return;
      }
      ctrl->fused_store_position = AutoOre_FusedSelectStorePosition(ctrl);
      if (ctrl->fused_store_position != AUTO_ORE_POSITION_TRANSFORM_LOW &&
          ctrl->fused_store_position != AUTO_ORE_POSITION_TRANSFORM_HIGH) {
        AutoOre_FailStoreInvalidOccupancy(ctrl);
        return;
      }
      ctrl->prealign_target_yaw_rad = ctrl->feedback.yaw_auto_rad;
      ctrl->prealign_yaw_target_valid = true;
      AutoOre_UpdatePrealignYawError(ctrl);
      AutoOre_NextStep(ctrl);
      return;
    case 1:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, AutoOre_RecoverArmPoint(), SUCTION_ON,
                              &ctrl->param.arm_speed.pick_place)) {
        AutoOre_FailPickInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitPickArmCommandTargetStable(ctrl, now_ms)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeoutWithFailure(
            ctrl, now_ms, AUTO_ORE_FAILURE_PICK_ORE);
      }
      return;
    case 2:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, AutoOre_RecoverArmPoint(), SUCTION_ON,
                              &ctrl->param.arm_speed.pick_fetch)) {
        AutoOre_FailPickInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMove(ctrl, AutoOre_RecoverForwardVxMps(ctrl));
      if (AutoOre_WaitLatchedConditionThenDelay(
              ctrl, now_ms,
               AutoOre_RecoverFrontSickReached(ctrl),
              AutoOre_RecoverFrontSickDelayMs(ctrl))) {
        AutoOre_NextStep(ctrl);
      } else if (!ctrl->step_condition_met &&
                 AutoOre_StepElapsed(ctrl, now_ms) >=
                 AutoOre_RecoverForwardMs(ctrl)) {
        AutoOre_CommandChassisHold(ctrl);
        AutoOre_AddFailureMask(ctrl, AUTO_ORE_FAILURE_PICK_ORE);
        ctrl->state = AUTO_ORE_STATE_FAIL;
        ctrl->result = AUTO_ORE_RESULT_FAIL;
        ctrl->fault = AUTO_ORE_FAULT_TIMEOUT;
      }
      return;
    case 3:
      AutoOre_EnterStep(ctrl, now_ms);
      AutoOre_CommandChassisHold(ctrl);
      if (!AutoOre_CommandArm(ctrl, AutoOre_RecoverArmPoint(), SUCTION_ON,
                              &ctrl->param.arm_speed.pick_fetch)) {
        AutoOre_FailPickInvalidParam(ctrl);
        return;
      }
      if (AutoOre_StepElapsed(ctrl, now_ms) >=
          AutoOre_RecoverSuctionSettleMs(ctrl)) {
        AutoOre_NextStep(ctrl);
      }
      return;
    case 4:
      AutoOre_EnterStep(ctrl, now_ms);
      if (!AutoOre_CommandArm(ctrl, AutoOre_RecoverArmPoint(), SUCTION_ON,
                              &ctrl->param.arm_speed.pick_fetch)) {
        AutoOre_FailPickInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMove(ctrl, -AutoOre_RecoverRetreatVxMps(ctrl));
      if (AutoOre_StepElapsed(ctrl, now_ms) >= AutoOre_RecoverRetreatMs(ctrl)) {
        AutoOre_NextStep(ctrl);
      }
      return;
    case 5:
      AutoOre_EnterStep(ctrl, now_ms);
      AutoOre_ReleaseChassisCommand(ctrl);
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_PICK_LIFT_DETECT,
                              SUCTION_ON,
                              &ctrl->param.arm_speed.pick_lift_detect)) {
        AutoOre_FailPickInvalidParam(ctrl);
        return;
      }
      if (AutoOre_WaitConditionThenDelay(
              ctrl, now_ms, ctrl->feedback.arm_at_target,
              AutoOre_FusedPickLiftDetectMs(ctrl)) &&
          AutoOre_ArmPhotoConfirmed(ctrl, now_ms, false)) {
        AutoOre_SetArmHasOre(ctrl, true);
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeoutWithFailure(
            ctrl, now_ms, AUTO_ORE_FAILURE_PICK_ORE);
      }
      return;
    case 6:
      AutoOre_EnterStep(ctrl, now_ms);
      AutoOre_ReleaseChassisCommand(ctrl);
      if (!ctrl->fused_store_done) {
        AutoOre_RunFusedStore(ctrl, now_ms);
      }
      if (ctrl->state != AUTO_ORE_STATE_RUNNING) {
        return;
      }
      if (ctrl->fused_store_done) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeoutWithFailure(
            ctrl, now_ms, AUTO_ORE_FAILURE_STORE_ORE);
      }
      return;
    case 7:
      AutoOre_FinishSuccess(ctrl);
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
  ctrl->failure_mask = AUTO_ORE_FAILURE_NONE;
  ctrl->active_position = AUTO_ORE_POSITION_NONE;
  AutoOre_PublishOutputSnapshot(ctrl);
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

static bool AutoOre_StartResolved(AutoOre_t *ctrl, AutoOre_Action_t action,
                                  AutoOre_Position_t position,
                                  bool occupancy_suspect,
                                  uint32_t now_ms) {
  if (!AutoOre_IsPositionValid(position)) {
    if (ctrl->failure_mask == AUTO_ORE_FAILURE_NONE) {
      AutoOre_AddFailureMask(ctrl, AUTO_ORE_FAILURE_SETUP);
    }
    AutoOre_FailInvalidOccupancy(ctrl);
    return false;
  }
  ctrl->state = AUTO_ORE_STATE_RUNNING;
  ctrl->result = AUTO_ORE_RESULT_RUNNING;
  ctrl->failure_mask = AUTO_ORE_FAILURE_NONE;
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
  ctrl->arm_target_stable_since_ms = 0u;
  ctrl->step_enter_time_ms = now_ms;
  if (AutoOre_ActionIsReleaseStep2(action)) {
    ctrl->step_index = 2u;
  }
  ctrl->step_ctrl_active = false;
  ctrl->step_ctrl_started = false;
  ctrl->fused_pick_done = false;
  ctrl->fused_step_done = false;
  ctrl->fused_store_done = false;
  ctrl->fused_step_start_pole_seen_not_ready = false;
  AutoOre_ResetParallelContext(ctrl);
  ctrl->pick_lift_confirmed = false;
  ctrl->prealign_yaw_target_valid = false;
  ctrl->prealign_target_yaw_rad = 0.0f;
  ctrl->prealign_yaw_error_rad = 0.0f;
  ctrl->fused_arm_photo_since_ms = 0u;
  ctrl->fused_photo1_stable_trigger_seen = false;
  ctrl->fused_photo1_stable_release_latched = false;
  ctrl->fused_photo1_arm_protection_latched = false;
  ctrl->fused_photo1_triggered_since_ms = 0u;
  ctrl->fused_photo1_released_since_ms = 0u;
  AutoOre_ResetPhoto2Latch(ctrl);
  ctrl->fused_store_step_index = 0u;
  ctrl->fused_store_step_phase = 0u;
  ctrl->fused_step_template_start_step_index = 0u;
  ctrl->fused_store_position = AUTO_ORE_POSITION_NONE;
  ctrl->release_ir_allow_latched = false;
  ctrl->release_ir_abort_pending = false;
  ctrl->release_ir_finish_pending = false;
  ctrl->release_ir_wait_ready = false;
  ctrl->release_ir_release_started = false;
  ctrl->release_ir_suction_released = false;
  ctrl->release_ir_abort_recovery_active = false;
  ctrl->release_ir_abort_recovered = false;
  ctrl->release_ir_finish_return_active = false;
  ctrl->release_ir_abort_suction_on = true;
  ctrl->release_ir_abort_source_step = 0u;
  ctrl->release_ir_abort_recovery_phase = 0u;
  ctrl->release_ir_abort_count_at_start =
      ctrl->feedback.release_lift_ir_abort_count;
  ctrl->release_ir_finish_count_at_start =
      ctrl->feedback.release_lift_ir_finish_count;
  ctrl->release_ir_wait_ready_time_ms = 0u;
  AutoOre_ResetReleaseLiftObserver(ctrl);
  AutoOre_ResetDistanceGate(ctrl);
  AutoOre_ClearOutputs(ctrl);
  AutoOre_PublishOutputSnapshot(ctrl);
  return true;
}

static bool AutoOre_Start(AutoOre_t *ctrl, AutoOre_Action_t action,
                          uint32_t now_ms) {
  if (ctrl == 0 || AutoOre_IsBusy(ctrl)) {
    return false;
  }
  ctrl->failure_mask = AUTO_ORE_FAILURE_NONE;
  ctrl->fault = AUTO_ORE_FAULT_NONE;
  AutoOre_ApplyFeedbackOccupancy(ctrl);
  AutoOre_Position_t position = AUTO_ORE_POSITION_NONE;
  if (action == AUTO_ORE_ACTION_STORE) {
    position = AutoOre_SelectStorePosition(&ctrl->occupancy);
  } else if (AutoOre_ActionIsReleaseLike(action)) {
    if (ctrl->occupancy.arm_has_ore) {
      position = AUTO_ORE_POSITION_ARM;
    } else {
      position = AutoOre_SelectChamberPosition(&ctrl->occupancy);
    }
  } else if (action == AUTO_ORE_ACTION_CHAMBER) {
    position = AutoOre_SelectChamberPosition(&ctrl->occupancy);
  } else if (AutoOre_ActionIsPick(action)) {
    if (!ctrl->occupancy.arm_has_ore) {
      position = AUTO_ORE_POSITION_ARM;
    }
  } else if (AutoOre_ActionIsPickStoreFused(action)) {
    if (!ctrl->occupancy.arm_has_ore) {
      position = AUTO_ORE_POSITION_ARM;
    }
  } else if (AutoOre_ActionIsRecoverStore(action)) {
    if (!ctrl->occupancy.arm_has_ore) {
      position = AUTO_ORE_POSITION_ARM;
    }
  } else if (AutoOre_ActionIsDropStoreFused(action)) {
    if (!ctrl->occupancy.transform_high_has_ore) {
      AutoOre_FailInvalidOccupancy(ctrl);
      return false;
    }
    if (!ctrl->occupancy.arm_has_ore) {
      position = AUTO_ORE_POSITION_ARM;
    }
  } else if (AutoOre_ActionIsFused(action)) {
    if (!ctrl->occupancy.arm_has_ore) {
      position = AUTO_ORE_POSITION_ARM;
    }
  }
  return AutoOre_StartResolved(ctrl, action, position, false, now_ms);
}

bool AutoOre_StartStore(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_STORE, now_ms);
}

bool AutoOre_StartStoreAtPosition(AutoOre_t *ctrl,
                                  AutoOre_Position_t position,
                                  uint32_t now_ms) {
  if (ctrl == 0 || AutoOre_IsBusy(ctrl)) {
    return false;
  }
  ctrl->failure_mask = AUTO_ORE_FAILURE_NONE;
  ctrl->fault = AUTO_ORE_FAULT_NONE;
  AutoOre_ApplyFeedbackOccupancy(ctrl);
  return AutoOre_StartResolved(ctrl, AUTO_ORE_ACTION_STORE, position, false,
                               now_ms);
}

bool AutoOre_StartRelease(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_RELEASE, now_ms);
}

bool AutoOre_StartReleaseLiftDetect(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_RELEASE_LIFT_DETECT, now_ms);
}

bool AutoOre_StartReleaseIrLiftDetect(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT, now_ms);
}

bool AutoOre_StartReleaseStep1(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_RELEASE_STEP1, now_ms);
}

bool AutoOre_StartReleaseStep2(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_RELEASE_STEP2, now_ms);
}

bool AutoOre_StartReleaseLiftDetectStep1(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_RELEASE_LIFT_DETECT_STEP1,
                       now_ms);
}

bool AutoOre_StartReleaseLiftDetectStep2(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_RELEASE_LIFT_DETECT_STEP2,
                       now_ms);
}

bool AutoOre_StartReleaseIrLiftDetectStep1(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT_STEP1,
                       now_ms);
}

bool AutoOre_StartReleaseIrLiftDetectStep2(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT_STEP2,
                       now_ms);
}

bool AutoOre_ContinueReleaseStep2(AutoOre_t *ctrl,
                                  AutoOre_Action_t step2_action,
                                  uint32_t now_ms) {
  if (ctrl == 0 || ctrl->state != AUTO_ORE_STATE_RUNNING ||
      !ctrl->fused_store_done) {
    return false;
  }

  const bool action_matches =
      (ctrl->action == AUTO_ORE_ACTION_RELEASE_STEP1 &&
       step2_action == AUTO_ORE_ACTION_RELEASE_STEP2) ||
      (ctrl->action == AUTO_ORE_ACTION_RELEASE_LIFT_DETECT_STEP1 &&
       step2_action == AUTO_ORE_ACTION_RELEASE_LIFT_DETECT_STEP2) ||
      (ctrl->action == AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT_STEP1 &&
       step2_action == AUTO_ORE_ACTION_RELEASE_IR_LIFT_DETECT_STEP2);
  if (!action_matches) {
    return false;
  }

  ctrl->action = step2_action;
  ctrl->step_index = 2u;
  ctrl->step_phase = 0u;
  ctrl->step_entered = false;
  ctrl->step_condition_met = false;
  ctrl->step_condition_time_ms = now_ms;
  ctrl->step_enter_time_ms = now_ms;
  ctrl->fused_store_done = false;
  return true;
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

bool AutoOre_StartRecoverStore(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_RECOVER_STORE, now_ms);
}

bool AutoOre_StartStepPickStoreAscend200Head(AutoOre_t *ctrl,
                                             uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD,
                       now_ms);
}

bool AutoOre_StartStepPickStoreDescend200Head(AutoOre_t *ctrl,
                                              uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD,
                       now_ms);
}

bool AutoOre_StartStepPickStoreAscend400Head(AutoOre_t *ctrl,
                                             uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD,
                       now_ms);
}

bool AutoOre_StartStepDropStoreAscend200Head(AutoOre_t *ctrl,
                                             uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_STEP_DROP_STORE_ASCEND_200_HEAD,
                       now_ms);
}

bool AutoOre_StartStepDropStoreDescend200Head(AutoOre_t *ctrl,
                                              uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_STEP_DROP_STORE_DESCEND_200_HEAD,
                       now_ms);
}

bool AutoOre_StartStepDropStoreAscend400Head(AutoOre_t *ctrl,
                                             uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_STEP_DROP_STORE_ASCEND_400_HEAD,
                       now_ms);
}

bool AutoOre_StartPickStorePos400(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_PICK_STORE_POS_400, now_ms);
}

bool AutoOre_StartPickStorePos200(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_PICK_STORE_POS_200, now_ms);
}

bool AutoOre_StartPickStoreNeg200(AutoOre_t *ctrl, uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_PICK_STORE_NEG_200, now_ms);
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
    AutoOre_ClearOutputs(ctrl);
    AutoOre_PublishOutputSnapshot(ctrl);
    return;
  }
  if (AutoOre_ActionUsesIrReleaseLiftDetect(ctrl->action) &&
      ctrl->feedback.release_lift_ir_abort_count !=
          ctrl->release_ir_abort_count_at_start) {
    ctrl->release_ir_abort_pending = true;
    ctrl->release_ir_abort_count_at_start =
        ctrl->feedback.release_lift_ir_abort_count;
  }
  if (AutoOre_ActionIsZone3IrRelease(ctrl->action) &&
      ctrl->feedback.release_lift_ir_finish_count !=
          ctrl->release_ir_finish_count_at_start) {
    ctrl->release_ir_finish_pending = true;
    ctrl->release_ir_finish_count_at_start =
        ctrl->feedback.release_lift_ir_finish_count;
  }
  AutoOre_UpdateFusedAscendPhoto1Latch(ctrl, now_ms);
  AutoOre_UpdateFusedDescendPhoto2Latch(ctrl, now_ms);
  if ((!AutoOre_ActionIsPick(ctrl->action) ||
      AutoOre_ActionIsFused(ctrl->action) ||
      AutoOre_ActionIsPickStoreFused(ctrl->action) ||
      AutoOre_ActionIsRecoverStore(ctrl->action)) &&
      !ctrl->feedback.ore_store_all_homed) {
    ctrl->state = AUTO_ORE_STATE_FAIL;
    ctrl->result = AUTO_ORE_RESULT_FAIL;
    ctrl->fault = AUTO_ORE_FAULT_NOT_HOMED;
    AutoOre_AddFailureMask(ctrl, AUTO_ORE_FAILURE_SETUP);
    AutoOre_SyncParallelContext(ctrl, now_ms);
    AutoOre_ClearOutputs(ctrl);
    AutoOre_PublishOutputSnapshot(ctrl);
    return;
  }

  AutoOre_ClearOutputs(ctrl);
  bool zone3_control_handled = false;
  if (AutoOre_ActionIsZone3IrRelease(ctrl->action)) {
    if (ctrl->release_ir_abort_pending) {
      AutoOre_RunIrReleaseAbortRecovery(ctrl, now_ms);
      zone3_control_handled = true;
    } else if (ctrl->release_ir_abort_recovered) {
      if (ctrl->release_ir_finish_pending) {
        AutoOre_RunZone3FinishReturn(ctrl, now_ms);
      } else {
        AutoOre_RunZone3AbortHold(ctrl, now_ms);
      }
      zone3_control_handled = true;
    } else if (ctrl->release_ir_finish_pending &&
               (!ctrl->release_ir_release_started ||
                ctrl->step_index >= 6u)) {
      AutoOre_RunZone3FinishReturn(ctrl, now_ms);
      zone3_control_handled = true;
    }
  }

  if (zone3_control_handled) {
    /* Command 06 holds WAIT_RELEASE_ORE; command 05 owns the final STANDBY
     * transition.  Keep the normal release state machine paused meanwhile. */
  } else if (ctrl->action == AUTO_ORE_ACTION_STORE) {
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
  } else if (AutoOre_ActionIsReleaseLike(ctrl->action)) {
    switch (ctrl->active_position) {
      case AUTO_ORE_POSITION_ARM:
        AutoOre_RunReleaseArm(ctrl, now_ms);
        break;
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
  } else if (AutoOre_ActionIsPickStoreFused(ctrl->action)) {
    if (ctrl->active_position == AUTO_ORE_POSITION_ARM) {
      AutoOre_RunPickStoreFused(ctrl, now_ms);
    } else {
      ctrl->state = AUTO_ORE_STATE_FAIL;
      ctrl->result = AUTO_ORE_RESULT_FAIL;
      ctrl->fault = AUTO_ORE_FAULT_INVALID_OCCUPANCY;
    }
  } else if (AutoOre_ActionIsRecoverStore(ctrl->action)) {
    if (ctrl->active_position == AUTO_ORE_POSITION_ARM) {
      AutoOre_RunRecoverStore(ctrl, now_ms);
    } else {
      ctrl->state = AUTO_ORE_STATE_FAIL;
      ctrl->result = AUTO_ORE_RESULT_FAIL;
      ctrl->fault = AUTO_ORE_FAULT_INVALID_OCCUPANCY;
    }
  } else if (AutoOre_ActionIsFused(ctrl->action)) {
    AutoOre_RunStepPickStoreFused(ctrl, now_ms);
  }
  AutoOre_SyncParallelContext(ctrl, now_ms);
  AutoOre_PublishOutputSnapshot(ctrl);
}

void AutoOre_Abort(AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return;
  }
  ctrl->state = AUTO_ORE_STATE_ABORT;
  ctrl->result = AUTO_ORE_RESULT_ABORTED;
  ctrl->fault = AUTO_ORE_FAULT_ABORTED;
  ctrl->failure_mask = AUTO_ORE_FAILURE_ABORTED;
  AutoOre_BranchCancel(&ctrl->parallel.handoff, 0u);
  AutoOre_BranchCancel(&ctrl->parallel.store, 0u);
  AutoOre_BranchCancel(&ctrl->parallel.step, 0u);
  ctrl->parallel.acquired_resource_mask = AUTO_ORE_RESOURCE_NONE;
  if (ctrl->step_ctrl_active || ctrl->step_ctrl_started) {
    AutoCtrl_Abort(&ctrl->step_ctrl);
  }
  ctrl->action = AUTO_ORE_ACTION_NONE;
  ctrl->active_position = AUTO_ORE_POSITION_NONE;
  ctrl->step_ctrl_active = false;
  ctrl->step_ctrl_started = false;
  AutoOre_ClearOutputs(ctrl);
  AutoOre_PublishOutputSnapshot(ctrl);
}

bool AutoOre_IsBusy(const AutoOre_t *ctrl) {
  return ctrl != 0 && ctrl->state == AUTO_ORE_STATE_RUNNING;
}

bool AutoOre_HasSplitResult(const AutoOre_t *ctrl) {
  return ctrl != 0 &&
         (AutoOre_ActionIsFused(ctrl->action) ||
          AutoOre_ActionIsPickStoreFused(ctrl->action) ||
          AutoOre_ActionIsReleaseStep1(ctrl->action));
}

bool AutoOre_IsLowerFinished(const AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return false;
  }
  if (ctrl->state == AUTO_ORE_STATE_SUCCESS) {
    return true;
  }
  return AutoOre_HasSplitResult(ctrl) && ctrl->fused_step_done;
}

bool AutoOre_IsUpperFinished(const AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return false;
  }
  if (ctrl->state == AUTO_ORE_STATE_SUCCESS) {
    return true;
  }
  return AutoOre_HasSplitResult(ctrl) && ctrl->fused_store_done;
}

bool AutoOre_IsPickFinished(const AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return false;
  }
  if (ctrl->state == AUTO_ORE_STATE_SUCCESS) {
    return true;
  }
  return AutoOre_HasSplitResult(ctrl) && ctrl->fused_pick_done;
}

bool AutoOre_IsStoreFinished(const AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return false;
  }
  if (ctrl->state == AUTO_ORE_STATE_SUCCESS) {
    return true;
  }
  return AutoOre_HasSplitResult(ctrl) && ctrl->fused_store_done;
}

bool AutoOre_IsStepFinished(const AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return false;
  }
  if (ctrl->state == AUTO_ORE_STATE_SUCCESS) {
    return true;
  }
  return AutoOre_HasSplitResult(ctrl) && ctrl->fused_step_done;
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

uint16_t AutoOre_GetFailureMask(const AutoOre_t *ctrl) {
  return (ctrl == 0) ? AUTO_ORE_FAILURE_SETUP : ctrl->failure_mask;
}

uint8_t AutoOre_GetCompletedSegmentMask(const AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return AUTO_ORE_SEGMENT_NONE;
  }
  return (uint8_t)((ctrl->fused_pick_done ? AUTO_ORE_SEGMENT_HANDOFF : 0u) |
                   (ctrl->fused_store_done ? AUTO_ORE_SEGMENT_STORE : 0u) |
                   (ctrl->fused_step_done ? AUTO_ORE_SEGMENT_STEP : 0u));
}

uint8_t AutoOre_GetRunningSegmentMask(const AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return AUTO_ORE_SEGMENT_NONE;
  }
  return (uint8_t)(
      ((ctrl->parallel.handoff.state == AUTO_ORE_BRANCH_RUNNING)
           ? AUTO_ORE_SEGMENT_HANDOFF
           : 0u) |
      ((ctrl->parallel.store.state == AUTO_ORE_BRANCH_RUNNING)
           ? AUTO_ORE_SEGMENT_STORE
           : 0u) |
      ((ctrl->parallel.step.state == AUTO_ORE_BRANCH_RUNNING)
           ? AUTO_ORE_SEGMENT_STEP
           : 0u));
}

uint8_t AutoOre_GetFailedSegmentMask(const AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return AUTO_ORE_SEGMENT_NONE;
  }
  return (uint8_t)(
      ((ctrl->parallel.handoff.state == AUTO_ORE_BRANCH_FAILED)
           ? AUTO_ORE_SEGMENT_HANDOFF
           : 0u) |
      ((ctrl->parallel.store.state == AUTO_ORE_BRANCH_FAILED)
           ? AUTO_ORE_SEGMENT_STORE
           : 0u) |
      ((ctrl->parallel.step.state == AUTO_ORE_BRANCH_FAILED)
           ? AUTO_ORE_SEGMENT_STEP
           : 0u));
}

bool AutoOre_ReadOutputSnapshot(const AutoOre_t *ctrl,
                                AutoOre_OutputSnapshot_t *snapshot) {
  if (ctrl == 0 || snapshot == 0) {
    return false;
  }

  const uint8_t index = (uint8_t)(ctrl->output_snapshot_index & 1u);
  __atomic_thread_fence(__ATOMIC_ACQUIRE);
  *snapshot = ctrl->output_snapshot[index];
  __atomic_thread_fence(__ATOMIC_ACQUIRE);
  return index == (uint8_t)(ctrl->output_snapshot_index & 1u);
}

uint8_t AutoOre_GetOwnedResourceMask(const AutoOre_t *ctrl) {
  AutoOre_OutputSnapshot_t snapshot;
  if (!AutoOre_ReadOutputSnapshot(ctrl, &snapshot)) {
    return AUTO_ORE_RESOURCE_NONE;
  }
  return snapshot.owned_resource_mask;
}

void AutoOre_SetReservedResourceMask(AutoOre_t *ctrl, uint8_t resource_mask) {
  if (ctrl == 0) {
    return;
  }
  ctrl->reserved_resource_mask = resource_mask;
  AutoOre_PublishOutputSnapshot(ctrl);
}

uint8_t AutoOre_GetActiveResourceMask(const AutoOre_t *ctrl) {
  return AutoOre_GetOwnedResourceMask(ctrl);
}

AutoOre_Position_t AutoOre_GetActivePosition(const AutoOre_t *ctrl) {
  return (ctrl == 0) ? AUTO_ORE_POSITION_NONE : ctrl->active_position;
}

uint8_t AutoOre_GetStepIndex(const AutoOre_t *ctrl) {
  return (ctrl == 0) ? 0u : ctrl->step_index;
}

const ArmSimple_CMD_t *AutoOre_GetArmCommand(const AutoOre_t *ctrl) {
  return (ctrl != 0 && ctrl->state == AUTO_ORE_STATE_RUNNING &&
          ctrl->arm_cmd_valid)
             ? &ctrl->arm_cmd
             : 0;
}

const OreStore_CMD_t *AutoOre_GetOreStoreCommand(const AutoOre_t *ctrl) {
  return (ctrl != 0 && ctrl->state == AUTO_ORE_STATE_RUNNING &&
          ctrl->ore_store_cmd_valid)
             ? &ctrl->ore_store_cmd
             : 0;
}

const Pole_CMD_t *AutoOre_GetPoleCommand(const AutoOre_t *ctrl) {
  return (ctrl != 0 && ctrl->pole_cmd_valid) ? &ctrl->pole_cmd : 0;
}

const Chassis_CMD_t *AutoOre_GetChassisCommand(const AutoOre_t *ctrl) {
  return (ctrl != 0 && ctrl->chassis_cmd_valid) ? &ctrl->chassis_cmd : 0;
}
