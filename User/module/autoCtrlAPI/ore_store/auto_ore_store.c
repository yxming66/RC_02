#include "module/autoCtrlAPI/ore_store/auto_ore_store.h"

#include "module/autoCtrlAPI/core/auto_ctrl_math.h"
#include "module/config.h"

#include <string.h>

#define AUTO_ORE_DEFAULT_STORE_ARM_SETTLE_MS (100u)
#define AUTO_ORE_DEFAULT_STORE_CYLINDER_CLOSE_MS (50u)
#define AUTO_ORE_DEFAULT_STORE_ARM_SUCTION_OFF_MS (300u)
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
#define AUTO_ORE_FUSED_ARM_PHOTO_ENABLE_JOINT1_RAD (0.6981317f)

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
static float AutoOre_SelectPrealignWz(AutoOre_t *ctrl);

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

static void AutoOre_SetFusedStepSidePhase(AutoOre_t *ctrl, uint8_t phase,
                                          uint32_t now_ms) {
  ctrl->fused_step_phase = phase;
  ctrl->fused_step_distance_latch_valid = false;
  ctrl->fused_step_photo_seen = false;
  ctrl->fused_step_wheel_delta_rad = 0.0f;
  ctrl->fused_step_target_wheel_delta_rad = 0.0f;
  ctrl->fused_step_phase_enter_time_ms = now_ms;
  ctrl->fused_step_condition_met = false;
  ctrl->fused_step_condition_time_ms = 0u;
}

static bool AutoOre_FusedStepWaitPoleTarget(AutoOre_t *ctrl, uint32_t now_ms,
                                            bool at_target) {
  if (!ctrl->fused_step_condition_met) {
    ctrl->fused_step_condition_met = true;
    ctrl->fused_step_condition_time_ms = now_ms;
    return false;
  }
  if (now_ms == ctrl->fused_step_condition_time_ms) {
    return false;
  }
  return at_target;
}

static void AutoOre_FusedPickStoreNextStep(AutoOre_t *ctrl) {
  ctrl->fused_pick_store_step_index++;
  ctrl->fused_pick_store_condition_met = false;
  ctrl->fused_pick_store_condition_time_ms = 0u;
  ctrl->distance_latch_valid = false;
  ctrl->wheel_delta_rad = 0.0f;
  ctrl->target_wheel_delta_rad = 0.0f;
}

static bool AutoOre_FusedPickStoreWaitConditionThenDelay(AutoOre_t *ctrl,
                                                         uint32_t now_ms,
                                                         bool condition,
                                                         uint32_t delay_ms) {
  if (!condition) {
    ctrl->fused_pick_store_condition_met = false;
    return false;
  }
  if (!ctrl->fused_pick_store_condition_met) {
    ctrl->fused_pick_store_condition_met = true;
    ctrl->fused_pick_store_condition_time_ms = now_ms;
    return false;
  }
  return (now_ms - ctrl->fused_pick_store_condition_time_ms) >= delay_ms;
}

static bool AutoOre_FusedPickStoreWaitArmCommandTarget(AutoOre_t *ctrl,
                                                       uint32_t now_ms) {
  if (!ctrl->fused_pick_store_condition_met) {
    ctrl->fused_pick_store_condition_met = true;
    ctrl->fused_pick_store_condition_time_ms = now_ms;
    return false;
  }
  if (now_ms == ctrl->fused_pick_store_condition_time_ms) {
    return false;
  }
  return ctrl->feedback.arm_at_target;
}

static bool AutoOre_FusedPickStoreWaitOreStoreCommandTarget(
    AutoOre_t *ctrl, uint32_t now_ms) {
  if (!ctrl->fused_pick_store_condition_met) {
    ctrl->fused_pick_store_condition_met = true;
    ctrl->fused_pick_store_condition_time_ms = now_ms;
    return false;
  }
  if (now_ms == ctrl->fused_pick_store_condition_time_ms) {
    return false;
  }
  return ctrl->feedback.ore_store_all_at_target;
}

static uint32_t AutoOre_FusedPickStoreStepElapsed(AutoOre_t *ctrl,
                                                  uint32_t now_ms) {
  if (!ctrl->fused_pick_store_condition_met) {
    ctrl->fused_pick_store_condition_met = true;
    ctrl->fused_pick_store_condition_time_ms = now_ms;
    return 0u;
  }
  return now_ms - ctrl->fused_pick_store_condition_time_ms;
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

static bool AutoOre_CommandPoleTargetValues(AutoOre_t *ctrl,
                                            float front_lift,
                                            float rear_lift) {
  memset(&ctrl->pole_cmd, 0, sizeof(ctrl->pole_cmd));
  ctrl->pole_cmd.mode = POLE_MODE_ACTIVE;
  ctrl->pole_cmd.lift[0] = 0.0f;
  ctrl->pole_cmd.lift[1] = 0.0f;
  ctrl->pole_cmd.auto_target_enable[0] = true;
  ctrl->pole_cmd.auto_target_enable[1] = true;
  ctrl->pole_cmd.auto_target_lift[0] = front_lift;
  ctrl->pole_cmd.auto_target_lift[1] = rear_lift;
  ctrl->pole_cmd.auto_lift_speed[0] = 0.0f;
  ctrl->pole_cmd.auto_lift_speed[1] = 0.0f;
  ctrl->pole_cmd.auto_lift_accel[0] = 0.0f;
  ctrl->pole_cmd.auto_lift_accel[1] = 0.0f;
  ctrl->pole_cmd.disable_lift_accel = false;
  ctrl->pole_cmd_valid = true;
  return true;
}

static bool AutoOre_CommandPoleTarget(AutoOre_t *ctrl,
                                      const float target_lift[2]) {
  if (target_lift == 0) {
    ctrl->pole_cmd_valid = false;
    return false;
  }
  return AutoOre_CommandPoleTargetValues(ctrl, target_lift[0],
                                         target_lift[1]);
}

static void AutoOre_CommandChassisMove(AutoOre_t *ctrl, float vx_mps) {
  memset(&ctrl->chassis_cmd, 0, sizeof(ctrl->chassis_cmd));
  ctrl->chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
  ctrl->chassis_cmd.ctrl_vec.vx = vx_mps;
  ctrl->chassis_cmd.ctrl_vec.vy = 0.0f;
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

static bool AutoOre_FusedStepWheelDeltaMoveReached(
    AutoOre_t *ctrl, float target_wheel_delta_rad) {
  if (ctrl == 0 || target_wheel_delta_rad <= 0.0f) {
    return false;
  }
  ctrl->fused_step_target_wheel_delta_rad = target_wheel_delta_rad;

  if (!ctrl->fused_step_distance_latch_valid) {
    for (uint8_t i = 0u; i < 4u; ++i) {
      ctrl->fused_step_distance_start_wheel_rad[i] =
          ctrl->feedback.wheel_position_rad[i];
    }
    ctrl->fused_step_wheel_delta_rad = 0.0f;
    ctrl->fused_step_distance_latch_valid = true;
    return false;
  }

  float wheel_delta_abs_sum_rad = 0.0f;
  for (uint8_t i = 0u; i < 4u; ++i) {
    wheel_delta_abs_sum_rad += AutoOre_AbsFloat(
        ctrl->feedback.wheel_position_rad[i] -
        ctrl->fused_step_distance_start_wheel_rad[i]);
  }
  ctrl->fused_step_wheel_delta_rad = wheel_delta_abs_sum_rad * 0.25f;
  return ctrl->fused_step_wheel_delta_rad >= target_wheel_delta_rad;
}

static uint32_t AutoOre_FusedStepPhaseElapsed(const AutoOre_t *ctrl,
                                              uint32_t now_ms) {
  if (ctrl == 0 || ctrl->fused_step_phase_enter_time_ms == 0u) {
    return 0u;
  }
  return now_ms - ctrl->fused_step_phase_enter_time_ms;
}

static bool AutoOre_FusedStepMoveGateReady(AutoOre_t *ctrl, uint32_t now_ms,
                                           float wheel_delta_rad,
                                           uint32_t timeout_ms) {
  const bool distance_ready =
      AutoOre_FusedStepWheelDeltaMoveReached(ctrl, wheel_delta_rad);
  const bool timeout_ready =
      timeout_ms > 0u && AutoOre_FusedStepPhaseElapsed(ctrl, now_ms) >=
                             timeout_ms;
  return wheel_delta_rad <= 0.0f || distance_ready || timeout_ready;
}

static bool AutoOre_FusedPhotoFallingEdge(AutoOre_t *ctrl, bool triggered) {
  if (ctrl == 0) {
    return false;
  }
  if (!ctrl->fused_step_photo_seen) {
    if (triggered) {
      ctrl->fused_step_photo_seen = true;
    }
    return false;
  }
  return !triggered;
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
          SUCTION_ON, &ctrl->param.arm_speed.store_place) ||
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
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE,
          SUCTION_OFF, &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, false)) {
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
  return action == AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD ||
         action == AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD ||
         action == AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD ||
         action == AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_400_HEAD;
}

static bool AutoOre_FusedActionIsAscend(AutoOre_Action_t action) {
  return action == AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD ||
         action == AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD;
}

static bool AutoOre_FusedActionUses400(AutoOre_Action_t action) {
  return action == AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD ||
         action == AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_400_HEAD;
}

static const AutoOre_FusedParam_t *AutoOre_FusedParam(const AutoOre_t *ctrl) {
  if (ctrl == 0) {
    return 0;
  }
  switch (ctrl->action) {
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
      return &ctrl->param.fused_step_pick_store_ascend_200_head;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
      return &ctrl->param.fused_step_pick_store_descend_200_head;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
      return &ctrl->param.fused_step_pick_store_ascend_400_head;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_400_HEAD:
      return &ctrl->param.fused_step_pick_store_descend_400_head;
    default:
      return 0;
  }
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
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
      return ARM_SIMPLE_BEHAVIOR_PICK_POS_200;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_400_HEAD:
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

static const float *AutoOre_FusedStepStartPoleTarget(const AutoOre_t *ctrl) {
  if (ctrl == 0 || ctrl->param.pole_param == 0) {
    return 0;
  }
  switch (ctrl->action) {
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_400_HEAD:
      return ctrl->param.pole_param->preset.step_400_all_extend;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
      return ctrl->param.pole_param->preset.step_200_all_extend;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
      return ctrl->param.pole_param->preset.step_200_small;
    default:
      return 0;
  }
}

static const float *AutoOre_FusedFrontRetractPoleTarget(const AutoOre_t *ctrl) {
  if (ctrl == 0 || ctrl->param.pole_param == 0) {
    return 0;
  }
  switch (ctrl->action) {
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_400_HEAD:
      return ctrl->param.pole_param->preset.step_400_front_retract;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
      return ctrl->param.pole_param->preset.step_200_front_retract;
    default:
      return 0;
  }
}

static const float *AutoOre_FusedAllRetractPoleTarget(const AutoOre_t *ctrl) {
  if (ctrl == 0 || ctrl->param.pole_param == 0) {
    return 0;
  }
  switch (ctrl->action) {
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_400_HEAD:
      return ctrl->param.pole_param->preset.step_400_all_retract;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
      return ctrl->param.pole_param->preset.step_200_all_retract;
    default:
      return 0;
  }
}

static bool AutoOre_CommandFusedPolePair(AutoOre_t *ctrl,
                                         const float front_target[2],
                                         const float rear_target[2]) {
  if (front_target == 0 || rear_target == 0) {
    return false;
  }
  return AutoOre_CommandPoleTargetValues(ctrl, front_target[0],
                                         rear_target[1]);
}

static AutoOre_Action_t AutoOre_FusedDefaultPickAction(
    AutoOre_Action_t action) {
  switch (action) {
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_400_HEAD:
      return AUTO_ORE_ACTION_PICK_POS_400;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
      return AUTO_ORE_ACTION_PICK_POS_200;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
      return AUTO_ORE_ACTION_PICK_NEG_200;
    default:
      return AUTO_ORE_ACTION_PICK_POS_200;
  }
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

static bool AutoOre_FusedArmPhotoConfirmed(AutoOre_t *ctrl,
                                           uint32_t now_ms) {
  const AutoOre_FusedParam_t *param = AutoOre_FusedParam(ctrl);
  if (param == 0) {
    return false;
  }
  if (!param->use_arm_photo_confirm) {
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

static AutoOre_Position_t AutoOre_FusedSelectStorePosition(AutoOre_t *ctrl) {
  AutoOre_ApplyFeedbackOccupancy(ctrl);
  const AutoOre_Position_t position = AutoOre_SelectStorePosition(&ctrl->occupancy);
  return AutoOre_IsPositionValid(position) ? position : AUTO_ORE_POSITION_ARM;
}

static void AutoOre_FusedStoreNextStep(AutoOre_t *ctrl) {
  ctrl->fused_store_step_index++;
  ctrl->fused_store_step_phase = 0u;
  ctrl->fused_pick_store_condition_met = false;
  ctrl->fused_pick_store_condition_time_ms = 0u;
}

static void AutoOre_FusedStoreMarkDone(AutoOre_t *ctrl) {
  ctrl->fused_store_done = true;
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
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_FusedPickStoreWaitConditionThenDelay(
              ctrl, now_ms,
              ctrl->feedback.arm_at_target &&
                  ctrl->feedback.ore_store_all_at_target,
              0u)) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 1:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE, SUCTION_ON,
                              &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_MID_WAIT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_FusedPickStoreWaitConditionThenDelay(
              ctrl, now_ms, ctrl->feedback.arm_at_target,
              AutoOre_StoreArmSettleMs(ctrl))) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 2:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE, SUCTION_ON,
                              &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_MID_WAIT, true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_FusedPickStoreWaitConditionThenDelay(
              ctrl, now_ms, true, AutoOre_StoreCylinderCloseMs(ctrl))) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 3:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE, SUCTION_OFF,
                              &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, true)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_FusedPickStoreWaitOreStoreCommandTarget(ctrl, now_ms) &&
          AutoOre_FusedPickStoreStepElapsed(ctrl, now_ms) >=
              AutoOre_StoreArmSuctionOffMs(ctrl)) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 4:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE, SUCTION_OFF,
                              &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_LIFT, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_FusedPickStoreWaitConditionThenDelay(
              ctrl, now_ms, true, AutoOre_StoreCylinderOpenMs(ctrl))) {
        AutoOre_FusedStoreNextStep(ctrl);
      }
      return;
    case 5:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STORE_ORE, SUCTION_OFF,
                              &ctrl->param.arm_speed.store_place) ||
          !AutoOre_CommandOreStore(ctrl, ORE_STORE_TRANSFORM_STANDBY, false)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_FusedPickStoreWaitOreStoreCommandTarget(ctrl, now_ms)) {
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
      if (AutoOre_FusedPickStoreWaitConditionThenDelay(
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
      if (AutoOre_FusedPickStoreWaitConditionThenDelay(
              ctrl, now_ms, true, AutoOre_StoreCylinderCloseMs(ctrl))) {
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
    ctrl->fused_pick_store_condition_met = false;
    ctrl->fused_pick_store_condition_time_ms = 0u;
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
      AutoOre_FailInvalidOccupancy(ctrl);
      return;
  }
}

static void AutoOre_FinishFusedStepSide(AutoOre_t *ctrl) {
  ctrl->fused_step_done = true;
  AutoOre_CommandChassisHold(ctrl);
}

static void AutoOre_RunFusedAscendStepSide(AutoOre_t *ctrl, uint32_t now_ms,
                                           const AutoOre_FusedParam_t *fused) {
  const float *start_target = AutoOre_FusedStepStartPoleTarget(ctrl);
  const float *front_retract_target = AutoOre_FusedFrontRetractPoleTarget(ctrl);
  const float *all_retract_target = AutoOre_FusedAllRetractPoleTarget(ctrl);
  if (start_target == 0 || front_retract_target == 0 ||
      all_retract_target == 0) {
    AutoOre_FailInvalidParam(ctrl);
    return;
  }

  switch (ctrl->fused_step_phase) {
    case 0:
      if (!AutoOre_CommandPoleTarget(ctrl, start_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMoveYawRate(ctrl, fused->step_start_vx_mps,
                                        ctrl->feedback.yaw_rate_cmd_rad_s);
      if (ctrl->feedback.pe13_photo1_triggered ||
          AutoOre_FusedStepMoveGateReady(
              ctrl, now_ms, fused->step_start_wheel_delta_rad,
              AutoOre_FusedPrecontactTimeoutMs(ctrl, fused))) {
        AutoOre_SetFusedStepSidePhase(ctrl, 1u, now_ms);
      }
      return;

    case 1:
      if (!AutoOre_CommandPoleTarget(ctrl, front_retract_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMoveYawRate(ctrl, fused->step_start_vx_mps,
                                        ctrl->feedback.yaw_rate_cmd_rad_s);
      if (AutoOre_FusedStepWaitPoleTarget(
              ctrl, now_ms, ctrl->feedback.pole_front_at_target)) {
        AutoOre_SetFusedStepSidePhase(ctrl, 2u, now_ms);
      } else {
        (void)AutoOre_CheckFusedParallelTimeout(ctrl, now_ms);
      }
      return;

    case 2:
      if (!AutoOre_CommandPoleTarget(ctrl, front_retract_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMoveYawRate(ctrl, fused->step_start_vx_mps,
                                        ctrl->feedback.yaw_rate_cmd_rad_s);
      if (ctrl->feedback.pa2_photo3_triggered ||
          ctrl->feedback.pa0_photo4_triggered) {
        AutoOre_SetFusedStepSidePhase(ctrl, 3u, now_ms);
      } else {
        (void)AutoOre_CheckFusedParallelTimeout(ctrl, now_ms);
      }
      return;

    case 3:
      if (!AutoOre_CommandPoleTarget(ctrl, all_retract_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMoveYawRate(ctrl, fused->step_start_vx_mps,
                                        ctrl->feedback.yaw_rate_cmd_rad_s);
      if (AutoOre_FusedStepWaitPoleTarget(
              ctrl, now_ms, ctrl->feedback.pole_all_at_target)) {
        AutoOre_FinishFusedStepSide(ctrl);
      } else {
        (void)AutoOre_CheckFusedParallelTimeout(ctrl, now_ms);
      }
      return;

    default:
      AutoOre_FinishFusedStepSide(ctrl);
      return;
  }
}

static void AutoOre_RunFusedDescendStepSide(AutoOre_t *ctrl, uint32_t now_ms,
                                            const AutoOre_FusedParam_t *fused) {
  if (ctrl->param.pole_param == 0) {
    AutoOre_FailInvalidParam(ctrl);
    return;
  }
  const float *start_target = AutoOre_FusedStepStartPoleTarget(ctrl);
  const float *all_extend_target = AutoOre_FusedActionUses400(ctrl->action)
                                       ? ctrl->param.pole_param->preset
                                             .step_400_all_extend
                                       : ctrl->param.pole_param->preset
                                             .step_200_all_extend;
  const float *all_retract_target = AutoOre_FusedAllRetractPoleTarget(ctrl);
  if (start_target == 0 || all_extend_target == 0 ||
      all_retract_target == 0) {
    AutoOre_FailInvalidParam(ctrl);
    return;
  }

  switch (ctrl->fused_step_phase) {
    case 0:
      if (!AutoOre_CommandPoleTarget(ctrl, start_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMoveYawRate(ctrl, fused->step_start_vx_mps,
                                        ctrl->feedback.yaw_rate_cmd_rad_s);
      if (AutoOre_FusedPhotoFallingEdge(ctrl,
                                        ctrl->feedback.pe9_photo2_triggered) ||
          AutoOre_FusedStepMoveGateReady(
              ctrl, now_ms, fused->step_start_wheel_delta_rad,
              AutoOre_FusedPrecontactTimeoutMs(ctrl, fused))) {
        AutoOre_SetFusedStepSidePhase(ctrl, 1u, now_ms);
      }
      return;

    case 1:
      if (!AutoOre_CommandFusedPolePair(ctrl, all_extend_target,
                                        all_retract_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMoveYawRate(ctrl, fused->step_start_vx_mps,
                                        ctrl->feedback.yaw_rate_cmd_rad_s);
      if (AutoOre_FusedStepWaitPoleTarget(
              ctrl, now_ms, ctrl->feedback.pole_front_at_target)) {
        AutoOre_SetFusedStepSidePhase(ctrl, 2u, now_ms);
      } else {
        (void)AutoOre_CheckFusedParallelTimeout(ctrl, now_ms);
      }
      return;

    case 2:
      if (!AutoOre_CommandFusedPolePair(ctrl, all_extend_target,
                                        all_retract_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMoveYawRate(ctrl, fused->step_start_vx_mps,
                                        ctrl->feedback.yaw_rate_cmd_rad_s);
      if (AutoOre_FusedPhotoFallingEdge(ctrl,
                                        ctrl->feedback.pa0_photo4_triggered)) {
        AutoOre_SetFusedStepSidePhase(ctrl, 3u, now_ms);
      } else {
        (void)AutoOre_CheckFusedParallelTimeout(ctrl, now_ms);
      }
      return;

    case 3:
      if (!AutoOre_CommandPoleTarget(ctrl, all_extend_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMoveYawRate(ctrl, fused->step_start_vx_mps,
                                        ctrl->feedback.yaw_rate_cmd_rad_s);
      if (AutoOre_FusedStepWaitPoleTarget(
              ctrl, now_ms, ctrl->feedback.pole_all_at_target)) {
        AutoOre_SetFusedStepSidePhase(ctrl, 4u, now_ms);
      } else {
        (void)AutoOre_CheckFusedParallelTimeout(ctrl, now_ms);
      }
      return;

    case 4:
      if (!AutoOre_CommandPoleTarget(ctrl, all_retract_target)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      AutoOre_CommandChassisMoveYawRate(ctrl, fused->step_start_vx_mps,
                                        ctrl->feedback.yaw_rate_cmd_rad_s);
      if (AutoOre_FusedStepWaitPoleTarget(
              ctrl, now_ms, ctrl->feedback.pole_all_at_target)) {
        AutoOre_FinishFusedStepSide(ctrl);
      } else {
        (void)AutoOre_CheckFusedParallelTimeout(ctrl, now_ms);
      }
      return;

    default:
      AutoOre_FinishFusedStepSide(ctrl);
      return;
  }
}

static void AutoOre_RunFusedStepSide(AutoOre_t *ctrl, uint32_t now_ms,
                                     const AutoOre_FusedParam_t *fused) {
  if (ctrl == 0 || fused == 0 || ctrl->fused_step_done) {
    return;
  }
  if (ctrl->fused_step_phase_enter_time_ms == 0u) {
    ctrl->fused_step_phase_enter_time_ms = now_ms;
  }

  if (AutoOre_FusedActionIsAscend(ctrl->action)) {
    AutoOre_RunFusedAscendStepSide(ctrl, now_ms, fused);
  } else {
    AutoOre_RunFusedDescendStepSide(ctrl, now_ms, fused);
  }
}

static void AutoOre_RunFusedPickStoreSide(AutoOre_t *ctrl, uint32_t now_ms,
                                          const AutoOre_FusedParam_t *fused) {
  if (ctrl == 0 || fused == 0 || ctrl->fused_store_done) {
    return;
  }

  switch (ctrl->fused_pick_store_step_index) {
    case 0:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_STANDBY, SUCTION_ON,
                              &ctrl->param.arm_speed.pick_standby)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_FusedPickStoreWaitConditionThenDelay(
              ctrl, now_ms,
              ctrl->feedback.arm_at_target && ctrl->feedback.pole_all_at_target,
              0u)) {
        AutoOre_FusedPickStoreNextStep(ctrl);
      }
      return;

    case 1:
      if (!AutoOre_CommandArm(ctrl, AutoOre_FusedPickArmPoint(ctrl),
                              SUCTION_ON,
                              &ctrl->param.arm_speed.pick_place)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_FusedPickStoreWaitArmCommandTarget(ctrl, now_ms)) {
        AutoOre_FusedPickStoreNextStep(ctrl);
      }
      return;

    case 2:
      if (!AutoOre_CommandArm(ctrl, AutoOre_FusedPickArmPoint(ctrl),
                              SUCTION_ON,
                              &ctrl->param.arm_speed.pick_fetch)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      const bool precontact_distance_ready =
          AutoOre_WheelDeltaMoveReached(ctrl,
                                        fused->precontact_wheel_delta_rad);
      const bool precontact_timeout_ready =
          AutoOre_FusedPickStoreStepElapsed(ctrl, now_ms) >=
          AutoOre_FusedPrecontactTimeoutMs(ctrl, fused);
      if (precontact_distance_ready || precontact_timeout_ready) {
        AutoOre_FusedPickStoreNextStep(ctrl);
      }
      return;

    case 3:
      if (!AutoOre_CommandArm(ctrl, ARM_SIMPLE_BEHAVIOR_PICK_LIFT_DETECT,
                              SUCTION_ON,
                              &ctrl->param.arm_speed.pick_lift_detect)) {
        AutoOre_FailInvalidParam(ctrl);
        return;
      }
      if (AutoOre_FusedPickStoreWaitConditionThenDelay(
              ctrl, now_ms, ctrl->feedback.arm_at_target,
              AutoOre_FusedPickLiftDetectMs(ctrl)) &&
          AutoOre_FusedArmPhotoConfirmed(ctrl, now_ms)) {
        AutoOre_SetArmHasOre(ctrl, true);
        ctrl->fused_pick_done = true;
        AutoOre_FusedPickStoreNextStep(ctrl);
      }
      return;

    case 4:
      if (!ctrl->fused_store_done) {
        AutoOre_RunFusedStore(ctrl, now_ms);
      }
      return;

    default:
      ctrl->fused_store_done = true;
      return;
  }
}

static void AutoOre_RunFusedDedicatedParallel(AutoOre_t *ctrl,
                                              uint32_t now_ms,
                                              const AutoOre_FusedParam_t *fused) {
  AutoOre_EnterStep(ctrl, now_ms);
  ctrl->step_phase = ctrl->fused_step_phase;

  if (!ctrl->fused_step_done) {
    AutoOre_RunFusedStepSide(ctrl, now_ms, fused);
    ctrl->step_phase = ctrl->fused_step_phase;
  } else {
    AutoOre_CommandChassisHold(ctrl);
  }

  if (ctrl->state != AUTO_ORE_STATE_RUNNING) {
    return;
  }

  if (!ctrl->fused_store_done) {
    AutoOre_RunFusedPickStoreSide(ctrl, now_ms, fused);
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

static void AutoOre_RunStepPickStoreFused(AutoOre_t *ctrl, uint32_t now_ms) {
  const AutoOre_FusedParam_t *fused = AutoOre_FusedParam(ctrl);
  if (fused == 0 || AutoOre_FusedStepStartPoleTarget(ctrl) == 0) {
    AutoOre_FailInvalidParam(ctrl);
    return;
  }

  switch (ctrl->step_index) {
    case 0:
      AutoOre_EnterStep(ctrl, now_ms);
      if (AutoOre_RunPickPrealign(ctrl, now_ms)) {
        AutoOre_NextStep(ctrl);
      } else {
        (void)AutoOre_CheckTimeout(ctrl, now_ms);
      }
      return;
    case 1:
      AutoOre_RunFusedDedicatedParallel(ctrl, now_ms, fused);
      return;
    case 2:
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

static bool AutoOre_StartResolved(AutoOre_t *ctrl, AutoOre_Action_t action,
                                  AutoOre_Position_t position,
                                  bool occupancy_suspect,
                                  uint32_t now_ms) {
  if (!AutoOre_IsPositionValid(position)) {
    AutoOre_FailInvalidOccupancy(ctrl);
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
  ctrl->fused_step_done = false;
  ctrl->fused_pick_done = false;
  ctrl->fused_store_done = false;
  ctrl->pick_lift_confirmed = false;
  ctrl->prealign_yaw_target_valid = false;
  ctrl->prealign_target_yaw_rad = 0.0f;
  ctrl->prealign_yaw_error_rad = 0.0f;
  ctrl->fused_arm_photo_since_ms = 0u;
  ctrl->fused_pick_store_step_index = 0u;
  ctrl->fused_pick_store_condition_met = false;
  ctrl->fused_pick_store_condition_time_ms = 0u;
  ctrl->fused_store_step_index = 0u;
  ctrl->fused_store_step_phase = 0u;
  ctrl->fused_step_phase = 0u;
  ctrl->fused_step_distance_latch_valid = false;
  ctrl->fused_step_photo_seen = false;
  ctrl->fused_step_wheel_delta_rad = 0.0f;
  ctrl->fused_step_target_wheel_delta_rad = 0.0f;
  ctrl->fused_step_phase_enter_time_ms = 0u;
  ctrl->fused_step_condition_met = false;
  ctrl->fused_step_condition_time_ms = 0u;
  ctrl->fused_store_position = AUTO_ORE_POSITION_NONE;
  AutoOre_ResetDistanceGate(ctrl);
  AutoOre_ClearOutputs(ctrl);
  return true;
}

static bool AutoOre_Start(AutoOre_t *ctrl, AutoOre_Action_t action,
                          uint32_t now_ms) {
  if (ctrl == 0 || AutoOre_IsBusy(ctrl)) {
    return false;
  }
  AutoOre_ApplyFeedbackOccupancy(ctrl);
  AutoOre_Position_t position = AUTO_ORE_POSITION_NONE;
  if (action == AUTO_ORE_ACTION_STORE) {
    position = AutoOre_SelectStorePosition(&ctrl->occupancy);
  } else if (action == AUTO_ORE_ACTION_RELEASE) {
    if (ctrl->occupancy.arm_has_ore) {
      position = AUTO_ORE_POSITION_ARM;
    }
  } else if (action == AUTO_ORE_ACTION_CHAMBER) {
    position = AutoOre_SelectChamberPosition(&ctrl->occupancy);
  } else if (AutoOre_ActionIsPick(action)) {
    position = AUTO_ORE_POSITION_ARM;
  } else if (AutoOre_ActionIsFused(action)) {
    position = AUTO_ORE_POSITION_ARM;
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
  AutoOre_ApplyFeedbackOccupancy(ctrl);
  return AutoOre_StartResolved(ctrl, AUTO_ORE_ACTION_STORE, position, false,
                               now_ms);
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

bool AutoOre_StartStepPickStoreDescend400Head(AutoOre_t *ctrl,
                                              uint32_t now_ms) {
  return AutoOre_Start(ctrl, AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_400_HEAD,
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
  } else if (AutoOre_ActionIsFused(ctrl->action)) {
    AutoOre_RunStepPickStoreFused(ctrl, now_ms);
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

const Chassis_CMD_t *AutoOre_GetChassisCommand(const AutoOre_t *ctrl) {
  return (ctrl != 0 && ctrl->chassis_cmd_valid) ? &ctrl->chassis_cmd : 0;
}
