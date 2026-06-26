/*
    auto_ctrl_feed Task
    Dedicated task for collecting sensor data and feeding AutoCtrl feedback.
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "main.h"
#include "device/dr16.h"
#include "device/ir_dock/ir_dock.h"
#include "device/photo_transfer.h"
#include "module/autoCtrlAPI/api/auto_ctrl_api.h"
#include "module/autoCtrlAPI/ore_store/auto_ore_store.h"
#include "module/autoCtrlAPI/rod/auto_rod_spearhead.h"
#include "module/chassis.h"
#include "module/config.h"
#include "module/mrlink_pc_comm/mrlink_pc_comm.h"

#include <math.h>

/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
extern Chassis_IMU_t chassis_imu;
extern DR16_t dr16;

auto_ctrl_t auto_ctrl;
bool auto_ctrl_inited = false;
AutoOre_t auto_ore_ctrl;
bool auto_ore_inited = false;

/* Ozone/调试器手动触发一键动作：request=1存矿，2放矿，3上膛，4中止，5取正400，6取正200，7取负200，8取矛头，9矛头sick校正，10放矿sick校正，11等待对接。 */
volatile AutoOre_DebugControl_t g_auto_ore_debug = {0};//手动调用一键函数

AutoRodSpearhead_t auto_rod_spearhead_ctrl;
bool auto_rod_spearhead_inited = false;
AutoSickCorrect_t auto_sick_correct_ctrl;
bool auto_sick_correct_inited = false;
bool auto_ctrl_local_yaw_zero_initialized = false; 
float auto_ctrl_local_yaw_zero_rad = 0.0f;
auto_ctrl_feedback_t feedback = {0};
static Sick_Output_t auto_ctrl_sick_output = {0};
static PC_AutoAction_t auto_action_last_action = PC_AUTO_ACTION_NONE;
static AutoOre_Action_t auto_ore_last_action = AUTO_ORE_ACTION_NONE;
static PhotoTransfer_Snapshot_t photo_transfer_snapshot = {0};
static bool photo_transfer_inited = false;

#ifndef AUTO_CTRL_POLE_TARGET_THRESHOLD_RAD
#define AUTO_CTRL_POLE_TARGET_THRESHOLD_RAD (0.30f)
#endif

#ifndef AUTO_CTRL_POLE_TARGET_STABLE_CYCLES
#define AUTO_CTRL_POLE_TARGET_STABLE_CYCLES (5u)
#endif

static uint8_t pole_front_at_target_stable_count = 0u;
static uint8_t pole_rear_at_target_stable_count = 0u;
static uint8_t pole_all_at_target_stable_count = 0u;
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
static void AutoCtrlFeed_UpdateAutoOre(uint32_t now_ms);

static float AutoCtrlFeed_SelectYawRad(void) {
  if (AutoCtrl_GetYawSource(&auto_ctrl) == AUTO_CTRL_YAW_SOURCE_PC &&
      MrlinkPc_IsHeartbeatValid()) {
    const PC_ImuCMD_t *pc_imu = MrlinkPc_GetImuCMD();
    if (pc_imu != NULL && isfinite(pc_imu->yaw)) {
      return pc_imu->yaw;
    }
  }

  return chassis_imu.eulr.yaw;
}

static void AutoCtrlFeed_UpdateYawRateCommand(void) {
  float wz_rad_s = 0.0f;

  if (AutoCtrl_GetYawSource(&auto_ctrl) == AUTO_CTRL_YAW_SOURCE_PC &&
      MrlinkPc_IsHeartbeatValid()) {
    const PC_ChassisCMD_t *pc_chassis_cmd = MrlinkPc_GetChassisCMD();
    if (pc_chassis_cmd != NULL) {
      wz_rad_s = pc_chassis_cmd->wz;
    }
  }

  AutoCtrl_SetYawRateCommand(&auto_ctrl, wz_rad_s);
}

static void AutoCtrlFeed_CacheLocalYawZero(void) {
  if (!auto_ctrl_local_yaw_zero_initialized) {
    if (!isfinite(chassis_imu.eulr.yaw)) {
      return;
    }

    auto_ctrl_local_yaw_zero_rad = chassis_imu.eulr.yaw;
    auto_ctrl_local_yaw_zero_initialized = true;
  }
}

static bool AutoCtrlFeed_DebouncePoleReady(bool raw_ready,
                                           uint8_t *stable_count) {
  if (stable_count == NULL) {
    return false;
  }

  if (!raw_ready) {
    *stable_count = 0u;
    return false;
  }

  if (*stable_count < AUTO_CTRL_POLE_TARGET_STABLE_CYCLES) {
    (*stable_count)++;
  }
  return *stable_count >= AUTO_CTRL_POLE_TARGET_STABLE_CYCLES;
}

static bool AutoCtrlFeed_ReadPhotoTransferBit(PhotoTransfer_Bit_t bit) {
  return PhotoTransfer_IsBitTriggered(&photo_transfer_snapshot, bit);
}

static bool AutoCtrlFeed_ReadRodSpearheadPhoto(void) {
  return AutoCtrlFeed_ReadPhotoTransferBit(PHOTO_TRANSFER_BIT_SPEARHEAD);
}

static bool AutoCtrlFeed_ReadOreLowPhoto(void) {
  return AutoCtrlFeed_ReadPhotoTransferBit(PHOTO_TRANSFER_BIT_ORE_LOW);
}

static bool AutoCtrlFeed_ReadOreHighPhoto(void) {
  return AutoCtrlFeed_ReadPhotoTransferBit(PHOTO_TRANSFER_BIT_ORE_HIGH);
}

static bool AutoCtrlFeed_ReadArmOrePhoto(void) {
  return AutoCtrlFeed_ReadPhotoTransferBit(PHOTO_TRANSFER_BIT_ARM_ORE);
}

static PC_AutoAction_t AutoCtrlFeed_MapOreAction(AutoOre_Action_t action) {
  switch (action) {
    case AUTO_ORE_ACTION_STORE:
      return PC_AUTO_ACTION_STORE;
    case AUTO_ORE_ACTION_RELEASE:
      return PC_AUTO_ACTION_RELEASE;
    case AUTO_ORE_ACTION_CHAMBER:
      return PC_AUTO_ACTION_CHAMBER;
    case AUTO_ORE_ACTION_PICK_POS_400:
      return PC_AUTO_ACTION_PICK_POS_400;
    case AUTO_ORE_ACTION_PICK_POS_200:
      return PC_AUTO_ACTION_PICK_POS_200;
    case AUTO_ORE_ACTION_PICK_NEG_200:
      return PC_AUTO_ACTION_PICK_NEG_200;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
      return PC_AUTO_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
      return PC_AUTO_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
      return PC_AUTO_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD;
    case AUTO_ORE_ACTION_NONE:
    default:
      return PC_AUTO_ACTION_NONE;
  }
}

static AutoOre_Action_t AutoCtrlFeed_RequestToOreAction(
    AutoOre_DebugRequest_t request) {
  switch (request) {
    case AUTO_ORE_DEBUG_REQUEST_STORE:
      return AUTO_ORE_ACTION_STORE;
    case AUTO_ORE_DEBUG_REQUEST_RELEASE:
      return AUTO_ORE_ACTION_RELEASE;
    case AUTO_ORE_DEBUG_REQUEST_CHAMBER:
      return AUTO_ORE_ACTION_CHAMBER;
    case AUTO_ORE_DEBUG_REQUEST_PICK_POS_400:
      return AUTO_ORE_ACTION_PICK_POS_400;
    case AUTO_ORE_DEBUG_REQUEST_PICK_POS_200:
      return AUTO_ORE_ACTION_PICK_POS_200;
    case AUTO_ORE_DEBUG_REQUEST_PICK_NEG_200:
      return AUTO_ORE_ACTION_PICK_NEG_200;
    case AUTO_ORE_DEBUG_REQUEST_STEP_PICK_STORE_ASCEND_200_HEAD:
      return AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD;
    case AUTO_ORE_DEBUG_REQUEST_STEP_PICK_STORE_DESCEND_200_HEAD:
      return AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD;
    case AUTO_ORE_DEBUG_REQUEST_STEP_PICK_STORE_ASCEND_400_HEAD:
      return AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD;
    case AUTO_ORE_DEBUG_REQUEST_NONE:
    case AUTO_ORE_DEBUG_REQUEST_ABORT:
    case AUTO_ORE_DEBUG_REQUEST_ROD_SPEARHEAD:
    case AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ROD_SPEARHEAD:
    case AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ORE_RELEASE:
    case AUTO_ORE_DEBUG_REQUEST_ROD_DOCK_WAIT:
    case AUTO_ORE_DEBUG_REQUEST_STEP_ASCEND_200_HEAD:
    case AUTO_ORE_DEBUG_REQUEST_STEP_DESCEND_200_HEAD:
    case AUTO_ORE_DEBUG_REQUEST_STEP_ASCEND_400_HEAD:
    case AUTO_ORE_DEBUG_REQUEST_STEP_DESCEND_400_HEAD:
    default:
      return AUTO_ORE_ACTION_NONE;
  }
}

static bool AutoCtrlFeed_RequestMatchesRunningOreAction(
    AutoOre_DebugRequest_t request) {
  const AutoOre_Action_t action = AutoCtrlFeed_RequestToOreAction(request);
  return action != AUTO_ORE_ACTION_NONE && auto_ore_inited &&
         AutoOre_IsBusy(&auto_ore_ctrl) && auto_ore_ctrl.action == action;
}

static PC_AutoAction_t AutoCtrlFeed_MapRodAction(
    AutoRodSpearhead_Action_t action) {
  switch (action) {
    case AUTO_ROD_SPEARHEAD_ACTION_PICKUP:
      return PC_AUTO_ACTION_ROD_SPEARHEAD;
    case AUTO_ROD_SPEARHEAD_ACTION_DOCK_WAIT:
      return PC_AUTO_ACTION_ROD_DOCK_WAIT;
    case AUTO_ROD_SPEARHEAD_ACTION_NONE:
    default:
      return PC_AUTO_ACTION_NONE;
  }
}

static PC_AutoAction_t AutoCtrlFeed_MapSickCorrectAction(
    AutoSickCorrect_Action_t action) {
  switch (action) {
    case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD:
      return PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD;
    case AUTO_SICK_CORRECT_ACTION_ORE_RELEASE:
      return PC_AUTO_ACTION_SICK_CORRECT_ORE_RELEASE;
    case AUTO_SICK_CORRECT_ACTION_NONE:
    default:
      return PC_AUTO_ACTION_NONE;
  }
}

static void AutoCtrlFeed_RememberOreAction(AutoOre_Action_t action) {
  const PC_AutoAction_t pc_action = AutoCtrlFeed_MapOreAction(action);
  if (pc_action == PC_AUTO_ACTION_NONE) {
    return;
  }

  auto_ore_last_action = action;
  auto_action_last_action = pc_action;
}

static void AutoCtrlFeed_RememberRodSpearheadAction(
    AutoRodSpearhead_Action_t action) {
  const PC_AutoAction_t pc_action = AutoCtrlFeed_MapRodAction(action);
  if (pc_action == PC_AUTO_ACTION_NONE) {
    return;
  }

  auto_action_last_action = pc_action;
}

static void AutoCtrlFeed_RememberSickCorrectAction(
    AutoSickCorrect_Action_t action) {
  const PC_AutoAction_t pc_action = AutoCtrlFeed_MapSickCorrectAction(action);
  if (pc_action == PC_AUTO_ACTION_NONE) {
    return;
  }

  auto_action_last_action = pc_action;
}

static AutoSickCorrect_PointParams_t *AutoCtrlFeed_SickCorrectParamsForAction(
    AutoSickCorrect_Action_t action) {
  switch (action) {
    case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD:
      return &auto_sick_correct_ctrl.param.rod_spearhead;
    case AUTO_SICK_CORRECT_ACTION_ORE_RELEASE:
      return &auto_sick_correct_ctrl.param.ore_release;
    case AUTO_SICK_CORRECT_ACTION_NONE:
    default:
      return NULL;
  }
}

static void AutoCtrlFeed_CopySickCorrectParamToDebug(
    const AutoSickCorrect_PointParams_t *param) {
  if (param == NULL) {
    return;
  }

  g_auto_ore_debug.auto_sick_correct_x_target_adc = param->x_target_adc;
  g_auto_ore_debug.auto_sick_correct_y_target_adc = param->y_target_adc;
  g_auto_ore_debug.auto_sick_correct_z_target_diff_adc =
      param->yaw_target_diff_adc;
  g_auto_ore_debug.auto_sick_correct_x_kp_mps_per_adc =
      param->x_kp_mps_per_adc;
  g_auto_ore_debug.auto_sick_correct_y_kp_mps_per_adc =
      param->y_kp_mps_per_adc;
  g_auto_ore_debug.auto_sick_correct_z_kp_rad_s_per_adc =
      param->yaw_kp_rad_s_per_adc;
}

static void AutoCtrlFeed_InitSickCorrectDebugOverride(
    const AutoSickCorrect_PointParams_t *param) {
  if (param == NULL) {
    return;
  }

  g_auto_ore_debug.auto_sick_correct_override_x_target_adc =
      param->x_target_adc;
  g_auto_ore_debug.auto_sick_correct_override_y_target_adc =
      param->y_target_adc;
  g_auto_ore_debug.auto_sick_correct_override_z_target_diff_adc =
      param->yaw_target_diff_adc;
  g_auto_ore_debug.auto_sick_correct_override_x_kp_mps_per_adc =
      param->x_kp_mps_per_adc;
  g_auto_ore_debug.auto_sick_correct_override_y_kp_mps_per_adc =
      param->y_kp_mps_per_adc;
  g_auto_ore_debug.auto_sick_correct_override_z_kp_rad_s_per_adc =
      param->yaw_kp_rad_s_per_adc;
}

static void AutoCtrlFeed_LoadSickCorrectConfigParams(void) {
  Config_RobotParam_t *cfg = Config_GetRobotParam();
  if (cfg == NULL) {
    return;
  }

  auto_sick_correct_ctrl.param = cfg->auto_ctrl_param.sick_correct;
}

static void AutoCtrlFeed_ApplySickCorrectDebugOverride(
    AutoSickCorrect_Action_t action) {
  if (!g_auto_ore_debug.auto_sick_correct_param_override_enable) {
    return;
  }

  AutoSickCorrect_PointParams_t *param =
      AutoCtrlFeed_SickCorrectParamsForAction(action);
  if (param == NULL) {
    return;
  }

  const float x_target_adc =
      g_auto_ore_debug.auto_sick_correct_override_x_target_adc;
  const float y_target_adc =
      g_auto_ore_debug.auto_sick_correct_override_y_target_adc;
  const float z_target_diff_adc =
      g_auto_ore_debug.auto_sick_correct_override_z_target_diff_adc;
  const float x_kp_mps_per_adc =
      g_auto_ore_debug.auto_sick_correct_override_x_kp_mps_per_adc;
  const float y_kp_mps_per_adc =
      g_auto_ore_debug.auto_sick_correct_override_y_kp_mps_per_adc;
  const float z_kp_rad_s_per_adc =
      g_auto_ore_debug.auto_sick_correct_override_z_kp_rad_s_per_adc;

  if (isfinite(x_target_adc)) {
    param->x_target_adc = x_target_adc;
  }
  if (isfinite(y_target_adc)) {
    param->y_target_adc = y_target_adc;
  }
  if (isfinite(z_target_diff_adc)) {
    param->yaw_target_diff_adc = z_target_diff_adc;
  }
  if (isfinite(x_kp_mps_per_adc)) {
    param->x_kp_mps_per_adc = x_kp_mps_per_adc;
  }
  if (isfinite(y_kp_mps_per_adc)) {
    param->y_kp_mps_per_adc = y_kp_mps_per_adc;
  }
  if (isfinite(z_kp_rad_s_per_adc)) {
    param->yaw_kp_rad_s_per_adc = z_kp_rad_s_per_adc;
  }
}

static void AutoCtrlFeed_PrepareSickCorrectParams(
    AutoSickCorrect_Action_t action) {
  AutoCtrlFeed_LoadSickCorrectConfigParams();
  AutoCtrlFeed_ApplySickCorrectDebugOverride(action);
  AutoCtrlFeed_CopySickCorrectParamToDebug(
      AutoCtrlFeed_SickCorrectParamsForAction(action));
}

static PC_AutoAction_t AutoCtrlFeed_GetOreFeedbackAction(void) {
  PC_AutoAction_t action = AutoCtrlFeed_MapOreAction(auto_ore_ctrl.action);
  if (action == PC_AUTO_ACTION_NONE) {
    action = AutoCtrlFeed_MapOreAction(auto_ore_last_action);
  }
  return action;
}

static bool AutoCtrlFeed_IsOreAction(PC_AutoAction_t action) {
  switch (action) {
    case PC_AUTO_ACTION_STORE:
    case PC_AUTO_ACTION_RELEASE:
    case PC_AUTO_ACTION_CHAMBER:
    case PC_AUTO_ACTION_PICK_POS_400:
    case PC_AUTO_ACTION_PICK_POS_200:
    case PC_AUTO_ACTION_PICK_NEG_200:
    case PC_AUTO_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
    case PC_AUTO_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
    case PC_AUTO_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
      return true;
    default:
      return false;
  }
}

static PC_AutoAction_t AutoCtrlFeed_MapStepRequest(
    AutoOre_DebugRequest_t request) {
  switch (request) {
    case AUTO_ORE_DEBUG_REQUEST_STEP_ASCEND_200_HEAD:
      return PC_AUTO_ACTION_STEP_ASCEND_200_HEAD;
    case AUTO_ORE_DEBUG_REQUEST_STEP_DESCEND_200_HEAD:
      return PC_AUTO_ACTION_STEP_DESCEND_200_HEAD;
    case AUTO_ORE_DEBUG_REQUEST_STEP_ASCEND_400_HEAD:
      return PC_AUTO_ACTION_STEP_ASCEND_400_HEAD;
    case AUTO_ORE_DEBUG_REQUEST_STEP_DESCEND_400_HEAD:
      return PC_AUTO_ACTION_STEP_DESCEND_400_HEAD;
    default:
      return PC_AUTO_ACTION_NONE;
  }
}

static auto_ctrl_template_e AutoCtrlFeed_MapStepTemplate(
    PC_AutoAction_t action) {
  switch (action) {
    case PC_AUTO_ACTION_STEP_ASCEND_200_HEAD:
      return AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD;
    case PC_AUTO_ACTION_STEP_DESCEND_200_HEAD:
      return AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD;
    case PC_AUTO_ACTION_STEP_ASCEND_400_HEAD:
      return AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD;
    case PC_AUTO_ACTION_STEP_DESCEND_400_HEAD:
      return AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD;
    default:
      return AUTO_CTRL_TEMPLATE_NONE;
  }
}

static bool AutoCtrlFeed_IsStepAction(PC_AutoAction_t action) {
  return AutoCtrlFeed_MapStepTemplate(action) != AUTO_CTRL_TEMPLATE_NONE;
}

static bool AutoCtrlFeed_IsFusedOreAction(AutoOre_Action_t action) {
  switch (action) {
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
      return true;
    default:
      return false;
  }
}

static bool AutoCtrlFeed_IsRodSpearheadAction(PC_AutoAction_t action) {
  return action == PC_AUTO_ACTION_ROD_SPEARHEAD ||
         action == PC_AUTO_ACTION_ROD_DOCK_WAIT;
}

static bool AutoCtrlFeed_IsSickCorrectAction(PC_AutoAction_t action) {
  return action == PC_AUTO_ACTION_SICK_CORRECT_ROD_SPEARHEAD ||
         action == PC_AUTO_ACTION_SICK_CORRECT_ORE_RELEASE;
}

static PC_AutoAction_t AutoCtrlFeed_GetRodSpearheadFeedbackAction(void) {
  PC_AutoAction_t action = AutoCtrlFeed_MapRodAction(
      AutoRodSpearhead_GetAction(&auto_rod_spearhead_ctrl));
  if (action == PC_AUTO_ACTION_NONE &&
      AutoCtrlFeed_IsRodSpearheadAction(auto_action_last_action)) {
    action = auto_action_last_action;
  }
  return action;
}

static PC_AutoAction_t AutoCtrlFeed_GetSickCorrectFeedbackAction(void) {
  PC_AutoAction_t action = AutoCtrlFeed_MapSickCorrectAction(
      AutoSickCorrect_GetAction(&auto_sick_correct_ctrl));
  if (action == PC_AUTO_ACTION_NONE &&
      AutoCtrlFeed_IsSickCorrectAction(auto_action_last_action)) {
    action = auto_action_last_action;
  }
  return action;
}

static uint16_t AutoCtrlFeed_OreActionFailureMask(AutoOre_Action_t action) {
  switch (action) {
    case AUTO_ORE_ACTION_STORE:
      return PC_AUTO_ACTION_FAILURE_STORE_ORE;
    case AUTO_ORE_ACTION_RELEASE:
      return PC_AUTO_ACTION_FAILURE_RELEASE_ORE;
    case AUTO_ORE_ACTION_CHAMBER:
      return PC_AUTO_ACTION_FAILURE_CHAMBER;
    case AUTO_ORE_ACTION_PICK_POS_400:
    case AUTO_ORE_ACTION_PICK_POS_200:
    case AUTO_ORE_ACTION_PICK_NEG_200:
      return PC_AUTO_ACTION_FAILURE_PICK_ORE;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
      return PC_AUTO_ACTION_FAILURE_SETUP;
    case AUTO_ORE_ACTION_NONE:
    default:
      return PC_AUTO_ACTION_FAILURE_SETUP;
  }
}

static uint16_t AutoCtrlFeed_OreFailureMask(AutoOre_Action_t action) {
  if (AutoCtrlFeed_IsFusedOreAction(action)) {
    const uint16_t fused_failure = AutoOre_GetFailureMask(&auto_ore_ctrl);
    return (fused_failure != 0u) ? fused_failure : PC_AUTO_ACTION_FAILURE_SETUP;
  }
  return AutoCtrlFeed_OreActionFailureMask(action);
}

static uint16_t AutoCtrlFeed_RodFailureMask(PC_AutoAction_t action) {
  return (action == PC_AUTO_ACTION_ROD_DOCK_WAIT)
             ? PC_AUTO_ACTION_FAILURE_ROD_DOCK_WAIT
             : PC_AUTO_ACTION_FAILURE_ROD_SPEARHEAD;
}

static void AutoCtrlFeed_SetFeedbackSuccess(
    PC_AutoActionFeedback_t *feedback) {
  feedback->finished = 1u;
  feedback->result = (uint8_t)PC_AUTO_ACTION_RESULT_SUCCESS;
  feedback->failure_mask = 0u;
}

static void AutoCtrlFeed_SetFeedbackFail(PC_AutoActionFeedback_t *feedback,
                                         uint16_t failure_mask) {
  feedback->finished = 1u;
  feedback->result = (uint8_t)PC_AUTO_ACTION_RESULT_FAIL;
  feedback->failure_mask = failure_mask;
}

static bool AutoCtrlFeed_StartOreAction(AutoOre_Action_t action) {
  if (!auto_ore_inited) {
    return false;
  }
  if (AutoOre_IsBusy(&auto_ore_ctrl)) {
    return false;
  }
  if (Task_AutoSickCorrectIsBusy()) {
    return false;
  }

  bool result = false;
  const uint32_t now_ms = BSP_TIME_Get_ms();
  AutoCtrlFeed_UpdateAutoOre(now_ms);
  switch (action) {
    case AUTO_ORE_ACTION_STORE:
      result = AutoOre_StartStore(&auto_ore_ctrl, now_ms);
      break;
    case AUTO_ORE_ACTION_RELEASE:
      result = AutoOre_StartRelease(&auto_ore_ctrl, now_ms);
      break;
    case AUTO_ORE_ACTION_CHAMBER:
      result = AutoOre_StartChamber(&auto_ore_ctrl, now_ms);
      break;
    case AUTO_ORE_ACTION_PICK_POS_400:
      result = AutoOre_StartPickPos400(&auto_ore_ctrl, now_ms);
      break;
    case AUTO_ORE_ACTION_PICK_POS_200:
      result = AutoOre_StartPickPos200(&auto_ore_ctrl, now_ms);
      break;
    case AUTO_ORE_ACTION_PICK_NEG_200:
      result = AutoOre_StartPickNeg200(&auto_ore_ctrl, now_ms);
      break;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD:
      result = AutoOre_StartStepPickStoreAscend200Head(&auto_ore_ctrl,
                                                       now_ms);
      break;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD:
      result = AutoOre_StartStepPickStoreDescend200Head(&auto_ore_ctrl,
                                                        now_ms);
      break;
    case AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD:
      result = AutoOre_StartStepPickStoreAscend400Head(&auto_ore_ctrl,
                                                       now_ms);
      break;
    case AUTO_ORE_ACTION_NONE:
    default:
      result = false;
      break;
  }

  if (result || AutoOre_GetState(&auto_ore_ctrl) == AUTO_ORE_STATE_FAIL) {
    AutoCtrlFeed_RememberOreAction(action);
  }
  return result;
}

static bool AutoCtrlFeed_StartStepAction(PC_AutoAction_t action) {
  if (!auto_ctrl_inited || AutoCtrl_IsBusy(&auto_ctrl) ||
      (auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl)) ||
      Task_AutoRodSpearheadIsBusy() || Task_AutoSickCorrectIsBusy()) {
    return false;
  }

  const auto_ctrl_template_e template_id =
      AutoCtrlFeed_MapStepTemplate(action);
  if (template_id == AUTO_CTRL_TEMPLATE_NONE) {
    return false;
  }

  const uint32_t now_ms = BSP_TIME_Get_ms();
  AutoCtrl_SetYawSource(&auto_ctrl, AUTO_CTRL_YAW_SOURCE_PC);
  AutoCtrl_SetYawZeroOffset(&auto_ctrl, 0.0f);
  const bool result = AutoCtrl_StartTemplate(
      &auto_ctrl,
      template_id,
      AUTO_CTRL_TRAVEL_DIR_HEAD_FORWARD,
      feedback.yaw_auto_rad,
      0.0f,
      AUTO_CTRL_SENSOR_MODE_NONE,
      now_ms);
  if (result) {
    auto_action_last_action = action;
  }
  return result;
}

static bool AutoCtrlFeed_StartSickCorrectAction(
    AutoSickCorrect_Action_t action) {
  if (!auto_sick_correct_inited) {
    return false;
  }
  if (AutoSickCorrect_IsBusy(&auto_sick_correct_ctrl)) {
    AutoCtrlFeed_RememberSickCorrectAction(
        AutoSickCorrect_GetAction(&auto_sick_correct_ctrl));
    return true;
  }
  if ((auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl)) ||
      (auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl)) ||
      Task_AutoRodSpearheadIsBusy()) {
    return false;
  }

  bool result = false;
  const uint32_t now_ms = BSP_TIME_Get_ms();
  AutoCtrlFeed_PrepareSickCorrectParams(action);
  switch (action) {
    case AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD:
      result = AutoSickCorrect_StartRodSpearhead(&auto_sick_correct_ctrl,
                                                 now_ms);
      break;
    case AUTO_SICK_CORRECT_ACTION_ORE_RELEASE:
      result = AutoSickCorrect_StartOreRelease(&auto_sick_correct_ctrl,
                                               now_ms);
      break;
    case AUTO_SICK_CORRECT_ACTION_NONE:
    default:
      result = false;
      break;
  }

  if (result ||
      AutoSickCorrect_GetState(&auto_sick_correct_ctrl) ==
          AUTO_SICK_CORRECT_STATE_FAIL) {
    AutoCtrlFeed_RememberSickCorrectAction(action);
  }
  return result;
}

static void AutoCtrlFeed_PublishAutoActionFeedback(void) {
  PC_AutoActionFeedback_t pc_feedback = {0};
  const bool ore_busy = auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl);
  const bool rod_busy = auto_rod_spearhead_inited &&
                        AutoRodSpearhead_IsBusy(&auto_rod_spearhead_ctrl);
  const bool sick_busy = auto_sick_correct_inited &&
                         AutoSickCorrect_IsBusy(&auto_sick_correct_ctrl);

  if (ore_busy) {
    AutoCtrlFeed_RememberOreAction(auto_ore_ctrl.action);
  }

  if (rod_busy) {
    AutoCtrlFeed_RememberRodSpearheadAction(
        AutoRodSpearhead_GetAction(&auto_rod_spearhead_ctrl));
  }

  if (sick_busy) {
    AutoCtrlFeed_RememberSickCorrectAction(
        AutoSickCorrect_GetAction(&auto_sick_correct_ctrl));
  }

  if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl) &&
      AutoCtrlFeed_IsStepAction(auto_action_last_action)) {
    pc_feedback.busy = 1u;
    pc_feedback.action = (uint8_t)auto_action_last_action;
    (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_AUTO_ACTION, &pc_feedback);
    return;
  }

  pc_feedback.busy = (ore_busy || rod_busy || sick_busy) ? 1u : 0u;
  pc_feedback.action = (uint8_t)auto_action_last_action;

  if (pc_feedback.busy != 0u || auto_action_last_action == PC_AUTO_ACTION_NONE) {
    (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_AUTO_ACTION, &pc_feedback);
    return;
  }

  if (auto_action_last_action == PC_AUTO_ACTION_ABORT) {
    AutoCtrlFeed_SetFeedbackFail(&pc_feedback, PC_AUTO_ACTION_FAILURE_ABORTED);
  } else if (AutoCtrlFeed_IsOreAction(auto_action_last_action) &&
             auto_ore_inited) {
    pc_feedback.action = (uint8_t)AutoCtrlFeed_GetOreFeedbackAction();
    const AutoOre_Result_t result = AutoOre_GetResult(&auto_ore_ctrl);
    if (result == AUTO_ORE_RESULT_SUCCESS) {
      AutoCtrlFeed_SetFeedbackSuccess(&pc_feedback);
    } else if (result == AUTO_ORE_RESULT_FAIL) {
      AutoCtrlFeed_SetFeedbackFail(
          &pc_feedback, AutoCtrlFeed_OreFailureMask(auto_ore_last_action));
    } else if (result == AUTO_ORE_RESULT_ABORTED) {
      AutoCtrlFeed_SetFeedbackFail(&pc_feedback,
                                   PC_AUTO_ACTION_FAILURE_ABORTED);
    }
  } else if (AutoCtrlFeed_IsStepAction(auto_action_last_action) &&
             auto_ctrl_inited) {
    const auto_ctrl_result_e result = AutoCtrl_GetResult(&auto_ctrl);
    if (result == AUTO_CTRL_RESULT_SUCCESS) {
      AutoCtrlFeed_SetFeedbackSuccess(&pc_feedback);
    } else if (result == AUTO_CTRL_RESULT_FAIL) {
      AutoCtrlFeed_SetFeedbackFail(&pc_feedback,
                                   PC_AUTO_ACTION_FAILURE_STEP);
    } else if (result == AUTO_CTRL_RESULT_ABORTED) {
      AutoCtrlFeed_SetFeedbackFail(&pc_feedback,
                                   PC_AUTO_ACTION_FAILURE_ABORTED);
    }
  } else if (AutoCtrlFeed_IsRodSpearheadAction(auto_action_last_action) &&
             auto_rod_spearhead_inited) {
    pc_feedback.action =
        (uint8_t)AutoCtrlFeed_GetRodSpearheadFeedbackAction();
    const AutoRodSpearhead_Result_t result =
        AutoRodSpearhead_GetResult(&auto_rod_spearhead_ctrl);
    if (result == AUTO_ROD_SPEARHEAD_RESULT_SUCCESS) {
      AutoCtrlFeed_SetFeedbackSuccess(&pc_feedback);
    } else if (result == AUTO_ROD_SPEARHEAD_RESULT_FAIL) {
      AutoCtrlFeed_SetFeedbackFail(
          &pc_feedback,
          AutoCtrlFeed_RodFailureMask((PC_AutoAction_t)pc_feedback.action));
    } else if (result == AUTO_ROD_SPEARHEAD_RESULT_ABORTED) {
      AutoCtrlFeed_SetFeedbackFail(&pc_feedback,
                                   PC_AUTO_ACTION_FAILURE_ABORTED);
    }
  } else if (AutoCtrlFeed_IsSickCorrectAction(auto_action_last_action) &&
             auto_sick_correct_inited) {
    pc_feedback.action =
        (uint8_t)AutoCtrlFeed_GetSickCorrectFeedbackAction();
    const AutoSickCorrect_Result_t result =
        AutoSickCorrect_GetResult(&auto_sick_correct_ctrl);
    if (result == AUTO_SICK_CORRECT_RESULT_SUCCESS) {
      AutoCtrlFeed_SetFeedbackSuccess(&pc_feedback);
    } else if (result == AUTO_SICK_CORRECT_RESULT_FAIL) {
      AutoCtrlFeed_SetFeedbackFail(&pc_feedback,
                                   PC_AUTO_ACTION_FAILURE_SICK_CORRECT);
    } else if (result == AUTO_SICK_CORRECT_RESULT_ABORTED) {
      AutoCtrlFeed_SetFeedbackFail(&pc_feedback,
                                   PC_AUTO_ACTION_FAILURE_ABORTED);
    }
  }

  (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_AUTO_ACTION, &pc_feedback);
}

static bool AutoCtrlFeed_ArmSimpleAtTarget(float threshold) {
  const ArmSimple_Feedback_t *arm_fb = Task_ArmSimpleGetFeedback();
  if (arm_fb == NULL) {
    return false;
  }

  if (threshold <= 0.0f) {
    Config_RobotParam_t *cfg = Config_GetRobotParam();
    threshold = 0.05f;
    if (cfg != NULL &&
        cfg->arm_simple_param.preset.arrive_threshold_rad > 0.0f) {
      threshold = cfg->arm_simple_param.preset.arrive_threshold_rad;
    }
  }

  return fabsf(arm_fb->joint1_angle_rad - arm_fb->target_joint1_rad) <=
             threshold &&
         fabsf(arm_fb->joint2_angle_rad - arm_fb->target_joint2_rad) <=
             threshold;
}

static bool AutoCtrlFeed_ArmSimpleAtCommandTarget(
    const ArmSimple_CMD_t *cmd, float threshold) {
  const ArmSimple_Feedback_t *arm_fb = Task_ArmSimpleGetFeedback();
  if (arm_fb == NULL || cmd == NULL) {
    return false;
  }

  if (threshold <= 0.0f) {
    threshold = 0.05f;
  }

  return fabsf(arm_fb->joint1_angle_rad - cmd->target_joint.joint1) <=
             threshold &&
         fabsf(arm_fb->joint2_angle_rad - cmd->target_joint.joint2) <=
             threshold;
}

static bool AutoCtrlFeed_OreStoreAtCommandTarget(
    const OreStore_CMD_t *cmd, float threshold) {
  const OreStore_Feedback_t *ore_store_fb = Task_OreStoreGetFeedback();
  if (ore_store_fb == NULL || cmd == NULL ||
      !Task_OreStoreIsAllHomed()) {
    return false;
  }

  if (threshold <= 0.0f) {
    threshold = 0.05f;
  }

  return fabsf(ore_store_fb->position_rad[ORE_STORE_AXIS_PLATFORM] -
               cmd->platform_target_rad) <= threshold;
}

static void AutoCtrlFeed_InitAutoOre(void) {
  Config_RobotParam_t *cfg = Config_GetRobotParam();
  if (cfg == NULL) {
    return;
  }

  AutoOre_Params_t params = cfg->auto_ore_param;
  params.arm_param = &cfg->arm_simple_param;
  params.ore_store_param = &cfg->ore_store_param;
  params.pole_param = &cfg->pole_param;
  AutoOre_Occupancy_t initial_occupancy = {0};
  AutoOre_Init(&auto_ore_ctrl, &params, &initial_occupancy);
  auto_ore_inited = true;
}

static void AutoCtrlFeed_UpdateAutoOre(uint32_t now_ms) {
  if (!auto_ore_inited) {
    return;
  }

  bool arm_at_target = AutoCtrlFeed_ArmSimpleAtTarget(
      auto_ore_ctrl.param.arm_arrive_threshold_rad);
  if (auto_ore_ctrl.arm_cmd_valid) {
    arm_at_target = AutoCtrlFeed_ArmSimpleAtCommandTarget(
        &auto_ore_ctrl.arm_cmd, auto_ore_ctrl.param.arm_arrive_threshold_rad);
  }
  bool ore_store_all_at_target = Task_OreStoreIsAllAtTarget(
      auto_ore_ctrl.param.ore_store_arrive_threshold_rad);
  float ore_store_platform_position_rad = 0.0f;
  float ore_store_platform_error_rad = 0.0f;
  const OreStore_Feedback_t *ore_store_fb = Task_OreStoreGetFeedback();
  if (ore_store_fb != NULL) {
    ore_store_platform_position_rad =
        ore_store_fb->position_rad[ORE_STORE_AXIS_PLATFORM];
    const float ore_store_cmd_target = auto_ore_ctrl.ore_store_cmd_valid
                                          ? auto_ore_ctrl.ore_store_cmd
                                                .platform_target_rad
                                          : 0.0f;
    ore_store_platform_error_rad =
        ore_store_cmd_target - ore_store_platform_position_rad;
  }
  if (auto_ore_ctrl.ore_store_cmd_valid) {
    ore_store_all_at_target = AutoCtrlFeed_OreStoreAtCommandTarget(
        &auto_ore_ctrl.ore_store_cmd,
        auto_ore_ctrl.param.ore_store_arrive_threshold_rad);
  }
  const bool ore_low_photo_triggered = AutoCtrlFeed_ReadOreLowPhoto();
  const bool ore_high_photo_triggered = AutoCtrlFeed_ReadOreHighPhoto();
  const bool arm_ore_photo_triggered = AutoCtrlFeed_ReadArmOrePhoto();
  const bool spear_photo_triggered = AutoCtrlFeed_ReadRodSpearheadPhoto();
  const ArmSimple_Feedback_t *arm_fb = Task_ArmSimpleGetFeedback();
  const Chassis_Feedback_t *chassis_feedback = Task_ChassisGetFeedback();

  AutoOre_Feedback_t auto_ore_feedback = {
      .arm_at_target = arm_at_target,
      .ore_store_all_homed = Task_OreStoreIsAllHomed(),
      .ore_store_all_at_target = ore_store_all_at_target,
      .pole_all_at_target = Task_PoleMainAllAtTarget(
        auto_ore_ctrl.param.pole_arrive_threshold_rad),
      .pole_front_at_target = feedback.pole_front_at_target,
      .pole_rear_at_target = feedback.pole_rear_at_target,
      .arm_photo_has_ore = arm_ore_photo_triggered,
      .pe13_photo1_triggered = feedback.pe13_photo1_triggered,
      .pe9_photo2_triggered = feedback.pe9_photo2_triggered,
      .pa2_photo3_triggered = feedback.pa2_photo3_triggered,
      .pa0_photo4_triggered = feedback.pa0_photo4_triggered,
      .yaw_source = AutoCtrl_GetYawSource(&auto_ctrl),
      .yaw_auto_rad = feedback.yaw_auto_rad,
      .yaw_rate_cmd_rad_s = auto_ctrl.yaw_rate_cmd_rad_s,
      .arm_joint1_rad = (arm_fb != NULL) ? arm_fb->joint1_angle_rad : 0.0f,
      .arm_joint2_rad = (arm_fb != NULL) ? arm_fb->joint2_angle_rad : 0.0f,
      .pole_front_lift_rad = feedback.pole_front_lift_rad,
      .pole_rear_lift_rad = feedback.pole_rear_lift_rad,
      .ore_store_platform_position_rad = ore_store_platform_position_rad,
      .ore_store_platform_error_rad = ore_store_platform_error_rad,
      .photoelectric_occupancy = {
          .transform_low_has_ore = ore_low_photo_triggered,
          .transform_high_has_ore = ore_high_photo_triggered,
          .arm_has_ore = arm_ore_photo_triggered,
      },
  };
  if (chassis_feedback != NULL) {
    for (uint8_t i = 0u; i < 4u; ++i) {
      auto_ore_feedback.wheel_position_rad[i] =
          chassis_feedback->motor[i].position_rad;
    }
  }
  AutoOre_Update(&auto_ore_ctrl, &auto_ore_feedback, now_ms);

  g_auto_ore_debug.busy = AutoOre_IsBusy(&auto_ore_ctrl);
  g_auto_ore_debug.state = AutoOre_GetState(&auto_ore_ctrl);
  g_auto_ore_debug.result = AutoOre_GetResult(&auto_ore_ctrl);
  g_auto_ore_debug.fault = AutoOre_GetFault(&auto_ore_ctrl);
  g_auto_ore_debug.action = auto_ore_ctrl.action;
  g_auto_ore_debug.active_position = AutoOre_GetActivePosition(&auto_ore_ctrl);
  if (g_auto_ore_debug.active_position != AUTO_ORE_POSITION_NONE) {
    g_auto_ore_debug.last_active_position = g_auto_ore_debug.active_position;
  }
  g_auto_ore_debug.step_index = AutoOre_GetStepIndex(&auto_ore_ctrl);
  g_auto_ore_debug.step_phase = auto_ore_ctrl.step_phase;
  const AutoOre_Occupancy_t occupancy = AutoOre_GetOccupancy(&auto_ore_ctrl);
  g_auto_ore_debug.occupancy_mask = AutoOre_GetOccupancyMask(&auto_ore_ctrl);
  g_auto_ore_debug.transform_low_has_ore = occupancy.transform_low_has_ore;
  g_auto_ore_debug.transform_high_has_ore = occupancy.transform_high_has_ore;
  g_auto_ore_debug.arm_has_ore = occupancy.arm_has_ore;
  g_auto_ore_debug.checkphoto_spear_triggered = spear_photo_triggered;
  g_auto_ore_debug.checkphoto_orelow_triggered = ore_low_photo_triggered;
  g_auto_ore_debug.checkphoto_orehigh_triggered = ore_high_photo_triggered;
  g_auto_ore_debug.photo_transfer_valid = photo_transfer_snapshot.valid;
  g_auto_ore_debug.photo_transfer_raw_mask = photo_transfer_snapshot.raw_mask;
  g_auto_ore_debug.photo_transfer_age_ms = photo_transfer_snapshot.age_ms;
  g_auto_ore_debug.photo_transfer_rx_count = photo_transfer_snapshot.rx_count;
  g_auto_ore_debug.photo_transfer_timeout_count =
      photo_transfer_snapshot.timeout_count;
  g_auto_ore_debug.arm_cmd_valid = auto_ore_ctrl.arm_cmd_valid;
  g_auto_ore_debug.ore_store_cmd_valid = auto_ore_ctrl.ore_store_cmd_valid;
  g_auto_ore_debug.arm_at_target = auto_ore_feedback.arm_at_target;
  g_auto_ore_debug.pick_lift_confirmed = auto_ore_ctrl.pick_lift_confirmed;
  g_auto_ore_debug.fused_wheel_delta_rad = auto_ore_ctrl.wheel_delta_rad;
  g_auto_ore_debug.fused_target_wheel_delta_rad =
      auto_ore_ctrl.target_wheel_delta_rad;
  g_auto_ore_debug.fused_step_done = auto_ore_ctrl.fused_step_done;
  g_auto_ore_debug.fused_store_done = auto_ore_ctrl.fused_store_done;
  if (arm_fb != NULL) {
    g_auto_ore_debug.arm_feedback_joint1_rad = arm_fb->joint1_angle_rad;
    g_auto_ore_debug.arm_feedback_joint2_rad = arm_fb->joint2_angle_rad;
    g_auto_ore_debug.arm_feedback_target_joint1_rad = arm_fb->target_joint1_rad;
    g_auto_ore_debug.arm_feedback_target_joint2_rad = arm_fb->target_joint2_rad;
    g_auto_ore_debug.arm_feedback_output_target_joint1_rad =
        arm_fb->output_target_joint1_rad;
    g_auto_ore_debug.arm_feedback_output_target_joint2_rad =
        arm_fb->output_target_joint2_rad;
    g_auto_ore_debug.arm_joint1_error_rad =
        auto_ore_ctrl.arm_cmd.target_joint.joint1 - arm_fb->joint1_angle_rad;
    g_auto_ore_debug.arm_joint2_error_rad =
        auto_ore_ctrl.arm_cmd.target_joint.joint2 - arm_fb->joint2_angle_rad;
    g_auto_ore_debug.arm_output_joint1_error_rad =
        arm_fb->output_target_joint1_rad - arm_fb->joint1_angle_rad;
    g_auto_ore_debug.arm_output_joint2_error_rad =
        arm_fb->output_target_joint2_rad - arm_fb->joint2_angle_rad;
  }
  g_auto_ore_debug.ore_store_all_at_target =
      auto_ore_feedback.ore_store_all_at_target;
  g_auto_ore_debug.arm_cmd_joint1_rad = auto_ore_ctrl.arm_cmd.target_joint.joint1;
  g_auto_ore_debug.arm_cmd_joint2_rad = auto_ore_ctrl.arm_cmd.target_joint.joint2;
  g_auto_ore_debug.arm_cmd_joint1_max_vel_rad_s =
      auto_ore_ctrl.arm_cmd.joint1_max_vel_rad_s;
  g_auto_ore_debug.arm_cmd_joint2_max_vel_rad_s =
      auto_ore_ctrl.arm_cmd.joint2_max_vel_rad_s;
  g_auto_ore_debug.ore_store_cmd_platform_rad =
      auto_ore_ctrl.ore_store_cmd.platform_target_rad;
  if (ore_store_fb != NULL) {
    g_auto_ore_debug.ore_store_feedback_platform_rad =
        ore_store_fb->position_rad[ORE_STORE_AXIS_PLATFORM];
    g_auto_ore_debug.ore_store_platform_error_rad = ore_store_platform_error_rad;
  }
  g_auto_ore_debug.pole_cmd_valid = auto_ore_ctrl.pole_cmd_valid;
  g_auto_ore_debug.chassis_cmd_valid = auto_ore_ctrl.chassis_cmd_valid;
  g_auto_ore_debug.pole_all_at_target = auto_ore_feedback.pole_all_at_target;
  g_auto_ore_debug.pole_cmd_front_lift_rad =
      auto_ore_ctrl.pole_cmd.auto_target_lift[0];
  g_auto_ore_debug.pole_cmd_rear_lift_rad =
      auto_ore_ctrl.pole_cmd.auto_target_lift[1];
  g_auto_ore_debug.chassis_cmd_vx_mps =
      auto_ore_ctrl.chassis_cmd.ctrl_vec.vx;
}

static void AutoCtrlFeed_InitAutoRodSpearhead(void) {
  Config_RobotParam_t *cfg = Config_GetRobotParam();
  if (cfg == NULL) {
    return;
  }

  AutoRodSpearhead_Params_t params = cfg->auto_rod_spearhead_param;
  params.rod_param = &cfg->rod_new_param;
  params.ore_store_param = &cfg->ore_store_param;
  AutoRodSpearhead_Init(&auto_rod_spearhead_ctrl, &params);
  auto_rod_spearhead_inited = true;
}

static void AutoCtrlFeed_UpdateAutoRodSpearhead(uint32_t now_ms) {
  if (!auto_rod_spearhead_inited) {
    return;
  }

  AutoRodSpearhead_Feedback_t feedback = {
      .rod_photo_triggered = AutoCtrlFeed_ReadRodSpearheadPhoto(),
      .rod_at_target = false,
      .ore_store_at_target = false,
      .dock_complete_received = IrDock_IsDockCompleteFresh(now_ms),
  };
  const RodNew_Feedback_t *rod_fb = Task_RodNewGetFeedback();
  if (rod_fb != NULL) {
    feedback.rod_at_target = rod_fb->at_target;
  }
  const OreStore_CMD_t *auto_rod_ore_store_cmd =
      AutoRodSpearhead_GetOreStoreCommand(&auto_rod_spearhead_ctrl);
  if (auto_rod_ore_store_cmd != NULL) {
    feedback.ore_store_at_target = AutoCtrlFeed_OreStoreAtCommandTarget(
        auto_rod_ore_store_cmd,
        auto_ore_ctrl.param.ore_store_arrive_threshold_rad);
  }
  AutoRodSpearhead_Update(&auto_rod_spearhead_ctrl, &feedback, now_ms);

  g_auto_ore_debug.auto_rod_spearhead_busy =
      AutoRodSpearhead_IsBusy(&auto_rod_spearhead_ctrl);
  g_auto_ore_debug.auto_rod_spearhead_state =
      AutoRodSpearhead_GetState(&auto_rod_spearhead_ctrl);
  g_auto_ore_debug.auto_rod_spearhead_result =
      AutoRodSpearhead_GetResult(&auto_rod_spearhead_ctrl);
  g_auto_ore_debug.auto_rod_spearhead_fault =
      AutoRodSpearhead_GetFault(&auto_rod_spearhead_ctrl);
  g_auto_ore_debug.auto_rod_spearhead_action =
      AutoRodSpearhead_GetAction(&auto_rod_spearhead_ctrl);
  g_auto_ore_debug.auto_rod_spearhead_step_index =
      AutoRodSpearhead_GetStepIndex(&auto_rod_spearhead_ctrl);
  g_auto_ore_debug.auto_rod_spearhead_rod_at_target = feedback.rod_at_target;
  g_auto_ore_debug.auto_rod_spearhead_ore_store_at_target =
      feedback.ore_store_at_target;
  g_auto_ore_debug.auto_rod_spearhead_photo_stable_state =
      auto_rod_spearhead_ctrl.photo_stable_state;
  g_auto_ore_debug.auto_rod_spearhead_rod_cmd_valid =
      auto_rod_spearhead_ctrl.rod_cmd_valid;
  g_auto_ore_debug.auto_rod_spearhead_rod_cmd_pose =
      auto_rod_spearhead_ctrl.rod_cmd.pose;
  g_auto_ore_debug.auto_rod_spearhead_rod_cmd_grip =
      auto_rod_spearhead_ctrl.rod_cmd.grip;
  g_auto_ore_debug.auto_rod_spearhead_rod_cmd_target_angle_rad =
      auto_rod_spearhead_ctrl.rod_cmd.target_angle_rad;
  g_auto_ore_debug.auto_rod_spearhead_ore_store_cmd_valid =
      auto_rod_spearhead_ctrl.ore_store_cmd_valid;
  g_auto_ore_debug.auto_rod_spearhead_ore_store_cmd_platform_rad =
      auto_rod_spearhead_ctrl.ore_store_cmd.platform_target_rad;
  g_auto_ore_debug.ir_dock_complete_fresh = feedback.dock_complete_received;
  g_auto_ore_debug.ir_dock_last_rx_status = g_ir_dock_debug.last_rx_status;
  g_auto_ore_debug.ir_dock_last_rx_age_ms = g_ir_dock_debug.last_rx_age_ms;
  g_auto_ore_debug.ir_dock_rx_count = g_ir_dock_debug.rx_count;
  g_auto_ore_debug.ir_dock_error_count = g_ir_dock_debug.error_count;
}

static void AutoCtrlFeed_InitAutoSickCorrect(void) {
  Config_RobotParam_t *cfg = Config_GetRobotParam();
  if (cfg == NULL) {
    return;
  }

  AutoSickCorrect_Init(&auto_sick_correct_ctrl,
                       &cfg->auto_ctrl_param.sick_correct);
  AutoCtrlFeed_InitSickCorrectDebugOverride(
      &auto_sick_correct_ctrl.param.rod_spearhead);
  AutoCtrlFeed_CopySickCorrectParamToDebug(
      &auto_sick_correct_ctrl.param.rod_spearhead);
  auto_sick_correct_inited = true;
}

static void AutoCtrlFeed_UpdateAutoSickCorrect(uint32_t now_ms) {
  if (!auto_sick_correct_inited) {
    return;
  }

  AutoSickCorrect_Feedback_t sick_feedback = {0};
  for (uint8_t i = 0u; i < AUTO_SICK_CORRECT_SENSOR_COUNT; i++) {
    sick_feedback.adc_raw[i] = auto_ctrl_sick_output.adc_raw[i];
    sick_feedback.valid[i] = auto_ctrl_sick_output.valid[i];
  }
  sick_feedback.pole_all_at_target = feedback.pole_all_at_target;

  AutoSickCorrect_Update(&auto_sick_correct_ctrl, &sick_feedback, now_ms);

  g_auto_ore_debug.auto_sick_correct_busy =
      AutoSickCorrect_IsBusy(&auto_sick_correct_ctrl);
  g_auto_ore_debug.auto_sick_correct_state =
      AutoSickCorrect_GetState(&auto_sick_correct_ctrl);
  g_auto_ore_debug.auto_sick_correct_result =
      AutoSickCorrect_GetResult(&auto_sick_correct_ctrl);
  g_auto_ore_debug.auto_sick_correct_fault =
      AutoSickCorrect_GetFault(&auto_sick_correct_ctrl);
  g_auto_ore_debug.auto_sick_correct_action =
      AutoSickCorrect_GetAction(&auto_sick_correct_ctrl);
  g_auto_ore_debug.auto_sick_correct_step_index =
      AutoSickCorrect_GetStepIndex(&auto_sick_correct_ctrl);
  g_auto_ore_debug.auto_sick_correct_pole_all_at_target =
      sick_feedback.pole_all_at_target;
  g_auto_ore_debug.auto_sick_correct_x_sample_adc =
      auto_sick_correct_ctrl.x_sample_adc;
  g_auto_ore_debug.auto_sick_correct_y_sample_adc =
      auto_sick_correct_ctrl.y_sample_adc;
  g_auto_ore_debug.auto_sick_correct_yaw_sample_diff_adc =
      auto_sick_correct_ctrl.yaw_sample_diff_adc;
  g_auto_ore_debug.auto_sick_correct_x_error_adc =
      auto_sick_correct_ctrl.x_error_adc;
  g_auto_ore_debug.auto_sick_correct_y_error_adc =
      auto_sick_correct_ctrl.y_error_adc;
  g_auto_ore_debug.auto_sick_correct_yaw_error_adc =
      auto_sick_correct_ctrl.yaw_error_adc;
  g_auto_ore_debug.auto_sick_correct_vx_mps =
      auto_sick_correct_ctrl.chassis_cmd.ctrl_vec.vx;
  g_auto_ore_debug.auto_sick_correct_vy_mps =
      auto_sick_correct_ctrl.chassis_cmd.ctrl_vec.vy;
  g_auto_ore_debug.auto_sick_correct_wz_rad_s =
      auto_sick_correct_ctrl.chassis_cmd.ctrl_vec.wz;
  g_auto_ore_debug.auto_sick_correct_pole_target_lift =
      auto_sick_correct_ctrl.pole_cmd.auto_target_lift[0];
  g_auto_ore_debug.auto_sick_correct_x_sample_index =
      auto_sick_correct_ctrl.x_sample_index;
  g_auto_ore_debug.auto_sick_correct_y_target_adc =
      auto_sick_correct_ctrl.y_target_adc;
  const AutoSickCorrect_PointParams_t *active_param =
      AutoCtrlFeed_SickCorrectParamsForAction(
          AutoSickCorrect_GetAction(&auto_sick_correct_ctrl));
  if (active_param == NULL) {
    active_param = &auto_sick_correct_ctrl.param.rod_spearhead;
  }
  AutoCtrlFeed_CopySickCorrectParamToDebug(active_param);
}

bool Task_AutoOreStartStore(void) {
  return AutoCtrlFeed_StartOreAction(AUTO_ORE_ACTION_STORE);
}

bool Task_AutoOreStartRelease(void) {
  return AutoCtrlFeed_StartOreAction(AUTO_ORE_ACTION_RELEASE);
}

bool Task_AutoOreStartChamber(void) {
  return AutoCtrlFeed_StartOreAction(AUTO_ORE_ACTION_CHAMBER);
}

bool Task_AutoOreStartPickPos400(void) {
  return AutoCtrlFeed_StartOreAction(AUTO_ORE_ACTION_PICK_POS_400);
}

bool Task_AutoOreStartPickPos200(void) {
  return AutoCtrlFeed_StartOreAction(AUTO_ORE_ACTION_PICK_POS_200);
}

bool Task_AutoOreStartPickNeg200(void) {
  return AutoCtrlFeed_StartOreAction(AUTO_ORE_ACTION_PICK_NEG_200);
}

bool Task_AutoOreStartStepPickStoreAscend200Head(void) {
  return AutoCtrlFeed_StartOreAction(
      AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD);
}

bool Task_AutoOreStartStepPickStoreDescend200Head(void) {
  return AutoCtrlFeed_StartOreAction(
      AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD);
}

bool Task_AutoOreStartStepPickStoreAscend400Head(void) {
  return AutoCtrlFeed_StartOreAction(
      AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD);
}

bool Task_AutoStepStartAscend200Head(void) {
  return AutoCtrlFeed_StartStepAction(PC_AUTO_ACTION_STEP_ASCEND_200_HEAD);
}

bool Task_AutoStepStartDescend200Head(void) {
  return AutoCtrlFeed_StartStepAction(PC_AUTO_ACTION_STEP_DESCEND_200_HEAD);
}

bool Task_AutoStepStartAscend400Head(void) {
  return AutoCtrlFeed_StartStepAction(PC_AUTO_ACTION_STEP_ASCEND_400_HEAD);
}

bool Task_AutoStepStartDescend400Head(void) {
  return AutoCtrlFeed_StartStepAction(PC_AUTO_ACTION_STEP_DESCEND_400_HEAD);
}

void Task_AutoOreAbort(void) {
  if (auto_ore_inited) {
    if (AutoOre_IsBusy(&auto_ore_ctrl)) {
      AutoCtrlFeed_RememberOreAction(auto_ore_ctrl.action);
    }
    AutoOre_Abort(&auto_ore_ctrl);
  }
}

static void AutoCtrlFeed_HandleAutoOreDebugRequest(void) {
  const AutoOre_DebugRequest_t request = g_auto_ore_debug.request;
  bool result = false;

  if (request == AUTO_ORE_DEBUG_REQUEST_NONE) {
    return;
  }

  g_auto_ore_debug.request = AUTO_ORE_DEBUG_REQUEST_NONE;
  g_auto_ore_debug.last_request = request;
  g_auto_ore_debug.request_count++;

  if (AutoCtrlFeed_RequestMatchesRunningOreAction(request)) {
    AutoCtrlFeed_RememberOreAction(auto_ore_ctrl.action);
    g_auto_ore_debug.last_result = true;
    g_auto_ore_debug.accept_count++;
    g_auto_ore_debug.force_output_enable = true;
    return;
  }

  switch (request) {
    case AUTO_ORE_DEBUG_REQUEST_STORE:
      result = Task_AutoOreStartStore();
      break;
    case AUTO_ORE_DEBUG_REQUEST_RELEASE:
      result = Task_AutoOreStartRelease();
      break;
    case AUTO_ORE_DEBUG_REQUEST_CHAMBER:
      result = Task_AutoOreStartChamber();
      break;
    case AUTO_ORE_DEBUG_REQUEST_PICK_POS_400:
      result = Task_AutoOreStartPickPos400();
      break;
    case AUTO_ORE_DEBUG_REQUEST_PICK_POS_200:
      result = Task_AutoOreStartPickPos200();
      break;
    case AUTO_ORE_DEBUG_REQUEST_PICK_NEG_200:
      result = Task_AutoOreStartPickNeg200();
      break;
    case AUTO_ORE_DEBUG_REQUEST_STEP_PICK_STORE_ASCEND_200_HEAD:
      result = Task_AutoOreStartStepPickStoreAscend200Head();
      break;
    case AUTO_ORE_DEBUG_REQUEST_STEP_PICK_STORE_DESCEND_200_HEAD:
      result = Task_AutoOreStartStepPickStoreDescend200Head();
      break;
    case AUTO_ORE_DEBUG_REQUEST_STEP_PICK_STORE_ASCEND_400_HEAD:
      result = Task_AutoOreStartStepPickStoreAscend400Head();
      break;
    case AUTO_ORE_DEBUG_REQUEST_STEP_ASCEND_200_HEAD:
    case AUTO_ORE_DEBUG_REQUEST_STEP_DESCEND_200_HEAD:
    case AUTO_ORE_DEBUG_REQUEST_STEP_ASCEND_400_HEAD:
    case AUTO_ORE_DEBUG_REQUEST_STEP_DESCEND_400_HEAD:
      result = AutoCtrlFeed_StartStepAction(
          AutoCtrlFeed_MapStepRequest(request));
      break;
    case AUTO_ORE_DEBUG_REQUEST_ROD_SPEARHEAD:
      result = Task_AutoRodSpearheadStart();
      break;
    case AUTO_ORE_DEBUG_REQUEST_ROD_DOCK_WAIT:
      result = Task_AutoRodSpearheadStartDockWait();
      break;
    case AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ROD_SPEARHEAD:
      result = Task_AutoSickCorrectStartRodSpearhead();
      break;
    case AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ORE_RELEASE:
      result = Task_AutoSickCorrectStartOreRelease();
      break;
    case AUTO_ORE_DEBUG_REQUEST_ABORT: {
      const bool any_auto_busy =
          (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl)) ||
          (auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl)) ||
          (auto_rod_spearhead_inited &&
           AutoRodSpearhead_IsBusy(&auto_rod_spearhead_ctrl)) ||
          (auto_sick_correct_inited &&
           AutoSickCorrect_IsBusy(&auto_sick_correct_ctrl));
      if (auto_ctrl_inited) {
        AutoCtrl_Abort(&auto_ctrl);
      }
      Task_AutoOreAbort();
      Task_AutoRodSpearheadAbort();
      Task_AutoSickCorrectAbort();
      if (!any_auto_busy) {
        auto_action_last_action = PC_AUTO_ACTION_ABORT;
      }
      result = true;
      break;
    }
    case AUTO_ORE_DEBUG_REQUEST_NONE:
    default:
      result = false;
      break;
  }

  g_auto_ore_debug.last_result = result;
  if (result) {
    g_auto_ore_debug.accept_count++;
    g_auto_ore_debug.force_output_enable =
        request != AUTO_ORE_DEBUG_REQUEST_ABORT &&
        request != AUTO_ORE_DEBUG_REQUEST_ROD_SPEARHEAD &&
        request != AUTO_ORE_DEBUG_REQUEST_ROD_DOCK_WAIT &&
        request != AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ROD_SPEARHEAD &&
          request != AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ORE_RELEASE &&
          request != AUTO_ORE_DEBUG_REQUEST_STEP_ASCEND_200_HEAD &&
          request != AUTO_ORE_DEBUG_REQUEST_STEP_DESCEND_200_HEAD &&
          request != AUTO_ORE_DEBUG_REQUEST_STEP_ASCEND_400_HEAD &&
          request != AUTO_ORE_DEBUG_REQUEST_STEP_DESCEND_400_HEAD;
  }
  if (request == AUTO_ORE_DEBUG_REQUEST_ABORT) {
    g_auto_ore_debug.force_output_enable = false;
  }
}

static bool AutoCtrlFeed_StartRodSpearheadAction(
    AutoRodSpearhead_Action_t action) {
  if (Task_AutoSickCorrectIsBusy() ||
      (auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl))) {
    return false;
  }

  if (auto_rod_spearhead_inited &&
      AutoRodSpearhead_IsBusy(&auto_rod_spearhead_ctrl)) {
    AutoCtrlFeed_RememberRodSpearheadAction(
        AutoRodSpearhead_GetAction(&auto_rod_spearhead_ctrl));
    return true;
  }

  bool result = false;
  const uint32_t now_ms = BSP_TIME_Get_ms();
  if (auto_rod_spearhead_inited) {
    switch (action) {
      case AUTO_ROD_SPEARHEAD_ACTION_PICKUP:
        result = AutoRodSpearhead_StartPickup(&auto_rod_spearhead_ctrl,
                                              now_ms);
        break;
      case AUTO_ROD_SPEARHEAD_ACTION_DOCK_WAIT:
        result = AutoRodSpearhead_StartDockWait(&auto_rod_spearhead_ctrl,
                                                now_ms);
        break;
      case AUTO_ROD_SPEARHEAD_ACTION_NONE:
      default:
        result = false;
        break;
    }
  }
  if (result ||
      AutoRodSpearhead_GetState(&auto_rod_spearhead_ctrl) ==
          AUTO_ROD_SPEARHEAD_STATE_FAIL) {
    AutoCtrlFeed_RememberRodSpearheadAction(action);
  }
  return result;
}

bool Task_AutoRodSpearheadStart(void) {
  return AutoCtrlFeed_StartRodSpearheadAction(
      AUTO_ROD_SPEARHEAD_ACTION_PICKUP);
}

bool Task_AutoRodSpearheadStartDockWait(void) {
  return AutoCtrlFeed_StartRodSpearheadAction(
      AUTO_ROD_SPEARHEAD_ACTION_DOCK_WAIT);
}

void Task_AutoRodSpearheadAbort(void) {
  if (auto_rod_spearhead_inited) {
    if (AutoRodSpearhead_IsBusy(&auto_rod_spearhead_ctrl)) {
      AutoCtrlFeed_RememberRodSpearheadAction(
          AutoRodSpearhead_GetAction(&auto_rod_spearhead_ctrl));
    }
    AutoRodSpearhead_Abort(&auto_rod_spearhead_ctrl);
  }
}

bool Task_AutoRodSpearheadIsBusy(void) {
  return auto_rod_spearhead_inited &&
         AutoRodSpearhead_IsBusy(&auto_rod_spearhead_ctrl);
}

const RodNew_CMD_t *Task_AutoRodSpearheadGetCommand(void) {
  return auto_rod_spearhead_inited
             ? AutoRodSpearhead_GetRodCommand(&auto_rod_spearhead_ctrl)
             : NULL;
}

const OreStore_CMD_t *Task_AutoRodSpearheadGetOreStoreCommand(void) {
  return auto_rod_spearhead_inited
             ? AutoRodSpearhead_GetOreStoreCommand(&auto_rod_spearhead_ctrl)
             : NULL;
}

bool Task_AutoSickCorrectStartRodSpearhead(void) {
  return AutoCtrlFeed_StartSickCorrectAction(
      AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD);
}

bool Task_AutoSickCorrectStartOreRelease(void) {
  return AutoCtrlFeed_StartSickCorrectAction(
      AUTO_SICK_CORRECT_ACTION_ORE_RELEASE);
}

void Task_AutoSickCorrectAbort(void) {
  if (auto_sick_correct_inited) {
    if (AutoSickCorrect_IsBusy(&auto_sick_correct_ctrl)) {
      AutoCtrlFeed_RememberSickCorrectAction(
          AutoSickCorrect_GetAction(&auto_sick_correct_ctrl));
    }
    AutoSickCorrect_Abort(&auto_sick_correct_ctrl);
  }
}

bool Task_AutoSickCorrectIsBusy(void) {
  return auto_sick_correct_inited &&
         AutoSickCorrect_IsBusy(&auto_sick_correct_ctrl);
}

const Chassis_CMD_t *Task_AutoSickCorrectGetChassisCommand(void) {
  return auto_sick_correct_inited
             ? AutoSickCorrect_GetChassisCommand(&auto_sick_correct_ctrl)
             : NULL;
}

const Pole_CMD_t *Task_AutoSickCorrectGetPoleCommand(void) {
  return auto_sick_correct_inited
             ? AutoSickCorrect_GetPoleCommand(&auto_sick_correct_ctrl)
             : NULL;
}

bool Task_IrDockIsDockCompleteFresh(void) {
  return IrDock_IsDockCompleteFresh(BSP_TIME_Get_ms());
}

/* Exported functions ------------------------------------------------------- */
void Task_auto_ctrl(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  const uint32_t delay_tick = osKernelGetTickFreq() / AUTO_CTRL_FREQ;

  osDelay(AUTO_CTRL_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();

  AutoCtrl_Init(&auto_ctrl);
  auto_ctrl_inited = true;
  photo_transfer_inited = PhotoTransfer_Init() == DEVICE_OK;
  AutoCtrlFeed_InitAutoOre();
  AutoCtrlFeed_InitAutoRodSpearhead();
  AutoCtrlFeed_InitAutoSickCorrect();

  while (1) {
    const uint32_t profile_start_us =
        Task_ProfilerLoopBegin(TASK_PROFILE_AUTO_CTRL,
                               TASK_PERIOD_US(AUTO_CTRL_FREQ));
    uint32_t now_ms;

    tick += delay_tick;

    if (auto_ctrl_inited) {
      now_ms = BSP_TIME_Get_ms();
      if (photo_transfer_inited) {
        PhotoTransfer_Update(now_ms);
        photo_transfer_snapshot = PhotoTransfer_GetSnapshot(now_ms);
      } else {
        photo_transfer_snapshot = PhotoTransfer_GetSnapshot(now_ms);
      }

      AutoCtrlFeed_CacheLocalYawZero();

      feedback.yaw_auto_rad = AutoCtrlFeed_SelectYawRad();
      /* 新 SICK 布局没有成对前侧传感器，保持旧 yaw 辅助无效。 */
      (void)Task_SickGetLatestOutput(&auto_ctrl_sick_output);
      feedback.sick_front_left_cm =
          auto_ctrl_sick_output.valid[SICK_FRONT_INDEX]
                    ? auto_ctrl_sick_output.distance_m[SICK_FRONT_INDEX] *
                          100.0f
                    : -1.0f;
      feedback.sick_front_right_cm = -1.0f;

      feedback.pe13_photo1_triggered =
          AutoCtrlFeed_ReadPhotoTransferBit(PHOTO_TRANSFER_BIT_PHOTO1_FRONT);
      feedback.pe9_photo2_triggered =
          AutoCtrlFeed_ReadPhotoTransferBit(
              PHOTO_TRANSFER_BIT_PHOTO2_THIRD_LAST);
      feedback.pa2_photo3_triggered =
          AutoCtrlFeed_ReadPhotoTransferBit(
              PHOTO_TRANSFER_BIT_PHOTO3_SECOND_LAST);
      feedback.pa0_photo4_triggered =
          AutoCtrlFeed_ReadPhotoTransferBit(PHOTO_TRANSFER_BIT_PHOTO4_LAST);
      float pole_front_lift_rad = 0.0f;
      float pole_rear_lift_rad = 0.0f;
      (void)Task_PoleMainGetSupportLift(&pole_front_lift_rad,
                                        &pole_rear_lift_rad);
      feedback.pole_front_lift_rad = pole_front_lift_rad;
      feedback.pole_rear_lift_rad = pole_rear_lift_rad;
      const bool raw_pole_front_at_target =
          Task_PoleMainGroupAtTarget(0u,
                                     AUTO_CTRL_POLE_TARGET_THRESHOLD_RAD);
      const bool raw_pole_rear_at_target =
          Task_PoleMainGroupAtTarget(1u,
                                     AUTO_CTRL_POLE_TARGET_THRESHOLD_RAD);
      const bool raw_pole_all_at_target =
          Task_PoleMainAllAtTarget(AUTO_CTRL_POLE_TARGET_THRESHOLD_RAD);

      feedback.pole_front_at_target = AutoCtrlFeed_DebouncePoleReady(
          raw_pole_front_at_target, &pole_front_at_target_stable_count);
      feedback.pole_rear_at_target = AutoCtrlFeed_DebouncePoleReady(
          raw_pole_rear_at_target, &pole_rear_at_target_stable_count);
      feedback.pole_all_at_target = AutoCtrlFeed_DebouncePoleReady(
          raw_pole_all_at_target, &pole_all_at_target_stable_count);
      const Chassis_Feedback_t *chassis_feedback = Task_ChassisGetFeedback();
      if (chassis_feedback != NULL) {
        for (uint8_t i = 0u; i < 4u; ++i) {
          feedback.wheel_position_rad[i] =
              chassis_feedback->motor[i].position_rad;
        }
      }

      AutoCtrl_SetFeedback(&auto_ctrl, &feedback);
      AutoCtrlFeed_UpdateYawRateCommand();

      AutoCtrl_Update(&auto_ctrl, now_ms);
      AutoCtrlFeed_HandleAutoOreDebugRequest();
      AutoCtrlFeed_UpdateAutoOre(now_ms);
      AutoCtrlFeed_UpdateAutoRodSpearhead(now_ms);
      AutoCtrlFeed_UpdateAutoSickCorrect(now_ms);
      AutoCtrlFeed_PublishAutoActionFeedback();

      if (MrlinkPc_GetState() != NULL) {
        PC_StepFeedback_t step_feedback = {0};
        step_feedback.state = (PC_StepState_t)AutoCtrl_GetState(&auto_ctrl);
        step_feedback.result = (PC_StepResult_t)AutoCtrl_GetResult(&auto_ctrl);
        step_feedback.fault = (PC_StepFault_t)AutoCtrl_GetFault(&auto_ctrl);
        step_feedback.template_id = (PC_StepTemplate_t)AutoCtrl_GetTemplate(&auto_ctrl);
        step_feedback.step_index = AutoCtrl_GetStepIndex(&auto_ctrl);
        step_feedback.progress = 0.0f;
        (void)MrlinkPc_PublishFeedback(PC_FEEDBACK_STEP, &step_feedback);
      }
    }

    task_runtime.stack_water_mark.auto_ctrl = uxTaskGetStackHighWaterMark(NULL);
    task_runtime.heartbeat.auto_ctrl++;
    Task_ProfilerLoopEnd(TASK_PROFILE_AUTO_CTRL, profile_start_us);
    Task_DelayUntil(TASK_PROFILE_AUTO_CTRL, &tick, delay_tick);
  }
}
