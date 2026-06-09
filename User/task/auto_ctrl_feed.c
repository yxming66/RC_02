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

/* Ozone/调试器手动触发一键动作：request=1存矿，2放矿，3上膛，4中止，5取正400，6取正200，7取负200，8取矛头，11等待对接。 */
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
static PC_AutoActionSubsystem_t auto_action_last_subsystem =
    PC_AUTO_ACTION_SUBSYSTEM_NONE;
static AutoOre_Action_t auto_ore_last_action = AUTO_ORE_ACTION_NONE;
static const GPIO_PinState photo1_active_state = GPIO_PIN_RESET;
static const GPIO_PinState photo2_active_state = GPIO_PIN_RESET;
static const GPIO_PinState photo3_active_state = GPIO_PIN_RESET;
static const GPIO_PinState photo4_active_state = GPIO_PIN_RESET;
static const GPIO_PinState checkphoto_ore_has_ore_state = GPIO_PIN_RESET;

#ifndef AUTO_CTRL_POLE_TARGET_THRESHOLD_RAD
#define AUTO_CTRL_POLE_TARGET_THRESHOLD_RAD (0.30f)
#endif

#ifndef AUTO_CTRL_POLE_TARGET_STABLE_CYCLES
#define AUTO_CTRL_POLE_TARGET_STABLE_CYCLES (5u)
#endif

#ifndef AUTO_ORE_PHOTO_STABLE_MS
#define AUTO_ORE_PHOTO_STABLE_MS (500u)
#endif

#ifndef AUTO_ROD_SPEARHEAD_PHOTO_GPIO_PORT
#define AUTO_ROD_SPEARHEAD_PHOTO_GPIO_PORT checkphoto_spear_GPIO_Port
#endif

#ifndef AUTO_ROD_SPEARHEAD_PHOTO_PIN
#define AUTO_ROD_SPEARHEAD_PHOTO_PIN checkphoto_spear_Pin
#endif

#ifndef AUTO_ROD_SPEARHEAD_PHOTO_ACTIVE_STATE
#define AUTO_ROD_SPEARHEAD_PHOTO_ACTIVE_STATE GPIO_PIN_RESET
#endif

#if defined(AUTO_ROD_SPEARHEAD_PHOTO_GPIO_PORT) && \
  defined(AUTO_ROD_SPEARHEAD_PHOTO_PIN)
#define AUTO_ROD_SPEARHEAD_PHOTO_CONFIGURED (1u)
#else
#define AUTO_ROD_SPEARHEAD_PHOTO_CONFIGURED (0u)
#endif

static uint8_t pole_front_at_target_stable_count = 0u;
static uint8_t pole_rear_at_target_stable_count = 0u;
static uint8_t pole_all_at_target_stable_count = 0u;
static bool checkphoto_orelow_has_ore = false;
static bool checkphoto_orehigh_has_ore = false;
static GPIO_PinState checkphoto_orelow_candidate_state = GPIO_PIN_RESET;
static GPIO_PinState checkphoto_orehigh_candidate_state = GPIO_PIN_RESET;
static uint32_t checkphoto_orelow_candidate_start_ms = 0u;
static uint32_t checkphoto_orehigh_candidate_start_ms = 0u;
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
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

static bool AutoCtrlFeed_ReadRodSpearheadPhoto(void) {
#if AUTO_ROD_SPEARHEAD_PHOTO_CONFIGURED
  return HAL_GPIO_ReadPin(AUTO_ROD_SPEARHEAD_PHOTO_GPIO_PORT,
                         AUTO_ROD_SPEARHEAD_PHOTO_PIN) ==
         AUTO_ROD_SPEARHEAD_PHOTO_ACTIVE_STATE;
#else
  return false;
#endif
}

static bool AutoCtrlFeed_DebounceOrePhoto(GPIO_PinState raw_state,
                                          bool *stable_has_ore,
                                          GPIO_PinState *candidate_state,
                                          uint32_t *candidate_start_ms,
                                          uint32_t now_ms) {
  if (stable_has_ore == NULL || candidate_state == NULL ||
      candidate_start_ms == NULL) {
    return false;
  }

  if (raw_state != *candidate_state) {
    *candidate_state = raw_state;
    *candidate_start_ms = now_ms;
    return *stable_has_ore;
  }

  if ((now_ms - *candidate_start_ms) >= AUTO_ORE_PHOTO_STABLE_MS) {
    if (raw_state == checkphoto_ore_has_ore_state) {
      *stable_has_ore = true;
    } else {
      *stable_has_ore = false;
    }
  }

  return *stable_has_ore;
}

static bool AutoCtrlFeed_ReadOreLowPhoto(uint32_t now_ms) {
  const GPIO_PinState raw_state = HAL_GPIO_ReadPin(checkphoto_orelow_GPIO_Port,
                                                  checkphoto_orelow_Pin);
  return AutoCtrlFeed_DebounceOrePhoto(raw_state, &checkphoto_orelow_has_ore,
                                       &checkphoto_orelow_candidate_state,
                                       &checkphoto_orelow_candidate_start_ms,
                                       now_ms);
}

static bool AutoCtrlFeed_ReadOreHighPhoto(uint32_t now_ms) {
  const GPIO_PinState raw_state = HAL_GPIO_ReadPin(checkphoto_orehigh_GPIO_Port,
                                                  checkphoto_orehigh_Pin);
  return AutoCtrlFeed_DebounceOrePhoto(raw_state, &checkphoto_orehigh_has_ore,
                                       &checkphoto_orehigh_candidate_state,
                                       &checkphoto_orehigh_candidate_start_ms,
                                       now_ms);
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
    case AUTO_ORE_ACTION_NONE:
    default:
      return PC_AUTO_ACTION_NONE;
  }
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

static PC_AutoActionState_t AutoCtrlFeed_MapOreState(AutoOre_State_t state) {
  switch (state) {
    case AUTO_ORE_STATE_RUNNING:
      return PC_AUTO_ACTION_STATE_RUNNING;
    case AUTO_ORE_STATE_SUCCESS:
      return PC_AUTO_ACTION_STATE_SUCCESS;
    case AUTO_ORE_STATE_FAIL:
      return PC_AUTO_ACTION_STATE_FAIL;
    case AUTO_ORE_STATE_ABORT:
      return PC_AUTO_ACTION_STATE_ABORT;
    case AUTO_ORE_STATE_IDLE:
    default:
      return PC_AUTO_ACTION_STATE_IDLE;
  }
}

static PC_AutoActionResult_t AutoCtrlFeed_MapOreResult(
    AutoOre_Result_t result) {
  switch (result) {
    case AUTO_ORE_RESULT_RUNNING:
      return PC_AUTO_ACTION_RESULT_RUNNING;
    case AUTO_ORE_RESULT_SUCCESS:
      return PC_AUTO_ACTION_RESULT_SUCCESS;
    case AUTO_ORE_RESULT_FAIL:
      return PC_AUTO_ACTION_RESULT_FAIL;
    case AUTO_ORE_RESULT_ABORTED:
      return PC_AUTO_ACTION_RESULT_ABORTED;
    case AUTO_ORE_RESULT_NONE:
    default:
      return PC_AUTO_ACTION_RESULT_NONE;
  }
}

static PC_AutoActionFault_t AutoCtrlFeed_MapOreFault(AutoOre_Fault_t fault) {
  switch (fault) {
    case AUTO_ORE_FAULT_INVALID_OCCUPANCY:
      return PC_AUTO_ACTION_FAULT_INVALID_OCCUPANCY;
    case AUTO_ORE_FAULT_INVALID_PARAM:
      return PC_AUTO_ACTION_FAULT_INVALID_PARAM;
    case AUTO_ORE_FAULT_NOT_HOMED:
      return PC_AUTO_ACTION_FAULT_NOT_HOMED;
    case AUTO_ORE_FAULT_TIMEOUT:
      return PC_AUTO_ACTION_FAULT_TIMEOUT;
    case AUTO_ORE_FAULT_ABORTED:
      return PC_AUTO_ACTION_FAULT_ABORTED;
    case AUTO_ORE_FAULT_NONE:
    default:
      return PC_AUTO_ACTION_FAULT_NONE;
  }
}

static PC_AutoActionState_t AutoCtrlFeed_MapRodState(
    AutoRodSpearhead_State_t state) {
  switch (state) {
    case AUTO_ROD_SPEARHEAD_STATE_RUNNING:
      return PC_AUTO_ACTION_STATE_RUNNING;
    case AUTO_ROD_SPEARHEAD_STATE_SUCCESS:
      return PC_AUTO_ACTION_STATE_SUCCESS;
    case AUTO_ROD_SPEARHEAD_STATE_FAIL:
      return PC_AUTO_ACTION_STATE_FAIL;
    case AUTO_ROD_SPEARHEAD_STATE_ABORT:
      return PC_AUTO_ACTION_STATE_ABORT;
    case AUTO_ROD_SPEARHEAD_STATE_IDLE:
    default:
      return PC_AUTO_ACTION_STATE_IDLE;
  }
}

static PC_AutoActionResult_t AutoCtrlFeed_MapRodResult(
    AutoRodSpearhead_Result_t result) {
  switch (result) {
    case AUTO_ROD_SPEARHEAD_RESULT_RUNNING:
      return PC_AUTO_ACTION_RESULT_RUNNING;
    case AUTO_ROD_SPEARHEAD_RESULT_SUCCESS:
      return PC_AUTO_ACTION_RESULT_SUCCESS;
    case AUTO_ROD_SPEARHEAD_RESULT_FAIL:
      return PC_AUTO_ACTION_RESULT_FAIL;
    case AUTO_ROD_SPEARHEAD_RESULT_ABORTED:
      return PC_AUTO_ACTION_RESULT_ABORTED;
    case AUTO_ROD_SPEARHEAD_RESULT_NONE:
    default:
      return PC_AUTO_ACTION_RESULT_NONE;
  }
}

static PC_AutoActionFault_t AutoCtrlFeed_MapRodFault(
    AutoRodSpearhead_Fault_t fault) {
  switch (fault) {
    case AUTO_ROD_SPEARHEAD_FAULT_TIMEOUT:
      return PC_AUTO_ACTION_FAULT_TIMEOUT;
    case AUTO_ROD_SPEARHEAD_FAULT_INVALID_PARAM:
      return PC_AUTO_ACTION_FAULT_INVALID_PARAM;
    case AUTO_ROD_SPEARHEAD_FAULT_ABORTED:
      return PC_AUTO_ACTION_FAULT_ABORTED;
    case AUTO_ROD_SPEARHEAD_FAULT_NO_SPEARHEAD:
      return PC_AUTO_ACTION_FAULT_NO_SPEARHEAD;
    case AUTO_ROD_SPEARHEAD_FAULT_NONE:
    default:
      return PC_AUTO_ACTION_FAULT_NONE;
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

static PC_AutoActionState_t AutoCtrlFeed_MapSickCorrectState(
    AutoSickCorrect_State_t state) {
  switch (state) {
    case AUTO_SICK_CORRECT_STATE_RUNNING:
      return PC_AUTO_ACTION_STATE_RUNNING;
    case AUTO_SICK_CORRECT_STATE_SUCCESS:
      return PC_AUTO_ACTION_STATE_SUCCESS;
    case AUTO_SICK_CORRECT_STATE_FAIL:
      return PC_AUTO_ACTION_STATE_FAIL;
    case AUTO_SICK_CORRECT_STATE_ABORT:
      return PC_AUTO_ACTION_STATE_ABORT;
    case AUTO_SICK_CORRECT_STATE_IDLE:
    default:
      return PC_AUTO_ACTION_STATE_IDLE;
  }
}

static PC_AutoActionResult_t AutoCtrlFeed_MapSickCorrectResult(
    AutoSickCorrect_Result_t result) {
  switch (result) {
    case AUTO_SICK_CORRECT_RESULT_RUNNING:
      return PC_AUTO_ACTION_RESULT_RUNNING;
    case AUTO_SICK_CORRECT_RESULT_SUCCESS:
      return PC_AUTO_ACTION_RESULT_SUCCESS;
    case AUTO_SICK_CORRECT_RESULT_FAIL:
      return PC_AUTO_ACTION_RESULT_FAIL;
    case AUTO_SICK_CORRECT_RESULT_ABORTED:
      return PC_AUTO_ACTION_RESULT_ABORTED;
    case AUTO_SICK_CORRECT_RESULT_NONE:
    default:
      return PC_AUTO_ACTION_RESULT_NONE;
  }
}

static PC_AutoActionFault_t AutoCtrlFeed_MapSickCorrectFault(
    AutoSickCorrect_Fault_t fault) {
  switch (fault) {
    case AUTO_SICK_CORRECT_FAULT_TIMEOUT:
      return PC_AUTO_ACTION_FAULT_TIMEOUT;
    case AUTO_SICK_CORRECT_FAULT_INVALID_PARAM:
      return PC_AUTO_ACTION_FAULT_INVALID_PARAM;
    case AUTO_SICK_CORRECT_FAULT_SENSOR_INVALID:
      return PC_AUTO_ACTION_FAULT_SENSOR_INVALID;
    case AUTO_SICK_CORRECT_FAULT_UNSUPPORTED:
      return PC_AUTO_ACTION_FAULT_UNSUPPORTED;
    case AUTO_SICK_CORRECT_FAULT_ABORTED:
      return PC_AUTO_ACTION_FAULT_ABORTED;
    case AUTO_SICK_CORRECT_FAULT_NONE:
    default:
      return PC_AUTO_ACTION_FAULT_NONE;
  }
}

static void AutoCtrlFeed_RememberOreAction(AutoOre_Action_t action) {
  const PC_AutoAction_t pc_action = AutoCtrlFeed_MapOreAction(action);
  if (pc_action == PC_AUTO_ACTION_NONE) {
    return;
  }

  auto_ore_last_action = action;
  auto_action_last_action = pc_action;
  auto_action_last_subsystem = PC_AUTO_ACTION_SUBSYSTEM_ORE;
}

static void AutoCtrlFeed_RememberRodSpearheadAction(
    AutoRodSpearhead_Action_t action) {
  const PC_AutoAction_t pc_action = AutoCtrlFeed_MapRodAction(action);
  if (pc_action == PC_AUTO_ACTION_NONE) {
    return;
  }

  auto_action_last_action = pc_action;
  auto_action_last_subsystem = PC_AUTO_ACTION_SUBSYSTEM_ROD_SPEARHEAD;
}

static void AutoCtrlFeed_RememberSickCorrectAction(
    AutoSickCorrect_Action_t action) {
  const PC_AutoAction_t pc_action = AutoCtrlFeed_MapSickCorrectAction(action);
  if (pc_action == PC_AUTO_ACTION_NONE) {
    return;
  }

  auto_action_last_action = pc_action;
  auto_action_last_subsystem = PC_AUTO_ACTION_SUBSYSTEM_SICK_CORRECT;
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

  if (auto_ore_inited) {
    pc_feedback.flags |= PC_AUTO_ACTION_FLAG_ORE_INITED;
    if (ore_busy) {
      pc_feedback.flags |= PC_AUTO_ACTION_FLAG_ORE_BUSY;
      AutoCtrlFeed_RememberOreAction(auto_ore_ctrl.action);
    }

    pc_feedback.ore_action =
        (uint8_t)AutoCtrlFeed_GetOreFeedbackAction();
    pc_feedback.ore_state =
        (uint8_t)AutoCtrlFeed_MapOreState(AutoOre_GetState(&auto_ore_ctrl));
    pc_feedback.ore_result =
        (uint8_t)AutoCtrlFeed_MapOreResult(AutoOre_GetResult(&auto_ore_ctrl));
    pc_feedback.ore_fault =
        (uint8_t)AutoCtrlFeed_MapOreFault(AutoOre_GetFault(&auto_ore_ctrl));
    pc_feedback.ore_step_index = AutoOre_GetStepIndex(&auto_ore_ctrl);
    pc_feedback.ore_step_phase = auto_ore_ctrl.step_phase;
    pc_feedback.ore_active_position =
        (uint8_t)AutoOre_GetActivePosition(&auto_ore_ctrl);
    pc_feedback.ore_occupancy_mask = AutoOre_GetOccupancyMask(&auto_ore_ctrl);
  }

  if (auto_rod_spearhead_inited) {
    pc_feedback.flags |= PC_AUTO_ACTION_FLAG_ROD_INITED;
    if (rod_busy) {
      pc_feedback.flags |= PC_AUTO_ACTION_FLAG_ROD_BUSY;
      AutoCtrlFeed_RememberRodSpearheadAction(
          AutoRodSpearhead_GetAction(&auto_rod_spearhead_ctrl));
    }

    pc_feedback.rod_state = (uint8_t)AutoCtrlFeed_MapRodState(
        AutoRodSpearhead_GetState(&auto_rod_spearhead_ctrl));
    pc_feedback.rod_result = (uint8_t)AutoCtrlFeed_MapRodResult(
        AutoRodSpearhead_GetResult(&auto_rod_spearhead_ctrl));
    pc_feedback.rod_fault = (uint8_t)AutoCtrlFeed_MapRodFault(
        AutoRodSpearhead_GetFault(&auto_rod_spearhead_ctrl));
    pc_feedback.rod_step_index =
        AutoRodSpearhead_GetStepIndex(&auto_rod_spearhead_ctrl);
  }

  if (auto_sick_correct_inited) {
    pc_feedback.flags |= PC_AUTO_ACTION_FLAG_SICK_CORRECT_INITED;
    if (sick_busy) {
      pc_feedback.flags |= PC_AUTO_ACTION_FLAG_SICK_CORRECT_BUSY;
      AutoCtrlFeed_RememberSickCorrectAction(
          AutoSickCorrect_GetAction(&auto_sick_correct_ctrl));
    }
  }

  PC_AutoActionSubsystem_t active_subsystem = PC_AUTO_ACTION_SUBSYSTEM_NONE;
  if ((ore_busy && rod_busy) || (ore_busy && sick_busy) ||
      (rod_busy && sick_busy)) {
    active_subsystem =
        (auto_action_last_subsystem != PC_AUTO_ACTION_SUBSYSTEM_NONE)
            ? auto_action_last_subsystem
            : PC_AUTO_ACTION_SUBSYSTEM_ORE;
  } else if (ore_busy) {
    active_subsystem = PC_AUTO_ACTION_SUBSYSTEM_ORE;
  } else if (rod_busy) {
    active_subsystem = PC_AUTO_ACTION_SUBSYSTEM_ROD_SPEARHEAD;
  } else if (sick_busy) {
    active_subsystem = PC_AUTO_ACTION_SUBSYSTEM_SICK_CORRECT;
  } else {
    active_subsystem = auto_action_last_subsystem;
  }

  pc_feedback.busy = (ore_busy || rod_busy || sick_busy) ? 1u : 0u;
  pc_feedback.subsystem = (uint8_t)active_subsystem;
  pc_feedback.action = (uint8_t)auto_action_last_action;

  if (active_subsystem == PC_AUTO_ACTION_SUBSYSTEM_ORE && auto_ore_inited) {
    pc_feedback.action = (uint8_t)AutoCtrlFeed_GetOreFeedbackAction();
    pc_feedback.state = pc_feedback.ore_state;
    pc_feedback.result = pc_feedback.ore_result;
    pc_feedback.fault = pc_feedback.ore_fault;
    pc_feedback.step_index = pc_feedback.ore_step_index;
    pc_feedback.step_phase = pc_feedback.ore_step_phase;
    pc_feedback.active_position = pc_feedback.ore_active_position;
    pc_feedback.occupancy_mask = pc_feedback.ore_occupancy_mask;
  } else if (active_subsystem ==
             PC_AUTO_ACTION_SUBSYSTEM_ROD_SPEARHEAD &&
             auto_rod_spearhead_inited) {
    const PC_AutoAction_t rod_action = AutoCtrlFeed_MapRodAction(
        AutoRodSpearhead_GetAction(&auto_rod_spearhead_ctrl));
    pc_feedback.action =
        (uint8_t)((rod_action != PC_AUTO_ACTION_NONE)
                      ? rod_action
                      : auto_action_last_action);
    pc_feedback.state = pc_feedback.rod_state;
    pc_feedback.result = pc_feedback.rod_result;
    pc_feedback.fault = pc_feedback.rod_fault;
    pc_feedback.step_index = pc_feedback.rod_step_index;
    pc_feedback.step_phase = 0u;
    pc_feedback.active_position = 0u;
    pc_feedback.occupancy_mask = 0u;
  } else if (active_subsystem == PC_AUTO_ACTION_SUBSYSTEM_SICK_CORRECT &&
             auto_sick_correct_inited) {
    pc_feedback.action = (uint8_t)AutoCtrlFeed_MapSickCorrectAction(
        AutoSickCorrect_GetAction(&auto_sick_correct_ctrl));
    pc_feedback.state = (uint8_t)AutoCtrlFeed_MapSickCorrectState(
        AutoSickCorrect_GetState(&auto_sick_correct_ctrl));
    pc_feedback.result = (uint8_t)AutoCtrlFeed_MapSickCorrectResult(
        AutoSickCorrect_GetResult(&auto_sick_correct_ctrl));
    pc_feedback.fault = (uint8_t)AutoCtrlFeed_MapSickCorrectFault(
        AutoSickCorrect_GetFault(&auto_sick_correct_ctrl));
    pc_feedback.step_index =
        AutoSickCorrect_GetStepIndex(&auto_sick_correct_ctrl);
    pc_feedback.step_phase = 0u;
    pc_feedback.active_position = 0u;
    pc_feedback.occupancy_mask = 0u;
  }

  pc_feedback.reserved = 0u;
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
  float ore_store_platform_error_rad = 0.0f;
  const OreStore_Feedback_t *ore_store_fb = Task_OreStoreGetFeedback();
  if (ore_store_fb != NULL) {
    const float ore_store_cmd_target = auto_ore_ctrl.ore_store_cmd_valid
                                          ? auto_ore_ctrl.ore_store_cmd
                                                .platform_target_rad
                                          : 0.0f;
    ore_store_platform_error_rad =
        ore_store_cmd_target -
        ore_store_fb->position_rad[ORE_STORE_AXIS_PLATFORM];
  }
  if (auto_ore_ctrl.ore_store_cmd_valid) {
    ore_store_all_at_target = AutoCtrlFeed_OreStoreAtCommandTarget(
        &auto_ore_ctrl.ore_store_cmd,
        auto_ore_ctrl.param.ore_store_arrive_threshold_rad);
  }
  const bool ore_low_photo_triggered = AutoCtrlFeed_ReadOreLowPhoto(now_ms);
  const bool ore_high_photo_triggered = AutoCtrlFeed_ReadOreHighPhoto(now_ms);
  const bool spear_photo_triggered = AutoCtrlFeed_ReadRodSpearheadPhoto();

  AutoOre_Feedback_t auto_ore_feedback = {
      .arm_at_target = arm_at_target,
      .ore_store_all_homed = Task_OreStoreIsAllHomed(),
      .ore_store_all_at_target = ore_store_all_at_target,
      .pole_all_at_target = Task_ChassisMainPoleAllAtTarget(
        auto_ore_ctrl.param.pole_arrive_threshold_rad),
      .ore_store_platform_error_rad = ore_store_platform_error_rad,
      .photoelectric_occupancy = {
          .transform_low_has_ore = ore_low_photo_triggered,
          .transform_high_has_ore = ore_high_photo_triggered,
          .arm_has_ore = false,
      },
  };
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
  g_auto_ore_debug.arm_cmd_valid = auto_ore_ctrl.arm_cmd_valid;
  g_auto_ore_debug.ore_store_cmd_valid = auto_ore_ctrl.ore_store_cmd_valid;
  g_auto_ore_debug.arm_at_target = auto_ore_feedback.arm_at_target;
  const ArmSimple_Feedback_t *arm_fb = Task_ArmSimpleGetFeedback();
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
    case AUTO_ORE_DEBUG_REQUEST_ABORT:
      Task_AutoOreAbort();
      Task_AutoRodSpearheadAbort();
      Task_AutoSickCorrectAbort();
      result = true;
      break;
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
        request != AUTO_ORE_DEBUG_REQUEST_SICK_CORRECT_ORE_RELEASE;
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
  AutoCtrlFeed_InitAutoOre();
  AutoCtrlFeed_InitAutoRodSpearhead();
  AutoCtrlFeed_InitAutoSickCorrect();

  while (1) {
    uint32_t now_ms;

    tick += delay_tick;

    if (auto_ctrl_inited) {
      now_ms = BSP_TIME_Get_ms();

      AutoCtrlFeed_CacheLocalYawZero();

      feedback.yaw_auto_rad = AutoCtrlFeed_SelectYawRad();
      /* 当前 AutoCtrl API 只消费前向两路 SICK。 */
      (void)Task_SickGetLatestOutput(&auto_ctrl_sick_output);
      feedback.sick_front_left_cm =
          auto_ctrl_sick_output.valid[SICK_FRONT_S1_INDEX]
                    ? auto_ctrl_sick_output.distance_m[SICK_FRONT_S1_INDEX] *
                          100.0f
                    : -1.0f;
      feedback.sick_front_right_cm =
          auto_ctrl_sick_output.valid[SICK_FRONT_S2_INDEX]
                     ? auto_ctrl_sick_output.distance_m[SICK_FRONT_S2_INDEX] *
                           100.0f
                     : -1.0f;

      GPIO_PinState photo1_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
      GPIO_PinState photo2_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
      GPIO_PinState photo3_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
      GPIO_PinState photo4_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

      feedback.pe13_photo1_triggered = (photo1_state == photo1_active_state);
      feedback.pe9_photo2_triggered = (photo2_state == photo2_active_state);
      feedback.pa2_photo3_triggered = (photo3_state == photo3_active_state);
      feedback.pa0_photo4_triggered = (photo4_state == photo4_active_state);
      const bool raw_pole_front_at_target =
          Task_ChassisMainPoleGroupAtTarget(0u,
                                            AUTO_CTRL_POLE_TARGET_THRESHOLD_RAD);
      const bool raw_pole_rear_at_target =
          Task_ChassisMainPoleGroupAtTarget(1u,
                                            AUTO_CTRL_POLE_TARGET_THRESHOLD_RAD);
      const bool raw_pole_all_at_target =
          Task_ChassisMainPoleAllAtTarget(AUTO_CTRL_POLE_TARGET_THRESHOLD_RAD);

      feedback.pole_front_at_target = AutoCtrlFeed_DebouncePoleReady(
          raw_pole_front_at_target, &pole_front_at_target_stable_count);
      feedback.pole_rear_at_target = AutoCtrlFeed_DebouncePoleReady(
          raw_pole_rear_at_target, &pole_rear_at_target_stable_count);
      feedback.pole_all_at_target = AutoCtrlFeed_DebouncePoleReady(
          raw_pole_all_at_target, &pole_all_at_target_stable_count);

      AutoCtrl_SetFeedback(&auto_ctrl, &feedback);

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
    osDelayUntil(tick);
  }
}
