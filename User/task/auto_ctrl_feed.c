/*
    auto_ctrl_feed Task
    Dedicated task for collecting sensor data and feeding AutoCtrl feedback.
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "main.h"
#include "device/dr16.h"
#include "module/autoCtrlAPI/api/auto_ctrl_api.h"
#include "module/autoCtrlAPI/ore_store/auto_ore_store.h"
#include "module/chassis.h"
#include "module/config.h"
#include "module/pc_protocol/pc_protocol.h"

#include <math.h>

/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
extern Chassis_IMU_t chassis_imu;
extern float distance_cm[4];
extern DR16_t dr16;
extern PC_Protocol_t *g_pc_protocol_ptr;

auto_ctrl_t auto_ctrl;
bool auto_ctrl_inited = false;
AutoOre_t auto_ore_ctrl;
bool auto_ore_inited = false;
bool auto_ctrl_local_yaw_zero_initialized = false;
float auto_ctrl_local_yaw_zero_rad = 0.0f;
auto_ctrl_feedback_t feedback = {0};
static const GPIO_PinState photo1_active_state = GPIO_PIN_RESET;
static const GPIO_PinState photo2_active_state = GPIO_PIN_RESET;
static const GPIO_PinState photo3_active_state = GPIO_PIN_RESET;
static const GPIO_PinState photo4_active_state = GPIO_PIN_RESET;

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
static float AutoCtrlFeed_SelectYawRad(void) {
  if (AutoCtrl_GetYawSource(&auto_ctrl) == AUTO_CTRL_YAW_SOURCE_PC &&
      g_pc_protocol_ptr != NULL) {
    const PC_ImuCMD_t *pc_imu = PC_Protocol_GetImuCMD(g_pc_protocol_ptr);
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

static bool AutoCtrlFeed_ArmSimpleAtTarget(void) {
  const ArmSimple_Feedback_t *arm_fb = Task_ArmSimpleGetFeedback();
  if (arm_fb == NULL) {
    return false;
  }

  Config_RobotParam_t *cfg = Config_GetRobotParam();
  float threshold = 0.05f;
  if (cfg != NULL && cfg->arm_simple_param.preset.arrive_threshold_rad > 0.0f) {
    threshold = cfg->arm_simple_param.preset.arrive_threshold_rad;
  }

  return fabsf(arm_fb->joint1_angle_rad - arm_fb->target_joint1_rad) <=
             threshold &&
         fabsf(arm_fb->joint2_angle_rad - arm_fb->target_joint2_rad) <=
             threshold;
}

static void AutoCtrlFeed_InitAutoOre(void) {
  Config_RobotParam_t *cfg = Config_GetRobotParam();
  if (cfg == NULL) {
    return;
  }

  AutoOre_Params_t params = {
      .arm_param = &cfg->arm_simple_param,
      .ore_store_param = &cfg->ore_store_param,
      .pole_param = &cfg->pole_param,
      .default_step_timeout_ms = 5000u,
      .arm_arrive_threshold_rad = cfg->arm_simple_param.preset.arrive_threshold_rad,
      .ore_store_arrive_threshold_rad = 0.05f,
      .pole_arrive_threshold_rad = AUTO_CTRL_POLE_TARGET_THRESHOLD_RAD,
  };
  AutoOre_Init(&auto_ore_ctrl, &params, 0u);
  auto_ore_inited = true;
}

static void AutoCtrlFeed_UpdateAutoOre(uint32_t now_ms) {
  if (!auto_ore_inited) {
    return;
  }

  AutoOre_Feedback_t auto_ore_feedback = {
      .arm_at_target = AutoCtrlFeed_ArmSimpleAtTarget(),
      .ore_store_all_homed = Task_OreStoreIsAllHomed(),
      .ore_store_all_at_target = Task_OreStoreIsAllAtTarget(0.05f),
      .pole_all_at_target = Task_ChassisMainPoleAllAtTarget(
          AUTO_CTRL_POLE_TARGET_THRESHOLD_RAD),
  };
  AutoOre_Update(&auto_ore_ctrl, &auto_ore_feedback, now_ms);
}

bool Task_AutoOreStartStore(void) {
  return auto_ore_inited &&
         AutoOre_StartStore(&auto_ore_ctrl, osKernelGetTickCount());
}

bool Task_AutoOreStartRelease(void) {
  return auto_ore_inited &&
         AutoOre_StartRelease(&auto_ore_ctrl, osKernelGetTickCount());
}

void Task_AutoOreAbort(void) {
  if (auto_ore_inited) {
    AutoOre_Abort(&auto_ore_ctrl);
  }
}

void Task_AutoOreSetHeldOreCount(uint8_t held_ore_count) {
  if (auto_ore_inited) {
    AutoOre_SetHeldOreCount(&auto_ore_ctrl, held_ore_count);
  }
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

  while (1) {
    uint32_t now_ms;

    tick += delay_tick;

    if (auto_ctrl_inited) {
      now_ms = osKernelGetTickCount();

      AutoCtrlFeed_CacheLocalYawZero();

      feedback.yaw_auto_rad = AutoCtrlFeed_SelectYawRad();
      /* 当前 AutoCtrl API 只消费前向两路 SICK。 */
      feedback.sick_front_left_cm = distance_cm[2];
      feedback.sick_front_right_cm = distance_cm[3];

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
      AutoCtrlFeed_UpdateAutoOre(now_ms);

      if (g_pc_protocol_ptr != NULL) {
        PC_StepFeedback_t step_feedback = {0};
        step_feedback.state = (PC_StepState_t)AutoCtrl_GetState(&auto_ctrl);
        step_feedback.result = (PC_StepResult_t)AutoCtrl_GetResult(&auto_ctrl);
        step_feedback.fault = (PC_StepFault_t)AutoCtrl_GetFault(&auto_ctrl);
        step_feedback.template_id = (PC_StepTemplate_t)AutoCtrl_GetTemplate(&auto_ctrl);
        step_feedback.step_index = AutoCtrl_GetStepIndex(&auto_ctrl);
        step_feedback.progress = 0.0f;
        PC_Protocol_SetStepFeedback(g_pc_protocol_ptr, &step_feedback);
      }
    }

    task_runtime.stack_water_mark.auto_ctrl = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick);
  }
}
