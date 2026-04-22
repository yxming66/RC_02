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
#include "module/autoCtrlAPI/transition/auto_ctrl_transition.h"
#include "module/autoCtrlAPI/transition/auto_ctrl_zone.h"
#include "module/chassis.h"

/* USER INCLUDE END */

#ifndef AUTO_CTRL_DEFAULT_START_ZONE
#define AUTO_CTRL_DEFAULT_START_ZONE AUTO_CTRL_ZONE_R2_ENTRY2
#endif

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
extern Chassis_t chassis;
extern Chassis_IMU_t chassis_imu;
extern float distance_cm[4];
extern DR16_t dr16;

auto_ctrl_t auto_ctrl;
bool auto_ctrl_inited = false;
auto_ctrl_feedback_t feedback = {0};
static DR16_SwitchPos_t auto_last_sw_r = DR16_SW_ERR;
static const GPIO_PinState front_pole_photo_active_state = GPIO_PIN_RESET;
static const GPIO_PinState rear_pole_photo_active_state = GPIO_PIN_RESET;

static auto_ctrl_zone_e AutoCtrlTask_FindZoneByHeight(
    auto_ctrl_zone_e from_zone, int16_t target_height_mm) {
  auto_ctrl_zone_e zone;

  for (zone = (auto_ctrl_zone_e)0; zone < AUTO_CTRL_ZONE_COUNT;
       zone = (auto_ctrl_zone_e)(zone + 1)) {
    if (!AutoCtrlZone_IsPlatform(zone)) {
      continue;
    }
    if (AutoCtrlZone_GetHeightMm(zone) != target_height_mm) {
      continue;
    }
    if (!AutoCtrlTransition_IsLegal(from_zone, zone)) {
      continue;
    }
    return zone;
  }

  return AUTO_CTRL_ZONE_INVALID;
}

static bool AutoCtrlTask_StartByDelta(auto_ctrl_t *ctrl, int16_t delta_height_mm,
                                      uint32_t now_ms) {
  auto_ctrl_zone_e from_zone = ctrl->current_zone;
  int16_t current_height_mm;
  auto_ctrl_zone_e to_zone;

  if (!AutoCtrlZone_IsValid(from_zone)) {
    return false;
  }

  current_height_mm = AutoCtrlZone_GetHeightMm(from_zone);
  to_zone = AutoCtrlTask_FindZoneByHeight(
      from_zone, (int16_t)(current_height_mm + delta_height_mm));
  if (to_zone == AUTO_CTRL_ZONE_INVALID) {
    return false;
  }

  return AutoCtrl_StartTransition(ctrl, from_zone, to_zone, now_ms);
}
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_auto_ctrl(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  const uint32_t delay_tick = osKernelGetTickFreq() / AUTO_CTRL_FREQ;

  osDelay(AUTO_CTRL_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();

  AutoCtrl_Init(&auto_ctrl);
  if (auto_ctrl.current_zone == AUTO_CTRL_ZONE_INVALID) {
    auto_ctrl.current_zone = AUTO_CTRL_DEFAULT_START_ZONE;
  }
  auto_ctrl_inited = true;

  while (1) {
    uint32_t now_ms;

    tick += delay_tick;

    if (auto_ctrl_inited) {
      now_ms = osKernelGetTickCount();

      feedback.yaw_auto_rad = chassis_imu.eulr.yaw;
      /* 当前 AutoCtrl API 只消费前向两路 SICK。 */
      feedback.sick_front_left_cm = distance_cm[2];
      feedback.sick_front_right_cm = distance_cm[3];

      GPIO_PinState front_pole_gpio_state =
        HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
      GPIO_PinState rear_pole_gpio_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);

      feedback.front_pole_retracted =
        (front_pole_gpio_state == front_pole_photo_active_state);
      feedback.rear_pole_retracted =
        (rear_pole_gpio_state == rear_pole_photo_active_state);

      AutoCtrl_SetFeedback(&auto_ctrl, &feedback);

      if (dr16.header.online) {
        if ((dr16.data.sw_l == DR16_SW_MID || dr16.data.sw_l == DR16_SW_DOWN) &&
            AutoCtrl_IsBusy(&auto_ctrl)) {
          AutoCtrl_Update(&auto_ctrl, now_ms);
        } else if ((dr16.data.sw_l == DR16_SW_MID ||
                    dr16.data.sw_l == DR16_SW_DOWN) &&
                   auto_last_sw_r == DR16_SW_MID) {
          if (dr16.data.sw_l == DR16_SW_MID) {
            if (dr16.data.sw_r == DR16_SW_UP) {
              (void)AutoCtrlTask_StartByDelta(&auto_ctrl, 200, now_ms);
            } else if (dr16.data.sw_r == DR16_SW_DOWN) {
              (void)AutoCtrlTask_StartByDelta(&auto_ctrl, -200, now_ms);
            }
          } else {
            if (dr16.data.sw_r == DR16_SW_UP) {
              (void)AutoCtrlTask_StartByDelta(&auto_ctrl, 400, now_ms);
            } else if (dr16.data.sw_r == DR16_SW_DOWN) {
              (void)AutoCtrlTask_StartByDelta(&auto_ctrl, -400, now_ms);
            }
          }
        }

        auto_last_sw_r = dr16.data.sw_r;
      } else {
        auto_last_sw_r = DR16_SW_ERR;
      }
    }

    osDelayUntil(tick);
  }
}
