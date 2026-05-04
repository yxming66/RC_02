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
#include "module/chassis.h"

/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
extern Chassis_IMU_t chassis_imu;
extern float distance_cm[4];
extern DR16_t dr16;

auto_ctrl_t auto_ctrl;
bool auto_ctrl_inited = false;
auto_ctrl_feedback_t feedback = {0};
static const GPIO_PinState head_front_photo_active_state = GPIO_PIN_RESET;
static const GPIO_PinState head_rear_photo_active_state = GPIO_PIN_RESET;
static const GPIO_PinState tail_front_photo_active_state = GPIO_PIN_RESET;
static const GPIO_PinState tail_rear_photo_active_state = GPIO_PIN_RESET;
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_auto_ctrl(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  const uint32_t delay_tick = osKernelGetTickFreq() / AUTO_CTRL_FREQ;

  osDelay(AUTO_CTRL_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();

  AutoCtrl_Init(&auto_ctrl);
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

      GPIO_PinState head_front_photo_gpio_state =
        HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
      GPIO_PinState head_rear_photo_gpio_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
      const bool tail_front_photo_triggered = false;
      const bool tail_rear_photo_triggered = false;

      feedback.head_front_photo_triggered =
        (head_front_photo_gpio_state == head_front_photo_active_state);
      feedback.head_rear_photo_triggered =
        (head_rear_photo_gpio_state == head_rear_photo_active_state);
      feedback.tail_front_photo_triggered =
        (tail_front_photo_triggered == (tail_front_photo_active_state == GPIO_PIN_SET));
      feedback.tail_rear_photo_triggered =
        (tail_rear_photo_triggered == (tail_rear_photo_active_state == GPIO_PIN_SET));

      AutoCtrl_SetFeedback(&auto_ctrl, &feedback);

      AutoCtrl_Update(&auto_ctrl, now_ms);
    }

    task_runtime.stack_water_mark.auto_ctrl = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick);
  }
}
