/*
    auto_ctrl_feed Task
    Dedicated task for collecting sensor data and feeding AutoCtrl feedback.
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "main.h"
#include "module/autoCtrlAPI/api/auto_ctrl_api.h"
#include "module/chassis.h"

/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
extern Chassis_t chassis;
extern Chassis_IMU_t chassis_imu;
extern float distance_cm[4];
auto_ctrl_t auto_ctrl;
bool auto_ctrl_inited = false;
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_auto_ctrl_feed(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  const uint32_t delay_tick = osKernelGetTickFreq() / AUTO_CTRL_FEED_FREQ;

  osDelay(AUTO_CTRL_FEED_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();

  while (1) {
    tick += delay_tick;

    if (auto_ctrl_inited) {
      auto_ctrl_feedback_t feedback = {0};

      feedback.yaw_auto_deg = chassis_imu.eulr.yaw;
      /* distance_cm mapping: [0]=front, [1]=left, [2]=right, [3]=rear */
      feedback.sick_front_cm = distance_cm[0];
      feedback.sick_left_cm = distance_cm[1];
      feedback.sick_right_cm = distance_cm[2];
      feedback.sick_rear_cm = distance_cm[3];

      /* Optical gates are active-high: blocked path means pole retracted. */
        feedback.front_pole_retracted =
          HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET;
        feedback.rear_pole_retracted =
          HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) == GPIO_PIN_SET;

      AutoCtrl_SetFeedback(&auto_ctrl, &feedback);
    }

    osDelayUntil(tick);
  }
}
