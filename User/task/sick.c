/*
    sick Task
    
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "bsp/can.h"
#include <math.h>
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
  #define SICKTRAVEL 10.0f

    BSP_CAN_Message_t msg[4];
    float ad[4]={0.0f, 0.0f, 0.0f, 0.0f};
    float distance_m[4]={0.0f, 0.0f, 0.0f, 0.0f};
    float distance_cm[4]={0.0f, 0.0f, 0.0f, 0.0f};
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_sick(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  
  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / SICK_FREQ;

  osDelay(SICK_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */
  BSP_CAN_Init();
  BSP_CAN_RegisterId(BSP_CAN_1, 0x121, 3);
  BSP_CAN_RegisterId(BSP_CAN_1, 0x122, 3);
  BSP_CAN_RegisterId(BSP_CAN_1, 0x123, 3);
  BSP_CAN_RegisterId(BSP_CAN_1, 0x124, 3);

  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */

    if (BSP_CAN_GetMessage(BSP_CAN_1, 0x121, &msg[0], BSP_CAN_TIMEOUT_IMMEDIATE) == BSP_OK) {
      /* 成功获取消息，处理msg */
      ad[0] = (float)msg[0].data[0]*256.0f+(float)msg[0].data[1]; /* 假设前两个字节是模拟量 */
    }
    if (BSP_CAN_GetMessage(BSP_CAN_1, 0x122, &msg[1], BSP_CAN_TIMEOUT_IMMEDIATE) == BSP_OK) {
      /* 成功获取消息，处理msg */
      ad[1] = (float)msg[1].data[0]*256.0f+(float)msg[1].data[1]; /* 假设前两个字节是模拟量 */
    }
    if (BSP_CAN_GetMessage(BSP_CAN_1, 0x123, &msg[2], BSP_CAN_TIMEOUT_IMMEDIATE) == BSP_OK) {
      /* 成功获取消息，处理msg */
      ad[2] = (float)msg[2].data[0]*256.0f+(float)msg[2].data[1]; /* 假设前两个字节是模拟量 */
    }
    if (BSP_CAN_GetMessage(BSP_CAN_1, 0x124, &msg[3], BSP_CAN_TIMEOUT_IMMEDIATE) == BSP_OK) {
      /* 成功获取消息，处理msg */
      ad[3] = (float)msg[3].data[0]*256.0f+(float)msg[3].data[1]; /* 假设前两个字节是模拟量 */
    }

    for (int i = 0; i < 4; i++) {
      distance_m[i] = ad[i] / 65535.0f * SICKTRAVEL; 
      distance_cm[i] = distance_m[i] * 100.0f;
    }
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}