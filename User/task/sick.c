/*
    sick Task
    
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "bsp/can.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
#define SICK_ADC_MIN (0.0)
#define SICK_ADC_MAX (65535.0)
#define SICK_DISTANCE_MIN_M (0.05)
#define SICK_DISTANCE_MAX_M (6.00)

BSP_CAN_Message_t msg[4];
uint16_t adc_raw[4] = {0u, 0u, 0u, 0u};
float distance_m[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float distance_cm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
static double SICK_MapAdcToDistanceM(uint16_t adc_raw_value) {
  const double adc_span = SICK_ADC_MAX - SICK_ADC_MIN;
  const double distance_span = SICK_DISTANCE_MAX_M - SICK_DISTANCE_MIN_M;
  double norm = 0.0;

  if (adc_span > 0.0) {
    norm = ((double)adc_raw_value - SICK_ADC_MIN) / adc_span;
  }

  if (norm < 0.0) {
    norm = 0.0;
  } else if (norm > 1.0) {
    norm = 1.0;
  }

  if (distance_span <= 0.0) {
    return SICK_DISTANCE_MIN_M;
  }

  return SICK_DISTANCE_MIN_M + norm * distance_span;
}

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
      adc_raw[0] = ((uint16_t)msg[0].data[0] << 8) | (uint16_t)msg[0].data[1];
    }
    if (BSP_CAN_GetMessage(BSP_CAN_1, 0x122, &msg[1], BSP_CAN_TIMEOUT_IMMEDIATE) == BSP_OK) {
      /* 成功获取消息，处理msg */
      adc_raw[1] = ((uint16_t)msg[1].data[0] << 8) | (uint16_t)msg[1].data[1];
    }
    if (BSP_CAN_GetMessage(BSP_CAN_1, 0x123, &msg[2], BSP_CAN_TIMEOUT_IMMEDIATE) == BSP_OK) {
      /* 成功获取消息，处理msg */
      adc_raw[2] = ((uint16_t)msg[2].data[0] << 8) | (uint16_t)msg[2].data[1];
    }
    if (BSP_CAN_GetMessage(BSP_CAN_1, 0x124, &msg[3], BSP_CAN_TIMEOUT_IMMEDIATE) == BSP_OK) {
      /* 成功获取消息，处理msg */
      adc_raw[3] = ((uint16_t)msg[3].data[0] << 8) | (uint16_t)msg[3].data[1];
    }

    for (int i = 0; i < 4; i++) {
      const double mapped_m = SICK_MapAdcToDistanceM(adc_raw[i]);
      distance_m[i] = (float)mapped_m;
      distance_cm[i] = (float)(mapped_m * 100.0);
    }
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}