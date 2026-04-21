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
#define SICK_DISTANCE_MIN_M (0.011)
#define SICK_DISTANCE_MAX_M (6.514)

#ifndef SICK_MISS_THRESHOLD
#define SICK_MISS_THRESHOLD (5u)
#endif

#ifndef SICK_CAN_QUEUE_SIZE
#define SICK_CAN_QUEUE_SIZE (20u)
#endif

#ifndef SICK_FRONT_S1_INDEX
#define SICK_FRONT_S1_INDEX (2u)
#endif

#ifndef SICK_FRONT_S2_INDEX
#define SICK_FRONT_S2_INDEX (3u)
#endif

#ifndef SICK_FRONT_S1_REF_M
#define SICK_FRONT_S1_REF_M (0.239326894)
#endif

#ifndef SICK_FRONT_S2_REF_M
#define SICK_FRONT_S2_REF_M (0.265225768)
#endif

#ifndef SICK_FRONT_S2_COMP_ENABLE
#define SICK_FRONT_S2_COMP_ENABLE (1u)
#endif

#define SICK_FRONT_S2_COMP_OFFSET_M (SICK_FRONT_S2_REF_M - SICK_FRONT_S1_REF_M)

BSP_CAN_Message_t msg[4];
uint16_t adc_raw[4] = {0u, 0u, 0u, 0u};
float distance_m[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float distance_cm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static uint16_t sick_miss_count[4] = {0u, 0u, 0u, 0u};
static const uint32_t sick_can_id[4] = {0x121u, 0x122u, 0x123u, 0x124u};
static bool sick_can_available[4] = {false, false, false, false};
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

static void SICK_UpdateAdcRaw(uint8_t index, uint32_t can_id) {
  bool updated = false;

  if (!sick_can_available[index]) {
    if (sick_miss_count[index] < 0xFFu) {
      sick_miss_count[index]++;
    }
    return;
  }

  /* Read all pending frames for this ID and keep the latest sample. */
  while (BSP_CAN_GetMessage(BSP_CAN_3, can_id, &msg[index], BSP_CAN_TIMEOUT_IMMEDIATE) ==
         BSP_OK) {
    adc_raw[index] = ((uint16_t)msg[index].data[0] << 8) | (uint16_t)msg[index].data[1];
    updated = true;
  }

  if (updated) {
    sick_miss_count[index] = 0u;
    return;
  }

  if (sick_miss_count[index] < 0xFFFFu) {
    sick_miss_count[index]++;
  }
}

static double SICK_ApplyChannelCalibrationM(uint8_t index, double distance_m_raw) {
#if SICK_FRONT_S2_COMP_ENABLE
  if (index == SICK_FRONT_S2_INDEX) {
    const double corrected_m = distance_m_raw - SICK_FRONT_S2_COMP_OFFSET_M;
    return (corrected_m < SICK_DISTANCE_MIN_M) ? SICK_DISTANCE_MIN_M : corrected_m;
  }
#endif

  (void)index;
  return distance_m_raw;
}

/* Exported functions ------------------------------------------------------- */
void Task_sick(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  
  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / SICK_FREQ;

  osDelay(SICK_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */
  int i;

  BSP_CAN_Init();
  for (i = 0; i < 4; i++) {
    const int8_t reg_ret = BSP_CAN_RegisterId(BSP_CAN_3, sick_can_id[i], SICK_CAN_QUEUE_SIZE);
    if (reg_ret == BSP_OK || BSP_CAN_GetQueueCount(BSP_CAN_3, sick_can_id[i]) >= 0) {
      sick_can_available[i] = true;
    }
  }

  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */

    SICK_UpdateAdcRaw(0u, sick_can_id[0]);
    SICK_UpdateAdcRaw(1u, sick_can_id[1]);
    SICK_UpdateAdcRaw(2u, sick_can_id[2]);
    SICK_UpdateAdcRaw(3u, sick_can_id[3]);

    for (int i = 0; i < 4; i++) {
      if (sick_miss_count[i] >= SICK_MISS_THRESHOLD) {
        distance_m[i] = -1.0f;
        distance_cm[i] = -1.0f;
        continue;
      }

      const double mapped_m = SICK_MapAdcToDistanceM(adc_raw[i]);
      const double calibrated_m = SICK_ApplyChannelCalibrationM((uint8_t)i, mapped_m);
      distance_m[i] = (float)calibrated_m;
      distance_cm[i] = (float)(calibrated_m * 100.0);
    }
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}