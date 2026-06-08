/* Includes ----------------------------------------------------------------- */
#include "bsp/time.h"
#include "bsp.h"

#include <cmsis_os2.h>
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#include "stm32h7xx_hal_tim.h"

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */
/* Private define ----------------------------------------------------------- */
/* USER DEFINE BEGIN */

/* USER DEFINE END */

/* Private macro ------------------------------------------------------------ */
/* Private typedef ---------------------------------------------------------- */
/* USER STRUCT BEGIN */

/* USER STRUCT END */

/* Private variables -------------------------------------------------------- */
extern TIM_HandleTypeDef htim2;

/* Private function  -------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */

uint32_t BSP_TIME_Get_ms() {
  return (uint32_t)(BSP_TIME_Get_us() / 1000ULL);
}

uint64_t BSP_TIME_Get_us() {
  uint32_t ms = HAL_GetTick();
  uint32_t us_in_ms = __HAL_TIM_GET_COUNTER(&htim2);

  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET &&
      us_in_ms < 500U) {
    ms++;
  }

  return (uint64_t)ms * 1000ULL + (uint64_t)us_in_ms;
}

uint64_t BSP_TIME_Get() __attribute__((alias("BSP_TIME_Get_us")));

int8_t BSP_TIME_Delay_ms(uint32_t ms) {
  uint32_t tick_period = 1000u / osKernelGetTickFreq();
  uint32_t ticks = ms / tick_period;

  switch (osKernelGetState()) {
    case osKernelError:
    case osKernelReserved:
    case osKernelLocked:
    case osKernelSuspended:
      return BSP_ERR;

    case osKernelRunning:
      osDelay(ticks ? ticks : 1);
      break;

    case osKernelInactive:
    case osKernelReady:
      HAL_Delay(ms);
      break;
  }
  return BSP_OK;
}

/*阻塞us延迟*/
int8_t BSP_TIME_Delay_us(uint32_t us) {
    uint64_t start = BSP_TIME_Get_us();
    while (BSP_TIME_Get_us() - start < us) {
        // 等待us时间
    }
    return BSP_OK;
}

int8_t BSP_TIME_Delay(uint32_t ms) __attribute__((alias("BSP_TIME_Delay_ms")));

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */
