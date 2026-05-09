/* Includes ----------------------------------------------------------------- */
#include "tim.h"
#include "bsp/pwm.h"
#include "bsp.h"
#include "stm32h7xx_hal_rcc.h"
#include <math.h>

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */

/* Private define ----------------------------------------------------------- */
/* USER DEFINE BEGIN */

/* USER DEFINE END */

/* Private macro ------------------------------------------------------------ */
/* Private typedef ---------------------------------------------------------- */
typedef struct {
  TIM_HandleTypeDef *tim;
  uint16_t channel;
} BSP_PWM_Config_t;

/* USER STRUCT BEGIN */

/* USER STRUCT END */

/* Private variables -------------------------------------------------------- */
static const BSP_PWM_Config_t PWM_Map[BSP_PWM_NUM] = {
  {&htim12, TIM_CHANNEL_2},
    {&htim3, TIM_CHANNEL_4},
};

static uint32_t BSP_PWM_GetTimerClock(TIM_HandleTypeDef *htim) {
  RCC_ClkInitTypeDef clk_init = {0};
  uint32_t flash_latency = 0;

  HAL_RCC_GetClockConfig(&clk_init, &flash_latency);

  if (htim->Instance == TIM1 || htim->Instance == TIM8 || htim->Instance == TIM15 ||
      htim->Instance == TIM16 || htim->Instance == TIM17) {
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    return (clk_init.APB2CLKDivider == RCC_APB2_DIV1) ? pclk2 : (pclk2 * 2U);
  }

  if (htim->Instance == TIM2 || htim->Instance == TIM3 || htim->Instance == TIM4 ||
      htim->Instance == TIM5 || htim->Instance == TIM6 || htim->Instance == TIM7 ||
      htim->Instance == TIM12 || htim->Instance == TIM13 || htim->Instance == TIM14) {
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    return (clk_init.APB1CLKDivider == RCC_APB1_DIV1) ? pclk1 : (pclk1 * 2U);
  }

  return HAL_RCC_GetPCLK1Freq();
}

/* Private function  -------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */

int8_t BSP_PWM_Start(BSP_PWM_Channel_t ch) {
  if (ch >= BSP_PWM_NUM) return BSP_ERR;
  
  HAL_TIM_PWM_Start(PWM_Map[ch].tim, PWM_Map[ch].channel);
  return BSP_OK;
}

int8_t BSP_PWM_SetComp(BSP_PWM_Channel_t ch, float duty_cycle) {
  if (ch >= BSP_PWM_NUM) return BSP_ERR;
  
  if (duty_cycle > 1.0f) {
    duty_cycle = 1.0f;
  }
  if (duty_cycle < 0.0f) {
    duty_cycle = 0.0f;
  }
  // 获取ARR值（周期值）
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(PWM_Map[ch].tim);
  
  // 计算比较值：CCR = duty_cycle * (ARR + 1)
  uint32_t ccr = (uint32_t)(duty_cycle * (arr + 1));
  
  __HAL_TIM_SET_COMPARE(PWM_Map[ch].tim, PWM_Map[ch].channel, ccr);

  return BSP_OK;
}

int8_t BSP_PWM_SetFreq(BSP_PWM_Channel_t ch, float freq) {
  if (ch >= BSP_PWM_NUM) return BSP_ERR;

  if (freq <= 0.0f) return BSP_ERR;

  uint32_t timer_clock = BSP_PWM_GetTimerClock(PWM_Map[ch].tim);
  uint32_t prescaler = PWM_Map[ch].tim->Init.Prescaler;
  uint32_t period = (uint32_t)(((float)timer_clock / (float)(prescaler + 1U)) / freq) - 1U;
  
  if (period > UINT16_MAX) {
    uint32_t divider = (uint32_t)ceilf((float)timer_clock /
                                       (freq * (float)(UINT16_MAX + 1U)));
    if (divider == 0U || divider > (UINT16_MAX + 1U)) {
      return BSP_ERR; // Frequency too low
    }

    prescaler = divider - 1U;
    period = (uint32_t)(((float)timer_clock / (float)(prescaler + 1U)) / freq) - 1U;
    if (period > UINT16_MAX) {
      return BSP_ERR; // Frequency too low
    }

    PWM_Map[ch].tim->Init.Prescaler = prescaler;
    __HAL_TIM_SET_PRESCALER(PWM_Map[ch].tim, prescaler);
  }
  __HAL_TIM_SET_AUTORELOAD(PWM_Map[ch].tim, period);
  __HAL_TIM_SET_COUNTER(PWM_Map[ch].tim, 0);
  PWM_Map[ch].tim->Instance->EGR = TIM_EGR_UG;
  
  return BSP_OK;
}

int8_t BSP_PWM_Stop(BSP_PWM_Channel_t ch) {
  if (ch >= BSP_PWM_NUM) return BSP_ERR;
  
  HAL_TIM_PWM_Stop(PWM_Map[ch].tim, PWM_Map[ch].channel);
  return BSP_OK;
}

uint32_t BSP_PWM_GetAutoReloadPreload(BSP_PWM_Channel_t ch) {
    if (ch >= BSP_PWM_NUM) return BSP_ERR;
    return PWM_Map[ch].tim->Init.AutoReloadPreload;  
}

TIM_HandleTypeDef* BSP_PWM_GetHandle(BSP_PWM_Channel_t ch) {
  return PWM_Map[ch].tim;
}


uint16_t BSP_PWM_GetChannel(BSP_PWM_Channel_t ch) {
  if (ch >= BSP_PWM_NUM) return BSP_ERR;
  return PWM_Map[ch].channel;
}

int8_t BSP_PWM_Start_DMA(BSP_PWM_Channel_t ch, uint32_t *pData, uint16_t Length) {
  if (ch >= BSP_PWM_NUM) return BSP_ERR;
  
  HAL_TIM_PWM_Start_DMA(PWM_Map[ch].tim, PWM_Map[ch].channel, pData, Length);
  return BSP_OK;
}

int8_t BSP_PWM_Stop_DMA(BSP_PWM_Channel_t ch) {
  if (ch >= BSP_PWM_NUM) return BSP_ERR;
  
  HAL_TIM_PWM_Stop_DMA(PWM_Map[ch].tim, PWM_Map[ch].channel);
  return BSP_OK;
}
