/*
    blink Task
    
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "bsp/pwm.h"
#include <math.h>
#include "device/buzzer.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
BUZZER_t buzzer;
static uint16_t count;
bool reset=0;
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_blink(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  
  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / BLINK_FREQ;

  osDelay(BLINK_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */
  BUZZER_Init(&buzzer, BSP_PWM_BUZZER);
    BUZZER_PlayMusic(&buzzer, MUSIC_NOKIA);

  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
    count++;
    uint16_t phase = count % 1000;
    if (count == 1001) count = 1;
    if (phase == 0) {
      /* 每秒开始播放C4音符 */
      // BUZZER_Set(&buzzer, 800.63f, 0.5f); // C4音符频率约261.63Hz
      // BUZZER_Start(&buzzer);
    } else if (phase == 50) {
      /* 播放100ms后停止 (50/500Hz = 0.1s) */
      BUZZER_Stop(&buzzer);
    }
    if (reset) {
      __set_FAULTMASK(1);      /* 关闭所有中断 */
      NVIC_SystemReset();       /* 系统复位 */
    }
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}