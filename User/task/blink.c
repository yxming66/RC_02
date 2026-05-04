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
static bool heartbeat_on;
static uint32_t heartbeat_start_tick;
static uint32_t heartbeat_last_start_tick;
bool reset=0;
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_blink(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  
  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / BLINK_FREQ;
  const uint32_t heartbeat_period_tick = osKernelGetTickFreq()*3u;
  const uint32_t heartbeat_on_tick = osKernelGetTickFreq() / 20u;

  osDelay(BLINK_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */
  if (!buzzer.header.online) {
    if (BUZZER_Init(&buzzer, BSP_PWM_BUZZER) != DEVICE_OK) {
      osThreadTerminate(osThreadGetId());
      return;
    }
  }
    BUZZER_PlayMusic(&buzzer, MUSIC_NOKIA);

  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
    const uint32_t now_tick = osKernelGetTickCount();
    if (!g_buzzer_calib_active) {
      if (!heartbeat_on &&
          (heartbeat_last_start_tick == 0u ||
           (uint32_t)(now_tick - heartbeat_last_start_tick) >=
               heartbeat_period_tick)) {
        /* 每秒开始播放C4音符 */
        BUZZER_Set(&buzzer, 523.25f, 0.15f);
        BUZZER_Start(&buzzer);
        heartbeat_on = true;
        heartbeat_start_tick = now_tick;
        heartbeat_last_start_tick = now_tick;
      } else if (heartbeat_on &&
                 (uint32_t)(now_tick - heartbeat_start_tick) >=
                     heartbeat_on_tick) {
        /* Stop after about 100 ms. */
        BUZZER_Stop(&buzzer);
        heartbeat_on = false;
      }
    } else {
      BUZZER_Stop(&buzzer);
      heartbeat_on = false;
      heartbeat_last_start_tick = now_tick;
    }
    if (reset) {
      __set_FAULTMASK(1);      /* 关闭所有中断 */
      NVIC_SystemReset();       /* 系统复位 */
    }
    /* USER CODE END */
    task_runtime.stack_water_mark.blink = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}
