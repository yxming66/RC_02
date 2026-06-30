/*
 * ore_info Task
 * Dedicated low-rate task for UART10 ore position messages.
 */

#include "task/user_task.h"

volatile OreInfo_Debug_t g_ore_info_debug = {0};

void Task_ore_info(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / ORE_INFO_FREQ;

  osDelay(ORE_INFO_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  OreInfo_Init(BSP_TIME_Get_ms());

  while (1) {
    const uint32_t profile_start_us =
        Task_ProfilerLoopBegin(TASK_PROFILE_ORE_INFO,
                               TASK_PERIOD_US(ORE_INFO_FREQ));
    tick += delay_tick;

    const uint32_t now_ms = BSP_TIME_Get_ms();
    OreInfo_Process(now_ms);

    task_runtime.heartbeat.ore_info++;
    Task_ProfilerLoopEnd(TASK_PROFILE_ORE_INFO, profile_start_us);
    Task_DelayUntil(TASK_PROFILE_ORE_INFO, &tick, delay_tick);
  }
}
