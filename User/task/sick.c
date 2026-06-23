#include "task/user_task.h"

void Task_sick(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / SICK_FREQ;

  osDelay(SICK_INIT_DELAY);

  if (SICK_Init() != DEVICE_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }

  uint32_t tick = osKernelGetTickCount();

  while (1) {
    const uint32_t profile_start_us =
        Task_ProfilerLoopBegin(TASK_PROFILE_SICK, TASK_PERIOD_US(SICK_FREQ));
    tick += delay_tick;

    SICK_Update(BSP_TIME_Get_ms());
    task_runtime.stack_water_mark.sick = uxTaskGetStackHighWaterMark(NULL);
    task_runtime.heartbeat.sick++;
    Task_ProfilerLoopEnd(TASK_PROFILE_SICK, profile_start_us);
    osDelayUntil(tick);
  }
}

bool Task_SickGetLatestOutput(Sick_Output_t *output) {
  return SICK_GetLatestOutput(output);
}
