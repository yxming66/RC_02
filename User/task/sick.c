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
    tick += delay_tick;

    SICK_Update(osKernelGetTickCount());
    task_runtime.stack_water_mark.sick = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick);
  }
}

bool Task_SickGetLatestOutput(Sick_Output_t *output) {
  return SICK_GetLatestOutput(output);
}
