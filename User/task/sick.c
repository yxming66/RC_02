#include "task/user_task.h"

static bool sick_init_attempted = false;
static bool sick_inited = false;

bool Task_SickInitOnce(void) {
  if (sick_inited) {
    return true;
  }
  if (sick_init_attempted) {
    return false;
  }
  sick_init_attempted = true;

  if (SICK_Init() != DEVICE_OK) {
    return false;
  }

  sick_inited = true;
  return true;
}

void Task_SickStep(void) {
  if (!sick_inited) {
    return;
  }

  const uint32_t profile_start_us =
      Task_ProfilerLoopBegin(TASK_PROFILE_SICK, TASK_PERIOD_US(SICK_FREQ));

  SICK_Update(BSP_TIME_Get_ms());
  task_runtime.stack_water_mark.sick = uxTaskGetStackHighWaterMark(NULL);
  task_runtime.heartbeat.sick++;
  Task_ProfilerLoopEnd(TASK_PROFILE_SICK, profile_start_us);
}

void Task_sick(void *argument) {
  (void)argument;

  uint32_t delay_tick = osKernelGetTickFreq() / SICK_FREQ;
  if (delay_tick == 0U) {
    delay_tick = 1U;
  }

  osDelay(SICK_INIT_DELAY);

  if (!Task_SickInitOnce()) {
    osThreadTerminate(osThreadGetId());
    return;
  }

  uint32_t tick = osKernelGetTickCount();

  while (1) {
    tick += delay_tick;
    Task_SickStep();
    Task_DelayUntil(TASK_PROFILE_SICK, &tick, delay_tick);
  }
}

bool Task_SickGetLatestOutput(Sick_Output_t *output) {
  return SICK_GetLatestOutput(output);
}
