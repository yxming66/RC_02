#include "task/user_task.h"

volatile Sick_Debug_t g_sick_debug = {0};

static bool sick_init_attempted = false;
static bool sick_inited = false;

static void Task_SickUpdateDebug(void) {
  Sick_Output_t output = {0};
  g_sick_debug.can_bus = SICK_CAN_BUS;
  g_sick_debug.can_id = SICK_CAN_ID;

  if (!Task_SickGetLatestOutput(&output)) {
    g_sick_debug.read_fail_count++;
    return;
  }

  for (uint8_t i = 0u; i < SICK_OUTPUT_CHANNEL_COUNT; i++) {
    g_sick_debug.adc_raw[i] = output.adc_raw[i];
    g_sick_debug.distance_mm[i] = output.distance_mm[i];
    g_sick_debug.distance_m[i] = output.distance_m[i];
    g_sick_debug.valid[i] = output.valid[i];
  }
  g_sick_debug.miss_count = output.miss_count;
  g_sick_debug.update_tick = output.update_tick;
  g_sick_debug.read_count++;
}

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
  Task_SickUpdateDebug();
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
