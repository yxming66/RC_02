/*
    rc_main Task

    This task owns DR16 receive/update only. Command generation lives in the
    cmd_center application task.
*/

#include "task/user_task.h"

#include "device/dr16.h"

DR16_t dr16;

void Task_rc_main(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / RC_MAIN_FREQ;
  osDelay(RC_MAIN_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  if (DR16_Init(&dr16) != DEVICE_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }
  if (DR16_StartDmaRecv(&dr16) != DEVICE_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }

  while (1) {
    tick += delay_tick;
    DR16_StartDmaRecv(&dr16);
    if (DR16_WaitDmaCplt(100)) {
      DR16_ParseData(&dr16);
    } else {
      DR16_Offline(&dr16);
    }

    task_runtime.stack_water_mark.rc_main = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick);
  }
}
