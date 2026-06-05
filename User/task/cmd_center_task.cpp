/*
 * Cmd center task.
 */

#include "task/user_task.h"

#include "module/cmd_center/rc_cmd_center_app.h"

void Task_cmd_center(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / CMD_CENTER_FREQ;
  osDelay(CMD_CENTER_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  RcCmdCenterApp_Init();

  while (1) {
    tick += delay_tick;
    RcCmdCenterApp_Update(osKernelGetTickCount());
    task_runtime.stack_water_mark.cmd_center =
        uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick);
  }
}
