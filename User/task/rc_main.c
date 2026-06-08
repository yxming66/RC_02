/*
    rc_main task
*/

#include "task/user_task.h"

#include "device/dr16.h"
#include "module/rc_cmd_center/rc_cmd_center_app.h"

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

  RcCmdCenterApp_Init();

  while (1) {
    tick += delay_tick;
    DR16_StartDmaRecv(&dr16);
    if (DR16_WaitDmaCplt(100)) {
      DR16_ParseData(&dr16);
    } else {
      DR16_Offline(&dr16);
    }

    RcCmdCenterApp_Update(BSP_TIME_Get_ms());

    task_runtime.stack_water_mark.rc_main =
        uxTaskGetStackHighWaterMark(NULL);
    task_runtime.heartbeat.rc_main++;
    osDelayUntil(tick);
  }
}
