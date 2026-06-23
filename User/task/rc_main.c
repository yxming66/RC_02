/*
    rc_main task
*/

#include "task/user_task.h"

#include "device/dr16.h"
#include "module/rc_cmd_center/rc_cmd_center_app.h"

#define RC_MAIN_DR16_OFFLINE_TIMEOUT_US (100000ULL)

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
    DR16_Restart();
    if (DR16_StartDmaRecv(&dr16) != DEVICE_OK) {
      osThreadTerminate(osThreadGetId());
      return;
    }
  }

  RcCmdCenterApp_Init();

  while (1) {
    const uint32_t profile_start_us =
        Task_ProfilerLoopBegin(TASK_PROFILE_RC_MAIN,
                               TASK_PERIOD_US(RC_MAIN_FREQ));
    tick += delay_tick;
    if (DR16_WaitDmaCplt(0)) {
      if (DR16_ParseData(&dr16) != DEVICE_OK) {
        DR16_Offline(&dr16);
        DR16_Restart();
      }
      if (DR16_StartDmaRecv(&dr16) != DEVICE_OK) {
        DR16_Offline(&dr16);
        DR16_Restart();
        (void)DR16_StartDmaRecv(&dr16);
      }
    } else if (dr16.header.online &&
               (BSP_TIME_Get_us() - dr16.header.last_online_time) >
                   RC_MAIN_DR16_OFFLINE_TIMEOUT_US) {
      DR16_Offline(&dr16);
      DR16_Restart();
      (void)DR16_StartDmaRecv(&dr16);
    }

    RcCmdCenterApp_Update(BSP_TIME_Get_ms());

    task_runtime.stack_water_mark.rc_main =
        uxTaskGetStackHighWaterMark(NULL);
    task_runtime.heartbeat.rc_main++;
    Task_ProfilerLoopEnd(TASK_PROFILE_RC_MAIN, profile_start_us);
    osDelayUntil(tick);
  }
}
