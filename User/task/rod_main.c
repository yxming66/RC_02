/*
 * Rod control task.
 */

#include "task/user_task.h"

#include "module/config.h"
#include "module/rod.h"

static Rod_t rod;
static Rod_CMD_t rod_cmd;
bool rodsetzero=0;
void Task_rod(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / ROD_FREQ;
  osDelay(ROD_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  Rod_Init(&rod, &Config_GetRobotParam()->rod_param, (float)ROD_FREQ);

  while (1) {
    tick += delay_tick;

    osMessageQueueGet(task_runtime.msgq.rod.cmd, &rod_cmd, NULL, 0);
    
    if (rodsetzero) {
      MOTOR_DM_SetZero(&rod.param->pit_motor_param);
      MOTOR_DM_SetZero(&rod.param->rol_motor_param);
      rodsetzero = 0;  
    }
    Rod_UpdateFeedback(&rod);
    Rod_Control(&rod, &rod_cmd, osKernelGetTickCount());
    // Rod_Output(&rod);
    // Rod_ResetOutput(&rod);
    osDelayUntil(tick);
  }
}
