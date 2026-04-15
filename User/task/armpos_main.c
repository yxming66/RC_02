/*
 * ArmPos control task.
 */

#include "task/user_task.h"

#include "module/armpos.h"
#include "module/config.h"

static ArmPos_t armpos;
static ArmPos_CMD_t armpos_cmd;

void Task_armpos(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / ARM_FREQ;
  osDelay(ARM_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  ArmPos_Init(&armpos, &Config_GetRobotParam()->armpos_param, (float)ARM_FREQ);

  armpos_cmd.mode = ARMPOS_MODE_RELAX;
  armpos_cmd.lz_delta = 0.0f;
  armpos_cmd.dm_delta = 0.0f;
  armpos_cmd.rm_delta = 0.0f;

  while (1) {
    tick += delay_tick;

    (void)osMessageQueueGet(task_runtime.msgq.armpos.cmd, &armpos_cmd, NULL, 0);

    ArmPos_UpdateFeedback(&armpos);
    ArmPos_Control(&armpos, &armpos_cmd);
    if (armpos.mode == ARMPOS_MODE_RELAX) {
      ArmPos_Relax(&armpos);
    } else {
      ArmPos_Output(&armpos);
    }

    osDelayUntil(tick);
  }
}