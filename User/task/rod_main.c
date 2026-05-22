/*
 * Rod control task.
 */

#include "task/user_task.h"

#include "module/config.h"
#include "module/rod_new.h"

static RodNew_t rod_new;
static RodNew_CMD_t rod_new_cmd;

volatile RodNew_DebugControl_t g_rod_new_debug = {
  false,
  false,
  BSP_PWM_ROD_SERVO,
  0.0f,
  ROD_NEW_SERVO_PULSE_NEUTRAL_US,
  ROD_NEW_GRIP_RELEASE,
};

void Task_rod(void *argument) {
  (void)argument;
 
  const uint32_t delay_tick = osKernelGetTickFreq() / ROD_FREQ;
  osDelay(ROD_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  Config_RobotParam_t *cfg = Config_GetRobotParam();
  if (cfg == NULL ||
      RodNew_Init(&rod_new, &cfg->rod_new_param) != ROD_NEW_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }

  rod_new_cmd.mode = ROD_NEW_MODE_RELAX;
  rod_new_cmd.pose = ROD_NEW_POSE_STANDBY;
  rod_new_cmd.grip = ROD_NEW_GRIP_RELEASE;

  while (1) { 
    tick += delay_tick;

    osMessageQueueGet(task_runtime.msgq.rod.cmd, &rod_new_cmd, NULL, 0);

    if (!g_rod_new_debug.enable) {
      RodNew_Control(&rod_new, rod_new_cmd.mode, rod_new_cmd.pose,
                     rod_new_cmd.grip, osKernelGetTickCount());
    }
    RodNew_Output(&rod_new);

    task_runtime.stack_water_mark.rod = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick);
  }
}
