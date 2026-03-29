/*
    pole_main Task
*/

#include "task/user_task.h"

/* USER INCLUDE BEGIN */
#include "module/config.h"
#include "module/pole.h"
/* USER INCLUDE END */

 Pole_t pole;
static Pole_CMD_t pole_cmd;

void Task_pole_main(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / POLE_MAIN_FREQ;
  osDelay(POLE_MAIN_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();

  Config_RobotParam_t *cfg = Config_GetRobotParam();
  Pole_Init(&pole, &cfg->pole_param, (float)POLE_MAIN_FREQ);
   
  while (1) {
    tick += delay_tick;

    osMessageQueueGet(task_runtime.msgq.pole.cmd, &pole_cmd, NULL, 0);

    Pole_UpdateFeedback(&pole);
    Pole_Control(&pole, &pole_cmd, osKernelGetTickCount());
Pole_Output(&pole);

		
    // static float out[4]={0.0f,0.0f,0.0f,0.0f};

    // MOTOR_RM_SetOutput((MOTOR_RM_Param_t *)&cfg->pole_param.motor_param[0], out[0]);
    // MOTOR_RM_SetOutput((MOTOR_RM_Param_t *)&cfg->pole_param.motor_param[1], out[1]);
    // MOTOR_RM_SetOutput((MOTOR_RM_Param_t *)&cfg->pole_param.motor_param[2], out[2]);
    // MOTOR_RM_SetOutput((MOTOR_RM_Param_t *)&cfg->pole_param.motor_param[3], out[3]);

    // MOTOR_RM_Ctrl((MOTOR_RM_Param_t *)&cfg->pole_param.motor_param[0]);
    // MOTOR_RM_Ctrl((MOTOR_RM_Param_t *)&cfg->pole_param.motor_param[1]);
    // MOTOR_RM_Ctrl((MOTOR_RM_Param_t *)&cfg->pole_param.motor_param[2]);
    // MOTOR_RM_Ctrl((MOTOR_RM_Param_t *)&cfg->pole_param.motor_param[3]);

    osDelayUntil(tick);
  }
}
