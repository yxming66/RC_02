/*
    chassis_main Task
    
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "module/chassis.h"
#include "module/config.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
Chassis_t chassis;
Chassis_CMD_t chassis_cmd;   
Chassis_IMU_t chassis_imu;
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_chassis_main(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  
  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / CHASSIS_MAIN_FREQ;

  osDelay(CHASSIS_MAIN_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */

  Config_RobotParam_t *cfg = Config_GetRobotParam();
  Chassis_Init(&chassis, &cfg->chassis_param, (float)CHASSIS_MAIN_FREQ);
  chassis.mech_zero=0.57f;
  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
		osMessageQueueGet(task_runtime.msgq.chassis.cmd, &chassis_cmd, NULL, 0);
    chassis.feedback.encoder_gimbalYawMotor = 0.0f;
    
	  Chassis_UpdateFeedback(&chassis);
    Chassis_Control(&chassis, &chassis_cmd, osKernelGetTickCount());
    Chassis_Output(&chassis);
    
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}