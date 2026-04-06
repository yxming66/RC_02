/*
    blink Task
    
*/

/* Includes ----------------------------------------------------------------- */
#include "cmsis_os2.h"
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "module/arm.h"
#include "device/motor_dm.h"
#include "device/motor_lz.h"
#include "module/config.h"

/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
Arm_t arm;
Arm_CMD_t arm_cmd;

bool setzero=0;
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_arm(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  
  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / ARM_FREQ;

  osDelay(ARM_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */
    Arm_Init(&arm, &Config_GetRobotParam()->arm_param, (float)ARM_FREQ);

  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
    if(setzero){
      MOTOR_DM_SetZero(&arm.param->dmmotor_param);
      MOTOR_LZ_SetZero(&arm.param->lzmotor_param);
      setzero=0;
    }

    osMessageQueueGet(task_runtime.msgq.arm.cmd, &arm_cmd, NULL, 0);

    arm.mode = arm_cmd.mode;
    arm.point2point_mode = arm_cmd.point2point_mode;

    for (int i=0; i<ARM_POINT_NONE; i++) {
      arm.point2point[i].lzmotor_pos = 1.50471783f;
      arm.point2point[i].dmmotor_pos =  2.37619781f;
      // arm.point2point[i].rmmotor_pos = arm_cmd.point2point[i].rmmotor_pos;
    
    }
    arm.point2point[ARM_POINT_SLEEP].lzmotor_pos = 1.33017349f;
    arm.point2point[ARM_POINT_SLEEP].dmmotor_pos = 4.18284988f;

    arm.point2point[ARM_POINT_MINUS_20CM].lzmotor_pos = 1.33017349f;
    arm.point2point[ARM_POINT_MINUS_20CM].dmmotor_pos = 4.18284988f;

    arm.point2point[ARM_POINT_PLUS_20CM].lzmotor_pos = 1.33017349f;
    arm.point2point[ARM_POINT_PLUS_20CM].dmmotor_pos = 4.18284988f;  

    arm.point2point[ARM_POINT_PLUS_40CM].lzmotor_pos = 1.33017349f;
    arm.point2point[ARM_POINT_PLUS_40CM].dmmotor_pos = 4.18284988f;

    arm.point2point[ARM_POINT_SAVE_LOW].lzmotor_pos = 1.73181534f;
    arm.point2point[ARM_POINT_SAVE_LOW].dmmotor_pos = 5.78561783f;

    arm.point2point[ARM_POINT_SAVE_HIGH].lzmotor_pos = 2.11772919f;
    arm.point2point[ARM_POINT_SAVE_HIGH].dmmotor_pos = 5.21008968f;

    Arm_UpdateFeedback(&arm);
    Arm_Control(&arm, &arm_cmd);
    Arm_Output(&arm);
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}