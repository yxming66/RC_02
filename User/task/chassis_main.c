/*
    chassis_main Task
    
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "module/chassis.h"
#include "module/config.h"
#include "module/pole.h"
#include "module/pole_auto.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
Chassis_t chassis;
Chassis_CMD_t chassis_cmd;   
Chassis_IMU_t chassis_imu;

Pole_t pole;
static Pole_CMD_t pole_cmd;
static Pole_Auto_t pole_auto;
static Pole_AutoCmd_t pole_auto_cmd;
extern float distance_m[4];
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

  Pole_Init(&pole, &cfg->pole_param, (float)CHASSIS_MAIN_FREQ);
  Pole_AutoInit(&pole_auto);
  pole_auto_cmd.enable = false;
  pole_auto_cmd.restart = false;
  pole_auto_cmd.step_type = POLE_AUTO_STEP_NONE;
  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
		osMessageQueueGet(task_runtime.msgq.chassis.cmd, &chassis_cmd, NULL, 0);
    chassis.feedback.encoder_gimbalYawMotor = 0.0f;
    
	  Chassis_UpdateFeedback(&chassis);
    Chassis_Control(&chassis, &chassis_cmd, osKernelGetTickCount());
    Chassis_Output(&chassis);
    
		osMessageQueueGet(task_runtime.msgq.pole.cmd, &pole_cmd, NULL, 0);
    (void)osMessageQueueGet(task_runtime.msgq.pole.auto_cmd, &pole_auto_cmd, NULL, 0);
    Pole_UpdateFeedback(&pole);
    if (pole_auto_cmd.enable) {
      Pole_AutoSensor_t sensor = {0};
      for (uint8_t i = 0; i < 4u; ++i) {
        sensor.distance_m[i] = distance_m[i];
        sensor.valid[i] = (distance_m[i] > 0.01f);
      }
      (void)Pole_AutoUpdate(&pole_auto, &pole, &pole_auto_cmd, &sensor, &pole_cmd,
                            osKernelGetTickCount());
    }
    Pole_Control(&pole, &pole_cmd, osKernelGetTickCount());
    Pole_Output(&pole);
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}
