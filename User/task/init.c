/*
    Init Task
    任务初始化，创建各个线程任务和消息队列
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"

/* USER INCLUDE BEGIN */
#include "device/dr16.h"
#include "module/chassis.h"
#include "module/pole.h"
#include "module/arm.h"
#include "module/rod.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */

/**
 * \brief 初始化
 *
 * \param argument 未使用
 */
void Task_Init(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */
    /* USER CODE INIT BEGIN */

    /* USER CODE INIT END */
  osKernelLock(); /* 锁定内核，防止任务切换 */
  
  /* 创建任务线程 */
  task_runtime.thread.blink = osThreadNew(Task_blink, NULL, &attr_blink);
  task_runtime.thread.chassis_main = osThreadNew(Task_chassis_main, NULL, &attr_chassis_main);
  task_runtime.thread.rc_main = osThreadNew(Task_rc_main, NULL, &attr_rc_main);
  task_runtime.thread.cmd_main = osThreadNew(Task_cmd_main, NULL, &attr_cmd_main);
  task_runtime.thread.sick = osThreadNew(Task_sick, NULL, &attr_sick);
  task_runtime.thread.arm = osThreadNew(Task_arm, NULL, &attr_arm);
  task_runtime.thread.rod = osThreadNew(Task_rod, NULL, &attr_rod);
  // 创建消息队列
  /* USER MESSAGE BEGIN */
  task_runtime.msgq.user_msg= osMessageQueueNew(2u, 10, NULL);
  task_runtime.msgq.chassis.cmd = osMessageQueueNew(1u, sizeof(Chassis_CMD_t), NULL);
  task_runtime.msgq.pole.cmd = osMessageQueueNew(1u, sizeof(Pole_CMD_t), NULL);
  task_runtime.msgq.arm.cmd = osMessageQueueNew(1u, sizeof(Arm_CMD_t), NULL);
  task_runtime.msgq.rod.cmd = osMessageQueueNew(1u, sizeof(Rod_CMD_t), NULL);
  /* USER MESSAGE END */

  osKernelUnlock(); // 解锁内核
  osThreadTerminate(osThreadGetId()); // 任务完成后结束自身
}