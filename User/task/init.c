/*
    Init Task
    任务初始化，创建各个线程任务和消息队列
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"

/* USER INCLUDE BEGIN */
#include "device/buzzer.h"
#include "device/dr16.h"
#include "module/chassis.h"
#include "module/pole.h"
#include "module/arm.h"
#include "module/rod.h"
#include "bsp/gpio.h"
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
  task_runtime.thread.atti_esti = osThreadNew(Task_atti_esti, NULL, &attr_atti_esti);
  task_runtime.thread.chassis_main = osThreadNew(Task_chassis_main, NULL, &attr_chassis_main);
  task_runtime.thread.rc_main = osThreadNew(Task_rc_main, NULL, &attr_rc_main);
  task_runtime.thread.cmd_main = osThreadNew(Task_cmd_main, NULL, &attr_cmd_main);
  task_runtime.thread.sick = osThreadNew(Task_sick, NULL, &attr_sick);
  task_runtime.thread.auto_ctrl = osThreadNew(Task_auto_ctrl, NULL, &attr_auto_ctrl);
  task_runtime.thread.arm = osThreadNew(Task_arm, NULL, &attr_arm);
  // task_runtime.thread.pc_uart_rx = osThreadNew(Task_pc_uart_rx, NULL, &attr_pc_uart_rx);
  // task_runtime.thread.rod = osThreadNew(Task_rod, NULL, &attr_rod);
  // 创建消息队列
  /* USER MESSAGE BEGIN */
  task_runtime.msgq.user_msg= osMessageQueueNew(2u, 10, NULL);
  task_runtime.msgq.chassis.imu = osMessageQueueNew(1u, sizeof(Chassis_IMU_t), NULL);
  task_runtime.msgq.chassis.cmd = osMessageQueueNew(1u, sizeof(Chassis_CMD_t), NULL);
  task_runtime.msgq.pole.cmd = osMessageQueueNew(1u, sizeof(Pole_CMD_t), NULL);
  task_runtime.msgq.arm.cmd = osMessageQueueNew(1u, sizeof(Arm_CMD_t), NULL);
  task_runtime.msgq.rod.cmd = osMessageQueueNew(1u, sizeof(Rod_CMD_t), NULL);
  
  
  BUZZER_Init(&buzzer, BSP_PWM_BUZZER);
  
  BSP_GPIO_WritePin(BSP_GPIO_POWER_24V_1,1);
  BSP_GPIO_WritePin(BSP_GPIO_POWER_24V_1,1);
    BSP_GPIO_WritePin(BSP_GPIO_POWER_5V,1);
  /* USER MESSAGE END */

  osKernelUnlock(); // 解锁内核
  osThreadTerminate(osThreadGetId()); // 任务完成后结束自身
}