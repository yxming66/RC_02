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
#include "module/arm/arm_control_types.h"
#include "module/arm_simple.h"
#include "module/rod_new.h"
#include "bsp/gpio.h"
/* USER INCLUDE END */

/* Exported functions ------------------------------------------------------- */
void Task_Init(void *argument) {
  (void)argument;

  osKernelLock();

  task_runtime.thread.blink = osThreadNew(Task_blink, NULL, &attr_blink);
  task_runtime.thread.atti_esti = osThreadNew(Task_atti_esti, NULL, &attr_atti_esti);
  task_runtime.thread.chassis_main = osThreadNew(Task_chassis_main, NULL, &attr_chassis_main);
  task_runtime.thread.rc_main = osThreadNew(Task_rc_main, NULL, &attr_rc_main);
  // task_runtime.thread.cmd_main =osThreadNew(Task_cmd_main, NULL, &attr_cmd_main);
  task_runtime.thread.sick = osThreadNew(Task_sick, NULL, &attr_sick);
  task_runtime.thread.auto_ctrl =osThreadNew(Task_auto_ctrl, NULL, &attr_auto_ctrl);
  // task_runtime.thread.arm = osThreadNew(Task_arm, NULL, &attr_arm);
    task_runtime.thread.arm_simple = osThreadNew(Task_arm_simple, NULL, &attr_arm_simple);
  task_runtime.thread.pc_comm = osThreadNew(Task_pc_comm, NULL, &attr_pc_comm);
    task_runtime.thread.rod = osThreadNew(Task_rod, NULL, &attr_rod);
    task_runtime.thread.ore_store = osThreadNew(Task_ore_store, NULL, &attr_ore_store);

  task_runtime.msgq.user_msg = osMessageQueueNew(2u, 10u, NULL);
  task_runtime.msgq.chassis.imu =
      osMessageQueueNew(1u, sizeof(Chassis_IMU_t), NULL);
  task_runtime.msgq.chassis.cmd =
      osMessageQueueNew(1u, sizeof(Chassis_CMD_t), NULL);
  task_runtime.msgq.pole.cmd =
      osMessageQueueNew(1u, sizeof(Pole_CMD_t), NULL);
  task_runtime.msgq.arm.cmd =
      osMessageQueueNew(1u, sizeof(Arm_CMD_t), NULL);
  task_runtime.msgq.arm_simple.cmd =
      osMessageQueueNew(1u, sizeof(ArmSimple_CMD_t), NULL);
  task_runtime.msgq.rod.cmd =
      osMessageQueueNew(1u, sizeof(RodNew_CMD_t), NULL);
  task_runtime.msgq.ore_store.cmd =
      osMessageQueueNew(1u, sizeof(OreStore_CMD_t), NULL);

  BUZZER_Init(&buzzer, BSP_PWM_BUZZER);

  BSP_GPIO_WritePin(BSP_GPIO_POWER_24V_1, 1);
  BSP_GPIO_WritePin(BSP_GPIO_POWER_24V_2, 1);
  BSP_GPIO_WritePin(BSP_GPIO_POWER_5V, 1);

  osKernelUnlock();
  osThreadTerminate(osThreadGetId());
}
