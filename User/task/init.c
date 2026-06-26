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
#include "module/arm_simple.h"
#include "module/rod_new.h"
#include "module/camera_yaw.h"
#include "module/shared_valve.h"
#include "bsp/gpio.h"
/* USER INCLUDE END */

/* Exported functions ------------------------------------------------------- */
void Task_Init(void *argument) {
  (void)argument;

  task_runtime.heartbeat.init++;

  BSP_GPIO_WritePin(BSP_GPIO_POWER_24V_1, 1);
  BSP_GPIO_WritePin(BSP_GPIO_POWER_24V_2, 1);
  BSP_GPIO_WritePin(BSP_GPIO_POWER_5V, 1);

  osKernelLock();

  task_runtime.msgq.chassis.imu =
      osMessageQueueNew(1u, sizeof(Chassis_IMU_t), NULL);
  task_runtime.msgq.chassis.cmd =
      osMessageQueueNew(1u, sizeof(Chassis_CMD_t), NULL);
  task_runtime.msgq.pole.cmd =
      osMessageQueueNew(1u, sizeof(Pole_CMD_t), NULL);
  task_runtime.msgq.arm_simple.cmd =
      osMessageQueueNew(1u, sizeof(ArmSimple_CMD_t), NULL);
  task_runtime.msgq.rod.cmd =
      osMessageQueueNew(1u, sizeof(RodNew_CMD_t), NULL);
  task_runtime.msgq.camera_yaw.cmd =
      osMessageQueueNew(1u, sizeof(CameraYaw_GroupCMD_t), NULL);
  task_runtime.msgq.ore_store.cmd =
      osMessageQueueNew(1u, sizeof(OreStore_CMD_t), NULL);

    if (task_runtime.msgq.chassis.imu == NULL ||
            task_runtime.msgq.chassis.cmd == NULL ||
            task_runtime.msgq.pole.cmd == NULL ||
            task_runtime.msgq.arm_simple.cmd == NULL ||
            task_runtime.msgq.rod.cmd == NULL ||
            task_runtime.msgq.camera_yaw.cmd == NULL ||
            task_runtime.msgq.ore_store.cmd == NULL) {
        osKernelUnlock();
        osThreadTerminate(osThreadGetId());
        return;
    }

  BUZZER_Init(&buzzer, BSP_PWM_BUZZER);

    SharedValve_Init(BSP_GPIO_SPEARHEAD_RELAY);

  task_runtime.heartbeat.init++;

  task_runtime.thread.blink = 
      osThreadNew(Task_blink, NULL, &attr_blink);
  task_runtime.thread.atti_esti =
      osThreadNew(Task_atti_esti, NULL, &attr_atti_esti);
  task_runtime.thread.chassis_ore =
      osThreadNew(Task_chassis_ore, NULL, &attr_chassis_ore);
  task_runtime.thread.upper_mech =
      osThreadNew(Task_upper_mech, NULL, &attr_upper_mech);
  task_runtime.thread.pole_main =
      osThreadNew(Task_pole_main, NULL, &attr_pole_main);
  task_runtime.thread.rc_main = 
      osThreadNew(Task_rc_main, NULL, &attr_rc_main);
  task_runtime.thread.auto_ctrl =
      osThreadNew(Task_auto_ctrl, NULL, &attr_auto_ctrl);
  task_runtime.thread.pc_comm_sick =
      osThreadNew(Task_pc_comm_sick, NULL, &attr_pc_comm_sick);
  task_runtime.thread.ir_dock = osThreadNew(Task_ir_dock, NULL, &attr_ir_dock);

  task_runtime.heartbeat.init++;





  osKernelUnlock();
  osThreadTerminate(osThreadGetId());
}
