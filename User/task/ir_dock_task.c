/*
    ir_dock Task
  Dedicated task for UART8 infrared docking communication.
*/

#include "task/user_task.h"

volatile IrDock_Debug_t g_ir_dock_debug = {0};

void Task_ir_dock(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / IR_DOCK_FREQ;

  osDelay(IR_DOCK_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  IrDock_Init(BSP_TIME_Get_ms());

  while (1) {
    const uint32_t profile_start_us =
        Task_ProfilerLoopBegin(TASK_PROFILE_IR_DOCK,
                               TASK_PERIOD_US(IR_DOCK_FREQ));
    tick += delay_tick;

    const uint32_t now_ms = BSP_TIME_Get_ms();
    IrDock_Process(now_ms);

    task_runtime.heartbeat.ir_dock++;
    Task_ProfilerLoopEnd(TASK_PROFILE_IR_DOCK, profile_start_us);
    Task_DelayUntil(TASK_PROFILE_IR_DOCK, &tick, delay_tick);
  }
}
