/*
    ir_dock Task
    Dedicated low-rate task for UART7 infrared docking communication.
*/

#include "task/user_task.h"

void Task_ir_dock(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / IR_DOCK_FREQ;

  osDelay(IR_DOCK_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  IrDock_Init(tick);

  while (1) {
    tick += delay_tick;

    const uint32_t now_ms = osKernelGetTickCount();
    IrDock_Process(now_ms);

    task_runtime.stack_water_mark.ir_dock = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick);
  }
}
