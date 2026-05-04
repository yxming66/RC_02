#include "cmsis_os2.h"
#include "task/user_task.h"

#include "module/arm/arm.hpp"

mr::arm::Runtime g_arm_runtime;

extern "C" {

mr::arm::Runtime* g_arm_runtime_ptr = &g_arm_runtime;
mr::arm::RuntimeDebugData* g_arm_debug = &g_arm_runtime.debug();
volatile uint8_t g_arm_gravity_only_torque_enable = 0U;

}
uint8_t setzero=0;

extern "C" {

void Task_arm(void* argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / ARM_FREQ;
  const float tick_period_s = 1.0f / static_cast<float>(osKernelGetTickFreq());
  osDelay(ARM_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  uint32_t last_tick = tick;

  mr::arm::SetDefaultRuntime(&g_arm_runtime);
  if (!g_arm_runtime.Init(static_cast<float>(ARM_FREQ))) {
    osThreadTerminate(osThreadGetId());
  }

  while (1) {
    tick += delay_tick;

    const uint32_t now_tick = osKernelGetTickCount();
    const float dt_s = (now_tick - last_tick) * tick_period_s;
    last_tick = now_tick;

    g_arm_runtime.Update();
    g_arm_runtime.PollCommand(task_runtime.msgq.arm.cmd);
    g_arm_runtime.Control(dt_s);
    g_arm_runtime.Commit();
    task_runtime.stack_water_mark.arm = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick);

    switch (setzero) {

      case 1:
         g_arm_runtime.SetMotorZero(0);
          setzero=0;
         break;
      case 2:
         g_arm_runtime.SetMotorZero(1);
         setzero=0;
          break;
      case 3:
         g_arm_runtime.SetMotorZero(2);
         setzero=0;

      break;
      case 4:
         g_arm_runtime.SetMotorZero(0);
          g_arm_runtime.SetMotorZero(1);
          g_arm_runtime.SetMotorZero(2);
         setzero=0;
          break;
      default:
      setzero=0;
        break;
    }
  }
}

}  // extern "C"
