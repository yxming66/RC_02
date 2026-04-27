#include "cmsis_os2.h"
#include "task/user_task.h"

#include "module/arm/arm.hpp"

extern "C" {

void Task_arm(void* argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / ARM_FREQ;
  const float tick_period_s = 1.0f / static_cast<float>(osKernelGetTickFreq());
  osDelay(ARM_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  uint32_t last_tick = tick;

  if (!mrobot::arm::Init(static_cast<float>(ARM_FREQ))) {
    osThreadTerminate(osThreadGetId());
  }

  while (1) {
    tick += delay_tick;

    const uint32_t now_tick = osKernelGetTickCount();
    const float dt_s = (now_tick - last_tick) * tick_period_s;
    last_tick = now_tick;

    mrobot::arm::Update();
    mrobot::arm::PollCommand(task_runtime.msgq.arm.cmd);
    mrobot::arm::Control(dt_s);
    osDelayUntil(tick);
  }
}

}  // extern "C"
