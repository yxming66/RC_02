#include "task/user_task.h"

#include "module/chassis/front_omni_rear_mecanum.hpp"
#include "module/config.h"
#include "module/pole.h"

namespace {

mr::module::chassis::FrontOmniRearMecanumController chassis;
Chassis_CMD_t chassis_cmd{};

Pole_t pole;
static Pole_CMD_t pole_cmd{};

}  // namespace

extern "C" {
Chassis_IMU_t chassis_imu{};

bool Task_ChassisMainPoleGroupAtTarget(uint8_t group, float threshold_rad) {
  return Pole_IsGroupAtFinalTarget(&pole, group, threshold_rad);
}

bool Task_ChassisMainPoleAllAtTarget(float threshold_rad) {
  return Pole_IsAllAtFinalTarget(&pole, threshold_rad);
}
}

extern "C" void Task_chassis_main(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / CHASSIS_MAIN_FREQ;

  osDelay(CHASSIS_MAIN_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();

  Config_RobotParam_t *cfg = Config_GetRobotParam();
  if (cfg == nullptr ||
      chassis.Init(&cfg->chassis_param, static_cast<float>(CHASSIS_MAIN_FREQ)) !=
          CHASSIS_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }

  if (Pole_Init(&pole, &cfg->pole_param, static_cast<float>(CHASSIS_MAIN_FREQ)) !=
      POLE_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }

  while (1) {
    tick += delay_tick;

    osMessageQueueGet(task_runtime.msgq.chassis.imu, &chassis_imu, nullptr, 0);
    osMessageQueueGet(task_runtime.msgq.chassis.cmd, &chassis_cmd, nullptr, 0);

    chassis.SetGimbalYaw(chassis_imu.eulr.yaw, chassis_imu.gyro.z);
    (void)chassis.UpdateFeedback();
    (void)chassis.Control(chassis_cmd, osKernelGetTickCount());
    chassis.Output();

    osMessageQueueGet(task_runtime.msgq.pole.cmd, &pole_cmd, nullptr, 0);
    Pole_UpdateFeedback(&pole);
    Pole_Control(&pole, &pole_cmd, osKernelGetTickCount());
    Pole_Output(&pole);

    task_runtime.stack_water_mark.chassis_main =
        uxTaskGetStackHighWaterMark(nullptr);
    osDelayUntil(tick);
  }
}
