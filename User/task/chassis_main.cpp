#include "module/mrlink_pc_comm/pc_messages.hpp"

#include "task/user_task.h"

#include "module/chassis/mecanum.hpp"
#include "module/config.h"
#include "module/mrlink_pc_comm/mrlink_pc_comm.h"

namespace {

mr::module::chassis::MecanumController chassis;
Chassis_CMD_t chassis_cmd{};

}  // namespace

extern "C" {
Chassis_IMU_t chassis_imu{};
float chassis_motor_speed_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
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

  while (1) {
    tick += delay_tick;

    osMessageQueueGet(task_runtime.msgq.chassis.imu, &chassis_imu, nullptr, 0);
    osMessageQueueGet(task_runtime.msgq.chassis.cmd, &chassis_cmd, nullptr, 0);

    chassis.SetGimbalYaw(chassis_imu.eulr.yaw);
    (void)chassis.UpdateFeedback();

    /* Update motor speed for gyro calibration in atti_esti task */
    const auto &fb = chassis.feedback();
    for (int i = 0; i < 4; i++) {
      chassis_motor_speed_rpm[i] =
          fb.motor[i].velocity_rad_s * 60.0f / (2.0f * 3.14159265358979f);
    }

    (void)chassis.Control(chassis_cmd, osKernelGetTickCount());
    chassis.Output();

    PC_ChassisFeedback_t chassis_fb = {0};
    chassis_fb.vx = fb.chassis_vel.vx;
    chassis_fb.vy = fb.chassis_vel.vy;
    chassis_fb.wz = fb.chassis_vel.wz;
    (void)MrlinkPc_Bus().StoreLatest(chassis_fb);

    task_runtime.stack_water_mark.chassis_main =
        uxTaskGetStackHighWaterMark(nullptr);
    osDelayUntil(tick);
  }
}
