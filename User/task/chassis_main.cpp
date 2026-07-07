#include "module/mrlink_pc_comm/pc_messages.hpp"

#include "task/user_task.h"

#include "module/chassis/mecanum.hpp"
#include "module/config.h"
#include "module/mrlink_pc_comm/mrlink_pc_comm.h"

namespace {

mr::module::chassis::MecanumController chassis;
Chassis_CMD_t chassis_cmd{};
bool chassis_main_inited = false;

}  // namespace

extern "C" {
Chassis_IMU_t chassis_imu{};
float chassis_motor_speed_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};

const Chassis_Feedback_t *Task_ChassisGetFeedback(void) {
  return &chassis.feedback();
}

bool Task_ChassisMainInitOnce(void) {
  if (chassis_main_inited) {
    return true;
  }

  Config_RobotParam_t *cfg = Config_GetRobotParam();
  if (cfg == nullptr ||
      chassis.Init(&cfg->chassis_param, static_cast<float>(CHASSIS_MAIN_FREQ)) !=
          CHASSIS_OK) {
    return false;
  }

  chassis_main_inited = true;
  return true;
}

void Task_ChassisMainStep(void) {
  if (!chassis_main_inited) {
    return;
  }

  const uint32_t profile_start_us =
      Task_ProfilerLoopBegin(TASK_PROFILE_CHASSIS_MAIN,
                             TASK_PERIOD_US(CHASSIS_MAIN_FREQ));
  const uint32_t loop_tick = BSP_TIME_Get_ms();

  osMessageQueueGet(task_runtime.msgq.chassis.imu, &chassis_imu, nullptr, 0);
  (void)LatestSlot_ReadIfUpdated(
      &task_runtime.latest.chassis_cmd.slot, &chassis_cmd,
      sizeof(chassis_cmd), &task_runtime.latest.chassis_cmd.read_seq);

  chassis.SetGimbalYaw(chassis_imu.eulr.yaw);
  float pole_front_lift_rad = 0.0f;
  float pole_rear_lift_rad = 0.0f;
  (void)Task_PoleMainGetSupportLift(&pole_front_lift_rad, &pole_rear_lift_rad);
  chassis.SetPoleLift(pole_front_lift_rad, pole_rear_lift_rad);
  (void)chassis.UpdateFeedback();

  const auto &fb = chassis.feedback();
  for (int i = 0; i < 4; i++) {
    chassis_motor_speed_rpm[i] =
        fb.motor[i].velocity_rad_s * 60.0f / (2.0f * 3.14159265358979f);
  }

  (void)chassis.Control(chassis_cmd, loop_tick);
  chassis.Output();

  PC_ChassisFeedback_t chassis_fb = {0};
  chassis_fb.vx = fb.chassis_vel.vx;
  chassis_fb.vy = fb.chassis_vel.vy;
  chassis_fb.wz = fb.chassis_vel.wz;
  (void)MrlinkPc_Bus().StoreLatest(chassis_fb);

  task_runtime.heartbeat.chassis_main++;
  Task_ProfilerLoopEnd(TASK_PROFILE_CHASSIS_MAIN, profile_start_us);
}

void Task_chassis_main(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / CHASSIS_MAIN_FREQ;

  osDelay(CHASSIS_MAIN_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();

  if (!Task_ChassisMainInitOnce()) {
    osThreadTerminate(osThreadGetId());
    return;
  }

  while (1) {
    tick += delay_tick;
    Task_ChassisMainStep();
    Task_DelayUntil(TASK_PROFILE_CHASSIS_MAIN, &tick, delay_tick);
  }
}

void Task_chassis_ore(void *argument) {
  (void)argument;

  uint32_t base_delay_tick = osKernelGetTickFreq() / ORE_STORE_FREQ;
  if (base_delay_tick == 0U) {
    base_delay_tick = 1U;
  }
  uint32_t chassis_divider = (uint32_t)(ORE_STORE_FREQ / CHASSIS_MAIN_FREQ);
  if (chassis_divider == 0U) {
    chassis_divider = 1U;
  }

  osDelay(CHASSIS_MAIN_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  if (!Task_ChassisMainInitOnce()) {
    osThreadTerminate(osThreadGetId());
    return;
  }

  const uint32_t ore_store_init_tick = tick + ORE_STORE_INIT_DELAY;
  bool ore_store_init_done = false;
  bool ore_store_ready = false;
  uint32_t chassis_div = chassis_divider - 1U;

  while (1) {
    const uint32_t profile_start_us =
        Task_ProfilerLoopBegin(TASK_PROFILE_CHASSIS_ORE,
                               TASK_PERIOD_US(ORE_STORE_FREQ));
    tick += base_delay_tick;

    const uint32_t now_tick = osKernelGetTickCount();
    if (!ore_store_init_done &&
        (int32_t)(now_tick - ore_store_init_tick) >= 0) {
      ore_store_ready = Task_OreStoreInitOnce();
      ore_store_init_done = true;
    }

    if (ore_store_ready) {
      Task_OreStoreStep();
    }

    chassis_div++;
    if (chassis_div >= chassis_divider) {
      chassis_div = 0U;
      Task_ChassisMainStep();
    }

    task_runtime.heartbeat.chassis_ore++;
    Task_ProfilerLoopEnd(TASK_PROFILE_CHASSIS_ORE, profile_start_us);
    Task_DelayUntil(TASK_PROFILE_CHASSIS_ORE, &tick, base_delay_tick);
  }
}
}
