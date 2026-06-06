#include "module/mrlink_pc_comm/pc_messages.hpp"

#include "task/user_task.h"

#include "module/chassis/mecanum.hpp"
#include "module/config.h"
#include "module/pole.h"
#include "module/mrlink_pc_comm/mrlink_pc_comm.h"

#define POLE_TEMP_WARNING_ALARM_MS (3000u)
#define POLE_TEMP_OVER_LIMIT_ALARM_MS (5000u)
#define CHASSIS_POLE_UPDATE_DIVIDER (2u)

namespace {

mr::module::chassis::MecanumController chassis;
Chassis_CMD_t chassis_cmd{};

Pole_t pole;
static Pole_CMD_t pole_cmd{};

}  // namespace

extern "C" {
Chassis_IMU_t chassis_imu{};
float chassis_motor_speed_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};

bool Task_ChassisMainPoleGroupAtTarget(uint8_t group, float threshold_rad) {
  return Pole_IsGroupAtFinalTarget(&pole, group, threshold_rad);
}

bool Task_ChassisMainPoleAllAtTarget(float threshold_rad) {
  return Pole_IsAllAtFinalTarget(&pole, threshold_rad);
}

bool Task_ChassisMainGetPoleHoldCommand(Pole_CMD_t *cmd) {
  if (cmd == nullptr || pole.param == nullptr) {
    return false;
  }

  *cmd = {};
  cmd->mode = POLE_MODE_ACTIVE;
  cmd->auto_target_enable[0] = true;
  cmd->auto_target_enable[1] = true;
  cmd->auto_target_lift[0] = pole.debug.tracked_target_lift[0];
  cmd->auto_target_lift[1] = pole.debug.tracked_target_lift[1];
  cmd->auto_lift_speed[0] = 0.0f;
  cmd->auto_lift_speed[1] = 0.0f;
  return true;
}
}

static void Task_ChassisMainUpdatePoleTemperatureAlarm(uint32_t now_tick) {
  const bool has_over_limit = Pole_HasTemperatureOverLimit(&pole);
  const bool has_warning = Pole_HasTemperatureWarning(&pole);

  for (uint8_t i = 0u; i < POLE_MOTOR_NUM; i++) {
    task_runtime.status.pole_alarm.motor_temperature_warning[i] =
        pole.feedback.temperature_warning[i];
    task_runtime.status.pole_alarm.motor_temperature_over_limit[i] =
        pole.feedback.temperature_over_limit[i];
  }
  task_runtime.status.pole_alarm.max_temperature_c =
      Pole_GetMaxTemperature(&pole);

  if (has_over_limit) {
    task_runtime.status.pole_alarm.level = BUZZER_ALARM_TEMP_OVER_LIMIT;
    g_buzzer_alarm_request.level = BUZZER_ALARM_TEMP_OVER_LIMIT;
    g_buzzer_alarm_request.min_duration_ms = POLE_TEMP_OVER_LIMIT_ALARM_MS;
    g_buzzer_alarm_request.request_tick = now_tick;
  } else if (has_warning) {
    task_runtime.status.pole_alarm.level = BUZZER_ALARM_TEMP_WARNING;
    g_buzzer_alarm_request.level = BUZZER_ALARM_TEMP_WARNING;
    g_buzzer_alarm_request.min_duration_ms = POLE_TEMP_WARNING_ALARM_MS;
    g_buzzer_alarm_request.request_tick = now_tick;
  } else {
    task_runtime.status.pole_alarm.level = BUZZER_ALARM_NONE;
  }
}

extern "C" void Task_chassis_main(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / CHASSIS_MAIN_FREQ;

  osDelay(CHASSIS_MAIN_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  uint32_t pole_update_phase = 0u;

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
    const uint32_t loop_tick = osKernelGetTickCount();

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

    (void)chassis.Control(chassis_cmd, loop_tick);
    chassis.Output();

    const bool run_pole_update = (pole_update_phase == 0u);
    pole_update_phase++;
    if (pole_update_phase >= CHASSIS_POLE_UPDATE_DIVIDER) {
      pole_update_phase = 0u;
    }

    if (run_pole_update) {
      osMessageQueueGet(task_runtime.msgq.pole.cmd, &pole_cmd, nullptr, 0);
      Pole_UpdateFeedback(&pole);
      Task_ChassisMainUpdatePoleTemperatureAlarm(osKernelGetTickCount());
      Pole_Control(&pole, &pole_cmd, osKernelGetTickCount());
      Pole_Output(&pole);
    }

    PC_ChassisFeedback_t chassis_fb = {0};
    chassis_fb.vx = fb.chassis_vel.vx;
    chassis_fb.vy = fb.chassis_vel.vy;
    chassis_fb.wz = fb.chassis_vel.wz;
    (void)MrlinkPc_Bus().StoreLatest(chassis_fb);

    if (run_pole_update) {
      PC_PoleFeedback_t pole_fb = {0};
      pole_fb.lift[0] = pole.feedback.support_angle_avg;
      pole_fb.lift[1] = pole.feedback.support_angle_avg;
      for (uint8_t i = 0u; i < POLE_MOTOR_NUM; i++) {
        pole_fb.motor_total_angle[i] = pole.feedback.motor[i].rotor_total_angle;
      }
      (void)MrlinkPc_Bus().StoreLatest(pole_fb);
    }

    task_runtime.stack_water_mark.chassis_main =
        uxTaskGetStackHighWaterMark(nullptr);
    osDelayUntil(tick);
  }
}
