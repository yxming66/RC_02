#include "task/user_task.h"

#include "module/chassis/mecanum.hpp"
#include "module/config.h"
#include "module/pole.h"

#define POLE_TEMP_WARNING_ALARM_MS (3000u)
#define POLE_TEMP_OVER_LIMIT_ALARM_MS (5000u)

namespace {

mr::module::chassis::MecanumController chassis;
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

    chassis.SetGimbalYaw(chassis_imu.eulr.yaw);
    (void)chassis.UpdateFeedback();
    (void)chassis.Control(chassis_cmd, osKernelGetTickCount());
    chassis.Output();

    osMessageQueueGet(task_runtime.msgq.pole.cmd, &pole_cmd, nullptr, 0);
    Pole_UpdateFeedback(&pole);
  Task_ChassisMainUpdatePoleTemperatureAlarm(osKernelGetTickCount());
    Pole_Control(&pole, &pole_cmd, osKernelGetTickCount());
    Pole_Output(&pole);

    task_runtime.stack_water_mark.chassis_main =
        uxTaskGetStackHighWaterMark(nullptr);
    osDelayUntil(tick);
  }
}
