#include "module/mrlink_pc_comm/pc_messages.hpp"

#include "task/user_task.h"

#include "module/config.h"
#include "module/mrlink_pc_comm/mrlink_pc_comm.h"
#include "module/pole.h"

#define POLE_TEMP_WARNING_ALARM_MS (3000u)
#define POLE_TEMP_OVER_LIMIT_ALARM_MS (5000u)

namespace {

Pole_t pole;
Pole_CMD_t pole_cmd{};

}  // namespace

extern "C" {

bool Task_PoleMainGroupAtTarget(uint8_t group, float threshold_rad) {
  return Pole_IsGroupAtFinalTarget(&pole, group, threshold_rad);
}

bool Task_PoleMainAllAtTarget(float threshold_rad) {
  return Pole_IsAllAtFinalTarget(&pole, threshold_rad);
}

bool Task_PoleMainGetSupportLift(float *front_lift_rad, float *rear_lift_rad) {
  if (front_lift_rad == nullptr || rear_lift_rad == nullptr) {
    return false;
  }

  *front_lift_rad = pole.feedback.support_lift[0];
  *rear_lift_rad = pole.feedback.support_lift[1];
  return true;
}

bool Task_PoleMainGetHoldCommand(Pole_CMD_t *cmd) {
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
  cmd->auto_lift_accel[0] = 0.0f;
  cmd->auto_lift_accel[1] = 0.0f;
  return true;
}

}

static void Task_PoleMainUpdateTemperatureAlarm(uint32_t now_tick) {
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

extern "C" void Task_pole_main(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / POLE_MAIN_FREQ;

  osDelay(POLE_MAIN_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();

  Config_RobotParam_t *cfg = Config_GetRobotParam();
  if (cfg == nullptr ||
      Pole_Init(&pole, &cfg->pole_param, static_cast<float>(POLE_MAIN_FREQ)) !=
          POLE_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }

  while (1) {
    tick += delay_tick;

    osMessageQueueGet(task_runtime.msgq.pole.cmd, &pole_cmd, nullptr, 0);
    Pole_UpdateFeedback(&pole);

    const uint32_t now_ms = BSP_TIME_Get_ms();
    Task_PoleMainUpdateTemperatureAlarm(now_ms);
    Pole_Control(&pole, &pole_cmd, now_ms);
    Pole_Output(&pole);

    PC_PoleFeedback_t pole_fb = {0};
    pole_fb.lift[0] = pole.feedback.support_lift[0];
    pole_fb.lift[1] = pole.feedback.support_lift[1];
    for (uint8_t i = 0u; i < POLE_MOTOR_NUM; i++) {
      pole_fb.motor_total_angle[i] = pole.feedback.motor[i].rotor_total_angle;
    }
    (void)MrlinkPc_Bus().StoreLatest(pole_fb);

    task_runtime.stack_water_mark.pole_main =
        uxTaskGetStackHighWaterMark(nullptr);
    task_runtime.heartbeat.pole_main++;
    osDelayUntil(tick);
  }
}
