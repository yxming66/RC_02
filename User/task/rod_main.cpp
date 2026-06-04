/*
 * Rod control task.
 *
 * 兼顾 rod_new 与 pole 两个子系统的控制节拍。
 * rod 任务以 ROD_FREQ (400Hz) 稳定运行，提供稳定的 dt，
 * 不再受底盘 1000Hz 节拍不齐的干扰。
 */

#include "task/user_task.h"

#include "module/config.h"
#include "module/mrlink_pc_comm/mrlink_pc_comm.h"
#include "module/pole.h"
#include "module/rod_new.h"
#include "module/shared_valve.h"

#define POLE_TEMP_WARNING_ALARM_MS (3000u)
#define POLE_TEMP_OVER_LIMIT_ALARM_MS (5000u)

namespace {

RodNew_t rod_new;
RodNew_CMD_t rod_new_cmd;
RodNew_Feedback_t rod_new_feedback;

Pole_t pole;
Pole_CMD_t pole_cmd{};

}  // namespace

volatile RodNew_DebugControl_t g_rod_new_debug = {
  false,
  false,
  BSP_PWM_ROD_SERVO,
  0.0f,
  ROD_NEW_SERVO_PULSE_NEUTRAL_US,
  ROD_NEW_GRIP_RELEASE,
};

extern "C" {

const RodNew_Feedback_t *Task_RodNewGetFeedback(void) {
  return &rod_new_feedback;
}

bool Task_RodPoleGroupAtTarget(uint8_t group, float threshold_rad) {
  return Pole_IsGroupAtFinalTarget(&pole, group, threshold_rad);
}

bool Task_RodPoleAllAtTarget(float threshold_rad) {
  return Pole_IsAllAtFinalTarget(&pole, threshold_rad);
}

bool Task_RodPoleGetHoldCommand(Pole_CMD_t *cmd) {
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

}  // extern "C"

static void Task_RodUpdatePoleTemperatureAlarm(uint32_t now_tick) {
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

extern "C" void Task_rod(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / ROD_FREQ;
  osDelay(ROD_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  uint32_t last_control_tick = tick;
  Config_RobotParam_t *cfg = Config_GetRobotParam();
  if (cfg == NULL ||
      RodNew_Init(&rod_new, &cfg->rod_new_param) != ROD_NEW_OK ||
      Pole_Init(&pole, &cfg->pole_param,
                static_cast<float>(ROD_FREQ)) != POLE_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }

  rod_new_cmd.mode = ROD_NEW_MODE_RELAX;
  rod_new_cmd.pose = ROD_NEW_POSE_STANDBY;
  rod_new_cmd.grip = ROD_NEW_GRIP_RELEASE;

  while (1) {
    tick += delay_tick;
    const uint32_t now = osKernelGetTickCount();

    /* === pole 子节拍（每个 rod 周期都跑） === */
    osMessageQueueGet(task_runtime.msgq.pole.cmd, &pole_cmd, nullptr, 0);
    Pole_UpdateFeedback(&pole);
    Task_RodUpdatePoleTemperatureAlarm(now);
    Pole_Control(&pole, &pole_cmd, now);
    Pole_Output(&pole);

    PC_PoleFeedback_t pole_fb = {0};
    pole_fb.lift[0] = pole.feedback.support_angle_avg;
    pole_fb.lift[1] = pole.feedback.support_angle_avg;
    for (uint8_t i = 0u; i < POLE_MOTOR_NUM; i++) {
      pole_fb.motor_total_angle[i] = pole.feedback.motor[i].rotor_total_angle;
    }
    (void)MrlinkPc_Bus().StoreLatest(pole_fb);

    /* === rod_new 主节拍 === */
    osMessageQueueGet(task_runtime.msgq.rod.cmd, &rod_new_cmd, NULL, 0);

    if (!g_rod_new_debug.enable) {
      RodNew_Control(&rod_new, rod_new_cmd.mode, rod_new_cmd.pose,
                     rod_new_cmd.grip, rod_new_cmd.target_angle_rad,
                     last_control_tick);
      last_control_tick = tick;
    }
    rod_new_feedback.mode = rod_new.mode;
    rod_new_feedback.pose = rod_new_cmd.pose;
    rod_new_feedback.grip = rod_new.gripper.state;
    rod_new_feedback.target_angle_rad = rod_new.servo.target_angle_rad;
    rod_new_feedback.tracked_angle_rad = rod_new.servo.tracked_angle_rad;
    rod_new_feedback.tracked_velocity_rad_s = rod_new.servo.tracked_vel_rad_s;
    rod_new_feedback.feedback_angle_rad = rod_new.servo.feedback_angle_rad;
    rod_new_feedback.at_target = rod_new.servo.at_target;
    RodNew_Output(&rod_new);
    SharedValve_Output();

    task_runtime.stack_water_mark.rod = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick);
  }
}
