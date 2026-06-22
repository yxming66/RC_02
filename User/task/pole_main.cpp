#include "module/mrlink_pc_comm/pc_messages.hpp"

#include <cmath>
#include <cstring>

#include "task/user_task.h"

#include "module/config.h"
#include "module/mrlink_pc_comm/mrlink_pc_comm.h"
#include "module/pole.h"

#define POLE_TEMP_WARNING_ALARM_MS (3000u)
#define POLE_TEMP_OVER_LIMIT_ALARM_MS (5000u)

namespace {

Pole_t pole;
Pole_CMD_t pole_cmd{};
KPID_Params_t pole_default_support_pos_pid{};
KPID_Params_t pole_default_support_vel_pid{};
KPID_Params_t pole_last_applied_support_pos_pid{};
KPID_Params_t pole_last_applied_support_vel_pid{};
bool pole_pid_defaults_saved = false;

float PolePidDebugFiniteOrFallback(float value, float fallback) {
  return std::isfinite(value) ? value : fallback;
}

void PolePidDebugCopyToDebug(const KPID_Params_t &pos,
                             const KPID_Params_t &vel) {
  g_pole_pid_debug.pos_k = pos.k;
  g_pole_pid_debug.pos_p = pos.p;
  g_pole_pid_debug.pos_i = pos.i;
  g_pole_pid_debug.pos_d = pos.d;
  g_pole_pid_debug.pos_i_limit = pos.i_limit;
  g_pole_pid_debug.pos_out_limit = pos.out_limit;
  g_pole_pid_debug.pos_d_cutoff_freq = pos.d_cutoff_freq;
  g_pole_pid_debug.pos_range = pos.range;

  g_pole_pid_debug.vel_k = vel.k;
  g_pole_pid_debug.vel_p = vel.p;
  g_pole_pid_debug.vel_i = vel.i;
  g_pole_pid_debug.vel_d = vel.d;
  g_pole_pid_debug.vel_i_limit = vel.i_limit;
  g_pole_pid_debug.vel_out_limit = vel.out_limit;
  g_pole_pid_debug.vel_d_cutoff_freq = vel.d_cutoff_freq;
  g_pole_pid_debug.vel_range = vel.range;
}

void PolePidDebugCopyFromDebug(KPID_Params_t *pos, KPID_Params_t *vel) {
  if (pos == nullptr || vel == nullptr) {
    return;
  }

  pos->k = PolePidDebugFiniteOrFallback(g_pole_pid_debug.pos_k, pos->k);
  pos->p = PolePidDebugFiniteOrFallback(g_pole_pid_debug.pos_p, pos->p);
  pos->i = PolePidDebugFiniteOrFallback(g_pole_pid_debug.pos_i, pos->i);
  pos->d = PolePidDebugFiniteOrFallback(g_pole_pid_debug.pos_d, pos->d);
  pos->i_limit = PolePidDebugFiniteOrFallback(
      g_pole_pid_debug.pos_i_limit, pos->i_limit);
  pos->out_limit = PolePidDebugFiniteOrFallback(
      g_pole_pid_debug.pos_out_limit, pos->out_limit);
  pos->d_cutoff_freq = PolePidDebugFiniteOrFallback(
      g_pole_pid_debug.pos_d_cutoff_freq, pos->d_cutoff_freq);
  pos->range =
      PolePidDebugFiniteOrFallback(g_pole_pid_debug.pos_range, pos->range);

  vel->k = PolePidDebugFiniteOrFallback(g_pole_pid_debug.vel_k, vel->k);
  vel->p = PolePidDebugFiniteOrFallback(g_pole_pid_debug.vel_p, vel->p);
  vel->i = PolePidDebugFiniteOrFallback(g_pole_pid_debug.vel_i, vel->i);
  vel->d = PolePidDebugFiniteOrFallback(g_pole_pid_debug.vel_d, vel->d);
  vel->i_limit = PolePidDebugFiniteOrFallback(
      g_pole_pid_debug.vel_i_limit, vel->i_limit);
  vel->out_limit = PolePidDebugFiniteOrFallback(
      g_pole_pid_debug.vel_out_limit, vel->out_limit);
  vel->d_cutoff_freq = PolePidDebugFiniteOrFallback(
      g_pole_pid_debug.vel_d_cutoff_freq, vel->d_cutoff_freq);
  vel->range =
      PolePidDebugFiniteOrFallback(g_pole_pid_debug.vel_range, vel->range);
}

bool PolePidDebugPidChanged(const KPID_Params_t &pos,
                            const KPID_Params_t &vel) {
  return std::memcmp(&pos, &pole_last_applied_support_pos_pid, sizeof(pos)) !=
             0 ||
         std::memcmp(&vel, &pole_last_applied_support_vel_pid, sizeof(vel)) !=
             0;
}

void PolePidDebugReinitPolePids(const KPID_Params_t &pos,
                                const KPID_Params_t &vel) {
  for (uint8_t i = 0u; i < POLE_SUPPORT_MOTOR_NUM; i++) {
    (void)PID_Init(&pole.pid.support_pos[i], KPID_MODE_CALC_D,
                   static_cast<float>(POLE_MAIN_FREQ), &pole.param->pid.support_pos_pid);
    (void)PID_Init(&pole.pid.support_vel[i], KPID_MODE_NO_D,
                   static_cast<float>(POLE_MAIN_FREQ), &pole.param->pid.support_vel_pid);
  }
  pole_last_applied_support_pos_pid = pos;
  pole_last_applied_support_vel_pid = vel;
}

void PolePidDebugInit(Config_RobotParam_t *cfg) {
  if (cfg == nullptr) {
    return;
  }

  pole_default_support_pos_pid = cfg->pole_param.pid.support_pos_pid;
  pole_default_support_vel_pid = cfg->pole_param.pid.support_vel_pid;
  pole_last_applied_support_pos_pid = cfg->pole_param.pid.support_pos_pid;
  pole_last_applied_support_vel_pid = cfg->pole_param.pid.support_vel_pid;
  pole_pid_defaults_saved = true;
  PolePidDebugCopyToDebug(cfg->pole_param.pid.support_pos_pid,
                          cfg->pole_param.pid.support_vel_pid);
  g_pole_pid_debug.initialized = true;
}

void PolePidDebugUpdate(Config_RobotParam_t *cfg) {
  if (cfg == nullptr || !pole_pid_defaults_saved) {
    return;
  }

  if (g_pole_pid_debug.load_from_config_once) {
    PolePidDebugCopyToDebug(cfg->pole_param.pid.support_pos_pid,
                            cfg->pole_param.pid.support_vel_pid);
    g_pole_pid_debug.load_from_config_once = false;
    g_pole_pid_debug.load_count++;
  }

  if (g_pole_pid_debug.restore_defaults_once) {
    cfg->pole_param.pid.support_pos_pid = pole_default_support_pos_pid;
    cfg->pole_param.pid.support_vel_pid = pole_default_support_vel_pid;
    PolePidDebugCopyToDebug(cfg->pole_param.pid.support_pos_pid,
                            cfg->pole_param.pid.support_vel_pid);
    g_pole_pid_debug.restore_defaults_once = false;
    g_pole_pid_debug.restore_count++;
  } else if (g_pole_pid_debug.override_enable) {
    PolePidDebugCopyFromDebug(&cfg->pole_param.pid.support_pos_pid,
                              &cfg->pole_param.pid.support_vel_pid);
  }

  if (PolePidDebugPidChanged(cfg->pole_param.pid.support_pos_pid,
                             cfg->pole_param.pid.support_vel_pid)) {
    PolePidDebugReinitPolePids(cfg->pole_param.pid.support_pos_pid,
                               cfg->pole_param.pid.support_vel_pid);
    g_pole_pid_debug.apply_count++;
  }
}

}  // namespace

extern "C" volatile PolePidDebugControl_t g_pole_pid_debug = {0};

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
  PolePidDebugInit(cfg);

  while (1) {
    tick += delay_tick;

    osMessageQueueGet(task_runtime.msgq.pole.cmd, &pole_cmd, nullptr, 0);
    Pole_UpdateFeedback(&pole);

    const uint32_t now_ms = BSP_TIME_Get_ms();
    Task_PoleMainUpdateTemperatureAlarm(now_ms);
    PolePidDebugUpdate(cfg);
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
