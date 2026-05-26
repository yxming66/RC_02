/*
 * Ore store task.
 */

#include "task/user_task.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "module/config.h"
#include "module/ore_store.h"

#define ORE_STORE_POWER_ON_HOME_EXTRA_TIMEOUT_S (0.5f)
#define ORE_STORE_POWER_ON_HOME_MIN_HOLD_CYCLES (100u)

typedef enum {
  ORE_STORE_POWER_ON_HOME_EXIT_NONE = 0,
  ORE_STORE_POWER_ON_HOME_EXIT_SUCCESS = 1,
  ORE_STORE_POWER_ON_HOME_EXIT_TIMEOUT = 2,
  ORE_STORE_POWER_ON_HOME_EXIT_FAILURE_AFTER_START = 3,
} OreStoreTask_PowerOnHomeExitReason_t;

static OreStore_t ore_store;
static OreStore_CMD_t ore_store_cmd;
static volatile bool ore_store_power_on_home_active = true;
static volatile bool ore_store_power_on_home_attempt_done = false;
static volatile bool ore_store_power_on_home_failed = false;
static float ore_store_power_on_home_elapsed_s = 0.0f;
static float ore_store_power_on_home_timeout_s = 0.0f;
static uint32_t ore_store_power_on_home_active_cycles = 0u;
static volatile uint8_t ore_store_power_on_home_exit_reason =
  ORE_STORE_POWER_ON_HOME_EXIT_NONE;
static volatile OreStore_Mode_t ore_store_debug_last_received_cmd_mode =
  ORE_STORE_MODE_RELAX;
static volatile OreStore_Mode_t ore_store_debug_pre_control_cmd_mode =
  ORE_STORE_MODE_RELAX;
static volatile OreStore_Mode_t ore_store_debug_post_control_mode =
  ORE_STORE_MODE_RELAX;
static volatile uint32_t ore_store_debug_home_override_fault_count = 0u;
static volatile uint32_t ore_store_debug_home_finish_count = 0u;
static volatile bool ore_store_inited = false;
static volatile bool ore_store_rehome_requested = false;
volatile int8_t g_ore_store_init_ret = ORE_STORE_ERR_NULL;
volatile int8_t g_ore_store_update_ret = ORE_STORE_ERR_NULL;
volatile int8_t g_ore_store_control_ret = ORE_STORE_ERR_NULL;

typedef struct {
  volatile bool inited;
  volatile bool power_on_home_active;
  volatile bool power_on_home_attempt_done;
  volatile bool power_on_home_failed;
  volatile float power_on_home_elapsed_s;
  volatile float power_on_home_timeout_s;
  volatile uint32_t power_on_home_active_cycles;
  volatile uint8_t power_on_home_exit_reason;
  volatile OreStore_Mode_t last_received_cmd_mode;
  volatile OreStore_Mode_t pre_control_cmd_mode;
  volatile OreStore_Mode_t post_control_mode;
  volatile uint32_t home_override_fault_count;
  volatile uint32_t home_finish_count;
  volatile OreStore_Mode_t mode;
  volatile bool platform_online;
  volatile bool platform_homed;
  volatile bool all_homed;
  volatile float platform_position_rad;
  volatile float platform_filtered_position_rad;
  volatile float platform_filtered_velocity_rad_s;
  volatile float platform_target_rad;
  volatile float platform_command_rad;
  volatile int8_t platform_update_ret;
  volatile int8_t platform_set_ret;
  volatile int8_t platform_commit_ret;
  volatile bool platform_command_pending;
  volatile uint8_t platform_soft_limit_state;
  volatile float platform_feedback_torque_nm;
  volatile float platform_filtered_output_torque_nm;
  volatile float platform_velocity_setpoint_rad_s;
  volatile float platform_cmd_torque_nm;
  volatile float platform_cmd_current_a;
  volatile int16_t platform_rm_output_raw;
  volatile uint16_t platform_rm_tx_frame_id;
  volatile bool track_left_online;
  volatile bool track_left_homed;
  volatile float track_left_position_rad;
  volatile float track_left_target_rad;
  volatile float track_left_command_rad;
  volatile int16_t track_left_rm_output_raw;
  volatile bool track_right_online;
  volatile bool track_right_homed;
  volatile float track_right_position_rad;
  volatile float track_right_target_rad;
  volatile float track_right_command_rad;
  volatile int16_t track_right_rm_output_raw;
} OreStoreTask_DebugView_t;

volatile OreStoreTask_DebugView_t g_ore_store_debug_view = {0};

volatile OreStore_DebugCommand_t g_ore_store_debug_command = {
    .enable = false,
    .mode = ORE_STORE_MODE_RELAX,
  .direct_set_ret = ORE_STORE_ERR_NULL,
  .direct_ctrl_ret = ORE_STORE_ERR_NULL,
  .assume_homed_ret = ORE_STORE_ERR_NULL,
};

#define ORE_STORE_TEMP_WARNING_ALARM_MS (3000u)
#define ORE_STORE_TEMP_OVER_LIMIT_ALARM_MS (5000u)

static void OreStoreTask_SetDefaultCommand(OreStore_CMD_t *cmd) {
  if (cmd == NULL) {
    return;
  }

  memset(cmd, 0, sizeof(*cmd));
  cmd->mode = ORE_STORE_MODE_HOME;
}

static bool OreStoreTask_ModeIsValid(OreStore_Mode_t mode) {
  return mode == ORE_STORE_MODE_RELAX || mode == ORE_STORE_MODE_HOME ||
         mode == ORE_STORE_MODE_ACTIVE;
}

static bool OreStoreTask_TargetsAreFinite(const OreStore_CMD_t *cmd) {
  if (cmd == NULL) {
    return false;
  }

  if (!isfinite(cmd->platform_target_rad)) {
    return false;
  }

  for (uint8_t i = 0; i < ORE_STORE_GATE_NUM; ++i) {
    if (!isfinite(cmd->gate_target_rad[i])) {
      return false;
    }
  }

  for (uint8_t i = 0; i < ORE_STORE_TRACK_NUM; ++i) {
    if (!isfinite(cmd->track_target_rad[i])) {
      return false;
    }
  }

  return true;
}

static void OreStoreTask_SanitizeCommand(OreStore_CMD_t *cmd) {
  if (cmd == NULL) {
    return;
  }

  if (!OreStoreTask_ModeIsValid(cmd->mode) ||
      (cmd->mode == ORE_STORE_MODE_ACTIVE &&
       !OreStoreTask_TargetsAreFinite(cmd))) {
    OreStoreTask_SetDefaultCommand(cmd);
  }
}

static bool OreStoreTask_TryApplyDebugCommand(OreStore_CMD_t *cmd) {
  if (cmd == NULL || !g_ore_store_debug_command.enable) {
    return false;
  }

  cmd->mode = g_ore_store_debug_command.mode;
  cmd->force_rehome = g_ore_store_debug_command.force_rehome;
  cmd->platform_target_rad = g_ore_store_debug_command.platform_target_rad;
  for (uint8_t i = 0; i < ORE_STORE_GATE_NUM; ++i) {
    cmd->gate_target_rad[i] = g_ore_store_debug_command.gate_target_rad[i];
  }
  for (uint8_t i = 0; i < ORE_STORE_TRACK_NUM; ++i) {
    cmd->track_target_rad[i] = g_ore_store_debug_command.track_target_rad[i];
  }

  OreStoreTask_SanitizeCommand(cmd);
  ++g_ore_store_debug_command.applied_count;
  return true;
}

static float OreStoreTask_ClampDirectOutput(float value) {
  if (!isfinite(value)) {
    return 0.0f;
  }
  if (value > 0.2f) {
    return 0.2f;
  }
  if (value < -0.2f) {
    return -0.2f;
  }
  return value;
}

static void OreStoreTask_ApplyDirectDebugOutput(void) {
  if (!g_ore_store_debug_command.direct_output_enable) {
    g_ore_store_debug_command.direct_set_ret = ORE_STORE_ERR_NULL;
    g_ore_store_debug_command.direct_ctrl_ret = ORE_STORE_ERR_NULL;
    return;
  }

  if (ore_store.param == NULL ||
      g_ore_store_debug_command.direct_output_axis >= ORE_STORE_AXIS_NUM) {
    g_ore_store_debug_command.direct_set_ret = ORE_STORE_ERR_NULL;
    g_ore_store_debug_command.direct_ctrl_ret = ORE_STORE_ERR_NULL;
    return;
  }

  MOTOR_RM_Param_t *motor_param =
      (MOTOR_RM_Param_t *)&ore_store.param
           ->motor_param[g_ore_store_debug_command.direct_output_axis];
  const float output =
      OreStoreTask_ClampDirectOutput(g_ore_store_debug_command.direct_output);
  g_ore_store_debug_command.direct_set_ret =
      MOTOR_RM_SetOutput(motor_param, output);
  g_ore_store_debug_command.direct_ctrl_ret = MOTOR_RM_Ctrl(motor_param);
}

static void OreStoreTask_ApplyAssumeHomedDebug(void) {
  if (!g_ore_store_debug_command.assume_homed_trigger) {
    return;
  }

  g_ore_store_debug_command.assume_homed_ret =
      OreStore_AssumeAxisHomedAtCurrent(
          &ore_store, g_ore_store_debug_command.assume_homed_axis,
          g_ore_store_debug_command.assume_homed_position_rad);
  g_ore_store_debug_command.assume_homed_trigger = false;
}

static void OreStoreTask_UpdateDebugView(void) {
  g_ore_store_debug_view.inited = ore_store_inited;
  g_ore_store_debug_view.power_on_home_active = ore_store_power_on_home_active;
  g_ore_store_debug_view.power_on_home_attempt_done =
    ore_store_power_on_home_attempt_done;
  g_ore_store_debug_view.power_on_home_failed = ore_store_power_on_home_failed;
  g_ore_store_debug_view.power_on_home_elapsed_s =
    ore_store_power_on_home_elapsed_s;
  g_ore_store_debug_view.power_on_home_timeout_s =
    ore_store_power_on_home_timeout_s;
  g_ore_store_debug_view.power_on_home_active_cycles =
      ore_store_power_on_home_active_cycles;
    g_ore_store_debug_view.power_on_home_exit_reason =
      ore_store_power_on_home_exit_reason;
    g_ore_store_debug_view.last_received_cmd_mode =
      ore_store_debug_last_received_cmd_mode;
    g_ore_store_debug_view.pre_control_cmd_mode =
      ore_store_debug_pre_control_cmd_mode;
    g_ore_store_debug_view.post_control_mode =
      ore_store_debug_post_control_mode;
    g_ore_store_debug_view.home_override_fault_count =
      ore_store_debug_home_override_fault_count;
    g_ore_store_debug_view.home_finish_count = ore_store_debug_home_finish_count;
  g_ore_store_debug_view.mode = ore_store.mode;
  g_ore_store_debug_view.platform_online =
      ore_store.feedback.online[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_homed =
      ore_store.feedback.homed[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.all_homed = ore_store.feedback.all_homed;
  g_ore_store_debug_view.platform_position_rad =
      ore_store.feedback.position_rad[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_filtered_position_rad =
      ore_store.debug.filtered_position_rad[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_filtered_velocity_rad_s =
      ore_store.debug.filtered_velocity_rad_s[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_target_rad =
      ore_store.debug.target_position_rad[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_command_rad =
      ore_store.debug.command_position_rad[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_update_ret =
      ore_store.debug.controller_update_ret[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_set_ret =
      ore_store.debug.set_command_ret[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_commit_ret =
      ore_store.debug.commit_ret[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_command_pending =
      ore_store.debug.command_pending[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_soft_limit_state =
      ore_store.debug.soft_limit_state[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_feedback_torque_nm =
      ore_store.debug.motor_torque_nm[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_filtered_output_torque_nm =
      ore_store.debug.filtered_output_torque_nm[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_velocity_setpoint_rad_s =
      ore_store.debug.velocity_setpoint_rad_s[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_cmd_torque_nm =
      ore_store.debug.rm_last_set_torque_nm[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_cmd_current_a =
      ore_store.debug.rm_pending_current_a[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_rm_output_raw =
      ore_store.debug.rm_output_raw[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_rm_tx_frame_id =
      ore_store.debug.rm_tx_frame_id[ORE_STORE_AXIS_PLATFORM];
    g_ore_store_debug_view.track_left_online =
      ore_store.feedback.online[ORE_STORE_AXIS_TRACK_LEFT];
    g_ore_store_debug_view.track_left_homed =
      ore_store.feedback.homed[ORE_STORE_AXIS_TRACK_LEFT];
    g_ore_store_debug_view.track_left_position_rad =
      ore_store.feedback.position_rad[ORE_STORE_AXIS_TRACK_LEFT];
    g_ore_store_debug_view.track_left_target_rad =
      ore_store.debug.target_position_rad[ORE_STORE_AXIS_TRACK_LEFT];
    g_ore_store_debug_view.track_left_command_rad =
      ore_store.debug.command_position_rad[ORE_STORE_AXIS_TRACK_LEFT];
    g_ore_store_debug_view.track_left_rm_output_raw =
      ore_store.debug.rm_output_raw[ORE_STORE_AXIS_TRACK_LEFT];
    g_ore_store_debug_view.track_right_online =
      ore_store.feedback.online[ORE_STORE_AXIS_TRACK_RIGHT];
    g_ore_store_debug_view.track_right_homed =
      ore_store.feedback.homed[ORE_STORE_AXIS_TRACK_RIGHT];
    g_ore_store_debug_view.track_right_position_rad =
      ore_store.feedback.position_rad[ORE_STORE_AXIS_TRACK_RIGHT];
    g_ore_store_debug_view.track_right_target_rad =
      ore_store.debug.target_position_rad[ORE_STORE_AXIS_TRACK_RIGHT];
    g_ore_store_debug_view.track_right_command_rad =
      ore_store.debug.command_position_rad[ORE_STORE_AXIS_TRACK_RIGHT];
    g_ore_store_debug_view.track_right_rm_output_raw =
      ore_store.debug.rm_output_raw[ORE_STORE_AXIS_TRACK_RIGHT];
}

static void OreStoreTask_UpdateTemperatureAlarm(uint32_t now_tick) {
  bool has_warning = false;
  bool has_over_limit = false;
  float max_temperature_c = 0.0f;

  for (uint8_t axis = 0; axis < ORE_STORE_AXIS_NUM; ++axis) {
    const bool axis_warning = ore_store.feedback.temperature_warning[axis];
    const bool axis_over_limit =
        ore_store.feedback.temperature_over_limit[axis];
    const float temperature_c = ore_store.feedback.motor[axis].temp;

    task_runtime.status.ore_store_alarm.axis_temperature_warning[axis] =
        axis_warning;
    task_runtime.status.ore_store_alarm.axis_temperature_over_limit[axis] =
        axis_over_limit;

    has_warning = has_warning || axis_warning;
    has_over_limit = has_over_limit || axis_over_limit;
    if (axis == 0u || temperature_c > max_temperature_c) {
      max_temperature_c = temperature_c;
    }
  }

  task_runtime.status.ore_store_alarm.max_temperature_c = max_temperature_c;
  if (has_over_limit) {
    task_runtime.status.ore_store_alarm.level = BUZZER_ALARM_TEMP_OVER_LIMIT;
    g_buzzer_alarm_request.level = BUZZER_ALARM_TEMP_OVER_LIMIT;
    g_buzzer_alarm_request.min_duration_ms =
        ORE_STORE_TEMP_OVER_LIMIT_ALARM_MS;
    g_buzzer_alarm_request.request_tick = now_tick;
  } else if (has_warning) {
    task_runtime.status.ore_store_alarm.level = BUZZER_ALARM_TEMP_WARNING;
    g_buzzer_alarm_request.level = BUZZER_ALARM_TEMP_WARNING;
    g_buzzer_alarm_request.min_duration_ms = ORE_STORE_TEMP_WARNING_ALARM_MS;
    g_buzzer_alarm_request.request_tick = now_tick;
  } else {
    task_runtime.status.ore_store_alarm.level = BUZZER_ALARM_NONE;
  }
}

static float OreStoreTask_CalcPowerOnHomeTimeoutS(
    const OreStore_Params_t *param) {
  if (param == NULL) {
    return ORE_STORE_POWER_ON_HOME_EXTRA_TIMEOUT_S;
  }

  float timeout_s = 0.0f;
  for (uint8_t axis = 0; axis < ORE_STORE_AXIS_NUM; ++axis) {
    const OreStore_SoftLimitConfig_t *limit = &param->limit.config[axis];
    float axis_timeout_s = 0.0f;
    if (isfinite(limit->online_wait_timeout_s) &&
        limit->online_wait_timeout_s > 0.0f) {
      axis_timeout_s += limit->online_wait_timeout_s;
    }
    if (isfinite(limit->seek_timeout_s) && limit->seek_timeout_s > 0.0f) {
      axis_timeout_s += limit->seek_timeout_s;
    }
    if (axis_timeout_s > timeout_s) {
      timeout_s = axis_timeout_s;
    }
  }

  return timeout_s + ORE_STORE_POWER_ON_HOME_EXTRA_TIMEOUT_S;
}

static bool OreStoreTask_HasHomeFailureAfterStart(void) {
  const OreStore_Debug_t *debug = OreStore_GetDebug(&ore_store);
  if (debug == NULL) {
    return true;
  }

  for (uint8_t axis = 0; axis < ORE_STORE_AXIS_NUM; ++axis) {
    if (debug->homing_started[axis] && debug->axis_failed[axis]) {
      return true;
    }
  }

  return false;
}

static bool OreStoreTask_PowerOnHomeCanExit(void) {
  return ore_store_power_on_home_active_cycles >=
         ORE_STORE_POWER_ON_HOME_MIN_HOLD_CYCLES;
}

static void OreStoreTask_FinishPowerOnHomeAttempt(
    bool failed, OreStoreTask_PowerOnHomeExitReason_t reason) {
  ore_store_power_on_home_active = false;
  ore_store_power_on_home_attempt_done = true;
  ore_store_power_on_home_failed = failed;
  ore_store_power_on_home_exit_reason = (uint8_t)reason;
  ++ore_store_debug_home_finish_count;
  memset(&ore_store_cmd, 0, sizeof(ore_store_cmd));
  ore_store_cmd.mode = ORE_STORE_MODE_RELAX;
}

static void OreStoreTask_ForcePowerOnHomeCommand(OreStore_CMD_t *cmd) {
  if (cmd == NULL || !ore_store_power_on_home_active ||
      ore_store_power_on_home_attempt_done) {
    return;
  }

  cmd->mode = ORE_STORE_MODE_HOME;
  cmd->force_rehome = false;
}

static void OreStoreTask_ApplyPowerOnHomeLock(OreStore_CMD_t *cmd) {
  if (cmd == NULL || !ore_store_power_on_home_active ||
      ore_store_power_on_home_attempt_done) {
    return;
  }

  if (OreStoreTask_PowerOnHomeCanExit() && OreStore_IsAllHomed(&ore_store)) {
    OreStoreTask_FinishPowerOnHomeAttempt(
        false, ORE_STORE_POWER_ON_HOME_EXIT_SUCCESS);
    return;
  }

  if (OreStoreTask_PowerOnHomeCanExit() &&
      ore_store_power_on_home_timeout_s > 0.0f &&
      ore_store_power_on_home_elapsed_s >= ore_store_power_on_home_timeout_s) {
    OreStoreTask_FinishPowerOnHomeAttempt(
        true, ORE_STORE_POWER_ON_HOME_EXIT_TIMEOUT);
    return;
  }

  OreStoreTask_ForcePowerOnHomeCommand(cmd);
}

void Task_ore_store(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / ORE_STORE_FREQ;
  osDelay(ORE_STORE_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  Config_RobotParam_t *cfg = Config_GetRobotParam();
  if (cfg == NULL) {
    g_ore_store_init_ret = ORE_STORE_ERR_NULL;
    osThreadTerminate(osThreadGetId());
    return;
  }

  OreStoreTask_SetDefaultCommand(&ore_store_cmd);
  g_ore_store_init_ret =
      OreStore_Init(&ore_store, &cfg->ore_store_param, (float)ORE_STORE_FREQ);
  if (g_ore_store_init_ret != ORE_STORE_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }
  ore_store_inited = true;
  ore_store_power_on_home_active = true;
  ore_store_power_on_home_attempt_done = false;
  ore_store_power_on_home_failed = false;
  ore_store_power_on_home_elapsed_s = 0.0f;
  ore_store_power_on_home_active_cycles = 0u;
  ore_store_power_on_home_exit_reason = ORE_STORE_POWER_ON_HOME_EXIT_NONE;
  ore_store_debug_last_received_cmd_mode = ORE_STORE_MODE_HOME;
  ore_store_debug_pre_control_cmd_mode = ORE_STORE_MODE_HOME;
  ore_store_debug_post_control_mode = ORE_STORE_MODE_RELAX;
  ore_store_debug_home_override_fault_count = 0u;
  ore_store_debug_home_finish_count = 0u;
  ore_store_power_on_home_timeout_s =
      OreStoreTask_CalcPowerOnHomeTimeoutS(&cfg->ore_store_param);

  while (1) {
    tick += delay_tick;

    OreStore_CMD_t next_cmd;
    if (osMessageQueueGet(task_runtime.msgq.ore_store.cmd, &next_cmd, NULL,
                          0) == osOK) {
      OreStoreTask_SanitizeCommand(&next_cmd);
      ore_store_debug_last_received_cmd_mode = next_cmd.mode;
      if (OreStore_IsAllHomed(&ore_store) || next_cmd.mode != ORE_STORE_MODE_RELAX) {
        ore_store_cmd = next_cmd;
      } else {
        ore_store_cmd.mode = ORE_STORE_MODE_HOME;
        ore_store_cmd.force_rehome = false;
      }
    }

    (void)OreStoreTask_TryApplyDebugCommand(&ore_store_cmd);

    if (ore_store_rehome_requested) {
      OreStore_RequestRehome(&ore_store);
      ore_store_rehome_requested = false;
    }

    g_ore_store_update_ret = OreStore_UpdateFeedback(&ore_store);
    OreStoreTask_ApplyAssumeHomedDebug();
    const uint32_t now_tick = osKernelGetTickCount();
    OreStoreTask_UpdateTemperatureAlarm(now_tick);
    OreStoreTask_ApplyPowerOnHomeLock(&ore_store_cmd);
    OreStoreTask_ForcePowerOnHomeCommand(&ore_store_cmd);
    ore_store_debug_pre_control_cmd_mode = ore_store_cmd.mode;
    g_ore_store_control_ret =
        OreStore_Control(&ore_store, &ore_store_cmd, now_tick);
    ore_store_debug_post_control_mode = ore_store.mode;
    if (ore_store_power_on_home_active &&
        !ore_store_power_on_home_attempt_done &&
        ore_store.mode != ORE_STORE_MODE_HOME) {
      ++ore_store_debug_home_override_fault_count;
      ore_store.mode = ORE_STORE_MODE_HOME;
      ore_store_debug_post_control_mode = ore_store.mode;
    }
    if (ore_store_power_on_home_active &&
        !ore_store_power_on_home_attempt_done) {
      ore_store_power_on_home_elapsed_s += ore_store.debug.dt_s;
      ++ore_store_power_on_home_active_cycles;
      if (OreStoreTask_PowerOnHomeCanExit() &&
          OreStore_IsAllHomed(&ore_store)) {
        OreStoreTask_FinishPowerOnHomeAttempt(
            false, ORE_STORE_POWER_ON_HOME_EXIT_SUCCESS);
      } else if (OreStoreTask_PowerOnHomeCanExit() &&
                 OreStoreTask_HasHomeFailureAfterStart()) {
        OreStoreTask_FinishPowerOnHomeAttempt(
            true, ORE_STORE_POWER_ON_HOME_EXIT_FAILURE_AFTER_START);
      } else if (OreStoreTask_PowerOnHomeCanExit() &&
                 (ore_store_power_on_home_timeout_s > 0.0f &&
                  ore_store_power_on_home_elapsed_s >=
                      ore_store_power_on_home_timeout_s)) {
        OreStoreTask_FinishPowerOnHomeAttempt(
            true, ORE_STORE_POWER_ON_HOME_EXIT_TIMEOUT);
      }
    }
    OreStore_Output(&ore_store);
    OreStoreTask_ApplyDirectDebugOutput();
    OreStoreTask_UpdateDebugView();
    ore_store_cmd.force_rehome = false;

    task_runtime.stack_water_mark.ore_store =
        uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick);
  }
}

int8_t Task_OreStorePostCommand(const OreStore_CMD_t *cmd) {
  if (cmd == NULL || task_runtime.msgq.ore_store.cmd == NULL) {
    return ORE_STORE_ERR_NULL;
  }

  OreStore_CMD_t safe_cmd = *cmd;
  OreStoreTask_SanitizeCommand(&safe_cmd);
  (void)osMessageQueueReset(task_runtime.msgq.ore_store.cmd);
  return (osMessageQueuePut(task_runtime.msgq.ore_store.cmd, &safe_cmd, 0, 0) ==
          osOK)
             ? ORE_STORE_OK
             : ORE_STORE_ERR;
}

void Task_OreStoreRequestRehome(void) {
  ore_store_rehome_requested = true;
}

int8_t Task_OreStoreAssumeAxisHomedAtCurrent(uint8_t axis,
                                             float position_rad) {
  if (!ore_store_inited) {
    return ORE_STORE_ERR_NULL;
  }
  return OreStore_AssumeAxisHomedAtCurrent(&ore_store, axis, position_rad);
}

bool Task_OreStoreIsAxisHomed(uint8_t axis) {
  return ore_store_inited && OreStore_IsAxisHomed(&ore_store, axis);
}

bool Task_OreStoreIsAllHomed(void) {
  return ore_store_inited && OreStore_IsAllHomed(&ore_store);
}

bool Task_OreStorePowerOnHomeInProgress(void) {
  return ore_store_inited && ore_store_power_on_home_active &&
         !ore_store_power_on_home_attempt_done;
}

bool Task_OreStorePowerOnHomeAttemptDone(void) {
  return ore_store_inited && ore_store_power_on_home_attempt_done;
}

bool Task_OreStorePowerOnHomeFailed(void) {
  return ore_store_inited && ore_store_power_on_home_failed;
}

bool Task_OreStoreIsAxisAtTarget(uint8_t axis, float threshold_rad) {
  return ore_store_inited &&
         OreStore_IsAxisAtTarget(&ore_store, axis, threshold_rad);
}

bool Task_OreStoreIsAllAtTarget(float threshold_rad) {
  return ore_store_inited &&
         OreStore_IsAllAtTarget(&ore_store, threshold_rad);
}

const OreStore_Feedback_t *Task_OreStoreGetFeedback(void) {
  return ore_store_inited ? &ore_store.feedback : NULL;
}

const OreStore_Debug_t *Task_OreStoreGetDebug(void) {
  return ore_store_inited ? OreStore_GetDebug(&ore_store) : NULL;
}
