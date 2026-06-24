/*
 * Ore store task.
 */

#include "task/user_task.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "module/config.h"
#include "module/camera_yaw.h"
#include "module/ore_store.h"

static OreStore_t ore_store;
static OreStore_CMD_t ore_store_cmd;
static CameraYaw_t camera_yaw[CAMERA_YAW_NUM];
static CameraYaw_GroupCMD_t camera_yaw_cmd;
static CameraYaw_GroupFeedback_t camera_yaw_feedback;
static volatile bool ore_store_init_attempted = false;
static volatile bool ore_store_inited = false;
static volatile bool camera_yaw_init_attempted = false;
static volatile bool camera_yaw_inited[CAMERA_YAW_NUM] = {false};
static volatile bool ore_store_rehome_requested = false;
static volatile bool ore_store_power_on_home_in_progress = false;
static volatile bool ore_store_power_on_home_attempt_done = false;
static volatile bool ore_store_power_on_home_failed = false;
static uint32_t ore_store_power_on_home_start_ms = 0u;
volatile int8_t g_ore_store_init_ret = ORE_STORE_ERR_NULL;
volatile int8_t g_ore_store_update_ret = ORE_STORE_ERR_NULL;
volatile int8_t g_ore_store_control_ret = ORE_STORE_ERR_NULL;
volatile int8_t g_camera_yaw_init_ret = CAMERA_YAW_ERR_NULL;
volatile int8_t g_camera_yaw_update_ret = CAMERA_YAW_ERR_NULL;
volatile int8_t g_camera_yaw_control_ret = CAMERA_YAW_ERR_NULL;
volatile int8_t g_camera_yaw_init_ret_by_channel[CAMERA_YAW_NUM] = {
    CAMERA_YAW_ERR_NULL, CAMERA_YAW_ERR_NULL};
volatile int8_t g_camera_yaw_update_ret_by_channel[CAMERA_YAW_NUM] = {
    CAMERA_YAW_ERR_NULL, CAMERA_YAW_ERR_NULL};
volatile int8_t g_camera_yaw_control_ret_by_channel[CAMERA_YAW_NUM] = {
    CAMERA_YAW_ERR_NULL, CAMERA_YAW_ERR_NULL};

typedef struct {
  volatile bool inited;
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
  volatile uint16_t platform_stall_cycles;
  volatile float platform_seek_velocity_rad_s;
  volatile float platform_seek_travel_rad;
  volatile float platform_feedback_torque_nm;
  volatile float platform_filtered_output_torque_nm;
  volatile float platform_velocity_setpoint_rad_s;
  volatile float platform_cmd_torque_nm;
  volatile float platform_cmd_current_a;
  volatile int16_t platform_rm_output_raw;
  volatile uint16_t platform_rm_tx_frame_id;
  volatile bool power_on_home_in_progress;
  volatile bool power_on_home_attempt_done;
  volatile bool power_on_home_failed;
} OreStoreTask_DebugView_t;

volatile OreStoreTask_DebugView_t g_ore_store_debug_view = {0};

volatile OreStore_DebugCommand_t g_ore_store_debug_command = {
    .enable = false,
    .mode = ORE_STORE_MODE_RELAX,
  .direct_set_ret = ORE_STORE_ERR_NULL,
  .direct_ctrl_ret = ORE_STORE_ERR_NULL,
  .assume_homed_ret = ORE_STORE_ERR_NULL,
};

volatile CameraYaw_DebugControl_t g_camera_yaw_debug = {
    .enable = false,
    .direct_output_enable = false,
    .mode = CAMERA_YAW_MODE_RELAX,
    .target_yaw_rad = 0.0f,
    .feedback_yaw_rad = 0.0f,
    .direct_output = 0.0f,
};

#define ORE_STORE_TEMP_WARNING_ALARM_MS (3000u)
#define ORE_STORE_TEMP_OVER_LIMIT_ALARM_MS (5000u)
#define ORE_STORE_POWER_ON_HOME_READY_TIMEOUT_MS (10000u)

static void OreStoreTask_SetDefaultCommand(OreStore_CMD_t *cmd) {
  if (cmd == NULL) {
    return;
  }

  memset(cmd, 0, sizeof(*cmd));
  cmd->mode = ORE_STORE_MODE_RELAX;
}

static void OreStoreTask_SetPowerOnHomeCommand(OreStore_CMD_t *cmd) {
  if (cmd == NULL) {
    return;
  }

  memset(cmd, 0, sizeof(*cmd));
  cmd->mode = ORE_STORE_MODE_HOME;
}

static void OreStoreTask_StartPowerOnHome(void) {
  ore_store_power_on_home_in_progress = true;
  ore_store_power_on_home_attempt_done = false;
  ore_store_power_on_home_failed = false;
  ore_store_power_on_home_start_ms = BSP_TIME_Get_ms();
  OreStoreTask_SetPowerOnHomeCommand(&ore_store_cmd);
}

static int8_t OreStoreTask_AssumePowerOnHomeAtLowest(void) {
  int8_t result = ORE_STORE_OK;

  for (uint8_t axis = 0u; axis < ORE_STORE_AXIS_NUM; ++axis) {
    if (OreStore_IsAxisHomed(&ore_store, axis)) {
      continue;
    }

    const int8_t ret =
        OreStore_AssumeAxisHomedAtCurrent(&ore_store, axis, 0.0f);
    if (ret != ORE_STORE_OK) {
      result = ORE_STORE_ERR;
    }
  }

  return result;
}

static bool OreStoreTask_HomeAxisFailed(void) {
  for (uint8_t axis = 0u; axis < ORE_STORE_AXIS_NUM; ++axis) {
    if (ore_store.debug.axis_failed[axis]) {
      return true;
    }
  }
  return false;
}

static bool OreStoreTask_FeedbackReadyForPowerOnHome(void) {
  for (uint8_t axis = 0u; axis < ORE_STORE_AXIS_NUM; ++axis) {
    if (!ore_store.feedback.online[axis] ||
        ore_store.debug.controller_update_ret[axis] != DEVICE_OK) {
      return false;
    }
  }
  return true;
}

static bool OreStoreTask_PowerOnHomeReadyTimedOut(uint32_t now_ms) {
  return (uint32_t)(now_ms - ore_store_power_on_home_start_ms) >=
         ORE_STORE_POWER_ON_HOME_READY_TIMEOUT_MS;
}

static void OreStoreTask_FinishPowerOnHome(bool failed) {
  ore_store_power_on_home_in_progress = false;
  ore_store_power_on_home_attempt_done = true;
  ore_store_power_on_home_failed = failed;
  OreStoreTask_SetDefaultCommand(&ore_store_cmd);
}

static void OreStoreTask_UpdatePowerOnHomeState(int8_t control_ret,
                                                uint32_t now_ms) {
  (void)control_ret;

  if (!ore_store_power_on_home_in_progress) {
    return;
  }

  if (OreStore_IsAllHomed(&ore_store)) {
    OreStoreTask_FinishPowerOnHome(false);
    return;
  }

  if (!OreStoreTask_FeedbackReadyForPowerOnHome()) {
    if (OreStoreTask_PowerOnHomeReadyTimedOut(now_ms)) {
      OreStoreTask_FinishPowerOnHome(true);
    }
    return;
  }

  const int8_t assume_ret = OreStoreTask_AssumePowerOnHomeAtLowest();
  OreStoreTask_FinishPowerOnHome(assume_ret != ORE_STORE_OK ||
                                 OreStoreTask_HomeAxisFailed());
}

static void OreStoreTask_SelectControlCommand(OreStore_CMD_t *control_cmd) {
  if (control_cmd == NULL) {
    return;
  }

  if (ore_store_power_on_home_in_progress) {
    if (OreStoreTask_FeedbackReadyForPowerOnHome()) {
      OreStoreTask_SetPowerOnHomeCommand(control_cmd);
    } else {
      OreStoreTask_SetDefaultCommand(control_cmd);
    }
  } else {
    *control_cmd = ore_store_cmd;
  }
}

static void OreStoreTask_SetDefaultCameraYawCommand(CameraYaw_CMD_t *cmd) {
  if (cmd == NULL) {
    return;
  }

  memset(cmd, 0, sizeof(*cmd));
  cmd->mode = CAMERA_YAW_MODE_RELAX;
  cmd->feedback_valid = false;
}

static void OreStoreTask_SetDefaultCameraYawGroupCommand(
    CameraYaw_GroupCMD_t *cmd) {
  if (cmd == NULL) {
    return;
  }

  memset(cmd, 0, sizeof(*cmd));
  for (uint8_t yaw = 0u; yaw < CAMERA_YAW_NUM; ++yaw) {
    OreStoreTask_SetDefaultCameraYawCommand(&cmd->yaw[yaw]);
  }
}

static bool OreStoreTask_ModeIsValid(OreStore_Mode_t mode) {
  return mode == ORE_STORE_MODE_RELAX || mode == ORE_STORE_MODE_HOME ||
         mode == ORE_STORE_MODE_ACTIVE;
}

static bool OreStoreTask_TargetsAreFinite(const OreStore_CMD_t *cmd) {
  if (cmd == NULL) {
    return false;
  }

  return isfinite(cmd->platform_target_rad);
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

static void OreStoreTask_SanitizeCameraYawCommand(CameraYaw_CMD_t *cmd) {
  if (cmd == NULL) {
    return;
  }

  const bool mode_valid =
      cmd->mode == CAMERA_YAW_MODE_RELAX ||
      cmd->mode == CAMERA_YAW_MODE_ACTIVE;
  if (!mode_valid || !isfinite(cmd->target_yaw_rad)) {
    OreStoreTask_SetDefaultCameraYawCommand(cmd);
  }
}

static void OreStoreTask_SanitizeCameraYawGroupCommand(
    CameraYaw_GroupCMD_t *cmd) {
  if (cmd == NULL) {
    return;
  }

  for (uint8_t yaw = 0u; yaw < CAMERA_YAW_NUM; ++yaw) {
    OreStoreTask_SanitizeCameraYawCommand(&cmd->yaw[yaw]);
  }
}

static bool OreStoreTask_TryApplyDebugCommand(OreStore_CMD_t *cmd) {
  if (cmd == NULL || !g_ore_store_debug_command.enable) {
    return false;
  }

  cmd->mode = g_ore_store_debug_command.mode;
  cmd->force_rehome = g_ore_store_debug_command.force_rehome;
  cmd->platform_target_rad = g_ore_store_debug_command.platform_target_rad;

  OreStoreTask_SanitizeCommand(cmd);
  ++g_ore_store_debug_command.applied_count;
  return true;
}

static bool OreStoreTask_TryApplyCameraYawDebugCommand(CameraYaw_CMD_t *cmd,
                                                       uint32_t now_ms) {
  if (cmd == NULL || !g_camera_yaw_debug.enable ||
      g_camera_yaw_debug.direct_output_enable) {
    return false;
  }

  cmd->mode = g_camera_yaw_debug.mode;
  cmd->target_yaw_rad = g_camera_yaw_debug.target_yaw_rad;
  cmd->feedback_yaw_rad = g_camera_yaw_debug.feedback_yaw_rad;
  cmd->feedback_tick_ms = now_ms;
  cmd->feedback_valid = true;
  OreStoreTask_SanitizeCameraYawCommand(cmd);
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
      g_ore_store_debug_command.direct_output_axis != ORE_STORE_AXIS_PLATFORM) {
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
  g_ore_store_debug_view.platform_stall_cycles =
      ore_store.debug.stall_cycles[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_seek_velocity_rad_s =
      ore_store.debug.seek_velocity_rad_s[ORE_STORE_AXIS_PLATFORM];
  g_ore_store_debug_view.platform_seek_travel_rad =
      ore_store.debug.seek_travel_rad[ORE_STORE_AXIS_PLATFORM];
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
  g_ore_store_debug_view.power_on_home_in_progress =
      ore_store_power_on_home_in_progress;
  g_ore_store_debug_view.power_on_home_attempt_done =
      ore_store_power_on_home_attempt_done;
  g_ore_store_debug_view.power_on_home_failed =
      ore_store_power_on_home_failed;
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

bool Task_OreStoreInitOnce(void) {
  if (ore_store_inited) {
    return true;
  }
  if (ore_store_init_attempted) {
    return false;
  }
  ore_store_init_attempted = true;

  Config_RobotParam_t *cfg = Config_GetRobotParam();
  if (cfg == NULL) {
    g_ore_store_init_ret = ORE_STORE_ERR_NULL;
    return false;
  }

  OreStoreTask_SetDefaultCommand(&ore_store_cmd);

  g_ore_store_init_ret =
      OreStore_Init(&ore_store, &cfg->ore_store_param, (float)ORE_STORE_FREQ);
  if (g_ore_store_init_ret != ORE_STORE_OK) {
    return false;
  }
  ore_store_inited = true;
  OreStoreTask_StartPowerOnHome();

  return true;
}

bool Task_CameraYawInitOnce(void) {
  bool any_inited = false;
  for (uint8_t yaw = 0u; yaw < CAMERA_YAW_NUM; ++yaw) {
    any_inited = any_inited || camera_yaw_inited[yaw];
  }
  if (any_inited) {
    return true;
  }
  if (camera_yaw_init_attempted) {
    return false;
  }
  camera_yaw_init_attempted = true;

  Config_RobotParam_t *cfg = Config_GetRobotParam();
  if (cfg == NULL) {
    g_camera_yaw_init_ret = CAMERA_YAW_ERR_NULL;
    return false;
  }

  OreStoreTask_SetDefaultCameraYawGroupCommand(&camera_yaw_cmd);
  memset(&camera_yaw_feedback, 0, sizeof(camera_yaw_feedback));

  g_camera_yaw_init_ret = CAMERA_YAW_ERR;
  for (uint8_t yaw = 0u; yaw < CAMERA_YAW_NUM; ++yaw) {
    g_camera_yaw_init_ret_by_channel[yaw] =
        CameraYaw_Init(&camera_yaw[yaw], &cfg->camera_yaw_param[yaw],
                       (float)CAMERA_YAW_FREQ);
    camera_yaw_inited[yaw] =
        (g_camera_yaw_init_ret_by_channel[yaw] == CAMERA_YAW_OK);
    if (camera_yaw_inited[yaw]) {
      g_camera_yaw_init_ret = CAMERA_YAW_OK;
      any_inited = true;
    }
  }

  return any_inited;
}

void Task_OreStoreStep(void) {
  if (!ore_store_inited) {
    return;
  }

  const uint32_t profile_start_us =
      Task_ProfilerLoopBegin(TASK_PROFILE_ORE_STORE,
                             TASK_PERIOD_US(ORE_STORE_FREQ));

  OreStore_CMD_t next_cmd;
  if (osMessageQueueGet(task_runtime.msgq.ore_store.cmd, &next_cmd, NULL,
                        0) == osOK) {
    OreStoreTask_SanitizeCommand(&next_cmd);
    ore_store_cmd = next_cmd;
  }

  (void)OreStoreTask_TryApplyDebugCommand(&ore_store_cmd);

  if (ore_store_rehome_requested) {
    OreStore_RequestRehome(&ore_store);
    ore_store_rehome_requested = false;
  }

  g_ore_store_update_ret = OreStore_UpdateFeedback(&ore_store);
  OreStoreTask_ApplyAssumeHomedDebug();
  const uint32_t now_tick = BSP_TIME_Get_ms();
  OreStoreTask_UpdatePowerOnHomeState(ORE_STORE_OK, now_tick);

  OreStoreTask_UpdateTemperatureAlarm(now_tick);

  OreStore_CMD_t control_cmd;
  OreStoreTask_SelectControlCommand(&control_cmd);
  g_ore_store_control_ret =
      OreStore_Control(&ore_store, &control_cmd, now_tick);
  OreStore_Output(&ore_store);
  OreStoreTask_ApplyDirectDebugOutput();
  OreStoreTask_UpdateDebugView();
  ore_store_cmd.force_rehome = false;

  task_runtime.stack_water_mark.ore_store =
      uxTaskGetStackHighWaterMark(NULL);
  task_runtime.heartbeat.ore_store++;
  Task_ProfilerLoopEnd(TASK_PROFILE_ORE_STORE, profile_start_us);
}

void Task_CameraYawStep(void) {
  bool any_inited = false;
  for (uint8_t yaw = 0u; yaw < CAMERA_YAW_NUM; ++yaw) {
    any_inited = any_inited || camera_yaw_inited[yaw];
  }
  if (!any_inited) {
    return;
  }

  const uint32_t profile_start_us =
      Task_ProfilerLoopBegin(TASK_PROFILE_CAMERA_YAW,
                             TASK_PERIOD_US(CAMERA_YAW_FREQ));
  const uint32_t now_tick = BSP_TIME_Get_ms();

  CameraYaw_GroupCMD_t next_camera_yaw_cmd;
  if (osMessageQueueGet(task_runtime.msgq.camera_yaw.cmd,
                        &next_camera_yaw_cmd, NULL, 0) == osOK) {
    OreStoreTask_SanitizeCameraYawGroupCommand(&next_camera_yaw_cmd);
    camera_yaw_cmd = next_camera_yaw_cmd;
  }

  for (uint8_t yaw = 0u; yaw < CAMERA_YAW_NUM; ++yaw) {
    if (camera_yaw_inited[yaw]) {
      (void)OreStoreTask_TryApplyCameraYawDebugCommand(
          &camera_yaw_cmd.yaw[yaw], now_tick);
    }
  }

  for (uint8_t yaw = 0u; yaw < CAMERA_YAW_NUM; ++yaw) {
    if (!camera_yaw_inited[yaw]) {
      continue;
    }
    g_camera_yaw_update_ret_by_channel[yaw] =
        CameraYaw_UpdateFeedback(&camera_yaw[yaw]);
    g_camera_yaw_control_ret_by_channel[yaw] =
        CameraYaw_Control(&camera_yaw[yaw], &camera_yaw_cmd.yaw[yaw],
                          now_tick);
    CameraYaw_SetOutput(&camera_yaw[yaw]);
    camera_yaw_feedback.yaw[yaw] = camera_yaw[yaw].feedback;
  }
  for (uint8_t yaw = 0u; yaw < CAMERA_YAW_NUM; ++yaw) {
    if (camera_yaw_inited[yaw]) {
      CameraYaw_FlushOutput(&camera_yaw[yaw]);
    }
  }

  g_camera_yaw_update_ret =
      g_camera_yaw_update_ret_by_channel[CAMERA_YAW_LEFT];
  g_camera_yaw_control_ret =
      g_camera_yaw_control_ret_by_channel[CAMERA_YAW_LEFT];
  task_runtime.stack_water_mark.camera_yaw = uxTaskGetStackHighWaterMark(NULL);
  task_runtime.heartbeat.camera_yaw++;
  Task_ProfilerLoopEnd(TASK_PROFILE_CAMERA_YAW, profile_start_us);
}

void Task_ore_store(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / ORE_STORE_FREQ;
  osDelay(ORE_STORE_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  if (!Task_OreStoreInitOnce()) {
    osThreadTerminate(osThreadGetId());
    return;
  }

  while (1) {
    tick += delay_tick;
    Task_OreStoreStep();
    Task_DelayUntil(TASK_PROFILE_ORE_STORE, &tick, delay_tick);
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
  return ore_store_power_on_home_in_progress;
}

bool Task_OreStorePowerOnHomeAttemptDone(void) {
  return ore_store_power_on_home_attempt_done;
}

bool Task_OreStorePowerOnHomeFailed(void) {
  return ore_store_power_on_home_failed;
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

const CameraYaw_Feedback_t *Task_CameraYawGetFeedback(void) {
  if (camera_yaw_inited[CAMERA_YAW_LEFT]) {
    return &camera_yaw_feedback.yaw[CAMERA_YAW_LEFT];
  }
  for (uint8_t yaw = 0u; yaw < CAMERA_YAW_NUM; ++yaw) {
    if (camera_yaw_inited[yaw]) {
      return &camera_yaw_feedback.yaw[yaw];
    }
  }
  return NULL;
}

const CameraYaw_GroupFeedback_t *Task_CameraYawGetGroupFeedback(void) {
  for (uint8_t yaw = 0u; yaw < CAMERA_YAW_NUM; ++yaw) {
    if (camera_yaw_inited[yaw]) {
      return &camera_yaw_feedback;
    }
  }
  return NULL;
}

int8_t Task_CameraYawPostCommand(const CameraYaw_CMD_t *cmd) {
  if (cmd == NULL || task_runtime.msgq.camera_yaw.cmd == NULL) {
    return CAMERA_YAW_ERR_NULL;
  }

  CameraYaw_GroupCMD_t safe_cmd = camera_yaw_cmd;
  safe_cmd.yaw[CAMERA_YAW_LEFT] = *cmd;
  OreStoreTask_SanitizeCameraYawGroupCommand(&safe_cmd);
  (void)osMessageQueueReset(task_runtime.msgq.camera_yaw.cmd);
  return (osMessageQueuePut(task_runtime.msgq.camera_yaw.cmd, &safe_cmd, 0,
                            0) == osOK)
             ? CAMERA_YAW_OK
             : CAMERA_YAW_ERR;
}

int8_t Task_CameraYawPostGroupCommand(const CameraYaw_GroupCMD_t *cmd) {
  if (cmd == NULL || task_runtime.msgq.camera_yaw.cmd == NULL) {
    return CAMERA_YAW_ERR_NULL;
  }

  CameraYaw_GroupCMD_t safe_cmd = *cmd;
  OreStoreTask_SanitizeCameraYawGroupCommand(&safe_cmd);
  (void)osMessageQueueReset(task_runtime.msgq.camera_yaw.cmd);
  return (osMessageQueuePut(task_runtime.msgq.camera_yaw.cmd, &safe_cmd, 0,
                            0) == osOK)
             ? CAMERA_YAW_OK
             : CAMERA_YAW_ERR;
}
