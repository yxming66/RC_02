/*
 * Ore store task.
 */

#include "task/user_task.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "module/config.h"
#include "module/ore_store.h"

static OreStore_t ore_store;
static OreStore_CMD_t ore_store_cmd;
static volatile bool ore_store_inited = false;
static volatile bool ore_store_rehome_requested = false;
volatile int8_t g_ore_store_init_ret = ORE_STORE_ERR_NULL;
volatile int8_t g_ore_store_update_ret = ORE_STORE_ERR_NULL;
volatile int8_t g_ore_store_control_ret = ORE_STORE_ERR_NULL;

static void OreStoreTask_SetDefaultCommand(OreStore_CMD_t *cmd) {
  if (cmd == NULL) {
    return;
  }

  memset(cmd, 0, sizeof(*cmd));
  cmd->mode = ORE_STORE_MODE_RELAX;
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

  while (1) {
    tick += delay_tick;

    OreStore_CMD_t next_cmd;
    if (osMessageQueueGet(task_runtime.msgq.ore_store.cmd, &next_cmd, NULL,
                          0) == osOK) {
      OreStoreTask_SanitizeCommand(&next_cmd);
      ore_store_cmd = next_cmd;
    }

    if (ore_store_rehome_requested) {
      OreStore_RequestRehome(&ore_store);
      ore_store_rehome_requested = false;
    }

    g_ore_store_update_ret = OreStore_UpdateFeedback(&ore_store);
    g_ore_store_control_ret =
        OreStore_Control(&ore_store, &ore_store_cmd, osKernelGetTickCount());
    // OreStore_Output(&ore_store);
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

bool Task_OreStoreIsAxisHomed(uint8_t axis) {
  return ore_store_inited && OreStore_IsAxisHomed(&ore_store, axis);
}

bool Task_OreStoreIsAllHomed(void) {
  return ore_store_inited && OreStore_IsAllHomed(&ore_store);
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
