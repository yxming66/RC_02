#include "task/user_task.h"

Task_Runtime_t task_runtime;
Buzzer_AlarmRequest_t g_buzzer_alarm_request;

uint32_t Task_ProfilerLoopBegin(Task_ProfileId_t id, uint32_t target_period_us) {
  const uint32_t now_us = (uint32_t)BSP_TIME_Get_us();
  if (id >= TASK_PROFILE_COUNT) {
    return now_us;
  }

  Task_ProfileStats_t *stats = &task_runtime.profile[id];
  const uint32_t last_start_us = stats->last_start_us;
  stats->target_period_us = target_period_us;
  stats->last_start_us = now_us;
  stats->loop_count++;

  if (last_start_us != 0U) {
    const uint32_t period_us = now_us - last_start_us;
    stats->period_us = period_us;
    if (stats->min_period_us == 0U || period_us < stats->min_period_us) {
      stats->min_period_us = period_us;
    }
    if (period_us > stats->max_period_us) {
      stats->max_period_us = period_us;
    }
    if (target_period_us != 0U) {
      const uint32_t slack_us =
          (target_period_us > 4000U) ? (target_period_us / 4U) : 1000U;
      if (period_us > target_period_us + slack_us) {
        stats->long_period_count++;
      }
    }
  }

  return now_us;
}

void Task_ProfilerLoopEnd(Task_ProfileId_t id, uint32_t loop_start_us) {
  const uint32_t now_us = (uint32_t)BSP_TIME_Get_us();
  if (id >= TASK_PROFILE_COUNT) {
    return;
  }

  Task_ProfileStats_t *stats = &task_runtime.profile[id];
  const uint32_t exec_us = now_us - loop_start_us;
  stats->exec_us = exec_us;
  if (exec_us > stats->max_exec_us) {
    stats->max_exec_us = exec_us;
  }
  if (stats->target_period_us != 0U && exec_us > stats->target_period_us) {
    stats->overrun_count++;
  }
}

const osThreadAttr_t attr_init = {
    .name = "Task_Init",
    .priority = osPriorityRealtime,
    .stack_size = 256 * 4,
};

/* User_task */
const osThreadAttr_t attr_blink = {
    .name = "blink",
    .priority = osPriorityAboveNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_atti_esti = {
    .name = "atti_esti",
    .priority = osPriorityAboveNormal,
    .stack_size = 512 * 4,
};
const osThreadAttr_t attr_chassis_main = {
    .name = "chassis_main",
    .priority = osPriorityAboveNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_pole_main = {
    .name = "pole_main",
    .priority = osPriorityHigh,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_rc_main = {
    .name = "rc_main",
    .priority = osPriorityAboveNormal,
    .stack_size = 512 * 4,
};
const osThreadAttr_t attr_sick = {
    .name = "sick",
    .priority = osPriorityAboveNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_auto_ctrl = {
    .name = "auto_ctrl",
    .priority = osPriorityAboveNormal,
    .stack_size = 512 * 4,
};
const osThreadAttr_t attr_arm_simple = {
    .name = "arm_simple",
    .priority = osPriorityAboveNormal,
    .stack_size = 512 * 4,
};
const osThreadAttr_t attr_rod = {
    .name = "rod",
    .priority = osPriorityAboveNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_pc_comm = {
    .name = "pc_comm",
    .priority = osPriorityAboveNormal,
    .stack_size = 512 * 4,
};
const osThreadAttr_t attr_ore_store = {
    .name = "ore_store",
    .priority = osPriorityAboveNormal,
    .stack_size = 512 * 4,
};
const osThreadAttr_t attr_ir_dock = {
    .name = "ir_dock",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
