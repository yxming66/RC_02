#include "task/user_task.h"

Task_Runtime_t task_runtime;
Buzzer_AlarmRequest_t g_buzzer_alarm_request;

uint32_t Task_ProfilerLoopBegin(Task_ProfileId_t id, uint32_t target_period_us) {
#if TASK_RUNTIME_PROFILER_ENABLE
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
#else
  (void)id;
  (void)target_period_us;
  return 0U;
#endif
}

void Task_ProfilerLoopEnd(Task_ProfileId_t id, uint32_t loop_start_us) {
#if TASK_RUNTIME_PROFILER_ENABLE
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
#else
  (void)id;
  (void)loop_start_us;
#endif
}

void Task_DelayUntil(Task_ProfileId_t id, uint32_t *wake_tick,
                     uint32_t delay_tick) {
  if (wake_tick == NULL || delay_tick == 0U) {
    osDelay(1U);
    return;
  }

  const uint32_t now_tick = osKernelGetTickCount();
  if ((int32_t)(now_tick - *wake_tick) >= 0) {
    const uint32_t late_tick = now_tick - *wake_tick;
#if TASK_RUNTIME_PROFILER_ENABLE
    if (id < TASK_PROFILE_COUNT) {
      Task_ProfileStats_t *stats = &task_runtime.profile[id];
      stats->deadline_miss_count++;
      stats->late_tick = late_tick;
      if (late_tick > stats->max_late_tick) {
        stats->max_late_tick = late_tick;
      }
      stats->resync_count++;
    }
#else
    (void)id;
    (void)late_tick;
#endif
    *wake_tick = now_tick + delay_tick;
  }

  (void)osDelayUntil(*wake_tick);
}

void Task_SubtaskTimerInit(Task_SubtaskTimer_t *timer, float frequency_hz,
                           uint32_t now_us) {
  if (timer == NULL) {
    return;
  }

  timer->period_us = TASK_PERIOD_US(frequency_hz);
  if (timer->period_us == 0U) {
    timer->period_us = 1U;
  }
  timer->next_due_us = now_us;
  timer->initialized = true;
}

bool Task_SubtaskTimerDue(Task_SubtaskTimer_t *timer, uint32_t now_us) {
  if (timer == NULL || !timer->initialized) {
    return false;
  }

  if ((int32_t)(now_us - timer->next_due_us) < 0) {
    return false;
  }

  const uint32_t period_us = (timer->period_us == 0U) ? 1U : timer->period_us;
  do {
    timer->next_due_us += period_us;
  } while ((int32_t)(now_us - timer->next_due_us) >= 0);
  return true;
}

const osThreadAttr_t attr_init = {
    .name = "Task_Init",
    .priority = osPriorityRealtime,
    .stack_size = 256 * 4,
};

/* User_task */
const osThreadAttr_t attr_blink = {
    .name = "blink",
    .priority = osPriorityLow,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_atti_esti = {
    .name = "atti_esti",
    .priority = osPriorityAboveNormal,
    .stack_size = 512 * 4,
};
const osThreadAttr_t attr_chassis_ore = {
    .name = "chassis_ore",
    .priority = osPriorityHigh,
    .stack_size = 1024 * 4,
};
const osThreadAttr_t attr_pole_main = {
    .name = "pole_main",
    .priority = osPriorityRealtime,
    .stack_size = 512 * 4,
};
const osThreadAttr_t attr_rc_main = {
    .name = "rc_main",
    .priority = osPriorityRealtime,
    .stack_size = 512 * 4,
};
const osThreadAttr_t attr_auto_ctrl = {
    .name = "auto_ctrl",
  .priority = osPriorityNormal,
    .stack_size = 512 * 4,
};
const osThreadAttr_t attr_upper_mech = {
    .name = "upper_mech",
    .priority = osPriorityAboveNormal,
    .stack_size = 1024 * 4,
};
const osThreadAttr_t attr_pc_comm_sick = {
    .name = "pc_comm_sick",
    .priority = osPriorityAboveNormal,
    .stack_size = 768 * 4,
};
const osThreadAttr_t attr_ir_dock = {
    .name = "ir_dock",
    .priority = osPriorityHigh,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_ore_info = {
  .name = "ore_info",
  .priority = osPriorityHigh,
  .stack_size = 512 * 4,
};
