/*
 * 部分上层机构融合任务
 *
 * Subtasks:
 * - camera_yaw: camera yaw motors and group command queue
 * - arm_simple: two-joint ore arm and suction
 * - rod_new: spearhead rod servo and shared valve output
 */

#include "task/user_task.h"

void Task_upper_mech(void *argument) {
  (void)argument;

  uint32_t base_delay_tick =
      osKernelGetTickFreq() / (uint32_t)TASK_FUSED_LOOP_FREQ;
  if (base_delay_tick == 0U) {
    base_delay_tick = 1U;
  }

  const uint32_t now_us = (uint32_t)BSP_TIME_Get_us();
  Task_SubtaskTimer_t camera_yaw_timer;
  Task_SubtaskTimer_t arm_simple_timer;
  Task_SubtaskTimer_t rod_new_timer;
  Task_SubtaskTimerInit(&camera_yaw_timer, CAMERA_YAW_FREQ, now_us);
  Task_SubtaskTimerInit(&arm_simple_timer, ARM_SIMPLE_FREQ, now_us);
  Task_SubtaskTimerInit(&rod_new_timer, ROD_FREQ, now_us);

  uint32_t tick = osKernelGetTickCount();
  const uint32_t camera_yaw_init_tick = tick + CAMERA_YAW_INIT_DELAY;
  const uint32_t arm_simple_init_tick = tick + ARM_SIMPLE_INIT_DELAY;
  const uint32_t rod_new_init_tick = tick + ROD_INIT_DELAY;
  bool camera_yaw_init_due = (CAMERA_YAW_INIT_DELAY == 0U);
  bool arm_simple_init_due = (ARM_SIMPLE_INIT_DELAY == 0U);
  bool rod_new_init_due = (ROD_INIT_DELAY == 0U);
  bool camera_yaw_ready = false;
  bool arm_simple_ready = false;
  bool rod_new_ready = false;

  while (1) {
    const uint32_t profile_start_us =
        Task_ProfilerLoopBegin(TASK_PROFILE_UPPER_MECH,
                               TASK_PERIOD_US(TASK_FUSED_LOOP_FREQ));
    tick += base_delay_tick;
    const uint32_t now_tick = osKernelGetTickCount();
    const uint32_t loop_now_us = (uint32_t)BSP_TIME_Get_us();

    if (!camera_yaw_init_due &&
        (int32_t)(now_tick - camera_yaw_init_tick) >= 0) {
      camera_yaw_init_due = true;
    }
    if (camera_yaw_init_due && !camera_yaw_ready) {
      camera_yaw_ready = Task_CameraYawInitOnce();
    }
    if (camera_yaw_ready &&
        Task_SubtaskTimerDue(&camera_yaw_timer, loop_now_us)) {
      Task_CameraYawStep();
    }

    if (!arm_simple_init_due &&
        (int32_t)(now_tick - arm_simple_init_tick) >= 0) {
      arm_simple_init_due = true;
    }
    if (arm_simple_init_due && !arm_simple_ready) {
      arm_simple_ready = Task_ArmSimpleInitOnce();
    }
    if (arm_simple_ready &&
        Task_SubtaskTimerDue(&arm_simple_timer, loop_now_us)) {
      Task_ArmSimpleStep();
    }

    if (!rod_new_init_due &&
        (int32_t)(now_tick - rod_new_init_tick) >= 0) {
      rod_new_init_due = true;
    }
    if (rod_new_init_due && !rod_new_ready) {
      rod_new_ready = Task_RodNewInitOnce();
    }
    if (rod_new_ready && Task_SubtaskTimerDue(&rod_new_timer, loop_now_us)) {
      Task_RodNewStep();
    }

    task_runtime.stack_water_mark.upper_mech =
        uxTaskGetStackHighWaterMark(NULL);
    task_runtime.heartbeat.upper_mech++;
    Task_ProfilerLoopEnd(TASK_PROFILE_UPPER_MECH, profile_start_us);
    Task_DelayUntil(TASK_PROFILE_UPPER_MECH, &tick, base_delay_tick);
  }
}
