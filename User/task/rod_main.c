/*
 * Rod control task.
 */

#include "task/user_task.h"

#include "module/config.h"
#include "module/rod_new.h"
#include "module/shared_valve.h"




static RodNew_t rod_new;
static RodNew_CMD_t rod_new_cmd;
static RodNew_Feedback_t rod_new_feedback;

volatile RodNew_DebugControl_t g_rod_new_debug = {
  false,
  false,
  BSP_PWM_ROD_SERVO,
  0.0f,
  ROD_NEW_SERVO_PULSE_NEUTRAL_US,
  ROD_NEW_GRIP_RELEASE,
};

const RodNew_Feedback_t *Task_RodNewGetFeedback(void) {
  return &rod_new_feedback;
}

void Task_rod(void *argument) {
  (void)argument;
 
  const uint32_t delay_tick = osKernelGetTickFreq() / ROD_FREQ;
  osDelay(ROD_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  Config_RobotParam_t *cfg = Config_GetRobotParam();
  if (cfg == NULL ||
      RodNew_Init(&rod_new, &cfg->rod_new_param, ROD_FREQ) != ROD_NEW_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }

  rod_new_cmd.mode = ROD_NEW_MODE_RELAX;
  rod_new_cmd.pose = ROD_NEW_POSE_DOCK_WAIT;
  rod_new_cmd.grip = ROD_NEW_GRIP_RELEASE;

  while (1) {  
    const uint32_t profile_start_us =
        Task_ProfilerLoopBegin(TASK_PROFILE_ROD, TASK_PERIOD_US(ROD_FREQ));
    tick += delay_tick;

    osMessageQueueGet(task_runtime.msgq.rod.cmd, &rod_new_cmd, NULL, 0);

    if (!g_rod_new_debug.enable) {
      const uint32_t now_ms = BSP_TIME_Get_ms();
      RodNew_Control(&rod_new, rod_new_cmd.mode, rod_new_cmd.pose,
                     rod_new_cmd.grip, rod_new_cmd.target_angle_rad,
                     now_ms);
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
    task_runtime.heartbeat.rod++;
    Task_ProfilerLoopEnd(TASK_PROFILE_ROD, profile_start_us);
    Task_DelayUntil(TASK_PROFILE_ROD, &tick, delay_tick);
  }
}
