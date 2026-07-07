/*
    Arm Simple Task
    控制DM4340关节1 + JS6660舵机关节2 + 电磁阀吸盘
*/

/* Includes ----------------------------------------------------------------- */
#include "cmsis_os2.h"
#include "device/motor_dm.h"
#include "task/user_task.h"
#include "module/arm_simple.h"
#include "module/config.h"
#include "bsp/time.h"
#include "bsp/gpio.h"
/* Private variables --------------------------------------------------------- */
static ArmSimple_t arm_simple;
static ArmSimple_Feedback_t arm_simple_feedback;
static bool arm_simple_init_attempted = false;
static bool arm_simple_inited = false;
static bool set_zero_requested = false;

volatile ArmSimple_DebugControl_t g_arm_simple_debug = {
    false,
    ARM_SIMPLE_MODE_RELAX,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    SUCTION_OFF,
};

/* Exported functions ------------------------------------------------------- */
const ArmSimple_Feedback_t *Task_ArmSimpleGetFeedback(void) {
    return &arm_simple_feedback;
}

/* 设置零位（在主循环外调用） */
bool Task_ArmSimpleInitOnce(void) {
    if (arm_simple_inited) {
        return true;
    }
    if (arm_simple_init_attempted) {
        return false;
    }
    arm_simple_init_attempted = true;

    Config_RobotParam_t *cfg = Config_GetRobotParam();
    if (cfg == NULL ||
        ArmSimple_Init(&arm_simple, &cfg->arm_simple_param,
                       ARM_SIMPLE_FREQ) != ARM_SIMPLE_OK) {
        return false;
    }

    for (uint8_t i = 0U; i < 50U; ++i) {
        if (ArmSimple_UpdateFeedback(&arm_simple) == ARM_SIMPLE_OK) {
            break;
        }
        osDelay(2U);
    }

    ArmSimple_CMD_t startup_cmd = {
        .mode = ARM_SIMPLE_MODE_JOINT,
        .point_mode = ARM_SIMPLE_POINT_NONE,
        .suction = SUCTION_OFF,
        .target_joint = {
            .joint1 = arm_simple.feedback.joint1_angle,
            .joint2 = 0.0f,
        },
        .joint1_vel = 0.0f,
    };
    ArmSimple_Control(&arm_simple, &startup_cmd);
    arm_simple_inited = true;
    return true;
}

void Task_ArmSimpleStep(void) {
    if (!arm_simple_inited) {
        return;
    }

    const uint32_t profile_start_us =
        Task_ProfilerLoopBegin(TASK_PROFILE_ARM_SIMPLE,
                               TASK_PERIOD_US(ARM_SIMPLE_FREQ));

    const uint64_t now_us = BSP_TIME_Get_us();
    const float nominal_dt = 1.0f / (float)ARM_SIMPLE_FREQ;
    const float min_dt = nominal_dt * 0.5f;
    const float max_dt = nominal_dt * 3.0f;
    if (arm_simple.timer.last_wakeup_us != 0U &&
        now_us > arm_simple.timer.last_wakeup_us) {
        arm_simple.timer.dt =
            (float)(now_us - arm_simple.timer.last_wakeup_us) * 0.000001f;
    } else {
        arm_simple.timer.dt = nominal_dt;
    }
    if (arm_simple.timer.dt < min_dt || arm_simple.timer.dt > max_dt) {
        arm_simple.timer.dt = nominal_dt;
    }
    arm_simple.timer.now = (float)now_us * 0.000001f;
    arm_simple.timer.last_wakeup_us = now_us;

    ArmSimple_UpdateFeedback(&arm_simple);
    arm_simple_feedback.mode = arm_simple.mode;
    arm_simple_feedback.point_mode = arm_simple.cmd.point_mode;
    arm_simple_feedback.suction = arm_simple.suction;
    arm_simple_feedback.joint1_temperature_warning =
        arm_simple.feedback.joint1_temperature_warning;
    arm_simple_feedback.joint1_temperature_over_limit =
        arm_simple.feedback.joint1_temperature_over_limit;
    arm_simple_feedback.joint1_temperature_limit_latched =
        arm_simple.feedback.joint1_temperature_limit_latched;
    arm_simple_feedback.joint1_angle_rad = arm_simple.feedback.joint1_angle;
    arm_simple_feedback.joint1_velocity_rad_s = arm_simple.feedback.joint1_vel;
    arm_simple_feedback.joint1_temperature_c = arm_simple.feedback.joint1_temp;
    arm_simple_feedback.joint2_angle_rad = arm_simple.feedback.joint2_angle;
    arm_simple_feedback.target_joint1_rad = arm_simple.target.joint1_target;
    arm_simple_feedback.target_joint2_rad = arm_simple.target.joint2_target;
    arm_simple_feedback.output_target_joint1_rad =
        arm_simple.target.joint1_output_target;
    arm_simple_feedback.output_target_joint2_rad =
        arm_simple.target.joint2_output_target;
    arm_simple_feedback.joint1_max_vel_rad_s =
        arm_simple.target.joint1_max_vel_rad_s;
    arm_simple_feedback.joint2_max_vel_rad_s =
        arm_simple.target.joint2_max_vel_rad_s;

    ArmSimple_CMD_t cmd;
    if (LatestSlot_ReadIfUpdated(&task_runtime.latest.arm_simple_cmd.slot,
                                 &cmd, sizeof(cmd),
                                 &task_runtime.latest.arm_simple_cmd.read_seq)) {
        ArmSimple_Control(&arm_simple, &cmd);
    }

    if (g_arm_simple_debug.enable) {
        ArmSimple_CMD_t debug_cmd = {
            .mode = g_arm_simple_debug.mode,
            .point_mode = ARM_SIMPLE_POINT_NONE,
            .suction = g_arm_simple_debug.suction,
            .target_joint = {
                .joint1 = g_arm_simple_debug.target_joint1_rad,
                .joint2 = g_arm_simple_debug.target_joint2_rad,
            },
            .joint1_vel = 0.0f,
            .joint1_max_vel_rad_s = g_arm_simple_debug.joint1_max_vel_rad_s,
            .joint2_max_vel_rad_s = g_arm_simple_debug.joint2_max_vel_rad_s,
        };
        ArmSimple_Control(&arm_simple, &debug_cmd);
    }

    if (set_zero_requested) {
        (void)ArmSimple_SetJoint1Zero(&arm_simple);
        set_zero_requested = false;
    }

    ArmSimple_Output(&arm_simple);

    task_runtime.heartbeat.arm_simple++;
    Task_ProfilerLoopEnd(TASK_PROFILE_ARM_SIMPLE, profile_start_us);
}

void Task_arm_simple(void *argument) {
    (void)argument;

    uint32_t delay_tick = osKernelGetTickFreq() / ARM_SIMPLE_FREQ;
    if (delay_tick == 0U) {
        delay_tick = 1U;
    }

    osDelay(ARM_SIMPLE_INIT_DELAY);

    uint32_t tick = osKernelGetTickCount();
    if (!Task_ArmSimpleInitOnce()) {
        osThreadTerminate(osThreadGetId());
        return;
    }

    while (1) {
        tick += delay_tick;
        Task_ArmSimpleStep();
        Task_DelayUntil(TASK_PROFILE_ARM_SIMPLE, &tick, delay_tick);
    }
}

void ArmSimple_SetZero(void)
{
    set_zero_requested = true;
}
