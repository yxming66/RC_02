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
static bool set_zero_requested = false;

volatile ArmSimple_DebugControl_t g_arm_simple_debug = {
    false,
    ARM_SIMPLE_MODE_RELAX,
    0.0f,
    0.0f,
    SUCTION_OFF,
};

/* Exported functions ------------------------------------------------------- */
void Task_arm_simple(void *argument)
{
    (void)argument;

    const uint32_t delay_tick = osKernelGetTickFreq() / ARM_SIMPLE_FREQ;

    osDelay(ARM_SIMPLE_INIT_DELAY);

    uint32_t tick = osKernelGetTickCount();

    /* 初始化臂 */
    if (ArmSimple_Init(&arm_simple, &Config_GetRobotParam()->arm_simple_param, ARM_SIMPLE_FREQ) != ARM_SIMPLE_OK) {
        osThreadTerminate(osThreadGetId());
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

    while (1) {
        tick += delay_tick;

        // BSP_GPIO_WritePin(BSP_GPIO_ARM_SOLENOID, true);
        // BSP_GPIO_WritePin(BSP_GPIO_ROD_SOLENOID, true);
        const uint64_t now_us = BSP_TIME_Get_us();
        arm_simple.timer.now = (float)now_us * 0.000001f;
        arm_simple.timer.last_wakeup_us = now_us;

        
        /* 先更新反馈，后续任何保持目标都使用最新电机角度 */
        ArmSimple_UpdateFeedback(&arm_simple);

        ArmSimple_CMD_t cmd;
        if (task_runtime.msgq.arm_simple.cmd != NULL &&
            osMessageQueueGet(task_runtime.msgq.arm_simple.cmd, &cmd, NULL, 0U) == osOK) {
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
            };
            ArmSimple_Control(&arm_simple, &debug_cmd);
        }

        /* 处理零位设置请求 */
        if (set_zero_requested) {
            (void)ArmSimple_SetJoint1Zero(&arm_simple);
            set_zero_requested = false;
        }

        ArmSimple_Output(&arm_simple);

        osDelayUntil(tick);
    }
}

/* 设置零位（在主循环外调用） */
void ArmSimple_SetZero(void)
{
    set_zero_requested = true;
}
