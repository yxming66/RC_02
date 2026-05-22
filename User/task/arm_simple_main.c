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

/* Private variables --------------------------------------------------------- */
static ArmSimple_t arm_simple;
static bool set_zero_requested = false;

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

    MOTOR_DM_Enable(&arm_simple.param->dm4340_param);

    while (1) {
        tick += delay_tick;

        /* 处理零位设置请求 */
        if (set_zero_requested) {
            MOTOR_DM_SetZero(&arm_simple.param->dm4340_param);
            set_zero_requested = false;
        }

        /* 更新反馈 */
        ArmSimple_UpdateFeedback(&arm_simple);

        /* 观察反馈阶段：只持续发送MIT零输出帧 */
        ArmSimple_Output(&arm_simple);

        osDelayUntil(tick);
    }
}

/* 设置零位（在主循环外调用） */
void ArmSimple_SetZero(void)
{
    set_zero_requested = true;
}
