/*
    Arm Simple Task
    控制DM4310关节1 + JS6660舵机关节2 + 电磁阀吸盘
*/

/* Includes ----------------------------------------------------------------- */
#include "cmsis_os2.h"
#include "task/user_task.h"
#include "module/arm_simple.h"
#include "module/config.h"

#define ARM_SIMPLE_FREQ (250.0f)
#define ARM_SIMPLE_INIT_DELAY (100u)
#define ARM_SIMPLE_ARRIVE_THRESHOLD (0.05f)

/* Private variables --------------------------------------------------------- */
static ArmSimple_t arm_simple;
static ArmSimple_CMD_t arm_simple_cmd;
static bool set_zero_requested = false;

/* Private function --------------------------------------------------------- */
static bool ArmSimpleModeIsValid(ArmSimple_Mode_t mode)
{
    return (mode >= ARM_SIMPLE_MODE_RELAX) && (mode <= ARM_SIMPLE_MODE_POS_VEL);
}

static bool ArmSimplePointModeIsValid(ArmSimple_PointMode_t mode)
{
    return (mode >= ARM_SIMPLE_POINT_SLEEP) && (mode < ARM_SIMPLE_POINT_NONE);
}

/* Exported functions ------------------------------------------------------- */
void Task_arm_simple(void *argument)
{
    (void)argument;

    const uint32_t delay_tick = osKernelGetTickFreq() / ARM_SIMPLE_FREQ;

    osDelay(ARM_SIMPLE_INIT_DELAY);

    uint32_t tick = osKernelGetTickCount();

    /* 初始化臂 */
    ArmSimple_Init(&arm_simple, &Config_GetRobotParam()->arm_simple_param, ARM_SIMPLE_FREQ);

    arm_simple_cmd.mode = ARM_SIMPLE_MODE_RELAX;
    arm_simple_cmd.point_mode = ARM_SIMPLE_POINT_SLEEP;
    arm_simple_cmd.suction = SUCTION_OFF;

    while (1) {
        tick += delay_tick;

        /* 处理零位设置请求 */
        if (set_zero_requested) {
            MOTOR_DM_SetZero(&arm_simple.param->dm4310_param);
            set_zero_requested = false;
        }

        /* 从消息队列获取命令 */
        if (osMessageQueueGet(task_runtime.msgq.arm_simple.cmd, &arm_simple_cmd, NULL, 0) == osOK) {
            if (!ArmSimpleModeIsValid(arm_simple_cmd.mode)) {
                arm_simple_cmd.mode = ARM_SIMPLE_MODE_RELAX;
            }
            if (!ArmSimplePointModeIsValid(arm_simple_cmd.point_mode)) {
                arm_simple_cmd.point_mode = ARM_SIMPLE_POINT_SLEEP;
            }
        }

        /* 更新反馈 */
        ArmSimple_UpdateFeedback(&arm_simple);

        /* 控制 */
        ArmSimple_Control(&arm_simple, &arm_simple_cmd);

        /* 输出 */
        ArmSimple_Output(&arm_simple);

        osDelayUntil(tick);
    }
}

/* 设置零位（在主循环外调用） */
void ArmSimple_SetZero(void)
{
    set_zero_requested = true;
}
