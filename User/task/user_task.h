#pragma once

/* Includes ----------------------------------------------------------------- */
#include <cmsis_os2.h>
#include "FreeRTOS.h"
#include "task.h"

/* USER INCLUDE BEGIN */
#include <stdbool.h>
#include "module/autoCtrlAPI/api/auto_ctrl_api.h"
#include "device/buzzer.h"

/* USER INCLUDE END */

#ifdef __cplusplus
extern "C" {
#endif
/* Exported constants ------------------------------------------------------- */
/* 任务运行频率 */
#define BLINK_FREQ (500.0)
#define ATTI_ESTI_FREQ (500.0)
#define CHASSIS_MAIN_FREQ (500.0)
#define RC_MAIN_FREQ (500.0)
#define CMD_MAIN_FREQ (500.0)
#define SICK_FREQ (500.0)
#define AUTO_CTRL_FREQ (500.0)
#define ARM_FREQ (500.0)
#define ROD_FREQ (500.0)
#define PC_UART_RX_FREQ (500.0)
/* 任务初始化延时ms */
#define TASK_INIT_DELAY (100u)
#define BLINK_INIT_DELAY (0)
#define ATTI_ESTI_INIT_DELAY (0)
#define CHASSIS_MAIN_INIT_DELAY (0)
#define RC_MAIN_INIT_DELAY (0)
#define CMD_MAIN_INIT_DELAY (0)
#define SICK_INIT_DELAY (0)
#define AUTO_CTRL_INIT_DELAY (0)
#define ARM_INIT_DELAY (0)
#define ROD_INIT_DELAY (0)
#define PC_UART_RX_INIT_DELAY (0)
/* Exported defines --------------------------------------------------------- */
/* Exported macro ----------------------------------------------------------- */
/* Exported types ----------------------------------------------------------- */

/* 任务运行时结构体 */
typedef struct {
    /* 各任务，也可以叫做线程 */
    struct {
        osThreadId_t blink;
        osThreadId_t atti_esti;
        osThreadId_t chassis_main;
        osThreadId_t rc_main;
        osThreadId_t cmd_main;
        osThreadId_t sick;
        osThreadId_t auto_ctrl;
        osThreadId_t arm;
        osThreadId_t rod;
        osThreadId_t pc_uart_rx;

    } thread;

    /* USER MESSAGE BEGIN */
    struct {
        osMessageQueueId_t user_msg; /* 用户自定义任务消息队列 */

		struct {
            osMessageQueueId_t imu;
            osMessageQueueId_t cmd;
            osMessageQueueId_t yaw;
        }chassis;
        struct {
            osMessageQueueId_t cmd;
        } pole;
		    struct{
            osMessageQueueId_t imu;
			      osMessageQueueId_t rc; 

        }cmd;
        struct {
            osMessageQueueId_t cmd;
        } arm;
        struct {
            osMessageQueueId_t cmd;
        } rod;
    } msgq;
    /* USER MESSAGE END */

    /* 机器人状态 */
    struct {
        float battery; /* 电池电量百分比 */
        float vbat; /* 电池电压 */
        float cpu_temp; /* CPU温度 */
    } status;

    /* USER CONFIG BEGIN */

    /* USER CONFIG END */

    /* 各任务的stack使用 */
    struct {
        UBaseType_t blink;
        UBaseType_t atti_esti;
        UBaseType_t chassis_main;
        UBaseType_t rc_main;
        UBaseType_t cmd_main;
        UBaseType_t sick;
        UBaseType_t auto_ctrl;
        UBaseType_t arm;
        UBaseType_t rod;
        UBaseType_t pc_uart_rx;

    } stack_water_mark;

    /* 各任务运行频率 */
    struct {
        float blink;
        float atti_esti;
        float chassis_main;
        float rc_main;
        float cmd_main;
        float sick;
        float auto_ctrl;
        float pc_uart_rx;
    } freq;

    /* 任务最近运行时间 */
    struct {
        float blink;
        float atti_esti;
        float chassis_main;
        float rc_main;
        float cmd_main;
        float sick;
        float auto_ctrl;
        float pc_uart_rx;
    } last_up_time;

} Task_Runtime_t;

/* 任务运行时结构体 */
extern Task_Runtime_t task_runtime;

extern auto_ctrl_t auto_ctrl;
extern bool auto_ctrl_inited;
extern BUZZER_t buzzer;
extern bool g_buzzer_calib_active;

/* 初始化任务句柄 */
extern const osThreadAttr_t attr_init;
extern const osThreadAttr_t attr_blink;
extern const osThreadAttr_t attr_atti_esti;
extern const osThreadAttr_t attr_chassis_main;
extern const osThreadAttr_t attr_rc_main;
extern const osThreadAttr_t attr_cmd_main;
extern const osThreadAttr_t attr_sick;
extern const osThreadAttr_t attr_auto_ctrl;
extern const osThreadAttr_t attr_arm;
extern const osThreadAttr_t attr_rod;
extern const osThreadAttr_t attr_pc_uart_rx;
/* 任务函数声明 */
void Task_Init(void *argument);
void Task_blink(void *argument);
void Task_atti_esti(void *argument);
void Task_chassis_main(void *argument);
void Task_rc_main(void *argument);
void Task_cmd_main(void *argument);
void Task_sick(void *argument);
void Task_auto_ctrl(void *argument);
void Task_arm(void *argument);
void Task_rod(void *argument);
void Task_pc_uart_rx(void *argument);
#ifdef __cplusplus
}
#endif
