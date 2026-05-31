#include "task/user_task.h"

Task_Runtime_t task_runtime;
Buzzer_AlarmRequest_t g_buzzer_alarm_request;

const osThreadAttr_t attr_init = {
    .name = "Task_Init",
    .priority = osPriorityRealtime,
    .stack_size = 256 * 4,
};

/* User_task */
const osThreadAttr_t attr_blink = {
    .name = "blink",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_atti_esti = {
    .name = "atti_esti",
    .priority = osPriorityNormal,
    .stack_size = 512 * 4,
};
const osThreadAttr_t attr_chassis_main = {
    .name = "chassis_main",
    .priority = osPriorityAboveNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_rc_main = {
    .name = "rc_main",
    .priority = osPriorityNormal,
    .stack_size = 512 * 4,
};
const osThreadAttr_t attr_sick = {
    .name = "sick",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_auto_ctrl = {
    .name = "auto_ctrl",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_arm_simple = {
    .name = "arm_simple",
    .priority = osPriorityNormal,
    .stack_size = 512 * 4,
};
const osThreadAttr_t attr_rod = {
    .name = "rod",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_pc_comm = {
    .name = "pc_comm",
    .priority = osPriorityNormal,
    .stack_size = 512 * 4,
};
const osThreadAttr_t attr_ore_store = {
    .name = "ore_store",
    .priority = osPriorityNormal,
    .stack_size = 512 * 4,
};
