#include "task/user_task.h"

Task_Runtime_t task_runtime;

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
const osThreadAttr_t attr_chassis_main = {
    .name = "chassis_main",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_rc_main = {
    .name = "rc_main",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_cmd_main = {
    .name = "cmd_main",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_sick = {
    .name = "sick",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_arm = {
    .name = "arm",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_rod = {
    .name = "rod",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};