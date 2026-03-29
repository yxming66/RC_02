/*
 * 配置相关
 */

/* Includes ----------------------------------------------------------------- */
#include "module/config.h"
#include "bsp/can.h"
#include "device/motor_rm.h"
#include "module/cmd/cmd.h"
#include <stdbool.h>
/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */

/* Exported variables ------------------------------------------------------- */



// 机器人参数配置
Config_RobotParam_t robot_config = {
  .chassis_param = {
    .motor_param = {
      [0] = {.can = BSP_CAN_1, .id = 0x201, .module = MOTOR_M3508, .reverse = false, .gear = true},
      [1] = {.can = BSP_CAN_1, .id = 0x202, .module = MOTOR_M3508, .reverse = false, .gear = true},
      [2] = {.can = BSP_CAN_1, .id = 0x203, .module = MOTOR_M3508, .reverse = false, .gear = true},
      [3] = {.can = BSP_CAN_1, .id = 0x204, .module = MOTOR_M3508, .reverse = false, .gear = true},
    },
    .pid = {
      .follow_pid_param = {
        .k = 1.0f,
        .p = 0.0015f,
        .i = 0.0f,
        .d = 0.0f,
        .i_limit = 0.0f,
        .out_limit = 1.0f,
        .d_cutoff_freq = 50.0f,
        .range = 0.0f,
      },
      .motor_pid_param = {
        .k = 1.0f,
        .p = 0.0015f,
        .i = 0.0f,
        .d = 0.0f,
        .i_limit = 0.0f,
        .out_limit = 1.0f,
        .d_cutoff_freq = 50.0f,
        .range = 0.0f,
      },
    },
      .low_pass_cutoff_freq = {
      .in = {20.0f, 20.0f, 20.0f, 20.0f},
      .out = {20.0f, 20.0f, 20.0f, 20.0f},
    },
      .limit = {
          .max_current = 1.0f,
          .max_vx = 3.0f,
          .max_vy = 3.0f,
          .max_wz = 6.28f,
      },
      .type = CHASSIS_TYPE_MECANUM,
  },
	.pole_param = {
        .motor_param = {
            [0] = {.can = BSP_CAN_2, .id = 0x201, .module = MOTOR_M3508, .reverse = false, .gear = true},
            [1] = {.can = BSP_CAN_2, .id = 0x202, .module = MOTOR_M3508, .reverse = true, .gear = true},
            [2] = {.can = BSP_CAN_2, .id = 0x203, .module = MOTOR_M3508, .reverse = false, .gear = true},
            [3] = {.can = BSP_CAN_2, .id = 0x204, .module = MOTOR_M3508, .reverse = true, .gear = true},
            [4] = {.can = BSP_CAN_2, .id = 0x205, .module = MOTOR_M2006, .reverse = false, .gear = false},
            [5] = {.can = BSP_CAN_2, .id = 0x206, .module = MOTOR_M2006, .reverse = false, .gear = false},
        },
        .pid = {
            .support_pos_pid = {
                .k = 25.0f,
                .p = 10.0f,
                .i = 0.0f,
                .d = 0.0f,
                .i_limit = 0.15f,
                .out_limit = 150,0,
                .d_cutoff_freq = -1.0f,
                .range = 0.0f,
            },
              .support_vel_pid = {
                .k = 0.1f,
                .p = 0.1f,
                .i = 0.0f,
                .d = 0.0f,
                .i_limit = 0.0f,
                .out_limit = 1.0f,
                .d_cutoff_freq = -1.0f,
                .range = 0.0f,
            },
            .drive_spd_pid = {
                .k = 0.5f,
                .p = 0.1f,
                .i = 0.0f,
                .d = 0.0f,
                .i_limit = 0.0f,
                .out_limit = 0.3f,
                .d_cutoff_freq = 50.0f,
                .range = 0.0f,
            },
        },
        .limit = {
            .max_current = 1.0f,
            .support_total_travel = 19.0f,
            .support_lift_speed = 8.0f,
            .drive_enable_angle = 0.35f,
            .drive_max_rpm = 1000.0f,
        },
    },
};

/* Private function prototypes ---------------------------------------------- */
/* Exported functions ------------------------------------------------------- */

/**
 * @brief 获取机器人配置参数
 * @return 机器人配置参数指针
 */
Config_RobotParam_t* Config_GetRobotParam(void) {
    return &robot_config;
}
