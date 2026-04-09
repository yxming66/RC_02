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
            [4] = {.can = BSP_CAN_2, .id = 0x205, .module = MOTOR_M2006, .reverse = false, .gear = true},
            [5] = {.can = BSP_CAN_2, .id = 0x206, .module = MOTOR_M2006, .reverse = false, .gear = true},
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
                .k = 0.1f,
                .p = 0.1f,
                .i = 0.0f,
                .d = 0.0f,
                .i_limit = 0.0f,
                .out_limit = 1.0f,
                .d_cutoff_freq = 50.0f,
                .range = 0.0f,
            },
        },
        .limit = {
            .max_current = 1.0f,
            .support_total_travel = 19.0f,
            .support_lift_speed = 8.0f,
            .drive_enable_angle = 0.5f,
            .drive_max_rpm = 200.0f,
        },
    },
    .arm_param = {
      .joint3_cali = {
        .mode = ARM_CALI_MODE_ZERO_LIMIT_TRAVEL,
        .seek_velocity_rad_per_sec = 0.8f,
        .seek_positive_direction = false,
        .user_travel_rad = 1.57f,
      },
        .lzmotor_param = {.can = BSP_CAN_1, .motor_id = 127, .host_id = 0xff, .module = MOTOR_LZ_RSO3, .reverse = true,   .mode=MOTOR_LZ_MODE_MOTION},
        .dmmotor_param = {.can = BSP_CAN_1, .master_id = 0x12,  .can_id = 0x02, .module = MOTOR_DM_J4310, .reverse = false},
        .rmmotor_param = {.can = BSP_CAN_1, .id = 0x205, .module = MOTOR_M3508, .reverse = false, .gear = true},
            .pid_rmmotor_pos = {
                .k = 1.0f,
                .p = 1.0f,
                .i = 0.0f,
                .d = 0.0f,
                .i_limit = 0.0f,
                .out_limit = 1.0f,
                .d_cutoff_freq = -1.0f,
                .range = 0.0f,
            },
            .pid_rmmotor_vel = {
                .k = 0.1f,
                .p = 0.1f,
                .i = 0.0f,
                .d = 0.0f,
                .i_limit = 0.0f,
                .out_limit = 1.0f,
                .d_cutoff_freq = -1.0f,
                .range = 0.0f,
            },
    },
          .rod_param = {
            .pit_motor_param = {.can = BSP_CAN_1, .master_id = 0x14, .can_id = 0x04, .module = MOTOR_DM_J4310, .reverse = true},
            .rol_motor_param = {.can = BSP_CAN_1, .master_id = 0x13, .can_id = 0x03, .module = MOTOR_DM_J4310, .reverse = false},
            .pose = {
              .pit_down_angle = 0.0f,
              .pit_up_angle = 1.7f,
              .rol_home_angle = 0.0f,
              .rol_flip_angle = 3.1415926f,
            },
            .limit = {
              .pit_arrive_threshold = 0.03f,
              .rol_arrive_threshold = 0.03f,
              .sequence_timeout = 2.0f,
              .grip_wait_time = 0.25f,
              .pit_kp = 30.0f,
              .pit_kd = 0.1f,
              .rol_kp = 15.0f,
              .rol_kd = 0.05f,
              .pit_max_vel = 1.5f,
              .pit_max_acc = 1.0f,
              .rol_max_vel = 1.8f,
              .rol_max_acc = 1.5f,
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
