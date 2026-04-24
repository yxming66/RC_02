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
      [0] = {.can = BSP_CAN_2, .id = 0x201, .module = MOTOR_M3508, .reverse = false, .gear = true},
      [1] = {.can = BSP_CAN_2, .id = 0x202, .module = MOTOR_M3508, .reverse = false, .gear = true},
      [2] = {.can = BSP_CAN_2, .id = 0x203, .module = MOTOR_M3508, .reverse = false, .gear = true},
      [3] = {.can = BSP_CAN_2, .id = 0x204, .module = MOTOR_M3508, .reverse = false, .gear = true},
    },
    .pid = {
      .follow_pid_param = {
        .k = 1.0f,
        .p = 3.8f,
        .i = 0.10f,
        .d = 0.08f,
        .i_limit = 1.5f,
        .out_limit = 5.2f,
        .d_cutoff_freq = 50.0f,
        .range = 0.0f,
      },
      .motor_pid_param = {
        .k = 2.2f,
        .p = 1.8f,
        .i = 0.24f,
        .d = 0.01f,
        .i_limit = 2.0f,
        .out_limit = 6.0f,
        .d_cutoff_freq = 35.0f,
        .range = 0.0f,
      },
    },
    .physical = {
      .wheel_radius_m = 0.076f,
      .wheelbase_m = 0.23f,
      .trackwidth_m = 0.325f,
      .wheel_output_max_speed = 5.0f,
    },
      .low_pass_cutoff_freq = {
      .in = 20.0f,
      .out = 24.0f,
    },
      .limit = {
          .max_torque_cmd = 6.0f,
          .max_vx = 6.0f,
          .max_vy = 6.0f,
          .max_wz = 6.28f,
      },
      .type = CHASSIS_TYPE_MECANUM,
  },
.pole_param = {
        .motor_param = {
            [0] = {.can = BSP_CAN_1, .id = 0x201, .module = MOTOR_M3508, .reverse = false, .gear = true},
            [1] = {.can = BSP_CAN_1, .id = 0x202, .module = MOTOR_M3508, .reverse = true, .gear = true},
            [2] = {.can = BSP_CAN_1, .id = 0x203, .module = MOTOR_M3508, .reverse = false, .gear = true},
            [3] = {.can = BSP_CAN_1, .id = 0x204, .module = MOTOR_M3508, .reverse = true, .gear = true},
        },
        .pid = {
            .support_pos_pid = {
                .k = 25.0f,
                .p = 10.0f,
                .i = 0.0f,
                .d = 0.0f,
                .i_limit = 0.15f,
                .out_limit = 425.0f,
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
        },
            .preset = {
              .step_200_all_extend = {12.7912035f, 12.7912035f},      // 200mm台阶，右拨杆UP：前两杆/后两杆
              .step_200_front_retract = {0.0f, 12.7912035f},   // 200mm台阶，右拨杆MID：前两杆收，后两杆位置
              .step_200_all_retract = {0.0f, 0.0f},     // 200mm台阶，右拨杆DOWN：前两杆/后两杆
              .step_400_all_extend = {26.0381184f, 26.0315285f},      // 400mm台阶，右拨杆UP：前两杆/后两杆
              .step_400_front_retract = {0.0f, 26.0381184f},   // 400mm台阶，右拨杆MID：前两杆收，后两杆位置
              .step_400_all_retract = {0.0f, 0.0f},     // 400mm台阶，右拨杆DOWN：前两杆/后两杆
            },
        .limit = {
            .max_current = 1.0f,
            .support_total_travel = 26.0f,
            .support_lift_speed = 50.0f,
        },
    },
    .auto_ctrl_param = {
      /* 通用姿态修正参数 */
      .prealign_kp = 4.0f,              // yaw误差到wz指令的比例系数，越大修正越激进
      .prealign_wz_limit = 1.5f,         // 自动矫正时wz最大输出限幅 (rad/s)

      /* 简单平移模板参数 */
      .flat_move_speed = 0.25f,          // 简单平移模板默认前进速度 (m/s)
      .flat_move_hold_ms = 600u,         // 简单平移模板默认保持时间 (ms)

      /* 200台阶/跨越前段参数 */ 
      .climb_align_forward_speed = 0.30f, // 对正阶段同步慢速前进速度 (m/s)
        .climb_pole_extend_forward_speed = 0.50f, // 对正成功后，四杆伸起稳定阶段前进速度 (m/s)
      .climb_forward_speed = 1.0f,      // 前段并行动作时的基础前进速度 (m/s)
      .climb_forward_kick_speed = 0.50f, // 起步短促前冲速度，用于增加跨越动量 (m/s)
      .climb_forward_kick_ms = 80u,      // 起步短促前冲保持时间 (ms) 
      .pole_extend_settle_ms = 900u,     // 四杆伸出后等待机构稳定的时间 (ms)
      .pole_all_extend_lift_speed = 50.0f, // 四杆全伸阶段速度 (rad/s)
      .pole_front_retract_lift_speed = 18.0f, // 前杆回收阶段速度 (rad/s)
      .pole_front_extend_lift_speed = 20.0f, // 前杆放下/重新伸出阶段速度 (rad/s)
      .pole_rear_retract_lift_speed = 30.0f, // 后杆回收阶段速度 (rad/s)
      .pole_rear_extend_lift_speed = 18.0f, // 后杆放下/重新伸出阶段速度 (rad/s)
      .head_front_photo_timeout_ms =5000u,   // 头向前光电等待超时时间 (ms)
      .climb_front_retract_speed = 0.50f, // 前杆回收阶段基础前进速度 (m/s)
      .climb_front_retract_vy = 0.04f,    // 前杆回收阶段附加横移速度vy (m/s)
      .climb_front_retract_timeout_ms = 5000u, // 前杆回收阶段最长等待时间 (ms)
      .front_retract_settle_ms = 800u,   // 前杆回收后的稳定等待时间 (ms)

      /* 200台阶/跨越中后段参数 */
      .climb_mid_forward_speed = 0.5f,   // 前杆收回后，中段继续前进速度 (m/s)
      .climb_mid_forward_ms = 1500u,       // 前杆收回后，中段继续纯前进的保持时间 (ms)
      .climb_rear_retract_speed = 0.20f, // 后杆回收阶段的基础前进速度 (m/s)
      .climb_rear_retract_vy = 0.15f,    // 后杆回收阶段附加横移速度vy，便于边收杆边修正姿态 (m/s)
      .head_rear_photo_timeout_ms = 10000u,    // 头向后光电等待超时时间 (ms)
      .climb_rear_retract_timeout_ms = 5000u, // 后杆回收阶段最长等待时间 (ms)
      .rear_retract_move_ms = 700u,      // 后杆回收完成后继续保持运动的时间 (ms)

      /* 200下台阶逆流程参数 */
      .descend_200_align_speed = 0.30f,             // 下台阶对正阶段速度 (m/s)
      .descend_200_climb_forward_speed = 0.50f,     // 下台阶跨越时前进速度 (m/s)
      .descend_200_pole_extend_settle_ms = 900u,    // 下台阶撑杆伸出稳定时间 (ms)
      .descend_200_head_rear_photo_timeout_ms = 5000u, /* 下台阶头向后光电超时 (ms) */
      .descend_200_climb_rear_retract_speed = 0.20f,      // 下台阶后杆回收速度 (m/s)
      .descend_200_climb_rear_retract_timeout_ms = 5000u, // 下台阶后杆回收超时 (ms)
      .descend_200_rear_retract_move_ms = 700u,           // 下台阶后杆回收后移动时长 (ms)
      .descend_200_climb_front_retract_speed = 0.50f,     // 下台阶前杆回收速度 (m/s)
      .descend_200_climb_front_retract_timeout_ms = 5000u, /* 下台阶前杆回收超时 (ms) */
      .descend_200_front_retract_settle_ms = 800u,     // 下台阶前杆回收后稳定时长 (ms)
      .descend_200_flat_move_ms = 600u,                // 下台阶脱离台阶平移时长 (ms)
      .descend_200_pole_all_extend_lift_speed = 50.0f,     // 下台阶四杆全伸阶段速度 (rad/s)
      .descend_200_pole_rear_retract_lift_speed = 30.0f,  // 下台阶后杆回收阶段速度 (rad/s)
      .descend_200_pole_front_retract_lift_speed = 18.0f, // 下台阶前杆回收阶段速度 (rad/s)

      /* 400上台阶逆流程参数 */
      .ascend_400_align_forward_speed = 0.30f,  // 上台阶对正阶段同步前进速度 (m/s)
      .ascend_400_pole_extend_forward_speed = 0.30f, // 上台阶四杆伸起稳定阶段前进速度 (m/s)
      .ascend_400_forward_speed = 0.40f,       // 上台阶前杆回收阶段基础前进速度 (m/s)
      .ascend_400_pole_extend_settle_ms = 1000u, // 上台阶四杆伸出后稳定时间 (ms)
      .ascend_400_pole_all_extend_lift_speed = 40.0f, // 上台阶四杆全伸阶段速度 (rad/s)
      .ascend_400_head_front_photo_timeout_ms = 6000u, /* 上台阶头向前光电超时 (ms) */
      .ascend_400_front_retract_speed = 0.25f,  // 上台阶前杆回收速度 (m/s)
      .ascend_400_front_retract_vy = 0.15f,     // 上台阶前杆回收附加横移速度vy (m/s)
      .ascend_400_front_retract_timeout_ms = 5000u, // 上台阶前杆回收超时 (ms)
      .ascend_400_front_retract_settle_ms = 800u, // 上台阶前杆回收后稳定时间 (ms)
      .ascend_400_mid_forward_speed = 0.50f,     // 上台阶前杆收回后中段前进速度 (m/s)
      .ascend_400_mid_forward_ms = 1500u,        // 上台阶中段前进保持时间 (ms)
      .ascend_400_rear_retract_speed = 0.20f,  // 上台阶后杆回收速度 (m/s)
      .ascend_400_rear_retract_vy = 0.15f,     // 上台阶后杆回收附加横移速度vy (m/s)
      .ascend_400_head_rear_photo_timeout_ms = 6000u, /* 上台阶头向后光电超时 (ms) */
      .ascend_400_rear_retract_timeout_ms = 5000u, // 上台阶后杆回收超时 (ms)
      .ascend_400_rear_retract_move_ms = 800u,  // 上台阶后杆回收后移动时长 (ms)
      .ascend_400_pole_rear_retract_lift_speed = 30.0f, // 上台阶后杆回收阶段速度 (rad/s)
      .ascend_400_pole_front_retract_lift_speed = 18.0f, // 上台阶前杆回收阶段速度 (rad/s)

      /* 400下台阶逆流程参数 */
      .descend_400_align_speed = 0.30f,             // 下台阶对正阶段速度 (m/s)
      .descend_400_climb_forward_speed = 0.40f,     // 下台阶跨越时前进速度 (m/s)
      .descend_400_pole_extend_settle_ms = 1200u,  // 下台阶撑杆伸出稳定时间 (ms)
      .descend_400_head_rear_photo_timeout_ms = 6000u, /* 下台阶头向后光电超时 (ms) */
      .descend_400_climb_rear_retract_speed = 0.15f,     // 下台阶后杆回收速度 (m/s)
      .descend_400_climb_rear_retract_timeout_ms = 6000u, // 下台阶后杆回收超时 (ms)
      .descend_400_rear_retract_move_ms = 800u,           // 下台阶后杆回收后移动时长 (ms)
      .descend_400_climb_front_retract_speed = 0.40f,     // 下台阶前杆回收速度 (m/s)
      .descend_400_climb_front_retract_timeout_ms = 6000u, /* 下台阶前杆回收超时 (ms) */
      .descend_400_front_retract_settle_ms = 1000u,     // 下台阶前杆回收后稳定时长 (ms)
      .descend_400_flat_move_ms = 800u,                // 下台阶脱离台阶平移时长 (ms)
      .descend_400_pole_all_extend_lift_speed = 40.0f,     // 下台阶四杆全伸阶段速度 (rad/s)
      .descend_400_pole_rear_retract_lift_speed = 25.0f,  // 下台阶后杆回收阶段速度 (rad/s)
      .descend_400_pole_front_retract_lift_speed = 15.0f, // 下台阶前杆回收阶段速度 (rad/s)

      /* SICK辅助姿态修正参数 */
      .sick_valid_min_cm = 0.0f,         // SICK测距判定为有效的最小值 (cm)
      .sick_valid_max_cm = 2.0f,       // SICK测距判定为有效的最大值 (cm)
      .sick_norm_err_deadband = 0.03f,   // 左右SICK归一化差分误差死区 (无量纲)
      .sick_norm_err_to_rad = 0.34906585f, // 归一化误差映射到姿态辅助量的比例 (rad/ratio)
      .sick_assist_gain = 0.75f,         // SICK辅助误差融合增益，越大修正越积极
      .sick_assist_max_rad = 0.13962634f, // SICK辅助误差的限幅上限 (rad)
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
            .pit_motor_param = {.can = BSP_CAN_1, .master_id = 0x13, .can_id = 0x03, .module = MOTOR_DM_J4310, .reverse = false},
            .rol_motor_param = {.can = BSP_CAN_1, .master_id = 0x14, .can_id = 0x04, .module = MOTOR_DM_J4310, .reverse = false},
            .pose = {
              .pit_down_angle = 0.267107785f + 0.0f,
              .pit_up_angle = 0.107107785f + 1.6465615f,
              .pit_grip_angle = 0.267107785f + 1.7f,
              .pit_lift_angle = 0.267107785f + 2.0f,
              .rol_home_angle = 1.67915916f + 0.0f,
              .rol_flip_angle = 1.67915916f + 3.1415926f,
            },
            .limit = {
              .pit_arrive_threshold = 0.03f,
              .rol_arrive_threshold = 0.03f,
              .sequence_timeout = 2.0f,
              .grip_open_wait_time = 1.0f,
              .grip_wait_time = 0.25f,
              .rol_flip_wait_time = 1.0f,
              .pit_kp = 80.0f,
              .pit_kd = 2.0f,
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
