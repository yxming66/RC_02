/*
 * Robot configuration.
 */

#include "module/config.h"

#include <stdbool.h>

#include "bsp/can.h"
#include "device/motor_dm.h"
#include "device/motor_lz.h"
#include "device/motor_rm.h"

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
            [0] = {.can = BSP_CAN_2, .id = 0x201, .module = MOTOR_M3508, .reverse = true, .gear = true},
            [1] = {.can = BSP_CAN_2, .id = 0x202, .module = MOTOR_M3508, .reverse = false, .gear = true},
            [2] = {.can = BSP_CAN_2, .id = 0x203, .module = MOTOR_M3508, .reverse = true, .gear = true},
            [3] = {.can = BSP_CAN_2, .id = 0x204, .module = MOTOR_M3508, .reverse = true, .gear = true},
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
            .step_200_all_extend = {13.3f, 13.3f},
            .step_200_front_retract = {0.0f, 13.3f},
            .step_200_all_retract = {0.0f, 0.0f},
            .step_200_small = {3.0f, 3.0f},
            .step_400_all_extend = {26.3f, 26.3f},
            .step_400_front_retract = {0.0f, 26.3f},
            .step_400_all_retract = {0.0f, 0.0f},
        },
        .limit = {
            .max_current = 1.0f,
            .support_total_travel = 26.3f,
            .support_lift_speed = 50.0f,
        },
    },
    .arm_param = {
        .joint1_motor_param = {
            .can = BSP_CAN_3,
            .motor_id = 127,
            .host_id = 0xff,
            .module = MOTOR_LZ_RSO3,
            .reverse = true,
            .mode = MOTOR_LZ_MODE_MOTION,
        },
        .joint2_motor_param = {
            .can = BSP_CAN_3,
            .master_id = 0x11,
            .can_id = 0x01,
            .module = MOTOR_DM_J4340,
            .reverse = false,
        },
        .joint3_motor_param = {
            .can = BSP_CAN_3,
            .master_id = 0x13,
            .can_id = 0x03,
            .module = MOTOR_DM_J4310P,
            .reverse = false,
        },
        .joint_kp = {8.0f, 6.0f, 6.0f},
        .joint_kd = {3.35f, 2.65f, 2.45f},
        .gravity_comp_scale = {1.05f, 0.5f, 1.0f},
        .joint_soft_limit_lower = {-0.87f, -2.20f, -1.84f},
        .joint_soft_limit_upper = {2.0f, 2.20f, 1.75f},
        .remote_cartesian = {
            .input_deadzone = 0.05f,
            .max_y_velocity = 0.30f,
            .max_z_velocity = 0.30f,
            .max_pitch_velocity = 1.0f,
            .max_linear_velocity = 0.20f,
            .max_angular_velocity = 1.0f,
            .max_linear_acceleration = 0.50f,
            .max_angular_acceleration = 2.0f,
            .max_joint_step = 0.04f,
            .joint_max_velocity = {2.0f, 2.0f, 2.0f},
            .joint_max_acceleration = {8.0f, 8.0f, 8.0f},
            .workspace = {
                .y_min = -1.0f,
                .y_max = 1.0f,
                .z_min = -0.8f,
                .z_max = 1.2f,
                .pitch_min = -3.0f,
                .pitch_max = 3.0f,
            },
        },
    },
    .auto_ctrl_param = {
        .common = {
            .prealign_kp = 4.0f,             /* yaw 误差到 wz 指令的比例系数。 */
            .prealign_wz_limit = 1.5f,       /* yaw 对正最大角速度，单位 rad/s。 */
            .flat_move_speed = 0.25f,        /* 简单平移流程默认 vx，单位 m/s；当前模板未直接使用。 */
            .flat_move_hold_ms = 600u,       /* 简单平移流程默认保持时间，单位 ms；当前模板未直接使用。 */
            .sick_valid_min_cm = 1.0f,       /* SICK 测距有效下限，单位 cm。 */
            .sick_valid_max_cm = 650.0f,     /* SICK 测距有效上限，单位 cm。 */
            .sick_norm_err_deadband = 0.02f, /* 左右 SICK 归一化差分误差死区。 */
            .sick_norm_err_to_rad = 0.50f,   /* SICK 归一化误差到 yaw 辅助角的映射系数，单位 rad。 */
            .sick_assist_max_rad = 0.35f,    /* SICK yaw 辅助角限幅，单位 rad。 */
            .sick_assist_gain = 1.0f,        /* SICK yaw 辅助量融合增益。 */
        },
        /* 头向 / 上台阶 / 200mm 模板参数。 */
        .head_ascend_200 = {
            .prealign_move_speed = 1.0f,        /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .align_move_speed = 0.30f,          /* 模板 step0 对正阶段 vx，单位 m/s。 */
            .pole_extend_move_speed = 0.50f,    /* 四杆伸出阶段 vx，单位 m/s。 */
            .pole_extend_settle_ms = 100u,      /* 四杆伸出后稳定等待时间，单位 ms。 */
            .front_retract_move_speed = 0.50f,  /* 前杆回收阶段 vx，单位 m/s。 */
            .front_retract_vy = 0.04f,          /* 前杆等待/动作阶段附加 vy，单位 m/s。 */
            .front_retract_settle_ms = 800u,    /* 前杆动作后稳定等待时间，单位 ms。 */
            .front_retract_timeout_ms = 5000u,  /* 前杆动作或前光电等待超时，单位 ms。 */
            .mid_move_speed = 1.0f,            /* 中段平移 vx，单位 m/s。 */
            .mid_move_ms = 1500u,               /* 中段平移持续时间，单位 ms。 */
            .rear_retract_move_speed = 1.50f,   /* 后杆回收阶段 vx，单位 m/s。 */
            .rear_retract_vy = 0.15f,           /* 后杆动作阶段预留附加 vy，单位 m/s。 */
            .rear_retract_timeout_ms = 5000u,   /* 后杆动作或后光电等待超时，单位 ms。 */
            .rear_retract_move_ms = 700u,       /* 后杆回收后继续移动时间，单位 ms。 */
            .final_move_speed = 0.50f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 700u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_all_extend_speed = 50.0f,     /* 四杆全伸目标跟随速度，单位 rad/s。 */
            .pole_front_extend_speed = 20.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 18.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 18.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 30.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发超时，单位 ms。 */
            .rear_photo_timeout_ms = 10000u,    /* 等待后光电触发超时，单位 ms。 */
            .hold_ms = 1200u,                   /* 保留等待时间，单位 ms；当前模板未直接使用。 */
        },
        /* 头向 / 上台阶 / 400mm 模板参数。 */
        .head_ascend_400 = {
            .prealign_move_speed = 0.40f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .align_move_speed = 0.30f,          /* 模板 step0 对正阶段 vx，单位 m/s。 */
            .pole_extend_move_speed = 0.30f,    /* 四杆伸出阶段 vx，单位 m/s。 */
            .pole_extend_settle_ms = 1000u,     /* 四杆伸出后稳定等待时间，单位 ms。 */
            .front_retract_move_speed = 0.25f,  /* 前杆回收阶段 vx，单位 m/s。 */
            .front_retract_vy = 0.15f,          /* 前杆等待/动作阶段附加 vy，单位 m/s。 */
            .front_retract_settle_ms = 800u,    /* 前杆动作后稳定等待时间，单位 ms。 */
            .front_retract_timeout_ms = 5000u,  /* 前杆动作或前光电等待超时，单位 ms。 */
            .mid_move_speed = 0.50f,            /* 中段平移 vx，单位 m/s。 */
            .mid_move_ms = 1500u,               /* 中段平移持续时间，单位 ms。 */
            .rear_retract_move_speed = 0.20f,   /* 后杆回收阶段 vx，单位 m/s。 */
            .rear_retract_vy = 0.15f,           /* 后杆动作阶段预留附加 vy，单位 m/s。 */
            .rear_retract_timeout_ms = 5000u,   /* 后杆动作或后光电等待超时，单位 ms。 */
            .rear_retract_move_ms = 800u,       /* 后杆回收后继续移动时间，单位 ms。 */
            .final_move_speed = 0.50f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 800u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_all_extend_speed = 40.0f,     /* 四杆全伸目标跟随速度，单位 rad/s。 */
            .pole_front_extend_speed = 40.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 18.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 40.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 30.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .front_photo_timeout_ms = 6000u,    /* 等待前光电触发超时，单位 ms。 */
            .rear_photo_timeout_ms = 6000u,     /* 等待后光电触发超时，单位 ms。 */
            .hold_ms = 1200u,                   /* 保留等待时间，单位 ms；当前模板未直接使用。 */
        },
        /* 头向 / 下台阶 / 200mm 模板参数。 */
        .head_descend_200 = {
            .prealign_move_speed = 0.20f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .align_move_speed = 0.30f,          /* 模板 step0 对正阶段 vx，单位 m/s。 */
            .pole_extend_move_speed = 0.20f,    /* 撑杆伸出阶段 vx，单位 m/s。 */
            .pole_extend_settle_ms = 900u,      /* 撑杆伸出后稳定等待时间，单位 ms。 */
            .front_retract_move_speed = 0.10f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .front_retract_vy = 0.0f,           /* 前杆动作阶段附加 vy，单位 m/s。 */
            .front_retract_settle_ms = 800u,    /* 前杆动作后稳定等待时间，单位 ms。 */
            .front_retract_timeout_ms = 5000u,  /* 前杆动作或前光电等待超时，单位 ms。 */
            .mid_move_speed = 0.20f,            /* 中段平移 vx，单位 m/s。 */
            .mid_move_ms = 700u,                /* 中段平移持续时间，单位 ms。 */
            .rear_retract_move_speed = 0.10f,   /* 后杆动作阶段 vx，单位 m/s。 */
            .rear_retract_vy = 0.0f,            /* 后杆动作阶段预留附加 vy，单位 m/s。 */
            .rear_retract_timeout_ms = 5000u,   /* 后杆动作或后光电等待超时，单位 ms。 */
            .rear_retract_move_ms = 700u,       /* 后杆动作后继续移动时间，单位 ms。 */
            .final_move_speed = 0.10f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 600u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_all_extend_speed = 50.0f,     /* 四杆全伸目标跟随速度，单位 rad/s。 */
            .pole_front_extend_speed = 30.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 18.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 30.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 30.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 5000u,     /* 等待后光电下降沿超时，单位 ms。 */
            .hold_ms = 1200u,                   /* 保留等待时间，单位 ms；当前模板未直接使用。 */
        },
        /* 头向 / 下台阶 / 400mm 模板参数。 */
        .head_descend_400 = {
            .prealign_move_speed = 0.40f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .align_move_speed = 0.30f,          /* 模板 step0 对正阶段 vx，单位 m/s。 */
            .pole_extend_move_speed = 0.40f,    /* 撑杆伸出阶段 vx，单位 m/s。 */
            .pole_extend_settle_ms = 1200u,     /* 撑杆伸出后稳定等待时间，单位 ms。 */
            .front_retract_move_speed = 0.40f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .front_retract_vy = 0.15f,          /* 前杆动作阶段附加 vy，单位 m/s。 */
            .front_retract_settle_ms = 1000u,   /* 前杆动作后稳定等待时间，单位 ms。 */
            .front_retract_timeout_ms = 6000u,  /* 前杆动作或前光电等待超时，单位 ms。 */
            .mid_move_speed = 0.50f,            /* 中段平移 vx，单位 m/s。 */
            .mid_move_ms = 1500u,               /* 中段平移持续时间，单位 ms。 */
            .rear_retract_move_speed = 0.15f,   /* 后杆动作阶段 vx，单位 m/s。 */
            .rear_retract_vy = 0.15f,           /* 后杆动作阶段预留附加 vy，单位 m/s。 */
            .rear_retract_timeout_ms = 6000u,   /* 后杆动作或后光电等待超时，单位 ms。 */
            .rear_retract_move_ms = 800u,       /* 后杆动作后继续移动时间，单位 ms。 */
            .final_move_speed = 0.10f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 800u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_all_extend_speed = 40.0f,     /* 四杆全伸目标跟随速度，单位 rad/s。 */
            .pole_front_extend_speed = 40.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 15.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 40.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 25.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .front_photo_timeout_ms = 6000u,    /* 等待前光电下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 6000u,     /* 等待后光电下降沿超时，单位 ms。 */
            .hold_ms = 1200u,                   /* 保留等待时间，单位 ms；当前模板未直接使用。 */
        },
        /* 尾向 / 上台阶 / 200mm 模板参数。 */
        .tail_ascend_200 = {
            .prealign_move_speed = 0.50f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .align_move_speed = 0.30f,          /* 模板 step0 对正阶段 vx，单位 m/s。 */
            .pole_extend_move_speed = 0.50f,    /* 四杆伸出阶段 vx，单位 m/s。 */
            .pole_extend_settle_ms = 900u,      /* 四杆伸出后稳定等待时间，单位 ms。 */
            .front_retract_move_speed = 0.50f,  /* 前杆回收阶段 vx，单位 m/s。 */
            .front_retract_vy = 0.04f,          /* 前杆等待/动作阶段附加 vy，单位 m/s。 */
            .front_retract_settle_ms = 800u,    /* 前杆动作后稳定等待时间，单位 ms。 */
            .front_retract_timeout_ms = 5000u,  /* 前杆动作或前光电等待超时，单位 ms。 */
            .mid_move_speed = 0.50f,            /* 中段平移 vx，单位 m/s。 */
            .mid_move_ms = 1500u,               /* 中段平移持续时间，单位 ms。 */
            .rear_retract_move_speed = 0.20f,   /* 后杆回收阶段 vx，单位 m/s。 */
            .rear_retract_vy = 0.15f,           /* 后杆动作阶段预留附加 vy，单位 m/s。 */
            .rear_retract_timeout_ms = 5000u,   /* 后杆动作或后光电等待超时，单位 ms。 */
            .rear_retract_move_ms = 700u,       /* 后杆回收后继续移动时间，单位 ms。 */
            .final_move_speed = 0.25f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 600u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_all_extend_speed = 50.0f,     /* 四杆全伸目标跟随速度，单位 rad/s。 */
            .pole_front_extend_speed = 50.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 18.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 50.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 30.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发超时，单位 ms。 */
            .rear_photo_timeout_ms = 10000u,    /* 等待后光电触发超时，单位 ms。 */
            .hold_ms = 1200u,                   /* 保留等待时间，单位 ms；当前模板未直接使用。 */
        },
        /* 尾向 / 上台阶 / 400mm 模板参数。 */
        .tail_ascend_400 = {
            .prealign_move_speed = 0.40f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .align_move_speed = 0.30f,          /* 模板 step0 对正阶段 vx，单位 m/s。 */
            .pole_extend_move_speed = 0.30f,    /* 四杆伸出阶段 vx，单位 m/s。 */
            .pole_extend_settle_ms = 1000u,     /* 四杆伸出后稳定等待时间，单位 ms。 */
            .front_retract_move_speed = 0.25f,  /* 前杆回收阶段 vx，单位 m/s。 */
            .front_retract_vy = 0.15f,          /* 前杆等待/动作阶段附加 vy，单位 m/s。 */
            .front_retract_settle_ms = 800u,    /* 前杆动作后稳定等待时间，单位 ms。 */
            .front_retract_timeout_ms = 5000u,  /* 前杆动作或前光电等待超时，单位 ms。 */
            .mid_move_speed = 0.50f,            /* 中段平移 vx，单位 m/s。 */
            .mid_move_ms = 1500u,               /* 中段平移持续时间，单位 ms。 */
            .rear_retract_move_speed = 0.20f,   /* 后杆回收阶段 vx，单位 m/s。 */
            .rear_retract_vy = 0.15f,           /* 后杆动作阶段预留附加 vy，单位 m/s。 */
            .rear_retract_timeout_ms = 5000u,   /* 后杆动作或后光电等待超时，单位 ms。 */
            .rear_retract_move_ms = 800u,       /* 后杆回收后继续移动时间，单位 ms。 */
            .final_move_speed = 0.50f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 800u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_all_extend_speed = 40.0f,     /* 四杆全伸目标跟随速度，单位 rad/s。 */
            .pole_front_extend_speed = 40.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 18.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 40.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 30.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .front_photo_timeout_ms = 6000u,    /* 等待前光电触发超时，单位 ms。 */
            .rear_photo_timeout_ms = 6000u,     /* 等待后光电触发超时，单位 ms。 */
            .hold_ms = 1200u,                   /* 保留等待时间，单位 ms；当前模板未直接使用。 */
        },
        /* 尾向 / 下台阶 / 200mm 模板参数。 */
        .tail_descend_200 = {
            .prealign_move_speed = 0.20f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .align_move_speed = 0.30f,          /* 模板 step0 对正阶段 vx，单位 m/s。 */
            .pole_extend_move_speed = 0.20f,    /* 撑杆伸出阶段 vx，单位 m/s。 */
            .pole_extend_settle_ms = 900u,      /* 撑杆伸出后稳定等待时间，单位 ms。 */
            .front_retract_move_speed = 0.10f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .front_retract_vy = 0.0f,           /* 前杆动作阶段附加 vy，单位 m/s。 */
            .front_retract_settle_ms = 800u,    /* 前杆动作后稳定等待时间，单位 ms。 */
            .front_retract_timeout_ms = 5000u,  /* 前杆动作或前光电等待超时，单位 ms。 */
            .mid_move_speed = 0.50f,            /* 中段平移 vx，单位 m/s。 */
            .mid_move_ms = 300u,                /* 中段平移持续时间，单位 ms。 */
            .rear_retract_move_speed = 0.10f,   /* 后杆动作阶段 vx，单位 m/s。 */
            .rear_retract_vy = 0.0f,            /* 后杆动作阶段预留附加 vy，单位 m/s。 */
            .rear_retract_timeout_ms = 5000u,   /* 后杆动作或后光电等待超时，单位 ms。 */
            .rear_retract_move_ms = 700u,       /* 后杆动作后继续移动时间，单位 ms。 */
            .final_move_speed = 0.10f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 500u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_all_extend_speed = 50.0f,     /* 四杆全伸目标跟随速度，单位 rad/s。 */
            .pole_front_extend_speed = 50.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 25.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 50.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 25.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 5000u,     /* 等待后光电下降沿超时，单位 ms。 */
            .hold_ms = 1200u,                   /* 保留等待时间，单位 ms；当前模板未直接使用。 */
        },
        /* 尾向 / 下台阶 / 400mm 模板参数。 */
        .tail_descend_400 = {
            .prealign_move_speed = 0.40f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .align_move_speed = 0.30f,          /* 模板 step0 对正阶段 vx，单位 m/s。 */
            .pole_extend_move_speed = 0.40f,    /* 撑杆伸出阶段 vx，单位 m/s。 */
            .pole_extend_settle_ms = 1200u,     /* 撑杆伸出后稳定等待时间，单位 ms。 */
            .front_retract_move_speed = 0.40f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .front_retract_vy = 0.15f,          /* 前杆动作阶段附加 vy，单位 m/s。 */
            .front_retract_settle_ms = 1000u,   /* 前杆动作后稳定等待时间，单位 ms。 */
            .front_retract_timeout_ms = 6000u,  /* 前杆动作或前光电等待超时，单位 ms。 */
            .mid_move_speed = 0.50f,            /* 中段平移 vx，单位 m/s。 */
            .mid_move_ms = 1500u,               /* 中段平移持续时间，单位 ms。 */
            .rear_retract_move_speed = 0.15f,   /* 后杆动作阶段 vx，单位 m/s。 */
            .rear_retract_vy = 0.15f,           /* 后杆动作阶段预留附加 vy，单位 m/s。 */
            .rear_retract_timeout_ms = 6000u,   /* 后杆动作或后光电等待超时，单位 ms。 */
            .rear_retract_move_ms = 800u,       /* 后杆动作后继续移动时间，单位 ms。 */
            .final_move_speed = 0.10f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 800u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_all_extend_speed = 40.0f,     /* 四杆全伸目标跟随速度，单位 rad/s。 */
            .pole_front_extend_speed = 40.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 15.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 40.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 25.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .front_photo_timeout_ms = 6000u,    /* 等待前光电下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 6000u,     /* 等待后光电下降沿超时，单位 ms。 */
            .hold_ms = 1200u,                   /* 保留等待时间，单位 ms；当前模板未直接使用。 */
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

Config_RobotParam_t *Config_GetRobotParam(void) {
  return &robot_config;
}
