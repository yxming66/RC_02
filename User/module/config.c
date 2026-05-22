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
                .d = 0.0f,
                .i_limit = 2.0f,
                .out_limit = 6.0f,
                .d_cutoff_freq = 35.0f,
                .range = 0.0f,
            },
            .motor_pos_pid_param = {
                .k = 1.5f,
                .p = 5.0f,
                .i = 2.0f,
                .d = 0.50f,
                .i_limit = 1.0f,
                .out_limit = 6.0f,
                .d_cutoff_freq = 0.0f,
                .range = 0.0f,
            },
        },
        .physical = {
            .wheel_radius_m = 0.076f,
            .wheelbase_m = 0.2f, //L,旧0.23f, //L,新0.2f
            .trackwidth_m = 0.345f,//W,旧0.325f, //W,新0.345f
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
        .controller = {
            .sample_freq = 500.0f,
            .position_to_velocity_limit = 1.2f,
            .velocity_to_torque_limit = 6.0f,
            .lateral_vy_to_wz_feedforward = 0.10f,//1.1f不会偏航会有-vx
            .lateral_heading_hold_enable = true,
            .lateral_heading_hold_kp = 8.0f,
            .lateral_heading_hold_kd = 0.8f,
            .lateral_heading_hold_max_wz = 2.5f,
            .lateral_heading_hold_wz_deadband = 0.03f,
            .lateral_heading_hold_error_deadband = 0.005f,
        },
        .motor_temperature_protection = {
            .warning_c = 50.0f,
            .limit_c = 80.0f,
            .auto_relax_on_limit = true,
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
                .out_limit = 500.0f,
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
            .step_200_all_extend = {13.8f, 13.8f},//13.3f
            .step_200_front_retract = {0.0f, 13.8f},
            .step_200_all_retract = {0.0f, 0.0f},
            .step_200_small = {3.0f, 3.0f},
            .step_400_all_extend = {26.7f, 26.7f},
            .step_400_front_retract = {0.0f, 26.7f},
            .step_400_all_retract = {0.0f, 0.0f},
        },
        .limit = {
            .max_current = 1.0f,
            .support_total_travel = 26.7f,
            .support_lift_speed = 70.0f,
        },
    },
    .ore_store_param = {
        .motor_temperature_protection = {
            .warning_c = 50.0f,
            .limit_c = 80.0f,
            .auto_relax_on_limit = true,
        },
        .motor_param = {
            [ORE_STORE_AXIS_PLATFORM] =    {.can = BSP_CAN_1, .id = 0x205, .module = MOTOR_M3508, .reverse = false, .gear = true},
            [ORE_STORE_AXIS_GATE_LEFT] =   {.can = BSP_CAN_3, .id = 0x201, .module = MOTOR_M2006, .reverse = false, .gear = true},
            [ORE_STORE_AXIS_GATE_RIGHT] =  {.can = BSP_CAN_3, .id = 0x202, .module = MOTOR_M2006, .reverse = true, .gear = true},
            [ORE_STORE_AXIS_TRACK_LEFT] =  {.can = BSP_CAN_3, .id = 0x203, .module = MOTOR_M2006, .reverse = true, .gear = true},
            [ORE_STORE_AXIS_TRACK_RIGHT] = {.can = BSP_CAN_3, .id = 0x204, .module = MOTOR_M2006, .reverse = false, .gear = true},
        },
        .motor_install = {
            [ORE_STORE_AXIS_PLATFORM] = {.external_ratio = 1.0f, .reverse_output = false},
            [ORE_STORE_AXIS_GATE_LEFT] = {.external_ratio = 1.0f, .reverse_output = false},
            [ORE_STORE_AXIS_GATE_RIGHT] = {.external_ratio = 1.0f, .reverse_output = false},
            [ORE_STORE_AXIS_TRACK_LEFT] = {.external_ratio = 1.0f, .reverse_output = false},
            [ORE_STORE_AXIS_TRACK_RIGHT] = {.external_ratio = 1.0f, .reverse_output = false},
        },
        .pid = {
            .position_pid = {
                [ORE_STORE_AXIS_PLATFORM] = {.k = 1.0f, .p = 3.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.30f, .d_cutoff_freq = 0.0f, .range = 0.0f},
                [ORE_STORE_AXIS_GATE_LEFT] = {.k = 1.0f, .p = 9.0f, .i = 0.0f, .d = 0.04f, .i_limit = 0.0f, .out_limit = 3.5f, .d_cutoff_freq = 18.0f, .range = 0.0f},
                [ORE_STORE_AXIS_GATE_RIGHT] = {.k = 1.0f, .p = 9.0f, .i = 0.0f, .d = 0.04f, .i_limit = 0.0f, .out_limit = 3.5f, .d_cutoff_freq = 18.0f, .range = 0.0f},
                [ORE_STORE_AXIS_TRACK_LEFT] = {.k = 1.0f, .p = 2.5f, .i = 0.0f, .d = 0.01f, .i_limit = 0.0f, .out_limit = 8.0f, .d_cutoff_freq = 12.0f, .range = 0.0f},
                [ORE_STORE_AXIS_TRACK_RIGHT] = {.k = 1.0f, .p = 2.5f, .i = 0.0f, .d = 0.01f, .i_limit = 0.0f, .out_limit = 8.0f, .d_cutoff_freq = 12.0f, .range = 0.0f},
            },
            .velocity_pid = {
                [ORE_STORE_AXIS_PLATFORM] = {.k = 1.0f, .p = 0.22f, .i = 0.010f, .d = 0.0f, .i_limit = 0.20f, .out_limit = 0.25f, .d_cutoff_freq = 0.0f, .range = 0.0f},
                [ORE_STORE_AXIS_GATE_LEFT] = {.k = 1.0f, .p = 0.12f, .i = 0.002f, .d = 0.0f, .i_limit = 0.08f, .out_limit = 0.28f, .d_cutoff_freq = 0.0f, .range = 0.0f},
                [ORE_STORE_AXIS_GATE_RIGHT] = {.k = 1.0f, .p = 0.12f, .i = 0.002f, .d = 0.0f, .i_limit = 0.08f, .out_limit = 0.28f, .d_cutoff_freq = 0.0f, .range = 0.0f},
                [ORE_STORE_AXIS_TRACK_LEFT] = {.k = 1.0f, .p = 0.055f, .i = 0.0015f, .d = 0.0f, .i_limit = 0.06f, .out_limit = 0.16f, .d_cutoff_freq = 0.0f, .range = 0.0f},
                [ORE_STORE_AXIS_TRACK_RIGHT] = {.k = 1.0f, .p = 0.055f, .i = 0.0015f, .d = 0.0f, .i_limit = 0.06f, .out_limit = 0.16f, .d_cutoff_freq = 0.0f, .range = 0.0f},
            },
            .pole_position_pid = {
                [ORE_STORE_AXIS_PLATFORM] = {.k = 15.0f, .p = 5.0f, .i = 3.0f, .d = 0.0f, .i_limit = 5.0f, .out_limit = 300.00f, .d_cutoff_freq = 0.0f, .range = 0.0f},
                [ORE_STORE_AXIS_GATE_LEFT] = {.k = 1.0f, .p = 9.0f, .i = 0.0f, .d = 0.04f, .i_limit = 0.0f, .out_limit = 3.5f, .d_cutoff_freq = 18.0f, .range = 0.0f},
                [ORE_STORE_AXIS_GATE_RIGHT] = {.k = 1.0f, .p = 9.0f, .i = 0.0f, .d = 0.04f, .i_limit = 0.0f, .out_limit = 3.5f, .d_cutoff_freq = 18.0f, .range = 0.0f},
                [ORE_STORE_AXIS_TRACK_LEFT] = {.k = 4.0f, .p = 9.0f, .i = 0.0f, .d = 0.01f, .i_limit = 0.0f, .out_limit = 300.0f, .d_cutoff_freq = 18.0f, .range = 0.0f},
                [ORE_STORE_AXIS_TRACK_RIGHT] = {.k = 4.0f, .p = 9.0f, .i = 0.0f, .d = 0.01f, .i_limit = 0.0f, .out_limit = 300.0f, .d_cutoff_freq = 18.0f, .range = 0.0f},
            }, 
            .pole_velocity_pid = {
                [ORE_STORE_AXIS_PLATFORM] = {.k = 0.1f, .p = 0.070f, .i = 0.004f, .d = 0.0f, .i_limit = 8.0f, .out_limit = 1.0f, .d_cutoff_freq = 18.0f, .range = 0.0f},
                [ORE_STORE_AXIS_GATE_LEFT] = {.k = 1.0f, .p = 0.16f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .d_cutoff_freq = 20.0f, .range = 0.0f},
                [ORE_STORE_AXIS_GATE_RIGHT] = {.k = 1.0f, .p = 0.16f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .d_cutoff_freq = 20.0f, .range = 0.0f},
                [ORE_STORE_AXIS_TRACK_LEFT] = {.k = 1.0f, .p = 0.16f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .d_cutoff_freq = 30.0f, .range = 0.0f},
                [ORE_STORE_AXIS_TRACK_RIGHT] = {.k = 1.0f, .p = 0.16f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .d_cutoff_freq = 30.0f, .range = 0.0f},
            },
            // MIT风格控制参数 (Kp/Kd)，用于 MIT_STYLE 模式
            // MIT控制律: torque = Kp * (target_pos - current_pos) - Kd * velocity + torque_ff
            // Kp 单位: N·m/rad (每弧度位置误差产生 N·m 力矩)
            // Kd 单位: N·m·s/rad (每 rad/s 速度产生 N·m 阻尼)
            // PLATFORM 惯量大(减速器)，需要较小 Kp + 较大 Kd 阻尼；GATE/TRACK 惯量小
            // 震荡时优先增大 Kd 阻尼，Kp 适中即可
            .mit_kp = {0.90f, 0.35f, 0.35f, 0.30f, 0.30f},
            .mit_kd = {0.15f, 0.18f, 0.18f, 0.16f, 0.16f},
        },
        .controller = { 
            .sample_freq = 500.0f, 
            .position_to_velocity_limit = {21.80f, 4.00f, 4.00f, 10.00f, 10.00f},
            .velocity_to_torque_limit = {0.25f, 0.28f, 0.28f, 0.16f, 0.16f},
            .feedback_lowpass_cutoff_hz = {-1.0f, 20.0f, 20.0f, 20.0f, 20.0f},
            .output_lowpass_cutoff_hz = {-1.0f, 12.0f, 12.0f, 12.0f, 12.0f},
            .normalized_output_limit = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
            .normalized_to_torque_nm = {8.0f, 2.0f, 2.0f, 2.0f, 2.0f},
        },
        .limit = {
            .config = { 
                [ORE_STORE_AXIS_PLATFORM] = {.stall_velocity_threshold_rad_s = 0.04f, .stall_position_window_rad = 0.0015f, .stall_cycles_required = 30u, .seek_timeout_s = 8.0f, .online_wait_timeout_s = 1.5f, .limit_margin_rad = 0.02f, .learned_limit_margin_rad = 0.02f, .min_range_rad = 0.01f},
                [ORE_STORE_AXIS_GATE_LEFT] = {.stall_velocity_threshold_rad_s = 0.04f, .stall_position_window_rad = 0.0015f, .stall_cycles_required = 30u, .seek_timeout_s = 6.0f, .online_wait_timeout_s = 1.5f, .limit_margin_rad = 0.01f, .learned_limit_margin_rad = 0.01f, .min_range_rad = 0.01f},
                [ORE_STORE_AXIS_GATE_RIGHT] = {.stall_velocity_threshold_rad_s = 0.04f, .stall_position_window_rad = 0.0015f, .stall_cycles_required = 30u, .seek_timeout_s = 6.0f, .online_wait_timeout_s = 1.5f, .limit_margin_rad = 0.01f, .learned_limit_margin_rad = 0.01f, .min_range_rad = 0.01f},
                [ORE_STORE_AXIS_TRACK_LEFT] = {.stall_velocity_threshold_rad_s = 0.04f, .stall_position_window_rad = 0.0015f, .stall_cycles_required = 30u, .seek_timeout_s = 6.0f, .online_wait_timeout_s = 1.5f, .limit_margin_rad = 0.02f, .learned_limit_margin_rad = 0.02f, .min_range_rad = 0.01f},
                [ORE_STORE_AXIS_TRACK_RIGHT] = {.stall_velocity_threshold_rad_s = 0.04f, .stall_position_window_rad = 0.0015f, .stall_cycles_required = 30u, .seek_timeout_s = 6.0f, .online_wait_timeout_s = 1.5f, .limit_margin_rad = 0.02f, .learned_limit_margin_rad = 0.02f, .min_range_rad = 0.01f},
            },
            .travel_rad = {22.7f, 2.5f, 2.5f, 40.0f, 40.0f},
            .lower_seek_velocity_rad_s = {0.20f, 0.20f, 0.20f, 0.60f, 0.60f},
            .move_velocity_rad_s = {80.0f, 80.0f, 80.0f, 60.00f, 60.00f},
            .move_accel_rad_s2 = {80.0f, 80.0f, 80.0f, 40.0f, 40.0f},
            .arrive_threshold_rad = {0.05f, 0.03f, 0.03f, 0.08f, 0.08f},
        },
        .power_on = {
            .fixed_position_enable = {true, true, true, true, true},
            .fixed_position_rad = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
            .auto_assume_on_active = true,
            .home_mode_uses_fixed_position = true,
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
        .joint_temperature_protection = {
            [0] = {.warning_c = 70.0f, .limit_c = 85.0f, .auto_relax_on_limit = true},
            [1] = {.warning_c = 70.0f, .limit_c = 85.0f, .auto_relax_on_limit = true},
            [2] = {.warning_c = 70.0f, .limit_c = 85.0f, .auto_relax_on_limit = true},
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
    .arm_simple_param = {
        .dm4340_param = {
            .can = BSP_CAN_3,
            .master_id = 0x13,
            .can_id = 0x03,
            .module = MOTOR_DM_J4340,
            .reverse = false,
        },
        .servo_param = {
            .pwm_channel = BSP_PWM_ARM_SERVO,
            .freq_hz = 50.0f,
        },
        .suction_param = {
            .gpio = BSP_GPIO_ARM_SOLENOID,
        },
        .pid = {
            .joint1_pos = {
                .k = 1.0f, .p = 10.0f, .i = 0.0f, .d = 0.0f,
                .i_limit = 0.0f, .out_limit = 10.0f,
                .d_cutoff_freq = 0.0f, .range = 0.0f,
            },
            .joint1_vel = {
                .k = 1.0f, .p = 5.0f, .i = 0.0f, .d = 0.0f,
                .i_limit = 0.0f, .out_limit = 5.0f,
                .d_cutoff_freq = 0.0f, .range = 0.0f,
            },
        },
        .soft_limit = {
            .joint1_min = -3.14f,
            .joint1_max = 3.14f,
            .joint2_min = -4.71f,   /* -270° */
            .joint2_max = 4.71f,    /* +270° */
        },
        .vel_limit = {
            .joint1_max_vel = 5.0f,
            .joint2_max_vel = 3.0f,
        },
    },
    .auto_ctrl_param = {
        .common = {
            .prealign_kp = 13.0f,             /* yaw 误差到 wz 指令的比例系数。 */
            .prealign_wz_limit = 5.0f,       /* yaw 对正最大角速度，单位 rad/s。 */
            .sick_valid_min_cm = 1.0f,       /* SICK 测距有效下限，单位 cm。 */
            .sick_valid_max_cm = 650.0f,     /* SICK 测距有效上限，单位 cm。 */
            .sick_norm_err_deadband = 0.02f, /* 左右 SICK 归一化差分误差死区。 */
            .sick_norm_err_to_rad = 0.50f,   /* SICK 归一化误差到 yaw 辅助角的映射系数，单位 rad。 */
            .sick_assist_max_rad = 0.35f,    /* SICK yaw 辅助角限幅，单位 rad。 */
            .sick_assist_gain = 1.0f,        /* SICK yaw 辅助量融合增益。 */
        },
        /* 头向 / 上台阶 / 200mm 模板参数。 */
        .head_ascend_200 = {
            /*
             * 专用模板 AutoCtrlTemplate_RunHeadAscendOptimized:
             * - PREALIGN 已完成 yaw 对正，模板内不再使用 align_move_speed。
             * - pole_extend_move_speed: 四杆全伸以及等待前光电的前进速度。
             * - front_retract_move_speed: 前光电触发后，收前腿时的前进速度。
             * - rear_retract_move_speed: 中段后低速等待后光电的前进速度。
             * - second_photo_retract_move_speed: 后光电触发后，全收腿时的前进速度。
             */
            .prealign_move_speed = 0.0f,        /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .pole_extend_move_speed = 0.50f,    /* 撑杆伸出阶段 vx，单位 m/s。 */
            .front_retract_move_speed = 0.50f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .front_retract_timeout_ms = 5000u,  /* 前光电触发后，等待前杆收回到位超时，单位 ms。 */
            .mid_move_speed = 3.0f,             /* 前杆收回到位后的中段平移 vx，单位 m/s。 */
            .mid_move_ms = 350u,                  /* 中段平移持续时间，单位 ms。 */
            .rear_retract_move_speed = 0.40f,   /* 等待后光电触发的低速 vx，单位 m/s。 */
            .rear_retract_timeout_ms = 5000u,   /* 后光电触发后，全收腿动作超时，单位 ms。 */
            .rear_retract_move_ms = 300u,       /* 后光电触发后，全收腿移动持续时间，单位 ms。 */
            .second_photo_retract_move_speed = 0.50f, /* 后一个光电触发收腿时向头向移动 vx，单位 m/s。 */
            .final_move_speed = 2.5f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 400u,               /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_all_extend_speed = 50.0f,     /* 四杆全伸目标跟随速度，单位 rad/s。 */
            .pole_front_extend_speed = 20.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 65.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 18.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 30.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发/下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 10000u,    /* 等待后光电触发/下降沿超时，单位 ms。 */
        },
        /* 头向 / 上台阶 / 400mm 模板参数。 */
        .head_ascend_400 = {
            /*
             * 专用模板 AutoCtrlTemplate_RunHeadAscendOptimized:
             * - PREALIGN 已完成 yaw 对正，模板内不再使用 align_move_speed。
             * - pole_extend_move_speed: 四杆全伸以及等待前光电的前进速度。
             * - front_retract_move_speed: 前光电触发后，收前腿时的前进速度。
             * - rear_retract_move_speed: 中段后低速等待后光电的前进速度。
             * - second_photo_retract_move_speed: 后光电触发后，全收腿时的前进速度。
             */
            .prealign_move_speed = 0.0f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .pole_extend_move_speed = 0.50f,    /* 撑杆伸出阶段 vx，单位 m/s。 */
            .front_retract_move_speed = 0.50f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .front_retract_timeout_ms = 5000u,  /* 前光电触发后，等待前杆收回到位超时，单位 ms。 */
            .mid_move_speed = 1.5f,             /* 前杆收回到位后的中段平移 vx，单位 m/s。 */
            .mid_move_ms = 250u,               /* 中段平移持续时间，单位 ms。 */
            .rear_retract_move_speed = 0.20f,   /* 等待后光电触发的低速 vx，单位 m/s。 */
            .rear_retract_timeout_ms = 5000u,   /* 后光电触发后，全收腿动作超时，单位 ms。 */
            .rear_retract_move_ms = 300u,       /* 后光电触发后，全收腿移动持续时间，单位 ms。 */
            .second_photo_retract_move_speed = 0.50f, /* 后一个光电触发收腿时向头向移动 vx，单位 m/s。 */
            .final_move_speed = 1.5f,           /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 800u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_all_extend_speed = 50.0f,     /* 四杆全伸目标跟随速度，单位 rad/s。 */
            .pole_front_extend_speed = 20.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 65.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 18.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 30.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发/下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 10000u,     /* 等待后光电触发/下降沿超时，单位 ms。 */
        },
        /* 头向 / 下台阶 / 200mm 模板参数。 */
        .head_descend_200 = {
            /*
             * Head-descend 200 optimized flow:
             * - step0: fast approach for mid_move_ms using mid_move_speed.
             * - step1: slow capture PA2/photo3 falling edge using rear_retract_move_speed.
             * - step2: stop and extend front poles, then wait for front-pole target.
             * - step3: second fixed fast approach for rear_retract_move_ms using mid_move_speed.
             * - step4: slow capture PE13/photo1 falling edge using front_retract_move_speed.
             * - step5: stop and extend all poles, then wait for all-pole target.
             * - step6: all-pole support pass using pole_extend_move_speed for hold_ms.
             * - step7: retract all poles and leave using second_photo_retract_move_speed for final_move_ms.
             */
            /* Active fields used by AutoCtrlTemplate_RunHeadDescend200Optimized. */
            .prealign_move_speed = 0.20f,       /* PREALIGN yaw 对正时叠加的前进 vx，单位 m/s。 */
            .front_retract_move_speed = 0.3f,  /* step4 等待 PE13/photo1 下降沿的慢速 vx，单位 m/s。 */
            .mid_move_speed = 1.50f,            /* step0/step3 两段固定快跑 vx，单位 m/s。 */
            .mid_move_ms = 280u,                /* step0 第一次固定快跑持续时间，单位 ms。 */
            .rear_retract_move_speed = 0.3f,   /* step1 等待 PA2/photo3 下降沿的慢速 vx，单位 m/s。 */
            .rear_retract_move_ms = 280u,       /* step3 第二次固定快跑持续时间，单位 ms。 */
            .second_photo_retract_move_speed = 0.20f, /* step7 第二个下降沿后全收杆离开 vx，单位 m/s。 */
            .final_move_speed = 0.05f,          /* step7 离开 vx 的备用值；second_photo_retract_move_speed <= 0 时使用。 */
            .final_move_ms = 100u,              /* step7 全收杆离开持续时间，单位 ms。 */
            .pole_front_extend_speed = 65.0f,   /* step2 前杆伸出、step5/6 四杆全伸时的前杆速度，单位 rad/s。 */
            .pole_front_retract_speed = 25.0f,  /* step0/1/7 前杆保持或回收到全收目标的速度，单位 rad/s。 */
            .pole_rear_extend_speed = 65.0f,    /* step5/6 四杆全伸时的后杆速度，单位 rad/s。 */
            .pole_rear_retract_speed = 25.0f,   /* step0-4/7 后杆保持或回收到全收目标的速度，单位 rad/s。 */
            .front_photo_timeout_ms = 5000u,    /* step4 等待 PE13/photo1 下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 5000u,     /* step1 等待 PA2/photo3 下降沿超时，单位 ms。 */
            .pole_extend_move_speed = 1.5f,    /* step6 四杆全伸行走 vx，单位 m/s。 */
            .hold_ms = 100u,                    /* step6 四杆全伸行走持续时间，单位 ms。 */
        },
        /* 头向 / 下台阶 / 400mm 模板参数。 */
        .head_descend_400 = {
            /*
             * Optimized fixed-start flow:
             * - mid_move_speed/mid_move_ms: first fixed fast approach.
             * - rear_retract_move_speed: slow capture speed for photo3 falling edge.
             * - rear_retract_move_ms: middle fixed fast run after front poles extend.
             * - front_retract_move_speed: slow capture speed for photo1 falling edge.
             * - pole_extend_move_speed/hold_ms: all-pole support pass.
             * - pole extend steps command vx = 0 in the template.
             */
            /* Active fields used by AutoCtrlTemplate_RunHeadDescend400Optimized. */
            .prealign_move_speed = 0.20f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .front_retract_move_speed = 0.15f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .mid_move_speed = 1.50f,            /* 中段平移 vx，单位 m/s。 */
            .mid_move_ms = 275u,                /* 中段平移持续时间，单位 ms。 */
            .rear_retract_move_speed = 0.18f,   /* 后杆动作阶段 vx，单位 m/s。 */
            .rear_retract_move_ms = 275u,       /* 后杆动作后继续移动时间，单位 ms。 */
            .second_photo_retract_move_speed = 0.20f, /* step7 第二个下降沿后全收杆离开 vx，单位 m/s。 */
            .final_move_speed = 0.1f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 100u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_front_extend_speed = 65.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 25.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 65.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 25.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发/下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 5000u,     /* 等待后光电触发/下降沿超时，单位 ms。 */
            .pole_extend_move_speed = 1.5f,    /* step6 四杆全伸行走 vx，单位 m/s。 */
            .hold_ms = 250u,                    /* step6 四杆全伸行走持续时间，单位 ms。 */
        },
        /* 尾向 / 上台阶 / 200mm 模板参数。 */
        .tail_ascend_200 = {
            .prealign_move_speed = 0.50f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .align_move_speed = 0.0f,           /* 模板 step0 对正阶段 vx；0 表示只对正不前进。 */
            .pole_extend_move_speed = 0.50f,    /* 撑杆伸出阶段 vx，单位 m/s。 */
            .pole_extend_settle_ms = 900u,      /* 撑杆伸出后稳定等待时间，单位 ms。 */
            .front_retract_move_speed = 0.50f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .front_retract_vy = 0.04f,          /* 前杆动作阶段附加 vy，单位 m/s。 */
            .front_retract_timeout_ms = 5000u,  /* 前杆动作或前光电等待超时，单位 ms。 */
            .mid_move_speed = 0.50f,            /* 中段平移 vx，单位 m/s。 */
            .mid_move_ms = 1500u,               /* 中段平移持续时间，单位 ms。 */
            .rear_retract_move_speed = 0.20f,   /* 后杆动作阶段 vx，单位 m/s。 */
            .rear_retract_timeout_ms = 5000u,   /* 后杆动作或后光电等待超时，单位 ms。 */
            .rear_retract_move_ms = 700u,       /* 后杆动作后继续移动时间，单位 ms。 */
            .second_photo_retract_move_speed = 0.20f, /* 后一个光电触发收腿时向尾向移动 vx，单位 m/s。 */
            .final_move_speed = 0.25f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 600u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_all_extend_speed = 50.0f,     /* 四杆全伸目标跟随速度，单位 rad/s。 */
            .pole_front_extend_speed = 50.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 18.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 50.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 30.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发/下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 10000u,    /* 等待后光电触发/下降沿超时，单位 ms。 */
        },
        /* 尾向 / 上台阶 / 400mm 模板参数。 */
        .tail_ascend_400 = {
            .prealign_move_speed = 0.50f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .align_move_speed = 0.30f,          /* 模板 step0 对正阶段 vx，单位 m/s。 */
            .pole_extend_move_speed = 0.50f,    /* 撑杆伸出阶段 vx，单位 m/s。 */
            .pole_extend_settle_ms = 0u,        /* 四杆全伸后稳定等待时间；0 表示到位后立即切步。 */
            .front_retract_move_speed = 0.50f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .front_retract_vy = 0.04f,          /* 前杆动作阶段附加 vy，单位 m/s。 */
            .front_retract_timeout_ms = 5000u,  /* 前杆动作或前光电等待超时，单位 ms。 */
            .mid_move_speed = 0.50f,            /* 中段平移 vx，单位 m/s。 */
            .mid_move_ms = 1500u,               /* 中段平移持续时间，单位 ms。 */
            .rear_retract_move_speed = 0.20f,   /* 后杆动作阶段 vx，单位 m/s。 */
            .rear_retract_timeout_ms = 5000u,   /* 后杆动作或后光电等待超时，单位 ms。 */
            .rear_retract_move_ms = 700u,       /* 后杆动作后继续移动时间，单位 ms。 */
            .final_move_speed = 0.25f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 600u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_all_extend_speed = 50.0f,     /* 四杆全伸目标跟随速度，单位 rad/s。 */
            .pole_front_extend_speed = 50.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 18.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 50.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 30.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发/下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 10000u,     /* 等待后光电触发/下降沿超时，单位 ms。 */
        },
        /* 尾向 / 下台阶 / 200mm 模板参数。 */
        .tail_descend_200 = {
            /*
             * Optimized fixed-start flow:
             * - mid_move_speed/mid_move_ms: first fixed fast approach.
             * - rear_retract_move_speed: slow capture speed for photo3 falling edge.
             * - rear_retract_move_ms: middle fixed fast run after rear poles extend.
             * - front_retract_move_speed: slow capture speed for photo1 falling edge.
             * - pole_extend_move_speed/hold_ms: all-pole support pass.
             * - pole extend steps command vx = 0 in the template.
             */
            /* Active fields used by AutoCtrlTemplate_RunTailDescendOptimized. */
            .prealign_move_speed = 0.20f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .pole_extend_move_speed = 0.20f,    /* 四杆全伸支撑通过 vx，单位 m/s。 */
            .front_retract_move_speed = 0.05f,  /* 慢速捕获 photo1 下降沿 vx，单位 m/s。 */
            .mid_move_speed = 0.80f,            /* 固定快跑 vx，单位 m/s。 */
            .mid_move_ms = 20u,                /* 第一次固定快跑持续时间，单位 ms。 */
            .rear_retract_move_speed = 0.05f,   /* 慢速捕获 photo3 下降沿 vx，单位 m/s。 */
            .rear_retract_move_ms = 20u,       /* 后杆伸出后的中间固定快跑时间，单位 ms。 */
            .second_photo_retract_move_speed = 0.10f, /* 第二个下降沿后收腿时向尾向移动 vx，单位 m/s。 */
            .final_move_speed = 0.10f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 500u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_front_extend_speed = 50.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 25.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 50.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 25.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .front_photo_timeout_ms = 5000u,    /* 等待 photo1 下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 5000u,     /* 等待 photo3 下降沿超时，单位 ms。 */
            .hold_ms = 1200u,                   /* 四杆全伸支撑通过时间，单位 ms。 */
        },
        /* 尾向 / 下台阶 / 400mm 模板参数。 */
        .tail_descend_400 = {
            /*
             * Optimized fixed-start flow:
             * - mid_move_speed/mid_move_ms: first fixed fast approach.
             * - rear_retract_move_speed: slow capture speed for photo3 falling edge.
             * - rear_retract_move_ms: middle fixed fast run after rear poles extend.
             * - front_retract_move_speed: slow capture speed for photo1 falling edge.
             * - pole_extend_move_speed/hold_ms: all-pole support pass.
             * - pole extend steps command vx = 0 in the template.
             */
            /* Active fields used by AutoCtrlTemplate_RunTailDescendOptimized. */
            .prealign_move_speed = 0.20f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .pole_extend_move_speed = 0.25f,    /* 撑杆伸出阶段 vx，单位 m/s。 */
            .front_retract_move_speed = 0.10f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .mid_move_speed = 0.35f,            /* 中段平移 vx，单位 m/s。 */
            .mid_move_ms = 200u,               /* 中段平移持续时间，单位 ms。 */
            .rear_retract_move_speed = 0.10f,   /* 后杆动作阶段 vx，单位 m/s。 */
            .rear_retract_move_ms = 700u,       /* 后杆动作后继续移动时间，单位 ms。 */
            .final_move_speed = 0.10f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 500u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_front_extend_speed = 60.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 25.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 60.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 25.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发/下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 5000u,     /* 等待后光电触发/下降沿超时，单位 ms。 */
            .hold_ms = 1200u,                   /* 四杆全伸支撑通过时间，单位 ms。 */
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
    .rod_new_param = {
        .servo = {
            .pwm_channel = BSP_PWM_ROD_SERVO,
            .freq_hz = 50.0f,
            /* TODO: 根据实际机构调整角度参数 */
            .angle_standby_rad = 0.0f,       /* 待机位：放平 */
            .angle_ready_rad = 0.5f,          /* 预备位 */
            .angle_grab_low_rad = 0.3f,     /* 低位夹取 */
            .angle_grab_high_rad = 0.8f,     /* 高位夹取 */
            .angle_lift_rad = 1.2f,          /* 抬升位 */
            .angle_min_rad = -2.356f,        /* 约 -135°，270°舵机行程下限(500μs) */
            .angle_max_rad = 2.356f,         /* 约 +135°，270°舵机行程上限(2500μs) */
            .arrive_threshold_rad = 0.05f,  /* 到位判定阈值 */
            .max_vel_rad_s = 2.0f,           /* 最大角速度 */
            .max_acc_rad_s = 5.0f,          /* 最大角加速度 */
        },
        .gripper = {
            /* TODO: 在gpio.c中配置具体GPIO引脚 */
            .gripper_gpio = BSP_GPIO_ROD_SOLENOID,
            .grip_timeout_ms = 2000u,        /* 夹取超时 */
        },
    },
};

Config_RobotParam_t *Config_GetRobotParam(void) {
  return &robot_config;
}
