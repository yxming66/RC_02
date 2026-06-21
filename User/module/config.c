/*
 * Robot configuration.
 */

#include "module/config.h"

#include <stdbool.h>

#include "bsp/can.h"
#include "device/motor_dm.h"
#include "device/motor_lz.h"
#include "device/motor_rm.h"
#include "device/sick.h"

#define CONFIG_AUTO_ORE_PREALIGN_YAW_TOLERANCE_RAD (0.0872664626f)

Config_RobotParam_t robot_config = {
    /* 模块参数：底盘 chassis_param，电机、PID、底盘几何、速度/力矩限幅。 */
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
                .k = 2.4f,
                .p = 1.8f,
                .i = 0.24f,
                .d = 0.0f,
                .i_limit = 2.0f,
                .out_limit = 6.0f,
                .d_cutoff_freq = 35.0f,
                .range = 0.0f,
            },
            .motor_high_pole_pid_param = {
                .k = 1.0f,
                .p = 1.5f,
                .i = 0.10f,
                .d = 0.0f,
                .i_limit = 1.0f,
                .out_limit = 4.0f,
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
            .wheel_radius_m = 0.0635f,
            .wheelbase_m = 0.456f,  /* L：前后轮距的一半，单位 m。 */
            .trackwidth_m = 0.345f, /* W：左右轮距的一半，单位 m。 */
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
            .sample_freq = 1000.0f,
            .wheel_start_accel_mps2 = 0.0f,
            .wheel_static_friction_nm = 0.5f,
            .wheel_static_friction_deadband_mps = 0.01f,
            .wheel_high_pole_pid_switch_lift = 15.0f,
        },
        .front_omni_rear_mecanum = {
            .lateral_vy_to_wz_feedforward = 0.10f,//1.1f不会偏航会有-vx
            .lateral_heading_hold_enable = true,
            .lateral_heading_hold_kp = 8.0f,
            .lateral_heading_hold_kd = 0.8f,
            .lateral_heading_hold_max_wz = 2.5f,
            .lateral_heading_hold_wz_deadband = 0.03f,
            .lateral_heading_hold_error_deadband = 0.005f,
        },
        .type = CHASSIS_TYPE_MECANUM,
    },
    /* 模块参数：撑杆 pole_param，四根撑杆电机、PID、台阶预设高度和行程限幅。 */
    .pole_param = {
        .motor_param = {
            [0] = {.can = BSP_CAN_2, .id = 0x201, .module = MOTOR_M3508, .reverse = true, .gear = true},
            [1] = {.can = BSP_CAN_2, .id = 0x202, .module = MOTOR_M3508, .reverse = false, .gear = true},
            [2] = {.can = BSP_CAN_2, .id = 0x203, .module = MOTOR_M3508, .reverse = true, .gear = true},
            [3] = {.can = BSP_CAN_2, .id = 0x204, .module = MOTOR_M3508, .reverse = false, .gear = true},
        },
        .pid = {
            .support_pos_pid = {
                .k = 30.0f,
                .p = 25.0f,
                .i = 0.0f,
                .d = 0.0f,
                .i_limit = 0.15f,
                .out_limit = 400.0f,
                .d_cutoff_freq = -1.0f,
                .range = 0.0f,
            },
            .support_vel_pid = {
                .k = 0.15f,
                .p = 0.15f,
                .i = 0.0f,
                .d = 0.0f,
                .i_limit = 0.0f,
                .out_limit = 1.0f,
                .d_cutoff_freq = -1.0f,
                .range = 0.0f,
            },
        },
        .preset = {
            /* 200mm 台阶四杆全伸位；一键取矿 PICK_POS_200 也使用该撑杆高度。 */
            .step_200_all_extend = {14.2f, 14.2f},//13.3f
            /* 200mm 台阶前杆收回、后杆保持支撑位。 */
            .step_200_front_retract = {0.8f, 13.8f},
            /* 200mm 台阶四杆全收位。 */
            .step_200_all_retract = {0.8f, 0.8f},
            /* 200mm 小抬升位；一键取矿 PICK_NEG_200 使用该撑杆高度。 */
            .step_200_small = {3.0f, 3.0f},
            /* 400mm 台阶四杆全伸位；一键取矿 PICK_POS_400 也使用该撑杆高度。 */
            .step_400_all_extend = {27.3f, 27.3f},
            /* 400mm 台阶前杆收回、后杆保持支撑位。 */
            .step_400_front_retract = {0.5f, 27.3f},
               /* 400mm 台阶四杆全收位。 */
            .step_400_all_retract = {0.5f, 0.8f},
        },
        .limit = {
            .max_current = 1.0f,
            .support_total_travel = 26.7f,//26.7//27.3
            .support_lift_speed = 40.0f,
            .support_lift_accel = 300.0f,
        },
    },
    /* 模块参数：矿仓 ore_store_param，矿仓平台电机、回零、预设位置和气缸。 */
    .ore_store_param = {
        .motor_param = {
            [ORE_STORE_AXIS_PLATFORM] =    {.can = BSP_CAN_1, .id = 0x205, .module = MOTOR_M3508, .reverse = false, .gear = true},
        },
        .motor_install = {
            [ORE_STORE_AXIS_PLATFORM] = {.external_ratio = 1.0f, .reverse_output = false},
        },
        .pid = {
            .position_pid = {    
                [ORE_STORE_AXIS_PLATFORM] = {.k = 1.0f, .p = 24.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 60.0f, .d_cutoff_freq = 80.0f, .range = 0.0f},
            },
            .velocity_pid = {
                [ORE_STORE_AXIS_PLATFORM] = {.k = 0.1f, .p = 1.550f, .i = 0.02f, .d = 0.0f, .i_limit = 0.35f, .out_limit = 2.5f, .d_cutoff_freq = 18.0f, .range = 0.0f},
            },
            // MIT风格控制参数 (Kp/Kd)，用于 MIT_STYLE 模式
            // MIT控制律: torque = Kp * (target_pos - current_pos) - Kd * velocity + torque_ff
            // Kp 单位: N·m/rad (每弧度位置误差产生 N·m 力矩)
            // Kd 单位: N·m·s/rad (每 rad/s 速度产生 N·m 阻尼)
            // PLATFORM 惯量大(减速器)，需要较小 Kp + 较大 Kd 阻尼；GATE/TRACK 惯量小
            // 震荡时优先增大 Kd 阻尼，Kp 适中即可
            .mit_kp = {0.90f},
            .mit_kd = {0.15f},
        },
        .controller = { 
            .sample_freq = 500.0f, 
            .feedback_lowpass_cutoff_hz = {-1.0f},
            .output_lowpass_cutoff_hz = {-1.0f},
        },
        .limit = {
            .config = { 
                [ORE_STORE_AXIS_PLATFORM] = {.stall_velocity_threshold_rad_s = 0.04f, .stall_position_window_rad = 0.0015f, .stall_cycles_required = 30u, .seek_timeout_s = 8.0f, .online_wait_timeout_s = 1.5f, .limit_margin_rad = 0.02f, .learned_limit_margin_rad = 0.02f, .min_range_rad = 0.01f},
            },
            .travel_rad = {24.7f},
            .lower_seek_velocity_rad_s = {0.20f},
            .move_velocity_rad_s = {400.0f},
            .arrive_threshold_rad = {0.05f},
        }, 
        .power_on = {
            .fixed_position_enable = {false},
            .fixed_position_rad = {0.0f},
            .auto_assume_on_active = false,
            .home_mode_uses_fixed_position = false,
        },
        .preset = {
            /* 平台升降轴预设位置：低位待机 -> 中间等待 -> 抬升交接，BUFFER 预留待标定。 */
            .transform_position_rad = {
                /* 平台待机/复位低位。 */
                [ORE_STORE_TRANSFORM_STANDBY] = 0.0f,
                /* 平台中间等待位，存高位矿时等待 arm 交接。 */
                [ORE_STORE_TRANSFORM_MID_WAIT] = 11.0f,
                /* 平台抬升位/满行程位，用于低位存矿或低位上膛交接。 */
                [ORE_STORE_TRANSFORM_LIFT] = 21.580307f,
                /* 平台缓冲/备用预设位，待实机标定。 */
                [ORE_STORE_TRANSFORM_BUFFER] = 0.0f,
                /* 取矛头平台预设位，待实机标定。 */
                [ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP] = 22.5139351f,
            },
        },
        .fixed_ore_cylinder = {    
            .gpio = BSP_GPIO_ROD_SOLENOID,
        },
    },
    /* 模块参数：机械臂 arm_param，三关节电机、关节增益、软限位和笛卡尔遥控。 */
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
    /* 模块参数：简易机械臂 arm_simple_param，DM 电机、舵机、吸盘、预设点位。 */
    .arm_simple_param = {
        .dm4340_param = {
            .can = BSP_CAN_3,
            .master_id = 0x12,
            .can_id = 0x02,
            .module = MOTOR_DM_J4340,
            .reverse = true,
        },
        .servo_param = {
            .pwm_channel = BSP_PWM_ARM_SERVO,
            .freq_hz = 50.0f,
            .reverse = true,
        },
        .suction_param = {
            .gpio = BSP_GPIO_ORE_RELAY,
        },
        .mit = {
            .joint1_kp = 150.0f,
            .joint1_kd = 10.0f,
            .joint1_torque_ff = 0.0f,
            .joint1_gravity_mass_kg = -0.10f,
            .joint1_gravity_com_m = 0.12f,
            .joint1_gravity_zero_rad = 0.0f,
            .joint1_gravity_ff_limit_nm = 5.0f,
        },
        .soft_limit = {
            .joint1_min = -0.239599289f,
            .joint1_max = 1.92628715f,
            .joint2_min = -2.356194f,   /* 舵机中心0点向负方向最大约-135° */
            .joint2_max = 2.356194f,    /* 舵机中心0点向正方向最大约+135° */
        },
        .vel_limit = {
            .joint1_max_vel = 15.5f,
            .joint2_max_vel = 50.0f,
        },
        .preset = {
            .behavior_point = {
                /* ArmSimple 待机/安全收回位，一键动作开始和结束常用位置。 */
                [ARM_SIMPLE_BEHAVIOR_STANDBY] = {.joint1_pos = 0.0f, .joint2_pos = 0.0f},
                /* ArmSimple 存矿位，arm 伸到 ore_store 交接处释放矿。 */
                [ARM_SIMPLE_BEHAVIOR_STORE_ORE] = {.joint1_pos = 0.0f, .joint2_pos = -1.57431853f},
                /* ArmSimple 上膛位，arm 伸到 ore_store 上膛交接处吸取矿。 */
                [ARM_SIMPLE_BEHAVIOR_CHAMBER_ORE] = {.joint1_pos = -0.216335908f, .joint2_pos = -1.64886379f},
                /* ArmSimple 存矿等待位，等待平台到位后再伸到存矿位。 */
                [ARM_SIMPLE_BEHAVIOR_WAIT_STORE_ORE] = {.joint1_pos = 0.174757361f, .joint2_pos = -2.04568338f},
                /* ArmSimple 放矿等待位，放矿动作前的预备/稳定位置。 */
                [ARM_SIMPLE_BEHAVIOR_WAIT_RELEASE_ORE] = {.joint1_pos = -0.164476007f, .joint2_pos = 1.64204562f},
                /* ArmSimple 放矿位，吸盘关闭后从该位置把矿释放出去。 */
                [ARM_SIMPLE_BEHAVIOR_RELEASE_ORE] = {.joint1_pos = 0.191354722f, .joint2_pos = 1.44863617f},
                /* ArmSimple 正向 400mm 取矿位，配合 pole 的 step_400_all_extend 使用。 */
                [ARM_SIMPLE_BEHAVIOR_PICK_POS_400] = {.joint1_pos = 1.53361797f, .joint2_pos = 0.0f},
                /* ArmSimple 正向 200mm 取矿位，配合 pole 的 step_200_all_extend 使用。 */
                [ARM_SIMPLE_BEHAVIOR_PICK_POS_200] = {.joint1_pos = 1.53361797f, .joint2_pos = 0.0f},
                /* ArmSimple 反向/负向 200mm 取矿位，配合 pole 的 step_200_small 使用。 */
                [ARM_SIMPLE_BEHAVIOR_PICK_NEG_200] = {.joint1_pos = 1.92628717f, .joint2_pos = -0.39897722f},
                /* 融合取矿：抬矿检测/安全携带位，实车确认矿石离地间隙后再标定。 */
                [ARM_SIMPLE_BEHAVIOR_PICK_LIFT_DETECT] = {.joint1_pos = 1.20f, .joint2_pos = -0.35f},
            },
            .arrive_threshold_rad = 0.05f,
        },
    },
    /* 模块参数：一键存取矿 auto_ore_param，存矿、放矿、上膛、取矿流程参数。 */
    .auto_ore_param = {
        /* ArmSimple 一键行为速度上限，单位 rad/s；<=0 表示使用 arm_simple_param.vel_limit 默认值。 */
        .arm_speed = {
            /* store_*：一键存矿流程，wait=等待/预备位，place=伸到存矿位，standby=回待机位。 */
            .store_wait = {.joint1_max_vel_rad_s = 1.0f, .joint2_max_vel_rad_s = 2.0f},
            .store_place = {.joint1_max_vel_rad_s = 1.0f, .joint2_max_vel_rad_s = 2.0f},
            .store_standby = {.joint1_max_vel_rad_s = 1.0f, .joint2_max_vel_rad_s = 2.0f},
            /* release_*：一键放矿流程，wait=放矿前等待位，place=放矿位，standby=放矿后回待机位。 */
            .release_wait = {.joint1_max_vel_rad_s = 1.0f, .joint2_max_vel_rad_s = 2.0f},
            .release_place = {.joint1_max_vel_rad_s = 1.0f, .joint2_max_vel_rad_s = 2.0f},
            .release_standby = {.joint1_max_vel_rad_s = 1.0f, .joint2_max_vel_rad_s = 2.0f},
            /* chamber_*：一键上膛流程，wait=对接等待位，place=取/交接位，standby=上膛后回待机位。 */
            .chamber_wait = {.joint1_max_vel_rad_s = 5.0f, .joint2_max_vel_rad_s = 8.0f},
            .chamber_place = {.joint1_max_vel_rad_s = 1.0f, .joint2_max_vel_rad_s = 4.0f},
            .chamber_standby = {.joint1_max_vel_rad_s = 1.0f, .joint2_max_vel_rad_s = 2.0f},
            /* pick_*：一键取矿流程，standby=取矿前/后待机位，place=伸到取矿位，fetch=底盘前进取矿时保持取矿位。 */
            .pick_standby = {.joint1_max_vel_rad_s = 2.0f, .joint2_max_vel_rad_s = 2.0f},
            .pick_place = {.joint1_max_vel_rad_s = 8.0f, .joint2_max_vel_rad_s = 2.0f},
            .pick_fetch = {.joint1_max_vel_rad_s = 1.0f, .joint2_max_vel_rad_s = 2.0f},
            .pick_lift_detect = {.joint1_max_vel_rad_s = 2.0f, .joint2_max_vel_rad_s = 2.0f},
        },
        /* 一键存/放/上膛/取矿流程中的动作延时，单位 ms；<=0 时使用状态机内置默认值。 */
        .timing = {
            /* 存矿：arm 到存矿位后的稳定等待、固矿气缸关闭等待、气缸重新打开等待。 */
            .store_arm_settle_ms = 400u,
            .store_cylinder_close_ms = 200u,
            .store_arm_suction_off_ms = 300u,
            .store_cylinder_open_ms = 200u, 
            /* 放矿：放矿前等待、到放矿位后短暂停稳、吸盘关闭后矿石脱离等待。 */
            .release_wait_ms = 400u,
            .release_arm_settle_ms = 10u,
            .release_suction_off_ms = 400u,
            /* 上膛：低位矿夹紧等待、arm 到交接位稳定等待、气缸打开释放等待。 */
            .chamber_low_clamp_ms = 400u,
            .chamber_arm_settle_ms = 400u,
            .chamber_cylinder_open_ms = 400u,
            /* 取矿：正向 200/400 取矿时，底盘向矿位前进的持续时间。 */
            .fetch_chassis_move_ms = 1000u,
            /* 取 -200 矿：底盘向矿位前进的独立持续时间；可设为 0 禁止底盘前进，避免走太多掉下去。 */
            .fetch_neg_200_chassis_move_ms = 1000u,
            /* 融合取矿/存矿/上台阶动作延时；0 使用代码默认值。 */
            .fused_prealign_stable_ms = 120u,          /* 融合动作 yaw 对正后稳定等待时间，单位 ms。 */
            .fused_pick_precontact_timeout_ms = 2000u, /* 融合取矿接触确认默认超时，单位 ms。 */
            .fused_pick_lift_detect_ms = 200u,         /* 融合取矿抬矿检测位到位后的确认延时，单位 ms。 */
            .fused_arm_photo_stable_ms = 120u,         /* 机械臂取矿传感器稳定确认时间，单位 ms。 */
        },
        /* 单个状态机 step 的超时时间，单位 ms。 */
        .default_step_timeout_ms = 5000u,
        /* 到位判定阈值：arm/ore_store/pole 分别使用的误差阈值，单位 rad。 */
        .arm_arrive_threshold_rad = 0.1f,
        .ore_store_arrive_threshold_rad = 0.05f,
        .pole_arrive_threshold_rad = 0.30f,
        .prealign_yaw_tolerance_rad =
            CONFIG_AUTO_ORE_PREALIGN_YAW_TOLERANCE_RAD,
        /* 取矿流程中正向 200/400 取矿的底盘前进速度，单位 m/s。 */
        .fetch_chassis_vx_mps = 0.50f,
        /* 取 -200 矿时底盘前进速度，单位 m/s；可设为 0 禁止底盘前进。 */
        .fetch_neg_200_chassis_vx_mps = 0.20f,
        /*
         * 融合动作轮转角阈值说明：
         * - *_wheel_delta_rad 使用四轮累计转角变化绝对值的平均值，单位 rad。
         * - <=0 表示跳过对应轮转角门控或回退到时间门控。
         * - use_arm_photo_confirm 需要真实机械臂取矿传感器；false 表示机械臂到位 + 延时确认。
         */
        .fused_step_pick_store_ascend_200_head = {
            .pick_action = AUTO_ORE_ACTION_PICK_POS_200,         /* 融合动作取正 200mm 矿。 */
            .precontact_wheel_delta_rad = 4.5f,                 /* 取矿接触确认轮转角阈值，单位 rad。 */
            .precontact_timeout_ms = 3000u,                     /* 取矿接触确认兜底超时，单位 ms。 */
            .step_start_vx_mps = 0.40f,                         /* 融合台阶 side 底盘前进速度，同时兼作取矿靠近速度，单位 m/s。 */
            .step_start_wheel_delta_rad = 2.36f,                /* 融合台阶起始阶段轮转角阈值，单位 rad。 */
            .use_arm_photo_confirm = false,                      /* false=机械臂到位加延时确认，true=机械臂取矿传感器确认。 */
        },
        .fused_step_pick_store_descend_200_head = {
            .pick_action = AUTO_ORE_ACTION_PICK_NEG_200,          /* 融合动作取负 200mm 矿。 */
            .precontact_wheel_delta_rad = 4.0f,                  /* 取矿接触确认轮转角阈值，单位 rad。 */
            .precontact_timeout_ms = 2000u,                      /* 取矿接触确认兜底超时，单位 ms。 */
            .step_start_vx_mps = 0.20f,                          /* 融合台阶 side 底盘前进速度，同时兼作取矿靠近速度，单位 m/s。 */
            .step_start_wheel_delta_rad = 0.0f,                  /* 融合台阶起始阶段轮转角阈值；<=0 时直接进入下一阶段。 */
            .use_arm_photo_confirm = false,                       /* false=机械臂到位加延时确认，true=机械臂取矿传感器确认。 */
        },
        .fused_step_pick_store_ascend_400_head = {
            .pick_action = AUTO_ORE_ACTION_PICK_POS_400,         /* 融合动作取正 400mm 矿。 */
            .precontact_wheel_delta_rad = 4.5f,                 /* 取矿接触确认轮转角阈值，单位 rad。 */
            .precontact_timeout_ms = 2500u,                     /* 取矿接触确认兜底超时，单位 ms。 */
            .step_start_vx_mps = 0.40f,                         /* 融合台阶 side 底盘前进速度，同时兼作取矿靠近速度，单位 m/s。 */
            .step_start_wheel_delta_rad = 2.36f,                /* 融合台阶起始阶段轮转角阈值，单位 rad。 */
            .use_arm_photo_confirm = false,                      /* false=机械臂到位加延时确认，true=机械臂取矿传感器确认。 */
        },
        .fused_step_pick_store_descend_400_head = {
            .pick_action = AUTO_ORE_ACTION_PICK_POS_400,          /* 融合动作取正 400mm 矿。 */
            .precontact_wheel_delta_rad = 4.0f,                   /* 取矿接触确认轮转角阈值，单位 rad。 */
            .precontact_timeout_ms = 2000u,                       /* 取矿接触确认兜底超时，单位 ms。 */
            .step_start_vx_mps = 0.20f,                          /* 融合台阶 side 底盘前进速度，同时兼作取矿靠近速度，单位 m/s。 */
            .step_start_wheel_delta_rad = 0.0f,                  /* 融合台阶起始阶段轮转角阈值；<=0 时直接进入下一阶段。 */
            .use_arm_photo_confirm = false,                       /* false=机械臂到位加延时确认，true=机械臂取矿传感器确认。 */
        },
    },
    /* 模块参数：一键取矛头 auto_rod_spearhead_param，取矛头机构动作时序和检测。 */
    .auto_rod_spearhead_param = {
        /* One-key spearhead action timing; rod_param is filled during init. */
        .open_delay_ms = 1000u,
        .grab_high_delay_ms = 200u,
        .dock_wait_delay_ms = 5000u,
        .use_photo_check = true,
        .photo_check_ms = 500u,
    },
    /* 模块参数：自动台阶/自动控制 auto_ctrl_param，台阶模板和 SICK 校正参数。 */
    .auto_ctrl_param = {
        /* 模块参数：AutoCtrl 通用参数 common，预对正和 SICK yaw 辅助参数。 */
        .common = {
            .prealign_kp = 5.0f,             /* yaw 误差到 wz 指令的比例系数。 */
            .prealign_wz_limit = 5.0f,       /* yaw 对正最大角速度，单位 rad/s。 */
            .sick_valid_min_cm = 1.0f,       /* SICK 测距有效下限，单位 cm。 */
            .sick_valid_max_cm = 650.0f,     /* SICK 测距有效上限，单位 cm。 */
            .sick_norm_err_deadband = 0.02f, /* 左右 SICK 归一化差分误差死区。 */
            .sick_norm_err_to_rad = 0.50f,   /* SICK 归一化误差到 yaw 辅助角的映射系数，单位 rad。 */
            .sick_assist_max_rad = 0.35f,    /* SICK yaw 辅助角限幅，单位 rad。 */
            .sick_assist_gain = 1.0f,        /* SICK yaw 辅助量融合增益。 */
        },
        /*
         * SICK 校正参数：
         * - index 字段选择校正使用的 4 路 SICK ADC 通道。
         *   当前硬件：前侧=0，矛头前侧=1，后侧=2，矛头后侧=3。
         * - x_sample_adc 使用前/后侧中距离更近的一路 SICK。
         * - y_sample_adc = average(矛头前侧, 矛头后侧)。
         * - yaw_sample_diff_adc = 矛头前侧 - 矛头后侧。
         * - *_kp 字段把 ADC 误差映射到底盘速度；*_limit 字段做命令限幅。
         *   tolerance/stable/timeout 用于判定校正完成。
         */
        /* 模块参数：SICK 一键校正 sick_correct，取矛头/放矿前的 SICK 对位参数。 */
        .sick_correct = {
            .rod_spearhead = {
                .front_index = SICK_FRONT_INDEX,
                .rod_front_index = SICK_ROD_FRONT_INDEX,
                .rear_index = SICK_REAR_INDEX,
                .rod_rear_index = SICK_ROD_REAR_INDEX,
                .valid_adc_min = 100u,               /* 有效 ADC 下限，低于认为传感器无效。 */
                .valid_adc_max = 32100u,             /* 有效 ADC 上限，高于认为传感器无效。 */
                .x_target_adc = 3825.0f,             /* 前/后侧中距离更近一路的 x 目标 ADC。 */
                .y_target_adc = (959.0f + 1058.0f) / 2.0f,             /* 矛头侧平均 y 目标 ADC。 */
                .yaw_target_diff_adc = -99.0f,         /* yaw 目标 ADC：矛头前侧 - 矛头后侧。 */
                .x_tolerance_adc = 30.0f,            /* x 误差小于该值认为 x 到位。 */
                .y_tolerance_adc = 20.0f,            /* y 误差小于该值认为 y 到位。 */
                .yaw_tolerance_adc = 20.0f,          /* yaw/z 误差小于该值认为姿态到位。 */
                .x_kp_mps_per_adc = -0.0005f,         /* x ADC 误差到 vx(m/s) 的比例系数。 */
                .y_kp_mps_per_adc = 0.0019f,       /* y ADC 误差到 vy(m/s) 的比例系数。 */
                .yaw_kp_rad_s_per_adc = 0.0017f,     /* yaw/z ADC 误差到 wz(rad/s) 的比例系数。 */
                .vx_limit_mps = 0.30f,               /* vx 指令限幅，单位 m/s。 */
                .vy_limit_mps = 0.30f,               /* vy 指令限幅，单位 m/s。 */
                .wz_limit_rad_s = 0.80f,             /* wz 指令限幅，单位 rad/s。 */
                .pole_target_lift = 2.0f,            /* 校正时撑杆目标高度/角度。 */
                .pole_speed = 50.0f,                 /* 校正时撑杆运动速度。 */
                .finish_stable_ms = 100u,            /* 全部误差到位后保持多久才判成功。 */
                .timeout_ms = 3000u,                 /* 校正总超时，单位 ms。 */
            }, 
            /* 预留给放矿前 SICK 校正。
             * 当前 AutoSickCorrect_StartOreRelease 路径返回不支持，
             * 因此这些值仅作占位。 */
            .ore_release = {
                .front_index = SICK_FRONT_INDEX,
                .rod_front_index = SICK_ROD_FRONT_INDEX,
                .rear_index = SICK_REAR_INDEX,
                .rod_rear_index = SICK_ROD_REAR_INDEX,
                .valid_adc_min = 700u,               /* 有效 ADC 下限。 */
                .valid_adc_max = 32100u,             /* 有效 ADC 上限。 */
                .x_target_adc = 0.0f,                /* 前侧平均 x 目标 ADC；0 保持该动作非法。 */
                .y_target_adc = 0.0f,                /* 矛头侧平均 y 目标 ADC。 */
                .yaw_target_diff_adc = 0.0f,         /* 目标 ADC：矛头前侧 - 矛头后侧。 */
                .x_tolerance_adc = 30.0f,            /* 允许的 x ADC 误差。 */
                .y_tolerance_adc = 30.0f,            /* 允许的 y ADC 误差。 */
                .yaw_tolerance_adc = 30.0f,          /* 允许的 yaw/z ADC 误差。 */
                .x_kp_mps_per_adc = 0.0010f,         /* x 误差到 vx 的增益。 */
                .y_kp_mps_per_adc = 0.0010f,         /* y 误差到 vy 的增益。 */
                .yaw_kp_rad_s_per_adc = 0.0020f,     /* yaw/z 误差到 wz 的增益。 */
                .vx_limit_mps = 0.30f,               /* vx 命令限幅，单位 m/s。 */
                .vy_limit_mps = 0.30f,               /* vy 命令限幅，单位 m/s。 */
                .wz_limit_rad_s = 0.80f,             /* wz 命令限幅，单位 rad/s。 */
                .pole_target_lift = 2.0f,            /* 校正阶段撑杆目标高度。 */
                .pole_speed = 50.0f,                 /* 校正阶段撑杆速度。 */
                .finish_stable_ms = 120u,            /* 成功前需要保持稳定的时间。 */
                .timeout_ms = 5000u,                 /* 校正总超时，单位 ms。 */
            },
        },
        /* 头向 / 上台阶 / 200mm 模板参数。 */
        /* 模块参数：头向上台阶 200mm head_ascend_200。 */
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
            .pole_extend_move_speed = 0.30f,    /* 撑杆伸出阶段 vx，单位 m/s。 */
            .front_retract_move_speed = 0.30f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .front_retract_timeout_ms = 5000u,  /* 前光电触发后，等待前杆收回到位超时，单位 ms。 */
            .mid_move_speed = 1.5f,             /* 前杆收回到位后的中段平移 vx，单位 m/s。 */
            .mid_move_ms = 280u,               /* 中段角度门控兜底超时，单位 ms。 */
            .mid_move_wheel_delta_rad = 8.66f,  /* 编码器门控的中段冲刺轮转角阈值，单位 rad；>0 优先按角度切步，<=0 使用 mid_move_ms。 */
            .timed_move_yaw_tolerance_rad = 0.35f, /* 中段移动切步 yaw 容差，约 10 deg。 */
            .rear_retract_move_speed = 0.04f,   /* 等待后光电触发的低速 vx，单位 m/s。 */
            .rear_retract_timeout_ms = 5000u,   /* 后光电触发后，全收腿动作超时，单位 ms。 */
            .rear_retract_move_ms = 300u,       /* 后光电触发后，全收腿移动持续时间，单位 ms。 */
            .second_photo_retract_move_speed = 0.50f, /* 后一个光电触发收腿时向头向移动 vx，单位 m/s。 */
            .final_move_speed = 1.0f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 800u,              /* 收尾离开台阶角度门控兜底超时，单位 ms。 */
            .final_move_wheel_delta_rad = 0.0f, /* 编码器门控的收尾离开轮转角阈值，单位 rad；>0 优先按角度切步，<=0 使用 final_move_ms。 */
            .pole_all_extend_speed = 65.0f,     /* 四杆全伸目标跟随速度，单位 rad/s。 */
            .pole_front_extend_speed = 20.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 60.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 18.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 60.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .pole_lift_accel = 300.0f,          /* 当前模板撑杆加速度限幅，单位 rad/s^2。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发/下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 10000u,    /* 等待后光电触发/下降沿超时，单位 ms。 */
        },
        /* 头向 / 上台阶 / 400mm 模板参数。 */
        /* 模块参数：头向上台阶 400mm head_ascend_400。 */
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
            .pole_extend_move_speed = 0.30f,    /* 撑杆伸出阶段 vx，单位 m/s。 */
            .front_retract_move_speed = 0.30f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .front_retract_timeout_ms = 5000u,  /* 前光电触发后，等待前杆收回到位超时，单位 ms。 */
            .mid_move_speed = 1.5f,             /* 前杆收回到位后的中段平移 vx，单位 m/s。 */
            .mid_move_ms = 200u,                /* 中段平移持续时间，单位 ms。 */
            .mid_move_wheel_delta_rad = 8.66f,   /* 0 表示该模板继续按时间切步。 */
            .timed_move_yaw_tolerance_rad = 0.35f, /* 中段定时移动切步 yaw 容差，约 10 deg。 */
            .rear_retract_move_speed = 0.20f,   /* 等待后光电触发的低速 vx，单位 m/s。 */
            .rear_retract_timeout_ms = 5000u,   /* 后光电触发后，全收腿动作超时，单位 ms。 */
            .rear_retract_move_ms = 300u,       /* 后光电触发后，全收腿移动持续时间，单位 ms。 */
            .second_photo_retract_move_speed = 0.50f, /* 后一个光电触发收腿时向头向移动 vx，单位 m/s。 */
            .final_move_speed = 1.0f,           /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 1000u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .final_move_wheel_delta_rad = 0.0f, /* 0 表示该模板继续按时间切步。 */
            .pole_all_extend_speed = 65.0f,     /* 四杆全伸目标跟随速度，单位 rad/s。 */
            .pole_front_extend_speed = 20.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 65.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 18.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 60.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .pole_lift_accel = 500.0f,          /* 当前模板撑杆加速度限幅，单位 rad/s^2。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发/下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 10000u,     /* 等待后光电触发/下降沿超时，单位 ms。 */
        },
        /* 头向 / 下台阶 / 200mm 模板参数。 */
        /* 模块参数：头向下台阶 200mm head_descend_200。 */
        .head_descend_200 = {
            .photo_stop_settle_ms = 100u,
            .descend_start_pole_lift_threshold = 2.0f,
            /*
             * 头向下 200mm 优化流程：
             * - step0：使用 mid_move_speed 快速靠近，持续 mid_move_ms。
             * - step1：使用 rear_retract_move_speed 慢速捕获 PA2/photo3 下降沿。
             * - step2：停车并伸前杆，然后等待前杆到位。
             * - step3：使用 mid_move_speed 第二次固定快速靠近，持续 rear_retract_move_ms。
             * - step4：使用 front_retract_move_speed 慢速捕获 PE13/photo1 下降沿。
             * - step5：停车并四杆全伸，然后等待四杆到位。
             * - step6：使用 pole_extend_move_speed 四杆支撑通过，持续 hold_ms。
             * - step7：四杆全收，并使用 second_photo_retract_move_speed 离开，持续 final_move_ms。
             */
            /* AutoCtrlTemplate_RunHeadDescend200Optimized 使用的有效字段。 */
            .prealign_move_speed = 0.20f,       /* PREALIGN yaw 对正时叠加的前进 vx，单位 m/s。 */
            .front_retract_move_speed = 0.1f,  /* step4 等待 PE13/photo1 下降沿的慢速 vx，单位 m/s。 */
            .mid_move_speed = 1.5f,            /* step0/step3 两段固定快跑 vx，单位 m/s。 */
            .mid_move_ms = 150u,                /* step0 第一次固定快跑持续时间，单位 ms。 */ 
            .timed_move_yaw_tolerance_rad = 0.35f, /* 中段定时移动切步 yaw 容差，约 10 deg。 */
            .rear_retract_move_speed = 0.1f,   /* step1 等待 PA2/photo3 下降沿的慢速 vx，单位 m/s。 */
            .rear_retract_move_ms = 250u,       /* step3 第二次固定快跑持续时间，单位 ms。 */
            .rear_retract_move_wheel_delta_rad = 8.66f, /* step3 第二次固定快跑轮转角阈值，单位 rad；0 表示按时间切步。 */
            .second_photo_retract_move_speed = 1.0f, /* step7 第二个下降沿后全收杆离开 vx，单位 m/s。 */
            .final_move_speed = 0.5f,          /* step7 离开 vx 的备用值；second_photo_retract_move_speed <= 0 时使用。 */
            .final_move_ms = 200u,              /* step7 全收杆离开持续时间，单位 ms。 */
            .pole_front_extend_speed = 65.0f,   /* step2 前杆伸出、step5/6 四杆全伸时的前杆速度，单位 rad/s。 */
            .pole_front_retract_speed = 40.0f,  /* step0/1/7 前杆保持或回收到全收目标的速度，单位 rad/s。 */
            .pole_rear_extend_speed = 65.0f,    /* step5/6 四杆全伸时的后杆速度，单位 rad/s。 */
            .pole_rear_retract_speed = 40.0f,   /* step0-4/7 后杆保持或回收到全收目标的速度，单位 rad/s。 */
            .pole_lift_accel = 500.0f,          /* 当前模板撑杆加速度限幅，单位 rad/s^2。 */
            .front_photo_timeout_ms = 5000u,    /* step4 等待 PE13/photo1 下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 5000u,     /* step1 等待 PA2/photo3 下降沿超时，单位 ms。 */
            .pole_extend_move_speed = 1.5f,    /* step6 四杆全伸行走 vx，单位 m/s。 */
            .hold_ms = 100u,                    /* step6 四杆全伸行走持续时间，单位 ms。 */
        },
        /* 头向 / 下台阶 / 400mm 模板参数。 */
        /* 模块参数：头向下台阶 400mm head_descend_400。 */
        .head_descend_400 = {
            .photo_stop_settle_ms = 100u,
            .descend_start_pole_lift_threshold = 2.0f,
            /*
             * 固定起步优化流程：
             * - mid_move_speed/mid_move_ms：第一次固定快速靠近。
             * - rear_retract_move_speed：慢速捕获 photo3 下降沿的速度。
             * - rear_retract_move_ms：前杆伸出后的中段固定快速移动时间。
             * - front_retract_move_speed：慢速捕获 photo1 下降沿的速度。
             * - pole_extend_move_speed/hold_ms：四杆支撑通过。
             * - 模板内伸杆 step 的 vx 命令为 0。
             */
            /* AutoCtrlTemplate_RunHeadDescend400Optimized 使用的有效字段。 */
            .prealign_move_speed = 0.20f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .front_retract_move_speed = 0.1f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .mid_move_speed = 1.50f,            /* 中段平移 vx，单位 m/s。 */
            .mid_move_ms = 150u,                /* 中段平移持续时间，单位 ms。 */
            .timed_move_yaw_tolerance_rad = 0.35f, /* 中段定时移动切步 yaw 容差，约 10 deg。 */
            .rear_retract_move_speed = 0.1f,   /* 后杆动作阶段 vx，单位 m/s。 */ 
            .rear_retract_move_ms = 250u,       /* 后杆动作后继续移动时间，单位 ms。 */
            .rear_retract_move_wheel_delta_rad = 8.66f, /* 后杆动作后继续移动轮转角阈值，单位 rad；0 表示按时间切步。 */
            .second_photo_retract_move_speed = 0.50f, /* step7 第二个下降沿后全收杆离开 vx，单位 m/s。 */
            .final_move_speed = 0.5f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 200u,             /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_front_extend_speed = 65.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 40.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 65.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 40.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .pole_lift_accel = 500.0f,          /* 当前模板撑杆加速度限幅，单位 rad/s^2。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发/下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 5000u,     /* 等待后光电触发/下降沿超时，单位 ms。 */
            .pole_extend_move_speed = 1.5f,    /* step6 四杆全伸行走 vx，单位 m/s。 */
            .hold_ms = 100u,                    /* step6 四杆全伸行走持续时间，单位 ms。 */
        },
        /* 尾向 / 上台阶 / 200mm 模板参数。 */
        /* 模块参数：尾向上台阶 200mm tail_ascend_200。 */
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
            .timed_move_yaw_tolerance_rad = 0.1745329252f, /* 中段定时移动切步 yaw 容差，约 10 deg。 */
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
            .pole_lift_accel = 0.0f,            /* 当前模板撑杆加速度；0 使用 Pole 默认值。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发/下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 10000u,    /* 等待后光电触发/下降沿超时，单位 ms。 */
        },
        /* 尾向 / 上台阶 / 400mm 模板参数。 */
        /* 模块参数：尾向上台阶 400mm tail_ascend_400。 */
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
            .timed_move_yaw_tolerance_rad = 0.1745329252f, /* 中段定时移动切步 yaw 容差，约 10 deg。 */
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
            .pole_lift_accel = 0.0f,            /* 当前模板撑杆加速度；0 使用 Pole 默认值。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发/下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 10000u,     /* 等待后光电触发/下降沿超时，单位 ms。 */
        },
        /* 尾向 / 下台阶 / 200mm 模板参数。 */
        /* 模块参数：尾向下台阶 200mm tail_descend_200。 */
        .tail_descend_200 = {
            .photo_stop_settle_ms = 100u,
            .descend_start_pole_lift_threshold = 2.0f,
            /*
             * 固定起步优化流程：
             * - mid_move_speed/mid_move_ms：第一次固定快速靠近。
             * - rear_retract_move_speed：慢速捕获 photo3 下降沿的速度。
             * - rear_retract_move_ms：后杆伸出后的中段固定快速移动时间。
             * - front_retract_move_speed：慢速捕获 photo1 下降沿的速度。
             * - pole_extend_move_speed/hold_ms：四杆支撑通过。
             * - 模板内伸杆 step 的 vx 命令为 0。
             */
            /* AutoCtrlTemplate_RunTailDescendOptimized 使用的有效字段。 */
            .prealign_move_speed = 0.20f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .pole_extend_move_speed = 0.20f,    /* 四杆全伸支撑通过 vx，单位 m/s。 */
            .front_retract_move_speed = 0.05f,  /* 慢速捕获 photo1 下降沿 vx，单位 m/s。 */
            .mid_move_speed = 0.80f,            /* 固定快跑 vx，单位 m/s。 */
            .mid_move_ms = 20u,                /* 第一次固定快跑持续时间，单位 ms。 */
            .timed_move_yaw_tolerance_rad = 0.1745329252f, /* 中段定时移动切步 yaw 容差，约 10 deg。 */
            .rear_retract_move_speed = 0.05f,   /* 慢速捕获 photo3 下降沿 vx，单位 m/s。 */
            .rear_retract_move_ms = 20u,       /* 后杆伸出后的中间固定快跑时间，单位 ms。 */
            .rear_retract_move_wheel_delta_rad = 0.0f, /* 后杆伸出后的中间固定快跑轮转角阈值，单位 rad；0 表示按时间切步。 */
            .second_photo_retract_move_speed = 0.10f, /* 第二个下降沿后收腿时向尾向移动 vx，单位 m/s。 */
            .final_move_speed = 0.10f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 500u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_front_extend_speed = 50.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 25.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 50.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 25.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .pole_lift_accel = 140.0f,          /* 当前模板撑杆加速度限幅，单位 rad/s^2。 */
            .front_photo_timeout_ms = 5000u,    /* 等待 photo1 下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 5000u,     /* 等待 photo3 下降沿超时，单位 ms。 */
            .hold_ms = 1200u,                   /* 四杆全伸支撑通过时间，单位 ms。 */
        },
        /* 尾向 / 下台阶 / 400mm 模板参数。 */
        /* 模块参数：尾向下台阶 400mm tail_descend_400。 */
        .tail_descend_400 = {
            .photo_stop_settle_ms = 100u,
            .descend_start_pole_lift_threshold = 2.0f,
            /*
             * 固定起步优化流程：
             * - mid_move_speed/mid_move_ms：第一次固定快速靠近。
             * - rear_retract_move_speed：慢速捕获 photo3 下降沿的速度。
             * - rear_retract_move_ms：后杆伸出后的中段固定快速移动时间。
             * - front_retract_move_speed：慢速捕获 photo1 下降沿的速度。
             * - pole_extend_move_speed/hold_ms：四杆支撑通过。
             * - 模板内伸杆 step 的 vx 命令为 0。
             */
            /* AutoCtrlTemplate_RunTailDescendOptimized 使用的有效字段。 */
            .prealign_move_speed = 0.20f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .pole_extend_move_speed = 0.25f,    /* 撑杆伸出阶段 vx，单位 m/s。 */
            .front_retract_move_speed = 0.10f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .mid_move_speed = 0.35f,            /* 中段平移 vx，单位 m/s。 */
            .mid_move_ms = 200u,               /* 中段平移持续时间，单位 ms。 */
            .timed_move_yaw_tolerance_rad = 0.1745329252f, /* 中段定时移动切步 yaw 容差，约 10 deg。 */
            .rear_retract_move_speed = 0.10f,   /* 后杆动作阶段 vx，单位 m/s。 */
            .rear_retract_move_ms = 700u,       /* 后杆动作后继续移动时间，单位 ms。 */
            .rear_retract_move_wheel_delta_rad = 0.0f, /* 后杆动作后继续移动轮转角阈值，单位 rad；0 表示按时间切步。 */
            .final_move_speed = 0.10f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 500u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_front_extend_speed = 60.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 25.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 60.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 25.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .pole_lift_accel = 140.0f,          /* 当前模板撑杆加速度限幅，单位 rad/s^2。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发/下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 5000u,     /* 等待后光电触发/下降沿超时，单位 ms。 */
            .hold_ms = 1200u,                   /* 四杆全伸支撑通过时间，单位 ms。 */
        },
    },
    /* 模块参数：取矛头机构 rod_new_param，舵机、夹爪和到位参数。 */
    .rod_new_param = {
        .servo = {
            .pwm_channel = BSP_PWM_ROD_SERVO,
            .freq_hz = 50.0f,
            .zero_pulse_us = ROD_NEW_SERVO_PULSE_NEUTRAL_US,
            /* Rod 舵机软件行程：0.0 -> 776us 等待位，1.0 -> 1504us 水平位。 */
            .angle_standby_rad = 1.0f,
            .angle_grab_high_rad = 0.35f,
            .angle_dock_wait_rad = 0.0f,
            .angle_min_rad = 0.0f,
            .angle_max_rad = 1.0f,
            .arrive_threshold_rad = 0.05f,  /* 到位判定阈值 */
            .max_vel_rad_s = 2.0f,           /* 最大角速度 */
            .max_acc_rad_s = 5.0f,          /* 最大角加速度 */
        },
        .gripper = {
            .gripper_gpio = BSP_GPIO_SPEARHEAD_RELAY,
               .grip_timeout_ms = 2000u,        /* 夹取超时 */
        },
    },
    /* 模块参数：相机 yaw camera_yaw_param，相机云台电机、PID 和限幅。 */
    .camera_yaw_param = {
        [CAMERA_YAW_LEFT] = {
            .motor_param = {
                .can = BSP_CAN_2,
                .id = 0x209,
                .module = MOTOR_GM6020,
                .reverse = false,
                .gear = false,
            },
            .encoder_zero_offset_rad = 0.562970936f,
            .pid = {
                .yaw_pid = {
                    .k = 1.0f,
                    .p = 0.40f,
                    .i = 0.0f,
                    .d = 0.0f,
                    .i_limit = 0.0f,
                    .out_limit = 0.80f,
                    .d_cutoff_freq = -1.0f,
                    .range = M_2PI,
                },
            },
            .limit = {
                .max_output = 0.80f,
                .arrive_threshold_rad = 0.02f,
                .feedback_timeout_ms = 200u,
            },
        },
        [CAMERA_YAW_RIGHT] = {
            .motor_param = {
                .can = BSP_CAN_2,
                .id = 0x20A,
                .module = MOTOR_GM6020,
                .reverse = false,
                .gear = false,
            },
            .encoder_zero_offset_rad = 1.61221385f,
            .pid = {
                .yaw_pid = {
                    .k = 1.0f,
                    .p = 0.40f,
                    .i = 0.0f,
                    .d = 0.0f,
                    .i_limit = 0.0f,
                    .out_limit = 0.80f,
                    .d_cutoff_freq = -1.0f,
                    .range = M_2PI,
                },
            },
            .limit = {
                .max_output = 0.80f,
                .arrive_threshold_rad = 0.02f,
                .feedback_timeout_ms = 200u,
            },
        },
    },
};

Config_RobotParam_t *Config_GetRobotParam(void) {
  return &robot_config;
}
