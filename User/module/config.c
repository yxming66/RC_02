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
#define CONFIG_SICK_ROD_SPEARHEAD_PARAM(x_target, y_target) \
    { \
        .front_index = SICK_FRONT_PHOTO_INDEX, \
        .rod_front_index = SICK_ROD_SIDE_PHOTO_INDEX, \
        .rear_index = SICK_BOTTOM_PHOTO_INDEX, \
        .rod_rear_index = SICK_ROD_SIDE_PHOTO_INDEX, \
        .valid_adc_min = 0u, \
        .valid_adc_max = 32100u, \
        .x_target_adc = (x_target), \
        .y_target_adc = (y_target), \
        .yaw_target_diff_adc = 0.0f, \
        .x_tolerance_adc = 2.0f, \
        .y_tolerance_adc = 2.0f, \
        .yaw_tolerance_adc = 20.0f, \
        .x_kp_mps_per_adc = -0.0002f, \
        .y_kp_mps_per_adc = 0.0019f, \
        .yaw_kp_rad_s_per_adc = 0.0f, \
        .vx_limit_mps = 0.30f, \
        .vy_limit_mps = 0.30f, \
        .wz_limit_rad_s = 0.80f, \
        .pole_target_lift = 2.0f, \
        .pole_speed = 20.0f, \
        .finish_stable_ms = 100u, \
        .timeout_ms = 2000u, \
    }

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
                .k = 2.8f,
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
                .k = 3.1f,
                .p = 5.0f,
                .i = 0.0f,
                .d = 0.2f,
                .i_limit = 1.0f,
                .out_limit = 6.0f,
                .d_cutoff_freq = 0.0f,
                .range = 0.0f,
            },
        },
        .physical = {
            .wheel_radius_m = 0.0635f,//0.0   635
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
                .k = 40.0f,
                .p = 20.0f,
                .i = 0.0f,
                .d = 0.0f,
                .i_limit = 0.0f,
                .out_limit = 800.0f,
                .d_cutoff_freq = -1.0f, 
                .range = 0.0f,
            },
            .support_vel_pid = {
                .k = 0.15f,
                .p = 0.18f,
                .i = 0.0f,
                .d = 0.008f,
                .i_limit = 0.0f,
                .out_limit = 0.95f,
                .d_cutoff_freq = -1.0f,
                .range = 0.0f,
            },
        },
        .preset = {
            /* 200mm 上台阶四杆全伸位；一键取矿 PICK_POS_200 也使用该撑杆高度。 */
            .step_200_all_extend = {5.65490055f, 5.65490055f},
            /* 200mm 上台阶前杆收回、后杆保持支撑位。 */
            .step_200_front_retract = {0.2f, 5.65490055f},
            /* 200mm 上台阶四杆全收位。 */
            .step_200_all_retract = {0.2f, 0.2f},
            /* 200mm 上台阶小抬升位；一键取矿 PICK_NEG_200 使用该撑杆高度。 */
            .step_200_small = {5.60490055f, 5.60490055f},
            /* 400mm 上台阶四杆全伸位；一键取矿 PICK_POS_400 也使用该撑杆高度。 */
            .step_400_all_extend = {10.8f, 10.8f},
            /* 400mm 上台阶前杆收回、后杆保持支撑位。 */  
            .step_400_front_retract = {0.05f, 10.8f},
            /* 400mm 上台阶四杆全收位。 */
            .step_400_all_retract = {0.05f, 0.05f},

            /* 200mm 下台阶四杆全伸位。 */
            .step_200_descend_all_extend = {5.0929637f, 5.0929637f},
            /* 200mm 下台阶前杆收回、后杆保持支撑位。 */
            .step_200_descend_front_retract = {0.1f, 5.0929637f},
            /* 200mm 下台阶四杆全收位。 */
            .step_200_descend_all_retract = {0.1f, 0.1f},
            /* 200mm 下台阶起步/小抬升位。 */
            .step_200_descend_small = {0.1f, 0.1f},
            /* 400mm 下台阶四杆全伸位。 */
            .step_400_descend_all_extend = {10.75f, 10.75f},
            /* 400mm 下台阶前杆收回、后杆保持支撑位。         */
            .step_400_descend_front_retract = {0.08f, 10.75f},
            /* 400mm 下台阶四杆全收位。 */
            .step_400_descend_all_retract = {0.08f, 0.08f},
            /* 一键放矿撑杆目标位*/
            .ore_release_target = {10.6f, 10.6f},
            .ore_release_speed = 10.0f,
            .ore_release_accel = 80.0f, 
        },
        .limit = {
            .max_current = 1.0f,
            .support_total_travel = 10.75f,//26.7//27.3
            .support_min_target_lift = 0.1f,
            .support_lift_speed = 0.0f,
            .support_lift_accel = 0.0f,
        },
    },
    /* 模块参数：矿仓 ore_store_param，矿仓平台电机、回零、预设位置和气缸。 */
    .ore_store_param = {
        .motor_param = {
            [ORE_STORE_AXIS_PLATFORM] = {.can = BSP_CAN_1, .id = 0x205, .module = MOTOR_M3508, .reverse = false, .gear = true},
        },
        .motor_install = {
            [ORE_STORE_AXIS_PLATFORM] = {.external_ratio = 1.0f, .reverse_output = false},
        },
        .pid = {
            .position_pid = {    
                [ORE_STORE_AXIS_PLATFORM] = {.k = 1.0f, .p = 28.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 80.0f, .d_cutoff_freq = 80.0f, .range = 0.0f},
            },
            .velocity_pid = {
                [ORE_STORE_AXIS_PLATFORM] = {.k = 0.1f, .p = 2.550f, .i = 0.02f, .d = 0.0f, .i_limit = 0.35f, .out_limit = 4.5f, .d_cutoff_freq = 18.0f, .range = 0.0f},
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
            .move_velocity_rad_s = {150.0f},
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
                /* 平台抬升位/满行程位，用于  低位存矿或低位上膛交接。 */
                [ORE_STORE_TRANSFORM_LIFT] = 18.4731674f,
                /* 平台等待矛头对接位置 */ 
                [ORE_STORE_TRANSFORM_SPEARHEAD_DOCK_WAIT] = 0.0f,//0.183727468f,
                /* 取矛头平台预设位 */
                [ORE_STORE_TRANSFORM_SPEARHEAD_PICKUP] = 22.2083613//23.2407284f//22.3003597f,
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
            .center_angle_rad = -0.0769318342f,
        },
        .suction_param = {
            .gpio = BSP_GPIO_ORE_RELAY,
        },
        .mit = {
            .joint1_kp = 300.0f,
            .joint1_kd = 200.0f,
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
            .joint1_max_vel = 6.5f,
            .joint2_max_vel = 20.0f,
            .joint1_max_accel = 20.0f,
            .joint2_max_accel = 60.0f,
        },
        .preset = {
            .behavior_point = {
                /* ArmSimple 待机/安全收回位，一键动作开始和结束常用位置。 */
                [ARM_SIMPLE_BEHAVIOR_STANDBY] = {.joint1_pos = 0.224461725f, .joint2_pos = -0.252654374f},
                /* ArmSimple 存矿位，arm 伸到 ore_store 交接处释放矿。 */
                [ARM_SIMPLE_BEHAVIOR_STORE_ORE] = {.joint1_pos = 0.0f, .joint2_pos = -1.51068195f},
                /* ArmSimple 上膛位，arm 伸到 ore_store 上膛交接处吸取矿。 */
                [ARM_SIMPLE_BEHAVIOR_CHAMBER_ORE] = {.joint1_pos = -0.112332456f, .joint2_pos = -1.5956496f},
                /* ArmSimple 存矿等待位，等待平台到位后再伸到存矿位。 */
                [ARM_SIMPLE_BEHAVIOR_WAIT_STORE_ORE] = {.joint1_pos = 0.246562153f, .joint2_pos = -1.82005286f},
                /* ArmSimple 放矿等待位，放矿动作前的预备/稳定位置。 */
                [ARM_SIMPLE_BEHAVIOR_WAIT_RELEASE_ORE] = {.joint1_pos = -0.236811638f, .joint2_pos =2.12289429f},
                /* ArmSimple 放矿位，吸盘关闭后从该位置把矿释放出去。 */
                [ARM_SIMPLE_BEHAVIOR_RELEASE_ORE] = {.joint1_pos = 0.274778366f, .joint2_pos = 0.72164762f},
                /* ArmSimple 正向 400mm 取矿位，配合 pole 的 step_400_all_extend 使用。 */ 
                [ARM_SIMPLE_BEHAVIOR_PICK_POS_400] = {.joint1_pos = 1.53361797f, .joint2_pos = 0.0f},
                /* ArmSimple 正向 200mm 取矿位，配合 pole 的 step_200_all_extend 使用。 */
                [ARM_SIMPLE_BEHAVIOR_PICK_POS_200] = {.joint1_pos = 1.53361797f, .joint2_pos = 0.0f},
                /* ArmSimple 反向/负向 200mm 取矿位，配合 pole 的 step_200_small 使用。 */
                [ARM_SIMPLE_BEHAVIOR_PICK_NEG_200] = {.joint1_pos = 1.92628717f, .joint2_pos = -0.39897722f},
                /* 融合取矿：抬矿检测/安全携带位，实车确认矿石离地间隙后再标定。 */
                [ARM_SIMPLE_BEHAVIOR_PICK_LIFT_DETECT] = {.joint1_pos = 1.20f, .joint2_pos = -0.35f},
                     /* ArmSimple 放矿辅助进位，放矿流程从等待位先经过该位置再进放矿位。 */
                     [ARM_SIMPLE_BEHAVIOR_RELEASE_ORE_ASSIST] = {.joint1_pos = 0.160879135f, .joint2_pos = 1.25246606f},
                 /* ArmSimple 竖直避障位，三层放矿抬升过程中保持矿石不与前方场地干涉。 */
                 [ARM_SIMPLE_BEHAVIOR_VERTICAL] = {.joint1_pos = 0.0f, .joint2_pos = 0.0f},
            },
            .arrive_threshold_rad = 0.05f,
        },
    },
    /* 模块参数：一键存取矿 auto_ore_param，存矿、放矿、上膛、取矿流程参数。 */
    .auto_ore_param = {
        /* ArmSimple 一键行为速度上限，单位 rad/s；<=0 表示使用 arm_simple_param.vel_limit 默认值。 */
        .arm_speed = {
            /* store_*：一键存矿流程，wait=等待/预备位，place=伸到存矿位，standby=回待机位。 */
            .store_wait = {.joint1_max_vel_rad_s = 2.0f, .joint2_max_vel_rad_s = 4.5f},
            .store_place = {.joint1_max_vel_rad_s = 2.0f, .joint2_max_vel_rad_s = 4.5f},
            .store_standby = {.joint1_max_vel_rad_s = 2.0f, .joint2_max_vel_rad_s = 4.5f},
            /* release_*：一键放矿流程，wait=放矿前等待位，assist=放矿辅助进位，place=放矿位，standby=放矿后回待机位。 */
            .release_wait = {.joint1_max_vel_rad_s = 3.0f, .joint2_max_vel_rad_s = 5.0f},
            .release_assist = {.joint1_max_vel_rad_s = 3.0f, .joint2_max_vel_rad_s = 5.0f},
            .release_place = {.joint1_max_vel_rad_s = 3.0f, .joint2_max_vel_rad_s = 5.0f},
            .release_standby = {.joint1_max_vel_rad_s = 3.0f, .joint2_max_vel_rad_s = 5.0f},
            /* chamber_*：一键上膛流程，wait=对接等待位，place=取/交接位，standby=上膛后回待机位。 */
            .chamber_wait = {.joint1_max_vel_rad_s = 8.0f, .joint2_max_vel_rad_s = 4.0f},
            .chamber_place = {.joint1_max_vel_rad_s = 8.0f, .joint2_max_vel_rad_s = 6.0f},
            .chamber_standby = {.joint1_max_vel_rad_s = 8.0f, .joint2_max_vel_rad_s = 4.0f},
            /* pick_*：一键取矿流程，standby=取矿前/后待机位，place=伸到取矿位，fetch=底盘前进取矿时保持取矿位。 */
            .pick_standby = {
                .joint1_max_vel_rad_s = 3.0f,
                .joint2_max_vel_rad_s = 2.0f,
                .joint1_max_accel_rad_s2 = 25.0f,
                .joint2_max_accel_rad_s2 = 50.0f,
            },
            .pick_place = {
                .joint1_max_vel_rad_s = 6.0f,
                .joint2_max_vel_rad_s = 4.0f,
                .joint1_max_accel_rad_s2 = 30.0f,
                .joint2_max_accel_rad_s2 = 60.0f,
            },
            .pick_fetch = {
                .joint1_max_vel_rad_s = 4.5f,
                .joint2_max_vel_rad_s = 4.0f,
                .joint1_max_accel_rad_s2 = 30.0f,
                .joint2_max_accel_rad_s2 = 60.0f,
            },
            .pick_lift_detect = {
                .joint1_max_vel_rad_s = 6.0f,
                .joint2_max_vel_rad_s = 4.0f,
                .joint1_max_accel_rad_s2 = 30.0f,
                .joint2_max_accel_rad_s2 = 60.0f,
            },
        },
        /* 一键存/放/上膛/取矿流程中的动作延时，单位 ms；<=0 时使用状态机内置默认值。 */
        .timing = {
            /* 存矿：arm 到存矿位后的稳定等待、固矿气缸关闭等待、气缸重新打开等待。 */
            .store_arm_settle_ms = 300u,
            .store_cylinder_close_ms = 200u,
            .store_arm_suction_off_ms = 1500u,  
            .store_cylinder_open_ms = 200u, 
            /* 放矿：放矿前等待、Pole 到位后抬升观测超时、抬升确认后稳定等待、到放矿位后短暂停稳、吸盘关闭后矿石脱离等待。 */
            .release_wait_ms = 150u,
            .release_lift_detect_timeout_ms = 20000u,
            .release_lift_detect_settle_ms = 250u,//三层矿触发观测阈值延时放矿
            .release_arm_settle_ms = 10u,
            .release_suction_off_ms = 200u,
            /* 上膛：低位矿夹紧等待、arm 到交接位稳定等待、气缸打开释放等待。 */
            .chamber_low_clamp_ms = 400u,
            .chamber_arm_settle_ms = 400u,
            .chamber_cylinder_open_ms = 400u,
            /* 取矿：正向 200/400 取矿时，底盘向矿位前进的持续时间。 */ 
            .fetch_chassis_move_ms = 1000u,
            /* 取 -200 矿：底盘向矿位前进的独立持续时间；可设为 0 禁止底盘前进，避免走太多掉下去。 */
            .fetch_neg_200_chassis_move_ms = 1000u,
            /* 回收地面矿：前进最长等待、前 SICK 触发后续行、静止吸附和后退时间。 */
            .recover_chassis_forward_ms = 3000u, /* 前 SICK 未触发时的安全超时。 */
            .recover_front_sick_delay_ms = 500u,  /* 前 SICK 达阈值后继续前进时间。 */
            .recover_suction_settle_ms = 50u,
            .recover_chassis_retreat_ms = 500u,
            /* 动作 9/10/11：前进等待光电1的安全超时，以及光电1有效沿后的前进/后退续行时间。 */
            .pick_store_chassis_forward_timeout_ms = 3000u,
            .pick_store_photo1_forward_delay_ms = 150u,
            .pick_store_photo1_retreat_delay_ms = 150u,
            /* 融合取矿/存矿/上台阶动作延时；0 使用代码默认值。 */
            .fused_prealign_stable_ms = 120u,          /* 融合动作 yaw 对正后稳定等待时间，单位 ms。 */
            .fused_pick_precontact_timeout_ms = 500u,  /* 融合取矿低速靠近兜底超时，避免光电条件未命中时长时间卡住。 */
            .fused_pick_lift_detect_ms = 200u,         /* 融合取矿抬矿检测位到位后的确认延时，单位 ms。 */
            .fused_arm_photo_stable_ms = 120u,         /* 机械臂取矿传感器稳定确认时间，单位 ms。 */
            .fused_photo1_lift_delay_ms = 200u,         /* 光电1有效沿确认后立即抬 Arm/开始并行存矿。 */
        },
        /* 单个状态机 step 的超时时间，单位 ms。 */
        .default_step_timeout_ms = 5000u,
        /* 到位判定阈值：arm/ore_store/pole 分别使用的误差阈值，单位 rad。 */
        .arm_arrive_threshold_rad = 0.5f,
        .arm_arrive_velocity_threshold_rad_s = 0.25f,
        .pick_arm_arrive_stable_ms = 50u,
        .ore_store_arrive_threshold_rad = 0.05f,
        .pole_arrive_threshold_rad = 1.50f,
        .prealign_yaw_tolerance_rad =
            CONFIG_AUTO_ORE_PREALIGN_YAW_TOLERANCE_RAD,
        .release_lift_detect_sick_index = SICK_BOTTOM_PHOTO_INDEX,
        .release_lift_detect_sick_adc_threshold = 3200u,
        .release_lift_detect_sick_greater_than_threshold = true,
        /* 动作 33 专用：向地面矿靠近时，前 SICK ADC <= 此值即触发。 */
        .recover_front_sick_adc_threshold = 2950u,
        /* 取矿流程中正向 200/400 取矿的底盘前进速度，单位 m/s；与融合上台阶取矿靠近速度保持一致。 */
        .fetch_chassis_vx_mps = 0.20f,
        /* 取 -200 矿时底盘前进速度，单位 m/s；可设为 0 禁止底盘前进。 */
        .fetch_neg_200_chassis_vx_mps = 0.20f,
        /* 回收地面矿动作全程保持的 Pole 自动目标：[0]前组，[1]后组，单位 rad。 */
        .recover_pole_target_lift_rad = {0.8f, 0.8f},
        /* 回收地面矿时底盘低速前进/后退速度，单位 m/s。 */
        .recover_chassis_forward_vx_mps = 0.25f,
        .recover_chassis_retreat_vx_mps = 0.5f,
        /* 动作 9/10/11 专用：取矿前进和并行存矿后退速度。 */
        .pick_store_chassis_forward_vx_mps = 0.25f,
        .pick_store_chassis_retreat_vx_mps = 0.5f,
        /* 低位存矿完成后，transform 从高位 LIFT 回低位 STANDBY 的旧梯形速度规划；三段速度未配置时兜底使用。 */
        .store_low_return_velocity_rad_s = 35.0f,
        .store_low_return_accel_rad_s2 = 60.0f,
        .store_low_return_decel_rad_s2 = 60.0f,
        /*
         * 低位存矿返回三段速度规划：end_ratio 为该段结束进度 0~1，velocity_rad_s 为该段速度。
         * 最多 3 段；end_ratio/velocity <=0 表示该段关闭；至少一段有效时禁用上面的梯形规划。
         */
        .store_low_return_segments = {
            {0},
            {0},
            {0},
        },
        /*
         * 低位存矿收尾颠动：平台回到 STANDBY 后，向 LIFT 方向小幅快速往返。
         * cycles 为完整“上+下”次数，0 表示关闭；只作用于最终存入低位矿仓的流程。
         */
        .store_low_shake_amplitude_rad = 2.0f,
        .store_low_shake_velocity_rad_s = 50.0f,
        .store_low_shake_cycles = 2u,
        /*
         * 融合动作轮转角阈值说明：
         * - *_wheel_delta_rad 使用四轮累计转角变化绝对值的平均值，单位 rad。
         * - <=0 表示跳过对应轮转角门控或回退到时间门控。
         * - use_arm_photo_confirm 需要真实机械臂取矿传感器；false 表示机械臂到位 + 延时确认。
         */
        .fused_step_pick_store_ascend_200_head = {
            .step_template = AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD, /* 取矿存矿后执行的头向 200mm 上台阶模板。 */
            .pick_action = AUTO_ORE_ACTION_PICK_POS_200,         /* 融合动作取正 200mm 矿。 */
            .precontact_vx_mps = 0.20f,                          /* 取矿前低速靠近速度，单位 m/s。 */
            .precontact_wheel_delta_rad = 4.5f,                 /* 取矿前低速靠近轮转角阈值，单位 rad。 */
            .precontact_timeout_ms = 2500u,                      /* 光电未命中时快速进入抬矿检测兜底。 */
            .step_start_vx_mps = 0.30f,                          /* 存矿后进入台阶模板前的起步冲刺速度，单位 m/s。 */
            .step_start_wheel_delta_rad = 2.36f,                 /* 存矿后进入台阶模板前的起步冲刺轮转角阈值，单位 rad。 */
            .fast_pick_on_front_photo = true,                    /* true=前光电触发即认为取矿完成，跳过停顿抬矿检测并直接并行存矿/上台阶。 */
            .use_arm_photo_confirm = false,                      /* false=机械臂到位加延时确认，true=机械臂取矿传感器确认。 */
        },
        .fused_step_pick_store_descend_200_head = {
            .step_template = AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD, /* 取矿存矿后执行的头向 200mm 下台阶模板。 */
            .pick_action = AUTO_ORE_ACTION_PICK_NEG_200,          /* 融合动作取负 200mm 矿。 */
            .precontact_vx_mps = 0.20f,                           /* Arm 下放及取矿前低速靠近速度，单位 m/s。 */
            .precontact_wheel_delta_rad = 4.0f,                  /* 取矿前低速靠近轮转角阈值，单位 rad。 */
            .precontact_timeout_ms = 2500u,                       /* 光电未命中时快速进入抬矿检测兜底。 */
            .step_start_vx_mps = 0.20f,                           /* 存矿后进入台阶模板前的起步冲刺速度，单位 m/s。 */
            .step_start_wheel_delta_rad = 0.0f,                   /* 头向下台阶默认不额外起步冲刺；>0 时按轮转角门控冲刺。 */
            .override_descend_move = true,
            .descend_mid_move_ms = 0u,
            .descend_rear_retract_move_ms = 250u,
            .descend_rear_retract_move_wheel_delta_rad = 8.66f,
            .fast_pick_on_front_photo = true,                     /* true=photo1 稳定下降沿即认为取矿完成，跳过停顿抬矿检测并直接并行存矿/下台阶。 */
            .use_arm_photo_confirm = false,                       /* false=机械臂到位加延时确认，true=机械臂取矿传感器确认。 */
        },
        .fused_step_pick_store_ascend_400_head = {
            .step_template = AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD, /* 取矿存矿后执行的头向 400mm 上台阶模板。 */
            .pick_action = AUTO_ORE_ACTION_PICK_POS_400,         /* 融合动作取正 400mm 矿。 */
            .precontact_vx_mps = 0.15f,                          /* 取矿前低速靠近速度，单位 m/s。 */
            .precontact_wheel_delta_rad = 4.5f,                 /* 取矿前低速靠近轮转角阈值，单位 rad。 */
            .precontact_timeout_ms = 2500u,                      /* 光电未命中时快速进入抬矿检测兜底。 */
            .step_start_vx_mps = 0.15f,                          /* 存矿后进入台阶模板前的起步冲刺速度，单位 m/s。 */
            .step_start_wheel_delta_rad = 2.36f,                 /* 存矿后进入台阶模板前的起步冲刺轮转角阈值，单位 rad。 */
            .fast_pick_on_front_photo = true,                    /* true=前光电触发即认为取矿完成，跳过停顿抬矿检测并直接并行存矿/上台阶。 */
            .use_arm_photo_confirm = false,                      /* false=机械臂到位加延时确认，true=机械臂取矿传感器确认。 */
        },
    },
    /* 模块参数：一键取矛头 auto_rod_spearhead_param，取矛头机构动作时序和检测。 */
    .auto_rod_spearhead_param = {
        /* One-key spearhead action timing; rod_param is filled during init. */
        .open_delay_ms = 20u,
        .grab_high_delay_ms = 250u,
        .detect_grip_delay_ms = 300u,
        .detect_pose_delay_ms = 600u,
        .detect_success_hold_ms = 200u,
        /* 取矛头 step1/step2 等待平台到取矛头位的移动超时；动作 15 对接等待本身不超时。 */
        .dock_wait_delay_ms = 10000u,
        /* 等待对接时 transform 的目标点；实际角度在 ore_store_param.preset.transform_position_rad 中标定。 */
        .dock_wait_transform = ORE_STORE_TRANSFORM_SPEARHEAD_DOCK_WAIT,
        .use_photo_check = true,
        .photo_check_ms = 150u,
        .photo_timeout_ms = 1500u,
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
            .sick_front_ore_adc_min = 2739u, /* 前 SICK 矿检测 ADC 下限；按实测修正。 */
            .sick_front_ore_adc_max = 4206u, /* 前 SICK 矿检测 ADC 上限；按实测修正。 */
            .sick_front_ore_stable_ms = 60u, /* 前 SICK 矿检测稳定确认时间，单位 ms。 */
        },
        /*
         * SICK 校正参数：
         * - index 字段选择 SICK 采样板 4 路 ADC 通道。
         *   当前硬件：rawdata[2]=前光电，rawdata[1]=rod 侧光电，rawdata[0]=下光电，rawdata[3]=未用。
         * - 取矛头校正由 auto_sick_correct.c 的 X/Y 轴宏开关控制；
         *   当前 x/y 两轴都关闭，动作会直接返回成功。
         *   重新开启时，y 使用 rod 侧光电 rawdata[1]，x 使用前光电 rawdata[2]。
         * - yaw_sample_diff_adc 保留给其他 SICK 校正动作调试。
         * - *_kp 字段把 ADC 误差映射到底盘速度；*_limit 字段做命令限幅。
         *   tolerance/stable/timeout 用于判定校正完成。
         */
        /* 模块参数：SICK 一键校正 sick_correct，取矛头/放矿前的 SICK 对位参数。 */
        .sick_correct = {
            .rod_spearhead_position = {
                [0] = CONFIG_SICK_ROD_SPEARHEAD_PARAM(666.0f,  1021.0f),
                [1] = CONFIG_SICK_ROD_SPEARHEAD_PARAM(1599.0f, 1021.0f),
                [2] = CONFIG_SICK_ROD_SPEARHEAD_PARAM(2566.0f, 1021.0f),
                [3] = CONFIG_SICK_ROD_SPEARHEAD_PARAM(3500.0f, 1021.0f),
                [4] = CONFIG_SICK_ROD_SPEARHEAD_PARAM(4406.0f, 1021.0f),
                [5] = CONFIG_SICK_ROD_SPEARHEAD_PARAM(5377.0f, 1021.0f),
            },
            /* 放矿前 SICK 校正：只使用前光电 rawdata[2] 做 x 方向校正。 */
            .ore_release = {  
                .front_index = SICK_FRONT_PHOTO_INDEX,
                .rod_front_index = SICK_ROD_SIDE_PHOTO_INDEX,
                .rear_index = SICK_BOTTOM_PHOTO_INDEX,
                .rod_rear_index = SICK_ROD_SIDE_PHOTO_INDEX,
                .valid_adc_min = 0u,               /* 有效 ADC 下限。 */
                .valid_adc_max = 32100u,             /* 有效 ADC 上限。 */
                .x_target_adc = 1079.0f,             /* 前光电 rawdata[2] 的 x 目标 ADC。 */
                .y_target_adc = 1.0f,                /* 放矿校正不使用 y，仅保持参数有效。 */
                .yaw_target_diff_adc = 0.0f,         /* 当前硬件没有成对 yaw 传感器。 */
                .x_tolerance_adc = 100.0f,           /* 允许的 x ADC 误差。     */
                .y_tolerance_adc = 1.0f,             /* 放矿校正不使用 y。 */
                .yaw_tolerance_adc = 30.0f,          /* 允许的 yaw/z ADC 误差。 */
                .x_kp_mps_per_adc = -0.0010f,         /* x 误差到 vx 的增益。 */
                .y_kp_mps_per_adc = 0.0f,            /* 放矿校正不输出 vy。 */
                .yaw_kp_rad_s_per_adc = 0.0f,        /* 放矿校正不输出 wz。 */
                .vx_limit_mps = 0.30f,               /* vx 命令限幅，单位 m/s。 */
                .vy_limit_mps =  0.30f,               /* vy 命令限幅，单位 m/s。 */
                .wz_limit_rad_s = 0.80f,             /* wz 命令限幅，单位 rad/s。 */
                .pole_target_lift = 3.0f,            /* 校正阶段撑杆目标高度。 */
                .pole_speed = 20.0f,                 /* 校正阶段撑杆速度。 */
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
            .pole_extend_move_speed = 0.25f,    /* 撑杆伸出阶段 vx，单位 m/s。 */
            .front_retract_move_speed = 0.3f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .front_retract_timeout_ms = 5000u,  /* 前光电触发后，等待前杆收回到位超时，单位 ms。 */
            .mid_move_speed = 1.0f,             /* 前杆收回到位后的中段平移 vx，单位 m/s。 */
            .mid_move_ms = 120u,               /* 中段角度门控兜底超时，单位 ms。 */
            .mid_move_wheel_delta_rad = 10.66f, /* 编码器门控的中段冲刺轮转角阈值，单位 rad；>0 优先按角度切步，<=0 使用 mid_move_ms。 */
            .timed_move_yaw_tolerance_rad = 0.35f, /* 中段移动切步 yaw 容差，约 10 deg。 */
            .rear_retract_move_speed = 0.40f,   /* 等待后光电触发及触发后延时阶段的低速 vx，单位 m/s。 */
            .rear_retract_timeout_ms = 5000u,   /* 后光电触发后，全收腿动作超时，单位 ms。 */
            .rear_retract_move_ms = 300u,       /* 后光电触发后，全收腿移动持续时间，单位 ms。 */
            .second_photo_retract_move_speed = 0.40f, /* 后一个光电触发收腿时向头向移动 vx，单位 m/s。 */
            .final_move_speed = 0.6f,          /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 1200u,              /* 收尾离开台阶角度门控兜底超时，单位 ms。 */
            .final_photo_sprint_ms = 50u,       /* 末尾光电触发后继续冲刺时间，单位 ms。 */
            .final_move_wheel_delta_rad = 0.0f, /* 编码器门控的收尾离开轮转角阈值，单位 rad；>0 优先按角度切步，<=0 使用 final_move_ms。 */
            /* 200mm全伸：快速建立速度，并在目标前按120rad/s^2主动制动。 */
            .pole_all_extend_speed = 15.0f,
            .pole_front_extend_speed = 22.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 50.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 22.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 60.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .pole_all_extend_accel = 120.0f,
            .pole_all_retract_speed = 30.0f,
            .pole_all_retract_accel = 600.0f,
            .pole_front_extend_accel = 120.0f,
            .pole_front_retract_accel = 1000.0f,
            .pole_rear_extend_accel = 120.0f,
            .pole_rear_retract_accel = 1500.0f,
            
            /* 当前模板撑杆加速度限幅，单位 rad/s^2。 */
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
            .pole_extend_move_speed = 0.1f,    /* 撑杆伸出阶段 vx，单位 m/s。 */
            .front_retract_move_speed = 0.2f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .front_retract_timeout_ms = 5000u,  /* 前光电触发后，等待前杆收回到位超时，单位 ms。 */
            .mid_move_speed = 0.6f,             /* 前杆收回到位后的中段平移 vx，单位 m/s。 */
            .mid_move_ms = 250u,                /* 中段平移持续时间，单位 ms。 */
            .mid_move_wheel_delta_rad = 8.66f,   /* 0 表示该模板继续按时间切步。 */
            .timed_move_yaw_tolerance_rad = 0.35f, /* 中段定时移动切步 yaw 容差，约 10 deg。 */
            .rear_retract_move_speed = 0.3f,   /* 等待后光电触发的低速 vx，单位 m/s。 */
            .rear_retract_timeout_ms = 5000u,   /* 后光电触发后，全收腿动作超时，单位 ms。 */
            .rear_retract_move_ms = 300u,       /* 后光电触发后，全收腿移动持续时间，单位 ms。 */
            .second_photo_retract_move_speed = 0.0f, /* 后一个光电触发收腿时向头向移动 vx，单位 m/s。*/
            .final_move_speed = 1.2f,           /* 收尾离开台阶 vx，单位 m/s。 */
            .final_move_ms = 1000u,              /* 收尾离开台阶持续时间，单位 ms。 */
            .final_photo_sprint_ms = 100u,       /* 末尾光电触发后继续冲刺时间，单位 ms。 */
            .final_move_wheel_delta_rad = 0.0f, /* 0 表示该模板继续按时间切步。 */
            /* 400mm伸出：约0.67s完成10.7rad行程，末端提前约2.02rad制动。 */
            .pole_all_extend_speed = 15.0f,
            .pole_front_extend_speed = 22.0f,   /* 前杆伸出目标跟随速度，单位 rad/s。 */
            .pole_front_retract_speed = 50.0f,  /* 前杆回收目标跟随速度，单位 rad/s。 */
            .pole_rear_extend_speed = 22.0f,    /* 后杆伸出目标跟随速度，单位 rad/s。 */
            .pole_rear_retract_speed = 50.0f,   /* 后杆回收目标跟随速度，单位 rad/s。 */
            .pole_all_extend_accel = 200.0f,
            .pole_all_retract_speed = 30.0f,
            .pole_all_retract_accel = 120.0f,
            .pole_front_extend_accel = 250.0f,
            .pole_front_retract_accel = 600.0f,
            .pole_rear_extend_accel = 250.0f,
            .pole_rear_retract_accel = 600.0f,
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发/下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 10000u,     /* 等待后光电触发/下降沿超时，单位 ms。 */
        },
        /* 头向 / 下台阶 / 200mm 模板参数。 */
        /* 模块参数：头向下台阶 200mm head_descend_200。 */
        .head_descend_200 = {
            .photo_stop_settle_ms = 0u,
            .descend_start_pole_lift_threshold = 0.0f,
            /*
     
            
             * 头向下 200mm 优化流程：
             * - step0：使用 mid_move_speed 快速靠近，持续 mid_move_ms。
             * - step1：使用 rear_retract_move_speed 慢速捕获 PA2/photo3 下降沿。
             * - step2：停车并伸前杆，然后等待前杆到位。
             * - step3：使用 mid_move_speed 第二次固定快速靠近，持续 rear_retract_move_ms。
             * - step4：使用 front_retract_move_speed 慢速捕获 PE13/photo1 下降沿。
             * - step5：停车并四杆全伸，然后等待四杆到位。
             * - step6：使用 pole_extend_move_speed 四杆支撑通过，持续 hold_ms。
             * - step7：四杆保持全伸，并使用 second_photo_retract_move_speed 离开，持续 final_move_ms 后结束。
             */
            /* AutoCtrlTemplate_RunHeadDescend200Optimized 使用的有效字段。 */
            .prealign_move_speed = 0.2f,       /* PREALIGN yaw 对正时叠加的前进 vx，单位 m/s。 */
            .front_retract_move_speed = 0.20f,  /* step4 等待 PE13/photo1 下降沿的安全慢速 vx，单位 m/s。 */
            .mid_move_speed = 0.6f,            /* step0/step3 两段固定快跑 vx，单位 m/s。 */
            .mid_move_ms = 200u,                 /* step0 第一次固定快跑持续时间，单位 ms。 */ 
            .timed_move_yaw_tolerance_rad = 0.35f, /* 中段定时移动切步 yaw 容差，约 10 deg。 */
            .rear_retract_move_speed = 0.25f,  /* step1 等待 PA2/photo3 下降沿的安全慢速 vx，单位 m/s。 */
            .rear_retract_move_ms = 150u,      /* step3 第二次固定快跑持续时间，单位 ms。 */
            .rear_retract_move_wheel_delta_rad = 8.66f, /* step3 第二次固定快跑轮转角阈值，单位 rad；0 表示按时间切步。 */
            .second_photo_retract_move_speed = 0.25f, /* step7 第二个下降沿后保持全伸离开 vx，单位 m/s。 */
            .final_move_speed = 0.0f,          /* step7 离开 vx 的备用值；second_photo_retract_move_speed <= 0 时使用。 */
            .final_move_ms = 150u,              /* step7 保持全伸离开持续时间，单位 ms。 */

            .pole_front_extend_speed = 35.0f,   /* step2 前杆伸出、step5-7 四杆全伸时的前杆速度，单位 rad/s。 */
            .pole_front_retract_speed = 30.0f,  /* step0/1 前杆保持或回收到全收目标的速度，单位 rad/s。 */
            .pole_rear_extend_speed = 35.0f,    /* step5-7 四杆全伸时的后杆速度，单位 rad/s。 */
            .pole_rear_retract_speed = 30.0f,   /* step0-4 后杆保持或回收到全收目标的速度，单位 rad/s。 */
            .pole_all_extend_speed = 30.0f,

            .pole_all_extend_accel = 250.0f,
            .pole_all_retract_speed = 30.0f,
            .pole_all_retract_accel = 250.0f,
            .pole_front_extend_accel = 700.0f,
            .pole_front_retract_accel = 30.0f,
            .pole_rear_extend_accel = 700.0f,
            .pole_rear_retract_accel = 30.0f,
            .pole_extend_landing_zone_rad = 0.08f,
            .pole_extend_landing_speed = 15.0f,
            .front_photo_timeout_ms = 5000u,    /* step4 等待 PE13/photo1 下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 5000u,     /* step1 等待 PA2/photo3 下降沿超时，单位 ms。 */
            .pole_extend_move_speed = 1.0f,    /* step6 四杆全伸安全通过 vx，单位 m/s。 */
            .hold_ms = 50u,                    /* step6 四杆全伸行走持续时间，单位 ms。 */
        },
        /* 头向 / 下台阶 / 400mm 模板参数。 */
        /* 模块参数：头向下台阶 400mm head_descend_400。 */
        .head_descend_400 = {
            .photo_stop_settle_ms = 0u,
            .descend_start_pole_lift_threshold = 0.0f,
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
            .prealign_move_speed = 0.10f,       /* PREALIGN 对正阶段叠加 vx，单位 m/s。 */
            .front_retract_move_speed = 0.1f,  /* 前杆动作阶段 vx，单位 m/s。 */
            .mid_move_speed = 0.7f,            /* 中段平移 vx，单位 m/s。 */
            .mid_move_ms = 300u,                /* 中段平移持续时间，单位 ms。 */
            .timed_move_yaw_tolerance_rad = 0.35f, /* 中段定时移动切步 yaw 容差，约 10 deg。 */
            .rear_retract_move_speed = 0.15f,   /* step1 等待首个光电下降沿、伸前 Pole 前的安全 vx，单位 m/s。 */
            .rear_retract_move_ms = 250u,       /* 后杆动作后继续移动时间，单位 ms。 */
            .rear_retract_move_wheel_delta_rad = 8.66f, /* 后杆动作后继续移动轮转角阈值，单位 rad；0 表示按时间切步。 */
            .second_photo_retract_move_speed = 0.8f, /* step7 第二个下降沿后保持全伸安全离开 vx，单位 m/s。 */
            .final_move_speed = 0.8f,          /* 收尾离开台阶备用 vx，单位 m/s。 */
            .final_move_ms = 400u,             /* 收尾离开台阶持续时间，单位 ms。 */
            .pole_front_extend_speed =  25.0f,   /* 400mm伸出速度，单位 rad/s。 */
            .pole_front_retract_speed = 25.0f,  /* 400mm回收速度，单位 rad/s。 */
            .pole_rear_extend_speed  =  25.0f,   /* 400mm伸出速度，单位 rad/s。 */
            .pole_rear_retract_speed =  25.0f,   /* 400mm回收速度，单位 rad/s。 */
            .pole_all_extend_speed = 30.0f,
            .pole_all_extend_accel = 150.0f,
            .pole_all_retract_speed = 30.0f,
            .pole_all_retract_accel = 150.0f,
            .pole_front_extend_accel = 250.0f,    
            .pole_front_retract_accel = 150.0f,
            .pole_rear_extend_accel = 250.0f,
            .pole_rear_retract_accel = 150.0f,
            .pole_extend_landing_zone_rad = 0.08f, /* 前/后 Pole 伸出最后 0.1 rad 进入低速落地段。 */
            .pole_extend_landing_speed = 15.0f,   /* 末段伸出速度，单位 rad/s。 */
            .front_photo_timeout_ms = 5000u,    /* 等待前光电触发/下降沿超时，单位 ms。 */
            .rear_photo_timeout_ms = 5000u,     /* 等待后光电触发/下降沿超时，单位 ms。 */
            .pole_extend_move_speed = 0.7f,    /* step6 四杆全伸安全通过 vx，单位 m/s。 */
            .hold_ms = 100u,                    /* step6 四杆全伸行走持续时间，单位 ms。 */
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
            .angle_detect_rad = 0.25f,
            .angle_dock_wait_rad = 0.0f,
            .angle_min_rad = 0.0f,
            .angle_max_rad = 1.0f,
            .arrive_threshold_rad = 0.05f,  /* 到位判定阈值 */
            .max_vel_rad_s = 8.0f,           /* 最大角速度 */
            .max_acc_rad_s = 16.0f,          /* 最大角加速度 */
        },
        .gripper = {
            .gripper_gpio = BSP_GPIO_SPEARHEAD_RELAY,
               .grip_timeout_ms = 2000u,        /* 夹取超时 */
        },
    },
    /* 模块参数：相机 yaw camera_yaw_param，相机云台电机、原生 MIT 增益和限幅。 */
    .camera_yaw_param = {
        [CAMERA_YAW_RIGHT] = {
            .motor_param = {
                .can = BSP_CAN_2,
                .master_id = 0x13,
                .can_id = 0x03,
                .module = MOTOR_DM_H3510,
                .reverse = false,
            },
            .encoder_zero_offset_rad = 0.0f,
            .mit = {
                .kp = 1.0f,
                .kd = 0.08f,
                .target_velocity_rad_s = 0.0f,
                .torque_ff_nm = 0.0f,
                .max_position_error_rad = 0.35f,
            },
            .limit = {
                .max_torque_nm = 0.45f,
                .arrive_threshold_rad = 0.02f,
                .feedback_timeout_ms = 200u,
            },
        },
    },
};

Config_RobotParam_t *Config_GetRobotParam(void) {
  return &robot_config;
}
