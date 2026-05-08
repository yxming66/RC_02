/*
 * 配置相关
 */

#pragma once

#include <stdint.h>
#include "component/pid.h"
#include "device/motor.h"
#include "device/motor_rm.h"
#include "module/chassis.h"
#include "module/pole.h"
#include "module/arm/arm_control_types.h"

#include "module/rod.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Chassis_Params_t chassis_param;
    Pole_Params_t pole_param;
    Arm_Params_t arm_param;
    struct {
        /* 通用姿态修正参数：所有带自动矫正的模板都会用到 */
        float prealign_kp;                  /* yaw误差映射到wz指令的比例系数 */
        float prealign_wz_limit;            /* yaw自动矫正时wz限幅 (rad/s) */

        /* 简单平移模板参数：AUTO_CTRL_TEMPLATE_FLAT_MOVE */
        float flat_move_speed;              /* 简单平移模板默认前进速度 (m/s) */
        uint32_t flat_move_hold_ms;         /* 简单平移模板默认保持时间 (ms) */

        /* 200台阶/跨越前段参数：起步、撑杆、前段到位判定 */
        float climb_align_forward_speed;    /* 对正阶段同步慢速前进速度 (m/s) */
            float climb_pole_extend_forward_speed; /* 对正成功后，四杆伸起稳定阶段的前进速度 (m/s) */
        float climb_forward_speed;          /* 前段并行动作时的基础前进速度 (m/s) */
        float climb_forward_kick_speed;     /* 起步短促前冲速度，用于提高跨越动量 (m/s) */
        uint32_t climb_forward_kick_ms;     /* 起步短促前冲持续时间 (ms) */
        uint32_t pole_extend_settle_ms;     /* 四杆伸出后等待机构稳定的时间 (ms) */
            float pole_all_extend_lift_speed;   /* 四杆全伸阶段目标跟踪速度 (rad/s) */
            float pole_front_retract_lift_speed; /* 前杆回收阶段目标跟踪速度 (rad/s) */
            float pole_front_extend_lift_speed; /* 前杆放下/重新伸出阶段目标跟踪速度 (rad/s) */
            float pole_rear_retract_lift_speed; /* 后杆回收阶段目标跟踪速度 (rad/s) */
            float pole_rear_extend_lift_speed;  /* 后杆放下/重新伸出阶段目标跟踪速度 (rad/s) */
        uint32_t head_front_photo_timeout_ms;    /* 头向前光电等待超时 (ms) */
        uint32_t tail_front_photo_timeout_ms;    /* 尾向前光电等待超时 (ms) */
        float climb_front_retract_speed;    /* 前杆回收阶段基础前进速度 (m/s) */
        float climb_front_retract_vy;       /* 前杆回收阶段附加横移速度vy (m/s) */
        uint32_t climb_front_retract_timeout_ms; /* 前杆回收阶段最长等待时间 (ms) */
        uint32_t front_retract_settle_ms;   /* 前杆回收后的稳定等待时间 (ms) */

        /* 200台阶/跨越中后段参数：中段纯前进、后杆回收与尾段脱离 */
        float climb_mid_forward_speed;      /* 前杆收回后中段继续前进速度 (m/s) */
        uint32_t climb_mid_forward_ms;      /* 前杆收回后，中段纯前进保持时间 (ms) */
        float climb_rear_retract_speed;     /* 后杆回收阶段的基础前进速度 (m/s) */
        float climb_rear_retract_vy;        /* 后杆回收阶段附加横移速度vy (m/s) */
        uint32_t head_rear_photo_timeout_ms;     /* 头向后光电等待超时 (ms) */
        uint32_t tail_rear_photo_timeout_ms;     /* 尾向后光电等待超时 (ms) */
        uint32_t climb_rear_retract_timeout_ms; /* 后杆回收阶段最长等待时间 (ms) */
        uint32_t rear_retract_move_ms;      /* 后杆回收完成后继续保持运动的时间 (ms) */

        /* 200下台阶逆流程参数：对应ASCEND_200的逆动作 */
        float descend_200_align_speed;             /* 下台阶对正阶段速度 (m/s) */
        float descend_200_climb_forward_speed;     /* 下台阶跨越时前进速度 (m/s) */
        float descend_200_pole_extend_settle_ms;   /* 下台阶撑杆伸出稳定时间 (ms) */
        uint32_t descend_200_head_rear_photo_timeout_ms; /* 下台阶头向后光电超时 (ms) */
        float descend_200_climb_rear_retract_speed;      /* 下台阶后杆回收速度 (m/s) */
        uint32_t descend_200_climb_rear_retract_timeout_ms; /* 下台阶后杆回收超时 (ms) */
        uint32_t descend_200_rear_retract_move_ms;       /* 下台阶后杆回收后移动时长 (ms) */
        float descend_200_climb_front_retract_speed;      /* 下台阶前杆回收速度 (m/s) */
        uint32_t descend_200_climb_front_retract_timeout_ms; /* 下台阶前杆回收超时 (ms) */
        uint32_t descend_200_front_retract_settle_ms;     /* 下台阶前杆回收后稳定时长 (ms) */
        uint32_t descend_200_flat_move_ms;                /* 下台阶脱离台阶平移时长 (ms) */
        float descend_200_flat_move_speed;                /* 下台阶脱离台阶平移速度 (m/s) */
        float descend_200_pole_all_extend_lift_speed;     /* 下台阶四杆全伸阶段速度 (rad/s) */
        float descend_200_pole_rear_retract_lift_speed;    /* 下台阶后杆回收阶段速度 (rad/s) */
        float descend_200_pole_front_retract_lift_speed;   /* 下台阶前杆回收阶段速度 (rad/s) */

        /* 400上台阶逆流程参数：对应ASCEND_400_STD的逆动作 */
        float ascend_400_align_forward_speed;    /* 上台阶对正阶段同步前进速度 (m/s) */
        float ascend_400_pole_extend_forward_speed; /* 上台阶四杆伸起稳定阶段的前进速度 (m/s) */
        float ascend_400_forward_speed;          /* 上台阶前杆回收阶段基础前进速度 (m/s) */
        uint32_t ascend_400_pole_extend_settle_ms; /* 上台阶四杆伸出后稳定时间 (ms) */
        float ascend_400_pole_all_extend_lift_speed; /* 上台阶四杆全伸阶段速度 (rad/s) */
        uint32_t ascend_400_head_front_photo_timeout_ms; /* 上台阶头向前光电超时 (ms) */
        float ascend_400_front_retract_speed;    /* 上台阶前杆回收速度 (m/s) */
        float ascend_400_front_retract_vy;       /* 上台阶前杆回收附加横移速度vy (m/s) */
        uint32_t ascend_400_front_retract_timeout_ms; /* 上台阶前杆回收超时 (ms) */
        uint32_t ascend_400_front_retract_settle_ms; /* 上台阶前杆回收后稳定时间 (ms) */
        float ascend_400_mid_forward_speed;      /* 上台阶前杆收回后中段前进速度 (m/s) */
        uint32_t ascend_400_mid_forward_ms;      /* 上台阶中段前进保持时间 (ms) */
        float ascend_400_rear_retract_speed;     /* 上台阶后杆回收速度 (m/s) */
        float ascend_400_rear_retract_vy;        /* 上台阶后杆回收附加横移速度vy (m/s) */
        uint32_t ascend_400_head_rear_photo_timeout_ms; /* 上台阶头向后光电超时 (ms) */
        uint32_t ascend_400_rear_retract_timeout_ms; /* 上台阶后杆回收超时 (ms) */
        uint32_t ascend_400_rear_retract_move_ms; /* 上台阶后杆回收后移动时长 (ms) */
        float ascend_400_pole_rear_retract_lift_speed; /* 上台阶后杆回收阶段速度 (rad/s) */
        float ascend_400_pole_front_retract_lift_speed; /* 上台阶前杆回收阶段速度 (rad/s) */

        /* 400下台阶逆流程参数：对应ASCEND_400_STD的逆动作 */
        float descend_400_align_speed;             /* 下台阶对正阶段速度 (m/s) */
        float descend_400_climb_forward_speed;     /* 下台阶跨越时前进速度 (m/s) */
        float descend_400_pole_extend_settle_ms;  /* 下台阶撑杆伸出稳定时间 (ms) */
        uint32_t descend_400_head_rear_photo_timeout_ms; /* 下台阶头向后光电超时 (ms) */
        float descend_400_climb_rear_retract_speed;      /* 下台阶后杆回收速度 (m/s) */
        uint32_t descend_400_climb_rear_retract_timeout_ms; /* 下台阶后杆回收超时 (ms) */
        uint32_t descend_400_rear_retract_move_ms;       /* 下台阶后杆回收后移动时长 (ms) */
        float descend_400_climb_front_retract_speed;      /* 下台阶前杆回收速度 (m/s) */
        uint32_t descend_400_climb_front_retract_timeout_ms; /* 下台阶前杆回收超时 (ms) */
        uint32_t descend_400_front_retract_settle_ms;     /* 下台阶前杆回收后稳定时长 (ms) */
        uint32_t descend_400_flat_move_ms;                /* 下台阶脱离台阶平移时长 (ms) */
        float descend_400_pole_all_extend_lift_speed;     /* 下台阶四杆全伸阶段速度 (rad/s) */
        float descend_400_pole_rear_retract_lift_speed;    /* 下台阶后杆回收阶段速度 (rad/s) */
        float descend_400_pole_front_retract_lift_speed;  /* 下台阶前杆回收阶段速度 (rad/s) */

        /* 尾部方向上200台阶参数（尾向前）：对称于头部方向上200台阶 */
        float ascend_200_tail_align_forward_speed;   /* 尾部上台阶对正阶段同步前进速度 (m/s) */
        float ascend_200_tail_pole_extend_forward_speed; /* 尾部上台阶四杆伸起稳定阶段前进速度 (m/s) */
        float ascend_200_tail_forward_speed;         /* 尾部上台阶前杆回收阶段基础前进速度 (m/s) */
        uint32_t ascend_200_tail_pole_extend_settle_ms; /* 尾部上台阶四杆伸出后稳定时间 (ms) */
        float ascend_200_tail_pole_all_extend_lift_speed; /* 尾部上台阶四杆全伸阶段速度 (rad/s) */
        uint32_t ascend_200_tail_tail_front_photo_timeout_ms; /* 尾部上台阶尾向前光电超时 (ms) */
        float ascend_200_tail_front_retract_speed;   /* 尾部上台阶前杆回收速度 (m/s) */
        float ascend_200_tail_front_retract_vy;      /* 尾部上台阶前杆回收附加横移速度vy (m/s) */
        uint32_t ascend_200_tail_front_retract_timeout_ms; /* 尾部上台阶前杆回收超时 (ms) */
        uint32_t ascend_200_tail_front_retract_settle_ms; /* 尾部上台阶前杆回收后稳定时间 (ms) */
        float ascend_200_tail_mid_forward_speed;     /* 尾部上台阶前杆收回后中段前进速度 (m/s) */
        uint32_t ascend_200_tail_mid_forward_ms;    /* 尾部上台阶中段前进保持时间 (ms) */
        float ascend_200_tail_rear_retract_speed;   /* 尾部上台阶后杆回收速度 (m/s) */
        float ascend_200_tail_rear_retract_vy;      /* 尾部上台阶后杆回收附加横移速度vy (m/s) */
        uint32_t ascend_200_tail_tail_rear_photo_timeout_ms; /* 尾部上台阶尾向后光电超时 (ms) */
        uint32_t ascend_200_tail_rear_retract_timeout_ms; /* 尾部上台阶后杆回收超时 (ms) */
        uint32_t ascend_200_tail_rear_retract_move_ms; /* 尾部上台阶后杆回收后移动时长 (ms) */
        float ascend_200_tail_pole_front_retract_lift_speed; /* 尾部上台阶前杆回收阶段速度 (rad/s) */
        float ascend_200_tail_pole_rear_retract_lift_speed; /* 尾部上台阶后杆回收阶段速度 (rad/s) */

        /* 尾部方向下200台阶参数（尾向前）：对称于头部方向下200台阶 */
        float descend_200_tail_align_speed;             /* 尾部下台阶对正阶段速度 (m/s) */
        float descend_200_tail_climb_forward_speed;     /* 尾部下台阶跨越时前进速度 (m/s) */
        uint32_t descend_200_tail_pole_extend_settle_ms; /* 尾部下台阶撑杆伸出稳定时间 (ms) */
        uint32_t descend_200_tail_tail_rear_photo_timeout_ms; /* 尾部下台阶尾向后光电超时 (ms) */
        float descend_200_tail_climb_rear_retract_speed;     /* 尾部下台阶后杆回收速度 (m/s) */
        uint32_t descend_200_tail_climb_rear_retract_timeout_ms; /* 尾部下台阶后杆回收超时 (ms) */
        uint32_t descend_200_tail_rear_retract_move_ms;       /* 尾部下台阶后杆回收后移动时长 (ms) */
        float descend_200_tail_climb_front_retract_speed;     /* 尾部下台阶前杆回收速度 (m/s) */
        uint32_t descend_200_tail_climb_front_retract_timeout_ms; /* 尾部下台阶前杆回收超时 (ms) */
        uint32_t descend_200_tail_front_retract_settle_ms;     /* 尾部下台阶前杆回收后稳定时长 (ms) */
        uint32_t descend_200_tail_flat_move_ms;                /* 尾部下台阶脱离台阶平移时长 (ms) */
        float descend_200_tail_pole_all_extend_lift_speed;     /* 尾部下台阶四杆全伸阶段速度 (rad/s) */
        float descend_200_tail_pole_rear_retract_lift_speed;    /* 尾部下台阶后杆回收阶段速度 (rad/s) */
        float descend_200_tail_pole_front_retract_lift_speed;   /* 尾部下台阶前杆回收阶段速度 (rad/s) */

        /* SICK辅助姿态修正参数 */
        float sick_valid_min_cm;
        float sick_valid_max_cm;
        float sick_norm_err_deadband;
        float sick_norm_err_to_rad;
        float sick_assist_gain;
        float sick_assist_max_rad;
    } auto_ctrl_param;
    Rod_Params_t rod_param;
} Config_RobotParam_t;

/* Exported functions prototypes -------------------------------------------- */

/**
 * @brief 获取机器人配置参数
 * @return 机器人配置参数指针
 */
Config_RobotParam_t* Config_GetRobotParam(void);
#ifdef __cplusplus
}
#endif
