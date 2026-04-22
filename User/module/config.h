/*
 * 配置相关
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "component/pid.h"
#include "device/motor.h"
#include "device/motor_rm.h"
#include "module/chassis.h"
#include "module/pole.h"
#include "module/cmd/cmd.h"
#include "module/rod.h"
#ifdef __cplusplus
}
#include "module/arm.h"
extern "C" {
#else
#include "module/arm.h"
#endif
typedef struct {
    Chassis_Params_t chassis_param;
    Pole_Params_t pole_param;
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
        uint32_t front_photo_timeout_ms;    /* 前段等待底部光电/前段反馈超时 (ms) */
        float climb_front_retract_speed;    /* 前杆回收阶段基础前进速度 (m/s) */
        float climb_front_retract_vy;       /* 前杆回收阶段附加横移速度vy (m/s) */
        uint32_t climb_front_retract_timeout_ms; /* 前杆回收阶段最长等待时间 (ms) */
        uint32_t front_retract_settle_ms;   /* 前杆回收后的稳定等待时间 (ms) */

        /* 200台阶/跨越中后段参数：中段纯前进、后杆回收与尾段脱离 */
        float climb_mid_forward_speed;      /* 前杆收回后中段继续前进速度 (m/s) */
        uint32_t climb_mid_forward_ms;      /* 前杆收回后，中段纯前进保持时间 (ms) */
        float climb_rear_retract_speed;     /* 后杆回收阶段的基础前进速度 (m/s) */
        float climb_rear_retract_vy;        /* 后杆回收阶段附加横移速度vy (m/s) */
        uint32_t rear_photo_timeout_ms;     /* 后段等待后杆/后段反馈超时 (ms) */
        uint32_t climb_rear_retract_timeout_ms; /* 后杆回收阶段最长等待时间 (ms) */
        uint32_t rear_retract_move_ms;      /* 后杆回收完成后继续保持运动的时间 (ms) */

        /* SICK辅助姿态修正参数 */
        float sick_valid_min_cm;
        float sick_valid_max_cm;
        float sick_norm_err_deadband;
        float sick_norm_err_to_rad;
        float sick_assist_gain;
        float sick_assist_max_rad;
    } auto_ctrl_param;
    CMD_Config_t cmd_param;
    Arm_Params_t arm_param;
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
