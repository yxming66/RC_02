/*
 * Arm Simple: DM4310关节1 + JS6660舵机关节2 + 电磁阀吸盘
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "device/motor_dm.h"
#include "bsp/pwm.h"
#include "bsp/gpio.h"
#include "component/pid.h"

#define ARM_SIMPLE_OK (0)
#define ARM_SIMPLE_ERR (-1)

/* 舵机角度范围 */
#define SERVO_JS6660_MIN_ANGLE_RAD (-4.712389f)   /* -270° */
#define SERVO_JS6660_MAX_ANGLE_RAD (4.712389f)    /* +270° */
#define SERVO_JS6660_DEFAULT_FREQ_HZ (50.0f)       /* 50Hz PWM */
#define SERVO_JS6660_PULSE_MIN_US (500)            /* 0.5ms */
#define SERVO_JS6660_PULSE_MAX_US (2500)           /* 2.5ms */
#define SERVO_JS6660_PULSE_NEUTRAL_US (1500)       /* 1.5ms */

/* 吸盘状态 */
typedef enum {
    SUCTION_OFF = 0,
    SUCTION_ON,
} Suction_State_t;

/* 控制模式 */
typedef enum {
    ARM_SIMPLE_MODE_RELAX = 0,    /* 松弛，所有输出归零 */
    ARM_SIMPLE_MODE_JOINT,        /* 关节角度控制 */
    ARM_SIMPLE_MODE_POS_VEL,      /* 位置+速度控制 */
} ArmSimple_Mode_t;

/* 关节角度（弧度） */
typedef struct {
    float joint1;  /* DM4310关节1角度（绝对值） */
    float joint2;  /* 舵机关节2角度（-270° ~ +270°） */
} ArmSimple_JointAngle_t;

/* 预设姿态 */
typedef enum {
    ARM_SIMPLE_POINT_SLEEP = 0,   /* 待机姿态 */
    ARM_SIMPLE_POINT_GRAB,         /* 抓取姿态 */
    ARM_SIMPLE_POINT_LIFT,         /* 抬起姿态 */
    ARM_SIMPLE_POINT_RELEASE,     /* 释放姿态 */
    ARM_SIMPLE_POINT_NONE,
} ArmSimple_PointMode_t;

/* 控制命令 */
typedef struct {
    ArmSimple_Mode_t mode;
    ArmSimple_PointMode_t point_mode;
    Suction_State_t suction;
    ArmSimple_JointAngle_t target_joint;  /* 目标关节角度 */
    float joint1_vel;                     /* 关节1速度 */
} ArmSimple_CMD_t;

/* 预设姿态点 */
typedef struct {
    float joint1_pos;
    float joint2_pos;
} ArmSimple_Point2Point_t;

/* 初始化参数 */
typedef struct {
    /* DM4310参数 */
    struct {
        BSP_CAN_t can;
        uint16_t master_id;
        uint16_t can_id;
        bool reverse;
    } dm4310_param;

    /* 舵机PWM参数 */
    struct {
        BSP_PWM_Channel_t pwm_channel;
        float freq_hz;
    } servo_param;

    /* 吸盘GPIO参数 */
    struct {
        BSP_GPIO_t gpio;
    } suction_param;

    /* PID参数 */
    struct {
        KPID_Params_t joint1_pos;
        KPID_Params_t joint1_vel;
    } pid;

    /* 软限位 */
    struct {
        float joint1_min;
        float joint1_max;
        float joint2_min;
        float joint2_max;
    } soft_limit;

    /* 速度限制 */
    struct {
        float joint1_max_vel;
        float joint2_max_vel;
    } vel_limit;
} ArmSimple_Params_t;

/* 臂实例 */
typedef struct {
    ArmSimple_Params_t *param;
    ArmSimple_Mode_t mode;
    Suction_State_t suction;

    struct {
        float now;
        uint64_t last_wakeup_us;
        float dt;
    } timer;

    /* 反馈 */
    struct {
        float joint1_angle;   /* DM4310当前角度 */
        float joint1_vel;     /* DM4310当前速度 */
        float joint2_angle;   /* 舵机当前角度 */
    } feedback;

    /* 目标 */
    struct {
        float joint1_target;
        float joint2_target;
        float joint1_vel_target;
    } target;

    /* PID */
    struct {
        KPID_t joint1_pos;
        KPID_t joint1_vel;
    } pid;

    /* 预设姿态 */
    ArmSimple_Point2Point_t point2point[ARM_SIMPLE_POINT_NONE];

    ArmSimple_CMD_t cmd;
} ArmSimple_t;

/* 函数声明 */
int8_t ArmSimple_Init(ArmSimple_t *a, ArmSimple_Params_t *param, float target_freq);
int8_t ArmSimple_UpdateFeedback(ArmSimple_t *a);
int8_t ArmSimple_Control(ArmSimple_t *a, const ArmSimple_CMD_t *cmd);
int8_t ArmSimple_Output(ArmSimple_t *a);
void ArmSimple_Relax(ArmSimple_t *a);
void ArmSimple_SetSuction(ArmSimple_t *a, Suction_State_t state);
bool ArmSimple_Joint1AtTarget(ArmSimple_t *a, float threshold_rad);
bool ArmSimple_Joint2AtTarget(ArmSimple_t *a, float threshold_rad);

#ifdef __cplusplus
}
#endif
