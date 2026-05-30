/*
 * Arm Simple: DM4340关节1 + JS6660舵机关节2 + 电磁阀吸盘
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

#define ARM_SIMPLE_OK (0)
#define ARM_SIMPLE_ERR (-1)

/* 舵机角度范围 */
#define SERVO_JS6660_MIN_ANGLE_RAD (-2.356194f)   /* -135° from center */
#define SERVO_JS6660_MAX_ANGLE_RAD (2.356194f)    /* +135° from center */
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
    ARM_SIMPLE_MODE_RELAX = 0,    /* 松弛，关节1失能 */
    ARM_SIMPLE_MODE_JOINT,        /* 关节角度控制 */
    ARM_SIMPLE_MODE_POS_VEL,      /* 兼容旧命令：按角度控制处理 */
} ArmSimple_Mode_t;

/* 关节角度（弧度） */
typedef struct {
    float joint1;  /* DM4340关节1角度（绝对值） */
    float joint2;  /* 舵机关节2角度，默认相对中心0点-135° ~ +135° */
} ArmSimple_JointAngle_t;

/* 预设姿态 */
typedef enum {
    ARM_SIMPLE_POINT_SLEEP = 0,   /* 待机姿态 */
    ARM_SIMPLE_POINT_GRAB,         /* 抓取姿态 */
    ARM_SIMPLE_POINT_LIFT,         /* 抬起姿态 */
    ARM_SIMPLE_POINT_RELEASE,     /* 释放姿态 */
    ARM_SIMPLE_POINT_NONE,
} ArmSimple_PointMode_t;

typedef enum {
    ARM_SIMPLE_BEHAVIOR_STANDBY = 0,  /* 一键存取矿：待机位置 */
    ARM_SIMPLE_BEHAVIOR_STORE_ORE,        /* 一键存取矿：存矿位置 */
    ARM_SIMPLE_BEHAVIOR_WAIT_STORE_ORE,   /* 一键存取矿：待存矿位置 */
    ARM_SIMPLE_BEHAVIOR_WAIT_RELEASE_ORE, /* 一键存取矿：待放矿位置 */
    ARM_SIMPLE_BEHAVIOR_RELEASE_ORE,      /* 一键存取矿：放矿位置 */
    ARM_SIMPLE_BEHAVIOR_NUM,
} ArmSimple_BehaviorPoint_t;

/* 预设姿态点 */
typedef struct {
    float joint1_pos;
    float joint2_pos;
} ArmSimple_Point2Point_t;

/* 控制命令 */
typedef struct {
    ArmSimple_Mode_t mode;
    ArmSimple_PointMode_t point_mode;   /* ARM_SIMPLE_POINT_NONE表示使用target_joint */
    Suction_State_t suction;
    ArmSimple_JointAngle_t target_joint;  /* 目标关节角度 */
    float joint1_vel;                     /* 兼容旧字段，角度控制链路不使用 */
} ArmSimple_CMD_t;

typedef struct {
    bool enable;
    ArmSimple_Mode_t mode;
    float target_joint1_rad;
    float target_joint2_rad;
    Suction_State_t suction;
} ArmSimple_DebugControl_t;

typedef struct {
    ArmSimple_Mode_t mode;
    ArmSimple_PointMode_t point_mode;
    Suction_State_t suction;
    bool joint1_temperature_warning;
    bool joint1_temperature_over_limit;
    bool joint1_temperature_limit_latched;
    float joint1_angle_rad;
    float joint1_velocity_rad_s;
    float joint1_temperature_c;
    float joint2_angle_rad;
    float target_joint1_rad;
    float target_joint2_rad;
} ArmSimple_Feedback_t;

/* 初始化参数 */
typedef struct {
    /* DM4340参数 */
    MOTOR_DM_Param_t dm4340_param;

    /* 舵机PWM参数 */
    struct {
        BSP_PWM_Channel_t pwm_channel;
        float freq_hz;
        bool reverse;
    } servo_param;

    /* 吸盘GPIO参数 */
    struct {
        BSP_GPIO_t gpio;
    } suction_param;

    /* DM MIT 原生位置控制参数 */
    struct {
        float joint1_kp;
        float joint1_kd;
        float joint1_torque_ff;
        float joint1_gravity_mass_kg;
        float joint1_gravity_com_m;
        float joint1_gravity_zero_rad;
        float joint1_gravity_ff_limit_nm;
    } mit;

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

    struct {
        ArmSimple_Point2Point_t behavior_point[ARM_SIMPLE_BEHAVIOR_NUM];
        float arrive_threshold_rad;
    } preset;

    MOTOR_TemperatureProtectionConfig_t joint1_temperature_protection;
} ArmSimple_Params_t;

/* 臂实例 */
typedef struct {
    ArmSimple_Params_t *param;
    void *dm_motor;
    ArmSimple_Mode_t mode;
    ArmSimple_Mode_t last_output_mode;
    uint64_t last_joint1_disable_us;
    bool dm_enabled;
    Suction_State_t suction;

    struct {
        float now;
        uint64_t last_wakeup_us;
        float dt;
    } timer;

    /* 反馈 */
    struct {
        float joint1_angle;   /* DM4340当前角度 */
        float joint1_vel;     /* DM4340当前速度 */
        float joint1_temp;    /* DM4340温度，°C */
        bool joint1_temperature_warning;
        bool joint1_temperature_over_limit;
        bool joint1_temperature_limit_latched;
        float joint2_angle;   /* 舵机当前角度 */
    } feedback;

    /* 目标 */
    struct {
        float joint1_target;
        float joint2_target;
        float joint1_vel_target;
    } target;

    /* 预设姿态 */
    ArmSimple_Point2Point_t point2point[ARM_SIMPLE_POINT_NONE];

    ArmSimple_CMD_t cmd;
} ArmSimple_t;

/* 函数声明 */
int8_t ArmSimple_Init(ArmSimple_t *a, ArmSimple_Params_t *param, float target_freq);
int8_t ArmSimple_UpdateFeedback(ArmSimple_t *a);
int8_t ArmSimple_Control(ArmSimple_t *a, const ArmSimple_CMD_t *cmd);
int8_t ArmSimple_Output(ArmSimple_t *a);
int8_t ArmSimple_SetJoint1Zero(ArmSimple_t *a);
void ArmSimple_Relax(ArmSimple_t *a);
void ArmSimple_SetSuction(ArmSimple_t *a, Suction_State_t state);
bool ArmSimple_Joint1AtTarget(ArmSimple_t *a, float threshold_rad);
bool ArmSimple_Joint2AtTarget(ArmSimple_t *a, float threshold_rad);
uint32_t ArmSimple_AngleToPulseUs(float angle_rad, const ArmSimple_Params_t *param);
bool ArmSimple_MakeBehaviorCommand(const ArmSimple_Params_t *param,
                                   ArmSimple_BehaviorPoint_t point,
                                   Suction_State_t suction,
                                   ArmSimple_CMD_t *cmd);

extern volatile ArmSimple_DebugControl_t g_arm_simple_debug;

#ifdef __cplusplus
}
#endif
