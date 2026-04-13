#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "device/device.h"

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */

/* USER DEFINE BEGIN */

/* USER DEFINE END */

/* Exported constants ------------------------------------------------------- */
/* Exported macro ----------------------------------------------------------- */
/* Exported types ----------------------------------------------------------- */
typedef struct {
    float rotor_abs_angle; /* 转子绝对角度 */
    float rotor_speed; /* 实际转子转速 */
    float torque_current; /* 转矩电流 */
    float temp; /* 温度 */
} MOTOR_Feedback_t;

typedef struct {
    uint16_t raw_angle;      /* 原始编码器角度 */
    int16_t raw_speed;       /* 原始转速反馈 */
    int16_t raw_current;     /* 原始电流反馈 */
    uint8_t raw_temp;        /* 原始温度反馈 */
} MOTOR_RawFeedback_t;

/**
 * @brief mit电机输出参数结构体
 */
typedef struct {
    float torque;      /* 目标力矩 */
    float velocity;    /* 目标速度 */
    float angle;       /* 目标位置 */
    float kp;         /* 位置环增益 */
    float kd;         /* 速度环增益 */
} MOTOR_MIT_Output_t;

/**
 * @brief 转矩电流控制模式参数结构体
 */
typedef struct {
    float current;    /* 目标电流 */
} MOTOR_Current_Output_t;

typedef struct {
    DEVICE_Header_t header;
    bool reverse; /* 是否反装 true表示反装 */
    MOTOR_RawFeedback_t raw_feedback;
    MOTOR_Feedback_t feedback;
} MOTOR_t;

/* USER STRUCT BEGIN */

/* USER STRUCT END */

/* Exported functions prototypes -------------------------------------------- */
float MOTOR_GetRotorAbsAngle(const MOTOR_t *motor);
float MOTOR_GetRotorSpeed(const MOTOR_t *motor);
float MOTOR_GetTorqueCurrent(const MOTOR_t *motor);
float MOTOR_GetTemp(const MOTOR_t *motor);
const MOTOR_RawFeedback_t* MOTOR_GetRawFeedback(const MOTOR_t *motor);

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */

#ifdef __cplusplus
}
#endif
