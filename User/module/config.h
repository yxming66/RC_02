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
        float climb_forward_speed;
        float climb_forward_kick_speed;
        float climb_rear_retract_speed;
        uint32_t pole_extend_settle_ms;
        uint32_t front_photo_timeout_ms;
        uint32_t front_retract_settle_ms;
        uint32_t rear_photo_timeout_ms;
        uint32_t rear_retract_move_ms;
        float sick_valid_min_cm;
        float sick_valid_max_cm;
        float sick_norm_err_deadband;
        float sick_norm_err_to_deg;
        float sick_assist_gain;
        float sick_assist_max_deg;
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
