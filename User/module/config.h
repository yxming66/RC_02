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
#include "module/arm.h"
#include "module/rod.h"
typedef struct {
    Chassis_Params_t chassis_param;
    Pole_Params_t pole_param;
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
