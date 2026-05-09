#pragma once

/**
 * @file auto_ctrl_def.h
 * @brief AutoCtrl 模块的核心类型定义。
 *
 * 该文件只放“跨子模块共享”的基础定义：
 * 1) 模板枚举；
 * 2) 运行状态/结果/故障码。
 *
 * 设计约束：
 * - 业务状态机与模板执行器均依赖这些定义；
 * - 拨杆直接调用模板函数，无需区块表。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	AUTO_CTRL_TEMPLATE_NONE = 0,              /* 未分配模板。 */
	AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD,       /* 头向上 200mm 台阶。 */
	AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD,       /* 头向上 400mm 台阶。 */
	AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD,      /* 头向下 200mm 台阶。 */
	AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD,      /* 头向下 400mm 台阶。 */
	AUTO_CTRL_TEMPLATE_ASCEND_200_TAIL,        /* 尾向上 200mm 台阶。 */
	AUTO_CTRL_TEMPLATE_ASCEND_400_TAIL,        /* 尾向上 400mm 台阶。 */
	AUTO_CTRL_TEMPLATE_DESCEND_200_TAIL,       /* 尾向下 200mm 台阶。 */
	AUTO_CTRL_TEMPLATE_DESCEND_400_TAIL,       /* 尾向下 400mm 台阶。 */
} auto_ctrl_template_e;

typedef enum {
	AUTO_CTRL_TRAVEL_DIR_HEAD_FORWARD = 0, /* 当前任务按头向作为前进方向。 */
	AUTO_CTRL_TRAVEL_DIR_TAIL_FORWARD,     /* 当前任务按尾向作为前进方向。 */
} auto_ctrl_travel_dir_e;

typedef enum {
	AUTO_CTRL_SENSOR_MODE_NONE = 0, /* 不使用传感器约束。 */
	AUTO_CTRL_SENSOR_MODE_YAW_ONLY, /* 仅使用 IMU yaw 对齐。 */
	AUTO_CTRL_SENSOR_MODE_SICK_FRONT_AND_BOTTOM, /* SICK 前向/侧向 + 底部光电。 */
	AUTO_CTRL_SENSOR_MODE_BOTTOM_ONLY, /* 仅使用底部光电触发。 */
} auto_ctrl_sensor_mode_e;

typedef enum {
	AUTO_CTRL_YAW_SOURCE_STM32 = 0, /* 使用 STM32 本地目标 yaw / 陀螺仪 yaw。 */
	AUTO_CTRL_YAW_SOURCE_PC,        /* 使用上位机目标 yaw / 上位机欧拉角 yaw。 */
} auto_ctrl_yaw_source_e;

typedef enum {
	AUTO_CTRL_STATE_IDLE = 0,     /* 空闲态，等待任务启动。 */
	AUTO_CTRL_STATE_PREALIGN,     /* 姿态预对齐阶段。 */
	AUTO_CTRL_STATE_RUN_TEMPLATE, /* 模板执行阶段。 */
	AUTO_CTRL_STATE_SUCCESS,      /* 任务成功结束态。 */
	AUTO_CTRL_STATE_FAIL,         /* 任务失败结束态。 */
	AUTO_CTRL_STATE_ABORT,        /* 外部中止结束态。 */
} auto_ctrl_run_state_e;

typedef enum {
	AUTO_CTRL_RESULT_NONE = 0, /* 尚未开始或结果未定义。 */
	AUTO_CTRL_RESULT_RUNNING,  /* 正在执行中。 */
	AUTO_CTRL_RESULT_SUCCESS,  /* 执行成功。 */
	AUTO_CTRL_RESULT_FAIL,     /* 执行失败。 */
	AUTO_CTRL_RESULT_ABORTED,  /* 被外部中止。 */
} auto_ctrl_result_e;

typedef enum {
	AUTO_CTRL_FAULT_NONE = 0,             /* 无故障。 */
	AUTO_CTRL_FAULT_INVALID_TEMPLATE,     /* 模板 ID 非法或未配置。 */
	AUTO_CTRL_FAULT_PREALIGN_TIMEOUT,     /* 姿态预对齐阶段超时。 */
	AUTO_CTRL_FAULT_TEMPLATE_TIMEOUT,     /* 模板执行总超时。 */
	AUTO_CTRL_FAULT_SENSOR_INVALID,       /* 传感器触发条件异常/超时。 */
	AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED, /* 运行到未支持模板/step。 */
	AUTO_CTRL_FAULT_ABORTED,              /* 任务被上层中止。 */
} auto_ctrl_fault_e;

#ifdef __cplusplus
}
#endif
