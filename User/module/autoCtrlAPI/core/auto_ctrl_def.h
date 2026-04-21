#pragma once

/**
 * @file auto_ctrl_def.h
 * @brief AutoCtrl 模块的核心类型定义。
 *
 * 该文件只放“跨子模块共享”的基础定义：
 * 1) 区块枚举与模板枚举；
 * 2) 运行状态/结果/故障码；
 * 3) 区块属性表与转移表项结构体。
 *
 * 设计约束：
 * - zone 枚举值与 zone 表索引强绑定；
 * - transition 表通过 from/to + template + yaw 组成行为描述；
 * - 业务状态机与模板执行器均依赖这些定义。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  AUTO_CTRL_ZONE_INVALID = 0, /* 非法区块/兜底区块。 */
  AUTO_CTRL_ZONE_R2_ENTRY1,   /* R2 入口 1（起始地面区）。 */
  AUTO_CTRL_ZONE_R2_ENTRY2,   /* R2 入口 2（起始地面区）。 */
  AUTO_CTRL_ZONE_R2_ENTRY3,   /* R2 入口 3（起始地面区）。 */
  AUTO_CTRL_ZONE_PLATFORM_1,  /* 平台区块 1。 */
  AUTO_CTRL_ZONE_PLATFORM_2,  /* 平台区块 2。 */
  AUTO_CTRL_ZONE_PLATFORM_3,  /* 平台区块 3。 */
  AUTO_CTRL_ZONE_PLATFORM_4,  /* 平台区块 4。 */
  AUTO_CTRL_ZONE_PLATFORM_5,  /* 平台区块 5。 */
  AUTO_CTRL_ZONE_PLATFORM_6,  /* 平台区块 6。 */
  AUTO_CTRL_ZONE_PLATFORM_7,  /* 平台区块 7。 */
  AUTO_CTRL_ZONE_PLATFORM_8,  /* 平台区块 8。 */
  AUTO_CTRL_ZONE_PLATFORM_9,  /* 平台区块 9。 */
  AUTO_CTRL_ZONE_PLATFORM_10, /* 平台区块 10。 */
  AUTO_CTRL_ZONE_PLATFORM_11, /* 平台区块 11。 */
  AUTO_CTRL_ZONE_PLATFORM_12, /* 平台区块 12。 */
  AUTO_CTRL_ZONE_R2_EXIT1,    /* R2 出口 1（终止地面区）。 */
  AUTO_CTRL_ZONE_R2_EXIT2,    /* R2 出口 2（终止地面区）。 */
  AUTO_CTRL_ZONE_R2_EXIT3,    /* R2 出口 3（终止地面区）。 */
  AUTO_CTRL_ZONE_COUNT,       /* 区块枚举总数（非有效区块）。 */
} auto_ctrl_zone_e;

typedef enum {
  AUTO_CTRL_TEMPLATE_NONE = 0,     /* 未分配模板。 */
  AUTO_CTRL_TEMPLATE_FLAT_MOVE,    /* 平地匀速直行模板。 */
  AUTO_CTRL_TEMPLATE_ASCEND_200,   /* 上 200mm 台阶模板。 */
  AUTO_CTRL_TEMPLATE_DESCEND_200,  /* 下 200mm 台阶模板。 */
  AUTO_CTRL_TEMPLATE_ASCEND_400_STD, /* 标准上 400mm 台阶模板。 */
  AUTO_CTRL_TEMPLATE_DESCEND_400_STD, /* 标准下 400mm 台阶模板。 */
} auto_ctrl_template_e;

typedef enum {
  AUTO_CTRL_SENSOR_MODE_NONE = 0, /* 不使用传感器约束。 */
  AUTO_CTRL_SENSOR_MODE_YAW_ONLY, /* 仅使用 IMU yaw 对齐。 */
  AUTO_CTRL_SENSOR_MODE_SICK_FRONT_AND_BOTTOM, /* SICK 前向/侧向 + 底部光电。 */
  AUTO_CTRL_SENSOR_MODE_BOTTOM_ONLY, /* 仅使用底部光电触发。 */
} auto_ctrl_sensor_mode_e;

typedef enum {
  AUTO_CTRL_STATE_IDLE = 0,   /* 空闲态，等待任务启动。 */
  AUTO_CTRL_STATE_PREALIGN,   /* 姿态预对齐阶段。 */
  AUTO_CTRL_STATE_RUN_TEMPLATE, /* 模板执行阶段。 */
  AUTO_CTRL_STATE_SUCCESS,    /* 任务成功结束态。 */
  AUTO_CTRL_STATE_FAIL,       /* 任务失败结束态。 */
  AUTO_CTRL_STATE_ABORT,      /* 外部中止结束态。 */
} auto_ctrl_run_state_e;

typedef enum {
  AUTO_CTRL_RESULT_NONE = 0, /* 尚未开始或结果未定义。 */
  AUTO_CTRL_RESULT_RUNNING,  /* 正在执行中。 */
  AUTO_CTRL_RESULT_SUCCESS,  /* 执行成功。 */
  AUTO_CTRL_RESULT_FAIL,     /* 执行失败。 */
  AUTO_CTRL_RESULT_ABORTED,  /* 被外部中止。 */
} auto_ctrl_result_e;

typedef enum {
  AUTO_CTRL_FAULT_NONE = 0,            /* 无故障。 */
  AUTO_CTRL_FAULT_INVALID_ZONE,        /* 输入区块非法。 */
  AUTO_CTRL_FAULT_INVALID_TRANSITION,  /* 转移表无对应 from->to。 */
  AUTO_CTRL_FAULT_INVALID_TEMPLATE,    /* 模板 ID 非法或未配置。 */
  AUTO_CTRL_FAULT_PREALIGN_TIMEOUT,    /* 姿态预对齐阶段超时。 */
  AUTO_CTRL_FAULT_TEMPLATE_TIMEOUT,    /* 模板执行总超时。 */
  AUTO_CTRL_FAULT_SENSOR_INVALID,      /* 传感器触发条件异常/超时。 */
  AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED, /* 运行到未支持模板/step。 */
  AUTO_CTRL_FAULT_ABORTED,             /* 任务被上层中止。 */
} auto_ctrl_fault_e;

/* 区块静态属性：供 zone 子模块返回只读信息。 */
typedef struct {
  auto_ctrl_zone_e zone;
  const char *name;
  int16_t height_mm;
  bool is_platform;
} auto_ctrl_zone_info_t;

/* 区块转移定义：描述 from->to 对应的模板和姿态/传感器约束。 */
typedef struct {
  auto_ctrl_zone_e from_zone;
  auto_ctrl_zone_e to_zone;
  auto_ctrl_template_e template_id;
  float required_yaw_rad;
  float yaw_tolerance_rad;
  auto_ctrl_sensor_mode_e sensor_mode;
  int16_t height_delta_mm;
} auto_ctrl_transition_t;

#ifdef __cplusplus
}
#endif