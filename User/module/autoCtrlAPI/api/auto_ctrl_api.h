#pragma once

/**
 * @file auto_ctrl_api.h
 * @brief AutoCtrl 对外主接口。
 *
 * 该文件定义：
 * - 外部反馈输入格式；
 * - AutoCtrl 运行时上下文；
 * - 任务启动、周期更新、状态查询等 API。
 *
 * 上层模块通过本接口驱动自动控流程，不直接操作模板或原语层。
 */

#include "module/autoCtrlAPI/core/auto_ctrl_def.h"
#include "module/autoCtrlAPI/core/auto_ctrl_math.h"
#include "module/autoCtrlAPI/template/auto_ctrl_template.h"
#include "module/chassis.h"
#include "module/pole.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 外部传感器反馈快照。 */
typedef struct {
  float yaw_auto_rad;  /* 输入 yaw（单位: rad），SetFeedback 内会扣零点并归一化。 */
  float sick_front_left_cm;  /* 左前测距传感器读数，单位: cm。 */
  float sick_front_right_cm; /* 右前测距传感器读数，单位: cm。 */
  bool photo_transfer_valid; /* 光电转接板 CAN 快照当前是否有效。 */
  bool pe13_photo1_triggered; /* PE13, photo1. */
  bool pe9_photo2_triggered;  /* PE9, photo2. */
  bool pa2_photo3_triggered;  /* PA2, photo3. */
  bool pa0_photo4_triggered;  /* PA0, photo4. */
  float pole_front_lift_rad;  /* Front pole lift feedback, rad. */
  float pole_rear_lift_rad;   /* Rear pole lift feedback, rad. */
  bool pole_front_at_target;  /* Front pole group reached current target. */
  bool pole_rear_at_target;   /* Rear pole group reached current target. */
  bool pole_all_at_target;    /* All pole groups reached current target. */
  float wheel_position_rad[4]; /* 底盘四轮累计位置，单位 rad。 */
} auto_ctrl_feedback_t;

typedef enum {
  AUTO_CTRL_PREALIGN_BY_YAW = 0,
  AUTO_CTRL_PREALIGN_BY_SICK,
} auto_ctrl_prealign_mode_t;

typedef enum {
  AUTO_CTRL_SEARCH_DIR_CCW = 0,
  AUTO_CTRL_SEARCH_DIR_CW,
} auto_ctrl_search_dir_t;

/* AutoCtrl 全量运行时上下文。 */
typedef struct {
  auto_ctrl_run_state_e state; /* 当前运行状态机所处状态。 */
  auto_ctrl_result_e result;   /* 本次自动控制的高层结果。 */
  auto_ctrl_fault_e fault;     /* 失败/中止时记录的故障原因。 */

  auto_ctrl_template_e template_id; /* 当前执行的动作模板。 */
  auto_ctrl_travel_dir_e travel_dir; /* 当前任务采用的前进方向语义。 */
  auto_ctrl_sensor_mode_e sensor_mode; /* 当前任务使用的传感器约束模式。 */
  auto_ctrl_yaw_source_e yaw_source; /* 当前任务使用的 yaw 来源。 */

  float yaw_raw_rad;         /* 最近一次输入的原始 yaw，未扣零点。 */
  float yaw_zero_offset_rad; /* 将 raw yaw 转为 auto yaw 使用的零点偏移。 */
  float target_yaw_rad;      /* 本次模板要求的目标朝向，单位: rad，范围 [-pi, pi)。 */
  float yaw_tolerance_rad;   /* PREALIGN 通过阈值，单位: rad。 */
  float yaw_error_rad;       /* 目标减当前的有符号朝向误差，单位: rad。 */
  float lateral_velocity_cmd_mps; /* 外部横向速度命令，单位: m/s。 */
  float yaw_rate_cmd_rad_s;  /* 外部 yaw 修正角速度命令，单位: rad/s。 */

  auto_ctrl_prealign_mode_t prealign_mode; /* PREALIGN 当前使用的对正来源。 */
  float yaw_search_accept_center_rad;      /* 允许接受 SICK 的目标姿态中心。 */
  float sick_accept_max_yaw_diff_rad;      /* 允许接受 SICK 时的最大 yaw 偏差。 */
  bool sick_valid_now;                     /* 当前周期 SICK 是否有效。 */
  bool prealign_done_by_sick;              /* PREALIGN 是否由 SICK 完成。 */
  uint32_t sick_valid_stable_since_ms;     /* SICK 连续有效起始时刻。 */
  uint32_t sick_align_stable_since_ms;     /* SICK 对正连续满足阈值起始时刻。 */
  uint32_t yaw_align_stable_since_ms;      /* 纯 yaw 对正连续满足阈值起始时刻。 */

  uint32_t state_enter_time_ms; /* 进入当前 state 的时间戳，单位: ms。 */
  uint32_t prealign_timeout_ms; /* PREALIGN 阶段超时时间，单位: ms。 */
  uint32_t template_timeout_ms; /* RUN_TEMPLATE 阶段超时时间，单位: ms。 */

  auto_ctrl_template_ctx_t template_ctx; /* 模板内部 step 运行上下文。 */
  auto_ctrl_feedback_t feedback;         /* 外部最近一次写入的反馈快照。 */

  Chassis_CMD_t chassis_cmd; /* 当前 Update 周期生成的底盘指令。 */
  Pole_CMD_t pole_cmd;       /* 当前 Update 周期生成的撑杆指令。 */
} auto_ctrl_t;

/*
 * 上层接入伪流程（示例）：
 * 1) 上电初始化：AutoCtrl_Init(&ctrl);
 * 2) 周期循环中：
 *    a. 采集传感器并组织 auto_ctrl_feedback_t；
 *    b. AutoCtrl_SetFeedback(&ctrl, &feedback);
 *    c. 满足任务触发条件时调用 AutoCtrl_StartTemplate(...);
 *    d. AutoCtrl_Update(&ctrl, now_ms);
 *    e. 下发 ctrl.chassis_cmd 与 ctrl.pole_cmd 到执行层。
 * 3) 通过 AutoCtrl_GetResult / AutoCtrl_GetFault 读取任务结果。
 */

/* 初始化控制器上下文（不保留历史任务状态）。 */
void AutoCtrl_Init(auto_ctrl_t *ctrl);

/* 软复位控制器：保留 yaw 零点偏移，其余状态恢复初始。 */
void AutoCtrl_Reset(auto_ctrl_t *ctrl);

/* 设置 yaw 零点偏移。 */
void AutoCtrl_SetYawZeroOffset(auto_ctrl_t *ctrl, float raw_yaw_rad);

/* 设置自动控制使用的 yaw 来源。 */
void AutoCtrl_SetYawSource(auto_ctrl_t *ctrl, auto_ctrl_yaw_source_e source);

/* 查询自动控制当前使用的 yaw 来源。 */
auto_ctrl_yaw_source_e AutoCtrl_GetYawSource(const auto_ctrl_t *ctrl);

/* 设置外部 yaw 修正角速度命令。 */
void AutoCtrl_SetYawRateCommand(auto_ctrl_t *ctrl, float wz_rad_s);

/* 设置外部横向速度命令。 */
void AutoCtrl_SetLateralVelocityCommand(auto_ctrl_t *ctrl, float vy_mps);

/* 更新外部反馈快照（会完成 yaw 零点补偿）。 */
void AutoCtrl_SetFeedback(auto_ctrl_t *ctrl,
                          const auto_ctrl_feedback_t *feedback);

/* 直接启动一次模板任务。 */
bool AutoCtrl_StartTemplate(auto_ctrl_t *ctrl,
                            auto_ctrl_template_e template_id,
                            auto_ctrl_travel_dir_e travel_dir,
                            float target_yaw_rad,
                            float yaw_tolerance_rad,
                            auto_ctrl_sensor_mode_e sensor_mode,
                            uint32_t now_ms);

/* 周期更新状态机并生成输出命令。 */
void AutoCtrl_Update(auto_ctrl_t *ctrl, uint32_t now_ms);

/* 外部中止当前自动控任务。 */
void AutoCtrl_Abort(auto_ctrl_t *ctrl);

/* 查询当前是否处于运行中（PREALIGN/RUN_TEMPLATE）。 */
bool AutoCtrl_IsBusy(const auto_ctrl_t *ctrl);

/* 查询当前运行状态。 */
auto_ctrl_run_state_e AutoCtrl_GetState(const auto_ctrl_t *ctrl);

/* 查询本次任务结果。 */
auto_ctrl_result_e AutoCtrl_GetResult(const auto_ctrl_t *ctrl);

/* 查询故障码。 */
auto_ctrl_fault_e AutoCtrl_GetFault(const auto_ctrl_t *ctrl);

/* 查询当前模板编号。 */
auto_ctrl_template_e AutoCtrl_GetTemplate(const auto_ctrl_t *ctrl);

/* 查询模板当前 step 编号。 */
uint8_t AutoCtrl_GetStepIndex(const auto_ctrl_t *ctrl);

#ifdef __cplusplus
}
#endif
