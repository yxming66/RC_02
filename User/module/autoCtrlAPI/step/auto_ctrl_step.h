#pragma once

/**
 * @file auto_ctrl_step.h
 * @brief AutoCtrl 通用 step 执行器定义。
 *
 * step 模块提供与模板解耦的“单步动作执行器”，
 * 通过 step 定义 + 运行时上下文实现可复用的时序动作。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "module/autoCtrlAPI/api/auto_ctrl_api.h"

typedef enum {
  AUTO_CTRL_STEP_NONE = 0,
  AUTO_CTRL_STEP_STOP_CHASSIS,    /* 立即停底盘。 */
  AUTO_CTRL_STEP_FLAT_MOVE_TIME,  /* 以给定 vx 行走一段时间。 */
  AUTO_CTRL_STEP_SET_POLE_TARGET, /* 发送前后杆目标位并等待稳定。 */
  AUTO_CTRL_STEP_WAIT_TIMEOUT,    /* 纯时间等待。 */
  AUTO_CTRL_STEP_WAIT_HEAD_FRONT_PHOTO, /* 等待头向前光电。 */
  AUTO_CTRL_STEP_WAIT_HEAD_REAR_PHOTO,  /* 等待头向后光电。 */
} auto_ctrl_step_id_e;

typedef enum {
  AUTO_CTRL_STEP_STATUS_RUNNING = 0,
  AUTO_CTRL_STEP_STATUS_DONE, /* 当前 step 完成。 */
  AUTO_CTRL_STEP_STATUS_FAIL, /* 当前 step 失败。 */
} auto_ctrl_step_status_e;

typedef struct {
  auto_ctrl_step_id_e id;   /* 当前运行 step 类型。 */
  uint32_t enter_time_ms;    /* 当前 step 进入时刻（ms）。 */
  uint32_t timeout_ms;       /* 当前 step 超时阈值（ms）。 */
  bool entered;              /* 当前 step 是否已初始化。 */
} auto_ctrl_step_runtime_t;

typedef struct {
  auto_ctrl_step_id_e id; /* step 类型。 */
  float param0;           /* 参数 0：通常为 vx 或前杆目标。 */
  float param1;           /* 参数 1：通常为后杆目标。 */
  uint32_t timeout_ms;    /* step 超时/持续时间（ms）。 */
} auto_ctrl_step_def_t;

/* 复位 step 运行时上下文。 */
void AutoCtrlStep_Reset(auto_ctrl_step_runtime_t *runtime);

/* 执行一个 step，并返回运行状态。 */
auto_ctrl_step_status_e AutoCtrlStep_Run(auto_ctrl_t *ctrl,
                                         auto_ctrl_step_runtime_t *runtime,
                                         const auto_ctrl_step_def_t *step,
                                         uint32_t now_ms);

#ifdef __cplusplus
}
#endif