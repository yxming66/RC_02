#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "module/autoCtrlAPI/api/auto_ctrl_api.h"

typedef enum {
  AUTO_CTRL_STEP_NONE = 0,
  AUTO_CTRL_STEP_STOP_CHASSIS,
  AUTO_CTRL_STEP_FLAT_MOVE_TIME,
  AUTO_CTRL_STEP_SET_POLE_TARGET,
  AUTO_CTRL_STEP_WAIT_TIMEOUT,
} auto_ctrl_step_id_e;

typedef enum {
  AUTO_CTRL_STEP_STATUS_RUNNING = 0,
  AUTO_CTRL_STEP_STATUS_DONE,
  AUTO_CTRL_STEP_STATUS_FAIL,
} auto_ctrl_step_status_e;

typedef struct {
  auto_ctrl_step_id_e id;
  uint32_t enter_time_ms;
  uint32_t timeout_ms;
  bool entered;
} auto_ctrl_step_runtime_t;

typedef struct {
  auto_ctrl_step_id_e id;
  float param0;
  float param1;
  uint32_t timeout_ms;
} auto_ctrl_step_def_t;

void AutoCtrlStep_Reset(auto_ctrl_step_runtime_t *runtime);
auto_ctrl_step_status_e AutoCtrlStep_Run(auto_ctrl_t *ctrl,
                                         auto_ctrl_step_runtime_t *runtime,
                                         const auto_ctrl_step_def_t *step,
                                         uint32_t now_ms);

#ifdef __cplusplus
}
#endif