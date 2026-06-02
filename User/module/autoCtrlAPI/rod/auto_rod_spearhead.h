#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "module/rod_new.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  AUTO_ROD_SPEARHEAD_STATE_IDLE = 0,
  AUTO_ROD_SPEARHEAD_STATE_RUNNING,
  AUTO_ROD_SPEARHEAD_STATE_SUCCESS,
  AUTO_ROD_SPEARHEAD_STATE_FAIL,
  AUTO_ROD_SPEARHEAD_STATE_ABORT,
} AutoRodSpearhead_State_t;

typedef enum {
  AUTO_ROD_SPEARHEAD_RESULT_NONE = 0,
  AUTO_ROD_SPEARHEAD_RESULT_RUNNING,
  AUTO_ROD_SPEARHEAD_RESULT_SUCCESS,
  AUTO_ROD_SPEARHEAD_RESULT_FAIL,
  AUTO_ROD_SPEARHEAD_RESULT_ABORTED,
} AutoRodSpearhead_Result_t;

typedef enum {
  AUTO_ROD_SPEARHEAD_FAULT_NONE = 0,
  AUTO_ROD_SPEARHEAD_FAULT_TIMEOUT,
  AUTO_ROD_SPEARHEAD_FAULT_INVALID_PARAM,
  AUTO_ROD_SPEARHEAD_FAULT_ABORTED,
  AUTO_ROD_SPEARHEAD_FAULT_NO_SPEARHEAD,
} AutoRodSpearhead_Fault_t;

typedef struct {
  const RodNew_Params_t *rod_param;
  uint32_t open_delay_ms;
  uint32_t grab_high_delay_ms;
  uint32_t dock_wait_delay_ms;
  bool use_photo_check;
  uint32_t photo_check_ms;
} AutoRodSpearhead_Params_t;

typedef struct {
  bool rod_photo_triggered;
} AutoRodSpearhead_Feedback_t;

typedef struct {
  AutoRodSpearhead_State_t state;
  AutoRodSpearhead_Result_t result;
  AutoRodSpearhead_Fault_t fault;
  uint8_t step_index;
  uint32_t step_enter_time_ms;
  bool step_entered;
  bool photo_stable_started;
  bool photo_stable_state;
  uint32_t photo_stable_start_time_ms;
  bool rod_cmd_valid;
  RodNew_CMD_t rod_cmd;
  AutoRodSpearhead_Params_t param;
} AutoRodSpearhead_t;

void AutoRodSpearhead_Init(AutoRodSpearhead_t *ctrl,
                           const AutoRodSpearhead_Params_t *param);
bool AutoRodSpearhead_Start(AutoRodSpearhead_t *ctrl, uint32_t now_ms);
void AutoRodSpearhead_Update(AutoRodSpearhead_t *ctrl,
                             const AutoRodSpearhead_Feedback_t *feedback,
                             uint32_t now_ms);
void AutoRodSpearhead_Abort(AutoRodSpearhead_t *ctrl);
bool AutoRodSpearhead_IsBusy(const AutoRodSpearhead_t *ctrl);
AutoRodSpearhead_State_t AutoRodSpearhead_GetState(
    const AutoRodSpearhead_t *ctrl);
AutoRodSpearhead_Result_t AutoRodSpearhead_GetResult(
    const AutoRodSpearhead_t *ctrl);
AutoRodSpearhead_Fault_t AutoRodSpearhead_GetFault(
    const AutoRodSpearhead_t *ctrl);
uint8_t AutoRodSpearhead_GetStepIndex(const AutoRodSpearhead_t *ctrl);
const RodNew_CMD_t *AutoRodSpearhead_GetRodCommand(
    const AutoRodSpearhead_t *ctrl);

#ifdef __cplusplus
}
#endif
