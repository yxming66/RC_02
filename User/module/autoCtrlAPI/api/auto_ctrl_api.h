#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "module/autoCtrlAPI/core/auto_ctrl_def.h"
#include "module/autoCtrlAPI/core/auto_ctrl_math.h"
#include "module/autoCtrlAPI/template/auto_ctrl_template.h"
#include "module/autoCtrlAPI/transition/auto_ctrl_transition.h"
#include "module/chassis.h"
#include "module/pole.h"

typedef struct {
  float yaw_raw_deg;
  float yaw_auto_deg;
  float sick_front_cm;
  float sick_left_cm;
  float sick_right_cm;
  float sick_rear_cm;
  bool bottom_photo_triggered;
} auto_ctrl_feedback_t;

typedef struct {
  auto_ctrl_run_state_e state;
  auto_ctrl_result_e result;
  auto_ctrl_fault_e fault;

  auto_ctrl_zone_e current_zone;
  auto_ctrl_zone_e target_zone;
  const auto_ctrl_transition_t *transition;
  auto_ctrl_template_e template_id;

  float yaw_zero_offset_deg;
  float target_yaw_deg;
  float yaw_tolerance_deg;
  float yaw_error_deg;

  uint32_t state_enter_time_ms;
  uint32_t prealign_timeout_ms;
  uint32_t template_timeout_ms;

  auto_ctrl_template_ctx_t template_ctx;
  auto_ctrl_feedback_t feedback;

  Chassis_CMD_t chassis_cmd;
  Pole_CMD_t pole_cmd;
} auto_ctrl_t;

void AutoCtrl_Init(auto_ctrl_t *ctrl);
void AutoCtrl_Reset(auto_ctrl_t *ctrl);
void AutoCtrl_SetYawZeroOffset(auto_ctrl_t *ctrl, float raw_yaw_deg);
void AutoCtrl_SetFeedback(auto_ctrl_t *ctrl,
                          const auto_ctrl_feedback_t *feedback);
bool AutoCtrl_StartTransition(auto_ctrl_t *ctrl, auto_ctrl_zone_e from,
                              auto_ctrl_zone_e to, uint32_t now_ms);
void AutoCtrl_Update(auto_ctrl_t *ctrl, uint32_t now_ms);
void AutoCtrl_Abort(auto_ctrl_t *ctrl);

bool AutoCtrl_IsBusy(const auto_ctrl_t *ctrl);
auto_ctrl_run_state_e AutoCtrl_GetState(const auto_ctrl_t *ctrl);
auto_ctrl_result_e AutoCtrl_GetResult(const auto_ctrl_t *ctrl);
auto_ctrl_fault_e AutoCtrl_GetFault(const auto_ctrl_t *ctrl);
auto_ctrl_template_e AutoCtrl_GetTemplate(const auto_ctrl_t *ctrl);
uint8_t AutoCtrl_GetStepIndex(const auto_ctrl_t *ctrl);

#ifdef __cplusplus
}
#endif