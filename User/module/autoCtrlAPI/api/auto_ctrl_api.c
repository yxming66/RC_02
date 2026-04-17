#include "module/autoCtrlAPI/api/auto_ctrl_api.h"

#include <string.h>

#include "module/autoCtrlAPI/primitive/auto_ctrl_primitive.h"
#include "module/autoCtrlAPI/template/auto_ctrl_template_runner.h"
#include "module/autoCtrlAPI/transition/auto_ctrl_transition.h"
#include "module/autoCtrlAPI/transition/auto_ctrl_zone.h"

#define AUTO_CTRL_PREALIGN_TIMEOUT_MS (2000u)
#define AUTO_CTRL_TEMPLATE_TIMEOUT_MS (4000u)

static void AutoCtrl_EnterState(auto_ctrl_t *ctrl, auto_ctrl_run_state_e state,
                                uint32_t now_ms) {
  ctrl->state = state;
  ctrl->state_enter_time_ms = now_ms;
}

static void AutoCtrl_Finish(auto_ctrl_t *ctrl, auto_ctrl_result_e result,
                            auto_ctrl_fault_e fault,
                            auto_ctrl_run_state_e state) {
  ctrl->result = result;
  ctrl->fault = fault;
  ctrl->state = state;
  AutoCtrlPrimitive_ResetOutputs(ctrl);
}

void AutoCtrl_Init(auto_ctrl_t *ctrl) {
  if (ctrl == 0) return;
  memset(ctrl, 0, sizeof(*ctrl));
  ctrl->prealign_timeout_ms = AUTO_CTRL_PREALIGN_TIMEOUT_MS;
  ctrl->template_timeout_ms = AUTO_CTRL_TEMPLATE_TIMEOUT_MS;
  ctrl->current_zone = AUTO_CTRL_ZONE_INVALID;
  ctrl->target_zone = AUTO_CTRL_ZONE_INVALID;
  ctrl->state = AUTO_CTRL_STATE_IDLE;
  ctrl->result = AUTO_CTRL_RESULT_NONE;
  ctrl->fault = AUTO_CTRL_FAULT_NONE;
  AutoCtrlPrimitive_ResetOutputs(ctrl);
}

void AutoCtrl_Reset(auto_ctrl_t *ctrl) {
  if (ctrl == 0) return;
  float yaw_zero = ctrl->yaw_zero_offset_deg;
  AutoCtrl_Init(ctrl);
  ctrl->yaw_zero_offset_deg = yaw_zero;
}

void AutoCtrl_SetYawZeroOffset(auto_ctrl_t *ctrl, float raw_yaw_deg) {
  if (ctrl == 0) return;
  ctrl->yaw_zero_offset_deg = raw_yaw_deg;
}

void AutoCtrl_SetFeedback(auto_ctrl_t *ctrl,
                          const auto_ctrl_feedback_t *feedback) {
  if (ctrl == 0 || feedback == 0) return;
  ctrl->feedback = *feedback;
  ctrl->feedback.yaw_auto_deg =
      AutoCtrlMath_WrapYawDeg(feedback->yaw_raw_deg - ctrl->yaw_zero_offset_deg);
}

bool AutoCtrl_StartTransition(auto_ctrl_t *ctrl, auto_ctrl_zone_e from,
                              auto_ctrl_zone_e to, uint32_t now_ms) {
  const auto_ctrl_transition_t *transition;

  if (ctrl == 0) return false;
  if (!AutoCtrlZone_IsValid(from) || !AutoCtrlZone_IsValid(to)) {
    AutoCtrl_Finish(ctrl, AUTO_CTRL_RESULT_FAIL, AUTO_CTRL_FAULT_INVALID_ZONE,
                    AUTO_CTRL_STATE_FAIL);
    return false;
  }

  transition = AutoCtrlTransition_Find(from, to);
  if (transition == 0) {
    AutoCtrl_Finish(ctrl, AUTO_CTRL_RESULT_FAIL,
                    AUTO_CTRL_FAULT_INVALID_TRANSITION,
                    AUTO_CTRL_STATE_FAIL);
    return false;
  }

  memset(&ctrl->template_ctx, 0, sizeof(ctrl->template_ctx));
  ctrl->current_zone = from;
  ctrl->target_zone = to;
  ctrl->transition = transition;
  ctrl->template_id = transition->template_id;
  ctrl->target_yaw_deg = transition->required_yaw_deg;
  ctrl->yaw_tolerance_deg = transition->yaw_tolerance_deg;
  ctrl->yaw_error_deg = AutoCtrlMath_GetYawErrorDeg(ctrl->target_yaw_deg,
                                                    ctrl->feedback.yaw_auto_deg);
  ctrl->result = AUTO_CTRL_RESULT_RUNNING;
  ctrl->fault = AUTO_CTRL_FAULT_NONE;
  AutoCtrl_EnterState(ctrl, AUTO_CTRL_STATE_PREALIGN, now_ms);
  return true;
}

void AutoCtrl_Update(auto_ctrl_t *ctrl, uint32_t now_ms) {
  if (ctrl == 0) return;

  ctrl->yaw_error_deg = AutoCtrlMath_GetYawErrorDeg(ctrl->target_yaw_deg,
                                                    ctrl->feedback.yaw_auto_deg);
  AutoCtrlPrimitive_ResetOutputs(ctrl);

  switch (ctrl->state) {
    case AUTO_CTRL_STATE_IDLE:
    case AUTO_CTRL_STATE_SUCCESS:
    case AUTO_CTRL_STATE_FAIL:
    case AUTO_CTRL_STATE_ABORT:
      return;

    case AUTO_CTRL_STATE_PREALIGN:
      AutoCtrlPrimitive_ApplyPrealign(ctrl);
      if (AutoCtrlMath_IsYawAligned(ctrl->target_yaw_deg,
                                    ctrl->feedback.yaw_auto_deg,
                                    ctrl->yaw_tolerance_deg)) {
        memset(&ctrl->template_ctx, 0, sizeof(ctrl->template_ctx));
        AutoCtrl_EnterState(ctrl, AUTO_CTRL_STATE_RUN_TEMPLATE, now_ms);
      } else if ((now_ms - ctrl->state_enter_time_ms) > ctrl->prealign_timeout_ms) {
        AutoCtrl_Finish(ctrl, AUTO_CTRL_RESULT_FAIL,
                        AUTO_CTRL_FAULT_PREALIGN_TIMEOUT,
                        AUTO_CTRL_STATE_FAIL);
      }
      return;

    case AUTO_CTRL_STATE_RUN_TEMPLATE:
      if ((now_ms - ctrl->state_enter_time_ms) > ctrl->template_timeout_ms) {
        AutoCtrl_Finish(ctrl, AUTO_CTRL_RESULT_FAIL,
                        AUTO_CTRL_FAULT_TEMPLATE_TIMEOUT,
                        AUTO_CTRL_STATE_FAIL);
        return;
      }

      if (AutoCtrlTemplate_Run(ctrl, now_ms)) {
        ctrl->current_zone = ctrl->target_zone;
        AutoCtrl_Finish(ctrl, AUTO_CTRL_RESULT_SUCCESS, AUTO_CTRL_FAULT_NONE,
                        AUTO_CTRL_STATE_SUCCESS);
      } else if (ctrl->fault == AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED) {
        AutoCtrl_Finish(ctrl, AUTO_CTRL_RESULT_FAIL,
                        AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED,
                        AUTO_CTRL_STATE_FAIL);
      }
      return;

    default:
      AutoCtrl_Finish(ctrl, AUTO_CTRL_RESULT_FAIL, AUTO_CTRL_FAULT_INVALID_TEMPLATE,
                      AUTO_CTRL_STATE_FAIL);
      return;
  }
}

void AutoCtrl_Abort(auto_ctrl_t *ctrl) {
  if (ctrl == 0) return;
  AutoCtrl_Finish(ctrl, AUTO_CTRL_RESULT_ABORTED, AUTO_CTRL_FAULT_ABORTED,
                  AUTO_CTRL_STATE_ABORT);
}

bool AutoCtrl_IsBusy(const auto_ctrl_t *ctrl) {
  if (ctrl == 0) return false;
  return ctrl->state == AUTO_CTRL_STATE_PREALIGN ||
         ctrl->state == AUTO_CTRL_STATE_RUN_TEMPLATE;
}

auto_ctrl_run_state_e AutoCtrl_GetState(const auto_ctrl_t *ctrl) {
  return (ctrl == 0) ? AUTO_CTRL_STATE_IDLE : ctrl->state;
}

auto_ctrl_result_e AutoCtrl_GetResult(const auto_ctrl_t *ctrl) {
  return (ctrl == 0) ? AUTO_CTRL_RESULT_NONE : ctrl->result;
}

auto_ctrl_fault_e AutoCtrl_GetFault(const auto_ctrl_t *ctrl) {
  return (ctrl == 0) ? AUTO_CTRL_FAULT_INVALID_ZONE : ctrl->fault;
}

auto_ctrl_template_e AutoCtrl_GetTemplate(const auto_ctrl_t *ctrl) {
  return (ctrl == 0) ? AUTO_CTRL_TEMPLATE_NONE : ctrl->template_id;
}

uint8_t AutoCtrl_GetStepIndex(const auto_ctrl_t *ctrl) {
  return (ctrl == 0) ? 0u : ctrl->template_ctx.step_index;
}