#include "module/autoCtrlAPI/api/auto_ctrl_api.h"

/*
 * auto_ctrl_api.c
 *
 * 作用：
 * - 实现 AutoCtrl 主状态机（IDLE/PREALIGN/RUN_TEMPLATE/结束态）；
 * - 连接 transition、template 与 primitive 三层；
 * - 接收外部反馈并输出底盘/撑杆命令。
 *
 * 上层典型调用时序：
 * 1) 初始化阶段调用 AutoCtrl_Init；
 * 2) 每个控制周期先写入 AutoCtrl_SetFeedback，再调用 AutoCtrl_Update；
 * 3) 需要执行区块跳转时调用 AutoCtrl_StartTransition；
 * 4) 周期读取 ctrl->chassis_cmd / ctrl->pole_cmd 并下发执行机构；
 * 5) 任务结束后通过 GetResult/GetFault 判定成功或故障原因。
 */

#include <math.h>
#include <string.h>

#include "module/config.h"
#include "module/autoCtrlAPI/primitive/auto_ctrl_primitive.h"
#include "module/autoCtrlAPI/template/auto_ctrl_template_runner.h"
#include "module/autoCtrlAPI/transition/auto_ctrl_transition.h"
#include "module/autoCtrlAPI/transition/auto_ctrl_zone.h"

#define AUTO_CTRL_PREALIGN_TIMEOUT_MS (2000u)
#define AUTO_CTRL_TEMPLATE_TIMEOUT_MS (10000u)

/* 判断模板是否属于 200 跨越类（上/下都按跨越逻辑处理）。 */
static bool AutoCtrl_IsCross200Template(auto_ctrl_template_e template_id) {
  return template_id == AUTO_CTRL_TEMPLATE_ASCEND_200 ||
         template_id == AUTO_CTRL_TEMPLATE_DESCEND_200;
}

/* SICK 距离值有效性检查。 */
static bool AutoCtrl_IsSickDistanceValid(float distance_cm,
                float valid_min_cm,
                float valid_max_cm) {
  return isfinite(distance_cm) && distance_cm >= valid_min_cm &&
    distance_cm <= valid_max_cm;
}

/*
 * 估计 SICK 侧向差带来的姿态辅助角。
 * 仅在指定传感器模式下生效，返回 true 表示 assist_deg 可用。
 */
static bool AutoCtrl_TryGetSickAssistDeg(const auto_ctrl_t *ctrl,
                                         float *assist_deg) {
  const Config_RobotParam_t *robot_param;
  float valid_min_cm;
  float valid_max_cm;
  float norm_err_deadband;
  float norm_err_to_deg;
  float assist_max_deg;
  float left_cm;
  float right_cm;
  float norm_err;
  float denom;

  if (ctrl == 0 || assist_deg == 0 || ctrl->transition == 0) {
    return false;
  }

  if (ctrl->transition->sensor_mode != AUTO_CTRL_SENSOR_MODE_SICK_FRONT_AND_BOTTOM) {
    return false;
  }

  robot_param = Config_GetRobotParam();
  if (robot_param == 0) {
    return false;
  }

  valid_min_cm = robot_param->auto_ctrl_param.sick_valid_min_cm;
  valid_max_cm = robot_param->auto_ctrl_param.sick_valid_max_cm;
  norm_err_deadband = robot_param->auto_ctrl_param.sick_norm_err_deadband;
  norm_err_to_deg = robot_param->auto_ctrl_param.sick_norm_err_to_deg;
  assist_max_deg = robot_param->auto_ctrl_param.sick_assist_max_deg;

  if (!AutoCtrl_IsSickDistanceValid(ctrl->feedback.sick_left_cm, valid_min_cm,
                                    valid_max_cm) ||
      !AutoCtrl_IsSickDistanceValid(ctrl->feedback.sick_right_cm, valid_min_cm,
                                    valid_max_cm)) {
    return false;
  }

  left_cm = ctrl->feedback.sick_left_cm;
  right_cm = ctrl->feedback.sick_right_cm;
  denom = fabsf(left_cm) + fabsf(right_cm);
  if (denom < 1e-3f) {
    return false;
  }

  norm_err = (left_cm - right_cm) / denom;
  if (fabsf(norm_err) < norm_err_deadband) {
    return false;
  }

  *assist_deg =
      AutoCtrlPrimitive_Clamp(norm_err * norm_err_to_deg, -assist_max_deg,
                              assist_max_deg);
  return true;
}

/* 进入指定运行状态并刷新状态进入时间戳。 */
static void AutoCtrl_EnterState(auto_ctrl_t *ctrl, auto_ctrl_run_state_e state,
                                uint32_t now_ms) {
  ctrl->state = state;
  ctrl->state_enter_time_ms = now_ms;
}

/* 统一收敛结束态：写结果/故障并清空输出。 */
static void AutoCtrl_Finish(auto_ctrl_t *ctrl, auto_ctrl_result_e result,
                            auto_ctrl_fault_e fault,
                            auto_ctrl_run_state_e state) {
  ctrl->result = result;
  ctrl->fault = fault;
  ctrl->state = state;
  AutoCtrlPrimitive_ResetOutputs(ctrl);
}

/* 初始化控制器到可接收任务的空闲状态。 */
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

/* 重置控制器但保留 yaw 零点。 */
void AutoCtrl_Reset(auto_ctrl_t *ctrl) {
  if (ctrl == 0) return;
  float yaw_zero = ctrl->yaw_zero_offset_deg;
  AutoCtrl_Init(ctrl);
  ctrl->yaw_zero_offset_deg = yaw_zero;
}

/* 记录当前 raw yaw 为新的零点偏移。 */
void AutoCtrl_SetYawZeroOffset(auto_ctrl_t *ctrl, float raw_yaw_deg) {
  if (ctrl == 0) return;
  ctrl->yaw_zero_offset_deg = raw_yaw_deg;
}

/* 更新反馈并把输入 yaw 转为 auto yaw。 */
void AutoCtrl_SetFeedback(auto_ctrl_t *ctrl,
                          const auto_ctrl_feedback_t *feedback) {
  if (ctrl == 0 || feedback == 0) return;
  ctrl->feedback = *feedback;
  ctrl->feedback.yaw_auto_deg =
  AutoCtrlMath_WrapYawDeg(feedback->yaw_auto_deg - ctrl->yaw_zero_offset_deg);
}

/*
 * 启动一条 from->to 转移任务。
 * 时序位置：由上层在“空闲或结束态”发起，成功后立即进入 PREALIGN。
 */
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

/*
 * 主周期函数：
 * 1) 更新 yaw 误差（必要时融合 SICK 辅助）；
 * 2) 根据 state 执行 PREALIGN 或模板；
 * 3) 在完成/超时/异常时统一进入结束态。
 */
void AutoCtrl_Update(auto_ctrl_t *ctrl, uint32_t now_ms) {
  const Config_RobotParam_t *robot_param;
  float sick_assist_gain;
  float imu_yaw_error_deg;
  float sick_assist_deg;

  if (ctrl == 0) return;

  robot_param = Config_GetRobotParam();
  sick_assist_gain =
      (robot_param == 0) ? 0.0f : robot_param->auto_ctrl_param.sick_assist_gain;

  imu_yaw_error_deg = AutoCtrlMath_GetYawErrorDeg(ctrl->target_yaw_deg,
                                                   ctrl->feedback.yaw_auto_deg);
  ctrl->yaw_error_deg = imu_yaw_error_deg;

  if (ctrl->state == AUTO_CTRL_STATE_PREALIGN &&
      AutoCtrl_TryGetSickAssistDeg(ctrl, &sick_assist_deg)) {
    ctrl->yaw_error_deg =
        imu_yaw_error_deg - sick_assist_gain * sick_assist_deg;
  }

  AutoCtrlPrimitive_ResetOutputs(ctrl);

  switch (ctrl->state) {
    /* 空闲和结束态不再产生命令，等待上层下一次 StartTransition。 */
    case AUTO_CTRL_STATE_IDLE:
    case AUTO_CTRL_STATE_SUCCESS:
    case AUTO_CTRL_STATE_FAIL:
    case AUTO_CTRL_STATE_ABORT:
      return;

    /*
     * 预对齐阶段：
     * - 持续做 yaw 收敛（部分模板叠加缓慢前进）；
     * - 收敛后切换到 RUN_TEMPLATE；
     * - 超时则直接失败。
     */
    case AUTO_CTRL_STATE_PREALIGN:
      if (AutoCtrl_IsCross200Template(ctrl->template_id) && robot_param != 0) {
        const float move_sign =
            (ctrl->template_id == AUTO_CTRL_TEMPLATE_DESCEND_200) ? -1.0f : 1.0f;
        AutoCtrlPrimitive_ApplyPrealignWithForward(
            ctrl, move_sign * robot_param->auto_ctrl_param.climb_forward_speed);
      } else {
        AutoCtrlPrimitive_ApplyPrealign(ctrl);
      }
      if (fabsf(ctrl->yaw_error_deg) <= ctrl->yaw_tolerance_deg) {
        memset(&ctrl->template_ctx, 0, sizeof(ctrl->template_ctx));
        AutoCtrl_EnterState(ctrl, AUTO_CTRL_STATE_RUN_TEMPLATE, now_ms);
      } else if ((now_ms - ctrl->state_enter_time_ms) > ctrl->prealign_timeout_ms) {
        AutoCtrl_Finish(ctrl, AUTO_CTRL_RESULT_FAIL,
                        AUTO_CTRL_FAULT_PREALIGN_TIMEOUT,
                        AUTO_CTRL_STATE_FAIL);
      }
      return;

    /*
     * 模板执行阶段：
     * - 周期调用模板状态机推进 step；
     * - 模板返回 true 表示完成并切成功态；
     * - 模板写 fault 或总超时则切失败态。
     */
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
      } else if (ctrl->fault != AUTO_CTRL_FAULT_NONE) {
        AutoCtrl_Finish(ctrl, AUTO_CTRL_RESULT_FAIL, ctrl->fault,
                        AUTO_CTRL_STATE_FAIL);
      }
      return;

    default:
      AutoCtrl_Finish(ctrl, AUTO_CTRL_RESULT_FAIL, AUTO_CTRL_FAULT_INVALID_TEMPLATE,
                      AUTO_CTRL_STATE_FAIL);
      return;
  }
}

/* 外部主动中止任务。 */
void AutoCtrl_Abort(auto_ctrl_t *ctrl) {
  if (ctrl == 0) return;
  AutoCtrl_Finish(ctrl, AUTO_CTRL_RESULT_ABORTED, AUTO_CTRL_FAULT_ABORTED,
                  AUTO_CTRL_STATE_ABORT);
}

/* 判断是否处于忙状态。 */
bool AutoCtrl_IsBusy(const auto_ctrl_t *ctrl) {
  if (ctrl == 0) return false;
  return ctrl->state == AUTO_CTRL_STATE_PREALIGN ||
         ctrl->state == AUTO_CTRL_STATE_RUN_TEMPLATE;
}

/* 读取当前运行状态。 */
auto_ctrl_run_state_e AutoCtrl_GetState(const auto_ctrl_t *ctrl) {
  return (ctrl == 0) ? AUTO_CTRL_STATE_IDLE : ctrl->state;
}

/* 读取任务结果。 */
auto_ctrl_result_e AutoCtrl_GetResult(const auto_ctrl_t *ctrl) {
  return (ctrl == 0) ? AUTO_CTRL_RESULT_NONE : ctrl->result;
}

/* 读取故障码。 */
auto_ctrl_fault_e AutoCtrl_GetFault(const auto_ctrl_t *ctrl) {
  return (ctrl == 0) ? AUTO_CTRL_FAULT_INVALID_ZONE : ctrl->fault;
}

/* 读取当前模板 ID。 */
auto_ctrl_template_e AutoCtrl_GetTemplate(const auto_ctrl_t *ctrl) {
  return (ctrl == 0) ? AUTO_CTRL_TEMPLATE_NONE : ctrl->template_id;
}

/* 读取模板当前 step 索引。 */
uint8_t AutoCtrl_GetStepIndex(const auto_ctrl_t *ctrl) {
  return (ctrl == 0) ? 0u : ctrl->template_ctx.step_index;
}