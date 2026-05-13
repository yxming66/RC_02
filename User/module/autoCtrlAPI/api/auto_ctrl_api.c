#include "module/autoCtrlAPI/api/auto_ctrl_api.h"

/*
 * auto_ctrl_api.c
 *
 * 作用：
 * - 实现 AutoCtrl 主状态机（IDLE/PREALIGN/RUN_TEMPLATE/结束态）；
 * - 连接 template 与 primitive 两层；
 * - 接收外部反馈并输出底盘/撑杆命令。
 *
 * 上层典型调用时序：
 * 1) 初始化阶段调用 AutoCtrl_Init；
 * 2) 每个控制周期先写入 AutoCtrl_SetFeedback，再调用 AutoCtrl_Update；
 * 3) 需要执行模板时调用 AutoCtrl_StartTemplate；
 * 4) 周期读取 ctrl->chassis_cmd / ctrl->pole_cmd 并下发执行机构；
 * 5) 任务结束后通过 GetResult/GetFault 判定成功或故障原因。
 */

#include <math.h>
#include <string.h>

#include "module/config.h"
#include "module/autoCtrlAPI/primitive/auto_ctrl_primitive.h"
#include "module/autoCtrlAPI/template/auto_ctrl_template_runner.h"

#define AUTO_CTRL_PREALIGN_TIMEOUT_MS (2000u)
#define AUTO_CTRL_TEMPLATE_TIMEOUT_MS (10000u)
#define AUTO_CTRL_SICK_VALID_STABLE_MS (60u)
#define AUTO_CTRL_SICK_ALIGN_STABLE_MS (120u)
#define AUTO_CTRL_YAW_ALIGN_STABLE_MS (120u)
#define AUTO_CTRL_SICK_ACCEPT_MAX_YAW_DIFF_RAD (1.57079632679f)

/* 判断模板是否属于 200 跨越类（上/下都按跨越逻辑处理）。 */
static const AutoCtrl_TemplateParam_t *AutoCtrl_GetTemplateParams(
    const Config_RobotParam_t *robot_param, auto_ctrl_template_e template_id) {
  if (robot_param == 0) {
    return 0;
  }

  switch (template_id) {
    case AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD:
      return &robot_param->auto_ctrl_param.head_ascend_200;
    case AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD:
      return &robot_param->auto_ctrl_param.head_ascend_400;
    case AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD:
      return &robot_param->auto_ctrl_param.head_descend_200;
    case AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD:
      return &robot_param->auto_ctrl_param.head_descend_400;
    case AUTO_CTRL_TEMPLATE_ASCEND_200_TAIL:
      return &robot_param->auto_ctrl_param.tail_ascend_200;
    case AUTO_CTRL_TEMPLATE_ASCEND_400_TAIL:
      return &robot_param->auto_ctrl_param.tail_ascend_400;
    case AUTO_CTRL_TEMPLATE_DESCEND_200_TAIL:
      return &robot_param->auto_ctrl_param.tail_descend_200;
    case AUTO_CTRL_TEMPLATE_DESCEND_400_TAIL:
      return &robot_param->auto_ctrl_param.tail_descend_400;
    case AUTO_CTRL_TEMPLATE_NONE:
    default:
      return 0;
  }
}

/* SICK 距离值有效性检查。 */
static bool AutoCtrl_IsSickDistanceValid(float distance_cm,
                float valid_min_cm,
                float valid_max_cm) {
  return isfinite(distance_cm) && distance_cm >= valid_min_cm &&
    distance_cm <= valid_max_cm;
}

static bool AutoCtrl_IsSickYawUsable(const auto_ctrl_t *ctrl,
                                     const Config_RobotParam_t *robot_param) {
  const float valid_min_cm = robot_param->auto_ctrl_param.common.sick_valid_min_cm;
  const float valid_max_cm = robot_param->auto_ctrl_param.common.sick_valid_max_cm;

  if (ctrl == 0 || robot_param == 0) {
    return false;
  }

  if (ctrl->sensor_mode != AUTO_CTRL_SENSOR_MODE_SICK_FRONT_AND_BOTTOM) {
    return false;
  }

  return AutoCtrl_IsSickDistanceValid(ctrl->feedback.sick_front_left_cm,
                                      valid_min_cm, valid_max_cm) &&
         AutoCtrl_IsSickDistanceValid(ctrl->feedback.sick_front_right_cm,
                                      valid_min_cm, valid_max_cm);
}

static bool AutoCtrl_IsSickYawAcceptable(const auto_ctrl_t *ctrl) {
  float yaw_diff_rad;
  float accept_limit_rad;

  if (ctrl == 0) {
    return false;
  }

  accept_limit_rad = ctrl->sick_accept_max_yaw_diff_rad;
  if (accept_limit_rad <= 0.0f) {
    accept_limit_rad = AUTO_CTRL_SICK_ACCEPT_MAX_YAW_DIFF_RAD;
  }

  yaw_diff_rad = AutoCtrlMath_GetYawErrorRad(ctrl->yaw_search_accept_center_rad,
                                             ctrl->feedback.yaw_auto_rad);
  return fabsf(yaw_diff_rad) <= accept_limit_rad;
}

/*
 * 估计 SICK 侧向差带来的姿态辅助角。
 * 仅在指定传感器模式下生效，返回 true 表示 assist_rad 可用。
 */
static bool AutoCtrl_TryGetSickAssistRad(const auto_ctrl_t *ctrl,
                                         float *assist_rad) {
  const Config_RobotParam_t *robot_param;
  float valid_min_cm;
  float valid_max_cm;
  float norm_err_deadband;
  float norm_err_to_rad;
  float assist_max_rad;
  float left_cm;
  float right_cm;
  float norm_err;
  float denom;

  if (ctrl == 0 || assist_rad == 0) {
    return false;
  }

  robot_param = Config_GetRobotParam();
  if (!AutoCtrl_IsSickYawUsable(ctrl, robot_param)) {
    return false;
  }

  valid_min_cm = robot_param->auto_ctrl_param.common.sick_valid_min_cm;
  valid_max_cm = robot_param->auto_ctrl_param.common.sick_valid_max_cm;
  norm_err_deadband = robot_param->auto_ctrl_param.common.sick_norm_err_deadband;
  norm_err_to_rad = robot_param->auto_ctrl_param.common.sick_norm_err_to_rad;
  assist_max_rad = robot_param->auto_ctrl_param.common.sick_assist_max_rad;

  if (!AutoCtrl_IsSickDistanceValid(ctrl->feedback.sick_front_left_cm,
                                    valid_min_cm, valid_max_cm) ||
      !AutoCtrl_IsSickDistanceValid(ctrl->feedback.sick_front_right_cm,
                                    valid_min_cm, valid_max_cm)) {
    return false;
  }

  left_cm = ctrl->feedback.sick_front_left_cm;
  right_cm = ctrl->feedback.sick_front_right_cm;
  denom = fabsf(left_cm) + fabsf(right_cm);
  if (denom < 1e-3f) {
    return false;
  }

  norm_err = (left_cm - right_cm) / denom;
  if (fabsf(norm_err) < norm_err_deadband) {
    *assist_rad = 0.0f;
    return true;
  }

  *assist_rad =
      AutoCtrlPrimitive_Clamp(norm_err * norm_err_to_rad, -assist_max_rad,
                              assist_max_rad);
  return true;
}

static bool AutoCtrl_IsSickAlignedStable(auto_ctrl_t *ctrl, float sick_error_rad,
                                         uint32_t now_ms) {
  if (ctrl == 0) {
    return false;
  }

  if (fabsf(sick_error_rad) > ctrl->yaw_tolerance_rad) {
    ctrl->sick_align_stable_since_ms = 0u;
    return false;
  }

  if (ctrl->sick_align_stable_since_ms == 0u) {
    ctrl->sick_align_stable_since_ms = now_ms;
  }

  return (now_ms - ctrl->sick_align_stable_since_ms) >=
         AUTO_CTRL_SICK_ALIGN_STABLE_MS;
}

static bool AutoCtrl_IsYawAlignedStable(auto_ctrl_t *ctrl, float yaw_error_rad,
                                        uint32_t now_ms) {
  if (ctrl == 0) {
    return false;
  }

  if (fabsf(yaw_error_rad) > ctrl->yaw_tolerance_rad) {
    ctrl->yaw_align_stable_since_ms = 0u;
    return false;
  }

  if (ctrl->yaw_align_stable_since_ms == 0u) {
    ctrl->yaw_align_stable_since_ms = now_ms;
    return false;
  }

  return (now_ms - ctrl->yaw_align_stable_since_ms) >=
         AUTO_CTRL_YAW_ALIGN_STABLE_MS;
}

/* 进入指定运行状态并刷新状态进入时间戳。 */
static void AutoCtrl_EnterState(auto_ctrl_t *ctrl, auto_ctrl_run_state_e state,
                                uint32_t now_ms) {
  ctrl->state = state;
  ctrl->state_enter_time_ms = now_ms;
}

/* 统一收敛结束态：写结果/故障并按结束类型处理输出。 */
static void AutoCtrl_Finish(auto_ctrl_t *ctrl, auto_ctrl_result_e result,
                            auto_ctrl_fault_e fault,
                            auto_ctrl_run_state_e state) {
  ctrl->result = result;
  ctrl->fault = fault;
  ctrl->state = state;

  if (result == AUTO_CTRL_RESULT_SUCCESS || result == AUTO_CTRL_RESULT_NONE) {
    AutoCtrlPrimitive_ResetOutputs(ctrl);
    return;
  }

  AutoCtrlPrimitive_ResetChassis(ctrl);
}

/* 初始化控制器到可接收任务的空闲状态。 */
void AutoCtrl_Init(auto_ctrl_t *ctrl) {
  if (ctrl == 0) return;
  memset(ctrl, 0, sizeof(*ctrl));
  ctrl->prealign_timeout_ms = AUTO_CTRL_PREALIGN_TIMEOUT_MS;
  ctrl->template_timeout_ms = AUTO_CTRL_TEMPLATE_TIMEOUT_MS;
  ctrl->state = AUTO_CTRL_STATE_IDLE;
  ctrl->result = AUTO_CTRL_RESULT_NONE;
  ctrl->fault = AUTO_CTRL_FAULT_NONE;
  ctrl->template_id = AUTO_CTRL_TEMPLATE_NONE;
  ctrl->travel_dir = AUTO_CTRL_TRAVEL_DIR_HEAD_FORWARD;
  ctrl->sensor_mode = AUTO_CTRL_SENSOR_MODE_NONE;
  ctrl->yaw_source = AUTO_CTRL_YAW_SOURCE_STM32;
  AutoCtrlPrimitive_ResetOutputs(ctrl);
}

/* 重置控制器但保留 yaw 零点。 */
void AutoCtrl_Reset(auto_ctrl_t *ctrl) {
  if (ctrl == 0) return;
  float yaw_zero = ctrl->yaw_zero_offset_rad;
  AutoCtrl_Init(ctrl);
  ctrl->yaw_zero_offset_rad = yaw_zero;
}

/* 记录当前 raw yaw 为新的零点偏移。 */
void AutoCtrl_SetYawZeroOffset(auto_ctrl_t *ctrl, float raw_yaw_rad) {
  if (ctrl == 0) return;
  ctrl->yaw_zero_offset_rad = raw_yaw_rad;
}

/* 设置 yaw 来源。 */
void AutoCtrl_SetYawSource(auto_ctrl_t *ctrl, auto_ctrl_yaw_source_e source) {
  if (ctrl == 0) return;
  if (source < AUTO_CTRL_YAW_SOURCE_STM32 ||
      source > AUTO_CTRL_YAW_SOURCE_PC) {
    source = AUTO_CTRL_YAW_SOURCE_STM32;
  }
  ctrl->yaw_source = source;
}

/* 查询 yaw 来源。 */
auto_ctrl_yaw_source_e AutoCtrl_GetYawSource(const auto_ctrl_t *ctrl) {
  return (ctrl == 0) ? AUTO_CTRL_YAW_SOURCE_STM32 : ctrl->yaw_source;
}

/* 更新反馈并把输入 yaw 转为 auto yaw。 */
void AutoCtrl_SetFeedback(auto_ctrl_t *ctrl,
                          const auto_ctrl_feedback_t *feedback) {
  if (ctrl == 0 || feedback == 0) return;
  ctrl->feedback = *feedback;
  ctrl->yaw_raw_rad = feedback->yaw_auto_rad;
  ctrl->feedback.yaw_auto_rad = AutoCtrlMath_WrapYawRad(
      feedback->yaw_auto_rad - ctrl->yaw_zero_offset_rad);
}

/*
 * 直接启动模板任务。
 * 时序位置：由上层在”空闲或结束态”发起，成功后立即进入 PREALIGN。
 */
bool AutoCtrl_StartTemplate(auto_ctrl_t *ctrl,
                            auto_ctrl_template_e template_id,
                            auto_ctrl_travel_dir_e travel_dir,
                            float target_yaw_rad,
                            float yaw_tolerance_rad,
                            auto_ctrl_sensor_mode_e sensor_mode,
                            uint32_t now_ms) {
  (void)travel_dir; /* travel_dir 已内嵌到 template_id 中，不再需要映射。 */
  if (ctrl == 0) return false;

  if (template_id <= AUTO_CTRL_TEMPLATE_NONE ||
      template_id > AUTO_CTRL_TEMPLATE_DESCEND_400_TAIL) {
    AutoCtrl_Finish(ctrl, AUTO_CTRL_RESULT_FAIL, AUTO_CTRL_FAULT_INVALID_TEMPLATE,
                    AUTO_CTRL_STATE_FAIL);
    return false;
  }

  if (sensor_mode < AUTO_CTRL_SENSOR_MODE_NONE ||
      sensor_mode > AUTO_CTRL_SENSOR_MODE_BOTTOM_ONLY) {
    AutoCtrl_Finish(ctrl, AUTO_CTRL_RESULT_FAIL,
                    AUTO_CTRL_FAULT_INVALID_TEMPLATE,
                    AUTO_CTRL_STATE_FAIL);
    return false;
  }

  memset(&ctrl->template_ctx, 0, sizeof(ctrl->template_ctx));
  ctrl->template_id = template_id;
  ctrl->travel_dir = travel_dir;
  ctrl->sensor_mode = sensor_mode;
  ctrl->target_yaw_rad = AutoCtrlMath_WrapYawRad(target_yaw_rad);
  ctrl->yaw_tolerance_rad = yaw_tolerance_rad;
  ctrl->yaw_error_rad = AutoCtrlMath_GetYawErrorRad(ctrl->target_yaw_rad,
                                                    ctrl->feedback.yaw_auto_rad);
  ctrl->prealign_mode = AUTO_CTRL_PREALIGN_BY_YAW;
  ctrl->yaw_search_accept_center_rad = ctrl->target_yaw_rad;
  ctrl->sick_accept_max_yaw_diff_rad = AUTO_CTRL_SICK_ACCEPT_MAX_YAW_DIFF_RAD;
  ctrl->sick_valid_now = AutoCtrl_IsSickYawAcceptable(ctrl);
  ctrl->prealign_done_by_sick = false;
  ctrl->sick_valid_stable_since_ms = ctrl->sick_valid_now ? now_ms : 0u;
  ctrl->sick_align_stable_since_ms = 0u;
  ctrl->yaw_align_stable_since_ms = 0u;
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
  float imu_yaw_error_rad;
  float blended_yaw_error_rad;
  float sick_assist_rad;
  bool sick_usable;

  if (ctrl == 0) return;

  robot_param = Config_GetRobotParam();

  imu_yaw_error_rad = AutoCtrlMath_GetYawErrorRad(ctrl->target_yaw_rad,
                                                   ctrl->feedback.yaw_auto_rad);
  ctrl->yaw_error_rad = imu_yaw_error_rad;
  sick_usable = AutoCtrl_IsSickYawUsable(ctrl, robot_param) &&
                AutoCtrl_IsSickYawAcceptable(ctrl);
  ctrl->sick_valid_now = sick_usable;

  AutoCtrlPrimitive_ResetOutputs(ctrl);

  switch (ctrl->state) {
    /* 空闲和结束态不再产生命令，等待上层下一次 StartTemplate。 */
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
      if (sick_usable) {
        if (ctrl->sick_valid_stable_since_ms == 0u) {
          ctrl->sick_valid_stable_since_ms = now_ms;
        }
      } else {
        ctrl->sick_valid_stable_since_ms = 0u;
      }

      blended_yaw_error_rad = imu_yaw_error_rad;
      if (ctrl->sick_valid_stable_since_ms != 0u &&
          (now_ms - ctrl->sick_valid_stable_since_ms) >=
              AUTO_CTRL_SICK_VALID_STABLE_MS &&
          AutoCtrl_TryGetSickAssistRad(ctrl, &sick_assist_rad)) {
        blended_yaw_error_rad -=
            sick_assist_rad * robot_param->auto_ctrl_param.common.sick_assist_gain;
      }
      ctrl->yaw_error_rad = blended_yaw_error_rad;

      const AutoCtrl_TemplateParam_t *template_param =
          AutoCtrl_GetTemplateParams(robot_param, ctrl->template_id);
      if (template_param != 0 && template_param->prealign_move_speed != 0.0f) {
        AutoCtrlPrimitive_ApplyPrealignWithForward(
            ctrl, template_param->prealign_move_speed);
      } else {
        AutoCtrlPrimitive_ApplyPrealign(ctrl);
      }
      if (robot_param != 0) {
        const float prealign_pole_speed =
            robot_param->pole_param.limit.support_lift_speed;
        AutoCtrlPrimitive_CommandPoleTargetWithSpeed(
            ctrl, robot_param->pole_param.preset.step_200_small[0],
            robot_param->pole_param.preset.step_200_small[1],
            prealign_pole_speed, prealign_pole_speed);
      }

      if (sick_usable) {
        ctrl->yaw_align_stable_since_ms = 0u;
      }

      if ((sick_usable &&
           AutoCtrl_IsSickAlignedStable(ctrl, ctrl->yaw_error_rad, now_ms)) ||
          (!sick_usable &&
           AutoCtrl_IsYawAlignedStable(ctrl, imu_yaw_error_rad, now_ms))) {
        ctrl->prealign_done_by_sick = sick_usable;
        ctrl->yaw_zero_offset_rad = ctrl->yaw_raw_rad;
        ctrl->feedback.yaw_auto_rad = 0.0f;
        ctrl->target_yaw_rad = 0.0f;
        ctrl->yaw_error_rad = 0.0f;
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
  return (ctrl == 0) ? AUTO_CTRL_FAULT_INVALID_TEMPLATE : ctrl->fault;
}

/* 读取当前模板 ID。 */
auto_ctrl_template_e AutoCtrl_GetTemplate(const auto_ctrl_t *ctrl) {
  return (ctrl == 0) ? AUTO_CTRL_TEMPLATE_NONE : ctrl->template_id;
}

/* 读取模板当前 step 索引。 */
uint8_t AutoCtrl_GetStepIndex(const auto_ctrl_t *ctrl) {
  return (ctrl == 0) ? 0u : ctrl->template_ctx.step_index;
}
