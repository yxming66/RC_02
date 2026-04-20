#include "module/autoCtrlAPI/template/auto_ctrl_template_runner.h"

/*
 * auto_ctrl_template_runner.c
 *
 * 作用：
 * - 维护各模板的分步执行状态机；
 * - 在每个模板 step 中组合调用 primitive 动作；
 * - 对传感器条件、超时与故障进行模板内判定。
 *
 * 调用链：
 * AutoCtrl_Update -> AutoCtrlTemplate_Run -> AutoCtrlTemplate_RunXxx
 * -> AutoCtrlPrimitive_xxx（产出 chassis_cmd / pole_cmd）。
 */

#include <math.h>

#include "module/config.h"
#include "module/autoCtrlAPI/primitive/auto_ctrl_primitive.h"

/* step 首次进入时写入 enter 标记和 enter 时间。 */
static void AutoCtrlTemplate_EnterStep(auto_ctrl_t *ctrl, uint32_t now_ms) {
  if (!ctrl->template_ctx.step_entered) {
    ctrl->template_ctx.step_entered = true;
    ctrl->template_ctx.step_enter_time_ms = now_ms;
  }
}

/* 获取当前 step 已运行时长。 */
static uint32_t AutoCtrlTemplate_StepElapsed(const auto_ctrl_t *ctrl,
                                             uint32_t now_ms) {
  return now_ms - ctrl->template_ctx.step_enter_time_ms;
}

/* step 超时判断（内部确保 enter 初始化已完成）。 */
static bool AutoCtrlTemplate_StepTimeout(auto_ctrl_t *ctrl, uint32_t now_ms,
                                         uint32_t timeout_ms) {
  AutoCtrlTemplate_EnterStep(ctrl, now_ms);
  return AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >= timeout_ms;
}

/* 切换到下一个 step，并清除 enter 标记。 */
static void AutoCtrlTemplate_NextStep(auto_ctrl_t *ctrl) {
  ctrl->template_ctx.step_index++;
  ctrl->template_ctx.step_entered = false;
}

/* 判断当前 yaw 误差是否已经满足容差。 */
static bool AutoCtrlTemplate_IsYawAligned(const auto_ctrl_t *ctrl) {
  return fabsf(ctrl->yaw_error_deg) <= ctrl->yaw_tolerance_deg;
}

/* Ascend200 的首段切步条件：底部光电 + yaw 达标 + 撑杆稳定时间到。 */
static bool AutoCtrlTemplate_IsAscend200Ready(const auto_ctrl_t *ctrl,
                                              uint32_t now_ms,
                                              const Config_RobotParam_t *robot_param) {
  if (!ctrl->feedback.bottom_photo_triggered) {
    return false;
  }

  if (!AutoCtrlTemplate_IsYawAligned(ctrl)) {
    return false;
  }

  return AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
         robot_param->auto_ctrl_param.pole_extend_settle_ms;
}

/* 单目标撑杆模板：下发一次目标位并等待 hold_ms，适用于简化模板。 */
static bool AutoCtrlTemplate_RunSinglePoleTarget(auto_ctrl_t *ctrl,
                                                 uint32_t now_ms,
                                                 float front_target,
                                                 float rear_target,
                                                 uint32_t hold_ms) {
  switch (ctrl->template_ctx.step_index) {
    case 0:
      AutoCtrlPrimitive_CommandPoleTarget(ctrl, front_target, rear_target);
      if (AutoCtrlTemplate_StepTimeout(ctrl, now_ms, hold_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 1:
      return true;

    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}

/* 平地直行模板：固定速度走固定时长。 */
static bool AutoCtrlTemplate_RunFlatMove(auto_ctrl_t *ctrl, uint32_t now_ms) {
  const Config_RobotParam_t *robot_param = Config_GetRobotParam();

  switch (ctrl->template_ctx.step_index) {
    case 0:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, robot_param->auto_ctrl_param.flat_move_speed);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms, robot_param->auto_ctrl_param.flat_move_hold_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 1:
      return true;

    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}

/*
 * 通用 200 跨越模板（move_sign=+1 上台阶，-1 下台阶）。
 * step 时序：
 * 0 全杆撑起稳定 -> 1 前向小冲程 -> 2 前杆回收并校正
 * -> 3 中段前移 -> 4 后杆回收并校正 -> 5 尾段脱离 -> 6 完成。
 */
static bool AutoCtrlTemplate_RunCross200(auto_ctrl_t *ctrl,
                                         uint32_t now_ms,
                                         const Config_RobotParam_t *robot_param,
                                         float move_sign) {
  switch (ctrl->template_ctx.step_index) {
    case 0:
      AutoCtrlPrimitive_CommandPoleTarget(
          ctrl, robot_param->pole_param.preset.step_200_all_extend[0],
          robot_param->pole_param.preset.step_200_all_extend[1]);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms, robot_param->auto_ctrl_param.pole_extend_settle_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 1:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, move_sign * robot_param->auto_ctrl_param.climb_forward_kick_speed);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms,
              robot_param->auto_ctrl_param.climb_forward_kick_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 2:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithForward(
          ctrl, move_sign * robot_param->auto_ctrl_param.climb_forward_speed);
      AutoCtrlPrimitive_CommandPoleTarget(
          ctrl, robot_param->pole_param.preset.step_200_front_retract[0],
          robot_param->pole_param.preset.step_200_front_retract[1]);
      /* PE13 active-high is interpreted only in this cross-200 state machine. */
      if (ctrl->feedback.front_pole_retracted &&
          AutoCtrlTemplate_IsYawAligned(ctrl)) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (robot_param->auto_ctrl_param.front_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              robot_param->auto_ctrl_param.front_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 3:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, move_sign * robot_param->auto_ctrl_param.climb_forward_speed);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms,
              robot_param->auto_ctrl_param.climb_mid_forward_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 4:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithForward(
          ctrl, move_sign * robot_param->auto_ctrl_param.climb_forward_speed);
      AutoCtrlPrimitive_CommandPoleTarget(
          ctrl, robot_param->pole_param.preset.step_200_all_retract[0],
          robot_param->pole_param.preset.step_200_all_retract[1]);
      /* PE9 active-high is interpreted only in this cross-200 state machine. */
      if (ctrl->feedback.rear_pole_retracted &&
          AutoCtrlTemplate_IsYawAligned(ctrl)) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (robot_param->auto_ctrl_param.rear_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              robot_param->auto_ctrl_param.rear_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 5:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, move_sign * robot_param->auto_ctrl_param.climb_rear_retract_speed);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms, robot_param->auto_ctrl_param.rear_retract_move_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 6:
      return true;

    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}

/*
 * 专用 200 上台阶模板：按前/中/后段组织动作与切步条件。
 * step 时序：
 * 0 前段并行动作（撑杆+前进+对齐）
 * -> 1 前杆回收稳定
 * -> 2 中段慢速前移
 * -> 3 后段回收并叠加 vy
 * -> 4 尾段继续脱离
 * -> 5 完成。
 */
static bool AutoCtrlTemplate_RunAscend200(auto_ctrl_t *ctrl,
                                          uint32_t now_ms,
                                          const Config_RobotParam_t *robot_param) {
  switch (ctrl->template_ctx.step_index) {
    case 0:
      /*
       * autoctrltest 分支实验模式：
       * 本模板不主动下发实际命令，只保留状态检测与切步。
       * 前段切步条件：底部光电触发 + yaw 达标 + 稳定时间满足。
       */
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (AutoCtrlTemplate_IsAscend200Ready(ctrl, now_ms, robot_param)) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (robot_param->auto_ctrl_param.front_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              robot_param->auto_ctrl_param.front_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 1:
      /* 前杆阶段：仅按时间稳定窗口推进，不下发收杆命令。 */
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms,
              robot_param->auto_ctrl_param.front_retract_settle_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 2:
      /* 中段：仅检测后杆回收触发条件，不下发底盘命令。 */
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (ctrl->feedback.rear_pole_retracted) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (robot_param->auto_ctrl_param.rear_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              robot_param->auto_ctrl_param.rear_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 3:
      /* 后段：仅检测后杆回收状态，不下发矫正/移动/收杆命令。 */
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (ctrl->feedback.rear_pole_retracted) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (robot_param->auto_ctrl_param.rear_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              robot_param->auto_ctrl_param.rear_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 4:
      /* 尾段：仅以时间窗结束模板，不下发脱离动作命令。 */
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms,
              robot_param->auto_ctrl_param.rear_retract_move_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 5:
      return true;

    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}

/* 200 下台阶复用通用跨越模板，方向取反。 */
static bool AutoCtrlTemplate_RunDescend200(auto_ctrl_t *ctrl,
                                           uint32_t now_ms,
                                           const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_RunCross200(ctrl, now_ms, robot_param, -1.0f);
}

/* 标准 400 上台阶模板（当前实现为单次撑杆目标位动作）。 */
static bool AutoCtrlTemplate_RunAscend400Std(auto_ctrl_t *ctrl,
                                             uint32_t now_ms,
                                             const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_RunSinglePoleTarget(
      ctrl, now_ms, robot_param->pole_param.preset.step_400_all_extend[0],
      robot_param->pole_param.preset.step_400_all_extend[1], 1200u);
}

/* 标准 400 下台阶模板（当前实现与上台阶共用动作定义）。 */
static bool AutoCtrlTemplate_RunDescend400Std(auto_ctrl_t *ctrl,
                                              uint32_t now_ms,
                                              const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_RunSinglePoleTarget(
      ctrl, now_ms, robot_param->pole_param.preset.step_400_all_extend[0],
      robot_param->pole_param.preset.step_400_all_extend[1], 1200u);
}

/*
 * 模板调度入口：
 * - 首次进入记录 template_start_time_ms；
 * - 按 template_id 分发到具体模板执行函数；
 * - 未支持模板写故障并返回 false。
 *
 * 调用时序约束：
 * - 每个控制周期最多推进一个 step；
 * - step 切换依赖 AutoCtrlTemplate_NextStep；
 * - 失败通过 ctrl->fault 上报到上层主状态机。
 */
bool AutoCtrlTemplate_Run(auto_ctrl_t *ctrl, uint32_t now_ms) {
  const Config_RobotParam_t *robot_param;

  robot_param = Config_GetRobotParam();

  if (ctrl->template_ctx.template_start_time_ms == 0u) {
    ctrl->template_ctx.template_start_time_ms = now_ms;
  }

  switch (ctrl->template_id) {
    case AUTO_CTRL_TEMPLATE_FLAT_MOVE:
      return AutoCtrlTemplate_RunFlatMove(ctrl, now_ms);

    case AUTO_CTRL_TEMPLATE_ASCEND_200:
      return AutoCtrlTemplate_RunAscend200(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_DESCEND_200:
      return AutoCtrlTemplate_RunDescend200(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_ASCEND_400_STD:
      return AutoCtrlTemplate_RunAscend400Std(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_DESCEND_400_STD:
      return AutoCtrlTemplate_RunDescend400Std(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_NONE:
    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}