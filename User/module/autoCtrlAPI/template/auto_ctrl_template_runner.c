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

static bool AutoCtrlTemplate_IsCurrentFrontPhotoTriggered(
    const auto_ctrl_t *ctrl) {
  if (ctrl->travel_dir == AUTO_CTRL_TRAVEL_DIR_TAIL_FORWARD) {
    return ctrl->feedback.tail_front_photo_triggered;
  }
  return ctrl->feedback.head_front_photo_triggered;
}

static bool AutoCtrlTemplate_IsCurrentRearPhotoTriggered(
    const auto_ctrl_t *ctrl) {
  if (ctrl->travel_dir == AUTO_CTRL_TRAVEL_DIR_TAIL_FORWARD) {
    return ctrl->feedback.tail_rear_photo_triggered;
  }
  return ctrl->feedback.head_rear_photo_triggered;
}

static bool AutoCtrlTemplate_LatchCurrentFrontPhotoTriggered(auto_ctrl_t *ctrl) {
  if (ctrl->travel_dir == AUTO_CTRL_TRAVEL_DIR_TAIL_FORWARD) {
    if (ctrl->feedback.tail_front_photo_triggered) {
      ctrl->template_ctx.tail_front_photo_triggered_latched = true;
    }
    return ctrl->template_ctx.tail_front_photo_triggered_latched;
  }

  if (ctrl->feedback.head_front_photo_triggered) {
    ctrl->template_ctx.head_front_photo_triggered_latched = true;
  }
  return ctrl->template_ctx.head_front_photo_triggered_latched;
}

static bool AutoCtrlTemplate_LatchCurrentRearPhotoTriggered(auto_ctrl_t *ctrl) {
  if (ctrl->travel_dir == AUTO_CTRL_TRAVEL_DIR_TAIL_FORWARD) {
    if (ctrl->feedback.tail_rear_photo_triggered) {
      ctrl->template_ctx.tail_rear_photo_triggered_latched = true;
    }
    return ctrl->template_ctx.tail_rear_photo_triggered_latched;
  }

  if (ctrl->feedback.head_rear_photo_triggered) {
    ctrl->template_ctx.head_rear_photo_triggered_latched = true;
  }
  return ctrl->template_ctx.head_rear_photo_triggered_latched;
}

static uint32_t AutoCtrlTemplate_GetCurrentFrontPhotoTimeoutMs(
    const auto_ctrl_t *ctrl, const Config_RobotParam_t *robot_param) {
  (void)ctrl;
  return robot_param->auto_ctrl_param.head_front_photo_timeout_ms;
}

static uint32_t AutoCtrlTemplate_GetCurrentRearPhotoTimeoutMs(
    const auto_ctrl_t *ctrl, const Config_RobotParam_t *robot_param) {
  (void)ctrl;
  return robot_param->auto_ctrl_param.head_rear_photo_timeout_ms;
}

/* 判断当前 yaw 误差是否已经满足容差。 */
static bool AutoCtrlTemplate_IsYawAligned(const auto_ctrl_t *ctrl) {
  return fabsf(ctrl->yaw_error_rad) <= ctrl->yaw_tolerance_rad;
}

/* Ascend200 首段切步条件：yaw 达标。 */
static bool AutoCtrlTemplate_IsAscend200PrealignReady(const auto_ctrl_t *ctrl) {
  return AutoCtrlTemplate_IsYawAligned(ctrl);
}

/* Ascend200 第二段切步条件：四杆伸出稳定时间到。 */
static bool AutoCtrlTemplate_IsAscend200PoleExtendReady(
    const auto_ctrl_t *ctrl, uint32_t now_ms,
    const Config_RobotParam_t *robot_param) {
  return AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
         robot_param->auto_ctrl_param.pole_extend_settle_ms;
}

static void AutoCtrlTemplate_CommandPoleByStage(auto_ctrl_t *ctrl,
                                                float front_target,
                                                float rear_target,
                                                float front_speed,
                                                float rear_speed) {
  AutoCtrlPrimitive_CommandPoleTargetWithSpeed(ctrl, front_target, rear_target,
                                               front_speed, rear_speed);
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
 * 专用 200 上台阶模板：按前/中/后段组织动作与切步条件。
 * step 时序：
 * 0 姿态矫正阶段
 * -> 1 对正成功后四杆伸起稳定
 * -> 2 仅当头向前光电触发时回收前杆
 * -> 3 中段慢速前移
 * -> 4 仅当头向后光电触发时回收后杆
 * -> 5 尾段继续脱离
 * -> 6 完成。
 */
static bool AutoCtrlTemplate_RunAscend200(auto_ctrl_t *ctrl,
                                          uint32_t now_ms,
                                          const Config_RobotParam_t *robot_param) {
  const uint32_t current_front_photo_timeout_ms =
    AutoCtrlTemplate_GetCurrentFrontPhotoTimeoutMs(ctrl, robot_param);

  switch (ctrl->template_ctx.step_index) {
    case 0:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, 0.0f, robot_param->auto_ctrl_param.climb_align_forward_speed);
        AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_200_all_retract[0],
          robot_param->pole_param.preset.step_200_all_retract[1],
          robot_param->auto_ctrl_param.pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.pole_rear_retract_lift_speed);
      if (AutoCtrlTemplate_IsAscend200PrealignReady(ctrl)) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (current_front_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              current_front_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 1:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, 0.0f,
          robot_param->auto_ctrl_param.climb_pole_extend_forward_speed);
        AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_200_all_extend[0],
          robot_param->pole_param.preset.step_200_all_extend[1],
          robot_param->auto_ctrl_param.pole_all_extend_lift_speed,
          robot_param->auto_ctrl_param.pole_all_extend_lift_speed);
      if (AutoCtrlTemplate_IsAscend200PoleExtendReady(ctrl, now_ms, robot_param)) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (current_front_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              current_front_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 2:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (!AutoCtrlTemplate_LatchCurrentFrontPhotoTriggered(ctrl)) {
        if (current_front_photo_timeout_ms > 0u &&
            AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
          current_front_photo_timeout_ms) {
          ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
        }
        AutoCtrlPrimitive_ApplyPrealignWithMove(
            ctrl, 0.0f, robot_param->auto_ctrl_param.climb_front_retract_vy);
        AutoCtrlTemplate_CommandPoleByStage(
            ctrl, robot_param->pole_param.preset.step_200_all_extend[0],
          robot_param->pole_param.preset.step_200_all_extend[1],
          robot_param->auto_ctrl_param.pole_front_extend_lift_speed,
          robot_param->auto_ctrl_param.pole_rear_extend_lift_speed);
        return false;
      }
      AutoCtrlPrimitive_ApplyPrealignWithMove(
            ctrl, 0.0f,
            robot_param->auto_ctrl_param.climb_front_retract_speed);
        AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_200_front_retract[0],
          robot_param->pole_param.preset.step_200_front_retract[1],
          robot_param->auto_ctrl_param.pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.pole_rear_extend_lift_speed);
          if (AutoCtrlTemplate_LatchCurrentFrontPhotoTriggered(ctrl) &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              robot_param->auto_ctrl_param.front_retract_settle_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (robot_param->auto_ctrl_param.climb_front_retract_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              robot_param->auto_ctrl_param.climb_front_retract_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 3:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, robot_param->auto_ctrl_param.climb_mid_forward_speed);
        AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_200_front_retract[0],
          robot_param->pole_param.preset.step_200_front_retract[1],
          robot_param->auto_ctrl_param.pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.pole_rear_extend_lift_speed);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms,
              robot_param->auto_ctrl_param.climb_mid_forward_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 4:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (!AutoCtrlTemplate_LatchCurrentRearPhotoTriggered(ctrl)) {
        if (robot_param->auto_ctrl_param.climb_rear_retract_timeout_ms > 0u &&
            AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                robot_param->auto_ctrl_param.climb_rear_retract_timeout_ms) {
          ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
        }
        AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, 0.0f, robot_param->auto_ctrl_param.climb_mid_forward_speed);
        AutoCtrlTemplate_CommandPoleByStage(
            ctrl, robot_param->pole_param.preset.step_200_front_retract[0],
          robot_param->pole_param.preset.step_200_front_retract[1],
          robot_param->auto_ctrl_param.pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.pole_rear_extend_lift_speed);
        return false;
      }
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, 0.0f, robot_param->auto_ctrl_param.climb_rear_retract_speed);
        AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_200_all_retract[0],
          robot_param->pole_param.preset.step_200_all_retract[1],
          robot_param->auto_ctrl_param.pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.pole_rear_retract_lift_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
          robot_param->auto_ctrl_param.rear_retract_move_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (robot_param->auto_ctrl_param.climb_rear_retract_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              robot_param->auto_ctrl_param.climb_rear_retract_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 5:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, robot_param->auto_ctrl_param.climb_mid_forward_speed);
      AutoCtrlTemplate_CommandPoleByStage(
        ctrl, robot_param->pole_param.preset.step_200_all_retract[0],
        robot_param->pole_param.preset.step_200_all_retract[1],
        robot_param->auto_ctrl_param.pole_front_retract_lift_speed,
        robot_param->auto_ctrl_param.pole_rear_retract_lift_speed);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms,
              robot_param->auto_ctrl_param.rear_retract_move_ms)) {
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

/* 200 下台阶模板：保留原通用跨越流程，但作为独立下台阶实现。 */
static bool AutoCtrlTemplate_RunDescend200(auto_ctrl_t *ctrl,
                                           uint32_t now_ms,
                                           const Config_RobotParam_t *robot_param) {
  const uint32_t current_front_photo_timeout_ms =
    AutoCtrlTemplate_GetCurrentFrontPhotoTimeoutMs(ctrl, robot_param);
  const uint32_t current_rear_photo_timeout_ms =
    AutoCtrlTemplate_GetCurrentRearPhotoTimeoutMs(ctrl, robot_param);

  switch (ctrl->template_ctx.step_index) {
    case 0:
      AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_200_all_extend[0],
          robot_param->pole_param.preset.step_200_all_extend[1],
          robot_param->auto_ctrl_param.pole_all_extend_lift_speed,
          robot_param->auto_ctrl_param.pole_all_extend_lift_speed);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms,
              robot_param->auto_ctrl_param.pole_extend_settle_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 1:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, -robot_param->auto_ctrl_param.climb_forward_kick_speed);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms,
              robot_param->auto_ctrl_param.climb_forward_kick_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 2:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithForward(
          ctrl, -robot_param->auto_ctrl_param.climb_forward_speed);
      AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_200_front_retract[0],
          robot_param->pole_param.preset.step_200_front_retract[1],
          robot_param->auto_ctrl_param.pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.pole_rear_extend_lift_speed);
        if (AutoCtrlTemplate_IsCurrentFrontPhotoTriggered(ctrl) &&
          AutoCtrlTemplate_IsYawAligned(ctrl)) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (current_front_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              current_front_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 3:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, -robot_param->auto_ctrl_param.climb_forward_speed);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms,
              robot_param->auto_ctrl_param.climb_mid_forward_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    case 4:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithForward(
          ctrl, -robot_param->auto_ctrl_param.climb_forward_speed);
      AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_200_all_retract[0],
          robot_param->pole_param.preset.step_200_all_retract[1],
          robot_param->auto_ctrl_param.pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.pole_rear_retract_lift_speed);
        if (AutoCtrlTemplate_IsCurrentRearPhotoTriggered(ctrl) &&
          AutoCtrlTemplate_IsYawAligned(ctrl)) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (current_rear_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              current_rear_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    case 5:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, -robot_param->auto_ctrl_param.climb_rear_retract_speed);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms,
              robot_param->auto_ctrl_param.rear_retract_move_ms)) {
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
 * 200 下台阶逆流程模板：ASCEND_200 的完整逆动作。
 *
 * ASCEND_200 流程回顾（step 顺序）：
 *   0: 姿态矫正阶段
 *   1: 四杆伸起稳定
 *   2: 等待头向前光电 -> 回收前杆
 *   3: 中段慢速前移
 *   4: 等待头向后光电 -> 回收后杆
 *   5: 尾段继续脱离
 *   6: 完成
 *
 * DESCEND_200_ALT 逆流程（step 顺序）：
 *   0: 姿态矫正（对正）
 *   1: 四杆全伸（撑住台阶边缘）
 *   2: 等待头向后光电 -> 回收后杆（逆 step 4）
 *   3: 跨越台阶中段（匀速前进）
 *   4: 等待头向前光电 -> 回收前杆（逆 step 2）
 *   5: 脱离台阶（匀速平移）
 *   6: 完成
 *
 * step 时序：
 * 0 姿态矫正阶段
 * -> 1 四杆伸起稳定
 * -> 2 仅当头向后光电触发时回收后杆
 * -> 3 中段匀速跨越
 * -> 4 仅当头向前光电触发时回收前杆
 * -> 5 脱离台阶匀速平移
 * -> 6 完成。
 */
static bool AutoCtrlTemplate_RunDescend200Alt(auto_ctrl_t *ctrl,
                                             uint32_t now_ms,
                                             const Config_RobotParam_t *robot_param) {
  const uint32_t current_front_photo_timeout_ms =
      AutoCtrlTemplate_GetCurrentFrontPhotoTimeoutMs(ctrl, robot_param);
  const uint32_t current_rear_photo_timeout_ms =
      AutoCtrlTemplate_GetCurrentRearPhotoTimeoutMs(ctrl, robot_param);

  switch (ctrl->template_ctx.step_index) {
    /* step 0: 姿态矫正（对正），同时保持四杆收回姿态。 */
    case 0:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, 0.0f, robot_param->auto_ctrl_param.descend_200_align_speed);
      AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_200_all_retract[0],
          robot_param->pole_param.preset.step_200_all_retract[1],
          robot_param->auto_ctrl_param.descend_200_pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.descend_200_pole_rear_retract_lift_speed);
      if (AutoCtrlTemplate_IsAscend200PrealignReady(ctrl)) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (current_front_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              current_front_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    /* step 1: 对正成功后，四杆全伸撑住台阶边缘，等待机构稳定。 */
    case 1:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, 0.0f, robot_param->auto_ctrl_param.descend_200_align_speed);
      AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_200_all_extend[0],
          robot_param->pole_param.preset.step_200_all_extend[1],
          robot_param->auto_ctrl_param.descend_200_pole_all_extend_lift_speed,
          robot_param->auto_ctrl_param.descend_200_pole_all_extend_lift_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
          robot_param->auto_ctrl_param.descend_200_pole_extend_settle_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (current_rear_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              current_rear_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    /* step 2: 跨越中段匀速前进，仅当头向后光电触发时开始回收后杆。 */
    case 2:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (!AutoCtrlTemplate_LatchCurrentRearPhotoTriggered(ctrl)) {
        AutoCtrlPrimitive_ApplyPrealignWithMove(
            ctrl, 0.0f, robot_param->auto_ctrl_param.descend_200_climb_forward_speed);
        AutoCtrlTemplate_CommandPoleByStage(
            ctrl, robot_param->pole_param.preset.step_200_all_extend[0],
            robot_param->pole_param.preset.step_200_all_extend[1],
            robot_param->auto_ctrl_param.descend_200_pole_all_extend_lift_speed,
            robot_param->auto_ctrl_param.descend_200_pole_all_extend_lift_speed);
        if (current_rear_photo_timeout_ms > 0u &&
            AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                current_rear_photo_timeout_ms) {
          ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
        }
        return false;
      }
      /* 头向后光电已触发，开始匀速前进并回收后杆。 */
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, 0.0f, robot_param->auto_ctrl_param.descend_200_climb_rear_retract_speed);
      AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_200_front_retract[0],
          robot_param->pole_param.preset.step_200_front_retract[1],
          robot_param->auto_ctrl_param.descend_200_pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.descend_200_pole_rear_retract_lift_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
          robot_param->auto_ctrl_param.descend_200_rear_retract_move_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (robot_param->auto_ctrl_param.descend_200_climb_rear_retract_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              robot_param->auto_ctrl_param.descend_200_climb_rear_retract_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    /* step 3: 中段匀速跨越台阶，继续前进。 */
    case 3:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, robot_param->auto_ctrl_param.descend_200_climb_forward_speed);
      AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_200_front_retract[0],
          robot_param->pole_param.preset.step_200_front_retract[1],
          robot_param->auto_ctrl_param.descend_200_pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.descend_200_pole_rear_retract_lift_speed);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms,
              robot_param->auto_ctrl_param.descend_200_rear_retract_move_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    /* step 4: 跨越完成后，仅当头向前光电触发时回收前杆。 */
    case 4:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (!AutoCtrlTemplate_LatchCurrentFrontPhotoTriggered(ctrl)) {
        AutoCtrlPrimitive_ApplyPrealignWithMove(
            ctrl, 0.0f, robot_param->auto_ctrl_param.descend_200_climb_forward_speed);
        AutoCtrlTemplate_CommandPoleByStage(
            ctrl, robot_param->pole_param.preset.step_200_front_retract[0],
            robot_param->pole_param.preset.step_200_front_retract[1],
            robot_param->auto_ctrl_param.descend_200_pole_front_retract_lift_speed,
            robot_param->auto_ctrl_param.descend_200_pole_rear_retract_lift_speed);
        if (current_front_photo_timeout_ms > 0u &&
            AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                current_front_photo_timeout_ms) {
          ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
        }
        return false;
      }
      /* 头向前光电已触发，开始匀速前进并回收前杆。 */
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, 0.0f, robot_param->auto_ctrl_param.descend_200_climb_front_retract_speed);
      AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_200_all_retract[0],
          robot_param->pole_param.preset.step_200_all_retract[1],
          robot_param->auto_ctrl_param.descend_200_pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.descend_200_pole_rear_retract_lift_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
          robot_param->auto_ctrl_param.descend_200_front_retract_settle_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (robot_param->auto_ctrl_param.descend_200_climb_front_retract_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              robot_param->auto_ctrl_param.descend_200_climb_front_retract_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    /* step 5: 脱离台阶，匀速平移一段时间。 */
    case 5:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, robot_param->auto_ctrl_param.descend_200_climb_forward_speed);
      AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_200_all_retract[0],
          robot_param->pole_param.preset.step_200_all_retract[1],
          robot_param->auto_ctrl_param.descend_200_pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.descend_200_pole_rear_retract_lift_speed);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms,
              robot_param->auto_ctrl_param.descend_200_flat_move_ms)) {
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
 * 400 下台阶逆流程模板：ASCEND_400_STD 的完整逆动作。
 * 与 DESCEND_200_ALT 流程相同，仅使用 400mm 台阶对应参数。
 *
 * step 时序：
 * 0 姿态矫正阶段（对正）
 * -> 1 四杆伸起稳定
 * -> 2 仅当头向后光电触发时回收后杆
 * -> 3 中段匀速跨越
 * -> 4 仅当头向前光电触发时回收前杆
 * -> 5 脱离台阶匀速平移
 * -> 6 完成。
 */
static bool AutoCtrlTemplate_RunDescend400Alt(auto_ctrl_t *ctrl,
                                             uint32_t now_ms,
                                             const Config_RobotParam_t *robot_param) {
  const uint32_t current_front_photo_timeout_ms =
      AutoCtrlTemplate_GetCurrentFrontPhotoTimeoutMs(ctrl, robot_param);
  const uint32_t current_rear_photo_timeout_ms =
      AutoCtrlTemplate_GetCurrentRearPhotoTimeoutMs(ctrl, robot_param);

  switch (ctrl->template_ctx.step_index) {
    /* step 0: 姿态矫正（对正），同时保持四杆收回姿态。 */
    case 0:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, 0.0f, robot_param->auto_ctrl_param.descend_400_align_speed);
      AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_400_all_retract[0],
          robot_param->pole_param.preset.step_400_all_retract[1],
          robot_param->auto_ctrl_param.descend_400_pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.descend_400_pole_rear_retract_lift_speed);
      if (AutoCtrlTemplate_IsAscend200PrealignReady(ctrl)) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (current_front_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              current_front_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    /* step 1: 对正成功后，四杆全伸撑住台阶边缘，等待机构稳定。 */
    case 1:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, 0.0f, robot_param->auto_ctrl_param.descend_400_align_speed);
      AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_400_all_extend[0],
          robot_param->pole_param.preset.step_400_all_extend[1],
          robot_param->auto_ctrl_param.descend_400_pole_all_extend_lift_speed,
          robot_param->auto_ctrl_param.descend_400_pole_all_extend_lift_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
          robot_param->auto_ctrl_param.descend_400_pole_extend_settle_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (current_rear_photo_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              current_rear_photo_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    /* step 2: 跨越中段匀速前进，仅当头向后光电触发时开始回收后杆。 */
    case 2:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (!AutoCtrlTemplate_LatchCurrentRearPhotoTriggered(ctrl)) {
        AutoCtrlPrimitive_ApplyPrealignWithMove(
            ctrl, 0.0f, robot_param->auto_ctrl_param.descend_400_climb_forward_speed);
        AutoCtrlTemplate_CommandPoleByStage(
            ctrl, robot_param->pole_param.preset.step_400_all_extend[0],
            robot_param->pole_param.preset.step_400_all_extend[1],
            robot_param->auto_ctrl_param.descend_400_pole_all_extend_lift_speed,
            robot_param->auto_ctrl_param.descend_400_pole_all_extend_lift_speed);
        if (current_rear_photo_timeout_ms > 0u &&
            AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                current_rear_photo_timeout_ms) {
          ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
        }
        return false;
      }
      /* 头向后光电已触发，开始匀速前进并回收后杆。 */
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, 0.0f, robot_param->auto_ctrl_param.descend_400_climb_rear_retract_speed);
      AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_400_front_retract[0],
          robot_param->pole_param.preset.step_400_front_retract[1],
          robot_param->auto_ctrl_param.descend_400_pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.descend_400_pole_rear_retract_lift_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
          robot_param->auto_ctrl_param.descend_400_rear_retract_move_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (robot_param->auto_ctrl_param.descend_400_climb_rear_retract_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              robot_param->auto_ctrl_param.descend_400_climb_rear_retract_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    /* step 3: 中段匀速跨越台阶，继续前进。 */
    case 3:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, robot_param->auto_ctrl_param.descend_400_climb_forward_speed);
      AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_400_front_retract[0],
          robot_param->pole_param.preset.step_400_front_retract[1],
          robot_param->auto_ctrl_param.descend_400_pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.descend_400_pole_rear_retract_lift_speed);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms,
              robot_param->auto_ctrl_param.descend_400_rear_retract_move_ms)) {
        AutoCtrlTemplate_NextStep(ctrl);
      }
      return false;

    /* step 4: 跨越完成后，仅当头向前光电触发时回收前杆。 */
    case 4:
      AutoCtrlTemplate_EnterStep(ctrl, now_ms);
      if (!AutoCtrlTemplate_LatchCurrentFrontPhotoTriggered(ctrl)) {
        AutoCtrlPrimitive_ApplyPrealignWithMove(
            ctrl, 0.0f, robot_param->auto_ctrl_param.descend_400_climb_forward_speed);
        AutoCtrlTemplate_CommandPoleByStage(
            ctrl, robot_param->pole_param.preset.step_400_front_retract[0],
            robot_param->pole_param.preset.step_400_front_retract[1],
            robot_param->auto_ctrl_param.descend_400_pole_front_retract_lift_speed,
            robot_param->auto_ctrl_param.descend_400_pole_rear_retract_lift_speed);
        if (current_front_photo_timeout_ms > 0u &&
            AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
                current_front_photo_timeout_ms) {
          ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
        }
        return false;
      }
      /* 头向前光电已触发，开始匀速前进并回收前杆。 */
      AutoCtrlPrimitive_ApplyPrealignWithMove(
          ctrl, 0.0f, robot_param->auto_ctrl_param.descend_400_climb_front_retract_speed);
      AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_400_all_retract[0],
          robot_param->pole_param.preset.step_400_all_retract[1],
          robot_param->auto_ctrl_param.descend_400_pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.descend_400_pole_rear_retract_lift_speed);
      if (AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
          robot_param->auto_ctrl_param.descend_400_front_retract_settle_ms) {
        AutoCtrlTemplate_NextStep(ctrl);
        return false;
      }
      if (robot_param->auto_ctrl_param.descend_400_climb_front_retract_timeout_ms > 0u &&
          AutoCtrlTemplate_StepElapsed(ctrl, now_ms) >=
              robot_param->auto_ctrl_param.descend_400_climb_front_retract_timeout_ms) {
        ctrl->fault = AUTO_CTRL_FAULT_SENSOR_INVALID;
      }
      return false;

    /* step 5: 脱离台阶，匀速平移一段时间。 */
    case 5:
      AutoCtrlPrimitive_CommandFlatMove(
          ctrl, robot_param->auto_ctrl_param.descend_400_climb_forward_speed);
      AutoCtrlTemplate_CommandPoleByStage(
          ctrl, robot_param->pole_param.preset.step_400_all_retract[0],
          robot_param->pole_param.preset.step_400_all_retract[1],
          robot_param->auto_ctrl_param.descend_400_pole_front_retract_lift_speed,
          robot_param->auto_ctrl_param.descend_400_pole_rear_retract_lift_speed);
      if (AutoCtrlTemplate_StepTimeout(
              ctrl, now_ms,
              robot_param->auto_ctrl_param.descend_400_flat_move_ms)) {
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

    case AUTO_CTRL_TEMPLATE_DESCEND_200_ALT:
      return AutoCtrlTemplate_RunDescend200Alt(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_ASCEND_400_STD:
      return AutoCtrlTemplate_RunAscend400Std(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_DESCEND_400_STD:
      return AutoCtrlTemplate_RunDescend400Std(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_DESCEND_400_ALT:
      return AutoCtrlTemplate_RunDescend400Alt(ctrl, now_ms, robot_param);

    case AUTO_CTRL_TEMPLATE_NONE:
    default:
      ctrl->fault = AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED;
      return false;
  }
}
