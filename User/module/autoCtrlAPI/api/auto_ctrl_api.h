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
  float yaw_auto_deg;  /* 输入 yaw（单位: 度），SetFeedback 内会扣零点并归一化。 */
  float sick_front_cm; /* 前向测距传感器读数，单位: cm。 */
  float sick_left_cm;  /* 左向测距传感器读数，单位: cm。 */
  float sick_right_cm; /* 右向测距传感器读数，单位: cm。 */
  float sick_rear_cm;  /* 后向测距传感器读数，单位: cm。 */
  bool bottom_photo_triggered; /* 底部光电门状态，true 表示已触发。 */
  bool front_pole_retracted; /* PE13 光电门状态，高电平表示前腿已收起。 */
  bool rear_pole_retracted;  /* PE9 光电门状态，高电平表示后腿已收起。 */
} auto_ctrl_feedback_t;

typedef struct {
  auto_ctrl_run_state_e state; /* 当前运行状态机所处状态。 */
  auto_ctrl_result_e result;   /* 本次自动控制的高层结果。 */
  auto_ctrl_fault_e fault;     /* 失败/中止时记录的故障原因。 */

  auto_ctrl_zone_e current_zone; /* AutoCtrl 当前认为所在的区块。 */
  auto_ctrl_zone_e target_zone;  /* 当前转移任务的目标区块。 */
  const auto_ctrl_transition_t *transition; /* 当前命中的转移表项指针。 */
  auto_ctrl_template_e template_id; /* 由转移表选择出的动作模板。 */

  float yaw_zero_offset_deg; /* 将 raw yaw 转为 auto yaw 使用的零点偏移。 */
  float target_yaw_deg;      /* 本次转移要求的目标朝向，单位: 度。 */
  float yaw_tolerance_deg;   /* PREALIGN 通过阈值，单位: 度。 */
  float yaw_error_deg;       /* 目标减当前的有符号朝向误差，单位: 度。 */

  uint32_t state_enter_time_ms; /* 进入当前 state 的时间戳，单位: ms。 */
  uint32_t prealign_timeout_ms; /* PREALIGN 阶段超时时间，单位: ms。 */
  uint32_t template_timeout_ms; /* RUN_TEMPLATE 阶段超时时间，单位: ms。 */

  auto_ctrl_template_ctx_t template_ctx; /* 模板内部 step 运行上下文。 */
  auto_ctrl_feedback_t feedback;         /* 外部最近一次写入的反馈快照。 */

  Chassis_CMD_t chassis_cmd; /* 当前 Update 周期生成的底盘指令。 */
  Pole_CMD_t pole_cmd;       /* 当前 Update 周期生成的撑杆指令。 */
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