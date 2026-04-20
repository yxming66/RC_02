#pragma once

/**
 * @file auto_ctrl_template_runner.h
 * @brief AutoCtrl 模板执行器入口。
 *
 * 模板执行器根据 ctrl->template_id 调度对应模板状态机，
 * 持续产出底盘与撑杆命令，直到模板返回完成或上报故障。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "module/autoCtrlAPI/api/auto_ctrl_api.h"

/* 执行当前模板一个周期；返回 true 表示模板完成。 */
bool AutoCtrlTemplate_Run(auto_ctrl_t *ctrl, uint32_t now_ms);

#ifdef __cplusplus
}
#endif