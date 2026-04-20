#pragma once

/**
 * @file auto_ctrl_transition.h
 * @brief AutoCtrl 区块转移查询接口。
 *
 * 本模块维护 from->to 的合法转移表，
 * 每条表项包含动作模板、目标 yaw、yaw 容差、传感器模式和高度差。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "module/autoCtrlAPI/core/auto_ctrl_def.h"

/* 查找 from->to 的转移定义；找不到返回 0。 */
const auto_ctrl_transition_t *AutoCtrlTransition_Find(auto_ctrl_zone_e from,
                                                      auto_ctrl_zone_e to);

/* 判断 from->to 是否存在合法转移。 */
bool AutoCtrlTransition_IsLegal(auto_ctrl_zone_e from, auto_ctrl_zone_e to);

#ifdef __cplusplus
}
#endif