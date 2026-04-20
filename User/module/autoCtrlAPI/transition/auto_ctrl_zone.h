#pragma once

/**
 * @file auto_ctrl_zone.h
 * @brief AutoCtrl 区块属性查询接口。
 *
 * 本模块把区块编号映射到静态属性（名称/高度/是否平台），
 * 供转移合法性判断、高度差计算和调试打印复用。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "module/autoCtrlAPI/core/auto_ctrl_def.h"

/* 返回区块信息指针。非法区块统一回退到 INVALID 条目。 */
const auto_ctrl_zone_info_t *AutoCtrlZone_GetInfo(auto_ctrl_zone_e zone);

/* 返回区块高度（mm）。 */
int16_t AutoCtrlZone_GetHeightMm(auto_ctrl_zone_e zone);

/* 判断区块是否属于平台类。 */
bool AutoCtrlZone_IsPlatform(auto_ctrl_zone_e zone);

/* 判断区块枚举是否有效。 */
bool AutoCtrlZone_IsValid(auto_ctrl_zone_e zone);

#ifdef __cplusplus
}
#endif