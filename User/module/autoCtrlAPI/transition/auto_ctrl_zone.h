#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "module/autoCtrlAPI/core/auto_ctrl_def.h"

const auto_ctrl_zone_info_t *AutoCtrlZone_GetInfo(auto_ctrl_zone_e zone);
int16_t AutoCtrlZone_GetHeightMm(auto_ctrl_zone_e zone);
bool AutoCtrlZone_IsPlatform(auto_ctrl_zone_e zone);
bool AutoCtrlZone_IsValid(auto_ctrl_zone_e zone);

#ifdef __cplusplus
}
#endif