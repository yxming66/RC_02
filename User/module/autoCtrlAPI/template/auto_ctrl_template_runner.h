#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "module/autoCtrlAPI/api/auto_ctrl_api.h"

bool AutoCtrlTemplate_Run(auto_ctrl_t *ctrl, uint32_t now_ms);

#ifdef __cplusplus
}
#endif