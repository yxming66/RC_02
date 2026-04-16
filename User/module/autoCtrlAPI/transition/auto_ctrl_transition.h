#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "module/autoCtrlAPI/core/auto_ctrl_def.h"

const auto_ctrl_transition_t *AutoCtrlTransition_Find(auto_ctrl_zone_e from,
                                                      auto_ctrl_zone_e to);
bool AutoCtrlTransition_IsLegal(auto_ctrl_zone_e from, auto_ctrl_zone_e to);

#ifdef __cplusplus
}
#endif