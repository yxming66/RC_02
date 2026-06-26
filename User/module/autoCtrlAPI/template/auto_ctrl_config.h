#pragma once

/*
 * AutoCtrl parameter access helpers.
 *
 * The source of truth is Config_RobotParam_t::auto_ctrl_param. This header only
 * exposes named accessors for code that wants a specific parameter set.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "module/config.h"

AutoCtrl_Params_t *AutoCtrlConfig_GetParams(void);
AutoCtrl_CommonParam_t *AutoCtrlConfig_GetCommonParams(void);

AutoCtrl_TemplateParam_t *AutoCtrlConfig_GetHeadAscend200Params(void);
AutoCtrl_TemplateParam_t *AutoCtrlConfig_GetHeadAscend400Params(void);
AutoCtrl_TemplateParam_t *AutoCtrlConfig_GetHeadDescend200Params(void);
AutoCtrl_TemplateParam_t *AutoCtrlConfig_GetHeadDescend400Params(void);

#ifdef __cplusplus
}
#endif
