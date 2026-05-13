#include "module/autoCtrlAPI/template/auto_ctrl_config.h"

static AutoCtrl_Params_t *AutoCtrlConfig_Params(void) {
  Config_RobotParam_t *robot_param = Config_GetRobotParam();
  return (robot_param == 0) ? 0 : &robot_param->auto_ctrl_param;
}

AutoCtrl_Params_t *AutoCtrlConfig_GetParams(void) {
  return AutoCtrlConfig_Params();
}

AutoCtrl_CommonParam_t *AutoCtrlConfig_GetCommonParams(void) {
  AutoCtrl_Params_t *params = AutoCtrlConfig_Params();
  return (params == 0) ? 0 : &params->common;
}

AutoCtrl_TemplateParam_t *AutoCtrlConfig_GetHeadAscend200Params(void) {
  AutoCtrl_Params_t *params = AutoCtrlConfig_Params();
  return (params == 0) ? 0 : &params->head_ascend_200;
}

AutoCtrl_TemplateParam_t *AutoCtrlConfig_GetHeadAscend400Params(void) {
  AutoCtrl_Params_t *params = AutoCtrlConfig_Params();
  return (params == 0) ? 0 : &params->head_ascend_400;
}

AutoCtrl_TemplateParam_t *AutoCtrlConfig_GetHeadDescend200Params(void) {
  AutoCtrl_Params_t *params = AutoCtrlConfig_Params();
  return (params == 0) ? 0 : &params->head_descend_200;
}

AutoCtrl_TemplateParam_t *AutoCtrlConfig_GetHeadDescend400Params(void) {
  AutoCtrl_Params_t *params = AutoCtrlConfig_Params();
  return (params == 0) ? 0 : &params->head_descend_400;
}

AutoCtrl_TemplateParam_t *AutoCtrlConfig_GetTailAscend200Params(void) {
  AutoCtrl_Params_t *params = AutoCtrlConfig_Params();
  return (params == 0) ? 0 : &params->tail_ascend_200;
}

AutoCtrl_TemplateParam_t *AutoCtrlConfig_GetTailAscend400Params(void) {
  AutoCtrl_Params_t *params = AutoCtrlConfig_Params();
  return (params == 0) ? 0 : &params->tail_ascend_400;
}

AutoCtrl_TemplateParam_t *AutoCtrlConfig_GetTailDescend200Params(void) {
  AutoCtrl_Params_t *params = AutoCtrlConfig_Params();
  return (params == 0) ? 0 : &params->tail_descend_200;
}

AutoCtrl_TemplateParam_t *AutoCtrlConfig_GetTailDescend400Params(void) {
  AutoCtrl_Params_t *params = AutoCtrlConfig_Params();
  return (params == 0) ? 0 : &params->tail_descend_400;
}
