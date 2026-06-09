#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "module/chassis.h"
#include "module/pole.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef AUTO_SICK_CORRECT_SENSOR_COUNT
#define AUTO_SICK_CORRECT_SENSOR_COUNT (4u)
#endif

typedef enum {
  AUTO_SICK_CORRECT_ACTION_NONE = 0,
  AUTO_SICK_CORRECT_ACTION_ROD_SPEARHEAD,
  AUTO_SICK_CORRECT_ACTION_ORE_RELEASE,
} AutoSickCorrect_Action_t;

typedef enum {
  AUTO_SICK_CORRECT_STATE_IDLE = 0,
  AUTO_SICK_CORRECT_STATE_RUNNING,
  AUTO_SICK_CORRECT_STATE_SUCCESS,
  AUTO_SICK_CORRECT_STATE_FAIL,
  AUTO_SICK_CORRECT_STATE_ABORT,
} AutoSickCorrect_State_t;

typedef enum {
  AUTO_SICK_CORRECT_RESULT_NONE = 0,
  AUTO_SICK_CORRECT_RESULT_RUNNING,
  AUTO_SICK_CORRECT_RESULT_SUCCESS,
  AUTO_SICK_CORRECT_RESULT_FAIL,
  AUTO_SICK_CORRECT_RESULT_ABORTED,
} AutoSickCorrect_Result_t;

typedef enum {
  AUTO_SICK_CORRECT_FAULT_NONE = 0,
  AUTO_SICK_CORRECT_FAULT_TIMEOUT,
  AUTO_SICK_CORRECT_FAULT_INVALID_PARAM,
  AUTO_SICK_CORRECT_FAULT_SENSOR_INVALID,
  AUTO_SICK_CORRECT_FAULT_UNSUPPORTED,
  AUTO_SICK_CORRECT_FAULT_ABORTED,
} AutoSickCorrect_Fault_t;

typedef struct {
  uint8_t left_index;
  uint8_t front_left_index;
  uint8_t front_right_index;
  uint8_t right_index;
  uint16_t valid_adc_min;
  uint16_t valid_adc_max;
  float x_target_adc;           /* x correction standard ADC. */
  float y_left_target_adc;      /* y correction standard ADC when left side is nearer. */
  float y_right_target_adc;     /* y correction standard ADC when right side is nearer. */
  float yaw_target_diff_adc;    /* z/yaw target: front-left - front-right ADC. */
  float x_tolerance_adc;
  float y_tolerance_adc;
  float yaw_tolerance_adc;
  float x_kp_mps_per_adc;
  float y_kp_mps_per_adc;
  float yaw_kp_rad_s_per_adc;   /* z/yaw correction gain to chassis wz. */
  float vx_limit_mps;
  float vy_limit_mps;
  float wz_limit_rad_s;
  float pole_target_lift;
  float pole_speed;
  uint32_t finish_stable_ms;
  uint32_t timeout_ms;
} AutoSickCorrect_PointParams_t;

typedef struct {
  AutoSickCorrect_PointParams_t rod_spearhead;
  AutoSickCorrect_PointParams_t ore_release;
} AutoSickCorrect_Params_t;

typedef struct {
  uint16_t adc_raw[AUTO_SICK_CORRECT_SENSOR_COUNT];
  bool valid[AUTO_SICK_CORRECT_SENSOR_COUNT];
  bool pole_all_at_target;
} AutoSickCorrect_Feedback_t;

typedef struct {
  AutoSickCorrect_State_t state;
  AutoSickCorrect_Result_t result;
  AutoSickCorrect_Fault_t fault;
  AutoSickCorrect_Action_t action;
  uint8_t step_index;
  uint32_t start_time_ms;
  uint32_t stable_since_ms;
  bool chassis_cmd_valid;
  bool pole_cmd_valid;
  float x_sample_adc;
  float y_sample_adc;
  float yaw_sample_diff_adc;
  float y_target_adc;
  float x_error_adc;
  float y_error_adc;
  float yaw_error_adc;
  uint8_t y_sample_index;
  Chassis_CMD_t chassis_cmd;
  Pole_CMD_t pole_cmd;
  AutoSickCorrect_Params_t param;
} AutoSickCorrect_t;

void AutoSickCorrect_Init(AutoSickCorrect_t *ctrl,
                          const AutoSickCorrect_Params_t *param);
bool AutoSickCorrect_StartRodSpearhead(AutoSickCorrect_t *ctrl,
                                       uint32_t now_ms);
bool AutoSickCorrect_StartOreRelease(AutoSickCorrect_t *ctrl,
                                     uint32_t now_ms);
void AutoSickCorrect_Update(AutoSickCorrect_t *ctrl,
                            const AutoSickCorrect_Feedback_t *feedback,
                            uint32_t now_ms);
void AutoSickCorrect_Abort(AutoSickCorrect_t *ctrl);
bool AutoSickCorrect_IsBusy(const AutoSickCorrect_t *ctrl);
AutoSickCorrect_State_t AutoSickCorrect_GetState(
    const AutoSickCorrect_t *ctrl);
AutoSickCorrect_Result_t AutoSickCorrect_GetResult(
    const AutoSickCorrect_t *ctrl);
AutoSickCorrect_Fault_t AutoSickCorrect_GetFault(
    const AutoSickCorrect_t *ctrl);
AutoSickCorrect_Action_t AutoSickCorrect_GetAction(
    const AutoSickCorrect_t *ctrl);
uint8_t AutoSickCorrect_GetStepIndex(const AutoSickCorrect_t *ctrl);
const Chassis_CMD_t *AutoSickCorrect_GetChassisCommand(
    const AutoSickCorrect_t *ctrl);
const Pole_CMD_t *AutoSickCorrect_GetPoleCommand(
    const AutoSickCorrect_t *ctrl);

#ifdef __cplusplus
}
#endif
