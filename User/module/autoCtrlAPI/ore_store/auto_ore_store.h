#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "module/arm_simple.h"
#include "module/ore_store.h"
#include "module/pole.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  AUTO_ORE_ACTION_NONE = 0,
  AUTO_ORE_ACTION_STORE,
  AUTO_ORE_ACTION_RELEASE,
  AUTO_ORE_ACTION_CHAMBER,
} AutoOre_Action_t;

typedef enum {
  AUTO_ORE_STATE_IDLE = 0,
  AUTO_ORE_STATE_RUNNING,
  AUTO_ORE_STATE_SUCCESS,
  AUTO_ORE_STATE_FAIL,
  AUTO_ORE_STATE_ABORT,
} AutoOre_State_t;

typedef enum {
  AUTO_ORE_RESULT_NONE = 0,
  AUTO_ORE_RESULT_RUNNING,
  AUTO_ORE_RESULT_SUCCESS,
  AUTO_ORE_RESULT_FAIL,
  AUTO_ORE_RESULT_ABORTED,
} AutoOre_Result_t;

typedef enum {
  AUTO_ORE_FAULT_NONE = 0,
  AUTO_ORE_FAULT_INVALID_OCCUPANCY,
  AUTO_ORE_FAULT_INVALID_PARAM,
  AUTO_ORE_FAULT_NOT_HOMED,
  AUTO_ORE_FAULT_TIMEOUT,
  AUTO_ORE_FAULT_ABORTED,
} AutoOre_Fault_t;

typedef enum {
  AUTO_ORE_POSITION_NONE = 0,
  AUTO_ORE_POSITION_TRANSFORM_LOW,
  AUTO_ORE_POSITION_TRANSFORM_HIGH,
  AUTO_ORE_POSITION_ARM,
} AutoOre_Position_t;

typedef enum {
  AUTO_ORE_OCCUPANCY_SOURCE_STATE_MACHINE = 0,
  AUTO_ORE_OCCUPANCY_SOURCE_PHOTOELECTRIC,
} AutoOre_OccupancySource_t;

typedef struct {
  bool transform_low_has_ore;
  bool transform_high_has_ore;
  bool arm_has_ore;
} AutoOre_Occupancy_t;

typedef struct {
  bool arm_at_target;
  bool ore_store_all_homed;
  bool ore_store_all_at_target;
  bool pole_all_at_target;
  AutoOre_Occupancy_t photoelectric_occupancy;
} AutoOre_Feedback_t;

typedef struct {
  const ArmSimple_Params_t *arm_param;
  const OreStore_Params_t *ore_store_param;
  const Pole_Params_t *pole_param;
  uint32_t default_step_timeout_ms;
  float arm_arrive_threshold_rad;
  float ore_store_arrive_threshold_rad;
  float pole_arrive_threshold_rad;
} AutoOre_Params_t;

typedef struct {
  AutoOre_State_t state;
  AutoOre_Result_t result;
  AutoOre_Fault_t fault;
  AutoOre_Action_t action;
  AutoOre_Position_t active_position;
  AutoOre_Occupancy_t occupancy;
  uint8_t step_index;
  uint8_t step_phase;
  uint32_t step_enter_time_ms;
  uint32_t step_condition_time_ms;
  bool step_entered;
  bool step_condition_met;
  bool arm_cmd_valid;
  bool ore_store_cmd_valid;
  bool pole_cmd_valid;
  ArmSimple_CMD_t arm_cmd;
  OreStore_CMD_t ore_store_cmd;
  Pole_CMD_t pole_cmd;
  AutoOre_Params_t param;
  AutoOre_Feedback_t feedback;
} AutoOre_t;

void AutoOre_Init(AutoOre_t *ctrl, const AutoOre_Params_t *param,
                  const AutoOre_Occupancy_t *initial_occupancy);
AutoOre_Occupancy_t AutoOre_GetOccupancy(const AutoOre_t *ctrl);
uint8_t AutoOre_GetOccupancyMask(const AutoOre_t *ctrl);
bool AutoOre_StartStore(AutoOre_t *ctrl, uint32_t now_ms);
bool AutoOre_StartRelease(AutoOre_t *ctrl, uint32_t now_ms);
bool AutoOre_StartChamber(AutoOre_t *ctrl, uint32_t now_ms);
void AutoOre_Update(AutoOre_t *ctrl, const AutoOre_Feedback_t *feedback,
                    uint32_t now_ms);
void AutoOre_Abort(AutoOre_t *ctrl);
bool AutoOre_IsBusy(const AutoOre_t *ctrl);
AutoOre_State_t AutoOre_GetState(const AutoOre_t *ctrl);
AutoOre_Result_t AutoOre_GetResult(const AutoOre_t *ctrl);
AutoOre_Fault_t AutoOre_GetFault(const AutoOre_t *ctrl);
AutoOre_Position_t AutoOre_GetActivePosition(const AutoOre_t *ctrl);
uint8_t AutoOre_GetStepIndex(const AutoOre_t *ctrl);
const ArmSimple_CMD_t *AutoOre_GetArmCommand(const AutoOre_t *ctrl);
const OreStore_CMD_t *AutoOre_GetOreStoreCommand(const AutoOre_t *ctrl);
const Pole_CMD_t *AutoOre_GetPoleCommand(const AutoOre_t *ctrl);

#ifdef __cplusplus
}
#endif
