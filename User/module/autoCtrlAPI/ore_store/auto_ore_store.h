#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "module/arm_simple.h"
#include "module/autoCtrlAPI/api/auto_ctrl_api.h"
#include "module/chassis.h"
#include "module/ore_store.h"
#include "module/pole.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  AUTO_ORE_ACTION_NONE = 0,
  AUTO_ORE_ACTION_STORE,
  AUTO_ORE_ACTION_RELEASE,
  AUTO_ORE_ACTION_RELEASE_LIFT_DETECT,
  AUTO_ORE_ACTION_CHAMBER,
  AUTO_ORE_ACTION_PICK_POS_400,
  AUTO_ORE_ACTION_PICK_POS_200,
  AUTO_ORE_ACTION_PICK_NEG_200,
  AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_200_HEAD,
  AUTO_ORE_ACTION_STEP_PICK_STORE_DESCEND_200_HEAD,
  AUTO_ORE_ACTION_STEP_PICK_STORE_ASCEND_400_HEAD,
  AUTO_ORE_ACTION_PICK_STORE_POS_400,
  AUTO_ORE_ACTION_PICK_STORE_POS_200,
  AUTO_ORE_ACTION_PICK_STORE_NEG_200,
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
  AUTO_ORE_FAULT_SENSOR_INVALID,
} AutoOre_Fault_t;

typedef enum {
  AUTO_ORE_FAILURE_NONE = 0u,
  AUTO_ORE_FAILURE_SETUP = (1u << 0),
  AUTO_ORE_FAILURE_PICK_ORE = (1u << 1),
  AUTO_ORE_FAILURE_STORE_ORE = (1u << 2),
  AUTO_ORE_FAILURE_RELEASE_ORE = (1u << 3),
  AUTO_ORE_FAILURE_CHAMBER = (1u << 4),
  AUTO_ORE_FAILURE_STEP = (1u << 5),
  AUTO_ORE_FAILURE_ABORTED = (1u << 9),
} AutoOre_FailureMask_t;

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
  bool pole_front_at_target;
  bool pole_rear_at_target;
  bool arm_photo_has_ore;
  bool pe13_photo1_triggered;
  bool pe9_photo2_triggered;
  bool pa2_photo3_triggered;
  bool pa0_photo4_triggered;
  bool ore_store_fixed_ore_cylinder_closed;
  auto_ctrl_yaw_source_e yaw_source;
  float yaw_auto_rad;
  float yaw_rate_cmd_rad_s;
  float imu_accl_z_g;
  uint16_t release_lift_sick_adc_raw;
  bool release_lift_sick_valid;
  float arm_joint1_rad;
  float arm_joint2_rad;
  float pole_front_lift_rad;
  float pole_rear_lift_rad;
  float wheel_position_rad[4];
  float ore_store_platform_position_rad;
  float ore_store_platform_error_rad;
  AutoOre_Occupancy_t photoelectric_occupancy;
} AutoOre_Feedback_t;

typedef struct {
  uint32_t store_arm_settle_ms;
  uint32_t store_cylinder_close_ms;
  uint32_t store_arm_suction_off_ms;
  uint32_t store_cylinder_open_ms;
  uint32_t release_wait_ms;
  uint32_t release_lift_detect_timeout_ms;
  uint32_t release_lift_detect_settle_ms;
  uint32_t release_arm_settle_ms;
  uint32_t release_suction_off_ms;
  uint32_t chamber_low_clamp_ms;
  uint32_t chamber_arm_settle_ms;
  uint32_t chamber_cylinder_open_ms;
  uint32_t fetch_chassis_move_ms;
  uint32_t fetch_neg_200_chassis_move_ms;
  uint32_t fused_prealign_stable_ms;
  uint32_t fused_pick_precontact_timeout_ms;
  uint32_t fused_pick_lift_detect_ms;
  uint32_t fused_arm_photo_stable_ms;
  uint32_t fused_photo1_lift_delay_ms;
} AutoOre_TimingParam_t;

typedef struct {
  float joint1_max_vel_rad_s;
  float joint2_max_vel_rad_s;
} AutoOre_ArmSpeedLimit_t;

typedef struct {
  AutoOre_ArmSpeedLimit_t store_wait;
  AutoOre_ArmSpeedLimit_t store_place;
  AutoOre_ArmSpeedLimit_t store_standby;
  AutoOre_ArmSpeedLimit_t release_wait;
  AutoOre_ArmSpeedLimit_t release_assist;
  AutoOre_ArmSpeedLimit_t release_place;
  AutoOre_ArmSpeedLimit_t release_standby;
  AutoOre_ArmSpeedLimit_t chamber_wait;
  AutoOre_ArmSpeedLimit_t chamber_place;
  AutoOre_ArmSpeedLimit_t chamber_standby;
  AutoOre_ArmSpeedLimit_t pick_standby;
  AutoOre_ArmSpeedLimit_t pick_place;
  AutoOre_ArmSpeedLimit_t pick_fetch;
  AutoOre_ArmSpeedLimit_t pick_lift_detect;
} AutoOre_ArmSpeedParam_t;

typedef struct {
  auto_ctrl_template_e step_template;
  AutoOre_Action_t pick_action;
  float precontact_vx_mps;
  float precontact_wheel_delta_rad;
  uint32_t precontact_timeout_ms;
  float step_start_vx_mps;
  float step_start_wheel_delta_rad;
  bool override_descend_move;
  uint32_t descend_mid_move_ms;
  uint32_t descend_rear_retract_move_ms;
  float descend_rear_retract_move_wheel_delta_rad;
  bool fast_pick_on_front_photo;
  bool use_arm_photo_confirm;
} AutoOre_FusedParam_t;

typedef struct {
  const ArmSimple_Params_t *arm_param;
  const OreStore_Params_t *ore_store_param;
  const Pole_Params_t *pole_param;
  AutoOre_ArmSpeedParam_t arm_speed;
  AutoOre_TimingParam_t timing;
  uint32_t default_step_timeout_ms;
  float arm_arrive_threshold_rad;
  float ore_store_arrive_threshold_rad;
  float pole_arrive_threshold_rad;
  float prealign_yaw_tolerance_rad;
  uint8_t release_lift_detect_sick_index;
  uint16_t release_lift_detect_sick_adc_threshold;
  bool release_lift_detect_sick_greater_than_threshold;
  float fetch_chassis_vx_mps;
  float fetch_neg_200_chassis_vx_mps;
  float store_low_return_velocity_rad_s;
  float store_low_return_accel_rad_s2;
  float store_low_return_decel_rad_s2;
  AutoOre_FusedParam_t fused_step_pick_store_ascend_200_head;
  AutoOre_FusedParam_t fused_step_pick_store_descend_200_head;
  AutoOre_FusedParam_t fused_step_pick_store_ascend_400_head;
} AutoOre_Params_t;

typedef struct {
  AutoOre_State_t state;
  AutoOre_Result_t result;
  AutoOre_Fault_t fault;
  uint16_t failure_mask;
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
  bool chassis_cmd_valid;
  ArmSimple_CMD_t arm_cmd;
  OreStore_CMD_t ore_store_cmd;
  Pole_CMD_t pole_cmd;
  Chassis_CMD_t chassis_cmd;
  auto_ctrl_t step_ctrl;
  bool step_ctrl_active;
  bool step_ctrl_started;
  bool fused_step_done;
  bool fused_store_done;
  bool pick_lift_confirmed;
  bool prealign_yaw_target_valid;
  bool distance_latch_valid;
  float prealign_target_yaw_rad;
  float prealign_yaw_error_rad;
  float distance_start_wheel_rad[4];
  float wheel_delta_rad;
  float target_wheel_delta_rad;
  uint32_t fused_arm_photo_since_ms;
  bool fused_photo1_stable_trigger_seen;
  bool fused_photo1_stable_release_latched;
  uint32_t fused_photo1_triggered_since_ms;
  uint32_t fused_photo1_released_since_ms;
  uint8_t fused_store_step_index;
  uint8_t fused_store_step_phase;
  uint8_t fused_step_template_start_step_index;
  AutoOre_Position_t fused_store_position;
  bool release_lift_observer_active;
  bool release_lift_detected;
  uint32_t release_lift_observer_start_ms;
  uint32_t release_lift_observer_last_ms;
  uint32_t release_lift_detect_time_ms;
  uint8_t release_lift_sick_index;
  uint16_t release_lift_sick_adc_raw;
  uint16_t release_lift_sick_adc_threshold;
  bool release_lift_sick_valid;
  AutoOre_Params_t param;
  AutoOre_Feedback_t feedback;
} AutoOre_t;

void AutoOre_Init(AutoOre_t *ctrl, const AutoOre_Params_t *param,
                  const AutoOre_Occupancy_t *initial_occupancy);
AutoOre_Occupancy_t AutoOre_GetOccupancy(const AutoOre_t *ctrl);
uint8_t AutoOre_GetOccupancyMask(const AutoOre_t *ctrl);
bool AutoOre_StartStore(AutoOre_t *ctrl, uint32_t now_ms);
bool AutoOre_StartStoreAtPosition(AutoOre_t *ctrl,
                                  AutoOre_Position_t position,
                                  uint32_t now_ms);
bool AutoOre_StartRelease(AutoOre_t *ctrl, uint32_t now_ms);
bool AutoOre_StartReleaseLiftDetect(AutoOre_t *ctrl, uint32_t now_ms);
bool AutoOre_StartChamber(AutoOre_t *ctrl, uint32_t now_ms);
bool AutoOre_StartPickPos400(AutoOre_t *ctrl, uint32_t now_ms);
bool AutoOre_StartPickPos200(AutoOre_t *ctrl, uint32_t now_ms);
bool AutoOre_StartPickNeg200(AutoOre_t *ctrl, uint32_t now_ms);
bool AutoOre_StartStepPickStoreAscend200Head(AutoOre_t *ctrl, uint32_t now_ms);
bool AutoOre_StartStepPickStoreDescend200Head(AutoOre_t *ctrl, uint32_t now_ms);
bool AutoOre_StartStepPickStoreAscend400Head(AutoOre_t *ctrl, uint32_t now_ms);
bool AutoOre_StartPickStorePos400(AutoOre_t *ctrl, uint32_t now_ms);
bool AutoOre_StartPickStorePos200(AutoOre_t *ctrl, uint32_t now_ms);
bool AutoOre_StartPickStoreNeg200(AutoOre_t *ctrl, uint32_t now_ms);
void AutoOre_Update(AutoOre_t *ctrl, const AutoOre_Feedback_t *feedback,
                    uint32_t now_ms);
void AutoOre_Abort(AutoOre_t *ctrl);
bool AutoOre_IsBusy(const AutoOre_t *ctrl);
AutoOre_State_t AutoOre_GetState(const AutoOre_t *ctrl);
AutoOre_Result_t AutoOre_GetResult(const AutoOre_t *ctrl);
AutoOre_Fault_t AutoOre_GetFault(const AutoOre_t *ctrl);
uint16_t AutoOre_GetFailureMask(const AutoOre_t *ctrl);
AutoOre_Position_t AutoOre_GetActivePosition(const AutoOre_t *ctrl);
uint8_t AutoOre_GetStepIndex(const AutoOre_t *ctrl);
const ArmSimple_CMD_t *AutoOre_GetArmCommand(const AutoOre_t *ctrl);
const OreStore_CMD_t *AutoOre_GetOreStoreCommand(const AutoOre_t *ctrl);
const Pole_CMD_t *AutoOre_GetPoleCommand(const AutoOre_t *ctrl);
const Chassis_CMD_t *AutoOre_GetChassisCommand(const AutoOre_t *ctrl);

#ifdef __cplusplus
}
#endif
