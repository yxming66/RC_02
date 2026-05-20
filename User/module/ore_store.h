#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "component/pid.h"
#include "device/motor.h"
#include "device/motor_rm.h"

#define ORE_STORE_OK (0)
#define ORE_STORE_ERR (-1)
#define ORE_STORE_ERR_NULL (-2)

#define ORE_STORE_GATE_NUM (2u)
#define ORE_STORE_TRACK_NUM (2u)
#define ORE_STORE_AXIS_NUM (5u)

typedef enum {
  ORE_STORE_AXIS_PLATFORM = 0,
  ORE_STORE_AXIS_GATE_LEFT,
  ORE_STORE_AXIS_GATE_RIGHT,
  ORE_STORE_AXIS_TRACK_LEFT,
  ORE_STORE_AXIS_TRACK_RIGHT,
} OreStore_Axis_t;

typedef enum {
  ORE_STORE_MODE_RELAX = 0,
  ORE_STORE_MODE_HOME,
  ORE_STORE_MODE_ACTIVE,
} OreStore_Mode_t;

typedef struct {
  float stall_velocity_threshold_rad_s;
  float stall_position_window_rad;
  uint16_t stall_cycles_required;
  float seek_timeout_s;
  float online_wait_timeout_s;
  float limit_margin_rad;
  float learned_limit_margin_rad;
  float min_range_rad;
} OreStore_SoftLimitConfig_t;

typedef struct {
  MOTOR_RM_Param_t motor_param[ORE_STORE_AXIS_NUM];
  MOTOR_TemperatureProtectionConfig_t motor_temperature_protection;

  struct {
    float external_ratio;
    bool reverse_output;
  } motor_install[ORE_STORE_AXIS_NUM];

  struct {
    KPID_Params_t position_pid[ORE_STORE_AXIS_NUM];
    KPID_Params_t velocity_pid[ORE_STORE_AXIS_NUM];
  } pid;

  struct {
    float sample_freq;
    float position_to_velocity_limit[ORE_STORE_AXIS_NUM];
    float velocity_to_torque_limit[ORE_STORE_AXIS_NUM];
    float feedback_lowpass_cutoff_hz[ORE_STORE_AXIS_NUM];
    float output_lowpass_cutoff_hz[ORE_STORE_AXIS_NUM];
    float normalized_output_limit[ORE_STORE_AXIS_NUM];
    float normalized_to_torque_nm[ORE_STORE_AXIS_NUM];
  } controller;

  struct {
    OreStore_SoftLimitConfig_t config[ORE_STORE_AXIS_NUM];
    float travel_rad[ORE_STORE_AXIS_NUM];
    float lower_seek_velocity_rad_s[ORE_STORE_AXIS_NUM];
    float move_velocity_rad_s[ORE_STORE_AXIS_NUM];
    float arrive_threshold_rad[ORE_STORE_AXIS_NUM];
  } limit;
} OreStore_Params_t;

typedef struct {
  OreStore_Mode_t mode;
  bool force_rehome;
  float platform_target_rad;
  float gate_target_rad[ORE_STORE_GATE_NUM];
  float track_target_rad[ORE_STORE_TRACK_NUM];
} OreStore_CMD_t;

typedef struct {
  MOTOR_Feedback_t motor[ORE_STORE_AXIS_NUM];
  float raw_position_rad[ORE_STORE_AXIS_NUM];
  float position_rad[ORE_STORE_AXIS_NUM];
  float velocity_rad_s[ORE_STORE_AXIS_NUM];
  bool temperature_warning[ORE_STORE_AXIS_NUM];
  bool temperature_over_limit[ORE_STORE_AXIS_NUM];
  bool online[ORE_STORE_AXIS_NUM];
  bool homed[ORE_STORE_AXIS_NUM];
  bool all_homed;
} OreStore_Feedback_t;

typedef struct {
  float target_position_rad[ORE_STORE_AXIS_NUM];
  float command_position_rad[ORE_STORE_AXIS_NUM];
  float zero_offset_rad[ORE_STORE_AXIS_NUM];
  float learned_lower_raw_rad[ORE_STORE_AXIS_NUM];
  float travel_rad[ORE_STORE_AXIS_NUM];
  float seek_velocity_rad_s[ORE_STORE_AXIS_NUM];
  float online_wait_s[ORE_STORE_AXIS_NUM];
  uint8_t soft_limit_state[ORE_STORE_AXIS_NUM];
  uint16_t stall_cycles[ORE_STORE_AXIS_NUM];
  bool homing_started[ORE_STORE_AXIS_NUM];
  bool axis_failed[ORE_STORE_AXIS_NUM];
  int8_t controller_update_ret[ORE_STORE_AXIS_NUM];
  int8_t set_command_ret[ORE_STORE_AXIS_NUM];
  int8_t commit_ret[ORE_STORE_AXIS_NUM];
  bool command_pending[ORE_STORE_AXIS_NUM];
  float filtered_position_rad[ORE_STORE_AXIS_NUM];
  float filtered_velocity_rad_s[ORE_STORE_AXIS_NUM];
  float filtered_output_torque_nm[ORE_STORE_AXIS_NUM];
  float motor_torque_nm[ORE_STORE_AXIS_NUM];
  float normalized_output[ORE_STORE_AXIS_NUM];
  float rm_last_set_torque_nm[ORE_STORE_AXIS_NUM];
  float rm_pending_current_a[ORE_STORE_AXIS_NUM];
  int16_t rm_output_raw[ORE_STORE_AXIS_NUM];
  uint16_t rm_tx_frame_id[ORE_STORE_AXIS_NUM];
  float dt_s;
} OreStore_Debug_t;

typedef struct {
  uint32_t last_wakeup;
  float dt;

  const OreStore_Params_t *param;
  OreStore_Mode_t mode;
  bool rehome_latched;

  void *motor[ORE_STORE_AXIS_NUM];
  void *controller[ORE_STORE_AXIS_NUM];
  void *soft_limit[ORE_STORE_AXIS_NUM];

  bool axis_homed[ORE_STORE_AXIS_NUM];
  bool homing_started[ORE_STORE_AXIS_NUM];
  bool axis_failed[ORE_STORE_AXIS_NUM];
  float zero_offset_rad[ORE_STORE_AXIS_NUM];
  float learned_lower_raw_rad[ORE_STORE_AXIS_NUM];
  float homing_online_wait_s[ORE_STORE_AXIS_NUM];
  float target_position_rad[ORE_STORE_AXIS_NUM];
  float tracked_position_rad[ORE_STORE_AXIS_NUM];
  float command_position_rad[ORE_STORE_AXIS_NUM];

  OreStore_Feedback_t feedback;
  OreStore_Debug_t debug;
} OreStore_t;

int8_t OreStore_Init(OreStore_t *store, const OreStore_Params_t *param,
                     float target_freq);
int8_t OreStore_UpdateFeedback(OreStore_t *store);
int8_t OreStore_Control(OreStore_t *store, const OreStore_CMD_t *cmd,
                        uint32_t now);
void OreStore_Output(OreStore_t *store);
void OreStore_ResetOutput(OreStore_t *store);
void OreStore_RequestRehome(OreStore_t *store);
int8_t OreStore_AssumeAxisHomedAtCurrent(OreStore_t *store, uint8_t axis,
                                         float position_rad);
bool OreStore_IsAxisHomed(const OreStore_t *store, uint8_t axis);
bool OreStore_IsAllHomed(const OreStore_t *store);
bool OreStore_IsAxisAtTarget(const OreStore_t *store, uint8_t axis,
                             float threshold_rad);
bool OreStore_IsAllAtTarget(const OreStore_t *store, float threshold_rad);
const OreStore_Debug_t *OreStore_GetDebug(const OreStore_t *store);

#ifdef __cplusplus
}
#endif
