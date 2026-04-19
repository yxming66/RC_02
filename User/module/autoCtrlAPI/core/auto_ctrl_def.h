#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  AUTO_CTRL_ZONE_INVALID = 0,
  AUTO_CTRL_ZONE_R2_ENTRY1,
  AUTO_CTRL_ZONE_R2_ENTRY2,
  AUTO_CTRL_ZONE_R2_ENTRY3,
  AUTO_CTRL_ZONE_PLATFORM_1,
  AUTO_CTRL_ZONE_PLATFORM_2,
  AUTO_CTRL_ZONE_PLATFORM_3,
  AUTO_CTRL_ZONE_PLATFORM_4,
  AUTO_CTRL_ZONE_PLATFORM_5,
  AUTO_CTRL_ZONE_PLATFORM_6,
  AUTO_CTRL_ZONE_PLATFORM_7,
  AUTO_CTRL_ZONE_PLATFORM_8,
  AUTO_CTRL_ZONE_PLATFORM_9,
  AUTO_CTRL_ZONE_PLATFORM_10,
  AUTO_CTRL_ZONE_PLATFORM_11,
  AUTO_CTRL_ZONE_PLATFORM_12,
  AUTO_CTRL_ZONE_R2_EXIT1,
  AUTO_CTRL_ZONE_R2_EXIT2,
  AUTO_CTRL_ZONE_R2_EXIT3,
  AUTO_CTRL_ZONE_COUNT,
} auto_ctrl_zone_e;

typedef enum {
  AUTO_CTRL_TEMPLATE_NONE = 0,
  AUTO_CTRL_TEMPLATE_FLAT_MOVE,
  AUTO_CTRL_TEMPLATE_ASCEND_200,
  AUTO_CTRL_TEMPLATE_DESCEND_200,
  AUTO_CTRL_TEMPLATE_ASCEND_400_STD,
  AUTO_CTRL_TEMPLATE_DESCEND_400_STD,
} auto_ctrl_template_e;

typedef enum {
  AUTO_CTRL_SENSOR_MODE_NONE = 0,
  AUTO_CTRL_SENSOR_MODE_YAW_ONLY,
  AUTO_CTRL_SENSOR_MODE_SICK_FRONT_AND_BOTTOM,
  AUTO_CTRL_SENSOR_MODE_BOTTOM_ONLY,
} auto_ctrl_sensor_mode_e;

typedef enum {
  AUTO_CTRL_STATE_IDLE = 0,
  AUTO_CTRL_STATE_PREALIGN,
  AUTO_CTRL_STATE_RUN_TEMPLATE,
  AUTO_CTRL_STATE_SUCCESS,
  AUTO_CTRL_STATE_FAIL,
  AUTO_CTRL_STATE_ABORT,
} auto_ctrl_run_state_e;

typedef enum {
  AUTO_CTRL_RESULT_NONE = 0,
  AUTO_CTRL_RESULT_RUNNING,
  AUTO_CTRL_RESULT_SUCCESS,
  AUTO_CTRL_RESULT_FAIL,
  AUTO_CTRL_RESULT_ABORTED,
} auto_ctrl_result_e;

typedef enum {
  AUTO_CTRL_FAULT_NONE = 0,
  AUTO_CTRL_FAULT_INVALID_ZONE,
  AUTO_CTRL_FAULT_INVALID_TRANSITION,
  AUTO_CTRL_FAULT_INVALID_TEMPLATE,
  AUTO_CTRL_FAULT_PREALIGN_TIMEOUT,
  AUTO_CTRL_FAULT_TEMPLATE_TIMEOUT,
  AUTO_CTRL_FAULT_SENSOR_INVALID,
  AUTO_CTRL_FAULT_TEMPLATE_UNSUPPORTED,
  AUTO_CTRL_FAULT_ABORTED,
} auto_ctrl_fault_e;

typedef struct {
  auto_ctrl_zone_e zone;
  const char *name;
  int16_t height_mm;
  bool is_platform;
} auto_ctrl_zone_info_t;

typedef struct {
  auto_ctrl_zone_e from_zone;
  auto_ctrl_zone_e to_zone;
  auto_ctrl_template_e template_id;
  float required_yaw_deg;
  float yaw_tolerance_deg;
  auto_ctrl_sensor_mode_e sensor_mode;
  int16_t height_delta_mm;
} auto_ctrl_transition_t;

#ifdef __cplusplus
}
#endif