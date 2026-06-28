/*
    cmd_center application task
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "bsp/uart.h"
#include "bsp/gpio.h"
#include "device/dr16.h"
#include "module/chassis.h"
#include "module/pole.h"
#include "module/arm_simple.h"
#include "module/config.h"
#include "module/rod_new.h"
#include "module/autoCtrlAPI/api/auto_ctrl_api.h"
#include "module/autoCtrlAPI/core/auto_ctrl_def.h"
#include "component/cmd_center/cmd_center.hpp"
#include "module/rc_cmd_center/rc_cmd_center_app.h"
#include "module/mrlink_pc_comm/mrlink_pc_comm.h"
#include "main.h"
#include <math.h>
#include <string.h>
/* USER INCLUDE END */

#ifndef AUTO_CTRL_RC_YAW_TOLERANCE_RAD
#define AUTO_CTRL_RC_YAW_TOLERANCE_RAD (0.0872664626f)
#endif

#ifndef AUTO_CTRL_OUTPUT_ENABLE
#define AUTO_CTRL_OUTPUT_ENABLE (1u)
#endif

#if !AUTO_CTRL_OUTPUT_ENABLE
#define RC_AUTO_CTRL_DRY_RUN_ENABLED (1u)
#endif

#define AUTO_CTRL_RC_PI_RAD (3.14159265358979323846f)
#define RC_ARM_FINE_SCALE (1.50f)
#define RC_ARM_SIMPLE_DEADBAND (0.05f)
#define RC_ARM_SIMPLE_POINT_SLEEP_J1 (0.0f)
#define RC_ARM_SIMPLE_POINT_SLEEP_J2 (0.0f)
#define RC_ARM_SIMPLE_POINT_GRAB_J1 (1.0f)
#define RC_ARM_SIMPLE_POINT_GRAB_J2 (-1.57f)
#define RC_ARM_SIMPLE_POINT_LIFT_J1 (1.5f)
#define RC_ARM_SIMPLE_POINT_LIFT_J2 (0.0f)
#define RC_ARM_SIMPLE_POINT_RELEASE_J1 (1.0f)
#define RC_ARM_SIMPLE_POINT_RELEASE_J2 (1.57f)
#define RC_ROD_NEW_DEADBAND (0.05f)
#define RC_ROD_NEW_MANUAL_SPEED_RAD_S (1.0f)
#define RC_ROD_NEW_TARGET_MIN_RAD (0.0f)
#define RC_ROD_NEW_TARGET_MAX_RAD (1.0f)
#define RC_MAPPING_PRESET_DEFAULT (0u)
#define RC_MAPPING_PRESET_PC_FIRST (1u)
#ifndef RC_MAPPING_ACTIVE_PRESET
#define RC_MAPPING_ACTIVE_PRESET RC_MAPPING_PRESET_DEFAULT
#endif
#ifndef RC_LEFT_DOWN_MAPPING_AUTO_ORE
#define RC_LEFT_DOWN_MAPPING_AUTO_ORE (0u) // 0单独控制上层，1一键存取
#endif
#ifndef RC_AUTO_STEP_CH_RES_THRESHOLD
#define RC_AUTO_STEP_CH_RES_THRESHOLD (0.90f)
#endif

#ifndef RC_DEBUG_UPDATE_PERIOD_MS
#define RC_DEBUG_UPDATE_PERIOD_MS (50u)
#endif

#ifndef RC_MANUAL_IO_CH_THRESHOLD
#define RC_MANUAL_IO_CH_THRESHOLD (0.90f)

#define RC_CHASSIS_VX_SCALE (2.0f)
#define RC_CHASSIS_VY_SCALE (2.0f)
#define RC_CHASSIS_WZ_SCALE (3.0f)
#endif

#define RC_POLE_MANUAL_INPUT_SCALE (1.5f)
#define RC_POLE_CH_RES_DEADBAND (1.0e-4f)
#undef RC_POLE_CH_RES_ENABLE
#define RC_POLE_CH_RES_ENABLE (0u)

/* USER STRUCT BEGIN */
extern DR16_t dr16;
static Chassis_CMD_t chassis_cmd;
static Pole_CMD_t pole_cmd;
static ArmSimple_CMD_t arm_simple_cmd;
static OreStore_CMD_t ore_store_cmd;
static DR16_SwitchPos_t last_sw_l = DR16_SW_ERR;
static DR16_SwitchPos_t last_sw_r = DR16_SW_ERR;
static RodNew_CMD_t rod_cmd;
extern bool reset;
extern auto_ctrl_t auto_ctrl;
extern bool auto_ctrl_inited;
extern Chassis_IMU_t chassis_imu;

volatile PC_CommandSource_t g_pc_command_source = PC_COMMAND_SOURCE_RC;
volatile int8_t g_rc_ore_store_post_ret = ORE_STORE_ERR_NULL;
volatile int8_t g_rc_ore_store_assume_home_ret = ORE_STORE_ERR_NULL;
volatile uint32_t g_rc_ore_store_assume_home_count = 0u;

typedef enum {
  RC_CONTROL_PAGE_SAFE = 0,
  RC_CONTROL_PAGE_DRIVE,
  RC_CONTROL_PAGE_AUTO_200_UP,
  RC_CONTROL_PAGE_AUTO_200_DOWN,
  RC_CONTROL_PAGE_PC,
  RC_CONTROL_PAGE_ARM_SIMPLE,
  RC_CONTROL_PAGE_ORE_STORE,
  RC_CONTROL_PAGE_ROD_NEW,
  RC_CONTROL_PAGE_AUTO_ORE,
} RcControlPage_t;

typedef enum {
  RC_BEHAVIOR_SAFE = 0,
  RC_BEHAVIOR_DRIVE,
  RC_BEHAVIOR_AUTO_200_UP_STANDBY,
  RC_BEHAVIOR_AUTO_200_DOWN_STANDBY,
  RC_BEHAVIOR_PC,
  RC_BEHAVIOR_ARM_SIMPLE,
  RC_BEHAVIOR_ORE_STORE,
  RC_BEHAVIOR_ROD_NEW,
  RC_BEHAVIOR_AUTO_ORE,
} RcBehavior_t;

typedef enum {
  RC_CMD_PLAN_SAFE = 0,
  RC_CMD_PLAN_DRIVE,
  RC_CMD_PLAN_AUTO_STANDBY,
  RC_CMD_PLAN_PC,
  RC_CMD_PLAN_PC_AUTO_CTRL,
  RC_CMD_PLAN_ARM_SIMPLE,
  RC_CMD_PLAN_ORE_STORE,
  RC_CMD_PLAN_ROD_NEW,
  RC_CMD_PLAN_AUTO_ORE_STANDBY,
  RC_CMD_PLAN_AUTO_CTRL_OUTPUT,
  RC_CMD_PLAN_AUTO_ORE_OUTPUT,
  RC_CMD_PLAN_AUTO_ROD_OUTPUT,
  RC_CMD_PLAN_AUTO_ROD_STEP1_PC_OUTPUT,
  RC_CMD_PLAN_AUTO_SICK_CORRECT_OUTPUT,
} RcCommandPlan_t;

typedef struct {
  DR16_SwitchPos_t sw_l;
  DR16_SwitchPos_t sw_r;
  DR16_SwitchPos_t last_sw_l;
  DR16_SwitchPos_t last_sw_r;
  RcBehavior_t behavior;
} RcSwitchBehaviorMap_t;

typedef struct {
  volatile RcControlPage_t page;
  volatile bool reset_event;
  volatile bool auto_200_start_event;
  volatile bool auto_200_start_ok;
  volatile auto_ctrl_template_e auto_200_template;
  volatile bool arm_simple_active;
  volatile bool rod_active;
  volatile bool ore_store_active;
  volatile bool auto_ore_start_event;
  volatile bool auto_ore_start_ok;
  volatile AutoOre_State_t auto_ore_state;
  volatile AutoOre_Result_t auto_ore_result;
  volatile AutoOre_Fault_t auto_ore_fault;
  volatile AutoOre_Action_t auto_ore_action;
  volatile AutoOre_Position_t auto_ore_active_position;
  volatile uint8_t auto_ore_occupancy_mask;
  volatile uint8_t auto_ore_step_index;
  volatile AutoRodSpearhead_State_t auto_rod_spearhead_state;
  volatile AutoRodSpearhead_Result_t auto_rod_spearhead_result;
  volatile AutoRodSpearhead_Fault_t auto_rod_spearhead_fault;
  volatile AutoRodSpearhead_Action_t auto_rod_spearhead_action;
  volatile uint8_t auto_rod_spearhead_step_index;
  volatile AutoSickCorrect_State_t auto_sick_correct_state;
  volatile AutoSickCorrect_Result_t auto_sick_correct_result;
  volatile AutoSickCorrect_Fault_t auto_sick_correct_fault;
  volatile AutoSickCorrect_Action_t auto_sick_correct_action;
  volatile uint8_t auto_sick_correct_step_index;
} RcControlDebug_t;

volatile RcControlDebug_t g_rc_control_debug = {};

typedef struct {
  volatile bool online;
  volatile DR16_SwitchPos_t sw_l;
  volatile DR16_SwitchPos_t sw_r;
  volatile DR16_SwitchPos_t last_sw_l;
  volatile DR16_SwitchPos_t last_sw_r;
  volatile OreStore_Mode_t cmd_mode;
  volatile bool cmd_force_rehome;
  volatile float platform_target_rad; 
  volatile bool assume_home_event;
  volatile int8_t assume_home_ret;
  volatile int8_t post_ret;
} RcOreStoreDebug_t;

volatile RcOreStoreDebug_t g_rc_ore_store_debug = {0};

static bool ore_store_active_initialized = false;
static bool arm_simple_target_initialized = false;
static bool auto_ctrl_was_busy = false;
static bool auto_ore_was_busy = false;
static bool auto_rod_spearhead_was_busy = false;
static bool auto_sick_correct_was_busy = false;
static bool auto_rod_spearhead_hold_after_finish = false;
static RodNew_GripState_t rod_grip_latched = ROD_NEW_GRIP_RELEASE;
static float rod_target_angle_latched_rad = 1.0f;
static Suction_State_t arm_simple_suction_latched = SUCTION_OFF;
static RcBehavior_t rc_current_behavior = RC_BEHAVIOR_SAFE;
static RcCommandPlan_t rc_current_plan = RC_CMD_PLAN_SAFE;
static bool rc_cmd_center_configured = false;
static uint32_t rc_debug_last_update_ms = 0u;

static cmd::Center<1, 5, 96, 20, 64, 512, 128> rc_cmd_center;

typedef struct {
  auto_ctrl_template_e template_id;
  auto_ctrl_travel_dir_e travel_dir;
} RcAutoCtrlStartConfig_t;

typedef enum {
  RC_AUTO_STEP_PROFILE_NONE,
  RC_AUTO_STEP_PROFILE_FUSED_200,
  RC_AUTO_STEP_PROFILE_FUSED_UP_400_NORMAL_DOWN_400,
} RcAutoStepProfile_t;

static RcAutoStepProfile_t Rc_SelectAutoStepProfile(void) {
  if (dr16.data.ch_res <= -RC_AUTO_STEP_CH_RES_THRESHOLD) {
    return RC_AUTO_STEP_PROFILE_FUSED_200;
  }
  if (dr16.data.ch_res >= RC_AUTO_STEP_CH_RES_THRESHOLD) {
    return RC_AUTO_STEP_PROFILE_FUSED_UP_400_NORMAL_DOWN_400;
  }
  return RC_AUTO_STEP_PROFILE_NONE;
}

#if RC_MAPPING_ACTIVE_PRESET == RC_MAPPING_PRESET_PC_FIRST
static const RcSwitchBehaviorMap_t rc_behavior_map_active[] = {
  {DR16_SW_UP, DR16_SW_UP, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_PC},
  {DR16_SW_MID, DR16_SW_UP, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_AUTO_200_UP_STANDBY},
  {DR16_SW_MID, DR16_SW_DOWN, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_AUTO_200_DOWN_STANDBY},
  {DR16_SW_MID, DR16_SW_MID, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_DRIVE},
#if RC_LEFT_DOWN_MAPPING_AUTO_ORE
  {DR16_SW_DOWN, DR16_SW_UP, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_ARM_SIMPLE},
  {DR16_SW_DOWN, DR16_SW_MID, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_ARM_SIMPLE},
  {DR16_SW_DOWN, DR16_SW_DOWN, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_AUTO_ORE},
#else
  {DR16_SW_DOWN, DR16_SW_UP, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_ARM_SIMPLE},
  {DR16_SW_DOWN, DR16_SW_MID, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_ARM_SIMPLE},
  {DR16_SW_DOWN, DR16_SW_DOWN, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_ROD_NEW},
#endif
};
#else
static const RcSwitchBehaviorMap_t rc_behavior_map_active[] = {
  {DR16_SW_UP, DR16_SW_UP, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_PC},
  {DR16_SW_MID, DR16_SW_UP, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_AUTO_200_UP_STANDBY},
  {DR16_SW_MID, DR16_SW_DOWN, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_AUTO_200_DOWN_STANDBY},
  {DR16_SW_MID, DR16_SW_MID, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_DRIVE},
#if RC_LEFT_DOWN_MAPPING_AUTO_ORE
  {DR16_SW_DOWN, DR16_SW_UP, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_ARM_SIMPLE},
  {DR16_SW_DOWN, DR16_SW_MID, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_ARM_SIMPLE},
  {DR16_SW_DOWN, DR16_SW_DOWN, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_AUTO_ORE},
#else
  {DR16_SW_DOWN, DR16_SW_UP, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_ARM_SIMPLE},
  {DR16_SW_DOWN, DR16_SW_MID, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_ORE_STORE},
  {DR16_SW_DOWN, DR16_SW_DOWN, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_ROD_NEW},
#endif
};
#endif

static const RcSwitchBehaviorMap_t *Rc_GetBehaviorMap(size_t *count) {
  *count = sizeof(rc_behavior_map_active) / sizeof(rc_behavior_map_active[0]);
  return rc_behavior_map_active;
}

static bool Rc_SwitchMapEntryMatches(const RcSwitchBehaviorMap_t *entry) {
  if (entry == NULL) {
    return false;
  }
  if (entry->sw_l != dr16.data.sw_l || entry->sw_r != dr16.data.sw_r) {
    return false;
  }
  if (entry->last_sw_l != DR16_SW_ERR && entry->last_sw_l != last_sw_l) {
    return false;
  }
  if (entry->last_sw_r != DR16_SW_ERR && entry->last_sw_r != last_sw_r) {
    return false;
  }
  return true;
}

#define RC_ORE_STORE_DEADBAND (0.05f)
#define RC_ORE_STORE_FALLBACK_MOVE_SPEED_RAD_S (0.5f)

static bool Rc_KeyDown(DR16_Key_t key) {
  return key < DR16_KEY_NUM && dr16.data.keyboard.key[key];
}

static float Rc_ApplyDeadband(float value, float deadband) {
  return (fabsf(value) < deadband) ? 0.0f : value;
}

#if RC_POLE_CH_RES_ENABLE
static float Rc_ClampFloat(float value, float min_value, float max_value) {
  if (!isfinite(value)) {
    return 0.0f;
  }
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static float Rc_ClampPoleManualLift(float lift) {
  return Rc_ClampFloat(lift, -1.0f, 1.0f);
}

static float Rc_GetPoleChResManualLift(void) {
  return Rc_ClampPoleManualLift(dr16.data.ch_res);
}
#endif

static float Rc_ClampRodNewTarget(float target_rad) {
  if (!isfinite(target_rad)) {
    return 0.0f;
  }
  if (target_rad < RC_ROD_NEW_TARGET_MIN_RAD) {
    return RC_ROD_NEW_TARGET_MIN_RAD;
  }
  if (target_rad > RC_ROD_NEW_TARGET_MAX_RAD) {
    return RC_ROD_NEW_TARGET_MAX_RAD;
  }
  return target_rad;
}

static void Rc_SetRodRelax(void) {
  rod_cmd.mode = ROD_NEW_MODE_RELAX;
  rod_cmd.pose = ROD_NEW_POSE_DOCK_WAIT;
  rod_cmd.grip = rod_grip_latched;
  rod_cmd.target_angle_rad = rod_target_angle_latched_rad;
}

static void Rc_SetChassisRelax(void) {
  chassis_cmd.mode = CHASSIS_MODE_RELAX;
  chassis_cmd.ctrl_vec.vx = 0.0f;
  chassis_cmd.ctrl_vec.vy = 0.0f;
  chassis_cmd.ctrl_vec.wz = 0.0f;
}

static void Rc_SetChassisHold(void) {
  chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
  chassis_cmd.ctrl_vec.vx = 0.0f;
  chassis_cmd.ctrl_vec.vy = 0.0f;
  chassis_cmd.ctrl_vec.wz = 0.0f;
}

static void Rc_UpdateArmSimpleIoFromLeftX(void) {
  if (dr16.data.ch_l_x <= -RC_MANUAL_IO_CH_THRESHOLD) {
    arm_simple_suction_latched = SUCTION_OFF;
  } else if (dr16.data.ch_l_x >= RC_MANUAL_IO_CH_THRESHOLD) {
    arm_simple_suction_latched = SUCTION_ON;
  }

  arm_simple_cmd.suction = arm_simple_suction_latched;
}

static void Rc_UpdateRodIoFromLeftX(void) {
  if (dr16.data.ch_l_x <= -RC_MANUAL_IO_CH_THRESHOLD) {
    rod_grip_latched = ROD_NEW_GRIP_RELEASE;
  } else if (dr16.data.ch_l_x >= RC_MANUAL_IO_CH_THRESHOLD) {
    rod_grip_latched = ROD_NEW_GRIP_GRAB;
  }

  rod_cmd.grip = rod_grip_latched;
}

static bool Rc_OreStoreCylinderClosedFromLeftX(bool current_closed) {
  if (dr16.data.ch_l_x <= -RC_MANUAL_IO_CH_THRESHOLD) {
    return false;
  }
  if (dr16.data.ch_l_x >= RC_MANUAL_IO_CH_THRESHOLD) {
    return true;
  }
  return current_closed;
}

static void Rc_SetRodHold(void) {
  if (rod_cmd.mode == ROD_NEW_MODE_ACTIVE) {
    return;
  }
  Rc_SetRodRelax();
}

static RcAutoCtrlStartConfig_t Rc_SelectAutoCtrlStartConfig(void) {
  RcAutoCtrlStartConfig_t config = {
      AUTO_CTRL_TEMPLATE_NONE,
      AUTO_CTRL_TRAVEL_DIR_HEAD_FORWARD,
  };

  if (dr16.data.sw_l != DR16_SW_MID || last_sw_r != DR16_SW_MID) {
    return config;
  }

  if (Rc_SelectAutoStepProfile() !=
      RC_AUTO_STEP_PROFILE_FUSED_UP_400_NORMAL_DOWN_400) {
    return config;
  }

  if (dr16.data.sw_r == DR16_SW_DOWN) {
    config.template_id = AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD;
  } 

  return config;
}

static RcControlPage_t Rc_GetAutoCtrlPage(auto_ctrl_template_e template_id) {
  switch (template_id) {
    case AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD:
    case AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD:
      return RC_CONTROL_PAGE_AUTO_200_UP; 
    case AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD:
      return RC_CONTROL_PAGE_AUTO_200_DOWN;
    default:
      return RC_CONTROL_PAGE_AUTO_200_DOWN;
  }
}

static void Rc_SetRodOperator(void) {
  const float dt_s = 1.0f / (float)RC_MAIN_FREQ;
  const float target_delta = 12*Rc_ApplyDeadband(dr16.data.ch_r_y,
                                             RC_ROD_NEW_DEADBAND) *
                             RC_ROD_NEW_MANUAL_SPEED_RAD_S * dt_s;

    rod_target_angle_latched_rad = Rc_ClampRodNewTarget(
        rod_target_angle_latched_rad + target_delta);
  Rc_UpdateRodIoFromLeftX();
  rod_cmd.mode = ROD_NEW_MODE_ACTIVE;
  rod_cmd.pose = ROD_NEW_POSE_MANUAL;
  rod_cmd.grip = rod_grip_latched;
  rod_cmd.target_angle_rad = rod_target_angle_latched_rad;
}

static float Rc_ApplyOreStoreDeadband(float value) {
  return Rc_ApplyDeadband(value, RC_ORE_STORE_DEADBAND);
}

static float Rc_OreStoreAxisMoveSpeed(uint8_t axis) {
  Config_RobotParam_t* robot_param = Config_GetRobotParam();
  if (robot_param == NULL || axis >= ORE_STORE_AXIS_NUM) {
    return RC_ORE_STORE_FALLBACK_MOVE_SPEED_RAD_S;
  }

  const float speed = robot_param->ore_store_param.limit.move_velocity_rad_s[axis];
  return (speed > 0.0f && isfinite(speed)) ? speed
                                           : RC_ORE_STORE_FALLBACK_MOVE_SPEED_RAD_S;
}

static float Rc_OreStoreAxisTravel(uint8_t axis) {
  Config_RobotParam_t* robot_param = Config_GetRobotParam();
  if (robot_param == NULL || axis >= ORE_STORE_AXIS_NUM) {
    return 0.0f;
  }

  const float travel = robot_param->ore_store_param.limit.travel_rad[axis];
  return (travel > 0.0f && isfinite(travel)) ? travel : 0.0f;
}

static float Rc_ClampOreStoreTarget(uint8_t axis, float target_rad) {
  const float travel_rad = Rc_OreStoreAxisTravel(axis);
  if (!isfinite(target_rad) || travel_rad <= 0.0f) {
    return 0.0f;
  }

  if (target_rad < 0.0f) {
    return 0.0f;
  }
  if (target_rad > travel_rad) {
    return travel_rad;
  }
  return target_rad;
}

static void Rc_SetOreStoreRelax(void) {
  memset(&ore_store_cmd, 0, sizeof(ore_store_cmd));
  if (Task_OreStorePowerOnHomeInProgress()) {
    ore_store_cmd.mode = ORE_STORE_MODE_HOME;
    ore_store_active_initialized = false;
    return;
  }
  ore_store_cmd.mode = Task_OreStoreIsAllHomed() ? ORE_STORE_MODE_RELAX
                                                 : ORE_STORE_MODE_HOME;
  ore_store_active_initialized = false;
}

static void Rc_SetOreStoreHold(void) {
  if (Task_OreStorePowerOnHomeInProgress()) {
    Rc_SetOreStoreRelax();
    return;
  }
  if (ore_store_active_initialized) {
    ore_store_cmd.mode = ORE_STORE_MODE_ACTIVE;
    ore_store_cmd.force_rehome = false;
    return;
  }
  Rc_SetOreStoreRelax();
}

static void Rc_InitOreStoreActiveTargets(void) {
  const OreStore_Feedback_t* feedback = Task_OreStoreGetFeedback();

  memset(&ore_store_cmd, 0, sizeof(ore_store_cmd));
  ore_store_cmd.mode = ORE_STORE_MODE_ACTIVE;

  if (feedback != NULL) {
    if (feedback->homed[ORE_STORE_AXIS_PLATFORM]) {
      ore_store_cmd.platform_target_rad =
        feedback->position_rad[ORE_STORE_AXIS_PLATFORM];
    }
    ore_store_cmd.fixed_ore_cylinder_closed =
        feedback->fixed_ore_cylinder_closed;
  }

  ore_store_cmd.platform_target_rad = Rc_ClampOreStoreTarget(
      ORE_STORE_AXIS_PLATFORM, ore_store_cmd.platform_target_rad);
  ore_store_active_initialized = true;
}

static void Rc_SetOreStoreActiveManual(void) {
  if (Task_OreStorePowerOnHomeInProgress()) {
    Rc_SetOreStoreRelax();
    return;
  }

  const float dt_s = 1.0f / (float)RC_MAIN_FREQ;
  const float platform_velocity =
    Rc_ApplyOreStoreDeadband(dr16.data.ch_r_y) * 0.2*
    Rc_OreStoreAxisMoveSpeed(ORE_STORE_AXIS_PLATFORM);
  const float platform_delta = platform_velocity * dt_s;

  if (!ore_store_active_initialized) {
    Rc_InitOreStoreActiveTargets();
  }

  ore_store_cmd.mode = ORE_STORE_MODE_ACTIVE;
  ore_store_cmd.force_rehome = false;
  ore_store_cmd.fixed_ore_cylinder_closed =
      Rc_OreStoreCylinderClosedFromLeftX(
          ore_store_cmd.fixed_ore_cylinder_closed);
  ore_store_cmd.platform_target_rad = Rc_ClampOreStoreTarget(
      ORE_STORE_AXIS_PLATFORM, ore_store_cmd.platform_target_rad + platform_delta);
}

static void Rc_SetPoleManual(float left, float right) {
  pole_cmd.mode = POLE_MODE_ACTIVE;
  pole_cmd.lift[0] = left * RC_POLE_MANUAL_INPUT_SCALE;
  pole_cmd.lift[1] = right * RC_POLE_MANUAL_INPUT_SCALE;
  pole_cmd.auto_target_enable[0] = false;
  pole_cmd.auto_target_enable[1] = false;
  pole_cmd.auto_target_lift[0] = 0.0f;
  pole_cmd.auto_target_lift[1] = 0.0f;
  pole_cmd.auto_lift_speed[0] = 0.0f;
  pole_cmd.auto_lift_speed[1] = 0.0f;
  pole_cmd.auto_lift_accel[0] = 0.0f;
  pole_cmd.auto_lift_accel[1] = 0.0f;
  pole_cmd.disable_lift_accel = false;
}

static void Rc_SetPoleAutoTarget(float left_target, float right_target) {
  pole_cmd.mode = POLE_MODE_ACTIVE;
  pole_cmd.lift[0] = 0.0f;
  pole_cmd.lift[1] = 0.0f;
  pole_cmd.auto_target_enable[0] = true;
  pole_cmd.auto_target_enable[1] = true;
  pole_cmd.auto_target_lift[0] = left_target;
  pole_cmd.auto_target_lift[1] = right_target;
  pole_cmd.auto_lift_speed[0] = 0.0f;
  pole_cmd.auto_lift_speed[1] = 0.0f;
  pole_cmd.auto_lift_accel[0] = 0.0f;
  pole_cmd.auto_lift_accel[1] = 0.0f;
  pole_cmd.disable_lift_accel = false;
}

static void Rc_SetPoleAuto(float left_target, float right_target) {
  (void)left_target;
  (void)right_target;
  pole_cmd.mode = POLE_MODE_RELAX;
  pole_cmd.lift[0] = 0.0f;
  pole_cmd.lift[1] = 0.0f;
  pole_cmd.auto_target_enable[0] = false;
  pole_cmd.auto_target_enable[1] = false;
  pole_cmd.auto_target_lift[0] = 0.0f;
  pole_cmd.auto_target_lift[1] = 0.0f;
  pole_cmd.auto_lift_speed[0] = 0.0f;
  pole_cmd.auto_lift_speed[1] = 0.0f;
  pole_cmd.disable_lift_accel = false;
}

static bool Rc_SetPolePcCommand(bool require_received_cmd) {
  if (!MrlinkPc_IsPCControlMode()) {
    return false;
  }
  if (require_received_cmd && !MrlinkPc_HasPoleCMD()) {
    return false;
  }

  const PC_PoleCMD_t *pc_pole_cmd = MrlinkPc_GetPoleCMD();
  if (pc_pole_cmd == NULL) {
    return false;
  }

  pole_cmd.mode = (pc_pole_cmd->mode == 0u) ? POLE_MODE_RELAX
                                             : POLE_MODE_ACTIVE;
  pole_cmd.lift[0] = 0.0f;
  pole_cmd.lift[1] = 0.0f;
  pole_cmd.auto_target_enable[0] = (pole_cmd.mode == POLE_MODE_ACTIVE);
  pole_cmd.auto_target_enable[1] = (pole_cmd.mode == POLE_MODE_ACTIVE);
  pole_cmd.auto_target_lift[0] = pc_pole_cmd->lift[0];
  pole_cmd.auto_target_lift[1] = pc_pole_cmd->lift[1];
  pole_cmd.auto_lift_speed[0] = -1.0f;
  pole_cmd.auto_lift_speed[1] = -1.0f;
  pole_cmd.auto_lift_accel[0] = -1.0f;
  pole_cmd.auto_lift_accel[1] = -1.0f;
  pole_cmd.disable_lift_accel = false;
  return true;
}

static void Rc_SetPoleHold(void) {
  if (pole_cmd.mode == POLE_MODE_ACTIVE) {
    pole_cmd.lift[0] = 0.0f;
    pole_cmd.lift[1] = 0.0f;
    return;
  }

  Rc_SetPoleManual(0.0f, 0.0f);
}

#if RC_POLE_CH_RES_ENABLE
static bool Rc_PoleCommandIsManual(void) {
  return pole_cmd.mode == POLE_MODE_ACTIVE &&
         !pole_cmd.auto_target_enable[0] &&
         !pole_cmd.auto_target_enable[1];
}

static void Rc_ApplyPoleChResControl(RcBehavior_t behavior) {
  if (!dr16.header.online || behavior == RC_BEHAVIOR_SAFE ||
      behavior == RC_BEHAVIOR_PC) {
    return;
  }

  const float lift = Rc_GetPoleChResManualLift();
  if (fabsf(lift) <= RC_POLE_CH_RES_DEADBAND) {
    return;
  }

  if (Rc_PoleCommandIsManual()) {
    pole_cmd.lift[0] = Rc_ClampPoleManualLift(pole_cmd.lift[0] + lift);
    pole_cmd.lift[1] = Rc_ClampPoleManualLift(pole_cmd.lift[1] + lift);
    return;
  }

  Rc_SetPoleManual(lift, lift);
}
#endif

static void Rc_SetArmSimpleRelax(void) {
  memset(&arm_simple_cmd, 0, sizeof(arm_simple_cmd));
  arm_simple_cmd.mode = ARM_SIMPLE_MODE_RELAX;
  arm_simple_cmd.point_mode = ARM_SIMPLE_POINT_NONE;
  arm_simple_cmd.suction = arm_simple_suction_latched;
  arm_simple_target_initialized = false;
}

static void Rc_SetArmSimpleHold(void) {
  arm_simple_cmd.mode = ARM_SIMPLE_MODE_JOINT;
  arm_simple_cmd.point_mode = ARM_SIMPLE_POINT_NONE;
  arm_simple_cmd.suction = arm_simple_suction_latched;
  arm_simple_target_initialized = true;
}

static void Rc_LatchPoleCurrentTarget(void) {
  Pole_CMD_t hold_cmd;
  if (Task_PoleMainGetHoldCommand(&hold_cmd)) {
    pole_cmd = hold_cmd;
    return;
  }

  Rc_SetPoleHold();
}

static void Rc_LatchArmSimpleCurrentTarget(void) {
  const ArmSimple_Feedback_t* feedback = Task_ArmSimpleGetFeedback();
  if (feedback == NULL) {
    Rc_SetArmSimpleHold();
    return;
  }

  arm_simple_cmd.mode = ARM_SIMPLE_MODE_JOINT;
  arm_simple_cmd.point_mode = ARM_SIMPLE_POINT_NONE;
  arm_simple_cmd.suction = feedback->suction;
  arm_simple_cmd.target_joint.joint1 =
      isfinite(feedback->joint1_angle_rad) ? feedback->joint1_angle_rad
                                           : feedback->target_joint1_rad;
  arm_simple_cmd.target_joint.joint2 =
      isfinite(feedback->joint2_angle_rad) ? feedback->joint2_angle_rad
                                           : feedback->target_joint2_rad;
  arm_simple_cmd.joint1_vel = 0.0f;
  arm_simple_cmd.joint1_max_vel_rad_s = 0.0f;
  arm_simple_cmd.joint2_max_vel_rad_s = 0.0f;
  arm_simple_suction_latched = arm_simple_cmd.suction;
  arm_simple_target_initialized = true;
}

static void Rc_LatchOreStoreCurrentTarget(void) {
  if (Task_OreStorePowerOnHomeInProgress() || !Task_OreStoreIsAllHomed()) {
    Rc_SetOreStoreRelax();
    return;
  }

  Rc_InitOreStoreActiveTargets();
}

static void Rc_LatchAutoCtrlCurrentTargets(void) {
  Rc_SetChassisRelax();
  Rc_LatchPoleCurrentTarget();
}

static void Rc_LatchAutoOreCurrentTargets(void) {
  Rc_SetChassisRelax();
  Rc_LatchPoleCurrentTarget();
  Rc_LatchArmSimpleCurrentTarget();
  Rc_LatchOreStoreCurrentTarget();
  Rc_SetRodHold();
}

static void Rc_LatchAutoSickCorrectCurrentTargets(void) {
  Rc_SetChassisRelax();
  Rc_LatchPoleCurrentTarget();
}

static void Rc_LatchRodCurrentTarget(void) {
  const RodNew_Feedback_t *feedback = Task_RodNewGetFeedback();
  if (feedback == NULL) {
    Rc_SetRodHold();
    return;
  }

  rod_cmd.mode = ROD_NEW_MODE_ACTIVE;
  rod_cmd.pose = feedback->pose;
  rod_cmd.grip = feedback->grip;
  rod_cmd.target_angle_rad =
      Rc_ClampRodNewTarget(feedback->target_angle_rad);
  rod_grip_latched = rod_cmd.grip;
  rod_target_angle_latched_rad = rod_cmd.target_angle_rad;
}

static void Rc_LatchAutoRodSpearheadHoldTargets(void) {
  Rc_SetChassisHold();
  Rc_LatchPoleCurrentTarget();
  Rc_LatchArmSimpleCurrentTarget();
  Rc_LatchOreStoreCurrentTarget();
}

static void Rc_LatchFinishedAutoTargets(void) {
  const bool auto_ctrl_busy_now =
      auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl);
  const bool auto_ore_busy_now =
      auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl);
  const bool auto_rod_busy_now = Task_AutoRodSpearheadIsBusy();
  const bool auto_sick_correct_busy_now = Task_AutoSickCorrectIsBusy();

  if (auto_ctrl_was_busy && !auto_ctrl_busy_now) {
    Rc_LatchAutoCtrlCurrentTargets();
  }
  if (auto_ore_was_busy && !auto_ore_busy_now) {
    Rc_LatchAutoOreCurrentTargets();
  }
  if (auto_rod_spearhead_was_busy && !auto_rod_busy_now) {
    if (AutoRodSpearhead_GetState(&auto_rod_spearhead_ctrl) ==
        AUTO_ROD_SPEARHEAD_STATE_ABORT) {
      auto_rod_spearhead_hold_after_finish = false;
    } else {
      Rc_LatchOreStoreCurrentTarget();
      Rc_LatchRodCurrentTarget();
      auto_rod_spearhead_hold_after_finish = true;
    }
  }
  if (auto_sick_correct_was_busy && !auto_sick_correct_busy_now) {
    Rc_LatchAutoSickCorrectCurrentTargets();
  }
}

static void Rc_UpdateAutoBusyHistory(void) {
  auto_ctrl_was_busy = auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl);
  auto_ore_was_busy = auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl);
  auto_rod_spearhead_was_busy = Task_AutoRodSpearheadIsBusy();
  auto_sick_correct_was_busy = Task_AutoSickCorrectIsBusy();
}

static const ArmSimple_Params_t* Rc_GetArmSimpleParam(void) {
  Config_RobotParam_t* robot_param = Config_GetRobotParam();
  if (robot_param == NULL) {
    return NULL;
  }
  return &robot_param->arm_simple_param;
}

static float Rc_ClampArmSimpleJoint1(float target_rad) {
  const ArmSimple_Params_t* param = Rc_GetArmSimpleParam();
  if (param == NULL || !isfinite(target_rad)) {
    return 0.0f;
  }
  if (target_rad < param->soft_limit.joint1_min) {
    return param->soft_limit.joint1_min;
  }
  if (target_rad > param->soft_limit.joint1_max) {
    return param->soft_limit.joint1_max;
  }
  return target_rad;
}

static float Rc_ClampArmSimpleJoint2(float target_rad) {
  const ArmSimple_Params_t* param = Rc_GetArmSimpleParam();
  if (param == NULL || !isfinite(target_rad)) {
    return 0.0f;
  }
  if (target_rad < param->soft_limit.joint2_min) {
    return param->soft_limit.joint2_min;
  }
  if (target_rad > param->soft_limit.joint2_max) {
    return param->soft_limit.joint2_max;
  }
  return target_rad;
}

static float Rc_ArmSimpleJoint1MaxVel(void) {
  const ArmSimple_Params_t* param = Rc_GetArmSimpleParam();
  return (param != NULL && param->vel_limit.joint1_max_vel > 0.0f)
             ? param->vel_limit.joint1_max_vel
             : 1.0f;
}

static float Rc_ArmSimpleJoint2MaxVel(void) {
  const ArmSimple_Params_t* param = Rc_GetArmSimpleParam();
  return (param != NULL && param->vel_limit.joint2_max_vel > 0.0f)
             ? param->vel_limit.joint2_max_vel
             : 3.0f;
}

static void Rc_SetArmSimplePoint(ArmSimple_PointMode_t point,
                                 float joint1_rad,
                                 float joint2_rad) {
  arm_simple_cmd.mode = ARM_SIMPLE_MODE_JOINT;
  arm_simple_cmd.point_mode = point;
  arm_simple_cmd.target_joint.joint1 = Rc_ClampArmSimpleJoint1(joint1_rad);
  arm_simple_cmd.target_joint.joint2 = Rc_ClampArmSimpleJoint2(joint2_rad);
  arm_simple_target_initialized = true;
}

static void Rc_SetArmSimpleStandby(void) {
  const ArmSimple_Params_t* param = Rc_GetArmSimpleParam();
  if (param != NULL &&
      ArmSimple_MakeBehaviorCommand(param, ARM_SIMPLE_BEHAVIOR_STANDBY,
                                    arm_simple_suction_latched,
                                    &arm_simple_cmd)) {
    arm_simple_target_initialized = true;
    return;
  }

  Rc_SetArmSimplePoint(ARM_SIMPLE_POINT_SLEEP,
                       RC_ARM_SIMPLE_POINT_SLEEP_J1,
                       RC_ARM_SIMPLE_POINT_SLEEP_J2);
  arm_simple_cmd.suction = arm_simple_suction_latched;
}

static bool Rc_SetArmSimplePcAbstractCommand(
    const PC_AbstractPositionCMD_t *pc_abstract_cmd) {
  if (pc_abstract_cmd == NULL ||
      (pc_abstract_cmd->enable_mask & PC_ABSTRACT_MODULE_ARM_SIMPLE) == 0u) {
    return false;
  }

  const uint8_t position = pc_abstract_cmd->arm_simple_position;

  switch ((PC_AbstractArmSimplePosition_t)position) {
    case PC_ABSTRACT_ARM_SIMPLE_RELAX:
      Rc_SetArmSimpleRelax();
      return true;
    case PC_ABSTRACT_ARM_SIMPLE_SLEEP:
      Rc_SetArmSimplePoint(ARM_SIMPLE_POINT_SLEEP,
                           RC_ARM_SIMPLE_POINT_SLEEP_J1,
                           RC_ARM_SIMPLE_POINT_SLEEP_J2);
      break;
    case PC_ABSTRACT_ARM_SIMPLE_GRAB:
      Rc_SetArmSimplePoint(ARM_SIMPLE_POINT_GRAB,
                           RC_ARM_SIMPLE_POINT_GRAB_J1,
                           RC_ARM_SIMPLE_POINT_GRAB_J2);
      break;
    case PC_ABSTRACT_ARM_SIMPLE_LIFT:
      Rc_SetArmSimplePoint(ARM_SIMPLE_POINT_LIFT,
                           RC_ARM_SIMPLE_POINT_LIFT_J1,
                           RC_ARM_SIMPLE_POINT_LIFT_J2);
      break;
    case PC_ABSTRACT_ARM_SIMPLE_RELEASE:
      Rc_SetArmSimplePoint(ARM_SIMPLE_POINT_RELEASE,
                           RC_ARM_SIMPLE_POINT_RELEASE_J1,
                           RC_ARM_SIMPLE_POINT_RELEASE_J2);
      break;
    default: {
      const uint8_t behavior_index =
          position - PC_ABSTRACT_ARM_SIMPLE_BEHAVIOR_STANDBY;
      if (position < PC_ABSTRACT_ARM_SIMPLE_BEHAVIOR_STANDBY ||
          behavior_index >= ARM_SIMPLE_BEHAVIOR_NUM) {
        return false;
      }

      const ArmSimple_Params_t* param = Rc_GetArmSimpleParam();
      if (param == NULL ||
          !ArmSimple_MakeBehaviorCommand(
              param, (ArmSimple_BehaviorPoint_t)behavior_index,
              arm_simple_suction_latched,
              &arm_simple_cmd)) {
        return false;
      }
      break;
    }
  }

  arm_simple_cmd.suction = arm_simple_suction_latched;
  arm_simple_target_initialized = true;
  return true;
}

static void Rc_SetArmSimpleOperator(float scale) {
  const float dt_s = 1.0f / (float)RC_MAIN_FREQ;
  const float joint1_delta = Rc_ApplyDeadband(dr16.data.ch_r_y,
                                              RC_ARM_SIMPLE_DEADBAND) *
                             Rc_ArmSimpleJoint1MaxVel() * scale * dt_s;
  const float joint2_delta = Rc_ApplyDeadband(dr16.data.ch_l_y,
                                              RC_ARM_SIMPLE_DEADBAND) *
                             Rc_ArmSimpleJoint2MaxVel() * scale * dt_s;

  if (!arm_simple_target_initialized) {
    Rc_SetArmSimplePoint(ARM_SIMPLE_POINT_SLEEP,
                         RC_ARM_SIMPLE_POINT_SLEEP_J1,
                         RC_ARM_SIMPLE_POINT_SLEEP_J2);
  }

  arm_simple_cmd.mode = ARM_SIMPLE_MODE_JOINT;
  arm_simple_cmd.point_mode = ARM_SIMPLE_POINT_NONE;

  if (Rc_KeyDown(DR16_KEY_Z)) {
    Rc_SetArmSimplePoint(ARM_SIMPLE_POINT_SLEEP,
                         RC_ARM_SIMPLE_POINT_SLEEP_J1,
                         RC_ARM_SIMPLE_POINT_SLEEP_J2);
  } else if (Rc_KeyDown(DR16_KEY_X)) {
    Rc_SetArmSimplePoint(ARM_SIMPLE_POINT_GRAB,
                         RC_ARM_SIMPLE_POINT_GRAB_J1,
                         RC_ARM_SIMPLE_POINT_GRAB_J2);
  } else if (Rc_KeyDown(DR16_KEY_C)) {
    Rc_SetArmSimplePoint(ARM_SIMPLE_POINT_LIFT,
                         RC_ARM_SIMPLE_POINT_LIFT_J1,
                         RC_ARM_SIMPLE_POINT_LIFT_J2);
  } else if (Rc_KeyDown(DR16_KEY_V)) {
    Rc_SetArmSimplePoint(ARM_SIMPLE_POINT_RELEASE,
                         RC_ARM_SIMPLE_POINT_RELEASE_J1,
                         RC_ARM_SIMPLE_POINT_RELEASE_J2);
  } else {
    arm_simple_cmd.target_joint.joint1 = Rc_ClampArmSimpleJoint1(
        arm_simple_cmd.target_joint.joint1 + joint1_delta);
    arm_simple_cmd.target_joint.joint2 = Rc_ClampArmSimpleJoint2(
        arm_simple_cmd.target_joint.joint2 + joint2_delta);
  }

  Rc_UpdateArmSimpleIoFromLeftX();
}

static void Rc_SetChassisRemote(void) {
  chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
  chassis_cmd.ctrl_vec.vx = dr16.data.ch_r_y * RC_CHASSIS_VX_SCALE;
  chassis_cmd.ctrl_vec.vy = -dr16.data.ch_r_x * RC_CHASSIS_VY_SCALE;
  chassis_cmd.ctrl_vec.wz = -dr16.data.ch_l_x * RC_CHASSIS_WZ_SCALE;
}

#if defined(RC_AUTO_CTRL_DRY_RUN_ENABLED)
static void Rc_SetAutoDryRunCommands(void) {
  Rc_SetChassisRelax();

  Rc_SetPoleManual(0.0f, 0.0f);
  pole_cmd.mode = POLE_MODE_RELAX;
}
#endif

static bool Rc_ShouldExitAutoActionBySwitch(void) {
  return dr16.header.online &&
         (dr16.data.sw_l == DR16_SW_MID || dr16.data.sw_l == DR16_SW_DOWN) &&
         (last_sw_r == DR16_SW_UP || last_sw_r == DR16_SW_DOWN) &&
         dr16.data.sw_r == DR16_SW_MID;
}

static bool Rc_ShouldUsePcCommand(void) {
  return MrlinkPc_IsPCControlMode() &&
         dr16.data.sw_l == DR16_SW_UP &&
         dr16.data.sw_r == DR16_SW_UP;
}

static bool rc_pc_behavior_was_active = false;

static RcControlPage_t Rc_PageForBehavior(RcBehavior_t behavior) {
  switch (behavior) {
    case RC_BEHAVIOR_DRIVE:
      return RC_CONTROL_PAGE_DRIVE;
    case RC_BEHAVIOR_AUTO_200_UP_STANDBY:
      return RC_CONTROL_PAGE_AUTO_200_UP;
    case RC_BEHAVIOR_AUTO_200_DOWN_STANDBY:
      return RC_CONTROL_PAGE_AUTO_200_DOWN;
    case RC_BEHAVIOR_PC:
      return RC_CONTROL_PAGE_PC;
    case RC_BEHAVIOR_ARM_SIMPLE:
      return RC_CONTROL_PAGE_ARM_SIMPLE;
    case RC_BEHAVIOR_ORE_STORE:
      return RC_CONTROL_PAGE_ORE_STORE;
    case RC_BEHAVIOR_ROD_NEW:
      return RC_CONTROL_PAGE_ROD_NEW;
    case RC_BEHAVIOR_AUTO_ORE:
      return RC_CONTROL_PAGE_AUTO_ORE;
    case RC_BEHAVIOR_SAFE:
    default:
      return RC_CONTROL_PAGE_SAFE;
  }
}

static RcBehavior_t Rc_SelectMappedBehavior(void) {
  size_t map_count = 0u;
  const RcSwitchBehaviorMap_t *map = Rc_GetBehaviorMap(&map_count);

  if (!dr16.header.online) {
    return RC_BEHAVIOR_SAFE;
  }

  for (size_t index = 0u; index < map_count; ++index) {
    if (Rc_SwitchMapEntryMatches(&map[index])) {
      return map[index].behavior;
    }
  }

  return RC_BEHAVIOR_SAFE;
}

static bool Rc_BehaviorAllowsAutoCtrlOutput(RcBehavior_t behavior) {
  return behavior == RC_BEHAVIOR_AUTO_200_UP_STANDBY ||
         behavior == RC_BEHAVIOR_AUTO_200_DOWN_STANDBY;
}

static bool Rc_DebugPeriodicDue(uint32_t now_ms) {
  if (rc_debug_last_update_ms == 0u ||
      (uint32_t)(now_ms - rc_debug_last_update_ms) >= RC_DEBUG_UPDATE_PERIOD_MS) {
    rc_debug_last_update_ms = now_ms;
    return true;
  }
  return false;
}

static void Rc_ResetFrameDebugEvents(void) {
  g_pc_command_source = PC_COMMAND_SOURCE_RC;
  g_rc_control_debug.page = RC_CONTROL_PAGE_SAFE;
  g_rc_control_debug.reset_event = false;
  g_rc_control_debug.auto_200_start_event = false;
  g_rc_control_debug.auto_200_start_ok = false;
  g_rc_control_debug.auto_200_template = AUTO_CTRL_TEMPLATE_NONE;
  g_rc_control_debug.arm_simple_active = false;
  g_rc_control_debug.rod_active = false;
  g_rc_control_debug.ore_store_active = false;
  g_rc_control_debug.auto_ore_start_event = false;
  g_rc_control_debug.auto_ore_start_ok = false;
  g_rc_ore_store_debug.assume_home_event = false;
}

static void Rc_UpdateDebugSnapshot(void) {
  g_rc_control_debug.auto_ore_state = AutoOre_GetState(&auto_ore_ctrl);
  g_rc_control_debug.auto_ore_result = AutoOre_GetResult(&auto_ore_ctrl);
  g_rc_control_debug.auto_ore_fault = AutoOre_GetFault(&auto_ore_ctrl);
  g_rc_control_debug.auto_ore_action = auto_ore_ctrl.action;
  g_rc_control_debug.auto_ore_active_position =
      AutoOre_GetActivePosition(&auto_ore_ctrl);
  g_rc_control_debug.auto_ore_occupancy_mask =
      AutoOre_GetOccupancyMask(&auto_ore_ctrl);
  g_rc_control_debug.auto_ore_step_index = AutoOre_GetStepIndex(&auto_ore_ctrl);
  g_rc_control_debug.auto_rod_spearhead_state =
      AutoRodSpearhead_GetState(&auto_rod_spearhead_ctrl);
  g_rc_control_debug.auto_rod_spearhead_result =
      AutoRodSpearhead_GetResult(&auto_rod_spearhead_ctrl);
  g_rc_control_debug.auto_rod_spearhead_fault =
      AutoRodSpearhead_GetFault(&auto_rod_spearhead_ctrl);
  g_rc_control_debug.auto_rod_spearhead_action =
      AutoRodSpearhead_GetAction(&auto_rod_spearhead_ctrl);
  g_rc_control_debug.auto_rod_spearhead_step_index =
      AutoRodSpearhead_GetStepIndex(&auto_rod_spearhead_ctrl);
  g_rc_control_debug.auto_sick_correct_state =
      AutoSickCorrect_GetState(&auto_sick_correct_ctrl);
  g_rc_control_debug.auto_sick_correct_result =
      AutoSickCorrect_GetResult(&auto_sick_correct_ctrl);
  g_rc_control_debug.auto_sick_correct_fault =
      AutoSickCorrect_GetFault(&auto_sick_correct_ctrl);
  g_rc_control_debug.auto_sick_correct_action =
      AutoSickCorrect_GetAction(&auto_sick_correct_ctrl);
  g_rc_control_debug.auto_sick_correct_step_index =
      AutoSickCorrect_GetStepIndex(&auto_sick_correct_ctrl);
}

static void Rc_HandleResetEvent(void) {
  if (dr16.header.online && dr16.data.sw_l == DR16_SW_UP &&
      dr16.data.sw_r == DR16_SW_DOWN && last_sw_r != DR16_SW_DOWN &&
      last_sw_r != DR16_SW_ERR) {
    reset = true;
    g_rc_control_debug.reset_event = true;
  }
}

static void Rc_AbortAutoCtrlIfBusy(void) {
  if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl)) {
    AutoCtrl_Abort(&auto_ctrl);
  }
}

static void Rc_AbortAutoActionsBySwitch(void) {
  if (!Rc_ShouldExitAutoActionBySwitch()) {
    return;
  }

  g_auto_ore_debug.force_output_enable = false;

  if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl)) {
    Rc_LatchAutoCtrlCurrentTargets();
    AutoCtrl_Abort(&auto_ctrl);
  }

  if (auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl)) {
    Rc_LatchAutoOreCurrentTargets();
    Task_AutoOreAbort();
  }

  if (Task_AutoRodSpearheadIsBusy()) {
    auto_rod_spearhead_hold_after_finish = false;
    Task_AutoRodSpearheadAbort();
  }

  if (Task_AutoSickCorrectIsBusy()) {
    Rc_LatchAutoSickCorrectCurrentTargets();
    Task_AutoSickCorrectAbort();
  }
}

static bool Rc_AutoOreSwitchEdgeFromMid(void) {
  return dr16.header.online && dr16.data.sw_l == DR16_SW_DOWN &&
         last_sw_r == DR16_SW_MID && dr16.data.sw_r != DR16_SW_MID;
}

static void Rc_HandleBehaviorEvents(RcBehavior_t behavior) {
  if (behavior == RC_BEHAVIOR_AUTO_ORE && Rc_AutoOreSwitchEdgeFromMid()) {
    g_rc_control_debug.auto_ore_start_event = true;
    if (auto_ore_inited && !AutoOre_IsBusy(&auto_ore_ctrl)) {
      if (dr16.data.sw_r == DR16_SW_UP) {
        g_rc_control_debug.auto_ore_start_ok = Task_AutoOreStartStore();
      } else if (dr16.data.sw_r == DR16_SW_DOWN) {
        g_rc_control_debug.auto_ore_start_ok = Task_AutoOreStartRelease();
      }
    }
  }
}

static void Rc_PublishCommandsAndDebug(bool update_debug) {
  osMessageQueueReset(task_runtime.msgq.chassis.cmd);
  osMessageQueuePut(task_runtime.msgq.chassis.cmd, &chassis_cmd, 0, 0);
  osMessageQueueReset(task_runtime.msgq.pole.cmd);
  osMessageQueuePut(task_runtime.msgq.pole.cmd, &pole_cmd, 0, 0);
  osMessageQueueReset(task_runtime.msgq.arm_simple.cmd);
  osMessageQueuePut(task_runtime.msgq.arm_simple.cmd, &arm_simple_cmd, 0, 0);
  osMessageQueueReset(task_runtime.msgq.rod.cmd);
  osMessageQueuePut(task_runtime.msgq.rod.cmd, &rod_cmd, 0, 0);
  g_rc_ore_store_post_ret = Task_OreStorePostCommand(&ore_store_cmd);

  if (!update_debug) {
    return;
  }

  g_rc_ore_store_debug.online = dr16.header.online;
  g_rc_ore_store_debug.sw_l = dr16.data.sw_l;
  g_rc_ore_store_debug.sw_r = dr16.data.sw_r;
  g_rc_ore_store_debug.last_sw_l = last_sw_l;
  g_rc_ore_store_debug.last_sw_r = last_sw_r;
  g_rc_ore_store_debug.cmd_mode = ore_store_cmd.mode;
  g_rc_ore_store_debug.cmd_force_rehome = ore_store_cmd.force_rehome;
  g_rc_ore_store_debug.platform_target_rad = ore_store_cmd.platform_target_rad;
  g_rc_ore_store_debug.post_ret = g_rc_ore_store_post_ret;
}

static bool Rc_PrepareLocalAutoYawFeedback(void) {
  if (!isfinite(chassis_imu.eulr.yaw)) {
    return false;
  }

  if (!auto_ctrl_local_yaw_zero_initialized) {
    auto_ctrl_local_yaw_zero_rad = chassis_imu.eulr.yaw;
    auto_ctrl_local_yaw_zero_initialized = true;
  }

  AutoCtrl_SetYawSource(&auto_ctrl, AUTO_CTRL_YAW_SOURCE_STM32);
  AutoCtrl_SetYawZeroOffset(&auto_ctrl, 0.0f);

  auto_ctrl_feedback_t local_feedback = auto_ctrl.feedback;
  local_feedback.yaw_auto_rad = chassis_imu.eulr.yaw;
  AutoCtrl_SetFeedback(&auto_ctrl, &local_feedback);
  return true;
}

static float Rc_SelectLocalAutoTargetYawRad(float head_yaw_rad,
                                           auto_ctrl_travel_dir_e travel_dir) {
  (void)travel_dir;
  return AutoCtrlMath_NearestCardinalYawRad(head_yaw_rad);
}

static bool Rc_AutoCtrlTemplateIsAscend(auto_ctrl_template_e template_id) {
  return template_id == AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD ||
         template_id == AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD;
}

static bool Rc_StartAutoUpStepPickStore(void) {
  const RcAutoStepProfile_t profile = Rc_SelectAutoStepProfile();
  if (profile == RC_AUTO_STEP_PROFILE_FUSED_200) {
    return Task_AutoOreStartStepPickStoreAscend200Head();
  }
  if (profile == RC_AUTO_STEP_PROFILE_FUSED_UP_400_NORMAL_DOWN_400) {
    return Task_AutoOreStartStepPickStoreAscend400Head();
  }
  return false;
}

static bool Rc_StartAutoDownStepPickStore(void) {
  return Task_AutoOreStartStepPickStoreDescend200Head();
}

static void Rc_TryStartAutoCtrlBySwitch(uint32_t now_ms) {
  if (!dr16.header.online || !auto_ctrl_inited || AutoCtrl_IsBusy(&auto_ctrl) ||
      (auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl)) ||
      Task_AutoRodSpearheadIsBusy() || Task_AutoSickCorrectIsBusy()) {
    return;
  }

  const RcAutoStepProfile_t auto_step_profile = Rc_SelectAutoStepProfile();

  if (last_sw_l == DR16_SW_MID && last_sw_r == DR16_SW_MID &&
      dr16.data.sw_l == DR16_SW_MID && dr16.data.sw_r == DR16_SW_UP &&
      auto_step_profile != RC_AUTO_STEP_PROFILE_NONE) {
    g_rc_control_debug.auto_200_start_event = true;
    g_rc_control_debug.auto_200_template =
        (auto_step_profile == RC_AUTO_STEP_PROFILE_FUSED_UP_400_NORMAL_DOWN_400)
            ? AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD
            : AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD;
    if (!Rc_PrepareLocalAutoYawFeedback()) {
      g_rc_control_debug.auto_200_start_ok = false;
      return;
    }
    g_rc_control_debug.auto_200_start_ok = Rc_StartAutoUpStepPickStore();
    if (g_rc_control_debug.auto_200_start_ok) {
      g_auto_ore_debug.force_output_enable = true;
    }
    return;
  }

  if (last_sw_l == DR16_SW_MID && last_sw_r == DR16_SW_MID &&
      dr16.data.sw_l == DR16_SW_MID && dr16.data.sw_r == DR16_SW_DOWN &&
      auto_step_profile == RC_AUTO_STEP_PROFILE_FUSED_200) {
    g_rc_control_debug.auto_200_start_event = true;
    g_rc_control_debug.auto_200_template = AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD;
    if (!Rc_PrepareLocalAutoYawFeedback()) {
      g_rc_control_debug.auto_200_start_ok = false;
      return;
    }
    g_rc_control_debug.auto_200_start_ok = Rc_StartAutoDownStepPickStore();
    if (g_rc_control_debug.auto_200_start_ok) {
      g_auto_ore_debug.force_output_enable = true;
    }
    return;
  }

  const RcAutoCtrlStartConfig_t config = Rc_SelectAutoCtrlStartConfig();
  float target_yaw_rad = 0.0f;

  if (config.template_id != AUTO_CTRL_TEMPLATE_NONE) {
    g_rc_control_debug.auto_200_start_event = true;
    g_rc_control_debug.auto_200_template = config.template_id;
    if (!Rc_PrepareLocalAutoYawFeedback()) {
      g_rc_control_debug.auto_200_start_ok = false;
      return;
    }

    target_yaw_rad =
        Rc_SelectLocalAutoTargetYawRad(auto_ctrl.feedback.yaw_auto_rad,
                                       config.travel_dir);
    const auto_ctrl_sensor_mode_e sensor_mode =
        Rc_AutoCtrlTemplateIsAscend(config.template_id)
            ? AUTO_CTRL_SENSOR_MODE_YAW_ONLY
            : AUTO_CTRL_SENSOR_MODE_SICK_FRONT_AND_BOTTOM;
    g_rc_control_debug.auto_200_start_ok = AutoCtrl_StartTemplate(
        &auto_ctrl, config.template_id, config.travel_dir,
        target_yaw_rad, AUTO_CTRL_RC_YAW_TOLERANCE_RAD, sensor_mode, now_ms);
  }
} 

struct RcRuntimeInput {
  bool online() const { return dr16.header.online; }
};

template <typename Cmd>
struct RcStoreCommandSink {
  Cmd *target;

  bool operator()(const Cmd &cmd) const {
    if (target == nullptr) {
      return false;
    }
    *target = cmd;
    return true;
  }
};

template <RcCommandPlan_t... Plans>
struct RcPlanIn {
  bool operator()(const cmd::Context &) const {
    return ((rc_current_plan == Plans) || ...);
  }
};

struct RcPlanSafe {
  bool operator()(const cmd::Context &) const {
    return rc_current_plan == RC_CMD_PLAN_SAFE;
  }
};

#if RC_POLE_CH_RES_ENABLE
static bool Rc_CommandPlanUsesMappedPoleControl(void) {
  switch (rc_current_plan) {
    case RC_CMD_PLAN_DRIVE:
    case RC_CMD_PLAN_AUTO_STANDBY:
    case RC_CMD_PLAN_ARM_SIMPLE:
    case RC_CMD_PLAN_ORE_STORE:
    case RC_CMD_PLAN_ROD_NEW:
    case RC_CMD_PLAN_AUTO_ORE_STANDBY:
      return true;
    default:
      return false;
  }
}
#endif

static void Rc_ApplyPoleChResControlForCurrentPlan(void) {
#if RC_POLE_CH_RES_ENABLE
  if (Rc_CommandPlanUsesMappedPoleControl()) {
    Rc_ApplyPoleChResControl(rc_current_behavior);
  }
#else
#endif
}

static void Rc_PrepareSafePlanSideEffects(void) {
  if (auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl)) {
    Task_AutoOreAbort();
  }
}

static RcCommandPlan_t Rc_SelectCommandPlan(RcBehavior_t behavior) {
  rc_current_behavior = behavior;

  if (g_auto_ore_debug.force_output_enable) {
    if (!auto_ore_inited || !AutoOre_IsBusy(&auto_ore_ctrl)) {
      g_auto_ore_debug.force_output_enable = false;
    } else {
      g_rc_control_debug.page = RC_CONTROL_PAGE_AUTO_ORE;
      g_rc_control_debug.ore_store_active = true;
      g_rc_control_debug.arm_simple_active = true;
      g_auto_ore_debug.force_output_count++;
      return RC_CMD_PLAN_AUTO_ORE_OUTPUT;
    }
  }

  if (Task_AutoSickCorrectIsBusy()) {
    g_rc_control_debug.page = RC_CONTROL_PAGE_AUTO_ORE;
    return RC_CMD_PLAN_AUTO_SICK_CORRECT_OUTPUT;
  }

  if (Task_AutoRodSpearheadIsBusy()) {
    g_rc_control_debug.page = RC_CONTROL_PAGE_AUTO_ORE;
    g_rc_control_debug.rod_active = true;
    g_rc_control_debug.ore_store_active = true;
    if (!auto_rod_spearhead_was_busy) {
      Rc_LatchAutoRodSpearheadHoldTargets();
    }
    if (Task_AutoRodSpearheadIsPickupStep1() && behavior == RC_BEHAVIOR_PC &&
        Rc_ShouldUsePcCommand()) {
      return RC_CMD_PLAN_AUTO_ROD_STEP1_PC_OUTPUT;
    }
    return RC_CMD_PLAN_AUTO_ROD_OUTPUT;
  }

  if (behavior == RC_BEHAVIOR_SAFE) {
    Rc_PrepareSafePlanSideEffects();
    g_rc_control_debug.page = RC_CONTROL_PAGE_SAFE;
    return RC_CMD_PLAN_SAFE;
  }

  if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl) &&
      Rc_BehaviorAllowsAutoCtrlOutput(behavior)) {
    g_rc_control_debug.page = Rc_GetAutoCtrlPage(AutoCtrl_GetTemplate(&auto_ctrl));
    return RC_CMD_PLAN_AUTO_CTRL_OUTPUT;
  }

  if (auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl)) {
    if (behavior == RC_BEHAVIOR_AUTO_ORE) {
      g_rc_control_debug.ore_store_active = true;
      g_rc_control_debug.arm_simple_active = true;
      return RC_CMD_PLAN_AUTO_ORE_OUTPUT;
    }
    Rc_LatchAutoOreCurrentTargets();
    Task_AutoOreAbort();
  }

  if (!Rc_BehaviorAllowsAutoCtrlOutput(behavior) &&
      behavior != RC_BEHAVIOR_PC) {
    Rc_AbortAutoCtrlIfBusy();
  }

  if (behavior == RC_BEHAVIOR_PC && !Rc_ShouldUsePcCommand()) {
    Rc_AbortAutoCtrlIfBusy();
    Rc_PrepareSafePlanSideEffects();
    g_rc_control_debug.page = RC_CONTROL_PAGE_SAFE;
    return RC_CMD_PLAN_SAFE;
  }

  Rc_HandleBehaviorEvents(behavior);

  switch (behavior) {
    case RC_BEHAVIOR_DRIVE:
      g_rc_control_debug.page = RC_CONTROL_PAGE_DRIVE;
      return RC_CMD_PLAN_DRIVE;
    case RC_BEHAVIOR_AUTO_200_UP_STANDBY:
    case RC_BEHAVIOR_AUTO_200_DOWN_STANDBY:
      g_rc_control_debug.page = Rc_PageForBehavior(behavior);
      return RC_CMD_PLAN_AUTO_STANDBY;
    case RC_BEHAVIOR_PC:
      g_rc_control_debug.page = RC_CONTROL_PAGE_PC;
      g_pc_command_source = PC_COMMAND_SOURCE_PC;
      if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl)) {
        return RC_CMD_PLAN_PC_AUTO_CTRL;
      }
      return RC_CMD_PLAN_PC;
    case RC_BEHAVIOR_ARM_SIMPLE:
      g_rc_control_debug.page = RC_CONTROL_PAGE_ARM_SIMPLE;
      g_rc_control_debug.arm_simple_active = true;
      return RC_CMD_PLAN_ARM_SIMPLE;
    case RC_BEHAVIOR_ORE_STORE:
      g_rc_control_debug.page = RC_CONTROL_PAGE_ORE_STORE;
      g_rc_control_debug.ore_store_active = true;
      return RC_CMD_PLAN_ORE_STORE;
    case RC_BEHAVIOR_ROD_NEW:
      g_rc_control_debug.page = RC_CONTROL_PAGE_ROD_NEW;
      g_rc_control_debug.rod_active = true;
      return RC_CMD_PLAN_ROD_NEW;
    case RC_BEHAVIOR_AUTO_ORE:
      g_rc_control_debug.page = RC_CONTROL_PAGE_AUTO_ORE;
      return RC_CMD_PLAN_AUTO_ORE_STANDBY;
    case RC_BEHAVIOR_SAFE:
    default:
      Rc_PrepareSafePlanSideEffects();
      g_rc_control_debug.page = RC_CONTROL_PAGE_SAFE;
      return RC_CMD_PLAN_SAFE;
  }
}

struct RcChassisSafeRoute {
  bool operator()(cmd::Context &, Chassis_CMD_t &out) const {
    Rc_SetChassisRelax();
    out = chassis_cmd;
    return true;
  }
};

struct RcChassisKeepRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &,
                  Chassis_CMD_t &out) const {
    out = chassis_cmd;
    return true;
  }
};

struct RcChassisHoldRoute {
  bool operator()(cmd::Context &, Chassis_CMD_t &out) const {
    Rc_SetChassisHold();
    out = chassis_cmd;
    return true;
  }
};

struct RcChassisRemoteRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &, Chassis_CMD_t &out) const {
    Rc_SetChassisRemote();
    out = chassis_cmd;
    return true;
  }
};

struct RcChassisPcRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &, Chassis_CMD_t &out) const {
    const PC_ChassisCMD_t *pc_chassis_cmd = MrlinkPc_GetChassisCMD();
    if (pc_chassis_cmd == NULL) {
      Rc_SetChassisRelax();
    } else {
      chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
      chassis_cmd.ctrl_vec.vx = pc_chassis_cmd->vx;
      chassis_cmd.ctrl_vec.vy = pc_chassis_cmd->vy;
      chassis_cmd.ctrl_vec.wz = pc_chassis_cmd->wz;
    }
    out = chassis_cmd;
    return true;
  }
};

struct RcChassisAutoCtrlRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &, Chassis_CMD_t &out) const {
#if AUTO_CTRL_OUTPUT_ENABLE
    out = auto_ctrl.chassis_cmd;
#else
    Rc_SetAutoDryRunCommands();
    out = chassis_cmd;
#endif
    return true;
  }
};

struct RcChassisAutoOreRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &, Chassis_CMD_t &out) const {
    const Chassis_CMD_t *auto_chassis_cmd =
        AutoOre_GetChassisCommand(&auto_ore_ctrl);
    if (auto_chassis_cmd != NULL) {
      out = *auto_chassis_cmd;
    } else if (auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl) &&
               auto_ore_ctrl.action == AUTO_ORE_ACTION_RELEASE) {
      Rc_SetChassisHold();
      out = chassis_cmd;
    } else {
      Rc_SetChassisRelax();
      out = chassis_cmd;
    }
    return true;
  }
};

struct RcChassisAutoSickCorrectRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &,
                  Chassis_CMD_t &out) const {
    Rc_SetChassisHold();
    const Chassis_CMD_t *auto_chassis_cmd =
        Task_AutoSickCorrectGetChassisCommand();
    if (auto_chassis_cmd != NULL) {
      out = *auto_chassis_cmd;
    } else {
      out = chassis_cmd;
    }
    return true;
  }
};

struct RcPoleSafeRoute {
  bool operator()(cmd::Context &, Pole_CMD_t &out) const {
    Rc_SetPoleAuto(0.0f, 0.0f);
    out = pole_cmd;
    return true;
  }
};

struct RcPoleKeepRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &, Pole_CMD_t &out) const {
    out = pole_cmd;
    return true;
  }
};

struct RcPoleHoldRoute {
  bool operator()(cmd::Context &, Pole_CMD_t &out) const {
    Rc_SetPoleHold();
    Rc_ApplyPoleChResControlForCurrentPlan();
    out = pole_cmd;
    return true;
  }
};

struct RcPoleDriveRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &, Pole_CMD_t &out) const {
    Rc_SetPoleManual(dr16.data.ch_l_y, dr16.data.ch_l_y);
    Rc_ApplyPoleChResControlForCurrentPlan();
    out = pole_cmd;
    return true;
  }
};

struct RcPolePcRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &, Pole_CMD_t &out) const {
    if (!Rc_SetPolePcCommand(false)) {
      Rc_SetPoleHold();
    }
    out = pole_cmd;
    return true;
  }
};

struct RcPoleAutoCtrlRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &, Pole_CMD_t &out) const {
#if AUTO_CTRL_OUTPUT_ENABLE
    out = auto_ctrl.pole_cmd;
#else
    Rc_SetAutoDryRunCommands();
    out = pole_cmd;
#endif
    return true;
  }
};

struct RcPoleAutoOreRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &, Pole_CMD_t &out) const {
    const Pole_CMD_t *auto_pole_cmd = AutoOre_GetPoleCommand(&auto_ore_ctrl);
    if (auto_pole_cmd != NULL) {
      out = *auto_pole_cmd;
    } else {
      Rc_SetPoleHold();
      out = pole_cmd;
    }
    return true;
  }
};

struct RcPoleAutoSickCorrectRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &, Pole_CMD_t &out) const {
    const Pole_CMD_t *auto_pole_cmd = Task_AutoSickCorrectGetPoleCommand();
    if (auto_pole_cmd != NULL) {
      out = *auto_pole_cmd;
    } else {
      Rc_SetPoleHold();
      out = pole_cmd;
    }
    return true;
  }
};

struct RcPoleAutoRodRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &, Pole_CMD_t &out) const {
    Rc_SetPoleAutoTarget(0.0f, 0.0f);
    out = pole_cmd;
    return true;
  }
};

struct RcArmSimpleSafeRoute {
  bool operator()(cmd::Context &, ArmSimple_CMD_t &out) const {
    Rc_SetArmSimpleRelax();
    out = arm_simple_cmd;
    return true;
  }
};

struct RcArmSimpleHoldRoute {
  bool operator()(cmd::Context &, ArmSimple_CMD_t &out) const {
    Rc_SetArmSimpleHold();
    out = arm_simple_cmd;
    return true;
  }
};

struct RcArmSimpleKeepRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &,
                  ArmSimple_CMD_t &out) const {
    out = arm_simple_cmd;
    return true;
  }
};

struct RcArmSimpleOperatorRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &,
                  ArmSimple_CMD_t &out) const {
    Rc_SetArmSimpleOperator(RC_ARM_FINE_SCALE);
    out = arm_simple_cmd;
    return true;
  }
};

struct RcArmSimplePcRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &,
                  ArmSimple_CMD_t &out) const {
    const PC_AbstractPositionCMD_t *pc_abstract_cmd =
        MrlinkPc_GetAbstractPositionCMD();
    const PC_ArmSimpleCMD_t *pc_arm_simple_cmd =
        MrlinkPc_HasArmSimpleCMD() ? MrlinkPc_GetArmSimpleCMD() : NULL;
    if (Rc_SetArmSimplePcAbstractCommand(pc_abstract_cmd)) {
      out = arm_simple_cmd;
      return true;
    } else if (pc_arm_simple_cmd != NULL) {
      arm_simple_cmd.mode = (ArmSimple_Mode_t)pc_arm_simple_cmd->mode;
      arm_simple_cmd.point_mode =
          (ArmSimple_PointMode_t)pc_arm_simple_cmd->point_mode;
      arm_simple_cmd.suction = arm_simple_suction_latched;
      arm_simple_cmd.target_joint.joint1 =
          pc_arm_simple_cmd->target_joint1_rad;
      arm_simple_cmd.target_joint.joint2 =
          pc_arm_simple_cmd->target_joint2_rad;
      arm_simple_cmd.joint1_vel = 0.0f;
      arm_simple_target_initialized = true;
    } else {
      Rc_SetArmSimpleStandby();
    }
    out = arm_simple_cmd;
    return true;
  }
};

struct RcArmSimpleAutoOreRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &,
                  ArmSimple_CMD_t &out) const {
    const ArmSimple_CMD_t *auto_arm_cmd =
        AutoOre_GetArmCommand(&auto_ore_ctrl);
    if (auto_arm_cmd != NULL) {
      out = *auto_arm_cmd;
      arm_simple_suction_latched = out.suction;
      arm_simple_target_initialized = true;
    } else {
      Rc_SetArmSimpleHold();
      out = arm_simple_cmd;
    }
    return true;
  }
};

struct RcOreStoreSafeRoute {
  bool operator()(cmd::Context &, OreStore_CMD_t &out) const {
    Rc_SetOreStoreRelax();
    out = ore_store_cmd;
    return true;
  }
};

struct RcOreStoreHoldRoute {
  bool operator()(cmd::Context &, OreStore_CMD_t &out) const {
    Rc_SetOreStoreHold();
    out = ore_store_cmd;
    return true;
  }
};

struct RcOreStoreKeepRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &,
                  OreStore_CMD_t &out) const {
    out = ore_store_cmd;
    return true;
  }
};

struct RcOreStoreManualRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &,
                  OreStore_CMD_t &out) const {
    Rc_SetOreStoreActiveManual();
    out = ore_store_cmd;
    return true;
  }
};

struct RcOreStorePcRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &,
                  OreStore_CMD_t &out) const {
    const PC_OreStoreCMD_t *pc_ore_store_cmd =
        MrlinkPc_GetOreStoreCMD();
    if (pc_ore_store_cmd != NULL) {
      if (Task_OreStorePowerOnHomeInProgress()) {
        Rc_SetOreStoreRelax();
      } else {
        ore_store_cmd.mode = (OreStore_Mode_t)pc_ore_store_cmd->mode;
        ore_store_cmd.force_rehome = pc_ore_store_cmd->force_rehome != 0u;
        ore_store_cmd.platform_target_rad =
            Rc_ClampOreStoreTarget(ORE_STORE_AXIS_PLATFORM,
                                   pc_ore_store_cmd->platform_target_rad);
        ore_store_active_initialized =
            ore_store_cmd.mode == ORE_STORE_MODE_ACTIVE;
      }
    } else {
      Rc_SetOreStoreHold();
    }
    out = ore_store_cmd;
    return true;
  }
};

struct RcOreStoreAutoOreRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &,
                  OreStore_CMD_t &out) const {
    const OreStore_CMD_t *auto_ore_cmd =
        AutoOre_GetOreStoreCommand(&auto_ore_ctrl);
    if (auto_ore_cmd != NULL) {
      out = *auto_ore_cmd;
      ore_store_active_initialized = out.mode == ORE_STORE_MODE_ACTIVE;
    } else {
      Rc_SetOreStoreHold();
      out = ore_store_cmd;
    }
    return true;
  }
};

struct RcOreStoreAutoRodRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &,
                  OreStore_CMD_t &out) const {
    const OreStore_CMD_t *auto_rod_ore_cmd =
        Task_AutoRodSpearheadGetOreStoreCommand();
    if (auto_rod_ore_cmd != NULL) {
      out = *auto_rod_ore_cmd;
      ore_store_active_initialized = out.mode == ORE_STORE_MODE_ACTIVE;
    } else {
      Rc_SetOreStoreHold();
      out = ore_store_cmd;
    }
    return true;
  }
};

struct RcRodNewSafeRoute {
  bool operator()(cmd::Context &, RodNew_CMD_t &out) const {
    if (auto_rod_spearhead_hold_after_finish) {
      Rc_SetRodHold();
    } else {
      Rc_SetRodRelax();
    }
    out = rod_cmd;
    return true;
  }
};

struct RcRodNewHoldRoute {
  bool operator()(cmd::Context &, RodNew_CMD_t &out) const {
    Rc_SetRodHold();
    out = rod_cmd;
    return true;
  }
};

struct RcRodNewKeepRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &, RodNew_CMD_t &out) const {
    out = rod_cmd;
    return true;
  }
};

struct RcRodNewOperatorRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &, RodNew_CMD_t &out) const {
    auto_rod_spearhead_hold_after_finish = false;
    Rc_SetRodOperator();
    out = rod_cmd;
    return true;
  }
};

struct RcRodNewPcRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &, RodNew_CMD_t &out) const {
    const PC_RodNewCMD_t *pc_rod_new_cmd =
        MrlinkPc_HasRodNewCMD() ? MrlinkPc_GetRodNewCMD() : NULL;
    if (pc_rod_new_cmd != NULL) {
      auto_rod_spearhead_hold_after_finish = false;
      rod_cmd.mode = (RodNew_Mode_t)pc_rod_new_cmd->mode;
      rod_cmd.pose = (RodNew_Pose_t)pc_rod_new_cmd->pose;
      rod_cmd.grip = rod_grip_latched;
      rod_cmd.target_angle_rad =
          Rc_ClampRodNewTarget(pc_rod_new_cmd->target_angle_rad);
      rod_target_angle_latched_rad = rod_cmd.target_angle_rad;
    } else {
      Rc_SetRodHold();
    }
    out = rod_cmd;
    return true;
  }
};

struct RcRodNewAutoRoute {
  bool operator()(const RcRuntimeInput &, cmd::Context &, RodNew_CMD_t &out) const {
    const RodNew_CMD_t *auto_rod_cmd = Task_AutoRodSpearheadGetCommand();
    if (auto_rod_cmd != NULL) {
      auto_rod_spearhead_hold_after_finish = false;
      out = *auto_rod_cmd;
      rod_grip_latched = out.grip;
      rod_target_angle_latched_rad = out.target_angle_rad;
    } else {
      Rc_SetRodHold();
      out = rod_cmd;
    }
    return true;
  }
};

static void Rc_ConfigureCmdCenter(void) {
  if (rc_cmd_center_configured) {
    return;
  }

  rc_cmd_center
      .input<RcRuntimeInput>("rc_runtime")
      .global()
      .safety<RcPlanSafe>();

  rc_cmd_center
      .output<Chassis_CMD_t>("chassis",
                             RcStoreCommandSink<Chassis_CMD_t>{&chassis_cmd})
      .safe<RcChassisSafeRoute>()
      .arbitrate<cmd::HighestPriority>()
      .routes(
          cmd::from<RcRuntimeInput, RcChassisRemoteRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_DRIVE> >()
              .priority(cmd::Priority::Manual),
          cmd::from<RcRuntimeInput, RcChassisPcRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_PC,
                     RC_CMD_PLAN_AUTO_ROD_STEP1_PC_OUTPUT> >()
              .priority(cmd::Priority::Remote),
          cmd::from<RcRuntimeInput, RcChassisAutoCtrlRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_AUTO_CTRL_OUTPUT,
                             RC_CMD_PLAN_PC_AUTO_CTRL> >()
              .priority(cmd::Priority::Auto),
          cmd::from<RcRuntimeInput, RcChassisAutoOreRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_AUTO_ORE_OUTPUT> >()
              .priority(cmd::Priority::CriticalAuto),
          cmd::from<RcRuntimeInput, RcChassisAutoSickCorrectRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_AUTO_SICK_CORRECT_OUTPUT> >()
              .priority(cmd::Priority::CriticalAuto),
          cmd::from<RcRuntimeInput, RcChassisSafeRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_AUTO_STANDBY,
                             RC_CMD_PLAN_ARM_SIMPLE,
                             RC_CMD_PLAN_ORE_STORE,
                             RC_CMD_PLAN_ROD_NEW,
                             RC_CMD_PLAN_AUTO_ORE_STANDBY> >()
              .priority(cmd::Priority::Fallback),
          cmd::from<RcRuntimeInput, RcChassisHoldRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_AUTO_ROD_OUTPUT> >()
              .priority(cmd::Priority::Fallback));

  rc_cmd_center
      .output<Pole_CMD_t>("pole", RcStoreCommandSink<Pole_CMD_t>{&pole_cmd})
      .safe<RcPoleSafeRoute>()
      .hold<RcPoleHoldRoute>()
      .arbitrate<cmd::HighestPriority>()
      .routes(
          cmd::from<RcRuntimeInput, RcPoleDriveRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_DRIVE> >()
              .priority(cmd::Priority::Manual),
          cmd::from<RcRuntimeInput, RcPolePcRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_PC,
                     RC_CMD_PLAN_AUTO_ROD_STEP1_PC_OUTPUT> >()
              .priority(cmd::Priority::Remote),
          cmd::from<RcRuntimeInput, RcPoleAutoCtrlRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_AUTO_CTRL_OUTPUT,
                             RC_CMD_PLAN_PC_AUTO_CTRL> >()
              .priority(cmd::Priority::Auto),
          cmd::from<RcRuntimeInput, RcPoleAutoOreRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_AUTO_ORE_OUTPUT> >()
              .priority(cmd::Priority::CriticalAuto),
          cmd::from<RcRuntimeInput, RcPoleAutoSickCorrectRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_AUTO_SICK_CORRECT_OUTPUT> >()
              .priority(cmd::Priority::CriticalAuto),
          cmd::from<RcRuntimeInput, RcPoleAutoRodRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_AUTO_ROD_OUTPUT> >()
              .priority(cmd::Priority::Auto),
          cmd::from<RcRuntimeInput, RcPoleHoldRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_AUTO_STANDBY,
                             RC_CMD_PLAN_ARM_SIMPLE,
                             RC_CMD_PLAN_ORE_STORE,
                             RC_CMD_PLAN_ROD_NEW,
                             RC_CMD_PLAN_AUTO_ORE_STANDBY> >()
              .priority(cmd::Priority::Fallback));

  rc_cmd_center
      .output<ArmSimple_CMD_t>(
          "arm_simple", RcStoreCommandSink<ArmSimple_CMD_t>{&arm_simple_cmd})
      .safe<RcArmSimpleSafeRoute>()
      .hold<RcArmSimpleHoldRoute>()
      .arbitrate<cmd::HighestPriority>()
      .routes(
          cmd::from<RcRuntimeInput, RcArmSimpleOperatorRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_ARM_SIMPLE> >()
              .priority(cmd::Priority::Manual),
          cmd::from<RcRuntimeInput, RcArmSimplePcRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_PC> >()
              .priority(cmd::Priority::Remote),
          cmd::from<RcRuntimeInput, RcArmSimpleAutoOreRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_AUTO_ORE_OUTPUT> >()
              .priority(cmd::Priority::CriticalAuto),
          cmd::from<RcRuntimeInput, RcArmSimpleHoldRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_DRIVE,
                             RC_CMD_PLAN_AUTO_STANDBY,
                             RC_CMD_PLAN_ORE_STORE,
                             RC_CMD_PLAN_ROD_NEW,
                             RC_CMD_PLAN_AUTO_ORE_STANDBY,
                             RC_CMD_PLAN_AUTO_CTRL_OUTPUT,
                             RC_CMD_PLAN_PC_AUTO_CTRL,
                             RC_CMD_PLAN_AUTO_ROD_OUTPUT,
                     RC_CMD_PLAN_AUTO_ROD_STEP1_PC_OUTPUT,
                             RC_CMD_PLAN_AUTO_SICK_CORRECT_OUTPUT> >()
              .priority(cmd::Priority::Fallback));

  rc_cmd_center
      .output<OreStore_CMD_t>("ore_store",
                              RcStoreCommandSink<OreStore_CMD_t>{&ore_store_cmd})
      .safe<RcOreStoreSafeRoute>()
      .hold<RcOreStoreHoldRoute>()
      .arbitrate<cmd::HighestPriority>()
      .routes(
          cmd::from<RcRuntimeInput, RcOreStoreManualRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_ORE_STORE> >()
              .priority(cmd::Priority::Manual),
          cmd::from<RcRuntimeInput, RcOreStorePcRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_PC> >()
              .priority(cmd::Priority::Remote),
          cmd::from<RcRuntimeInput, RcOreStoreAutoOreRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_AUTO_ORE_OUTPUT> >()
              .priority(cmd::Priority::CriticalAuto),
          cmd::from<RcRuntimeInput, RcOreStoreAutoRodRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_AUTO_ROD_OUTPUT,
                     RC_CMD_PLAN_AUTO_ROD_STEP1_PC_OUTPUT> >()
              .priority(cmd::Priority::CriticalAuto),
          cmd::from<RcRuntimeInput, RcOreStoreHoldRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_DRIVE,
                             RC_CMD_PLAN_AUTO_STANDBY,
                             RC_CMD_PLAN_ARM_SIMPLE,
                             RC_CMD_PLAN_ROD_NEW,
                             RC_CMD_PLAN_AUTO_ORE_STANDBY,
                             RC_CMD_PLAN_AUTO_CTRL_OUTPUT,
                             RC_CMD_PLAN_PC_AUTO_CTRL,
                             RC_CMD_PLAN_AUTO_SICK_CORRECT_OUTPUT> >()
              .priority(cmd::Priority::Fallback));

  rc_cmd_center
      .output<RodNew_CMD_t>("rod", RcStoreCommandSink<RodNew_CMD_t>{&rod_cmd})
      .safe<RcRodNewSafeRoute>()
      .hold<RcRodNewHoldRoute>()
      .arbitrate<cmd::HighestPriority>()
      .routes(
          cmd::from<RcRuntimeInput, RcRodNewOperatorRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_ROD_NEW> >()
              .priority(cmd::Priority::Manual),
          cmd::from<RcRuntimeInput, RcRodNewPcRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_PC> >()
              .priority(cmd::Priority::Remote),
          cmd::from<RcRuntimeInput, RcRodNewAutoRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_AUTO_ROD_OUTPUT,
                     RC_CMD_PLAN_AUTO_ROD_STEP1_PC_OUTPUT> >()
              .priority(cmd::Priority::CriticalAuto),
          cmd::from<RcRuntimeInput, RcRodNewHoldRoute>()
              .when<RcPlanIn<RC_CMD_PLAN_DRIVE,
                             RC_CMD_PLAN_AUTO_STANDBY,
                             RC_CMD_PLAN_ARM_SIMPLE,
                             RC_CMD_PLAN_ORE_STORE,
                             RC_CMD_PLAN_AUTO_ORE_STANDBY,
                             RC_CMD_PLAN_AUTO_CTRL_OUTPUT,
                             RC_CMD_PLAN_PC_AUTO_CTRL,
                             RC_CMD_PLAN_AUTO_ORE_OUTPUT,
                             RC_CMD_PLAN_AUTO_SICK_CORRECT_OUTPUT> >()
              .priority(cmd::Priority::Fallback));

  rc_cmd_center_configured = true;
}
/* USER STRUCT END */
 
/* Exported functions ------------------------------------------------------- */
void RcCmdCenterApp_Init(void) {
  Rc_SetArmSimpleRelax();
  Rc_SetOreStoreRelax();
  Rc_ConfigureCmdCenter();
}

void RcCmdCenterApp_Update(uint32_t now_ms) {
  RcBehavior_t behavior;
  const bool update_debug = Rc_DebugPeriodicDue(now_ms);

  Rc_ResetFrameDebugEvents();
  if (update_debug) {
    Rc_UpdateDebugSnapshot();
  }
  Rc_HandleResetEvent();

  Rc_LatchFinishedAutoTargets();

  Rc_TryStartAutoCtrlBySwitch(now_ms);

  Rc_AbortAutoActionsBySwitch();

  behavior = Rc_SelectMappedBehavior();
  rc_current_plan = Rc_SelectCommandPlan(behavior);
  const bool pc_behavior_active = (behavior == RC_BEHAVIOR_PC);
  if (pc_behavior_active && !rc_pc_behavior_was_active) {
    (void)MrlinkPc_RequestStartMatch(1u);
  }
  rc_pc_behavior_was_active = pc_behavior_active;
  rc_cmd_center.tick(now_ms).publish();
  Rc_PublishCommandsAndDebug(update_debug);
  Rc_UpdateAutoBusyHistory();
  last_sw_l = dr16.data.sw_l;
  last_sw_r = dr16.data.sw_r;
}
