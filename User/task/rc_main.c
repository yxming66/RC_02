/*
    rc_main Task
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
#define RC_ROD_NEW_TARGET_MIN_RAD (-2.356f)
#define RC_ROD_NEW_TARGET_MAX_RAD (2.356f)
#define RC_MAPPING_PRESET_DEFAULT (0u)
#define RC_MAPPING_PRESET_PC_FIRST (1u)
#ifndef RC_MAPPING_ACTIVE_PRESET
#define RC_MAPPING_ACTIVE_PRESET RC_MAPPING_PRESET_DEFAULT
#endif
#ifndef RC_LEFT_DOWN_MAPPING_AUTO_ORE
#define RC_LEFT_DOWN_MAPPING_AUTO_ORE (0u) // 0单独控制上层，1一键存取
#endif
#ifndef RC_MANUAL_IO_CH_THRESHOLD
#define RC_MANUAL_IO_CH_THRESHOLD (0.90f)

#define RC_CHASSIS_VX_SCALE (2.0f)
#define RC_CHASSIS_VY_SCALE (2.0f)
#define RC_CHASSIS_WZ_SCALE (3.0f)

#endif

/* USER STRUCT BEGIN */
DR16_t dr16;
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
  volatile uint8_t auto_rod_spearhead_step_index;
} RcControlDebug_t;

volatile RcControlDebug_t g_rc_control_debug = {0};

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
static RodNew_GripState_t rod_grip_latched = ROD_NEW_GRIP_RELEASE;
static float rod_target_angle_latched_rad = 0.0f;
static Suction_State_t arm_simple_suction_latched = SUCTION_OFF;

typedef struct {
  auto_ctrl_template_e template_id;
  auto_ctrl_travel_dir_e travel_dir;
} RcAutoCtrlStartConfig_t;

#if RC_MAPPING_ACTIVE_PRESET == RC_MAPPING_PRESET_PC_FIRST
static const RcSwitchBehaviorMap_t rc_behavior_map_active[] = {
  {DR16_SW_MID, DR16_SW_UP, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_AUTO_200_UP_STANDBY},
  {DR16_SW_MID, DR16_SW_DOWN, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_AUTO_200_DOWN_STANDBY},
  {DR16_SW_MID, DR16_SW_MID, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_DRIVE},
#if RC_LEFT_DOWN_MAPPING_AUTO_ORE
  {DR16_SW_DOWN, DR16_SW_UP, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_AUTO_ORE},
  {DR16_SW_DOWN, DR16_SW_MID, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_ARM_SIMPLE},
  {DR16_SW_DOWN, DR16_SW_DOWN, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_AUTO_ORE},
#else
  {DR16_SW_DOWN, DR16_SW_UP, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_PC},
  {DR16_SW_DOWN, DR16_SW_MID, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_ARM_SIMPLE},
  {DR16_SW_DOWN, DR16_SW_DOWN, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_ROD_NEW},
#endif
};
#else
static const RcSwitchBehaviorMap_t rc_behavior_map_active[] = {
  {DR16_SW_MID, DR16_SW_UP, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_AUTO_200_UP_STANDBY},
  {DR16_SW_MID, DR16_SW_DOWN, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_AUTO_200_DOWN_STANDBY},
  {DR16_SW_MID, DR16_SW_MID, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_DRIVE},
#if RC_LEFT_DOWN_MAPPING_AUTO_ORE
  {DR16_SW_DOWN, DR16_SW_UP, DR16_SW_ERR, DR16_SW_ERR,
   RC_BEHAVIOR_AUTO_ORE},
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
  rod_cmd.pose = ROD_NEW_POSE_STANDBY;
  rod_cmd.grip = rod_grip_latched;
  rod_cmd.target_angle_rad = rod_target_angle_latched_rad;
}

static void Rc_SetChassisRelax(void) {
  chassis_cmd.mode = CHASSIS_MODE_RELAX;
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

  if (dr16.data.sw_r == DR16_SW_UP) {
    config.template_id = AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD;
  } else if (dr16.data.sw_r == DR16_SW_DOWN) {
    config.template_id = AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD;
  } 

  return config;
}

static RcControlPage_t Rc_GetAutoCtrlPage(auto_ctrl_template_e template_id) {
  switch (template_id) {
    case AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD:
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
    Rc_ApplyOreStoreDeadband(dr16.data.ch_r_y) *
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
  pole_cmd.lift[0] = left;
  pole_cmd.lift[1] = right;
  pole_cmd.auto_target_enable[0] = false;
  pole_cmd.auto_target_enable[1] = false;
  pole_cmd.auto_target_lift[0] = 0.0f;
  pole_cmd.auto_target_lift[1] = 0.0f;
  pole_cmd.auto_lift_speed[0] = 0.0f;
  pole_cmd.auto_lift_speed[1] = 0.0f;
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

static void Rc_SetPoleHold(void) {
  if (pole_cmd.mode == POLE_MODE_ACTIVE) {
    pole_cmd.lift[0] = 0.0f;
    pole_cmd.lift[1] = 0.0f;
    return;
  }

  Rc_SetPoleManual(0.0f, 0.0f);
}

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
  if (Task_ChassisMainGetPoleHoldCommand(&hold_cmd)) {
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

static void Rc_LatchFinishedAutoTargets(void) {
  const bool auto_ctrl_busy_now =
      auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl);
  const bool auto_ore_busy_now =
      auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl);

  if (auto_ctrl_was_busy && !auto_ctrl_busy_now) {
    Rc_LatchAutoCtrlCurrentTargets();
  }
  if (auto_ore_was_busy && !auto_ore_busy_now) {
    Rc_LatchAutoOreCurrentTargets();
  }
}

static void Rc_UpdateAutoBusyHistory(void) {
  auto_ctrl_was_busy = auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl);
  auto_ore_was_busy = auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl);
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

static bool Rc_ShouldExitAutoCtrlBySwitch(void) {
  return dr16.header.online &&
         (dr16.data.sw_l == DR16_SW_MID || dr16.data.sw_l == DR16_SW_DOWN) &&
         (last_sw_r == DR16_SW_UP || last_sw_r == DR16_SW_DOWN) &&
         dr16.data.sw_r == DR16_SW_MID;
}

static bool Rc_ShouldUsePcCommand(void) {
  return MrlinkPc_IsPCControlMode() &&
         dr16.data.sw_l != DR16_SW_UP;
}

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

  if (!dr16.header.online || dr16.data.sw_l == DR16_SW_UP) {
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

static void Rc_ResetFrameDebug(void) {
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
    g_rc_control_debug.auto_rod_spearhead_step_index =
      AutoRodSpearhead_GetStepIndex(&auto_rod_spearhead_ctrl);
  g_rc_ore_store_debug.assume_home_event = false;
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

static void Rc_ApplySafeBehavior(void) {
  g_rc_control_debug.page = RC_CONTROL_PAGE_SAFE;
  if (auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl)) {
    Task_AutoOreAbort();
  }
  if (Task_AutoRodSpearheadIsBusy()) {
    Task_AutoRodSpearheadAbort();
  }
  Rc_SetChassisRelax();
  Rc_SetPoleAuto(0.0f, 0.0f);
  Rc_SetArmSimpleRelax();
  Rc_SetRodRelax();
  Rc_SetOreStoreRelax();
}

static void Rc_ApplyAutoOreOutputs(void) {
  Rc_SetChassisRelax();
  Rc_SetRodHold();

  const Chassis_CMD_t *auto_chassis_cmd =
      AutoOre_GetChassisCommand(&auto_ore_ctrl);
  if (auto_chassis_cmd != NULL) {
    chassis_cmd = *auto_chassis_cmd;
  }

  const Pole_CMD_t *auto_pole_cmd = AutoOre_GetPoleCommand(&auto_ore_ctrl);
  if (auto_pole_cmd != NULL) {
    pole_cmd = *auto_pole_cmd;
  } else {
    Rc_SetPoleHold();
  }

  const ArmSimple_CMD_t *auto_arm_cmd =
      AutoOre_GetArmCommand(&auto_ore_ctrl);
  if (auto_arm_cmd != NULL) {
    arm_simple_cmd = *auto_arm_cmd;
    arm_simple_suction_latched = arm_simple_cmd.suction;
    arm_simple_target_initialized = true;
  } else {
    Rc_SetArmSimpleHold();
  }

  const OreStore_CMD_t *auto_ore_cmd =
      AutoOre_GetOreStoreCommand(&auto_ore_ctrl);
  if (auto_ore_cmd != NULL) {
    ore_store_cmd = *auto_ore_cmd;
    ore_store_active_initialized = ore_store_cmd.mode == ORE_STORE_MODE_ACTIVE;
  } else {
    Rc_SetOreStoreHold();
  }

  g_rc_control_debug.ore_store_active = true;
  g_rc_control_debug.arm_simple_active = true;
}

static bool Rc_TryApplyAutoOreDebugOutputs(void) {
  if (!g_auto_ore_debug.force_output_enable) {
    return false;
  }
  if (!auto_ore_inited || !AutoOre_IsBusy(&auto_ore_ctrl)) {
    g_auto_ore_debug.force_output_enable = false;
    return false;
  }

  Rc_ApplyAutoOreOutputs();
  g_rc_control_debug.page = RC_CONTROL_PAGE_AUTO_ORE;
  g_auto_ore_debug.force_output_count++;
  return true;
}

static void Rc_ApplyAutoRodSpearheadOutputs(void) {
  const RodNew_CMD_t *auto_rod_cmd = Task_AutoRodSpearheadGetCommand();
  if (auto_rod_cmd != NULL) {
    rod_cmd = *auto_rod_cmd;
    rod_grip_latched = rod_cmd.grip;
    rod_target_angle_latched_rad = rod_cmd.target_angle_rad;
  } else {
    Rc_SetRodHold();
  }
  g_rc_control_debug.rod_active = true;
}

static void Rc_ApplyAutoCtrlOutputs(void) {
  g_rc_control_debug.page = Rc_GetAutoCtrlPage(AutoCtrl_GetTemplate(&auto_ctrl));
#if AUTO_CTRL_OUTPUT_ENABLE
  chassis_cmd = auto_ctrl.chassis_cmd;
  pole_cmd = auto_ctrl.pole_cmd;
#else
  Rc_SetAutoDryRunCommands();
#endif
  Rc_SetArmSimpleHold();
  Rc_SetRodHold();
  Rc_SetOreStoreHold();
}

static void Rc_ApplyDriveBehavior(void) {
  g_rc_control_debug.page = RC_CONTROL_PAGE_DRIVE;
  Rc_SetChassisRemote();
  Rc_SetPoleManual(dr16.data.ch_l_y, dr16.data.ch_l_y);
  Rc_SetArmSimpleHold();
  Rc_SetRodHold();
  Rc_SetOreStoreHold();
}

static void Rc_ApplyAutoStandbyBehavior(RcBehavior_t behavior) {
  g_rc_control_debug.page = Rc_PageForBehavior(behavior);
  Rc_SetChassisRelax();
  Rc_SetPoleHold();
  Rc_SetArmSimpleHold();
  Rc_SetRodHold();
  Rc_SetOreStoreHold();
}

static void Rc_ApplyPcBehavior(void) {
  g_rc_control_debug.page = RC_CONTROL_PAGE_PC;
  g_pc_command_source = PC_COMMAND_SOURCE_PC;

  if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl)) {
#if AUTO_CTRL_OUTPUT_ENABLE
    chassis_cmd = auto_ctrl.chassis_cmd;
    pole_cmd = auto_ctrl.pole_cmd;
#else
    Rc_SetAutoDryRunCommands();
#endif
  } else {
    const PC_ChassisCMD_t* pc_chassis_cmd = MrlinkPc_GetChassisCMD();
    const PC_PoleCMD_t* pc_pole_cmd = MrlinkPc_GetPoleCMD();
    const PC_ArmSimpleCMD_t* pc_arm_simple_cmd =
      MrlinkPc_GetArmSimpleCMD();
    const PC_RodNewCMD_t* pc_rod_new_cmd =
      MrlinkPc_GetRodNewCMD();
    const PC_OreStoreCMD_t* pc_ore_store_cmd =
      MrlinkPc_GetOreStoreCMD();

    if (pc_chassis_cmd != NULL) {
      chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
      chassis_cmd.ctrl_vec.vx = pc_chassis_cmd->vx;
      chassis_cmd.ctrl_vec.vy = pc_chassis_cmd->vy;
      chassis_cmd.ctrl_vec.wz = pc_chassis_cmd->wz;
    } else {
      Rc_SetChassisRelax();
    }

    if (pc_pole_cmd != NULL) {
      pole_cmd.mode = (pc_pole_cmd->mode == 0) ? POLE_MODE_RELAX
                                               : POLE_MODE_ACTIVE;
      pole_cmd.lift[0] = 0.0f;
      pole_cmd.lift[1] = 0.0f;
      pole_cmd.auto_target_enable[0] = (pole_cmd.mode == POLE_MODE_ACTIVE);
      pole_cmd.auto_target_enable[1] = (pole_cmd.mode == POLE_MODE_ACTIVE);
      pole_cmd.auto_target_lift[0] = pc_pole_cmd->lift[0];
      pole_cmd.auto_target_lift[1] = pc_pole_cmd->lift[1];
      pole_cmd.auto_lift_speed[0] = 0.0f;
      pole_cmd.auto_lift_speed[1] = 0.0f;
      pole_cmd.disable_lift_accel = false;
    }

    if (pc_arm_simple_cmd != NULL) {
      arm_simple_cmd.mode = (ArmSimple_Mode_t)pc_arm_simple_cmd->mode;
      arm_simple_cmd.point_mode =
          (ArmSimple_PointMode_t)pc_arm_simple_cmd->point_mode;
      arm_simple_cmd.suction = (Suction_State_t)pc_arm_simple_cmd->suction;
      arm_simple_cmd.target_joint.joint1 = pc_arm_simple_cmd->target_joint1_rad;
      arm_simple_cmd.target_joint.joint2 = pc_arm_simple_cmd->target_joint2_rad;
      arm_simple_cmd.joint1_vel = 0.0f;
      arm_simple_suction_latched = arm_simple_cmd.suction;
      arm_simple_target_initialized = true;
    } else {
      Rc_SetArmSimpleHold();
    }

    if (pc_rod_new_cmd != NULL) {
      rod_cmd.mode = (RodNew_Mode_t)pc_rod_new_cmd->mode;
      rod_cmd.pose = (RodNew_Pose_t)pc_rod_new_cmd->pose;
      rod_cmd.grip = (RodNew_GripState_t)pc_rod_new_cmd->grip;
      rod_cmd.target_angle_rad =
          Rc_ClampRodNewTarget(pc_rod_new_cmd->target_angle_rad);
      rod_grip_latched = rod_cmd.grip;
      rod_target_angle_latched_rad = rod_cmd.target_angle_rad;
    } else {
      Rc_SetRodHold();
    }

    if (pc_ore_store_cmd != NULL) {
      if (Task_OreStorePowerOnHomeInProgress()) {
        Rc_SetOreStoreRelax();
        return;
      }
      ore_store_cmd.mode = (OreStore_Mode_t)pc_ore_store_cmd->mode;
      ore_store_cmd.force_rehome = pc_ore_store_cmd->force_rehome != 0u;
      ore_store_cmd.platform_target_rad =
          Rc_ClampOreStoreTarget(ORE_STORE_AXIS_PLATFORM,
                                 pc_ore_store_cmd->platform_target_rad);
      ore_store_active_initialized = ore_store_cmd.mode == ORE_STORE_MODE_ACTIVE;
    } else {
      Rc_SetOreStoreHold();
    }
  }
}

static void Rc_ApplyArmSimpleBehavior(void) {
  g_rc_control_debug.page = RC_CONTROL_PAGE_ARM_SIMPLE;
  Rc_SetChassisRelax();
  Rc_SetPoleHold();
  Rc_SetArmSimpleOperator(RC_ARM_FINE_SCALE);
  Rc_SetRodHold();
  Rc_SetOreStoreHold();
  g_rc_control_debug.arm_simple_active = true;
}

static void Rc_ApplyOreStoreBehavior(void) {
  g_rc_control_debug.page = RC_CONTROL_PAGE_ORE_STORE;
  Rc_SetChassisRelax();
  Rc_SetPoleHold();
  Rc_SetArmSimpleHold();
  Rc_SetOreStoreActiveManual();
  Rc_SetRodHold();
  g_rc_control_debug.ore_store_active = true;
}

static void Rc_ApplyRodNewBehavior(void) {
  g_rc_control_debug.page = RC_CONTROL_PAGE_ROD_NEW;
  Rc_SetChassisRelax();
  Rc_SetPoleHold();
  Rc_SetArmSimpleHold();
  Rc_SetOreStoreHold();
  Rc_SetRodOperator();
  g_rc_control_debug.rod_active = true;
}

static void Rc_ApplyAutoOreStandbyBehavior(void) {
  g_rc_control_debug.page = RC_CONTROL_PAGE_AUTO_ORE;
  Rc_SetChassisRelax();
  Rc_SetPoleHold();
  Rc_SetArmSimpleHold();
  Rc_SetOreStoreHold();
  Rc_SetRodHold();
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

static void Rc_ApplyMappedBehavior(RcBehavior_t behavior) {
  if (auto_ore_inited && AutoOre_IsBusy(&auto_ore_ctrl)) {
    if (behavior == RC_BEHAVIOR_AUTO_ORE) {
      Rc_ApplyAutoOreOutputs();
      return;
    }
    Rc_LatchAutoOreCurrentTargets();
    Task_AutoOreAbort();
  }

  if (Task_AutoRodSpearheadIsBusy()) {
    Rc_ApplyAutoRodSpearheadOutputs();
    return;
  }

  if (!Rc_BehaviorAllowsAutoCtrlOutput(behavior) &&
      behavior != RC_BEHAVIOR_PC) {
    Rc_AbortAutoCtrlIfBusy();
  }

  if (behavior == RC_BEHAVIOR_PC && !Rc_ShouldUsePcCommand()) {
    Rc_AbortAutoCtrlIfBusy();
    Rc_ApplySafeBehavior();
    return;
  }

  Rc_HandleBehaviorEvents(behavior);

  switch (behavior) {
    case RC_BEHAVIOR_DRIVE:
      Rc_ApplyDriveBehavior();
      break;
    case RC_BEHAVIOR_AUTO_200_UP_STANDBY:
    case RC_BEHAVIOR_AUTO_200_DOWN_STANDBY:
      Rc_ApplyAutoStandbyBehavior(behavior);
      break;
    case RC_BEHAVIOR_PC:
      Rc_ApplyPcBehavior();
      break;
    case RC_BEHAVIOR_ARM_SIMPLE:
      Rc_ApplyArmSimpleBehavior();
      break;
    case RC_BEHAVIOR_ORE_STORE:
      Rc_ApplyOreStoreBehavior();
      break;
    case RC_BEHAVIOR_ROD_NEW:
      Rc_ApplyRodNewBehavior();
      break;
    case RC_BEHAVIOR_AUTO_ORE:
      Rc_ApplyAutoOreStandbyBehavior();
      break;
    case RC_BEHAVIOR_SAFE:
    default:
      Rc_ApplySafeBehavior();
      break;
  }
}

static void Rc_PublishCommandsAndDebug(void) {
  osMessageQueueReset(task_runtime.msgq.chassis.cmd);
  osMessageQueuePut(task_runtime.msgq.chassis.cmd, &chassis_cmd, 0, 0);
  osMessageQueueReset(task_runtime.msgq.pole.cmd);
  osMessageQueuePut(task_runtime.msgq.pole.cmd, &pole_cmd, 0, 0);
  osMessageQueueReset(task_runtime.msgq.arm_simple.cmd);
  osMessageQueuePut(task_runtime.msgq.arm_simple.cmd, &arm_simple_cmd, 0, 0);
  osMessageQueueReset(task_runtime.msgq.rod.cmd);
  osMessageQueuePut(task_runtime.msgq.rod.cmd, &rod_cmd, 0, 0);
  g_rc_ore_store_post_ret = Task_OreStorePostCommand(&ore_store_cmd);

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
  /* Tail-forward tasks choose the cardinal target by tail heading first. */
  if (travel_dir == AUTO_CTRL_TRAVEL_DIR_TAIL_FORWARD) {
    const float tail_yaw_rad =
        AutoCtrlMath_WrapYawRad(head_yaw_rad + AUTO_CTRL_RC_PI_RAD);
    const float tail_target_rad =
        AutoCtrlMath_NearestCardinalYawRad(tail_yaw_rad);
    return AutoCtrlMath_WrapYawRad(tail_target_rad + AUTO_CTRL_RC_PI_RAD);
  }

  return AutoCtrlMath_NearestCardinalYawRad(head_yaw_rad);
}

static void Rc_TryStartAutoCtrlBySwitch(uint32_t now_ms) {
  if (!dr16.header.online || !auto_ctrl_inited || AutoCtrl_IsBusy(&auto_ctrl)) {
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
    g_rc_control_debug.auto_200_start_ok = AutoCtrl_StartTemplate(
        &auto_ctrl, config.template_id, config.travel_dir,
        target_yaw_rad, AUTO_CTRL_RC_YAW_TOLERANCE_RAD,
        AUTO_CTRL_SENSOR_MODE_SICK_FRONT_AND_BOTTOM, now_ms);
  }
} 
/* USER STRUCT END */
 
/* Exported functions ------------------------------------------------------- */
void Task_rc_main(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / RC_MAIN_FREQ;
  osDelay(RC_MAIN_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  if (DR16_Init(&dr16) != DEVICE_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }
  if (DR16_StartDmaRecv(&dr16) != DEVICE_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }
  Rc_SetArmSimpleRelax();
  Rc_SetOreStoreRelax();

  while (1) {
    uint32_t now_ms;
    RcBehavior_t behavior;
    tick += delay_tick;
    DR16_StartDmaRecv(&dr16);
    if (DR16_WaitDmaCplt(100)) {
      DR16_ParseData(&dr16);
    } else {
      DR16_Offline(&dr16);
    }

    now_ms = osKernelGetTickCount();
    Rc_ResetFrameDebug();
    Rc_HandleResetEvent();

    Rc_LatchFinishedAutoTargets();

    Rc_TryStartAutoCtrlBySwitch(now_ms);

    if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl) &&
        Rc_ShouldExitAutoCtrlBySwitch()) {
      Rc_LatchAutoCtrlCurrentTargets();
      AutoCtrl_Abort(&auto_ctrl);
    }

    behavior = Rc_SelectMappedBehavior();
    if (Rc_TryApplyAutoOreDebugOutputs()) {
      /* Commands were filled by AutoOre debug output. */
    } else if (behavior == RC_BEHAVIOR_SAFE) {
      Rc_ApplySafeBehavior();
    } else if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl) &&
               Rc_BehaviorAllowsAutoCtrlOutput(behavior)) {
      Rc_ApplyAutoCtrlOutputs();
    } else {
      Rc_ApplyMappedBehavior(behavior);
    }

    Rc_PublishCommandsAndDebug();
  Rc_UpdateAutoBusyHistory();
    last_sw_l = dr16.data.sw_l;
    last_sw_r = dr16.data.sw_r;
    task_runtime.stack_water_mark.rc_main = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick);
  }
}
