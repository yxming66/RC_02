/*
    rc_main Task
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "bsp/uart.h"
#include "device/dr16.h"
#include "module/chassis.h"
#include "module/pole.h"
#include "module/arm/arm_control_types.h"
#include "module/arm_simple.h"
#include "module/config.h"
#include "module/rod_new.h"
#include "module/autoCtrlAPI/api/auto_ctrl_api.h"
#include "module/autoCtrlAPI/core/auto_ctrl_def.h"
#include "module/pc_protocol/pc_protocol.h"
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

/* USER STRUCT BEGIN */
DR16_t dr16;
static Chassis_CMD_t chassis_cmd;
static Pole_CMD_t pole_cmd;
static Arm_CMD_t arm_cmd;
static ArmSimple_CMD_t arm_simple_cmd;
static OreStore_CMD_t ore_store_cmd;
static DR16_SwitchPos_t last_sw_l = DR16_SW_ERR;
static DR16_SwitchPos_t last_sw_r = DR16_SW_ERR;
static RodNew_CMD_t rod_cmd;
extern bool reset;
extern auto_ctrl_t auto_ctrl;
extern bool auto_ctrl_inited;
extern Chassis_IMU_t chassis_imu;

typedef struct {
  bool enabled;
  bool force_enable;
  bool command_queued;
  uint32_t request_seq;
  uint32_t applied_seq;
  ArmPose_t target_pose;
} ArmTaskDebugPoseControl_t;

// Temporary task-layer hook for debugger pose injection; remove after bring-up.
volatile ArmTaskDebugPoseControl_t g_arm_task_debug_pose_control = {0};

/* PC上位机控制相关 */
#define RC_PC_ENABLE_SWITCH (DR16_SW_UP)  /* sw_l DOWN 且 sw_r UP 时允许PC接管 */
extern PC_Protocol_t* g_pc_protocol_ptr;
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
  RC_CONTROL_PAGE_ORE_STORE_ROD,
} RcControlPage_t;

typedef struct {
  volatile RcControlPage_t page;
  volatile bool reset_event;
  volatile bool arm_simple_active;
  volatile bool rod_active;
  volatile bool ore_store_active;
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
  volatile float gate_target_rad[ORE_STORE_GATE_NUM];
  volatile float track_target_rad[ORE_STORE_TRACK_NUM];
  volatile bool assume_home_event;
  volatile int8_t assume_home_ret;
  volatile int8_t post_ret;
} RcOreStoreDebug_t;

volatile RcOreStoreDebug_t g_rc_ore_store_debug = {0};

static bool ore_store_active_initialized = false;
static bool arm_simple_target_initialized = false;
static RodNew_Pose_t rod_pose_latched = ROD_NEW_POSE_STANDBY;
static RodNew_GripState_t rod_grip_latched = ROD_NEW_GRIP_RELEASE;

#define RC_ORE_STORE_DEADBAND (0.05f)
#define RC_ORE_STORE_FALLBACK_MOVE_SPEED_RAD_S (0.5f)

static bool Rc_ArmPoseIsFinite(const ArmPose_t* pose) {
  return pose != NULL && isfinite(pose->x) && isfinite(pose->y) &&
         isfinite(pose->z) && isfinite(pose->roll) &&
         isfinite(pose->pitch) && isfinite(pose->yaw);
}

static bool Rc_KeyDown(DR16_Key_t key) {
  return key < DR16_KEY_NUM && dr16.data.keyboard.key[key];
}

static float Rc_ApplyDeadband(float value, float deadband) {
  return (fabsf(value) < deadband) ? 0.0f : value;
}

static void Rc_SetRodRelax(void) {
  rod_cmd.mode = ROD_NEW_MODE_RELAX;
  rod_cmd.pose = ROD_NEW_POSE_STANDBY;
  rod_cmd.grip = ROD_NEW_GRIP_RELEASE;
}

static void Rc_SetRodHold(void) {
  if (rod_cmd.mode == ROD_NEW_MODE_ACTIVE) {
    return;
  }
  Rc_SetRodRelax();
}

static void Rc_SetRodOperator(void) {
  if (Rc_KeyDown(DR16_KEY_Q)) {
    rod_pose_latched = ROD_NEW_POSE_STANDBY;
    rod_grip_latched = ROD_NEW_GRIP_RELEASE;
  } else if (Rc_KeyDown(DR16_KEY_E)) {
    rod_pose_latched = ROD_NEW_POSE_READY;
    rod_grip_latched = ROD_NEW_GRIP_RELEASE;
  } else if (Rc_KeyDown(DR16_KEY_R)) {
    rod_pose_latched = ROD_NEW_POSE_GRAB_LOW;
  } else if (Rc_KeyDown(DR16_KEY_F)) {
    rod_pose_latched = ROD_NEW_POSE_GRAB_HIGH;
  } else if (Rc_KeyDown(DR16_KEY_G)) {
    rod_pose_latched = ROD_NEW_POSE_LIFT;
  }

  if (dr16.data.mouse.l_click || Rc_KeyDown(DR16_KEY_Z)) {
    rod_grip_latched = ROD_NEW_GRIP_RELEASE;
  } else if (dr16.data.mouse.r_click || Rc_KeyDown(DR16_KEY_X)) {
    rod_grip_latched = ROD_NEW_GRIP_GRAB;
  }

  rod_cmd.mode = ROD_NEW_MODE_ACTIVE;
  rod_cmd.pose = rod_pose_latched;
  rod_cmd.grip = rod_grip_latched;
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
  ore_store_cmd.mode = ORE_STORE_MODE_RELAX;
  ore_store_active_initialized = false;
}

static void Rc_SetOreStoreHold(void) {
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
    if (feedback->homed[ORE_STORE_AXIS_GATE_LEFT]) {
    ore_store_cmd.gate_target_rad[0] =
        feedback->position_rad[ORE_STORE_AXIS_GATE_LEFT];
    }
    if (feedback->homed[ORE_STORE_AXIS_GATE_RIGHT]) {
    ore_store_cmd.gate_target_rad[1] =
        feedback->position_rad[ORE_STORE_AXIS_GATE_RIGHT];
    }
    if (feedback->homed[ORE_STORE_AXIS_TRACK_LEFT]) {
    ore_store_cmd.track_target_rad[0] =
        feedback->position_rad[ORE_STORE_AXIS_TRACK_LEFT];
    }
    if (feedback->homed[ORE_STORE_AXIS_TRACK_RIGHT]) {
    ore_store_cmd.track_target_rad[1] =
        feedback->position_rad[ORE_STORE_AXIS_TRACK_RIGHT];
    }
  }

  ore_store_cmd.platform_target_rad = Rc_ClampOreStoreTarget(
      ORE_STORE_AXIS_PLATFORM, ore_store_cmd.platform_target_rad);
  ore_store_cmd.gate_target_rad[0] = Rc_ClampOreStoreTarget(
      ORE_STORE_AXIS_GATE_LEFT, ore_store_cmd.gate_target_rad[0]);
  ore_store_cmd.gate_target_rad[1] = Rc_ClampOreStoreTarget(
      ORE_STORE_AXIS_GATE_RIGHT, ore_store_cmd.gate_target_rad[1]);
  ore_store_cmd.track_target_rad[0] = Rc_ClampOreStoreTarget(
      ORE_STORE_AXIS_TRACK_LEFT, ore_store_cmd.track_target_rad[0]);
  ore_store_cmd.track_target_rad[1] = Rc_ClampOreStoreTarget(
      ORE_STORE_AXIS_TRACK_RIGHT, ore_store_cmd.track_target_rad[1]);
  ore_store_active_initialized = true;
}

static void Rc_SetOreStoreActiveManual(void) {
  const float dt_s = 1.0f / (float)RC_MAIN_FREQ;
  const float platform_velocity =
    Rc_ApplyOreStoreDeadband(dr16.data.ch_r_y) *
    Rc_OreStoreAxisMoveSpeed(ORE_STORE_AXIS_PLATFORM);
  const float gate_velocity =
    Rc_ApplyOreStoreDeadband(dr16.data.ch_l_x) *
    Rc_OreStoreAxisMoveSpeed(ORE_STORE_AXIS_GATE_LEFT);
  const float track_velocity =
    Rc_ApplyOreStoreDeadband(dr16.data.ch_l_y) *
    Rc_OreStoreAxisMoveSpeed(ORE_STORE_AXIS_TRACK_LEFT);
  const float platform_delta = platform_velocity * dt_s;
  const float gate_delta = gate_velocity * dt_s;
  const float track_delta = track_velocity * dt_s;

  if (!ore_store_active_initialized) {
    Rc_InitOreStoreActiveTargets();
  }

  ore_store_cmd.mode = ORE_STORE_MODE_ACTIVE;
  ore_store_cmd.force_rehome = false;
  ore_store_cmd.platform_target_rad = Rc_ClampOreStoreTarget(
      ORE_STORE_AXIS_PLATFORM, ore_store_cmd.platform_target_rad + platform_delta);
  ore_store_cmd.gate_target_rad[0] = Rc_ClampOreStoreTarget(
      ORE_STORE_AXIS_GATE_LEFT, ore_store_cmd.gate_target_rad[0] + gate_delta);
  ore_store_cmd.gate_target_rad[1] = Rc_ClampOreStoreTarget(
      ORE_STORE_AXIS_GATE_RIGHT, ore_store_cmd.gate_target_rad[1] + gate_delta);
  ore_store_cmd.track_target_rad[0] = Rc_ClampOreStoreTarget(
      ORE_STORE_AXIS_TRACK_LEFT, ore_store_cmd.track_target_rad[0] + track_delta);
  ore_store_cmd.track_target_rad[1] = Rc_ClampOreStoreTarget(
      ORE_STORE_AXIS_TRACK_RIGHT, ore_store_cmd.track_target_rad[1] + track_delta);
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
}

static void Rc_SetPoleHold(void) {
  if (pole_cmd.mode == POLE_MODE_ACTIVE) {
    pole_cmd.lift[0] = 0.0f;
    pole_cmd.lift[1] = 0.0f;
    return;
  }

  Rc_SetPoleManual(0.0f, 0.0f);
}

static void Rc_SetArmDisabled(void) {
  memset(&arm_cmd, 0, sizeof(arm_cmd));
  arm_cmd.enable = dr16.header.online;
  arm_cmd.set_target_as_current = dr16.header.online;
  arm_cmd.ctrl_type = ARM_CTRL_REMOTE_CARTESIAN;
  arm_cmd.frame = ARM_CTRL_FRAME_WORLD;
}

static void Rc_SetArmSimpleRelax(void) {
  memset(&arm_simple_cmd, 0, sizeof(arm_simple_cmd));
  arm_simple_cmd.mode = ARM_SIMPLE_MODE_RELAX;
  arm_simple_cmd.point_mode = ARM_SIMPLE_POINT_NONE;
  arm_simple_cmd.suction = SUCTION_OFF;
  arm_simple_target_initialized = false;
}

static void Rc_SetArmSimpleHold(void) {
  arm_simple_cmd.mode = ARM_SIMPLE_MODE_JOINT;
  arm_simple_cmd.point_mode = ARM_SIMPLE_POINT_NONE;
  arm_simple_target_initialized = true;
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

  if (dr16.data.mouse.l_click || Rc_KeyDown(DR16_KEY_Q)) {
    arm_simple_cmd.suction = SUCTION_OFF;
  } else if (dr16.data.mouse.r_click || Rc_KeyDown(DR16_KEY_E)) {
    arm_simple_cmd.suction = SUCTION_ON;
  }
}

static void Rc_SetArmPoseAbsolute(const ArmPose_t* target_pose,
                                  bool enable) {
  memset(&arm_cmd, 0, sizeof(arm_cmd));
  arm_cmd.enable = enable;
  arm_cmd.ctrl_type = ARM_CTRL_POSE_ABSOLUTE;
  arm_cmd.frame = ARM_CTRL_FRAME_WORLD;
  if (target_pose != NULL) {
    arm_cmd.target_pose = *target_pose;
  }
}

static void Rc_ApplyArmDebugPoseOverride(void) {
  if (!g_arm_task_debug_pose_control.enabled) {
    g_arm_task_debug_pose_control.command_queued = false;
    return;
  }

  ArmPose_t target_pose;
  target_pose.x = g_arm_task_debug_pose_control.target_pose.x;
  target_pose.y = g_arm_task_debug_pose_control.target_pose.y;
  target_pose.z = g_arm_task_debug_pose_control.target_pose.z;
  target_pose.roll = g_arm_task_debug_pose_control.target_pose.roll;
  target_pose.pitch = g_arm_task_debug_pose_control.target_pose.pitch;
  target_pose.yaw = g_arm_task_debug_pose_control.target_pose.yaw;

  const bool valid_pose = Rc_ArmPoseIsFinite(&target_pose);
  if (!valid_pose) {
    g_arm_task_debug_pose_control.command_queued = false;
    return;
  }

  const bool enable =
      dr16.header.online || g_arm_task_debug_pose_control.force_enable;
  Rc_SetArmPoseAbsolute(&target_pose, enable);
  g_arm_task_debug_pose_control.applied_seq =
      g_arm_task_debug_pose_control.request_seq;
  g_arm_task_debug_pose_control.command_queued = true;
}

static void Rc_SetChassisRelax(void) {
  chassis_cmd.mode = CHASSIS_MODE_RELAX;
  chassis_cmd.ctrl_vec.vx = 0.0f;
  chassis_cmd.ctrl_vec.vy = 0.0f;
  chassis_cmd.ctrl_vec.wz = 0.0f;
}

static void Rc_SetChassisRemote(void) {
  chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
  chassis_cmd.ctrl_vec.vx = dr16.data.ch_r_y;
  chassis_cmd.ctrl_vec.vy = -dr16.data.ch_r_x;
  chassis_cmd.ctrl_vec.wz = -dr16.data.ch_l_x;
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
  return g_pc_protocol_ptr != NULL &&
         PC_Protocol_IsPCControlMode(g_pc_protocol_ptr) &&
         dr16.data.sw_l == DR16_SW_DOWN &&
         dr16.data.sw_r == RC_PC_ENABLE_SWITCH;
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

  if (dr16.data.sw_l != DR16_SW_MID) {
    return;
  }

  if (last_sw_r != DR16_SW_MID) {
    return;
  }

  auto_ctrl_template_e template_id = AUTO_CTRL_TEMPLATE_NONE;
  auto_ctrl_travel_dir_e travel_dir = AUTO_CTRL_TRAVEL_DIR_HEAD_FORWARD;
  float target_yaw_rad = 0.0f;

  if (dr16.data.sw_r == DR16_SW_UP) {
    template_id = AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD;
  } else if (dr16.data.sw_r == DR16_SW_DOWN) {
    template_id = AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD;
    
    travel_dir = AUTO_CTRL_TRAVEL_DIR_TAIL_FORWARD;
  }

  if (template_id != AUTO_CTRL_TEMPLATE_NONE) {
    if (!Rc_PrepareLocalAutoYawFeedback()) {
      return;
    }

    target_yaw_rad =
        Rc_SelectLocalAutoTargetYawRad(auto_ctrl.feedback.yaw_auto_rad,
                                       travel_dir);
    (void)AutoCtrl_StartTemplate(
        &auto_ctrl, template_id, travel_dir,
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
  Rc_SetArmDisabled();
  Rc_SetArmSimpleRelax();
  Rc_SetOreStoreRelax();

  while (1) {
    uint32_t now_ms;
    tick += delay_tick;

    DR16_StartDmaRecv(&dr16);
    if (DR16_WaitDmaCplt(100)) {
      DR16_ParseData(&dr16);
    } else {
      DR16_Offline(&dr16);
    }

    now_ms = osKernelGetTickCount();
    g_pc_command_source = PC_COMMAND_SOURCE_RC;
    g_rc_control_debug.page = RC_CONTROL_PAGE_SAFE;
    g_rc_control_debug.reset_event = false;
    g_rc_control_debug.arm_simple_active = false;
    g_rc_control_debug.rod_active = false;
    g_rc_control_debug.ore_store_active = false;
    g_rc_ore_store_debug.assume_home_event = false;

    if (dr16.header.online && dr16.data.sw_l == DR16_SW_UP &&
        dr16.data.sw_r == DR16_SW_DOWN && last_sw_r != DR16_SW_DOWN &&
        last_sw_r != DR16_SW_ERR) {
      reset = true;
      g_rc_control_debug.reset_event = true;
    }

    Rc_TryStartAutoCtrlBySwitch(now_ms);

    if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl) &&
        Rc_ShouldExitAutoCtrlBySwitch()) {
      AutoCtrl_Abort(&auto_ctrl);
    }

    if (!dr16.header.online || dr16.data.sw_l == DR16_SW_UP) {
      g_rc_control_debug.page = RC_CONTROL_PAGE_SAFE;
      Rc_SetChassisRelax();
      Rc_SetPoleAuto(0.0f, 0.0f);
      Rc_SetArmDisabled();
      Rc_SetArmSimpleRelax();
      Rc_SetRodRelax();
      Rc_SetOreStoreRelax();
    } else if (dr16.data.sw_l == DR16_SW_MID) {
      if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl)) {
        g_rc_control_debug.page = (AutoCtrl_GetTemplate(&auto_ctrl) ==
                                  AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD)
                                     ? RC_CONTROL_PAGE_AUTO_200_UP
                                     : RC_CONTROL_PAGE_AUTO_200_DOWN;
#if AUTO_CTRL_OUTPUT_ENABLE
        chassis_cmd = auto_ctrl.chassis_cmd;
        pole_cmd = auto_ctrl.pole_cmd;
#else
        Rc_SetAutoDryRunCommands();
#endif
        Rc_SetOreStoreHold();
      } else if (dr16.data.sw_r == DR16_SW_UP) {
        g_rc_control_debug.page = RC_CONTROL_PAGE_AUTO_200_UP;
        Rc_SetChassisRelax();
        Rc_SetPoleAuto(0.0f, 0.0f);
        Rc_SetOreStoreHold();
      } else if (dr16.data.sw_r == DR16_SW_DOWN) {
        g_rc_control_debug.page = RC_CONTROL_PAGE_AUTO_200_DOWN;
        Rc_SetChassisRelax();
        Rc_SetPoleAuto(0.0f, 0.0f);
        Rc_SetOreStoreHold();
      } else if (dr16.data.sw_r == DR16_SW_MID) {
        g_rc_control_debug.page = RC_CONTROL_PAGE_DRIVE;
        Rc_SetChassisRemote();
        Rc_SetPoleManual(dr16.data.ch_l_y, dr16.data.ch_l_y);
        Rc_SetOreStoreHold();
      } else {
        Rc_SetChassisRelax();
        Rc_SetPoleAuto(0.0f, 0.0f);
        Rc_SetOreStoreHold();
      }

      Rc_SetArmDisabled();
      Rc_SetArmSimpleHold();
      Rc_SetRodHold();
    } else if (dr16.data.sw_l == DR16_SW_DOWN) {
      const bool use_pc_command = Rc_ShouldUsePcCommand();

      if (!use_pc_command && auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl)) {
        AutoCtrl_Abort(&auto_ctrl);
      }

      /* sw_l DOWN 且 sw_r UP 时才允许使用上位机命令。 */
      if (use_pc_command) {
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
        /* 上位机控制模式 */
        const PC_ChassisCMD_t* pc_chassis_cmd = PC_Protocol_GetChassisCMD(g_pc_protocol_ptr);
        const PC_PoleCMD_t* pc_pole_cmd = PC_Protocol_GetPoleCMD(g_pc_protocol_ptr);

        /* 使用上位机Chassis命令 */
        if (pc_chassis_cmd != NULL) {
          chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
          chassis_cmd.ctrl_vec.vx = pc_chassis_cmd->vx;
          chassis_cmd.ctrl_vec.vy = pc_chassis_cmd->vy;
          chassis_cmd.ctrl_vec.wz = pc_chassis_cmd->wz;
        } else {
          Rc_SetChassisRelax();
        }

        /* 使用上位机Pole命令 */
        if (pc_pole_cmd != NULL) {
          pole_cmd.mode = (pc_pole_cmd->mode == 0) ? POLE_MODE_RELAX : POLE_MODE_ACTIVE;
          pole_cmd.lift[0] = 0.0f;
          pole_cmd.lift[1] = 0.0f;
          pole_cmd.auto_target_enable[0] = (pole_cmd.mode == POLE_MODE_ACTIVE);
          pole_cmd.auto_target_enable[1] = (pole_cmd.mode == POLE_MODE_ACTIVE);
          pole_cmd.auto_target_lift[0] = pc_pole_cmd->lift[0];
          pole_cmd.auto_target_lift[1] = pc_pole_cmd->lift[1];
          pole_cmd.auto_lift_speed[0] = 0.0f;
          pole_cmd.auto_lift_speed[1] = 0.0f;
        }
        }
        Rc_SetArmDisabled();
        Rc_SetArmSimpleHold();
        Rc_SetRodHold();
        Rc_SetOreStoreHold();
      } else if (dr16.data.sw_r == DR16_SW_MID) {
        g_rc_control_debug.page = RC_CONTROL_PAGE_ARM_SIMPLE;
        Rc_SetChassisRelax();
        Rc_SetPoleHold();
        Rc_SetArmDisabled();
        Rc_SetArmSimpleOperator(RC_ARM_FINE_SCALE);
        Rc_SetRodHold();
        Rc_SetOreStoreHold();
        g_rc_control_debug.arm_simple_active = true;
      } else if (dr16.data.sw_r == DR16_SW_DOWN) {
        g_rc_control_debug.page = RC_CONTROL_PAGE_ORE_STORE_ROD;
        Rc_SetChassisRelax();
        Rc_SetPoleHold();
        Rc_SetArmDisabled();
        Rc_SetArmSimpleHold();
        Rc_SetOreStoreActiveManual();
        Rc_SetRodOperator();
        g_rc_control_debug.rod_active = true;
        g_rc_control_debug.ore_store_active = true;
      } else {
        g_rc_control_debug.page = RC_CONTROL_PAGE_SAFE;
        Rc_SetChassisRelax();
        Rc_SetPoleHold();
        Rc_SetArmDisabled();
        Rc_SetArmSimpleHold();
        Rc_SetRodHold();
        Rc_SetOreStoreHold();
      }
    }

    Rc_ApplyArmDebugPoseOverride();

    osMessageQueueReset(task_runtime.msgq.chassis.cmd);
    osMessageQueuePut(task_runtime.msgq.chassis.cmd, &chassis_cmd, 0, 0);
    osMessageQueueReset(task_runtime.msgq.pole.cmd);
    osMessageQueuePut(task_runtime.msgq.pole.cmd, &pole_cmd, 0, 0);
    osMessageQueueReset(task_runtime.msgq.arm.cmd);
    osMessageQueuePut(task_runtime.msgq.arm.cmd, &arm_cmd, 0, 0);
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
    g_rc_ore_store_debug.gate_target_rad[0] = ore_store_cmd.gate_target_rad[0];
    g_rc_ore_store_debug.gate_target_rad[1] = ore_store_cmd.gate_target_rad[1];
    g_rc_ore_store_debug.track_target_rad[0] = ore_store_cmd.track_target_rad[0];
    g_rc_ore_store_debug.track_target_rad[1] = ore_store_cmd.track_target_rad[1];
    g_rc_ore_store_debug.post_ret = g_rc_ore_store_post_ret;
    last_sw_l = dr16.data.sw_l;
    last_sw_r = dr16.data.sw_r;
    task_runtime.stack_water_mark.rc_main = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick);
  }
}
