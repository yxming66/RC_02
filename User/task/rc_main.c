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
#include "module/config.h"
#include "module/rod.h"
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
#define RC_ARM_FINE_SCALE (0.35f)

/* USER STRUCT BEGIN */
DR16_t dr16;
static Chassis_CMD_t chassis_cmd;
static Pole_CMD_t pole_cmd;
static Arm_CMD_t arm_cmd;
static OreStore_CMD_t ore_store_cmd;
static DR16_SwitchPos_t last_sw_l = DR16_SW_ERR;
static DR16_SwitchPos_t last_sw_r = DR16_SW_ERR;
static Rod_CMD_t rod_cmd;
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

static bool ore_store_active_initialized = false;

#define RC_ORE_STORE_DEADBAND (0.05f)
#define RC_ORE_STORE_FALLBACK_MOVE_SPEED_RAD_S (0.5f)

static bool Rc_ArmPoseIsFinite(const ArmPose_t* pose) {
  return pose != NULL && isfinite(pose->x) && isfinite(pose->y) &&
         isfinite(pose->z) && isfinite(pose->roll) &&
         isfinite(pose->pitch) && isfinite(pose->yaw);
}

static void Rc_SetRodRelax(void) {
  rod_cmd.mode = ROD_MODE_RELAX;
  rod_cmd.pose = ROD_POSE_DOWN;
  rod_cmd.sequence_trigger = false;
  rod_cmd.grip_done = false;
}

static float Rc_ApplyOreStoreDeadband(float value) {
  return (fabsf(value) < RC_ORE_STORE_DEADBAND) ? 0.0f : value;
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

static void Rc_SetOreStoreHome(bool force_rehome) {
  memset(&ore_store_cmd, 0, sizeof(ore_store_cmd));
  ore_store_cmd.mode = ORE_STORE_MODE_HOME;
  ore_store_cmd.force_rehome = force_rehome;
  ore_store_active_initialized = false;
}

static void Rc_InitOreStoreActiveTargets(void) {
  const OreStore_Feedback_t* feedback = Task_OreStoreGetFeedback();

  memset(&ore_store_cmd, 0, sizeof(ore_store_cmd));
  ore_store_cmd.mode = ORE_STORE_MODE_ACTIVE;

  if (feedback != NULL && feedback->all_homed) {
    ore_store_cmd.platform_target_rad =
        feedback->position_rad[ORE_STORE_AXIS_PLATFORM];
    ore_store_cmd.gate_target_rad[0] =
        feedback->position_rad[ORE_STORE_AXIS_GATE_LEFT];
    ore_store_cmd.gate_target_rad[1] =
        feedback->position_rad[ORE_STORE_AXIS_GATE_RIGHT];
    ore_store_cmd.track_target_rad[0] =
        feedback->position_rad[ORE_STORE_AXIS_TRACK_LEFT];
    ore_store_cmd.track_target_rad[1] =
        feedback->position_rad[ORE_STORE_AXIS_TRACK_RIGHT];
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
  const float platform_delta = Rc_ApplyOreStoreDeadband(dr16.data.ch_r_y) *
                               Rc_OreStoreAxisMoveSpeed(
                                   ORE_STORE_AXIS_PLATFORM) *
                               dt_s;
  const float gate_common_delta = Rc_ApplyOreStoreDeadband(dr16.data.ch_l_y) *
                                  Rc_OreStoreAxisMoveSpeed(
                                      ORE_STORE_AXIS_GATE_LEFT) *
                                  dt_s;
  const float gate_diff_delta = Rc_ApplyOreStoreDeadband(dr16.data.ch_r_x) *
                                Rc_OreStoreAxisMoveSpeed(
                                    ORE_STORE_AXIS_GATE_LEFT) *
                                dt_s;
  const float track_delta = Rc_ApplyOreStoreDeadband(dr16.data.ch_l_x) *
                            Rc_OreStoreAxisMoveSpeed(
                                ORE_STORE_AXIS_TRACK_LEFT) *
                            dt_s;

  if (!ore_store_active_initialized) {
    Rc_InitOreStoreActiveTargets();
  }

  ore_store_cmd.mode = ORE_STORE_MODE_ACTIVE;
  ore_store_cmd.force_rehome = false;
  ore_store_cmd.platform_target_rad = Rc_ClampOreStoreTarget(
      ORE_STORE_AXIS_PLATFORM, ore_store_cmd.platform_target_rad + platform_delta);
  ore_store_cmd.gate_target_rad[0] = Rc_ClampOreStoreTarget(
      ORE_STORE_AXIS_GATE_LEFT,
      ore_store_cmd.gate_target_rad[0] + gate_common_delta + gate_diff_delta);
  ore_store_cmd.gate_target_rad[1] = Rc_ClampOreStoreTarget(
      ORE_STORE_AXIS_GATE_RIGHT,
      ore_store_cmd.gate_target_rad[1] + gate_common_delta - gate_diff_delta);
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

static void Rc_SetArmDisabled(void) {
  memset(&arm_cmd, 0, sizeof(arm_cmd));
  arm_cmd.enable = dr16.header.online;
  arm_cmd.set_target_as_current = dr16.header.online;
  arm_cmd.ctrl_type = ARM_CTRL_REMOTE_CARTESIAN;
  arm_cmd.frame = ARM_CTRL_FRAME_WORLD;
}

static const ArmCartesianRemoteParam_t* Rc_GetArmRemoteParam(void) {
  Config_RobotParam_t* robot_param = Config_GetRobotParam();
  if (robot_param == NULL) {
    return NULL;
  }
  return &robot_param->arm_param.remote_cartesian;
}

static float Rc_ArmMaxYVelocity(void) {
  const ArmCartesianRemoteParam_t* remote = Rc_GetArmRemoteParam();
  return (remote != NULL) ? remote->max_y_velocity : 0.30f;
}

static float Rc_ArmMaxZVelocity(void) {
  const ArmCartesianRemoteParam_t* remote = Rc_GetArmRemoteParam();
  return (remote != NULL) ? remote->max_z_velocity : 0.30f;
}

static float Rc_ArmMaxPitchVelocity(void) {
  const ArmCartesianRemoteParam_t* remote = Rc_GetArmRemoteParam();
  return (remote != NULL) ? remote->max_pitch_velocity : 1.0f;
}

static void Rc_SetArmRemoteCartesian(float scale) {
  memset(&arm_cmd, 0, sizeof(arm_cmd));
  arm_cmd.enable = dr16.header.online;
  arm_cmd.ctrl_type = ARM_CTRL_REMOTE_CARTESIAN;
  arm_cmd.frame = ARM_CTRL_FRAME_WORLD;

  if (!dr16.header.online) {
    return;
  }

  arm_cmd.joy_vel.y = dr16.data.ch_r_y * Rc_ArmMaxYVelocity() * scale;
  arm_cmd.joy_vel.z = dr16.data.ch_l_y * Rc_ArmMaxZVelocity() * scale;
  arm_cmd.joy_vel.pitch =
      dr16.data.ch_l_x * Rc_ArmMaxPitchVelocity() * scale;
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

  if (dr16.data.sw_l == DR16_SW_MID &&
      (dr16.data.sw_r == DR16_SW_UP || dr16.data.sw_r == DR16_SW_DOWN)) {
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

    Rc_TryStartAutoCtrlBySwitch(now_ms);

    if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl) &&
        Rc_ShouldExitAutoCtrlBySwitch()) {
      AutoCtrl_Abort(&auto_ctrl);
    }

    if (!dr16.header.online || dr16.data.sw_l == DR16_SW_UP) {
      pole_cmd.mode = POLE_MODE_RELAX;
      pole_cmd.lift[0] = 0.0f;
      pole_cmd.lift[1] = 0.0f;
      pole_cmd.auto_target_enable[0] = false;
      pole_cmd.auto_target_enable[1] = false;
      pole_cmd.auto_target_lift[0] = 0.0f;
      pole_cmd.auto_target_lift[1] = 0.0f;
      pole_cmd.auto_lift_speed[0] = 0.0f;
      pole_cmd.auto_lift_speed[1] = 0.0f;

      switch (dr16.data.sw_r) {
        case DR16_SW_UP:
          chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
          chassis_cmd.ctrl_vec.vx = 2.0f * dr16.data.ch_r_y;
          chassis_cmd.ctrl_vec.vy = -4.0f * dr16.data.ch_r_x;
          chassis_cmd.ctrl_vec.wz = -2.0f * dr16.data.ch_l_x;
          Rc_SetPoleManual(dr16.data.ch_l_y, dr16.data.ch_l_y);
          break;
        case DR16_SW_MID:
          chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
          chassis_cmd.ctrl_vec.vx = dr16.data.ch_r_y;
          chassis_cmd.ctrl_vec.vy = -dr16.data.ch_r_x;
          chassis_cmd.ctrl_vec.wz = -dr16.data.ch_l_x;
          Rc_SetPoleManual(dr16.data.ch_l_y, dr16.data.ch_l_x);
          break;
        case DR16_SW_DOWN:
          chassis_cmd.mode = CHASSIS_MODE_RELAX;
          chassis_cmd.ctrl_vec.vx = 0.0f;
          chassis_cmd.ctrl_vec.vy = 0.0f;
          chassis_cmd.ctrl_vec.wz = 0.0f;
          Rc_SetPoleManual(0.0f, 0.0f);
          pole_cmd.mode = POLE_MODE_RELAX;
          break;
        default:
          chassis_cmd.mode = CHASSIS_MODE_RELAX;
          chassis_cmd.ctrl_vec.vx = 0.0f;
          chassis_cmd.ctrl_vec.vy = 0.0f;
          chassis_cmd.ctrl_vec.wz = 0.0f;
          Rc_SetPoleManual(0.0f, 0.0f);
          pole_cmd.mode = POLE_MODE_RELAX;
          break;
      }

      Rc_SetArmDisabled();
      Rc_SetRodRelax();
      Rc_SetOreStoreRelax();
    } else if (dr16.data.sw_l == DR16_SW_MID) {
      if (dr16.data.sw_r == DR16_SW_UP) {
        if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl)) {
          AutoCtrl_Abort(&auto_ctrl);
        }
        Rc_SetChassisRelax();
        Rc_SetPoleAuto(0.0f, 0.0f);
        Rc_SetOreStoreHome(last_sw_r != DR16_SW_UP);
      } else if (dr16.data.sw_r == DR16_SW_DOWN) {
        if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl)) {
          AutoCtrl_Abort(&auto_ctrl);
        }
        Rc_SetChassisRelax();
        Rc_SetPoleAuto(0.0f, 0.0f);
        Rc_SetOreStoreActiveManual();
      } else if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl)) {
#if AUTO_CTRL_OUTPUT_ENABLE
        chassis_cmd = auto_ctrl.chassis_cmd;
        pole_cmd = auto_ctrl.pole_cmd;
#else
        Rc_SetAutoDryRunCommands();
#endif
        Rc_SetOreStoreRelax();
      } else if (dr16.data.sw_r == DR16_SW_MID) {
        /* 左MID右MID：底盘控制，pole用左摇杆Y轴同时控制 */
        Rc_SetChassisRemote();
        Rc_SetPoleManual(dr16.data.ch_l_y, dr16.data.ch_l_y);
        Rc_SetOreStoreRelax();
      } else {
        Rc_SetChassisRelax();
        Rc_SetPoleAuto(0.0f, 0.0f);
        Rc_SetOreStoreRelax();
      }

      Rc_SetArmDisabled();
      Rc_SetRodRelax();
    } else if (dr16.data.sw_l == DR16_SW_DOWN) {
      const bool use_pc_command = Rc_ShouldUsePcCommand();

      if (!use_pc_command && auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl)) {
        AutoCtrl_Abort(&auto_ctrl);
      }

      /* sw_l DOWN 且 sw_r UP 时才允许使用上位机命令。 */
      if (use_pc_command) {
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
      } else {
        /* 遥控器控制模式（默认机械臂） */
        Rc_SetChassisRelax();
        Rc_SetPoleAuto(0.0f, 0.0f);
        Rc_SetArmRemoteCartesian(
            (dr16.data.sw_r == DR16_SW_UP) ? 1.0f : RC_ARM_FINE_SCALE);
      }
      Rc_SetRodRelax();
      Rc_SetOreStoreRelax();
    }

    Rc_ApplyArmDebugPoseOverride();

    osMessageQueueReset(task_runtime.msgq.chassis.cmd);
    osMessageQueuePut(task_runtime.msgq.chassis.cmd, &chassis_cmd, 0, 0);
    osMessageQueueReset(task_runtime.msgq.pole.cmd);
    osMessageQueuePut(task_runtime.msgq.pole.cmd, &pole_cmd, 0, 0);
    osMessageQueueReset(task_runtime.msgq.arm.cmd);
    osMessageQueuePut(task_runtime.msgq.arm.cmd, &arm_cmd, 0, 0);
    osMessageQueueReset(task_runtime.msgq.rod.cmd);
    osMessageQueuePut(task_runtime.msgq.rod.cmd, &rod_cmd, 0, 0);
    g_rc_ore_store_post_ret = Task_OreStorePostCommand(&ore_store_cmd);

    if (dr16.header.online) {
      if (dr16.data.sw_l == DR16_SW_UP && dr16.data.sw_r == DR16_SW_DOWN &&
          last_sw_r != DR16_SW_DOWN && last_sw_r != DR16_SW_ERR) {
        reset = !reset;
      }
    }

    last_sw_l = dr16.data.sw_l;
    last_sw_r = dr16.data.sw_r;
    task_runtime.stack_water_mark.rc_main = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick);
  }
}
