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
#include "module/rod.h"
#include "module/autoCtrlAPI/api/auto_ctrl_api.h"
#include "module/autoCtrlAPI/core/auto_ctrl_def.h"
#include "main.h"
#include <string.h>
/* USER INCLUDE END */

#ifndef AUTO_CTRL_RC_YAW_POS_90_RAD
#define AUTO_CTRL_RC_YAW_POS_90_RAD (1.57079632679f)
#endif

#ifndef AUTO_CTRL_RC_YAW_NEG_90_RAD
#define AUTO_CTRL_RC_YAW_NEG_90_RAD (-1.57079632679f)
#endif

#ifndef AUTO_CTRL_RC_YAW_TOLERANCE_RAD
#define AUTO_CTRL_RC_YAW_TOLERANCE_RAD (0.0872664626f)
#endif

#ifndef AUTO_CTRL_RC_DIR_THRESHOLD
#define AUTO_CTRL_RC_DIR_THRESHOLD (0.35f)
#endif

#ifndef AUTO_CTRL_OUTPUT_ENABLE
#define AUTO_CTRL_OUTPUT_ENABLE (1u)
#endif

#ifndef ARM_RC_Y_SPEED_MPS
#define ARM_RC_Y_SPEED_MPS (0.08f)
#endif

#ifndef ARM_RC_Z_SPEED_MPS
#define ARM_RC_Z_SPEED_MPS (0.08f)
#endif

#ifndef ARM_RC_PITCH_RATE_RAD_S
#define ARM_RC_PITCH_RATE_RAD_S (0.45f)
#endif

/* USER STRUCT BEGIN */
DR16_t dr16;
static Chassis_CMD_t chassis_cmd;
static Pole_CMD_t pole_cmd;
static Arm_CMD_t arm_cmd;
static DR16_SwitchPos_t last_sw_l = DR16_SW_ERR;
static DR16_SwitchPos_t last_sw_r = DR16_SW_ERR;
static Rod_CMD_t rod_cmd;
extern bool reset;
extern auto_ctrl_t auto_ctrl;
extern bool auto_ctrl_inited;

static auto_ctrl_travel_dir_e auto_ctrl_requested_dir =
    AUTO_CTRL_TRAVEL_DIR_HEAD_FORWARD;

static void Rc_SetRodRelax(void) {
  rod_cmd.mode = ROD_MODE_RELAX;
  rod_cmd.pose = ROD_POSE_DOWN;
  rod_cmd.sequence_trigger = false;
  rod_cmd.grip_done = false;
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
  arm_cmd.ctrl_type = ARM_CTRL_REMOTE_CARTESIAN;
  arm_cmd.frame = ARM_CTRL_FRAME_WORLD;
}

static void Rc_SetArmRemoteCartesian(ArmControlFrame_t frame,
                                     bool sync_target) {
  memset(&arm_cmd.joy_vel, 0, sizeof(arm_cmd.joy_vel));
  memset(&arm_cmd.target_delta, 0, sizeof(arm_cmd.target_delta));
  arm_cmd.enable = true;
  arm_cmd.set_target_as_current = sync_target;
  arm_cmd.ctrl_type = ARM_CTRL_REMOTE_CARTESIAN;
  arm_cmd.frame = frame;
  arm_cmd.joy_vel.y = ARM_RC_Y_SPEED_MPS * dr16.data.ch_r_y;
  arm_cmd.joy_vel.z = ARM_RC_Z_SPEED_MPS * dr16.data.ch_l_y;
  arm_cmd.joy_vel.pitch = -ARM_RC_PITCH_RATE_RAD_S * dr16.data.ch_l_x;
}

static void Rc_SetAutoDryRunCommands(void) {
  chassis_cmd.mode = CHASSIS_MODE_RELAX;
  chassis_cmd.ctrl_vec.vx = 0.0f;
  chassis_cmd.ctrl_vec.vy = 0.0f;
  chassis_cmd.ctrl_vec.wz = 0.0f;

  Rc_SetPoleManual(0.0f, 0.0f);
  pole_cmd.mode = POLE_MODE_RELAX;
}

static bool Rc_ShouldExitAutoCtrlBySwitch(void) {
  return dr16.header.online &&
         (dr16.data.sw_l == DR16_SW_MID || dr16.data.sw_l == DR16_SW_DOWN) &&
         last_sw_r == DR16_SW_UP && dr16.data.sw_r == DR16_SW_MID;
}

static auto_ctrl_travel_dir_e Rc_GetRequestedAutoCtrlDir(void) {
  if (dr16.data.ch_r_x <= -AUTO_CTRL_RC_DIR_THRESHOLD) {
    return AUTO_CTRL_TRAVEL_DIR_TAIL_FORWARD;
  }
  if (dr16.data.ch_r_x >= AUTO_CTRL_RC_DIR_THRESHOLD) {
    return AUTO_CTRL_TRAVEL_DIR_HEAD_FORWARD;
  }
  return auto_ctrl_requested_dir;
}

static void Rc_TryStartAutoCtrlBySwitch(uint32_t now_ms) {
  if (!dr16.header.online || !auto_ctrl_inited || AutoCtrl_IsBusy(&auto_ctrl)) {
    return;
  }

  if (dr16.data.sw_l != DR16_SW_DOWN) {
    return;
  }

  if (last_sw_r != DR16_SW_MID) {
    return;
  }

  auto_ctrl_requested_dir = Rc_GetRequestedAutoCtrlDir();

  if (dr16.data.sw_r == DR16_SW_UP) {
    (void)AutoCtrl_StartTemplate(
        &auto_ctrl, AUTO_CTRL_TEMPLATE_ASCEND_400_STD,
        auto_ctrl_requested_dir, AUTO_CTRL_RC_YAW_POS_90_RAD,
        AUTO_CTRL_RC_YAW_TOLERANCE_RAD,
        AUTO_CTRL_SENSOR_MODE_SICK_FRONT_AND_BOTTOM, now_ms);
  } else if (dr16.data.sw_r == DR16_SW_DOWN) {
    (void)AutoCtrl_StartTemplate(
        &auto_ctrl, AUTO_CTRL_TEMPLATE_DESCEND_400_STD,
        auto_ctrl_requested_dir, AUTO_CTRL_RC_YAW_NEG_90_RAD,
        AUTO_CTRL_RC_YAW_TOLERANCE_RAD,
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
  DR16_Init(&dr16);
  DR16_StartDmaRecv(&dr16);
  Rc_SetArmDisabled();

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

    if (auto_ctrl_inited && !AutoCtrl_IsBusy(&auto_ctrl)) {
      auto_ctrl_requested_dir = Rc_GetRequestedAutoCtrlDir();
    }

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
          chassis_cmd.ctrl_vec.vx = 2.0f * dr16.data.ch_r_x;
          chassis_cmd.ctrl_vec.vy = 2.0f * dr16.data.ch_r_y;
          chassis_cmd.ctrl_vec.wz = -2.0f * dr16.data.ch_l_x;
          Rc_SetPoleManual(dr16.data.ch_l_y, dr16.data.ch_l_y);
          break;
        case DR16_SW_MID:
          chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
          chassis_cmd.ctrl_vec.vx = dr16.data.ch_r_x;
          chassis_cmd.ctrl_vec.vy = dr16.data.ch_r_y;
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
    } else if (dr16.data.sw_l == DR16_SW_MID) {
      ArmControlFrame_t frame = ARM_CTRL_FRAME_WORLD;

      if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl)) {
        AutoCtrl_Abort(&auto_ctrl);
      }

      chassis_cmd.mode = CHASSIS_MODE_RELAX;
      chassis_cmd.ctrl_vec.vx = 0.0f;
      chassis_cmd.ctrl_vec.vy = 0.0f;
      chassis_cmd.ctrl_vec.wz = 0.0f;
      Rc_SetPoleManual(0.0f, 0.0f);
      pole_cmd.mode = POLE_MODE_RELAX;

      switch (dr16.data.sw_r) {
        case DR16_SW_UP:
          frame = ARM_CTRL_FRAME_WORLD;
          break;
        case DR16_SW_MID:
          frame = ARM_CTRL_FRAME_TOOL;
          break;
        case DR16_SW_DOWN:
          frame = ARM_CTRL_FRAME_HEADING;
          break;
        default:
          frame = ARM_CTRL_FRAME_WORLD;
          break;
      }

      Rc_SetArmRemoteCartesian(
          frame, last_sw_l != DR16_SW_MID || last_sw_r != dr16.data.sw_r);
      Rc_SetRodRelax();
    } else if (dr16.data.sw_l == DR16_SW_DOWN) {
      if (auto_ctrl_inited && AutoCtrl_IsBusy(&auto_ctrl)) {
#if AUTO_CTRL_OUTPUT_ENABLE
        chassis_cmd = auto_ctrl.chassis_cmd;
        pole_cmd = auto_ctrl.pole_cmd;
#else
        Rc_SetAutoDryRunCommands();
#endif
      } else {
        chassis_cmd.mode = CHASSIS_MODE_INDEPENDENT;
        chassis_cmd.ctrl_vec.vx = dr16.data.ch_r_x;
        chassis_cmd.ctrl_vec.vy = dr16.data.ch_r_y;
        chassis_cmd.ctrl_vec.wz = -dr16.data.ch_l_x;
        Rc_SetPoleAuto(0.0f, 0.0f);
      }

      Rc_SetArmDisabled();
      Rc_SetRodRelax();
    }

    osMessageQueueReset(task_runtime.msgq.chassis.cmd);
    osMessageQueuePut(task_runtime.msgq.chassis.cmd, &chassis_cmd, 0, 0);
    osMessageQueueReset(task_runtime.msgq.pole.cmd);
    osMessageQueuePut(task_runtime.msgq.pole.cmd, &pole_cmd, 0, 0);
    osMessageQueueReset(task_runtime.msgq.arm.cmd);
    osMessageQueuePut(task_runtime.msgq.arm.cmd, &arm_cmd, 0, 0);
    osMessageQueueReset(task_runtime.msgq.rod.cmd);
    osMessageQueuePut(task_runtime.msgq.rod.cmd, &rod_cmd, 0, 0);

    if (dr16.header.online) {
      if (dr16.data.sw_l == DR16_SW_UP && dr16.data.sw_r == DR16_SW_DOWN &&
          last_sw_r != DR16_SW_DOWN && last_sw_r != DR16_SW_ERR) {
        reset = !reset;
      }
    }

    last_sw_l = dr16.data.sw_l;
    last_sw_r = dr16.data.sw_r;
    osDelayUntil(tick);
  }
}
