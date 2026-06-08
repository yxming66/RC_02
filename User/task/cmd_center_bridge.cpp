#include "task/cmd_center_bridge.h"

#include <stddef.h>
#include <cmath>

#include "task/user_task.h"

#if __has_include("module/cmd_center/cmd_center.hpp")
#include "module/cmd_center/cmd_center.hpp"
#include "module/cmd_center/cmd_center_cmsis.hpp"
#define CMD_CENTER_BRIDGE_HAS_CORE 1
#elif __has_include("module/cmd_center/center.hpp")
#include "module/cmd_center/center.hpp"
#define CMD_CENTER_BRIDGE_HAS_CORE 1
#elif __has_include("module/cmd_center/cmd_center.h")
#include "module/cmd_center/cmd_center.h"
#define CMD_CENTER_BRIDGE_HAS_CORE 1
#elif __has_include("module/cmd_center/center.h")
#include "module/cmd_center/center.h"
#define CMD_CENTER_BRIDGE_HAS_CORE 1
#else
#define CMD_CENTER_BRIDGE_HAS_CORE 0
#endif

#ifndef CMD_CENTER_BRIDGE_ENABLE_DPLUS_HOST
#define CMD_CENTER_BRIDGE_ENABLE_DPLUS_HOST 0
#endif

namespace {

const DR16_t *g_dr16 = nullptr;

float ClampScalar(float value, float min_value, float max_value) {
  if (!std::isfinite(value)) {
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

float PoleLiftFromChRes(float ch_res) {
  return ClampScalar(ch_res, -1.0f, 1.0f);
}

struct Dr16RcInput {
  const DR16_t *source;

  bool online() const {
    return source != nullptr && source->header.online;
  }

  float left_x() const {
    return source != nullptr ? source->data.ch_l_x : 0.0f;
  }

  float left_y() const {
    return source != nullptr ? source->data.ch_l_y : 0.0f;
  }

  float right_x() const {
    return source != nullptr ? source->data.ch_r_x : 0.0f;
  }

  float right_y() const {
    return source != nullptr ? source->data.ch_r_y : 0.0f;
  }

  float ch_res() const {
    return source != nullptr ? source->data.ch_res : 0.0f;
  }
};

struct ChassisSafe {
  template <typename Context>
  bool operator()(Context &, Chassis_CMD_t &out) const {
    out.mode = CHASSIS_MODE_RELAX;
    out.mode_rotor = ROTOR_MODE_RAND;
    out.ctrl_vec.vx = 0.0f;
    out.ctrl_vec.vy = 0.0f;
    out.ctrl_vec.wz = 0.0f;
    return true;
  }
};

struct PoleSafe {
  template <typename Context>
  bool operator()(Context &, Pole_CMD_t &out) const {
    out.mode = POLE_MODE_RELAX;
    out.lift[0] = 0.0f;
    out.lift[1] = 0.0f;
    out.auto_target_enable[0] = false;
    out.auto_target_enable[1] = false;
    out.auto_target_lift[0] = 0.0f;
    out.auto_target_lift[1] = 0.0f;
    out.auto_lift_speed[0] = 0.0f;
    out.auto_lift_speed[1] = 0.0f;
    out.auto_lift_accel[0] = 0.0f;
    out.auto_lift_accel[1] = 0.0f;
    out.disable_lift_accel = false;
    return true;
  }
};

struct ArmSimpleSafe {
  template <typename Context>
  bool operator()(Context &, ArmSimple_CMD_t &out) const {
    out.mode = ARM_SIMPLE_MODE_RELAX;
    out.point_mode = ARM_SIMPLE_POINT_NONE;
    out.suction = SUCTION_OFF;
    out.target_joint.joint1 = 0.0f;
    out.target_joint.joint2 = 0.0f;
    out.joint1_vel = 0.0f;
    out.joint1_max_vel_rad_s = 0.0f;
    out.joint2_max_vel_rad_s = 0.0f;
    return true;
  }
};

struct OreStoreSafe {
  template <typename Context>
  bool operator()(Context &, OreStore_CMD_t &out) const {
    out.mode = ORE_STORE_MODE_RELAX;
    out.force_rehome = false;
    out.fixed_ore_cylinder_closed = false;
    out.platform_target_rad = 0.0f;
    return true;
  }
};

struct RodNewSafe {
  template <typename Context>
  bool operator()(Context &, RodNew_CMD_t &out) const {
    out.mode = ROD_NEW_MODE_RELAX;
    out.pose = ROD_NEW_POSE_STANDBY;
    out.grip = ROD_NEW_GRIP_RELEASE;
    out.target_angle_rad = 0.0f;
    return true;
  }
};

struct RcChassis {
  template <typename Context>
  bool operator()(const Dr16RcInput &rc, Context &, Chassis_CMD_t &out) const {
    if (!rc.online()) {
      return false;
    }

    out.mode = CHASSIS_MODE_INDEPENDENT;
    out.mode_rotor = ROTOR_MODE_RAND;
    out.ctrl_vec.vx = rc.right_y() * 2.0f;
    out.ctrl_vec.vy = -rc.right_x() * 2.0f;
    out.ctrl_vec.wz = -rc.left_x() * 3.0f;
    return true;
  }
};

struct RcPole {
  template <typename Context>
  bool operator()(const Dr16RcInput &rc, Context &, Pole_CMD_t &out) const {
    if (!rc.online()) {
      return false;
    }

    const float lift =
        ClampScalar(rc.left_y() + PoleLiftFromChRes(rc.ch_res()), -1.0f, 1.0f);

    out.mode = POLE_MODE_ACTIVE;
    out.lift[0] = lift;
    out.lift[1] = lift;
    out.auto_target_enable[0] = false;
    out.auto_target_enable[1] = false;
    out.auto_target_lift[0] = 0.0f;
    out.auto_target_lift[1] = 0.0f;
    out.auto_lift_speed[0] = 0.0f;
    out.auto_lift_speed[1] = 0.0f;
    out.auto_lift_accel[0] = 0.0f;
    out.auto_lift_accel[1] = 0.0f;
    out.disable_lift_accel = false;
    return true;
  }
};

struct RcArmSimple {
  template <typename Context>
  bool operator()(const Dr16RcInput &rc, Context &ctx,
                  ArmSimple_CMD_t &out) const {
    if (!rc.online()) {
      return false;
    }

    return ArmSimpleSafe{}(ctx, out);
  }
};

struct RcOreStore {
  template <typename Context>
  bool operator()(const Dr16RcInput &rc, Context &ctx,
                  OreStore_CMD_t &out) const {
    if (!rc.online()) {
      return false;
    }

    return OreStoreSafe{}(ctx, out);
  }
};

struct RcRodNew {
  template <typename Context>
  bool operator()(const Dr16RcInput &rc, Context &ctx, RodNew_CMD_t &out) const {
    if (!rc.online()) {
      return false;
    }

    return RodNewSafe{}(ctx, out);
  }
};

#if CMD_CENTER_BRIDGE_HAS_CORE && CMD_CENTER_BRIDGE_ENABLE_DPLUS_HOST
static cmd::Center<8, 8, 32> g_cmd_center;

static void ConfigureCmdCenterRoutes() {
  g_cmd_center
      .input<Dr16RcInput>("rc", Dr16RcInput{g_dr16});

  g_cmd_center.output<Chassis_CMD_t>(
                  "chassis", cmd::queue(task_runtime.msgq.chassis.cmd))
      .safe<ChassisSafe>()
      .arbitrate<cmd::HighestPriority>()
      .routes(cmd::from<Dr16RcInput, RcChassis>()
                  .priority(cmd::Priority::Manual));

  g_cmd_center.output<Pole_CMD_t>(
                  "pole", cmd::queue(task_runtime.msgq.pole.cmd))
      .safe<PoleSafe>()
      .arbitrate<cmd::HighestPriority>()
      .routes(cmd::from<Dr16RcInput, RcPole>()
                  .priority(cmd::Priority::Manual));

  g_cmd_center.output<ArmSimple_CMD_t>(
                  "arm_simple", cmd::queue(task_runtime.msgq.arm_simple.cmd))
      .safe<ArmSimpleSafe>()
      .arbitrate<cmd::HighestPriority>()
      .routes(cmd::from<Dr16RcInput, RcArmSimple>()
                  .priority(cmd::Priority::Manual));

  g_cmd_center.output<OreStore_CMD_t>(
                  "ore_store", cmd::queue(task_runtime.msgq.ore_store.cmd))
      .safe<OreStoreSafe>()
      .arbitrate<cmd::HighestPriority>()
      .routes(cmd::from<Dr16RcInput, RcOreStore>()
                  .priority(cmd::Priority::Manual));

  g_cmd_center.output<RodNew_CMD_t>(
                  "rod", cmd::queue(task_runtime.msgq.rod.cmd))
      .safe<RodNewSafe>()
      .arbitrate<cmd::HighestPriority>()
      .routes(cmd::from<Dr16RcInput, RcRodNew>()
                  .priority(cmd::Priority::Manual));
}
#endif

}  // namespace

extern "C" void CmdCenterBridge_SetDr16Source(const DR16_t *dr16) {
  g_dr16 = dr16;
}

extern "C" bool CmdCenterBridge_IsCoreAvailable(void) {
  return CMD_CENTER_BRIDGE_HAS_CORE != 0;
}

extern "C" void CmdCenterBridge_Init(void) {
#if CMD_CENTER_BRIDGE_HAS_CORE && CMD_CENTER_BRIDGE_ENABLE_DPLUS_HOST
  ConfigureCmdCenterRoutes();
#endif
}

extern "C" bool CmdCenterBridge_Tick(uint32_t now_ms,
                                     CmdCenterBridge_OutputFrame_t *outputs) {
  (void)now_ms;
  (void)outputs;

#if CMD_CENTER_BRIDGE_HAS_CORE && CMD_CENTER_BRIDGE_ENABLE_DPLUS_HOST
  g_cmd_center.tick(now_ms).publish();
  return true;
#else
  return false;
#endif
}
