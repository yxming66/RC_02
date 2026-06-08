#include "component/cmd_center/cmd_center.hpp"
#include "component/cmd_center/cmd_center_cmsis.hpp"
#include "module/arm_simple.h"
#include "module/chassis.h"
#include "module/ore_store.h"
#include "module/pole.h"
#include "module/rod_new.h"

namespace {

struct MockCommand {
  int mode = 0;
  float value = 0.0f;
};

struct RcInput {
  bool enabled = true;
  float stick = 0.5f;

  bool update(cmd::Context&) { return true; }
  bool online() const { return enabled; }
};

struct PcInput {
  bool enabled = true;
  bool right_mouse = false;

  bool online() const { return enabled; }
};

struct UseRc {
  bool operator()(const cmd::Context& ctx) const {
    const auto* rc = ctx.input<RcInput>();
    return rc != nullptr && rc->online();
  }
};

struct PcRightMouseHeld {
  bool operator()(const cmd::Context& ctx) const {
    const auto* pc = ctx.input<PcInput>();
    return pc != nullptr && pc->right_mouse;
  }
};

struct SafeCommand {
  bool operator()(MockCommand& out) const {
    out.mode = 0;
    out.value = 0.0f;
    return true;
  }
};

struct NeverSafe {
  bool operator()(const cmd::Context&) const { return false; }
};

struct HoldCommand {
  bool operator()(cmd::Context&, MockCommand& out) const {
    out.mode = 1;
    out.value = 0.0f;
    return true;
  }
};

struct RcBuilder {
  bool operator()(const RcInput& rc, cmd::Context&, MockCommand& out) const {
    out.mode = 2;
    out.value = rc.stick;
    return true;
  }
};

struct PcRcMixedBuilder {
  bool operator()(const PcInput& pc, const RcInput& rc,
                  MockCommand& out) const {
    if (!pc.right_mouse || !rc.online()) {
      return false;
    }
    out.mode = 3;
    out.value = rc.stick * 2.0f;
    return true;
  }
};

struct StoreSink {
  bool operator()(const MockCommand& cmd) {
    last = cmd;
    return true;
  }

  MockCommand last{};
};

struct NullSink {
  template <typename Cmd>
  bool operator()(const Cmd&) const {
    return true;
  }
};

struct DefaultSafe {
  template <typename Cmd>
  bool operator()(Cmd& out) const {
    out = Cmd{};
    return true;
  }
};

}  // namespace

void CmdCenter_CompileCheck() {
  cmd::Center<2, 1, 8> center;

  center.input<RcInput>("rc").input<PcInput>("pc");
  center.global().safety<NeverSafe>();

  center.output<MockCommand>("mock", StoreSink{})
      .safe<SafeCommand>()
      .hold<HoldCommand>()
      .arbitrate<cmd::HighestPriority>()
      .routes(
          cmd::from<RcInput, RcBuilder>()
              .when<UseRc>()
              .priority(cmd::Priority::Manual),
          cmd::from<PcInput, RcInput, PcRcMixedBuilder>()
              .when<PcRightMouseHeld>()
              .priority(cmd::Priority::CriticalAuto));

  center.tick(1).publish();
}

void CmdCenter_ModuleCommandCompileCheck() {
  cmd::Center<1, 5, 8> center;

  center.input<RcInput>("rc");
  center.output<Chassis_CMD_t>("chassis", NullSink{}).safe<DefaultSafe>();
  center.output<Pole_CMD_t>("pole", NullSink{}).safe<DefaultSafe>();
  center.output<ArmSimple_CMD_t>("arm_simple", NullSink{}).safe<DefaultSafe>();
  center.output<OreStore_CMD_t>("ore_store", NullSink{}).safe<DefaultSafe>();
  center.output<RodNew_CMD_t>("rod", NullSink{}).safe<DefaultSafe>();

  center.tick(1).publish();
}

void CmdCenter_CmsisSinkCompileCheck() {
  cmd::Center<1, 1, 2> center;

  center.input<RcInput>("rc");
  center.output<Chassis_CMD_t>("chassis", cmd::queue(nullptr))
      .safe<DefaultSafe>();

  center.tick(1).publish();
}
