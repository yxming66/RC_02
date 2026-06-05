# CMD Center C++ API Design

## 目标定位

新 CMD 库定位为机器人控制的“命令生成、调度、裁决中心”。

它负责把多种输入源的数据转化为各输出模块的命令，并在多个候选命令之间完成裁决。它不直接控制电机，也不实现具体执行机构闭环控制。

核心目标是优秀的 API 调用体验：

- 新增输入源时，不需要修改全局枚举、宏表、总上下文结构体和中心调度代码。
- 新增输出模块时，只新增该模块自己的命令路由配置和少量生成器。
- 输入到输出的映射关系应在 API 调用处清楚可见。
- 支持每个输出模块独立仲裁，而不是全局只选一个输入源。
- 支持动态控制权切换、动态优先级、按键组合触发、按住某个键让部分输出切到其他输入源。
- 支持多个输入源混合生成同一个输出命令。
- 适配 STM32 嵌入式约束：静态容量、无异常、无 RTTI、尽量无堆分配。

## 当前淘汰的方向

### 全局模板表方案

类似：

```cpp
using RobotCmd = cmd::Center<
    cmd::Inputs<Dr16Input, PcLinkInput>,
    cmd::Outputs<Chassis_CMD_t, Pole_CMD_t>,
    cmd::Policies<RcPolicy, SafePolicy>
>;
```

这个方案类型安全，但新增模块仍然需要改全局模板列表，本质上接近当前 C 版本的宏表思路，因此淘汰。

### 行为 DSL 方案

类似：

```cpp
cmd::robot()
    .input("rc", Dr16Input{dr16})
    .output("chassis", queue(...))
    .when(mode::drive, DriveCommands{})
    .when(mode::pc, PcCommands{});
```

这个方案业务表达强，但新增输出模块时往往需要去多个 `when(mode, ...)` 规则里补该模块在各模式下的行为。新增/删除模块的改动范围不够局部，因此淘汰。

## 确定方向：D+ 输出模块中心路由

最终方向采用“输出模块为中心，路由显式声明”的设计。

每个输出模块声明：

- 命令类型是什么。
- 最终命令发到哪里。
- 安全命令如何生成。
- 没有候选命令时如何保持。
- 哪些输入源可以生成它。
- 每条输入到输出的路由在什么条件下有效。
- 每条路由的优先级或动态优先级是什么。

示例：

```cpp
cmd_center.output<Chassis_CMD_t>("chassis", queue(task_runtime.msgq.chassis.cmd))
    .safe<ChassisSafe>()
    .hold<ChassisHold>()
    .routes(
        from<Dr16RcInput, RcChassis>()
            .when<UseRcMode>()
            .priority(cmd::Priority::Manual),

        from<Dr16PcInput, PcChassis>()
            .when<UsePcMode>()
            .priority(cmd::Priority::Manual),

        from<AutoCtrlInput, AutoChassis>()
            .when<AutoCtrlBusy>()
            .priority(cmd::Priority::Auto)
    );
```

这段 API 的含义是：

- `Chassis_CMD_t` 是底盘输出命令。
- 命令发布到 `task_runtime.msgq.chassis.cmd`。
- 无有效输入时使用 `ChassisSafe`。
- 无候选但允许保持时使用 `ChassisHold`。
- RC、PC、AutoCtrl 都可以生成底盘命令。
- 每条路由是否参与仲裁由 `.when<...>()` 每帧动态判断。
- 候选命令优先级由命名档位决定。

## 输入源注册

输入源只负责采样、在线判断、事件抽象和向生成器提供数据。

```cpp
cmd_center
    .input<Dr16RcInput>("rc", dr16)
    .input<Dr16PcInput>("pc", dr16)
    .input<PcLinkInput>("pc_link")
    .input<NucInput>("nuc", nuc)
    .input<AutoCtrlInput>("auto_ctrl", auto_ctrl)
    .input<AutoOreInput>("auto_ore", auto_ore_ctrl);
```

新增输入源时：

1. 实现一个输入源类，例如 `NucInput`。
2. 在初始化处加一行 `.input<NucInput>("nuc", nuc)`。
3. 只在它影响的输出模块里加 `.routes(from<NucInput, XxxBuilder>())`。

不需要改中心类、总枚举、宏表、总 `CMD_t` 结构。

## 输出模块注册

输出模块围绕命令类型组织。

```cpp
cmd_center.output<Pole_CMD_t>("pole", queue(task_runtime.msgq.pole.cmd))
    .safe<PoleSafe>()
    .hold<PoleHoldFromFeedback>()
    .routes(
        from<Dr16RcInput, RcPole>()
            .when<UseRcMode>()
            .priority(cmd::Priority::Manual),

        from<PcLinkInput, PcPole>()
            .when<PcControlMode>()
            .priority(cmd::Priority::Remote),

        from<AutoCtrlInput, AutoPole>()
            .when<AutoCtrlBusy>()
            .priority(cmd::Priority::Auto),

        from<AutoOreInput, AutoOrePole>()
            .when<AutoOreBusy>()
            .priority(cmd::Priority::CriticalAuto)
    );
```

新增输出模块时：

1. 实现该模块命令类型已有或新增的 `Xxx_CMD_t`。
2. 实现 `XxxSafe`。
3. 如需要保持当前目标，实现 `XxxHold`。
4. 为相关输入源实现小生成器，例如 `RcXxx`、`PcXxx`、`AutoXxx`。
5. 在初始化处新增一整块 `cmd_center.output<Xxx_CMD_t>(...)`。

不需要修改其他输出模块。

## 命令生成器

命令生成器只负责一条输入到一个输出的映射。

```cpp
struct RcChassis {
  bool operator()(const Dr16RcInput& rc,
                  cmd::Context& ctx,
                  Chassis_CMD_t& out) const {
    if (!rc.online()) {
      return false;
    }

    out.mode = CHASSIS_MODE_INDEPENDENT;
    out.ctrl_vec.vx = rc.right_y() * 2.0f;
    out.ctrl_vec.vy = -rc.right_x() * 2.0f;
    out.ctrl_vec.wz = -rc.left_x() * 3.0f;
    return true;
  }
};
```

返回值含义：

- `true`：本帧生成了有效候选命令，参与仲裁。
- `false`：本帧没有有效候选命令，不参与仲裁。

## 多输入混合生成

某些输出命令可能需要多个输入源共同决定。例如 NUC 给目标，PC 鼠标右键决定是否启用，PC 左键决定是否允许射击。

```cpp
cmd_center.output<Shoot_CMD_t>("shoot", queue(task_runtime.msgq.shoot.cmd))
    .safe<ShootSafe>()
    .routes(
        from<NucInput, Dr16PcInput, NucShootWithPcPermission>()
            .when<RightMouseHeld>()
            .priority(cmd::Priority::CriticalAuto)
    );
```

生成器示例：

```cpp
struct NucShootWithPcPermission {
  bool operator()(const NucInput& nuc,
                  const Dr16PcInput& pc,
                  cmd::Context& ctx,
                  Shoot_CMD_t& out) const {
    if (!nuc.online() || !pc.mouse().right) {
      return false;
    }

    out.ready = true;
    out.firecmd = pc.mouse().left && nuc.target_locked();
    return true;
  }
};
```

## 仲裁机制

仲裁按输出模块独立执行。

每个输出模块每帧收集多个候选命令：

```text
chassis:
  rc         Manual
  pc         Remote
  auto_ctrl  Auto

pole:
  rc         Manual
  pc         Remote
  auto_ctrl  Auto
  auto_ore   CriticalAuto
```

每个输出模块独立选择最终命令。不要做“全局选一个输入源”。

推荐流程：

```text
1. 所有输入源 update
2. 更新状态机和事件
3. 全局 safety 检查
4. 每个输出模块运行 routes，生成候选命令
5. 每个输出模块独立仲裁
6. 无候选则 hold
7. hold 不可用则 safe
8. publish
```

### 优先级

API 层不建议写裸数字，例如 `41`、`42`。

推荐使用命名优先级：

```cpp
enum class Priority : uint8_t {
  Fallback     = 10,
  Manual       = 40,
  Remote       = 60,
  Auto         = 80,
  CriticalAuto = 100,
};
```

调用处：

```cpp
from<Dr16RcInput, RcChassis>().priority(cmd::Priority::Manual)
from<PcLinkInput, PcChassis>().priority(cmd::Priority::Remote)
from<AutoCtrlInput, AutoChassis>().priority(cmd::Priority::Auto)
```

内部可以按数值比较，数值越大优先级越高，但调用者不应频繁使用魔法数字。

同优先级冲突时使用明确的 tie-breaker：

```cpp
.arbitrate(cmd::HighestPriority{
    .tie_break = cmd::TieBreak::KeepLastWinner,
});
```

可选策略：

- `KeepLastWinner`：保持上一帧赢家，避免控制源抖动。
- `RouteOrder`：按 routes 声明顺序。
- `Freshest`：选最新鲜的数据。
- `RejectConflict`：冲突时拒绝候选，走 hold 或 safe。

## 动态控制权切换

路由条件和优先级每帧都可以动态变化。

### RC 和 PC 控制切换

使用状态机保存当前控制源：

```cpp
cmd_center.state<ControlModeState>()
    .toggle<UseRcMode, UsePcMode>()
    .on<Dr16PcInput>(KeyCombo{KEY_CTRL, KEY_SHIFT, KEY_V});
```

输出路由只读取状态：

```cpp
from<Dr16RcInput, RcChassis>()
    .when<UseRcMode>()
    .priority(cmd::Priority::Manual)

from<Dr16PcInput, PcChassis>()
    .when<UsePcMode>()
    .priority(cmd::Priority::Manual)
```

### 按住右键让部分输出使用 NUC

只给需要被 NUC 接管的输出模块添加 NUC 路由。

```cpp
cmd_center.output<Gimbal_CMD_t>("gimbal", queue(...))
    .safe<GimbalSafe>()
    .routes(
        from<Dr16PcInput, PcGimbal>()
            .when<UsePcMode>()
            .priority(cmd::Priority::Manual),

        from<NucInput, NucGimbal>()
            .when<RightMouseHeld>()
            .priority(cmd::Priority::CriticalAuto)
    );
```

底盘没有 NUC 路由，因此右键按住时底盘仍由 RC 或 PC 控制。

### 动态优先级

如果条件启停不够，可以让优先级本身每帧计算：

```cpp
from<PcLinkInput, PcChassis>()
    .priority([](const cmd::Context& ctx) {
      return ctx.state<ControlModeState>().pc_enabled
          ? cmd::Priority::Remote
          : cmd::Priority::Disabled;
    });
```

常规场景优先使用 `.when<...>()` 控制是否参与，动态优先级作为补充能力。

## Safety、Hold、Fallback

安全命令不是普通候选命令，应具备最高覆盖权。

```cpp
cmd_center.global()
    .safety<RemoteOfflineSafety>()
    .safety<ResetSafety>();
```

全局 safety 触发时：

```text
所有输出直接 safe，不进入普通仲裁。
```

模块级 safety 示例：

```cpp
cmd_center.output<OreStore_CMD_t>("ore_store", queue(...))
    .safety<OreStoreHomingSafety>()
    .safe<OreStoreSafe>()
    .hold<OreStoreHoldFromFeedback>()
    .routes(...);
```

每个输出模块的 fallback 顺序：

```text
1. 有候选命令：仲裁选中最终命令
2. 无候选命令但 hold 可用：使用 hold
3. hold 不可用：使用 safe
```

## 模块预设

显式路由适合调试和定制，但稳定后可以封装成模块预设，减少初始化代码。

显式写法：

```cpp
cmd_center.output<Chassis_CMD_t>("chassis", queue(task_runtime.msgq.chassis.cmd))
    .safe<ChassisSafe>()
    .routes(
        from<Dr16RcInput, RcChassis>(),
        from<PcLinkInput, PcChassis>(),
        from<AutoCtrlInput, AutoChassis>()
    );
```

预设写法：

```cpp
cmd_center.install(ChassisCmdModule{
    queue(task_runtime.msgq.chassis.cmd)
});
```

`ChassisCmdModule` 内部仍然使用同一套路由声明：

```cpp
struct ChassisCmdModule {
  template <typename Center>
  void install(Center& center) const {
    center.output<Chassis_CMD_t>("chassis", sink)
        .safe<ChassisSafe>()
        .routes(
            from<Dr16RcInput, RcChassis>(),
            from<PcLinkInput, PcChassis>(),
            from<AutoCtrlInput, AutoChassis>()
        );
  }

  cmd::QueueSink<Chassis_CMD_t> sink;
};
```

这样保留两种使用层级：

- 需要定制时，在初始化处显式写 routes。
- 模块稳定后，用一行 `install(...)`。

## 推荐最终 API 草案

```cpp
static cmd::Center<8, 8, 32> cmd_center;

void CmdCenter_Init() {
  cmd_center
      .input<Dr16RcInput>("rc", dr16)
      .input<Dr16PcInput>("pc", dr16)
      .input<PcLinkInput>("pc_link")
      .input<AutoCtrlInput>("auto_ctrl", auto_ctrl)
      .input<AutoOreInput>("auto_ore", auto_ore_ctrl);

  cmd_center.global()
      .safety<RemoteOfflineSafety>()
      .safety<ResetSafety>();

  cmd_center.output<Chassis_CMD_t>("chassis", queue(task_runtime.msgq.chassis.cmd))
      .safe<ChassisSafe>()
      .hold<ChassisHold>()
      .arbitrate<cmd::HighestPriority>()
      .routes(
          from<Dr16RcInput, RcChassis>()
              .when<UseRcMode>()
              .priority(cmd::Priority::Manual),

          from<Dr16PcInput, PcChassis>()
              .when<UsePcMode>()
              .priority(cmd::Priority::Manual),

          from<PcLinkInput, PcLinkChassis>()
              .when<PcControlMode>()
              .priority(cmd::Priority::Remote),

          from<AutoCtrlInput, AutoChassis>()
              .when<AutoCtrlBusy>()
              .priority(cmd::Priority::Auto),

          from<AutoOreInput, AutoOreChassis>()
              .when<AutoOreBusy>()
              .priority(cmd::Priority::CriticalAuto)
      );

  cmd_center.output<Pole_CMD_t>("pole", queue(task_runtime.msgq.pole.cmd))
      .safe<PoleSafe>()
      .hold<PoleHoldFromFeedback>()
      .arbitrate<cmd::HighestPriority>()
      .routes(
          from<Dr16RcInput, RcPole>()
              .when<UseRcMode>()
              .priority(cmd::Priority::Manual),

          from<PcLinkInput, PcLinkPole>()
              .when<PcControlMode>()
              .priority(cmd::Priority::Remote),

          from<AutoCtrlInput, AutoPole>()
              .when<AutoCtrlBusy>()
              .priority(cmd::Priority::Auto),

          from<AutoOreInput, AutoOrePole>()
              .when<AutoOreBusy>()
              .priority(cmd::Priority::CriticalAuto)
      );
}

void Task_cmd_main(void*) {
  while (1) {
    cmd_center.tick(osKernelGetTickCount()).publish();
  }
}
```

## 仍需后续确定的问题

- `Center<8, 8, 32>` 三个容量参数分别是否为输入数、输出数、路由数，还是拆成更明确的配置结构。
- `queue(...)`、`function(...)` 等 sink API 的最终命名。
- `state<...>()` 状态机 API 是否内置，还是先让用户在 `Context` 里维护。
- `when<...>()` 条件对象是否允许持有参数。
- 动态优先级 API 是否第一版就实现。
- 是否第一阶段直接迁移 `rc_main.c`，还是先建立新库并写示例。
