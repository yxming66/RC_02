# PC 侧基于有无矿反馈控制 R2 自动动作流程

本文档说明 PC 如何结合矿仓/机械臂占矿反馈，通过单一 AutoAction 调用接口控制 R2 上膛、存矿、普通放矿、分步放矿和红外三层放矿。

适用代码与协议：

- `User/module/mrlink_pc_comm/mrlink_pc_comm.h`
- `User/module/mrlink_pc_comm/pc_messages.hpp`
- `User/task/pc_comm_task.c`
- `docs/pccomm_protocol.md`

## 1. PC 需要订阅的反馈

### 1.1 矿仓/机械臂占矿反馈 `0x95`

`PC_FEEDBACK_ORE_STORE = 0x95`，payload 长度 12 字节：

```python
mode, all_homed, online_mask, homed_mask, platform_position_rad, low, high, arm, release_grid = struct.unpack(
    "<BBBBfBBBB", payload
)
```

| 字段 | 含义 | PC 用途 |
|---|---|---|
| `low` | 低位变形机构是否有矿，`0/1` | 判断低位是否可存矿或可上膛 |
| `high` | 高位变形机构是否有矿，`0/1` | 判断高位是否可存矿或可上膛 |
| `arm` | Arm 是否持矿，`0/1` | 判断是否需要上膛、存矿或放矿 |
| `release_grid` | 当前放矿目标格是否已有矿，`0/1` | `STEP1` 终态后决定是否调用 `STEP2` |

推荐 PC 内部状态：

```python
ore = {
    "low_has_ore": bool(low),
    "high_has_ore": bool(high),
    "arm_has_ore": bool(arm),
    "release_grid_has_ore": bool(release_grid),
}
```

### 1.2 AutoAction 整体反馈 `0x96`

`PC_FEEDBACK_AUTO_ACTION = 0x96`，payload 固定为 4 字节：

```python
action, busy, finished, result = struct.unpack("<BBBB", payload)
```

| 字段 | 含义 | PC 用途 |
|---|---|---|
| `action` | 当前运行或最近结束的 action ID | 匹配本次调用 |
| `busy` | 整体动作是否仍在执行 | `busy=1` 时只等待或中止，不调用下一动作 |
| `finished` | 是否已有整体终态 | `busy=0 && finished=1` 表示底层已终止 |
| `result` | `0=SUCCESS`，`1=FAIL` | 仅 `finished=1` 时读取 |

PC 必须把一个 action 当作不可拆分的整体：

1. 正常调用后等待相同 `action` 的 `busy=0 && finished=1`。
2. `busy=1` 期间不启动下一 action，不提前接管相关模块。
3. 不根据底盘停止、Pole 到位、Arm 到位、矿仓变化或融合分支完成判断整体结束。
4. 所有放矿类动作会报告真实失败；无矿导致的启动失败返回 `result=1` 和放矿失败位，不再映射为成功。其他动作延续兼容策略，但 synthetic success 也只能在底层真实终态后出现。
5. 动作终态后再结合 `0x95` 校验矿物实际位置。

### 1.3 R1/R2 红外对接反馈 `0x9A`

`PC_FEEDBACK_IR_DOCK = 0x9A`，payload 长度 3 字节：

```python
valid, fresh, command = struct.unpack("<BBB", payload)
```

`command` 是最近收到的合法红外命令 `1~4`。`0x9A` 用于观察红外链路与最近命令，不用于判断 action 38 的整体完成；action 38 的运行和终态只看 `0x96`。

## 2. 单动作调用方法

PC 只通过 `PC_CMD_AUTO_ACTION = 0x16` 下发 1 字节 action：

```python
def send_auto_action(action_id: int):
    send_mrlink_frame(0x16, struct.pack("<B", action_id))
```

调用规则：

1. 每次正常调用先发送 `action=0` 清除 latch。
2. 再发送目标 action。
3. 等待相同 action 的 `busy=0 && finished=1`。
4. 读取 `result`，并结合 `0x95` 做业务校验。
5. 只有整体终态后才能调用下一 action。

同一个非零 action 若未先发送 `0`，重复发送不会再次触发。中止当前动作时发送 `action=1`，然后等待 `busy=0`；底层中止完成前资源仍由当前动作持有。

```python
def start_auto_action(action_id: int):
    send_auto_action(0)
    send_auto_action(action_id)


def auto_action_done(auto, action_id: int) -> bool:
    return auto.action == action_id and not auto.busy and auto.finished
```

## 3. 本流程相关 action

| ID | 名称 | 用途 |
|---:|---|---|
| `1` | `ABORT` | 中止当前整体 action |
| `2` | `STORE` | 将 Arm 持有矿存入低位或高位空仓 |
| `3` | `RELEASE` | 普通完整放矿 |
| `4` | `RELEASE_LIFT_DETECT` | 带抬升检测的完整放矿 |
| `5` | `CHAMBER` | 从低位或高位取矿到 Arm |
| `34` | `RELEASE_STEP1` | 独立观察 action；终态后保持 Arm 持矿 |
| `35` | `RELEASE_STEP2` | 普通放矿第二阶段，后续独立调用 |
| `36` | `RELEASE_LIFT_DETECT_STEP1` | 带抬升检测的独立观察 action；终态后保持 Arm 持矿 |
| `37` | `RELEASE_LIFT_DETECT_STEP2` | 带抬升检测放矿第二阶段，后续独立调用 |
| `38` | `RELEASE_IR_LIFT_DETECT` | 红外协同三层放矿整体 action |

普通动作、融合动作和以上每一个分步 action 都共享同一个 active 槽。PC 不得同时运行两个 action。

## 4. PC 基础状态

```python
from dataclasses import dataclass


@dataclass
class AutoActionState:
    action: int = 0
    busy: bool = False
    finished: bool = False
    result: int = 0


@dataclass
class OreState:
    low_has_ore: bool = False
    high_has_ore: bool = False
    arm_has_ore: bool = False
    release_grid_has_ore: bool = False


@dataclass
class IrDockState:
    valid: bool = False
    fresh: bool = False
    command: int = 0
```

反馈更新规则：

1. 从 `0x95` 更新 `OreState`。
2. 从 `0x96` 更新唯一的 `AutoActionState`。
3. 从 `0x9A` 更新 `IrDockState`。
4. `auto.busy` 为真时，不做下一动作决策。
5. `auto.finished` 为真后，先处理结果和占矿校验，再进入下一业务状态。

## 5. 上膛流程

### 5.1 决策条件

```python
need_chamber = (not ore.arm_has_ore) and (ore.low_has_ore or ore.high_has_ore)
```

以下情况不调用 `CHAMBER`：

| 条件 | 原因 |
|---|---|
| `auto.busy` | 当前已有整体 action 在运行 |
| `ore.arm_has_ore` | Arm 已持矿 |
| `not ore.low_has_ore and not ore.high_has_ore` | 矿仓无矿可上膛 |

### 5.2 调用与校验

```python
if not auto.busy and need_chamber:
    start_auto_action(5)
```

PC 等待：

```python
if auto_action_done(auto, 5):
    chamber_reported_success = auto.result == 0
    chamber_physical_ok = ore.arm_has_ore
```

`CHAMBER` 属于兼容结果动作，因此业务成功应同时要求整体终态和 `0x95.arm_has_ore == 1`。

## 6. 存矿流程

### 6.1 决策条件

```python
need_store = ore.arm_has_ore and ((not ore.low_has_ore) or (not ore.high_has_ore))
```

以下情况不调用 `STORE`：

| 条件 | 原因 |
|---|---|
| `auto.busy` | 当前已有整体 action 在运行 |
| `not ore.arm_has_ore` | Arm 无矿 |
| `ore.low_has_ore and ore.high_has_ore` | 两个仓位都满 |

### 6.2 调用与校验

```python
if not auto.busy and need_store:
    start_auto_action(2)
```

等待 action 2 整体终态后，推荐校验：

```python
store_physical_ok = (
    not ore.arm_has_ore
    and (ore.low_has_ore or ore.high_has_ore)
)
```

低位存矿的收尾颠动属于 action 2 内部过程。在颠动及平台回到 `STANDBY` 前，PC 仍会看到 `busy=1`，不能提前调用下一动作。

## 7. 普通完整放矿

如果不需要先观察目标格，可调用完整 action 3：

```python
if not auto.busy and ore.arm_has_ore:
    start_auto_action(3)
```

等待 action 3 整体终态后校验：

```python
release_physical_ok = not ore.arm_has_ore
```

PC 不得因 Arm 开始离开放矿位或 `arm_has_ore` 提前变化而启动下一动作，必须先收到 action 3 的整体终态。

## 8. 普通分步放矿

分步放矿用于 PC 在观察完成后，根据目标格是否已有矿决定是否释放。

### 8.1 调用 `STEP1`

```python
if not auto.busy and ore.arm_has_ore:
    start_auto_action(34)
```

action 34 完成观察后立即进入自己的整体终态。此时 AutoAction 已结束，但 Arm 仍保持持矿状态。PC 必须先看到：

```python
auto_action_done(auto, 34)
```

再读取 `ore.release_grid_has_ore`：

| 目标格状态 | PC 行为 |
|---:|---|
| `0` | 目标格为空，可独立调用 action 35 |
| `1` | 目标格已有矿，不调用 action 35，重新规划目标格 |

### 8.2 独立调用 `STEP2`

```python
if auto_action_done(auto, 34) and not ore.release_grid_has_ore:
    start_auto_action(35)
```

`start_auto_action(35)` 会重新发送 `action=0` 和 `action=35`。action 35 是新的整体动作，PC 必须重新等待其自身终态。

两个阶段之间没有隐式续接。action 34 失败时不得调用 action 35；目标格有矿时也不得调用 action 35。

## 9. 带抬升检测的分步放矿

action 36 和 action 37 的调用原则与普通分步放矿一致：

```python
start_auto_action(36)

if auto_action_done(auto, 36) and auto.result == 0:
    if not ore.release_grid_has_ore:
        start_auto_action(37)
```

- action 36 是真实结果动作；`result=1` 时停止流程，不调用 action 37。
- action 36 终态后保持 Arm 持矿。
- action 37 必须通过新的 `0x16` 调用独立启动，并等待自己的整体终态。

## 10. 红外三层放矿 action 38

action 38 不拆分为 `STEP1/STEP2`，也不使用前方目标格光电/SICK 决定是否释放。它由 PC 启动，并在运行中由 R1 红外命令控制放矿循环和正常结束。

### 10.1 启动与运行

```python
if need_zone3_release and not auto.busy:
    start_auto_action(38)
```

时序：

1. PC 发送 `action=0`，再发送 `action=38`。
2. RC02 展开 Pole；必要时从矿仓上膛；Arm 到 `WAIT_RELEASE_ORE`。
3. 动作整个运行期保持自动路由和完整资源租约，`0x96` 保持 `action=38, busy=1, finished=0`。
4. 平台、Arm 和 Pole 就绪后，R1 发送的新红外命令 `3` 触发一次放矿循环。
5. 放矿循环完成后 Arm 返回等待位，action 38 继续运行，`busy` 仍为 1，资源不释放。
6. 若业务需要，可在机构再次就绪后由 R1 再发送命令 `3`，执行下一次放矿循环。
7. R1 发送红外命令 `4` 后，RC02 正常收尾，Arm 回到 `STANDBY`；底层成功后 `0x96` 才发布整体终态并释放资源。

机构就绪前到达的红外命令 `3` 不会延后补触发。R1 必须在机构就绪后重新发送。PC 可以用 `0x9A` 观察最近红外命令，但不能用它代替 action 38 的整体终态。

### 10.2 中止

PC 需要中止 action 38 时发送：

```python
send_auto_action(1)
```

发送中止命令后继续等待 `0x96.busy == 0`。在底层真正进入 `ABORTED` 前：

- `busy` 保持 1；
- 自动路由保持有效；
- CHASSIS、POLE、ARM、STORE、ROD 和共享气路等完整资源不释放；
- PC 不得启动下一 action 或提前接管模块。

wire `result` 继续遵循兼容映射；是否完成中止首先以 `busy=0` 为准。

### 10.3 异常处理

| 场景 | PC/固件行为 |
|---|---|
| action 38 启动条件不满足 | 等待其整体终态并检查占矿/配置；修复后先发 `0` 再重新调用 38 |
| 另一个 action 正在运行 | 不调用 38，等待当前 action 整体终态 |
| 红外命令 `3` 在机构就绪前到达 | 不执行放矿；R1 在就绪后重新发送 `3` |
| 前方目标格检测为有矿 | action 38 忽略该检测，放矿许可只由红外命令 `3` 决定 |
| PC 需要停止 | 发送 `0x16 action=1` 并等待 `busy=0` |
| 收到红外旧命令 `5/6` | 按无效帧处理，不改变动作状态 |

## 11. 推荐总控优先级

1. 安全/中止最高优先级；需要停止时发送 `action=1` 并等待 `busy=0`。
2. `auto.busy` 为真时只监听反馈，不调用任何新的普通 action。
3. 需要红外三层放矿时调用 action 38，并持续等待红外 `4` 或中止后的整体终态。
4. 需要分步放矿时，先独立调用 `STEP1`；其终态后再按目标格状态决定是否独立调用 `STEP2`。
5. Arm 无矿且低/高位有矿时，可调用 `CHAMBER=5`。
6. Arm 有矿且矿仓有空位时，可调用 `STORE=2`。
7. Arm 有矿且需要放矿时，调用完整放矿或分步放矿 action。
8. 每个动作终态后结合 `0x95` 校验物理占矿，再做下一次决策。

## 12. 最小 PC 侧示例

```python
import struct

AUTO_ABORT = 1
AUTO_STORE = 2
AUTO_RELEASE = 3
AUTO_CHAMBER = 5
AUTO_RELEASE_STEP1 = 34
AUTO_RELEASE_STEP2 = 35
AUTO_RELEASE_LIFT_STEP1 = 36
AUTO_RELEASE_LIFT_STEP2 = 37
AUTO_RELEASE_IR = 38


def on_feedback_ore_store(payload):
    _, _, _, _, _, low, high, arm, grid = struct.unpack("<BBBBfBBBB", payload)
    return {
        "low_has_ore": bool(low),
        "high_has_ore": bool(high),
        "arm_has_ore": bool(arm),
        "release_grid_has_ore": bool(grid),
    }


def on_feedback_auto_action(payload):
    action, busy, finished, result = struct.unpack("<BBBB", payload)
    return {
        "action": action,
        "busy": bool(busy),
        "finished": bool(finished),
        "result": result,
    }


def choose_next_action(ore, auto, flow):
    if auto["busy"]:
        return None

    if flow.need_zone3_release:
        return AUTO_RELEASE_IR

    if flow.waiting_release_step2:
        if ore["release_grid_has_ore"]:
            return None
        return AUTO_RELEASE_STEP2

    if not ore["arm_has_ore"] and (ore["low_has_ore"] or ore["high_has_ore"]):
        return AUTO_CHAMBER

    if ore["arm_has_ore"] and (
        not ore["low_has_ore"] or not ore["high_has_ore"]
    ):
        return AUTO_STORE

    return None


def invoke_selected_action(action_id):
    if action_id is None:
        return
    send_auto_action(0)
    send_auto_action(action_id)
```

`waiting_release_step2` 只能在对应 `STEP1` 已收到整体成功终态后设置。选择函数返回 action 后，调用方必须进入等待状态，在该 action 整体终态前不得再次调用 `choose_next_action()` 启动别的动作。

## 13. 注意事项

1. `0x95` 是上膛、存矿和放矿后的物理占矿校验来源。
2. `0x96` 是唯一 AutoAction 整体运行/终态反馈，payload 必须按 `<BBBB>` 解析。
3. PC 任意时刻只调用一个 action，并等待整体终态后再做下一步。
4. 融合动作内部并行不对 PC 暴露，不能据此提前导航或接管模块。
5. `STEP1` 和 `STEP2` 是两个独立 action；`STEP1` 终态保持 Arm 持矿，`STEP2` 必须重新调用。
6. action 38 通过 `0x16 action=38` 启动；红外 `3` 执行放矿循环，红外 `4` 正常结束。
7. action 38 中止使用 `0x16 action=1`，并等待 `busy=0` 后再继续业务。
8. 同一个 action 重复调用前必须先发送 `action=0`。
