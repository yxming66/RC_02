# PC 侧基于有无矿反馈控制 R2 自动动作流程

本文档说明 PC 如何根据 STM32 上报的有无矿信息，以及 `PC_CMD_AUTO_ACTION` 一键动作协议，控制 R2 执行上膛、放矿、三层放矿 step2 等动作。

适用代码与协议：

- `User/module/mrlink_pc_comm/mrlink_pc_comm.h`
- `User/module/mrlink_pc_comm/pc_messages.hpp`
- `User/task/pc_comm_task.c`
- `docs/pccomm_protocol.md`

## 1. PC 需要订阅的反馈

### 1.1 矿仓/机械臂占矿反馈 `PC_FEEDBACK_ORE_STORE = 0x95`

payload 长度 12 字节，Python 解包格式：

```python
mode, all_homed, online_mask, homed_mask, platform_position_rad, low, high, arm, release_grid = struct.unpack(
    "<BBBBfBBBB", payload
)
```

字段含义：

| 字段 | 含义 | PC 用途 |
|---|---|---|
| `low` / `transform_low_has_ore` | 低位变形机构是否有矿，`0/1` | 判断低位是否可继续存矿/是否需要上膛 |
| `high` / `transform_high_has_ore` | 高位变形机构是否有矿，`0/1` | 判断高位是否可上膛/放矿 |
| `arm` / `arm_has_ore` | 机械臂是否持矿，`0/1` | 判断是否需要先存矿或上膛 |
| `release_grid` / `release_grid_has_ore` | 放矿目标格是否已有矿，`0/1` | 放矿 step1 后用于决定是否继续 step2 |

推荐 PC 内部状态命名：

```python
ore = {
    "low_has_ore": bool(low),
    "high_has_ore": bool(high),
    "arm_has_ore": bool(arm),
    "release_grid_has_ore": bool(release_grid),
}
```

### 1.2 一键动作反馈 `PC_FEEDBACK_AUTO_ACTION = 0x96`

payload 长度 8 字节，Python 解包格式：

```python
action, busy, finished, result, failure_mask, segment_mask, reserved = struct.unpack(
    "<BBBBHBB", payload
)
```

关键规则：

| 字段 | 含义 | PC 用途 |
|---|---|---|
| `action` | 当前或最近的一键动作 ID | 判断反馈属于哪个动作 |
| `busy` | 自动动作是否仍在执行 | `busy=1` 时不要发互斥动作 |
| `finished` | 是否已有结束结果 | `busy=0 && finished=1` 表示动作结束 |
| `segment_mask bit0` | pick/arm 交矿侧完成 | 融合动作中 arm 侧可继续下一步 |
| `segment_mask bit1` | store 存矿侧完成 | 存矿机构侧完成 |
| `segment_mask bit2` | step/底盘侧完成 | 台阶/底盘侧完成，可继续导航 |

当前固件只对放矿观察 step1（普通、SICK 抬升、红外抬升）以及取矛头分段/等待动作填写真实失败；其他动作在底层状态机结束后兼容映射为成功。该映射不会提前结束动作。PC 应同时依赖：

1. `busy` 判断是否正在跑。
2. `finished` 判断总动作是否结束。
3. `segment_mask` 判断融合动作的分段完成。
4. `0x95` 的占矿反馈判断真实矿位状态。

### 1.3 R1/R2 红外对接反馈 `PC_FEEDBACK_IR_DOCK = 0x9A`

payload 长度 28 字节，Python 解包格式：

```python
(
    valid,
    fresh,
    dock_complete,
    r2_leave_zone1_allowed,
    release_lift_step2_ready,
    cleared_ore_id,
    zone3_r2_state,
    last_dock_complete_cmd,
    last_r2_leave_zone1_cmd,
    last_release_lift_step2_cmd,
    age_ms,
    rx_count,
    crc_error_count,
    error_count,
) = struct.unpack("<BBBBBBBBBBxxIIII", payload)
```

其中：

| 字段 | 含义 | PC 用途 |
|---|---|---|
| `release_lift_step2_ready` | 红外通知三层放矿 step2 可执行，`0/1` | 为 `1` 时，PC 可下发 `PC_AUTO_ACTION_RELEASE_LIFT_DETECT_STEP2` |
| `fresh` | 红外帧是否新鲜 | 建议仅在 `fresh=1` 时信任红外状态 |
| `dock_complete` | 对接完成状态 | 取矛头/对接类流程可使用 |
| `r2_leave_zone1_allowed` | R2 是否可以出一区 | 导航/离区策略使用 |

## 2. PC 下发一键动作的方法

PC 通过 `PC_CMD_AUTO_ACTION = 0x16` 下发 1 字节动作 ID：

```python
payload = struct.pack("<B", action_id)
send_mrlink_frame(0x16, payload)
```

一键动作是“触发型命令”，不是连续 setpoint：

1. 新的非零 `action_id` 会触发动作。
2. 旧协议中，同一个非零动作只有在先发送 `action=0` 后才能再次启动；V2 使用 `request_id` 幂等去重。
3. 如需重新触发同一个动作，建议先发 `action=0` 清 latch，再发目标动作。
4. 中止动作使用 `action=1`。

推荐发送函数：

```python
def send_auto_action(action_id: int):
    send_mrlink_frame(0x16, struct.pack("<B", action_id))


def clear_auto_action_latch():
    send_auto_action(0)
```

## 3. 本流程相关动作 ID

| ID | 名称 | 用途 |
|---:|---|---|
| `1` | `PC_AUTO_ACTION_ABORT` | 中止当前一键动作 |
| `2` | `PC_AUTO_ACTION_STORE` | 一键存矿，把 arm 持有矿存入低/高位空位 |
| `3` | `PC_AUTO_ACTION_RELEASE` | 一键放矿，普通完整放矿流程 |
| `4` | `PC_AUTO_ACTION_RELEASE_LIFT_DETECT` | 一键三层放矿，含抬升检测流程 |
| `5` | `PC_AUTO_ACTION_CHAMBER` | 一键上膛，从低/高位取矿到 arm/发射准备位 |
| `34` | `PC_AUTO_ACTION_RELEASE_STEP1` | 普通放矿 step1：观察目标格并保持矿 |
| `35` | `PC_AUTO_ACTION_RELEASE_STEP2` | 普通放矿 step2：确认后继续释放 |
| `36` | `PC_AUTO_ACTION_RELEASE_LIFT_DETECT_STEP1` | 三层放矿 step1：观察目标格并等待抬升检测 |
| `37` | `PC_AUTO_ACTION_RELEASE_LIFT_DETECT_STEP2` | 三层放矿 step2
：红外允许后继续释放 |

## 4. 基础状态机

PC 侧建议维护以下状态：

```python
class AutoActionState:
    action = 0
    busy = False
    finished = False
    segment_mask = 0

class OreState:
    low_has_ore = False
    high_has_ore = False
    arm_has_ore = False
    release_grid_has_ore = False

class IrDockState:
    fresh = False
    release_lift_step2_ready = False
```

每轮收到反馈后更新状态：

1. 从 `0x95` 更新 `OreState`。
2. 从 `0x96` 更新 `AutoActionState`。
3. 从 `0x9A` 更新 `IrDockState`。
4. 只有 `auto.busy == False` 时才主动下发新的互斥一键动作。

动作等待函数：

```python
def auto_action_idle(auto):
    return not auto.busy


def auto_action_done(auto, action_id):
    return auto.action == action_id and not auto.busy and auto.finished
```

## 5. 上膛动作决策

### 5.1 何时需要上膛

PC 侧可以按以下条件判断：

```python
need_chamber = (not ore.arm_has_ore) and (ore.low_has_ore or ore.high_has_ore)
```

含义：

- `arm_has_ore == 0`：arm 当前没有矿，可以接收上膛矿。
- `low_has_ore || high_has_ore`：至少有一个存矿位有矿，可执行上膛。

### 5.2 何时不要上膛

不要下发 `PC_AUTO_ACTION_CHAMBER` 的情况：

| 条件 | 原因 |
|---|---|
| `auto.busy == 1` | 当前已有一键动作在跑 |
| `arm_has_ore == 1` | arm 已持矿，继续上膛可能冲突 |
| `low_has_ore == 0 && high_has_ore == 0` | 无矿可上膛 |

### 5.3 上膛流程

```python
if auto_action_idle(auto):
    if (not ore.arm_has_ore) and (ore.low_has_ore or ore.high_has_ore):
        send_auto_action(5)  # CHAMBER
```

等待完成：

```python
if auto_action_done(auto, 5):
    # 再看 0x95 验证结果
    chamber_ok = ore.arm_has_ore
```

推荐成功判据：

- `0x96` 显示 `action=5, busy=0, finished=1`。
- 随后 `0x95.arm_has_ore == 1`。

## 6. 存矿动作决策

### 6.1 何时需要存矿

如果 arm 持矿，并且低位或高位还有空位，就可以存矿：

```python
need_store = ore.arm_has_ore and ((not ore.low_has_ore) or (not ore.high_has_ore))
```

### 6.2 何时不要存矿

| 条件 | 原因 |
|---|---|
| `auto.busy == 1` | 当前已有一键动作在跑 |
| `arm_has_ore == 0` | arm 没矿，无需存矿 |
| `low_has_ore == 1 && high_has_ore == 1` | 两个存矿位都满，不能继续存 |

### 6.3 存矿流程

```python
if auto_action_idle(auto):
    if ore.arm_has_ore and ((not ore.low_has_ore) or (not ore.high_has_ore)):
        send_auto_action(2)  # STORE
```

推荐成功判据：

- `0x96` 显示 `action=2, busy=0, finished=1`。
- `0x95.arm_has_ore == 0`。
- `0x95.low_has_ore == 1` 或 `0x95.high_has_ore == 1` 至少一个为真。

## 7. 普通放矿流程

普通放矿有两种使用方式：完整动作和分步动作。

### 7.1 完整普通放矿

如果 PC 不需要先观察目标格/等待外部确认，可直接使用完整动作：

```python
if auto_action_idle(auto) and ore.arm_has_ore:
    send_auto_action(3)  # RELEASE
```

推荐完成判据：

- `0x96.action == 3 && busy == 0 && finished == 1`。
- `0x95.arm_has_ore == 0`。

### 7.2 分步普通放矿

分步放矿用于 PC 需要根据 `release_grid_has_ore` 决定是否继续放。

Step1：

```python
if auto_action_idle(auto) and ore.arm_has_ore:
    send_auto_action(34)  # RELEASE_STEP1
```

Step1 完成后，固件会保持 arm 持矿并等待 PC 决策。PC 读取：

```python
if auto_action_done(auto, 34):
    if not ore.release_grid_has_ore:
        send_auto_action(35)  # RELEASE_STEP2
    else:
        # 目标格已有矿：不要放，可导航换格或中止/重规划
        pass
```

推荐策略：

| `release_grid_has_ore` | PC 行为 |
|---:|---|
| `0` | 目标格空，可发 `RELEASE_STEP2=35` |
| `1` | 目标格已有矿，不发 step2，重新选择目标格 |

## 8. 三层放矿流程

三层放矿建议使用分步动作：

1. PC 先发 `PC_AUTO_ACTION_RELEASE_LIFT_DETECT_STEP1 = 36`。
2. 固件执行竖直观察/抬升检测相关准备，并保持矿。
3. PC 看到 `0x96.action=36` 且 `segment_mask bit1` 已置位，表示 step1 的等待/保持段已完成，但此时固件通常仍保持 `busy=1`。
4. PC 等待红外 `0x9A.release_lift_step2_ready == 1`。
5. PC 再发 `PC_AUTO_ACTION_RELEASE_LIFT_DETECT_STEP2 = 37`；固件允许 step1 正在 busy 时续接 step2。

### 8.1 Step1 启动条件

```python
can_start_lift_release_step1 = auto_action_idle(auto) and ore.arm_has_ore
```

如果 arm 没矿，PC 应先尝试上膛：

```python
if not ore.arm_has_ore and (ore.low_has_ore or ore.high_has_ore):
    send_auto_action(5)  # CHAMBER
```

### 8.2 Step1 执行

```python
if auto_action_idle(auto) and ore.arm_has_ore:
    send_auto_action(36)  # RELEASE_LIFT_DETECT_STEP1
```

等待 step1 的可续接条件：

```python
step1_ready_for_step2 = (
    auto.action == 36
    and bool(auto.segment_mask & (1 << 1))  # store/upper finished
)
```

注意：三层放矿 step1 完成后，固件会保持 AutoOre 运行以继续持矿等待 PC 决策，所以此时 `auto.busy` 可能仍为 `1`。PC 不应等待 `busy=0` 才发 step2。

### 8.3 Step2 触发条件

PC 需要同时满足：

```python
can_send_lift_step2 = (
    step1_ready_for_step2
    and ir.fresh
    and ir.release_lift_step2_ready
    and ore.arm_has_ore
    and auto.action == 36
)
```

满足后下发：

```python
send_auto_action(37)  # RELEASE_LIFT_DETECT_STEP2
```

### 8.4 三层放矿完整伪代码

```python
def update_r2_release_lift_flow(auto, ore, ir, state):
    if state == "IDLE":
        if auto_action_idle(auto):
            if not ore.arm_has_ore:
                if ore.low_has_ore or ore.high_has_ore:
                    send_auto_action(5)  # CHAMBER
                    return "CHAMBERING"
                return "WAIT_ORE"
            send_auto_action(36)  # RELEASE_LIFT_DETECT_STEP1
            return "LIFT_STEP1"

    if state == "CHAMBERING":
        if auto_action_done(auto, 5) and ore.arm_has_ore:
            send_auto_action(36)
            return "LIFT_STEP1"
        return state

    if state == "LIFT_STEP1":
        if auto.action == 36 and (auto.segment_mask & (1 << 1)):
            return "WAIT_IR_STEP2"
        return state

    if state == "WAIT_IR_STEP2":
        if ir.fresh and ir.release_lift_step2_ready and ore.arm_has_ore and auto.action == 36:
            send_auto_action(37)
            return "LIFT_STEP2"
        return state

    if state == "LIFT_STEP2":
        if auto_action_done(auto, 37):
            return "DONE"
        return state

    return state
```

### 8.5 异常处理建议

| 场景 | 建议 |
|---|---|
| 等待 `release_lift_step2_ready` 超时 | PC 重规划或发送 `ABORT=1` |
| `ore.arm_has_ore` 在 step2 前变 0 | 不发 step2，进入异常处理 |
| `ir.fresh == 0` | 不信任 `release_lift_step2_ready` |
| `auto.busy == 1 && auto.action == 36 && segment bit1 已置位` | 这是三层 step1 等待 step2 的正常状态，可以在红外允许后发 `37` |
| `auto.busy == 1 && auto.action != 36` | 等待当前动作结束，不抢发互斥动作 |

## 9. 推荐总控优先级

PC 侧可以按如下优先级处理动作：

1. 安全/中止最高优先级：需要停止时发 `ABORT=1`。
2. 如果正在三层放矿 step1 且 `segment_mask bit1` 已置位，则优先等待红外 `release_lift_step2_ready` 并发 `37`。
3. 除上述 step1→step2 续接特例外，如果已有 `auto.busy=1`，只监听反馈，不发新的互斥 autoaction。
4. 如果需要放矿并且 arm 有矿，按目标类型发普通放矿或三层分步放矿。
5. 如果 arm 没矿但低/高位有矿，先发 `CHAMBER=5`。
6. 如果 arm 有矿且存矿位有空位，可发 `STORE=2`。
7. 如果低/高位满且 arm 也有矿，避免继续取矿/存矿，优先规划放矿。

## 10. 最小 PC 侧示例

```python
AUTO_ABORT = 1
AUTO_STORE = 2
AUTO_RELEASE = 3
AUTO_CHAMBER = 5
AUTO_RELEASE_STEP1 = 34
AUTO_RELEASE_STEP2 = 35
AUTO_RELEASE_LIFT_STEP1 = 36
AUTO_RELEASE_LIFT_STEP2 = 37


def on_feedback_ore_store(payload):
    mode, all_homed, online_mask, homed_mask, platform, low, high, arm, grid = struct.unpack(
        "<BBBBfBBBB", payload
    )
    return {
        "low_has_ore": bool(low),
        "high_has_ore": bool(high),
        "arm_has_ore": bool(arm),
        "release_grid_has_ore": bool(grid),
    }


def on_feedback_auto_action(payload):
    action, busy, finished, result, failure_mask, segment, _ = struct.unpack(
        "<BBBBHBB", payload
    )
    return {
        "action": action,
        "busy": bool(busy),
        "finished": bool(finished),
        "segment": segment,
    }


def on_feedback_ir_dock(payload):
    fields = struct.unpack("<BBBBBBBBBBxxIIII", payload)
    return {
        "fresh": bool(fields[1]),
        "release_lift_step2_ready": bool(fields[4]),
    }


def choose_next_action(ore, auto, ir, flow_state):
    if (
        flow_state == "WAIT_IR_STEP2"
        and auto["action"] == AUTO_RELEASE_LIFT_STEP1
        and (auto["segment"] & (1 << 1))
        and ore["arm_has_ore"]
        and ir["fresh"]
        and ir["release_lift_step2_ready"]
    ):
        return AUTO_RELEASE_LIFT_STEP2

    if auto["busy"]:
        return None

    if not ore["arm_has_ore"] and (ore["low_has_ore"] or ore["high_has_ore"]):
        return AUTO_CHAMBER

    if ore["arm_has_ore"] and ((not ore["low_has_ore"]) or (not ore["high_has_ore"])):
        return AUTO_STORE

    return None
```

## 11. 注意事项

1. `0x95` 的占矿信息是 PC 做“是否能上膛/是否能存矿/是否仍持矿”的主要依据。
2. `0x96` 是动作调度反馈，不能单独证明矿真的在目标位置；动作结束后应再看 `0x95`。
3. `0x9A.release_lift_step2_ready` 只表示红外允许三层放矿 step2，不代表固件会自动执行 step2；PC 仍需下发 `action=37`。
4. 分步放矿 step1 后如果目标格已有矿，不要发 step2，应重新规划目标格或中止。
5. 同一个 autoaction 需要重复触发时，先发 `action=0` 清 latch，再发目标 action。
