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
    zone3_action_locked,
    release_allowed,
    release_abort_latched,
    zone3_action_state,
    last_dock_complete_cmd,
    last_zone3_action_cmd,
    last_release_cmd,
    last_command,
    release_abort_count_lsb,
    age_ms,
    rx_count,
    crc_error_count,
    error_count,
) = struct.unpack("<12B4I", payload)
```

其中：

| 字段 | 含义 | PC 用途 |
|---|---|---|
| `zone3_action_locked` | 三层放矿启动锁，命令 3 后为 1、命令 5 后为 0 | 判断本轮红外流程是否仍锁定 |
| `release_allowed` | 命令 4 放矿许可是否仍在 1000 ms 新鲜期 | 观察动作是否获得放矿许可 |
| `release_abort_latched` | 本轮是否已接受命令 6 中止 | 判断是否进入安全回撤 |
| `zone3_action_state` | `0=未知,1=启动锁定,2=结束解锁` | 观察 03/05 锁死生命周期 |
| `fresh` | 红外帧是否新鲜 | 建议仅在 `fresh=1` 时信任红外状态 |
| `dock_complete` | 对接完成状态 | 取矛头/对接类流程可使用 |

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
| `2` | `PC_AUTO_ACTION_STORE` | 一键存矿，把 arm 持有矿存入低/高位空位；低位流程最后执行平台快速上下颠动，回到 STANDBY 后才完成 |
| `3` | `PC_AUTO_ACTION_RELEASE` | 一键放矿，普通完整放矿流程 |
| `4` | `PC_AUTO_ACTION_RELEASE_LIFT_DETECT` | 一键三层放矿，含抬升检测流程 |
| `5` | `PC_AUTO_ACTION_CHAMBER` | 一键上膛，从低/高位取矿到 arm/发射准备位 |
| `34` | `PC_AUTO_ACTION_RELEASE_STEP1` | 普通放矿 step1：观察目标格并保持矿 |
| `35` | `PC_AUTO_ACTION_RELEASE_STEP2` | 普通放矿 step2：确认后继续释放 |
| `36` | `PC_AUTO_ACTION_RELEASE_LIFT_DETECT_STEP1` | 三层放矿 step1：观察目标格并等待抬升检测 |
| `37` | `PC_AUTO_ACTION_RELEASE_LIFT_DETECT_STEP2` | SICK 抬升检测放矿 step2 |
| `38` | `PC_AUTO_ACTION_RELEASE_IR_LIFT_DETECT` | 与 R1 操作手配合的三层放矿单事务 |

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
    zone3_action_locked = False
    release_allowed = False
    release_abort_latched = False
    zone3_action_state = 0
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

三层放矿只使用完整动作 `38`，不使用前方光电/SICK 判断，也不再拆分 step1/step2：

1. R1 发送 `A5 03`，RC02 建立本轮锁；`03` 本身不启动 AutoAction。
2. PC 从 `0x9A.zone3_action_locked=1` 检测到新一轮，用 V3 `SUBMIT(action=38)` 创建 Job。相同 `request_id` 重发保持幂等。
3. 动作 38 持续锁定 CHASSIS，并展开 Pole；若矿仍在矿仓则先上膛，然后机械臂到 `WAIT_RELEASE_ORE` 等待放矿位。
4. 平台、机械臂和 Pole 全部准备完成后，动作无限等待 R1 的新命令 `A5 04`。早到的 `04` 无效，必须在机构就绪后重新发送。
5. 收到有效 `04` 后执行完整放矿；不检查目标格前方是否有矿。放矿完成后 Arm 回到 `WAIT_RELEASE_ORE`，Job 仍保持 `RUNNING`，底盘继续锁定。
6. 任意运行阶段收到 `A5 06`，固件撤销放矿并让 Arm 回到 `WAIT_RELEASE_ORE`；Job 暂不结束，保持资源锁并记住本轮已经中止。
7. R1 发送 `A5 05` 后，Arm 才回到 `STANDBY`；到位后解除资源和红外锁。正常流程进入 `SUCCEEDED`，发生过 `06` 的流程进入 `CANCELLED`。

### 8.1 PC 观察逻辑

PC 只提交动作 38。PC 必须对本轮 `03` 做一次性去重：

```python
if ir.zone3_action_locked and not zone3_job_submitted:
    submit_v3(action=38, request_id=new_nonzero_request_id())
    zone3_job_submitted = True

if ir.release_abort_latched:
    state = "SAFE_RETREAT"
elif zone3_job_is_running:
    state = "ZONE3_RUNNING"
else:
    state = "ZONE3_IDLE"

if not ir.zone3_action_locked:
    zone3_job_submitted = False
```

### 8.2 异常处理

| 场景 | 固件行为 |
|---|---|
| PC 提交 38 时没有可用矿或配置非法 | Job 启动失败；需发 05 结束本轮，修复条件后重新走 03 |
| PC 提交 38 时资源被占用 | V3 Job 进入 `WAIT_RESOURCE`，不持有部分资源 |
| 命令 04 在机构就绪前到达或超过 1000 ms | 动作继续保持，不释放矿；就绪后重新发送 04 |
| 前方光电显示有矿 | 动作 38 忽略该检测，是否允许释放完全由 R1 命令 04 决定 |
| 运行中收到命令 06 | Arm 回 `WAIT_RELEASE_ORE` 并继续持锁；重复 06 无副作用，收到 05 并回到 STANDBY 后才返回 `CANCELLED` |

## 9. 推荐总控优先级

PC 侧可以按如下优先级处理动作：

1. 安全/中止最高优先级：需要停止时发 `ABORT=1`。
2. `zone3_action_locked=1` 时，PC 为本轮只提交一次动作 38。
3. 如果已有 `auto.busy=1`，只监听反馈，不发新的互斥 autoaction。
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
    fields = struct.unpack("<12B4I", payload)
    return {
        "fresh": bool(fields[1]),
        "zone3_action_locked": bool(fields[3]),
        "release_allowed": bool(fields[4]),
        "release_abort_latched": bool(fields[5]),
        "zone3_action_state": fields[6],
    }


def choose_next_action(ore, auto, ir, flow_state):
    if ir["zone3_action_locked"] and not flow_state.zone3_job_submitted:
        flow_state.zone3_job_submitted = True
        return 38

    if ir["zone3_action_locked"]:
        return None

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
3. 红外命令 3 只建立锁；PC 必须用 V3 为本轮提交一次动作 38。
4. 分步放矿 step1 后如果目标格已有矿，不要发 step2，应重新规划目标格或中止。
5. 同一个 autoaction 需要重复触发时，先发 `action=0` 清 latch，再发目标 action。
