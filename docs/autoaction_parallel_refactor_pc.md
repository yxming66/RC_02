# AutoAction 单动作调用与整体反馈规范

本文档定义 PC 调用 RC02 AutoAction 的唯一现行方式。PC 只负责选择一个 action、等待该 action 的整体终态，并在终态后决定下一步；动作内部步骤、并行分支、模块命令和资源变化均不属于 PC 协议。

## 1. 核心规则

1. PC 只使用 `PC_CMD_AUTO_ACTION = 0x16`，payload 为 1 字节 `<B action>`。
2. STM32 只使用 `PC_FEEDBACK_AUTO_ACTION = 0x96`，payload 为 4 字节 `<BBBB>`：`action, busy, finished, result`。
3. 任意时刻最多运行一个 AutoAction。动作运行中不接受另一个普通 action，不排队，也不提前准备下一动作。
4. 普通动作、融合动作、普通台阶、`STEP1` 和 `STEP2` 都是独立的整体 action。
5. PC 只以 `0x96` 的整体终态判断完成，不使用内部步骤、模块反馈或机构到位状态替代整体结果。
6. `busy=1` 期间保持自动命令路由和该动作的完整资源租约；只有底层真正进入 `SUCCESS`、`FAIL` 或 `ABORTED` 后才释放。
7. AutoOre 内部融合动作可以并行执行多个分支，但内部并行不向 PC 暴露，任何局部分支完成都不能触发下一动作或允许 PC 提前接管模块。

## 2. 调用命令 `0x16`

payload：

```python
AUTO_ACTION_CMD = struct.Struct("<B")
```

| action | 含义 |
|---:|---|
| `0` | 清除接收 latch，为后续触发重新布防 |
| `1` | 中止当前 AutoAction |
| `2~38` | 启动对应整体 action |

一键动作是边沿触发命令，不是连续 setpoint。同一个非零 action 连续发送只触发一次；再次调用相同 action 前必须先发送 `action=0`。

推荐所有正常调用都使用统一序列：

```text
发送 action=0
发送目标 action
等待相同 action 的整体终态
读取 result
```

动作已经 `busy=1` 时，PC 不得发送另一个普通 action。需要停止当前动作时发送 `action=1`，随后等待 `busy=0`；资源不会在中止命令到达瞬间释放，而是在底层进入真实中止终态后释放。

## 3. 整体反馈 `0x96`

payload 固定为 4 字节：

```python
AUTO_ACTION_FEEDBACK = struct.Struct("<BBBB")
# action, busy, finished, result
```

| 偏移 | 字段 | 含义 |
|---:|---|---|
| 0 | `action` | 当前运行或最近结束的 action |
| 1 | `busy` | `1=整体动作正在运行`，`0=当前无动作运行` |
| 2 | `finished` | `1=已有整体终态`，`0=无可用整体终态` |
| 3 | `result` | 仅 `finished=1` 有效；`0=SUCCESS`，`1=FAIL` |

状态解释：

| `busy` | `finished` | PC 解释 |
|---:|---:|---|
| `0` | `0` | 空闲且当前没有可消费的整体结果 |
| `1` | `0` | action 正在运行；继续等待，不启动下一动作 |
| `0` | `1` | action 已进入整体终态；读取 `result` |

正常调用必须同时满足以下条件才算完成：

```python
done = feedback.action == expected_action and not feedback.busy and feedback.finished
```

`busy=1` 时必须忽略 `result`。PC 不得根据底盘停止、Pole 到位、Arm 到位、矿仓状态变化或融合动作某一内部支路完成来提前生成成功。

## 4. 单动作互斥

RC02 对 PC 暴露的是单 active action 模型：

```text
IDLE
  -> 接受一个 action
  -> RUNNING（拒绝其他普通 action）
  -> 底层真实 SUCCESS / FAIL / ABORTED
  -> 发布整体终态并释放资源
  -> PC 可调用下一 action
```

因此：

- PC 不维护并行动作集合。
- PC 不提交等待队列。
- PC 不因两个动作使用不同机构而同时调用它们。
- PC 不在融合台阶分支提前完成时启动导航、普通台阶、取矿或存矿动作。
- PC 不在普通取矿推进结束但 Arm 仍运动时提前接管底盘或其他被租约覆盖的模块。

## 5. 自动路由与资源租约

每个 action 启动时确定运行所需的完整资源集合。动作 `busy=1` 的整个期间，资源集合和自动路由持续有效，不因以下情况缩减：

- 内部 step index 改变；
- 某个融合分支完成；
- 某个模块当前周期没有新目标；
- 底盘暂时停止；
- Arm 已完成交矿但矿仓仍在收尾；
- 普通台阶机构已经到达某个中间姿态。

PC 可以继续周期发送心跳、底盘和其他连续控制帧以保持链路与最新目标，但这些帧不能穿透当前 action 的资源租约。只有收到整体终态后，PC 才能认为自动路由结束并接管相关模块。

安全/急停逻辑仍高于自动路由；安全中止同样必须等待底层进入真实终态后，整体反馈才清除 `busy`。

## 6. 融合动作

融合取矿、存矿和上下台阶动作对 PC 是一个 action。固件内部可以同时推进取矿、存矿和台阶分支，但必须满足整个动作的底层结束条件后才发布 `finished=1`。

PC 不可见也不可使用以下内部信息：

- 哪个融合分支正在运行；
- 哪个分支先完成；
- 某个机构是否已暂时空闲；
- 内部步骤索引或局部成功状态；
- 某个模块是否可以被下一动作复用。

即使台阶分支先完成，只要取矿或存矿分支尚未到真实终态，`0x96` 仍保持 `busy=1, finished=0`，完整资源租约不释放。

## 7. 结果兼容策略

固件内部始终区分真实 `SUCCESS`、`FAIL` 和 `ABORTED`。wire 只发送 `result=0/1`，并延续当前兼容策略：

- 所有放矿类动作（完整动作及对应 `STEP1/STEP2`）、`ROD_SPEARHEAD_STEP1`、`ROD_SPEARHEAD_STEP2`、`ROD_DOCK_WAIT` 报告真实失败，底层失败或中止时 PC 收到 `result=1` 和对应 `failure_mask`；无矿导致的启动失败不得映射为成功。
- 其余动作只在底层真正进入终态后，允许将底层失败或中止映射为 synthetic success，即 PC 收到 `result=0`。
- synthetic success 只改变 PC 可见结果，不得提前设置 `finished=1`，也不得提前释放资源。

PC 若需要确认矿物实际位置，应在整体终态后结合 `0x95` 占矿反馈做业务校验；不能把兼容成功解释为所有物理目标必然达成。

## 8. `STEP1` 与 `STEP2`

放矿分步动作采用两个独立 action：

| 第一阶段 | 第二阶段 |
|---|---|
| `34 RELEASE_STEP1` | `35 RELEASE_STEP2` |
| `36 RELEASE_LIFT_DETECT_STEP1` | `37 RELEASE_LIFT_DETECT_STEP2` |

规则：

1. PC 调用 `STEP1` 并等待该 action 的 `busy=0 && finished=1`。
2. `STEP1` 完成观察后立即进入自己的整体终态，不继续占用 AutoAction 运行槽。
3. `STEP1` 终态保留 Arm 持矿状态，便于 PC 根据 `0x95.release_grid_has_ore` 决策。
4. 如果目标格可放，PC 对 `STEP2` 发起一次新的独立调用：先发 `action=0`，再发对应 `STEP2`。
5. 如果目标格不可放，PC 不调用 `STEP2`，可重新规划目标格或执行其他安全流程。

两个阶段之间没有续接命令、等待门或隐式事务。`STEP2` 的成功与失败只属于其自身 action。

## 9. action 38 红外三层放矿

`38 RELEASE_IR_LIFT_DETECT` 是一个持续等待红外协同的整体 action：

```text
PC: action=0 -> action=38
RC02: 准备 Pole / 矿仓 / Arm，保持 busy=1
R1: 红外 3 -> RC02 执行一次放矿循环
RC02: 循环完成后回等待位，继续 busy=1
R1: 可再次发送红外 3，执行下一次放矿循环
R1: 红外 4 -> RC02 正常收尾并进入整体终态
```

详细规则：

- action 38 必须通过 `0x16 action=38` 启动。
- 动作运行期间持续持有 CHASSIS、POLE、ARM、STORE、ROD 和共享气路等完整资源，并保持自动路由。
- 固件先展开 Pole，必要时从矿仓上膛，再让 Arm 到 `WAIT_RELEASE_ORE`。平台、Arm 和 Pole 全部准备完成后才接受新的红外命令 `3`。
- 机构就绪前到达的红外命令 `3` 不会延后补触发；R1 必须在就绪后重新发送。
- 每个有效红外命令 `3` 执行一次完整放矿循环。循环完成后 Arm 回到等待位，`busy` 继续为 1，资源不释放。
- action 38 忽略目标格前方光电/SICK 有无矿结果，是否执行放矿由红外命令 `3` 决定。
- 红外命令 `4` 启动正常结束流程；Arm 回到 `STANDBY` 且底层真正成功后，才发布整体终态并释放资源。
- PC 要中止时发送 `0x16 action=1`。在底层进入 `ABORTED` 前，`busy=1` 且资源保持不变。

`0x9A` 只用于观察红外帧的有效性、新鲜度和最近命令；action 38 是否结束必须以 `0x96` 整体反馈为准。

## 10. PC 参考伪代码

```python
import struct
import time

CMD_AUTO_ACTION = 0x16
FEEDBACK_AUTO_ACTION = 0x96
AUTO_ACTION_CMD = struct.Struct("<B")
AUTO_ACTION_FEEDBACK = struct.Struct("<BBBB")


def send_auto_action(send_frame, action: int) -> None:
    send_frame(CMD_AUTO_ACTION, AUTO_ACTION_CMD.pack(action))


def call_auto_action(send_frame, wait_feedback, action: int, timeout_s: float):
    send_auto_action(send_frame, 0)
    send_auto_action(send_frame, action)

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        payload = wait_feedback(FEEDBACK_AUTO_ACTION, deadline)
        current_action, busy, finished, result = AUTO_ACTION_FEEDBACK.unpack(payload)

        if current_action == action and not busy and finished:
            return result == 0

    raise TimeoutError(f"auto action {action} did not reach terminal state")


def abort_auto_action(send_frame, wait_feedback, timeout_s: float) -> None:
    send_auto_action(send_frame, 1)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        payload = wait_feedback(FEEDBACK_AUTO_ACTION, deadline)
        _, busy, _, _ = AUTO_ACTION_FEEDBACK.unpack(payload)
        if not busy:
            return
    raise TimeoutError("auto action abort did not reach terminal state")
```

调用方必须串行执行 `call_auto_action()`，不得为多个 action 并发创建等待任务。超时是 PC 的通信/业务保护，不代表可以直接启动下一动作；超时后应继续查询整体反馈，必要时发送 `action=1` 并等待 `busy=0`。

## 11. 迁移说明

旧的多事务命令、全量状态快照、拒绝反馈、请求编号、动作编号、资源掩码、分段完成位、等待队列和外层资源并行机制均已删除，不能再作为当前协议使用。

上位机迁移步骤：

1. 删除多事务提交、查询、续接、取消和确认逻辑。
2. 删除并行动作容器、资源冲突等待和终态表项管理。
3. 将所有动作统一改为 `0x16` 单字节调用。
4. 将反馈解析统一改为 `0x96` 的 4 字节 `<BBBB>`。
5. 删除分段完成驱动的提前导航、提前接管和下一动作预启动。
6. 将分步放矿改为两个独立 action，各自完整调用并等待终态。
7. 将 action 38 的中止改为发送 `0x16 action=1`，正常结束仍由红外命令 `4` 驱动。

## 12. 联调验收清单

- [ ] `0x16` payload 严格为 1 字节。
- [ ] `0x96` payload 严格为 4 字节 `<BBBB>`。
- [ ] 同 action 未先发送 `0` 时不会重复触发。
- [ ] 任一 action `busy=1` 时不会启动或排队另一个普通 action。
- [ ] 融合动作任一内部分支提前完成时仍保持 `busy=1` 和完整资源租约。
- [ ] PC 只在相同 action 的 `busy=0 && finished=1` 后启动下一动作。
- [ ] `STEP1` 独立结束并保持 Arm 持矿；`STEP2` 通过新的调用独立启动。
- [ ] action 38 收到红外 `3` 后可完成放矿循环并继续保持 `busy=1`。
- [ ] action 38 只有红外 `4` 正常结束，或 `action=1` 中止后，才清除 `busy` 并释放资源。
- [ ] PC 不使用模块命令、机构到位或占矿变化替代整体终态。
