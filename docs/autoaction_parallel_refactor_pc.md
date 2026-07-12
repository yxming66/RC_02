# AutoAction 并行调度与上位机接入规范

固件状态：RC02 V3 接口已冻结

调度容量：4 个 Job

兼容性：V1/V2 线格式保留；新增 V3，不允许 V2 与 V3 活动事务混用

## 1. 固件调度模型

RC02 不再使用“一个 AutoAction 占用整车”的模型。V3 将动作提交为固定容量 Job，并按物理资源进行仲裁：

```text
SUBMIT
  -> QUEUED
  -> WAIT_RESOURCE / WAIT_EXECUTOR
  -> STARTING
  -> RUNNING / WAIT_GATE
  -> SUCCEEDED / FAILED / CANCELLED
  -> ACK 后释放 Job 表项
```

节点启动时必须一次性获得全部初始资源。若任何资源冲突，Job 保持 `WAIT_RESOURCE`，并且不持有任何部分资源，因此不存在 hold-and-wait 型循环死锁。

同一执行器同一时间只运行一个 Job；不同执行器在资源和命令路由兼容时可以并行。调度器不使用堆内存，Job 表固定为 4 项。

## 2. 资源位

| bit | 掩码 | 资源 |
|---:|---:|---|
| 0 | `0x01` | CHASSIS，底盘 |
| 1 | `0x02` | POLE，撑杆/台阶机构 |
| 2 | `0x04` | ARM，简易机械臂 |
| 3 | `0x08` | STORE，存矿机构 |
| 4 | `0x10` | ROD，取矛头机构 |
| 5 | `0x20` | SHARED_VALVE，共享气阀 |

共享气阀在 V3 中按独占资源处理。等待 Job 不会通过“相同气阀状态”绕过互斥。

资源优先级：

```text
安全/急停
  > 持有资源租约的自动 Job
  > 新鲜的 PC 命令
  > 遥控器命令
  > 安全保持/Relax
```

租约和实际命令通过双缓冲快照发布。资源已经保留、但该周期尚无具体目标命令时，命令中心输出安全保持，PC 不会穿透租约。

## 3. 已实现的并行场景

| 场景 | 固件行为 |
|---|---|
| 融合动作起步时 Arm/Pole 尚在准备 | CHASSIS 立即按 `precontact_vx_mps` 前进；若有效 PE13/photo1 在 Arm 取矿目标稳定前触发，则锁存保护停车，Arm 达到位置、速度和稳定时间条件后自动恢复前进 |
| 融合台阶先完成，上层仍在取矿/存矿 | 释放 CHASSIS、POLE；等待的纯台阶 Job 可以自动启动；也可由 PC 跑航点 |
| 普通取矿推进完成，机械臂仍在抬升 | 仅释放 CHASSIS；PC 可跑航点，POLE 仍由当前 Job 持有 |
| 取矿存矿自动后退完成 | 释放 CHASSIS、POLE；ARM/STORE/VALVE 继续运行 |
| 回收地面矿后退完成 | 释放 CHASSIS；ARM/STORE/VALVE 继续运行，POLE 未被该 Job 占用 |
| 纯 STORE 与纯台阶同时提交 | 分别使用上层资源和 CHASSIS/POLE，可并行运行 |
| 两个 Job 需要同一资源 | 后提交者进入 WAIT_RESOURCE，持有资源为 0 |
| 两个 Job 使用同一执行器 | 后提交者等待 EXECUTOR |

ROD 与 STEP/SICK 当前按命令路由不兼容处理，不并行启动。AutoOre 上层动作与 STEP/SICK 在资源释放后可以并行。

## 4. PC 底盘连续发送要求

上位机在 AutoAction 运行期间仍须持续发送 `PC_CMD_CHASSIS (0x10)`：

```python
CHASSIS_CMD = struct.Struct("<fff")  # vx, vy, wz，12 字节小端
```

- 推荐 20–50 Hz；
- 最近 200 ms 没有底盘帧时，底盘 PC 命令失效并进入 Relax；
- 心跳超时为 500 ms；心跳不能替代底盘帧新鲜度；
- Job 持有 CHASSIS 时，PC 帧仍会接收和更新，但不会覆盖自动命令；
- Job 释放 CHASSIS 后，在一次 AutoOre 发布加下一次 RC 仲裁内切回最新 PC 命令，当前周期通常不超过 7 ms。

推荐结构：

```python
while pc_control_enabled:
    send_heartbeat()
    vx, vy, wz = waypoint_controller.update()
    send_chassis(vx, vy, wz)       # AutoAction 运行期间也发送
    consume_auto_action_v3()
```

## 5. V3 命令

Topic：`PC_CMD_AUTO_ACTION_V3 = 0x24`

Payload：8 字节，小端 packed

```python
AUTO_ACTION_V3_CMD = struct.Struct("<HHBBBB")
# request_id, job_id, operation, action, gate_id, flags
```

### 5.1 operation

| 名称 | 值 | 字段要求 |
|---|---:|---|
| NONE | 0 | 不发送 |
| SUBMIT | 1 | `request_id != 0`，`job_id = 0`，设置 `action` |
| CANCEL | 2 | 设置目标 `job_id`；建议同时带原 `request_id` |
| CONTINUE | 3 | 两段式放矿使用；设置 `job_id` 和 `gate_id` |
| ACK | 4 | Job 终态保存完成后释放表项 |
| QUERY | 5 | 检查 `job_id` 是否仍存在；完整状态本来就周期上报 |

同一 `request_id + action` 的 SUBMIT 是幂等重发，返回已有 Job，不会重复启动。同一 `request_id` 携带不同 action 会通过拒绝反馈报告 `REQUEST_CONFLICT`。

Job 到达终态后仍占用一个 Job 表项，但不占用物理资源。上位机必须在保存终态后发送 ACK，否则 4 个未 ACK 终态会占满队列。

## 6. V3 全量反馈

Topic：`PC_FEEDBACK_AUTO_ACTION_V3 = 0x9F`

Payload：固定 64 字节，小端 packed，一帧包含全部 4 个 Job 槽位

```python
V3_HEADER = struct.Struct("<HBB")
# generation, count, capacity

V3_JOB = struct.Struct("<HHHBBBBBBBBB")
# request_id, job_id, failure_mask,
# action, state, owned_resource_mask, waiting_resource_mask,
# running_segment_mask, completed_segment_mask, failed_segment_mask,
# blocked_reason, reject_reason

assert V3_HEADER.size == 4
assert V3_JOB.size == 15
assert V3_HEADER.size + 4 * V3_JOB.size == 64
```

解析时只使用前 `count` 个 Job；其余槽位为零。`generation` 在调度状态改变时递增，允许上位机识别新快照。

```python
def parse_v3(payload: bytes):
    generation, count, capacity = V3_HEADER.unpack_from(payload, 0)
    assert capacity == 4
    jobs = []
    offset = V3_HEADER.size
    for index in range(capacity):
        fields = V3_JOB.unpack_from(payload, offset)
        offset += V3_JOB.size
        if index < count:
            jobs.append(fields)
    return generation, jobs
```

### 6.1 Job state

| 状态 | 值 | 是否终态 |
|---|---:|---|
| FREE | 0 | — |
| QUEUED | 1 | 否 |
| WAIT_RESOURCE | 2 | 否 |
| STARTING | 3 | 否 |
| RUNNING | 4 | 否 |
| WAIT_GATE | 5 | 否 |
| CANCEL_REQUESTED | 6 | 否 |
| SUCCEEDED | 7 | 是 |
| FAILED | 8 | 是 |
| CANCELLED | 9 | 是 |
| REJECTED | 10 | 是 |

### 6.2 blocked_reason

| 原因 | 值 |
|---|---:|
| NONE | 0 |
| RESOURCE | 1 |
| EXECUTOR | 2 |
| DEPENDENCY | 3 |
| MOTION_CONSTRAINT | 4 |
| SENSOR | 5 |
| EXTERNAL_GATE | 6 |

`waiting_resource_mask` 只在资源冲突时有效。等待 Job 的 `owned_resource_mask` 必须为 0；若不是 0，应视为固件异常。

### 6.3 分支位

| bit | 掩码 | 含义 |
|---:|---:|---|
| 0 | `0x01` | PICK / ARM 交矿侧 |
| 1 | `0x02` | STORE |
| 2 | `0x04` | STEP / 底盘台阶侧 |

Job 可以保持 RUNNING，同时 `completed_segment_mask.STEP = 1`。此时表示台阶资源已经归还，但其他分支尚未结束。

低位存矿包含平台收尾颠动。平台完成配置次数的快速上/下往返并回到 `STANDBY` 后，才设置 `completed_segment_mask.STORE`；颠动期间 Job 和 STORE 分支仍为运行中。高位存矿不执行该阶段。

### 6.4 终态与“仅上报成功”策略

资源释放、`completed_segment_mask` 某一位置位，都不代表整个 Job 已结束。RC02 只有在对应底层执行器真正进入 `SUCCESS`、`FAIL` 或 `ABORTED` 后，才会生成 Job 终态。因此，普通取矿释放 CHASSIS、融合动作释放 CHASSIS/POLE 后，Job 仍保持 `RUNNING`，上层机构继续执行到自己的状态机终点。

为了兼容既有比赛流程，部分动作故意不向 PC 暴露内部失败。底层状态机仍正常运行到终态；仅在生成 PC 终态反馈时进行映射：

| 动作 | 底层 FAIL 时 PC 看到 |
|---|---|
| `RELEASE_STEP1`、`RELEASE_LIFT_DETECT_STEP1` | `FAILED`，保留真实 `failure_mask` |
| `ROD_SPEARHEAD_STEP1`、`ROD_SPEARHEAD_STEP2`、`ROD_DOCK_WAIT` | `FAILED`，保留真实 `failure_mask` |
| 其他可提交动作 | `SUCCEEDED`，`failure_mask=0`、`failed_segment_mask=0`，并补齐 required segment |

以下情况不参与“仅上报成功”映射：

- V3 显式 `CANCEL`、遥控器退出 PC 控制页、PC 心跳失效，或底层执行器进入 `ABORTED`：返回 `CANCELLED`。动作 38 的红外命令 6 会先回 `WAIT_RELEASE_ORE` 并保持运行，收到命令 5、回到 `STANDBY` 后才进入 `ABORTED/CANCELLED`；
- 请求非法、队列满、状态不允许：返回 reject；
- Job 尚未成功启动执行器：返回 `FAILED/START_FAILED` 或 reject，不能伪造为已正常执行成功。

因此上位机不能把底盘资源释放当作 AutoAction 成功；必须以该 `job_id` 的终态为准。

## 7. V3 拒绝反馈

Topic：`PC_FEEDBACK_AUTO_ACTION_V3_REJECT = 0xA1`

Payload：8 字节，小端 packed

```python
AUTO_ACTION_V3_REJECT = struct.Struct("<HHBBBB")
# request_id, job_id, operation, action, reject_reason, reserved
```

拒绝反馈保持约 500 ms，之后清零。

| reason | 值 |
|---|---:|
| NONE | 0 |
| INVALID_REQUEST | 1 |
| INVALID_ACTION | 2 |
| QUEUE_FULL | 3 |
| REQUEST_CONFLICT | 4 |
| JOB_NOT_FOUND | 5 |
| INVALID_STATE | 6 |
| START_FAILED | 7 |

资源暂时繁忙不会产生拒绝，而是接受 Job 并进入 WAIT_RESOURCE。

## 8. 典型时序

### 8.1 融合动作后排队纯台阶

```text
PC                  RC02 scheduler              Executors
 | SUBMIT fused            |                         |
 |------------------------>| start AutoOre          |
 | SUBMIT next step        | WAIT_RESOURCE          |
 |------------------------>| owned=0, wait=CHASSIS|POLE
 |                         |                         |
 |                         | fused STEP done         |
 |                         | release CHASSIS|POLE    |
 |                         | start queued step ----->| AutoCtrl
 |<------------------------| snapshot: both RUNNING |
 |                         | AutoOre upper + step    |
```

### 8.2 PC 航点接管

```text
AutoOre owns CHASSIS     -> 自动底盘命令胜出，PC 仍持续发送
AutoOre releases CHASSIS -> 最新且小于 200 ms 的 PC 命令自动胜出
AutoOre upper continues  -> ARM/STORE/VALVE 仍由原 Job 控制
```

## 9. CANCEL、急停与掉线

- CANCEL queued/waiting Job：直接进入 CANCELLED，不影响其他执行器；
- CANCEL running Job：只中止该 Job 对应的执行器，然后释放其全部资源；
- RC 离开 PC 控制页、遥控器离线、PC 心跳失效：所有非终态 V3 Job 转入取消流程；
- 安全输出优先级始终高于 V3；
- CANCELLED、FAILED、SUCCEEDED 都需要 ACK；
- 全局安全切换不会让等待 Job 在后台重新启动。

## 10. V1/V2 兼容规则

V1/V2 原线格式未修改，便于旧工具和单 Job 调试继续使用。但生产上位机应迁移到 V3。

- V3 存在非终态 Job 时，V2 START 返回 BUSY；
- V2 存在非终态事务时，V3 SUBMIT 返回 INVALID_STATE；
- 不要使用 V1 提交动作、V3 查询状态的混合方式；
- V2 仍只有一个前台 Job，不提供多 Job 并行语义。
- UART8 红外命令 `A5 03` 只建立三层放矿锁，不自动启动 legacy 动作。PC 在 `0x9A.zone3_action_locked=1` 后使用 V3 提交唯一的动作 38。动作 38 持有全部底盘和上层资源，在 `WAIT_RELEASE_ORE` 无限等待新的命令 4，并忽略目标格光电检测；完成释放后回该等待位并保持 RUNNING。命令 6 同样回等待位、记录中止但不提前产生终态；命令 5 才让 Arm 回 `STANDBY`，到位后正常流程返回 `SUCCEEDED`，中止流程返回 `CANCELLED`。

## 11. 安全与实车限制

当前固件完成了资源互斥、命令路由和失效保护，但机械臂展开时的底盘速度包络仍应由上位机航点控制器限制。第一轮实车联调建议：

```text
|vx|, |vy| <= 0.3 m/s
|wz|       <= 0.5 rad/s
```

这是联调建议，不是固件硬限制。确认机械空间无干涉后再根据机械设计放宽。

## 12. 联调验收清单

- [ ] V3 SUBMIT 后能在 0x9F 快照中找到相同 request_id，并获得 job_id；
- [ ] 相同 request_id 重发不会重复启动；
- [ ] 资源冲突 Job 显示 WAIT_RESOURCE、owned=0、waiting_resource 正确；
- [ ] 融合 STEP 完成后，等待纯台阶自动进入 RUNNING；
- [ ] 前一个 AutoOre 上层分支与新 AutoCtrl 台阶同时运行；
- [ ] 普通取矿进入抬升后，PC 航点能接管 CHASSIS；
- [ ] 停发 CHASSIS 超过 200 ms 后不会恢复旧速度；
- [ ] CANCEL queued Job 不会中止正在运行的同类执行器；
- [ ] CANCEL running Job 只影响目标执行器；
- [ ] 心跳超时或 RC 切出 PC 页后，所有等待和运行 Job 均被取消；
- [ ] 终态 ACK 后对应 Job 从快照消失；
- [ ] 4 个未 ACK Job 占满后，新 SUBMIT 收到 QUEUE_FULL；

## 13. 固件代码索引

- 固定容量调度器：`User/module/autoCtrlAPI/core/auto_action_scheduler.h/.c`
- 调度器主机测试：`User/module/autoCtrlAPI/core/tests/auto_action_scheduler_test.c`
- V3 执行器接入：`User/task/auto_ctrl_feed.c`
- AutoOre 租约与双缓冲快照：`User/module/autoCtrlAPI/ore_store/auto_ore_store.h/.c`
- 按资源命令仲裁：`User/module/rc_cmd_center/rc_cmd_center_app.cpp`
- MRLink topic 与 packed 结构：`User/module/mrlink_pc_comm/mrlink_pc_comm.h`
- MRLink 类型尺寸断言：`User/module/mrlink_pc_comm/pc_messages.hpp`
