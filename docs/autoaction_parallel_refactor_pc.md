# AutoAction 并行重构与上位机接入说明

版本：Phase 1（资源级输出仲裁）  
适用固件：本仓库当前分支  
协议兼容性：MRLink V1/V2 线格式未改变

## 1. 本阶段解决的问题

旧实现把 `AUTO_ORE_OUTPUT` 当作整车控制权。即使 AutoOre 已经停止输出底盘命令，命令中心仍会选择 AutoOre 底盘路由，结果是底盘被置零或 Relax，PC 航点命令无法接管。

本阶段改为“动作仍在运行，但只覆盖它实际持有的资源”：

```text
安全/急停
  > 持有该资源租约的自动动作
  > 新鲜的 PC 命令
  > 遥控器命令
  > 对应机构的安全保持/Relax
```

AutoOre 每个 200 Hz 更新周期发布一个双缓冲输出快照。快照同时包含资源租约和对应命令；RC 命令中心只读取完整快照，不再使用“全局动作计划”推断资源所有权。

资源位定义与固件 `AutoOre_ResourceMask_t` 一致：

| bit | 掩码 | 资源 |
|---:|---:|---|
| 0 | `0x01` | CHASSIS，底盘 |
| 1 | `0x02` | POLE，撑杆/台阶机构 |
| 2 | `0x04` | ARM，简易机械臂 |
| 3 | `0x08` | STORE，存矿机构 |
| 4 | `0x10` | ROD，取矛头机构（预留给后续统一调度器） |
| 5 | `0x20` | SHARED_VALVE，共享气阀 |

## 2. 已实现的提前释放点

| 动作 | 底盘归还时机 | 仍可继续运行的机构 |
|---|---|---|
| 普通取矿 | 推进接触完成，进入抬升检测时 | ARM、POLE |
| 取矿并存矿 | 自动后退完成时 | ARM、STORE、共享气阀 |
| 回收地面矿并存矿 | 自动后退完成，进入抬升/存矿时 | ARM、STORE、共享气阀 |
| 融合取矿存矿并上/下台阶 | 台阶分支完成时 | ARM、STORE、共享气阀 |

融合台阶完成时会同时归还 CHASSIS 和 POLE。因此固件内部已经允许独立 `AutoCtrl` 台阶流程与前一任务的上层存矿分支并行。当前 V2 事务接口仍是单活动 Job，见“已知限制”。

## 3. 上位机必须修改的行为

### 3.1 动作期间继续发送底盘速度

上位机不要在发送 AutoAction START 后暂停航点控制。航点模块应持续计算并发送 `PC_CMD_CHASSIS (0x10)`。其 payload 为小端 12 字节 `struct.Struct("<fff")`，依次是 `vx, vy, wz`：

- 推荐频率：20–50 Hz；
- 最低要求：相邻有效底盘帧不得超过 200 ms；
- AutoOre 持有 CHASSIS 时，固件接收并缓存这些帧，但自动动作命令优先；
- AutoOre 归还 CHASSIS 后，下一次 RC 命令中心周期会自动使用最新 PC 帧；
- 最近 200 ms 没有底盘帧时，固件输出底盘 Relax，不会执行陈旧航点速度。

心跳和底盘新鲜度是两个条件。仅发送 `PC_CMD_HEARTBEAT` 可以保持 PC 在线，但不能让旧底盘速度重新生效。

建议上位机控制结构：

```python
while enabled:
    send_heartbeat()                    # 建议 10 Hz 或更高
    vx, vy, wz = waypoint_controller()
    send_chassis(vx, vy, wz)           # 建议 20~50 Hz，动作期间也发送

    fb = latest_auto_action_v2_feedback()
    update_auto_action_ui(fb)
```

不要写成：

```python
send_auto_action_start(...)
while feedback.state not in TERMINAL:
    stop_sending_chassis()              # 错误：归还底盘后没有新鲜命令
```

### 3.2 不要把“Job 未完成”解释成“底盘不可用”

融合动作可能出现：

```text
completed_mask.STEP = 1
state                = RUNNING
completed_mask.STORE = 0
```

这表示台阶分支完成、上层分支仍在运行。上位机应继续航点控制，不应等待整个 Job 进入 `SUCCEEDED` 才恢复底盘发送。

普通取矿会在 PICK 分支完成之前释放底盘，因此最稳妥的规则仍然是“始终发送新鲜底盘帧”，而不是只依赖 `completed_mask` 决定何时恢复发送。

## 4. 当前 V2 协议（线格式不变）

### 4.1 命令 `PC_CMD_AUTO_ACTION_V2 = 0x23`

小端、packed、8 字节，Python `struct` 格式：

```python
AUTO_ACTION_V2_CMD = struct.Struct("<HHBBBB")
# request_id, job_id, operation, action, gate_id, flags
```

操作值：

| operation | 值 | 用法 |
|---|---:|---|
| NONE | 0 | 不发送 |
| START | 1 | `request_id != 0`，`job_id = 0` |
| CONTINUE | 2 | 两段式放矿继续执行，携带已分配的 `job_id` |
| ABORT | 3 | 中止指定 `job_id` |
| ACK | 4 | 确认并清除终态 Job |

START 重发必须复用相同 `request_id` 和 `action`，固件会把它视为同一请求，避免串口重试造成重复动作。

### 4.2 反馈 `PC_FEEDBACK_AUTO_ACTION_V2 = 0x9E`

小端、packed、14 字节，Python `struct` 格式：

```python
AUTO_ACTION_V2_FB = struct.Struct("<HHBBBBBBHBB")
# request_id, job_id, action, state,
# required_mask, running_mask, completed_mask, failed_mask,
# failure_mask, reject_reason, active_node
```

分支位：

| bit | 掩码 | 含义 |
|---:|---:|---|
| 0 | `0x01` | PICK / ARM 交矿侧 |
| 1 | `0x02` | STORE |
| 2 | `0x04` | STEP / 底盘台阶侧 |

Job 状态：

| state | 值 | 含义 |
|---|---:|---|
| IDLE | 0 | 无 Job |
| ACCEPTED | 1 | 已接收 |
| RUNNING | 2 | 执行中 |
| WAIT_GATE | 3 | 等待上位机 CONTINUE |
| ABORTING | 4 | 中止处理中 |
| SUCCEEDED | 5 | 成功终态 |
| FAILED | 6 | 失败终态 |
| ABORTED | 7 | 已中止终态 |
| REJECTED | 8 | 请求被拒绝 |

上位机应分别显示 Job 总状态与各分支掩码，不能只显示一个 `busy`。

推荐解析框架：

```python
TERMINAL = {5, 6, 7, 8}

def on_auto_action_v2(payload: bytes):
    fields = AUTO_ACTION_V2_FB.unpack(payload)
    fb = AutoActionFeedback(*fields)

    if fb.completed_mask & 0x04:
        ui.step_branch = "done"
    if fb.running_mask & 0x02:
        ui.store_branch = "running"
    if fb.failed_mask:
        report_branch_failure(fb.failed_mask, fb.failure_mask)

    if fb.state in TERMINAL:
        send_v2_ack(fb.request_id, fb.job_id)
```

ACK 应在上位机保存完终态结果后发送。不要在收到第一帧终态前预先 ACK。

## 5. 典型时序

### 5.1 融合台阶先完成，上层继续存矿

```text
PC                         Firmware AutoOre             Command center
 | START fused                   |                           |
 |------------------------------>|                           |
 | CHASSIS waypoint stream       | owns CHASSIS+POLE         |
 |==============================>|-------------------------->| Auto wins
 |                               | step branch done          |
 |                               | release CHASSIS+POLE      |
 | CHASSIS waypoint stream       | ARM+STORE still running   |
 |==============================>|-------------------------->| PC wins chassis
 |                               | store branch done         |
 |<------------------------------| V2 SUCCEEDED              |
```

### 5.2 普通取矿推进很短，机械臂抬升较慢

```text
推进阶段: AutoOre owns CHASSIS + ARM + POLE
接触完成: AutoOre releases CHASSIS
抬升阶段: AutoOre owns ARM + POLE; PC waypoint controls CHASSIS
动作完成: AutoOre releases remaining resources
```

## 6. 安全约束

- RC 开关必须处于 PC 控制页，且 PC 控制模式有效，PC 路由才可接管；
- 安全/急停始终高于自动动作和 PC；
- 底盘帧超过 200 ms 自动失效；
- PC 心跳当前超时为 500 ms；
- 自动动作持有某资源时，PC 对该资源的命令可以继续发送，但不会覆盖自动动作；
- 本阶段没有实现速度包络。上位机恢复航点后应自行限制机械臂展开期间的速度和加速度，直至固件 Phase 2 增加 `NAV_LIMITED/NAV_FULL/STEP_ALLOWED` 约束。

建议第一轮实车测试将航点速度限制在 `|vx|, |vy| <= 0.3 m/s`、`|wz| <= 0.5 rad/s`，确认机构空间无干涉后再按机械设计放宽。该数值是调试建议，不是当前固件硬限制。

## 7. 已知限制与下一阶段接口

当前已经解决“一个 Job 内部的资源提前归还”和“PC 底盘自动接管”，但还没有完成多 Job DAG 调度器：

1. V2 仍只保存一个活动 `job_id`；前一个 Job 未终态时，第二个 V2 START 会返回 `REJECT_BUSY`；
2. 固件内部已有 `AutoCtrl + AutoOre upper` 并行路由，旧调试/Legacy 入口可在融合 STEP 完成后启动纯台阶，但上位机生产代码不应混用 V1/V2 来绕过单 Job 语义；
3. ARM/STORE 对共享气阀仍采用保守的共同占用；
4. 尚无矿位 reservation，也没有多个动作节点的原子资源申请队列。

Phase 2 将增加固定容量 Job 表和 DAG 节点，资源申请必须一次性获得全部所需资源；申请失败的节点不持有任何部分资源，从结构上消除 hold-and-wait 死锁。计划中的上位机 V3 至少需要：

- `SUBMIT_JOB(request_id, action, flags)`；
- `CANCEL_JOB(job_id)`；
- 多 Job 状态列表；
- `waiting/running/completed/failed_node_mask`；
- `owned_resource_mask`；
- `blocked_reason`：依赖、资源、运动约束、传感器或外部门控。

V3 的 topic ID 和 packed 结构尚未冻结，上位机当前不要提前实现假定格式。

## 8. 联调验收清单

- [ ] AutoAction 执行全程，PC 仍以 20–50 Hz 发送 CHASSIS；
- [ ] AutoOre 持有 CHASSIS 时，实车执行自动推进/台阶而不是 PC 速度；
- [ ] 普通取矿进入抬升检测后，PC 航点在一次 AutoOre 发布加下一次 RC 仲裁内接管（当前周期下通常不超过 7 ms）；
- [ ] 融合 STEP 完成但 STORE 未完成时，PC 航点能够运行；
- [ ] 取矿存矿自动后退完成后，PC 航点能够运行；
- [ ] 停发 CHASSIS 超过 200 ms 后，资源归还不会触发旧速度；
- [ ] 心跳超时或 RC 切到安全页时，自动动作被中止并进入安全输出；
- [ ] V2 `completed_mask` 可逐分支变化，最终终态后上位机发送 ACK；
- [ ] 同一 `request_id` 重发不会启动第二次动作。

## 9. 固件代码索引

- 资源与输出快照：`User/module/autoCtrlAPI/ore_store/auto_ore_store.h`
- 资源发布及提前释放点：`User/module/autoCtrlAPI/ore_store/auto_ore_store.c`
- 按资源选择命令源：`User/module/rc_cmd_center/rc_cmd_center_app.cpp`
- PC 底盘 200 ms 新鲜度：`User/module/mrlink_pc_comm/mrlink_pc_comm.cpp`
- V2 Job/反馈同步：`User/task/auto_ctrl_feed.c`
- MRLink 枚举和 packed 结构：`User/module/mrlink_pc_comm/mrlink_pc_comm.h`
