# PC_COMM 上位机通信协议说明

本文档面向上位机开发人员，描述 `User/module/mrlink_pc_comm` 当前实现的串口协议、业务 topic、payload 字节布局和推荐用法。

对应源码：

- `User/module/mrlink_pc_comm/mrlink_pc_comm.h`
- `User/module/mrlink_pc_comm/pc_messages.hpp`
- `User/module/mrlink_pc_comm/mrlink_pc_comm.cpp`
- `User/task/pc_comm_task.c`
- `User/device/mrlink/*`

## 1. 串口与总体行为

默认 PC 通信口为 `BSP_UART_PC`，当前映射到 `USART1`：

| 项 | 参数 |
|---|---|
| 引脚 | PA9=TX，PA10=RX |
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验 | None |
| 停止位 | 1 |
| 数据格式 | 二进制，小端 |

固件 `pc_comm_task` 每 1 ms 处理接收，每 20 ms 发送一批反馈帧，反馈频率约 50 Hz。红外矿种反馈在首次收到或内容变化后持续低频追加发送，当前周期约 200 ms。

PC 在线判定：任意一帧合法 PC_COMM 帧都会刷新在线状态。若 500 ms 内没有收到合法帧，固件认为 PC 离线并回到 RC 控制模式。推荐上位机持续发送 `PC_CMD_HEARTBEAT (0x01)`，10~20 Hz 即可；高频控制命令本身也会保活。

PC 控制命令实际生效还要求遥控器进入 PC 控制页：当前代码条件为 `sw_l == UP && sw_r == UP` 且 PC 链路在线。上位机可通过 `PC_FEEDBACK_STATUS.command_source == 1` 判断当前是否由 PC 接管。

## CAN 灯效模式命令

灯效控制板使用一个简单 CAN 协议，独立于 PC_COMM 串口 mrlink 帧：

| 项 | 参数 |
|---|---|
| CAN 类型 | 标准数据帧 |
| CAN ID | `0x322` |
| DLC | 至少 1 |
| Data[0] | 灯效模式号 |
| Data[1..7] | 忽略 |

当前灯条控制板只解析第一个字节，RC02 使用以下模式：

| Data[0] | 含义 |
|---:|---|
| `0x00` | 全灭 |
| `0x01` | 上位机准备就绪灯效 |
| `0x02` | 重试灯效，建议黄色全灯闪烁 |
| `0x03` | 失败灯效 |
| `0x04` | 即将执行二层放矿提示灯效，建议两个紫色短段 |
| `0x05` | 即将执行三层放矿提示灯效，建议三个紫色短段 |
| `0x07` | 动作完成灯效，建议绿色从中间向两边扩散 |
| `0x08` | 三层放矿等待被抬升提示，建议三个黄色短段 |
| `0xFF` | 切换到下一个模式 |

注意发送的是二进制数值，不是 ASCII 字符。例如 1 号模式应发送 `0x01`，不要发送字符 `'1'`（`0x31`）。

## 2. mrlink 帧格式

PC_COMM 业务数据封装在 mrlink 字节流帧中。串口可能一次收到多帧拼接，上位机解析时应循环拆帧。

```
[0x4D][0x52][len][cmd][payload...][crc16_lo][crc16_hi]
```

| 字段 | 长度 | 说明 |
|---|---:|---|
| header | 2 | 固定 `0x4D 0x52`，ASCII `"MR"` |
| len | 1 | payload 字节数，0~64 |
| cmd | 1 | topic/命令字 |
| payload | len | 业务 payload |
| crc16 | 2 | 对 `header + len + cmd + payload` 计算，低字节在前 |

CRC16 以固件 `User/component/crc16.c` 为准：反射式 0x1021 查表算法，初值 `0xFFFF`，无 xorout。不要按名称猜成传统 CCITT-FALSE。

```python
import struct

def mrlink_crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc

def build_mrlink_frame(cmd: int, payload: bytes = b"") -> bytes:
    if len(payload) > 64:
        raise ValueError("payload too large")
    body = bytes([0x4D, 0x52, len(payload), cmd]) + payload
    crc = mrlink_crc16(body)
    return body + struct.pack("<H", crc)
```

## 3. 编码约定

| 类型 | 说明 |
|---|---|
| `u8` | unsigned 8-bit |
| `u32` | unsigned 32-bit little-endian |
| `f32` | IEEE-754 float32 little-endian |

重要：部分固件内部 C 结构体包含 enum 或自然对齐 padding，不能直接作为上位机结构体镜像。上位机必须使用本文档的 payload 长度、偏移和 `struct.pack("<...")` 无对齐格式。

## 4. PC 发给 STM32 的命令

| cmd | 名称 | payload | Python pack | 用途 |
|---:|---|---:|---|---|
| `0x01` | `PC_CMD_HEARTBEAT` | 0 | `b""` | 保活/切入 PC 在线状态 |
| `0x10` | `PC_CMD_CHASSIS` | 12 | `<fff` | 底盘速度 |
| `0x11` | `PC_CMD_POLE` | 9 | `<Bff` | 撑杆目标 |
| `0x12` | `PC_CMD_STEP` | 10 | `<BBff` | 启动自动台阶流程 |
| `0x13` | `PC_CMD_ARM_SIMPLE` | 10 | `<BBff` | 简易机械臂 |
| `0x14` | `PC_CMD_ROD_NEW` | 6 | `<BBf` | 取矛头机构 |
| `0x15` | `PC_CMD_ORE_STORE` | 6 | `<BBf` | 矿仓平台 |
| `0x16` | `PC_CMD_AUTO_ACTION` | 1 | `<B` | 一键动作 |
| `0x17` | `PC_CMD_CAMERA_YAW` | 5 | `<Bf` | 相机云台 yaw |
| `0x18` | `PC_CMD_ABSTRACT_POSITION` | 5 | `<BBBBB` | 多机构抽象点位 |
| `0x20` | `PC_CMD_IMU` | 28 | `<fffffff` | PC 姿态 |
| `0x21` | `PC_CMD_IR_ORE_ACK` | 6 | `<6B` | 红外对接 ACK 透传 |
| `0x22` | `PC_CMD_R2_READY_STATE` | 1 | `<B` | R2 准备/重试状态，驱动灯效 |

### 4.1 心跳 `0x01`

无 payload。建议独立 10~20 Hz 发送，即使没有控制命令也保持链路在线。

### 4.2 底盘 `0x10`

| 偏移 | 类型 | 字段 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | f32 | `vx` | m/s | 车体系 x 方向速度 |
| 4 | f32 | `vy` | m/s | 车体系 y 方向速度 |
| 8 | f32 | `wz` | rad/s | yaw 角速度 |

### 4.3 撑杆 `0x11`

| 偏移 | 类型 | 字段 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | u8 | `mode` | - | `0=RELAX`，非 0=主动控制 |
| 1 | f32 | `lift0` | rad | 前组目标高度/角度 |
| 5 | f32 | `lift1` | rad | 后组目标高度/角度 |

发送直接撑杆命令会清除抽象位置命令中 `POLE` 模块 bit。

### 4.4 自动台阶 `0x12`

| 偏移 | 类型 | 字段 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | u8 | `template_id` | - | 见 `PC_StepTemplate_t` |
| 1 | u8 | `travel_dir` | - | 见 `PC_StepDir_t` |
| 2 | f32 | `target_yaw_rad` | rad | 目标航向 |
| 6 | f32 | `yaw_tolerance_rad` | rad | 航向允许误差 |

该 topic 保留为 AutoCtrl 直接接口，不属于 AutoAction 调用协议。生产业务中的普通上下台阶应使用 `PC_CMD_AUTO_ACTION (0x16)` 的 action `23~26`，并等待 `PC_FEEDBACK_AUTO_ACTION (0x96)` 整体终态；不要用 `0x12` 绕过 AutoAction 的单动作互斥和资源租约。

### 4.5 简易机械臂 `0x13`

| 偏移 | 类型 | 字段 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | u8 | `mode` | - | `0=RELAX`，`1=JOINT`，`2=POS_VEL(兼容，按角度控制)` |
| 1 | u8 | `point_mode` | - | `0=SLEEP`，`1=GRAB`，`2=LIFT`，`3=RELEASE`，`4=NONE` |
| 2 | f32 | `target_joint1_rad` | rad | 关节 1 目标 |
| 6 | f32 | `target_joint2_rad` | rad | 关节 2 目标 |

`point_mode != NONE` 时业务层可使用预设点位；`point_mode == NONE` 时使用两个目标角度。PC 不再控制吸盘，吸盘由遥控器/自动流程内部控制。发送直接机械臂命令会清除抽象位置命令中 `ARM_SIMPLE` 模块 bit。

### 4.6 取矛头机构 `0x14`

| 偏移 | 类型 | 字段 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | u8 | `mode` | - | `0=RELAX`，`1=ACTIVE` |
| 1 | u8 | `pose` | - | `0=STANDBY`，`1=GRAB_HIGH`，`2=DOCK_WAIT`，`3=MANUAL` |
| 2 | f32 | `target_angle_rad` | rad | `pose=MANUAL` 时的舵机目标角度 |

PC 不再控制取矛头夹爪，夹爪由遥控器/自动流程内部控制。发送直接取矛头命令会清除抽象位置命令中 `ROD_NEW` 模块 bit。

### 4.7 矿仓 `0x15`

| 偏移 | 类型 | 字段 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | u8 | `mode` | - | `0=RELAX`，`1=HOME`，`2=ACTIVE` |
| 1 | u8 | `force_rehome` | - | `0=不触发`，`1=触发一次重新回零` |
| 2 | f32 | `platform_target_rad` | rad | 平台轴目标位置 |

发送直接矿仓命令会清除抽象位置命令中 `ORE_STORE` 模块 bit。

### 4.8 一键动作 `0x16`

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | `action` | 见 `PC_AutoAction_t` |

一键动作是触发型命令，不是连续 setpoint：

- `action=0` 只清除固件侧 latch，不会取消已经接收、等待任务消费的动作；中止运行中动作必须发送 `action=1`。
- `action=1` 中止当前动作；底层真正进入中止终态后才清除 `busy` 并释放资源。
- `action=2~38` 启动对应整体动作；任意时刻最多运行一个 AutoAction，已有动作 `busy=1` 时不得启动、排队或提前准备另一个动作。
- 同一个非零 action 连续发送不会重复触发。动作 38 在启动失败、正常结束或中止后会由固件自动重新武装，可直接再次发送 38；其它动作再次调用前仍建议先发送 `action=0`，再发送目标 action。
- 动作启动后，PC 只等待相同 action 的 `0x96` 整体终态，不依据内部步骤、模块命令或单个机构到位判断完成。
- `busy=1` 期间固件保持自动命令路由和该动作的完整资源租约。PC 可以继续刷新心跳和连续控制帧，但不得假设这些帧已接管被占用模块；只有真实 `SUCCESS/FAIL/ABORTED` 终态后才释放租约。
- 普通动作、融合动作、普通台阶、`STEP1`、`STEP2` 都是独立整体 action。AutoOre 内部可以并行执行融合分支，但不会向 PC 暴露分段，也不会因内部某一分支完成而允许下一动作启动或模块提前接管。
- `action=33` 为回收地面矿并存矿：动作全程占用 POLE 并保持前/后组 `auto_ore_param.recover_pole_target_lift_rad={0.8,0.8} rad`；机械臂到 0mm 取矿位吸盘开启，底盘低速前进，当前 SICK 有效且原始 ADC `<= auto_ore_param.recover_front_sick_adc_threshold`（当前 2950）时触发，再继续前进 50 ms，随后静止吸附、后退避障，抬矿确认后存入低/高矿仓空位；前 SICK 未在安全超时内触发时动作失败并停车。该阈值不再复用通用 `sick_front_ore_adc_min/max` 窗口。

### 4.9 相机云台 yaw `0x17`

| 偏移 | 类型 | 字段 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | u8 | `mode` | - | 云台模式，当前仅缓存/调试 |
| 1 | f32 | `target_yaw_rad` | rad | 云台车身系目标 yaw |

该命令新鲜期为 1000 ms。当前控制任务不按 `mode` 放松云台；收到新鲜 PC yaw 时固定按 ACTIVE 使用目标 yaw。过期后当前实现会回到默认 active/0 rad 控制；遥控器特定开关组合可强制云台 relax。固件内部将该单电机 PC yaw 映射到实际工作的 `CAMERA_YAW_RIGHT` 通道。

### 4.10 抽象位置 `0x18`

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | `enable_mask` | 模块 bitmask |
| 1 | u8 | `arm_simple_position` | 机械臂抽象点位 |
| 2 | u8 | `rod_new_position` | 取矛头抽象点位 |
| 3 | u8 | `ore_store_position` | 矿仓抽象点位 |
| 4 | u8 | `pole_position` | 撑杆抽象点位 |

抽象位置命令不控制机械臂吸盘、取矛头夹爪和矿仓固定气缸；这些外设由遥控器/自动流程内部控制。

`enable_mask`：

| bit | 值 | 模块 |
|---:|---:|---|
| 0 | `0x01` | ARM_SIMPLE |
| 1 | `0x02` | ROD_NEW |
| 2 | `0x04` | ORE_STORE |
| 3 | `0x08` | POLE |

抽象点位值：

| 模块 | 值 |
|---|---|
| ARM_SIMPLE | `0=RELAX`，`1=SLEEP`，`2=GRAB`，`3=LIFT`，`4=RELEASE`，`16=BEHAVIOR_STANDBY`，`17=STORE_ORE`，`18=CHAMBER_ORE`，`19=WAIT_STORE_ORE`，`20=WAIT_RELEASE_ORE`，`21=RELEASE_ORE`，`22=PICK_POS_400`，`23=PICK_POS_200`，`24=PICK_NEG_200`，`25=PICK_LIFT_DETECT`，`26=RELEASE_ORE_ASSIST`，`27=VERTICAL` |
| ROD_NEW | `0=RELAX`，`1=STANDBY`，`2=GRAB_HIGH`，`3=DOCK_WAIT` |
| ORE_STORE | `0=RELAX`，`1=HOME`，`2=STANDBY`，`3=MID_WAIT`，`4=LIFT`，`5=BUFFER`，`6=SPEARHEAD_PICKUP` |
| POLE | `0=RELAX`，`1=STEP_200_ALL_EXTEND`，`2=STEP_200_FRONT_RETRACT`，`3=STEP_200_ALL_RETRACT`，`4=STEP_200_SMALL`，`5=STEP_400_ALL_EXTEND`，`6=STEP_400_FRONT_RETRACT`，`7=STEP_400_ALL_RETRACT`，`8=ORE_RELEASE` |

### 4.11 PC 姿态 `0x20`

| 偏移 | 类型 | 字段 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | f32 | `qw` | - | 四元数 w |
| 4 | f32 | `qx` | - | 四元数 x |
| 8 | f32 | `qy` | - | 四元数 y |
| 12 | f32 | `qz` | - | 四元数 z |
| 16 | f32 | `roll` | rad | 横滚 |
| 20 | f32 | `pitch` | rad | 俯仰 |
| 24 | f32 | `yaw` | rad | 航向 |

PC 启动自动台阶/一键融合流程后，AutoCtrl 会优先使用 PC 下发 yaw 作为 yaw 反馈；上位机应保持发送该帧。

### 4.12 红外对接 ACK 透传 `0x21`

payload 为 6 字节原始 ACK：

```
AA 55 82 msg_id status crc8
```

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | `0xAA` | 红外原始帧头 |
| 1 | u8 | `0x55` | 红外原始帧头 |
| 2 | u8 | `0x82` | ACK 命令 |
| 3 | u8 | `msg_id` | 必须匹配最近 `PC_FEEDBACK_IR_ORE_BRIDGE.msg_id` |
| 4 | u8 | `status` | `0=OK`，`1=BUSY`，`2=CRC_ERR`，`3=INVALID` |
| 5 | u8 | `crc8` | 对前 5 字节计算，初值 `0xFF` |

固件会校验 ACK 帧头、命令、CRC、`ack_pending` 和 `msg_id`，通过后转发到红外 UART。

```python
def ir_crc8(data: bytes, crc: int = 0xFF) -> int:
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
            crc &= 0xFF
    return crc

def build_ir_ack(msg_id: int, status: int) -> bytes:
    body = bytes([0xAA, 0x55, 0x82, msg_id & 0xFF, status & 0xFF])
    return body + bytes([ir_crc8(body)])
```

### 4.13 R2 准备/重试状态 `0x22`

payload 为 1 字节，固件收到后会刷新 PC 在线状态，并通过现有 CAN 接口发送 `ID=0x322, DLC=1` 灯效帧。

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | `state` | R2 准备状态 |

| state | 含义 | 转发 CAN Data[0] |
|---:|---|---:|
| `0` | R2 未准备完毕 | `0` |
| `1` | R2 已准备完毕 | `1` |
| `2` | R2 需要重试 | `2` |
| `3` | R2/上位机流程失败 | `3` |

其它 `state` 值按未准备完毕处理，即执行全灭灯效。取矛头等待对接动作会把 `state=1` 作为新协议的对接完成条件；UART8 红外命令 `02` 仍兼容。只有矿仓平台已到 `SPEARHEAD_DOCK_WAIT`、取矛头舵机当前姿态为 `DOCK_WAIT` 且到位后，固件才记录本机 ready 时间；对接完成帧的接收时间必须不早于该 ready 时间。提前发送的旧 `state=1`/`02` 不会在机构随后到位时补触发，发送端必须在机构就绪后重新发送。等待对接信号没有超时：未收到有效完成信号且 PC 未发送 `0x16 action=1` 时，动作保持运行并持续保持平台、舵机和夹爪命令。

一键放矿流程还会直接发送灯效：目标为低层转换仓时发送 `0x04`，目标为高层转换仓时发送 `0x05`；三层抬升检测放矿进入等待被抬升阶段时发送 `0x08`。AutoAction 的 PC 终态遵循 5.7 的兼容反馈策略；终态灯效跟随映射后的 PC 结果，因此兼容成功动作发送成功灯效 `0x07`，真实失败动作发送失败灯效 `0x03`。

### 4.14 CAN 灯效通信测试

固件内置 `g_light_effect_test_debug` 调试变量，可在 Ozone/调试器中直接测试 CAN2 灯条通信：

| 字段 | 用途 |
|---|---|
| `mode` | 测试模式号，建议 `0x00~0x08` |
| `command=1` | 立即发送一次 `mode`，发送后自动清零 |
| `last_sent_mode` / `last_result` / `tx_count` / `error_count` | 最近发送模式、返回值、成功次数、失败次数 |

默认测试帧为 CAN2 标准数据帧 `ID=0x322, DLC=1, Data[0]=模式号`。

## 5. STM32 发给 PC 的反馈

固件每 20 ms 尝试发送以下反馈批次：`0x81, 0x96, 0x90, 0x91, 0x93, 0x94, 0x95, 0x98, 0x92, 0x9B, 0x9C, 0x9D, 0x9A, 0xA0`。`0x02` 开始比赛命令仅在遥控器左右拨杆从已知非 PC 页切入 PC 页（`sw_l=UP && sw_r=UP`）的变化沿追加一次；上电时若拨杆已在 UP-UP，不会因初始状态未知而误发。`0x17` 仅作为 PC 下发的 camera yaw 命令使用，固件不再把它作为命令镜像回传。`0x97/0x99` 旧红外矿种反馈在首次收到或内容变化后持续低频追加，当前周期约 200 ms；`0x9A` 为新 R1/R2 红外对接协议状态反馈，随 50 Hz 常规反馈发送。

| cmd | 名称 | payload | Python unpack | 说明 |
|---:|---|---:|---|---|
| `0x02` | `PC_FEEDBACK_START_MATCH` | 1 | `<B` | 开始比赛命令；`start=0` 等待，`start=1` 开始 |
| `0x03` | `PC_FEEDBACK_RETRY` | 1 | `<B` | 重试命令；`retry=0` 默认不重试，`retry=1` 启动 PC 侧重试逻辑 |
| `0x81` | `PC_FEEDBACK_HEARTBEAT` | 0 | - | STM32 心跳 |
| `0x90` | `PC_FEEDBACK_CHASSIS` | 12 | `<fff` | 底盘速度反馈 |
| `0x91` | `PC_FEEDBACK_POLE` | 24 | `<ffffff` | 撑杆反馈 |
| `0x92` | `PC_FEEDBACK_STEP` | 10 | `<BBBBBBf` | 自动台阶状态 |
| `0x93` | `PC_FEEDBACK_ARM_SIMPLE` | 15 | `<BBBfff` | 简易臂状态 |
| `0x94` | `PC_FEEDBACK_ROD_NEW` | 20 | `<BBBBffff` | 取矛头状态 |
| `0x95` | `PC_FEEDBACK_ORE_STORE` | 12 | `<BBBBfBBBB` | 矿仓状态 |
| `0x96` | `PC_FEEDBACK_AUTO_ACTION` | 8 | `<BBBBHBB` | 一键动作整体运行状态、终态结果与兼容完成位 |
| `0x97` | `PC_FEEDBACK_IR_ORE` | 24 | `<BBBB12BII` | 红外矿种简表 |
| `0x98` | `PC_FEEDBACK_CAMERA_YAW` | 32 | `<BBBBffffffI` | 云台状态 |
| `0x99` | `PC_FEEDBACK_IR_ORE_BRIDGE` | 56 | `<BBBBBBBB12B18BxxIIII` | 红外桥接调试 |
| `0x9A` | `PC_FEEDBACK_IR_DOCK` | 3 | `<BBB` | 红外有效、新鲜状态与最近命令 |
| `0x9B` | `PC_FEEDBACK_SICK_CORRECT` | 20 | `<BBBBffff` | 取矛头 SICK 校正标准值/实时值 |
| `0x9C` | `PC_FEEDBACK_SICK_FRONT_ORE` | 1 | `<B` | 前 SICK 区域正方形矿检测结果 |
| `0x9D` | `PC_FEEDBACK_SICK_RAW` | 32 | `<I4f4HHBB` | 4 路 SICK 原始 ADC/距离/有效位 |
| `0xA0` | `PC_FEEDBACK_STATUS` | 10 | `<BIfB` | 通信/系统状态 |

### 5.0 一次性控制反馈 `0x02/0x03`

`PC_FEEDBACK_START_MATCH (0x02)` 与 `PC_FEEDBACK_RETRY (0x03)` 是 STM32 发给 PC 的一次性控制反馈，不在固定 50 Hz 批次列表中；固件内部有 pending 标志时追加到下一次发送批次，发送成功后自动清除。上位机应按 `cmd` 分发并把它们当作边沿事件处理，避免依赖持续电平。

`PC_FEEDBACK_START_MATCH` payload 为 1 字节，`start=1` 表示开始比赛。该帧只在操作手通过遥控器拨杆从已知非 PC 页切入 PC 页（`sw_l=UP && sw_r=UP`）的变化沿发送一次；上电初始状态为未知时不会触发，自动动作结束、红外对接完成等流程也不会触发该帧。

`PC_FEEDBACK_RETRY` payload 为 1 字节，`retry=1` 表示 PC 需要启动重试相关逻辑。该帧仅在 PC 页下左摇杆 X 从未触发越过重试阈值时发送一次；`retry=0` 保留为默认不重试状态，固件正常不会周期发送 `retry=0`。

### 5.1 底盘反馈 `0x90`

| 偏移 | 类型 | 字段 | 单位 |
|---:|---|---|---|
| 0 | f32 | `vx` | m/s |
| 4 | f32 | `vy` | m/s |
| 8 | f32 | `wz` | rad/s |

### 5.2 撑杆反馈 `0x91`

| 偏移 | 类型 | 字段 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | f32[2] | `lift` | rad | `[0]=前组，[1]=后组` |
| 8 | f32[4] | `motor_total_angle` | rad | 4 个撑杆电机累计角 |

### 5.3 自动台阶反馈 `0x92`

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | `state` | `0=IDLE`，`1=PREALIGN`，`2=RUNNING`，`3=SUCCESS`，`4=FAIL`，`5=ABORT` |
| 1 | u8 | `result` | `0=NONE`，`1=RUNNING`，`2=SUCCESS`，`3=FAIL`，`4=ABORTED` |
| 2 | u8 | `fault` | `0=NONE`，`1=INVALID_TEMPLATE`，`2=PREALIGN_TIMEOUT`，`3=TEMPLATE_TIMEOUT`，`4=SENSOR_INVALID`，`5=UNSUPPORTED`，`6=ABORTED` |
| 3 | u8 | `template_id` | 见命令模板枚举 |
| 4 | u8 | `step_index` | 模板内部步骤索引 |
| 5 | u8 | `reserved` | 固定 0 |
| 6 | f32 | `progress` | 0.0~1.0，当前多为 0 |

### 5.4 简易机械臂反馈 `0x93`

| 偏移 | 类型 | 字段 | 单位 |
|---:|---|---|---|
| 0 | u8 | `mode` | - |
| 1 | u8 | `point_mode` | - |
| 2 | u8 | `suction` | - |
| 3 | f32 | `joint1_angle_rad` | rad |
| 7 | f32 | `joint1_velocity_rad_s` | rad/s |
| 11 | f32 | `joint2_angle_rad` | rad |

### 5.5 取矛头反馈 `0x94`

| 偏移 | 类型 | 字段 | 单位 |
|---:|---|---|---|
| 0 | u8 | `mode` | - |
| 1 | u8 | `pose` | - |
| 2 | u8 | `grip` | - |
| 3 | u8 | `at_target` | - |
| 4 | f32 | `target_angle_rad` | rad |
| 8 | f32 | `tracked_angle_rad` | rad |
| 12 | f32 | `tracked_velocity_rad_s` | rad/s |
| 16 | f32 | `feedback_angle_rad` | rad |

### 5.6 矿仓反馈 `0x95`

| 偏移 | 类型 | 字段 | 单位/说明 |
|---:|---|---|---|
| 0 | u8 | `mode` | 矿仓当前模式 |
| 1 | u8 | `all_homed` | `0/1` |
| 2 | u8 | `online_mask` | bit0=平台轴在线 |
| 3 | u8 | `homed_mask` | bit0=平台轴已回零 |
| 4 | f32 | `platform_position_rad` | rad |
| 8 | u8 | `transform_low_has_ore` | `0/1` |
| 9 | u8 | `transform_high_has_ore` | `0/1` |
| 10 | u8 | `arm_has_ore` | `0/1` |
| 11 | u8 | `release_grid_has_ore` | 放矿目标格占矿检测结果，`0/1` |

### 5.7 一键动作反馈 `0x96`

payload 固定为 8 字节，Python unpack 格式为 `<BBBBHBB`。

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | `action` | 当前或最近一键动作，见 `PC_AutoAction_t` |
| 1 | u8 | `busy` | 任意一键动作是否正在执行，`0/1` |
| 2 | u8 | `finished` | 是否已有该 action 的整体结束结果，`0/1`；为 0 时忽略 `result` |
| 3 | u8 | `result` | 仅 `finished=1` 有效；`0=SUCCESS, 1=FAIL` |
| 4 | u16 | `failure_mask` | 真实失败位；成功或 synthetic success 时为 0 |
| 6 | u8 | `segment_finished_mask` | 兼容完成位：bit0=pick，bit1=store，bit2=step |
| 7 | u8 | `reserved` | 固定为 0 |

上位机解析规则：

1. `busy=1 && finished=0` 表示 `action` 正在整体执行。此期间自动路由和完整资源租约持续有效，PC 不得启动下一动作或提前接管该动作涉及的模块。
2. `busy=0 && finished=1` 才表示该 action 的底层执行器已真正进入 `SUCCESS`、`FAIL` 或 `ABORTED` 终态，资源随后才可释放。模块反馈、内部步骤或融合分支提前结束都不能替代此条件。
3. `RELEASE_STEP1`、`RELEASE_LIFT_DETECT_STEP1`、`ROD_SPEARHEAD_STEP1`、`ROD_SPEARHEAD_STEP2`、`ROD_DOCK_WAIT` 报告真实结果；真实失败或中止时 `result=1`。
4. 其他动作延续兼容策略：底层真实 `FAIL` 或 `ABORTED` 后才允许映射为 `result=0` 的 synthetic success。映射只改变 PC 看到的结果，不会提前结束动作或提前释放资源。
5. `action` 始终表示当前运行或最近结束的一键动作；空闲且从未执行过时为 `PC_AUTO_ACTION_NONE`。
6. 普通动作、融合动作、普通台阶、`STEP1`、`STEP2` 均只发布整体结果。PC 不得根据动作内部并行、执行阶段或模块命令推断完成。
7. `RELEASE_STEP1` 与 `RELEASE_STEP2`、`RELEASE_LIFT_DETECT_STEP1` 与 `RELEASE_LIFT_DETECT_STEP2` 分别是独立 action。`STEP1` 完成观察后立即进入自己的终态，同时保持 Arm 持矿状态；PC 读取目标格信息并重新调用独立的 `STEP2`，不存在续接控制帧。
8. `ROD_SPEARHEAD_STEP2` 成功终态同时设置 `segment_finished_mask.bit0=1`，兼容保存版上位机的取矛头完成判定；整体完成仍必须同时满足相同 action 的 `busy=0 && finished=1`。

Python 示例：

```python
import struct

action, busy, finished, result, failure_mask, segment_mask, _ = struct.unpack(
    "<BBBBHBB", payload
)

if busy:
    status = "RUNNING"
elif finished:
    status = "SUCCESS" if result == 0 else "FAIL"
else:
    status = "IDLE"


def start_auto_action(send, target_action: int):
    send(0x16, struct.pack("<B", 0))
    send(0x16, struct.pack("<B", target_action))
```

### 5.8 红外矿种简表 `0x97`

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | `valid` | 是否成功解析过矿种包 |
| 1 | u8 | `fresh` | 是否在 1000 ms 内收到矿种包 |
| 2 | u8 | `status` | 保留字段，当前矿位上报固定为 `0` |
| 3 | u8 | `count` | 固定 12 |
| 4 | u8[12] | `ore_type` | `0=UNKNOWN`，`1=R1`，`2=R2`，`3=FAKE` |
| 16 | u32 | `age_ms` | 距最近矿种包时间 |
| 20 | u32 | `rx_count` | 成功接收矿种包次数 |

### 5.8a 取矛头 SICK 校正反馈 `0x9B`

payload 为 20 字节，Python unpack 格式为 `<BBBBffff`。当前用于取矛头位置 1~6 的 SICK 校正实时观测；非取矛头 SICK 校正时 `action=0`，`position_index=0xFF`，`valid_mask=0`。

| 偏移 | 类型 | 字段 | 单位/说明 |
|---:|---|---|---|
| 0 | u8 | `action` | 当前取矛头 SICK 校正动作，`16~21` 对应位置 1~6；无效时为 0 |
| 1 | u8 | `position_index` | 取矛头位置索引，`0~5`；无效时为 `0xFF` |
| 2 | u8 | `valid_mask` | bit0 表示 X 字段有效，bit1 表示 Y 字段有效；空闲时只要 SICK 原始值有效也会置位 |
| 3 | u8 | `reserved` | 固定 0 |
| 4 | f32 | `x_target_adc` | X 方向 SICK 标准 ADC 值 |
| 8 | f32 | `x_sample_adc` | X 方向 SICK 实时 ADC 值 |
| 12 | f32 | `y_target_adc` | Y 方向 SICK 标准 ADC 值 |
| 16 | f32 | `y_sample_adc` | Y 方向 SICK 实时 ADC 值 |

固件会在每次 PC 发送批次前刷新该帧。取矛头 SICK 校正运行时，`action/position_index` 指向正在校正的位置；空闲时 `action=0`、`position_index=0xFF`，目标值使用位置 1 的配置，实时值仍来自当前 SICK 原始 ADC。当前控制默认仅使用 Y 方向，但帧会同时上报可读到的 X/Y 实时值，方便上位机调试。

### 5.8b 前 SICK 区域正方形矿检测反馈 `0x9C`

payload 为 1 字节，Python unpack 格式为 `<B`。固件直接比较前光电 `rawdata[2]` 的 SICK 原始 ADC，判断前方指定 ADC 窗口内是否有矿；`detected=1` 表示样本持续落在窗口内并通过稳定时间确认。窗口由 `Config_GetRobotParam()->auto_ctrl_param.common.sick_front_ore_adc_min/max/stable_ms` 配置。

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | `detected` | 稳定确认后的矿检测结果，`0=无矿`，`1=有矿` |

### 5.8c SICK 原始数据反馈 `0x9D`

payload 为 32 字节，Python unpack 格式为 `<I4f4HHBB`。固件在每次 PC 发送批次前读取 `Task_SickGetLatestOutput()`，将 4 路 SICK 原始 ADC、距离和有效位发送给上位机；读取失败时整帧填 0。

| 偏移 | 类型 | 字段 | 单位/说明 |
|---:|---|---|---|
| 0 | u32 | `update_tick` | 最近一次 SICK 数据更新时间，单位 ms |
| 4 | f32[4] | `distance_mm` | 4 路距离，单位 mm；通道顺序为 0~3 |
| 20 | u16[4] | `adc_raw` | 4 路原始 ADC；通道顺序为 0~3 |
| 28 | u16 | `miss_count` | 连续丢帧/失效计数 |
| 30 | u8 | `valid_mask` | bit0~3 对应通道 0~3，1 表示该通道有效 |
| 31 | u8 | `reserved` | 固定 0 |

当前 SICK 物理通道映射：`[0]=bottom photoelectric`、`[1]=rod-side photoelectric`、`[2]=front photoelectric`、`[3]=unused`。

### 5.9 相机云台反馈 `0x98`

每个数组长度均为 2，索引 0=左云台，1=右云台。

| 偏移 | 类型 | 字段 | 单位/说明 |
|---:|---|---|---|
| 0 | u8[2] | `mode` | `0=RELAX`，`1=ACTIVE` |
| 2 | u8[2] | `motor_online` | `0/1` |
| 4 | u8[2] | `feedback_valid` | `0/1` |
| 6 | u8[2] | `at_target` | `0/1` |
| 8 | f32[2] | `target_yaw_rad` | rad |
| 16 | f32[2] | `feedback_yaw_rad` | rad |
| 24 | f32[2] | `error_yaw_rad` | rad |
| 32 | f32[2] | `motor_angle_rad` | rad |
| 40 | f32[2] | `motor_velocity_rad_s` | rad/s |
| 48 | f32[2] | `output` | 电机控制输出 |
| 56 | u32[2] | `feedback_age_ms` | ms |

### 5.10 红外桥接反馈 `0x99`

该反馈包含 2 字节 padding，便于对齐后面的 `u32` 字段。上位机用 Python unpack 时格式为 `<BBBBBBBB12B18BxxIIII`。

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | `valid` | 是否成功解析过 18 字节红外矿种帧 |
| 1 | u8 | `fresh` | 是否在 1000 ms 内收到矿种帧 |
| 2 | u8 | `status` | 保留字段，当前矿位上报固定为 `0` |
| 3 | u8 | `count` | 固定 12 |
| 4 | u8 | `msg_id` | 最近矿种帧消息 ID |
| 5 | u8 | `side` | 红外端侧别字段 |
| 6 | u8 | `ack_pending` | 固件是否等待 ACK |
| 7 | u8 | `parse_status` | `0=OK`，`1=BUSY`，`2=CRC_ERR`，`3=INVALID` |
| 8 | u8[12] | `ore_type` | 12 个矿位种类 |
| 20 | u8[18] | `raw_frame` | 原始红外矿种帧 |
| 38 | u8[2] | padding | 固件 C 结构体对齐填充 |
| 40 | u32 | `age_ms` | 距最近成功矿种帧时间 |
| 44 | u32 | `rx_count` | 成功矿种包次数 |
| 48 | u32 | `frame_rx_count` | 成功 18 字节帧次数 |
| 52 | u32 | `ack_tx_count` | ACK 转发次数 |

红外矿种原始帧格式：

```
AA 55 02 msg_id side ore_type[12] crc8
```

### 5.11 R1/R2 红外对接反馈 `0x9A`

该反馈来自 UART8 上的 R1/R2 红外新协议帧：

```
A5 command
```

固定包头为 `0xA5`，`command` 为一个二进制数据字节，不使用 CRC：

| command | 含义 | 固件行为 |
|---:|---|---|
| `1` | 在线 | 刷新红外在线状态 |
| `2` | 对接完成 | 刷新对接完成状态及时间戳 |
| `3` | 允许放矿 | 仅动作 38 锁定期间刷新放矿许可；本地准备完成后收到的新许可才有效 |
| `4` | 结束并解除三层矿动作锁 | 解除本轮锁定、清除许可；动作 38 将 Arm 收回 `STANDBY`，到位后结束 |

所有合法命令都会刷新红外在线状态；`command` 不在 `1~4` 时按无效帧统计。新一轮三层放矿流程为 `PC 发送 0x16 action=38 -> 红外 3 执行放矿循环 -> 红外 4 正常结束`。每次在机构就绪后收到的新命令 `3` 可触发一次放矿循环，循环完成后动作返回等待态，`busy` 仍保持 1，自动路由和完整资源租约均不释放。命令 `4` 使动作正常收尾；需要中止时 PC 发送 `0x16 action=1`。旧命令 `5/6` 无效。`0x9A` payload 为 3 字节，Python 解包格式为 `<BBB`。

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | `valid` | 是否曾成功解析过新红外帧，`0/1` |
| 1 | u8 | `fresh` | 最近一帧是否仍在超时时间内，`0/1` |
| 2 | u8 | `command` | 最近成功解析的原始命令，取值 `1~4` |

### 5.12 状态反馈 `0xA0`

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | `online` | PC 链路在线 |
| 1 | u32 | `recv_count` | 已成功接收的 PC 帧计数 |
| 5 | f32 | `cpu_temp` | STM32 CPU 温度，degC |
| 9 | u8 | `command_source` | `0=RC`，`1=PC` |

## 6. 常用枚举

### 6.1 自动台阶

`PC_StepTemplate_t`：`0=NONE`，`1=ASCEND_200_HEAD`，`2=ASCEND_400_HEAD`，`3=DESCEND_200_HEAD`，`4=DESCEND_400_HEAD`。

`PC_StepDir_t`：`0=HEAD_FORWARD`。当前仅支持头向台阶流程。

### 6.2 一键动作

`PC_AutoAction_t`：

| 值 | 动作 |
|---:|---|
| 0 | NONE |
| 1 | ABORT |
| 2 | STORE；若最终存入低位，平台返回 STANDBY 后执行小幅快速上下颠动，最后回到 STANDBY 才结束 |
| 3 | RELEASE |
| 4 | RELEASE_LIFT_DETECT，Pole 到放矿目标后比较预留 SICK 原始 ADC 阈值，确认被抬起后继续放矿 |
| 5 | CHAMBER |
| 6 | PICK_POS_400 |
| 7 | PICK_POS_200 |
| 8 | PICK_NEG_200 |
| 9 | PICK_STORE_POS_400，取正 400mm 矿后并行存矿和后退 |
| 10 | PICK_STORE_POS_200，取正 200mm 矿后并行存矿和后退 |
| 11 | PICK_STORE_NEG_200，取反 200mm 矿后并行存矿和后退 |
| 12 | ROD_SPEARHEAD |
| 13 | ROD_SPEARHEAD_STEP1 |
| 14 | ROD_SPEARHEAD_STEP2 |
| 15 | ROD_DOCK_WAIT；机构到位后无限等待有效对接完成信号，仅 `0x16 action=1` 或安全中止可结束等待 |
| 16 | SICK_CORRECT_ROD_SPEARHEAD_POS1，取矛头位置 1 的 SICK 校正：X/Y 由固件宏开关控制，当前 x/y 都关闭，直接 SUCCESS；重新开启后 y 使用 rod 侧光电 rawdata[1]，x 使用前光电 rawdata[3]；超时后启用轴误差均 <=15 ADC 则 SUCCESS |
| 17 | SICK_CORRECT_ROD_SPEARHEAD_POS2，取矛头位置 2 的 SICK 校正：X/Y 由固件宏开关控制，当前 x/y 都关闭，直接 SUCCESS；重新开启后 y 使用 rod 侧光电 rawdata[1]，x 使用前光电 rawdata[3]；超时后启用轴误差均 <=15 ADC 则 SUCCESS |
| 18 | SICK_CORRECT_ROD_SPEARHEAD_POS3，取矛头位置 3 的 SICK 校正：X/Y 由固件宏开关控制，当前 x/y 都关闭，直接 SUCCESS；重新开启后 y 使用 rod 侧光电 rawdata[1]，x 使用前光电 rawdata[3]；超时后启用轴误差均 <=15 ADC 则 SUCCESS |
| 19 | SICK_CORRECT_ROD_SPEARHEAD_POS4，取矛头位置 4 的 SICK 校正：X/Y 由固件宏开关控制，当前 x/y 都关闭，直接 SUCCESS；重新开启后 y 使用 rod 侧光电 rawdata[1]，x 使用前光电 rawdata[3]；超时后启用轴误差均 <=15 ADC 则 SUCCESS |
| 20 | SICK_CORRECT_ROD_SPEARHEAD_POS5，取矛头位置 5 的 SICK 校正：X/Y 由固件宏开关控制，当前 x/y 都关闭，直接 SUCCESS；重新开启后 y 使用 rod 侧光电 rawdata[1]，x 使用前光电 rawdata[3]；超时后启用轴误差均 <=15 ADC 则 SUCCESS |
| 21 | SICK_CORRECT_ROD_SPEARHEAD_POS6，取矛头位置 6 的 SICK 校正：X/Y 由固件宏开关控制，当前 x/y 都关闭，直接 SUCCESS；重新开启后 y 使用 rod 侧光电 rawdata[1]，x 使用前光电 rawdata[3]；超时后启用轴误差均 <=15 ADC 则 SUCCESS |
| 22 | SICK_CORRECT_ORE_RELEASE，放矿前 SICK 校正：仅 x 方向，使用前光电 rawdata[3] |
| 23 | STEP_ASCEND_200_HEAD |
| 24 | STEP_DESCEND_200_HEAD |
| 25 | STEP_ASCEND_400_HEAD |
| 26 | STEP_DESCEND_400_HEAD |
| 27 | STEP_PICK_STORE_ASCEND_200_HEAD |
| 28 | STEP_PICK_STORE_DESCEND_200_HEAD |
| 29 | STEP_PICK_STORE_ASCEND_400_HEAD |
| 30 | STEP_DROP_STORE_ASCEND_200_HEAD，丢矿版融合头向上 200mm 台阶；仅高位有矿时启动，存矿阶段强制存高位 |
| 31 | STEP_DROP_STORE_DESCEND_200_HEAD，丢矿版融合头向下 200mm 台阶；仅高位有矿时启动，存矿阶段强制存高位 |
| 32 | STEP_DROP_STORE_ASCEND_400_HEAD，丢矿版融合头向上 400mm 台阶；仅高位有矿时启动，存矿阶段强制存高位 |
| 33 | RECOVER_STORE，回收地面矿并存矿；全程占用 POLE 并保持前/后 `0.8 rad`；要求机械臂无矿、低/高矿仓有空位、矿仓已回零 |
| 34 | RELEASE_STEP1，独立整体 action：机械臂到竖直观察位并检测目标格，观察完成即终态，同时保持 Arm 持矿状态 |
| 35 | RELEASE_STEP2，独立整体 action：PC 后续单独调用，从保留的持矿姿态完成释放 |
| 36 | RELEASE_LIFT_DETECT_STEP1，独立整体 action：完成带 SICK 抬升检测的观察阶段后即终态，同时保持 Arm 持矿状态 |
| 37 | RELEASE_LIFT_DETECT_STEP2，独立整体 action：PC 后续单独调用并完成释放 |
| 38 | RELEASE_IR_LIFT_DETECT，红外协同三层放矿整体 action：规范 payload 字节为十进制 `38`（`0x26`）；固件兼容误按十六进制发送的 `0x38`。红外 `3` 执行放矿循环，红外 `4` 正常结束，`0x16 action=1` 中止 |

PC 通过 `0x16 action=38` 启动动作。启动成功后，动作在整个运行期持续占有 CHASSIS、POLE、ARM、STORE、ROD 和共享气路等完整资源，并持续输出自动路由命令。准备顺序为 Pole 展开、从矿仓上膛（如需要）、机械臂到 `WAIT_RELEASE_ORE`；平台、机械臂和 Pole 到位后才记录许可就绪时间。早于就绪时间的红外命令 `3` 不会补触发，R1 必须在就绪后重新发送。动作 38 忽略前方目标格光电/SICK 有无矿结果，不拆分为 `STEP1/STEP2`。每个有效红外命令 `3` 触发一次完整放矿循环，完成后回到 `WAIT_RELEASE_ORE` 继续保持 `busy=1` 和全部资源；红外命令 `4` 才启动正常收尾并在 Arm 回到 `STANDBY` 后形成整体成功终态。需要中止时 PC 发送 `0x16 action=1`；底层真正进入 `ABORTED` 后才令 `busy=0` 并释放资源，wire `result` 仍按 5.7 的兼容策略映射。

### 6.3 红外对接

UART8 简化帧为两个字节：`0xA5 command`。命令定义为 `1=在线`、`2=对接完成`、`3=允许放矿`、`4=结束并解除三层矿动作锁`，不使用 CRC 或 ACK；`5/6` 为非法命令。

## 7. 上位机推荐流程

1. 打开串口 `115200 8N1`。
2. 启动独立 RX 线程，按 mrlink 帧格式循环解析，允许一次读到多帧或半帧。
3. 周期发送心跳 `0x01`，10~20 Hz。
4. 需要控制时，让操作手把遥控器切到 PC 页；上位机等待 `PC_FEEDBACK_STATUS.online=1` 且 `command_source=1`。
5. 连续控制类命令按 20~50 Hz 发送即可，例如底盘、姿态、相机 yaw。
6. 调用 AutoAction 时可发送 `0x16 action=0` 清 latch，再发送目标 action；动作 38 在启动失败、结束或中止后由固件自动重新武装。等待相同 action 的 `0x96 busy=0 && finished=1` 后读取 `result`，才能调用下一动作或接管其资源。中止当前动作直接发送 `0x16 action=1`。
7. 不要 50 Hz 连续刷同一个一键动作，也不要根据模块反馈、内部阶段或融合分支提前启动下一动作。
8. 解析反馈时优先用 `cmd` 分发，不要假设一批反馈中一定包含所有 topic；IR 反馈是条件发送。

## 8. Python 发送示例

```python
import serial
import struct
import time

ser = serial.Serial("COM3", 115200, timeout=0.01)

def send(cmd, payload=b""):
    ser.write(build_mrlink_frame(cmd, payload))

while True:
    send(0x01)  # heartbeat
    chassis = struct.pack("<fff", 0.2, 0.0, 0.0)
    send(0x10, chassis)
    time.sleep(0.05)
```

启动一个普通头向上 200mm 台阶整体动作：

```python
PC_AUTO_ACTION_STEP_ASCEND_200_HEAD = 23
send(0x16, struct.pack("<B", 0))
send(0x16, struct.pack("<B", PC_AUTO_ACTION_STEP_ASCEND_200_HEAD))
```

`PC_CMD_STEP (0x12)` 当前只保留解析兼容，不会直接启动 AutoCtrl；普通台阶也必须作为整体 AutoAction 通过 `0x16` 调用。

调用并等待一个整体 AutoAction：

```python
def call_auto_action(action_id: int):
    send(0x16, struct.pack("<B", 0))
    send(0x16, struct.pack("<B", action_id))


def auto_action_terminal(payload: bytes, expected_action: int):
    action, busy, finished, result, failure_mask, segment_mask, _ = struct.unpack(
        "<BBBBHBB", payload
    )
    if action != expected_action or busy or not finished:
        return None
    return result == 0
```

发送红外 ACK：

```python
# 从 PC_FEEDBACK_IR_ORE_BRIDGE 中取得 msg_id，并确认 ack_pending == 1
ack = build_ir_ack(msg_id, status=0)
send(0x21, ack)
```
