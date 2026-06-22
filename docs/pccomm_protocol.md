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

固件 `pc_comm_task` 每 1 ms 处理接收，每 20 ms 发送一批反馈帧，反馈频率约 50 Hz。红外矿种反馈仅在数据变化或 500 ms 心跳到期时追加发送。

PC 在线判定：任意一帧合法 PC_COMM 帧都会刷新在线状态。若 500 ms 内没有收到合法帧，固件认为 PC 离线并回到 RC 控制模式。推荐上位机持续发送 `PC_CMD_HEARTBEAT (0x01)`，10~20 Hz 即可；高频控制命令本身也会保活。

PC 控制命令实际生效还要求遥控器进入 PC 控制页：当前代码条件为 `sw_l == UP && sw_r == UP` 且 PC 链路在线。上位机可通过 `PC_FEEDBACK_STATUS.command_source == 1` 判断当前是否由 PC 接管。

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
| `0x13` | `PC_CMD_ARM_SIMPLE` | 11 | `<BBBff` | 简易机械臂 |
| `0x14` | `PC_CMD_ROD_NEW` | 7 | `<BBBf` | 取矛头机构 |
| `0x15` | `PC_CMD_ORE_STORE` | 6 | `<BBf` | 矿仓平台 |
| `0x16` | `PC_CMD_AUTO_ACTION` | 1 | `<B` | 一键动作 |
| `0x17` | `PC_CMD_CAMERA_YAW` | 10 | `<BBff` | 相机云台 yaw |
| `0x18` | `PC_CMD_ABSTRACT_POSITION` | 8 | `<BBBBBBBB` | 多机构抽象点位 |
| `0x20` | `PC_CMD_IMU` | 28 | `<fffffff` | PC 姿态 |
| `0x21` | `PC_CMD_IR_ORE_ACK` | 6 | `<6B` | 红外对接 ACK 透传 |

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

固件仅在 PC 接管、AutoCtrl 空闲、矿仓/取矛头/SICK 一键动作均空闲时启动。成功启动后固件会清除该命令。

### 4.5 简易机械臂 `0x13`

| 偏移 | 类型 | 字段 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | u8 | `mode` | - | `0=RELAX`，`1=JOINT`，`2=POS_VEL(兼容，按角度控制)` |
| 1 | u8 | `point_mode` | - | `0=SLEEP`，`1=GRAB`，`2=LIFT`，`3=RELEASE`，`4=NONE` |
| 2 | u8 | `suction` | - | `0=关闭`，`1=开启` |
| 3 | f32 | `target_joint1_rad` | rad | 关节 1 目标 |
| 7 | f32 | `target_joint2_rad` | rad | 关节 2 目标 |

`point_mode != NONE` 时业务层可使用预设点位；`point_mode == NONE` 时使用两个目标角度。发送直接机械臂命令会清除抽象位置命令中 `ARM_SIMPLE` 模块 bit。

### 4.6 取矛头机构 `0x14`

| 偏移 | 类型 | 字段 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | u8 | `mode` | - | `0=RELAX`，`1=ACTIVE` |
| 1 | u8 | `pose` | - | `0=STANDBY`，`1=GRAB_HIGH`，`2=DOCK_WAIT`，`3=MANUAL` |
| 2 | u8 | `grip` | - | `0=松开`，`1=夹紧` |
| 3 | f32 | `target_angle_rad` | rad | `pose=MANUAL` 时的舵机目标角度 |

发送直接取矛头命令会清除抽象位置命令中 `ROD_NEW` 模块 bit。

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

- `action=0` 清除固件侧 latch。
- 新的非零 action 会立即接受。
- 同一个非零 action 连续高速发送不会重复触发；需要停止发送超过 300 ms，或先发 `action=0` 再发同一 action。
- `action=8` 为中止。

### 4.9 相机云台 yaw `0x17`

| 偏移 | 类型 | 字段 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | u8 | `mode[0]` | - | 左云台，当前仅缓存/调试 |
| 1 | u8 | `mode[1]` | - | 右云台，当前仅缓存/调试 |
| 2 | f32 | `target_yaw_rad[0]` | rad | 左云台车身系目标 yaw |
| 6 | f32 | `target_yaw_rad[1]` | rad | 右云台车身系目标 yaw |

该命令新鲜期为 1000 ms。当前控制任务不按 `mode[]` 放松云台；收到新鲜 PC yaw 时固定按 ACTIVE 使用目标 yaw。过期后当前实现会回到默认 active/0 rad 控制；遥控器特定开关组合可强制云台 relax。

### 4.10 抽象位置 `0x18`

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | `enable_mask` | 模块 bitmask |
| 1 | u8 | `arm_simple_position` | 机械臂抽象点位 |
| 2 | u8 | `arm_simple_suction` | `0=关闭`，非 0=开启 |
| 3 | u8 | `rod_new_position` | 取矛头抽象点位 |
| 4 | u8 | `rod_new_grip` | `0=松开`，非 0=夹紧 |
| 5 | u8 | `ore_store_position` | 矿仓抽象点位 |
| 6 | u8 | `ore_store_cylinder_closed` | `0=打开`，非 0=关闭 |
| 7 | u8 | `pole_position` | 撑杆抽象点位 |

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
| ARM_SIMPLE | `0=RELAX`，`1=SLEEP`，`2=GRAB`，`3=LIFT`，`4=RELEASE`，`16=BEHAVIOR_STANDBY`，`17=STORE_ORE`，`18=CHAMBER_ORE`，`19=WAIT_STORE_ORE`，`20=WAIT_RELEASE_ORE`，`21=RELEASE_ORE`，`22=PICK_POS_400`，`23=PICK_POS_200`，`24=PICK_NEG_200` |
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

## 5. STM32 发给 PC 的反馈

固件每 20 ms 尝试发送以下反馈批次：`0x81, 0x96, 0x90, 0x91, 0x93, 0x94, 0x95, 0x98, 0x92, 0xA0`。`0x97/0x99` 红外反馈仅在数据变化或 500 ms 心跳到期时追加。

| cmd | 名称 | payload | Python unpack | 说明 |
|---:|---|---:|---|---|
| `0x81` | `PC_FEEDBACK_HEARTBEAT` | 0 | - | STM32 心跳 |
| `0x90` | `PC_FEEDBACK_CHASSIS` | 12 | `<fff` | 底盘速度反馈 |
| `0x91` | `PC_FEEDBACK_POLE` | 24 | `<ffffff` | 撑杆反馈 |
| `0x92` | `PC_FEEDBACK_STEP` | 10 | `<BBBBBBf` | 自动台阶状态 |
| `0x93` | `PC_FEEDBACK_ARM_SIMPLE` | 15 | `<BBBfff` | 简易臂状态 |
| `0x94` | `PC_FEEDBACK_ROD_NEW` | 20 | `<BBBBffff` | 取矛头状态 |
| `0x95` | `PC_FEEDBACK_ORE_STORE` | 12 | `<BBBBfBBBB` | 矿仓状态 |
| `0x96` | `PC_FEEDBACK_AUTO_ACTION` | 6 | `<BBBBH` | 一键动作简化结果 |
| `0x97` | `PC_FEEDBACK_IR_ORE` | 24 | `<BBBB12BII` | 红外矿种简表 |
| `0x98` | `PC_FEEDBACK_CAMERA_YAW` | 64 | `<2B2B2B2B2f2f2f2f2f2f2I` | 云台状态 |
| `0x99` | `PC_FEEDBACK_IR_ORE_BRIDGE` | 56 | `<BBBBBBBB12B18BxxIIII` | 红外桥接调试 |
| `0xA0` | `PC_FEEDBACK_STATUS` | 10 | `<BIfB` | 通信/系统状态 |

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
| 11 | u8 | `reserved` | 固定 0 |

### 5.7 一键动作反馈 `0x96`

payload 为 6 字节，Python unpack 格式为 `<BBBBH`。

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | `action` | 当前或最近一键动作，见 `PC_AutoAction_t` |
| 1 | u8 | `busy` | 任意一键动作是否正在执行，`0/1` |
| 2 | u8 | `finished` | 是否已有结束结果，`0/1`；为 0 时忽略 `result` |
| 3 | u8 | `result` | 仅 `finished=1` 有效，`0=SUCCESS`，`1=FAIL` |
| 4 | u16 | `failure_mask` | 仅 `result=FAIL` 有效，little-endian bitmask |

`failure_mask`：

| bit | 名称 | 说明 |
|---:|---|---|
| 0 | `SETUP` | 启动条件/参数/配置类失败 |
| 1 | `PICK_ORE` | 取矿失败 |
| 2 | `STORE_ORE` | 存矿失败 |
| 3 | `RELEASE_ORE` | 放矿失败 |
| 4 | `CHAMBER` | 上膛失败 |
| 5 | `STEP` | 上台阶失败 |
| 6 | `ROD_SPEARHEAD` | 取矛头失败 |
| 7 | `ROD_DOCK_WAIT` | 矛头对接等待失败 |
| 8 | `SICK_CORRECT` | SICK 校正失败 |
| 9 | `ABORTED` | 人为中止 |

上位机解析规则：

1. `busy=1` 表示一键动作正在执行，此时 `finished=0`，不要使用 `result/failure_mask` 判定结束。
2. `busy=0 && finished=1 && result=0` 表示最近一次一键动作成功结束，`failure_mask=0`。
3. `busy=0 && finished=1 && result=1` 表示最近一次一键动作失败结束，按 `failure_mask` 判断失败部位；融合动作可能同时置多个 bit，例如 `STORE_ORE | STEP`。
4. `ABORT` 也按失败处理：`result=1`，`failure_mask` 包含 `ABORTED`。
5. `action` 始终表示当前运行或最近结束的一键动作；空闲且从未执行过时为 `PC_AUTO_ACTION_NONE`。

Python 示例：

```python
import struct

FAILURE_BITS = {
    0: "SETUP",
    1: "PICK_ORE",
    2: "STORE_ORE",
    3: "RELEASE_ORE",
    4: "CHAMBER",
    5: "STEP",
    6: "ROD_SPEARHEAD",
    7: "ROD_DOCK_WAIT",
    8: "SICK_CORRECT",
    9: "ABORTED",
}

action, busy, finished, result, failure_mask = struct.unpack("<BBBBH", payload)
if busy:
    status = "RUNNING"
elif finished and result == 0:
    status = "SUCCESS"
elif finished and result == 1:
    failed_parts = [
        name for bit, name in FAILURE_BITS.items() if failure_mask & (1 << bit)
    ]
    status = "FAIL: " + "|".join(failed_parts)
else:
    status = "IDLE"
```

### 5.8 红外矿种简表 `0x97`

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | `valid` | 是否成功解析过矿种包 |
| 1 | u8 | `fresh` | 是否在 1000 ms 内收到矿种包 |
| 2 | u8 | `status` | `0=IDLE`，`1=DOCKING`，`2=DOCK_COMPLETE` |
| 3 | u8 | `count` | 固定 12 |
| 4 | u8[12] | `ore_type` | `0=UNKNOWN`，`1=R1`，`2=R2`，`3=FAKE` |
| 16 | u32 | `age_ms` | 距最近矿种包时间 |
| 20 | u32 | `rx_count` | 成功接收矿种包次数 |

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
| 2 | u8 | `status` | `0=IDLE`，`1=DOCKING`，`2=DOCK_COMPLETE` |
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

### 5.11 状态反馈 `0xA0`

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | `online` | PC 链路在线 |
| 1 | u32 | `recv_count` | 已成功接收的 PC 帧计数 |
| 5 | f32 | `cpu_temp` | STM32 CPU 温度，degC |
| 9 | u8 | `command_source` | `0=RC`，`1=PC` |

## 6. 常用枚举

### 6.1 自动台阶

`PC_StepTemplate_t`：`0=NONE`，`1=ASCEND_200_HEAD`，`2=ASCEND_400_HEAD`，`3=DESCEND_200_HEAD`，`4=DESCEND_400_HEAD`，`5=ASCEND_200_TAIL`，`6=ASCEND_400_TAIL`，`7=DESCEND_200_TAIL`，`8=DESCEND_400_TAIL`。

`PC_StepDir_t`：`0=HEAD_FORWARD`，`1=TAIL_FORWARD`。

### 6.2 一键动作

`PC_AutoAction_t`：

| 值 | 动作 |
|---:|---|
| 0 | NONE |
| 1 | STORE |
| 2 | RELEASE |
| 3 | CHAMBER |
| 4 | PICK_POS_400 |
| 5 | PICK_POS_200 |
| 6 | PICK_NEG_200 |
| 7 | ROD_SPEARHEAD |
| 8 | ABORT |
| 9 | SICK_CORRECT_ROD_SPEARHEAD |
| 10 | SICK_CORRECT_ORE_RELEASE，当前预留/不支持 |
| 11 | ROD_DOCK_WAIT |
| 12 | STEP_PICK_STORE_ASCEND_200_HEAD |
| 13 | STEP_PICK_STORE_DESCEND_200_HEAD |
| 14 | STEP_PICK_STORE_ASCEND_400_HEAD |
| 15 | STEP_PICK_STORE_DESCEND_400_HEAD |

### 6.3 红外对接

矿种：`0=UNKNOWN`，`1=R1`，`2=R2`，`3=FAKE`。

状态：`0=IDLE`，`1=DOCKING`，`2=DOCK_COMPLETE`。

ACK/解析状态：`0=OK`，`1=BUSY`，`2=CRC_ERR`，`3=INVALID`。

## 7. 上位机推荐流程

1. 打开串口 `115200 8N1`。
2. 启动独立 RX 线程，按 mrlink 帧格式循环解析，允许一次读到多帧或半帧。
3. 周期发送心跳 `0x01`，10~20 Hz。
4. 需要控制时，让操作手把遥控器切到 PC 页；上位机等待 `PC_FEEDBACK_STATUS.online=1` 且 `command_source=1`。
5. 连续控制类命令按 20~50 Hz 发送即可，例如底盘、姿态、相机 yaw。
6. 触发类命令只发一次或低频发，例如一键动作和自动台阶。不要 50 Hz 连续刷同一个一键动作。
7. 解析反馈时优先用 `cmd` 分发，不要假设一批反馈中一定包含所有 topic；IR 反馈是条件发送。

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

启动一个自动上 200mm 台阶流程：

```python
PC_STEP_TEMPLATE_ASCEND_200_HEAD = 1
PC_STEP_DIR_HEAD_FORWARD = 0

payload = struct.pack(
    "<BBff",
    PC_STEP_TEMPLATE_ASCEND_200_HEAD,
    PC_STEP_DIR_HEAD_FORWARD,
    0.0,   # target_yaw_rad
    0.08,  # yaw_tolerance_rad
)
send(0x12, payload)
```

发送红外 ACK：

```python
# 从 PC_FEEDBACK_IR_ORE_BRIDGE 中取得 msg_id，并确认 ack_pending == 1
ack = build_ir_ack(msg_id, status=0)
send(0x21, ack)
```
