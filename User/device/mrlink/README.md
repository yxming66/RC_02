# mrlink 通用通信协议库

轻量级 byte-stream 帧协议库，面向 STM32 / 嵌入式 MCU。协议帧头 `{0x4D, 0x52}` = ASCII "MR"。

## 文件清单

| 文件 | 类型 | 作用 |
|---|---|---|
| `mrlink_ringbuf.h` / `.c` | C 库 | SPSC 字节环形缓冲 (零 OS 依赖) |
| `mrlink.h` / `.c` | C 库 | 通用 frame 协议 (32 槽 cmd 派发) |
| `mrlink.hpp` | C++ 包装 | OneMessage-style typed 注册 API (namespace `mrlink`) |

## 特性

- **帧头** `{0x4D, 0x52}` = ASCII "MR"（项目标识）
- **CRC16-CCITT-FALSE** 可选（`MrLink_Config.use_crc16`）
- **32 槽** cmd 派发表，O(N) 线性扫描
- **零 OS 依赖**（除 `cmsis_compiler.h` 用于 ISR 临界区）
- **加新命令 = 0 库代码修改**：写 handler + 1 行 `MrLink_RegisterHandler`
- **C 库 typed API**（`MrLink_RegisterTypedHandler`）：库内自动 length check + memcpy
- **C++ typed wrapper**（`mrlink.hpp`）：每条命令 1 行 lambda，库内自动 length check + memcpy
- **C++ 命名空间**：`mrlink::Instance`

## 帧格式

```
use_crc16 = true:  [0x4D][0x52][len][cmd][payload...][CRC_lo][CRC_hi]
use_crc16 = false: [0x4D][0x52][len][cmd][payload...]
```

| 字段 | 长度 | 说明 |
|---|---|---|
| header | 2B | 写死 `{0x4D, 0x52}` = "MR" |
| length | 1B | payload 字节数 (0~max_payload_size) |
| cmd | 1B | 命令字，库不解释 |
| payload | 0~64B | 业务数据 |
| CRC16 | 0 或 2B | CCITT-FALSE, init=0xFFFF, poly=0x1021, 小端写 |

**最大帧长**：6 + max_payload（默认 70 字节）

## 内存开销

| 组件 | 大小 / 实例 |
|---|---|
| `MrLink_RingBuf_t` | 8 字节 (head/tail + 指针) |
| `MrLink_t` (32 槽 + frame_buf) | ~456 字节 (含 70B typed 内部 buffer) |
| `mrlink::Instance` (16 typed slot) | ~1024 字节 (含 std::function 开销) |

## 最小示例 — C 风格

```c
#include "mrlink/mrlink.h"

static uint8_t s_rx[256], s_tx[80];
static MrLink_t s_link;

static int8_t OnHeartbeat(uint8_t cmd, const uint8_t* pl, uint16_t len, void* ctx) {
    (void)cmd; (void)pl; (void)len; (void)ctx;
    // ... 业务 (需要自己 memcpy payload)
    return MRLINK_OK;
}

void MyDev_Init(void) {
    MrLink_Config_t cfg = { .max_payload_size = 64, .use_crc16 = true };
    MrLink_Init(&s_link, &cfg, s_rx, sizeof(s_rx), s_tx, sizeof(s_tx));
    MrLink_RegisterHandler(&s_link, 0x01, OnHeartbeat, NULL);
}

void USARTx_IRQHandler_RX(void) {
    if (RX_done) MrLink_FeedBytes(&s_link, rx_buf, rx_len);
}

void SendHeartbeat(void) {
    uint16_t n = MrLink_Build(&s_link, 0x01, NULL, 0);
    if (n > 0u) BSP_UART_Transmit(BSP_UART_PC, s_tx, n, true);
}
```

## 最小示例 — C 风格 + typed handler

C 库提供 `MrLink_RegisterTypedHandler`，库内自动做 length check + memcpy 到内部 buffer，handler 拿到 `const void*`（业务 cast 成自己的类型）：

```c
#include "mrlink/mrlink.h"

typedef struct { float vx, vy, wz; } ChassisCmd_t;   // 12 字节

static int8_t OnChassisCmd(uint8_t cmd, const void* data,
                            uint16_t size, void* ctx) {
    (void)cmd; (void)ctx;
    if (size != sizeof(ChassisCmd_t)) return MRLINK_ERR_ARGS;
    const ChassisCmd_t* c = (const ChassisCmd_t*)data;
    Chassis_ApplyRemoteCmd(c);
    return MRLINK_OK;
}

void MyDev_Init(void) {
    // ... 同上
    MrLink_RegisterTypedHandler(&s_link, 0x10,
                                 sizeof(ChassisCmd_t),
                                 OnChassisCmd, NULL);
}
```

**对比**：

| 维度 | C 风格 (`MrLink_RegisterHandler`) | C typed 风格 (`MrLink_RegisterTypedHandler`) |
|---|---|---|
| handler 签名 | `(cmd, pl, len, ctx)` | `(cmd, data, size, ctx)` |
| length check | 业务手写 | **库自动** |
| memcpy | 业务手写 | **库自动** (到 frame_buf) |
| handler 拿到的 | `const uint8_t*` | `const void*` (typed by convention) |
| 加新命令代码 | 6-7 行 | **3-4 行**（仅 cast + dispatch）|

## 最小示例 — C++ OneMessage 风格

```cpp
#include "mrlink/mrlink.hpp"

static mrlink::Instance s_link;

void MyDev_Init() {
    MrLink_Config_t cfg = { .max_payload_size = 64, .use_crc16 = true };
    s_link.Init(&cfg, s_rx, sizeof(s_rx), s_tx, sizeof(s_tx));

    // ★ 1 行 1 命令, 0 行 boilerplate
    s_link.On<Heartbeat_t>(0x01, [](const Heartbeat_t&) {
        // 业务, 拿到的是 typed 引用
    });
    s_link.On<ChassisCmd_t>(0x10, [](const ChassisCmd_t& c) {
        // 自动 length check + memcpy
    });
}
```

## 集成到 STM32 工程

把以下源文件加入 `CMakeLists.txt`（或 `.mxproject`）：

```cmake
target_sources(${PROJECT_NAME} PRIVATE
    User/device/mrlink/mrlink_ringbuf.c
    User/device/mrlink/mrlink.c
)
```

链接时确保以下头文件路径在 include 列表：

```
User/device
User/device/mrlink
User/component     # crc16.h
Drivers/CMSIS/Include   # cmsis_compiler.h
```

## 适用场景 vs 不适用

**适用**：
- 跨 MCU 通信（板内 / 板间）
- UART / SPI / FDCAN / TCP 字节流
- 需要 5~30 种命令的小型总线

**不适用**：
- 大量 topic (>30)：用 OneMessage 原版（topic ID 是 2B）
- 主从/请求-响应时序：库**不**管，调用方自己实现
- 经典 CAN 8 字节：FDCAN (64 字节) 已普及
