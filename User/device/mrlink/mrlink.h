/**
  ******************************************************************************
  * @file    mrlink.h
  * @brief   通用 byte-stream frame 协议 / Generic byte-stream frame protocol.
  *
  * 库职责 (数据链路层):
  *   - 字节流 → 帧解析 (找帧头 / 长度校验 / CRC 校验)
  *   - 帧 → 字节流构建 (加帧头 / 长度 / CRC)
  *   - 内部维护 RX 流缓冲
  *
  * 库**不管**:
  *   - 主从角色 / 设备 ID / 命令字语义 / 物理层 / 业务 payload / 心跳
  *
  * 帧格式:
  *   use_crc16 = true:  [0x4D][0x52][len][cmd][payload...][CRC16_lo][CRC16_hi]
  *   use_crc16 = false: [0x4D][0x52][len][cmd][payload...]
  *   - header  写死 {0x4D, 0x52} = ASCII "MR"
  *   - length  payload 字节数 (0~max_payload_size)
  *   - cmd     单字节，库不解释
  *   - CRC16   CCITT-FALSE, init=0xFFFF, poly=0x1021, 小端写
  *
  * Quick start:
  *   ```c
  *   #include "mrlink/mrlink.h"
  *
  *   static uint8_t  s_rx[256], s_tx[80];
  *   static MrLink_t s_link;
  *
  *   MrLink_Config_t cfg = { .max_payload_size = 64, .use_crc16 = true };
  *   MrLink_Init(&s_link, &cfg, s_rx, sizeof(s_rx), s_tx, sizeof(s_tx));
  *   MrLink_RegisterHandler(&s_link, 0x10, OnX, NULL);
  *
  *   MrLink_FeedBytes(&s_link, rx_buf, Size);                       // ISR
  *   uint16_t n = MrLink_Build(&s_link, 0x10, pl, len);            // TX
  *   if (n > 0u) BSP_UART_Transmit(BSP_UART_X, s_tx, n, true);
  *   ```
  *
  * 想拿 1 行 1 命令 typed 体验, 用 C++ wrapper: #include "mrlink/mrlink.hpp".
  ******************************************************************************
  */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>

#include "mrlink/mrlink_ringbuf.h"

/* Exported constants ------------------------------------------------------- */

/* Return codes / 错误码 */
#define MRLINK_OK            (0)
#define MRLINK_ERR           (-1)   /**< 通用错误 (如无完整帧) */
#define MRLINK_ERR_NULL      (-2)   /**< 空指针 / Null pointer */
#define MRLINK_ERR_OVERSIZE  (-3)   /**< payload 超 max_payload_size */
#define MRLINK_ERR_CRC       (-4)   /**< CRC 校验失败 / CRC mismatch */
#define MRLINK_ERR_HEADER    (-5)   /**< 帧头不匹配 / Header mismatch */
#define MRLINK_ERR_ARGS      (-6)   /**< 参数非法 / Invalid args */
#define MRLINK_ERR_BUF_FULL  (-7)   /**< 内部流缓冲满 / Stream buf full */

/* Frame header (project identifier "MR") */
#define MRLINK_HEADER_0      (0x4D)  /**< 'M' */
#define MRLINK_HEADER_1      (0x52)  /**< 'R' */
#define MRLINK_HEADER_LEN    (2u)

/* Field lengths */
#define MRLINK_LEN_FIELD_LEN (1u)
#define MRLINK_CMD_FIELD_LEN (1u)
#define MRLINK_CRC_LEN       (2u)

/* Frame total length with CRC:    2 + 1 + 1 + payload + 2 = 6 + payload */
/* Frame total length without CRC: 2 + 1 + 1 + payload     = 4 + payload */

/* Defaults */
#define MRLINK_MAX_PAYLOAD_DEFAULT (64u)

/** Handler dispatch table size / handler 表大小.
 *  32 槽足够覆盖典型 5~15 种命令；超过返回 MRLINK_ERR。
 *  若需更大表，修改此宏并重新编译。 */
#define MRLINK_MAX_HANDLERS (32u)

/** 单帧最大长度 (header 2 + len 1 + cmd 1 + max_payload + crc 2)
 *  默认 max_payload=64, use_crc16=true 时 = 70. 内部 typed handler 缓冲
 *  按此大小分配。 */
#define MRLINK_MAX_FRAME_SIZE \
  (MRLINK_HEADER_LEN + MRLINK_LEN_FIELD_LEN + \
   MRLINK_CMD_FIELD_LEN + MRLINK_MAX_PAYLOAD_DEFAULT + \
   MRLINK_CRC_LEN)

/* Exported types ----------------------------------------------------------- */

/**
 * @brief 协议实例 (不透明) / Protocol instance (opaque).
 *
 * 调用方声明 static 变量，库内定义 layout。
 */
typedef struct MrLink MrLink_t;

/**
 * @brief 初始化配置 / Init config.
 */
typedef struct {
  uint16_t max_payload_size;     /**< 单帧 payload 上限，默认 64 */
  bool use_crc16;                /**< true=启用 CRC16-CCITT-FALSE；默认 true */
} MrLink_Config_t;

/**
 * @brief 运行统计 / Runtime stats.
 *
 * 所有字段在 IRQ 关 / 临界区外可读；写者只在库内临界区更新。
 */
typedef struct {
  uint32_t frame_rx_ok;            /**< 成功解析的帧数 */
  uint32_t frame_rx_crc_err;       /**< CRC 错 (use_crc16=true 时统计) */
  uint32_t frame_rx_oversize;      /**< length 字段超 max_payload_size */
  uint32_t frame_rx_header_skip;   /**< 跳过非帧头字节总数 */
  uint32_t frame_rx_truncated;     /**< 收不完整帧 (流缓冲不够) */
  uint32_t frame_rx_unknown_cmd;   /**< 收到但无 handler 的 cmd 帧数 */
  uint32_t frame_rx_size_mismatch; /**< typed handler 期望长度不匹配 (comm_rx_size_mismatch) */
  uint32_t frame_tx_ok;            /**< 成功 build 的帧数 */
  uint32_t feed_bytes_total;       /**< FeedBytes 喂入字节总数 */
  uint32_t feed_calls;             /**< FeedBytes 调用次数 */
} MrLink_Stats_t;

/**
 * @brief 帧回调 / Frame handler callback.
 *
 * 每次成功解析出一帧 (FeedBytes 内部或 Parse 触发) 时调用一次。
 * 回调内应**短小可重入**，不要做长阻塞。
 *
 * @param cmd          帧命令字
 * @param payload      指向库内临时 payload 缓冲 (本次调用内有效)
 * @param payload_len  payload 字节数
 * @param ctx          RegisterHandler 时传入的上下文
 */
typedef int8_t (*MrLink_FrameHandler_t)(uint8_t cmd,
                                            const uint8_t *payload,
                                            uint16_t payload_len,
                                            void *ctx);

/**
 * @brief  typed handler 回调签名 / Typed handler callback signature.
 *
 * 与 raw handler 的区别: typed handler 拿到的是**已校验长度 + 已 memcpy 到库内稳定缓冲**
 * 的 typed 指针, 业务侧无需 length check / memcpy, 直接 cast 即可。
 * 指针仅本次回调有效。
 *
 * 适用于"payload 是单一 POD 类型"的命令, 业务侧代码量比 raw handler 少约一半。
 *
 * @param cmd          帧命令字
 * @param typed_data   指向库内稳定 buffer (本次调用内有效, 已 len 校验)
 * @param typed_size   typed_data 实际字节数 (= 注册时的 expected_size)
 * @param ctx          RegisterTypedHandler 时传入的上下文
 */
typedef int8_t (*MrLink_TypedHandler_t)(uint8_t cmd,
                                            const void *typed_data,
                                            uint16_t typed_size,
                                            void *ctx);

/**
 * @brief cmd → handler 映射条目 / Handler dispatch entry.
 */
typedef struct {
  bool used;
  uint8_t cmd;
  uint16_t expected_size;   /**< 0 = raw handler, >0 = typed */
  union {
    MrLink_FrameHandler_t raw;
    MrLink_TypedHandler_t typed;
  } handler;
  void *ctx;
} MrLink_HandlerEntry_t;

/**
 * @brief 协议实例存储 / Protocol instance storage.
 *
 * 字段由库内部维护，应用侧只应通过 MrLink_* API 访问。
 */
struct MrLink {
  MrLink_Config_t cfg;
  MrLink_RingBuf_t rx_rb;
  uint8_t *rx_buf;
  uint16_t rx_buf_size;
  uint8_t *tx_buf;
  uint16_t tx_buf_size;
  uint8_t frame_buf[MRLINK_MAX_FRAME_SIZE];
  MrLink_HandlerEntry_t handlers[MRLINK_MAX_HANDLERS];
  MrLink_Stats_t stats;
};

/* Exported function prototypes --------------------------------------------- */

/**
 * @brief  初始化协议实例 / Initialize protocol instance.
 *
 * @param proto        非 NULL / Non-NULL
 * @param cfg          NULL 则用默认 (max=64, crc=true)
 * @param rx_buf       外部 RX 流缓冲 (由 mrlink_ringbuf 使用)，非 NULL
 * @param rx_buf_size  ≥ 4 (起码能装一帧最小)
 * @param tx_buf       外部 TX 缓冲 (Build 写入)，非 NULL
 * @param tx_buf_size  ≥ 4 + max_payload_size (无 CRC) 或 ≥ 6 + max_payload_size (有 CRC)
 * @return MRLINK_OK / ERR_NULL / ERR_ARGS
 */
int8_t MrLink_Init(MrLink_t *proto,
                      const MrLink_Config_t *cfg,
                      uint8_t *rx_buf, uint16_t rx_buf_size,
                      uint8_t *tx_buf, uint16_t tx_buf_size);

/**
 * @brief  重置 (清空 RX 缓冲、清零 stats、清除 handler) / Reset.
 *
 * 调用方需保证此时没有并发的 FeedBytes 在执行。
 */
void MrLink_Reset(MrLink_t *proto);

/**
 * @brief  喂入接收到的字节 / Feed received bytes.
 *
 * 内部依次: 写入 ring buffer → 扫描帧 → 若注册 handler 则触发回调。
 *
 * ISR / 任务上下文均可调用。
 *
 * @param proto 非 NULL / Non-NULL
 * @param data  源数据 (len=0 时可为 NULL)
 * @param len   字节数
 * @return MRLINK_OK
 */
int8_t MrLink_FeedBytes(MrLink_t *proto,
                            const uint8_t *data, uint16_t len);

/**
 * @brief  拉取一帧 / Pull one frame from the RX stream.
 *
 * 任务上下文调用。无完整帧时返回 MRLINK_ERR。
 * payload 指针指向库内临时缓冲，**仅本次调用有效**；调用方需在下次调用前消费。
 *
 * 注册了 handler 时，handler 已在 FeedBytes 时被触发过；此处 Parse 仍能再拉一次。
 * 二者可同时存在。
 */
int8_t MrLink_Parse(MrLink_t *proto, uint8_t *out_cmd,
                       const uint8_t **out_payload,
                       uint16_t *out_payload_len);

/**
 * @brief  构造一帧 / Build one frame into tx_buf.
 *
 * 任务上下文调用。返回写入 tx_buf 的总字节数；失败返回 0。
 */
uint16_t MrLink_Build(MrLink_t *proto, uint8_t cmd,
                         const uint8_t *payload, uint16_t payload_len);

/**
 * @brief  注册 cmd → handler 映射 / Register handler for a specific cmd.
 *
 * 同一 cmd 重复注册会**覆盖**旧的 handler+ctx。
 * 传 handler = NULL 表示**注销**该 cmd。
 *
 * 收到匹配 cmd 的帧时, 库内部自动调用对应 handler（不必用户写 switch/case）。
 * 无 handler 的 cmd 帧计入 stats.frame_rx_unknown_cmd, 不报错。
 *
 * @param proto   非 NULL / Non-NULL
 * @param cmd     要订阅的命令字 (0~255)
 * @param handler 非 NULL 注册；NULL 注销
 * @param ctx     透传给 handler 的上下文 / Opaque ctx passed to handler
 * @return MRLINK_OK / ERR_NULL / ERR_ARGS (handler 表满)
 */
int8_t MrLink_RegisterHandler(MrLink_t *proto, uint8_t cmd,
                                 MrLink_FrameHandler_t handler,
                                 void *ctx);

/**
 * @brief  注册 typed handler / Register typed handler for a specific cmd.
 *
 * 与 RegisterHandler 的区别:
 *   - 库自动校验 len == expected_size (不等计入 frame_rx_size_mismatch)
 *   - 库自动 memcpy payload 到内部 frame_buf (业务不再 memcpy)
 *   - 业务 handler 拿到的 const void* 已稳定, 直接 cast 为具体类型
 *
 * 注册时 handler 接受:
 *   - handler != NULL + ctx != NULL: 注册 typed handler
 *   - handler == NULL: 注销该 cmd
 *   - 同 cmd 重复注册: 覆盖 (raw → typed, typed → typed 都行)
 *   - 32 槽满: 返回 MRLINK_ERR_ARGS
 *
 * @param expected_size  payload 期望长度 (如 sizeof(ChassisCmd_t))
 * @param handler        业务回调, 形如 OnXxx(cmd, const T*, sz, ctx)
 */
int8_t MrLink_RegisterTypedHandler(MrLink_t *proto, uint8_t cmd,
                                       uint16_t expected_size,
                                       MrLink_TypedHandler_t handler,
                                       void *ctx);

/**
 * @brief  获取运行统计 / Get runtime stats.
 */
const MrLink_Stats_t *MrLink_GetStats(const MrLink_t *proto);

#ifdef __cplusplus
}
#endif
