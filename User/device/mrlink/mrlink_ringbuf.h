/**
  ******************************************************************************
  * @file    mrlink_ringbuf.h
  * @brief   通用单生产者-单消费者 (SPSC) 字节环形缓冲 / Generic SPSC byte ring
  *          buffer.
  *
  * 设计原则 / Design principles:
  *   - 通用字节缓冲，不耦合任何通信协议或物理层
  *   - SPSC 无锁：单写者（ISR / 任务）+ 单读者（任务）
  *   - 静态实例化（与项目 PC_Comm_t 风格一致），内存由调用方提供
  *   - 容量 = size - 1（保留 1 字节区分 full / empty）
  *
  * 并发约定 / Concurrency contract:
  *   - Put: 可在 ISR 或任务上下文调用
  *   - Get / Peek / Size / Space: 任务上下文调用
  *   - 单核 Cortex-M 上 Put 与 Get 互斥通过关中断实现
  *
  * Quick start:
  *   ```c
  *   #include "mrlink/mrlink_ringbuf.h"
  *
  *   static uint8_t          s_buf[256];
  *   static MrLink_RingBuf_t s_rb;
  *
  *   MrLink_RingBuf_Init(&s_rb, s_buf, sizeof(s_buf));
  *
  *   MrLink_RingBuf_Put(&s_rb, data, len);                        // ISR
  *   MrLink_RingBuf_Get(&s_rb, dst, sizeof(dst));                  // 任务
  *
  *   if (MrLink_RingBuf_IsEmpty(&s_rb)) { }
  *   uint16_t used  = MrLink_RingBuf_Size(&s_rb);
  *   uint16_t space = MrLink_RingBuf_Space(&s_rb);
  *   ```
  ******************************************************************************
  */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>

/* Exported constants ------------------------------------------------------- */

/* Return codes / 错误码 */
#define MRLINK_RINGBUF_OK          (0)
#define MRLINK_RINGBUF_ERR         (-1)
#define MRLINK_RINGBUF_ERR_NULL    (-2)   /**< 空指针参数 / Null pointer arg */
#define MRLINK_RINGBUF_ERR_ARGS    (-3)   /**< 参数非法 (如 size = 0) / Invalid arg */

/* Exported types ----------------------------------------------------------- */

/**
 * @brief SPSC 字节环形缓冲控制块 / SPSC byte ring buffer control block
 *
 * 字段由调用方拥有，库不分配内存。建议作为 static / 全局变量
 * （生命周期 = 程序生命周期），库内只读不改 storage。
 */
typedef struct {
  uint8_t *buf;            /**< 外部提供的存储 / External storage */
  uint16_t size;           /**< 存储总字节数 (建议 2 的幂) / Total capacity */
  volatile uint16_t head;  /**< 写指针 (生产者更新) / Write index */
  volatile uint16_t tail;  /**< 读指针 (消费者更新) / Read index */
} MrLink_RingBuf_t;

/* Exported function prototypes --------------------------------------------- */

/**
 * @brief  初始化环形缓冲 / Initialize ring buffer.
 * @param  rb       非 NULL 指针 / Non-NULL pointer
 * @param  storage  外部提供的存储区，长度 = size 字节 / External storage
 * @param  size     存储字节数，> 0 / Storage size, must be > 0
 * @return MRLINK_RINGBUF_OK / ERR_NULL / ERR_ARGS
 */
int8_t MrLink_RingBuf_Init(MrLink_RingBuf_t *rb, uint8_t *storage, uint16_t size);

/**
 * @brief  写入数据 / Write bytes into the buffer.
 *
 * ISR-safe. 当剩余空间不足时，**截断写入**（返回实际写入字节数），
 * 不会阻塞、不会回卷覆盖已有数据。
 *
 * @param  rb    非 NULL / Non-NULL
 * @param  data  源数据，data != NULL（len = 0 时可为 NULL）/ Source data
 * @param  len   写入字节数 / Number of bytes to write
 * @return 实际写入字节数；入参非法返回 0 / Bytes actually written
 */
uint16_t MrLink_RingBuf_Put(MrLink_RingBuf_t *rb, const uint8_t *data, uint16_t len);

/**
 * @brief  读出数据并前移读指针 / Read bytes and advance tail.
 *
 * 任务上下文调用。当缓冲内数据不足时，返回实际读到的字节数。
 *
 * @param  rb   非 NULL / Non-NULL
 * @param  dst  目标缓冲，dst != NULL（len = 0 时可为 NULL）/ Destination
 * @param  len  期望读取字节数 / Max bytes to read
 * @return 实际读取字节数
 */
uint16_t MrLink_RingBuf_Get(MrLink_RingBuf_t *rb, uint8_t *dst, uint16_t len);

/**
 * @brief  复制数据但不前移读指针 / Peek bytes without advancing tail.
 *
 * @param  rb   非 NULL / Non-NULL
 * @param  dst  目标缓冲 / Destination
 * @param  len  期望复制字节数 / Max bytes to copy
 * @return 实际复制字节数
 */
uint16_t MrLink_RingBuf_Peek(const MrLink_RingBuf_t *rb, uint8_t *dst, uint16_t len);

/**
 * @brief  返回已存数据字节数 / Return number of bytes currently stored.
 */
uint16_t MrLink_RingBuf_Size(const MrLink_RingBuf_t *rb);

/**
 * @brief  返回剩余可写字节数 / Return number of bytes that can be written.
 *
 * 注意：因为保留 1 字节区分 full / empty，
 * 实际可写入 = Space = size - 1 - Size.
 */
uint16_t MrLink_RingBuf_Space(const MrLink_RingBuf_t *rb);

/**
 * @brief  判空 / Test if buffer is empty.
 */
bool MrLink_RingBuf_IsEmpty(const MrLink_RingBuf_t *rb);

/**
 * @brief  重置（丢弃所有已存数据）/ Reset (discard all data).
 *
 * 调用方需保证此时没有并发的 Put 在执行。
 */
void MrLink_RingBuf_Reset(MrLink_RingBuf_t *rb);

#ifdef __cplusplus
}
#endif
