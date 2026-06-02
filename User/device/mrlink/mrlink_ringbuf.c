/**
  ******************************************************************************
  * @file    mrlink_ringbuf.c
  * @brief   通用 SPSC 字节环形缓冲实现 / Generic SPSC byte ring buffer impl.
  *
  * 容量 = size - 1（保留 1 字节区分 full / empty）。
  * 写 (Put) 和读 (Get/Peek) 走 PRIMASK 临界区互斥。
  ******************************************************************************
  */

#include "mrlink/mrlink_ringbuf.h"

#include <string.h>

#include "cmsis_compiler.h"   /**< __disable_irq / __enable_irq / __get_PRIMASK */

/* Exported functions ------------------------------------------------------- */

/**
 * @brief 初始化 ring buffer.
 *
 * @details 控制律 / Control law (3 步):
 *   1. 指针非空检查 (rb / storage)
 *   2. 容量检查: size >= 2 (否则 empty/full 不可分)
 *   3. head = tail = 0
 * 实际可用容量 = size - 1 (留 1 字节区分空/满)
 */
int8_t MrLink_RingBuf_Init(MrLink_RingBuf_t *rb, uint8_t *storage, uint16_t size) {
  if (rb == NULL || storage == NULL) {
    return MRLINK_RINGBUF_ERR_NULL;
  }
  if (size < 2u) {
    return MRLINK_RINGBUF_ERR_ARGS;
  }

  rb->buf = storage;
  rb->size = size;
  rb->head = 0u;
  rb->tail = 0u;
  return MRLINK_RINGBUF_OK;
}

/**
 * @brief 写入字节 (生产者, ISR/任务).
 *
 * @details 控制律 / Control law (SPSC 写, PRIMASK 临界区):
 *   1. 临界区关中断
 *   2. 算 used / space
 *   3. 截断 len = min(len, space) (满了不覆盖, 截断返回)
 *   4. 两段 memcpy: head → end-of-buf, 然后 0 → len
 *   5. head = (head + len) % size
 *   6. 临界区开中断
 * ISR/任务上下文均可调用 (临界区保护)
 * 返回: 实际写入字节数 (0 = 满了)
 */
uint16_t MrLink_RingBuf_Put(MrLink_RingBuf_t *rb, const uint8_t *data, uint16_t len) {
  if (rb == NULL || data == NULL || len == 0u) {
    return 0u;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  const uint16_t size = rb->size;
  const uint16_t head = rb->head;
  const uint16_t tail = rb->tail;
  const uint16_t used = (head >= tail) ? (uint16_t)(head - tail)
                                       : (uint16_t)(size - tail + head);
  uint16_t space = (uint16_t)(size - 1u - used);

  if (len > space) {
    len = space;
  }
  if (len == 0u) {
    if (primask == 0u) {
      __enable_irq();
    }
    return 0u;
  }

  /* Two-chunk copy: head → end-of-buf, then 0 → ... */
  const uint16_t first_chunk = (head + len > size) ? (uint16_t)(size - head)
                                                   : len;
  memcpy(&rb->buf[head], data, first_chunk);

  const uint16_t second_chunk = (uint16_t)(len - first_chunk);
  if (second_chunk > 0u) {
    memcpy(&rb->buf[0], &data[first_chunk], second_chunk);
  }

  uint16_t new_head = (uint16_t)(head + len);
  if (new_head >= size) {
    new_head = (uint16_t)(new_head - size);
  }
  rb->head = new_head;

  if (primask == 0u) {
    __enable_irq();
  }
  return len;
}

/**
 * @brief 读取并消费字节 (消费者, 任务上下文).
 *
 * @details 控制律 / Control law (SPSC 读, 推进 tail):
 *   1. 算 used
 *   2. 截断 len = min(len, used)
 *   3. 两段 memcpy: tail → end, 0 → len
 *   4. tail = (tail + len) % size
 * 与 Peek 区别: Get 推进 tail (消费), Peek 不动
 */
uint16_t MrLink_RingBuf_Get(MrLink_RingBuf_t *rb, uint8_t *dst, uint16_t len) {
  if (rb == NULL || dst == NULL || len == 0u) {
    return 0u;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  const uint16_t size = rb->size;
  const uint16_t head = rb->head;
  const uint16_t tail = rb->tail;
  const uint16_t used = (head >= tail) ? (uint16_t)(head - tail)
                                       : (uint16_t)(size - tail + head);

  if (len > used) {
    len = used;
  }
  if (len == 0u) {
    if (primask == 0u) {
      __enable_irq();
    }
    return 0u;
  }

  const uint16_t first_chunk = (tail + len > size) ? (uint16_t)(size - tail)
                                                   : len;
  memcpy(dst, &rb->buf[tail], first_chunk);

  const uint16_t second_chunk = (uint16_t)(len - first_chunk);
  if (second_chunk > 0u) {
    memcpy(&dst[first_chunk], &rb->buf[0], second_chunk);
  }

  uint16_t new_tail = (uint16_t)(tail + len);
  if (new_tail >= size) {
    new_tail = (uint16_t)(new_tail - size);
  }
  rb->tail = new_tail;

  if (primask == 0u) {
    __enable_irq();
  }
  return len;
}

/**
 * @brief 偷看字节 (不消费).
 *
 * @details 控制律 / Control law (SPSC 读, 不动 tail):
 *   1. 算 used
 *   2. 截断 len = min(len, used)
 *   3. 两段 memcpy 到 dst (不修改 tail)
 * 用途: parser 偷看 rx 头部找帧头 (不消费, 失败时还能回到原位)
 */
uint16_t MrLink_RingBuf_Peek(const MrLink_RingBuf_t *rb, uint8_t *dst, uint16_t len) {
  if (rb == NULL || dst == NULL || len == 0u) {
    return 0u;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  const uint16_t size = rb->size;
  const uint16_t head = rb->head;
  const uint16_t tail = rb->tail;
  const uint16_t used = (head >= tail) ? (uint16_t)(head - tail)
                                       : (uint16_t)(size - tail + head);

  if (len > used) {
    len = used;
  }
  if (len == 0u) {
    if (primask == 0u) {
      __enable_irq();
    }
    return 0u;
  }

  const uint16_t first_chunk = (tail + len > size) ? (uint16_t)(size - tail)
                                                   : len;
  memcpy(dst, &rb->buf[tail], first_chunk);

  const uint16_t second_chunk = (uint16_t)(len - first_chunk);
  if (second_chunk > 0u) {
    memcpy(&dst[first_chunk], &rb->buf[0], second_chunk);
  }

  if (primask == 0u) {
    __enable_irq();
  }
  return len;
}

/**
 * @brief 已存字节数.
 *
 * @details 控制律 / Control law (1 步):
 *   used = (head >= tail) ? head - tail : size - tail + head
 * 无锁 (SPSC 约定, head/tail 是 16 位单字写, 原子)
 */
uint16_t MrLink_RingBuf_Size(const MrLink_RingBuf_t *rb) {
  if (rb == NULL) {
    return 0u;
  }
  const uint16_t head = rb->head;
  const uint16_t tail = rb->tail;
  return (head >= tail) ? (uint16_t)(head - tail)
                        : (uint16_t)(rb->size - tail + head);
}

/**
 * @brief 剩余可写字节数.
 *
 * @details 控制律 / Control law (1 步):
 *   space = (size - 1) - Size(rb)
 * 等价于 Put 不截断时可写的最大长度
 */
uint16_t MrLink_RingBuf_Space(const MrLink_RingBuf_t *rb) {
  if (rb == NULL) {
    return 0u;
  }
  return (uint16_t)(rb->size - 1u - MrLink_RingBuf_Size(rb));
}

/**
 * @brief 是否空.
 *
 * @details 控制律 / Control law (1 步):
 *   return (head == tail)
 * 满判定: (head + 1) % size == tail
 */
bool MrLink_RingBuf_IsEmpty(const MrLink_RingBuf_t *rb) {
  if (rb == NULL) {
    return true;
  }
  return rb->head == rb->tail;
}

/**
 * @brief 重置 (清空所有数据).
 *
 * @details 控制律 / Control law (1 步, 临界区):
 *   PRIMASK 关中断:
 *     head = tail = 0
 *   开中断
 * 注意: 不清 storage 内容, 只重置指针 (旧数据残留但不可见)
 */
void MrLink_RingBuf_Reset(MrLink_RingBuf_t *rb) {
  if (rb == NULL) {
    return;
  }
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  rb->head = 0u;
  rb->tail = 0u;
  if (primask == 0u) {
    __enable_irq();
  }
}
