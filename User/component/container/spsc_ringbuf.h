/**
 ******************************************************************************
 * @file    spsc_ringbuf.h
 * @brief   通用单生产者-单消费者字节环形缓冲 / Generic SPSC byte ring buffer.
 ******************************************************************************
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define SPSC_RINGBUF_OK       (0)
#define SPSC_RINGBUF_ERR      (-1)
#define SPSC_RINGBUF_ERR_NULL (-2)
#define SPSC_RINGBUF_ERR_ARGS (-3)

typedef struct {
  uint8_t *buf;
  uint16_t size;
  volatile uint16_t head;
  volatile uint16_t tail;
} SpscRingBuf_t;

int8_t SpscRingBuf_Init(SpscRingBuf_t *rb, uint8_t *storage, uint16_t size);
uint16_t SpscRingBuf_Put(SpscRingBuf_t *rb, const uint8_t *data, uint16_t len);
uint16_t SpscRingBuf_Get(SpscRingBuf_t *rb, uint8_t *dst, uint16_t len);
uint16_t SpscRingBuf_Drop(SpscRingBuf_t *rb, uint16_t len);
uint16_t SpscRingBuf_Peek(const SpscRingBuf_t *rb, uint8_t *dst, uint16_t len);
uint16_t SpscRingBuf_Size(const SpscRingBuf_t *rb);
uint16_t SpscRingBuf_Space(const SpscRingBuf_t *rb);
bool SpscRingBuf_IsEmpty(const SpscRingBuf_t *rb);
void SpscRingBuf_Reset(SpscRingBuf_t *rb);

#ifdef __cplusplus
}
#endif