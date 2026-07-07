#pragma once

#ifdef __cplusplus
extern "C" {
#endif  

#include <stdbool.h>
#include <stdint.h>

#define LATEST_SLOT_OK       (0)
#define LATEST_SLOT_ERR      (-1)
#define LATEST_SLOT_ERR_NULL (-2)
#define LATEST_SLOT_ERR_ARGS (-3)

typedef struct {
  void *storage;
  uint16_t size;
  volatile uint32_t seq;
  volatile bool valid;
} LatestSlot_t;

int8_t LatestSlot_Init(LatestSlot_t *slot, void *storage, uint16_t size);
int8_t LatestSlot_Write(LatestSlot_t *slot, const void *value, uint16_t size);
bool LatestSlot_ReadIfUpdated(const LatestSlot_t *slot, void *out,
                              uint16_t size, uint32_t *last_seq);
bool LatestSlot_ReadLatest(const LatestSlot_t *slot, void *out, uint16_t size);
void LatestSlot_Reset(LatestSlot_t *slot);

#ifdef __cplusplus
}
#endif