#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "bsp/can.h"
#include "device.h"

#ifndef PHOTO_TRANSFER_CAN_ID
#define PHOTO_TRANSFER_CAN_ID (0x321u)
#endif

#ifndef PHOTO_TRANSFER_CAN_BUS
#define PHOTO_TRANSFER_CAN_BUS BSP_CAN_2
#endif

#ifndef PHOTO_TRANSFER_TIMEOUT_MS
#define PHOTO_TRANSFER_TIMEOUT_MS (1200u)
#endif

#define PHOTO_TRANSFER_INPUT_MASK (0x7FFFu)

typedef enum {
  PHOTO_TRANSFER_BIT_PHOTO4_LAST = 0u,
  PHOTO_TRANSFER_BIT_PHOTO3_SECOND_LAST = 1u,
  PHOTO_TRANSFER_BIT_PHOTO2_THIRD_LAST = 2u,
  PHOTO_TRANSFER_BIT_PHOTO1_FRONT = 3u,
  PHOTO_TRANSFER_BIT_SPEARHEAD = 4u,
  PHOTO_TRANSFER_BIT_ORE_LOW = 5u,
  PHOTO_TRANSFER_BIT_ORE_HIGH = 6u,
  PHOTO_TRANSFER_BIT_ARM_ORE = 7u,
} PhotoTransfer_Bit_t;

typedef struct {
  uint16_t raw_mask;
  bool valid;
  uint32_t last_update_ms;
  uint32_t age_ms;
  uint32_t rx_count;
  uint32_t timeout_count;
} PhotoTransfer_Snapshot_t;

int8_t PhotoTransfer_Init(void);
void PhotoTransfer_Update(uint32_t now_ms);
PhotoTransfer_Snapshot_t PhotoTransfer_GetSnapshot(uint32_t now_ms);
bool PhotoTransfer_IsBitTriggered(const PhotoTransfer_Snapshot_t *snapshot,
                                  PhotoTransfer_Bit_t bit);

#ifdef __cplusplus
}
#endif
