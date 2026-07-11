#include "device/photo_transfer.h"

#include <string.h>

#define PHOTO_TRANSFER_CAN_DATA_LEN (2u)

static bool photo_transfer_inited = false;
static bool photo_transfer_can_available = false;
static bool photo_transfer_received = false;
static PhotoTransfer_Snapshot_t photo_transfer_snapshot = {0};

static uint16_t PhotoTransfer_ParseMask(const uint8_t *data) {
  return (uint16_t)(((uint16_t)data[0] |
                     (((uint16_t)data[1] & 0x7Fu) << 8)) &
                    PHOTO_TRANSFER_INPUT_MASK);
}

static void PhotoTransfer_RefreshAgeAndValidity(uint32_t now_ms) {
  if (!photo_transfer_received) {
    photo_transfer_snapshot.age_ms = now_ms;
    photo_transfer_snapshot.valid = false;
    return;
  }

  photo_transfer_snapshot.age_ms =
      now_ms - photo_transfer_snapshot.last_update_ms;
  if (photo_transfer_snapshot.age_ms > PHOTO_TRANSFER_TIMEOUT_MS) {
    if (photo_transfer_snapshot.valid) {
      photo_transfer_snapshot.timeout_count++;
    }
    photo_transfer_snapshot.valid = false;
  }
}

int8_t PhotoTransfer_Init(void) {
  if (photo_transfer_inited) {
    return DEVICE_ERR_INITED;
  }

  memset(&photo_transfer_snapshot, 0, sizeof(photo_transfer_snapshot));
  photo_transfer_received = false;

  const int8_t can_init_ret = BSP_CAN_Init();
  if (can_init_ret != BSP_OK && can_init_ret != BSP_ERR_INITED) {
    return DEVICE_ERR;
  }

  const int8_t reg_ret =
      BSP_CAN_RegisterLatestId(PHOTO_TRANSFER_CAN_BUS, PHOTO_TRANSFER_CAN_ID);
  if (reg_ret != BSP_OK) {
    return DEVICE_ERR;
  }

  photo_transfer_can_available = true;
  photo_transfer_inited = true;
  return DEVICE_OK;
}

void PhotoTransfer_Update(uint32_t now_ms) {
  if (!photo_transfer_inited || !photo_transfer_can_available) {
    PhotoTransfer_RefreshAgeAndValidity(now_ms);
    return;
  }

  BSP_CAN_Message_t msg = {0};
  if (BSP_CAN_GetLatestMessage(PHOTO_TRANSFER_CAN_BUS, PHOTO_TRANSFER_CAN_ID,
                               &msg) == BSP_OK) {
    if (msg.dlc >= PHOTO_TRANSFER_CAN_DATA_LEN) {
      const uint16_t raw_mask = PhotoTransfer_ParseMask(msg.data);
      if (!photo_transfer_received ||
          msg.timestamp != photo_transfer_snapshot.last_update_ms ||
          raw_mask != photo_transfer_snapshot.raw_mask) {
        /* BSP_CAN_GetLatestMessage may keep returning the cached last frame.
         * Do not treat that stale frame as a reconnect after timeout; doing
         * so makes rx_count/timeout_count increase on every task cycle. */
        photo_transfer_snapshot.raw_mask = raw_mask;
        photo_transfer_snapshot.valid = true;
        photo_transfer_snapshot.last_update_ms = msg.timestamp;
        photo_transfer_snapshot.age_ms = now_ms - msg.timestamp;
        photo_transfer_snapshot.rx_count++;
        photo_transfer_received = true;
      }
    }
  }

  PhotoTransfer_RefreshAgeAndValidity(now_ms);
}

PhotoTransfer_Snapshot_t PhotoTransfer_GetSnapshot(uint32_t now_ms) {
  PhotoTransfer_RefreshAgeAndValidity(now_ms);
  return photo_transfer_snapshot;
}

bool PhotoTransfer_IsBitTriggered(const PhotoTransfer_Snapshot_t *snapshot,
                                  PhotoTransfer_Bit_t bit) {
  if (snapshot == NULL || !snapshot->valid || bit > 14u) {
    return false;
  }

  return (snapshot->raw_mask & (uint16_t)(1u << (uint8_t)bit)) != 0u;
}
