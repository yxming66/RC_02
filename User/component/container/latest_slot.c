#include "component/container/latest_slot.h"

#include <string.h>

#include "cmsis_compiler.h"

static uint32_t LatestSlot_EnterCritical(void) {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void LatestSlot_ExitCritical(uint32_t primask) {
  if (primask == 0u) {
    __enable_irq();
  }
}

static bool LatestSlot_IsValid(const LatestSlot_t *slot, uint16_t size) {
  return slot != NULL && slot->storage != NULL && slot->size == size &&
         size > 0u;
}

int8_t LatestSlot_Init(LatestSlot_t *slot, void *storage, uint16_t size) {
  if (slot == NULL || storage == NULL) {
    return LATEST_SLOT_ERR_NULL;
  }
  if (size == 0u) {
    return LATEST_SLOT_ERR_ARGS;
  }

  slot->storage = storage;
  slot->size = size;
  slot->seq = 0u;
  slot->valid = false;
  return LATEST_SLOT_OK;
}

int8_t LatestSlot_Write(LatestSlot_t *slot, const void *value, uint16_t size) {
  if (value == NULL) {
    return LATEST_SLOT_ERR_NULL;
  }
  if (!LatestSlot_IsValid(slot, size)) {
    return LATEST_SLOT_ERR_ARGS;
  }

  const uint32_t primask = LatestSlot_EnterCritical();
  memcpy(slot->storage, value, size);
  slot->seq++;
  if (slot->seq == 0u) {
    slot->seq = 1u;
  }
  slot->valid = true;
  LatestSlot_ExitCritical(primask);
  return LATEST_SLOT_OK;
}

bool LatestSlot_ReadIfUpdated(const LatestSlot_t *slot, void *out,
                              uint16_t size, uint32_t *last_seq) {
  if (out == NULL || last_seq == NULL || !LatestSlot_IsValid(slot, size)) {
    return false;
  }

  const uint32_t primask = LatestSlot_EnterCritical();
  const uint32_t seq = slot->seq;
  if (!slot->valid || seq == *last_seq) {
    LatestSlot_ExitCritical(primask);
    return false;
  }
  memcpy(out, slot->storage, size);
  *last_seq = seq;
  LatestSlot_ExitCritical(primask);
  return true;
}

bool LatestSlot_ReadLatest(const LatestSlot_t *slot, void *out, uint16_t size) {
  if (out == NULL || !LatestSlot_IsValid(slot, size)) {
    return false;
  }

  const uint32_t primask = LatestSlot_EnterCritical();
  if (!slot->valid) {
    LatestSlot_ExitCritical(primask);
    return false;
  }
  memcpy(out, slot->storage, size);
  LatestSlot_ExitCritical(primask);
  return true;
}

void LatestSlot_Reset(LatestSlot_t *slot) {
  if (slot == NULL || slot->storage == NULL || slot->size == 0u) {
    return;
  }

  const uint32_t primask = LatestSlot_EnterCritical();
  slot->seq = 0u;
  slot->valid = false;
  LatestSlot_ExitCritical(primask);
}