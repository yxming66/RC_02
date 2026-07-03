#include "device/light_effect_can.h"

#include <string.h>

static bool light_effect_can_inited = false;
static LightEffectCan_Status_t light_effect_can_status = {0};

static bool LightEffectCan_IsModeValid(uint8_t mode) {
  return mode >= LIGHT_EFFECT_MODE_MIN && mode <= LIGHT_EFFECT_MODE_MAX;
}

static void LightEffectCan_ApplyRequestedMode(uint8_t requested_mode,
                                             uint32_t now_ms,
                                             uint32_t sequence) {
  light_effect_can_status.requested_mode = requested_mode;
  light_effect_can_status.last_update_ms = now_ms;
  light_effect_can_status.last_sequence = sequence;

  if (requested_mode == LIGHT_EFFECT_MODE_NEXT_CMD) {
    light_effect_can_status.mode =
        (uint8_t)((light_effect_can_status.mode + 1u) %
                  (LIGHT_EFFECT_MODE_MAX + 1u));
    light_effect_can_status.valid = true;
    light_effect_can_status.valid_count++;
    light_effect_can_status.next_count++;
    return;
  }

  if (LightEffectCan_IsModeValid(requested_mode)) {
    light_effect_can_status.mode = requested_mode;
    light_effect_can_status.valid = true;
    light_effect_can_status.valid_count++;
    return;
  }

  light_effect_can_status.invalid_count++;
}

int8_t LightEffectCan_Init(void) {
  if (light_effect_can_inited) {
    return DEVICE_ERR_INITED;
  }

  memset(&light_effect_can_status, 0, sizeof(light_effect_can_status));
  light_effect_can_status.mode = 0u;
  light_effect_can_status.requested_mode = 0u;

  const int8_t can_init_ret = BSP_CAN_Init();
  if (can_init_ret != BSP_OK && can_init_ret != BSP_ERR_INITED) {
    return DEVICE_ERR;
  }

  const int8_t reg_ret =
      BSP_CAN_RegisterLatestId(LIGHT_EFFECT_CAN_BUS, LIGHT_EFFECT_CAN_ID);
  if (reg_ret != BSP_OK) {
    return DEVICE_ERR;
  }

  light_effect_can_status.can_available = true;
  light_effect_can_inited = true;
  return DEVICE_OK;
}

void LightEffectCan_Update(uint32_t now_ms) {
  if (!light_effect_can_inited || !light_effect_can_status.can_available) {
    return;
  }

  BSP_CAN_Message_t msg = {0};
  if (BSP_CAN_GetLatestMessage(LIGHT_EFFECT_CAN_BUS, LIGHT_EFFECT_CAN_ID,
                               &msg) != BSP_OK) {
    return;
  }

  if (msg.sequence == light_effect_can_status.last_sequence) {
    return;
  }

  light_effect_can_status.rx_count++;

  if (msg.frame_type != BSP_CAN_FRAME_STD_DATA || msg.dlc < 1u) {
    if (msg.dlc < 1u) {
      light_effect_can_status.short_dlc_count++;
    } else {
      light_effect_can_status.ignored_frame_count++;
    }
    light_effect_can_status.last_sequence = msg.sequence;
    return;
  }

  LightEffectCan_ApplyRequestedMode(msg.data[0], msg.timestamp, msg.sequence);
  if (light_effect_can_status.last_update_ms == 0u) {
    light_effect_can_status.last_update_ms = now_ms;
  }
}

LightEffectCan_Status_t LightEffectCan_GetStatus(void) {
  return light_effect_can_status;
}

uint8_t LightEffectCan_GetMode(void) {
  return light_effect_can_status.mode;
}