#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "bsp/can.h"
#include "device.h"

#ifndef LIGHT_EFFECT_CAN_BUS
#define LIGHT_EFFECT_CAN_BUS BSP_CAN_2
#endif

#ifndef LIGHT_EFFECT_CAN_ID
#define LIGHT_EFFECT_CAN_ID (0x322u)
#endif

#define LIGHT_EFFECT_MODE_MIN (0u)
#define LIGHT_EFFECT_MODE_MAX (32u)
#define LIGHT_EFFECT_MODE_NEXT_CMD (0xFFu)

typedef struct {
  uint8_t mode;
  uint8_t requested_mode;
  uint32_t rx_count;
  uint32_t valid_count;
  uint32_t invalid_count;
  uint32_t next_count;
  uint32_t short_dlc_count;
  uint32_t ignored_frame_count;
  uint32_t last_update_ms;
  uint32_t last_sequence;
  bool valid;
  bool can_available;
} LightEffectCan_Status_t;

int8_t LightEffectCan_Init(void);
void LightEffectCan_Update(uint32_t now_ms);
LightEffectCan_Status_t LightEffectCan_GetStatus(void);
uint8_t LightEffectCan_GetMode(void);

#ifdef __cplusplus
}
#endif