#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  LIGHT_EFFECT_MODE_OFF = 0x00u,
  LIGHT_EFFECT_MODE_PC_READY = 0x01u,
  LIGHT_EFFECT_MODE_RETRY = 0x02u,
  LIGHT_EFFECT_MODE_FAIL = 0x03u,
  LIGHT_EFFECT_MODE_RELEASE_LEVEL2 = 0x04u,
  LIGHT_EFFECT_MODE_RELEASE_LEVEL3 = 0x05u,
  LIGHT_EFFECT_MODE_RELEASE_LEVEL3_WAIT_LIFT = 0x02u,
} LightEffect_Mode_t;

int8_t LightEffect_SendMode(LightEffect_Mode_t mode);

#ifdef __cplusplus
}
#endif