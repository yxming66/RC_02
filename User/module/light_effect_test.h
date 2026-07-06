#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  LIGHT_EFFECT_TEST_CMD_NONE = 0u,
  LIGHT_EFFECT_TEST_CMD_SEND = 1u,
} LightEffectTest_Command_t;

typedef struct {
  volatile uint8_t command;
  volatile uint8_t mode;
  volatile uint8_t last_sent_mode;
  volatile int8_t last_result;
  volatile uint32_t tx_count;
  volatile uint32_t error_count;
  volatile uint32_t last_tx_tick_ms;
  volatile uint32_t can_bus;
  volatile uint32_t can_id;
  volatile uint8_t dlc;
} LightEffectTest_Debug_t;

extern volatile LightEffectTest_Debug_t g_light_effect_test_debug;

void LightEffectTest_Init(void);
void LightEffectTest_Step(uint32_t now_ms);

#ifdef __cplusplus
}
#endif