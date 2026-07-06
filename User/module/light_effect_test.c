#include "module/light_effect_test.h"

#include "bsp/bsp.h"
#include "module/light_effect.h"

#define LIGHT_EFFECT_TEST_CAN_BUS (2u)
#define LIGHT_EFFECT_TEST_CAN_ID (0x322u)
#define LIGHT_EFFECT_TEST_DLC (1u)
#define LIGHT_EFFECT_TEST_DEFAULT_MODE (0x01u)

volatile LightEffectTest_Debug_t g_light_effect_test_debug = {
    .command = LIGHT_EFFECT_TEST_CMD_NONE,
    .mode = LIGHT_EFFECT_TEST_DEFAULT_MODE,
    .last_result = BSP_OK,
    .can_bus = LIGHT_EFFECT_TEST_CAN_BUS,
    .can_id = LIGHT_EFFECT_TEST_CAN_ID,
    .dlc = LIGHT_EFFECT_TEST_DLC,
};

static void LightEffectTest_Send(uint8_t mode, uint32_t now_ms) {
  const int8_t result = LightEffect_SendMode((LightEffect_Mode_t)mode);
  g_light_effect_test_debug.last_sent_mode = mode;
  g_light_effect_test_debug.last_result = result;
  g_light_effect_test_debug.last_tx_tick_ms = now_ms;
  if (result == BSP_OK) {
    g_light_effect_test_debug.tx_count++;
  } else {
    g_light_effect_test_debug.error_count++;
  }
}

void LightEffectTest_Init(void) {
  g_light_effect_test_debug.can_bus = LIGHT_EFFECT_TEST_CAN_BUS;
  g_light_effect_test_debug.can_id = LIGHT_EFFECT_TEST_CAN_ID;
  g_light_effect_test_debug.dlc = LIGHT_EFFECT_TEST_DLC;
}

void LightEffectTest_Step(uint32_t now_ms) {
  if (g_light_effect_test_debug.command != LIGHT_EFFECT_TEST_CMD_SEND) {
    return;
  }

  g_light_effect_test_debug.command = LIGHT_EFFECT_TEST_CMD_NONE;
  LightEffectTest_Send(g_light_effect_test_debug.mode, now_ms);
}