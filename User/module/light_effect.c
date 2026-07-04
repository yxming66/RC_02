#include "module/light_effect.h"

#include "bsp/can.h"

#define LIGHT_EFFECT_CAN_BUS BSP_CAN_2
#define LIGHT_EFFECT_CAN_ID (0x322u)

int8_t LightEffect_SendMode(LightEffect_Mode_t mode) {
  BSP_CAN_StdDataFrame_t frame = {0};
  frame.id = LIGHT_EFFECT_CAN_ID;
  frame.dlc = 1u;
  frame.data[0] = (uint8_t)mode;

  const int8_t can_init_ret = BSP_CAN_Init();
  if (can_init_ret != BSP_OK && can_init_ret != BSP_ERR_INITED) {
    return can_init_ret;
  }

  return BSP_CAN_TransmitStdDataFrame(LIGHT_EFFECT_CAN_BUS, &frame);
}