#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "device/dr16.h"
#include "module/arm_simple.h"
#include "module/chassis.h"
#include "module/ore_store.h"
#include "module/pole.h"
#include "module/rod_new.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  Chassis_CMD_t *chassis;
  Pole_CMD_t *pole;
  ArmSimple_CMD_t *arm_simple;
  OreStore_CMD_t *ore_store;
  RodNew_CMD_t *rod;
} CmdCenterBridge_OutputFrame_t;

void CmdCenterBridge_SetDr16Source(const DR16_t *dr16);
bool CmdCenterBridge_IsCoreAvailable(void);
void CmdCenterBridge_Init(void);
bool CmdCenterBridge_Tick(uint32_t now_ms,
                          CmdCenterBridge_OutputFrame_t *outputs);

#ifdef __cplusplus
}
#endif
