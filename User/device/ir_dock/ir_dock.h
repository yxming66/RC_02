#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IR_DOCK_MIN_INTERVAL_MS
#define IR_DOCK_MIN_INTERVAL_MS (250u)
#endif

#ifndef IR_DOCK_COMPLETE_FRESH_MS
#define IR_DOCK_COMPLETE_FRESH_MS (1000u)
#endif

#ifndef IR_DOCK_ORE_INFO_FRESH_MS
#define IR_DOCK_ORE_INFO_FRESH_MS (1000u)
#endif

#define IR_DOCK_MAX_PACKET_SIZE (16u)
#define IR_DOCK_ORE_POSITION_COUNT (12u)
#define IR_DOCK_ORE_PACKET_SIZE (1u + IR_DOCK_ORE_POSITION_COUNT)
#define IR_DOCK_ORE_TYPE_ONLY_PACKET_SIZE (IR_DOCK_ORE_POSITION_COUNT)

typedef enum {
  IR_DOCK_STATUS_IDLE = 0x00u,
  IR_DOCK_STATUS_DOCKING = 0x01u,
  IR_DOCK_STATUS_DOCK_COMPLETE = 0x02u,
} IrDock_Status_t;

typedef enum {
  IR_DOCK_ORE_TYPE_UNKNOWN = 0u,
  IR_DOCK_ORE_TYPE_R1 = 1u,
  IR_DOCK_ORE_TYPE_R2 = 2u,
  IR_DOCK_ORE_TYPE_FAKE = 3u,
} IrDock_OreType_t;

typedef struct {
  bool valid;
  bool fresh;
  uint8_t status;
  uint8_t count;
  uint8_t ore_type[IR_DOCK_ORE_POSITION_COUNT];
  uint32_t last_rx_ms;
  uint32_t age_ms;
  uint32_t rx_count;
} IrDock_OreInfo_t;

typedef struct {
  volatile bool inited;
  volatile bool rx_busy;
  volatile bool tx_busy;
  volatile bool dock_complete_fresh;
  volatile bool ore_info_valid;
  volatile bool ore_info_fresh;
  volatile uint8_t last_rx_status;
  volatile uint8_t last_tx_status;
  volatile uint8_t last_rx_len;
  volatile uint8_t last_tx_len;
  volatile uint8_t ore_type[IR_DOCK_ORE_POSITION_COUNT];
  volatile uint32_t last_rx_start_ms;
  volatile uint32_t last_rx_ms;
  volatile uint32_t last_ore_rx_ms;
  volatile uint32_t last_tx_ms;
  volatile uint32_t last_rx_age_ms;
  volatile uint32_t last_ore_rx_age_ms;
  volatile uint32_t rx_count;
  volatile uint32_t tx_count;
  volatile uint32_t ore_info_rx_count;
  volatile uint32_t ore_info_error_count;
  volatile uint32_t error_count;
  volatile uint32_t rx_interval_block_count;
  volatile uint32_t tx_interval_block_count;
} IrDock_Debug_t;

extern volatile IrDock_Debug_t g_ir_dock_debug;

void IrDock_Init(uint32_t now_ms);
void IrDock_Process(uint32_t now_ms);
bool IrDock_SendStatus(IrDock_Status_t status, uint32_t now_ms);
bool IrDock_SendOreInfo(IrDock_Status_t status,
                        const uint8_t ore_type[IR_DOCK_ORE_POSITION_COUNT],
                        uint32_t now_ms);
bool IrDock_IsDockCompleteFresh(uint32_t now_ms);
bool IrDock_IsOreInfoFresh(uint32_t now_ms);
IrDock_Status_t IrDock_GetLastRxStatus(void);
bool IrDock_GetOreInfo(IrDock_OreInfo_t *info, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
