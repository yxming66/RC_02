#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IR_DOCK_COMPLETE_FRESH_MS
#define IR_DOCK_COMPLETE_FRESH_MS (1000u)
#endif

#ifndef IR_DOCK_ONLINE_TIMEOUT_MS
#define IR_DOCK_ONLINE_TIMEOUT_MS (750u)
#endif

#ifndef IR_DOCK_RX_BUFFER_SIZE
#define IR_DOCK_RX_BUFFER_SIZE (16u)
#endif

#define IR_DOCK_CMD_ONLINE (0x01u)
#define IR_DOCK_CMD_COMPLETE (0x02u)
#define IR_DOCK_STATUS_ACK_HEAD (0xA5u)
#define IR_DOCK_STATUS_ACK_FRAME_SIZE (2u)

typedef enum {
  IR_DOCK_STATUS_IDLE = 0x00u,
  IR_DOCK_STATUS_ONLINE = IR_DOCK_CMD_ONLINE,
  IR_DOCK_STATUS_DOCK_COMPLETE = IR_DOCK_CMD_COMPLETE,
} IrDock_Status_t;

typedef struct {
  volatile bool inited;
  volatile bool online;
  volatile bool rx_busy;
  volatile bool tx_busy;
  volatile bool dock_complete_fresh;
  volatile uint8_t last_rx_status;
  volatile uint8_t last_rx_raw_byte;
  volatile uint8_t last_tx_status;
  volatile uint8_t last_rx_len;
  volatile uint8_t last_rx_raw_len;
  volatile uint8_t last_tx_len;
  volatile uint8_t last_rx_raw_data[IR_DOCK_RX_BUFFER_SIZE];
  volatile uint8_t last_ack_frame[IR_DOCK_STATUS_ACK_FRAME_SIZE];
  volatile uint32_t last_rx_start_ms;
  volatile uint32_t last_rx_raw_ms;
  volatile uint32_t last_online_rx_ms;
  volatile uint32_t last_complete_rx_ms;
  volatile uint32_t last_rx_ms;
  volatile uint32_t last_tx_ms;
  volatile uint32_t last_rx_age_ms;
  volatile uint32_t last_online_age_ms;
  volatile uint32_t last_complete_age_ms;
  volatile uint32_t last_rx_raw_age_ms;
  volatile uint32_t rx_count;
  volatile uint32_t tx_count;
  volatile uint32_t ack_tx_count;
  volatile uint32_t status_ack_tx_count;
  volatile uint32_t online_rx_count;
  volatile uint32_t complete_rx_count;
  volatile uint32_t invalid_rx_count;
  volatile uint32_t error_count;
  volatile uint32_t rx_interval_block_count;
  volatile uint32_t tx_interval_block_count;
} IrDock_Debug_t;

extern volatile IrDock_Debug_t g_ir_dock_debug;

void IrDock_Init(uint32_t now_ms);
void IrDock_Process(uint32_t now_ms);
bool IrDock_SendStatus(IrDock_Status_t status, uint32_t now_ms);
bool IrDock_IsDockCompleteFresh(uint32_t now_ms);
bool IrDock_IsOnline(uint32_t now_ms);
IrDock_Status_t IrDock_GetLastRxStatus(void);

#ifdef __cplusplus
}
#endif
