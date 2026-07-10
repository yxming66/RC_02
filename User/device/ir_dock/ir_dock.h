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

#define IR_DOCK_FRAME_HEAD0 (0x4Du)
#define IR_DOCK_FRAME_HEAD1 (0x52u)
#define IR_DOCK_LEGACY_FRAME_SIZE (8u)
#define IR_DOCK_FRAME_SIZE (8u)
#define IR_DOCK_CRC_SIZE (2u)
#define IR_DOCK_CMD_UNFINISHED (0x00u)
#define IR_DOCK_CMD_COMPLETE (0x01u)
#define IR_DOCK_LEAVE_ZONE1_FORBID (0x00u)
#define IR_DOCK_LEAVE_ZONE1_ALLOW (0x01u)
#define IR_DOCK_CLAW_OPEN_WAIT (0x00u)
#define IR_DOCK_CLAW_OPEN_ALLOW (0x01u)
#define IR_DOCK_CLAW_OPEN_ABORT (0x02u)
#define IR_DOCK_ZONE3_R2_WORK (0x01u)
#define IR_DOCK_ZONE3_R2_STANDBY (0x02u)
#define IR_DOCK_ZONE3_R2_UNKNOWN (0x00u)
#define IR_DOCK_CLEARED_ORE_NONE (0xFFu)

typedef enum {
  IR_DOCK_STATUS_IDLE = 0x00u,
  IR_DOCK_STATUS_DOCK_COMPLETE = IR_DOCK_CMD_COMPLETE,
} IrDock_Status_t;

typedef struct {
  volatile bool inited;
  volatile bool online;
  volatile bool rx_busy;
  volatile bool dock_complete_fresh;
  volatile bool r2_leave_zone1_allowed;
  volatile bool claw_open;
  volatile uint8_t last_rx_status;
  volatile uint8_t last_rx_raw_byte;
  volatile uint8_t last_dock_complete_cmd;
  volatile uint8_t last_r2_leave_zone1_cmd;
  volatile uint8_t last_claw_open_cmd;
  volatile uint8_t last_cleared_ore_id;
  volatile uint8_t last_zone3_r2_state;
  volatile uint8_t last_rx_len;
  volatile uint8_t last_rx_raw_len;
  volatile uint8_t last_rx_raw_data[IR_DOCK_RX_BUFFER_SIZE];
  volatile uint32_t last_rx_start_ms;
  volatile uint32_t last_rx_raw_ms;
  volatile uint32_t last_online_rx_ms;
  volatile uint32_t last_complete_rx_ms;
  volatile uint32_t last_claw_open_rx_ms;
  volatile uint32_t last_rx_ms;
  volatile uint32_t last_rx_age_ms;
  volatile uint32_t last_online_age_ms;
  volatile uint32_t last_complete_age_ms;
  volatile uint32_t last_claw_open_age_ms;
  volatile uint32_t last_rx_raw_age_ms;
  volatile uint32_t rx_count;
  volatile uint32_t online_rx_count;
  volatile uint32_t complete_rx_count;
  volatile uint32_t claw_open_rx_count;
  volatile uint32_t claw_open_abort_rx_count;
  volatile uint32_t protocol_frame_rx_count;
  volatile uint32_t crc_error_count;
  volatile uint32_t invalid_rx_count;
  volatile uint32_t error_count;
  volatile uint32_t rx_interval_block_count;
} IrDock_Debug_t;

extern volatile IrDock_Debug_t g_ir_dock_debug;

void IrDock_Init(uint32_t now_ms);
void IrDock_Process(uint32_t now_ms);
bool IrDock_IsDockCompleteFresh(uint32_t now_ms);
bool IrDock_IsOnline(uint32_t now_ms);
IrDock_Status_t IrDock_GetLastRxStatus(void);
bool IrDock_IsR2LeaveZone1Allowed(void);
bool IrDock_IsClawOpenFresh(uint32_t now_ms);
uint8_t IrDock_GetLastClawOpenCommand(void);
uint32_t IrDock_GetClawOpenAbortCount(void);
uint8_t IrDock_GetLastClearedOreId(void);
uint8_t IrDock_GetLastZone3R2State(void);

#ifdef __cplusplus
}
#endif
