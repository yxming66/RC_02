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

#define IR_DOCK_FRAME_HEAD (0xA5u)
#define IR_DOCK_FRAME_SIZE (2u)
#define IR_DOCK_CMD_ONLINE (0x01u)
#define IR_DOCK_CMD_DOCK_COMPLETE (0x02u)
#define IR_DOCK_CMD_ZONE3_ACTION_START (0x03u)
#define IR_DOCK_CMD_RELEASE_ALLOW (0x04u)
#define IR_DOCK_CMD_ZONE3_ACTION_FINISH (0x05u)
#define IR_DOCK_CMD_RELEASE_ABORT (0x06u)
#define IR_DOCK_ZONE3_ACTION_CMD_NONE (0x00u)
#define IR_DOCK_ZONE3_ACTION_CMD_START (0x01u)
#define IR_DOCK_ZONE3_ACTION_CMD_FINISH (0x02u)
#define IR_DOCK_CLAW_OPEN_WAIT (0x00u)
#define IR_DOCK_CLAW_OPEN_ALLOW (0x01u)
#define IR_DOCK_CLAW_OPEN_ABORT (0x02u)
#define IR_DOCK_ZONE3_ACTION_UNKNOWN (0x00u)
#define IR_DOCK_ZONE3_ACTION_LOCKED (0x01u)
#define IR_DOCK_ZONE3_ACTION_FINISHED (0x02u)

typedef enum {
  IR_DOCK_STATUS_IDLE = 0x00u,
  IR_DOCK_STATUS_ONLINE = IR_DOCK_CMD_ONLINE,
  IR_DOCK_STATUS_DOCK_COMPLETE = IR_DOCK_CMD_DOCK_COMPLETE,
  IR_DOCK_STATUS_ZONE3_ACTION_START = IR_DOCK_CMD_ZONE3_ACTION_START,
  IR_DOCK_STATUS_RELEASE_ALLOW = IR_DOCK_CMD_RELEASE_ALLOW,
  IR_DOCK_STATUS_ZONE3_ACTION_FINISH = IR_DOCK_CMD_ZONE3_ACTION_FINISH,
  IR_DOCK_STATUS_RELEASE_ABORT = IR_DOCK_CMD_RELEASE_ABORT,
} IrDock_Status_t;

typedef struct {
  volatile bool inited;
  volatile bool online;
  volatile bool rx_busy;
  volatile bool dock_complete_fresh;
  volatile bool zone3_action_locked;
  volatile bool claw_open;
  volatile bool release_abort_latched;
  volatile uint8_t last_rx_status;
  volatile uint8_t last_rx_raw_byte;
  volatile uint8_t last_dock_complete_cmd;
  volatile uint8_t last_zone3_action_cmd;
  volatile uint8_t last_claw_open_cmd;
  volatile uint8_t zone3_action_state;
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
  volatile uint32_t zone3_action_start_rx_count;
  volatile uint32_t zone3_action_finish_rx_count;
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
bool IrDock_IsZone3ActionLocked(void);
bool IrDock_IsReleaseAbortLatched(void);
bool IrDock_IsClawOpenFresh(uint32_t now_ms);
uint8_t IrDock_GetLastClawOpenCommand(void);
uint32_t IrDock_GetClawOpenRxTimeMs(void);
uint32_t IrDock_GetClawOpenAbortCount(void);
uint32_t IrDock_GetZone3ActionStartCount(void);
uint32_t IrDock_GetZone3ActionFinishCount(void);
uint8_t IrDock_GetZone3ActionState(void);

#ifdef __cplusplus
}
#endif
