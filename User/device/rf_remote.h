#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "device.h"

#define RF_REMOTE_CELL_NUM (12u)
#define RF_REMOTE_KFS_FRAME_SIZE (18u)
#define RF_REMOTE_RETRY_FRAME_SIZE (6u)
#define RF_REMOTE_ACK_FRAME_SIZE (6u)

/*
 * The receiver has no E34 AUX connection. Keep RX stopped while replying and
 * use the fixed half-duplex delays required by the MineInput protocol.
 * Increase RF_REMOTE_ACK_REPEAT_COUNT to 2 or 3 for a noisy field link.
 */
#ifndef RF_REMOTE_ACK_REPEAT_COUNT
#define RF_REMOTE_ACK_REPEAT_COUNT (1u)
#endif

#if RF_REMOTE_ACK_REPEAT_COUNT < 1u || RF_REMOTE_ACK_REPEAT_COUNT > 3u
#error "RF_REMOTE_ACK_REPEAT_COUNT must be in the range 1..3"
#endif

typedef enum {
  RF_REMOTE_CMD_KFS = 0x02u,
  RF_REMOTE_CMD_R2_RETRY = 0x03u,
  RF_REMOTE_CMD_ACK = 0x82u,
} RF_Remote_Command_t;

typedef enum {
  RF_REMOTE_ACK_OK = 0x00u,
  RF_REMOTE_ACK_BUSY = 0x01u,
  RF_REMOTE_ACK_CRC_ERROR = 0x02u,
  RF_REMOTE_ACK_INVALID = 0x03u,
} RF_Remote_AckStatus_t;

typedef enum {
  RF_REMOTE_MODE_RELAX = 1u,
  RF_REMOTE_MODE_LOCK = 2u,
  RF_REMOTE_MODE_RESET = 3u,
  RF_REMOTE_MODE_START = 4u,
  RF_REMOTE_MODE_RETRY_REGION_1 = 7u,
  RF_REMOTE_MODE_RETRY_REGION_2 = 8u,
  RF_REMOTE_MODE_RETRY_REGION_3 = 9u,
} RF_Remote_Mode_t;

typedef enum {
  RF_REMOTE_CELL_EMPT3Y = 0u,
  RF_REMOTE_CELL_R1 = 1u,
  RF_REMOTE_CELL_R2 = 2u,
  RF_REMOTE_CELL_FAKE = 3u,
} RF_Remote_CellState_t;

typedef struct {
  uint8_t command;
  uint8_t msg_id;
  uint8_t side;
  uint8_t cells[RF_REMOTE_CELL_NUM];
  uint8_t retry_mode;
  uint64_t rx_time_us;
} RF_Remote_Frame_t;

typedef struct {
  volatile uint32_t rx_bytes;
  volatile uint32_t rx_events;
  volatile uint32_t good_frames;
  volatile uint32_t kfs_frames;
  volatile uint32_t retry_frames;
  volatile uint32_t duplicate_frames;
  volatile uint32_t crc_errors;
  volatile uint32_t data_errors;
  volatile uint32_t format_errors;
  volatile uint32_t sync_drops;
  volatile uint32_t dma_errors;
  volatile uint32_t busy_frames;
  volatile uint32_t ack_frames;
  volatile uint32_t ack_errors;
  volatile uint8_t last_command;
  volatile uint8_t last_msg_id;
  volatile uint8_t last_retry_mode;
  volatile uint8_t last_ack_status;
  volatile uint64_t last_rx_time_us;
  volatile bool online;
} RF_Remote_Stats_t;

typedef struct {
  RF_Remote_Frame_t latest;
  RF_Remote_Stats_t stats;
} RF_Remote_Debug_t;

extern volatile RF_Remote_Debug_t rf_remote_debug;

int8_t RF_Remote_Init(void);
int8_t RF_Remote_Restart(void);
int8_t RF_Remote_StartDmaRecv(void);
bool RF_Remote_WaitDmaCplt(uint32_t timeout);
int8_t RF_Remote_ParseData(void);
int8_t RF_Remote_Poll(uint32_t offline_timeout_us);
bool RF_Remote_GetLatest(RF_Remote_Frame_t *frame);
const volatile RF_Remote_Stats_t *RF_Remote_GetStats(void);
bool RF_Remote_HasPendingAck(void);
void RF_Remote_SetBusy(bool busy);

/* Override these weak callbacks in the business layer when required. */
void RF_Remote_OnKfsFrame(uint8_t side,
                          const uint8_t cells[RF_REMOTE_CELL_NUM],
                          uint8_t msg_id);
void RF_Remote_OnRetryFrame(uint8_t mode, uint8_t msg_id);

#ifdef __cplusplus
}
#endif
