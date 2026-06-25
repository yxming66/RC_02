#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IR_DOCK_COMPLETE_FRESH_MS
#define IR_DOCK_COMPLETE_FRESH_MS (1000u)
#endif

#ifndef IR_DOCK_ORE_INFO_FRESH_MS
#define IR_DOCK_ORE_INFO_FRESH_MS (1000u)
#endif

#ifndef IR_DOCK_ACK_PENDING_TIMEOUT_MS
#define IR_DOCK_ACK_PENDING_TIMEOUT_MS (350u)
#endif

#ifndef IR_DOCK_RX_BUFFER_SIZE
#define IR_DOCK_RX_BUFFER_SIZE (64u)
#endif

#define IR_DOCK_FRAME_HEAD0 (0xAAu)
#define IR_DOCK_FRAME_HEAD1 (0x55u)
#define IR_DOCK_CMD_MINE_INPUT (0x02u)
#define IR_DOCK_CMD_MINE_ACK (0x82u)
#define IR_DOCK_MINE_INPUT_FRAME_SIZE (18u)
#define IR_DOCK_MINE_INPUT_CRC_OFFSET (17u)
#define IR_DOCK_ACK_FRAME_SIZE (6u)
#define IR_DOCK_ACK_CRC_OFFSET (5u)
#define IR_DOCK_STATUS_ACK_HEAD (0xA5u)
#define IR_DOCK_STATUS_ACK_FRAME_SIZE (2u)
#define IR_DOCK_MAX_PACKET_SIZE (IR_DOCK_RX_BUFFER_SIZE)
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

typedef enum {
  IR_DOCK_ACK_STATUS_OK = 0x00u,
  IR_DOCK_ACK_STATUS_BUSY = 0x01u,
  IR_DOCK_ACK_STATUS_CRC_ERR = 0x02u,
  IR_DOCK_ACK_STATUS_INVALID = 0x03u,
} IrDock_AckStatus_t;

typedef enum {
  IR_DOCK_PARSE_STATUS_OK = IR_DOCK_ACK_STATUS_OK,
  IR_DOCK_PARSE_STATUS_BUSY = IR_DOCK_ACK_STATUS_BUSY,
  IR_DOCK_PARSE_STATUS_CRC_ERR = IR_DOCK_ACK_STATUS_CRC_ERR,
  IR_DOCK_PARSE_STATUS_INVALID = IR_DOCK_ACK_STATUS_INVALID,
} IrDock_ParseStatus_t;

typedef enum {
  IR_DOCK_ACK_SUBMIT_OK = 0,
  IR_DOCK_ACK_SUBMIT_BUSY = 1,
  IR_DOCK_ACK_SUBMIT_INVALID = 2,
  IR_DOCK_ACK_SUBMIT_NO_PENDING = 3,
  IR_DOCK_ACK_SUBMIT_MSG_MISMATCH = 4,
  IR_DOCK_ACK_SUBMIT_TX_ERROR = 5,
} IrDock_AckSubmitResult_t;

typedef struct {
  bool valid;
  bool fresh;
  uint8_t status;
  uint8_t count;
  uint8_t msg_id;
  uint8_t side;
  uint8_t ack_pending;
  uint8_t parse_status;
  uint8_t ore_type[IR_DOCK_ORE_POSITION_COUNT];
  uint8_t raw_frame[IR_DOCK_MINE_INPUT_FRAME_SIZE];
  uint32_t last_rx_ms;
  uint32_t age_ms;
  uint32_t rx_count;
  uint32_t frame_rx_count;
  uint32_t ack_tx_count;
} IrDock_OreInfo_t;

typedef struct {
  volatile bool inited;
  volatile bool rx_busy;
  volatile bool tx_busy;
  volatile bool ack_pending;
  volatile bool dock_complete_fresh;
  volatile bool ore_info_valid;
  volatile bool ore_info_fresh;
  volatile uint8_t last_rx_status;
  volatile uint8_t last_rx_raw_byte;
  volatile uint8_t last_tx_status;
  volatile uint8_t last_rx_len;
  volatile uint8_t last_rx_raw_len;
  volatile uint8_t last_tx_len;
  volatile uint8_t last_msg_id;
  volatile uint8_t last_side;
  volatile uint8_t last_parse_status;
  volatile uint8_t last_ack_status;
  volatile uint8_t parse_index;
  volatile uint8_t last_rx_raw_data[IR_DOCK_RX_BUFFER_SIZE];
  volatile uint8_t ore_type[IR_DOCK_ORE_POSITION_COUNT];
  volatile uint8_t raw_frame[IR_DOCK_MINE_INPUT_FRAME_SIZE];
  volatile uint8_t last_ack_frame[IR_DOCK_ACK_FRAME_SIZE];
  volatile uint32_t last_rx_start_ms;
  volatile uint32_t last_rx_raw_ms;
  volatile uint32_t last_rx_ms;
  volatile uint32_t last_ore_rx_ms;
  volatile uint32_t last_tx_ms;
  volatile uint32_t last_rx_age_ms;
  volatile uint32_t last_ore_rx_age_ms;
  volatile uint32_t last_rx_raw_age_ms;
  volatile uint32_t rx_count;
  volatile uint32_t tx_count;
  volatile uint32_t frame_rx_count;
  volatile uint32_t ack_tx_count;
  volatile uint32_t status_ack_tx_count;
  volatile uint32_t ack_invalid_count;
  volatile uint32_t ack_mismatch_count;
  volatile uint32_t ack_no_pending_count;
  volatile uint32_t ack_timeout_count;
  volatile uint32_t ore_info_rx_count;
  volatile uint32_t ore_info_error_count;
  volatile uint32_t crc_error_count;
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
IrDock_AckSubmitResult_t IrDock_SendAckFrame(
    const uint8_t ack_frame[IR_DOCK_ACK_FRAME_SIZE], uint32_t now_ms);
bool IrDock_IsDockCompleteFresh(uint32_t now_ms);
bool IrDock_IsOreInfoFresh(uint32_t now_ms);
IrDock_Status_t IrDock_GetLastRxStatus(void);
bool IrDock_GetOreInfo(IrDock_OreInfo_t *info, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
