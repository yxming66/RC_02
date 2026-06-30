#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ORE_INFO_FRESH_MS
#define ORE_INFO_FRESH_MS (1000u)
#endif

#ifndef ORE_INFO_ACK_PENDING_TIMEOUT_MS
#define ORE_INFO_ACK_PENDING_TIMEOUT_MS (350u)
#endif

#ifndef ORE_INFO_RX_BUFFER_SIZE
#define ORE_INFO_RX_BUFFER_SIZE (64u)
#endif

#define ORE_INFO_FRAME_HEAD0 (0xAAu)
#define ORE_INFO_FRAME_HEAD1 (0x55u)
#define ORE_INFO_CMD_MINE_INPUT (0x02u)
#define ORE_INFO_CMD_MINE_ACK (0x82u)
#define ORE_INFO_FRAME_SIZE (18u)
#define ORE_INFO_CRC_OFFSET (17u)
#define ORE_INFO_ACK_FRAME_SIZE (6u)
#define ORE_INFO_ACK_CRC_OFFSET (5u)
#define ORE_INFO_POSITION_COUNT (12u)

typedef enum {
  ORE_INFO_TYPE_UNKNOWN = 0u,
  ORE_INFO_TYPE_R1 = 1u,
  ORE_INFO_TYPE_R2 = 2u,
  ORE_INFO_TYPE_FAKE = 3u,
} OreInfo_Type_t;

typedef enum {
  ORE_INFO_ACK_STATUS_OK = 0x00u,
  ORE_INFO_ACK_STATUS_BUSY = 0x01u,
  ORE_INFO_ACK_STATUS_CRC_ERR = 0x02u,
  ORE_INFO_ACK_STATUS_INVALID = 0x03u,
} OreInfo_AckStatus_t;

typedef enum {
  ORE_INFO_PARSE_STATUS_OK = ORE_INFO_ACK_STATUS_OK,
  ORE_INFO_PARSE_STATUS_BUSY = ORE_INFO_ACK_STATUS_BUSY,
  ORE_INFO_PARSE_STATUS_CRC_ERR = ORE_INFO_ACK_STATUS_CRC_ERR,
  ORE_INFO_PARSE_STATUS_INVALID = ORE_INFO_ACK_STATUS_INVALID,
} OreInfo_ParseStatus_t;

typedef enum {
  ORE_INFO_ACK_SUBMIT_OK = 0,
  ORE_INFO_ACK_SUBMIT_BUSY = 1,
  ORE_INFO_ACK_SUBMIT_INVALID = 2,
  ORE_INFO_ACK_SUBMIT_NO_PENDING = 3,
  ORE_INFO_ACK_SUBMIT_MSG_MISMATCH = 4,
  ORE_INFO_ACK_SUBMIT_TX_ERROR = 5,
} OreInfo_AckSubmitResult_t;

typedef struct {
  bool valid;
  bool fresh;
  uint8_t count;
  uint8_t msg_id;
  uint8_t side;
  uint8_t ack_pending;
  uint8_t parse_status;
  uint8_t ore_type[ORE_INFO_POSITION_COUNT];
  uint8_t raw_frame[ORE_INFO_FRAME_SIZE];
  uint32_t last_rx_ms;
  uint32_t age_ms;
  uint32_t rx_count;
  uint32_t frame_rx_count;
  uint32_t ack_tx_count;
} OreInfo_Info_t;

typedef struct {
  volatile bool inited;
  volatile bool rx_busy;
  volatile bool tx_busy;
  volatile bool ack_pending;
  volatile bool info_valid;
  volatile bool info_fresh;
  volatile uint8_t last_rx_len;
  volatile uint8_t last_rx_raw_len;
  volatile uint8_t last_tx_len;
  volatile uint8_t last_msg_id;
  volatile uint8_t last_side;
  volatile uint8_t last_parse_status;
  volatile uint8_t last_ack_status;
  volatile uint8_t parse_index;
  volatile uint8_t last_rx_raw_data[ORE_INFO_RX_BUFFER_SIZE];
  volatile uint8_t ore_type[ORE_INFO_POSITION_COUNT];
  volatile uint8_t raw_frame[ORE_INFO_FRAME_SIZE];
  volatile uint8_t last_ack_frame[ORE_INFO_ACK_FRAME_SIZE];
  volatile uint32_t last_rx_start_ms;
  volatile uint32_t last_rx_raw_ms;
  volatile uint32_t last_rx_ms;
  volatile uint32_t last_tx_ms;
  volatile uint32_t last_rx_age_ms;
  volatile uint32_t last_rx_raw_age_ms;
  volatile uint32_t rx_count;
  volatile uint32_t tx_count;
  volatile uint32_t frame_rx_count;
  volatile uint32_t ack_tx_count;
  volatile uint32_t ack_invalid_count;
  volatile uint32_t ack_mismatch_count;
  volatile uint32_t ack_no_pending_count;
  volatile uint32_t ack_timeout_count;
  volatile uint32_t info_error_count;
  volatile uint32_t crc_error_count;
  volatile uint32_t error_count;
  volatile uint32_t rx_interval_block_count;
  volatile uint32_t tx_interval_block_count;
} OreInfo_Debug_t;

extern volatile OreInfo_Debug_t g_ore_info_debug;

void OreInfo_Init(uint32_t now_ms);
void OreInfo_Process(uint32_t now_ms);
OreInfo_AckSubmitResult_t OreInfo_SendAckFrame(
    const uint8_t ack_frame[ORE_INFO_ACK_FRAME_SIZE], uint32_t now_ms);
bool OreInfo_IsFresh(uint32_t now_ms);
bool OreInfo_GetInfo(OreInfo_Info_t *info, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
