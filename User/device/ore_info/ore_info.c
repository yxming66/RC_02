#include "device/ore_info/ore_info.h"

#include <stddef.h>

#include "bsp/uart.h"
#include "component/crc8.h"

static uint8_t ore_info_rx_buf[ORE_INFO_RX_BUFFER_SIZE] = {0};
static uint8_t ore_info_tx_buf[ORE_INFO_ACK_FRAME_SIZE] = {0};
static uint8_t ore_info_parse_frame[ORE_INFO_FRAME_SIZE] = {0};
static volatile uint16_t ore_info_rx_len = 0u;
static volatile bool ore_info_rx_complete_pending = false;
static volatile bool ore_info_tx_complete_pending = false;
static volatile bool ore_info_error_pending = false;

static void OreInfo_RxEventCallback(uint16_t size) {
  if (size > ORE_INFO_RX_BUFFER_SIZE) {
    size = ORE_INFO_RX_BUFFER_SIZE;
  }
  ore_info_rx_len = size;
  ore_info_rx_complete_pending = true;
  g_ore_info_debug.rx_busy = false;
}

static void OreInfo_RxCompleteCallback(void) {
  ore_info_rx_len = ORE_INFO_RX_BUFFER_SIZE;
  ore_info_rx_complete_pending = true;
  g_ore_info_debug.rx_busy = false;
}

static void OreInfo_TxCompleteCallback(void) {
  ore_info_tx_complete_pending = true;
  g_ore_info_debug.tx_busy = false;
}

static void OreInfo_ErrorCallback(void) {
  ore_info_error_pending = true;
  g_ore_info_debug.rx_busy = false;
  g_ore_info_debug.tx_busy = false;
}

static bool OreInfo_SideIsValid(uint8_t side) {
  return side == (uint8_t)'R' || side == (uint8_t)'B';
}

static bool OreInfo_TypeIsValid(uint8_t ore_type) {
  return ore_type == (uint8_t)ORE_INFO_TYPE_UNKNOWN ||
         ore_type == (uint8_t)ORE_INFO_TYPE_R1 ||
         ore_type == (uint8_t)ORE_INFO_TYPE_R2 ||
         ore_type == (uint8_t)ORE_INFO_TYPE_FAKE;
}

static uint8_t OreInfo_CountType(const uint8_t *ore_type, uint8_t type) {
  uint8_t count = 0u;

  for (uint8_t i = 0u; i < ORE_INFO_POSITION_COUNT; ++i) {
    if (ore_type[i] == type) {
      count++;
    }
  }
  return count;
}

static bool OreInfo_FrameInfoIsValid(uint8_t side, const uint8_t *ore_type) {
  if (ore_type == NULL || !OreInfo_SideIsValid(side)) {
    return false;
  }

  for (uint8_t i = 0u; i < ORE_INFO_POSITION_COUNT; ++i) {
    if (!OreInfo_TypeIsValid(ore_type[i])) {
      return false;
    }
  }

  return OreInfo_CountType(ore_type, (uint8_t)ORE_INFO_TYPE_R1) <= 3u &&
         OreInfo_CountType(ore_type, (uint8_t)ORE_INFO_TYPE_R2) <= 4u &&
         OreInfo_CountType(ore_type, (uint8_t)ORE_INFO_TYPE_FAKE) <= 1u;
}

static void OreInfo_ClearTypes(void) {
  for (uint8_t i = 0u; i < ORE_INFO_POSITION_COUNT; ++i) {
    g_ore_info_debug.ore_type[i] = (uint8_t)ORE_INFO_TYPE_UNKNOWN;
  }
}

static void OreInfo_ClearRawFrame(void) {
  for (uint8_t i = 0u; i < ORE_INFO_FRAME_SIZE; ++i) {
    g_ore_info_debug.raw_frame[i] = 0u;
    ore_info_parse_frame[i] = 0u;
  }
}

static void OreInfo_ClearRawRxData(void) {
  for (uint8_t i = 0u; i < ORE_INFO_RX_BUFFER_SIZE; ++i) {
    g_ore_info_debug.last_rx_raw_data[i] = 0u;
  }
  g_ore_info_debug.last_rx_raw_len = 0u;
}

static void OreInfo_RecordRawRxData(const uint8_t *rx_buf,
                                    uint16_t packet_len,
                                    uint32_t now_ms) {
  if (rx_buf == NULL) {
    return;
  }

  uint16_t copy_len = packet_len;
  if (copy_len > ORE_INFO_RX_BUFFER_SIZE) {
    copy_len = ORE_INFO_RX_BUFFER_SIZE;
  }

  OreInfo_ClearRawRxData();
  g_ore_info_debug.last_rx_raw_len = (uint8_t)copy_len;
  g_ore_info_debug.last_rx_raw_ms = now_ms;
  for (uint16_t i = 0u; i < copy_len; ++i) {
    g_ore_info_debug.last_rx_raw_data[i] = rx_buf[i];
  }
}

static void OreInfo_CopyVolatileBytes(volatile uint8_t *dst,
                                      const uint8_t *src,
                                      uint8_t len) {
  if (dst == NULL || src == NULL) {
    return;
  }

  for (uint8_t i = 0u; i < len; ++i) {
    dst[i] = src[i];
  }
}

static void OreInfo_BuildAck(uint8_t msg_id, uint8_t status,
                             uint8_t ack_frame[ORE_INFO_ACK_FRAME_SIZE]) {
  ack_frame[0] = ORE_INFO_FRAME_HEAD0;
  ack_frame[1] = ORE_INFO_FRAME_HEAD1;
  ack_frame[2] = ORE_INFO_CMD_MINE_ACK;
  ack_frame[3] = msg_id;
  ack_frame[4] = status;
  ack_frame[ORE_INFO_ACK_CRC_OFFSET] =
      CRC8_Calc(ack_frame, ORE_INFO_ACK_CRC_OFFSET, CRC8_INIT);
}

static bool OreInfo_SendMineAck(uint8_t msg_id, uint8_t status,
                                uint32_t now_ms) {
  if (!g_ore_info_debug.inited || g_ore_info_debug.tx_busy) {
    g_ore_info_debug.tx_interval_block_count++;
    return false;
  }

  OreInfo_BuildAck(msg_id, status, ore_info_tx_buf);

  if (BSP_UART_Transmit(BSP_UART_ORE, ore_info_tx_buf,
                        ORE_INFO_ACK_FRAME_SIZE, false) == HAL_OK) {
    OreInfo_CopyVolatileBytes(g_ore_info_debug.last_ack_frame,
                              ore_info_tx_buf, ORE_INFO_ACK_FRAME_SIZE);
    g_ore_info_debug.last_ack_status = status;
    g_ore_info_debug.last_tx_len = ORE_INFO_ACK_FRAME_SIZE;
    g_ore_info_debug.last_tx_ms = now_ms;
    g_ore_info_debug.tx_busy = true;
    g_ore_info_debug.ack_pending = false;
    g_ore_info_debug.tx_count++;
    g_ore_info_debug.ack_tx_count++;
    return true;
  }

  g_ore_info_debug.error_count++;
  return false;
}

static void OreInfo_ParseFrame(uint32_t now_ms) {
  const uint8_t msg_id = ore_info_parse_frame[3];
  const uint8_t side = ore_info_parse_frame[4];
  const uint8_t *ore_type = &ore_info_parse_frame[5];
  uint8_t ack_status = (uint8_t)ORE_INFO_ACK_STATUS_INVALID;

  g_ore_info_debug.last_rx_len = ORE_INFO_FRAME_SIZE;
  g_ore_info_debug.last_msg_id = msg_id;
  g_ore_info_debug.last_side = side;
  OreInfo_CopyVolatileBytes(g_ore_info_debug.raw_frame, ore_info_parse_frame,
                            ORE_INFO_FRAME_SIZE);
  g_ore_info_debug.last_rx_ms = now_ms;
  g_ore_info_debug.frame_rx_count++;
  g_ore_info_debug.ack_pending = true;

  if (ore_info_parse_frame[2] != ORE_INFO_CMD_MINE_INPUT) {
    g_ore_info_debug.last_parse_status =
        (uint8_t)ORE_INFO_PARSE_STATUS_INVALID;
    g_ore_info_debug.info_error_count++;
    g_ore_info_debug.error_count++;
  } else if (!CRC8_Verify(ore_info_parse_frame, ORE_INFO_FRAME_SIZE)) {
    ack_status = (uint8_t)ORE_INFO_ACK_STATUS_CRC_ERR;
    g_ore_info_debug.last_parse_status =
        (uint8_t)ORE_INFO_PARSE_STATUS_CRC_ERR;
    g_ore_info_debug.crc_error_count++;
    g_ore_info_debug.info_error_count++;
    g_ore_info_debug.error_count++;
  } else if (!OreInfo_FrameInfoIsValid(side, ore_type)) {
    g_ore_info_debug.last_parse_status =
        (uint8_t)ORE_INFO_PARSE_STATUS_INVALID;
    g_ore_info_debug.info_error_count++;
    g_ore_info_debug.error_count++;
  } else {
    for (uint8_t i = 0u; i < ORE_INFO_POSITION_COUNT; ++i) {
      g_ore_info_debug.ore_type[i] = ore_type[i];
    }
    g_ore_info_debug.info_valid = true;
    g_ore_info_debug.info_fresh = true;
    g_ore_info_debug.last_parse_status =
        (uint8_t)ORE_INFO_PARSE_STATUS_OK;
    g_ore_info_debug.rx_count++;
    ack_status = (uint8_t)ORE_INFO_ACK_STATUS_OK;
  }

  (void)OreInfo_SendMineAck(msg_id, ack_status, now_ms);
}

static void OreInfo_ProcessByte(uint8_t byte, uint32_t now_ms) {
  uint8_t count = g_ore_info_debug.parse_index;

  if (count == 0u) {
    if (byte == ORE_INFO_FRAME_HEAD0) {
      ore_info_parse_frame[0] = byte;
      g_ore_info_debug.parse_index = 1u;
    }
    return;
  }

  if (count == 1u) {
    if (byte == ORE_INFO_FRAME_HEAD1) {
      ore_info_parse_frame[1] = byte;
      g_ore_info_debug.parse_index = 2u;
    } else if (byte == ORE_INFO_FRAME_HEAD0) {
      ore_info_parse_frame[0] = byte;
      g_ore_info_debug.parse_index = 1u;
    } else {
      g_ore_info_debug.parse_index = 0u;
    }
    return;
  }

  ore_info_parse_frame[count] = byte;
  count = (uint8_t)(count + 1u);
  g_ore_info_debug.parse_index = count;

  if (count < ORE_INFO_FRAME_SIZE) {
    return;
  }

  g_ore_info_debug.parse_index = 0u;
  OreInfo_ParseFrame(now_ms);
}

static void OreInfo_ParseRxBytes(const uint8_t *rx_buf, uint16_t packet_len,
                                 uint32_t now_ms) {
  if (rx_buf == NULL) {
    return;
  }

  OreInfo_RecordRawRxData(rx_buf, packet_len, now_ms);
  g_ore_info_debug.last_rx_len = (uint8_t)((packet_len > 255u) ? 255u
                                                               : packet_len);
  for (uint16_t i = 0u; i < packet_len; ++i) {
    OreInfo_ProcessByte(rx_buf[i], now_ms);
  }
}

static void OreInfo_HandlePending(uint32_t now_ms) {
  if (ore_info_rx_complete_pending) {
    ore_info_rx_complete_pending = false;
    OreInfo_ParseRxBytes(ore_info_rx_buf, ore_info_rx_len, now_ms);
    ore_info_rx_len = 0u;
  }

  if (ore_info_tx_complete_pending) {
    ore_info_tx_complete_pending = false;
  }

  if (ore_info_error_pending) {
    ore_info_error_pending = false;
    g_ore_info_debug.parse_index = 0u;
    g_ore_info_debug.error_count++;
  }
}

static bool OreInfo_TryStartReceive(uint32_t now_ms) {
  if (g_ore_info_debug.rx_busy) {
    return false;
  }

  UART_HandleTypeDef *huart = BSP_UART_GetHandle(BSP_UART_ORE);
  if (huart != NULL) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
  }

  if (BSP_UART_ReceiveToIdle(BSP_UART_ORE, ore_info_rx_buf,
                             ORE_INFO_RX_BUFFER_SIZE, false) == HAL_OK) {
    g_ore_info_debug.last_rx_start_ms = now_ms;
    g_ore_info_debug.rx_busy = true;
    return true;
  }

  g_ore_info_debug.error_count++;
  return false;
}

static void OreInfo_UpdateAckTimeout(uint32_t now_ms) {
  if (!g_ore_info_debug.ack_pending || g_ore_info_debug.last_rx_ms == 0u) {
    return;
  }

  if ((now_ms - g_ore_info_debug.last_rx_ms) >
      ORE_INFO_ACK_PENDING_TIMEOUT_MS) {
    g_ore_info_debug.ack_pending = false;
    g_ore_info_debug.ack_timeout_count++;
  }
}

void OreInfo_Init(uint32_t now_ms) {
  (void)BSP_UART_RegisterCallback(BSP_UART_ORE, BSP_UART_RX_CPLT_CB,
                                  OreInfo_RxCompleteCallback);
  (void)BSP_UART_RegisterRxEventCallback(BSP_UART_ORE,
                                         OreInfo_RxEventCallback);
  (void)BSP_UART_RegisterCallback(BSP_UART_ORE, BSP_UART_TX_CPLT_CB,
                                  OreInfo_TxCompleteCallback);
  (void)BSP_UART_RegisterCallback(BSP_UART_ORE, BSP_UART_ERROR_CB,
                                  OreInfo_ErrorCallback);

  g_ore_info_debug.inited = true;
  g_ore_info_debug.rx_busy = false;
  g_ore_info_debug.tx_busy = false;
  g_ore_info_debug.ack_pending = false;
  g_ore_info_debug.info_valid = false;
  g_ore_info_debug.info_fresh = false;
  g_ore_info_debug.last_rx_len = 0u;
  g_ore_info_debug.last_tx_len = 0u;
  g_ore_info_debug.last_msg_id = 0u;
  g_ore_info_debug.last_side = 0u;
  g_ore_info_debug.last_parse_status =
      (uint8_t)ORE_INFO_PARSE_STATUS_INVALID;
  g_ore_info_debug.last_ack_status = (uint8_t)ORE_INFO_ACK_STATUS_INVALID;
  g_ore_info_debug.parse_index = 0u;
  OreInfo_ClearTypes();
  OreInfo_ClearRawFrame();
  OreInfo_ClearRawRxData();
  for (uint8_t i = 0u; i < ORE_INFO_ACK_FRAME_SIZE; ++i) {
    g_ore_info_debug.last_ack_frame[i] = 0u;
    ore_info_tx_buf[i] = 0u;
  }
  g_ore_info_debug.last_rx_start_ms = now_ms;
  g_ore_info_debug.last_rx_raw_ms = 0u;
  g_ore_info_debug.last_rx_ms = 0u;
  g_ore_info_debug.last_tx_ms = 0u;
  g_ore_info_debug.last_rx_age_ms = 0u;
  g_ore_info_debug.last_rx_raw_age_ms = 0u;
  (void)OreInfo_TryStartReceive(now_ms);
}

void OreInfo_Process(uint32_t now_ms) {
  if (!g_ore_info_debug.inited) {
    OreInfo_Init(now_ms);
  }

  OreInfo_HandlePending(now_ms);
  g_ore_info_debug.last_rx_raw_age_ms =
      (g_ore_info_debug.last_rx_raw_ms == 0u)
          ? 0u
          : (now_ms - g_ore_info_debug.last_rx_raw_ms);
  OreInfo_UpdateAckTimeout(now_ms);
  g_ore_info_debug.info_fresh = OreInfo_IsFresh(now_ms);
  (void)OreInfo_TryStartReceive(now_ms);
}

OreInfo_AckSubmitResult_t OreInfo_SendAckFrame(
    const uint8_t ack_frame[ORE_INFO_ACK_FRAME_SIZE], uint32_t now_ms) {
  if (ack_frame == NULL) {
    g_ore_info_debug.ack_invalid_count++;
    return ORE_INFO_ACK_SUBMIT_INVALID;
  }

  if (!g_ore_info_debug.inited || g_ore_info_debug.tx_busy) {
    return ORE_INFO_ACK_SUBMIT_BUSY;
  }

  if (ack_frame[0] != ORE_INFO_FRAME_HEAD0 ||
      ack_frame[1] != ORE_INFO_FRAME_HEAD1 ||
      ack_frame[2] != ORE_INFO_CMD_MINE_ACK ||
      !CRC8_Verify(ack_frame, ORE_INFO_ACK_FRAME_SIZE)) {
    g_ore_info_debug.ack_invalid_count++;
    return ORE_INFO_ACK_SUBMIT_INVALID;
  }

  if (!g_ore_info_debug.ack_pending) {
    g_ore_info_debug.ack_no_pending_count++;
    return ORE_INFO_ACK_SUBMIT_NO_PENDING;
  }

  if (ack_frame[3] != g_ore_info_debug.last_msg_id) {
    g_ore_info_debug.ack_mismatch_count++;
    return ORE_INFO_ACK_SUBMIT_MSG_MISMATCH;
  }

  for (uint8_t i = 0u; i < ORE_INFO_ACK_FRAME_SIZE; ++i) {
    ore_info_tx_buf[i] = ack_frame[i];
  }

  if (BSP_UART_Transmit(BSP_UART_ORE, ore_info_tx_buf,
                        ORE_INFO_ACK_FRAME_SIZE, false) == HAL_OK) {
    OreInfo_CopyVolatileBytes(g_ore_info_debug.last_ack_frame, ack_frame,
                              ORE_INFO_ACK_FRAME_SIZE);
    g_ore_info_debug.last_ack_status = ack_frame[4];
    g_ore_info_debug.last_tx_len = ORE_INFO_ACK_FRAME_SIZE;
    g_ore_info_debug.last_tx_ms = now_ms;
    g_ore_info_debug.tx_busy = true;
    g_ore_info_debug.ack_pending = false;
    g_ore_info_debug.tx_count++;
    g_ore_info_debug.ack_tx_count++;
    return ORE_INFO_ACK_SUBMIT_OK;
  }

  g_ore_info_debug.error_count++;
  return ORE_INFO_ACK_SUBMIT_TX_ERROR;
}

bool OreInfo_IsFresh(uint32_t now_ms) {
  if (!g_ore_info_debug.info_valid || g_ore_info_debug.last_rx_ms == 0u) {
    g_ore_info_debug.last_rx_age_ms = 0u;
    return false;
  }

  g_ore_info_debug.last_rx_age_ms = now_ms - g_ore_info_debug.last_rx_ms;
  return g_ore_info_debug.last_rx_age_ms <= ORE_INFO_FRESH_MS;
}

bool OreInfo_GetInfo(OreInfo_Info_t *info, uint32_t now_ms) {
  if (info == NULL) {
    return false;
  }

  info->valid = g_ore_info_debug.info_valid;
  info->fresh = OreInfo_IsFresh(now_ms);
  info->count = ORE_INFO_POSITION_COUNT;
  info->msg_id = g_ore_info_debug.last_msg_id;
  info->side = g_ore_info_debug.last_side;
  info->ack_pending = g_ore_info_debug.ack_pending ? 1u : 0u;
  info->parse_status = g_ore_info_debug.last_parse_status;
  for (uint8_t i = 0u; i < ORE_INFO_POSITION_COUNT; ++i) {
    info->ore_type[i] = g_ore_info_debug.ore_type[i];
  }
  for (uint8_t i = 0u; i < ORE_INFO_FRAME_SIZE; ++i) {
    info->raw_frame[i] = g_ore_info_debug.raw_frame[i];
  }
  info->last_rx_ms = g_ore_info_debug.last_rx_ms;
  info->age_ms = g_ore_info_debug.last_rx_age_ms;
  info->rx_count = g_ore_info_debug.rx_count;
  info->frame_rx_count = g_ore_info_debug.frame_rx_count;
  info->ack_tx_count = g_ore_info_debug.ack_tx_count;
  return info->valid;
}
