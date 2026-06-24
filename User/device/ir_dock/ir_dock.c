#include "device/ir_dock/ir_dock.h"

#include <stddef.h>

#include "bsp/uart.h"
#include "component/crc8.h"

volatile IrDock_Debug_t g_ir_dock_debug = {0};

static uint8_t ir_dock_rx_buf[IR_DOCK_RX_BUFFER_SIZE] = {0};
static uint8_t ir_dock_tx_buf[IR_DOCK_ACK_FRAME_SIZE] = {0};
static uint8_t ir_dock_parse_frame[IR_DOCK_MINE_INPUT_FRAME_SIZE] = {0};
static volatile uint16_t ir_dock_rx_len = 0u;
static volatile bool ir_dock_rx_complete_pending = false;
static volatile bool ir_dock_tx_complete_pending = false;
static volatile bool ir_dock_error_pending = false;

static bool IrDock_SendStatusAck(IrDock_Status_t status, uint32_t now_ms);

static void IrDock_RxEventCallback(uint16_t size) {
  if (size > IR_DOCK_RX_BUFFER_SIZE) {
    size = IR_DOCK_RX_BUFFER_SIZE;
  }
  ir_dock_rx_len = size;
  ir_dock_rx_complete_pending = true;
  g_ir_dock_debug.rx_busy = false;
}

static void IrDock_RxCompleteCallback(void) {
  ir_dock_rx_len = IR_DOCK_RX_BUFFER_SIZE;
  ir_dock_rx_complete_pending = true;
  g_ir_dock_debug.rx_busy = false;
}

static void IrDock_TxCompleteCallback(void) {
  ir_dock_tx_complete_pending = true;
  g_ir_dock_debug.tx_busy = false;
}

static void IrDock_ErrorCallback(void) {
  ir_dock_error_pending = true;
  g_ir_dock_debug.rx_busy = false;
  g_ir_dock_debug.tx_busy = false;
}

static bool IrDock_StatusIsValid(uint8_t status) {
  return status == (uint8_t)IR_DOCK_STATUS_IDLE ||
         status == (uint8_t)IR_DOCK_STATUS_DOCKING ||
         status == (uint8_t)IR_DOCK_STATUS_DOCK_COMPLETE;
}

static bool IrDock_SideIsValid(uint8_t side) {
  return side == (uint8_t)'R' || side == (uint8_t)'B';
}

static bool IrDock_OreTypeIsValid(uint8_t ore_type) {
  return ore_type == (uint8_t)IR_DOCK_ORE_TYPE_UNKNOWN ||
         ore_type == (uint8_t)IR_DOCK_ORE_TYPE_R1 ||
         ore_type == (uint8_t)IR_DOCK_ORE_TYPE_R2 ||
         ore_type == (uint8_t)IR_DOCK_ORE_TYPE_FAKE;
}

static uint8_t IrDock_CountOreType(const uint8_t *ore_type, uint8_t type) {
  uint8_t count = 0u;

  for (uint8_t i = 0u; i < IR_DOCK_ORE_POSITION_COUNT; ++i) {
    if (ore_type[i] == type) {
      count++;
    }
  }
  return count;
}

static bool IrDock_OreInfoIsValid(uint8_t side, const uint8_t *ore_type) {
  if (ore_type == NULL || !IrDock_SideIsValid(side)) {
    return false;
  }

  for (uint8_t i = 0u; i < IR_DOCK_ORE_POSITION_COUNT; ++i) {
    if (!IrDock_OreTypeIsValid(ore_type[i])) {
      return false;
    }
  }

  return IrDock_CountOreType(ore_type, (uint8_t)IR_DOCK_ORE_TYPE_R1) <= 3u &&
         IrDock_CountOreType(ore_type, (uint8_t)IR_DOCK_ORE_TYPE_R2) <= 4u &&
         IrDock_CountOreType(ore_type, (uint8_t)IR_DOCK_ORE_TYPE_FAKE) <= 1u;
}

static void IrDock_ClearOreTypes(void) {
  for (uint8_t i = 0u; i < IR_DOCK_ORE_POSITION_COUNT; ++i) {
    g_ir_dock_debug.ore_type[i] = (uint8_t)IR_DOCK_ORE_TYPE_UNKNOWN;
  }
}

static void IrDock_ClearRawFrame(void) {
  for (uint8_t i = 0u; i < IR_DOCK_MINE_INPUT_FRAME_SIZE; ++i) {
    g_ir_dock_debug.raw_frame[i] = 0u;
    ir_dock_parse_frame[i] = 0u;
  }
}

static void IrDock_CopyVolatileBytes(volatile uint8_t *dst,
                                     const uint8_t *src,
                                     uint8_t len) {
  if (dst == NULL || src == NULL) {
    return;
  }

  for (uint8_t i = 0u; i < len; ++i) {
    dst[i] = src[i];
  }
}

static void IrDock_ParseMineFrame(uint32_t now_ms) {
  const uint8_t msg_id = ir_dock_parse_frame[3];
  const uint8_t side = ir_dock_parse_frame[4];
  const uint8_t *ore_type = &ir_dock_parse_frame[5];

  g_ir_dock_debug.last_rx_len = IR_DOCK_MINE_INPUT_FRAME_SIZE;
  g_ir_dock_debug.last_msg_id = msg_id;
  g_ir_dock_debug.last_side = side;
  IrDock_CopyVolatileBytes(g_ir_dock_debug.raw_frame, ir_dock_parse_frame,
                           IR_DOCK_MINE_INPUT_FRAME_SIZE);
  g_ir_dock_debug.last_rx_ms = now_ms;
  g_ir_dock_debug.frame_rx_count++;

  if (ir_dock_parse_frame[2] != IR_DOCK_CMD_MINE_INPUT) {
    g_ir_dock_debug.last_parse_status = (uint8_t)IR_DOCK_PARSE_STATUS_INVALID;
    g_ir_dock_debug.ore_info_error_count++;
    g_ir_dock_debug.error_count++;
    return;
  }

  g_ir_dock_debug.ack_pending = true;

  if (!CRC8_Verify(ir_dock_parse_frame, IR_DOCK_MINE_INPUT_FRAME_SIZE)) {
    g_ir_dock_debug.last_parse_status = (uint8_t)IR_DOCK_PARSE_STATUS_CRC_ERR;
    g_ir_dock_debug.crc_error_count++;
    g_ir_dock_debug.ore_info_error_count++;
    g_ir_dock_debug.error_count++;
    return;
  }

  if (!IrDock_OreInfoIsValid(side, ore_type)) {
    g_ir_dock_debug.last_parse_status = (uint8_t)IR_DOCK_PARSE_STATUS_INVALID;
    g_ir_dock_debug.ore_info_error_count++;
    g_ir_dock_debug.error_count++;
    return;
  }

  for (uint8_t i = 0u; i < IR_DOCK_ORE_POSITION_COUNT; ++i) {
    g_ir_dock_debug.ore_type[i] = ore_type[i];
  }
  g_ir_dock_debug.last_rx_status = (uint8_t)IR_DOCK_STATUS_DOCK_COMPLETE;
  g_ir_dock_debug.last_ore_rx_ms = now_ms;
  g_ir_dock_debug.ore_info_valid = true;
  g_ir_dock_debug.ore_info_fresh = true;
  g_ir_dock_debug.last_parse_status = (uint8_t)IR_DOCK_PARSE_STATUS_OK;
  g_ir_dock_debug.rx_count++;
  g_ir_dock_debug.ore_info_rx_count++;
}

static void IrDock_ParseStatusByte(uint8_t status, uint32_t now_ms) {
  g_ir_dock_debug.last_rx_status = status;
  g_ir_dock_debug.last_rx_ms = now_ms;
  g_ir_dock_debug.last_parse_status = (uint8_t)IR_DOCK_PARSE_STATUS_OK;
  g_ir_dock_debug.rx_count++;
}

static bool IrDock_PacketIsStatusOnly(uint16_t packet_len) {
  if (packet_len == 0u || g_ir_dock_debug.parse_index != 0u) {
    return false;
  }

  for (uint16_t i = 0u; i < packet_len; ++i) {
    if (!IrDock_StatusIsValid(ir_dock_rx_buf[i])) {
      return false;
    }
  }

  return true;
}

static void IrDock_ProcessByte(uint8_t byte, uint32_t now_ms) {
  uint8_t count = g_ir_dock_debug.parse_index;

  if (count == 0u) {
    if (byte == IR_DOCK_FRAME_HEAD0) {
      ir_dock_parse_frame[0] = byte;
      g_ir_dock_debug.parse_index = 1u;
    }
    return;
  }

  if (count == 1u) {
    if (byte == IR_DOCK_FRAME_HEAD1) {
      ir_dock_parse_frame[1] = byte;
      g_ir_dock_debug.parse_index = 2u;
    } else if (byte == IR_DOCK_FRAME_HEAD0) {
      ir_dock_parse_frame[0] = byte;
      g_ir_dock_debug.parse_index = 1u;
    } else {
      g_ir_dock_debug.parse_index = 0u;
    }
    return;
  }

  ir_dock_parse_frame[count] = byte;
  count = (uint8_t)(count + 1u);
  g_ir_dock_debug.parse_index = count;

  if (count < IR_DOCK_MINE_INPUT_FRAME_SIZE) {
    return;
  }

  g_ir_dock_debug.parse_index = 0u;
  IrDock_ParseMineFrame(now_ms);
}

static void IrDock_ParseRxBytes(uint16_t packet_len, uint32_t now_ms) {
  g_ir_dock_debug.last_rx_len = (uint8_t)((packet_len > 255u) ? 255u
                                                              : packet_len);

  if (IrDock_PacketIsStatusOnly(packet_len)) {
    for (uint16_t i = 0u; i < packet_len; ++i) {
      IrDock_ParseStatusByte(ir_dock_rx_buf[i], now_ms);
    }
    (void)IrDock_SendStatusAck((IrDock_Status_t)ir_dock_rx_buf[packet_len - 1u],
                               now_ms);
    return;
  }

  for (uint16_t i = 0u; i < packet_len; ++i) {
    IrDock_ProcessByte(ir_dock_rx_buf[i], now_ms);
  }
}

static void IrDock_HandlePending(uint32_t now_ms) {
  if (ir_dock_rx_complete_pending) {
    ir_dock_rx_complete_pending = false;
    IrDock_ParseRxBytes(ir_dock_rx_len, now_ms);
    ir_dock_rx_len = 0u;
  }

  if (ir_dock_tx_complete_pending) {
    ir_dock_tx_complete_pending = false;
  }

  if (ir_dock_error_pending) {
    ir_dock_error_pending = false;
    g_ir_dock_debug.parse_index = 0u;
    g_ir_dock_debug.error_count++;
  }
}

static bool IrDock_TryStartReceive(uint32_t now_ms) {
  if (g_ir_dock_debug.rx_busy) {
    return false;
  }

  UART_HandleTypeDef *huart = BSP_UART_GetHandle(BSP_UART_IR);
  if (huart != NULL) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
  }

  if (BSP_UART_ReceiveToIdle(BSP_UART_IR, ir_dock_rx_buf,
                             IR_DOCK_RX_BUFFER_SIZE, false) == HAL_OK) {
    g_ir_dock_debug.last_rx_start_ms = now_ms;
    g_ir_dock_debug.rx_busy = true;
    return true;
  }

  g_ir_dock_debug.error_count++;
  return false;
}

static void IrDock_UpdateAckTimeout(uint32_t now_ms) {
  if (!g_ir_dock_debug.ack_pending || g_ir_dock_debug.last_rx_ms == 0u) {
    return;
  }

  if ((now_ms - g_ir_dock_debug.last_rx_ms) >
      IR_DOCK_ACK_PENDING_TIMEOUT_MS) {
    g_ir_dock_debug.ack_pending = false;
    g_ir_dock_debug.ack_timeout_count++;
  }
}

static void IrDock_BuildAck(uint8_t msg_id, uint8_t status,
                            uint8_t ack_frame[IR_DOCK_ACK_FRAME_SIZE]) {
  ack_frame[0] = IR_DOCK_FRAME_HEAD0;
  ack_frame[1] = IR_DOCK_FRAME_HEAD1;
  ack_frame[2] = IR_DOCK_CMD_MINE_ACK;
  ack_frame[3] = msg_id;
  ack_frame[4] = status;
  ack_frame[IR_DOCK_ACK_CRC_OFFSET] =
      CRC8_Calc(ack_frame, IR_DOCK_ACK_CRC_OFFSET, CRC8_INIT);
}

static bool IrDock_SendStatusAck(IrDock_Status_t status, uint32_t now_ms) {
  if (!IrDock_StatusIsValid((uint8_t)status)) {
    g_ir_dock_debug.ack_invalid_count++;
    return false;
  }

  if (!g_ir_dock_debug.inited || g_ir_dock_debug.tx_busy) {
    g_ir_dock_debug.tx_interval_block_count++;
    return false;
  }

  for (uint8_t i = 0u; i < IR_DOCK_ACK_FRAME_SIZE; ++i) {
    ir_dock_tx_buf[i] = 0u;
    g_ir_dock_debug.last_ack_frame[i] = 0u;
  }
  ir_dock_tx_buf[0] = IR_DOCK_STATUS_ACK_HEAD;
  ir_dock_tx_buf[1] = (uint8_t)status;

  if (BSP_UART_Transmit(BSP_UART_IR, ir_dock_tx_buf,
                        IR_DOCK_STATUS_ACK_FRAME_SIZE, false) == HAL_OK) {
    IrDock_CopyVolatileBytes(g_ir_dock_debug.last_ack_frame, ir_dock_tx_buf,
                             IR_DOCK_STATUS_ACK_FRAME_SIZE);
    g_ir_dock_debug.last_ack_status = (uint8_t)status;
    g_ir_dock_debug.last_tx_status = (uint8_t)status;
    g_ir_dock_debug.last_tx_len = IR_DOCK_STATUS_ACK_FRAME_SIZE;
    g_ir_dock_debug.last_tx_ms = now_ms;
    g_ir_dock_debug.tx_busy = true;
    g_ir_dock_debug.tx_count++;
    g_ir_dock_debug.ack_tx_count++;
    g_ir_dock_debug.status_ack_tx_count++;
    return true;
  }

  g_ir_dock_debug.error_count++;
  return false;
}

void IrDock_Init(uint32_t now_ms) {
  (void)BSP_UART_RegisterCallback(BSP_UART_IR, BSP_UART_RX_CPLT_CB,
                                  IrDock_RxCompleteCallback);
  (void)BSP_UART_RegisterRxEventCallback(BSP_UART_IR, IrDock_RxEventCallback);
  (void)BSP_UART_RegisterCallback(BSP_UART_IR, BSP_UART_TX_CPLT_CB,
                                  IrDock_TxCompleteCallback);
  (void)BSP_UART_RegisterCallback(BSP_UART_IR, BSP_UART_ERROR_CB,
                                  IrDock_ErrorCallback);

  g_ir_dock_debug.inited = true;
  g_ir_dock_debug.rx_busy = false;
  g_ir_dock_debug.tx_busy = false;
  g_ir_dock_debug.ack_pending = false;
  g_ir_dock_debug.dock_complete_fresh = false;
  g_ir_dock_debug.ore_info_valid = false;
  g_ir_dock_debug.ore_info_fresh = false;
  g_ir_dock_debug.last_rx_status = (uint8_t)IR_DOCK_STATUS_IDLE;
  g_ir_dock_debug.last_tx_status = (uint8_t)IR_DOCK_STATUS_IDLE;
  g_ir_dock_debug.last_rx_len = 0u;
  g_ir_dock_debug.last_tx_len = 0u;
  g_ir_dock_debug.last_msg_id = 0u;
  g_ir_dock_debug.last_side = 0u;
  g_ir_dock_debug.last_parse_status = (uint8_t)IR_DOCK_PARSE_STATUS_INVALID;
  g_ir_dock_debug.last_ack_status = (uint8_t)IR_DOCK_ACK_STATUS_INVALID;
  g_ir_dock_debug.parse_index = 0u;
  IrDock_ClearOreTypes();
  IrDock_ClearRawFrame();
  for (uint8_t i = 0u; i < IR_DOCK_ACK_FRAME_SIZE; ++i) {
    g_ir_dock_debug.last_ack_frame[i] = 0u;
    ir_dock_tx_buf[i] = 0u;
  }
  g_ir_dock_debug.last_rx_start_ms = now_ms;
  g_ir_dock_debug.last_rx_ms = 0u;
  g_ir_dock_debug.last_ore_rx_ms = 0u;
  g_ir_dock_debug.last_tx_ms = 0u;
  g_ir_dock_debug.last_rx_age_ms = 0u;
  g_ir_dock_debug.last_ore_rx_age_ms = 0u;
  (void)IrDock_TryStartReceive(now_ms);
}

void IrDock_Process(uint32_t now_ms) {
  if (!g_ir_dock_debug.inited) {
    IrDock_Init(now_ms);
  }

  IrDock_HandlePending(now_ms);
  IrDock_UpdateAckTimeout(now_ms);
  g_ir_dock_debug.dock_complete_fresh = IrDock_IsDockCompleteFresh(now_ms);
  g_ir_dock_debug.ore_info_fresh = IrDock_IsOreInfoFresh(now_ms);
  (void)IrDock_TryStartReceive(now_ms);
}

bool IrDock_SendStatus(IrDock_Status_t status, uint32_t now_ms) {
  if (!IrDock_StatusIsValid((uint8_t)status)) {
    return false;
  }

  uint8_t ack_status = (status == IR_DOCK_STATUS_IDLE)
                           ? (uint8_t)IR_DOCK_ACK_STATUS_OK
                           : (uint8_t)IR_DOCK_ACK_STATUS_BUSY;
  uint8_t ack_frame[IR_DOCK_ACK_FRAME_SIZE];
  IrDock_BuildAck(g_ir_dock_debug.last_msg_id, ack_status, ack_frame);
  return IrDock_SendAckFrame(ack_frame, now_ms) == IR_DOCK_ACK_SUBMIT_OK;
}

bool IrDock_SendOreInfo(IrDock_Status_t status,
                        const uint8_t ore_type[IR_DOCK_ORE_POSITION_COUNT],
                        uint32_t now_ms) {
  (void)ore_type;
  return IrDock_SendStatus(status, now_ms);
}

IrDock_AckSubmitResult_t IrDock_SendAckFrame(
    const uint8_t ack_frame[IR_DOCK_ACK_FRAME_SIZE], uint32_t now_ms) {
  if (ack_frame == NULL) {
    g_ir_dock_debug.ack_invalid_count++;
    return IR_DOCK_ACK_SUBMIT_INVALID;
  }

  if (!g_ir_dock_debug.inited || g_ir_dock_debug.tx_busy) {
    return IR_DOCK_ACK_SUBMIT_BUSY;
  }

  if (ack_frame[0] != IR_DOCK_FRAME_HEAD0 ||
      ack_frame[1] != IR_DOCK_FRAME_HEAD1 ||
      ack_frame[2] != IR_DOCK_CMD_MINE_ACK ||
      !CRC8_Verify(ack_frame, IR_DOCK_ACK_FRAME_SIZE)) {
    g_ir_dock_debug.ack_invalid_count++;
    return IR_DOCK_ACK_SUBMIT_INVALID;
  }

  if (!g_ir_dock_debug.ack_pending) {
    g_ir_dock_debug.ack_no_pending_count++;
    return IR_DOCK_ACK_SUBMIT_NO_PENDING;
  }

  if (ack_frame[3] != g_ir_dock_debug.last_msg_id) {
    g_ir_dock_debug.ack_mismatch_count++;
    return IR_DOCK_ACK_SUBMIT_MSG_MISMATCH;
  }

  for (uint8_t i = 0u; i < IR_DOCK_ACK_FRAME_SIZE; ++i) {
    ir_dock_tx_buf[i] = ack_frame[i];
  }

  if (BSP_UART_Transmit(BSP_UART_IR, ir_dock_tx_buf,
                        IR_DOCK_ACK_FRAME_SIZE, false) == HAL_OK) {
    IrDock_CopyVolatileBytes(g_ir_dock_debug.last_ack_frame, ack_frame,
                             IR_DOCK_ACK_FRAME_SIZE);
    g_ir_dock_debug.last_ack_status = ack_frame[4];
    g_ir_dock_debug.last_tx_status = ack_frame[4];
    g_ir_dock_debug.last_tx_len = IR_DOCK_ACK_FRAME_SIZE;
    g_ir_dock_debug.last_tx_ms = now_ms;
    g_ir_dock_debug.tx_busy = true;
    g_ir_dock_debug.ack_pending = false;
    g_ir_dock_debug.tx_count++;
    g_ir_dock_debug.ack_tx_count++;
    return IR_DOCK_ACK_SUBMIT_OK;
  }

  g_ir_dock_debug.error_count++;
  return IR_DOCK_ACK_SUBMIT_TX_ERROR;
}

bool IrDock_IsDockCompleteFresh(uint32_t now_ms) {
  if (g_ir_dock_debug.last_rx_status != (uint8_t)IR_DOCK_STATUS_DOCK_COMPLETE ||
      g_ir_dock_debug.last_rx_ms == 0u) {
    g_ir_dock_debug.last_rx_age_ms = 0u;
    return false;
  }

  g_ir_dock_debug.last_rx_age_ms = now_ms - g_ir_dock_debug.last_rx_ms;
  return g_ir_dock_debug.last_rx_age_ms <= IR_DOCK_COMPLETE_FRESH_MS;
}

bool IrDock_IsOreInfoFresh(uint32_t now_ms) {
  if (!g_ir_dock_debug.ore_info_valid ||
      g_ir_dock_debug.last_ore_rx_ms == 0u) {
    g_ir_dock_debug.last_ore_rx_age_ms = 0u;
    return false;
  }

  g_ir_dock_debug.last_ore_rx_age_ms = now_ms - g_ir_dock_debug.last_ore_rx_ms;
  return g_ir_dock_debug.last_ore_rx_age_ms <= IR_DOCK_ORE_INFO_FRESH_MS;
}

IrDock_Status_t IrDock_GetLastRxStatus(void) {
  return (IrDock_Status_t)g_ir_dock_debug.last_rx_status;
}

bool IrDock_GetOreInfo(IrDock_OreInfo_t *info, uint32_t now_ms) {
  if (info == NULL) {
    return false;
  }

  info->valid = g_ir_dock_debug.ore_info_valid;
  info->fresh = IrDock_IsOreInfoFresh(now_ms);
  info->status = g_ir_dock_debug.last_rx_status;
  info->count = IR_DOCK_ORE_POSITION_COUNT;
  info->msg_id = g_ir_dock_debug.last_msg_id;
  info->side = g_ir_dock_debug.last_side;
  info->ack_pending = g_ir_dock_debug.ack_pending ? 1u : 0u;
  info->parse_status = g_ir_dock_debug.last_parse_status;
  for (uint8_t i = 0u; i < IR_DOCK_ORE_POSITION_COUNT; ++i) {
    info->ore_type[i] = g_ir_dock_debug.ore_type[i];
  }
  for (uint8_t i = 0u; i < IR_DOCK_MINE_INPUT_FRAME_SIZE; ++i) {
    info->raw_frame[i] = g_ir_dock_debug.raw_frame[i];
  }
  info->last_rx_ms = g_ir_dock_debug.last_ore_rx_ms;
  info->age_ms = g_ir_dock_debug.last_ore_rx_age_ms;
  info->rx_count = g_ir_dock_debug.ore_info_rx_count;
  info->frame_rx_count = g_ir_dock_debug.frame_rx_count;
  info->ack_tx_count = g_ir_dock_debug.ack_tx_count;
  return info->valid;
}
