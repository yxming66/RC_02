#include "device/ir_dock/ir_dock.h"

#include <stddef.h>

#include "bsp/uart.h"

static uint8_t ir_dock_rx_buf[IR_DOCK_RX_BUFFER_SIZE] = {0};
static uint8_t ir_dock_tx_buf[IR_DOCK_STATUS_ACK_FRAME_SIZE] = {0};
static volatile uint16_t ir_dock_rx_len = 0u;
static volatile bool ir_dock_rx_complete_pending = false;
static volatile bool ir_dock_tx_complete_pending = false;
static volatile bool ir_dock_error_pending = false;

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
  return status == IR_DOCK_CMD_ONLINE || status == IR_DOCK_CMD_COMPLETE;
}

static void IrDock_ClearRawRxData(void) {
  for (uint8_t i = 0u; i < IR_DOCK_RX_BUFFER_SIZE; ++i) {
    g_ir_dock_debug.last_rx_raw_data[i] = 0u;
  }
  g_ir_dock_debug.last_rx_raw_len = 0u;
}

static void IrDock_RecordRawRxData(const uint8_t *rx_buf, uint16_t packet_len,
                                   uint32_t now_ms) {
  if (rx_buf == NULL) {
    return;
  }

  uint16_t copy_len = packet_len;
  if (copy_len > IR_DOCK_RX_BUFFER_SIZE) {
    copy_len = IR_DOCK_RX_BUFFER_SIZE;
  }

  IrDock_ClearRawRxData();
  g_ir_dock_debug.last_rx_raw_len = (uint8_t)copy_len;
  g_ir_dock_debug.last_rx_raw_ms = now_ms;
  for (uint16_t i = 0u; i < copy_len; ++i) {
    g_ir_dock_debug.last_rx_raw_data[i] = rx_buf[i];
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

static bool IrDock_SendCommandAck(uint8_t cmd, uint32_t now_ms) {
  if (!g_ir_dock_debug.inited || g_ir_dock_debug.tx_busy) {
    g_ir_dock_debug.tx_interval_block_count++;
    return false;
  }

  ir_dock_tx_buf[0] = IR_DOCK_STATUS_ACK_HEAD;
  ir_dock_tx_buf[1] = cmd;

  if (BSP_UART_Transmit(BSP_UART_IR, ir_dock_tx_buf,
                        IR_DOCK_STATUS_ACK_FRAME_SIZE, false) == HAL_OK) {
    IrDock_CopyVolatileBytes(g_ir_dock_debug.last_ack_frame, ir_dock_tx_buf,
                             IR_DOCK_STATUS_ACK_FRAME_SIZE);
    g_ir_dock_debug.last_tx_status = cmd;
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

static void IrDock_ParseStatusByte(uint8_t raw_byte, uint32_t now_ms) {
  g_ir_dock_debug.last_rx_raw_byte = raw_byte;
  g_ir_dock_debug.last_rx_ms = now_ms;
  g_ir_dock_debug.rx_count++;

  if (!IrDock_StatusIsValid(raw_byte)) {
    g_ir_dock_debug.invalid_rx_count++;
    g_ir_dock_debug.error_count++;
    return;
  }

  g_ir_dock_debug.last_rx_status = raw_byte;
  if (raw_byte == IR_DOCK_CMD_ONLINE) {
    g_ir_dock_debug.last_online_rx_ms = now_ms;
    g_ir_dock_debug.online = true;
    g_ir_dock_debug.online_rx_count++;
  } else if (raw_byte == IR_DOCK_CMD_COMPLETE) {
    g_ir_dock_debug.last_complete_rx_ms = now_ms;
    g_ir_dock_debug.complete_rx_count++;
  }

  (void)IrDock_SendCommandAck(raw_byte, now_ms);
}

static void IrDock_ParseRxBytes(const uint8_t *rx_buf, uint16_t packet_len,
                                uint32_t now_ms) {
  if (rx_buf == NULL) {
    return;
  }

  IrDock_RecordRawRxData(rx_buf, packet_len, now_ms);
  g_ir_dock_debug.last_rx_len = (uint8_t)((packet_len > 255u) ? 255u
                                                              : packet_len);
  if (packet_len > 0u) {
    g_ir_dock_debug.last_rx_raw_byte = rx_buf[packet_len - 1u];
  }

  for (uint16_t i = 0u; i < packet_len; ++i) {
    IrDock_ParseStatusByte(rx_buf[i], now_ms);
  }
}

static void IrDock_HandlePending(uint32_t now_ms) {
  if (ir_dock_rx_complete_pending) {
    ir_dock_rx_complete_pending = false;
    IrDock_ParseRxBytes(ir_dock_rx_buf, ir_dock_rx_len, now_ms);
    ir_dock_rx_len = 0u;
  }

  if (ir_dock_tx_complete_pending) {
    ir_dock_tx_complete_pending = false;
  }

  if (ir_dock_error_pending) {
    ir_dock_error_pending = false;
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

void IrDock_Init(uint32_t now_ms) {
  (void)BSP_UART_RegisterCallback(BSP_UART_IR, BSP_UART_RX_CPLT_CB,
                                  IrDock_RxCompleteCallback);
  (void)BSP_UART_RegisterRxEventCallback(BSP_UART_IR, IrDock_RxEventCallback);
  (void)BSP_UART_RegisterCallback(BSP_UART_IR, BSP_UART_TX_CPLT_CB,
                                  IrDock_TxCompleteCallback);
  (void)BSP_UART_RegisterCallback(BSP_UART_IR, BSP_UART_ERROR_CB,
                                  IrDock_ErrorCallback);

  g_ir_dock_debug.inited = true;
  g_ir_dock_debug.online = false;
  g_ir_dock_debug.rx_busy = false;
  g_ir_dock_debug.tx_busy = false;
  g_ir_dock_debug.dock_complete_fresh = false;
  g_ir_dock_debug.last_rx_status = (uint8_t)IR_DOCK_STATUS_IDLE;
  g_ir_dock_debug.last_rx_raw_byte = 0u;
  g_ir_dock_debug.last_tx_status = (uint8_t)IR_DOCK_STATUS_IDLE;
  g_ir_dock_debug.last_rx_len = 0u;
  g_ir_dock_debug.last_tx_len = 0u;
  IrDock_ClearRawRxData();
  for (uint8_t i = 0u; i < IR_DOCK_STATUS_ACK_FRAME_SIZE; ++i) {
    g_ir_dock_debug.last_ack_frame[i] = 0u;
    ir_dock_tx_buf[i] = 0u;
  }
  g_ir_dock_debug.last_rx_start_ms = now_ms;
  g_ir_dock_debug.last_rx_raw_ms = 0u;
  g_ir_dock_debug.last_online_rx_ms = 0u;
  g_ir_dock_debug.last_complete_rx_ms = 0u;
  g_ir_dock_debug.last_rx_ms = 0u;
  g_ir_dock_debug.last_tx_ms = 0u;
  g_ir_dock_debug.last_rx_age_ms = 0u;
  g_ir_dock_debug.last_online_age_ms = 0u;
  g_ir_dock_debug.last_complete_age_ms = 0u;
  g_ir_dock_debug.last_rx_raw_age_ms = 0u;
  (void)IrDock_TryStartReceive(now_ms);
}

void IrDock_Process(uint32_t now_ms) {
  if (!g_ir_dock_debug.inited) {
    IrDock_Init(now_ms);
  }

  IrDock_HandlePending(now_ms);
  g_ir_dock_debug.last_rx_raw_age_ms =
      (g_ir_dock_debug.last_rx_raw_ms == 0u)
          ? 0u
          : (now_ms - g_ir_dock_debug.last_rx_raw_ms);
    g_ir_dock_debug.last_rx_age_ms =
      (g_ir_dock_debug.last_rx_ms == 0u)
        ? 0u
        : (now_ms - g_ir_dock_debug.last_rx_ms);
  g_ir_dock_debug.dock_complete_fresh = IrDock_IsDockCompleteFresh(now_ms);
  g_ir_dock_debug.online = IrDock_IsOnline(now_ms);
  (void)IrDock_TryStartReceive(now_ms);
}

bool IrDock_SendStatus(IrDock_Status_t status, uint32_t now_ms) {
  if (!IrDock_StatusIsValid((uint8_t)status)) {
    return false;
  }
  return IrDock_SendCommandAck((uint8_t)status, now_ms);
}

bool IrDock_IsDockCompleteFresh(uint32_t now_ms) {
  if (g_ir_dock_debug.last_complete_rx_ms == 0u) {
    g_ir_dock_debug.last_complete_age_ms = 0u;
    return false;
  }

  g_ir_dock_debug.last_complete_age_ms =
      now_ms - g_ir_dock_debug.last_complete_rx_ms;
  return g_ir_dock_debug.last_complete_age_ms <= IR_DOCK_COMPLETE_FRESH_MS;
}

bool IrDock_IsOnline(uint32_t now_ms) {
  if (g_ir_dock_debug.last_online_rx_ms == 0u) {
    g_ir_dock_debug.last_online_age_ms = 0u;
    return false;
  }

  g_ir_dock_debug.last_online_age_ms =
      now_ms - g_ir_dock_debug.last_online_rx_ms;
  return g_ir_dock_debug.last_online_age_ms <= IR_DOCK_ONLINE_TIMEOUT_MS;
}

IrDock_Status_t IrDock_GetLastRxStatus(void) {
  return (IrDock_Status_t)g_ir_dock_debug.last_rx_status;
}
