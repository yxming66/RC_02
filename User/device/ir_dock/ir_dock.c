#include "device/ir_dock/ir_dock.h"

#include <stddef.h>

#include "bsp/uart.h"

typedef struct {
  BSP_UART_t uart;
  uint8_t rx_buf[IR_DOCK_RX_BUFFER_SIZE];
  volatile uint16_t rx_len;
  volatile bool rx_complete_pending;
  volatile bool error_pending;
  volatile bool ack_tx_busy;
  volatile uint8_t ack_pending_count;
  uint8_t ack_tx_byte;
  bool frame_head_pending;
} IrDock_Channel_t;

static IrDock_Channel_t ir_dock_channel[IR_DOCK_SOURCE_COUNT] = {
    [IR_DOCK_SOURCE_UART8] = {.uart = BSP_UART_IR},
    [IR_DOCK_SOURCE_UART7] = {.uart = BSP_UART_IR_AUX},
};

static bool IrDock_IsClawOpenFreshInternal(uint32_t now_ms);

static void IrDock_UpdateAckPendingDebug(void) {
  uint16_t total = 0u;
  for (uint8_t source = 0u; source < IR_DOCK_SOURCE_COUNT; ++source) {
    const uint8_t pending = __atomic_load_n(
        &ir_dock_channel[source].ack_pending_count, __ATOMIC_ACQUIRE);
    g_ir_dock_debug.channel[source].ack_pending_count = pending;
    total += pending;
  }
  g_ir_dock_debug.ack_pending_count =
      (uint8_t)((total > UINT8_MAX) ? UINT8_MAX : total);
}

static void IrDock_QueueAck(IrDock_Source_t source) {
  IrDock_Channel_t *channel = &ir_dock_channel[source];
  uint8_t pending = __atomic_load_n(&channel->ack_pending_count,
                                    __ATOMIC_ACQUIRE);
  while (pending != UINT8_MAX) {
    const uint8_t next = (uint8_t)(pending + 1u);
    if (__atomic_compare_exchange_n(&channel->ack_pending_count, &pending,
                                    next, true, __ATOMIC_ACQ_REL,
                                    __ATOMIC_ACQUIRE)) {
      g_ir_dock_debug.channel[source].ack_queued_count++;
      g_ir_dock_debug.ack_queued_count++;
      IrDock_UpdateAckPendingDebug();
      return;
    }
  }

  g_ir_dock_debug.channel[source].ack_drop_count++;
  g_ir_dock_debug.ack_drop_count++;
}

static void IrDock_TxComplete(IrDock_Source_t source) {
  IrDock_Channel_t *channel = &ir_dock_channel[source];
  uint8_t pending = __atomic_load_n(&channel->ack_pending_count,
                                    __ATOMIC_ACQUIRE);
  while (pending > 0u) {
    const uint8_t next = (uint8_t)(pending - 1u);
    if (__atomic_compare_exchange_n(&channel->ack_pending_count, &pending,
                                    next, true, __ATOMIC_ACQ_REL,
                                    __ATOMIC_ACQUIRE)) {
      break;
    }
  }
  __atomic_store_n(&channel->ack_tx_busy, false, __ATOMIC_RELEASE);
  g_ir_dock_debug.channel[source].tx_busy = false;
  g_ir_dock_debug.channel[source].last_ack_tx_byte = channel->ack_tx_byte;
  g_ir_dock_debug.channel[source].ack_tx_count++;
  g_ir_dock_debug.last_ack_tx_source = (uint8_t)source;
  g_ir_dock_debug.last_ack_tx_byte = channel->ack_tx_byte;
  g_ir_dock_debug.ack_tx_count++;
  IrDock_UpdateAckPendingDebug();
}

static void IrDock_Uart8TxCompleteCallback(void) {
  IrDock_TxComplete(IR_DOCK_SOURCE_UART8);
}

static void IrDock_Uart7TxCompleteCallback(void) {
  IrDock_TxComplete(IR_DOCK_SOURCE_UART7);
}

static void IrDock_RxEvent(IrDock_Source_t source, uint16_t size) {
  if (size > IR_DOCK_RX_BUFFER_SIZE) {
    size = IR_DOCK_RX_BUFFER_SIZE;
  }
  ir_dock_channel[source].rx_len = size;
  ir_dock_channel[source].rx_complete_pending = true;
  g_ir_dock_debug.channel[source].rx_busy = false;
}

static void IrDock_RxComplete(IrDock_Source_t source) {
  ir_dock_channel[source].rx_len = IR_DOCK_RX_BUFFER_SIZE;
  ir_dock_channel[source].rx_complete_pending = true;
  g_ir_dock_debug.channel[source].rx_busy = false;
}

static void IrDock_Error(IrDock_Source_t source) {
  IrDock_Channel_t *channel = &ir_dock_channel[source];
  channel->error_pending = true;
  g_ir_dock_debug.channel[source].rx_busy = false;
  UART_HandleTypeDef *huart = BSP_UART_GetHandle(channel->uart);
  if (__atomic_load_n(&channel->ack_tx_busy, __ATOMIC_ACQUIRE) &&
      (huart == NULL || huart->gState == HAL_UART_STATE_READY)) {
    __atomic_store_n(&channel->ack_tx_busy, false, __ATOMIC_RELEASE);
    g_ir_dock_debug.channel[source].tx_busy = false;
    g_ir_dock_debug.channel[source].ack_tx_error_count++;
    g_ir_dock_debug.ack_tx_error_count++;
  }
}

static void IrDock_Uart8RxEventCallback(uint16_t size) {
  IrDock_RxEvent(IR_DOCK_SOURCE_UART8, size);
}

static void IrDock_Uart7RxEventCallback(uint16_t size) {
  IrDock_RxEvent(IR_DOCK_SOURCE_UART7, size);
}

static void IrDock_Uart8RxCompleteCallback(void) {
  IrDock_RxComplete(IR_DOCK_SOURCE_UART8);
}

static void IrDock_Uart7RxCompleteCallback(void) {
  IrDock_RxComplete(IR_DOCK_SOURCE_UART7);
}

static void IrDock_Uart8ErrorCallback(void) {
  IrDock_Error(IR_DOCK_SOURCE_UART8);
}

static void IrDock_Uart7ErrorCallback(void) {
  IrDock_Error(IR_DOCK_SOURCE_UART7);
}

static bool IrDock_TrySendAck(IrDock_Source_t source) {
  IrDock_Channel_t *channel = &ir_dock_channel[source];
  if (__atomic_load_n(&channel->ack_pending_count, __ATOMIC_ACQUIRE) == 0u) {
    return false;
  }

  bool expected = false;
  if (!__atomic_compare_exchange_n(&channel->ack_tx_busy, &expected, true,
                                   false, __ATOMIC_ACQ_REL,
                                   __ATOMIC_ACQUIRE)) {
    return false;
  }

  channel->ack_tx_byte = IR_DOCK_ACK_BYTE;
  g_ir_dock_debug.channel[source].tx_busy = true;
  if (BSP_UART_Transmit(channel->uart, &channel->ack_tx_byte, 1u, false) ==
      HAL_OK) {
    return true;
  }

  __atomic_store_n(&channel->ack_tx_busy, false, __ATOMIC_RELEASE);
  g_ir_dock_debug.channel[source].tx_busy = false;
  g_ir_dock_debug.channel[source].ack_tx_error_count++;
  g_ir_dock_debug.ack_tx_error_count++;
  return false;
}

static bool IrDock_CommandIsValid(uint8_t command) {
  return command >= IR_DOCK_CMD_ONLINE &&
         command <= IR_DOCK_CMD_ZONE3_ACTION_FINISH;
}

static void IrDock_ClearRawRxData(IrDock_Source_t source) {
  for (uint8_t i = 0u; i < IR_DOCK_RX_BUFFER_SIZE; ++i) {
    g_ir_dock_debug.channel[source].last_rx_raw_data[i] = 0u;
    g_ir_dock_debug.last_rx_raw_data[i] = 0u;
  }
  g_ir_dock_debug.channel[source].last_rx_raw_len = 0u;
  g_ir_dock_debug.last_rx_raw_len = 0u;
}

static void IrDock_RecordRawRxData(IrDock_Source_t source,
                                   const uint8_t *rx_buf,
                                   uint16_t packet_len, uint32_t now_ms) {
  if (rx_buf == NULL) {
    return;
  }

  uint16_t copy_len = packet_len;
  if (copy_len > IR_DOCK_RX_BUFFER_SIZE) {
    copy_len = IR_DOCK_RX_BUFFER_SIZE;
  }

  IrDock_ClearRawRxData(source);
  g_ir_dock_debug.channel[source].last_rx_raw_len = (uint8_t)copy_len;
  g_ir_dock_debug.channel[source].last_rx_raw_ms = now_ms;
  g_ir_dock_debug.last_rx_raw_len = (uint8_t)copy_len;
  g_ir_dock_debug.last_rx_raw_ms = now_ms;
  for (uint16_t i = 0u; i < copy_len; ++i) {
    g_ir_dock_debug.channel[source].last_rx_raw_data[i] = rx_buf[i];
    g_ir_dock_debug.last_rx_raw_data[i] = rx_buf[i];
  }
}

static void IrDock_ParseProtocolFrame(IrDock_Source_t source, uint8_t command,
                                      uint32_t now_ms) {
  volatile IrDock_ChannelDebug_t *channel_debug =
      &g_ir_dock_debug.channel[source];
  channel_debug->last_rx_raw_byte = command;
  channel_debug->last_rx_ms = now_ms;
  channel_debug->rx_count++;
  g_ir_dock_debug.last_rx_raw_byte = command;
  g_ir_dock_debug.last_rx_ms = now_ms;
  g_ir_dock_debug.last_rx_source = (uint8_t)source;
  g_ir_dock_debug.rx_count++;

  if (!IrDock_CommandIsValid(command)) {
    channel_debug->invalid_rx_count++;
    channel_debug->error_count++;
    g_ir_dock_debug.invalid_rx_count++;
    g_ir_dock_debug.error_count++;
    return;
  }

  channel_debug->last_rx_status = command;
  channel_debug->last_online_rx_ms = now_ms;
  channel_debug->online = true;
  channel_debug->online_rx_count++;
  channel_debug->protocol_frame_rx_count++;
  g_ir_dock_debug.last_rx_status = command;
  g_ir_dock_debug.last_online_rx_ms = now_ms;
  g_ir_dock_debug.online = true;
  g_ir_dock_debug.online_rx_count++;
  g_ir_dock_debug.protocol_frame_rx_count++;

  switch (command) {
  case IR_DOCK_CMD_DOCK_COMPLETE:
    g_ir_dock_debug.last_dock_complete_cmd = 1u;
    g_ir_dock_debug.last_complete_rx_ms = now_ms;
    g_ir_dock_debug.complete_rx_count++;
    IrDock_QueueAck(source);
    break;

  case IR_DOCK_CMD_RELEASE_ALLOW:
    if (g_ir_dock_debug.zone3_action_locked &&
        !g_ir_dock_debug.release_abort_latched) {
      g_ir_dock_debug.last_claw_open_cmd = IR_DOCK_CLAW_OPEN_ALLOW;
      g_ir_dock_debug.last_claw_open_rx_ms = now_ms;
      g_ir_dock_debug.claw_open = true;
      g_ir_dock_debug.claw_open_rx_count++;
    }
    break;

  case IR_DOCK_CMD_ZONE3_ACTION_FINISH:
    if (g_ir_dock_debug.zone3_action_locked) {
      g_ir_dock_debug.zone3_action_locked = false;
      g_ir_dock_debug.zone3_action_finish_rx_count++;
    }
    g_ir_dock_debug.zone3_action_state = IR_DOCK_ZONE3_ACTION_FINISHED;
    g_ir_dock_debug.last_zone3_action_cmd =
        IR_DOCK_ZONE3_ACTION_CMD_FINISH;
    g_ir_dock_debug.last_claw_open_cmd = IR_DOCK_CLAW_OPEN_WAIT;
    g_ir_dock_debug.last_claw_open_rx_ms = 0u;
    g_ir_dock_debug.last_claw_open_age_ms = 0u;
    g_ir_dock_debug.claw_open = false;
    g_ir_dock_debug.release_abort_latched = false;
    break;

  case IR_DOCK_CMD_ONLINE:
  default:
    break;
  }
}

static void IrDock_ParseRxBytes(IrDock_Source_t source, const uint8_t *rx_buf,
                                uint16_t packet_len, uint32_t now_ms) {
  if (rx_buf == NULL) {
    return;
  }

  IrDock_Channel_t *channel = &ir_dock_channel[source];
  volatile IrDock_ChannelDebug_t *channel_debug =
      &g_ir_dock_debug.channel[source];
  IrDock_RecordRawRxData(source, rx_buf, packet_len, now_ms);
  channel_debug->last_rx_len =
      (uint8_t)((packet_len > 255u) ? 255u : packet_len);
  g_ir_dock_debug.last_rx_len = (uint8_t)((packet_len > 255u) ? 255u
                                                              : packet_len);
  if (packet_len > 0u) {
    channel_debug->last_rx_raw_byte = rx_buf[packet_len - 1u];
    g_ir_dock_debug.last_rx_raw_byte = rx_buf[packet_len - 1u];
  }

  for (uint16_t offset = 0u; offset < packet_len; ++offset) {
    const uint8_t rx_byte = rx_buf[offset];
    if (!channel->frame_head_pending) {
      if (rx_byte == IR_DOCK_FRAME_HEAD) {
        channel->frame_head_pending = true;
      } else {
        channel_debug->invalid_rx_count++;
        channel_debug->error_count++;
        g_ir_dock_debug.invalid_rx_count++;
        g_ir_dock_debug.error_count++;
      }
      continue;
    }

    if (rx_byte == IR_DOCK_FRAME_HEAD) {
      continue;
    }

    channel->frame_head_pending = false;
    IrDock_ParseProtocolFrame(source, rx_byte, now_ms);
  }
}

static void IrDock_HandlePending(uint32_t now_ms) {
  for (uint8_t source = 0u; source < IR_DOCK_SOURCE_COUNT; ++source) {
    IrDock_Channel_t *channel = &ir_dock_channel[source];
    volatile IrDock_ChannelDebug_t *channel_debug =
      &g_ir_dock_debug.channel[source];
    if (channel->rx_complete_pending) {
      channel->rx_complete_pending = false;
      IrDock_ParseRxBytes((IrDock_Source_t)source, channel->rx_buf,
                          channel->rx_len, now_ms);
      channel->rx_len = 0u;
    }

    if (channel->error_pending) {
      channel->error_pending = false;
      channel->frame_head_pending = false;
      channel_debug->error_count++;
      g_ir_dock_debug.error_count++;
    }
  }
}

static bool IrDock_TryStartReceive(IrDock_Source_t source, uint32_t now_ms) {
  IrDock_Channel_t *channel = &ir_dock_channel[source];
  volatile IrDock_ChannelDebug_t *channel_debug =
      &g_ir_dock_debug.channel[source];
  if (channel_debug->rx_busy) {
    return false;
  }

  UART_HandleTypeDef *huart = BSP_UART_GetHandle(channel->uart);
  if (huart != NULL) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
  }

  if (BSP_UART_ReceiveToIdle(channel->uart, channel->rx_buf,
                             IR_DOCK_RX_BUFFER_SIZE, true) == HAL_OK) {
    channel_debug->last_rx_start_ms = now_ms;
    channel_debug->rx_busy = true;
    return true;
  }

  channel_debug->error_count++;
  g_ir_dock_debug.error_count++;
  return false;
}

void IrDock_Init(uint32_t now_ms) {
  (void)BSP_UART_RegisterCallback(BSP_UART_IR, BSP_UART_RX_CPLT_CB,
                                  IrDock_Uart8RxCompleteCallback);
  (void)BSP_UART_RegisterCallback(BSP_UART_IR, BSP_UART_TX_CPLT_CB,
                                  IrDock_Uart8TxCompleteCallback);
  (void)BSP_UART_RegisterRxEventCallback(BSP_UART_IR,
                                         IrDock_Uart8RxEventCallback);
  (void)BSP_UART_RegisterCallback(BSP_UART_IR, BSP_UART_ERROR_CB,
                                  IrDock_Uart8ErrorCallback);
  (void)BSP_UART_RegisterCallback(BSP_UART_IR_AUX, BSP_UART_RX_CPLT_CB,
                                  IrDock_Uart7RxCompleteCallback);
  (void)BSP_UART_RegisterCallback(BSP_UART_IR_AUX, BSP_UART_TX_CPLT_CB,
                                  IrDock_Uart7TxCompleteCallback);
  (void)BSP_UART_RegisterRxEventCallback(BSP_UART_IR_AUX,
                                         IrDock_Uart7RxEventCallback);
  (void)BSP_UART_RegisterCallback(BSP_UART_IR_AUX, BSP_UART_ERROR_CB,
                                  IrDock_Uart7ErrorCallback);

  g_ir_dock_debug.inited = true;
  g_ir_dock_debug.online = false;
  g_ir_dock_debug.rx_busy = false;
  g_ir_dock_debug.tx_busy = false;
  g_ir_dock_debug.dock_complete_fresh = false;
  g_ir_dock_debug.zone3_action_locked = false;
  g_ir_dock_debug.claw_open = false;
  g_ir_dock_debug.release_abort_latched = false;
  g_ir_dock_debug.last_rx_source = IR_DOCK_SOURCE_NONE;
  g_ir_dock_debug.last_ack_tx_source = IR_DOCK_SOURCE_NONE;
  g_ir_dock_debug.last_rx_status = (uint8_t)IR_DOCK_STATUS_IDLE;
  g_ir_dock_debug.last_rx_raw_byte = 0u;
  g_ir_dock_debug.last_ack_tx_byte = 0u;
  g_ir_dock_debug.ack_pending_count = 0u;
  g_ir_dock_debug.last_dock_complete_cmd = 0u;
  g_ir_dock_debug.last_zone3_action_cmd = IR_DOCK_ZONE3_ACTION_CMD_NONE;
  g_ir_dock_debug.last_claw_open_cmd = IR_DOCK_CLAW_OPEN_WAIT;
  g_ir_dock_debug.zone3_action_state = IR_DOCK_ZONE3_ACTION_UNKNOWN;
  g_ir_dock_debug.last_rx_len = 0u;
  for (uint8_t source = 0u; source < IR_DOCK_SOURCE_COUNT; ++source) {
    IrDock_Channel_t *channel = &ir_dock_channel[source];
    volatile IrDock_ChannelDebug_t *channel_debug =
      &g_ir_dock_debug.channel[source];
    channel->rx_len = 0u;
    channel->rx_complete_pending = false;
    channel->error_pending = false;
    channel->ack_tx_busy = false;
    channel->ack_pending_count = 0u;
    channel->ack_tx_byte = IR_DOCK_ACK_BYTE;
    channel->frame_head_pending = false;
    channel_debug->inited = true;
    channel_debug->online = false;
    channel_debug->rx_busy = false;
    channel_debug->tx_busy = false;
    channel_debug->last_rx_status = (uint8_t)IR_DOCK_STATUS_IDLE;
    channel_debug->last_rx_raw_byte = 0u;
    channel_debug->last_ack_tx_byte = 0u;
    channel_debug->ack_pending_count = 0u;
    channel_debug->last_rx_len = 0u;
    channel_debug->last_rx_start_ms = now_ms;
    channel_debug->last_rx_raw_ms = 0u;
    channel_debug->last_online_rx_ms = 0u;
    channel_debug->last_rx_ms = 0u;
    channel_debug->last_rx_age_ms = 0u;
    channel_debug->last_online_age_ms = 0u;
    channel_debug->last_rx_raw_age_ms = 0u;
    IrDock_ClearRawRxData((IrDock_Source_t)source);
  }
  g_ir_dock_debug.last_rx_start_ms = now_ms;
  g_ir_dock_debug.last_rx_raw_ms = 0u;
  g_ir_dock_debug.last_online_rx_ms = 0u;
  g_ir_dock_debug.last_complete_rx_ms = 0u;
  g_ir_dock_debug.last_claw_open_rx_ms = 0u;
  g_ir_dock_debug.last_rx_ms = 0u;
  g_ir_dock_debug.last_rx_age_ms = 0u;
  g_ir_dock_debug.last_online_age_ms = 0u;
  g_ir_dock_debug.last_complete_age_ms = 0u;
  g_ir_dock_debug.last_claw_open_age_ms = 0u;
  g_ir_dock_debug.last_rx_raw_age_ms = 0u;
  for (uint8_t source = 0u; source < IR_DOCK_SOURCE_COUNT; ++source) {
    (void)IrDock_TryStartReceive((IrDock_Source_t)source, now_ms);
  }
}

void IrDock_Process(uint32_t now_ms) {
  if (!g_ir_dock_debug.inited) {
    IrDock_Init(now_ms);
  }

  IrDock_HandlePending(now_ms);
  bool any_rx_busy = false;
  bool any_tx_busy = false;
  bool any_online = false;
  for (uint8_t source = 0u; source < IR_DOCK_SOURCE_COUNT; ++source) {
    volatile IrDock_ChannelDebug_t *channel_debug =
      &g_ir_dock_debug.channel[source];
    channel_debug->last_rx_raw_age_ms =
        (channel_debug->last_rx_raw_ms == 0u)
            ? 0u
            : (now_ms - channel_debug->last_rx_raw_ms);
    channel_debug->last_rx_age_ms =
        (channel_debug->last_rx_ms == 0u)
            ? 0u
            : (now_ms - channel_debug->last_rx_ms);
    channel_debug->last_online_age_ms =
        (channel_debug->last_online_rx_ms == 0u)
            ? 0u
            : (now_ms - channel_debug->last_online_rx_ms);
    channel_debug->online = channel_debug->last_online_rx_ms != 0u &&
                            channel_debug->last_online_age_ms <=
                                IR_DOCK_ONLINE_TIMEOUT_MS;
    any_online = any_online || channel_debug->online;
    (void)IrDock_TryStartReceive((IrDock_Source_t)source, now_ms);
    (void)IrDock_TrySendAck((IrDock_Source_t)source);
    any_rx_busy = any_rx_busy || channel_debug->rx_busy;
    any_tx_busy = any_tx_busy || channel_debug->tx_busy;
  }
  g_ir_dock_debug.last_rx_raw_age_ms =
      (g_ir_dock_debug.last_rx_raw_ms == 0u)
          ? 0u
          : (now_ms - g_ir_dock_debug.last_rx_raw_ms);
  g_ir_dock_debug.last_rx_age_ms =
      (g_ir_dock_debug.last_rx_ms == 0u)
          ? 0u
          : (now_ms - g_ir_dock_debug.last_rx_ms);
  g_ir_dock_debug.dock_complete_fresh = IrDock_IsDockCompleteFresh(now_ms);
  g_ir_dock_debug.claw_open = IrDock_IsClawOpenFreshInternal(now_ms);
  g_ir_dock_debug.rx_busy = any_rx_busy;
  g_ir_dock_debug.tx_busy = any_tx_busy;
  g_ir_dock_debug.online = any_online;
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

static bool IrDock_IsClawOpenFreshInternal(uint32_t now_ms) {
  if (g_ir_dock_debug.last_claw_open_cmd != IR_DOCK_CLAW_OPEN_ALLOW) {
    g_ir_dock_debug.last_claw_open_age_ms = 0u;
    return false;
  }

  if (g_ir_dock_debug.last_claw_open_rx_ms == 0u) {
    g_ir_dock_debug.last_claw_open_age_ms = 0u;
    return false;
  }

  g_ir_dock_debug.last_claw_open_age_ms =
      now_ms - g_ir_dock_debug.last_claw_open_rx_ms;
  return g_ir_dock_debug.last_claw_open_age_ms <= IR_DOCK_COMPLETE_FRESH_MS;
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

void IrDock_BeginZone3Action(void) {
  g_ir_dock_debug.zone3_action_locked = true;
  g_ir_dock_debug.zone3_action_state = IR_DOCK_ZONE3_ACTION_LOCKED;
  g_ir_dock_debug.last_zone3_action_cmd = IR_DOCK_ZONE3_ACTION_CMD_START;
  g_ir_dock_debug.zone3_action_start_rx_count++;
  g_ir_dock_debug.last_claw_open_cmd = IR_DOCK_CLAW_OPEN_WAIT;
  g_ir_dock_debug.last_claw_open_rx_ms = 0u;
  g_ir_dock_debug.last_claw_open_age_ms = 0u;
  g_ir_dock_debug.claw_open = false;
  g_ir_dock_debug.release_abort_latched = false;
}

void IrDock_EndZone3Action(void) {
  g_ir_dock_debug.zone3_action_locked = false;
  g_ir_dock_debug.zone3_action_state = IR_DOCK_ZONE3_ACTION_FINISHED;
  g_ir_dock_debug.last_claw_open_cmd = IR_DOCK_CLAW_OPEN_WAIT;
  g_ir_dock_debug.last_claw_open_rx_ms = 0u;
  g_ir_dock_debug.last_claw_open_age_ms = 0u;
  g_ir_dock_debug.claw_open = false;
  g_ir_dock_debug.release_abort_latched = false;
}

bool IrDock_IsZone3ActionLocked(void) {
  return g_ir_dock_debug.zone3_action_locked;
}

bool IrDock_IsReleaseAbortLatched(void) {
  return g_ir_dock_debug.release_abort_latched;
}

bool IrDock_IsClawOpenFresh(uint32_t now_ms) {
  return IrDock_IsClawOpenFreshInternal(now_ms);
}

uint8_t IrDock_GetLastClawOpenCommand(void) {
  return g_ir_dock_debug.last_claw_open_cmd;
}

uint32_t IrDock_GetClawOpenRxTimeMs(void) {
  return g_ir_dock_debug.last_claw_open_rx_ms;
}

uint32_t IrDock_GetClawOpenAbortCount(void) {
  return g_ir_dock_debug.claw_open_abort_rx_count;
}

uint32_t IrDock_GetZone3ActionStartCount(void) {
  return g_ir_dock_debug.zone3_action_start_rx_count;
}

uint32_t IrDock_GetZone3ActionFinishCount(void) {
  return g_ir_dock_debug.zone3_action_finish_rx_count;
}

uint8_t IrDock_GetZone3ActionState(void) {
  return g_ir_dock_debug.zone3_action_state;
}
