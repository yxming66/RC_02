#include "device/ir_dock/ir_dock.h"

#include "bsp/uart.h"

#include <stddef.h>

volatile IrDock_Debug_t g_ir_dock_debug = {0};

static uint8_t ir_dock_rx_buf[IR_DOCK_MAX_PACKET_SIZE] = {0};
static uint8_t ir_dock_tx_buf[IR_DOCK_MAX_PACKET_SIZE] = {0};
static volatile uint16_t ir_dock_rx_len = 0u;
static volatile bool ir_dock_rx_complete_pending = false;
static volatile bool ir_dock_tx_complete_pending = false;
static volatile bool ir_dock_error_pending = false;

static void IrDock_RxEventCallback(uint16_t size) {
  if (size > IR_DOCK_MAX_PACKET_SIZE) {
    size = IR_DOCK_MAX_PACKET_SIZE;
  }
  ir_dock_rx_len = size;
  ir_dock_rx_complete_pending = true;
  g_ir_dock_debug.rx_busy = false;
}

static void IrDock_RxCompleteCallback(void) {
  ir_dock_rx_len = IR_DOCK_MAX_PACKET_SIZE;
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

static bool IrDock_OreTypeIsValid(uint8_t ore_type) {
  return ore_type == (uint8_t)IR_DOCK_ORE_TYPE_R1 ||
         ore_type == (uint8_t)IR_DOCK_ORE_TYPE_R2 ||
         ore_type == (uint8_t)IR_DOCK_ORE_TYPE_FAKE;
}

static void IrDock_ClearOreTypes(void) {
  for (uint8_t i = 0u; i < IR_DOCK_ORE_POSITION_COUNT; ++i) {
    g_ir_dock_debug.ore_type[i] = (uint8_t)IR_DOCK_ORE_TYPE_UNKNOWN;
  }
}

static bool IrDock_ParseOreInfoPacket(const uint8_t *ore_type,
                                      uint16_t ore_type_len,
                                      uint8_t status,
                                      uint32_t now_ms) {
  if (ore_type == NULL || ore_type_len < IR_DOCK_ORE_POSITION_COUNT) {
    return false;
  }

  for (uint8_t i = 0u; i < IR_DOCK_ORE_POSITION_COUNT; ++i) {
    if (!IrDock_OreTypeIsValid(ore_type[i])) {
      g_ir_dock_debug.ore_info_error_count++;
      return false;
    }
  }

  g_ir_dock_debug.last_rx_status = status;
  g_ir_dock_debug.last_rx_ms = now_ms;
  g_ir_dock_debug.last_ore_rx_ms = now_ms;
  g_ir_dock_debug.ore_info_valid = true;
  g_ir_dock_debug.ore_info_fresh = true;
  for (uint8_t i = 0u; i < IR_DOCK_ORE_POSITION_COUNT; ++i) {
    g_ir_dock_debug.ore_type[i] = ore_type[i];
  }
  g_ir_dock_debug.rx_count++;
  g_ir_dock_debug.ore_info_rx_count++;
  return true;
}

static void IrDock_ParseRxPacket(uint16_t packet_len, uint32_t now_ms) {
  g_ir_dock_debug.last_rx_len = (uint8_t)packet_len;
  if (packet_len == 1u) {
    if (IrDock_StatusIsValid(ir_dock_rx_buf[0])) {
      g_ir_dock_debug.last_rx_status = ir_dock_rx_buf[0];
      g_ir_dock_debug.last_rx_ms = now_ms;
      g_ir_dock_debug.rx_count++;
    } else {
      g_ir_dock_debug.error_count++;
    }
    return;
  }

  if (packet_len == IR_DOCK_ORE_TYPE_ONLY_PACKET_SIZE &&
      IrDock_ParseOreInfoPacket(ir_dock_rx_buf, packet_len,
                                g_ir_dock_debug.last_rx_status, now_ms)) {
    return;
  }

  if (packet_len >= IR_DOCK_ORE_PACKET_SIZE &&
      packet_len <= IR_DOCK_MAX_PACKET_SIZE &&
      IrDock_StatusIsValid(ir_dock_rx_buf[0]) &&
      IrDock_ParseOreInfoPacket(&ir_dock_rx_buf[1], packet_len - 1u,
                                ir_dock_rx_buf[0], now_ms)) {
    return;
  }

  g_ir_dock_debug.error_count++;
}

static void IrDock_HandlePending(uint32_t now_ms) {
  if (ir_dock_rx_complete_pending) {
    ir_dock_rx_complete_pending = false;
    IrDock_ParseRxPacket(ir_dock_rx_len, now_ms);
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

  if ((now_ms - g_ir_dock_debug.last_rx_start_ms) < IR_DOCK_MIN_INTERVAL_MS) {
    g_ir_dock_debug.rx_interval_block_count++;
    return false;
  }

  UART_HandleTypeDef *huart = BSP_UART_GetHandle(BSP_UART_IR);
  if (huart != NULL) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
  }

  if (BSP_UART_ReceiveToIdle(BSP_UART_IR, ir_dock_rx_buf,
                             IR_DOCK_MAX_PACKET_SIZE, false) == HAL_OK) {
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
  g_ir_dock_debug.rx_busy = false;
  g_ir_dock_debug.tx_busy = false;
  g_ir_dock_debug.dock_complete_fresh = false;
  g_ir_dock_debug.ore_info_valid = false;
  g_ir_dock_debug.ore_info_fresh = false;
  g_ir_dock_debug.last_rx_status = (uint8_t)IR_DOCK_STATUS_IDLE;
  g_ir_dock_debug.last_tx_status = (uint8_t)IR_DOCK_STATUS_IDLE;
  g_ir_dock_debug.last_rx_len = 0u;
  g_ir_dock_debug.last_tx_len = 0u;
  IrDock_ClearOreTypes();
  g_ir_dock_debug.last_rx_start_ms = now_ms - IR_DOCK_MIN_INTERVAL_MS;
  g_ir_dock_debug.last_rx_ms = 0u;
  g_ir_dock_debug.last_ore_rx_ms = 0u;
  g_ir_dock_debug.last_tx_ms = now_ms - IR_DOCK_MIN_INTERVAL_MS;
  g_ir_dock_debug.last_rx_age_ms = 0u;
  g_ir_dock_debug.last_ore_rx_age_ms = 0u;
  (void)IrDock_TryStartReceive(now_ms);
}

void IrDock_Process(uint32_t now_ms) {
  if (!g_ir_dock_debug.inited) {
    IrDock_Init(now_ms);
  }

  IrDock_HandlePending(now_ms);
  g_ir_dock_debug.dock_complete_fresh = IrDock_IsDockCompleteFresh(now_ms);
  g_ir_dock_debug.ore_info_fresh = IrDock_IsOreInfoFresh(now_ms);
  (void)IrDock_TryStartReceive(now_ms);
}

bool IrDock_SendStatus(IrDock_Status_t status, uint32_t now_ms) {
  if (!g_ir_dock_debug.inited || g_ir_dock_debug.tx_busy) {
    return false;
  }

  if ((now_ms - g_ir_dock_debug.last_tx_ms) < IR_DOCK_MIN_INTERVAL_MS) {
    g_ir_dock_debug.tx_interval_block_count++;
    return false;
  }

  ir_dock_tx_buf[0] = (uint8_t)status;
  if (!IrDock_StatusIsValid(ir_dock_tx_buf[0])) {
    return false;
  }

  if (BSP_UART_Transmit(BSP_UART_IR, ir_dock_tx_buf, 1u, false) == HAL_OK) {
    g_ir_dock_debug.last_tx_status = ir_dock_tx_buf[0];
    g_ir_dock_debug.last_tx_len = 1u;
    g_ir_dock_debug.last_tx_ms = now_ms;
    g_ir_dock_debug.tx_busy = true;
    g_ir_dock_debug.tx_count++;
    return true;
  }

  g_ir_dock_debug.error_count++;
  return false;
}

bool IrDock_SendOreInfo(IrDock_Status_t status,
                        const uint8_t ore_type[IR_DOCK_ORE_POSITION_COUNT],
                        uint32_t now_ms) {
  if (!g_ir_dock_debug.inited || g_ir_dock_debug.tx_busy) {
    return false;
  }

  if (ore_type == NULL || !IrDock_StatusIsValid((uint8_t)status)) {
    return false;
  }

  if ((now_ms - g_ir_dock_debug.last_tx_ms) < IR_DOCK_MIN_INTERVAL_MS) {
    g_ir_dock_debug.tx_interval_block_count++;
    return false;
  }

  ir_dock_tx_buf[0] = (uint8_t)status;
  for (uint8_t i = 0u; i < IR_DOCK_ORE_POSITION_COUNT; ++i) {
    if (!IrDock_OreTypeIsValid(ore_type[i])) {
      return false;
    }
    ir_dock_tx_buf[1u + i] = ore_type[i];
  }

  if (BSP_UART_Transmit(BSP_UART_IR, ir_dock_tx_buf,
                        IR_DOCK_ORE_PACKET_SIZE, false) == HAL_OK) {
    g_ir_dock_debug.last_tx_status = ir_dock_tx_buf[0];
    g_ir_dock_debug.last_tx_len = IR_DOCK_ORE_PACKET_SIZE;
    g_ir_dock_debug.last_tx_ms = now_ms;
    g_ir_dock_debug.tx_busy = true;
    g_ir_dock_debug.tx_count++;
    return true;
  }

  g_ir_dock_debug.error_count++;
  return false;
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
  for (uint8_t i = 0u; i < IR_DOCK_ORE_POSITION_COUNT; ++i) {
    info->ore_type[i] = g_ir_dock_debug.ore_type[i];
  }
  info->last_rx_ms = g_ir_dock_debug.last_ore_rx_ms;
  info->age_ms = g_ir_dock_debug.last_ore_rx_age_ms;
  info->rx_count = g_ir_dock_debug.ore_info_rx_count;
  return info->valid;
}
