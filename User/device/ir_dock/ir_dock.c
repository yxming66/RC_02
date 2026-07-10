#include "device/ir_dock/ir_dock.h"

#include <stddef.h>

#include "bsp/uart.h"

static uint8_t ir_dock_rx_buf[IR_DOCK_RX_BUFFER_SIZE] = {0};
static volatile uint16_t ir_dock_rx_len = 0u;
static volatile bool ir_dock_rx_complete_pending = false;
static volatile bool ir_dock_error_pending = false;
static bool ir_dock_frame_head_pending = false;

static bool IrDock_IsClawOpenFreshInternal(uint32_t now_ms);

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

static void IrDock_ErrorCallback(void) {
  ir_dock_error_pending = true;
  g_ir_dock_debug.rx_busy = false;
}

static bool IrDock_CommandIsValid(uint8_t command) {
  return command >= IR_DOCK_CMD_ONLINE && command <= IR_DOCK_CMD_MOVE_ALLOW;
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

static void IrDock_ParseProtocolFrame(uint8_t command, uint32_t now_ms) {
  g_ir_dock_debug.last_rx_raw_byte = command;
  g_ir_dock_debug.last_rx_ms = now_ms;
  g_ir_dock_debug.rx_count++;

  if (!IrDock_CommandIsValid(command)) {
    g_ir_dock_debug.invalid_rx_count++;
    g_ir_dock_debug.error_count++;
    return;
  }

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
    break;

  case IR_DOCK_CMD_RELEASE_ALLOW:
    g_ir_dock_debug.last_claw_open_cmd = IR_DOCK_CLAW_OPEN_ALLOW;
    g_ir_dock_debug.last_claw_open_rx_ms = now_ms;
    g_ir_dock_debug.claw_open = true;
    g_ir_dock_debug.claw_open_rx_count++;
    break;

  case IR_DOCK_CMD_RELEASE_ABORT:
    g_ir_dock_debug.last_claw_open_cmd = IR_DOCK_CLAW_OPEN_ABORT;
    g_ir_dock_debug.last_claw_open_rx_ms = 0u;
    g_ir_dock_debug.last_claw_open_age_ms = 0u;
    g_ir_dock_debug.claw_open = false;
    g_ir_dock_debug.claw_open_abort_rx_count++;
    break;

  case IR_DOCK_CMD_MOVE_ALLOW:
    g_ir_dock_debug.last_r2_leave_zone1_cmd = IR_DOCK_LEAVE_ZONE1_ALLOW;
    g_ir_dock_debug.r2_leave_zone1_allowed = true;
    break;

  case IR_DOCK_CMD_ONLINE:
  default:
    break;
  }
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

  for (uint16_t offset = 0u; offset < packet_len; ++offset) {
    const uint8_t rx_byte = rx_buf[offset];
    if (!ir_dock_frame_head_pending) {
      if (rx_byte == IR_DOCK_FRAME_HEAD) {
        ir_dock_frame_head_pending = true;
      } else {
        g_ir_dock_debug.invalid_rx_count++;
        g_ir_dock_debug.error_count++;
      }
      continue;
    }

    if (rx_byte == IR_DOCK_FRAME_HEAD) {
      continue;
    }

    ir_dock_frame_head_pending = false;
    IrDock_ParseProtocolFrame(rx_byte, now_ms);
  }
}

static void IrDock_HandlePending(uint32_t now_ms) {
  if (ir_dock_rx_complete_pending) {
    ir_dock_rx_complete_pending = false;
    IrDock_ParseRxBytes(ir_dock_rx_buf, ir_dock_rx_len, now_ms);
    ir_dock_rx_len = 0u;
  }

  if (ir_dock_error_pending) {
    ir_dock_error_pending = false;
    ir_dock_frame_head_pending = false;
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
                             IR_DOCK_RX_BUFFER_SIZE, true) == HAL_OK) {
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
  (void)BSP_UART_RegisterCallback(BSP_UART_IR, BSP_UART_ERROR_CB,
                                  IrDock_ErrorCallback);

  g_ir_dock_debug.inited = true;
  g_ir_dock_debug.online = false;
  g_ir_dock_debug.rx_busy = false;
  g_ir_dock_debug.dock_complete_fresh = false;
  g_ir_dock_debug.r2_leave_zone1_allowed = false;
  g_ir_dock_debug.claw_open = false;
  g_ir_dock_debug.last_rx_status = (uint8_t)IR_DOCK_STATUS_IDLE;
  g_ir_dock_debug.last_rx_raw_byte = 0u;
  g_ir_dock_debug.last_dock_complete_cmd = 0u;
  g_ir_dock_debug.last_r2_leave_zone1_cmd = IR_DOCK_LEAVE_ZONE1_FORBID;
  g_ir_dock_debug.last_claw_open_cmd = IR_DOCK_CLAW_OPEN_WAIT;
  g_ir_dock_debug.last_cleared_ore_id = IR_DOCK_CLEARED_ORE_NONE;
  g_ir_dock_debug.last_zone3_r2_state = IR_DOCK_ZONE3_R2_UNKNOWN;
  g_ir_dock_debug.last_rx_len = 0u;
  ir_dock_frame_head_pending = false;
  IrDock_ClearRawRxData();
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
  g_ir_dock_debug.claw_open = IrDock_IsClawOpenFreshInternal(now_ms);
  g_ir_dock_debug.online = IrDock_IsOnline(now_ms);
  (void)IrDock_TryStartReceive(now_ms);
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

bool IrDock_IsR2LeaveZone1Allowed(void) {
  return g_ir_dock_debug.r2_leave_zone1_allowed;
}

bool IrDock_IsClawOpenFresh(uint32_t now_ms) {
  return IrDock_IsClawOpenFreshInternal(now_ms);
}

uint8_t IrDock_GetLastClawOpenCommand(void) {
  return g_ir_dock_debug.last_claw_open_cmd;
}

uint32_t IrDock_GetClawOpenAbortCount(void) {
  return g_ir_dock_debug.claw_open_abort_rx_count;
}

uint8_t IrDock_GetLastClearedOreId(void) {
  return g_ir_dock_debug.last_cleared_ore_id;
}

uint8_t IrDock_GetLastZone3R2State(void) {
  return g_ir_dock_debug.last_zone3_r2_state;
}
