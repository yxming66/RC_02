#include "rf_remote.h"

#include <cmsis_os2.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "bsp/time.h"
#include "bsp/uart.h"
#include "component/crc8.h"

#define RF_REMOTE_HEAD_0 (0xAAu)
#define RF_REMOTE_HEAD_1 (0x55u)
#define RF_REMOTE_RX_BUF_SIZE (96u)
#define RF_REMOTE_STAGING_SIZE (128u)
#define RF_REMOTE_R1_ALLOWED_MASK (0x0F6Fu)
#define RF_REMOTE_TOP_ROW_MASK (0x0007u)
#define RF_REMOTE_ACK_FIRST_DELAY_US (15000ULL)
#define RF_REMOTE_ACK_SECOND_DELAY_US (25000ULL)
#define RF_REMOTE_ACK_THIRD_DELAY_US (40000ULL)
#define RF_REMOTE_TX_TIMEOUT_MS (10u)
#define RF_REMOTE_DUPLICATE_WINDOW_US (1000000ULL)

volatile RF_Remote_Debug_t rf_remote_debug;

static osThreadId_t thread_alert;

static bool inited = false;
static bool latest_valid = false;
static volatile bool receiver_busy = false;
static volatile bool rx_active = false;
static uint8_t rx_buf[RF_REMOTE_RX_BUF_SIZE];
static uint8_t staging_buf[RF_REMOTE_STAGING_SIZE];
static size_t staging_len = 0u;
static volatile uint16_t rx_size = 0u;
static volatile bool rx_error = false;

static bool ack_pending = false;
static uint8_t ack_frame[RF_REMOTE_ACK_FRAME_SIZE];
static uint8_t ack_repeat_index = 0u;
static uint64_t ack_due_time_us = 0u;

__weak void RF_Remote_OnKfsFrame(
    uint8_t side, const uint8_t cells[RF_REMOTE_CELL_NUM], uint8_t msg_id) {
  (void)side;
  (void)cells;
  (void)msg_id;
}

__weak void RF_Remote_OnRetryFrame(uint8_t mode, uint8_t msg_id) {
  (void)mode;
  (void)msg_id;
}

static void RF_Remote_RxEventCallback(uint16_t size) {
  rx_size = size;
  rx_active = false;
  rf_remote_debug.stats.rx_events++;
  osThreadFlagsSet(thread_alert, SIGNAL_RF_REMOTE_RX_READY);
}

static void RF_Remote_RxCpltCallback(void) {
  RF_Remote_RxEventCallback(RF_REMOTE_RX_BUF_SIZE);
}

static void RF_Remote_ErrorCallback(void) {
  rx_error = true;
  rx_size = 0u;
  rx_active = false;
  rf_remote_debug.stats.dma_errors++;
  osThreadFlagsSet(thread_alert, SIGNAL_RF_REMOTE_RX_READY);
}

static void RF_Remote_DropStaging(size_t drop_len) {
  if (drop_len == 0u) return;
  if (drop_len >= staging_len) {
    staging_len = 0u;
    return;
  }
  memmove(staging_buf, staging_buf + drop_len, staging_len - drop_len);
  staging_len -= drop_len;
}

static void RF_Remote_AppendBytes(const uint8_t *data, size_t len) {
  if (data == NULL || len == 0u) return;

  if (len >= RF_REMOTE_STAGING_SIZE) {
    data += len - RF_REMOTE_STAGING_SIZE;
    len = RF_REMOTE_STAGING_SIZE;
    staging_len = 0u;
    rf_remote_debug.stats.sync_drops++;
  }

  while (staging_len + len > RF_REMOTE_STAGING_SIZE) {
    RF_Remote_DropStaging(1u);
    rf_remote_debug.stats.sync_drops++;
  }

  memcpy(staging_buf + staging_len, data, len);
  staging_len += len;
}

static size_t RF_Remote_FindHeader(void) {
  for (size_t i = 0u; i + 1u < staging_len; ++i) {
    if (staging_buf[i] == RF_REMOTE_HEAD_0 &&
        staging_buf[i + 1u] == RF_REMOTE_HEAD_1) {
      return i;
    }
  }
  return staging_len;
}

static uint64_t RF_Remote_AckDelayUs(uint8_t repeat_index) {
  if (repeat_index == 0u) return RF_REMOTE_ACK_FIRST_DELAY_US;
  if (repeat_index == 1u) return RF_REMOTE_ACK_SECOND_DELAY_US;
  return RF_REMOTE_ACK_THIRD_DELAY_US;
}

static void RF_Remote_QueueAck(uint8_t msg_id, RF_Remote_AckStatus_t status) {
  ack_frame[0] = RF_REMOTE_HEAD_0;
  ack_frame[1] = RF_REMOTE_HEAD_1;
  ack_frame[2] = (uint8_t)RF_REMOTE_CMD_ACK;
  ack_frame[3] = msg_id;
  ack_frame[4] = (uint8_t)status;
  ack_frame[5] = CRC8_Calc(ack_frame, RF_REMOTE_ACK_FRAME_SIZE - 1u,
                           CRC8_INIT);
  ack_repeat_index = 0u;
  ack_due_time_us = BSP_TIME_Get_us() + RF_Remote_AckDelayUs(0u);
  ack_pending = true;
  rf_remote_debug.stats.last_ack_status = (uint8_t)status;
}

static bool RF_Remote_ValidateKfs(const uint8_t *frame) {
  const uint8_t side = frame[4u];
  uint8_t r1_count = 0u;
  uint8_t r2_count = 0u;
  uint8_t fake_count = 0u;

  if (frame[3u] == 0u || (side != (uint8_t)'R' && side != (uint8_t)'B')) {
    return false;
  }

  for (uint8_t i = 0u; i < RF_REMOTE_CELL_NUM; ++i) {
    const uint8_t state = frame[5u + i];
    const uint16_t position_bit = (uint16_t)(1u << i);
    if (state > (uint8_t)RF_REMOTE_CELL_FAKE) return false;

    if (state == (uint8_t)RF_REMOTE_CELL_R1) {
      if ((RF_REMOTE_R1_ALLOWED_MASK & position_bit) == 0u) return false;
      r1_count++;
    } else if (state == (uint8_t)RF_REMOTE_CELL_R2) {
      r2_count++;
    } else if (state == (uint8_t)RF_REMOTE_CELL_FAKE) {
      if ((RF_REMOTE_TOP_ROW_MASK & position_bit) != 0u) return false;
      fake_count++;
    }
  }

  return r1_count <= 3u && r2_count <= 4u && fake_count <= 1u;
}

static bool RF_Remote_ValidateRetry(const uint8_t *frame) {
  return frame[3u] != 0u && frame[4u] >= 1u && frame[4u] <= 12u;
}

static bool RF_Remote_IsDuplicate(uint8_t command, uint8_t msg_id,
                                  uint64_t now_us) {
  return latest_valid && rf_remote_debug.latest.command == command &&
         rf_remote_debug.latest.msg_id == msg_id &&
         (now_us - rf_remote_debug.latest.rx_time_us) <=
             RF_REMOTE_DUPLICATE_WINDOW_US;
}

static void RF_Remote_SaveFrame(const uint8_t *frame, uint64_t now_us) {
  RF_Remote_Frame_t latest;
  memset(&latest, 0, sizeof(latest));

  latest.command = frame[2u];
  latest.msg_id = frame[3u];
  latest.rx_time_us = now_us;

  if (latest.command == (uint8_t)RF_REMOTE_CMD_KFS) {
    latest.side = frame[4u];
    memcpy(latest.cells, frame + 5u, RF_REMOTE_CELL_NUM);
    rf_remote_debug.stats.kfs_frames++;
  } else {
    latest.retry_mode = frame[4u];
    rf_remote_debug.stats.retry_frames++;
  }

  rf_remote_debug.latest = latest;
  rf_remote_debug.stats.good_frames++;
  rf_remote_debug.stats.last_command = latest.command;
  rf_remote_debug.stats.last_msg_id = latest.msg_id;
  rf_remote_debug.stats.last_rx_time_us = now_us;
  rf_remote_debug.stats.online = true;
  latest_valid = true;

  if (latest.command == (uint8_t)RF_REMOTE_CMD_KFS) {
    RF_Remote_OnKfsFrame(latest.side, latest.cells, latest.msg_id);
  } else {
    RF_Remote_OnRetryFrame(latest.retry_mode, latest.msg_id);
  }
}

static size_t RF_Remote_FrameSize(uint8_t command) {
  if (command == (uint8_t)RF_REMOTE_CMD_KFS) {
    return RF_REMOTE_KFS_FRAME_SIZE;
  }
  if (command == (uint8_t)RF_REMOTE_CMD_R2_RETRY) {
    return RF_REMOTE_RETRY_FRAME_SIZE;
  }
  return 0u;
}

static void RF_Remote_ProcessStaging(void) {
  while (!ack_pending && staging_len >= 2u) {
    const size_t header_pos = RF_Remote_FindHeader();
    if (header_pos == staging_len) {
      const size_t keep_len =
          (staging_buf[staging_len - 1u] == RF_REMOTE_HEAD_0) ? 1u : 0u;
      if (staging_len > keep_len) {
        rf_remote_debug.stats.sync_drops +=
            (uint32_t)(staging_len - keep_len);
        memmove(staging_buf, staging_buf + staging_len - keep_len, keep_len);
        staging_len = keep_len;
      }
      return;
    }

    if (header_pos > 0u) {
      rf_remote_debug.stats.sync_drops += (uint32_t)header_pos;
      RF_Remote_DropStaging(header_pos);
    }

    if (staging_len < 3u) return;

    const size_t frame_size = RF_Remote_FrameSize(staging_buf[2u]);
    if (frame_size == 0u) {
      rf_remote_debug.stats.format_errors++;
      RF_Remote_DropStaging(1u);
      continue;
    }
    if (staging_len < frame_size) return;

    const uint8_t msg_id = staging_buf[3u];
    if (!CRC8_Verify(staging_buf, frame_size)) {
      rf_remote_debug.stats.crc_errors++;
      RF_Remote_QueueAck(msg_id, RF_REMOTE_ACK_CRC_ERROR);
      RF_Remote_DropStaging(frame_size);
      return;
    }

    const bool data_valid =
        (staging_buf[2u] == (uint8_t)RF_REMOTE_CMD_KFS)
            ? RF_Remote_ValidateKfs(staging_buf)
            : RF_Remote_ValidateRetry(staging_buf);
    if (!data_valid) {
      rf_remote_debug.stats.data_errors++;
      RF_Remote_QueueAck(msg_id, RF_REMOTE_ACK_INVALID);
      RF_Remote_DropStaging(frame_size);
      return;
    }

    if (receiver_busy) {
      rf_remote_debug.stats.busy_frames++;
      RF_Remote_QueueAck(msg_id, RF_REMOTE_ACK_BUSY);
      RF_Remote_DropStaging(frame_size);
      return;
    }

    const uint64_t now_us = BSP_TIME_Get_us();
    if (RF_Remote_IsDuplicate(staging_buf[2u], msg_id, now_us)) {
      rf_remote_debug.stats.duplicate_frames++;
    } else {
      RF_Remote_SaveFrame(staging_buf, now_us);
    }
    RF_Remote_QueueAck(msg_id, RF_REMOTE_ACK_OK);
    RF_Remote_DropStaging(frame_size);
  }
}

static int8_t RF_Remote_ServiceAck(void) {
  if (!ack_pending || BSP_TIME_Get_us() < ack_due_time_us) return DEVICE_OK;

  UART_HandleTypeDef *huart = BSP_UART_GetHandle(BSP_UART_RF_RC);
  if (huart == NULL) return DEVICE_ERR_NULL;

  int8_t status = DEVICE_OK;
  if (HAL_UART_Transmit(huart, ack_frame, RF_REMOTE_ACK_FRAME_SIZE,
                        RF_REMOTE_TX_TIMEOUT_MS) == HAL_OK) {
    rf_remote_debug.stats.ack_frames++;
  } else {
    rf_remote_debug.stats.ack_errors++;
    status = DEVICE_ERR;
  }

  ack_repeat_index++;
  if (ack_repeat_index < RF_REMOTE_ACK_REPEAT_COUNT) {
    ack_due_time_us =
        BSP_TIME_Get_us() + RF_Remote_AckDelayUs(ack_repeat_index);
    return status;
  }

  ack_pending = false;
  RF_Remote_ProcessStaging();
  if (!ack_pending && RF_Remote_StartDmaRecv() != DEVICE_OK) {
    rf_remote_debug.stats.dma_errors++;
    return DEVICE_ERR;
  }
  return status;
}

int8_t RF_Remote_Init(void) {
  if (inited) return DEVICE_ERR_INITED;
  if ((thread_alert = osThreadGetId()) == NULL) return DEVICE_ERR_NULL;

  BSP_UART_RegisterCallback(BSP_UART_RF_RC, BSP_UART_RX_CPLT_CB,
                            RF_Remote_RxCpltCallback);
  BSP_UART_RegisterRxEventCallback(BSP_UART_RF_RC, RF_Remote_RxEventCallback);
  BSP_UART_RegisterCallback(BSP_UART_RF_RC, BSP_UART_ERROR_CB,
                            RF_Remote_ErrorCallback);

  memset((void *)&rf_remote_debug, 0, sizeof(rf_remote_debug));
  staging_len = 0u;
  latest_valid = false;
  receiver_busy = false;
  rx_active = false;
  ack_pending = false;
  inited = true;
  return DEVICE_OK;
}

int8_t RF_Remote_Restart(void) {
  UART_HandleTypeDef *huart = BSP_UART_GetHandle(BSP_UART_RF_RC);
  if (huart == NULL) return DEVICE_ERR_NULL;

  (void)HAL_UART_AbortReceive(huart);
  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                   UART_CLEAR_NEF | UART_CLEAR_OREF |
                                   UART_CLEAR_IDLEF);
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  rx_size = 0u;
  rx_error = false;
  rx_active = false;
  return DEVICE_OK;
}

int8_t RF_Remote_StartDmaRecv(void) {
  if (ack_pending) return DEVICE_ERR;
  if (rx_active) return DEVICE_OK;

  rx_size = 0u;
  rx_error = false;
  osThreadFlagsClear(SIGNAL_RF_REMOTE_RX_READY);

  if (BSP_UART_ReceiveToIdle(BSP_UART_RF_RC, rx_buf, RF_REMOTE_RX_BUF_SIZE,
                             true) == HAL_OK) {
    rx_active = true;
    return DEVICE_OK;
  }
  return DEVICE_ERR;
}

bool RF_Remote_WaitDmaCplt(uint32_t timeout) {
  return (osThreadFlagsWait(SIGNAL_RF_REMOTE_RX_READY, osFlagsWaitAll,
                            timeout) == SIGNAL_RF_REMOTE_RX_READY);
}

int8_t RF_Remote_ParseData(void) {
  if (rx_error) return DEVICE_ERR;
  if (rx_size == 0u || rx_size > RF_REMOTE_RX_BUF_SIZE) return DEVICE_ERR;

  uint8_t local_buf[RF_REMOTE_RX_BUF_SIZE];
  const uint16_t local_size = rx_size;
  memcpy(local_buf, rx_buf, local_size);
  rf_remote_debug.stats.rx_bytes += local_size;
  RF_Remote_AppendBytes(local_buf, local_size);
  RF_Remote_ProcessStaging();
  return DEVICE_OK;
}

int8_t RF_Remote_Poll(uint32_t offline_timeout_us) {
  if (!inited) return DEVICE_ERR;

  int8_t status = RF_Remote_ServiceAck();
  if (!ack_pending && RF_Remote_WaitDmaCplt(0u)) {
    if (RF_Remote_ParseData() != DEVICE_OK) {
      status = DEVICE_ERR;
      (void)RF_Remote_Restart();
    }
    if (!ack_pending && RF_Remote_StartDmaRecv() != DEVICE_OK) {
      status = DEVICE_ERR;
      (void)RF_Remote_Restart();
      (void)RF_Remote_StartDmaRecv();
    }
  }

  if (rf_remote_debug.stats.online && offline_timeout_us > 0u &&
      (BSP_TIME_Get_us() - rf_remote_debug.stats.last_rx_time_us) >
          (uint64_t)offline_timeout_us) {
    rf_remote_debug.stats.online = false;
  }
  return status;
}

bool RF_Remote_GetLatest(RF_Remote_Frame_t *frame) {
  if (frame == NULL || !latest_valid) return false;
  *frame = rf_remote_debug.latest;
  return true;
}

const volatile RF_Remote_Stats_t *RF_Remote_GetStats(void) {
  return &rf_remote_debug.stats;
}

void RF_Remote_SetBusy(bool busy) { receiver_busy = busy; }
