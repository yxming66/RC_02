#include "rf_remote.h"

#include <cmsis_os2.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "bsp/time.h"
#include "bsp/uart.h"
#include "component/crc16.h"

#define RF_REMOTE_HEAD_0 (0xA5u)
#define RF_REMOTE_HEAD_1 (0x5Au)
#define RF_REMOTE_VERSION (0x01u)
#define RF_REMOTE_TYPE_CONTROL (0x10u)
#define RF_REMOTE_HEADER_SIZE (8u)
#define RF_REMOTE_META_SIZE (6u)
#define RF_REMOTE_CRC_SIZE (2u)
#define RF_REMOTE_RX_BUF_SIZE (96u)
#define RF_REMOTE_STAGING_SIZE (128u)

volatile RF_Remote_Debug_t rf_remote_debug;

static osThreadId_t thread_alert;

static bool inited = false;
static uint8_t rx_buf[RF_REMOTE_RX_BUF_SIZE];
static uint8_t staging_buf[RF_REMOTE_STAGING_SIZE];
static size_t staging_len = 0;
static volatile uint16_t rx_size = 0;
static volatile bool rx_error = false;
static bool latest_valid = false;

static uint16_t RF_Remote_ReadLe16(const uint8_t *data) {
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t RF_Remote_ReadLe32(const uint8_t *data) {
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static int16_t RF_Remote_ReadI16(const uint8_t *data) {
  return (int16_t)RF_Remote_ReadLe16(data);
}

static void RF_Remote_RxEventCallback(uint16_t size) {
  rx_size = size;
  rf_remote_debug.stats.rx_events++;
  osThreadFlagsSet(thread_alert, SIGNAL_RF_REMOTE_RX_READY);
}

static void RF_Remote_RxCpltCallback(void) {
  RF_Remote_RxEventCallback(RF_REMOTE_RX_BUF_SIZE);
}

static void RF_Remote_ErrorCallback(void) {
  rx_error = true;
  rx_size = 0;
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

static void RF_Remote_SaveFrame(const uint8_t *frame) {
  const uint8_t *payload = frame + RF_REMOTE_HEADER_SIZE;
  RF_Remote_Frame_t latest;

  memset(&latest, 0, sizeof(latest));
  latest.timestamp_ms = RF_Remote_ReadLe32(payload + 0u);
  latest.left_x_raw = RF_Remote_ReadI16(payload + 4u);
  latest.left_y_raw = RF_Remote_ReadI16(payload + 6u);
  latest.right_x_raw = RF_Remote_ReadI16(payload + 8u);
  latest.right_y_raw = RF_Remote_ReadI16(payload + 10u);
  latest.knob1_raw = RF_Remote_ReadI16(payload + 12u);
  latest.knob2_raw = RF_Remote_ReadI16(payload + 14u);
  for (uint8_t i = 0u; i < RF_REMOTE_SWITCH_NUM; ++i) {
    latest.sw[i] = (uint8_t)((payload[16u + i / 4u] >> ((i % 4u) * 2u)) & 0x03u);
  }
  latest.keys = RF_Remote_ReadLe16(payload + 19u);
  latest.reserved = RF_Remote_ReadLe16(payload + 21u);
  latest.flags = frame[4u];
  latest.seq = frame[5u];
  latest.rx_time_us = BSP_TIME_Get_us();

  rf_remote_debug.latest = latest;
  rf_remote_debug.stats.good_frames++;
  rf_remote_debug.stats.last_seq = latest.seq;
  rf_remote_debug.stats.last_rx_time_us = latest.rx_time_us;
  rf_remote_debug.stats.online = true;
  latest_valid = true;
}

static void RF_Remote_ProcessStaging(void) {
  while (staging_len >= 2u) {
    size_t header_pos = RF_Remote_FindHeader();
    if (header_pos == staging_len) {
      size_t keep_len = (staging_buf[staging_len - 1u] == RF_REMOTE_HEAD_0) ? 1u : 0u;
      if (staging_len > keep_len) {
        rf_remote_debug.stats.sync_drops += (uint32_t)(staging_len - keep_len);
        memmove(staging_buf, staging_buf + staging_len - keep_len, keep_len);
        staging_len = keep_len;
      }
      return;
    }

    if (header_pos > 0u) {
      rf_remote_debug.stats.sync_drops += (uint32_t)header_pos;
      RF_Remote_DropStaging(header_pos);
    }

    if (staging_len < RF_REMOTE_HEADER_SIZE) return;

    const uint16_t payload_len = RF_Remote_ReadLe16(staging_buf + 6u);
    const size_t frame_size = RF_REMOTE_HEADER_SIZE + (size_t)payload_len + RF_REMOTE_CRC_SIZE;
    if (payload_len > RF_REMOTE_PAYLOAD_SIZE) {
      rf_remote_debug.stats.format_errors++;
      RF_Remote_DropStaging(1u);
      continue;
    }
    if (staging_len < frame_size) return;

    if (staging_buf[2u] != RF_REMOTE_VERSION ||
        staging_buf[3u] != RF_REMOTE_TYPE_CONTROL ||
        payload_len != RF_REMOTE_PAYLOAD_SIZE) {
      rf_remote_debug.stats.format_errors++;
      RF_Remote_DropStaging(1u);
      continue;
    }

    const uint16_t crc_recv = RF_Remote_ReadLe16(staging_buf + RF_REMOTE_HEADER_SIZE + payload_len);
    const uint16_t crc_calc = CRC16_Calc(staging_buf + 2u,
                                         RF_REMOTE_META_SIZE + payload_len,
                                         CRC16_INIT);
    if (crc_recv != crc_calc) {
      rf_remote_debug.stats.crc_errors++;
      RF_Remote_DropStaging(1u);
      continue;
    }

    RF_Remote_SaveFrame(staging_buf);
    RF_Remote_DropStaging(frame_size);
  }
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
  return DEVICE_OK;
}

int8_t RF_Remote_StartDmaRecv(void) {
  rx_size = 0u;
  rx_error = false;
  osThreadFlagsClear(SIGNAL_RF_REMOTE_RX_READY);

  if (BSP_UART_ReceiveToIdle(BSP_UART_RF_RC, rx_buf, RF_REMOTE_RX_BUF_SIZE,
                             true) == HAL_OK) {
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

  int8_t status = DEVICE_OK;
  if (RF_Remote_WaitDmaCplt(0u)) {
    if (RF_Remote_ParseData() != DEVICE_OK) {
      status = DEVICE_ERR;
      (void)RF_Remote_Restart();
    }
    if (RF_Remote_StartDmaRecv() != DEVICE_OK) {
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