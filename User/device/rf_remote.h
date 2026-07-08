#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "device.h"

#define RF_REMOTE_PAYLOAD_SIZE (23u)
#define RF_REMOTE_SWITCH_NUM (10u)
#define RF_REMOTE_FRAME_SIZE (33u)

typedef struct {
  uint32_t timestamp_ms;
  int16_t left_x_raw;
  int16_t left_y_raw;
  int16_t right_x_raw;
  int16_t right_y_raw;
  int16_t knob1_raw;
  int16_t knob2_raw;
  uint8_t sw[RF_REMOTE_SWITCH_NUM];
  uint16_t keys;
  uint16_t reserved;
  uint8_t flags;
  uint8_t seq;
  uint64_t rx_time_us;
} RF_Remote_Frame_t;

typedef struct {
  volatile uint32_t rx_bytes;
  volatile uint32_t rx_events;
  volatile uint32_t good_frames;
  volatile uint32_t crc_errors;
  volatile uint32_t format_errors;
  volatile uint32_t sync_drops;
  volatile uint32_t dma_errors;
  volatile uint8_t last_seq;
  volatile uint64_t last_rx_time_us;
  volatile bool online;
} RF_Remote_Stats_t;

typedef struct {
  RF_Remote_Frame_t latest;
  RF_Remote_Stats_t stats;
} RF_Remote_Debug_t;

extern volatile RF_Remote_Debug_t rf_remote_debug;

int8_t RF_Remote_Init(void);
int8_t RF_Remote_Restart(void);
int8_t RF_Remote_StartDmaRecv(void);
bool RF_Remote_WaitDmaCplt(uint32_t timeout);
int8_t RF_Remote_ParseData(void);
int8_t RF_Remote_Poll(uint32_t offline_timeout_us);
bool RF_Remote_GetLatest(RF_Remote_Frame_t *frame);
const volatile RF_Remote_Stats_t *RF_Remote_GetStats(void);

#ifdef __cplusplus
}
#endif