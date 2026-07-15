/*
    rc_main task
*/

#include "task/user_task.h"

#include "device/dr16.h"
#ifndef RC_MAIN_RF_REMOTE_ENABLE
#define RC_MAIN_RF_REMOTE_ENABLE (1u)
#endif

#if RC_MAIN_RF_REMOTE_ENABLE
#include "device/rf_remote.h"
#include "component/crc8.h"
#include "module/mrlink_pc_comm/mrlink_pc_comm.h"
#endif
#include "module/rc_cmd_center/rc_cmd_center_app.h"

#define RC_MAIN_DR16_OFFLINE_TIMEOUT_US (100000ULL)

DR16_t dr16;

#if RC_MAIN_RF_REMOTE_ENABLE
static volatile bool rc_remote_kfs_pending = false;
static volatile bool rc_remote_reset_pending = false;
static uint8_t rc_remote_kfs_side = 0u;
static uint8_t rc_remote_kfs_msg_id = 0u;
static uint8_t rc_remote_kfs_cells[ORE_INFO_POSITION_COUNT] = {0};

void RF_Remote_OnKfsFrame(uint8_t side,
                          const uint8_t cells[RF_REMOTE_CELL_NUM],
                          uint8_t msg_id) {
  if (cells == NULL) return;

  rc_remote_kfs_side = side;
  rc_remote_kfs_msg_id = msg_id;
  for (uint8_t i = 0u; i < ORE_INFO_POSITION_COUNT; ++i) {
    rc_remote_kfs_cells[i] = cells[i];
  }
  rc_remote_kfs_pending = true;
}

void RF_Remote_OnRetryFrame(uint8_t mode, uint8_t msg_id) {
  (void)msg_id;
  switch ((RF_Remote_Mode_t)mode) {
    case RF_REMOTE_MODE_RELAX:
      RcCmdCenterApp_SetRfBehavior(RC_CMD_CENTER_RF_BEHAVIOR_RELAX);
      break;
    case RF_REMOTE_MODE_LOCK:
      RcCmdCenterApp_SetRfBehavior(RC_CMD_CENTER_RF_BEHAVIOR_LOCK);
      break;
    case RF_REMOTE_MODE_RESET:
      rc_remote_reset_pending = true;
      break;
    case RF_REMOTE_MODE_START:
    case RF_REMOTE_MODE_RETRY_REGION_1:
    case RF_REMOTE_MODE_RETRY_REGION_3:
      (void)MrlinkPc_RequestStartMatch(1u);
      break;
    case RF_REMOTE_MODE_RETRY_REGION_2:
      (void)MrlinkPc_RequestRetryRegion2();
      break;
    default:
      break;
  }
}

static void RcMain_ApplyRemoteKfs(uint32_t now_ms) {
  uint8_t frame[ORE_INFO_FRAME_SIZE] = {0};

  frame[0] = ORE_INFO_FRAME_HEAD0;
  frame[1] = ORE_INFO_FRAME_HEAD1;
  frame[2] = ORE_INFO_CMD_MINE_INPUT;
  frame[3] = rc_remote_kfs_msg_id;
  frame[4] = rc_remote_kfs_side;
  for (uint8_t i = 0u; i < ORE_INFO_POSITION_COUNT; ++i) {
    frame[5u + i] = rc_remote_kfs_cells[i];
  }
  frame[ORE_INFO_CRC_OFFSET] =
      CRC8_Calc(frame, ORE_INFO_CRC_OFFSET, CRC8_INIT);

  g_ore_info_debug.last_rx_len = ORE_INFO_FRAME_SIZE;
  g_ore_info_debug.last_rx_raw_len = ORE_INFO_FRAME_SIZE;
  g_ore_info_debug.last_msg_id = rc_remote_kfs_msg_id;
  g_ore_info_debug.last_side = rc_remote_kfs_side;
  g_ore_info_debug.last_parse_status = (uint8_t)ORE_INFO_PARSE_STATUS_OK;
  g_ore_info_debug.last_ack_status = (uint8_t)ORE_INFO_ACK_STATUS_OK;
  g_ore_info_debug.info_valid = true;
  g_ore_info_debug.info_fresh = true;
  g_ore_info_debug.last_rx_ms = now_ms;
  g_ore_info_debug.last_rx_raw_ms = now_ms;
  g_ore_info_debug.last_rx_age_ms = 0u;
  g_ore_info_debug.last_rx_raw_age_ms = 0u;
  g_ore_info_debug.rx_count++;
  g_ore_info_debug.frame_rx_count++;

  for (uint8_t i = 0u; i < ORE_INFO_POSITION_COUNT; ++i) {
    g_ore_info_debug.ore_type[i] = rc_remote_kfs_cells[i];
  }
  for (uint8_t i = 0u; i < ORE_INFO_FRAME_SIZE; ++i) {
    g_ore_info_debug.raw_frame[i] = frame[i];
    g_ore_info_debug.last_rx_raw_data[i] = frame[i];
  }
}

static void RcMain_ServiceRemoteKfs(void) {
  if (!rc_remote_kfs_pending || !g_ore_info_debug.inited) return;

  rc_remote_kfs_pending = false;
  RcMain_ApplyRemoteKfs(BSP_TIME_Get_ms());
}

static void RcMain_ServiceRemoteReset(void) {
  if (!rc_remote_reset_pending || RF_Remote_HasPendingAck()) return;

  rc_remote_reset_pending = false;
  RcCmdCenterApp_RequestReset();
}
#endif

static bool RcMain_Dr16TimedOut(uint64_t now_us) {
  return dr16.header.online &&
         (now_us - dr16.header.last_online_time) >
             RC_MAIN_DR16_OFFLINE_TIMEOUT_US;
}

void Task_rc_main(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / RC_MAIN_FREQ;
  osDelay(RC_MAIN_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  if (DR16_Init(&dr16) != DEVICE_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }
  if (DR16_StartDmaRecv(&dr16) != DEVICE_OK) {
    DR16_Restart();
    if (DR16_StartDmaRecv(&dr16) != DEVICE_OK) {
      osThreadTerminate(osThreadGetId());
      return;
    }
  }

#if RC_MAIN_RF_REMOTE_ENABLE
  if (RF_Remote_Init() == DEVICE_OK) {
    if (RF_Remote_StartDmaRecv() != DEVICE_OK) {
      RF_Remote_Restart();
      (void)RF_Remote_StartDmaRecv();
    }
  }
#endif

  RcCmdCenterApp_Init();

  while (1) {
    const uint32_t profile_start_us =
        Task_ProfilerLoopBegin(TASK_PROFILE_RC_MAIN,
                               TASK_PERIOD_US(RC_MAIN_FREQ));
    tick += delay_tick;
    if (DR16_WaitDmaCplt(0)) {
      if (DR16_ParseData(&dr16) != DEVICE_OK) {
        DR16_Restart();
      }
      if (DR16_StartDmaRecv(&dr16) != DEVICE_OK) {
        DR16_Restart();
        (void)DR16_StartDmaRecv(&dr16);
      }
    }

    DR16_Service(&dr16);

    if (RcMain_Dr16TimedOut(BSP_TIME_Get_us())) {
      DR16_RecordTimeoutOffline();
      DR16_Offline(&dr16);
      DR16_Restart();
      (void)DR16_StartDmaRecv(&dr16);
    }

#if RC_MAIN_RF_REMOTE_ENABLE
    (void)RF_Remote_Poll(RC_MAIN_DR16_OFFLINE_TIMEOUT_US);
    RcMain_ServiceRemoteKfs();
  RcMain_ServiceRemoteReset();
#endif

    RcCmdCenterApp_Update(BSP_TIME_Get_ms());

    task_runtime.heartbeat.rc_main++;
    Task_ProfilerLoopEnd(TASK_PROFILE_RC_MAIN, profile_start_us);
    Task_DelayUntil(TASK_PROFILE_RC_MAIN, &tick, delay_tick);
  }
}
