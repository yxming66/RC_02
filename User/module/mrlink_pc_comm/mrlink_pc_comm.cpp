#include "module/mrlink_pc_comm/mrlink_pc_comm.h"

#include <cstddef>
#include <cstring>

#include "bsp/time.h"
#include "bsp/uart.h"
#include "cmsis_compiler.h"
#include "cmsis_os2.h"
#include "device/mrlink/mrlink.hpp"
#include "main.h"

namespace {

constexpr uint32_t kRxCompleteFlag = (1u << 0);
constexpr uint32_t kHeartbeatTimeoutMs = 500u;
constexpr uint8_t kRxDmaSlotCount = 4u;
constexpr uint16_t kRxDmaBufSize = 256u;
constexpr uint16_t kMrlinkRxBufSize = 512u;
constexpr uint16_t kMrlinkTxBufSize = MRLINK_PC_MAX_FRAME_SIZE;

struct HeartbeatWire {};

template <typename T>
void CopyPlainToVolatile(volatile T *dst, const T *src) {
  volatile uint8_t *dst_bytes = reinterpret_cast<volatile uint8_t *>(dst);
  const uint8_t *src_bytes = reinterpret_cast<const uint8_t *>(src);
  for (std::size_t i = 0u; i < sizeof(T); i++) {
    dst_bytes[i] = src_bytes[i];
  }
}

struct __attribute__((packed)) PoleCmdWire {
  uint8_t mode;
  float lift0;
  float lift1;
};

struct __attribute__((packed)) ArmSimpleCmdWire {
  uint8_t mode;
  uint8_t point_mode;
  uint8_t suction;
  float target_joint1_rad;
  float target_joint2_rad;
};

struct __attribute__((packed)) RodNewCmdWire {
  uint8_t mode;
  uint8_t pose;
  uint8_t grip;
  float target_angle_rad;
};

struct __attribute__((packed)) OreStoreCmdWire {
  uint8_t mode;
  uint8_t force_rehome;
  float platform_target_rad;
  float gate_target_rad0;
  float gate_target_rad1;
  float track_target_rad0;
  float track_target_rad1;
};

struct __attribute__((packed)) StepCmdWire {
  uint8_t template_id;
  uint8_t travel_dir;
  float target_yaw_rad;
  float yaw_tolerance_rad;
};

struct __attribute__((packed)) ArmSimpleFeedbackWire {
  uint8_t mode;
  uint8_t point_mode;
  uint8_t suction;
  uint8_t joint1_temperature_warning;
  uint8_t joint1_temperature_over_limit;
  uint8_t joint1_temperature_limit_latched;
  float joint1_angle_rad;
  float joint1_velocity_rad_s;
  float joint1_temperature_c;
  float joint2_angle_rad;
  float target_joint1_rad;
  float target_joint2_rad;
};

struct __attribute__((packed)) RodNewFeedbackWire {
  uint8_t mode;
  uint8_t pose;
  uint8_t grip;
  uint8_t at_target;
  float target_angle_rad;
  float tracked_angle_rad;
  float tracked_velocity_rad_s;
  float feedback_angle_rad;
};

struct __attribute__((packed)) OreStoreFeedbackWire {
  uint8_t mode;
  uint8_t all_homed;
  uint8_t online_mask;
  uint8_t homed_mask;
  float platform_position_rad;
  float gate_position_rad0;
  float gate_position_rad1;
  float track_position_rad0;
  float track_position_rad1;
};

struct __attribute__((packed)) StepFeedbackWire {
  uint8_t state;
  uint8_t result;
  uint8_t fault;
  uint8_t template_id;
  uint8_t step_index;
  uint8_t reserved;
  float progress;
};

struct __attribute__((packed)) StatusFeedbackWire {
  uint8_t online;
  uint32_t recv_count;
  float cpu_temp;
  uint8_t command_source;
};

mr::link::Instance s_link;
MrlinkPc_State_t s_state{};
static uint8_t s_mrlink_rx_buf[kMrlinkRxBufSize];
static uint8_t s_mrlink_tx_buf[kMrlinkTxBufSize];
static uint8_t s_rx_dma_buf[kRxDmaSlotCount][kRxDmaBufSize];
static uint8_t s_rx_parse_buf[kRxDmaBufSize];
static volatile bool s_rx_active = false;
static volatile uint8_t s_rx_dma_write_idx = 0;
static volatile uint8_t s_rx_dma_read_idx = 0;
static volatile uint16_t s_rx_dma_len[kRxDmaSlotCount] = {0};
static osThreadId_t s_thread_id = nullptr;
static PC_AutoStepParams_t s_auto_step_params{};

uint16_t DebugCopyRaw(volatile uint8_t *dst, uint16_t dst_size,
                      const uint8_t *src, uint16_t src_len) {
  uint16_t copy_len = src_len;
  if (copy_len > dst_size) {
    copy_len = dst_size;
  }
  for (uint16_t i = 0; i < copy_len; ++i) {
    dst[i] = src[i];
  }
  for (uint16_t i = copy_len; i < dst_size; ++i) {
    dst[i] = 0;
  }
  return copy_len;
}

void DebugUpdateUsartConfig() {
  UART_HandleTypeDef *huart = BSP_UART_GetHandle(BSP_UART_PC);
  if (huart == nullptr) {
    g_pc_comm_debug.usart10_config_ok = 0;
    return;
  }

  g_pc_comm_debug.usart10_baudrate = huart->Init.BaudRate;
  g_pc_comm_debug.usart10_word_length = huart->Init.WordLength;
  g_pc_comm_debug.usart10_stop_bits = huart->Init.StopBits;
  g_pc_comm_debug.usart10_parity = huart->Init.Parity;
  g_pc_comm_debug.usart10_mode = huart->Init.Mode;
  g_pc_comm_debug.usart10_config_ok =
      (huart->Instance == USART10 && huart->Init.BaudRate == 115200u &&
       huart->Init.WordLength == UART_WORDLENGTH_8B &&
       huart->Init.StopBits == UART_STOPBITS_1 &&
       huart->Init.Parity == UART_PARITY_NONE)
          ? 1u
          : 0u;
}

void MarkRxFrame(uint8_t cmd, uint16_t payload_len, int8_t result) {
  g_pc_comm_debug.rx_batch_frame_count++;
  g_pc_comm_debug.last_rx_result = result;
  g_pc_comm_debug.last_rx_cmd = cmd;
  g_pc_comm_debug.last_rx_len = static_cast<uint8_t>(payload_len);
  g_pc_comm_debug.last_rx_frame_size =
      static_cast<uint16_t>(MRLINK_HEADER_LEN + MRLINK_LEN_FIELD_LEN +
                            MRLINK_CMD_FIELD_LEN + payload_len +
                            MRLINK_CRC_LEN);
  g_pc_comm_debug.last_rx_crc_received = 0u;
  g_pc_comm_debug.last_rx_crc_calculated = 0u;
}

void TouchOnline(uint8_t cmd) {
  const uint32_t now_ms = BSP_TIME_Get_ms();
  s_state.last_heartbeat_tick = now_ms;
  s_state.last_recv_time = now_ms;
  s_state.heartbeat_valid = true;
  s_state.control_mode = PC_MODE_PC;
  s_state.online = true;
  s_state.recv_count++;

  switch (cmd) {
    case PC_CMD_HEARTBEAT:
      g_pc_comm_debug.rx_heartbeat_count++;
      break;
    case PC_CMD_CHASSIS:
      g_pc_comm_debug.rx_chassis_count++;
      break;
    case PC_CMD_POLE:
      g_pc_comm_debug.rx_pole_count++;
      break;
    case PC_CMD_ARM_SIMPLE:
      g_pc_comm_debug.rx_arm_simple_count++;
      break;
    case PC_CMD_ROD_NEW:
      g_pc_comm_debug.rx_rod_new_count++;
      break;
    case PC_CMD_ORE_STORE:
      g_pc_comm_debug.rx_ore_store_count++;
      break;
    case PC_CMD_STEP:
      g_pc_comm_debug.rx_step_count++;
      break;
    case PC_CMD_IMU:
      g_pc_comm_debug.rx_imu_count++;
      break;
    default:
      break;
  }
}

void OnHeartbeat() {
  MarkRxFrame(PC_CMD_HEARTBEAT, 0u, MRLINK_OK);
  TouchOnline(PC_CMD_HEARTBEAT);
}

void OnChassis(const PC_ChassisCMD_t &cmd) {
  s_state.cmd.chassis = cmd;
  MarkRxFrame(PC_CMD_CHASSIS, sizeof(cmd), MRLINK_OK);
  TouchOnline(PC_CMD_CHASSIS);
}

void OnPole(const PoleCmdWire &wire) {
  s_state.cmd.pole.mode = wire.mode;
  s_state.cmd.pole.lift[0] = wire.lift0;
  s_state.cmd.pole.lift[1] = wire.lift1;
  MarkRxFrame(PC_CMD_POLE, sizeof(wire), MRLINK_OK);
  TouchOnline(PC_CMD_POLE);
}

void OnArmSimple(const ArmSimpleCmdWire &wire) {
  s_state.cmd.arm_simple.mode = wire.mode;
  s_state.cmd.arm_simple.point_mode = wire.point_mode;
  s_state.cmd.arm_simple.suction = wire.suction;
  s_state.cmd.arm_simple.target_joint1_rad = wire.target_joint1_rad;
  s_state.cmd.arm_simple.target_joint2_rad = wire.target_joint2_rad;
  MarkRxFrame(PC_CMD_ARM_SIMPLE, sizeof(wire), MRLINK_OK);
  TouchOnline(PC_CMD_ARM_SIMPLE);
}

void OnRodNew(const RodNewCmdWire &wire) {
  s_state.cmd.rod_new.mode = wire.mode;
  s_state.cmd.rod_new.pose = wire.pose;
  s_state.cmd.rod_new.grip = wire.grip;
  s_state.cmd.rod_new.target_angle_rad = wire.target_angle_rad;
  MarkRxFrame(PC_CMD_ROD_NEW, sizeof(wire), MRLINK_OK);
  TouchOnline(PC_CMD_ROD_NEW);
}

void OnOreStore(const OreStoreCmdWire &wire) {
  s_state.cmd.ore_store.mode = wire.mode;
  s_state.cmd.ore_store.force_rehome = wire.force_rehome;
  s_state.cmd.ore_store.platform_target_rad = wire.platform_target_rad;
  s_state.cmd.ore_store.gate_target_rad[0] = wire.gate_target_rad0;
  s_state.cmd.ore_store.gate_target_rad[1] = wire.gate_target_rad1;
  s_state.cmd.ore_store.track_target_rad[0] = wire.track_target_rad0;
  s_state.cmd.ore_store.track_target_rad[1] = wire.track_target_rad1;
  MarkRxFrame(PC_CMD_ORE_STORE, sizeof(wire), MRLINK_OK);
  TouchOnline(PC_CMD_ORE_STORE);
}

void OnStep(const StepCmdWire &wire) {
  s_state.cmd.step.template_id =
      static_cast<PC_StepTemplate_t>(wire.template_id);
  s_state.cmd.step.travel_dir = static_cast<PC_StepDir_t>(wire.travel_dir);
  s_state.cmd.step.target_yaw_rad = wire.target_yaw_rad;
  s_state.cmd.step.yaw_tolerance_rad = wire.yaw_tolerance_rad;
  MarkRxFrame(PC_CMD_STEP, sizeof(wire), MRLINK_OK);
  TouchOnline(PC_CMD_STEP);
}

void OnImu(const PC_ImuCMD_t &cmd) {
  s_state.cmd.imu = cmd;
  MarkRxFrame(PC_CMD_IMU, sizeof(cmd), MRLINK_OK);
  TouchOnline(PC_CMD_IMU);
}

bool RegisterHandlers() {
  return s_link.On(PC_CMD_HEARTBEAT, OnHeartbeat) == MRLINK_OK &&
         s_link.On<PC_ChassisCMD_t>(PC_CMD_CHASSIS, OnChassis) == MRLINK_OK &&
         s_link.On<PoleCmdWire>(PC_CMD_POLE, OnPole) == MRLINK_OK &&
         s_link.On<ArmSimpleCmdWire>(PC_CMD_ARM_SIMPLE, OnArmSimple) ==
             MRLINK_OK &&
         s_link.On<RodNewCmdWire>(PC_CMD_ROD_NEW, OnRodNew) == MRLINK_OK &&
         s_link.On<OreStoreCmdWire>(PC_CMD_ORE_STORE, OnOreStore) ==
             MRLINK_OK &&
         s_link.On<StepCmdWire>(PC_CMD_STEP, OnStep) == MRLINK_OK &&
         s_link.On<PC_ImuCMD_t>(PC_CMD_IMU, OnImu) == MRLINK_OK;
}

bool StartRecv() {
  const uint8_t idx = s_rx_dma_write_idx;
  if (s_rx_dma_len[idx] != 0u) {
    g_pc_comm_debug.rx_queue_overflow_count++;
    s_rx_dma_len[idx] = 0u;
    if (s_rx_dma_read_idx == idx) {
      s_rx_dma_read_idx = static_cast<uint8_t>((idx + 1u) % kRxDmaSlotCount);
    }
  }

  int8_t ret = BSP_UART_ReceiveToIdle(BSP_UART_PC, s_rx_dma_buf[idx],
                                      sizeof(s_rx_dma_buf[idx]), true);
  if (ret == BSP_OK) {
    s_rx_active = true;
    return true;
  }

  UART_HandleTypeDef *huart = BSP_UART_GetHandle(BSP_UART_PC);
  if (huart != nullptr) {
    (void)HAL_UART_AbortReceive(huart);
    ret = BSP_UART_ReceiveToIdle(BSP_UART_PC, s_rx_dma_buf[idx],
                                 sizeof(s_rx_dma_buf[idx]), true);
  }
  s_rx_active = (ret == BSP_OK);
  return s_rx_active;
}

void RxEventCallback(uint16_t size) {
  s_rx_active = false;
  const uint8_t ready_idx = s_rx_dma_write_idx;

  if (size > sizeof(s_rx_dma_buf[ready_idx])) {
    g_pc_comm_debug.rx_dma_start_fail_count++;
    (void)StartRecv();
    return;
  }

  if (size > 0u) {
    s_rx_dma_len[ready_idx] = size;
    g_pc_comm_debug.rx_irq_count++;
    g_pc_comm_debug.rx_irq_byte_count += size;

    const uint8_t next_idx =
        static_cast<uint8_t>((ready_idx + 1u) % kRxDmaSlotCount);
    if (s_rx_dma_len[next_idx] != 0u) {
      g_pc_comm_debug.rx_queue_overflow_count++;
      s_rx_dma_len[next_idx] = 0u;
      if (s_rx_dma_read_idx == next_idx) {
        s_rx_dma_read_idx =
            static_cast<uint8_t>((next_idx + 1u) % kRxDmaSlotCount);
      }
    }
    s_rx_dma_write_idx = next_idx;
  }

  if (!StartRecv()) {
    g_pc_comm_debug.rx_dma_start_fail_count++;
  }

  if (size > 0u && s_thread_id != nullptr) {
    (void)osThreadFlagsSet(s_thread_id, kRxCompleteFlag);
  }
}

uint16_t PopRxChunk(uint8_t *dst, uint16_t dst_size) {
  if (dst == nullptr || dst_size == 0u) {
    return 0u;
  }

  uint16_t len = 0u;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint8_t idx = s_rx_dma_read_idx;
  len = s_rx_dma_len[idx];
  if (len > 0u) {
    if (len > dst_size) {
      len = dst_size;
    }
    std::memcpy(dst, s_rx_dma_buf[idx], len);
    s_rx_dma_len[idx] = 0u;
    s_rx_dma_read_idx = static_cast<uint8_t>((idx + 1u) % kRxDmaSlotCount);
  }
  if (primask == 0u) {
    __enable_irq();
  }
  return len;
}

bool WaitRecvComplete(uint32_t timeout_ms) {
  const uint32_t flags =
      osThreadFlagsWait(kRxCompleteFlag, osFlagsWaitAny, timeout_ms);
  return (flags & kRxCompleteFlag) != 0u;
}

auto_ctrl_template_e MapTemplate(PC_StepTemplate_t pc_template) {
  switch (pc_template) {
    case PC_STEP_TEMPLATE_ASCEND_200_HEAD:
      return AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD;
    case PC_STEP_TEMPLATE_ASCEND_400_HEAD:
      return AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD;
    case PC_STEP_TEMPLATE_DESCEND_200_HEAD:
      return AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD;
    case PC_STEP_TEMPLATE_DESCEND_400_HEAD:
      return AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD;
    case PC_STEP_TEMPLATE_ASCEND_200_TAIL:
      return AUTO_CTRL_TEMPLATE_ASCEND_200_TAIL;
    case PC_STEP_TEMPLATE_ASCEND_400_TAIL:
      return AUTO_CTRL_TEMPLATE_ASCEND_400_TAIL;
    case PC_STEP_TEMPLATE_DESCEND_200_TAIL:
      return AUTO_CTRL_TEMPLATE_DESCEND_200_TAIL;
    case PC_STEP_TEMPLATE_DESCEND_400_TAIL:
      return AUTO_CTRL_TEMPLATE_DESCEND_400_TAIL;
    default:
      return AUTO_CTRL_TEMPLATE_NONE;
  }
}

auto_ctrl_travel_dir_e MapTravelDir(PC_StepDir_t pc_dir) {
  return (pc_dir == PC_STEP_DIR_TAIL_FORWARD)
             ? AUTO_CTRL_TRAVEL_DIR_TAIL_FORWARD
             : AUTO_CTRL_TRAVEL_DIR_HEAD_FORWARD;
}

void CopyFrame(uint8_t *dst, uint16_t buf_size, uint16_t frame_len) {
  if (dst != nullptr && frame_len <= buf_size && frame_len <= kMrlinkTxBufSize) {
    std::memcpy(dst, s_mrlink_tx_buf, frame_len);
  }
}

template <typename T>
uint16_t BuildWire(uint8_t cmd, const T &wire, uint8_t *tx_buf,
                   uint16_t buf_size) {
  const uint16_t frame_len =
      s_link.Build(cmd, reinterpret_cast<const uint8_t *>(&wire), sizeof(T));
  CopyFrame(tx_buf, buf_size, frame_len);
  return (frame_len <= buf_size) ? frame_len : 0u;
}

}  // namespace

volatile PC_CommDebug_t g_pc_comm_debug;

extern "C" bool MrlinkPc_CommInit(void) {
  s_thread_id = osThreadGetId();
  if (s_thread_id == nullptr) {
    g_pc_comm_debug.last_init_error = -21;
    return false;
  }

  std::memset(&s_state, 0, sizeof(s_state));
  s_state.control_mode = PC_MODE_RC;
  s_state.feedback.status.command_source = PC_COMMAND_SOURCE_RC;

  MrLink_Config_t cfg = {};
  cfg.max_payload_size = MRLINK_PC_MAX_PAYLOAD_SIZE;
  cfg.use_crc16 = true;
  const int8_t init_ret = s_link.Init(&cfg, s_mrlink_rx_buf,
                                      sizeof(s_mrlink_rx_buf),
                                      s_mrlink_tx_buf,
                                      sizeof(s_mrlink_tx_buf));
  if (init_ret != MRLINK_OK) {
    g_pc_comm_debug.last_init_error = init_ret;
    return false;
  }

  if (!RegisterHandlers()) {
    g_pc_comm_debug.last_init_error = -24;
    return false;
  }

  if (BSP_UART_RegisterRxEventCallback(BSP_UART_PC, RxEventCallback) !=
      BSP_OK) {
    g_pc_comm_debug.last_init_error = -22;
    return false;
  }

  if (!StartRecv()) {
    g_pc_comm_debug.last_init_error = -23;
    return false;
  }

  g_pc_comm_debug.last_init_error = 0;
  return true;
}

extern "C" void MrlinkPc_CommProcess(uint32_t now_ms) {
  if (s_state.last_heartbeat_tick > 0u &&
      (now_ms - s_state.last_heartbeat_tick) > kHeartbeatTimeoutMs) {
    s_state.heartbeat_valid = false;
    s_state.control_mode = PC_MODE_RC;
    s_state.online = false;
  }

  (void)WaitRecvComplete(0u);
  if (!s_rx_active && !StartRecv()) {
    g_pc_comm_debug.rx_restart_fail_count++;
  }

  while (true) {
    const uint16_t rx_len = PopRxChunk(s_rx_parse_buf, sizeof(s_rx_parse_buf));
    if (rx_len == 0u) {
      break;
    }

    g_pc_comm_debug.rx_batch_count++;
    g_pc_comm_debug.rx_batch_len = rx_len;
    g_pc_comm_debug.rx_batch_frame_count = 0;
    g_pc_comm_debug.rx_batch_raw_len = DebugCopyRaw(
        g_pc_comm_debug.rx_batch_raw, PC_COMM_DEBUG_RX_RAW_SIZE,
        s_rx_parse_buf, rx_len);
    g_pc_comm_debug.last_rx_raw_len = DebugCopyRaw(
        g_pc_comm_debug.last_rx_raw, MRLINK_PC_MAX_FRAME_SIZE,
        s_rx_parse_buf, rx_len);

    const uint32_t prev_ok = s_link.GetStats()->frame_rx_ok;
    (void)s_link.FeedBytes(s_rx_parse_buf, rx_len);
    const MrLink_Stats_t *stats = s_link.GetStats();
    if (stats->frame_rx_ok == prev_ok) {
      s_state.error_count++;
      g_pc_comm_debug.last_rx_result = MRLINK_ERR;
    }
  }

  MrlinkPc_DebugUpdate();
}

extern "C" bool MrlinkPc_CommIsOnline(void) { return s_state.online; }

extern "C" const MrlinkPc_State_t *MrlinkPc_GetState(void) {
  return &s_state;
}

extern "C" void MrlinkPc_DebugUpdate(void) {
  DebugUpdateUsartConfig();
  g_pc_comm_debug.online = s_state.online ? 1u : 0u;
  g_pc_comm_debug.heartbeat_valid = s_state.heartbeat_valid ? 1u : 0u;
  g_pc_comm_debug.control_mode = static_cast<uint8_t>(s_state.control_mode);
  g_pc_comm_debug.command_source =
      static_cast<uint8_t>(s_state.feedback.status.command_source);
  g_pc_comm_debug.last_heartbeat_tick = s_state.last_heartbeat_tick;
  g_pc_comm_debug.last_recv_time = s_state.last_recv_time;
  g_pc_comm_debug.recv_count = s_state.recv_count;
  g_pc_comm_debug.error_count = s_state.error_count;
  CopyPlainToVolatile(&g_pc_comm_debug.rx_chassis, &s_state.cmd.chassis);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_pole, &s_state.cmd.pole);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_arm_simple, &s_state.cmd.arm_simple);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_rod_new, &s_state.cmd.rod_new);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_ore_store, &s_state.cmd.ore_store);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_step, &s_state.cmd.step);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_imu, &s_state.cmd.imu);
  const MrLink_Stats_t *stats = s_link.GetStats();
  if (stats != nullptr) {
    CopyPlainToVolatile(&g_pc_comm_debug.mrlink_stats, stats);
    g_pc_comm_debug.rx_header_skip_count = stats->frame_rx_header_skip;
  }
}

extern "C" bool MrlinkPc_IsPCControlMode(void) {
  return s_state.control_mode == PC_MODE_PC;
}

extern "C" bool MrlinkPc_IsHeartbeatValid(void) {
  return s_state.heartbeat_valid;
}

extern "C" const PC_ChassisCMD_t *MrlinkPc_GetChassisCMD(void) {
  return &s_state.cmd.chassis;
}

extern "C" const PC_PoleCMD_t *MrlinkPc_GetPoleCMD(void) {
  return &s_state.cmd.pole;
}

extern "C" const PC_ArmCMD_t *MrlinkPc_GetArmCMD(void) {
  return &s_state.cmd.arm;
}

extern "C" const PC_ArmSimpleCMD_t *MrlinkPc_GetArmSimpleCMD(void) {
  return &s_state.cmd.arm_simple;
}

extern "C" const PC_RodNewCMD_t *MrlinkPc_GetRodNewCMD(void) {
  return &s_state.cmd.rod_new;
}

extern "C" const PC_OreStoreCMD_t *MrlinkPc_GetOreStoreCMD(void) {
  return &s_state.cmd.ore_store;
}

extern "C" const PC_StepCMD_t *MrlinkPc_GetStepCMD(void) {
  return &s_state.cmd.step;
}

extern "C" const PC_ImuCMD_t *MrlinkPc_GetImuCMD(void) {
  return &s_state.cmd.imu;
}

extern "C" void MrlinkPc_SetChassisFeedback(
    const PC_ChassisFeedback_t *fb) {
  if (fb != nullptr) {
    s_state.feedback.chassis = *fb;
  }
}

extern "C" void MrlinkPc_SetPoleFeedback(const PC_PoleFeedback_t *fb) {
  if (fb != nullptr) {
    s_state.feedback.pole = *fb;
  }
}

extern "C" void MrlinkPc_SetArmFeedback(const PC_ArmFeedback_t *fb) {
  if (fb != nullptr) {
    s_state.feedback.arm = *fb;
  }
}

extern "C" void MrlinkPc_SetArmSimpleFeedback(
    const PC_ArmSimpleFeedback_t *fb) {
  if (fb != nullptr) {
    s_state.feedback.arm_simple = *fb;
  }
}

extern "C" void MrlinkPc_SetRodNewFeedback(
    const PC_RodNewFeedback_t *fb) {
  if (fb != nullptr) {
    s_state.feedback.rod_new = *fb;
  }
}

extern "C" void MrlinkPc_SetOreStoreFeedback(
    const PC_OreStoreFeedback_t *fb) {
  if (fb != nullptr) {
    s_state.feedback.ore_store = *fb;
  }
}

extern "C" void MrlinkPc_SetStepFeedback(const PC_StepFeedback_t *fb) {
  if (fb != nullptr) {
    s_state.feedback.step = *fb;
  }
}

extern "C" void MrlinkPc_SetStatusFeedback(const PC_StatusFeedback_t *fb) {
  if (fb != nullptr) {
    s_state.feedback.status = *fb;
  }
}

extern "C" const PC_AutoStepParams_t *MrlinkPc_GetAutoStepParams(void) {
  const PC_StepCMD_t *step_cmd = &s_state.cmd.step;
  if (step_cmd->template_id == PC_STEP_TEMPLATE_NONE) {
    return nullptr;
  }

  s_auto_step_params.template_id = MapTemplate(step_cmd->template_id);
  if (s_auto_step_params.template_id == AUTO_CTRL_TEMPLATE_NONE) {
    return nullptr;
  }

  s_auto_step_params.travel_dir = MapTravelDir(step_cmd->travel_dir);
  s_auto_step_params.target_yaw_rad = step_cmd->target_yaw_rad;
  s_auto_step_params.yaw_tolerance_rad = step_cmd->yaw_tolerance_rad;
  return &s_auto_step_params;
}

extern "C" void MrlinkPc_ClearStepCommand(void) {
  s_state.cmd.step.template_id = PC_STEP_TEMPLATE_NONE;
}

extern "C" uint16_t MrlinkPc_BuildFeedbackFrame(uint8_t cmd, uint8_t *tx_buf,
                                                 uint16_t buf_size) {
  switch (cmd) {
    case PC_FEEDBACK_HEARTBEAT:
    {
      const uint16_t frame_len = s_link.Build(cmd, nullptr, 0u);
      CopyFrame(tx_buf, buf_size, frame_len);
      return (frame_len <= buf_size) ? frame_len : 0u;
    }
    case PC_FEEDBACK_CHASSIS:
      return BuildWire(cmd, s_state.feedback.chassis, tx_buf, buf_size);
    case PC_FEEDBACK_POLE:
      return BuildWire(cmd, s_state.feedback.pole, tx_buf, buf_size);
    case PC_FEEDBACK_ARM_SIMPLE: {
      ArmSimpleFeedbackWire wire = {};
      wire.mode = s_state.feedback.arm_simple.mode;
      wire.point_mode = s_state.feedback.arm_simple.point_mode;
      wire.suction = s_state.feedback.arm_simple.suction;
      wire.joint1_temperature_warning =
        s_state.feedback.arm_simple.joint1_temperature_warning;
      wire.joint1_temperature_over_limit =
        s_state.feedback.arm_simple.joint1_temperature_over_limit;
      wire.joint1_temperature_limit_latched =
        s_state.feedback.arm_simple.joint1_temperature_limit_latched;
      wire.joint1_angle_rad = s_state.feedback.arm_simple.joint1_angle_rad;
      wire.joint1_velocity_rad_s =
        s_state.feedback.arm_simple.joint1_velocity_rad_s;
      wire.joint1_temperature_c =
        s_state.feedback.arm_simple.joint1_temperature_c;
      wire.joint2_angle_rad = s_state.feedback.arm_simple.joint2_angle_rad;
      wire.target_joint1_rad =
        s_state.feedback.arm_simple.target_joint1_rad;
      wire.target_joint2_rad =
        s_state.feedback.arm_simple.target_joint2_rad;
      return BuildWire(cmd, wire, tx_buf, buf_size);
    }
    case PC_FEEDBACK_ROD_NEW: {
      RodNewFeedbackWire wire = {};
      wire.mode = s_state.feedback.rod_new.mode;
      wire.pose = s_state.feedback.rod_new.pose;
      wire.grip = s_state.feedback.rod_new.grip;
      wire.at_target = s_state.feedback.rod_new.at_target;
      wire.target_angle_rad = s_state.feedback.rod_new.target_angle_rad;
      wire.tracked_angle_rad = s_state.feedback.rod_new.tracked_angle_rad;
      wire.tracked_velocity_rad_s =
        s_state.feedback.rod_new.tracked_velocity_rad_s;
      wire.feedback_angle_rad = s_state.feedback.rod_new.feedback_angle_rad;
      return BuildWire(cmd, wire, tx_buf, buf_size);
    }
    case PC_FEEDBACK_ORE_STORE: {
      OreStoreFeedbackWire wire = {};
      wire.mode = s_state.feedback.ore_store.mode;
      wire.all_homed = s_state.feedback.ore_store.all_homed;
      wire.online_mask = s_state.feedback.ore_store.online_mask;
      wire.homed_mask = s_state.feedback.ore_store.homed_mask;
      wire.platform_position_rad =
        s_state.feedback.ore_store.platform_position_rad;
      wire.gate_position_rad0 = s_state.feedback.ore_store.gate_position_rad[0];
      wire.gate_position_rad1 = s_state.feedback.ore_store.gate_position_rad[1];
      wire.track_position_rad0 =
        s_state.feedback.ore_store.track_position_rad[0];
      wire.track_position_rad1 =
        s_state.feedback.ore_store.track_position_rad[1];
      return BuildWire(cmd, wire, tx_buf, buf_size);
    }
    case PC_FEEDBACK_STEP: {
      StepFeedbackWire wire = {};
      wire.state = static_cast<uint8_t>(s_state.feedback.step.state);
      wire.result = static_cast<uint8_t>(s_state.feedback.step.result);
      wire.fault = static_cast<uint8_t>(s_state.feedback.step.fault);
      wire.template_id = static_cast<uint8_t>(s_state.feedback.step.template_id);
      wire.step_index = s_state.feedback.step.step_index;
      wire.reserved = 0u;
      wire.progress = s_state.feedback.step.progress;
      return BuildWire(cmd, wire, tx_buf, buf_size);
    }
    case PC_FEEDBACK_STATUS: {
      StatusFeedbackWire wire = {};
      wire.online = s_state.feedback.status.online;
      wire.recv_count = s_state.feedback.status.recv_count;
      wire.cpu_temp = s_state.feedback.status.cpu_temp;
      wire.command_source =
        static_cast<uint8_t>(s_state.feedback.status.command_source);
      return BuildWire(cmd, wire, tx_buf, buf_size);
    }
    default:
      return 0u;
  }
}
