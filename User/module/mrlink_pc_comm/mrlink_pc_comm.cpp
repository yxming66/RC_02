#include "module/mrlink_pc_comm/mrlink_pc_comm.h"

#include <cstddef>
#include <cstring>

#include "bsp/time.h"
#include "bsp/uart.h"
#include "cmsis_os2.h"
#include "device/mrlink/mrlink_channel.h"
#include "device/mrlink/mrlink.hpp"
#include "main.h"
#include "module/mrlink_pc_comm/pc_messages.hpp"

namespace {

namespace wire = pc_comm::wire;

constexpr uint32_t kRxCompleteFlag = (1u << 0);
constexpr uint32_t kHeartbeatTimeoutMs = 500u;
constexpr uint8_t kRxDmaSlotCount = MRLINK_PC_RX_DMA_SLOT_COUNT;
constexpr uint16_t kRxDmaBufSize = MRLINK_PC_RX_DMA_BUF_SIZE;
constexpr uint16_t kMrlinkRxBufSize = MRLINK_PC_RX_STREAM_BUF_SIZE;
constexpr uint16_t kMrlinkTxBufSize = MRLINK_PC_MAX_FRAME_SIZE;
constexpr uint32_t kAutoActionRepeatReleaseMs = 300u;

static_assert(kRxDmaSlotCount >= 2u, "MRLINK_PC_RX_DMA_SLOT_COUNT must be >= 2");
static_assert(kRxDmaBufSize >= MRLINK_PC_MAX_FRAME_SIZE,
              "MRLINK_PC_RX_DMA_BUF_SIZE must fit one mrlink frame");
static_assert(kMrlinkRxBufSize >= MRLINK_PC_MAX_FRAME_SIZE,
              "MRLINK_PC_RX_STREAM_BUF_SIZE must fit one mrlink frame");

template <typename T>
void CopyPlainToVolatile(volatile T *dst, const T *src) {
  volatile uint8_t *dst_bytes = reinterpret_cast<volatile uint8_t *>(dst);
  const uint8_t *src_bytes = reinterpret_cast<const uint8_t *>(src);
  for (std::size_t i = 0u; i < sizeof(T); i++) {
    dst_bytes[i] = src_bytes[i];
  }
}

mr::link::Bus<kMrlinkRxBufSize,
              kMrlinkTxBufSize,
              kRxDmaBufSize,
              kRxDmaSlotCount,
              kRxDmaBufSize>
    s_bus;
MrlinkPc_State_t s_state{};
static uint8_t s_rx_parse_buf[kRxDmaBufSize];
static osThreadId_t s_thread_id = nullptr;
static PC_AutoStepParams_t s_auto_step_params{};
static PC_AutoActionFeedback_t s_auto_action_feedback{};
static uint8_t s_auto_action_rx_latch = PC_AUTO_ACTION_NONE;
static uint32_t s_auto_action_rx_latch_tick = 0u;
static PC_IrOreFeedback_t s_ir_ore_feedback{};
static PC_IrOreBridgeFeedback_t s_ir_ore_bridge_feedback{};
static MrlinkPc_TxCallback_t s_tx_done_callback = nullptr;
static MrlinkPc_TxCallback_t s_tx_error_callback = nullptr;
static BSP_UART_t s_channel_uart = BSP_UART_PC;
static bool s_comm_initialized = false;
static bool s_pole_cmd_received = false;
static bool s_ir_ore_ack_pending = false;

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

void MarkLastRxFrame(uint8_t cmd, uint16_t payload_len, int8_t result,
                     uint16_t crc_received, uint16_t crc_calculated);

void DebugUpdateUsartConfig() {
  UART_HandleTypeDef *huart = BSP_UART_GetHandle(s_channel_uart);
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
      (huart->Instance == USART1 && huart->Init.BaudRate == 115200u &&
       huart->Init.WordLength == UART_WORDLENGTH_8B &&
       huart->Init.StopBits == UART_STOPBITS_1 &&
       huart->Init.Parity == UART_PARITY_NONE)
          ? 1u
          : 0u;
}

void MarkRxFrame(uint8_t cmd, uint16_t payload_len, int8_t result) {
  g_pc_comm_debug.rx_batch_frame_count++;
  MarkLastRxFrame(cmd, payload_len, result, 0u, 0u);
}

void MarkLastRxFrame(uint8_t cmd, uint16_t payload_len, int8_t result,
                     uint16_t crc_received, uint16_t crc_calculated) {
  g_pc_comm_debug.last_rx_result = result;
  g_pc_comm_debug.last_rx_cmd = cmd;
  g_pc_comm_debug.last_rx_len = static_cast<uint8_t>(payload_len);
  g_pc_comm_debug.last_rx_frame_size =
      static_cast<uint16_t>(MRLINK_HEADER_LEN + MRLINK_LEN_FIELD_LEN +
                            MRLINK_CMD_FIELD_LEN + payload_len +
                            MRLINK_CRC_LEN);
  g_pc_comm_debug.last_rx_crc_received = crc_received;
  g_pc_comm_debug.last_rx_crc_calculated = crc_calculated;
}

void TouchOnline(uint8_t cmd) {
  /* Any valid PC frame keeps the PC link alive. PC_CMD_HEARTBEAT is the
   * recommended low-cost keepalive, but high-rate command frames also refresh
   * last_heartbeat_tick and keep control_mode in PC_MODE_PC. */
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
    case PC_CMD_AUTO_ACTION:
      g_pc_comm_debug.rx_auto_action_count++;
      break;
    case PC_CMD_CAMERA_YAW:
      g_pc_comm_debug.rx_camera_yaw_count++;
      break;
    case PC_CMD_ABSTRACT_POSITION:
      g_pc_comm_debug.rx_abstract_position_count++;
      break;
    case PC_CMD_STEP:
      g_pc_comm_debug.rx_step_count++;
      break;
    case PC_CMD_IMU:
      g_pc_comm_debug.rx_imu_count++;
      break;
    case PC_CMD_IR_ORE_ACK:
      g_pc_comm_debug.rx_ir_ore_ack_count++;
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

void OnPole(const wire::PoleCmd &cmd) {
  s_state.cmd.pole.mode = cmd.mode;
  s_state.cmd.pole.lift[0] = cmd.lift0;
  s_state.cmd.pole.lift[1] = cmd.lift1;
  s_pole_cmd_received = true;
  s_state.cmd.abstract_position.enable_mask &=
      static_cast<uint8_t>(~PC_ABSTRACT_MODULE_POLE);
  MarkRxFrame(PC_CMD_POLE, sizeof(cmd), MRLINK_OK);
  TouchOnline(PC_CMD_POLE);
}

void OnArmSimple(const wire::ArmSimpleCmd &cmd) {
  s_state.cmd.arm_simple.mode = cmd.mode;
  s_state.cmd.arm_simple.point_mode = cmd.point_mode;
  s_state.cmd.arm_simple.suction = cmd.suction;
  s_state.cmd.arm_simple.target_joint1_rad = cmd.target_joint1_rad;
  s_state.cmd.arm_simple.target_joint2_rad = cmd.target_joint2_rad;
  s_state.cmd.abstract_position.enable_mask &=
      static_cast<uint8_t>(~PC_ABSTRACT_MODULE_ARM_SIMPLE);
  MarkRxFrame(PC_CMD_ARM_SIMPLE, sizeof(cmd), MRLINK_OK);
  TouchOnline(PC_CMD_ARM_SIMPLE);
}

void OnRodNew(const wire::RodNewCmd &cmd) {
  s_state.cmd.rod_new.mode = cmd.mode;
  s_state.cmd.rod_new.pose = cmd.pose;
  s_state.cmd.rod_new.grip = cmd.grip;
  s_state.cmd.rod_new.target_angle_rad = cmd.target_angle_rad;
  s_state.cmd.abstract_position.enable_mask &=
      static_cast<uint8_t>(~PC_ABSTRACT_MODULE_ROD_NEW);
  MarkRxFrame(PC_CMD_ROD_NEW, sizeof(cmd), MRLINK_OK);
  TouchOnline(PC_CMD_ROD_NEW);
}

void OnOreStore(const wire::OreStoreCmd &cmd) {
  s_state.cmd.ore_store.mode = cmd.mode;
  s_state.cmd.ore_store.force_rehome = cmd.force_rehome;
  s_state.cmd.ore_store.platform_target_rad = cmd.platform_target_rad;
  s_state.cmd.abstract_position.enable_mask &=
      static_cast<uint8_t>(~PC_ABSTRACT_MODULE_ORE_STORE);
  MarkRxFrame(PC_CMD_ORE_STORE, sizeof(cmd), MRLINK_OK);
  TouchOnline(PC_CMD_ORE_STORE);
}

void OnAutoAction(const wire::AutoActionCmd &cmd) {
  const uint32_t now_ms = BSP_TIME_Get_ms();
  const bool latch_expired =
      s_auto_action_rx_latch != PC_AUTO_ACTION_NONE &&
      (now_ms - s_auto_action_rx_latch_tick) >= kAutoActionRepeatReleaseMs;

  /* Auto action is edge/latch based rather than a continuous setpoint:
   * - action=NONE clears the latch.
   * - a different non-zero action is accepted immediately.
   * - the same non-zero action is accepted again only after 300 ms, which
   *   prevents a 50 Hz host stream from repeatedly restarting one-shot flows. */
  if (cmd.action == PC_AUTO_ACTION_NONE) {
    s_auto_action_rx_latch = PC_AUTO_ACTION_NONE;
    s_auto_action_rx_latch_tick = now_ms;
    s_state.cmd.auto_action.action = PC_AUTO_ACTION_NONE;
  } else if (cmd.action != s_auto_action_rx_latch) {
    s_state.cmd.auto_action.action = cmd.action;
    s_auto_action_rx_latch = cmd.action;
    s_auto_action_rx_latch_tick = now_ms;
  } else if (latch_expired) {
    s_state.cmd.auto_action.action = cmd.action;
    s_auto_action_rx_latch_tick = now_ms;
  } else {
    s_auto_action_rx_latch_tick = now_ms;
  }
  MarkRxFrame(PC_CMD_AUTO_ACTION, sizeof(cmd), MRLINK_OK);
  TouchOnline(PC_CMD_AUTO_ACTION);
}

void OnCameraYaw(const wire::CameraYawCmd &cmd) {
  for (uint8_t yaw = 0u; yaw < PC_CAMERA_YAW_COUNT; ++yaw) {
    s_state.cmd.camera_yaw.mode[yaw] = cmd.mode[yaw];
    s_state.cmd.camera_yaw.target_yaw_rad[yaw] = cmd.target_yaw_rad[yaw];
  }
  MarkRxFrame(PC_CMD_CAMERA_YAW, sizeof(cmd), MRLINK_OK);
  TouchOnline(PC_CMD_CAMERA_YAW);
  s_state.camera_yaw_cmd_tick = BSP_TIME_Get_ms();
}

void OnAbstractPosition(const PC_AbstractPositionCMD_t &cmd) {
  s_state.cmd.abstract_position = cmd;
  MarkRxFrame(PC_CMD_ABSTRACT_POSITION, sizeof(cmd), MRLINK_OK);
  TouchOnline(PC_CMD_ABSTRACT_POSITION);
}

void OnStep(const wire::StepCmd &cmd) {
  s_state.cmd.step.template_id =
      static_cast<PC_StepTemplate_t>(cmd.template_id);
  s_state.cmd.step.travel_dir = static_cast<PC_StepDir_t>(cmd.travel_dir);
  s_state.cmd.step.target_yaw_rad = cmd.target_yaw_rad;
  s_state.cmd.step.yaw_tolerance_rad = cmd.yaw_tolerance_rad;
  MarkRxFrame(PC_CMD_STEP, sizeof(cmd), MRLINK_OK);
  TouchOnline(PC_CMD_STEP);
}

void OnImu(const PC_ImuCMD_t &cmd) {
  s_state.cmd.imu = cmd;
  MarkRxFrame(PC_CMD_IMU, sizeof(cmd), MRLINK_OK);
  TouchOnline(PC_CMD_IMU);
}

void OnIrOreAck(const PC_IrOreAckCMD_t &cmd) {
  s_state.cmd.ir_ore_ack = cmd;
  s_ir_ore_ack_pending = true;
  MarkRxFrame(PC_CMD_IR_ORE_ACK, sizeof(cmd), MRLINK_OK);
  TouchOnline(PC_CMD_IR_ORE_ACK);
}

void OnMrlinkError(const MrLink_ErrorInfo_t &err) {
  g_pc_comm_debug.mrlink_error_count++;
  g_pc_comm_debug.mrlink_last_error_code = static_cast<uint8_t>(err.code);
  g_pc_comm_debug.mrlink_last_error_cmd = err.cmd;
  g_pc_comm_debug.mrlink_last_error_payload_len = err.payload_len;
  g_pc_comm_debug.mrlink_last_error_expected_size = err.expected_size;
  g_pc_comm_debug.mrlink_last_error_available = err.available;
  g_pc_comm_debug.mrlink_last_error_dropped_bytes = err.dropped_bytes;
  g_pc_comm_debug.mrlink_last_error_crc_received = err.crc_received;
  g_pc_comm_debug.mrlink_last_error_crc_calculated = err.crc_calculated;
  MarkLastRxFrame(err.cmd, err.payload_len, MRLINK_ERR, err.crc_received,
                  err.crc_calculated);
}

bool RegisterHandlers() {
  return s_bus.Subscribe(wire::kCmdHeartbeat, OnHeartbeat) == MRLINK_OK &&
         s_bus.SubscribeLatest<PC_ChassisCMD_t>(OnChassis) == MRLINK_OK &&
         s_bus.SubscribeLatest<wire::PoleCmd>(OnPole) == MRLINK_OK &&
         s_bus.SubscribeLatest<wire::ArmSimpleCmd>(OnArmSimple) ==
             MRLINK_OK &&
         s_bus.SubscribeLatest<wire::RodNewCmd>(OnRodNew) == MRLINK_OK &&
         s_bus.SubscribeLatest<wire::OreStoreCmd>(OnOreStore) ==
             MRLINK_OK &&
         s_bus.Subscribe<wire::AutoActionCmd>(OnAutoAction) == MRLINK_OK &&
         s_bus.SubscribeLatest<wire::CameraYawCmd>(OnCameraYaw) == MRLINK_OK &&
         s_bus.SubscribeLatest<PC_AbstractPositionCMD_t>(OnAbstractPosition) ==
             MRLINK_OK &&
         s_bus.SubscribeLatest<wire::StepCmd>(OnStep) == MRLINK_OK &&
         s_bus.SubscribeLatest<PC_ImuCMD_t>(OnImu) == MRLINK_OK &&
         s_bus.Subscribe<PC_IrOreAckCMD_t>(OnIrOreAck) == MRLINK_OK;
}

void RxReadyCallback(void *ctx) {
  (void)ctx;
  if (s_thread_id != nullptr) {
    (void)osThreadFlagsSet(s_thread_id, kRxCompleteFlag);
  }
}

void TxDoneBridge(void *ctx) {
  (void)ctx;
  if (s_tx_done_callback != nullptr) {
    s_tx_done_callback();
  }
}

void TxErrorBridge(void *ctx) {
  (void)ctx;
  if (s_tx_error_callback != nullptr) {
    s_tx_error_callback();
  }
}

bool WaitRecvComplete(uint32_t timeout_ms) {
  const uint32_t flags =
      osThreadFlagsWait(kRxCompleteFlag, osFlagsWaitAny, timeout_ms);
  return (flags & kRxCompleteFlag) != 0u;
}

uint32_t MrlinkErrorTotal(const MrLink_Stats_t *stats) {
  if (stats == nullptr) {
    return 0u;
  }
  return stats->frame_rx_crc_err + stats->frame_rx_oversize +
         stats->frame_rx_header_skip + stats->frame_rx_truncated +
         stats->frame_rx_unknown_cmd + stats->frame_rx_size_mismatch +
         stats->feed_bytes_dropped;
}

void DispatchPendingRxFrames() {
  while (true) {
    const MrLink_Stats_t *before = s_bus.GetStats();
    if (before == nullptr) {
      return;
    }

    const uint32_t prev_ok = before->frame_rx_ok;
    const uint32_t prev_error_total = MrlinkErrorTotal(before);
    const int8_t dispatch_ret = s_bus.Dispatch();

    const MrLink_Stats_t *after = s_bus.GetStats();
    if (after == nullptr) {
      return;
    }

    const bool frame_dispatched = after->frame_rx_ok != prev_ok;
    const bool error_reported =
        MrlinkErrorTotal(after) != prev_error_total;
    if (error_reported) {
      s_state.error_count++;
      g_pc_comm_debug.last_rx_result = MRLINK_ERR;
    }

    if (dispatch_ret != MRLINK_OK || !frame_dispatched) {
      break;
    }
  }
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

template <typename T>
uint16_t BuildLatestOrFallback(const T &fallback,
                               uint8_t *tx_buf,
                               uint16_t buf_size) {
  T latest{};
  if (s_bus.CopyLatest(&latest)) {
    return s_bus.Publish(latest, tx_buf, buf_size);
  }
  return s_bus.Publish(fallback, tx_buf, buf_size);
}

wire::ArmSimpleFeedback MakeArmSimpleFeedbackWire(
    const PC_ArmSimpleFeedback_t &feedback) {
  wire::ArmSimpleFeedback wire_feedback = {};
  wire_feedback.mode = feedback.mode;
  wire_feedback.point_mode = feedback.point_mode;
  wire_feedback.suction = feedback.suction;
  wire_feedback.joint1_angle_rad = feedback.joint1_angle_rad;
  wire_feedback.joint1_velocity_rad_s = feedback.joint1_velocity_rad_s;
  wire_feedback.joint2_angle_rad = feedback.joint2_angle_rad;
  return wire_feedback;
}

wire::RodNewFeedback MakeRodNewFeedbackWire(
    const PC_RodNewFeedback_t &feedback) {
  wire::RodNewFeedback wire_feedback = {};
  wire_feedback.mode = feedback.mode;
  wire_feedback.pose = feedback.pose;
  wire_feedback.grip = feedback.grip;
  wire_feedback.at_target = feedback.at_target;
  wire_feedback.target_angle_rad = feedback.target_angle_rad;
  wire_feedback.tracked_angle_rad = feedback.tracked_angle_rad;
  wire_feedback.tracked_velocity_rad_s = feedback.tracked_velocity_rad_s;
  wire_feedback.feedback_angle_rad = feedback.feedback_angle_rad;
  return wire_feedback;
}

wire::OreStoreFeedback MakeOreStoreFeedbackWire(
    const PC_OreStoreFeedback_t &feedback) {
  wire::OreStoreFeedback wire_feedback = {};
  wire_feedback.mode = feedback.mode;
  wire_feedback.all_homed = feedback.all_homed;
  wire_feedback.online_mask = feedback.online_mask;
  wire_feedback.homed_mask = feedback.homed_mask;
  wire_feedback.platform_position_rad = feedback.platform_position_rad;
  wire_feedback.transform_low_has_ore = feedback.transform_low_has_ore;
  wire_feedback.transform_high_has_ore = feedback.transform_high_has_ore;
  wire_feedback.arm_has_ore = feedback.arm_has_ore;
  wire_feedback.reserved = 0u;
  return wire_feedback;
}

wire::StepFeedback MakeStepFeedbackWire(const PC_StepFeedback_t &feedback) {
  wire::StepFeedback wire_feedback = {};
  wire_feedback.state = static_cast<uint8_t>(feedback.state);
  wire_feedback.result = static_cast<uint8_t>(feedback.result);
  wire_feedback.fault = static_cast<uint8_t>(feedback.fault);
  wire_feedback.template_id = static_cast<uint8_t>(feedback.template_id);
  wire_feedback.step_index = feedback.step_index;
  wire_feedback.reserved = 0u;
  wire_feedback.progress = feedback.progress;
  return wire_feedback;
}

wire::StatusFeedback MakeStatusFeedbackWire(
    const PC_StatusFeedback_t &feedback) {
  wire::StatusFeedback wire_feedback = {};
  wire_feedback.online = feedback.online;
  wire_feedback.recv_count = feedback.recv_count;
  wire_feedback.cpu_temp = feedback.cpu_temp;
  wire_feedback.command_source = static_cast<uint8_t>(feedback.command_source);
  return wire_feedback;
}

}  // namespace

volatile PC_CommDebug_t g_pc_comm_debug;

mr::link::Instance &MrlinkPc_Bus(void) {
  return s_bus;
}

extern "C" bool MrlinkPc_SelectUart(BSP_UART_t uart) {
  if (s_comm_initialized || uart >= BSP_UART_NUM) {
    return false;
  }
  s_channel_uart = uart;
  return true;
}

extern "C" bool MrlinkPc_CommInit(void) {
  s_thread_id = osThreadGetId();
  if (s_thread_id == nullptr) {
    g_pc_comm_debug.last_init_error = -21;
    return false;
  }

  std::memset(&s_state, 0, sizeof(s_state));
  s_pole_cmd_received = false;
  s_ir_ore_ack_pending = false;
  s_state.control_mode = PC_MODE_RC;
  s_state.feedback.status.command_source = PC_COMMAND_SOURCE_RC;

  MrLink_Config_t cfg = {};
  cfg.max_payload_size = MRLINK_PC_MAX_PAYLOAD_SIZE;
  cfg.use_crc16 = true;
  const int8_t begin_ret =
      s_bus.BeginUart(s_channel_uart, &cfg, RxReadyCallback, nullptr);
  if (begin_ret != MRLINK_OK) {
    g_pc_comm_debug.last_init_error = begin_ret;
    return false;
  }

  if (s_bus.OnError(OnMrlinkError) != MRLINK_OK) {
    g_pc_comm_debug.last_init_error = -25;
    return false;
  }

  if (!RegisterHandlers()) {
    g_pc_comm_debug.last_init_error = -24;
    return false;
  }

  g_pc_comm_debug.last_init_error = 0;
  s_comm_initialized = true;
  return true;
}

extern "C" void MrlinkPc_CommProcess(uint32_t now_ms) {
  if (s_state.last_heartbeat_tick > 0u &&
      (now_ms - s_state.last_heartbeat_tick) > kHeartbeatTimeoutMs) {
    s_state.heartbeat_valid = false;
    s_state.control_mode = PC_MODE_RC;
    s_state.online = false;
    s_pole_cmd_received = false;
  }

  (void)WaitRecvComplete(0u);
  if (!MrLink_Channel_IsRxActive(s_bus.Channel()) &&
      s_bus.StartChannelRx() != MRLINK_CHANNEL_OK) {
    g_pc_comm_debug.rx_restart_fail_count++;
  }

  while (true) {
    const uint16_t rx_len =
        s_bus.PollChannelRx(s_rx_parse_buf, sizeof(s_rx_parse_buf), false);
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

    DispatchPendingRxFrames();
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
  g_pc_comm_debug.rx_irq_count = MrLink_Channel_GetRxIrqCount(s_bus.Channel());
  g_pc_comm_debug.rx_irq_byte_count =
      MrLink_Channel_GetRxIrqByteCount(s_bus.Channel());
  g_pc_comm_debug.rx_queue_overflow_count =
      MrLink_Channel_GetRxOverflowCount(s_bus.Channel());
  g_pc_comm_debug.rx_dma_start_fail_count =
      MrLink_Channel_GetRxStartFailCount(s_bus.Channel());
  CopyPlainToVolatile(&g_pc_comm_debug.rx_chassis, &s_state.cmd.chassis);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_pole, &s_state.cmd.pole);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_arm_simple, &s_state.cmd.arm_simple);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_rod_new, &s_state.cmd.rod_new);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_ore_store, &s_state.cmd.ore_store);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_camera_yaw,
                      &s_state.cmd.camera_yaw);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_abstract_position,
                      &s_state.cmd.abstract_position);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_auto_action,
                      &s_state.cmd.auto_action);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_step, &s_state.cmd.step);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_imu, &s_state.cmd.imu);
  CopyPlainToVolatile(&g_pc_comm_debug.rx_ir_ore_ack,
                      &s_state.cmd.ir_ore_ack);
  const MrLink_Stats_t *stats = s_bus.GetStats();
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

extern "C" bool MrlinkPc_HasPoleCMD(void) {
  return s_pole_cmd_received;
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

extern "C" const PC_CameraYawCMD_t *MrlinkPc_GetCameraYawCMD(void) {
  return &s_state.cmd.camera_yaw;
}

extern "C" const PC_AbstractPositionCMD_t *MrlinkPc_GetAbstractPositionCMD(void) {
  return (s_state.cmd.abstract_position.enable_mask != 0u)
             ? &s_state.cmd.abstract_position
             : nullptr;
}

extern "C" const PC_AutoActionCMD_t *MrlinkPc_GetAutoActionCMD(void) {
  return (s_state.cmd.auto_action.action != PC_AUTO_ACTION_NONE)
             ? &s_state.cmd.auto_action
             : nullptr;
}

extern "C" const PC_StepCMD_t *MrlinkPc_GetStepCMD(void) {
  return &s_state.cmd.step;
}

extern "C" const PC_ImuCMD_t *MrlinkPc_GetImuCMD(void) {
  return &s_state.cmd.imu;
}

extern "C" const PC_IrOreAckCMD_t *MrlinkPc_GetIrOreAckCMD(void) {
  return s_ir_ore_ack_pending ? &s_state.cmd.ir_ore_ack : nullptr;
}

extern "C" bool MrlinkPc_PublishFeedback(uint8_t topic,
                                           const void *feedback) {
  if (feedback == nullptr) {
    return false;
  }

  switch (topic) {
    case PC_FEEDBACK_CHASSIS: {
      const auto *typed = static_cast<const PC_ChassisFeedback_t *>(feedback);
      s_state.feedback.chassis = *typed;
      return s_bus.StoreLatest(*typed);
    }
    case PC_FEEDBACK_POLE: {
      const auto *typed = static_cast<const PC_PoleFeedback_t *>(feedback);
      s_state.feedback.pole = *typed;
      return s_bus.StoreLatest(*typed);
    }
    case PC_FEEDBACK_ARM_SIMPLE: {
      const auto *typed = static_cast<const PC_ArmSimpleFeedback_t *>(feedback);
      s_state.feedback.arm_simple = *typed;
      const wire::ArmSimpleFeedback wire_feedback =
          MakeArmSimpleFeedbackWire(*typed);
      return s_bus.StoreLatest(wire_feedback);
    }
    case PC_FEEDBACK_ROD_NEW: {
      const auto *typed = static_cast<const PC_RodNewFeedback_t *>(feedback);
      s_state.feedback.rod_new = *typed;
      const wire::RodNewFeedback wire_feedback = MakeRodNewFeedbackWire(*typed);
      return s_bus.StoreLatest(wire_feedback);
    }
    case PC_FEEDBACK_ORE_STORE: {
      const auto *typed = static_cast<const PC_OreStoreFeedback_t *>(feedback);
      s_state.feedback.ore_store = *typed;
      const wire::OreStoreFeedback wire_feedback =
          MakeOreStoreFeedbackWire(*typed);
      return s_bus.StoreLatest(wire_feedback);
    }
    case PC_FEEDBACK_AUTO_ACTION: {
      const auto *typed =
          static_cast<const PC_AutoActionFeedback_t *>(feedback);
      s_auto_action_feedback = *typed;
      return s_bus.StoreLatest(*typed);
    }
    case PC_FEEDBACK_CAMERA_YAW: {
      const auto *typed =
          static_cast<const PC_CameraYawFeedback_t *>(feedback);
      s_state.feedback.camera_yaw = *typed;
      return s_bus.StoreLatest(*typed);
    }
    case PC_FEEDBACK_IR_ORE: {
      const auto *typed = static_cast<const PC_IrOreFeedback_t *>(feedback);
      s_ir_ore_feedback = *typed;
      return s_bus.StoreLatest(*typed);
    }
    case PC_FEEDBACK_IR_ORE_BRIDGE: {
      const auto *typed =
          static_cast<const PC_IrOreBridgeFeedback_t *>(feedback);
      s_ir_ore_bridge_feedback = *typed;
      s_state.feedback.ir_ore_bridge = *typed;
      return s_bus.StoreLatest(*typed);
    }
    case PC_FEEDBACK_STEP: {
      const auto *typed = static_cast<const PC_StepFeedback_t *>(feedback);
      s_state.feedback.step = *typed;
      const wire::StepFeedback wire_feedback = MakeStepFeedbackWire(*typed);
      return s_bus.StoreLatest(wire_feedback);
    }
    case PC_FEEDBACK_STATUS: {
      const auto *typed = static_cast<const PC_StatusFeedback_t *>(feedback);
      s_state.feedback.status = *typed;
      const wire::StatusFeedback wire_feedback = MakeStatusFeedbackWire(*typed);
      return s_bus.StoreLatest(wire_feedback);
    }
    default:
      return false;
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

extern "C" void MrlinkPc_ClearAutoActionCommand(void) {
  s_state.cmd.auto_action.action = PC_AUTO_ACTION_NONE;
}

extern "C" void MrlinkPc_ClearIrOreAckCommand(void) {
  s_ir_ore_ack_pending = false;
}

extern "C" uint16_t MrlinkPc_BuildFeedbackFrame(uint8_t cmd, uint8_t *tx_buf,
                                                 uint16_t buf_size) {
  switch (cmd) {
    case PC_FEEDBACK_HEARTBEAT:
    {
      return s_bus.PublishEmpty(cmd, tx_buf, buf_size);
    }
    case PC_FEEDBACK_CHASSIS:
    {
      return BuildLatestOrFallback(s_state.feedback.chassis, tx_buf, buf_size);
    }
    case PC_FEEDBACK_POLE:
    {
      return BuildLatestOrFallback(s_state.feedback.pole, tx_buf, buf_size);
    }
    case PC_FEEDBACK_ARM_SIMPLE: {
      const wire::ArmSimpleFeedback fallback =
          MakeArmSimpleFeedbackWire(s_state.feedback.arm_simple);
      return BuildLatestOrFallback(fallback, tx_buf, buf_size);
    }
    case PC_FEEDBACK_ROD_NEW: {
      const wire::RodNewFeedback fallback =
          MakeRodNewFeedbackWire(s_state.feedback.rod_new);
      return BuildLatestOrFallback(fallback, tx_buf, buf_size);
    }
    case PC_FEEDBACK_ORE_STORE: {
      const wire::OreStoreFeedback fallback =
          MakeOreStoreFeedbackWire(s_state.feedback.ore_store);
      return BuildLatestOrFallback(fallback, tx_buf, buf_size);
    }
    case PC_FEEDBACK_AUTO_ACTION: {
      return BuildLatestOrFallback(s_auto_action_feedback, tx_buf, buf_size);
    }
    case PC_FEEDBACK_CAMERA_YAW: {
      return BuildLatestOrFallback(s_state.feedback.camera_yaw, tx_buf,
                                   buf_size);
    }
    case PC_FEEDBACK_IR_ORE: {
      return BuildLatestOrFallback(s_ir_ore_feedback, tx_buf, buf_size);
    }
    case PC_FEEDBACK_IR_ORE_BRIDGE: {
      return BuildLatestOrFallback(s_ir_ore_bridge_feedback, tx_buf, buf_size);
    }
    case PC_FEEDBACK_STEP: {
      const wire::StepFeedback fallback =
          MakeStepFeedbackWire(s_state.feedback.step);
      return BuildLatestOrFallback(fallback, tx_buf, buf_size);
    }
    case PC_FEEDBACK_STATUS: {
      const wire::StatusFeedback fallback =
          MakeStatusFeedbackWire(s_state.feedback.status);
      return BuildLatestOrFallback(fallback, tx_buf, buf_size);
    }
    default:
      return 0u;
  }
}


extern "C" int8_t MrlinkPc_SendFrame(uint8_t *data, uint16_t len) {
  return MrLink_Channel_Send(s_bus.Channel(), data, len);
}

extern "C" int8_t MrlinkPc_RegisterTxCallbacks(
    MrlinkPc_TxCallback_t tx_done, MrlinkPc_TxCallback_t tx_error) {
  s_tx_done_callback = tx_done;
  s_tx_error_callback = tx_error;
  return MrLink_Channel_RegisterTxCallbacks(s_bus.Channel(), TxDoneBridge,
                                            TxErrorBridge, nullptr);
}
