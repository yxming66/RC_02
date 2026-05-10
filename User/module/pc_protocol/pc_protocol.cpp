#include "module/pc_protocol/pc_protocol.h"

#include <string.h>

#include "cmsis_os2.h"
#include "bsp/time.h"
#include "bsp/uart.h"
#include "component/crc16.h"

/* USER INCLUDE BEGIN */
/* USER INCLUDE END */

/* Private variables -------------------------------------------------------- */
static PC_Comm_t s_pc_comm_inst;
PC_Protocol_t* g_pc_protocol_ptr = NULL;
static uint8_t s_rx_dma_buf[4][256];
static uint8_t s_rx_parse_buf[256];
static uint8_t s_rx_stream_buf[512];
static uint16_t s_rx_stream_len = 0;
static volatile bool s_rx_active = false;
static volatile uint8_t s_rx_dma_write_idx = 0;
static volatile uint8_t s_rx_dma_read_idx = 0;
static volatile uint16_t s_rx_dma_len[4] = {0};
static osThreadId_t s_thread_id = NULL;

/* Private define ----------------------------------------------------------- */
#define PC_PROTOCOL_MIN_FRAME_SIZE (6u)  /* Header(2) + Length(1) + CMD(1) + CRC(2) */
#define PC_PROTOCOL_HEADER_SIZE (2u)
#define PC_PROTOCOL_CMD_SIZE (1u)
#define PC_PROTOCOL_LENGTH_SIZE (1u)
#define PC_PROTOCOL_CRC_SIZE (2u)

#define PC_PROTOCOL_HEARTBEAT_TIMEOUT_MS (500u)
#define PC_CMD_HEARTBEAT_PAYLOAD_LEN (0u)
#define PC_CMD_CHASSIS_PAYLOAD_LEN (12u)
#define PC_CMD_POLE_PAYLOAD_LEN (9u)
#define PC_CMD_STEP_PAYLOAD_LEN_BASIC (2u)
#define PC_CMD_STEP_PAYLOAD_LEN_WITH_YAW (6u)
#define PC_CMD_STEP_PAYLOAD_LEN_WITH_TOLERANCE (10u)
#define PC_CMD_IMU_PAYLOAD_LEN (28u)

/* USER DEFINE BEGIN */
/* USER DEFINE END */

/* Private macro ------------------------------------------------------------ */
/* Private typedef ---------------------------------------------------------- */
typedef struct {
    uint8_t header[2];
    uint8_t length;
    uint8_t cmd;
} __attribute__((packed)) PC_FrameHeader_t;

typedef struct {
    uint8_t header[2];
    uint8_t length;
    uint8_t cmd;
    uint8_t payload[PC_PROTOCOL_MAX_PAYLOAD_SIZE];
    uint16_t crc;
} __attribute__((packed)) PC_Frame_t;

/* Private variables -------------------------------------------------------- */
static const uint8_t frame_header_[2] = {
    PC_PROTOCOL_FRAME_HEADER_0,
    PC_PROTOCOL_FRAME_HEADER_1
};

/* Private function prototypes -------------------------------------------- */
static uint16_t PC_Protocol_BuildHeartbeat(uint8_t *tx_buf);
static uint16_t PC_Protocol_BuildChassisFeedback(const PC_ChassisFeedback_t *fb, uint8_t *tx_buf);
static uint16_t PC_Protocol_BuildPoleFeedback(const PC_PoleFeedback_t *fb, uint8_t *tx_buf);
static uint16_t PC_Protocol_BuildStepFeedback(const PC_StepFeedback_t *fb, uint8_t *tx_buf);
static uint16_t PC_Protocol_BuildStatusFeedback(const PC_StatusFeedback_t *fb, uint8_t *tx_buf);

static bool PC_Protocol_ParseChassisCMD(const uint8_t *payload, uint8_t len, PC_ChassisCMD_t *cmd);
static bool PC_Protocol_ParsePoleCMD(const uint8_t *payload, uint8_t len, PC_PoleCMD_t *cmd);
static bool PC_Protocol_ParseHeartbeat(const uint8_t *payload, uint8_t len);
static bool PC_Protocol_ParseStepCMD(const uint8_t *payload, uint8_t len, PC_StepCMD_t *cmd);

/* Private function --------------------------------------------------------- */
static uint16_t PC_Protocol_BuildHeartbeat(uint8_t *tx_buf) {
    PC_FrameHeader_t *header = (PC_FrameHeader_t *)tx_buf;
    header->header[0] = frame_header_[0];
    header->header[1] = frame_header_[1];
    header->length = 0;
    header->cmd = PC_FEEDBACK_HEARTBEAT;

    uint16_t crc = CRC16_Calc(tx_buf, PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE, CRC16_INIT);
    tx_buf[4] = (uint8_t)(crc & 0xFFu);
    tx_buf[5] = (uint8_t)((crc >> 8) & 0xFFu);

    return PC_PROTOCOL_MIN_FRAME_SIZE;
}

static uint16_t PC_Protocol_BuildChassisFeedback(const PC_ChassisFeedback_t *fb, uint8_t *tx_buf) {
    PC_FrameHeader_t *header = (PC_FrameHeader_t *)tx_buf;
    header->header[0] = frame_header_[0];
    header->header[1] = frame_header_[1];
    header->length = sizeof(PC_ChassisFeedback_t);
    header->cmd = PC_FEEDBACK_CHASSIS;

    memcpy(&tx_buf[4], fb, sizeof(PC_ChassisFeedback_t));

    uint16_t crc = CRC16_Calc(tx_buf, PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE + header->length, CRC16_INIT);
    tx_buf[4 + header->length] = (uint8_t)(crc & 0xFFu);
    tx_buf[5 + header->length] = (uint8_t)((crc >> 8) & 0xFFu);

    return PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE + header->length + PC_PROTOCOL_CRC_SIZE;
}

static uint16_t PC_Protocol_BuildPoleFeedback(const PC_PoleFeedback_t *fb, uint8_t *tx_buf) {
    PC_FrameHeader_t *header = (PC_FrameHeader_t *)tx_buf;
    header->header[0] = frame_header_[0];
    header->header[1] = frame_header_[1];
    header->length = sizeof(PC_PoleFeedback_t);
    header->cmd = PC_FEEDBACK_POLE;

    memcpy(&tx_buf[4], fb, sizeof(PC_PoleFeedback_t));

    uint16_t crc = CRC16_Calc(tx_buf, PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE + header->length, CRC16_INIT);
    tx_buf[4 + header->length] = (uint8_t)(crc & 0xFFu);
    tx_buf[5 + header->length] = (uint8_t)((crc >> 8) & 0xFFu);

    return PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE + header->length + PC_PROTOCOL_CRC_SIZE;
}

static uint16_t PC_Protocol_BuildStepFeedback(const PC_StepFeedback_t *fb, uint8_t *tx_buf) {
    PC_FrameHeader_t *header = (PC_FrameHeader_t *)tx_buf;
    header->header[0] = frame_header_[0];
    header->header[1] = frame_header_[1];
    header->length = 10;  /* state + result + fault + template_id + step_index + reserved + progress */
    header->cmd = PC_FEEDBACK_STEP;

    tx_buf[4] = (uint8_t)fb->state;
    tx_buf[5] = (uint8_t)fb->result;
    tx_buf[6] = (uint8_t)fb->fault;
    tx_buf[7] = (uint8_t)fb->template_id;
    tx_buf[8] = fb->step_index;
    tx_buf[9] = 0;  /* reserved */
    memcpy(&tx_buf[10], &fb->progress, sizeof(float));

    uint16_t crc = CRC16_Calc(tx_buf, PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE + header->length, CRC16_INIT);
    tx_buf[4 + header->length] = (uint8_t)(crc & 0xFFu);
    tx_buf[5 + header->length] = (uint8_t)((crc >> 8) & 0xFFu);

    return PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE + header->length + PC_PROTOCOL_CRC_SIZE;
}

static uint16_t PC_Protocol_BuildStatusFeedback(const PC_StatusFeedback_t *fb, uint8_t *tx_buf) {
    PC_FrameHeader_t *header = (PC_FrameHeader_t *)tx_buf;
    header->header[0] = frame_header_[0];
    header->header[1] = frame_header_[1];
    header->length = 10;  /* online + recv_count + cpu_temp + command_source */
    header->cmd = PC_FEEDBACK_STATUS;

    tx_buf[4] = fb->online;
    memcpy(&tx_buf[5], &fb->recv_count, sizeof(uint32_t));
    memcpy(&tx_buf[9], &fb->cpu_temp, sizeof(float));
    tx_buf[13] = (uint8_t)fb->command_source;

    uint16_t crc = CRC16_Calc(tx_buf, PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE + header->length, CRC16_INIT);
    tx_buf[4 + header->length] = (uint8_t)(crc & 0xFFu);
    tx_buf[5 + header->length] = (uint8_t)((crc >> 8) & 0xFFu);

    return PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE + header->length + PC_PROTOCOL_CRC_SIZE;
}

static bool PC_Protocol_ParseChassisCMD(const uint8_t *payload, uint8_t len, PC_ChassisCMD_t *cmd) {
    if (len != PC_CMD_CHASSIS_PAYLOAD_LEN) {
        return false;
    }
    memcpy(cmd, payload, sizeof(PC_ChassisCMD_t));
    return true;
}

static bool PC_Protocol_ParsePoleCMD(const uint8_t *payload, uint8_t len, PC_PoleCMD_t *cmd) {
    if (len != PC_CMD_POLE_PAYLOAD_LEN) {  /* mode + lift[0] + lift[1] */
        return false;
    }
    cmd->mode = payload[0];
    memcpy(&cmd->lift[0], &payload[1], sizeof(float));
    memcpy(&cmd->lift[1], &payload[5], sizeof(float));
    return true;
}

static bool PC_Protocol_ParseHeartbeat(const uint8_t *payload, uint8_t len) {
    (void)payload;
    return len == PC_CMD_HEARTBEAT_PAYLOAD_LEN;
}

static bool PC_Protocol_ParseStepCMD(const uint8_t *payload, uint8_t len, PC_StepCMD_t *cmd) {
    if (len != PC_CMD_STEP_PAYLOAD_LEN_WITH_TOLERANCE) {
        return false;
    }
    cmd->template_id = (PC_StepTemplate_t)payload[0];
    cmd->travel_dir = (PC_StepDir_t)payload[1];
    cmd->target_yaw_rad = 0.0f;
    cmd->yaw_tolerance_rad = 0.1f;

    if (len >= 6) {
        memcpy(&cmd->target_yaw_rad, &payload[2], sizeof(float));
    }
    if (len >= 10) {
        memcpy(&cmd->yaw_tolerance_rad, &payload[6], sizeof(float));
    }
    return true;
}

static bool PC_Protocol_ParseImuCMD(const uint8_t *payload, uint8_t len, PC_ImuCMD_t *cmd) {
    /* IMU: qw(4) + qx(4) + qy(4) + qz(4) + roll(4) + pitch(4) + yaw(4) = 28 bytes */
    if (len != PC_CMD_IMU_PAYLOAD_LEN) {
        return false;
    }
    memcpy(&cmd->qw, &payload[0], sizeof(float));
    memcpy(&cmd->qx, &payload[4], sizeof(float));
    memcpy(&cmd->qy, &payload[8], sizeof(float));
    memcpy(&cmd->qz, &payload[12], sizeof(float));
    memcpy(&cmd->roll, &payload[16], sizeof(float));
    memcpy(&cmd->pitch, &payload[20], sizeof(float));
    memcpy(&cmd->yaw, &payload[24], sizeof(float));
    return true;
}

/* Exported functions ------------------------------------------------------- */
void PC_Protocol_Init(PC_Protocol_t *proto) {
    if (proto == NULL) return;

    memset(proto, 0, sizeof(PC_Protocol_t));
    proto->control_mode = PC_MODE_RC;
    proto->heartbeat_valid = false;
    proto->last_heartbeat_tick = 0;
}

void PC_Protocol_Update(PC_Protocol_t *proto) {
    if (proto == NULL) return;

    uint32_t now_ms = BSP_TIME_Get_ms();

    /* 检查心跳超时 */
    if (proto->last_heartbeat_tick > 0) {
        if ((now_ms - proto->last_heartbeat_tick) > PC_PROTOCOL_HEARTBEAT_TIMEOUT_MS) {
            proto->heartbeat_valid = false;
            proto->control_mode = PC_MODE_RC;
        }
    }

    /* 如果心跳有效且超时未到，切换到PC控制模式 */
    if (proto->heartbeat_valid) {
        proto->control_mode = PC_MODE_PC;
    }
}

int8_t PC_Protocol_ParseFrame(PC_Protocol_t *proto, const uint8_t *data, uint16_t size) {
    if (proto == NULL || data == NULL || size < PC_PROTOCOL_MIN_FRAME_SIZE) {
        return -1;
    }

    /* 检查帧头 */
    if (data[0] != PC_PROTOCOL_FRAME_HEADER_0 || data[1] != PC_PROTOCOL_FRAME_HEADER_1) {
        return -2;
    }

    uint8_t length = data[2];
    uint8_t cmd = data[3];

    if (length > PC_PROTOCOL_MAX_PAYLOAD_SIZE) {
        return -6;
    }

    /* 检查帧长度 */
    uint16_t expected_size = PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE + length + PC_PROTOCOL_CRC_SIZE;
    if (size < expected_size) {
        return -3;
    }

    if (length > PC_PROTOCOL_MAX_PAYLOAD_SIZE) {
        return -6;
    }

    /* CRC校验 */
    uint16_t crc_received = (uint16_t)data[expected_size - 2] | ((uint16_t)data[expected_size - 1] << 8);
    uint16_t crc_calculated = CRC16_Calc(data, expected_size - PC_PROTOCOL_CRC_SIZE, CRC16_INIT);
    if (crc_received != crc_calculated) {
        return -4;
    }

    const uint8_t *payload = &data[PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE];
    bool parsed = false;

    /* 处理命令 */
    switch (cmd) {
        case PC_CMD_HEARTBEAT:
            parsed = PC_Protocol_ParseHeartbeat(payload, length);
            break;

        case PC_CMD_CHASSIS:
            parsed = PC_Protocol_ParseChassisCMD(payload, length, &proto->cmd.chassis);
            break;

        case PC_CMD_POLE:
            parsed = PC_Protocol_ParsePoleCMD(payload, length, &proto->cmd.pole);
            break;

        case PC_CMD_STEP:
            parsed = PC_Protocol_ParseStepCMD(payload, length, &proto->cmd.step);
            break;

        case PC_CMD_IMU:
            parsed = PC_Protocol_ParseImuCMD(payload, length, &proto->cmd.imu);
            break;

        default:
            return -5;  /* 未知命令 */
    }

    if (!parsed) {
        return -7;
    }

    proto->last_heartbeat_tick = BSP_TIME_Get_ms();
    proto->heartbeat_valid = true;
    proto->control_mode = PC_MODE_PC;

    return 0;
}

uint16_t PC_Protocol_BuildFrame(PC_Protocol_t *proto, uint8_t cmd, uint8_t *tx_buf, uint16_t buf_size) {
    (void)proto;

    if (tx_buf == NULL || buf_size < PC_PROTOCOL_MIN_FRAME_SIZE) {
        return 0;
    }

    switch (cmd) {
        case PC_FEEDBACK_HEARTBEAT:
            return PC_Protocol_BuildHeartbeat(tx_buf);

        case PC_FEEDBACK_CHASSIS:
            return PC_Protocol_BuildChassisFeedback(&proto->feedback.chassis, tx_buf);

        case PC_FEEDBACK_POLE:
            return PC_Protocol_BuildPoleFeedback(&proto->feedback.pole, tx_buf);

        case PC_FEEDBACK_STEP:
            return PC_Protocol_BuildStepFeedback(&proto->feedback.step, tx_buf);

        case PC_FEEDBACK_STATUS:
            return PC_Protocol_BuildStatusFeedback(&proto->feedback.status, tx_buf);

        default:
            return 0;
    }
}

bool PC_Protocol_IsPCControlMode(const PC_Protocol_t *proto) {
    if (proto == NULL) return false;
    return proto->control_mode == PC_MODE_PC;
}

bool PC_Protocol_IsHeartbeatValid(const PC_Protocol_t *proto) {
    if (proto == NULL) return false;
    return proto->heartbeat_valid;
}

const PC_ChassisCMD_t* PC_Protocol_GetChassisCMD(const PC_Protocol_t *proto) {
    if (proto == NULL) return NULL;
    return &proto->cmd.chassis;
}

const PC_PoleCMD_t* PC_Protocol_GetPoleCMD(const PC_Protocol_t *proto) {
    if (proto == NULL) return NULL;
    return &proto->cmd.pole;
}

const PC_ArmCMD_t* PC_Protocol_GetArmCMD(const PC_Protocol_t *proto) {
    if (proto == NULL) return NULL;
    return &proto->cmd.arm;
}

const PC_StepCMD_t* PC_Protocol_GetStepCMD(const PC_Protocol_t *proto) {
    if (proto == NULL) return NULL;
    return &proto->cmd.step;
}

const PC_ImuCMD_t* PC_Protocol_GetImuCMD(const PC_Protocol_t *proto) {
    if (proto == NULL) return NULL;
    return &proto->cmd.imu;
}

void PC_Protocol_SetChassisFeedback(PC_Protocol_t *proto, const PC_ChassisFeedback_t *fb) {
    if (proto == NULL || fb == NULL) return;
    memcpy(&proto->feedback.chassis, fb, sizeof(PC_ChassisFeedback_t));
}

void PC_Protocol_SetPoleFeedback(PC_Protocol_t *proto, const PC_PoleFeedback_t *fb) {
    if (proto == NULL || fb == NULL) return;
    memcpy(&proto->feedback.pole, fb, sizeof(PC_PoleFeedback_t));
}

void PC_Protocol_SetArmFeedback(PC_Protocol_t *proto, const PC_ArmFeedback_t *fb) {
    if (proto == NULL || fb == NULL) return;
    memcpy(&proto->feedback.arm, fb, sizeof(PC_ArmFeedback_t));
}

void PC_Protocol_SetStepFeedback(PC_Protocol_t *proto, const PC_StepFeedback_t *fb) {
    if (proto == NULL || fb == NULL) return;
    memcpy(&proto->feedback.step, fb, sizeof(PC_StepFeedback_t));
}

void PC_Protocol_SetStatusFeedback(PC_Protocol_t *proto, const PC_StatusFeedback_t *fb) {
    if (proto == NULL || fb == NULL) return;
    memcpy(&proto->feedback.status, fb, sizeof(PC_StatusFeedback_t));
}

/* PC通信管理实现 */
static const uint32_t PC_COMM_RX_CPLT_FLAG = (1u << 0);

static bool PcComm_StartRecv(PC_Comm_t *comm);

static void PcCommRxEventCallback(uint16_t size) {
    s_rx_active = false;
    const uint8_t ready_idx = s_rx_dma_write_idx;

    if (size > sizeof(s_rx_dma_buf[ready_idx])) {
        g_pc_comm_debug.rx_dma_start_fail_count++;
        (void)PcComm_StartRecv(NULL);
        return;
    }

    if (size > 0) {
        s_rx_dma_len[ready_idx] = size;
        g_pc_comm_debug.rx_irq_count++;
        g_pc_comm_debug.rx_irq_byte_count += size;

        uint8_t next_idx = (uint8_t)((ready_idx + 1u) %
                                     (sizeof(s_rx_dma_buf) / sizeof(s_rx_dma_buf[0])));
        if (s_rx_dma_len[next_idx] != 0u) {
            g_pc_comm_debug.rx_queue_overflow_count++;
            s_rx_dma_len[next_idx] = 0u;
            if (s_rx_dma_read_idx == next_idx) {
                s_rx_dma_read_idx = (uint8_t)((next_idx + 1u) %
                                              (sizeof(s_rx_dma_buf) / sizeof(s_rx_dma_buf[0])));
            }
        }
        s_rx_dma_write_idx = next_idx;
    }

    if (!PcComm_StartRecv(NULL)) {
        g_pc_comm_debug.rx_dma_start_fail_count++;
    }

    if (size == 0) {
        return;
    }
    if (s_thread_id != NULL) {
        (void)osThreadFlagsSet(s_thread_id, PC_COMM_RX_CPLT_FLAG);
    }
}

static uint16_t PcComm_FrameSizeAt(const uint8_t *data, uint16_t size) {
    if (data == NULL || size < PC_PROTOCOL_MIN_FRAME_SIZE) {
        return 0;
    }
    if (data[0] != PC_PROTOCOL_FRAME_HEADER_0 ||
        data[1] != PC_PROTOCOL_FRAME_HEADER_1) {
        return 0;
    }

    const uint8_t length = data[2];
    if (length > PC_PROTOCOL_MAX_PAYLOAD_SIZE) {
        return 0;
    }

    uint16_t frame_size = PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE +
                          PC_PROTOCOL_CMD_SIZE + length + PC_PROTOCOL_CRC_SIZE;
    if (size < frame_size) {
        return 0;
    }

    return frame_size;
}

static bool PcComm_StartRecv(PC_Comm_t *comm) {
    (void)comm;
    const uint8_t idx = s_rx_dma_write_idx;
    if (s_rx_dma_len[idx] != 0u) {
        g_pc_comm_debug.rx_queue_overflow_count++;
        s_rx_dma_len[idx] = 0u;
        if (s_rx_dma_read_idx == idx) {
            s_rx_dma_read_idx = (uint8_t)((idx + 1u) %
                                          (sizeof(s_rx_dma_buf) / sizeof(s_rx_dma_buf[0])));
        }
    }

    int8_t ret = BSP_UART_ReceiveToIdle(BSP_UART_PC,
                                        s_rx_dma_buf[idx],
                                        sizeof(s_rx_dma_buf[idx]),
                                        true);
    if (ret == BSP_OK) {
        s_rx_active = true;
        return true;
    }

    if (comm == NULL) {
        s_rx_active = false;
        return false;
    }

    UART_HandleTypeDef *huart = BSP_UART_GetHandle(BSP_UART_PC);
    if (huart != NULL) {
        (void)HAL_UART_AbortReceive(huart);
        ret = BSP_UART_ReceiveToIdle(BSP_UART_PC,
                                     s_rx_dma_buf[idx],
                                     sizeof(s_rx_dma_buf[idx]),
                                     true);
    }
    s_rx_active = (ret == BSP_OK);
    return s_rx_active;
}

static uint16_t PcComm_PopRxChunk(uint8_t *dst, uint16_t dst_size) {
    if (dst == NULL || dst_size == 0u) {
        return 0;
    }

    uint16_t len = 0;
    uint8_t idx = 0;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    idx = s_rx_dma_read_idx;
    len = s_rx_dma_len[idx];
    if (len > 0u) {
        if (len > dst_size) {
            len = dst_size;
        }
        memcpy(dst, s_rx_dma_buf[idx], len);
        s_rx_dma_len[idx] = 0u;
        s_rx_dma_read_idx = (uint8_t)((idx + 1u) %
                                      (sizeof(s_rx_dma_buf) / sizeof(s_rx_dma_buf[0])));
    }
    if (primask == 0u) {
        __enable_irq();
    }
    return len;
}

static bool PcComm_WaitRecvCplt(uint32_t timeout_ms) {
    uint32_t flags = osThreadFlagsWait(PC_COMM_RX_CPLT_FLAG, osFlagsWaitAny, timeout_ms);
    return (flags & PC_COMM_RX_CPLT_FLAG) != 0;
}

bool PC_Comm_Init(PC_Comm_t *comm) {
    if (comm == NULL) {
        g_pc_comm_debug.last_init_error = -20;
        return false;
    }
    s_thread_id = osThreadGetId();
    if (s_thread_id == NULL) {
        g_pc_comm_debug.last_init_error = -21;
        return false;
    }

    PC_Protocol_Init(&comm->protocol);
    comm->last_recv_time = 0;
    comm->recv_count = 0;
    comm->error_count = 0;
    comm->online = false;

    g_pc_protocol_ptr = &comm->protocol;

    if (BSP_UART_RegisterRxEventCallback(BSP_UART_PC,
                                         PcCommRxEventCallback) != BSP_OK) {
        g_pc_comm_debug.last_init_error = -22;
        return false;
    }

    if (!PcComm_StartRecv(comm)) {
        g_pc_comm_debug.last_init_error = -23;
        return false;
    }

    g_pc_comm_debug.last_init_error = 0;
    return true;
}

bool PC_Comm_IsOnline(const PC_Comm_t *comm) {
    if (comm == NULL) return false;
    return comm->online;
}

void PC_Comm_Process(PC_Comm_t *comm, uint32_t now_ms) {
    if (comm == NULL) return;

    PC_Protocol_Update(&comm->protocol);
    comm->online = PC_Protocol_IsHeartbeatValid(&comm->protocol);
    PC_Comm_DebugUpdate(comm);

    (void)PcComm_WaitRecvCplt(0u);
    if (!s_rx_active && !PcComm_StartRecv(comm)) {
        g_pc_comm_debug.rx_restart_fail_count++;
    }

    uint16_t rx_len = PcComm_PopRxChunk(s_rx_parse_buf, sizeof(s_rx_parse_buf));
    if (rx_len == 0) {
        PC_Comm_DebugUpdate(comm);
        return;
    }

    PC_Comm_DebugRecordRxBatch(s_rx_parse_buf, rx_len);

    if ((uint16_t)(sizeof(s_rx_stream_buf) - s_rx_stream_len) < rx_len) {
        s_rx_stream_len = 0;
        comm->error_count++;
        PC_Comm_DebugRecordRxError(comm, s_rx_parse_buf, rx_len, -9);
    }
    memcpy(&s_rx_stream_buf[s_rx_stream_len], s_rx_parse_buf, rx_len);
    s_rx_stream_len = (uint16_t)(s_rx_stream_len + rx_len);

    uint16_t offset = 0;
    while ((uint16_t)(s_rx_stream_len - offset) >= PC_PROTOCOL_MIN_FRAME_SIZE) {
        uint16_t available = (uint16_t)(s_rx_stream_len - offset);
        if (s_rx_stream_buf[offset] == PC_PROTOCOL_FRAME_HEADER_0 &&
            s_rx_stream_buf[offset + 1u] == PC_PROTOCOL_FRAME_HEADER_1 &&
            available >= (PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE)) {
            uint8_t length = s_rx_stream_buf[offset + 2u];
            if (length <= PC_PROTOCOL_MAX_PAYLOAD_SIZE) {
                uint16_t expected_size = PC_PROTOCOL_HEADER_SIZE +
                                         PC_PROTOCOL_LENGTH_SIZE +
                                         PC_PROTOCOL_CMD_SIZE +
                                         length +
                                         PC_PROTOCOL_CRC_SIZE;
                if (available < expected_size) {
                    break;
                }
            }
        }

        uint16_t frame_size = PcComm_FrameSizeAt(&s_rx_stream_buf[offset],
                                                 available);
        if (frame_size == 0) {
            if (s_rx_stream_buf[offset] == PC_PROTOCOL_FRAME_HEADER_0 &&
                s_rx_stream_buf[offset + 1u] == PC_PROTOCOL_FRAME_HEADER_1) {
                comm->error_count++;
                PC_Comm_DebugRecordRxError(comm, &s_rx_stream_buf[offset],
                                           available, -8);
            } else {
                g_pc_comm_debug.rx_header_skip_count++;
            }
            offset++;
            continue;
        }

        int8_t parse_result = PC_Protocol_ParseFrame(&comm->protocol,
                                                     &s_rx_stream_buf[offset],
                                                     frame_size);
        PC_Comm_DebugRecordRxFrame(comm, &s_rx_stream_buf[offset],
                                   frame_size, parse_result);
        if (parse_result >= 0) {
            comm->recv_count++;
            switch (s_rx_stream_buf[offset + 3u]) {
                case PC_CMD_HEARTBEAT:
                    g_pc_comm_debug.rx_heartbeat_count++;
                    break;
                case PC_CMD_CHASSIS:
                    g_pc_comm_debug.rx_chassis_count++;
                    break;
                case PC_CMD_POLE:
                    g_pc_comm_debug.rx_pole_count++;
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
            comm->last_recv_time = now_ms;
            comm->online = true;
            offset = (uint16_t)(offset + frame_size);
        } else {
            comm->error_count++;
            offset++;
        }
    }

    if (offset > 0) {
        s_rx_stream_len = (uint16_t)(s_rx_stream_len - offset);
        if (s_rx_stream_len > 0) {
            memmove(s_rx_stream_buf, &s_rx_stream_buf[offset], s_rx_stream_len);
        }
    }

    PC_Comm_DebugUpdate(comm);
}

PC_Comm_t* PC_Comm_GetInstance(void) {
    return &s_pc_comm_inst;
}

/* PC Step命令映射到auto_ctrl参数 - 内部缓存 */
static PC_AutoStepParams_t s_auto_step_params;

static auto_ctrl_template_e PcStep_MapTemplate(PC_StepTemplate_t pc_template) {
    switch (pc_template) {
        case PC_STEP_TEMPLATE_ASCEND_200_HEAD:     return AUTO_CTRL_TEMPLATE_ASCEND_200_HEAD;
        case PC_STEP_TEMPLATE_ASCEND_400_HEAD:     return AUTO_CTRL_TEMPLATE_ASCEND_400_HEAD;
        case PC_STEP_TEMPLATE_DESCEND_200_HEAD:     return AUTO_CTRL_TEMPLATE_DESCEND_200_HEAD;
        case PC_STEP_TEMPLATE_DESCEND_400_HEAD:     return AUTO_CTRL_TEMPLATE_DESCEND_400_HEAD;
        case PC_STEP_TEMPLATE_ASCEND_200_TAIL:     return AUTO_CTRL_TEMPLATE_ASCEND_200_TAIL;
        case PC_STEP_TEMPLATE_ASCEND_400_TAIL:     return AUTO_CTRL_TEMPLATE_ASCEND_400_TAIL;
        case PC_STEP_TEMPLATE_DESCEND_200_TAIL:     return AUTO_CTRL_TEMPLATE_DESCEND_200_TAIL;
        case PC_STEP_TEMPLATE_DESCEND_400_TAIL:    return AUTO_CTRL_TEMPLATE_DESCEND_400_TAIL;
        default:                                   return AUTO_CTRL_TEMPLATE_NONE;
    }
}

static auto_ctrl_travel_dir_e PcStep_MapTravelDir(PC_StepDir_t pc_dir) {
    return (pc_dir == PC_STEP_DIR_TAIL_FORWARD) ?
           AUTO_CTRL_TRAVEL_DIR_TAIL_FORWARD :
           AUTO_CTRL_TRAVEL_DIR_HEAD_FORWARD;
}

const PC_AutoStepParams_t* PC_Protocol_GetAutoStepParams(const PC_Protocol_t *proto) {
    if (proto == NULL) return NULL;

    const PC_StepCMD_t *step_cmd = &proto->cmd.step;
    if (step_cmd->template_id == PC_STEP_TEMPLATE_NONE) {
        return NULL;
    }

    s_auto_step_params.template_id = PcStep_MapTemplate(step_cmd->template_id);
    if (s_auto_step_params.template_id == AUTO_CTRL_TEMPLATE_NONE) {
        return NULL;
    }

    s_auto_step_params.travel_dir = PcStep_MapTravelDir(step_cmd->travel_dir);
    s_auto_step_params.target_yaw_rad = step_cmd->target_yaw_rad;
    s_auto_step_params.yaw_tolerance_rad = step_cmd->yaw_tolerance_rad;

    return &s_auto_step_params;
}

/* USER FUNCTION BEGIN */
/* USER FUNCTION END */
