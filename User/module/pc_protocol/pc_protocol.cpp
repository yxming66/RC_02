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
static uint8_t s_rx_buf[256];
static uint8_t s_tx_buf[256];
static volatile uint8_t s_rx_cplt_flag = 0;
static osThreadId_t s_thread_id = NULL;

/* Private define ----------------------------------------------------------- */
#define PC_PROTOCOL_MIN_FRAME_SIZE (6u)  /* Header(2) + Length(1) + CMD(1) + CRC(2) */
#define PC_PROTOCOL_HEADER_SIZE (2u)
#define PC_PROTOCOL_CMD_SIZE (1u)
#define PC_PROTOCOL_LENGTH_SIZE (1u)
#define PC_PROTOCOL_CRC_SIZE (2u)

#define PC_PROTOCOL_HEARTBEAT_TIMEOUT_MS (500u)

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
static uint16_t PC_Protocol_BuildStatusFeedback(const PC_StatusFeedback_t *fb, uint8_t *tx_buf);

static bool PC_Protocol_ParseChassisCMD(const uint8_t *payload, uint8_t len, PC_ChassisCMD_t *cmd);
static bool PC_Protocol_ParsePoleCMD(const uint8_t *payload, uint8_t len, PC_PoleCMD_t *cmd);
static bool PC_Protocol_ParseHeartbeat(const uint8_t *payload, uint8_t len);

/* Private function --------------------------------------------------------- */
static uint16_t PC_Protocol_BuildHeartbeat(uint8_t *tx_buf) {
    PC_FrameHeader_t *header = (PC_FrameHeader_t *)tx_buf;
    header->header[0] = frame_header_[0];
    header->header[1] = frame_header_[1];
    header->length = 1;  /* CMD only */
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

static uint16_t PC_Protocol_BuildStatusFeedback(const PC_StatusFeedback_t *fb, uint8_t *tx_buf) {
    PC_FrameHeader_t *header = (PC_FrameHeader_t *)tx_buf;
    header->header[0] = frame_header_[0];
    header->header[1] = frame_header_[1];
    header->length = sizeof(PC_StatusFeedback_t);
    header->cmd = PC_FEEDBACK_STATUS;

    memcpy(&tx_buf[4], fb, sizeof(PC_StatusFeedback_t));

    uint16_t crc = CRC16_Calc(tx_buf, PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE + header->length, CRC16_INIT);
    tx_buf[4 + header->length] = (uint8_t)(crc & 0xFFu);
    tx_buf[5 + header->length] = (uint8_t)((crc >> 8) & 0xFFu);

    return PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE + header->length + PC_PROTOCOL_CRC_SIZE;
}

static bool PC_Protocol_ParseChassisCMD(const uint8_t *payload, uint8_t len, PC_ChassisCMD_t *cmd) {
    if (len < sizeof(PC_ChassisCMD_t)) {
        return false;
    }
    memcpy(cmd, payload, sizeof(PC_ChassisCMD_t));
    return true;
}

static bool PC_Protocol_ParsePoleCMD(const uint8_t *payload, uint8_t len, PC_PoleCMD_t *cmd) {
    if (len < sizeof(PC_PoleCMD_t)) {
        return false;
    }
    memcpy(cmd, payload, sizeof(PC_PoleCMD_t));
    return true;
}

static bool PC_Protocol_ParseHeartbeat(const uint8_t *payload, uint8_t len) {
    (void)payload;
    (void)len;
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

    /* 检查帧长度 */
    uint16_t expected_size = PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE + length + PC_PROTOCOL_CRC_SIZE;
    if (size < expected_size) {
        return -3;
    }

    /* CRC校验 */
    uint16_t crc_received = (uint16_t)data[expected_size - 2] | ((uint16_t)data[expected_size - 1] << 8);
    uint16_t crc_calculated = CRC16_Calc(data, expected_size - PC_PROTOCOL_CRC_SIZE, CRC16_INIT);
    if (crc_received != crc_calculated) {
        return -4;
    }

    const uint8_t *payload = &data[PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_LENGTH_SIZE + PC_PROTOCOL_CMD_SIZE];

    /* 处理命令 */
    switch (cmd) {
        case PC_CMD_HEARTBEAT:
            if (PC_Protocol_ParseHeartbeat(payload, length)) {
                proto->last_heartbeat_tick = BSP_TIME_Get_ms();
                proto->heartbeat_valid = true;
            }
            break;

        case PC_CMD_CHASSIS:
            if (PC_Protocol_ParseChassisCMD(payload, length, &proto->cmd.chassis)) {
                proto->last_heartbeat_tick = BSP_TIME_Get_ms();
                proto->heartbeat_valid = true;
            }
            break;

        case PC_CMD_POLE:
            if (PC_Protocol_ParsePoleCMD(payload, length, &proto->cmd.pole)) {
                proto->last_heartbeat_tick = BSP_TIME_Get_ms();
                proto->heartbeat_valid = true;
            }
            break;

        default:
            return -5;  /* 未知命令 */
    }

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

const PC_RodCMD_t* PC_Protocol_GetRodCMD(const PC_Protocol_t *proto) {
    if (proto == NULL) return NULL;
    return &proto->cmd.rod;
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

void PC_Protocol_SetRodFeedback(PC_Protocol_t *proto, const PC_RodFeedback_t *fb) {
    if (proto == NULL || fb == NULL) return;
    memcpy(&proto->feedback.rod, fb, sizeof(PC_RodFeedback_t));
}

void PC_Protocol_SetStatusFeedback(PC_Protocol_t *proto, const PC_StatusFeedback_t *fb) {
    if (proto == NULL || fb == NULL) return;
    memcpy(&proto->feedback.status, fb, sizeof(PC_StatusFeedback_t));
}

/* PC通信管理实现 */
static const uint32_t PC_COMM_RX_CPLT_FLAG = (1u << 0);
static const uint32_t PC_COMM_TX_PERIOD_MS = 20u;  /* 50Hz */

static void PcCommRxCpltCallback(void) {
    if (s_thread_id != NULL) {
        (void)osThreadFlagsSet(s_thread_id, PC_COMM_RX_CPLT_FLAG);
    }
}

static bool PcComm_StartRecv(PC_Comm_t *comm) {
    (void)comm;
    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    return BSP_UART_Receive(BSP_UART_PC, s_rx_buf, sizeof(s_rx_buf), true) == BSP_OK;
}

static bool PcComm_WaitRecvCplt(uint32_t timeout_ms) {
    uint32_t flags = osThreadFlagsWait(PC_COMM_RX_CPLT_FLAG, osFlagsWaitAny, timeout_ms);
    return (flags & PC_COMM_RX_CPLT_FLAG) != 0;
}

bool PC_Comm_Init(PC_Comm_t *comm) {
    if (comm == NULL) return false;

    PC_Protocol_Init(&comm->protocol);
    comm->last_recv_time = 0;
    comm->recv_count = 0;
    comm->error_count = 0;
    comm->online = false;

    g_pc_protocol_ptr = &comm->protocol;

    /* Register RX callback */
    if (BSP_UART_RegisterCallback(BSP_UART_PC, BSP_UART_RX_CPLT_CB,
                                  PcCommRxCpltCallback) != BSP_OK) {
        return false;
    }

    return true;
}

bool PC_Comm_IsOnline(const PC_Comm_t *comm) {
    if (comm == NULL) return false;
    return comm->online;
}

void PC_Comm_Process(PC_Comm_t *comm, uint32_t now_ms) {
    if (comm == NULL) return;

    PC_Protocol_Update(&comm->protocol);

    /* Wait for RX complete */
    if (!PcComm_WaitRecvCplt(PC_COMM_TX_PERIOD_MS)) {
        return;
    }

    /* Parse received frame */
    int8_t parse_result = PC_Protocol_ParseFrame(&comm->protocol,
                                                  s_rx_buf, sizeof(s_rx_buf));
    if (parse_result >= 0) {
        comm->recv_count++;
        comm->last_recv_time = now_ms;
        comm->online = true;
    } else {
        comm->error_count++;
    }

    /* Restart receive for next frame */
    (void)PcComm_StartRecv(comm);
}

PC_Comm_t* PC_Comm_GetInstance(void) {
    return &s_pc_comm_inst;
}

/* USER FUNCTION BEGIN */
/* USER FUNCTION END */
