/*
 * PC_COMM Task - 上位机通信任务
 */
#include <stddef.h>

#include "task/user_task.h"
#include "module/pc_protocol/pc_protocol.h"
#include "module/autoCtrlAPI/api/auto_ctrl_api.h"
#include "bsp/uart.h"
#include "component/crc16.h"
#include "main.h"

#define PC_COMM_TX_PERIOD_MS (20u)  /* 50Hz */
#define PC_COMM_LOOP_PERIOD_MS (1u)
#define PC_COMM_TX_DMA_TIMEOUT_MS (100u)

volatile PC_CommDebug_t g_pc_comm_debug;
extern volatile PC_CommandSource_t g_pc_command_source;

static uint8_t s_tx_buf[256];   
static volatile bool s_tx_dma_busy = false;
static volatile uint32_t s_tx_dma_start_tick = 0;
static const uint8_t s_feedback_cmds[] = {
    PC_FEEDBACK_HEARTBEAT,
    PC_FEEDBACK_CHASSIS,
    PC_FEEDBACK_POLE,
    PC_FEEDBACK_STEP,
    PC_FEEDBACK_STATUS,
};

static uint16_t PcComm_DebugCopyRaw(volatile uint8_t *dst, uint16_t dst_size,
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

static void PcComm_DebugUpdateUsartConfig(void) {
    UART_HandleTypeDef *huart = BSP_UART_GetHandle(BSP_UART_PC);
    if (huart == NULL) {
        g_pc_comm_debug.usart10_config_ok = 0;
        return;
    }

    g_pc_comm_debug.usart10_baudrate = huart->Init.BaudRate;
    g_pc_comm_debug.usart10_word_length = huart->Init.WordLength;
    g_pc_comm_debug.usart10_stop_bits = huart->Init.StopBits;
    g_pc_comm_debug.usart10_parity = huart->Init.Parity;
    g_pc_comm_debug.usart10_mode = huart->Init.Mode;
    g_pc_comm_debug.usart10_config_ok =
        (huart->Instance == USART10 &&
         huart->Init.BaudRate == 115200u &&
         huart->Init.WordLength == UART_WORDLENGTH_8B &&
         huart->Init.StopBits == UART_STOPBITS_1 &&
         huart->Init.Parity == UART_PARITY_NONE) ? 1u : 0u;
}

void PC_Comm_DebugUpdate(const PC_Comm_t *comm) {
    PcComm_DebugUpdateUsartConfig();
    if (comm == NULL) {
        return;
    }

    g_pc_comm_debug.online = comm->online ? 1u : 0u;
    g_pc_comm_debug.heartbeat_valid =
        PC_Protocol_IsHeartbeatValid(&comm->protocol) ? 1u : 0u;
    g_pc_comm_debug.control_mode = (uint8_t)comm->protocol.control_mode;
    g_pc_comm_debug.command_source =
        (uint8_t)comm->protocol.feedback.status.command_source;
    g_pc_comm_debug.last_heartbeat_tick = comm->protocol.last_heartbeat_tick;
    g_pc_comm_debug.last_recv_time = comm->last_recv_time;
    g_pc_comm_debug.recv_count = comm->recv_count;
    g_pc_comm_debug.error_count = comm->error_count;

    g_pc_comm_debug.rx_chassis = comm->protocol.cmd.chassis;
    g_pc_comm_debug.rx_pole = comm->protocol.cmd.pole;
    g_pc_comm_debug.rx_step = comm->protocol.cmd.step;
    g_pc_comm_debug.rx_imu = comm->protocol.cmd.imu;
}

static void PcComm_DebugRecordInitFail(int8_t reason) {
    g_pc_comm_debug.init_fail_count++;
    g_pc_comm_debug.last_init_error = reason;
}

void PC_Comm_DebugRecordRxBatch(const uint8_t *data, uint16_t len) {
    g_pc_comm_debug.rx_batch_count++;
    g_pc_comm_debug.rx_batch_len = len;
    g_pc_comm_debug.rx_batch_frame_count = 0;
    g_pc_comm_debug.rx_batch_raw_len = PcComm_DebugCopyRaw(
        g_pc_comm_debug.rx_batch_raw, PC_COMM_DEBUG_RX_RAW_SIZE, data, len);
}

void PC_Comm_DebugRecordRxFrame(const PC_Comm_t *comm, const uint8_t *frame,
                                uint16_t frame_size, int8_t parse_result) {
    g_pc_comm_debug.rx_batch_frame_count++;
    g_pc_comm_debug.last_rx_result = parse_result;
    g_pc_comm_debug.last_rx_frame_size = frame_size;
    g_pc_comm_debug.last_rx_cmd = (frame_size >= 4u) ? frame[3] : 0u;
    g_pc_comm_debug.last_rx_len = (frame_size >= 3u) ? frame[2] : 0u;
    g_pc_comm_debug.last_rx_raw_len = PcComm_DebugCopyRaw(
        g_pc_comm_debug.last_rx_raw, PC_PROTOCOL_MAX_FRAME_SIZE, frame,
        frame_size);

    if (frame_size >= PC_PROTOCOL_MAX_FRAME_SIZE) {
        frame_size = PC_PROTOCOL_MAX_FRAME_SIZE;
    }
    if (frame_size >= 6u) {
        g_pc_comm_debug.last_rx_crc_received =
            (uint16_t)frame[frame_size - 2u] |
            ((uint16_t)frame[frame_size - 1u] << 8);
        g_pc_comm_debug.last_rx_crc_calculated =
            CRC16_Calc(frame, (size_t)(frame_size - 2u), CRC16_INIT);
    } else {
        g_pc_comm_debug.last_rx_crc_received = 0u;
        g_pc_comm_debug.last_rx_crc_calculated = 0u;
    }

    PC_Comm_DebugUpdate(comm);
}

void PC_Comm_DebugRecordRxError(const PC_Comm_t *comm, const uint8_t *data,
                                uint16_t available_size,
                                int8_t parse_result) {
    uint16_t raw_len = available_size;
    if (raw_len > PC_PROTOCOL_MAX_FRAME_SIZE) {
        raw_len = PC_PROTOCOL_MAX_FRAME_SIZE;
    }

    g_pc_comm_debug.last_rx_result = parse_result;
    g_pc_comm_debug.last_rx_frame_size = 0u;
    g_pc_comm_debug.last_rx_cmd = (available_size >= 4u) ? data[3] : 0u;
    g_pc_comm_debug.last_rx_len = (available_size >= 3u) ? data[2] : 0u;
    g_pc_comm_debug.last_rx_crc_received = 0u;
    g_pc_comm_debug.last_rx_crc_calculated = 0u;
    g_pc_comm_debug.last_rx_raw_len = PcComm_DebugCopyRaw(
        g_pc_comm_debug.last_rx_raw, PC_PROTOCOL_MAX_FRAME_SIZE, data,
        raw_len);

    PC_Comm_DebugUpdate(comm);
}

void PC_Comm_DebugRecordTx(const uint8_t *data, uint16_t len,
                           uint8_t frame_count, int8_t tx_result) {
    g_pc_comm_debug.tx_count++;
    g_pc_comm_debug.tx_len = len;
    g_pc_comm_debug.tx_frame_count = frame_count;
    g_pc_comm_debug.tx_result = tx_result;
    g_pc_comm_debug.tx_dma_busy = s_tx_dma_busy ? 1u : 0u;
    g_pc_comm_debug.tx_raw_len = PcComm_DebugCopyRaw(
        g_pc_comm_debug.tx_raw, PC_COMM_DEBUG_TX_RAW_SIZE, data, len);
}

static void PcComm_TxDoneCallback(void) {
    s_tx_dma_busy = false;
    g_pc_comm_debug.tx_dma_busy = 0u;
}

static void PcComm_TxErrorCallback(void) {
    s_tx_dma_busy = false;
    g_pc_comm_debug.tx_dma_busy = 0u;
    g_pc_comm_debug.tx_dma_error_count++;
}

static bool PcComm_AppendFeedbackFrame(PC_Comm_t *comm, uint8_t cmd,
                                       uint16_t *tx_len) {
    if (comm == NULL || tx_len == NULL || *tx_len >= sizeof(s_tx_buf)) {
        return false;
    }

    uint16_t frame_len = PC_Protocol_BuildFrame(&comm->protocol, cmd,
                                                &s_tx_buf[*tx_len],
                                                (uint16_t)(sizeof(s_tx_buf) - *tx_len));
    if (frame_len == 0) {
        return false;
    }

    *tx_len = (uint16_t)(*tx_len + frame_len);
    return true;
}

static void PcComm_UpdateStatusFeedback(PC_Comm_t *comm) {
    PC_StatusFeedback_t status = {0};
    status.online = comm->online ? 1u : 0u;
    status.recv_count = comm->recv_count;
    status.cpu_temp = task_runtime.status.cpu_temp;
    status.command_source = g_pc_command_source;
    PC_Protocol_SetStatusFeedback(&comm->protocol, &status);
}

static bool PcComm_TransmitFeedback(PC_Comm_t *comm) {
    uint16_t tx_len = 0;
    uint8_t frame_count = 0;
    uint32_t now = osKernelGetTickCount();

    if (s_tx_dma_busy) {
        if ((now - s_tx_dma_start_tick) < PC_COMM_TX_DMA_TIMEOUT_MS) {
            g_pc_comm_debug.tx_dma_busy = 1u;
            g_pc_comm_debug.tx_busy_skip_count++;
            return false;
        }
        s_tx_dma_busy = false;
        g_pc_comm_debug.tx_dma_busy = 0u;
        g_pc_comm_debug.tx_dma_error_count++;
    }

    PcComm_UpdateStatusFeedback(comm);

    for (uint8_t i = 0; i < (uint8_t)(sizeof(s_feedback_cmds) / sizeof(s_feedback_cmds[0])); ++i) {
        if (PcComm_AppendFeedbackFrame(comm, s_feedback_cmds[i], &tx_len)) {
            frame_count++;
        }
    }

    if (tx_len > 0) {
        s_tx_dma_busy = true;
        s_tx_dma_start_tick = now;
        int8_t tx_result = BSP_UART_Transmit(BSP_UART_PC, s_tx_buf, tx_len, true);
        if (tx_result != BSP_OK) {
            s_tx_dma_busy = false;
            g_pc_comm_debug.tx_dma_error_count++;
        }
        PC_Comm_DebugRecordTx(s_tx_buf, tx_len, frame_count, tx_result);
        return tx_result == BSP_OK;
    }
    PC_Comm_DebugRecordTx(s_tx_buf, 0, frame_count, BSP_ERR);
    return false;
}

void Task_pc_comm(void *argument) {
    (void)argument;

    PC_Comm_t *comm = PC_Comm_GetInstance();
    if (comm == NULL) {
        osThreadTerminate(osThreadGetId());
        return;
    }

    while (!PC_Comm_Init(comm)) {
        PcComm_DebugRecordInitFail(g_pc_comm_debug.last_init_error);
        osDelay(20u);
    }

    while (BSP_UART_RegisterCallback(BSP_UART_PC, BSP_UART_TX_CPLT_CB,
                                     PcComm_TxDoneCallback) != BSP_OK ||
           BSP_UART_RegisterCallback(BSP_UART_PC, BSP_UART_ERROR_CB,
                                     PcComm_TxErrorCallback) != BSP_OK ||
           BSP_UART_RegisterCallback(BSP_UART_PC, BSP_UART_ABORT_TX_CPLT_CB,
                                     PcComm_TxErrorCallback) != BSP_OK) {
        PcComm_DebugRecordInitFail(-30);
        osDelay(20u);
    }

    uint32_t last_tx_tick = osKernelGetTickCount();

    while (1) {
        task_runtime.stack_water_mark.pc_comm = uxTaskGetStackHighWaterMark(NULL);

        uint32_t now = osKernelGetTickCount();

        /* Process receive and protocol */
        PC_Comm_Process(comm, now); 

        /* Periodic TX */
        if ((now - last_tx_tick) >= PC_COMM_TX_PERIOD_MS) {
            (void)PcComm_TransmitFeedback(comm);
            last_tx_tick = now;
        }

        /* 处理PC自动上台阶命令 */
        if (auto_ctrl_inited &&
            g_pc_command_source == PC_COMMAND_SOURCE_PC &&
            PC_Protocol_IsPCControlMode(&comm->protocol)) {
            const PC_AutoStepParams_t *step_params = PC_Protocol_GetAutoStepParams(&comm->protocol);
            if (step_params != NULL && !AutoCtrl_IsBusy(&auto_ctrl)) {
                AutoCtrl_SetYawSource(&auto_ctrl, AUTO_CTRL_YAW_SOURCE_PC);
                AutoCtrl_SetYawZeroOffset(&auto_ctrl, 0.0f);
                if (AutoCtrl_StartTemplate(
                    &auto_ctrl,
                    step_params->template_id,
                    step_params->travel_dir,
                    step_params->target_yaw_rad,
                    step_params->yaw_tolerance_rad,
                    AUTO_CTRL_SENSOR_MODE_NONE,
                    now)) {
                    comm->protocol.cmd.step.template_id = PC_STEP_TEMPLATE_NONE;
                }
            }
        }

        /* Check online status */
        if (now - comm->last_recv_time > 500u) {
            comm->online = false;
        }
        PC_Comm_DebugUpdate(comm);

        osDelay(PC_COMM_LOOP_PERIOD_MS);
    }
}
