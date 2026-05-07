/*
 * PC_COMM Task - 上位机通信任务
 */
#include "task/user_task.h"
#include "module/pc_protocol/pc_protocol.h"
#include "bsp/uart.h"
#include "main.h"

#define PC_COMM_TX_PERIOD_MS (20u)  /* 50Hz */

static uint8_t s_tx_buf[256];
static uint8_t s_rx_buf[256];

static bool PcComm_TransmitFeedback(PC_Comm_t *comm) {
    uint16_t tx_len = PC_Protocol_BuildFrame(&comm->protocol, PC_FEEDBACK_HEARTBEAT,
                                             s_tx_buf, sizeof(s_tx_buf));
    if (tx_len > 0) {
        return BSP_UART_Transmit(BSP_UART_PC, s_tx_buf, tx_len, false) == BSP_OK;
    }
    return false;
}

void Task_pc_comm(void *argument) {
    (void)argument;

    PC_Comm_t *comm = PC_Comm_GetInstance();
    if (comm == NULL) {
        osThreadTerminate(osThreadGetId());
        return;
    }

    if (!PC_Comm_Init(comm)) {
        osThreadTerminate(osThreadGetId());
        return;
    }

    /* Start initial receive */
    (void)BSP_UART_Receive(BSP_UART_PC, s_rx_buf, sizeof(s_rx_buf), true);

    uint32_t last_tx_tick = osKernelGetTickCount();

    while (1) {
        task_runtime.stack_water_mark.pc_comm = uxTaskGetStackHighWaterMark(NULL);

        uint32_t now = osKernelGetTickCount();

        /* Periodic TX */
        if ((now - last_tx_tick) >= PC_COMM_TX_PERIOD_MS) {
            (void)PcComm_TransmitFeedback(comm);
            last_tx_tick = now;
        }

        /* Process receive and protocol */
        PC_Comm_Process(comm, now);

        /* Check online status */
        if (now - comm->last_recv_time > 1000) {
            comm->online = false;
        }

        osDelay(PC_COMM_TX_PERIOD_MS);
    }
}
