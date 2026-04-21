/*
  pc_uart_tx Task
  通过 UART10 DMA 接收上位机下发的 XYZ 指令
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"

/* USER INCLUDE BEGIN */
#include <string.h>
#include <stdint.h>
#include "bsp/uart.h"
#include "component/crc16.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
#define PC_UART_RX_FRAME_HEADER_0 (0xA5u)
#define PC_UART_RX_FRAME_HEADER_1 (0x5Au)
#define PC_UART_RX_CPLT_FLAG      (1u << 0)
#define PC_UART_TX_PERIOD_MS      (200u)
#define PC_UART_FRAME_HEADER_SIZE (2u)
#define PC_UART_FRAME_PAYLOAD_SIZE (sizeof(float) * 3u)
#define PC_UART_FRAME_CRC_SIZE    (sizeof(uint16_t))
#define PC_UART_FRAME_SIZE \
  (PC_UART_FRAME_HEADER_SIZE + PC_UART_FRAME_PAYLOAD_SIZE + PC_UART_FRAME_CRC_SIZE)

/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
static uint8_t pc_uart_rx_buf[PC_UART_FRAME_SIZE];
static uint8_t pc_uart_tx_buf[PC_UART_FRAME_SIZE];
static osThreadId_t pc_uart_thread_id;
static volatile float pc_uart_target_x;
static volatile float pc_uart_target_y;
static volatile float pc_uart_target_z;

/* Private function --------------------------------------------------------- */
static void PcUartRxCpltCallback(void);
static bool PcUart_StartDmaRecv(void);
static void PcUart_SendTestFrame(void);
static bool PcUart_WaitDmaCplt(uint32_t timeout);
static bool PcUart_ParseFrame(const uint8_t *buf);
static void PcUart_WriteFloat(uint8_t *buf, uint32_t offset, float value);
static float PcUart_ReadFloat(const uint8_t *buf, uint32_t offset);

float PcUart_GetTargetX(void);
float PcUart_GetTargetY(void);
float PcUart_GetTargetZ(void);

/* Exported functions ------------------------------------------------------- */
void Task_pc_uart_rx(void *argument) {
  (void)argument;

  uint32_t last_tx_tick = 0u;

  osDelay(PC_UART_RX_INIT_DELAY);

  pc_uart_thread_id = osThreadGetId();
  BSP_UART_RegisterCallback(BSP_UART_PC, BSP_UART_RX_CPLT_CB, PcUartRxCpltCallback);
  last_tx_tick = osKernelGetTickCount();

  while (1) {
    if ((osKernelGetTickCount() - last_tx_tick) >= PC_UART_TX_PERIOD_MS) {
      PcUart_SendTestFrame();
      last_tx_tick = osKernelGetTickCount();
    }

    if (!PcUart_StartDmaRecv()) {
      osDelay(1);
      continue;
    }

    if (!PcUart_WaitDmaCplt(osWaitForever)) {
      continue;
    }

    if (!PcUart_ParseFrame(pc_uart_rx_buf)) {
      continue;
    }
  }
}

static void PcUartRxCpltCallback(void) {
  if (pc_uart_thread_id != NULL) {
    (void)osThreadFlagsSet(pc_uart_thread_id, PC_UART_RX_CPLT_FLAG);
  }
}

static bool PcUart_StartDmaRecv(void) {
  memset(pc_uart_rx_buf, 0, sizeof(pc_uart_rx_buf));
  return BSP_UART_Receive(BSP_UART_PC, pc_uart_rx_buf, sizeof(pc_uart_rx_buf), true) == BSP_OK;
}

static void PcUart_SendTestFrame(void) {
  uint16_t crc = 0u;

  pc_uart_tx_buf[0] = PC_UART_RX_FRAME_HEADER_0;
  pc_uart_tx_buf[1] = PC_UART_RX_FRAME_HEADER_1;
  PcUart_WriteFloat(pc_uart_tx_buf, 2u, pc_uart_target_x);
  PcUart_WriteFloat(pc_uart_tx_buf, 6u, pc_uart_target_y);
  PcUart_WriteFloat(pc_uart_tx_buf, 10u, pc_uart_target_z);
  crc = CRC16_Calc(pc_uart_tx_buf, PC_UART_FRAME_SIZE - PC_UART_FRAME_CRC_SIZE, CRC16_INIT);
  pc_uart_tx_buf[14] = (uint8_t)(crc & 0xFFu);
  pc_uart_tx_buf[15] = (uint8_t)((crc >> 8) & 0xFFu);

  (void)BSP_UART_Transmit(BSP_UART_PC, pc_uart_tx_buf, sizeof(pc_uart_tx_buf), false);
}

static bool PcUart_WaitDmaCplt(uint32_t timeout) {
  uint32_t flags = osThreadFlagsWait(PC_UART_RX_CPLT_FLAG, osFlagsWaitAny, timeout);
  return (flags & PC_UART_RX_CPLT_FLAG) != 0u;
}

static bool PcUart_ParseFrame(const uint8_t *buf) {
  if ((buf[0] != PC_UART_RX_FRAME_HEADER_0) || (buf[1] != PC_UART_RX_FRAME_HEADER_1)) {
    return false;
  }

  // if (!CRC16_Verify(buf, PC_UART_FRAME_SIZE)) {
  //   return false;
  // }

  pc_uart_target_x = PcUart_ReadFloat(buf, 2u);
  pc_uart_target_y = PcUart_ReadFloat(buf, 6u);
  pc_uart_target_z = PcUart_ReadFloat(buf, 10u);

  return true;
}

static void PcUart_WriteFloat(uint8_t *buf, uint32_t offset, float value) {
  memcpy(&buf[offset], &value, sizeof(float));
}

static float PcUart_ReadFloat(const uint8_t *buf, uint32_t offset) {
  float value = 0.0f;

  memcpy(&value, &buf[offset], sizeof(float));
  return value;
}

float PcUart_GetTargetX(void) {
  return pc_uart_target_x;
}

float PcUart_GetTargetY(void) {
  return pc_uart_target_y;
}

float PcUart_GetTargetZ(void) {
  return pc_uart_target_z;
}