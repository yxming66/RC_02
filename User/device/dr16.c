/*
    DR16接收机
    Example：

  DR16_Init(&dr16);
  
  while (1) {
    DR16_StartDmaRecv(&dr16);
    if (DR16_WaitDmaCplt(20)) {
      DR16_ParseData(&dr16);
    } else {
      DR16_Offline(&dr16);
    }
}
*/

/* Includes ----------------------------------------------------------------- */
#include "dr16.h"
#include "bsp/uart.h"
#include "bsp/time.h"
#include "device.h"

#include <string.h>
#include <stdbool.h>

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */
/* Private define ----------------------------------------------------------- */
#define DR16_CH_VALUE_MIN (364u)
#define DR16_CH_VALUE_MID (1024u)
#define DR16_CH_VALUE_MAX (1684u)
#define DR16_FRAME_SIZE ((uint16_t)sizeof(DR16_RawData_t))

/* USER DEFINE BEGIN */

/* USER DEFINE END */

/* Private macro ------------------------------------------------------------ */
/* Private typedef ---------------------------------------------------------- */
/* Private variables -------------------------------------------------------- */

static osThreadId_t thread_alert;
static bool inited = false;
static uint8_t rx_buf[sizeof(DR16_RawData_t)];
static volatile uint16_t rx_size = 0;
static volatile bool rx_error = false;

volatile DR16_Debug_t g_dr16_debug = {0};

/* Private function  -------------------------------------------------------- */
static void DR16_UpdateDebugUartState(void) {
  UART_HandleTypeDef *huart = BSP_UART_GetHandle(BSP_UART_RC);
  if (huart == NULL) {
    return;
  }

  g_dr16_debug.uart_isr = huart->Instance->ISR;
  g_dr16_debug.uart_cr1 = huart->Instance->CR1;
  g_dr16_debug.uart_cr3 = huart->Instance->CR3;
  g_dr16_debug.uart_error_code = huart->ErrorCode;
  g_dr16_debug.uart_rx_state = huart->RxState;
  g_dr16_debug.uart_reception_type = huart->ReceptionType;
  if (huart->hdmarx != NULL) {
    g_dr16_debug.dma_remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx);
  } else {
    g_dr16_debug.dma_remaining = 0u;
  }
}

static void DR16_RxCpltCallback(void) {
  rx_size = DR16_FRAME_SIZE;
  g_dr16_debug.rx_cplt_count++;
  g_dr16_debug.last_rx_size = rx_size;
  DR16_UpdateDebugUartState();
  osThreadFlagsSet(thread_alert, SIGNAL_DR16_RAW_REDY);
}

static void DR16_RxEventCallback(uint16_t size) {
  rx_size = size;
  g_dr16_debug.rx_event_count++;
  g_dr16_debug.last_rx_size = rx_size;
  DR16_UpdateDebugUartState();
  osThreadFlagsSet(thread_alert, SIGNAL_DR16_RAW_REDY);
}

static void DR16_ErrorCallback(void) {
  rx_error = true;
  rx_size = 0;
  g_dr16_debug.rx_error_count++;
  g_dr16_debug.last_rx_size = rx_size;
  DR16_UpdateDebugUartState();
  osThreadFlagsSet(thread_alert, SIGNAL_DR16_RAW_REDY);
}

static bool DR16_DataCorrupted(const DR16_t *dr16) {
  if (dr16 == NULL) return DEVICE_ERR_NULL;

  if ((dr16->raw_data.ch_r_x < DR16_CH_VALUE_MIN) ||
      (dr16->raw_data.ch_r_x > DR16_CH_VALUE_MAX))
    return DEVICE_ERR;

  if ((dr16->raw_data.ch_r_y < DR16_CH_VALUE_MIN) ||
      (dr16->raw_data.ch_r_y > DR16_CH_VALUE_MAX))
    return DEVICE_ERR;

  if ((dr16->raw_data.ch_l_x < DR16_CH_VALUE_MIN) ||
      (dr16->raw_data.ch_l_x > DR16_CH_VALUE_MAX))
    return DEVICE_ERR;

  if ((dr16->raw_data.ch_l_y < DR16_CH_VALUE_MIN) ||
      (dr16->raw_data.ch_l_y > DR16_CH_VALUE_MAX))
    return DEVICE_ERR;

  if (dr16->raw_data.sw_l == 0) return DEVICE_ERR;

  if (dr16->raw_data.sw_r == 0) return DEVICE_ERR;

  return DEVICE_OK;
}

/* Exported functions ------------------------------------------------------- */
int8_t DR16_Init(DR16_t *dr16) {
  if (dr16 == NULL) return DEVICE_ERR_NULL;
  if (inited) return DEVICE_ERR_INITED;
  if ((thread_alert = osThreadGetId()) == NULL) return DEVICE_ERR_NULL;

  BSP_UART_RegisterCallback(BSP_UART_RC, BSP_UART_RX_CPLT_CB,
                            DR16_RxCpltCallback);
  BSP_UART_RegisterRxEventCallback(BSP_UART_RC, DR16_RxEventCallback);
  BSP_UART_RegisterCallback(BSP_UART_RC, BSP_UART_ERROR_CB, DR16_ErrorCallback);

  inited = true;
  return DEVICE_OK;
}

int8_t DR16_Restart(void) {
  UART_HandleTypeDef *huart = BSP_UART_GetHandle(BSP_UART_RC);
  if (huart == NULL) return DEVICE_ERR_NULL;

  g_dr16_debug.restart_count++;
  (void)HAL_UART_AbortReceive(huart);
  (void)HAL_UART_DMAStop(huart);
  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                   UART_CLEAR_NEF | UART_CLEAR_OREF |
                                   UART_CLEAR_IDLEF);
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  DR16_UpdateDebugUartState();
  return DEVICE_OK;
}

int8_t DR16_StartDmaRecv(DR16_t *dr16) {
  if (dr16 == NULL) return DEVICE_ERR_NULL;

  rx_size = 0;
  rx_error = false;
  osThreadFlagsClear(SIGNAL_DR16_RAW_REDY);

  const int8_t result = BSP_UART_ReceiveToIdle(BSP_UART_RC, rx_buf,
                                               DR16_FRAME_SIZE, true);
  g_dr16_debug.last_start_result = result;
  DR16_UpdateDebugUartState();
  if (result == HAL_OK)
    return DEVICE_OK;
  g_dr16_debug.dma_start_error_count++;
  return DEVICE_ERR;
}

bool DR16_WaitDmaCplt(uint32_t timeout) {
  return (osThreadFlagsWait(SIGNAL_DR16_RAW_REDY, osFlagsWaitAll, timeout) ==
          SIGNAL_DR16_RAW_REDY);
}

int8_t DR16_ParseData(DR16_t *dr16){
  if (dr16 == NULL) {
    g_dr16_debug.last_parse_result = DEVICE_ERR_NULL;
    g_dr16_debug.parse_error_count++;
    return DEVICE_ERR_NULL;
  }
  if (rx_error) {
    g_dr16_debug.last_parse_result = DEVICE_ERR;
    g_dr16_debug.parse_error_count++;
    return DEVICE_ERR;
  }
  if (rx_size != DR16_FRAME_SIZE) {
    g_dr16_debug.last_parse_result = DEVICE_ERR;
    g_dr16_debug.parse_error_count++;
    g_dr16_debug.size_error_count++;
    return DEVICE_ERR;
  }

  memcpy(&(dr16->raw_data), rx_buf, sizeof(dr16->raw_data));

  if (DR16_DataCorrupted(dr16)) {
    g_dr16_debug.last_parse_result = DEVICE_ERR;
    g_dr16_debug.parse_error_count++;
    g_dr16_debug.corrupt_error_count++;
    return DEVICE_ERR;
  }

  dr16->header.online = true;
  dr16->header.last_online_time = BSP_TIME_Get_us();
  
  memset(&(dr16->data), 0, sizeof(dr16->data));

  float full_range = (float)(DR16_CH_VALUE_MAX - DR16_CH_VALUE_MIN);

  // 解析摇杆数据
  dr16->data.ch_r_x = 2.0f * ((float)dr16->raw_data.ch_r_x - DR16_CH_VALUE_MID) / full_range;
  dr16->data.ch_r_y = 2.0f * ((float)dr16->raw_data.ch_r_y - DR16_CH_VALUE_MID) / full_range;
  dr16->data.ch_l_x = 2.0f * ((float)dr16->raw_data.ch_l_x - DR16_CH_VALUE_MID) / full_range;
  dr16->data.ch_l_y = 2.0f * ((float)dr16->raw_data.ch_l_y - DR16_CH_VALUE_MID) / full_range;

  // 解析拨杆位置
  dr16->data.sw_l = (DR16_SwitchPos_t)dr16->raw_data.sw_l;
  dr16->data.sw_r = (DR16_SwitchPos_t)dr16->raw_data.sw_r;

  // 解析鼠标数据
  dr16->data.mouse.x = dr16->raw_data.x;
  dr16->data.mouse.y = dr16->raw_data.y;
  dr16->data.mouse.z = dr16->raw_data.z;

  dr16->data.mouse.l_click = dr16->raw_data.press_l;
  dr16->data.mouse.r_click = dr16->raw_data.press_r;

  // 解析键盘按键 - 使用union简化代码
  uint16_t key_value = dr16->raw_data.key;
  
  // 解析键盘位映射（W-B键，位0-15）
  for (int i = DR16_KEY_W; i <= DR16_KEY_B; i++) {
    dr16->data.keyboard.key[i] = (key_value & (1 << i)) != 0;
  }

  // 解析鼠标点击
  dr16->data.keyboard.key[DR16_L_CLICK] = dr16->data.mouse.l_click;
  dr16->data.keyboard.key[DR16_R_CLICK] = dr16->data.mouse.r_click;

  // 解析第五通道
  dr16->data.ch_res = -2.0f * ((float)dr16->raw_data.res - DR16_CH_VALUE_MID) / full_range;
  
  g_dr16_debug.last_parse_result = DEVICE_OK;
  g_dr16_debug.parse_ok_count++;
  return DEVICE_OK;
}

int8_t DR16_Offline(DR16_t *dr16){
  if (dr16 == NULL) return DEVICE_ERR_NULL;

  dr16->header.online = false;
  memset(&(dr16->data), 0, sizeof(dr16->data));
  g_dr16_debug.offline_count++;

  return DEVICE_OK;
}

void DR16_RecordTimeoutOffline(void) {
  g_dr16_debug.timeout_offline_count++;
}

void DR16_Service(DR16_t *dr16) {
  UART_HandleTypeDef *huart = BSP_UART_GetHandle(BSP_UART_RC);
  if (huart == NULL || dr16 == NULL) {
    return;
  }

  DR16_UpdateDebugUartState();
  const bool need_rearm =
      huart->RxState == HAL_UART_STATE_READY ||
      huart->ErrorCode != HAL_UART_ERROR_NONE ||
      huart->ReceptionType == HAL_UART_RECEPTION_STANDARD;
  if (!need_rearm) {
    return;
  }

  g_dr16_debug.service_rearm_count++;
  (void)DR16_Restart();
  (void)DR16_StartDmaRecv(dr16);
}

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */
