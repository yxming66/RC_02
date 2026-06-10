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

/* Private function  -------------------------------------------------------- */
static void DR16_RxCpltCallback(void) {
  rx_size = DR16_FRAME_SIZE;
  osThreadFlagsSet(thread_alert, SIGNAL_DR16_RAW_REDY);
}

static void DR16_RxEventCallback(uint16_t size) {
  rx_size = size;
  osThreadFlagsSet(thread_alert, SIGNAL_DR16_RAW_REDY);
}

static void DR16_ErrorCallback(void) {
  rx_error = true;
  rx_size = 0;
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

  (void)HAL_UART_AbortReceive(huart);
  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                   UART_CLEAR_NEF | UART_CLEAR_OREF |
                                   UART_CLEAR_IDLEF);
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  return DEVICE_OK;
}

int8_t DR16_StartDmaRecv(DR16_t *dr16) {
  if (dr16 == NULL) return DEVICE_ERR_NULL;

  rx_size = 0;
  rx_error = false;
  osThreadFlagsClear(SIGNAL_DR16_RAW_REDY);

  if (BSP_UART_ReceiveToIdle(BSP_UART_RC, rx_buf, DR16_FRAME_SIZE, true) ==
      HAL_OK)
    return DEVICE_OK;
  return DEVICE_ERR;
}

bool DR16_WaitDmaCplt(uint32_t timeout) {
  return (osThreadFlagsWait(SIGNAL_DR16_RAW_REDY, osFlagsWaitAll, timeout) ==
          SIGNAL_DR16_RAW_REDY);
}

int8_t DR16_ParseData(DR16_t *dr16){
  if (dr16 == NULL) return DEVICE_ERR_NULL;
  if (rx_error || rx_size != DR16_FRAME_SIZE) return DEVICE_ERR;

  memcpy(&(dr16->raw_data), rx_buf, sizeof(dr16->raw_data));

  if (DR16_DataCorrupted(dr16)) {
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
  
  return DEVICE_OK;
}

int8_t DR16_Offline(DR16_t *dr16){
  if (dr16 == NULL) return DEVICE_ERR_NULL;

  dr16->header.online = false;
  memset(&(dr16->data), 0, sizeof(dr16->data));

  return DEVICE_OK;
}

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */
