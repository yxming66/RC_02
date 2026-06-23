/* Includes ----------------------------------------------------------------- */
#include "fdcan.h"
#include "bsp/fdcan.h"
#include "bsp/bsp.h"
#include <fdcan.h>
#include <cmsis_os2.h>
#include <string.h>

#include "bsp/time.h"

/* Private define ----------------------------------------------------------- */
#define FDCAN_QUEUE_MUTEX_TIMEOUT         100
#define FDCAN_RX_MAX_FRAMES_PER_CALLBACK 16u
#define FDCAN_TX_ABORT_WAIT_COUNT        1000u

/* Private macro ------------------------------------------------------------ */

/* ===== FDCAN_FilterTypeDef 配置表 =====
 * 定义每个FDCAN实例的过滤器参数表。
 * 过滤器表参数说明：
 *   idx       idtype               ftype                id1           id2           rxidx
 *   过滤器编号 标识符类型            过滤器类型           过滤器ID1      过滤器ID2      接收缓冲区索引
 */
#ifdef FDCAN1_EN
  #define FDCAN1_FILTER_CONFIG_TABLE(X) \
    X(0,       FDCAN_STANDARD_ID,   FDCAN_FILTER_MASK,   0x000     ,   0x000     ,   0) \
    X(1,       FDCAN_EXTENDED_ID,   FDCAN_FILTER_MASK,   0x00000000,   0x00000000,   0) 
  #define FDCAN1_GLOBAL_FILTER FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE
#endif
#ifdef FDCAN2_EN
  #define FDCAN2_FILTER_CONFIG_TABLE(X) \
    X(0,       FDCAN_STANDARD_ID,   FDCAN_FILTER_MASK,   0x000     ,   0x000     ,   0) \
    X(1,       FDCAN_EXTENDED_ID,   FDCAN_FILTER_MASK,   0x00000000,   0x00000000,   0) 
  #define FDCAN2_GLOBAL_FILTER FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE
#endif
#ifdef FDCAN3_EN
  #define FDCAN3_FILTER_CONFIG_TABLE(X) \
    X(0,       FDCAN_STANDARD_ID,   FDCAN_FILTER_MASK,   0x000     ,   0x000     ,   0) \
    X(1,       FDCAN_EXTENDED_ID,   FDCAN_FILTER_MASK,   0x00000000,   0x00000000,   0) 
  #define FDCAN3_GLOBAL_FILTER FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE
#endif

/* ====宏展开实现==== */
#define FDCAN_FILTER_TO_RXFIFO_ENUM_INNER(FIFOIndex) FDCAN_FILTER_TO_RXFIFO##FIFOIndex 
#define FDCAN_FILTER_TO_RXFIFO_ENUM(FIFOIndex) FDCAN_FILTER_TO_RXFIFO_ENUM_INNER(FIFOIndex)
#define FDCAN_CONFIG_FILTER(idx, idtype, ftype, id1, id2, rxidx) \
    sFilterConfig.FilterIndex = (idx); \
    sFilterConfig.IdType = (idtype); \
    sFilterConfig.FilterType = (ftype); \
    sFilterConfig.FilterConfig = (FDCAN_FILTER_TO_RXFIFO_ENUM(FDCANX_RX_FIFO)); \
    sFilterConfig.FilterID1 = (id1); \
    sFilterConfig.FilterID2 = (id2); \
    sFilterConfig.RxBufferIndex = (rxidx); \
    HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig); 

#define FDCAN_NOTIFY_FLAG_RXFIFO_INNER(FIFO_IDX) \
  (FDCAN_IT_RX_FIFO##FIFO_IDX##_NEW_MESSAGE | \
   FDCAN_IT_RX_FIFO##FIFO_IDX##_FULL | \
   FDCAN_IT_RX_FIFO##FIFO_IDX##_MESSAGE_LOST)
#define FDCAN_NOTIFY_FLAG_RXFIFO(FIFO_IDX)         FDCAN_NOTIFY_FLAG_RXFIFO_INNER(FIFO_IDX)
#define FDCAN_NOTIFY_FLAG_ERROR \
  (FDCAN_IT_RAM_ACCESS_FAILURE | \
   FDCAN_IT_ERROR_LOGGING_OVERFLOW | \
   FDCAN_IT_ERROR_PASSIVE | \
   FDCAN_IT_ERROR_WARNING | \
   FDCAN_IT_BUS_OFF | \
   FDCAN_IT_RAM_WATCHDOG | \
   FDCAN_IT_ARB_PROTOCOL_ERROR | \
   FDCAN_IT_DATA_PROTOCOL_ERROR | \
   FDCAN_IT_RESERVED_ADDRESS_ACCESS)
#define FDCANx_NOTIFY_FLAGS(FIFO_MACRO)   (FDCAN_NOTIFY_FLAG_RXFIFO(FIFO_MACRO) | FDCAN_NOTIFY_FLAG_ERROR)

#define FDCANX_MSG_PENDING_CB_INNER(FIFO_IDX) HAL_FDCAN_RX_FIFO##FIFO_IDX##_MSG_PENDING_CB
#define FDCANX_MSG_PENDING_CB(FIFO_IDX)   FDCANX_MSG_PENDING_CB_INNER(FIFO_IDX)
/* Private typedef ---------------------------------------------------------- */
typedef struct BSP_FDCAN_QueueNode {
    BSP_FDCAN_t fdcan;
    uint32_t can_id;
    osMessageQueueId_t queue;
    uint8_t queue_size;
  bool latest_only;
  volatile bool latest_valid;
  BSP_FDCAN_Message_t latest_msg;
    struct BSP_FDCAN_QueueNode *next;
} BSP_FDCAN_QueueNode_t;

/* Private variables -------------------------------------------------------- */
static BSP_FDCAN_QueueNode_t *queue_list = NULL;
static osMutexId_t queue_mutex = NULL;
static void (*FDCAN_Callback[BSP_FDCAN_NUM][HAL_FDCAN_CB_NUM])(void);
static bool inited = false;
static BSP_FDCAN_IdParser_t id_parser = NULL;
static BSP_FDCAN_TxQueue_t tx_queues[BSP_FDCAN_NUM];
static const uint8_t fdcan_dlc2len[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
volatile BSP_FDCAN_RxDebug_t g_bsp_fdcan_rx_debug = {0};
static volatile bool fdcan_recover_pending[BSP_FDCAN_NUM] = {false};
static volatile bool fdcan_tx_drain_active[BSP_FDCAN_NUM] = {false};

/* Private function prototypes ---------------------------------------------- */
static BSP_FDCAN_t FDCAN_Get(FDCAN_HandleTypeDef *hfdcan);
static BSP_FDCAN_QueueNode_t *BSP_FDCAN_FindNode(BSP_FDCAN_t fdcan, uint32_t can_id);
static int8_t BSP_FDCAN_CreateIdNode(BSP_FDCAN_t fdcan, uint32_t can_id, uint8_t queue_size, bool latest_only);
static void BSP_FDCAN_RxFifo0Callback(void);
static void BSP_FDCAN_RxFifo1Callback(void);
static BSP_FDCAN_FrameType_t BSP_FDCAN_GetFrameType(FDCAN_RxHeaderTypeDef *header);
static uint32_t BSP_FDCAN_DefaultIdParser(uint32_t original_id, BSP_FDCAN_FrameType_t frame_type);
static void BSP_FDCAN_TxQueueInit(BSP_FDCAN_t fdcan);
static bool BSP_FDCAN_TxQueuePush(BSP_FDCAN_t fdcan, BSP_FDCAN_TxMessage_t *msg);
static bool BSP_FDCAN_TxQueuePeek(BSP_FDCAN_t fdcan, BSP_FDCAN_TxMessage_t *msg);
static bool BSP_FDCAN_TxQueuePop(BSP_FDCAN_t fdcan, BSP_FDCAN_TxMessage_t *msg);
static bool BSP_FDCAN_TxQueueIsEmpty(BSP_FDCAN_t fdcan);
static void BSP_FDCAN_TxQueueDrain(BSP_FDCAN_t fdcan);
static void BSP_FDCAN_SetTxFifoEmptyNotification(BSP_FDCAN_t fdcan, bool enable);
static uint32_t BSP_FDCAN_NotifyFlags(BSP_FDCAN_t fdcan);
static bool BSP_FDCAN_ErrorNeedsRecovery(FDCAN_HandleTypeDef *hfdcan);
static uint32_t BSP_FDCAN_TxBufferMask(FDCAN_HandleTypeDef *hfdcan);
static void BSP_FDCAN_AbortPendingTx(FDCAN_HandleTypeDef *hfdcan);
static void BSP_FDCAN_ClearIrFlags(FDCAN_HandleTypeDef *hfdcan);
static int8_t BSP_FDCAN_RecoverIfNeeded(BSP_FDCAN_t fdcan, bool force);

/* Private functions -------------------------------------------------------- */
static BSP_FDCAN_t FDCAN_Get(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == NULL) return BSP_FDCAN_ERR;
  if (hfdcan->Instance == FDCAN1) return BSP_FDCAN_1;
  else if (hfdcan->Instance == FDCAN2) return BSP_FDCAN_2;
  else if (hfdcan->Instance == FDCAN3) return BSP_FDCAN_3;
  else return BSP_FDCAN_ERR;
}

static BSP_FDCAN_QueueNode_t *BSP_FDCAN_FindNode(BSP_FDCAN_t fdcan, uint32_t can_id) {
  BSP_FDCAN_QueueNode_t *node = queue_list;
  while (node != NULL) {
    if (node->fdcan == fdcan && node->can_id == can_id) return node;
    node = node->next;
  }
  return NULL;
}

static BSP_FDCAN_QueueNode_t *BSP_FDCAN_FindNodeNoWait(BSP_FDCAN_t fdcan, uint32_t can_id) {
  if (osMutexAcquire(queue_mutex, 0u) != osOK) return NULL;
  BSP_FDCAN_QueueNode_t *node = BSP_FDCAN_FindNode(fdcan, can_id);
  osMutexRelease(queue_mutex);
  return node;
}

static int8_t BSP_FDCAN_CreateIdNode(BSP_FDCAN_t fdcan, uint32_t can_id, uint8_t queue_size, bool latest_only) {
  if (queue_size == 0) queue_size = BSP_FDCAN_DEFAULT_QUEUE_SIZE;
  if (osMutexAcquire(queue_mutex, FDCAN_QUEUE_MUTEX_TIMEOUT) != osOK) return BSP_ERR_TIMEOUT;
  BSP_FDCAN_QueueNode_t *node = queue_list;
  while (node != NULL) {
    if (node->fdcan == fdcan && node->can_id == can_id) {
      osMutexRelease(queue_mutex);
      return BSP_ERR;
    }
    node = node->next;
  }
  
  BSP_FDCAN_QueueNode_t *new_node = (BSP_FDCAN_QueueNode_t *)BSP_Malloc(sizeof(BSP_FDCAN_QueueNode_t));
  if (new_node == NULL) { osMutexRelease(queue_mutex); return BSP_ERR_NULL; }
  memset(new_node, 0, sizeof(BSP_FDCAN_QueueNode_t));
  if (!latest_only) {
    new_node->queue = osMessageQueueNew(queue_size, sizeof(BSP_FDCAN_Message_t), NULL);
    if (new_node->queue == NULL) { BSP_Free(new_node); osMutexRelease(queue_mutex); return BSP_ERR; }
  }
  new_node->fdcan = fdcan;
  new_node->can_id = can_id;
  new_node->queue_size = queue_size;
  new_node->latest_only = latest_only;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  new_node->next = queue_list;
  queue_list = new_node;
  if (primask == 0u) {
    __enable_irq();
  }
  osMutexRelease(queue_mutex);
  return BSP_OK;
}

static BSP_FDCAN_FrameType_t BSP_FDCAN_GetFrameType(FDCAN_RxHeaderTypeDef *header) {
  if (header->RxFrameType == FDCAN_REMOTE_FRAME) {
    return (header->IdType == FDCAN_EXTENDED_ID) ? BSP_FDCAN_FRAME_EXT_REMOTE : BSP_FDCAN_FRAME_STD_REMOTE;
  } else {
    return (header->IdType == FDCAN_EXTENDED_ID) ? BSP_FDCAN_FRAME_EXT_DATA : BSP_FDCAN_FRAME_STD_DATA;
  }
}

static uint32_t BSP_FDCAN_DefaultIdParser(uint32_t original_id, BSP_FDCAN_FrameType_t frame_type) {
  (void)frame_type;
  return original_id;
}

static uint32_t BSP_FDCAN_EncodeDLC(uint8_t dlc) {
  if (dlc <= 8) return dlc;
  if (dlc <= 12) return FDCAN_DLC_BYTES_12;
  if (dlc <= 16) return FDCAN_DLC_BYTES_16;
  if (dlc <= 20) return FDCAN_DLC_BYTES_20;
  if (dlc <= 24) return FDCAN_DLC_BYTES_24;
  if (dlc <= 32) return FDCAN_DLC_BYTES_32;
  if (dlc <= 48) return FDCAN_DLC_BYTES_48;
  return FDCAN_DLC_BYTES_64;
}

static void BSP_FDCAN_TxQueueInit(BSP_FDCAN_t fdcan) {
  if (fdcan >= BSP_FDCAN_NUM) return;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  tx_queues[fdcan].head = 0;
  tx_queues[fdcan].tail = 0;
  g_bsp_fdcan_rx_debug.tx_queue_depth[fdcan] = 0;
  if (primask == 0u) {
    __enable_irq();
  }
}

static uint32_t BSP_FDCAN_TxQueueDepthUnsafe(const BSP_FDCAN_TxQueue_t *queue) {
  if (queue == NULL) return 0u;
  if (queue->head >= queue->tail) {
    return queue->head - queue->tail;
  }
  return BSP_FDCAN_TX_QUEUE_SIZE - queue->tail + queue->head;
}

static bool BSP_FDCAN_TxQueuePush(BSP_FDCAN_t fdcan, BSP_FDCAN_TxMessage_t *msg) {
  if (fdcan >= BSP_FDCAN_NUM || msg == NULL) return false;
  BSP_FDCAN_TxQueue_t *queue = &tx_queues[fdcan];
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t next_head = (queue->head + 1) % BSP_FDCAN_TX_QUEUE_SIZE;
  if (next_head == queue->tail) {
    if (primask == 0u) {
      __enable_irq();
    }
    return false;
  }
  queue->buffer[queue->head] = *msg;
  queue->head = next_head;
  const uint32_t depth = BSP_FDCAN_TxQueueDepthUnsafe(queue);
  g_bsp_fdcan_rx_debug.tx_queue_push_count[fdcan]++;
  g_bsp_fdcan_rx_debug.tx_queue_depth[fdcan] = depth;
  if (depth > g_bsp_fdcan_rx_debug.tx_queue_max_depth[fdcan]) {
    g_bsp_fdcan_rx_debug.tx_queue_max_depth[fdcan] = depth;
  }
  if (primask == 0u) {
    __enable_irq();
  }
  return true;
}

static bool BSP_FDCAN_TxQueuePeek(BSP_FDCAN_t fdcan, BSP_FDCAN_TxMessage_t *msg) {
  if (fdcan >= BSP_FDCAN_NUM || msg == NULL) return false;
  BSP_FDCAN_TxQueue_t *queue = &tx_queues[fdcan];
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (queue->head == queue->tail) {
    if (primask == 0u) {
      __enable_irq();
    }
    return false;
  }
  *msg = queue->buffer[queue->tail];
  if (primask == 0u) {
    __enable_irq();
  }
  return true;
}

static bool BSP_FDCAN_TxQueuePop(BSP_FDCAN_t fdcan, BSP_FDCAN_TxMessage_t *msg) {
  if (fdcan >= BSP_FDCAN_NUM || msg == NULL) return false;
  BSP_FDCAN_TxQueue_t *queue = &tx_queues[fdcan];
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (queue->head == queue->tail) {
    if (primask == 0u) {
      __enable_irq();
    }
    return false;
  }
  *msg = queue->buffer[queue->tail];
  queue->tail = (queue->tail + 1) % BSP_FDCAN_TX_QUEUE_SIZE;
  g_bsp_fdcan_rx_debug.tx_queue_pop_count[fdcan]++;
  g_bsp_fdcan_rx_debug.tx_queue_depth[fdcan] =
      BSP_FDCAN_TxQueueDepthUnsafe(queue);
  if (primask == 0u) {
    __enable_irq();
  }
  return true;
}

static bool BSP_FDCAN_TxQueueIsEmpty(BSP_FDCAN_t fdcan) {
  if (fdcan >= BSP_FDCAN_NUM) return true;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bool is_empty = tx_queues[fdcan].head == tx_queues[fdcan].tail;
  if (primask == 0u) {
    __enable_irq();
  }
  return is_empty;
}

static void BSP_FDCAN_SetTxFifoEmptyNotification(BSP_FDCAN_t fdcan, bool enable) {
  FDCAN_HandleTypeDef *hfdcan = BSP_FDCAN_GetHandle(fdcan);
  if (hfdcan == NULL) return;
  if (enable) {
    (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_FIFO_EMPTY, 0);
  } else {
    (void)HAL_FDCAN_DeactivateNotification(hfdcan, FDCAN_IT_TX_FIFO_EMPTY);
  }
}

static void BSP_FDCAN_TxQueueDrain(BSP_FDCAN_t fdcan) {
  if (fdcan >= BSP_FDCAN_NUM) return;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (fdcan_tx_drain_active[fdcan]) {
    if (primask == 0u) {
      __enable_irq();
    }
    return;
  }
  fdcan_tx_drain_active[fdcan] = true;
  if (primask == 0u) {
    __enable_irq();
  }

  FDCAN_HandleTypeDef *hfdcan = BSP_FDCAN_GetHandle(fdcan);
  if (hfdcan == NULL || hfdcan->State != HAL_FDCAN_STATE_BUSY) {
    fdcan_tx_drain_active[fdcan] = false;
    return;
  }

  BSP_FDCAN_TxMessage_t msg;
  while (!BSP_FDCAN_TxQueueIsEmpty(fdcan) &&
         HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0u) {
    if (!BSP_FDCAN_TxQueuePeek(fdcan, &msg)) break;
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &msg.header, msg.data) != HAL_OK) {
      ++g_bsp_fdcan_rx_debug.tx_add_fail_count[fdcan];
      fdcan_recover_pending[fdcan] = true;
      break;
    }
    (void)BSP_FDCAN_TxQueuePop(fdcan, &msg);
  }

  BSP_FDCAN_SetTxFifoEmptyNotification(fdcan, !BSP_FDCAN_TxQueueIsEmpty(fdcan));
  fdcan_tx_drain_active[fdcan] = false;
}

static uint32_t BSP_FDCAN_TxBufferMask(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == NULL) return 0u;
  uint32_t count = hfdcan->Init.TxFifoQueueElmtsNbr + hfdcan->Init.TxBuffersNbr;
  if (count >= 32u) return 0xFFFFFFFFu;
  if (count == 0u) return 0u;
  return (1u << count) - 1u;
}

static void BSP_FDCAN_AbortPendingTx(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == NULL || hfdcan->State != HAL_FDCAN_STATE_BUSY) return;
  const uint32_t mask = BSP_FDCAN_TxBufferMask(hfdcan);
  uint32_t pending = hfdcan->Instance->TXBRP & mask;
  if (pending == 0u) return;

  BSP_FDCAN_t fdcan = FDCAN_Get(hfdcan);
  if (fdcan != BSP_FDCAN_ERR) {
    ++g_bsp_fdcan_rx_debug.tx_abort_count[fdcan];
  }
  (void)HAL_FDCAN_AbortTxRequest(hfdcan, pending);
  for (uint32_t i = 0u; i < FDCAN_TX_ABORT_WAIT_COUNT; i++) {
    if ((hfdcan->Instance->TXBRP & pending) == 0u) break;
  }
}

static void BSP_FDCAN_ClearIrFlags(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == NULL) return;
  const uint32_t flags =
      FDCAN_FLAG_TX_COMPLETE |
      FDCAN_FLAG_TX_ABORT_COMPLETE |
      FDCAN_FLAG_TX_FIFO_EMPTY |
      FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST |
      FDCAN_FLAG_RX_FIFO0_FULL |
      FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE |
      FDCAN_FLAG_RX_FIFO1_MESSAGE_LOST |
      FDCAN_FLAG_RX_FIFO1_FULL |
      FDCAN_FLAG_RX_FIFO1_NEW_MESSAGE |
      FDCAN_FLAG_RAM_ACCESS_FAILURE |
      FDCAN_FLAG_ERROR_LOGGING_OVERFLOW |
      FDCAN_FLAG_ERROR_PASSIVE |
      FDCAN_FLAG_ERROR_WARNING |
      FDCAN_FLAG_BUS_OFF |
      FDCAN_FLAG_RAM_WATCHDOG |
      FDCAN_FLAG_ARB_PROTOCOL_ERROR |
      FDCAN_FLAG_DATA_PROTOCOL_ERROR |
      FDCAN_FLAG_RESERVED_ADDRESS_ACCESS;
  __HAL_FDCAN_CLEAR_FLAG(hfdcan, flags);
}

static uint32_t BSP_FDCAN_NotifyFlags(BSP_FDCAN_t fdcan) {
  switch (fdcan) {
#ifdef FDCAN1_EN
    case BSP_FDCAN_1:
      return FDCANx_NOTIFY_FLAGS(FDCAN1_RX_FIFO);
#endif
#ifdef FDCAN2_EN
    case BSP_FDCAN_2:
      return FDCANx_NOTIFY_FLAGS(FDCAN2_RX_FIFO);
#endif
#ifdef FDCAN3_EN
    case BSP_FDCAN_3:
      return FDCANx_NOTIFY_FLAGS(FDCAN3_RX_FIFO);
#endif
    default:
      return 0u;
  }
}

static bool BSP_FDCAN_ErrorNeedsRecovery(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == NULL) return false;
  const uint32_t recover_error_mask =
      HAL_FDCAN_ERROR_TIMEOUT |
      HAL_FDCAN_ERROR_NOT_INITIALIZED |
      HAL_FDCAN_ERROR_NOT_READY |
      HAL_FDCAN_ERROR_NOT_STARTED |
      HAL_FDCAN_ERROR_RAM_ACCESS |
      HAL_FDCAN_ERROR_RAM_WDG |
      HAL_FDCAN_ERROR_RESERVED_AREA;
  return ((hfdcan->ErrorCode & recover_error_mask) != 0u) ||
         ((hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0u);
}

static int8_t BSP_FDCAN_RecoverIfNeeded(BSP_FDCAN_t fdcan, bool force) {
  if (!inited) return BSP_ERR_INITED;
  if (fdcan >= BSP_FDCAN_NUM) return BSP_ERR;
  FDCAN_HandleTypeDef *hfdcan = BSP_FDCAN_GetHandle(fdcan);
  if (hfdcan == NULL) return BSP_ERR_NULL;

  FDCAN_ProtocolStatusTypeDef status = {0};
  FDCAN_ErrorCountersTypeDef counters = {0};
  if (HAL_FDCAN_GetProtocolStatus(hfdcan, &status) == HAL_OK) {
    g_bsp_fdcan_rx_debug.protocol_status[fdcan] = hfdcan->Instance->PSR;
  }
  if (HAL_FDCAN_GetErrorCounters(hfdcan, &counters) == HAL_OK) {
    g_bsp_fdcan_rx_debug.tx_error_count[fdcan] = counters.TxErrorCnt;
    g_bsp_fdcan_rx_debug.rx_error_count[fdcan] = counters.RxErrorCnt;
  }

  const uint32_t psr = hfdcan->Instance->PSR;
  const uint32_t cccr = hfdcan->Instance->CCCR;
  g_bsp_fdcan_rx_debug.protocol_status[fdcan] = psr;

  const bool need_recover = force || fdcan_recover_pending[fdcan] ||
                            (status.BusOff != 0u) ||
                            ((psr & FDCAN_PSR_BO) != 0u) ||
                            BSP_FDCAN_ErrorNeedsRecovery(hfdcan) ||
                            (hfdcan->State == HAL_FDCAN_STATE_READY) ||
                            (hfdcan->State == HAL_FDCAN_STATE_ERROR) ||
                            ((hfdcan->State == HAL_FDCAN_STATE_BUSY) &&
                             ((cccr & FDCAN_CCCR_INIT) != 0u));
  if (!need_recover) return BSP_OK;

  ++g_bsp_fdcan_rx_debug.recover_request_count[fdcan];
  fdcan_recover_pending[fdcan] = false;
  fdcan_tx_drain_active[fdcan] = false;
  BSP_FDCAN_SetTxFifoEmptyNotification(fdcan, false);
  BSP_FDCAN_TxQueueInit(fdcan);
  BSP_FDCAN_AbortPendingTx(hfdcan);
  hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;
  if (hfdcan->State == HAL_FDCAN_STATE_BUSY) {
    if (HAL_FDCAN_Stop(hfdcan) != HAL_OK &&
        hfdcan->State == HAL_FDCAN_STATE_ERROR) {
      hfdcan->State = HAL_FDCAN_STATE_READY;
      SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
      SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_CCE);
    }
  }
  if (hfdcan->State == HAL_FDCAN_STATE_ERROR) {
    hfdcan->State = HAL_FDCAN_STATE_READY;
    SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_CCE);
  }
  BSP_FDCAN_ClearIrFlags(hfdcan);
  (void)HAL_FDCAN_ActivateNotification(hfdcan, BSP_FDCAN_NotifyFlags(fdcan), 0);
  if (HAL_FDCAN_Start(hfdcan) == HAL_OK) {
    BSP_FDCAN_SetTxFifoEmptyNotification(fdcan, false);
    ++g_bsp_fdcan_rx_debug.recover_success_count[fdcan];
    return BSP_OK;
  }
  ++g_bsp_fdcan_rx_debug.recover_fail_count[fdcan];
  return BSP_ERR;
}

static void BSP_FDCAN_RxFifo0Callback(void) {
  const uint64_t start_us = BSP_TIME_Get_us();
  uint32_t total_handled = 0u;
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[BSP_FDCAN_MAX_DLC];
  for (int fdcan_idx = 0; fdcan_idx < BSP_FDCAN_NUM; fdcan_idx++) {
    FDCAN_HandleTypeDef *hfdcan = BSP_FDCAN_GetHandle((BSP_FDCAN_t)fdcan_idx);
    if (hfdcan == NULL) continue;
    uint32_t handled = 0u;
    while (handled < FDCAN_RX_MAX_FRAMES_PER_CALLBACK &&
           HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        ++handled;
        ++total_handled;
        uint32_t original_id = (rx_header.IdType == FDCAN_STANDARD_ID) ? rx_header.Identifier&0x7ff : rx_header.Identifier&0x1fffffff;
        BSP_FDCAN_FrameType_t frame_type = BSP_FDCAN_GetFrameType(&rx_header);
        uint32_t parsed_id = BSP_FDCAN_ParseId(original_id, frame_type);
        BSP_FDCAN_QueueNode_t *node = BSP_FDCAN_FindNode((BSP_FDCAN_t)fdcan_idx, parsed_id);
        if (node != NULL) {
          BSP_FDCAN_Message_t msg;
          msg.frame_type = frame_type;
          msg.original_id = original_id;
          msg.parsed_id = parsed_id;
          msg.timestamp = (uint32_t)(start_us / 1000ULL);
          uint8_t real_len = fdcan_dlc2len[rx_header.DataLength & 0xF];
          msg.dlc = real_len;
          if (msg.dlc > BSP_FDCAN_MAX_DLC) msg.dlc = BSP_FDCAN_MAX_DLC;
          memset(msg.data, 0, BSP_FDCAN_MAX_DLC);//现在是最大缓冲区写法所以全清零
          memcpy(msg.data, rx_data, msg.dlc);
          if (node->latest_only) {
            node->latest_msg = msg;
            node->latest_valid = true;
          } else if (node->queue != NULL) {
            osMessageQueuePut(node->queue, &msg, 0, 0);
          }
        }
      } else {
          break;
      }
    }
    if (handled >= FDCAN_RX_MAX_FRAMES_PER_CALLBACK &&
        HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
      ++g_bsp_fdcan_rx_debug.fifo0_limited_count;
    }
  }
  ++g_bsp_fdcan_rx_debug.fifo0_callback_count;
  g_bsp_fdcan_rx_debug.fifo0_frame_count += total_handled;
  g_bsp_fdcan_rx_debug.fifo0_last_frames = total_handled;
  if (total_handled > g_bsp_fdcan_rx_debug.fifo0_max_frames_per_callback) {
    g_bsp_fdcan_rx_debug.fifo0_max_frames_per_callback = total_handled;
  }
  const uint32_t elapsed_us = (uint32_t)(BSP_TIME_Get_us() - start_us);
  g_bsp_fdcan_rx_debug.fifo0_last_us = elapsed_us;
  if (elapsed_us > g_bsp_fdcan_rx_debug.fifo0_max_us) {
    g_bsp_fdcan_rx_debug.fifo0_max_us = elapsed_us;
  }
}

static void BSP_FDCAN_RxFifo1Callback(void) {
  const uint64_t start_us = BSP_TIME_Get_us();
  uint32_t total_handled = 0u;
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[BSP_FDCAN_MAX_DLC];
  for (int fdcan_idx = 0; fdcan_idx < BSP_FDCAN_NUM; fdcan_idx++) {
    FDCAN_HandleTypeDef *hfdcan = BSP_FDCAN_GetHandle((BSP_FDCAN_t)fdcan_idx);
    if (hfdcan == NULL) continue;
    uint32_t handled = 0u;
    while (handled < FDCAN_RX_MAX_FRAMES_PER_CALLBACK &&
           HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0) {
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK) {
        ++handled;
        ++total_handled;
        uint32_t original_id = (rx_header.IdType == FDCAN_STANDARD_ID) ? rx_header.Identifier&0x7ff : rx_header.Identifier&0x1fffffff;
        BSP_FDCAN_FrameType_t frame_type = BSP_FDCAN_GetFrameType(&rx_header);
        uint32_t parsed_id = BSP_FDCAN_ParseId(original_id, frame_type);
        BSP_FDCAN_QueueNode_t *node = BSP_FDCAN_FindNode((BSP_FDCAN_t)fdcan_idx, parsed_id);
        if (node != NULL) {
          BSP_FDCAN_Message_t msg;
          msg.frame_type = frame_type;
          msg.original_id = original_id;
          msg.parsed_id = parsed_id;
          msg.timestamp = (uint32_t)(start_us / 1000ULL);
          uint8_t real_len = fdcan_dlc2len[rx_header.DataLength & 0xF];
          msg.dlc = real_len; 
          if (msg.dlc > BSP_FDCAN_MAX_DLC) msg.dlc = BSP_FDCAN_MAX_DLC;
          memset(msg.data, 0, BSP_FDCAN_MAX_DLC);//现在是最大缓冲区写法所以全清零
          memcpy(msg.data, rx_data, msg.dlc);
          if (node->latest_only) {
            node->latest_msg = msg;
            node->latest_valid = true;
          } else if (node->queue != NULL) {
            osMessageQueuePut(node->queue, &msg, 0, 0);
          }
        }
      } else {
          break;
      }
    }
    if (handled >= FDCAN_RX_MAX_FRAMES_PER_CALLBACK &&
        HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0) {
      ++g_bsp_fdcan_rx_debug.fifo1_limited_count;
    }
  }
  ++g_bsp_fdcan_rx_debug.fifo1_callback_count;
  g_bsp_fdcan_rx_debug.fifo1_frame_count += total_handled;
  g_bsp_fdcan_rx_debug.fifo1_last_frames = total_handled;
  if (total_handled > g_bsp_fdcan_rx_debug.fifo1_max_frames_per_callback) {
    g_bsp_fdcan_rx_debug.fifo1_max_frames_per_callback = total_handled;
  }
  const uint32_t elapsed_us = (uint32_t)(BSP_TIME_Get_us() - start_us);
  g_bsp_fdcan_rx_debug.fifo1_last_us = elapsed_us;
  if (elapsed_us > g_bsp_fdcan_rx_debug.fifo1_max_us) {
    g_bsp_fdcan_rx_debug.fifo1_max_us = elapsed_us;
  }
}

/* HAL Callback Stubs (map HAL FDCAN callbacks to user callbacks) */
void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs) {
  (void)TxEventFifoITs;
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan != BSP_FDCAN_ERR) {
    BSP_FDCAN_TxQueueDrain(bsp_fdcan);
    if (FDCAN_Callback[bsp_fdcan][HAL_FDCAN_TX_EVENT_FIFO_CB])
      FDCAN_Callback[bsp_fdcan][HAL_FDCAN_TX_EVENT_FIFO_CB]();
  }
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan) {
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan != BSP_FDCAN_ERR) {
    BSP_FDCAN_TxQueueDrain(bsp_fdcan);
  }
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndex) {
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan != BSP_FDCAN_ERR) {
    if (FDCAN_Callback[bsp_fdcan][HAL_FDCAN_TX_BUFFER_COMPLETE_CB])
      FDCAN_Callback[bsp_fdcan][HAL_FDCAN_TX_BUFFER_COMPLETE_CB]();
  }
}

void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndex) {
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan != BSP_FDCAN_ERR) {
    if (FDCAN_Callback[bsp_fdcan][HAL_FDCAN_TX_BUFFER_ABORT_CB])
      FDCAN_Callback[bsp_fdcan][HAL_FDCAN_TX_BUFFER_ABORT_CB]();
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan != BSP_FDCAN_ERR) {
    if (FDCAN_Callback[bsp_fdcan][HAL_FDCAN_RX_FIFO0_MSG_PENDING_CB])
      FDCAN_Callback[bsp_fdcan][HAL_FDCAN_RX_FIFO0_MSG_PENDING_CB]();
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan != BSP_FDCAN_ERR) {
    if (FDCAN_Callback[bsp_fdcan][HAL_FDCAN_RX_FIFO1_MSG_PENDING_CB])
      FDCAN_Callback[bsp_fdcan][HAL_FDCAN_RX_FIFO1_MSG_PENDING_CB]();
  }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan != BSP_FDCAN_ERR) {
    ++g_bsp_fdcan_rx_debug.error_callback_count[bsp_fdcan];
    g_bsp_fdcan_rx_debug.last_error_code[bsp_fdcan] = hfdcan->ErrorCode;
    g_bsp_fdcan_rx_debug.protocol_status[bsp_fdcan] = hfdcan->Instance->PSR;
    if (BSP_FDCAN_ErrorNeedsRecovery(hfdcan)) {
      fdcan_recover_pending[bsp_fdcan] = true;
    } else {
      hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;
    }
    if (FDCAN_Callback[bsp_fdcan][HAL_FDCAN_ERROR_CB])
      FDCAN_Callback[bsp_fdcan][HAL_FDCAN_ERROR_CB]();
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan == BSP_FDCAN_ERR) return;
  ++g_bsp_fdcan_rx_debug.error_status_count[bsp_fdcan];
  g_bsp_fdcan_rx_debug.last_error_status_its[bsp_fdcan] = ErrorStatusITs;
  g_bsp_fdcan_rx_debug.protocol_status[bsp_fdcan] = hfdcan->Instance->PSR;
  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0u) {
    ++g_bsp_fdcan_rx_debug.bus_off_count[bsp_fdcan];
    fdcan_recover_pending[bsp_fdcan] = true;
  }
}

/* Exported functions ------------------------------------------------------- */
int8_t BSP_FDCAN_Init(void) {
  if (inited) return BSP_ERR_INITED;

  memset(FDCAN_Callback, 0, sizeof(FDCAN_Callback));
  for (int i = 0; i < BSP_FDCAN_NUM; i++) BSP_FDCAN_TxQueueInit((BSP_FDCAN_t)i);
  id_parser = BSP_FDCAN_DefaultIdParser;
  queue_mutex = osMutexNew(NULL);
  if (queue_mutex == NULL) return BSP_ERR;

  inited = true;

  /* 配置并启动 FDCAN 实例，绑定中断/回调 */

  //========== 过滤器配置说明：==========================
  //  过滤器编号：相对于每个（相当于经典can过滤器的bank）
  //  sFilterConfig.FilterIndex = 0 to 127(标准ID) or 0 to 63(扩展ID);  
  //  关于过滤器索引的说明：
  //  由stm32h7xx_hal_fdcan.c的第1874行代码可知滤波器地址计算方式如下：
  //  StandardFilterSA（字节） = SRAMCAN_BASE + (MessageRAMOffset * 4U)
  //  标准滤波器物理地址（字节） = StandardFilterSA + (FilterIndex * 4U)（每个标准滤波器占 4 字节 = 1 word,扩展的则是8个字节）
  //  
  //
  //  标识符类型:
  //  sFilterConfig.IdType = FDCAN_STANDARD_ID or FDCAN_EXTENDED_ID;
  //  过滤器类型: （仅介绍掩码模式）
  //  sFilterConfig.FilterType = FDCAN_FILTER_MASK;(掩码模式）
  //  过滤器配置:
  //  sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE; (禁用该过滤器条目)
  //                               FDCAN_FILTER_TO_RXFIFO0; (将匹配的消息放入 FIFO 0（普通优先级）)
  //                               FDCAN_FILTER_TO_RXFIFO1; (将匹配的消息放入 FIFO 1（高优先级）)
  //                               FDCAN_FILTER_TO_RXBUFFER; (将匹配的消息放入 指定的接收缓冲区)
  //                               FDCAN_FILTER_REJECT; (拒绝接收该标识符对应的报文)
  //                               FDCAN_FILTER_ACCEPT; (接受所有消息)
  //                               FDCAN_FILTER_HP            (过滤器匹配时，将报文标记为高优先级)                            
  //                               FDCAN_FILTER_TO_RXFIFO0_HP (过滤器匹配时，将报文标记为高优先级并存储至接收FIFO 0)          
  //                               FDCAN_FILTER_TO_RXFIFO1_HP (过滤器匹配时，将报文标记为高优先级并存储至接收FIFO 1)          
  //                               FDCAN_FILTER_TO_RXBUFFER   (将报文存储至接收缓冲区，过滤器类型（FilterType）配置项失效 )
  //  过滤器ID与掩码(FilterType掩码模式下)
  //  比较值（要匹配的 ID 的参考位）
  //  sFilterConfig.FilterID1 = 0 to 0x7FF; 标准ID
  //                            0 to 0x1FFFFFFF 扩展ID
  //  掩码（1=比较该位，0=忽略该位）
  //  sFilterConfig.FilterID2 = 0 to 0x7FF; 标准ID
  //                            0 to 0x1FFFFFFF 扩展ID
  //  接收缓冲区索引
  //  FilterConfig == FDCAN_FILTER_TO_RXBUFFER 时有效;必须小于RxBuffersNbr配置的实际Rx buffer数量
  //  sFilterConfig.RxBufferIndex = 0 to (RxBuffersNbr - 1);
  //  标记校准信息（用于 FDCAN 校准/时钟相关单元作特殊处理或统计）
  //  仅在FilterConfig 设为 FDCAN_FILTER_TO_RXBUFFER 时才有意义，通常设置为0
  //  IsCalibrationMsg = 0 or 1; 
  // fdcan_filter_table.h
  //=================================================================================
  /* 依据上述说明，配置过滤器并启动FDCAN */
  FDCAN_FilterTypeDef sFilterConfig;

#ifdef FDCAN1_EN
  #define hfdcan hfdcan1
  #define FDCANX_RX_FIFO FDCAN1_RX_FIFO
  FDCAN1_FILTER_CONFIG_TABLE(FDCAN_CONFIG_FILTER)
  #undef hfdcan
  #undef FDCANX_RX_FIFO
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN1_GLOBAL_FILTER);
  HAL_FDCAN_ActivateNotification(&hfdcan1, BSP_FDCAN_NotifyFlags(BSP_FDCAN_1), 0);
  BSP_FDCAN_RegisterCallback(BSP_FDCAN_1, FDCANX_MSG_PENDING_CB(FDCAN1_RX_FIFO), BSP_FDCAN_RxFifo0Callback);
  HAL_FDCAN_Start(&hfdcan1);
#endif

#ifdef FDCAN2_EN
  #define hfdcan hfdcan2
  #define FDCANX_RX_FIFO FDCAN2_RX_FIFO
  FDCAN2_FILTER_CONFIG_TABLE(FDCAN_CONFIG_FILTER)
  #undef hfdcan
  #undef FDCANX_RX_FIFO
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN2_GLOBAL_FILTER);
  HAL_FDCAN_ActivateNotification(&hfdcan2, BSP_FDCAN_NotifyFlags(BSP_FDCAN_2), 0);
  BSP_FDCAN_RegisterCallback(BSP_FDCAN_2, FDCANX_MSG_PENDING_CB(FDCAN2_RX_FIFO), BSP_FDCAN_RxFifo1Callback);
  HAL_FDCAN_Start(&hfdcan2);
#endif

#ifdef FDCAN3_EN
  #define hfdcan hfdcan3
  #define FDCANX_RX_FIFO FDCAN3_RX_FIFO
  FDCAN3_FILTER_CONFIG_TABLE(FDCAN_CONFIG_FILTER)
  #undef hfdcan
  #undef FDCANX_RX_FIFO
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN3_GLOBAL_FILTER);
  HAL_FDCAN_ActivateNotification(&hfdcan3, BSP_FDCAN_NotifyFlags(BSP_FDCAN_3), 0);
  BSP_FDCAN_RegisterCallback(BSP_FDCAN_3, FDCANX_MSG_PENDING_CB(FDCAN3_RX_FIFO), BSP_FDCAN_RxFifo1Callback);
  HAL_FDCAN_Start(&hfdcan3);
#endif

#undef FDCAN_FILTER_TO_RXFIFO_ENUM_INNER
#undef FDCAN_FILTER_TO_RXFIFO_ENUM
#undef FDCAN_CONFIG_FILTER 
#undef FDCAN_NOTIFY_FLAG_RXFIFO_INNER
#undef FDCAN_NOTIFY_FLAG_RXFIFO
#undef FDCAN_NOTIFY_FLAG_ERROR
#undef FDCANx_NOTIFY_FLAGS
#undef FDCANX_MSG_PENDING_CB_INNER
#undef FDCANX_MSG_PENDING_CB

  return BSP_OK;
}

FDCAN_HandleTypeDef *BSP_FDCAN_GetHandle(BSP_FDCAN_t fdcan) {
    if (fdcan >= BSP_FDCAN_NUM) return NULL;
    switch (fdcan) {
        /* AUTO GENERATED BSP_FDCAN_GET_HANDLE BEGIN */
        case BSP_FDCAN_1: return &hfdcan1;
        case BSP_FDCAN_2: return &hfdcan2;
        case BSP_FDCAN_3: return &hfdcan3;
        /* AUTO GENERATED BSP_FDCAN_GET_HANDLE END */
        default: return NULL;
    }
}

int8_t BSP_FDCAN_RegisterCallback(BSP_FDCAN_t fdcan, BSP_FDCAN_Callback_t type, void (*callback)(void)) {
    if (!inited) return BSP_ERR_INITED;
    if (callback == NULL) return BSP_ERR_NULL;
    if (fdcan >= BSP_FDCAN_NUM) return BSP_ERR;
    if (type >= HAL_FDCAN_CB_NUM) return BSP_ERR;
    FDCAN_Callback[fdcan][type] = callback;
    return BSP_OK;
}

int8_t BSP_FDCAN_Transmit(BSP_FDCAN_t fdcan, BSP_FDCAN_Format_t format, uint32_t id, uint8_t *data, uint8_t dlc) {
    if (!inited) return BSP_ERR_INITED;
    if (fdcan >= BSP_FDCAN_NUM) return BSP_ERR;
    if (data == NULL && format != BSP_FDCAN_FORMAT_STD_REMOTE && format != BSP_FDCAN_FORMAT_EXT_REMOTE) return BSP_ERR_NULL;
    if (dlc > BSP_FDCAN_MAX_DLC) return BSP_ERR;
    FDCAN_HandleTypeDef *hfdcan = BSP_FDCAN_GetHandle(fdcan);
    if (hfdcan == NULL) return BSP_ERR_NULL;
  (void)BSP_FDCAN_RecoverIfNeeded(fdcan, false);

  BSP_FDCAN_TxMessage_t tx_msg = {0};
  switch (format) {
      case BSP_FDCAN_FORMAT_STD_DATA:
          tx_msg.header.Identifier = id;
          tx_msg.header.IdType = FDCAN_STANDARD_ID;
          tx_msg.header.TxFrameType = FDCAN_DATA_FRAME;
          break;
      case BSP_FDCAN_FORMAT_EXT_DATA:
          tx_msg.header.Identifier = id;
          tx_msg.header.IdType = FDCAN_EXTENDED_ID;
          tx_msg.header.TxFrameType = FDCAN_DATA_FRAME;
          break;
      case BSP_FDCAN_FORMAT_STD_REMOTE:
          tx_msg.header.Identifier = id;
          tx_msg.header.IdType = FDCAN_STANDARD_ID;
          tx_msg.header.TxFrameType = FDCAN_REMOTE_FRAME;
          break;
      case BSP_FDCAN_FORMAT_EXT_REMOTE:
          tx_msg.header.Identifier = id;
          tx_msg.header.IdType = FDCAN_EXTENDED_ID;
          tx_msg.header.TxFrameType = FDCAN_REMOTE_FRAME;
          break;
      default:
          return BSP_ERR;
  }
  switch (hfdcan->Init.FrameFormat) {
    case FDCAN_FRAME_FD_BRS:
      tx_msg.header.BitRateSwitch = FDCAN_BRS_ON;
      tx_msg.header.FDFormat = FDCAN_FD_CAN;
      break;
    case FDCAN_FRAME_FD_NO_BRS:
      tx_msg.header.BitRateSwitch = FDCAN_BRS_OFF;
      tx_msg.header.FDFormat = FDCAN_FD_CAN;
      break;
    case FDCAN_FRAME_CLASSIC:
    default:
      tx_msg.header.BitRateSwitch = FDCAN_BRS_OFF;
      tx_msg.header.FDFormat = FDCAN_CLASSIC_CAN;
      break;
  }
	tx_msg.header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_msg.header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_msg.header.MessageMarker = 0x00;
  tx_msg.header.DataLength = BSP_FDCAN_EncodeDLC(dlc);   

  memset(tx_msg.data, 0, dlc);
  if (data != NULL && dlc > 0) {memcpy(tx_msg.data, data, dlc);}

  if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0) {
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_msg.header, tx_msg.data) ==
        HAL_OK) {
      ++g_bsp_fdcan_rx_debug.tx_direct_count[fdcan];
      return BSP_OK;
    }
    ++g_bsp_fdcan_rx_debug.tx_add_fail_count[fdcan];
    fdcan_recover_pending[fdcan] = true;
  }
  if (BSP_FDCAN_TxQueuePush(fdcan, &tx_msg)) {
    BSP_FDCAN_SetTxFifoEmptyNotification(fdcan, true);
    BSP_FDCAN_TxQueueDrain(fdcan);
    return BSP_OK;
  }
  ++g_bsp_fdcan_rx_debug.tx_queue_full_count[fdcan];
  return BSP_ERR;
}

int8_t BSP_FDCAN_TransmitStdDataFrame(BSP_FDCAN_t fdcan, BSP_FDCAN_StdDataFrame_t *frame) {
    if (frame == NULL) return BSP_ERR_NULL;
    return BSP_FDCAN_Transmit(fdcan, BSP_FDCAN_FORMAT_STD_DATA, frame->id, frame->data, frame->dlc);
}

int8_t BSP_FDCAN_TransmitExtDataFrame(BSP_FDCAN_t fdcan, BSP_FDCAN_ExtDataFrame_t *frame) {
    if (frame == NULL) return BSP_ERR_NULL;
    return BSP_FDCAN_Transmit(fdcan, BSP_FDCAN_FORMAT_EXT_DATA, frame->id, frame->data, frame->dlc);
}

int8_t BSP_FDCAN_TransmitRemoteFrame(BSP_FDCAN_t fdcan, BSP_FDCAN_RemoteFrame_t *frame) {
    if (frame == NULL) return BSP_ERR_NULL;
    BSP_FDCAN_Format_t format = frame->is_extended ? BSP_FDCAN_FORMAT_EXT_REMOTE : BSP_FDCAN_FORMAT_STD_REMOTE;
    return BSP_FDCAN_Transmit(fdcan, format, frame->id, NULL, frame->dlc);
}

int8_t BSP_FDCAN_RegisterId(BSP_FDCAN_t fdcan, uint32_t can_id, uint8_t queue_size) {
    if (!inited) return BSP_ERR_INITED;
    return BSP_FDCAN_CreateIdNode(fdcan, can_id, queue_size, false);
}

int8_t BSP_FDCAN_RegisterLatestId(BSP_FDCAN_t fdcan, uint32_t can_id) {
    if (!inited) return BSP_ERR_INITED;
    return BSP_FDCAN_CreateIdNode(fdcan, can_id, 1u, true);
}

int8_t BSP_FDCAN_GetMessage(BSP_FDCAN_t fdcan, uint32_t can_id, BSP_FDCAN_Message_t *msg, uint32_t timeout) {
    if (!inited) return BSP_ERR_INITED;
    if (msg == NULL) return BSP_ERR_NULL;
    (void)BSP_FDCAN_RecoverIfNeeded(fdcan, false);
    BSP_FDCAN_QueueNode_t *node = BSP_FDCAN_FindNodeNoWait(fdcan, can_id);
    osMessageQueueId_t queue = (node != NULL) ? node->queue : NULL;
    if (queue == NULL) return BSP_ERR_NO_DEV;
    osStatus_t res = osMessageQueueGet(queue, msg, NULL, timeout);
    return (res == osOK) ? BSP_OK : BSP_ERR;
}

int8_t BSP_FDCAN_GetLatestMessage(BSP_FDCAN_t fdcan, uint32_t can_id, BSP_FDCAN_Message_t *msg) {
    if (!inited) return BSP_ERR_INITED;
    if (msg == NULL) return BSP_ERR_NULL;
  (void)BSP_FDCAN_RecoverIfNeeded(fdcan, false);
    BSP_FDCAN_QueueNode_t *node = BSP_FDCAN_FindNodeNoWait(fdcan, can_id);
    if (node == NULL) return BSP_ERR_NO_DEV;
    if (node->latest_only) {
        if (!node->latest_valid) return BSP_ERR;
        *msg = node->latest_msg;
        return BSP_OK;
    }
    osMessageQueueId_t queue = node->queue;
    if (queue == NULL) return BSP_ERR_NO_DEV;

    BSP_FDCAN_Message_t latest;
    bool has_latest = false;
    while (osMessageQueueGet(queue, &latest, NULL, BSP_FDCAN_TIMEOUT_IMMEDIATE) == osOK) {
        has_latest = true;
    }
    if (!has_latest) return BSP_ERR;
    *msg = latest;
    return BSP_OK;
}

  int8_t BSP_FDCAN_Recover(BSP_FDCAN_t fdcan) {
    return BSP_FDCAN_RecoverIfNeeded(fdcan, true);
  }

int32_t BSP_FDCAN_GetQueueCount(BSP_FDCAN_t fdcan, uint32_t can_id) {
    if (!inited) return -1;
    BSP_FDCAN_QueueNode_t *node = BSP_FDCAN_FindNodeNoWait(fdcan, can_id);
    osMessageQueueId_t queue = (node != NULL) ? node->queue : NULL;
    if (queue == NULL) return -1;
    return (int32_t)osMessageQueueGetCount(queue);
}

int8_t BSP_FDCAN_FlushQueue(BSP_FDCAN_t fdcan, uint32_t can_id) {
    if (!inited) return BSP_ERR_INITED;
    BSP_FDCAN_QueueNode_t *node = BSP_FDCAN_FindNodeNoWait(fdcan, can_id);
    if (node != NULL && node->latest_only) {
        node->latest_valid = false;
        return BSP_OK;
    }
    osMessageQueueId_t queue = (node != NULL) ? node->queue : NULL;
    if (queue == NULL) return BSP_ERR_NO_DEV;
    BSP_FDCAN_Message_t tmp;
    while (osMessageQueueGet(queue, &tmp, NULL, BSP_FDCAN_TIMEOUT_IMMEDIATE) == osOK) { }
    return BSP_OK;
}

int8_t BSP_FDCAN_RegisterIdParser(BSP_FDCAN_IdParser_t parser) {
    if (!inited) return BSP_ERR_INITED;
    if (parser == NULL) return BSP_ERR_NULL;
    id_parser = parser;
    return BSP_OK;
}

uint32_t BSP_FDCAN_ParseId(uint32_t original_id, BSP_FDCAN_FrameType_t frame_type) {
    if (id_parser != NULL) return id_parser(original_id, frame_type);
    return BSP_FDCAN_DefaultIdParser(original_id, frame_type);
}
/* */

