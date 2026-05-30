#include "sick.h"

#include <string.h>

#include "cmsis_os2.h"

/* Private define ----------------------------------------------------------- */
#define SICK_ADC_MIN (700.0)
#define SICK_ADC_MAX (32100.0)
#define SICK_DISTANCE_MIN_M (0.000)
#define SICK_DISTANCE_MAX_M (7.190)
#define SICK_CAN_DATA_LEN (8u)

/* Private variables -------------------------------------------------------- */
static uint16_t sick_adc_raw[SICK_OUTPUT_CHANNEL_COUNT] = {0u, 0u, 0u, 0u};
static float sick_distance_m[SICK_OUTPUT_CHANNEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static uint16_t sick_miss_count = 0u;
static BSP_CAN_Message_t sick_msg;
static bool sick_can_available = false;
static bool sick_inited = false;
static osMutexId_t sick_mutex = NULL;
static Sick_Output_t sick_latest_output = {0};
static const uint32_t sick_can_id = SICK_CAN_ID;

/* Private function --------------------------------------------------------- */
static double SICK_MapAdcToDistanceM(uint16_t adc_raw_value) {
  const double adc_span = SICK_ADC_MAX - SICK_ADC_MIN;
  const double distance_span = SICK_DISTANCE_MAX_M - SICK_DISTANCE_MIN_M;
  double norm = 0.0;

  if (adc_span > 0.0) {
    norm = ((double)adc_raw_value - SICK_ADC_MIN) / adc_span;
  }

  if (norm < 0.0) {
    norm = 0.0;
  } else if (norm > 1.0) {
    norm = 1.0;
  }

  if (distance_span <= 0.0) {
    return SICK_DISTANCE_MIN_M;
  }

  return SICK_DISTANCE_MIN_M + norm * distance_span;
}

static uint16_t SICK_ParseAdcRaw16(const uint8_t *data, uint8_t index) {
  const uint8_t offset = (uint8_t)(index * 2u);
  return ((uint16_t)data[offset] << 8) | (uint16_t)data[offset + 1u];
}

static void SICK_SetAllDistanceInvalid(void) {
  for (uint8_t i = 0u; i < SICK_OUTPUT_CHANNEL_COUNT; i++) {
    sick_distance_m[i] = -1.0f;
  }
}

static double SICK_ApplyChannelCalibrationM(uint8_t index, double distance_m_raw) {
#if SICK_FRONT_S2_COMP_ENABLE
  if (index == SICK_FRONT_S2_INDEX) {
    const double corrected_m = distance_m_raw - (SICK_FRONT_S2_REF_M - SICK_FRONT_S1_REF_M);
    return (corrected_m < SICK_DISTANCE_MIN_M) ? SICK_DISTANCE_MIN_M : corrected_m;
  }
#endif

  (void)index;
  return distance_m_raw;
}

static void SICK_PublishLatestOutput(uint32_t now_ms) {
  Sick_Output_t output = {0};

  for (uint8_t i = 0u; i < SICK_OUTPUT_CHANNEL_COUNT; i++) {
    output.adc_raw[i] = sick_adc_raw[i];
    output.distance_m[i] = sick_distance_m[i];
    output.distance_mm[i] = sick_distance_m[i] * 1000.0f;
    output.valid[i] = sick_distance_m[i] >= 0.0f;
  }

  output.miss_count = sick_miss_count;
  output.update_tick = now_ms;

  if (sick_mutex != NULL && osMutexAcquire(sick_mutex, osWaitForever) == osOK) {
    sick_latest_output = output;
    osMutexRelease(sick_mutex);
  }
}

static void SICK_UpdateAdcRaw(void) {
  bool updated = false;

  if (!sick_can_available) {
    if (sick_miss_count < 0xFFFFu) {
      sick_miss_count++;
    }
    return;
  }

  while (BSP_CAN_GetMessage(SICK_CAN_BUS, sick_can_id, &sick_msg, BSP_CAN_TIMEOUT_IMMEDIATE) ==
         BSP_OK) {
    if (sick_msg.dlc < SICK_CAN_DATA_LEN) {
      continue;
    }

    for (uint8_t i = 0u; i < SICK_OUTPUT_CHANNEL_COUNT; i++) {
      sick_adc_raw[i] = SICK_ParseAdcRaw16(sick_msg.data, i);
    }
    updated = true;
  }

  if (updated) {
    sick_miss_count = 0u;
    return;
  }

  if (sick_miss_count < 0xFFFFu) {
    sick_miss_count++;
  }
}

static void SICK_UpdateDistance(void) {
  if (sick_miss_count >= SICK_MISS_THRESHOLD) {
    SICK_SetAllDistanceInvalid();
    return;
  }

  for (uint8_t i = 0u; i < SICK_OUTPUT_CHANNEL_COUNT; i++) {
    const double mapped_m = SICK_MapAdcToDistanceM(sick_adc_raw[i]);
    const double calibrated_m = SICK_ApplyChannelCalibrationM(i, mapped_m);
    sick_distance_m[i] = (float)calibrated_m;
  }
}

/* Exported functions ------------------------------------------------------- */
int8_t SICK_Init(void) {
  if (sick_inited) {
    return DEVICE_ERR_INITED;
  }

  sick_mutex = osMutexNew(NULL);
  if (sick_mutex == NULL) {
    return DEVICE_ERR;
  }

  const int8_t can_init_ret = BSP_CAN_Init();
  if (can_init_ret != BSP_OK && can_init_ret != BSP_ERR_INITED) {
    return DEVICE_ERR;
  }

  const int8_t reg_ret = BSP_CAN_RegisterId(SICK_CAN_BUS, sick_can_id, SICK_CAN_QUEUE_SIZE);
  if (reg_ret == BSP_OK || BSP_CAN_GetQueueCount(SICK_CAN_BUS, sick_can_id) >= 0) {
    sick_can_available = true;
  }

  if (!sick_can_available) {
    return DEVICE_ERR;
  }

  sick_inited = true;
  memset(sick_adc_raw, 0, sizeof(sick_adc_raw));
  memset(sick_distance_m, 0, sizeof(sick_distance_m));
  memset(&sick_latest_output, 0, sizeof(sick_latest_output));
  sick_miss_count = 0u;
  return DEVICE_OK;
}

void SICK_Update(uint32_t now_ms) {
  SICK_UpdateAdcRaw();
  SICK_UpdateDistance();
  SICK_PublishLatestOutput(now_ms);
}

bool SICK_GetLatestOutput(Sick_Output_t *output) {
  if (output == NULL) {
    return false;
  }

  if (sick_mutex == NULL) {
    *output = sick_latest_output;
    return true;
  }

  if (osMutexAcquire(sick_mutex, 0u) == osOK) {
    *output = sick_latest_output;
    osMutexRelease(sick_mutex);
    return true;
  }

  return false;
}