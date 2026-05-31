#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>

#include "bsp/can.h"
#include "device.h"

/* USER DEFINE BEGIN */

#ifndef SICK_OUTPUT_CHANNEL_COUNT
#define SICK_OUTPUT_CHANNEL_COUNT (4u)
#endif

#ifndef SICK_MISS_THRESHOLD
#define SICK_MISS_THRESHOLD (2u)
#endif

#ifndef SICK_CAN_QUEUE_SIZE
#define SICK_CAN_QUEUE_SIZE (20u)
#endif

#ifndef SICK_CAN_ID
#define SICK_CAN_ID (0x121u)
#endif

#ifndef SICK_CAN_BUS
#define SICK_CAN_BUS BSP_CAN_3
#endif

#ifndef SICK_FRONT_S1_INDEX
#define SICK_FRONT_S1_INDEX (2u)
#endif

#ifndef SICK_FRONT_S2_INDEX
#define SICK_FRONT_S2_INDEX (3u)
#endif

#ifndef SICK_FRONT_S1_REF_M
#define SICK_FRONT_S1_REF_M (0.239326894)
#endif

#ifndef SICK_FRONT_S2_REF_M
#define SICK_FRONT_S2_REF_M (0.265225768)
#endif

#ifndef SICK_FRONT_S2_COMP_ENABLE
#define SICK_FRONT_S2_COMP_ENABLE (1u)
#endif

#ifndef SICK_CHANNEL_2_ADC_OFFSET
#define SICK_CHANNEL_2_ADC_OFFSET (20u)
#endif

#ifndef SICK_CHANNEL_3_ADC_OFFSET
#define SICK_CHANNEL_3_ADC_OFFSET (20u)
#endif

#ifndef SICK_CHANNEL_4_ADC_OFFSET
#define SICK_CHANNEL_4_ADC_OFFSET (0u)
#endif

/* USER DEFINE END */

/* Exported types ----------------------------------------------------------- */
typedef struct {
    uint16_t adc_raw[SICK_OUTPUT_CHANNEL_COUNT];
    float distance_mm[SICK_OUTPUT_CHANNEL_COUNT];
    float distance_m[SICK_OUTPUT_CHANNEL_COUNT];
    bool valid[SICK_OUTPUT_CHANNEL_COUNT];
    uint16_t miss_count;
    uint32_t update_tick;
} Sick_Output_t;

/* Exported functions prototypes -------------------------------------------- */
int8_t SICK_Init(void);
void SICK_Update(uint32_t now_ms);
bool SICK_GetLatestOutput(Sick_Output_t *output);

#ifdef __cplusplus
}
#endif