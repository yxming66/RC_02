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
#define SICK_CAN_BUS BSP_CAN_2
#endif

/* SICK sampling board sends 4 ADC channels per CAN frame.
 * Current mechanism uses 3 photoelectric sensors:
 * [0] bottom photoelectric
 * [1] rod-side photoelectric
 * [2] front photoelectric
 * [3] unused
 */
#ifndef SICK_BOTTOM_PHOTO_INDEX
#define SICK_BOTTOM_PHOTO_INDEX (0u)
#endif

#ifndef SICK_ROD_SIDE_PHOTO_INDEX
#define SICK_ROD_SIDE_PHOTO_INDEX (1u)
#endif

#ifndef SICK_UNUSED_PHOTO_INDEX
#define SICK_UNUSED_PHOTO_INDEX (3u)
#endif

#ifndef SICK_FRONT_PHOTO_INDEX
#define SICK_FRONT_PHOTO_INDEX (2u)
#endif

#ifndef SICK_FRONT_INDEX
#define SICK_FRONT_INDEX SICK_FRONT_PHOTO_INDEX
#endif

#ifndef SICK_ROD_FRONT_INDEX
#define SICK_ROD_FRONT_INDEX SICK_ROD_SIDE_PHOTO_INDEX
#endif

#ifndef SICK_REAR_INDEX
#define SICK_REAR_INDEX SICK_BOTTOM_PHOTO_INDEX
#endif

#ifndef SICK_ROD_REAR_INDEX
#define SICK_ROD_REAR_INDEX SICK_ROD_SIDE_PHOTO_INDEX
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

typedef struct {
    uint8_t channel_index;
    bool sample_valid;
    uint16_t adc_raw;
    float distance_mm;
    uint32_t update_tick;
} Sick_FrontOreDetect_t;

/* Exported functions prototypes -------------------------------------------- */
int8_t SICK_Init(void);
void SICK_Update(uint32_t now_ms);
bool SICK_GetLatestOutput(Sick_Output_t *output);
bool SICK_GetFrontOreDetect(Sick_FrontOreDetect_t *detect);

#ifdef __cplusplus
}
#endif
