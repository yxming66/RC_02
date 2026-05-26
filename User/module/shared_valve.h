#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "bsp/gpio.h"

typedef enum {
  SHARED_VALVE_SOURCE_ORE_STORE = 0,
  SHARED_VALVE_SOURCE_ROD,
  SHARED_VALVE_SOURCE_NUM,
} SharedValve_Source_t;

typedef struct {
  volatile bool source_request[SHARED_VALVE_SOURCE_NUM];
  volatile bool output_state;
  volatile uint32_t conflict_count;
  volatile uint32_t write_count;
  volatile BSP_GPIO_t gpio;
} SharedValve_Debug_t;

void SharedValve_Init(BSP_GPIO_t gpio);
void SharedValve_SetRequest(SharedValve_Source_t source, bool request);
void SharedValve_Output(void);
void SharedValve_SetOreStoreRequest(bool closed);
void SharedValve_SetRodRequest(bool grab);
const SharedValve_Debug_t *SharedValve_GetDebug(void);

#ifdef __cplusplus
}
#endif
