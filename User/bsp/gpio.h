#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>

#include "bsp/bsp.h"

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */

/* Exported constants ------------------------------------------------------- */
/* Exported macro ----------------------------------------------------------- */
/* USER DEFINE BEGIN */

/* USER DEFINE END */

/* Exported types ----------------------------------------------------------- */
typedef enum {
  BSP_GPIO_ACCL_CS,
  BSP_GPIO_POWER_24V_2,
  BSP_GPIO_POWER_24V_1,
  BSP_GPIO_POWER_5V,
  BSP_GPIO_GYRO_CS,
  BSP_GPIO_USER_KEY,
  BSP_GPIO_ACCL_INT,
  BSP_GPIO_GYRO_INT,
  BSP_GPIO_ARM_SOLENOID,    
  BSP_GPIO_ROD_SOLENOID,      
  BSP_GPIO_NUM,
  BSP_GPIO_ERR,
} BSP_GPIO_t;

/* Exported functions prototypes -------------------------------------------- */
int8_t BSP_GPIO_RegisterCallback(BSP_GPIO_t gpio, void (*callback)(void));

int8_t BSP_GPIO_EnableIRQ(BSP_GPIO_t gpio);
int8_t BSP_GPIO_DisableIRQ(BSP_GPIO_t gpio);

int8_t BSP_GPIO_WritePin(BSP_GPIO_t gpio, bool value);
int8_t BSP_GPIO_TogglePin(BSP_GPIO_t gpio);

bool BSP_GPIO_ReadPin(BSP_GPIO_t gpio);

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */

#ifdef __cplusplus
}
#endif