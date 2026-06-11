/**
 * @file buzzer.h
 * @brief Buzzer PWM hardware driver.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp/pwm.h"
#include "device.h"

#include <stdbool.h>
#include <stdint.h>

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */

/* Exported constants ------------------------------------------------------- */

/* USER DEFINE BEGIN */

/* USER DEFINE END */

/* Exported types ----------------------------------------------------------- */

typedef struct {
  DEVICE_Header_t header;
  BSP_PWM_Channel_t channel;
} BUZZER_t;

typedef enum {
  NOTE_C = 0,
  NOTE_CS = 1,
  NOTE_D = 2,
  NOTE_DS = 3,
  NOTE_E = 4,
  NOTE_F = 5,
  NOTE_FS = 6,
  NOTE_G = 7,
  NOTE_GS = 8,
  NOTE_A = 9,
  NOTE_AS = 10,
  NOTE_B = 11,
  NOTE_REST = 255
} NOTE_t;

typedef struct {
  NOTE_t note;
  uint8_t octave;
  uint16_t duration_ms;
} Tone_t;

/* USER STRUCT BEGIN */

/* USER STRUCT END */

/* Exported functions prototypes -------------------------------------------- */

int8_t BUZZER_Init(BUZZER_t *buzzer, BSP_PWM_Channel_t channel);
int8_t BUZZER_Start(BUZZER_t *buzzer);
int8_t BUZZER_Stop(BUZZER_t *buzzer);
int8_t BUZZER_Set(BUZZER_t *buzzer, float freq, float duty_cycle);
float BUZZER_CalcFreq(NOTE_t note, uint8_t octave);
int8_t BUZZER_ApplyTone(BUZZER_t *buzzer, NOTE_t note, uint8_t octave);
int8_t BUZZER_PlayTone(BUZZER_t *buzzer, NOTE_t note, uint8_t octave,
                       uint16_t duration_ms, uint16_t gap_ms);

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */

#ifdef __cplusplus
}
#endif
