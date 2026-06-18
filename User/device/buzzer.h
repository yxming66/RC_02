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
#include <stddef.h>
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

typedef struct {
  const Tone_t *melody;
  size_t melody_length;
  uint16_t tone_gap_ms;
} Buzzer_Score_t;

typedef struct {
  const Tone_t *melody;
  size_t melody_length;
  size_t tone_index;
  uint32_t next_tick;
  uint16_t tone_gap_ms;
  bool active;
  bool paused;
  bool loop;
  bool waiting_tone;
  bool waiting_gap;
} Buzzer_Player_t;

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
int8_t BUZZER_PlayScore(BUZZER_t *buzzer, const Buzzer_Score_t *score);
int8_t BUZZER_PlayerStart(Buzzer_Player_t *player, BUZZER_t *buzzer,
                          const Buzzer_Score_t *score, bool loop,
                          uint32_t now_tick);
int8_t BUZZER_PlayerUpdate(Buzzer_Player_t *player, BUZZER_t *buzzer,
                           uint32_t now_tick);
void BUZZER_PlayerStop(Buzzer_Player_t *player, BUZZER_t *buzzer);
void BUZZER_PlayerSilence(Buzzer_Player_t *player, BUZZER_t *buzzer);
void BUZZER_PlayerSetPaused(Buzzer_Player_t *player, BUZZER_t *buzzer,
                            bool paused, uint32_t now_tick);
bool BUZZER_PlayerIsActive(const Buzzer_Player_t *player);
bool BUZZER_PlayerIsPaused(const Buzzer_Player_t *player);

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */

#ifdef __cplusplus
}
#endif
