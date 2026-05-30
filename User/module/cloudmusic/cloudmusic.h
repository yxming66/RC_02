#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "device/buzzer.h"
#include "device/dr16.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CLOUDMUSIC_FEATURE_NONE (0U)
#define CLOUDMUSIC_FEATURE_STARTUP_MUSIC (1U << 0)
#define CLOUDMUSIC_FEATURE_BREATH_RESPONSE (1U << 1)
#define CLOUDMUSIC_FEATURE_MUSIC_LOOP (1U << 2)

#ifndef CLOUDMUSIC_FEATURES
#ifdef BLINK_AUDIO_FEATURES
#define CLOUDMUSIC_FEATURES BLINK_AUDIO_FEATURES
#else
#define CLOUDMUSIC_FEATURES CLOUDMUSIC_FEATURE_NONE
#endif
#endif

#ifndef CLOUDMUSIC_STARTUP_MUSIC
#ifdef BLINK_STARTUP_MUSIC
#define CLOUDMUSIC_STARTUP_MUSIC BLINK_STARTUP_MUSIC
#else
#define CLOUDMUSIC_STARTUP_MUSIC MUSIC_NOKIA
#endif
#endif

#ifndef CLOUDMUSIC_SWITCH_DOUBLE_TRIGGER_MS
#define CLOUDMUSIC_SWITCH_DOUBLE_TRIGGER_MS 500U
#endif

typedef struct {
  bool rc_online;
  DR16_SwitchPos_t sw_l;
  DR16_SwitchPos_t sw_r;
} CloudMusic_Input_t;

typedef struct {
  bool next_event;
  bool previous_event;
  bool toggle_pause_event;
  bool play_event;
  bool stop_event;
} CloudMusic_Control_t;

typedef struct {
  const MUSIC_t *playlist;
  size_t playlist_length;
  MUSIC_t startup_music;
  uint32_t switch_double_trigger_ms;
  bool enable_startup_music;
  bool enable_music_loop;
} CloudMusic_Config_t;

typedef struct {
  bool enabled;
  bool active;
  bool paused;
  bool startup_active;
  bool music_loop_enabled;
  bool switch_pending;
  size_t playlist_index;
  size_t playlist_length;
  MUSIC_t current_music;
  uint32_t next_count;
  uint32_t previous_count;
  uint32_t pause_toggle_count;
  uint32_t play_count;
  uint32_t stop_count;
} CloudMusic_Status_t;

typedef struct {
  BUZZER_t *buzzer;
  BUZZER_MusicPlayer_t player;
  const MUSIC_t *playlist;
  size_t playlist_length;
  size_t playlist_index;
  MUSIC_t startup_music;
  uint32_t switch_double_trigger_ms;
  bool enable_startup_music;
  bool enable_music_loop;
  bool startup_active;
  bool user_paused;
  bool switch_pending;
  uint32_t switch_pending_tick;
  DR16_SwitchPos_t last_sw_r;
  uint32_t next_count;
  uint32_t previous_count;
  uint32_t pause_toggle_count;
  uint32_t play_count;
  uint32_t stop_count;
} CloudMusic_t;

void CloudMusic_GetDefaultConfig(CloudMusic_Config_t *config);
int8_t CloudMusic_Init(CloudMusic_t *cloudmusic, BUZZER_t *buzzer,
                       const CloudMusic_Config_t *config);
int8_t CloudMusic_Start(CloudMusic_t *cloudmusic, uint32_t now_tick);
int8_t CloudMusic_Update(CloudMusic_t *cloudmusic,
                         const CloudMusic_Input_t *input,
                         uint32_t now_tick);
int8_t CloudMusic_ApplyControl(CloudMusic_t *cloudmusic,
                               const CloudMusic_Control_t *control,
                               uint32_t now_tick);
int8_t CloudMusic_Next(CloudMusic_t *cloudmusic, uint32_t now_tick);
int8_t CloudMusic_Previous(CloudMusic_t *cloudmusic, uint32_t now_tick);
void CloudMusic_SetPaused(CloudMusic_t *cloudmusic, bool paused,
                          uint32_t now_tick);
void CloudMusic_TogglePaused(CloudMusic_t *cloudmusic, uint32_t now_tick);
void CloudMusic_Stop(CloudMusic_t *cloudmusic);
void CloudMusic_Silence(CloudMusic_t *cloudmusic);
void CloudMusic_ResetGesture(CloudMusic_t *cloudmusic,
                             const CloudMusic_Input_t *input);
void CloudMusic_GetStatus(const CloudMusic_t *cloudmusic,
                          CloudMusic_Status_t *status);
bool CloudMusic_IsEnabled(const CloudMusic_t *cloudmusic);
bool CloudMusic_IsActive(const CloudMusic_t *cloudmusic);
bool CloudMusic_IsPaused(const CloudMusic_t *cloudmusic);
bool CloudMusic_AllowsIdleEffect(const CloudMusic_t *cloudmusic);

#ifdef __cplusplus
}
#endif
