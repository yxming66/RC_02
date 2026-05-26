/*
    blink Task
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "bsp/pwm.h"
#include "device/buzzer.h"
#include "device/dr16.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
#define BLINK_AUDIO_FEATURE_NONE (0U)
#define BLINK_AUDIO_FEATURE_STARTUP_MUSIC (1U << 0)
#define BLINK_AUDIO_FEATURE_BREATH_RESPONSE (1U << 1)
#define BLINK_AUDIO_FEATURE_MUSIC_LOOP (1U << 2)

/*
 * Quick audio feature config:
 * - STARTUP_MUSIC: play BLINK_STARTUP_MUSIC once after boot.
 * - BREATH_RESPONSE: idle buzzer breath response when music loop is disabled.
 * - MUSIC_LOOP: keep cycling the playlist after startup.
 */
// #ifndef BLINK_AUDIO_FEATURES
// #define BLINK_AUDIO_FEATURES \
//   (BLINK_AUDIO_FEATURE_STARTUP_MUSIC | BLINK_AUDIO_FEATURE_MUSIC_LOOP)
// #endif
// BLINK_AUDIO_FEATURE_STARTUP_MUSIC
// BLINK_AUDIO_FEATURE_MUSIC_LOOP
// BLINK_AUDIO_FEATURE_BREATH_RESPONSE

#if ((BLINK_AUDIO_FEATURES &                                               \
      ~(BLINK_AUDIO_FEATURE_STARTUP_MUSIC |                                \
        BLINK_AUDIO_FEATURE_BREATH_RESPONSE | BLINK_AUDIO_FEATURE_MUSIC_LOOP)) != 0U)
#error "BLINK_AUDIO_FEATURES contains unsupported feature bits"
#endif

#define BLINK_USE_STARTUP_MUSIC \
  ((BLINK_AUDIO_FEATURES & BLINK_AUDIO_FEATURE_STARTUP_MUSIC) != 0U)
#define BLINK_USE_BREATH_RESPONSE \
  ((BLINK_AUDIO_FEATURES & BLINK_AUDIO_FEATURE_BREATH_RESPONSE) != 0U)
#define BLINK_USE_MUSIC_LOOP \
  ((BLINK_AUDIO_FEATURES & BLINK_AUDIO_FEATURE_MUSIC_LOOP) != 0U)

#ifndef BLINK_STARTUP_MUSIC
#define BLINK_STARTUP_MUSIC MUSIC_NOKIA
#endif

#define MUSIC_SWITCH_DOUBLE_TRIGGER_MS 500U
#define BREATH_RESPONSE_PERIOD_MS 3000U
#define BREATH_RESPONSE_ON_MS 900U
#define BREATH_RESPONSE_STEP_MS 50U
#define BREATH_RESPONSE_FREQ_LOW_HZ 392.0f
#define BREATH_RESPONSE_FREQ_HIGH_HZ 659.25f
#define BREATH_RESPONSE_VOLUME 0.12f

#define TEMP_ALARM_TONE_LOW_HZ 800.0f
#define TEMP_ALARM_TONE_HIGH_HZ 1000.0f
#define TEMP_ALARM_DUTY 0.50f
#define TEMP_ALARM_TONE_MS 80U

#ifndef TEMP_ALARM_REQUEST_TIMEOUT_MS
#define TEMP_ALARM_REQUEST_TIMEOUT_MS 250U
#endif

/* Private macro ------------------------------------------------------------ */
#define ARRAY_LEN(arr) (sizeof(arr) / sizeof((arr)[0]))

/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
BUZZER_t buzzer;
extern DR16_t dr16;
bool reset = 0;

#if BLINK_USE_MUSIC_LOOP
static const MUSIC_t buzzer_playlist[] = {
  MUSIC_HONG_DOU,
  MUSIC_SUGAR_PLUM_FAIRY,
  MUSIC_HAO_YUN_LAI,
  MUSIC_JIAO_HUAN_YU_SHENG,
  MUSIC_STARTUP_BUMBLEBEE,
  MUSIC_CROATIAN_RHAPSODY,
  MUSIC_JIAO_HUAN_YU_SHENG,
  MUSIC_MOONLIGHT_SONATA,
  MUSIC_MOONLIGHT_SONATA_2ND,
  MUSIC_MOONLIGHT_SONATA_3RD,
  MUSIC_FUR_ELISE,
  MUSIC_MERRY_CHRISTMAS_MR_LAWRENCE,
  MUSIC_HAPPY_DOU_DI_ZHU_MELODY,
  MUSIC_YI_BU_ZHI_YAO,
  MUSIC_RENAI_CIRCULATION,
  MUSIC_FUYU_NO_HANA,
  MUSIC_HAPPY_BIRTHDAY,
  MUSIC_SENBONZAKURA,
  MUSIC_ER_QUAN_YING_YUE,
    MUSIC_GU_SHI_XI_NI,
    MUSIC_YUAN_YU_CHOU,
    MUSIC_TORI_NO_UTA,
    MUSIC_GU_SHI_XI_NI,
    MUSIC_FLOWER_DANCE,
    MUSIC_SHUN,
    MUSIC_JUE_BIE_SHU,
};
#endif

static BUZZER_MusicPlayer_t music_player;
static Buzzer_AlarmLevel_t temp_alarm_level = BUZZER_ALARM_NONE;
static uint32_t temp_alarm_start_tick;
static uint32_t temp_alarm_last_toggle_tick;
static uint32_t temp_alarm_min_duration_ticks;
static bool temp_alarm_tone_high;
#if BLINK_USE_MUSIC_LOOP
static size_t music_playlist_index;
static bool music_user_paused;
static bool music_switch_pending;
static uint32_t music_switch_pending_tick;
static DR16_SwitchPos_t music_last_sw_r = DR16_SW_ERR;
#endif
#if BLINK_USE_STARTUP_MUSIC
static bool music_startup_jingle_active;
#endif

#if BLINK_USE_BREATH_RESPONSE
static bool breath_response_active;
static uint32_t breath_response_last_tick;
static uint32_t breath_response_start_tick;
static uint32_t breath_response_next_update_tick;
#endif
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
static uint32_t Blink_MsToTicks(uint32_t ms) {
  uint32_t tick_freq = osKernelGetTickFreq();
  uint64_t ticks = ((uint64_t)ms * tick_freq + 999ULL) / 1000ULL;
  if (ticks == 0ULL) {
    ticks = 1ULL;
  }
  if (ticks > UINT32_MAX) {
    ticks = UINT32_MAX;
  }
  return (uint32_t)ticks;
}

static bool Blink_TickReached(uint32_t now_tick, uint32_t target_tick) {
  return (int32_t)(now_tick - target_tick) >= 0;
}

#if BLINK_USE_BREATH_RESPONSE
static void Blink_BreathResponseStop(void);
#endif

#if BLINK_USE_MUSIC_LOOP
static void Blink_ResetMusicSwitchGesture(void);
#endif

static bool Blink_TempAlarmRequestActive(uint32_t now_tick) {
  const Buzzer_AlarmLevel_t request_level = g_buzzer_alarm_request.level;
  if (request_level == BUZZER_ALARM_NONE) {
    return false;
  }

  const uint32_t timeout_ticks = Blink_MsToTicks(TEMP_ALARM_REQUEST_TIMEOUT_MS);
  const uint32_t request_tick = g_buzzer_alarm_request.request_tick;
  return (uint32_t)(now_tick - request_tick) <= timeout_ticks;
}

static void Blink_TempAlarmStop(void) {
  if (temp_alarm_level != BUZZER_ALARM_NONE) {
    BUZZER_Stop(&buzzer);
  }
  temp_alarm_level = BUZZER_ALARM_NONE;
  temp_alarm_start_tick = 0U;
  temp_alarm_last_toggle_tick = 0U;
  temp_alarm_min_duration_ticks = 0U;
  temp_alarm_tone_high = false;
}

static void Blink_TempAlarmStart(Buzzer_AlarmLevel_t level,
                                 uint32_t now_tick) {
  temp_alarm_level = level;
  temp_alarm_start_tick = now_tick;
  temp_alarm_last_toggle_tick = now_tick;
  temp_alarm_min_duration_ticks =
      Blink_MsToTicks(g_buzzer_alarm_request.min_duration_ms);
  temp_alarm_tone_high = true;
  if (BUZZER_Set(&buzzer, TEMP_ALARM_TONE_HIGH_HZ, TEMP_ALARM_DUTY) ==
      DEVICE_OK) {
    BUZZER_Start(&buzzer);
  }
}

static bool Blink_TempAlarmUpdate(uint32_t now_tick) {
  const Buzzer_AlarmLevel_t request_level = g_buzzer_alarm_request.level;
  const bool request_active = Blink_TempAlarmRequestActive(now_tick);
  Buzzer_AlarmLevel_t target_level = BUZZER_ALARM_NONE;

  if (request_active) {
    target_level = request_level;
  } else if (temp_alarm_level != BUZZER_ALARM_NONE &&
             !Blink_TickReached(now_tick,
                                temp_alarm_start_tick +
                                    temp_alarm_min_duration_ticks)) {
    target_level = temp_alarm_level;
  }

  if (target_level == BUZZER_ALARM_NONE) {
    Blink_TempAlarmStop();
    return false;
  }

  BUZZER_MusicPlayerSilence(&music_player, &buzzer);
#if BLINK_USE_BREATH_RESPONSE
  Blink_BreathResponseStop();
#endif
#if BLINK_USE_MUSIC_LOOP
  Blink_ResetMusicSwitchGesture();
#endif

  if (temp_alarm_level != target_level) {
    Blink_TempAlarmStart(target_level, now_tick);
    return true;
  }

  const uint32_t tone_ticks = Blink_MsToTicks(TEMP_ALARM_TONE_MS);
  if (Blink_TickReached(now_tick, temp_alarm_last_toggle_tick + tone_ticks)) {
    temp_alarm_tone_high = !temp_alarm_tone_high;
    temp_alarm_last_toggle_tick = now_tick;
    const float freq = temp_alarm_tone_high ? TEMP_ALARM_TONE_HIGH_HZ
                                            : TEMP_ALARM_TONE_LOW_HZ;
    if (BUZZER_Set(&buzzer, freq, TEMP_ALARM_DUTY) == DEVICE_OK) {
      BUZZER_Start(&buzzer);
    }
  }
  return true;
}

#if BLINK_USE_MUSIC_LOOP
static void Blink_StartPlaylistTrack(uint32_t now_tick, bool keep_pause_state) {
  bool should_pause = keep_pause_state && music_user_paused;

  if (ARRAY_LEN(buzzer_playlist) == 0U) {
    BUZZER_MusicPlayerStop(&music_player, &buzzer);
    return;
  }

  if (music_playlist_index >= ARRAY_LEN(buzzer_playlist)) {
    music_playlist_index = 0U;
  }

  if (BUZZER_MusicPlayerStart(&music_player, &buzzer,
                              buzzer_playlist[music_playlist_index], false,
                              now_tick) != DEVICE_OK) {
    music_playlist_index =
        (music_playlist_index + 1U) % ARRAY_LEN(buzzer_playlist);
    BUZZER_MusicPlayerStart(&music_player, &buzzer,
                            buzzer_playlist[music_playlist_index], false,
                            now_tick);
  }

  music_user_paused = should_pause;
  if (music_user_paused) {
    BUZZER_MusicPlayerSetPaused(&music_player, &buzzer, true, now_tick);
  }
}

static void Blink_StartNextPlaylistTrack(uint32_t now_tick) {
  if (ARRAY_LEN(buzzer_playlist) == 0U) {
    BUZZER_MusicPlayerStop(&music_player, &buzzer);
    return;
  }

  music_playlist_index = (music_playlist_index + 1U) % ARRAY_LEN(buzzer_playlist);
  Blink_StartPlaylistTrack(now_tick, true);
}

#if BLINK_USE_STARTUP_MUSIC
static void Blink_StartPlaylistAfterStartup(uint32_t now_tick) {
  music_startup_jingle_active = false;
  music_playlist_index = 0U;
  Blink_StartPlaylistTrack(now_tick, false);
}
#endif
#endif

#if BLINK_USE_STARTUP_MUSIC
static void Blink_FinishStartupMusic(uint32_t now_tick) {
#if BLINK_USE_MUSIC_LOOP
  Blink_StartPlaylistAfterStartup(now_tick);
#else
  music_startup_jingle_active = false;
  (void)now_tick;
#endif
}
#endif

#if BLINK_USE_BREATH_RESPONSE
static void Blink_BreathResponseStop(void) {
  breath_response_active = false;
  BUZZER_Stop(&buzzer);
}

static void Blink_BreathResponseUpdate(uint32_t now_tick) {
  const uint32_t period_ticks = Blink_MsToTicks(BREATH_RESPONSE_PERIOD_MS);
  const uint32_t on_ticks = Blink_MsToTicks(BREATH_RESPONSE_ON_MS);
  const uint32_t step_ticks = Blink_MsToTicks(BREATH_RESPONSE_STEP_MS);

  if (!breath_response_active) {
    if (breath_response_last_tick != 0U &&
        (uint32_t)(now_tick - breath_response_last_tick) < period_ticks) {
      return;
    }

    breath_response_active = true;
    breath_response_start_tick = now_tick;
    breath_response_next_update_tick = now_tick;
  }

  if (Blink_TickReached(now_tick, breath_response_start_tick + on_ticks)) {
    Blink_BreathResponseStop();
    breath_response_last_tick = now_tick;
    return;
  }

  if (!Blink_TickReached(now_tick, breath_response_next_update_tick)) {
    return;
  }

  const uint32_t elapsed_ticks =
      (uint32_t)(now_tick - breath_response_start_tick);
  uint32_t half_ticks = on_ticks / 2U;
  if (half_ticks == 0U) {
    half_ticks = 1U;
  }

  float phase;
  if (elapsed_ticks <= half_ticks) {
    phase = (float)elapsed_ticks / (float)half_ticks;
  } else {
    const uint32_t falling_ticks =
        (elapsed_ticks < on_ticks) ? (on_ticks - elapsed_ticks) : 0U;
    phase = (float)falling_ticks / (float)half_ticks;
  }

  if (phase < 0.0f) {
    phase = 0.0f;
  } else if (phase > 1.0f) {
    phase = 1.0f;
  }

  const float freq = BREATH_RESPONSE_FREQ_LOW_HZ +
                     (BREATH_RESPONSE_FREQ_HIGH_HZ -
                      BREATH_RESPONSE_FREQ_LOW_HZ) *
                         phase;
  if (BUZZER_Set(&buzzer, freq, BREATH_RESPONSE_VOLUME) == DEVICE_OK) {
    BUZZER_Start(&buzzer);
  }

  breath_response_next_update_tick = now_tick + step_ticks;
}
#endif

#if BLINK_USE_MUSIC_LOOP
static bool Blink_MusicSwitchGestureTriggered(void) {
  return dr16.header.online && dr16.data.sw_l == DR16_SW_UP &&
         music_last_sw_r == DR16_SW_UP && dr16.data.sw_r == DR16_SW_MID;
}

static void Blink_ResetMusicSwitchGesture(void) {
  music_switch_pending = false;
  music_last_sw_r = dr16.header.online ? dr16.data.sw_r : DR16_SW_ERR;
}

static void Blink_HandleMusicSwitchGesture(uint32_t now_tick) {
  const uint32_t double_trigger_ticks =
      Blink_MsToTicks(MUSIC_SWITCH_DOUBLE_TRIGGER_MS);
  const bool gesture_triggered = Blink_MusicSwitchGestureTriggered();

  if (gesture_triggered) {
    if (music_switch_pending) {
      const uint32_t elapsed_ticks =
          (uint32_t)(now_tick - music_switch_pending_tick);
      if (elapsed_ticks <= double_trigger_ticks) {
        music_switch_pending = false;
        music_user_paused = !music_user_paused;
        BUZZER_MusicPlayerSetPaused(&music_player, &buzzer, music_user_paused,
                                    now_tick);
      } else {
        Blink_StartNextPlaylistTrack(now_tick);
        music_switch_pending = true;
        music_switch_pending_tick = now_tick;
      }
    } else {
      music_switch_pending = true;
      music_switch_pending_tick = now_tick;
    }
  } else if (music_switch_pending &&
             Blink_TickReached(now_tick,
                               music_switch_pending_tick +
                                   double_trigger_ticks)) {
    music_switch_pending = false;
    Blink_StartNextPlaylistTrack(now_tick);
  }

  music_last_sw_r = dr16.header.online ? dr16.data.sw_r : DR16_SW_ERR;
}
#endif

static void Blink_UpdateAudio(uint32_t now_tick) {
  (void)now_tick;

  if (g_buzzer_calib_active) {
    Blink_TempAlarmStop();
    BUZZER_MusicPlayerSilence(&music_player, &buzzer);
#if BLINK_USE_BREATH_RESPONSE
    Blink_BreathResponseStop();
#endif
#if BLINK_USE_MUSIC_LOOP
    Blink_ResetMusicSwitchGesture();
#endif
    return;
  }

  if (Blink_TempAlarmUpdate(now_tick)) {
    return;
  }

#if BLINK_USE_MUSIC_LOOP
#if BLINK_USE_STARTUP_MUSIC
  if (!music_startup_jingle_active) {
    Blink_HandleMusicSwitchGesture(now_tick);
  } else {
    Blink_ResetMusicSwitchGesture();
  }
#else
  Blink_HandleMusicSwitchGesture(now_tick);
#endif
#endif

#if BLINK_USE_STARTUP_MUSIC
  if (music_startup_jingle_active) {
    if (BUZZER_MusicPlayerUpdate(&music_player, &buzzer, now_tick) !=
            DEVICE_OK ||
        !BUZZER_MusicPlayerIsActive(&music_player)) {
      Blink_FinishStartupMusic(now_tick);
    }
    return;
  }
#endif

#if BLINK_USE_MUSIC_LOOP
  if (music_user_paused) {
    BUZZER_Stop(&buzzer);
    return;
  }

  if (BUZZER_MusicPlayerUpdate(&music_player, &buzzer, now_tick) !=
          DEVICE_OK ||
      !BUZZER_MusicPlayerIsActive(&music_player)) {
    Blink_StartNextPlaylistTrack(now_tick);
  }
  return;
#endif

#if BLINK_USE_BREATH_RESPONSE
  Blink_BreathResponseUpdate(now_tick);
#endif
}

/* Exported functions ------------------------------------------------------- */
void Task_blink(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / BLINK_FREQ;

  osDelay(BLINK_INIT_DELAY);

  uint32_t tick = osKernelGetTickCount();
  /* USER CODE INIT BEGIN */
  if (!buzzer.header.online) {
    if (BUZZER_Init(&buzzer, BSP_PWM_BUZZER) != DEVICE_OK) {
      osThreadTerminate(osThreadGetId());
      return;
    }
  }
#if BLINK_USE_STARTUP_MUSIC
  music_startup_jingle_active =
      BUZZER_MusicPlayerStart(&music_player, &buzzer,
                              BLINK_STARTUP_MUSIC, false, tick) == DEVICE_OK;
#endif
#if BLINK_USE_MUSIC_LOOP
#if BLINK_USE_STARTUP_MUSIC
  if (!music_startup_jingle_active) {
    Blink_StartPlaylistTrack(tick, false);
  }
#else
  Blink_StartPlaylistTrack(tick, false);
#endif
#endif

  /* USER CODE INIT END */

  while (1) {
    tick += delay_tick;
    /* USER CODE BEGIN */
    const uint32_t now_tick = osKernelGetTickCount();
    Blink_UpdateAudio(now_tick);

    if (reset) {
      __set_FAULTMASK(1);
      NVIC_SystemReset();
    }
    /* USER CODE END */
    task_runtime.stack_water_mark.blink = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick);
  }
}
