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
#define BLINK_ENABLE_MUSIC_PLAYLIST 0U
#define BLINK_ENABLE_BREATH_RESPONSE 1U

#define MUSIC_SWITCH_DOUBLE_TRIGGER_MS 500U
#define BREATH_RESPONSE_PERIOD_MS 3000U
#define BREATH_RESPONSE_ON_MS 900U
#define BREATH_RESPONSE_STEP_MS 50U
#define BREATH_RESPONSE_FREQ_LOW_HZ 392.0f
#define BREATH_RESPONSE_FREQ_HIGH_HZ 659.25f
#define BREATH_RESPONSE_VOLUME 0.12f

/* Private macro ------------------------------------------------------------ */
#define ARRAY_LEN(arr) (sizeof(arr) / sizeof((arr)[0]))

/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
BUZZER_t buzzer;
extern DR16_t dr16;
bool reset = 0;

#if BLINK_ENABLE_MUSIC_PLAYLIST
static const MUSIC_t buzzer_playlist[] = {
    MUSIC_YUAN_YU_CHOU,
    MUSIC_TORI_NO_UTA,
    MUSIC_YI_BU_ZHI_YAO,
    MUSIC_FLOWER_DANCE,
    MUSIC_HONG_DOU,
    MUSIC_SHUN,
};
#endif

static BUZZER_MusicPlayer_t music_player;
#if BLINK_ENABLE_MUSIC_PLAYLIST
static size_t music_playlist_index;
static bool music_user_paused;
static bool music_switch_pending;
static uint32_t music_switch_pending_tick;
static DR16_SwitchPos_t music_last_sw_r = DR16_SW_ERR;
#endif
static bool music_startup_jingle_active;

#if BLINK_ENABLE_BREATH_RESPONSE
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

#if BLINK_ENABLE_MUSIC_PLAYLIST
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

static void Blink_StartPlaylistAfterStartup(uint32_t now_tick) {
  music_startup_jingle_active = false;
  music_playlist_index = 0U;
  Blink_StartPlaylistTrack(now_tick, false);
}
#endif

#if BLINK_ENABLE_BREATH_RESPONSE
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

#if BLINK_ENABLE_MUSIC_PLAYLIST
static bool Blink_MusicSwitchGestureTriggered(void) {
  return dr16.header.online && dr16.data.sw_l == DR16_SW_UP &&
         music_last_sw_r == DR16_SW_UP && dr16.data.sw_r == DR16_SW_MID;
}

static void Blink_HandleMusicSwitchGesture(uint32_t now_tick) {
  const uint32_t double_trigger_ticks =
      Blink_MsToTicks(MUSIC_SWITCH_DOUBLE_TRIGGER_MS);

  if (Blink_MusicSwitchGestureTriggered()) {
    if (music_switch_pending &&
        (uint32_t)(now_tick - music_switch_pending_tick) <=
            double_trigger_ticks) {
      music_switch_pending = false;
      music_user_paused = !music_user_paused;
      BUZZER_MusicPlayerSetPaused(&music_player, &buzzer, music_user_paused,
                                  now_tick);
    } else {
      music_switch_pending = true;
      music_switch_pending_tick = now_tick;
    }
  }

  if (music_switch_pending &&
      Blink_TickReached(now_tick,
                        music_switch_pending_tick + double_trigger_ticks)) {
    music_switch_pending = false;
    Blink_StartNextPlaylistTrack(now_tick);
  }

  music_last_sw_r = dr16.header.online ? dr16.data.sw_r : DR16_SW_ERR;
}
#endif

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
  music_startup_jingle_active =
      BUZZER_MusicPlayerStart(&music_player, &buzzer,
                              MUSIC_STARTUP_YUAN_YU_CHOU, false, tick) ==
      DEVICE_OK;
#if BLINK_ENABLE_MUSIC_PLAYLIST
  if (!music_startup_jingle_active) {
    Blink_StartPlaylistTrack(tick, false);
  }
#endif

  /* USER CODE INIT END */

  while (1) {
    tick += delay_tick;
    /* USER CODE BEGIN */
    const uint32_t now_tick = osKernelGetTickCount();
#if BLINK_ENABLE_MUSIC_PLAYLIST
    if (!music_startup_jingle_active) {
      Blink_HandleMusicSwitchGesture(now_tick);
    }
#endif

    if (g_buzzer_calib_active) {
      BUZZER_MusicPlayerSilence(&music_player, &buzzer);
#if BLINK_ENABLE_BREATH_RESPONSE
      Blink_BreathResponseStop();
#endif
    } else if (music_startup_jingle_active) {
      if (BUZZER_MusicPlayerUpdate(&music_player, &buzzer, now_tick) !=
          DEVICE_OK) {
        music_startup_jingle_active = false;
#if BLINK_ENABLE_MUSIC_PLAYLIST
        Blink_StartPlaylistAfterStartup(now_tick);
#endif
      } else if (!BUZZER_MusicPlayerIsActive(&music_player)) {
        music_startup_jingle_active = false;
#if BLINK_ENABLE_MUSIC_PLAYLIST
        Blink_StartPlaylistAfterStartup(now_tick);
#endif
      }
#if BLINK_ENABLE_MUSIC_PLAYLIST
    } else if (!music_user_paused) {
      if (BUZZER_MusicPlayerUpdate(&music_player, &buzzer, now_tick) !=
          DEVICE_OK) {
        if (music_startup_jingle_active) {
          Blink_StartPlaylistAfterStartup(now_tick);
        } else {
          Blink_StartNextPlaylistTrack(now_tick);
        }
      } else if (!BUZZER_MusicPlayerIsActive(&music_player)) {
        if (music_startup_jingle_active) {
          Blink_StartPlaylistAfterStartup(now_tick);
        } else {
          Blink_StartNextPlaylistTrack(now_tick);
        }
      }
    } else {
      BUZZER_Stop(&buzzer);
    }
#else
    } else {
      Blink_BreathResponseUpdate(now_tick);
    }
#endif

    if (reset) {
      __set_FAULTMASK(1);
      NVIC_SystemReset();
    }
    /* USER CODE END */
    task_runtime.stack_water_mark.blink = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(tick);
  }
   }
