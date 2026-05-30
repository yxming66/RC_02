#include "module/cloudmusic/cloudmusic.h"

#include <cmsis_os2.h>

#define ARRAY_LEN(arr) (sizeof(arr) / sizeof((arr)[0]))

#if ((CLOUDMUSIC_FEATURES &                                             \
      ~(CLOUDMUSIC_FEATURE_STARTUP_MUSIC |                              \
        CLOUDMUSIC_FEATURE_BREATH_RESPONSE | CLOUDMUSIC_FEATURE_MUSIC_LOOP)) != 0U)
#error "CLOUDMUSIC_FEATURES contains unsupported feature bits"
#endif

static const MUSIC_t cloudmusic_default_playlist[] = {
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

static uint32_t CloudMusic_MsToTicks(uint32_t ms) {
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

static bool CloudMusic_TickReached(uint32_t now_tick, uint32_t target_tick) {
  return (int32_t)(now_tick - target_tick) >= 0;
}

static bool CloudMusic_HasPlaylist(const CloudMusic_t *cloudmusic) {
  return cloudmusic != NULL && cloudmusic->playlist != NULL &&
         cloudmusic->playlist_length > 0U;
}

static int8_t CloudMusic_StartPlaylistTrack(CloudMusic_t *cloudmusic,
                                            uint32_t now_tick,
                                            bool keep_pause_state) {
  if (cloudmusic == NULL || cloudmusic->buzzer == NULL) {
    return DEVICE_ERR;
  }

  const bool should_pause = keep_pause_state && cloudmusic->user_paused;

  if (!CloudMusic_HasPlaylist(cloudmusic)) {
    BUZZER_MusicPlayerStop(&cloudmusic->player, cloudmusic->buzzer);
    return DEVICE_ERR;
  }

  if (cloudmusic->playlist_index >= cloudmusic->playlist_length) {
    cloudmusic->playlist_index = 0U;
  }

  int8_t ret = BUZZER_MusicPlayerStart(
      &cloudmusic->player, cloudmusic->buzzer,
      cloudmusic->playlist[cloudmusic->playlist_index], false, now_tick);
  if (ret != DEVICE_OK) {
    cloudmusic->playlist_index =
        (cloudmusic->playlist_index + 1U) % cloudmusic->playlist_length;
    ret = BUZZER_MusicPlayerStart(&cloudmusic->player, cloudmusic->buzzer,
                                  cloudmusic->playlist[cloudmusic->playlist_index],
                                  false, now_tick);
  }

  cloudmusic->user_paused = should_pause;
  if (cloudmusic->user_paused) {
    BUZZER_MusicPlayerSetPaused(&cloudmusic->player, cloudmusic->buzzer, true,
                                now_tick);
  }

  return ret;
}

static void CloudMusic_FinishStartupMusic(CloudMusic_t *cloudmusic,
                                          uint32_t now_tick) {
  if (cloudmusic == NULL) {
    return;
  }

  cloudmusic->startup_active = false;
  if (cloudmusic->enable_music_loop) {
    cloudmusic->playlist_index = 0U;
    (void)CloudMusic_StartPlaylistTrack(cloudmusic, now_tick, false);
  }
}

static bool CloudMusic_SwitchGestureTriggered(
    const CloudMusic_t *cloudmusic, const CloudMusic_Input_t *input) {
  return cloudmusic != NULL && input != NULL && input->rc_online &&
         input->sw_l == DR16_SW_UP && cloudmusic->last_sw_r == DR16_SW_UP &&
         input->sw_r == DR16_SW_MID;
}

static void CloudMusic_HandleSwitchGesture(CloudMusic_t *cloudmusic,
                                           const CloudMusic_Input_t *input,
                                           uint32_t now_tick) {
  if (cloudmusic == NULL || input == NULL) {
    return;
  }

  const uint32_t double_trigger_ticks =
      CloudMusic_MsToTicks(cloudmusic->switch_double_trigger_ms);
  const bool gesture_triggered =
      CloudMusic_SwitchGestureTriggered(cloudmusic, input);

  if (gesture_triggered) {
    if (cloudmusic->switch_pending) {
      const uint32_t elapsed_ticks =
          (uint32_t)(now_tick - cloudmusic->switch_pending_tick);
      if (elapsed_ticks <= double_trigger_ticks) {
        cloudmusic->switch_pending = false;
        CloudMusic_TogglePaused(cloudmusic, now_tick);
      } else {
        (void)CloudMusic_Next(cloudmusic, now_tick);
        cloudmusic->switch_pending = true;
        cloudmusic->switch_pending_tick = now_tick;
      }
    } else {
      cloudmusic->switch_pending = true;
      cloudmusic->switch_pending_tick = now_tick;
    }
  } else if (cloudmusic->switch_pending &&
             CloudMusic_TickReached(now_tick,
                                    cloudmusic->switch_pending_tick +
                                        double_trigger_ticks)) {
    cloudmusic->switch_pending = false;
    (void)CloudMusic_Next(cloudmusic, now_tick);
  }

  cloudmusic->last_sw_r = input->rc_online ? input->sw_r : DR16_SW_ERR;
}

void CloudMusic_GetDefaultConfig(CloudMusic_Config_t *config) {
  if (config == NULL) {
    return;
  }

  config->playlist = cloudmusic_default_playlist;
  config->playlist_length = ARRAY_LEN(cloudmusic_default_playlist);
  config->startup_music = CLOUDMUSIC_STARTUP_MUSIC;
  config->switch_double_trigger_ms = CLOUDMUSIC_SWITCH_DOUBLE_TRIGGER_MS;
  config->enable_startup_music =
      (CLOUDMUSIC_FEATURES & CLOUDMUSIC_FEATURE_STARTUP_MUSIC) != 0U;
  config->enable_music_loop =
      (CLOUDMUSIC_FEATURES & CLOUDMUSIC_FEATURE_MUSIC_LOOP) != 0U;
}

int8_t CloudMusic_Init(CloudMusic_t *cloudmusic, BUZZER_t *buzzer,
                       const CloudMusic_Config_t *config) {
  if (cloudmusic == NULL || buzzer == NULL) {
    return DEVICE_ERR;
  }

  CloudMusic_Config_t default_config;
  if (config == NULL) {
    CloudMusic_GetDefaultConfig(&default_config);
    config = &default_config;
  }

  *cloudmusic = (CloudMusic_t){0};
  cloudmusic->buzzer = buzzer;
  cloudmusic->playlist = config->playlist;
  cloudmusic->playlist_length = config->playlist_length;
  cloudmusic->startup_music = config->startup_music;
  cloudmusic->switch_double_trigger_ms = config->switch_double_trigger_ms;
  cloudmusic->enable_startup_music = config->enable_startup_music;
  cloudmusic->enable_music_loop = config->enable_music_loop;
  cloudmusic->last_sw_r = DR16_SW_ERR;

  if (cloudmusic->switch_double_trigger_ms == 0U) {
    cloudmusic->switch_double_trigger_ms = CLOUDMUSIC_SWITCH_DOUBLE_TRIGGER_MS;
  }

  return DEVICE_OK;
}

int8_t CloudMusic_Start(CloudMusic_t *cloudmusic, uint32_t now_tick) {
  if (cloudmusic == NULL || cloudmusic->buzzer == NULL) {
    return DEVICE_ERR;
  }

  if (cloudmusic->enable_startup_music) {
    cloudmusic->startup_active =
        BUZZER_MusicPlayerStart(&cloudmusic->player, cloudmusic->buzzer,
                                cloudmusic->startup_music, false,
                                now_tick) == DEVICE_OK;
  }

  if (cloudmusic->enable_music_loop && !cloudmusic->startup_active) {
    return CloudMusic_StartPlaylistTrack(cloudmusic, now_tick, false);
  }

  return DEVICE_OK;
}

int8_t CloudMusic_Update(CloudMusic_t *cloudmusic,
                         const CloudMusic_Input_t *input,
                         uint32_t now_tick) {
  if (cloudmusic == NULL || cloudmusic->buzzer == NULL) {
    return DEVICE_ERR;
  }

  if (cloudmusic->enable_music_loop) {
    if (!cloudmusic->startup_active) {
      CloudMusic_HandleSwitchGesture(cloudmusic, input, now_tick);
    } else {
      CloudMusic_ResetGesture(cloudmusic, input);
    }
  }

  if (cloudmusic->startup_active) {
    if (BUZZER_MusicPlayerUpdate(&cloudmusic->player, cloudmusic->buzzer,
                                 now_tick) != DEVICE_OK ||
        !BUZZER_MusicPlayerIsActive(&cloudmusic->player)) {
      CloudMusic_FinishStartupMusic(cloudmusic, now_tick);
    }
    return DEVICE_OK;
  }

  if (!cloudmusic->enable_music_loop) {
    return DEVICE_OK;
  }

  if (cloudmusic->user_paused) {
    BUZZER_Stop(cloudmusic->buzzer);
    return DEVICE_OK;
  }

  if (BUZZER_MusicPlayerUpdate(&cloudmusic->player, cloudmusic->buzzer,
                               now_tick) != DEVICE_OK ||
      !BUZZER_MusicPlayerIsActive(&cloudmusic->player)) {
    return CloudMusic_Next(cloudmusic, now_tick);
  }

  return DEVICE_OK;
}

int8_t CloudMusic_ApplyControl(CloudMusic_t *cloudmusic,
                               const CloudMusic_Control_t *control,
                               uint32_t now_tick) {
  if (cloudmusic == NULL || control == NULL) {
    return DEVICE_ERR;
  }

  if (control->stop_event) {
    CloudMusic_Stop(cloudmusic);
  }
  if (control->play_event) {
    CloudMusic_SetPaused(cloudmusic, false, now_tick);
  }
  if (control->previous_event) {
    (void)CloudMusic_Previous(cloudmusic, now_tick);
  }
  if (control->next_event) {
    (void)CloudMusic_Next(cloudmusic, now_tick);
  }
  if (control->toggle_pause_event) {
    CloudMusic_TogglePaused(cloudmusic, now_tick);
  }

  return DEVICE_OK;
}

int8_t CloudMusic_Next(CloudMusic_t *cloudmusic, uint32_t now_tick) {
  if (!CloudMusic_HasPlaylist(cloudmusic)) {
    if (cloudmusic != NULL && cloudmusic->buzzer != NULL) {
      BUZZER_MusicPlayerStop(&cloudmusic->player, cloudmusic->buzzer);
    }
    return DEVICE_ERR;
  }

  cloudmusic->startup_active = false;
  cloudmusic->playlist_index =
      (cloudmusic->playlist_index + 1U) % cloudmusic->playlist_length;
  cloudmusic->next_count++;
  return CloudMusic_StartPlaylistTrack(cloudmusic, now_tick, true);
}

int8_t CloudMusic_Previous(CloudMusic_t *cloudmusic, uint32_t now_tick) {
  if (!CloudMusic_HasPlaylist(cloudmusic)) {
    if (cloudmusic != NULL && cloudmusic->buzzer != NULL) {
      BUZZER_MusicPlayerStop(&cloudmusic->player, cloudmusic->buzzer);
    }
    return DEVICE_ERR;
  }

  cloudmusic->startup_active = false;
  if (cloudmusic->playlist_index == 0U) {
    cloudmusic->playlist_index = cloudmusic->playlist_length - 1U;
  } else {
    cloudmusic->playlist_index--;
  }
  cloudmusic->previous_count++;
  return CloudMusic_StartPlaylistTrack(cloudmusic, now_tick, true);
}

void CloudMusic_SetPaused(CloudMusic_t *cloudmusic, bool paused,
                          uint32_t now_tick) {
  if (cloudmusic == NULL || cloudmusic->buzzer == NULL) {
    return;
  }

  cloudmusic->startup_active = false;
  cloudmusic->user_paused = paused;
  BUZZER_MusicPlayerSetPaused(&cloudmusic->player, cloudmusic->buzzer, paused,
                              now_tick);
  if (!paused) {
    cloudmusic->play_count++;
  }
}

void CloudMusic_TogglePaused(CloudMusic_t *cloudmusic, uint32_t now_tick) {
  if (cloudmusic == NULL) {
    return;
  }

  cloudmusic->pause_toggle_count++;
  CloudMusic_SetPaused(cloudmusic, !cloudmusic->user_paused, now_tick);
}

void CloudMusic_Stop(CloudMusic_t *cloudmusic) {
  if (cloudmusic == NULL || cloudmusic->buzzer == NULL) {
    return;
  }

  cloudmusic->startup_active = false;
  cloudmusic->user_paused = false;
  cloudmusic->switch_pending = false;
  cloudmusic->stop_count++;
  BUZZER_MusicPlayerStop(&cloudmusic->player, cloudmusic->buzzer);
}

void CloudMusic_Silence(CloudMusic_t *cloudmusic) {
  if (cloudmusic == NULL || cloudmusic->buzzer == NULL) {
    return;
  }

  BUZZER_MusicPlayerSilence(&cloudmusic->player, cloudmusic->buzzer);
}

void CloudMusic_ResetGesture(CloudMusic_t *cloudmusic,
                             const CloudMusic_Input_t *input) {
  if (cloudmusic == NULL) {
    return;
  }

  cloudmusic->switch_pending = false;
  cloudmusic->last_sw_r =
      (input != NULL && input->rc_online) ? input->sw_r : DR16_SW_ERR;
}

void CloudMusic_GetStatus(const CloudMusic_t *cloudmusic,
                          CloudMusic_Status_t *status) {
  if (status == NULL) {
    return;
  }

  *status = (CloudMusic_Status_t){0};
  if (cloudmusic == NULL) {
    return;
  }

  status->enabled = cloudmusic->enable_startup_music ||
                    cloudmusic->enable_music_loop;
  status->active = BUZZER_MusicPlayerIsActive(&cloudmusic->player);
  status->paused = cloudmusic->user_paused ||
                   BUZZER_MusicPlayerIsPaused(&cloudmusic->player);
  status->startup_active = cloudmusic->startup_active;
  status->music_loop_enabled = cloudmusic->enable_music_loop;
  status->switch_pending = cloudmusic->switch_pending;
  status->playlist_index = cloudmusic->playlist_index;
  status->playlist_length = cloudmusic->playlist_length;
  status->current_music = cloudmusic->player.music;
  status->next_count = cloudmusic->next_count;
  status->previous_count = cloudmusic->previous_count;
  status->pause_toggle_count = cloudmusic->pause_toggle_count;
  status->play_count = cloudmusic->play_count;
  status->stop_count = cloudmusic->stop_count;
}

bool CloudMusic_IsEnabled(const CloudMusic_t *cloudmusic) {
  return cloudmusic != NULL &&
         (cloudmusic->enable_startup_music || cloudmusic->enable_music_loop);
}

bool CloudMusic_IsActive(const CloudMusic_t *cloudmusic) {
  return cloudmusic != NULL &&
         BUZZER_MusicPlayerIsActive(&cloudmusic->player);
}

bool CloudMusic_IsPaused(const CloudMusic_t *cloudmusic) {
  return cloudmusic != NULL && cloudmusic->user_paused;
}

bool CloudMusic_AllowsIdleEffect(const CloudMusic_t *cloudmusic) {
  return cloudmusic == NULL || !cloudmusic->enable_music_loop;
}
