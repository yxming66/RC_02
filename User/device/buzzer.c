#include "device/buzzer.h"

#include "bsp/time.h"

#include <cmsis_os2.h>
#include <math.h>

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */

/* USER DEFINE BEGIN */

/* USER DEFINE END */

#define BUZZER_DEFAULT_TONE_DUTY 0.5f
#define BUZZER_A4_FREQ_HZ 440.0f

static Buzzer_Player_t blocking_compat_player;
static Tone_t blocking_compat_tone;

static void BUZZER_Update(BUZZER_t *buzzer) {
    buzzer->header.online = true;
    buzzer->header.last_online_time = BSP_TIME_Get_ms();
}

static bool BUZZER_ScoreIsValid(const Buzzer_Score_t *score) {
    return score != NULL && score->melody != NULL && score->melody_length > 0U;
}

static uint32_t BUZZER_MsToTicks(uint32_t ms) {
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

static bool BUZZER_TickReached(uint32_t now_tick, uint32_t target_tick) {
    return (int32_t)(now_tick - target_tick) >= 0;
}

int8_t BUZZER_Init(BUZZER_t *buzzer, BSP_PWM_Channel_t channel) {
    if (buzzer == NULL) {
        return DEVICE_ERR;
    }

    buzzer->channel = channel;
    buzzer->header.online = true;
    buzzer->header.last_online_time = BSP_TIME_Get_ms();

    BUZZER_Stop(buzzer);

    return DEVICE_OK;
}

int8_t BUZZER_Start(BUZZER_t *buzzer) {
    if (buzzer == NULL || !buzzer->header.online) {
        return DEVICE_ERR;
    }

    BUZZER_Update(buzzer);
    return (BSP_PWM_Start(buzzer->channel) == BSP_OK) ? DEVICE_OK : DEVICE_ERR;
}

int8_t BUZZER_Stop(BUZZER_t *buzzer) {
    if (buzzer == NULL || !buzzer->header.online) {
        return DEVICE_ERR;
    }

    BUZZER_Update(buzzer);
    return (BSP_PWM_Stop(buzzer->channel) == BSP_OK) ? DEVICE_OK : DEVICE_ERR;
}

int8_t BUZZER_Set(BUZZER_t *buzzer, float freq, float duty_cycle) {
    if (buzzer == NULL || !buzzer->header.online) {
        return DEVICE_ERR;
    }

    int result = DEVICE_OK;
    BUZZER_Update(buzzer);
    if (BSP_PWM_SetFreq(buzzer->channel, freq) != BSP_OK) {
        result = DEVICE_ERR;
    }

    if (BSP_PWM_SetComp(buzzer->channel, duty_cycle) != BSP_OK) {
        result = DEVICE_ERR;
    }

    return result;
}

float BUZZER_CalcFreq(NOTE_t note, uint8_t octave) {
    if (note == NOTE_REST) {
        return 0.0f;
    }

    const int midi_num = (int)note + (int)((octave + 1U) * 12U);
    return BUZZER_A4_FREQ_HZ *
           powf(2.0f, ((float)midi_num - 69.0f) / 12.0f);
}

int8_t BUZZER_ApplyTone(BUZZER_t *buzzer, NOTE_t note, uint8_t octave) {
    if (buzzer == NULL || !buzzer->header.online) {
        return DEVICE_ERR;
    }

    const float freq = BUZZER_CalcFreq(note, octave);
    if (freq <= 0.0f) {
        return BUZZER_Stop(buzzer);
    }

    if (BUZZER_Set(buzzer, freq, BUZZER_DEFAULT_TONE_DUTY) != DEVICE_OK) {
        return DEVICE_ERR;
    }

    return BUZZER_Start(buzzer);
}

int8_t BUZZER_PlayTone(BUZZER_t *buzzer, NOTE_t note, uint8_t octave,
                       uint16_t duration_ms, uint16_t gap_ms) {
    if (buzzer == NULL || !buzzer->header.online) {
        return DEVICE_ERR;
    }

    blocking_compat_tone.note = note;
    blocking_compat_tone.octave = octave;
    blocking_compat_tone.duration_ms = duration_ms;
    const Buzzer_Score_t score = {
        .melody = &blocking_compat_tone,
        .melody_length = 1U,
        .tone_gap_ms = gap_ms,
    };
    return BUZZER_PlayerStart(&blocking_compat_player, buzzer, &score, false,
                              osKernelGetTickCount());
}

int8_t BUZZER_PlayScore(BUZZER_t *buzzer, const Buzzer_Score_t *score) {
    if (buzzer == NULL || !buzzer->header.online ||
        !BUZZER_ScoreIsValid(score)) {
        return DEVICE_ERR;
    }

    return BUZZER_PlayerStart(&blocking_compat_player, buzzer, score, false,
                              osKernelGetTickCount());
}

int8_t BUZZER_PlayerStart(Buzzer_Player_t *player, BUZZER_t *buzzer,
                          const Buzzer_Score_t *score, bool loop,
                          uint32_t now_tick) {
    if (player == NULL || buzzer == NULL || !buzzer->header.online ||
        !BUZZER_ScoreIsValid(score)) {
        return DEVICE_ERR;
    }

    player->melody = score->melody;
    player->melody_length = score->melody_length;
    player->tone_index = 0;
    player->next_tick = now_tick;
    player->tone_gap_ms = score->tone_gap_ms;
    player->active = true;
    player->paused = false;
    player->loop = loop;
    player->waiting_tone = false;
    player->waiting_gap = false;

    BUZZER_Stop(buzzer);
    return BUZZER_PlayerUpdate(player, buzzer, now_tick);
}

int8_t BUZZER_PlayerUpdate(Buzzer_Player_t *player, BUZZER_t *buzzer,
                           uint32_t now_tick) {
    if (player == NULL || buzzer == NULL || !buzzer->header.online) {
        return DEVICE_ERR;
    }

    if (!player->active || player->paused) {
        return DEVICE_OK;
    }

    if ((player->waiting_tone || player->waiting_gap) &&
        !BUZZER_TickReached(now_tick, player->next_tick)) {
        return DEVICE_OK;
    }

    if (player->waiting_tone) {
        BUZZER_Stop(buzzer);
        player->waiting_tone = false;

        if (player->tone_gap_ms > 0U) {
            player->waiting_gap = true;
            player->next_tick = now_tick + BUZZER_MsToTicks(player->tone_gap_ms);
            return DEVICE_OK;
        }
    }

    if (player->waiting_gap) {
        player->waiting_gap = false;
    }

    if (player->tone_index >= player->melody_length) {
        if (player->loop) {
            player->tone_index = 0;
        } else {
            player->active = false;
            BUZZER_Stop(buzzer);
            return DEVICE_OK;
        }
    }

    const Tone_t *tone = &player->melody[player->tone_index++];
    if (BUZZER_ApplyTone(buzzer, tone->note, tone->octave) != DEVICE_OK) {
        player->active = false;
        BUZZER_Stop(buzzer);
        return DEVICE_ERR;
    }

    player->waiting_tone = true;
    player->next_tick = now_tick + BUZZER_MsToTicks(tone->duration_ms);
    return DEVICE_OK;
}

void BUZZER_PlayerStop(Buzzer_Player_t *player, BUZZER_t *buzzer) {
    if (player != NULL) {
        player->active = false;
        player->paused = false;
        player->waiting_tone = false;
        player->waiting_gap = false;
        player->tone_index = 0;
    }

    if (buzzer != NULL) {
        BUZZER_Stop(buzzer);
    }
}

void BUZZER_PlayerSilence(Buzzer_Player_t *player, BUZZER_t *buzzer) {
    if (player != NULL) {
        if (player->waiting_tone && player->tone_index > 0U) {
            player->tone_index--;
        }
        player->waiting_tone = false;
        player->waiting_gap = false;
    }

    if (buzzer != NULL) {
        BUZZER_Stop(buzzer);
    }
}

void BUZZER_PlayerSetPaused(Buzzer_Player_t *player, BUZZER_t *buzzer,
                            bool paused, uint32_t now_tick) {
    if (player == NULL) {
        return;
    }

    if (paused) {
        if (!player->paused) {
            BUZZER_PlayerSilence(player, buzzer);
        }
        player->paused = true;
        return;
    }

    player->paused = false;
    player->next_tick = now_tick;
}

bool BUZZER_PlayerIsActive(const Buzzer_Player_t *player) {
    return player != NULL && player->active;
}

bool BUZZER_PlayerIsPaused(const Buzzer_Player_t *player) {
    return player != NULL && player->paused;
}

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */
