#include "device/buzzer.h"

#include "bsp/time.h"

#include <math.h>

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */

/* USER DEFINE BEGIN */

/* USER DEFINE END */

#define BUZZER_DEFAULT_TONE_DUTY 0.5f
#define BUZZER_A4_FREQ_HZ 440.0f

static void BUZZER_Update(BUZZER_t *buzzer) {
    buzzer->header.online = true;
    buzzer->header.last_online_time = BSP_TIME_Get_ms();
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
    if (BUZZER_ApplyTone(buzzer, note, octave) != DEVICE_OK) {
        return DEVICE_ERR;
    }

    BSP_TIME_Delay_ms(duration_ms);

    if (BUZZER_Stop(buzzer) != DEVICE_OK) {
        return DEVICE_ERR;
    }

    if (gap_ms > 0U) {
        BSP_TIME_Delay_ms(gap_ms);
    }

    return DEVICE_OK;
}

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */
