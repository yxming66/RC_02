/*
    blink Task
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "bsp/pwm.h"
#include "device/buzzer.h"
#include "device/dr16.h"

#ifndef BLINK_AUDIO_FEATURES
#define BLINK_AUDIO_FEATURES (1U << 0)  /* startup music only */
#endif

#include "module/cloudmusic/cloudmusic.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
#define BLINK_USE_BREATH_RESPONSE \
  ((CLOUDMUSIC_FEATURES & CLOUDMUSIC_FEATURE_BREATH_RESPONSE) != 0U)

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
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
BUZZER_t buzzer;
extern DR16_t dr16;
bool reset = 0;

static CloudMusic_t cloudmusic;
static Buzzer_AlarmLevel_t temp_alarm_level = BUZZER_ALARM_NONE;
static uint32_t temp_alarm_start_tick;
static uint32_t temp_alarm_last_toggle_tick;
static uint32_t temp_alarm_min_duration_ticks;
static bool temp_alarm_tone_high;

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

static CloudMusic_Input_t Blink_GetCloudMusicInput(void) {
  CloudMusic_Input_t input = {
      .rc_online = dr16.header.online,
      .sw_l = dr16.data.sw_l,
      .sw_r = dr16.data.sw_r,
  };
  return input;
}

#if BLINK_USE_BREATH_RESPONSE
static void Blink_BreathResponseStop(void);
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

  const CloudMusic_Input_t input = Blink_GetCloudMusicInput();
  CloudMusic_Silence(&cloudmusic);
#if BLINK_USE_BREATH_RESPONSE
  Blink_BreathResponseStop();
#endif
  CloudMusic_ResetGesture(&cloudmusic, &input);

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

static void Blink_UpdateAudio(uint32_t now_tick) {
  if (g_buzzer_calib_active) {
    const CloudMusic_Input_t input = Blink_GetCloudMusicInput();
    Blink_TempAlarmStop();
    CloudMusic_Silence(&cloudmusic);
#if BLINK_USE_BREATH_RESPONSE
    Blink_BreathResponseStop();
#endif
    CloudMusic_ResetGesture(&cloudmusic, &input);
    return;
  }

  if (Blink_TempAlarmUpdate(now_tick)) {
    return;
  }

  const CloudMusic_Input_t input = Blink_GetCloudMusicInput();
  (void)CloudMusic_Update(&cloudmusic, &input, now_tick);

#if BLINK_USE_BREATH_RESPONSE
  if (CloudMusic_AllowsIdleEffect(&cloudmusic)) {
    Blink_BreathResponseUpdate(now_tick);
  }
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

  CloudMusic_Config_t cloudmusic_config;
  CloudMusic_GetDefaultConfig(&cloudmusic_config);
  cloudmusic_config.enable_startup_music = true;
  cloudmusic_config.enable_music_loop = false;
  if (CloudMusic_Init(&cloudmusic, &buzzer, &cloudmusic_config) != DEVICE_OK) {
    osThreadTerminate(osThreadGetId());
    return;
  }
  (void)CloudMusic_Start(&cloudmusic, tick);

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
    task_runtime.heartbeat.blink++;
    osDelayUntil(tick);
  }
}
