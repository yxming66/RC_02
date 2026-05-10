#include "device/buzzer.h"
#include "bsp/time.h"
#include <cmsis_os2.h>
#include <math.h>

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */

/* USER DEFINE BEGIN */

/* USER DEFINE END */

#define MUSIC_DEFAULT_VOLUME 0.5f
#define MUSIC_A4_FREQ 440.0f  // A4音符频率
#define MUSIC_TONE_GAP_MS 3U
#define FLOWER_DANCE_TONE_GAP_MS 0U
#define TORI_NO_UTA_TONE_GAP_MS 0U
#define GU_SHI_XI_NI_TONE_GAP_MS 0U
#define YI_BU_ZHI_YAO_TONE_GAP_MS 0U
#define HONG_DOU_TONE_GAP_MS 0U
#define YUAN_YU_CHOU_TONE_GAP_MS 0U
#define MOONLIGHT_SONATA_TONE_GAP_MS 1U
#define MOONLIGHT_SONATA_2ND_TONE_GAP_MS 0U
#define MOONLIGHT_SONATA_3RD_TONE_GAP_MS 0U
#define MERRY_CHRISTMAS_MR_LAWRENCE_TONE_GAP_MS 1U
#define BUMBLEBEE_TEMPO_BPM 165U
#define BUMBLEBEE_SIXTEENTH_MS ((60000U + (BUMBLEBEE_TEMPO_BPM * 2U)) / (BUMBLEBEE_TEMPO_BPM * 4U))
#define BUMBLEBEE_TONE_GAP_MS 0U
#define BB16(note, octave) {note, octave, BUMBLEBEE_SIXTEENTH_MS}
#define BBR16 {NOTE_REST, 0, BUMBLEBEE_SIXTEENTH_MS}

/* USER MUSIC MENU BEGIN */
// RM音乐
const Tone_t RM[] = {
    {NOTE_B, 5, 200},
    {NOTE_G, 4, 200},
    {NOTE_B, 5, 400},
    {NOTE_G, 4, 200},
    {NOTE_B, 5, 400},
    {NOTE_G, 4, 200},
    {NOTE_D, 5, 400},
    {NOTE_G, 4, 200},
    {NOTE_C, 5, 200},
    {NOTE_C, 5, 200},
    {NOTE_G, 4, 200},
    {NOTE_B, 5, 200},
    {NOTE_C, 5, 200}
};

// Nokia 经典铃声音符
const Tone_t NOKIA[] = {
    {NOTE_E, 5, 125}, {NOTE_D, 5, 125}, {NOTE_FS, 4, 250}, {NOTE_GS, 4, 250},
    {NOTE_CS, 5, 125}, {NOTE_B, 4, 125}, {NOTE_D, 4, 250}, {NOTE_E, 4, 250},
    {NOTE_B, 4, 125}, {NOTE_A, 4, 125}, {NOTE_CS, 4, 250}, {NOTE_E, 4, 250},
    {NOTE_A, 4, 500}
};

#include "buzzer_fur_elise.inc"
#include "buzzer_jue_bie_shu.inc"
#include "buzzer_hong_dou.inc"
#include "buzzer_shun.inc"
#include "buzzer_yuan_yu_chou.inc"
#include "buzzer_dream_wedding.inc"
#include "buzzer_flower_dance.inc"
#include "buzzer_tori_no_uta.inc"
#include "buzzer_gu_shi_xi_ni.inc"
#include "buzzer_yi_bu_zhi_yao.inc"
#include "buzzer_moonlight_sonata.inc"
#include "buzzer_moonlight_sonata_2.inc"
#include "buzzer_moonlight_sonata_3rd.inc"
#include "buzzer_merry_christmas_mr_lawrence.inc"

const Tone_t STARTUP_TOMATO_GARDEN[] = {
    {NOTE_DS, 4, 300}, {NOTE_F, 4, 300}, {NOTE_G, 4, 450},
    {NOTE_AS, 4, 650}, {NOTE_REST, 0, 100},
    {NOTE_G, 4, 300}, {NOTE_F, 4, 300}, {NOTE_DS, 4, 450},
    {NOTE_AS, 3, 650}, {NOTE_REST, 0, 120},
    {NOTE_DS, 4, 300}, {NOTE_F, 4, 300}, {NOTE_G, 4, 450},
    {NOTE_F, 4, 300}, {NOTE_DS, 4, 900}
};

const Tone_t STARTUP_YUAN_YU_CHOU[] = {
    {NOTE_B, 5, 198}, {NOTE_C, 6, 198}, {NOTE_B, 5, 198},
    {NOTE_G, 5, 198}, {NOTE_E, 5, 198}, {NOTE_G, 5, 198},
    {NOTE_B, 5, 198}, {NOTE_C, 6, 198}, {NOTE_FS, 5, 198},
    {NOTE_FS, 4, 198}, {NOTE_A, 4, 198}, {NOTE_FS, 4, 198},
    {NOTE_FS, 5, 198}, {NOTE_G, 5, 198}, {NOTE_FS, 5, 198},
    {NOTE_D, 5, 198}, {NOTE_E, 5, 594}, {NOTE_REST, 0, 80},
    {NOTE_B, 4, 198}, {NOTE_D, 5, 198}, {NOTE_E, 5, 594}
};

const Tone_t STARTUP_BUMBLEBEE[] = {
    BB16(NOTE_F, 5), BB16(NOTE_F, 5), BB16(NOTE_F, 5), BB16(NOTE_F, 5),
    BB16(NOTE_F, 5), BB16(NOTE_E, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_CS, 5), BB16(NOTE_C, 5), BB16(NOTE_CS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_DS, 5), BB16(NOTE_E, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_CS, 5), BB16(NOTE_C, 5), BB16(NOTE_F, 5), BB16(NOTE_E, 5),
    BB16(NOTE_DS, 5), BB16(NOTE_E, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_CS, 5), BB16(NOTE_C, 5), BB16(NOTE_CS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_DS, 5), BB16(NOTE_E, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_CS, 5), BB16(NOTE_C, 5), BB16(NOTE_CS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_DS, 5), BB16(NOTE_E, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_CS, 5), BB16(NOTE_D, 5), BB16(NOTE_CS, 5), BB16(NOTE_C, 5),
    BB16(NOTE_B, 4), BB16(NOTE_C, 5), BB16(NOTE_CS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_DS, 5), BB16(NOTE_E, 5), BB16(NOTE_F, 5), BB16(NOTE_E, 5),
    BB16(NOTE_DS, 5), BB16(NOTE_E, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_CS, 5), BB16(NOTE_D, 5), BB16(NOTE_CS, 5), BB16(NOTE_C, 5),
    BB16(NOTE_B, 4), BB16(NOTE_C, 5), BB16(NOTE_CS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_DS, 5), BB16(NOTE_E, 5), BB16(NOTE_FS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_FS, 5), BB16(NOTE_F, 5), BB16(NOTE_AS, 5), BB16(NOTE_A, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_FS, 5), BB16(NOTE_F, 5), BB16(NOTE_FS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_FS, 5), BB16(NOTE_F, 5), BB16(NOTE_AS, 5), BB16(NOTE_A, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_FS, 5), BB16(NOTE_F, 5), BB16(NOTE_AS, 5), BB16(NOTE_A, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_FS, 5), BB16(NOTE_F, 5), BB16(NOTE_FS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_FS, 5), BB16(NOTE_G, 5), BB16(NOTE_FS, 5), BB16(NOTE_F, 5),
    BB16(NOTE_E, 5), BB16(NOTE_F, 5), BB16(NOTE_FS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_AS, 5), BB16(NOTE_A, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_FS, 5), BB16(NOTE_G, 5), BB16(NOTE_FS, 5), BB16(NOTE_F, 5),
    BB16(NOTE_E, 5), BB16(NOTE_F, 5), BB16(NOTE_FS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_AS, 5), BB16(NOTE_A, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_E, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 4),
    BB16(NOTE_CS, 4), BB16(NOTE_C, 4), BB16(NOTE_F, 5), BB16(NOTE_E, 5),
    BB16(NOTE_DS, 4), BB16(NOTE_E, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 4),
    BB16(NOTE_CS, 4), BB16(NOTE_C, 4), BB16(NOTE_CS, 4), BB16(NOTE_D, 4),
    BB16(NOTE_DS, 4), BB16(NOTE_E, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 4),
    BB16(NOTE_CS, 4), BB16(NOTE_C, 4), BB16(NOTE_CS, 4), BB16(NOTE_D, 4),
    BB16(NOTE_DS, 4), BB16(NOTE_E, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 4),
    BB16(NOTE_CS, 4), BB16(NOTE_D, 4), BB16(NOTE_CS, 4), BB16(NOTE_C, 4),
    BB16(NOTE_B, 4), BB16(NOTE_C, 4), BB16(NOTE_CS, 4), BB16(NOTE_D, 4),
    BB16(NOTE_DS, 4), BB16(NOTE_E, 5), BB16(NOTE_F, 5), BB16(NOTE_E, 5),
    BB16(NOTE_DS, 4), BB16(NOTE_E, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 4),
    BB16(NOTE_CS, 4), BB16(NOTE_D, 4), BB16(NOTE_CS, 4), BB16(NOTE_C, 4),
    BB16(NOTE_B, 4), BB16(NOTE_C, 4), BB16(NOTE_CS, 4), BB16(NOTE_D, 4),
    BB16(NOTE_DS, 4), BB16(NOTE_E, 5), BB16(NOTE_FS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_FS, 5), BB16(NOTE_F, 5), BB16(NOTE_AS, 5), BB16(NOTE_A, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_FS, 5), BB16(NOTE_F, 5), BB16(NOTE_FS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_FS, 5), BB16(NOTE_F, 5), BB16(NOTE_AS, 5), BB16(NOTE_A, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_FS, 5), BB16(NOTE_F, 5), BB16(NOTE_AS, 5), BB16(NOTE_A, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_FS, 5), BB16(NOTE_F, 5), BB16(NOTE_FS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_FS, 5), BB16(NOTE_G, 5), BB16(NOTE_FS, 5), BB16(NOTE_F, 5),
    BB16(NOTE_E, 5), BB16(NOTE_F, 5), BB16(NOTE_FS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_AS, 5), BB16(NOTE_A, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_FS, 5), BB16(NOTE_G, 5), BB16(NOTE_FS, 5), BB16(NOTE_F, 5),
    BB16(NOTE_E, 5), BB16(NOTE_F, 5), BB16(NOTE_FS, 5), BB16(NOTE_G, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_AS, 5), BB16(NOTE_A, 5),
    BB16(NOTE_GS, 5), BB16(NOTE_A, 5), BB16(NOTE_A, 5), BB16(NOTE_AS, 5),
    BB16(NOTE_A, 5), BB16(NOTE_A, 5), BB16(NOTE_AS, 5), BB16(NOTE_A, 5),
    BB16(NOTE_AS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 4), BB16(NOTE_A, 5),
    BB16(NOTE_AS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 4), BB16(NOTE_A, 5),
    BB16(NOTE_AS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 4), BB16(NOTE_A, 5),
    BB16(NOTE_AS, 5), BB16(NOTE_A, 5), BB16(NOTE_GS, 4), BB16(NOTE_A, 5),
    BB16(NOTE_AS, 5), BB16(NOTE_B, 5), BB16(NOTE_C, 5), BB16(NOTE_CS, 5),
    BB16(NOTE_C, 5), BB16(NOTE_B, 5), BB16(NOTE_AS, 5), BB16(NOTE_A, 5),
    BB16(NOTE_AS, 5), BB16(NOTE_B, 5), BB16(NOTE_C, 5), BB16(NOTE_CS, 5),
    BB16(NOTE_C, 5), BB16(NOTE_B, 5), BB16(NOTE_AS, 5), BB16(NOTE_A, 5),
    BB16(NOTE_D, 5), BB16(NOTE_DS, 5), BB16(NOTE_A, 5), BB16(NOTE_D, 5),
    BB16(NOTE_DS, 5), BB16(NOTE_D, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_CS, 5), BB16(NOTE_D, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_CS, 5), BB16(NOTE_D, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_CS, 5), BB16(NOTE_D, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_CS, 5), BB16(NOTE_D, 5), BB16(NOTE_DS, 5), BB16(NOTE_E, 5),
    BB16(NOTE_F, 5), BB16(NOTE_FS, 5), BB16(NOTE_E, 5), BB16(NOTE_DS, 5),
    BB16(NOTE_D, 5), BB16(NOTE_DS, 5), BB16(NOTE_E, 5), BB16(NOTE_F, 5),
    BB16(NOTE_FS, 5), BB16(NOTE_E, 5), BB16(NOTE_DS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_CS, 5), BB16(NOTE_C, 5), BB16(NOTE_B, 5), BB16(NOTE_AS, 5),
    BB16(NOTE_DS, 5), BB16(NOTE_D, 5), BB16(NOTE_CS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_CS, 5), BB16(NOTE_C, 5), BB16(NOTE_B, 5), BB16(NOTE_AS, 5),
    BB16(NOTE_B, 5), BB16(NOTE_C, 5), BB16(NOTE_CS, 5), BB16(NOTE_D, 5),
    BB16(NOTE_CS, 5), BB16(NOTE_C, 5), BB16(NOTE_B, 5), BB16(NOTE_C, 5),
    BB16(NOTE_B, 5), BB16(NOTE_AS, 5), BB16(NOTE_A, 5), BB16(NOTE_E, 5),
    BB16(NOTE_F, 5), BB16(NOTE_E, 5), BB16(NOTE_DS, 5), BB16(NOTE_E, 5),
    BB16(NOTE_F, 5), BB16(NOTE_E, 5), BB16(NOTE_DS, 5), BBR16,
    BB16(NOTE_E, 5), BBR16, BB16(NOTE_C, 5), BBR16,
    BB16(NOTE_A, 5), BBR16, BB16(NOTE_F, 4), BBR16,
    BB16(NOTE_A, 5), BBR16, BB16(NOTE_C, 5), BBR16,
    BB16(NOTE_E, 5), BB16(NOTE_E, 5), BBR16, BB16(NOTE_C, 5),
    BBR16, BB16(NOTE_A, 5), BBR16, BB16(NOTE_F, 4),
    BBR16, BB16(NOTE_A, 5), BBR16, BB16(NOTE_C, 5),
    BBR16
};

#define DDZ_SIXTEENTH_MS 109U
#define DDZ_EIGHTH_MS 217U
#define DDZ_DOTTED_EIGHTH_MS 326U
#define DDZ_QUARTER_MS 435U
#define DDZ_DOTTED_QUARTER_MS 652U
#define DDZ_HALF_MS 870U
#define DDZ_WHOLE_MS 1740U

#define DDZ_1(ms) {NOTE_C, 5, (ms)}
#define DDZ_2(ms) {NOTE_D, 5, (ms)}
#define DDZ_3(ms) {NOTE_E, 5, (ms)}
#define DDZ_5(ms) {NOTE_G, 4, (ms)}
#define DDZ_6(ms) {NOTE_A, 4, (ms)}
#define DDZ_SHARP_6(ms) {NOTE_AS, 4, (ms)}

/* Right-hand numbered score, 1=C, 4/4, quarter ~= 138 BPM. */
const Tone_t HAPPY_DOU_DI_ZHU_MELODY[] = {
    /* Bars 1-4 */
    DDZ_5(DDZ_QUARTER_MS),
    DDZ_5(DDZ_EIGHTH_MS), DDZ_6(DDZ_EIGHTH_MS),
    DDZ_1(DDZ_EIGHTH_MS), DDZ_5(DDZ_EIGHTH_MS),
    DDZ_6(DDZ_EIGHTH_MS), DDZ_1(DDZ_EIGHTH_MS),
    DDZ_5(DDZ_EIGHTH_MS), DDZ_5(DDZ_EIGHTH_MS),
    DDZ_5(DDZ_EIGHTH_MS), DDZ_3(DDZ_EIGHTH_MS),
    DDZ_5(DDZ_HALF_MS),
    DDZ_5(DDZ_QUARTER_MS),
    DDZ_5(DDZ_EIGHTH_MS), DDZ_6(DDZ_EIGHTH_MS),
    DDZ_1(DDZ_EIGHTH_MS), DDZ_5(DDZ_EIGHTH_MS),
    DDZ_6(DDZ_EIGHTH_MS), DDZ_3(DDZ_EIGHTH_MS),
    DDZ_2(DDZ_EIGHTH_MS), DDZ_2(DDZ_EIGHTH_MS),
    DDZ_2(DDZ_EIGHTH_MS), DDZ_1(DDZ_EIGHTH_MS),
    DDZ_2(DDZ_HALF_MS),

    /* Bars 5-8 */
    DDZ_3(DDZ_QUARTER_MS),
    DDZ_3(DDZ_EIGHTH_MS), DDZ_2(DDZ_EIGHTH_MS),
    DDZ_3(DDZ_QUARTER_MS), DDZ_5(DDZ_QUARTER_MS),
    DDZ_6(DDZ_EIGHTH_MS), DDZ_1(DDZ_EIGHTH_MS),
    DDZ_6(DDZ_EIGHTH_MS), DDZ_5(DDZ_EIGHTH_MS),
    DDZ_6(DDZ_QUARTER_MS),
    DDZ_5(DDZ_EIGHTH_MS), DDZ_3(DDZ_EIGHTH_MS),
    DDZ_2(DDZ_QUARTER_MS),
    DDZ_6(DDZ_EIGHTH_MS), DDZ_3(DDZ_EIGHTH_MS),
    DDZ_2(DDZ_EIGHTH_MS), DDZ_5(DDZ_EIGHTH_MS),
    DDZ_3(DDZ_EIGHTH_MS), DDZ_2(DDZ_EIGHTH_MS),
    DDZ_1(DDZ_QUARTER_MS),
    DDZ_1(DDZ_EIGHTH_MS), DDZ_6(DDZ_EIGHTH_MS),
    DDZ_1(DDZ_HALF_MS),

    /* Bars 9-12 */
    DDZ_5(DDZ_QUARTER_MS),
    DDZ_5(DDZ_EIGHTH_MS), DDZ_6(DDZ_EIGHTH_MS),
    DDZ_1(DDZ_QUARTER_MS),
    DDZ_6(DDZ_EIGHTH_MS), DDZ_1(DDZ_EIGHTH_MS),
    DDZ_5(DDZ_DOTTED_QUARTER_MS), DDZ_3(DDZ_EIGHTH_MS),
    DDZ_5(DDZ_HALF_MS),
    DDZ_5(DDZ_QUARTER_MS),
    DDZ_5(DDZ_EIGHTH_MS), DDZ_6(DDZ_EIGHTH_MS),
    DDZ_1(DDZ_QUARTER_MS), DDZ_3(DDZ_QUARTER_MS),
    DDZ_2(DDZ_EIGHTH_MS), DDZ_3(DDZ_EIGHTH_MS),
    DDZ_2(DDZ_EIGHTH_MS), DDZ_1(DDZ_EIGHTH_MS),
    DDZ_2(DDZ_HALF_MS),

    /* Bars 13-16 */
    DDZ_3(DDZ_QUARTER_MS),
    DDZ_3(DDZ_EIGHTH_MS), DDZ_2(DDZ_EIGHTH_MS),
    DDZ_3(DDZ_QUARTER_MS), DDZ_5(DDZ_QUARTER_MS),
    DDZ_6(DDZ_EIGHTH_MS), DDZ_1(DDZ_EIGHTH_MS),
    DDZ_6(DDZ_EIGHTH_MS), DDZ_5(DDZ_EIGHTH_MS),
    DDZ_6(DDZ_QUARTER_MS),
    DDZ_5(DDZ_EIGHTH_MS), DDZ_3(DDZ_EIGHTH_MS),
    DDZ_2(DDZ_QUARTER_MS),
    DDZ_6(DDZ_EIGHTH_MS), DDZ_3(DDZ_EIGHTH_MS),
    DDZ_2(DDZ_EIGHTH_MS), DDZ_5(DDZ_EIGHTH_MS),
    DDZ_3(DDZ_EIGHTH_MS), DDZ_2(DDZ_EIGHTH_MS),
    DDZ_3(DDZ_QUARTER_MS), DDZ_2(DDZ_QUARTER_MS),
    DDZ_1(DDZ_HALF_MS),

    /* Bars 17-22 */
    DDZ_2(DDZ_DOTTED_EIGHTH_MS), DDZ_3(DDZ_SIXTEENTH_MS),
    DDZ_2(DDZ_EIGHTH_MS), DDZ_3(DDZ_EIGHTH_MS),
    DDZ_5(DDZ_EIGHTH_MS), DDZ_6(DDZ_EIGHTH_MS),
    DDZ_2(DDZ_QUARTER_MS),
    DDZ_1(DDZ_QUARTER_MS), DDZ_5(DDZ_QUARTER_MS),
    DDZ_1(DDZ_HALF_MS),
    DDZ_1(DDZ_WHOLE_MS),
    DDZ_5(DDZ_HALF_MS), DDZ_SHARP_6(DDZ_HALF_MS),
    DDZ_6(DDZ_QUARTER_MS), DDZ_5(DDZ_QUARTER_MS),
    DDZ_6(DDZ_HALF_MS),
    DDZ_6(DDZ_WHOLE_MS)
};

#undef DDZ_SHARP_6
#undef DDZ_6
#undef DDZ_5
#undef DDZ_3
#undef DDZ_2
#undef DDZ_1
#undef DDZ_WHOLE_MS
#undef DDZ_HALF_MS
#undef DDZ_DOTTED_QUARTER_MS
#undef DDZ_QUARTER_MS
#undef DDZ_DOTTED_EIGHTH_MS
#undef DDZ_EIGHTH_MS
#undef DDZ_SIXTEENTH_MS

/* USER MUSIC MENU END */

typedef struct {
    const Tone_t *melody;
    size_t melody_length;
    uint16_t tone_gap_ms;
} BuzzerMusicData_t;

static void BUZZER_Update(BUZZER_t *buzzer){
    buzzer->header.online = true;
    buzzer->header.last_online_time = BSP_TIME_Get_ms();
}

// 根据音符和八度计算频率的辅助函数
static float BUZZER_CalcFreq(NOTE_t note, uint8_t octave) {
    if (note == NOTE_REST) {
        return 0.0f; // 休止符返回0频率
    }
    
    // 将音符和八度转换为MIDI音符编号
    int midi_num = (int)note + (int)((octave + 1) * 12);
    
    // 使用A4 (440Hz) 作为参考，计算频率
    // 公式: freq = 440 * 2^((midi_num - 69)/12)
    float freq = 440.0f * powf(2.0f, ((float)midi_num - 69.0f) / 12.0f);
    
    return freq;
}

// 播放单个音符
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

static int8_t BUZZER_GetMusicData(MUSIC_t music, BuzzerMusicData_t *data) {
    if (data == NULL) {
        return DEVICE_ERR;
    }

    data->melody = NULL;
    data->melody_length = 0;
    data->tone_gap_ms = MUSIC_TONE_GAP_MS;

    switch (music) {
        case MUSIC_RM:
            data->melody = RM;
            data->melody_length = sizeof(RM) / sizeof(Tone_t);
            break;
        case MUSIC_NOKIA:
            data->melody = NOKIA;
            data->melody_length = sizeof(NOKIA) / sizeof(Tone_t);
            break;
        case MUSIC_FUR_ELISE:
            data->melody = FUR_ELISE;
            data->melody_length = sizeof(FUR_ELISE) / sizeof(Tone_t);
            break;
        case MUSIC_JUE_BIE_SHU:
            data->melody = JUE_BIE_SHU;
            data->melody_length = sizeof(JUE_BIE_SHU) / sizeof(Tone_t);
            break;
        case MUSIC_HONG_DOU:
            data->melody = HONG_DOU;
            data->melody_length = sizeof(HONG_DOU) / sizeof(Tone_t);
            data->tone_gap_ms = HONG_DOU_TONE_GAP_MS;
            break;
        case MUSIC_SHUN:
            data->melody = SHUN;
            data->melody_length = sizeof(SHUN) / sizeof(Tone_t);
            break;
        case MUSIC_YUAN_YU_CHOU:
            data->melody = YUAN_YU_CHOU;
            data->melody_length = sizeof(YUAN_YU_CHOU) / sizeof(Tone_t);
            data->tone_gap_ms = YUAN_YU_CHOU_TONE_GAP_MS;
            break;
        case MUSIC_DREAM_WEDDING:
            data->melody = DREAM_WEDDING;
            data->melody_length = sizeof(DREAM_WEDDING) / sizeof(Tone_t);
            break;
        case MUSIC_FLOWER_DANCE:
            data->melody = FLOWER_DANCE;
            data->melody_length = sizeof(FLOWER_DANCE) / sizeof(Tone_t);
            data->tone_gap_ms = FLOWER_DANCE_TONE_GAP_MS;
            break;
        case MUSIC_TORI_NO_UTA:
            data->melody = TORI_NO_UTA;
            data->melody_length = sizeof(TORI_NO_UTA) / sizeof(Tone_t);
            data->tone_gap_ms = TORI_NO_UTA_TONE_GAP_MS;
            break;
        case MUSIC_GU_SHI_XI_NI:
            data->melody = GU_SHI_XI_NI;
            data->melody_length = sizeof(GU_SHI_XI_NI) / sizeof(Tone_t);
            data->tone_gap_ms = GU_SHI_XI_NI_TONE_GAP_MS;
            break;
        case MUSIC_YI_BU_ZHI_YAO:
            data->melody = YI_BU_ZHI_YAO;
            data->melody_length = sizeof(YI_BU_ZHI_YAO) / sizeof(Tone_t);
            data->tone_gap_ms = YI_BU_ZHI_YAO_TONE_GAP_MS;
            break;
        case MUSIC_MOONLIGHT_SONATA:
            data->melody = MOONLIGHT_SONATA_1ST;
            data->melody_length = sizeof(MOONLIGHT_SONATA_1ST) / sizeof(Tone_t);
            data->tone_gap_ms = MOONLIGHT_SONATA_TONE_GAP_MS;
            break;
        case MUSIC_MOONLIGHT_SONATA_2ND:
            data->melody = MOONLIGHT_SONATA_2;
            data->melody_length = sizeof(MOONLIGHT_SONATA_2) / sizeof(Tone_t);
            data->tone_gap_ms = MOONLIGHT_SONATA_2ND_TONE_GAP_MS;
            break;
        case MUSIC_MERRY_CHRISTMAS_MR_LAWRENCE:
            data->melody = MERRY_CHRISTMAS_MR_LAWRENCE;
            data->melody_length =
                sizeof(MERRY_CHRISTMAS_MR_LAWRENCE) / sizeof(Tone_t);
            data->tone_gap_ms = MERRY_CHRISTMAS_MR_LAWRENCE_TONE_GAP_MS;
            break;
        case MUSIC_MOONLIGHT_SONATA_3RD:
            data->melody = MOONLIGHT_SONATA_3RD;
            data->melody_length = sizeof(MOONLIGHT_SONATA_3RD) / sizeof(Tone_t);
            data->tone_gap_ms = MOONLIGHT_SONATA_3RD_TONE_GAP_MS;
            break;
        case MUSIC_HAPPY_DOU_DI_ZHU_MELODY:
            data->melody = HAPPY_DOU_DI_ZHU_MELODY;
            data->melody_length =
                sizeof(HAPPY_DOU_DI_ZHU_MELODY) / sizeof(Tone_t);
            break;
        case MUSIC_STARTUP_YUAN_YU_CHOU:
            data->melody = STARTUP_YUAN_YU_CHOU;
            data->melody_length = sizeof(STARTUP_YUAN_YU_CHOU) / sizeof(Tone_t);
            data->tone_gap_ms = YUAN_YU_CHOU_TONE_GAP_MS;
            break;
        case MUSIC_STARTUP_TOMATO_GARDEN:
            data->melody = STARTUP_TOMATO_GARDEN;
            data->melody_length = sizeof(STARTUP_TOMATO_GARDEN) / sizeof(Tone_t);
            break;
        case MUSIC_STARTUP_BUMBLEBEE:
            data->melody = STARTUP_BUMBLEBEE;
            data->melody_length = sizeof(STARTUP_BUMBLEBEE) / sizeof(Tone_t);
            data->tone_gap_ms = BUMBLEBEE_TONE_GAP_MS;
            break;
        default:
            return DEVICE_ERR;
    }

    return (data->melody != NULL && data->melody_length > 0U) ? DEVICE_OK
                                                               : DEVICE_ERR;
}

static int8_t BUZZER_ApplyTone(BUZZER_t *buzzer, NOTE_t note, uint8_t octave) {
    if (buzzer == NULL || !buzzer->header.online)
        return DEVICE_ERR;

    float freq = BUZZER_CalcFreq(note, octave);

    if (freq > 0.0f) {
        if (BUZZER_Set(buzzer, freq, MUSIC_DEFAULT_VOLUME) != DEVICE_OK)
            return DEVICE_ERR;

        if (BUZZER_Start(buzzer) != DEVICE_OK)
            return DEVICE_ERR;
    } else {
        BUZZER_Stop(buzzer);
    }

    return DEVICE_OK;
}

static int8_t BUZZER_PlayTone(BUZZER_t *buzzer, NOTE_t note, uint8_t octave,
                              uint16_t duration_ms, uint16_t gap_ms) {
    if (buzzer == NULL || !buzzer->header.online) 
        return DEVICE_ERR;
    
    float freq = BUZZER_CalcFreq(note, octave);
    
    if (freq > 0.0f) {
        // 播放音符
        if (BUZZER_Set(buzzer, freq, MUSIC_DEFAULT_VOLUME) != DEVICE_OK) 
            return DEVICE_ERR;
        
        if (BUZZER_Start(buzzer) != DEVICE_OK) 
            return DEVICE_ERR;
    } else {
        // 休止符，停止播放
        BUZZER_Stop(buzzer);
    }
    
    // 等待指定时间
    BSP_TIME_Delay_ms(duration_ms);
    
    // 停止当前音符，为下一个音符做准备
    BUZZER_Stop(buzzer);
    if (gap_ms > 0U) {
        BSP_TIME_Delay_ms(gap_ms); // 短暂间隔
    }
    
    return DEVICE_OK;
}

int8_t BUZZER_Init(BUZZER_t *buzzer, BSP_PWM_Channel_t channel) {
    if (buzzer == NULL) return DEVICE_ERR;
    
    buzzer->channel = channel;
    buzzer->header.online = true;
    
    BUZZER_Stop(buzzer);
    
    return DEVICE_OK ;
}

int8_t BUZZER_Start(BUZZER_t *buzzer) {
    if (buzzer == NULL || !buzzer->header.online) 
        return DEVICE_ERR;
    BUZZER_Update(buzzer);
    return (BSP_PWM_Start(buzzer->channel) == BSP_OK) ? 
           DEVICE_OK  : DEVICE_ERR;
}

int8_t BUZZER_Stop(BUZZER_t *buzzer) {
    if (buzzer == NULL || !buzzer->header.online) 
        return DEVICE_ERR;
    BUZZER_Update(buzzer);
    return (BSP_PWM_Stop(buzzer->channel) == BSP_OK) ? 
           DEVICE_OK  : DEVICE_ERR;
}

int8_t BUZZER_Set(BUZZER_t *buzzer, float freq, float duty_cycle) {
    if (buzzer == NULL || !buzzer->header.online) 
        return DEVICE_ERR;
    
    int result = DEVICE_OK ;
    BUZZER_Update(buzzer);
    if (BSP_PWM_SetFreq(buzzer->channel, freq) != BSP_OK) 
        result = DEVICE_ERR;
    
    if (BSP_PWM_SetComp(buzzer->channel, duty_cycle) != BSP_OK) 
        result = DEVICE_ERR;
    
    return result;
}

int8_t BUZZER_PlayMusic(BUZZER_t *buzzer, MUSIC_t music) {
    if (buzzer == NULL || !buzzer->header.online) 
        return DEVICE_ERR;
    
    const Tone_t *melody = NULL;
    size_t melody_length = 0;
    uint16_t tone_gap_ms = MUSIC_TONE_GAP_MS;
    
    // 根据音乐类型选择对应的音符数组
    switch (music) {
        case MUSIC_RM:
            melody = RM;
            melody_length = sizeof(RM) / sizeof(Tone_t);
            break;
        case MUSIC_NOKIA:
            melody = NOKIA;
            melody_length = sizeof(NOKIA) / sizeof(Tone_t);
            break;
        case MUSIC_FUR_ELISE:
            melody = FUR_ELISE;
            melody_length = sizeof(FUR_ELISE) / sizeof(Tone_t);
            break;
        case MUSIC_JUE_BIE_SHU:
            melody = JUE_BIE_SHU;
            melody_length = sizeof(JUE_BIE_SHU) / sizeof(Tone_t);
            break;
        case MUSIC_HONG_DOU:
            melody = HONG_DOU;
            melody_length = sizeof(HONG_DOU) / sizeof(Tone_t);
            tone_gap_ms = HONG_DOU_TONE_GAP_MS;
            break;
        case MUSIC_SHUN:
            melody = SHUN;
            melody_length = sizeof(SHUN) / sizeof(Tone_t);
            break;
        case MUSIC_YUAN_YU_CHOU:
            melody = YUAN_YU_CHOU;
            melody_length = sizeof(YUAN_YU_CHOU) / sizeof(Tone_t);
            tone_gap_ms = YUAN_YU_CHOU_TONE_GAP_MS;
            break;
        case MUSIC_DREAM_WEDDING:
            melody = DREAM_WEDDING;
            melody_length = sizeof(DREAM_WEDDING) / sizeof(Tone_t);
            break;
        case MUSIC_FLOWER_DANCE:
            melody = FLOWER_DANCE;
            melody_length = sizeof(FLOWER_DANCE) / sizeof(Tone_t);
            tone_gap_ms = FLOWER_DANCE_TONE_GAP_MS;
            break;
        case MUSIC_TORI_NO_UTA:
            melody = TORI_NO_UTA;
            melody_length = sizeof(TORI_NO_UTA) / sizeof(Tone_t);
            tone_gap_ms = TORI_NO_UTA_TONE_GAP_MS;
            break;
        case MUSIC_GU_SHI_XI_NI:
            melody = GU_SHI_XI_NI;
            melody_length = sizeof(GU_SHI_XI_NI) / sizeof(Tone_t);
            tone_gap_ms = GU_SHI_XI_NI_TONE_GAP_MS;
            break;
        case MUSIC_YI_BU_ZHI_YAO:
            melody = YI_BU_ZHI_YAO;
            melody_length = sizeof(YI_BU_ZHI_YAO) / sizeof(Tone_t);
            tone_gap_ms = YI_BU_ZHI_YAO_TONE_GAP_MS;
            break;
        case MUSIC_MOONLIGHT_SONATA:
            melody = MOONLIGHT_SONATA_1ST;
            melody_length = sizeof(MOONLIGHT_SONATA_1ST) / sizeof(Tone_t);
            tone_gap_ms = MOONLIGHT_SONATA_TONE_GAP_MS;
            break;
        case MUSIC_MOONLIGHT_SONATA_2ND:
            melody = MOONLIGHT_SONATA_2;
            melody_length = sizeof(MOONLIGHT_SONATA_2) / sizeof(Tone_t);
            tone_gap_ms = MOONLIGHT_SONATA_2ND_TONE_GAP_MS;
            break;
        case MUSIC_MERRY_CHRISTMAS_MR_LAWRENCE:
            melody = MERRY_CHRISTMAS_MR_LAWRENCE;
            melody_length = sizeof(MERRY_CHRISTMAS_MR_LAWRENCE) / sizeof(Tone_t);
            tone_gap_ms = MERRY_CHRISTMAS_MR_LAWRENCE_TONE_GAP_MS;
            break;
        case MUSIC_MOONLIGHT_SONATA_3RD:
            melody = MOONLIGHT_SONATA_3RD;
            melody_length = sizeof(MOONLIGHT_SONATA_3RD) / sizeof(Tone_t);
            tone_gap_ms = MOONLIGHT_SONATA_3RD_TONE_GAP_MS;
            break;
        case MUSIC_HAPPY_DOU_DI_ZHU_MELODY:
            melody = HAPPY_DOU_DI_ZHU_MELODY;
            melody_length = sizeof(HAPPY_DOU_DI_ZHU_MELODY) / sizeof(Tone_t);
            break;
        case MUSIC_STARTUP_YUAN_YU_CHOU:
            melody = STARTUP_YUAN_YU_CHOU;
            melody_length = sizeof(STARTUP_YUAN_YU_CHOU) / sizeof(Tone_t);
            tone_gap_ms = YUAN_YU_CHOU_TONE_GAP_MS;
            break;
        case MUSIC_STARTUP_TOMATO_GARDEN:
            melody = STARTUP_TOMATO_GARDEN;
            melody_length = sizeof(STARTUP_TOMATO_GARDEN) / sizeof(Tone_t);
            break;
        case MUSIC_STARTUP_BUMBLEBEE:
            melody = STARTUP_BUMBLEBEE;
            melody_length = sizeof(STARTUP_BUMBLEBEE) / sizeof(Tone_t);
            tone_gap_ms = BUMBLEBEE_TONE_GAP_MS;
            break;
        default:
            return DEVICE_ERR;
    }
    
    // 播放整首音乐
    for (size_t i = 0; i < melody_length; i++) {
        if (BUZZER_PlayTone(buzzer, melody[i].note, melody[i].octave,
                            melody[i].duration_ms, tone_gap_ms) != DEVICE_OK) {
            BUZZER_Stop(buzzer); // 出错时停止播放
            return DEVICE_ERR;
        }
    }
    
    // 音乐播放完成后停止
    BUZZER_Stop(buzzer);
    return DEVICE_OK;
}

int8_t BUZZER_MusicPlayerStart(BUZZER_MusicPlayer_t *player,
                                BUZZER_t *buzzer, MUSIC_t music, bool loop,
                                uint32_t now_tick) {
    if (player == NULL || buzzer == NULL || !buzzer->header.online) {
        return DEVICE_ERR;
    }

    BuzzerMusicData_t data;
    if (BUZZER_GetMusicData(music, &data) != DEVICE_OK) {
        return DEVICE_ERR;
    }

    player->melody = data.melody;
    player->melody_length = data.melody_length;
    player->tone_index = 0;
    player->next_tick = now_tick;
    player->tone_gap_ms = data.tone_gap_ms;
    player->music = music;
    player->active = true;
    player->paused = false;
    player->loop = loop;
    player->waiting_tone = false;
    player->waiting_gap = false;

    BUZZER_Stop(buzzer);
    return BUZZER_MusicPlayerUpdate(player, buzzer, now_tick);
}

int8_t BUZZER_MusicPlayerUpdate(BUZZER_MusicPlayer_t *player,
                                 BUZZER_t *buzzer, uint32_t now_tick) {
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
            player->next_tick =
                now_tick + BUZZER_MsToTicks(player->tone_gap_ms);
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

void BUZZER_MusicPlayerStop(BUZZER_MusicPlayer_t *player, BUZZER_t *buzzer) {
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

void BUZZER_MusicPlayerSilence(BUZZER_MusicPlayer_t *player,
                                BUZZER_t *buzzer) {
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

void BUZZER_MusicPlayerSetPaused(BUZZER_MusicPlayer_t *player,
                                  BUZZER_t *buzzer, bool paused,
                                  uint32_t now_tick) {
    if (player == NULL) {
        return;
    }

    if (paused) {
        if (!player->paused) {
            BUZZER_MusicPlayerSilence(player, buzzer);
        }
        player->paused = true;
        return;
    }

    player->paused = false;
    player->next_tick = now_tick;
}

bool BUZZER_MusicPlayerIsActive(const BUZZER_MusicPlayer_t *player) {
    return player != NULL && player->active;
}

bool BUZZER_MusicPlayerIsPaused(const BUZZER_MusicPlayer_t *player) {
    return player != NULL && player->paused;
}

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */
