#include "module/cloudmusic/cloudmusic.h"

#include <cmsis_os2.h>

#define ARRAY_LEN(arr) (sizeof(arr) / sizeof((arr)[0]))

#if ((CLOUDMUSIC_FEATURES &                                             \
      ~(CLOUDMUSIC_FEATURE_STARTUP_MUSIC |                              \
        CLOUDMUSIC_FEATURE_BREATH_RESPONSE | CLOUDMUSIC_FEATURE_MUSIC_LOOP)) != 0U)
#error "CLOUDMUSIC_FEATURES contains unsupported feature bits"
#endif

#define MUSIC_TONE_GAP_MS 3U
#define CLOUDMUSIC_STICK_TRIGGER_THRESHOLD 0.75f
#define CLOUDMUSIC_STICK_RELEASE_THRESHOLD 0.30f
#define FLOWER_DANCE_TONE_GAP_MS 0U
#define TORI_NO_UTA_TONE_GAP_MS 0U
#define GU_SHI_XI_NI_TONE_GAP_MS 0U
#define SENBONZAKURA_TONE_GAP_MS 0U
#define ER_QUAN_YING_YUE_TONE_GAP_MS 0U
#define YI_BU_ZHI_YAO_TONE_GAP_MS 0U
#define RENAI_CIRCULATION_TONE_GAP_MS 0U
#define FUYU_NO_HANA_TONE_GAP_MS 0U
#define HAPPY_BIRTHDAY_TONE_GAP_MS 8U
#define HONG_DOU_TONE_GAP_MS 0U
#define YUAN_YU_CHOU_TONE_GAP_MS 0U
#define MOONLIGHT_SONATA_TONE_GAP_MS 1U
#define MOONLIGHT_SONATA_2ND_TONE_GAP_MS 0U
#define MOONLIGHT_SONATA_3RD_TONE_GAP_MS 0U
#define MERRY_CHRISTMAS_MR_LAWRENCE_TONE_GAP_MS 1U
#define HAO_YUN_LAI_TONE_GAP_MS 0U
#define CROATIAN_RHAPSODY_TONE_GAP_MS 0U
#define SUGAR_PLUM_FAIRY_TONE_GAP_MS 0U
#define JIAO_HUAN_YU_SHENG_TONE_GAP_MS 0U
#define ZOU_ZOU_TONE_GAP_MS 0U
#define XING_CUN_ZHE_TONE_GAP_MS 0U
#define BUMBLEBEE_TEMPO_BPM 165U
#define BUMBLEBEE_SIXTEENTH_MS ((60000U + (BUMBLEBEE_TEMPO_BPM * 2U)) / (BUMBLEBEE_TEMPO_BPM * 4U))
#define BUMBLEBEE_TONE_GAP_MS 0U
#define BB16(note, octave) {note, octave, BUMBLEBEE_SIXTEENTH_MS}
#define BBR16 {NOTE_REST, 0, BUMBLEBEE_SIXTEENTH_MS}

static const MUSIC_t cloudmusic_default_playlist[] = {
  MUSIC_GU_SHI_XI_NI,
  MUSIC_JIAO_HUAN_YU_SHENG,
  MUSIC_ZOU_ZOU,
  MUSIC_XING_CUN_ZHE,  
  MUSIC_SUGAR_PLUM_FAIRY,
  MUSIC_STARTUP_BUMBLEBEE,
  MUSIC_CROATIAN_RHAPSODY,
  MUSIC_MOONLIGHT_SONATA,
  MUSIC_MOONLIGHT_SONATA_2ND,
  MUSIC_MOONLIGHT_SONATA_3RD,
  MUSIC_FUR_ELISE,
  MUSIC_MERRY_CHRISTMAS_MR_LAWRENCE,
  MUSIC_HAPPY_DOU_DI_ZHU_MELODY,
  MUSIC_YI_BU_ZHI_YAO,
  MUSIC_RENAI_CIRCULATION,
  MUSIC_FUYU_NO_HANA,
  MUSIC_HAO_YUN_LAI,
  MUSIC_HAPPY_BIRTHDAY,
  MUSIC_SENBONZAKURA,
  MUSIC_HONG_DOU,
  MUSIC_ER_QUAN_YING_YUE,
  MUSIC_YUAN_YU_CHOU,
  MUSIC_TORI_NO_UTA,
  MUSIC_FLOWER_DANCE,
  MUSIC_SHUN,
  MUSIC_JUE_BIE_SHU,
};



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

#include "module/cloudmusic/scores/buzzer_fur_elise.inc"
#include "module/cloudmusic/scores/buzzer_jue_bie_shu.inc"
#include "module/cloudmusic/scores/buzzer_hong_dou.inc"
#include "module/cloudmusic/scores/buzzer_shun.inc"
#include "module/cloudmusic/scores/buzzer_yuan_yu_chou.inc"
#include "module/cloudmusic/scores/buzzer_dream_wedding.inc"
#include "module/cloudmusic/scores/buzzer_flower_dance.inc"
#include "module/cloudmusic/scores/buzzer_tori_no_uta.inc"
#include "module/cloudmusic/scores/buzzer_gu_shi_xi_ni.inc"
#include "module/cloudmusic/scores/buzzer_senbonzakura.inc"
#include "module/cloudmusic/scores/buzzer_er_quan_ying_yue.inc"
#include "module/cloudmusic/scores/buzzer_yi_bu_zhi_yao.inc"
#include "module/cloudmusic/scores/buzzer_renai_circulation.inc"
#include "module/cloudmusic/scores/buzzer_fuyu_no_hana.inc"
#include "module/cloudmusic/scores/buzzer_happy_birthday.inc"
#include "module/cloudmusic/scores/buzzer_moonlight_sonata.inc"
#include "module/cloudmusic/scores/buzzer_moonlight_sonata_2.inc"
#include "module/cloudmusic/scores/buzzer_moonlight_sonata_3rd.inc"
#include "module/cloudmusic/scores/buzzer_merry_christmas_mr_lawrence.inc"
#include "module/cloudmusic/scores/buzzer_hao_yun_lai.inc"
#include "module/cloudmusic/scores/buzzer_croatian_rhapsody.inc"
#include "module/cloudmusic/scores/buzzer_sugar_plum_fairy.inc"
#include "module/cloudmusic/scores/buzzer_jiao_huan_yu_sheng.inc"
#include "module/cloudmusic/scores/buzzer_zou_zou.inc"
#include "module/cloudmusic/scores/buzzer_xing_cun_zhe.inc"

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
} CloudMusicData_t;

static uint32_t CloudMusic_PlayerMsToTicks(uint32_t ms) {
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

static bool CloudMusic_PlayerTickReached(uint32_t now_tick, uint32_t target_tick) {
    return (int32_t)(now_tick - target_tick) >= 0;
}

static int8_t CloudMusic_GetMusicData(MUSIC_t music, CloudMusicData_t *data) {
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
        case MUSIC_SENBONZAKURA:
            data->melody = SENBONZAKURA;
            data->melody_length = sizeof(SENBONZAKURA) / sizeof(Tone_t);
            data->tone_gap_ms = SENBONZAKURA_TONE_GAP_MS;
            break;
        case MUSIC_ER_QUAN_YING_YUE:
            data->melody = ER_QUAN_YING_YUE;
            data->melody_length = sizeof(ER_QUAN_YING_YUE) / sizeof(Tone_t);
            data->tone_gap_ms = ER_QUAN_YING_YUE_TONE_GAP_MS;
            break;
        case MUSIC_YI_BU_ZHI_YAO:
            data->melody = YI_BU_ZHI_YAO;
            data->melody_length = sizeof(YI_BU_ZHI_YAO) / sizeof(Tone_t);
            data->tone_gap_ms = YI_BU_ZHI_YAO_TONE_GAP_MS;
            break;
        case MUSIC_RENAI_CIRCULATION:
            data->melody = RENAI_CIRCULATION;
            data->melody_length = sizeof(RENAI_CIRCULATION) / sizeof(Tone_t);
            data->tone_gap_ms = RENAI_CIRCULATION_TONE_GAP_MS;
            break;
        case MUSIC_FUYU_NO_HANA:
            data->melody = FUYU_NO_HANA;
            data->melody_length = sizeof(FUYU_NO_HANA) / sizeof(Tone_t);
            data->tone_gap_ms = FUYU_NO_HANA_TONE_GAP_MS;
            break;
        case MUSIC_HAPPY_BIRTHDAY:
            data->melody = HAPPY_BIRTHDAY;
            data->melody_length = sizeof(HAPPY_BIRTHDAY) / sizeof(Tone_t);
            data->tone_gap_ms = HAPPY_BIRTHDAY_TONE_GAP_MS;
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
        case MUSIC_HAO_YUN_LAI:
            data->melody = HAO_YUN_LAI;
            data->melody_length = sizeof(HAO_YUN_LAI) / sizeof(Tone_t);
            data->tone_gap_ms = HAO_YUN_LAI_TONE_GAP_MS;
            break;
        case MUSIC_CROATIAN_RHAPSODY:
            data->melody = CROATIAN_RHAPSODY;
            data->melody_length = sizeof(CROATIAN_RHAPSODY) / sizeof(Tone_t);
            data->tone_gap_ms = CROATIAN_RHAPSODY_TONE_GAP_MS;
            break;
        case MUSIC_JIAO_HUAN_YU_SHENG:
            data->melody = JIAO_HUAN_YU_SHENG;
            data->melody_length = sizeof(JIAO_HUAN_YU_SHENG) / sizeof(Tone_t);
            data->tone_gap_ms = JIAO_HUAN_YU_SHENG_TONE_GAP_MS;
            break;
        case MUSIC_ZOU_ZOU:
            data->melody = ZOU_ZOU;
            data->melody_length = sizeof(ZOU_ZOU) / sizeof(Tone_t);
            data->tone_gap_ms = ZOU_ZOU_TONE_GAP_MS;
            break;
        case MUSIC_XING_CUN_ZHE:
            data->melody = XING_CUN_ZHE;
            data->melody_length = sizeof(XING_CUN_ZHE) / sizeof(Tone_t);
            data->tone_gap_ms = XING_CUN_ZHE_TONE_GAP_MS;
            break;
        case MUSIC_SUGAR_PLUM_FAIRY:
            data->melody = SUGAR_PLUM_FAIRY;
            data->melody_length = sizeof(SUGAR_PLUM_FAIRY) / sizeof(Tone_t);
            data->tone_gap_ms = SUGAR_PLUM_FAIRY_TONE_GAP_MS;
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

int8_t CloudMusic_PlayMusic(BUZZER_t *buzzer, MUSIC_t music) {
    if (buzzer == NULL || !buzzer->header.online) {
        return DEVICE_ERR;
    }

    CloudMusicData_t data;
    if (CloudMusic_GetMusicData(music, &data) != DEVICE_OK) {
        return DEVICE_ERR;
    }

    for (size_t i = 0; i < data.melody_length; i++) {
        if (BUZZER_PlayTone(buzzer, data.melody[i].note,
                                data.melody[i].octave,
                                data.melody[i].duration_ms,
                                data.tone_gap_ms) != DEVICE_OK) {
            BUZZER_Stop(buzzer);
            return DEVICE_ERR;
        }
    }

    BUZZER_Stop(buzzer);
    return DEVICE_OK;
}
int8_t CloudMusic_PlayerStart(CloudMusic_Player_t *player,
                                BUZZER_t *buzzer, MUSIC_t music, bool loop,
                                uint32_t now_tick) {
    if (player == NULL || buzzer == NULL || !buzzer->header.online) {
        return DEVICE_ERR;
    }

    CloudMusicData_t data;
    if (CloudMusic_GetMusicData(music, &data) != DEVICE_OK) {
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
    return CloudMusic_PlayerUpdate(player, buzzer, now_tick);
}

int8_t CloudMusic_PlayerUpdate(CloudMusic_Player_t *player,
                                 BUZZER_t *buzzer, uint32_t now_tick) {
    if (player == NULL || buzzer == NULL || !buzzer->header.online) {
        return DEVICE_ERR;
    }

    if (!player->active || player->paused) {
        return DEVICE_OK;
    }

    if ((player->waiting_tone || player->waiting_gap) &&
        !CloudMusic_PlayerTickReached(now_tick, player->next_tick)) {
        return DEVICE_OK;
    }

    if (player->waiting_tone) {
        BUZZER_Stop(buzzer);
        player->waiting_tone = false;

        if (player->tone_gap_ms > 0U) {
            player->waiting_gap = true;
            player->next_tick =
                now_tick + CloudMusic_PlayerMsToTicks(player->tone_gap_ms);
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
    player->next_tick = now_tick + CloudMusic_PlayerMsToTicks(tone->duration_ms);
    return DEVICE_OK;
}

void CloudMusic_PlayerStop(CloudMusic_Player_t *player, BUZZER_t *buzzer) {
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

void CloudMusic_PlayerSilence(CloudMusic_Player_t *player,
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

void CloudMusic_PlayerSetPaused(CloudMusic_Player_t *player,
                                  BUZZER_t *buzzer, bool paused,
                                  uint32_t now_tick) {
    if (player == NULL) {
        return;
    }

    if (paused) {
        if (!player->paused) {
            CloudMusic_PlayerSilence(player, buzzer);
        }
        player->paused = true;
        return;
    }

    player->paused = false;
    player->next_tick = now_tick;
}

bool CloudMusic_PlayerIsActive(const CloudMusic_Player_t *player) {
    return player != NULL && player->active;
}

bool CloudMusic_PlayerIsPaused(const CloudMusic_Player_t *player) {
    return player != NULL && player->paused;
}

static bool CloudMusic_AxisReleased(float value) {
  return value > -CLOUDMUSIC_STICK_RELEASE_THRESHOLD &&
         value < CLOUDMUSIC_STICK_RELEASE_THRESHOLD;
}

static bool CloudMusic_StickReleased(const CloudMusic_Input_t *input) {
  return input == NULL ||
         (CloudMusic_AxisReleased(input->ch_r_x) &&
          CloudMusic_AxisReleased(input->ch_r_y));
}

static bool CloudMusic_ControlModeActive(const CloudMusic_Input_t *input) {
  return input != NULL && input->rc_online && input->sw_l == DR16_SW_UP &&
         input->sw_r == DR16_SW_MID;
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
    CloudMusic_PlayerStop(&cloudmusic->player, cloudmusic->buzzer);
    return DEVICE_ERR;
  }

  if (cloudmusic->playlist_index >= cloudmusic->playlist_length) {
    cloudmusic->playlist_index = 0U;
  }

  int8_t ret = CloudMusic_PlayerStart(
      &cloudmusic->player, cloudmusic->buzzer,
      cloudmusic->playlist[cloudmusic->playlist_index], false, now_tick);
  if (ret != DEVICE_OK) {
    cloudmusic->playlist_index =
        (cloudmusic->playlist_index + 1U) % cloudmusic->playlist_length;
    ret = CloudMusic_PlayerStart(&cloudmusic->player, cloudmusic->buzzer,
                                  cloudmusic->playlist[cloudmusic->playlist_index],
                                  false, now_tick);
  }

  cloudmusic->user_paused = should_pause;
  if (cloudmusic->user_paused) {
    CloudMusic_PlayerSetPaused(&cloudmusic->player, cloudmusic->buzzer, true,
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
    if (cloudmusic->start_music_loop_paused) {
      cloudmusic->user_paused = true;
      return;
    }
    (void)CloudMusic_StartPlaylistTrack(cloudmusic, now_tick, false);
  }
}

static void CloudMusic_HandleStickControl(CloudMusic_t *cloudmusic,
                                          const CloudMusic_Input_t *input,
                                          uint32_t now_tick) {
  if (cloudmusic == NULL || input == NULL) {
    return;
  }

  if (!CloudMusic_ControlModeActive(input)) {
    cloudmusic->switch_pending = false;
    return;
  }

  if (cloudmusic->switch_pending) {
    if (CloudMusic_StickReleased(input)) {
      cloudmusic->switch_pending = false;
    }
    return;
  }

  if (input->ch_r_x >= CLOUDMUSIC_STICK_TRIGGER_THRESHOLD) {
    cloudmusic->switch_pending = true;
    (void)CloudMusic_Next(cloudmusic, now_tick);
  } else if (input->ch_r_x <= -CLOUDMUSIC_STICK_TRIGGER_THRESHOLD) {
    cloudmusic->switch_pending = true;
    (void)CloudMusic_Previous(cloudmusic, now_tick);
  } else if (input->ch_r_y >= CLOUDMUSIC_STICK_TRIGGER_THRESHOLD) {
    cloudmusic->switch_pending = true;
    CloudMusic_SetPaused(cloudmusic, false, now_tick);
  } else if (input->ch_r_y <= -CLOUDMUSIC_STICK_TRIGGER_THRESHOLD) {
    cloudmusic->switch_pending = true;
    CloudMusic_SetPaused(cloudmusic, true, now_tick);
  }
}

void CloudMusic_GetDefaultConfig(CloudMusic_Config_t *config) {
  if (config == NULL) {
    return;
  }

  config->playlist = cloudmusic_default_playlist;
  config->playlist_length = ARRAY_LEN(cloudmusic_default_playlist);
  config->startup_music = CLOUDMUSIC_STARTUP_MUSIC;
  config->enable_startup_music =
      (CLOUDMUSIC_FEATURES & CLOUDMUSIC_FEATURE_STARTUP_MUSIC) != 0U;
  config->enable_music_loop =
      (CLOUDMUSIC_FEATURES & CLOUDMUSIC_FEATURE_MUSIC_LOOP) != 0U;
  config->start_music_loop_paused = false;
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
  cloudmusic->enable_startup_music = config->enable_startup_music;
  cloudmusic->enable_music_loop = config->enable_music_loop;
  cloudmusic->start_music_loop_paused = config->start_music_loop_paused;

  return DEVICE_OK;
}

int8_t CloudMusic_Start(CloudMusic_t *cloudmusic, uint32_t now_tick) {
  if (cloudmusic == NULL || cloudmusic->buzzer == NULL) {
    return DEVICE_ERR;
  }

  if (cloudmusic->enable_startup_music) {
    cloudmusic->startup_active =
        CloudMusic_PlayerStart(&cloudmusic->player, cloudmusic->buzzer,
                                cloudmusic->startup_music, false,
                                now_tick) == DEVICE_OK;
  }

  if (cloudmusic->enable_music_loop && !cloudmusic->startup_active) {
    if (cloudmusic->start_music_loop_paused) {
      cloudmusic->user_paused = true;
      return DEVICE_OK;
    }
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
      CloudMusic_HandleStickControl(cloudmusic, input, now_tick);
    } else {
      CloudMusic_ResetGesture(cloudmusic, input);
    }
  }

  if (cloudmusic->startup_active) {
    if (CloudMusic_PlayerUpdate(&cloudmusic->player, cloudmusic->buzzer,
                                 now_tick) != DEVICE_OK ||
        !CloudMusic_PlayerIsActive(&cloudmusic->player)) {
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

  if (CloudMusic_PlayerUpdate(&cloudmusic->player, cloudmusic->buzzer,
                               now_tick) != DEVICE_OK ||
      !CloudMusic_PlayerIsActive(&cloudmusic->player)) {
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
      CloudMusic_PlayerStop(&cloudmusic->player, cloudmusic->buzzer);
    }
    return DEVICE_ERR;
  }

  cloudmusic->startup_active = false;
  cloudmusic->playlist_index =
      (cloudmusic->playlist_index + 1U) % cloudmusic->playlist_length;
  cloudmusic->next_count++;
  if (cloudmusic->user_paused) {
    CloudMusic_PlayerStop(&cloudmusic->player, cloudmusic->buzzer);
    return DEVICE_OK;
  }
  return CloudMusic_StartPlaylistTrack(cloudmusic, now_tick, true);
}

int8_t CloudMusic_Previous(CloudMusic_t *cloudmusic, uint32_t now_tick) {
  if (!CloudMusic_HasPlaylist(cloudmusic)) {
    if (cloudmusic != NULL && cloudmusic->buzzer != NULL) {
      CloudMusic_PlayerStop(&cloudmusic->player, cloudmusic->buzzer);
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
  if (cloudmusic->user_paused) {
    CloudMusic_PlayerStop(&cloudmusic->player, cloudmusic->buzzer);
    return DEVICE_OK;
  }
  return CloudMusic_StartPlaylistTrack(cloudmusic, now_tick, true);
}

void CloudMusic_SetPaused(CloudMusic_t *cloudmusic, bool paused,
                          uint32_t now_tick) {
  if (cloudmusic == NULL || cloudmusic->buzzer == NULL) {
    return;
  }

  cloudmusic->startup_active = false;
  cloudmusic->user_paused = paused;
  if (!paused && !CloudMusic_PlayerIsActive(&cloudmusic->player)) {
    if (CloudMusic_StartPlaylistTrack(cloudmusic, now_tick, false) !=
        DEVICE_OK) {
      return;
    }
  }
  CloudMusic_PlayerSetPaused(&cloudmusic->player, cloudmusic->buzzer, paused,
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
  CloudMusic_PlayerStop(&cloudmusic->player, cloudmusic->buzzer);
}

void CloudMusic_Silence(CloudMusic_t *cloudmusic) {
  if (cloudmusic == NULL || cloudmusic->buzzer == NULL) {
    return;
  }

  CloudMusic_PlayerSilence(&cloudmusic->player, cloudmusic->buzzer);
}

void CloudMusic_ResetGesture(CloudMusic_t *cloudmusic,
                             const CloudMusic_Input_t *input) {
  (void)input;
  if (cloudmusic == NULL) {
    return;
  }

  cloudmusic->switch_pending = false;
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
  status->active = CloudMusic_PlayerIsActive(&cloudmusic->player);
  status->paused = cloudmusic->user_paused ||
                   CloudMusic_PlayerIsPaused(&cloudmusic->player);
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
         CloudMusic_PlayerIsActive(&cloudmusic->player);
}

bool CloudMusic_IsPaused(const CloudMusic_t *cloudmusic) {
  return cloudmusic != NULL && cloudmusic->user_paused;
}

bool CloudMusic_AllowsIdleEffect(const CloudMusic_t *cloudmusic) {
  return cloudmusic == NULL || !cloudmusic->enable_music_loop;
}
