/*
 * Faye Wong - Hong Dou / Red Bean.
 * Rebuilt from the local piastudy PDF score "hong dou.pdf".
 * This is a monophonic PWM buzzer reduction of the PDF right-hand top
 * voice, not the older A-major hand-written melody approximation.
 * Tempo follows the score marking at quarter = 56 BPM.
 */
#define HD_UNIT_MS 268U
#define HD_T(note, octave, units) {note, octave, (uint16_t)(HD_UNIT_MS * (units))}
#define HD_R(units) {NOTE_REST, 0, (uint16_t)(HD_UNIT_MS * (units))}

const Tone_t HONG_DOU[] = {
    /* measure 1 */
    HD_R(4U), HD_T(NOTE_E, 5, 2U), HD_T(NOTE_G, 4, 1U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_G, 5, 1U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_C, 5, 4U),
    HD_T(NOTE_B, 4, 2U),
    /* measure 2 */
    HD_T(NOTE_C, 5, 2U), HD_T(NOTE_A, 4, 1U), HD_T(NOTE_G, 4, 1U), HD_T(NOTE_G, 4, 1U),
    HD_T(NOTE_D, 4, 1U), HD_T(NOTE_G, 4, 1U), HD_T(NOTE_E, 4, 2U), HD_T(NOTE_E, 4, 7U),
    /* measure 3 */
    HD_T(NOTE_E, 5, 2U), HD_T(NOTE_G, 4, 1U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_G, 5, 1U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_C, 5, 2U), HD_T(NOTE_E, 5, 2U),
    HD_T(NOTE_A, 5, 2U), HD_T(NOTE_B, 5, 4U),
    /* measure 4 */
    HD_T(NOTE_C, 6, 9U), HD_T(NOTE_D, 6, 2U), HD_T(NOTE_G, 4, 2U), HD_T(NOTE_A, 4, 2U),
    /* measure 5 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_C, 5, 3U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_G, 4, 2U), HD_T(NOTE_A, 4, 3U),
    /* measure 6 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_E, 5, 3U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_G, 4, 2U), HD_T(NOTE_A, 4, 4U),
    /* measure 7 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_C, 5, 2U), HD_T(NOTE_C, 5, 1U),
    HD_T(NOTE_A, 4, 1U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_E, 5, 2U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_C, 5, 3U),
    /* measure 8 */
    HD_T(NOTE_D, 5, 1U), HD_T(NOTE_C, 5, 2U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_C, 5, 1U),
    HD_T(NOTE_G, 4, 1U), HD_T(NOTE_A, 4, 1U), HD_T(NOTE_G, 4, 1U), HD_T(NOTE_G, 4, 4U),
    HD_T(NOTE_G, 4, 2U), HD_T(NOTE_A, 4, 2U),
    /* measure 9 */
    HD_T(NOTE_C, 5, 1U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_C, 5, 2U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_D, 5, 1U), HD_T(NOTE_C, 5, 2U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_C, 5, 1U), HD_T(NOTE_B, 4, 1U), HD_T(NOTE_G, 4, 2U), HD_T(NOTE_A, 4, 3U),
    /* measure 10 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_E, 5, 2U), HD_T(NOTE_D, 5, 3U), HD_T(NOTE_G, 4, 2U), HD_T(NOTE_A, 4, 2U),
    /* measure 11 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_C, 5, 1U),
    HD_T(NOTE_A, 4, 2U), HD_T(NOTE_D, 5, 3U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_D, 5, 1U), HD_T(NOTE_C, 5, 3U),
    /* measure 12 */
    HD_T(NOTE_A, 5, 2U), HD_T(NOTE_G, 5, 2U), HD_T(NOTE_G, 5, 1U), HD_T(NOTE_G, 5, 1U),
    HD_T(NOTE_D, 5, 1U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_D, 5, 2U),
    HD_T(NOTE_C, 5, 2U), HD_T(NOTE_D, 5, 3U),
    /* measure 13 */
    HD_T(NOTE_C, 5, 1U), HD_T(NOTE_G, 5, 2U), HD_T(NOTE_C, 5, 2U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_E, 5, 1U), HD_T(NOTE_E, 5, 3U), HD_T(NOTE_E, 5, 2U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_C, 5, 2U),
    /* measure 14 */
    HD_T(NOTE_E, 5, 1U), HD_T(NOTE_C, 6, 2U), HD_T(NOTE_B, 5, 2U), HD_T(NOTE_A, 5, 2U),
    HD_T(NOTE_E, 5, 4U), HD_T(NOTE_B, 5, 1U), HD_T(NOTE_A, 5, 2U), HD_T(NOTE_G, 5, 2U),
    /* measure 15 */
    HD_T(NOTE_A, 5, 3U), HD_T(NOTE_A, 5, 1U), HD_T(NOTE_G, 5, 1U), HD_T(NOTE_F, 5, 2U),
    HD_T(NOTE_G, 5, 3U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 1U),
    HD_T(NOTE_D, 5, 3U),
    /* measure 16 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 2U), HD_T(NOTE_A, 4, 1U), HD_T(NOTE_E, 5, 1U),
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_D, 5, 3U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_G, 5, 2U),
    /* measure 17 */
    HD_T(NOTE_G, 5, 3U), HD_T(NOTE_C, 5, 2U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_E, 5, 1U),
    HD_T(NOTE_E, 5, 3U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_C, 5, 2U),
    /* measure 18 */
    HD_T(NOTE_E, 5, 2U), HD_T(NOTE_C, 6, 2U), HD_T(NOTE_B, 5, 2U), HD_T(NOTE_C, 6, 3U),
    HD_T(NOTE_A, 5, 2U), HD_T(NOTE_A, 5, 2U), HD_T(NOTE_E, 6, 4U),
    /* measure 19 */
    HD_T(NOTE_D, 6, 2U), HD_T(NOTE_B, 5, 1U), HD_T(NOTE_G, 5, 1U), HD_T(NOTE_E, 5, 3U),
    HD_T(NOTE_A, 5, 3U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_C, 5, 1U),
    HD_T(NOTE_D, 5, 2U),
    /* measure 20 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_A, 5, 2U), HD_T(NOTE_A, 5, 2U), HD_T(NOTE_G, 5, 2U),
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_E, 5, 4U), HD_T(NOTE_D, 5, 2U),
    /* measure 21 */
    HD_T(NOTE_C, 5, 3U), HD_T(NOTE_F, 5, 1U), HD_T(NOTE_G, 5, 1U), HD_T(NOTE_F, 5, 2U),
    HD_T(NOTE_E, 5, 1U), HD_T(NOTE_F, 5, 1U), HD_T(NOTE_E, 5, 2U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_C, 5, 2U), HD_T(NOTE_A, 4, 3U),
    /* measure 22 */
    HD_T(NOTE_G, 4, 3U), HD_T(NOTE_G, 5, 1U), HD_T(NOTE_G, 5, 2U), HD_T(NOTE_G, 4, 2U),
    HD_T(NOTE_F, 5, 3U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 3U),
    /* measure 23 */
    HD_T(NOTE_C, 5, 2U), HD_T(NOTE_E, 4, 1U), HD_T(NOTE_F, 4, 1U), HD_T(NOTE_G, 4, 1U),
    HD_T(NOTE_F, 4, 2U), HD_T(NOTE_E, 4, 1U), HD_T(NOTE_F, 4, 2U), HD_T(NOTE_C, 5, 2U),
    HD_T(NOTE_C, 5, 2U), HD_T(NOTE_F, 4, 2U),
    /* measure 24 */
    HD_T(NOTE_G, 4, 3U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_D, 4, 2U),
    HD_T(NOTE_F, 4, 3U), HD_T(NOTE_G, 4, 2U), HD_T(NOTE_A, 4, 4U),
    /* measure 25 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_C, 5, 2U), HD_T(NOTE_D, 5, 4U), HD_T(NOTE_G, 4, 2U), HD_T(NOTE_A, 4, 2U),
    /* measure 26 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_E, 5, 3U), HD_T(NOTE_D, 5, 3U), HD_T(NOTE_G, 4, 2U), HD_T(NOTE_A, 4, 3U),
    /* measure 27 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_C, 5, 1U),
    HD_T(NOTE_A, 4, 2U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_E, 5, 2U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_D, 5, 1U), HD_T(NOTE_C, 5, 4U),
    /* measure 28 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_C, 5, 1U),
    HD_T(NOTE_G, 4, 1U), HD_T(NOTE_A, 4, 1U), HD_T(NOTE_G, 4, 1U), HD_T(NOTE_G, 4, 3U),
    HD_T(NOTE_G, 4, 2U), HD_T(NOTE_A, 4, 2U),
    /* measure 29 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_C, 5, 2U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_D, 5, 4U), HD_T(NOTE_G, 4, 1U),
    HD_T(NOTE_A, 4, 3U),
    /* measure 30 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_E, 5, 3U), HD_T(NOTE_D, 5, 4U), HD_T(NOTE_G, 4, 1U), HD_T(NOTE_A, 4, 4U),
    /* measure 31 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_C, 5, 1U),
    HD_T(NOTE_A, 4, 2U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_D, 5, 1U), HD_T(NOTE_C, 5, 3U),
    /* measure 32 */
    HD_T(NOTE_A, 5, 2U), HD_T(NOTE_G, 5, 1U), HD_T(NOTE_G, 5, 1U), HD_T(NOTE_G, 5, 1U),
    HD_T(NOTE_D, 5, 1U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_G, 4, 1U), HD_T(NOTE_B, 4, 1U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_B, 4, 1U),
    HD_T(NOTE_C, 5, 1U), HD_T(NOTE_D, 5, 4U),
    /* measure 33 */
    HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_F, 5, 2U),
    HD_T(NOTE_C, 6, 2U), HD_T(NOTE_D, 6, 2U), HD_T(NOTE_G, 6, 5U),
    /* measure 34 */
    HD_T(NOTE_G, 6, 3U), HD_T(NOTE_C, 6, 2U), HD_T(NOTE_D, 6, 1U), HD_T(NOTE_E, 6, 1U),
    HD_T(NOTE_E, 6, 3U), HD_T(NOTE_E, 6, 1U), HD_T(NOTE_D, 6, 1U), HD_T(NOTE_D, 6, 1U),
    HD_T(NOTE_C, 6, 2U),
    /* measure 35 */
    HD_T(NOTE_E, 6, 1U), HD_T(NOTE_C, 7, 2U), HD_T(NOTE_B, 6, 2U), HD_T(NOTE_A, 6, 1U),
    HD_T(NOTE_E, 6, 1U), HD_T(NOTE_E, 6, 4U), HD_T(NOTE_B, 5, 1U), HD_T(NOTE_A, 5, 2U),
    HD_T(NOTE_G, 5, 3U),
    /* measure 36 */
    HD_T(NOTE_A, 5, 3U), HD_T(NOTE_A, 5, 1U), HD_T(NOTE_G, 5, 1U), HD_T(NOTE_F, 5, 2U),
    HD_T(NOTE_G, 5, 3U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_C, 5, 1U),
    HD_T(NOTE_D, 5, 2U),
    /* measure 37 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 2U), HD_T(NOTE_A, 4, 1U), HD_T(NOTE_E, 5, 2U),
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_D, 5, 3U), HD_T(NOTE_C, 6, 2U), HD_T(NOTE_D, 6, 2U),
    /* measure 38 */
    HD_T(NOTE_G, 6, 3U), HD_T(NOTE_C, 6, 2U), HD_T(NOTE_D, 6, 1U), HD_T(NOTE_E, 6, 1U),
    HD_T(NOTE_E, 6, 3U), HD_T(NOTE_E, 6, 1U), HD_T(NOTE_D, 6, 2U), HD_T(NOTE_D, 6, 1U),
    HD_T(NOTE_C, 6, 3U),
    /* measure 39 */
    HD_T(NOTE_E, 6, 2U), HD_T(NOTE_C, 7, 2U), HD_T(NOTE_B, 6, 2U), HD_T(NOTE_C, 7, 1U),
    HD_T(NOTE_A, 6, 4U), HD_T(NOTE_A, 5, 2U), HD_T(NOTE_E, 6, 2U),
    /* measure 40 */
    HD_T(NOTE_D, 6, 2U), HD_T(NOTE_B, 5, 1U), HD_T(NOTE_G, 5, 2U), HD_T(NOTE_E, 5, 1U),
    HD_T(NOTE_A, 5, 4U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 1U),
    HD_T(NOTE_D, 5, 2U),
    /* measure 41 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_A, 5, 1U), HD_T(NOTE_A, 5, 2U), HD_T(NOTE_G, 5, 2U),
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_E, 5, 3U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 4U),
    /* measure 42 */
    HD_T(NOTE_C, 5, 5U), HD_T(NOTE_C, 6, 1U), HD_T(NOTE_D, 6, 2U), HD_T(NOTE_A, 5, 1U),
    HD_T(NOTE_G, 5, 1U), HD_T(NOTE_A, 5, 1U), HD_T(NOTE_G, 6, 1U), HD_T(NOTE_E, 6, 2U),
    /* measure 43 */
    HD_T(NOTE_D, 6, 4U), HD_T(NOTE_C, 5, 2U), HD_T(NOTE_C, 6, 2U), HD_T(NOTE_B, 5, 1U),
    HD_T(NOTE_C, 6, 2U), HD_T(NOTE_B, 5, 1U), HD_T(NOTE_G, 5, 2U), HD_T(NOTE_E, 5, 2U),
    /* measure 44 */
    HD_T(NOTE_A, 5, 6U), HD_T(NOTE_B, 5, 1U), HD_T(NOTE_C, 6, 1U), HD_T(NOTE_F, 6, 1U),
    HD_T(NOTE_E, 6, 2U), HD_T(NOTE_D, 6, 1U), HD_T(NOTE_D, 6, 1U), HD_T(NOTE_E, 6, 1U),
    HD_T(NOTE_F, 6, 4U),
    /* measure 45 */
    HD_T(NOTE_G, 6, 4U), HD_T(NOTE_C, 6, 3U), HD_T(NOTE_D, 6, 3U), HD_T(NOTE_G, 6, 5U),
    /* measure 46 */
    HD_T(NOTE_G, 6, 4U), HD_T(NOTE_C, 6, 1U), HD_T(NOTE_D, 6, 1U), HD_T(NOTE_E, 6, 1U),
    HD_T(NOTE_E, 6, 3U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_C, 5, 2U),
    /* measure 47 */
    HD_T(NOTE_E, 5, 2U), HD_T(NOTE_C, 6, 2U), HD_T(NOTE_B, 5, 2U), HD_T(NOTE_A, 5, 1U),
    HD_T(NOTE_E, 5, 1U), HD_T(NOTE_E, 5, 3U), HD_T(NOTE_B, 5, 2U), HD_T(NOTE_A, 5, 1U),
    HD_T(NOTE_G, 5, 3U),
    /* measure 48 */
    HD_T(NOTE_A, 5, 3U), HD_T(NOTE_A, 5, 1U), HD_T(NOTE_G, 5, 1U), HD_T(NOTE_F, 5, 2U),
    HD_T(NOTE_G, 5, 3U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_C, 5, 1U),
    HD_T(NOTE_D, 5, 2U),
    /* measure 49 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 2U), HD_T(NOTE_A, 4, 2U), HD_T(NOTE_E, 5, 2U),
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 2U), HD_T(NOTE_D, 5, 2U),
    /* measure 50 */
    HD_T(NOTE_G, 5, 3U), HD_T(NOTE_C, 5, 2U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_E, 5, 1U),
    HD_T(NOTE_E, 5, 3U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_D, 5, 1U),
    HD_T(NOTE_C, 5, 4U),
    /* measure 51 */
    HD_T(NOTE_E, 5, 1U), HD_T(NOTE_C, 6, 2U), HD_T(NOTE_B, 5, 2U), HD_T(NOTE_C, 6, 2U),
    HD_T(NOTE_A, 5, 3U), HD_T(NOTE_A, 5, 1U), HD_T(NOTE_E, 6, 3U),
    /* measure 52 */
    HD_T(NOTE_D, 6, 2U), HD_T(NOTE_B, 5, 2U), HD_T(NOTE_G, 5, 1U), HD_T(NOTE_E, 5, 1U),
    HD_T(NOTE_A, 5, 4U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 1U),
    HD_T(NOTE_D, 5, 2U),
    /* measure 53 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_A, 5, 2U), HD_T(NOTE_A, 5, 2U), HD_T(NOTE_G, 5, 1U),
    HD_T(NOTE_D, 5, 1U), HD_T(NOTE_D, 5, 2U), HD_T(NOTE_E, 5, 2U), HD_T(NOTE_E, 5, 5U),
    /* measure 54 */
    HD_T(NOTE_D, 5, 2U), HD_T(NOTE_C, 5, 2U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_G, 5, 1U),
    HD_T(NOTE_D, 5, 1U), HD_T(NOTE_C, 5, 2U), HD_T(NOTE_C, 5, 4U), HD_T(NOTE_B, 4, 2U),
    /* measure 55 */
    HD_T(NOTE_C, 5, 1U), HD_T(NOTE_B, 4, 1U), HD_T(NOTE_A, 4, 1U), HD_T(NOTE_G, 4, 1U),
    HD_T(NOTE_G, 4, 1U), HD_T(NOTE_D, 4, 1U), HD_T(NOTE_G, 4, 2U), HD_T(NOTE_E, 4, 1U),
    HD_T(NOTE_E, 4, 7U),
    /* measure 56 */
    HD_T(NOTE_E, 5, 2U), HD_T(NOTE_E, 5, 1U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_G, 5, 1U),
    HD_T(NOTE_D, 5, 1U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_E, 5, 2U),
    HD_T(NOTE_A, 4, 1U), HD_T(NOTE_C, 5, 1U), HD_T(NOTE_D, 5, 1U), HD_T(NOTE_F, 5, 1U),
    HD_T(NOTE_G, 5, 1U), HD_T(NOTE_A, 5, 3U),
    /* measure 57 */
    HD_T(NOTE_A, 6, 4U), HD_T(NOTE_C, 4, 1U), HD_T(NOTE_F, 4, 1U), HD_T(NOTE_G, 4, 1U),
    HD_T(NOTE_A, 4, 1U), HD_T(NOTE_C, 5, 3U), HD_T(NOTE_G, 5, 34U)
};

/* Extracted 493 note onsets from 59 PDF measures; total duration about 253.0 seconds. */
#undef HD_R
#undef HD_T
#undef HD_UNIT_MS
