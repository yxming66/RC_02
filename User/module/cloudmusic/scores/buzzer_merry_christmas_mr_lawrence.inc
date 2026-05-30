/*
 * Ryuichi Sakamoto - Merry Christmas Mr. Lawrence.
 * Rebuilt from the local piastudy PDF score.
 * Monophonic PWM buzzer reduction of the PDF right-hand top voice.
 * The score is in 12/8 with four flats; flat notes are encoded as
 * sharp enharmonics because NOTE_t does not define flat note names.
 * One unit is one eighth note at dotted-quarter = 105 BPM.
 */
#define MCM_EIGHTH_MS 190U
#define MCM_T(note, octave, units) {note, octave, (uint16_t)(MCM_EIGHTH_MS * (units))}
#define MCM_R(units) {NOTE_REST, 0, (uint16_t)(MCM_EIGHTH_MS * (units))}

const Tone_t MERRY_CHRISTMAS_MR_LAWRENCE[] = {
    /* measure 1 */
    MCM_R(3U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_AS, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_AS, 5, 1U), MCM_T(NOTE_F, 5, 2U),
    /* measure 2 */
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_GS, 5, 1U),
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 3U),
    /* measure 3 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_GS, 5, 1U),
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_DS, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_CS, 5, 2U),
    /* measure 4 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_DS, 5, 1U),
    MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_C, 5, 1U), MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_C, 5, 1U),
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_C, 5, 1U), MCM_T(NOTE_AS, 4, 4U),
    /* measure 5 */
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_AS, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_AS, 5, 1U),
    MCM_T(NOTE_F, 5, 2U),
    /* measure 6 */
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_GS, 5, 1U),
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 3U),
    /* measure 7 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_C, 6, 1U),
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 1U),
    MCM_T(NOTE_C, 6, 1U), MCM_T(NOTE_CS, 5, 2U),
    /* measure 8 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_C, 6, 1U), MCM_T(NOTE_DS, 5, 1U),
    MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 1U),
    MCM_T(NOTE_C, 6, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 4U),
    /* measure 9 */
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_AS, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_AS, 5, 1U),
    MCM_T(NOTE_F, 5, 2U),
    /* measure 10 */
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_GS, 5, 1U),
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 3U),
    /* measure 11 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_GS, 5, 1U),
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_DS, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_CS, 5, 2U),
    /* measure 12 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_GS, 5, 1U),
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_C, 5, 1U),
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_C, 5, 1U), MCM_T(NOTE_AS, 4, 4U),
    /* measure 13 */
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_AS, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_AS, 5, 1U),
    MCM_T(NOTE_F, 5, 2U),
    /* measure 14 */
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_GS, 5, 1U),
    MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 4U),
    /* measure 15 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_C, 6, 1U), MCM_T(NOTE_DS, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_C, 6, 1U),
    MCM_T(NOTE_DS, 5, 2U),
    /* measure 16 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_C, 6, 1U),
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_DS, 5, 1U),
    MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_C, 6, 4U),
    /* measure 17 */
    MCM_T(NOTE_DS, 4, 2U), MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_DS, 4, 4U), MCM_T(NOTE_DS, 4, 9U),
    /* measure 18 */
    MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_F, 4, 1U),
    MCM_T(NOTE_GS, 4, 1U), MCM_T(NOTE_F, 4, 2U),
    /* measure 19 */
    MCM_T(NOTE_DS, 4, 2U), MCM_T(NOTE_F, 4, 2U), MCM_T(NOTE_DS, 4, 3U), MCM_T(NOTE_CS, 4, 8U),
    /* measure 20 */
    MCM_T(NOTE_CS, 5, 2U), MCM_T(NOTE_C, 5, 2U), MCM_T(NOTE_GS, 4, 2U), MCM_T(NOTE_F, 4, 3U),
    /* measure 21 */
    MCM_T(NOTE_DS, 4, 2U), MCM_T(NOTE_F, 4, 2U), MCM_T(NOTE_DS, 4, 3U), MCM_T(NOTE_DS, 4, 7U),
    /* measure 22 */
    MCM_T(NOTE_DS, 4, 2U), MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_DS, 4, 2U), MCM_T(NOTE_F, 4, 1U),
    MCM_T(NOTE_GS, 4, 2U), MCM_T(NOTE_F, 4, 5U),
    /* measure 23 */
    MCM_T(NOTE_DS, 4, 2U), MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_CS, 4, 17U),
    /* measure 25 */
    MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_DS, 5, 7U),
    /* measure 26 */
    MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 2U), MCM_T(NOTE_F, 5, 2U),
    /* measure 27 */
    MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_CS, 5, 10U),
    /* measure 28 */
    MCM_T(NOTE_CS, 6, 2U), MCM_T(NOTE_C, 6, 1U), MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 3U),
    /* measure 29 */
    MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_DS, 5, 7U),
    /* measure 30 */
    MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 2U), MCM_T(NOTE_F, 5, 2U),
    /* measure 31 */
    MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 2U),
    MCM_T(NOTE_AS, 4, 5U),
    /* measure 32 */
    MCM_T(NOTE_AS, 4, 6U), MCM_T(NOTE_F, 4, 3U), MCM_T(NOTE_CS, 4, 6U),
    /* measure 33 */
    MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_GS, 4, 1U), MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_GS, 4, 1U),
    MCM_T(NOTE_GS, 4, 1U), MCM_T(NOTE_AS, 4, 2U), MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_C, 4, 1U),
    /* measure 34 */
    MCM_T(NOTE_AS, 4, 2U), MCM_T(NOTE_GS, 4, 1U), MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_GS, 4, 2U),
    MCM_T(NOTE_GS, 4, 1U), MCM_T(NOTE_AS, 4, 2U), MCM_T(NOTE_GS, 4, 1U), MCM_T(NOTE_G, 4, 2U),
    /* measure 35 */
    MCM_T(NOTE_F, 4, 2U), MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_F, 4, 2U), MCM_T(NOTE_DS, 4, 1U),
    MCM_T(NOTE_DS, 4, 2U), MCM_T(NOTE_F, 4, 2U), MCM_T(NOTE_F, 4, 2U),
    /* measure 36 */
    MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_DS, 4, 2U), MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_DS, 4, 2U),
    MCM_T(NOTE_DS, 4, 2U), MCM_T(NOTE_F, 4, 2U), MCM_T(NOTE_G, 4, 5U),
    /* measure 37 */
    MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_GS, 4, 1U), MCM_T(NOTE_AS, 4, 2U), MCM_T(NOTE_GS, 4, 1U),
    MCM_T(NOTE_GS, 4, 1U), MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_AS, 4, 2U),
    /* measure 38 */
    MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_GS, 4, 2U), MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_GS, 4, 1U),
    MCM_T(NOTE_GS, 4, 2U), MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_GS, 4, 2U), MCM_T(NOTE_G, 4, 2U),
    /* measure 39 */
    MCM_T(NOTE_F, 4, 2U), MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_F, 4, 3U), MCM_T(NOTE_F, 4, 2U),
    /* measure 40 */
    MCM_T(NOTE_F, 4, 2U), MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_F, 4, 2U), MCM_T(NOTE_GS, 4, 2U),
    MCM_T(NOTE_GS, 4, 9U),
    /* measure 41 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_AS, 4, 1U),
    MCM_T(NOTE_DS, 5, 6U),
    /* measure 42 */
    MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 2U), MCM_T(NOTE_F, 5, 2U),
    /* measure 43 */
    MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_CS, 5, 8U),
    /* measure 44 */
    MCM_T(NOTE_CS, 6, 2U), MCM_T(NOTE_C, 6, 2U), MCM_T(NOTE_GS, 5, 2U), MCM_T(NOTE_F, 5, 3U),
    /* measure 45 */
    MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_DS, 5, 9U),
    /* measure 46 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 2U),
    /* measure 47 */
    MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_CS, 5, 2U),
    MCM_T(NOTE_AS, 4, 5U),
    /* measure 48 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_C, 5, 2U), MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_F, 4, 1U),
    MCM_T(NOTE_DS, 4, 2U), MCM_T(NOTE_C, 4, 1U), MCM_T(NOTE_CS, 4, 2U), MCM_T(NOTE_C, 5, 2U),
    /* measure 49 */
    MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_DS, 5, 9U),
    /* measure 50 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 2U),
    /* measure 51 */
    MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_CS, 5, 7U),
    /* measure 52 */
    MCM_T(NOTE_CS, 6, 3U), MCM_T(NOTE_C, 6, 2U), MCM_T(NOTE_GS, 5, 2U), MCM_T(NOTE_F, 5, 3U),
    /* measure 53 */
    MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_AS, 4, 1U),
    MCM_T(NOTE_DS, 5, 11U),
    /* measure 54 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 2U),
    /* measure 55 */
    MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_CS, 5, 1U),
    MCM_T(NOTE_AS, 4, 4U), MCM_T(NOTE_CS, 5, 2U),
    /* measure 56 */
    MCM_T(NOTE_AS, 4, 4U), MCM_T(NOTE_GS, 4, 2U), MCM_T(NOTE_GS, 4, 2U), MCM_T(NOTE_F, 4, 2U),
    MCM_T(NOTE_DS, 4, 2U),
    /* measure 57 */
    MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_F, 4, 2U), MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_F, 4, 1U),
    MCM_T(NOTE_F, 4, 2U), MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_F, 4, 2U), MCM_T(NOTE_F, 4, 5U),
    /* measure 58 */
    MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_F, 4, 1U),
    MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_DS, 4, 2U),
    /* measure 59 */
    MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_CS, 4, 1U),
    MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_DS, 4, 1U),
    MCM_T(NOTE_CS, 4, 1U), MCM_T(NOTE_CS, 4, 1U), MCM_T(NOTE_CS, 4, 2U),
    /* measure 60 */
    MCM_T(NOTE_CS, 4, 1U), MCM_T(NOTE_C, 4, 1U), MCM_T(NOTE_CS, 4, 1U), MCM_T(NOTE_CS, 4, 1U),
    MCM_T(NOTE_C, 4, 1U), MCM_T(NOTE_CS, 4, 1U), MCM_T(NOTE_CS, 4, 2U), MCM_T(NOTE_DS, 4, 4U),
    /* measure 61 */
    MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_F, 4, 2U), MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_F, 4, 2U),
    MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_F, 4, 2U), MCM_T(NOTE_F, 4, 5U),
    /* measure 62 */
    MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_F, 4, 1U),
    MCM_T(NOTE_DS, 4, 2U), MCM_T(NOTE_AS, 4, 3U),
    /* measure 63 */
    MCM_T(NOTE_GS, 4, 1U), MCM_T(NOTE_GS, 4, 1U), MCM_T(NOTE_CS, 4, 1U), MCM_T(NOTE_GS, 4, 1U),
    MCM_T(NOTE_GS, 4, 1U), MCM_T(NOTE_CS, 4, 1U), MCM_T(NOTE_GS, 4, 1U), MCM_T(NOTE_GS, 4, 1U),
    MCM_T(NOTE_C, 5, 1U), MCM_T(NOTE_GS, 4, 3U),
    /* measure 64 */
    MCM_T(NOTE_C, 5, 1U), MCM_T(NOTE_C, 5, 1U), MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_C, 5, 1U),
    MCM_T(NOTE_C, 5, 1U), MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_CS, 5, 2U), MCM_T(NOTE_DS, 5, 4U),
    /* measure 65 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_DS, 5, 11U),
    /* measure 66 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 2U),
    /* measure 67 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_CS, 5, 9U),
    /* measure 68 */
    MCM_T(NOTE_CS, 6, 2U), MCM_T(NOTE_C, 6, 2U), MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 4U),
    /* measure 69 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_DS, 5, 11U),
    /* measure 70 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 2U),
    /* measure 71 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 2U),
    MCM_T(NOTE_AS, 4, 6U),
    /* measure 72 */
    MCM_T(NOTE_AS, 4, 3U), MCM_T(NOTE_CS, 5, 3U), MCM_T(NOTE_CS, 5, 2U), MCM_T(NOTE_DS, 5, 4U),
    /* measure 73 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_DS, 5, 11U),
    /* measure 74 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 2U),
    /* measure 75 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_CS, 5, 9U),
    /* measure 76 */
    MCM_T(NOTE_CS, 6, 3U), MCM_T(NOTE_C, 6, 1U), MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 4U),
    /* measure 77 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_DS, 5, 11U),
    /* measure 78 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 2U),
    /* measure 79 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 2U),
    MCM_T(NOTE_AS, 4, 5U),
    /* measure 80 */
    MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_GS, 4, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_C, 5, 1U),
    MCM_T(NOTE_CS, 5, 2U), MCM_T(NOTE_F, 4, 1U), MCM_T(NOTE_DS, 4, 1U), MCM_T(NOTE_C, 4, 2U),
    MCM_T(NOTE_CS, 4, 1U), MCM_T(NOTE_C, 5, 2U),
    /* measure 81 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_DS, 5, 11U),
    /* measure 82 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 2U),
    /* measure 83 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_CS, 5, 9U),
    /* measure 84 */
    MCM_T(NOTE_CS, 6, 3U), MCM_T(NOTE_C, 6, 1U), MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 4U),
    /* measure 85 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_AS, 4, 2U),
    MCM_T(NOTE_DS, 5, 11U),
    /* measure 86 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 1U),
    MCM_T(NOTE_GS, 5, 1U), MCM_T(NOTE_F, 5, 2U),
    /* measure 87 */
    MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_F, 5, 2U), MCM_T(NOTE_DS, 5, 1U), MCM_T(NOTE_CS, 5, 1U),
    MCM_T(NOTE_AS, 4, 5U), MCM_T(NOTE_CS, 5, 2U),
    /* measure 88 */
    MCM_T(NOTE_AS, 4, 4U), MCM_T(NOTE_GS, 4, 1U), MCM_T(NOTE_GS, 4, 3U), MCM_T(NOTE_F, 4, 2U),
    MCM_T(NOTE_DS, 4, 2U),
    /* measure 89 */
    MCM_T(NOTE_F, 4, 17U),
    /* measure 90 */
    MCM_T(NOTE_C, 4, 1U), MCM_T(NOTE_GS, 4, 1U), MCM_T(NOTE_G, 4, 1U), MCM_T(NOTE_C, 4, 5U),
    /* measure 91 */
    MCM_T(NOTE_AS, 5, 5U), MCM_T(NOTE_AS, 4, 6U),
    /* measure 92 */
    MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_CS, 5, 1U),
    MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_C, 5, 3U), MCM_T(NOTE_F, 4, 4U),
    /* measure 93 */
    MCM_T(NOTE_F, 5, 5U), MCM_T(NOTE_AS, 4, 6U),
    /* measure 94 */
    MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_DS, 5, 2U), MCM_T(NOTE_CS, 5, 1U),
    MCM_T(NOTE_CS, 5, 1U), MCM_T(NOTE_C, 5, 3U), MCM_T(NOTE_F, 4, 3U),
    /* measure 95 */
    MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_F, 5, 5U), MCM_T(NOTE_F, 4, 10U),
    /* measure 96 */
    MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_F, 5, 3U), MCM_T(NOTE_F, 4, 5U),
    /* measure 97 */
    MCM_T(NOTE_AS, 4, 1U), MCM_T(NOTE_F, 5, 9U)
};

/* Extracted 602 note onsets from 97 PDF measures; total duration about 221.2 seconds. */
#undef MCM_R
#undef MCM_T
#undef MCM_EIGHTH_MS
