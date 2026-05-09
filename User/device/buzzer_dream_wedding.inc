/*
 * Dream Wedding / Mariage d'amour.
 * Source material: local PDF score in the Edge save directory, cross-checked against
 * its 1=Bb numbered notation. The piano texture is reduced to one buzzer line:
 * right-hand melody first, with the opening accompaniment folded into the intro.
 */
#define MDA_UNIT_MS 208U
#define MDA_T(note, octave, units) {note, octave, (uint16_t)(MDA_UNIT_MS * (units))}
#define MDA_R(units) {NOTE_REST, 0, (uint16_t)(MDA_UNIT_MS * (units))}

#define MDA_LO1(units) MDA_T(NOTE_AS, 3, units)
#define MDA_LO3(units) MDA_T(NOTE_D, 4, units)
#define MDA_LO6(units) MDA_T(NOTE_G, 3, units)

#define MDA_H1(units) MDA_T(NOTE_AS, 4, units)
#define MDA_H2(units) MDA_T(NOTE_C, 5, units)
#define MDA_H3(units) MDA_T(NOTE_D, 5, units)
#define MDA_H4(units) MDA_T(NOTE_DS, 5, units)
#define MDA_H5(units) MDA_T(NOTE_F, 5, units)
#define MDA_HS5(units) MDA_T(NOTE_FS, 5, units)
#define MDA_H6(units) MDA_T(NOTE_G, 5, units)
#define MDA_H7(units) MDA_T(NOTE_A, 5, units)

#define MDA_X2(units) MDA_T(NOTE_C, 6, units)
#define MDA_X3(units) MDA_T(NOTE_D, 6, units)
#define MDA_X4(units) MDA_T(NOTE_DS, 6, units)

#define MDA_INTRO_4 \
    MDA_LO6(2), MDA_LO3(2), MDA_LO1(2), MDA_LO3(2), \
    MDA_LO1(2), MDA_LO3(2), MDA_LO1(2), MDA_LO3(2),

#define MDA_OPEN_5 \
    MDA_R(1), MDA_H6(1), MDA_H6(1), MDA_H7(1), \
    MDA_H7(1), MDA_H1(1), MDA_H1(1), MDA_H7(1), \
    MDA_H7(1), MDA_H6(1), MDA_H6(1), MDA_H3(1), \
    MDA_H3(1), MDA_H1(1), MDA_H1(1), MDA_H6(1), \
    MDA_H6(1), MDA_H1(1), MDA_H1(1), MDA_H5(1),

#define MDA_FALL_4 \
    MDA_H5(1), MDA_H4(1), MDA_H4(1), MDA_H3(1), \
    MDA_H4(1), MDA_H5(1), MDA_H4(1), MDA_H3(1), \
    MDA_H4(4), MDA_R(4),

#define MDA_RUN_4 \
    MDA_R(1), MDA_H4(1), MDA_H4(1), MDA_H5(1), \
    MDA_H5(1), MDA_H6(1), MDA_H6(1), MDA_H7(1), \
    MDA_H7(1), MDA_H5(1), MDA_H5(1), MDA_H2(1), \
    MDA_H2(1), MDA_H4(1), MDA_H4(1), MDA_H2(1),

#define MDA_CAD_4 \
    MDA_H4(1), MDA_H3(1), MDA_H3(1), MDA_H2(1), \
    MDA_H3(1), MDA_H4(1), MDA_H3(1), MDA_H2(1), \
    MDA_H3(4), MDA_R(4),

#define MDA_CAD_3 \
    MDA_H4(1), MDA_H3(1), MDA_H3(1), MDA_H2(1), \
    MDA_H3(1), MDA_H4(1), MDA_H3(1), MDA_H2(1), \
    MDA_H3(4),

#define MDA_WAVE_4 \
    MDA_H3(2), MDA_H6(1), MDA_H1(1), \
    MDA_H2(1), MDA_H3(1), MDA_H2(1), MDA_H1(1), \
    MDA_H3(2), MDA_H6(1), MDA_H1(1), \
    MDA_H2(1), MDA_H3(1), MDA_H2(1), MDA_H1(1),

#define MDA_RISE_4 \
    MDA_H4(2), MDA_H6(1), MDA_H2(1), \
    MDA_H3(1), MDA_H4(1), MDA_H3(1), MDA_H2(1), \
    MDA_H4(2), MDA_H6(1), MDA_H2(1), \
    MDA_H3(2), MDA_H4(2),

#define MDA_HOLD_4 \
    MDA_H5(3), MDA_H6(1), MDA_H5(3), MDA_H6(1), \
    MDA_H5(2), MDA_H5(1), MDA_H6(1), \
    MDA_H5(2), MDA_H5(1), MDA_H6(1),

#define MDA_TURN_4 \
    MDA_H3(4), MDA_H3(1), MDA_H4(1), \
    MDA_H3(1), MDA_H2(1), MDA_H1(1), MDA_H7(1), \
    MDA_H6(1), MDA_HS5(1), MDA_H6(1), MDA_H7(1), \
    MDA_H6(2),

#define MDA_SECTION_A_4 \
    MDA_H1(2), MDA_H3(2), MDA_H3(2), MDA_H4(2), \
    MDA_H4(2), MDA_H2(2), MDA_H2(1), MDA_H7(1), MDA_H6(2),

#define MDA_SECTION_B_4 \
    MDA_H7(2), MDA_H2(2), MDA_H2(2), MDA_H4(2), \
    MDA_H3(2), MDA_H1(2), MDA_H1(1), MDA_H6(1), MDA_H5(2),

#define MDA_SECTION_C_4 \
    MDA_H6(2), MDA_H1(2), MDA_H1(2), MDA_H2(2), \
    MDA_H2(2), MDA_H7(2), MDA_H3(2), MDA_H2(2),

#define MDA_SECTION_D_4 \
    MDA_H3(4), MDA_H7(2), MDA_H2(2), \
    MDA_H3(2), MDA_HS5(2), MDA_H7(2), MDA_H3(2),

#define MDA_CLIMAX_CAD_4 \
    MDA_X4(1), MDA_X3(1), MDA_X3(1), MDA_X2(1), \
    MDA_X3(1), MDA_X4(1), MDA_X3(1), MDA_X2(1), \
    MDA_X3(4), MDA_R(4),

const Tone_t DREAM_WEDDING[] = {
    /* Bars 1-6 */
    MDA_INTRO_4
    MDA_INTRO_4
    MDA_OPEN_5
    MDA_FALL_4
    MDA_RUN_4
    MDA_CAD_4

    /* Bars 7-12 */
    MDA_OPEN_5
    MDA_FALL_4
    MDA_RUN_4
    MDA_CAD_3
    MDA_CAD_4
    MDA_WAVE_4

    /* Bars 13-18 */
    MDA_RISE_4
    MDA_HOLD_4
    MDA_TURN_4
    MDA_WAVE_4
    MDA_RISE_4
    MDA_HOLD_4

    /* Bars 19-24 */
    MDA_TURN_4
    MDA_TURN_4
    MDA_OPEN_5
    MDA_FALL_4
    MDA_RUN_4
    MDA_CAD_4

    /* Bars 25-30 */
    MDA_OPEN_5
    MDA_FALL_4
    MDA_RUN_4
    MDA_CAD_4
    MDA_WAVE_4
    MDA_RISE_4

    /* Bars 31-36 */
    MDA_HOLD_4
    MDA_TURN_4
    MDA_WAVE_4
    MDA_RISE_4
    MDA_HOLD_4
    MDA_TURN_4

    /* Bars 37-45 */
    MDA_SECTION_A_4
    MDA_SECTION_B_4
    MDA_SECTION_C_4
    MDA_SECTION_C_4
    MDA_SECTION_D_4
    MDA_SECTION_A_4
    MDA_SECTION_B_4
    MDA_SECTION_C_4
    MDA_H3(4), MDA_R(12),

    /* Bars 46-50 */
    MDA_OPEN_5
    MDA_FALL_4
    MDA_RUN_4
    MDA_CLIMAX_CAD_4
    MDA_R(4), MDA_H3(12)
};

#undef MDA_CLIMAX_CAD_4
#undef MDA_SECTION_D_4
#undef MDA_SECTION_C_4
#undef MDA_SECTION_B_4
#undef MDA_SECTION_A_4
#undef MDA_TURN_4
#undef MDA_HOLD_4
#undef MDA_RISE_4
#undef MDA_WAVE_4
#undef MDA_CAD_3
#undef MDA_CAD_4
#undef MDA_RUN_4
#undef MDA_FALL_4
#undef MDA_OPEN_5
#undef MDA_INTRO_4
#undef MDA_X4
#undef MDA_X3
#undef MDA_X2
#undef MDA_H7
#undef MDA_H6
#undef MDA_HS5
#undef MDA_H5
#undef MDA_H4
#undef MDA_H3
#undef MDA_H2
#undef MDA_H1
#undef MDA_LO6
#undef MDA_LO3
#undef MDA_LO1
#undef MDA_R
#undef MDA_T
#undef MDA_UNIT_MS
