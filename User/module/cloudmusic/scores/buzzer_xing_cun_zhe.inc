/*
 * JJ Lin - Xing Cun Zhe / Drifter.
 * Monophonic PWM buzzer reduction from the local PDF score
 * "xing cun zhe.pdf": B major, 4/4, quarter note ~= 69 BPM.
 *
 * The source piano score is polyphonic.  This arrangement keeps the upper
 * melody contour and folds repeated accompaniment gestures into a compact
 * single-line version that is practical for the buzzer player.
 */
#define XCZ_UNIT_MS 217U
#define XCZ(note, octave, units) {note, octave, (uint16_t)(XCZ_UNIT_MS * (units))}
#define XCZ_R(units) {NOTE_REST, 0, (uint16_t)(XCZ_UNIT_MS * (units))}

const Tone_t XING_CUN_ZHE[] = {
    /* Intro, bars 1-8 */
    XCZ(NOTE_B, 5, 1), XCZ(NOTE_CS, 6, 1), XCZ(NOTE_B, 5, 1), XCZ(NOTE_AS, 5, 1),
    XCZ(NOTE_GS, 5, 2), XCZ(NOTE_FS, 5, 1), XCZ(NOTE_DS, 5, 1),
    XCZ(NOTE_FS, 5, 4), XCZ(NOTE_DS, 5, 2), XCZ(NOTE_E, 5, 2),
    XCZ(NOTE_B, 5, 1), XCZ(NOTE_CS, 6, 1), XCZ(NOTE_B, 5, 1), XCZ(NOTE_AS, 5, 1),
    XCZ(NOTE_GS, 5, 2), XCZ(NOTE_FS, 5, 1), XCZ(NOTE_E, 5, 1),
    XCZ(NOTE_DS, 5, 4), XCZ(NOTE_B, 4, 2), XCZ(NOTE_CS, 5, 2),
    XCZ(NOTE_B, 5, 1), XCZ(NOTE_CS, 6, 1), XCZ(NOTE_DS, 6, 1), XCZ(NOTE_CS, 6, 1),
    XCZ(NOTE_B, 5, 2), XCZ(NOTE_AS, 5, 1), XCZ(NOTE_GS, 5, 1),
    XCZ(NOTE_FS, 5, 6), XCZ_R(2),
    XCZ(NOTE_DS, 5, 2), XCZ(NOTE_FS, 5, 2), XCZ(NOTE_GS, 5, 2), XCZ(NOTE_B, 5, 2),
    XCZ(NOTE_AS, 5, 2), XCZ(NOTE_GS, 5, 2), XCZ(NOTE_FS, 5, 4),

    /* Verse A */
    XCZ(NOTE_B, 4, 2), XCZ(NOTE_CS, 5, 2), XCZ(NOTE_DS, 5, 2), XCZ(NOTE_FS, 5, 2),
    XCZ(NOTE_GS, 5, 2), XCZ(NOTE_FS, 5, 2), XCZ(NOTE_DS, 5, 4),
    XCZ(NOTE_CS, 5, 2), XCZ(NOTE_DS, 5, 2), XCZ(NOTE_E, 5, 2), XCZ(NOTE_FS, 5, 2),
    XCZ(NOTE_DS, 5, 4), XCZ(NOTE_B, 4, 4),
    XCZ(NOTE_B, 4, 2), XCZ(NOTE_CS, 5, 2), XCZ(NOTE_DS, 5, 2), XCZ(NOTE_FS, 5, 2),
    XCZ(NOTE_GS, 5, 3), XCZ(NOTE_FS, 5, 1), XCZ(NOTE_E, 5, 2), XCZ(NOTE_DS, 5, 4),
    XCZ(NOTE_CS, 5, 2), XCZ(NOTE_B, 4, 2), XCZ(NOTE_CS, 5, 2), XCZ(NOTE_DS, 5, 2),
    XCZ(NOTE_FS, 5, 6), XCZ_R(2),

    /* Verse B */
    XCZ(NOTE_DS, 5, 2), XCZ(NOTE_E, 5, 2), XCZ(NOTE_FS, 5, 2), XCZ(NOTE_GS, 5, 2),
    XCZ(NOTE_FS, 5, 2), XCZ(NOTE_DS, 5, 2), XCZ(NOTE_CS, 5, 4),
    XCZ(NOTE_B, 4, 2), XCZ(NOTE_CS, 5, 2), XCZ(NOTE_DS, 5, 2), XCZ(NOTE_E, 5, 2),
    XCZ(NOTE_FS, 5, 4), XCZ(NOTE_DS, 5, 4),
    XCZ(NOTE_FS, 5, 2), XCZ(NOTE_GS, 5, 2), XCZ(NOTE_AS, 5, 2), XCZ(NOTE_B, 5, 2),
    XCZ(NOTE_CS, 6, 3), XCZ(NOTE_B, 5, 1), XCZ(NOTE_AS, 5, 2), XCZ(NOTE_GS, 5, 4),
    XCZ(NOTE_FS, 5, 2), XCZ(NOTE_E, 5, 2), XCZ(NOTE_DS, 5, 2), XCZ(NOTE_CS, 5, 2),
    XCZ(NOTE_B, 4, 6), XCZ_R(2),

    /* Pre-chorus */
    XCZ(NOTE_FS, 5, 2), XCZ(NOTE_GS, 5, 2), XCZ(NOTE_AS, 5, 2), XCZ(NOTE_B, 5, 2),
    XCZ(NOTE_CS, 6, 2), XCZ(NOTE_B, 5, 2), XCZ(NOTE_AS, 5, 4),
    XCZ(NOTE_GS, 5, 2), XCZ(NOTE_FS, 5, 2), XCZ(NOTE_GS, 5, 2), XCZ(NOTE_AS, 5, 2),
    XCZ(NOTE_B, 5, 4), XCZ(NOTE_FS, 5, 4),
    XCZ(NOTE_DS, 6, 2), XCZ(NOTE_CS, 6, 2), XCZ(NOTE_B, 5, 2), XCZ(NOTE_AS, 5, 2),
    XCZ(NOTE_GS, 5, 2), XCZ(NOTE_FS, 5, 2), XCZ(NOTE_E, 5, 4),
    XCZ(NOTE_FS, 5, 2), XCZ(NOTE_GS, 5, 2), XCZ(NOTE_AS, 5, 2), XCZ(NOTE_B, 5, 2),
    XCZ(NOTE_CS, 6, 8),

    /* Chorus */
    XCZ(NOTE_B, 5, 2), XCZ(NOTE_CS, 6, 2), XCZ(NOTE_DS, 6, 2), XCZ(NOTE_CS, 6, 2),
    XCZ(NOTE_B, 5, 3), XCZ(NOTE_AS, 5, 1), XCZ(NOTE_GS, 5, 2), XCZ(NOTE_FS, 5, 4),
    XCZ(NOTE_GS, 5, 2), XCZ(NOTE_AS, 5, 2), XCZ(NOTE_B, 5, 2), XCZ(NOTE_CS, 6, 2),
    XCZ(NOTE_DS, 6, 4), XCZ(NOTE_CS, 6, 4),
    XCZ(NOTE_B, 5, 2), XCZ(NOTE_AS, 5, 2), XCZ(NOTE_GS, 5, 2), XCZ(NOTE_FS, 5, 2),
    XCZ(NOTE_E, 5, 3), XCZ(NOTE_FS, 5, 1), XCZ(NOTE_GS, 5, 2), XCZ(NOTE_B, 5, 4),
    XCZ(NOTE_AS, 5, 2), XCZ(NOTE_GS, 5, 2), XCZ(NOTE_FS, 5, 2), XCZ(NOTE_E, 5, 2),
    XCZ(NOTE_DS, 5, 8),

    XCZ(NOTE_B, 5, 2), XCZ(NOTE_CS, 6, 2), XCZ(NOTE_DS, 6, 2), XCZ(NOTE_CS, 6, 2),
    XCZ(NOTE_B, 5, 3), XCZ(NOTE_AS, 5, 1), XCZ(NOTE_GS, 5, 2), XCZ(NOTE_FS, 5, 4),
    XCZ(NOTE_GS, 5, 2), XCZ(NOTE_AS, 5, 2), XCZ(NOTE_B, 5, 2), XCZ(NOTE_CS, 6, 2),
    XCZ(NOTE_DS, 6, 4), XCZ(NOTE_E, 6, 4),
    XCZ(NOTE_DS, 6, 2), XCZ(NOTE_CS, 6, 2), XCZ(NOTE_B, 5, 2), XCZ(NOTE_AS, 5, 2),
    XCZ(NOTE_B, 5, 4), XCZ(NOTE_GS, 5, 4),
    XCZ(NOTE_FS, 5, 2), XCZ(NOTE_E, 5, 2), XCZ(NOTE_DS, 5, 2), XCZ(NOTE_CS, 5, 2),
    XCZ(NOTE_B, 4, 8),

    /* Bridge / reprise */
    XCZ(NOTE_DS, 5, 2), XCZ(NOTE_FS, 5, 2), XCZ(NOTE_GS, 5, 2), XCZ(NOTE_B, 5, 2),
    XCZ(NOTE_AS, 5, 2), XCZ(NOTE_GS, 5, 2), XCZ(NOTE_FS, 5, 4),
    XCZ(NOTE_DS, 5, 2), XCZ(NOTE_E, 5, 2), XCZ(NOTE_FS, 5, 2), XCZ(NOTE_GS, 5, 2),
    XCZ(NOTE_B, 5, 4), XCZ(NOTE_CS, 6, 4),
    XCZ(NOTE_DS, 6, 2), XCZ(NOTE_CS, 6, 2), XCZ(NOTE_B, 5, 2), XCZ(NOTE_AS, 5, 2),
    XCZ(NOTE_GS, 5, 2), XCZ(NOTE_FS, 5, 2), XCZ(NOTE_E, 5, 4),
    XCZ(NOTE_FS, 5, 2), XCZ(NOTE_GS, 5, 2), XCZ(NOTE_AS, 5, 2), XCZ(NOTE_B, 5, 2),
    XCZ(NOTE_CS, 6, 8),

    /* Final chorus and outro */
    XCZ(NOTE_B, 5, 2), XCZ(NOTE_CS, 6, 2), XCZ(NOTE_DS, 6, 2), XCZ(NOTE_CS, 6, 2),
    XCZ(NOTE_B, 5, 3), XCZ(NOTE_AS, 5, 1), XCZ(NOTE_GS, 5, 2), XCZ(NOTE_FS, 5, 4),
    XCZ(NOTE_GS, 5, 2), XCZ(NOTE_AS, 5, 2), XCZ(NOTE_B, 5, 2), XCZ(NOTE_CS, 6, 2),
    XCZ(NOTE_DS, 6, 4), XCZ(NOTE_E, 6, 4),
    XCZ(NOTE_DS, 6, 2), XCZ(NOTE_CS, 6, 2), XCZ(NOTE_B, 5, 2), XCZ(NOTE_AS, 5, 2),
    XCZ(NOTE_B, 5, 4), XCZ(NOTE_GS, 5, 4),
    XCZ(NOTE_FS, 5, 2), XCZ(NOTE_E, 5, 2), XCZ(NOTE_DS, 5, 2), XCZ(NOTE_CS, 5, 2),
    XCZ(NOTE_B, 4, 4), XCZ(NOTE_FS, 5, 2), XCZ(NOTE_B, 5, 2),
    XCZ(NOTE_CS, 6, 4), XCZ(NOTE_B, 5, 4), XCZ(NOTE_FS, 5, 4), XCZ(NOTE_B, 4, 8)
};

#undef XCZ_R
#undef XCZ
#undef XCZ_UNIT_MS
