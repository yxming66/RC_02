/* Beethoven - Piano Sonata No. 14, Op. 27 No. 2, movement II.
 * Complete single-line PWM buzzer reduction from the local second-movement PDF.
 * The score repeats are expanded; bass notes below C3 are raised by octaves
 * to keep them audible on small passive buzzers. */
const Tone_t MOONLIGHT_SONATA_2[] = {
    /* ~measure 1 */
    {NOTE_CS, 5, 600}, {NOTE_C, 5, 1200}, {NOTE_AS, 4, 600}, 
    {NOTE_DS, 5, 600}, {NOTE_REST, 0, 600}, {NOTE_CS, 5, 600}, 
    {NOTE_C, 5, 600}, {NOTE_REST, 0, 600}, {NOTE_AS, 4, 600}, 
    {NOTE_GS, 4, 600}, {NOTE_REST, 0, 600}, {NOTE_FS, 5, 600}, 
    {NOTE_F, 5, 1200}, {NOTE_DS, 5, 600}, {NOTE_GS, 5, 600}, 
    {NOTE_REST, 0, 600}, {NOTE_FS, 5, 600}, {NOTE_F, 5, 600}, 
    {NOTE_REST, 0, 600}, {NOTE_DS, 5, 600}, {NOTE_CS, 5, 600}, 
    {NOTE_REST, 0, 600},
    /* ~measure 9 */
    {NOTE_CS, 5, 600}, {NOTE_CS, 5, 600}, {NOTE_C, 5, 600}, 
    {NOTE_AS, 4, 600}, {NOTE_DS, 4, 600}, {NOTE_DS, 5, 600}, 
    {NOTE_CS, 5, 600}, {NOTE_GS, 4, 600}, {NOTE_C, 5, 600}, 
    {NOTE_AS, 4, 600}, {NOTE_C, 4, 600}, {NOTE_GS, 4, 600}, 
    {NOTE_FS, 5, 600}, {NOTE_FS, 5, 600}, {NOTE_F, 5, 600}, 
    {NOTE_DS, 5, 600}, {NOTE_GS, 4, 600}, {NOTE_GS, 5, 600}, 
    {NOTE_FS, 5, 600}, {NOTE_CS, 5, 600}, {NOTE_F, 5, 600}, 
    {NOTE_DS, 5, 600}, {NOTE_F, 4, 600}, {NOTE_CS, 5, 600},
    /* ~measure 17 */
    {NOTE_CS, 5, 600}, {NOTE_CS, 5, 1200}, {NOTE_CS, 5, 600}, 
    {NOTE_CS, 5, 600}, {NOTE_C, 5, 600}, {NOTE_B, 4, 600}, 
    {NOTE_B, 4, 1200}, {NOTE_B, 4, 600}, {NOTE_B, 4, 600}, 
    {NOTE_AS, 4, 600}, {NOTE_AS, 4, 600}, {NOTE_C, 5, 1200}, 
    {NOTE_CS, 5, 600}, {NOTE_DS, 5, 1200}, {NOTE_FS, 4, 600}, 
    {NOTE_GS, 3, 600}, {NOTE_F, 4, 600}, {NOTE_AS, 4, 300}, 
    {NOTE_FS, 4, 300}, {NOTE_F, 4, 600}, {NOTE_DS, 4, 600},
    /* ~measure 25 */
    {NOTE_CS, 5, 600}, {NOTE_C, 5, 1200}, {NOTE_AS, 4, 600}, 
    {NOTE_DS, 5, 600}, {NOTE_REST, 0, 600}, {NOTE_CS, 5, 600}, 
    {NOTE_GS, 4, 600}, {NOTE_C, 5, 600}, {NOTE_AS, 4, 600}, 
    {NOTE_C, 4, 600}, {NOTE_GS, 4, 600}, {NOTE_FS, 5, 600}, 
    {NOTE_F, 5, 1200}, {NOTE_DS, 5, 600}, {NOTE_GS, 5, 600}, 
    {NOTE_REST, 0, 600}, {NOTE_FS, 5, 600}, {NOTE_FS, 5, 600}, 
    {NOTE_F, 5, 600}, {NOTE_DS, 5, 600}, {NOTE_GS, 5, 600}, 
    {NOTE_REST, 0, 600},
    /* ~measure 33 */
    {NOTE_FS, 5, 600}, {NOTE_FS, 5, 600}, {NOTE_F, 5, 600}, 
    {NOTE_AS, 5, 600}, {NOTE_AS, 5, 600}, {NOTE_GS, 5, 600}, 
    {NOTE_REST, 0, 600}, {NOTE_C, 4, 1200}, {NOTE_DS, 4, 300}, 
    {NOTE_CS, 4, 300}, {NOTE_CS, 4, 600}, {NOTE_REST, 0, 600}, 
    {NOTE_CS, 5, 600}, {NOTE_CS, 5, 1200}, {NOTE_CS, 5, 600}, 
    {NOTE_CS, 5, 600}, {NOTE_C, 5, 600}, {NOTE_B, 4, 600}, 
    {NOTE_B, 4, 1200}, {NOTE_B, 4, 600}, {NOTE_B, 4, 600}, 
    {NOTE_AS, 4, 600},
    /* ~measure 41 */
    {NOTE_AS, 4, 600}, {NOTE_C, 5, 1200}, {NOTE_CS, 5, 600}, 
    {NOTE_DS, 5, 1200}, {NOTE_FS, 4, 600}, {NOTE_FS, 4, 600}, 
    {NOTE_F, 4, 600}, {NOTE_AS, 4, 300}, {NOTE_FS, 4, 300}, 
    {NOTE_F, 4, 600}, {NOTE_DS, 4, 600}, {NOTE_CS, 5, 600}, 
    {NOTE_C, 5, 1200}, {NOTE_AS, 4, 600}, {NOTE_DS, 5, 600}, 
    {NOTE_REST, 0, 600}, {NOTE_CS, 5, 600}, {NOTE_CS, 5, 600}, 
    {NOTE_C, 5, 600}, {NOTE_AS, 4, 600}, {NOTE_AS, 4, 600}, 
    {NOTE_GS, 4, 600},
    /* ~measure 49 */
    {NOTE_FS, 5, 600}, {NOTE_F, 5, 1200}, {NOTE_DS, 5, 600}, 
    {NOTE_GS, 5, 600}, {NOTE_REST, 0, 600}, {NOTE_FS, 5, 600}, 
    {NOTE_FS, 5, 600}, {NOTE_F, 5, 600}, {NOTE_DS, 5, 600}, 
    {NOTE_GS, 5, 600}, {NOTE_REST, 0, 600}, {NOTE_FS, 5, 600}, 
    {NOTE_FS, 5, 600}, {NOTE_F, 5, 600}, {NOTE_AS, 5, 600}, 
    {NOTE_AS, 5, 600}, {NOTE_GS, 5, 600}, {NOTE_REST, 0, 600}, 
    {NOTE_C, 4, 1200}, {NOTE_DS, 4, 300}, {NOTE_CS, 4, 300}, 
    {NOTE_CS, 4, 600}, {NOTE_REST, 0, 600},
    /* ~measure 57 */
    {NOTE_F, 4, 1800}, {NOTE_REST, 0, 600}, {NOTE_F, 4, 1200}, 
    {NOTE_FS, 4, 600}, {NOTE_FS, 4, 1200}, {NOTE_DS, 5, 600}, 
    {NOTE_DS, 5, 600}, {NOTE_C, 5, 600}, {NOTE_GS, 4, 600}, 
    {NOTE_CS, 5, 600}, {NOTE_F, 5, 600}, {NOTE_F, 4, 600}, 
    {NOTE_F, 4, 1200}, {NOTE_FS, 4, 600}, {NOTE_FS, 4, 1200}, 
    {NOTE_FS, 5, 600}, {NOTE_FS, 5, 600}, {NOTE_AS, 5, 600},
    /* ~measure 65 */
    {NOTE_C, 5, 600}, {NOTE_DS, 5, 600}, {NOTE_CS, 5, 600}, 
    {NOTE_F, 4, 1800}, {NOTE_REST, 0, 600}, {NOTE_F, 4, 1200}, 
    {NOTE_FS, 4, 600}, {NOTE_FS, 4, 1200}, {NOTE_DS, 5, 600}, 
    {NOTE_DS, 5, 600}, {NOTE_C, 5, 600}, {NOTE_GS, 4, 600}, 
    {NOTE_CS, 5, 600}, {NOTE_F, 5, 600}, {NOTE_F, 4, 600}, 
    {NOTE_F, 4, 1200}, {NOTE_FS, 4, 600}, {NOTE_FS, 4, 1200},
    /* ~measure 73 */
    {NOTE_FS, 5, 600}, {NOTE_FS, 5, 600}, {NOTE_AS, 5, 600}, 
    {NOTE_C, 5, 600}, {NOTE_DS, 5, 600}, {NOTE_CS, 5, 600}, 
    {NOTE_F, 5, 600}, {NOTE_REST, 0, 1800}, {NOTE_F, 5, 1200}, 
    {NOTE_AS, 4, 600}, {NOTE_AS, 4, 1200}, {NOTE_DS, 5, 600}, 
    {NOTE_DS, 5, 1200}, {NOTE_GS, 4, 600}, {NOTE_GS, 4, 1200}, 
    {NOTE_CS, 5, 600}, {NOTE_CS, 5, 1200},
    /* ~measure 81 */
    {NOTE_FS, 4, 600}, {NOTE_FS, 4, 1200}, {NOTE_F, 4, 600}, 
    {NOTE_F, 4, 600}, {NOTE_GS, 4, 600}, {NOTE_B, 4, 600}, 
    {NOTE_B, 4, 600}, {NOTE_AS, 4, 600}, {NOTE_CS, 5, 600}, 
    {NOTE_CS, 5, 1200}, {NOTE_FS, 4, 600}, {NOTE_FS, 4, 1200}, 
    {NOTE_F, 4, 600}, {NOTE_F, 4, 1200}, {NOTE_E, 4, 600}, 
    {NOTE_E, 4, 1200}, {NOTE_F, 4, 600}, {NOTE_F, 4, 1200},
    /* ~measure 89 */
    {NOTE_FS, 4, 600}, {NOTE_FS, 4, 1200}, {NOTE_F, 4, 600}, 
    {NOTE_F, 4, 600}, {NOTE_FS, 4, 600}, {NOTE_C, 4, 600}, 
    {NOTE_CS, 4, 600}, {NOTE_REST, 0, 600}, {NOTE_F, 5, 600}, 
    {NOTE_REST, 0, 1800}, {NOTE_F, 5, 1200}, {NOTE_AS, 4, 600}, 
    {NOTE_AS, 4, 1200}, {NOTE_DS, 5, 600}, {NOTE_DS, 5, 1200}, 
    {NOTE_GS, 4, 600}, {NOTE_GS, 4, 1200},
    /* ~measure 97 */
    {NOTE_CS, 5, 600}, {NOTE_CS, 5, 1200}, {NOTE_FS, 4, 600}, 
    {NOTE_FS, 4, 1200}, {NOTE_F, 4, 600}, {NOTE_F, 4, 600}, 
    {NOTE_GS, 4, 600}, {NOTE_B, 4, 600}, {NOTE_B, 4, 600}, 
    {NOTE_AS, 4, 600}, {NOTE_CS, 5, 600}, {NOTE_CS, 5, 1200}, 
    {NOTE_FS, 4, 600}, {NOTE_FS, 4, 1200}, {NOTE_F, 4, 600}, 
    {NOTE_F, 4, 1200}, {NOTE_E, 4, 600}, {NOTE_E, 4, 1200},
    /* ~measure 105 */
    {NOTE_F, 4, 600}, {NOTE_F, 4, 1200}, {NOTE_FS, 4, 600}, 
    {NOTE_FS, 4, 1200}, {NOTE_F, 4, 600}, {NOTE_F, 4, 600}, 
    {NOTE_FS, 4, 600}, {NOTE_C, 4, 600}, {NOTE_CS, 4, 600}
};

/* 271 buzzer events, about 194 seconds at the source tempo. */
