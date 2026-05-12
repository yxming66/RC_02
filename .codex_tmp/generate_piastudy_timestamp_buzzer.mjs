import fs from "node:fs";

const NOTE_NAMES = [
  "NOTE_C",
  "NOTE_CS",
  "NOTE_D",
  "NOTE_DS",
  "NOTE_E",
  "NOTE_F",
  "NOTE_FS",
  "NOTE_G",
  "NOTE_GS",
  "NOTE_A",
  "NOTE_AS",
  "NOTE_B",
];

const [
  jsonPath,
  outPath,
  arrayName,
  macroPrefix,
  title,
  source = jsonPath,
  keySignature = "C",
] = process.argv.slice(2);

if (!jsonPath || !outPath || !arrayName || !macroPrefix || !title) {
  console.error(
    "usage: node generate_piastudy_timestamp_buzzer.mjs <space.json> <out.inc> <array> <macro-prefix> <title> [source]"
  );
  process.exit(2);
}

const MIN_REST_MS = 16;
const KEY_SIGNATURES = {
  C: { sharps: [], flats: [] },
  G: { sharps: [5], flats: [] },
  D: { sharps: [5, 0], flats: [] },
  A: { sharps: [5, 0, 7], flats: [] },
  E: { sharps: [5, 0, 7, 2], flats: [] },
  B: { sharps: [5, 0, 7, 2, 9], flats: [] },
  FSharp: { sharps: [5, 0, 7, 2, 9, 4], flats: [] },
  CSharp: { sharps: [5, 0, 7, 2, 9, 4, 11], flats: [] },
  F: { sharps: [], flats: [11] },
  Bb: { sharps: [], flats: [11, 4] },
  Eb: { sharps: [], flats: [11, 4, 9] },
  Ab: { sharps: [], flats: [11, 4, 9, 2] },
  Db: { sharps: [], flats: [11, 4, 9, 2, 7] },
  Gb: { sharps: [], flats: [11, 4, 9, 2, 7, 0] },
  Cb: { sharps: [], flats: [11, 4, 9, 2, 7, 0, 5] },
};
const activeKeySignature =
  KEY_SIGNATURES[keySignature] ??
  KEY_SIGNATURES[keySignature.replace("#", "Sharp")] ??
  KEY_SIGNATURES.C;
const score = JSON.parse(fs.readFileSync(jsonPath, "utf8"));
const noteDefs = new Map();

function walk(value, visit) {
  if (Array.isArray(value)) {
    for (const child of value) walk(child, visit);
    return;
  }
  if (!value || typeof value !== "object") return;
  visit(value);
  for (const child of Object.values(value)) {
    if (child && typeof child === "object") walk(child, visit);
  }
}

walk(score.notesPositionInfo, (value) => {
  const start = Number(value.starTime);
  const end = Number(value.notesInfo?.[0]?.endTime);
  if (
    typeof value.noteName === "string" &&
    Array.isArray(value.key) &&
    Number.isFinite(start) &&
    Number.isFinite(end) &&
    end >= start
  ) {
    noteDefs.set(value.noteName, {
      key: value.key,
      start,
      duration: Math.max(0, end - start),
      isGrace: value.isGrace === true,
    });
  }
});

const occurrenceMap = new Map();
let missingRefs = 0;

walk(score.timesTamp, (value) => {
  if (!Array.isArray(value.noteTimestamp)) return;
  for (const point of value.noteTimestamp) {
    if (!point || !Array.isArray(point.notesInfo)) continue;
    for (const ref of point.notesInfo) {
      const def = noteDefs.get(ref?.noteName);
      if (!def) {
        missingRefs += 1;
        continue;
      }
      if (def.isGrace || def.duration < 12) continue;
      const start = Number(ref.startTime ?? point.startTime);
      if (!Number.isFinite(start)) continue;
      const key = `${ref.noteName}:${Math.round(start * 1000)}`;
      if (!occurrenceMap.has(key)) {
        occurrenceMap.set(key, {
          start,
          end: start + def.duration,
          keyGroups: def.key,
        });
      }
    }
  }
});

function orderedPitches(keyGroups) {
  const ordered = [];
  const seen = new Set();
  if (!Array.isArray(keyGroups)) return ordered;

  for (const group of keyGroups) {
    if (!Array.isArray(group)) continue;
    const pitches = group
      .filter((pitch) => Number.isFinite(Number(pitch)))
      .map((pitch) => Number(pitch))
      .sort((a, b) => b - a);
    for (const pitch of pitches) {
      if (!seen.has(pitch)) {
        seen.add(pitch);
        ordered.push(pitch);
      }
    }
  }

  return ordered;
}

function foldPitch(pitch) {
  while (pitch < 48) pitch += 12;
  while (pitch > 96) pitch -= 12;
  return pitch;
}

function applyKeySignature(pitch) {
  const pitchClass = ((pitch % 12) + 12) % 12;
  if (activeKeySignature.sharps.includes(pitchClass)) return pitch + 1;
  if (activeKeySignature.flats.includes(pitchClass)) return pitch - 1;
  return pitch;
}

function toneForPitch(pitch) {
  const folded = foldPitch(applyKeySignature(pitch));
  return {
    note: NOTE_NAMES[((folded % 12) + 12) % 12],
    octave: Math.floor(folded / 12) - 1,
  };
}

function sameFoldedTone(a, b) {
  if (!Number.isFinite(a) || !Number.isFinite(b)) return false;
  return foldPitch(a) === foldPitch(b);
}

function choosePitch(pitches, previousPitch) {
  if (pitches.length === 0) return null;
  if (!sameFoldedTone(pitches[0], previousPitch)) return pitches[0];

  const movingPitch = pitches.find((pitch) => !sameFoldedTone(pitch, previousPitch));
  return movingPitch ?? pitches[0];
}

function pushEvent(events, note, octave, durationMs) {
  const ms = Math.max(1, Math.min(65535, Math.round(durationMs)));
  const last = events[events.length - 1];
  if (
    note === "NOTE_REST" &&
    last &&
    last.note === "NOTE_REST" &&
    last.durationMs + ms <= 65535
  ) {
    last.durationMs += ms;
    return;
  }
  events.push({ note, octave, durationMs: ms });
}

const pointsByStart = new Map();
for (const occurrence of occurrenceMap.values()) {
  const startKey = Math.round(occurrence.start);
  let point = pointsByStart.get(startKey);
  if (!point) {
    point = { start: occurrence.start, end: occurrence.end, keyGroups: [] };
    pointsByStart.set(startKey, point);
  }
  point.end = Math.max(point.end, occurrence.end);
  point.keyGroups.push(occurrence.keyGroups);
}

const points = [...pointsByStart.values()]
  .map((point) => {
    const keyGroups = [];
    for (const groups of point.keyGroups) {
      if (Array.isArray(groups)) keyGroups.push(...groups);
    }
    return {
      start: point.start,
      end: point.end,
      pitches: orderedPitches(keyGroups),
    };
  })
  .filter((point) => point.pitches.length > 0)
  .sort((a, b) => a.start - b.start || b.pitches[0] - a.pitches[0]);

const events = [];
let previousPitch = null;
if (points.length && points[0].start >= MIN_REST_MS) {
  pushEvent(events, "NOTE_REST", 0, points[0].start);
}

for (let i = 0; i < points.length; i += 1) {
  const point = points[i];
  const nextStart = i + 1 < points.length ? points[i + 1].start : point.end;
  const slotEnd = Math.min(point.end, nextStart);
  const slotMs = slotEnd - point.start;
  if (slotMs > 0) {
    const pitch = choosePitch(point.pitches, previousPitch);
    const tone = toneForPitch(pitch);
    pushEvent(events, tone.note, tone.octave, slotMs);
    previousPitch = pitch;
  }

  if (i + 1 < points.length) {
    const restMs = nextStart - point.end;
    if (restMs >= MIN_REST_MS) {
      pushEvent(events, "NOTE_REST", 0, restMs);
      previousPitch = null;
    }
  }
}

const totalMs = events.reduce((sum, event) => sum + event.durationMs, 0);
const lines = [];
lines.push("/*");
lines.push(` * ${title}.`);
lines.push(` * Single-line PWM buzzer reduction generated from ${source}.`);
lines.push(" * The source is PiaStudy timestamp/space data matching the provided PDF.");
lines.push(" * Each visible playback onset contributes one buzzer tone, so moving");
lines.push(" * accompaniment is preserved instead of being hidden under long held notes.");
lines.push(" * Simultaneous notes are reduced upper-voice first, but repeated");
lines.push(" * top notes fall through to another chord tone when that exposes");
lines.push(" * motion that would otherwise sound like a tied note.");
lines.push(" * Notes outside the practical buzzer range are folded by octaves into C3-C7.");
lines.push(` * ${events.length} events, about ${Math.round(totalMs / 1000)} seconds.`);
if (missingRefs > 0) {
  lines.push(` * Warning: ${missingRefs} timestamp references were not resolved.`);
}
lines.push(" */");
lines.push(`#define ${macroPrefix}(note, octave, ms) {note, octave, (uint16_t)(ms)}`);
lines.push(`#define ${macroPrefix}_R(ms) {NOTE_REST, 0, (uint16_t)(ms)}`);
lines.push("");
lines.push(`const Tone_t ${arrayName}[] = {`);

for (let i = 0; i < events.length; i += 1) {
  if (i % 16 === 0) {
    lines.push(`    /* Events ${i + 1}-${Math.min(i + 16, events.length)} */`);
  }
  const event = events[i];
  const value =
    event.note === "NOTE_REST"
      ? `${macroPrefix}_R(${event.durationMs})`
      : `${macroPrefix}(${event.note}, ${event.octave}, ${event.durationMs})`;
  lines.push(`    ${value}${i === events.length - 1 ? "" : ","}`);
}

lines.push("};");
lines.push("");
lines.push(`#undef ${macroPrefix}_R`);
lines.push(`#undef ${macroPrefix}`);
lines.push("");

fs.writeFileSync(outPath, lines.join("\n"), "utf8");
console.log(
  JSON.stringify(
    {
      noteDefs: noteDefs.size,
      occurrences: occurrenceMap.size,
      points: points.length,
      events: events.length,
      durationMs: totalMs,
      missingRefs,
    },
    null,
    2
  )
);
