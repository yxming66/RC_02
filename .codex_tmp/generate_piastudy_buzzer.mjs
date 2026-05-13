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
  durationScaleArg = "1",
] = process.argv.slice(2);

if (!jsonPath || !outPath || !arrayName || !macroPrefix || !title) {
  console.error(
    "usage: node generate_piastudy_buzzer.mjs <space.json> <out.inc> <array> <macro-prefix> <title> [source] [key] [duration-scale]"
  );
  process.exit(2);
}

const MIN_REST_MS = 16;
const durationScale = Number(durationScaleArg);
if (!Number.isFinite(durationScale) || durationScale <= 0) {
  console.error("duration-scale must be a positive number");
  process.exit(2);
}
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
  KEY_SIGNATURES[keySignature] ?? KEY_SIGNATURES[keySignature.replace("#", "Sharp")] ??
  KEY_SIGNATURES.C;
const score = JSON.parse(fs.readFileSync(jsonPath, "utf8"));
const timestamps = [];

function collect(value) {
  if (Array.isArray(value)) {
    for (const child of value) collect(child);
    return;
  }
  if (!value || typeof value !== "object") return;

  if (Array.isArray(value.noteTimestamp)) {
    timestamps.push(...value.noteTimestamp);
  }
  for (const child of Object.values(value)) {
    if (child && typeof child === "object") collect(child);
  }
}

collect(score);

function orderedPitches(keyGroups) {
  const ordered = [];
  const seen = new Set();

  // PiaStudy stores upper/right-hand voices before lower/left-hand voices.
  for (const group of keyGroups) {
    if (!Array.isArray(group)) continue;
    const pitches = group.slice().sort((a, b) => b - a);
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
    note: NOTE_NAMES[folded % 12],
    octave: Math.floor(folded / 12) - 1,
  };
}

function pushEvent(events, note, octave, durationMs) {
  const ms = Math.max(1, Math.min(65535, Math.round(durationMs * durationScale)));
  const last = events[events.length - 1];
  if (
    last &&
    note === "NOTE_REST" &&
    last.note === note &&
    last.octave === octave &&
    last.durationMs + ms <= 65535
  ) {
    last.durationMs += ms;
  } else {
    events.push({ note, octave, durationMs: ms });
  }
}

const points = timestamps
  .map((entry) => {
    const start = Number(entry.starTime ?? entry.startTime);
    const end = Number(entry.notesInfo?.[0]?.endTime);
    const keys = orderedPitches(entry.key ?? []);
    return { start, end, keys };
  })
  .filter(
    (entry) =>
      Number.isFinite(entry.start) &&
      Number.isFinite(entry.end) &&
      entry.end > entry.start &&
      entry.keys.length > 0
  )
  .sort((a, b) => a.start - b.start || a.end - b.end);

const events = [];
if (points.length && points[0].start >= MIN_REST_MS) {
  pushEvent(events, "NOTE_REST", 0, points[0].start);
}

for (let i = 0; i < points.length; i += 1) {
  const point = points[i];
  const nextStart = i + 1 < points.length ? points[i + 1].start : point.end;
  const slotEnd = Math.min(point.end, nextStart);
  const slotMs = Math.round(slotEnd - point.start);

  if (slotMs > 0) {
    const base = Math.floor(slotMs / point.keys.length);
    let remaining = slotMs % point.keys.length;

    for (const pitch of point.keys) {
      const duration = base + (remaining > 0 ? 1 : 0);
      if (remaining > 0) remaining -= 1;
      const tone = toneForPitch(pitch);
      pushEvent(events, tone.note, tone.octave, duration);
    }
  }

  if (i + 1 < points.length) {
    const restMs = Math.round(nextStart - point.end);
    if (restMs >= MIN_REST_MS) {
      pushEvent(events, "NOTE_REST", 0, restMs);
    }
  }
}

const totalMs = events.reduce((sum, event) => sum + event.durationMs, 0);
const lines = [];
lines.push("/*");
lines.push(` * ${title}.`);
lines.push(` * Complete single-line PWM buzzer reduction generated from ${source}.`);
lines.push(" * Simultaneous right-hand, left-hand, and chord tones are");
lines.push(" * arpeggiated into one buzzer line with the upper voice first;");
lines.push(" * notes outside the practical buzzer range are folded by octaves.");
lines.push(` * ${events.length} buzzer events, about ${Math.round(totalMs / 1000)} seconds.`);
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

console.log(JSON.stringify({
  points: points.length,
  events: events.length,
  durationMs: totalMs,
  durationSec: totalMs / 1000,
}, null, 2));
