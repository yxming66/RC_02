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

const [jsonPath, outPath] = process.argv.slice(2);
if (!jsonPath || !outPath) {
  console.error(
    "usage: node generate_flower_from_piastudy_json.mjs <space.json> <out.inc>"
  );
  process.exit(2);
}

const MIN_REST_MS = 16;
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
  // Keep the upper line's attack first, then fold in accompaniment notes.
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

function toneForPitch(pitch) {
  const folded = foldPitch(pitch);
  return {
    note: NOTE_NAMES[folded % 12],
    octave: Math.floor(folded / 12) - 1,
  };
}

function pushEvent(events, note, octave, durationMs) {
  const ms = Math.max(1, Math.min(65535, Math.round(durationMs)));
  const last = events[events.length - 1];
  if (
    last &&
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
let out = "";
out += "/*\n";
out += " * DJ Okawari - Flower Dance.\n";
out += " * Complete single-line PWM buzzer reduction generated from the local\n";
out += " * PiaStudy PDF data. Simultaneous right-hand, left-hand, and chord tones\n";
out += " * are arpeggiated into one buzzer line, with the upper line kept first;\n";
out += " * notes outside the practical buzzer range are folded by octaves.\n";
out += ` * ${events.length} buzzer events, about ${Math.round(totalMs / 1000)} seconds.\n`;
out += " */\n";
out += "#define FD(note, octave, ms) {note, octave, (uint16_t)(ms)}\n";
out += "#define FD_R(ms) {NOTE_REST, 0, (uint16_t)(ms)}\n\n";
out += "const Tone_t FLOWER_DANCE[] = {\n";

for (let i = 0; i < events.length; i += 1) {
  if (i % 16 === 0) {
    const end = Math.min(i + 16, events.length);
    out += `    /* Events ${i + 1}-${end} */\n`;
  }
  const event = events[i];
  if (event.note === "NOTE_REST") {
    out += `    FD_R(${event.durationMs})`;
  } else {
    out += `    FD(${event.note}, ${event.octave}, ${event.durationMs})`;
  }
  out += i + 1 === events.length ? "\n" : ",\n";
}

out += "};\n\n";
out += "#undef FD_R\n";
out += "#undef FD\n";

fs.writeFileSync(outPath, out);
console.log(
  JSON.stringify(
    {
      points: points.length,
      events: events.length,
      durationMs: totalMs,
      durationSec: totalMs / 1000,
    },
    null,
    2
  )
);
