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

const [midiPath, outPath, arrayName, title = "Generated MIDI", source = midiPath] =
  process.argv.slice(2);

if (!midiPath || !outPath || !arrayName) {
  console.error(
    "usage: node midi_to_buzzer_monophonic.mjs <midi> <out.inc> <array> [title] [source]"
  );
  process.exit(2);
}

const data = fs.readFileSync(midiPath);
let off = 0;

function readStr(n) {
  const s = data.subarray(off, off + n).toString("ascii");
  off += n;
  return s;
}

function readU16() {
  const v = (data[off] << 8) | data[off + 1];
  off += 2;
  return v;
}

function readU32() {
  const v =
    ((data[off] << 24) |
      (data[off + 1] << 16) |
      (data[off + 2] << 8) |
      data[off + 3]) >>>
    0;
  off += 4;
  return v;
}

function readVar() {
  let value = 0;
  for (;;) {
    const b = data[off++];
    value = (value << 7) | (b & 0x7f);
    if ((b & 0x80) === 0) return value;
  }
}

if (readStr(4) !== "MThd") {
  throw new Error("not a MIDI file");
}

const headerLen = readU32();
const format = readU16();
const trackCount = readU16();
const division = readU16();
if ((division & 0x8000) !== 0) {
  throw new Error("SMPTE division is not supported");
}
off += headerLen - 6;

const notes = [];
const starts = [];
const tempos = [{ tick: 0, usPerQuarter: 500000 }];
const tracks = [];

for (let trackIndex = 0; trackIndex < trackCount; trackIndex += 1) {
  const id = readStr(4);
  if (id !== "MTrk") {
    throw new Error(`missing MTrk at track ${trackIndex}`);
  }
  const len = readU32();
  const end = off + len;
  let tick = 0;
  let status = 0;
  let name = "";
  const active = new Map();

  while (off < end) {
    tick += readVar();
    let b = data[off++];
    if (b < 0x80) {
      off -= 1;
      b = status;
    } else if (b < 0xf0) {
      status = b;
    }

    if (b === 0xff) {
      const type = data[off++];
      const n = readVar();
      const payload = data.subarray(off, off + n);
      off += n;
      if ((type === 0x03 || type === 0x04) && payload.length) {
        name = payload.toString("utf8").trim();
      } else if (type === 0x51 && n === 3) {
        tempos.push({
          tick,
          usPerQuarter: (payload[0] << 16) | (payload[1] << 8) | payload[2],
        });
      }
      continue;
    }

    if (b === 0xf0 || b === 0xf7) {
      off += readVar();
      continue;
    }

    const event = b & 0xf0;
    const ch = b & 0x0f;
    const d1 = data[off++];
    const d2 = event === 0xc0 || event === 0xd0 ? 0 : data[off++];

    if (event === 0x90 && d2 > 0) {
      const key = `${ch}:${d1}`;
      if (!active.has(key)) active.set(key, []);
      const start = { trackIndex, ch, pitch: d1, tick, velocity: d2 };
      active.get(key).push(start);
      starts.push(start);
    } else if (event === 0x80 || (event === 0x90 && d2 === 0)) {
      const key = `${ch}:${d1}`;
      const stack = active.get(key);
      if (stack?.length) {
        const start = stack.shift();
        notes.push({ ...start, endTick: tick });
      }
    }
  }

  tracks.push({ trackIndex, name });
  off = end;
}

tempos.sort((a, b) => a.tick - b.tick);
const tempoMap = [];
let lastTick = 0;
let lastMs = 0;
let lastTempo = tempos[0].usPerQuarter;
for (const tempo of tempos) {
  if (tempo.tick !== lastTick) {
    lastMs += ((tempo.tick - lastTick) * lastTempo) / division / 1000;
    lastTick = tempo.tick;
  }
  lastTempo = tempo.usPerQuarter;
  tempoMap.push({ tick: lastTick, ms: lastMs, usPerQuarter: lastTempo });
}

function tickToMs(tick) {
  let lo = 0;
  let hi = tempoMap.length - 1;
  while (lo <= hi) {
    const mid = (lo + hi) >> 1;
    if (tempoMap[mid].tick <= tick) lo = mid + 1;
    else hi = mid - 1;
  }
  const tempo = tempoMap[Math.max(0, hi)];
  return tempo.ms + ((tick - tempo.tick) * tempo.usPerQuarter) / division / 1000;
}

const endByStart = new Map();
for (const note of notes) {
  const key = `${note.trackIndex}:${note.ch}:${note.pitch}:${note.tick}`;
  endByStart.set(key, Math.max(endByStart.get(key) ?? 0, note.endTick));
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

function chooseNote(group) {
  const right = group.filter((n) => n.trackIndex === 1);
  const pool = right.length ? right : group;
  return pool
    .slice()
    .sort((a, b) => b.pitch - a.pitch || b.velocity - a.velocity || a.trackIndex - b.trackIndex)[0];
}

starts.sort((a, b) => a.tick - b.tick || b.pitch - a.pitch);
const groups = [];
for (const start of starts) {
  const last = groups[groups.length - 1];
  if (!last || last.tick !== start.tick) {
    groups.push({ tick: start.tick, notes: [start] });
  } else {
    last.notes.push(start);
  }
}

const rawEvents = [];
if (groups.length > 0 && groups[0].tick > 0) {
  const leadingRestMs = Math.round(tickToMs(groups[0].tick));
  if (leadingRestMs >= 16) {
    rawEvents.push({
      note: "NOTE_REST",
      octave: 0,
      durationMs: Math.min(65535, leadingRestMs),
    });
  }
}
for (let i = 0; i < groups.length; i += 1) {
  const group = groups[i];
  const chosen = chooseNote(group.notes);
  const nextTick = i + 1 < groups.length ? groups[i + 1].tick : null;
  const endKey = `${chosen.trackIndex}:${chosen.ch}:${chosen.pitch}:${chosen.tick}`;
  const chosenEnd = endByStart.get(endKey) ?? chosen.tick;
  const eventEnd =
    nextTick === null ? chosenEnd : Math.max(group.tick, Math.min(chosenEnd, nextTick));

  const noteMs = Math.round(tickToMs(eventEnd) - tickToMs(group.tick));
  if (noteMs > 0) {
    const tone = toneForPitch(chosen.pitch);
    rawEvents.push({ ...tone, durationMs: Math.max(1, Math.min(65535, noteMs)) });
  }

  if (nextTick !== null && chosenEnd < nextTick) {
    const restMs = Math.round(tickToMs(nextTick) - tickToMs(chosenEnd));
    if (restMs >= 16) {
      rawEvents.push({ note: "NOTE_REST", octave: 0, durationMs: Math.min(65535, restMs) });
    }
  }
}

const events = [];
for (const event of rawEvents) {
  const prev = events[events.length - 1];
  if (
    prev &&
    prev.note === event.note &&
    prev.octave === event.octave &&
    prev.durationMs + event.durationMs <= 65535
  ) {
    prev.durationMs += event.durationMs;
  } else {
    events.push({ ...event });
  }
}

const totalMs = events.reduce((sum, event) => sum + event.durationMs, 0);
const lines = [];
lines.push("/*");
lines.push(` * ${title}.`);
lines.push(` * Single-line PWM buzzer reduction generated from ${source}.`);
lines.push(" * The right-hand line is preferred at simultaneous onsets; bass notes");
lines.push(" * are folded by octaves into the practical buzzer range.");
lines.push(` * ${events.length} events, about ${Math.round(totalMs / 1000)} seconds.`);
lines.push(" */");
lines.push("#define FD(note, octave, ms) {note, octave, (uint16_t)(ms)}");
lines.push("#define FD_R(ms) {NOTE_REST, 0, (uint16_t)(ms)}");
lines.push("");
lines.push(`const Tone_t ${arrayName}[] = {`);
for (let i = 0; i < events.length; i += 1) {
  if (i % 16 === 0) {
    lines.push(`    /* Events ${i + 1}-${Math.min(i + 16, events.length)} */`);
  }
  const event = events[i];
  const value =
    event.note === "NOTE_REST"
      ? `FD_R(${event.durationMs})`
      : `FD(${event.note}, ${event.octave}, ${event.durationMs})`;
  lines.push(`    ${value}${i === events.length - 1 ? "" : ","}`);
}
lines.push("};");
lines.push("");
lines.push("#undef FD_R");
lines.push("#undef FD");
lines.push("");

fs.writeFileSync(outPath, lines.join("\n"), "utf8");

console.log(`format=${format}`);
console.log(`tracks=${trackCount}`);
console.log(`division=${division}`);
console.log(`track_names=${tracks.map((t) => `${t.trackIndex}:${t.name}`).join(",")}`);
console.log(`groups=${groups.length}`);
console.log(`events=${events.length}`);
console.log(`duration_seconds=${(totalMs / 1000).toFixed(1)}`);
