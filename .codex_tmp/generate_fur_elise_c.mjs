import fs from "node:fs";

const [
  midiPath,
  outPath,
  arrayName = "FUR_ELISE",
  title = "Generated MIDI",
  measureQuartersArg = "1.5",
] = process.argv.slice(2);
if (!midiPath || !outPath) {
  console.error("usage: node generate_fur_elise_c.mjs <midi> <out> [array-name] [title]");
  process.exit(2);
}

const data = fs.readFileSync(midiPath);
let off = 0;
const readStr = (n) => data.subarray((off += n) - n, off).toString("ascii");
const readU16 = () => ((data[off++] << 8) | data[off++]);
const readU32 = () => ((data[off++] << 24) | (data[off++] << 16) | (data[off++] << 8) | data[off++]) >>> 0;
const readVar = () => {
  let value = 0;
  for (;;) {
    const b = data[off++];
    value = (value << 7) | (b & 0x7f);
    if ((b & 0x80) === 0) return value;
  }
};

if (readStr(4) !== "MThd") throw new Error("not a MIDI file");
const hlen = readU32();
readU16();
const trackCount = readU16();
const division = readU16();
off += hlen - 6;

const tracks = [];
let usPerQuarter = 833333;

for (let trackIndex = 0; trackIndex < trackCount; trackIndex += 1) {
  const id = readStr(4);
  const len = readU32();
  const end = off + len;
  let tick = 0;
  let status = 0;
  const active = new Map();
  const notes = [];
  while (off < end) {
    tick += readVar();
    let b = data[off++];
    if (b < 0x80) {
      off -= 1;
      b = status;
    } else {
      status = b;
    }
    if (b === 0xff) {
      const type = data[off++];
      const n = readVar();
      const payload = data.subarray(off, off + n);
      off += n;
      if (type === 0x51 && n === 3) {
        usPerQuarter = (payload[0] << 16) | (payload[1] << 8) | payload[2];
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
      active.get(key).push({ trackIndex, ch, note: d1, start: tick, vel: d2 });
    } else if (event === 0x80 || (event === 0x90 && d2 === 0)) {
      const key = `${ch}:${d1}`;
      const stack = active.get(key);
      if (stack?.length) {
        const start = stack.shift();
        notes.push({ ...start, end: tick });
      }
    }
  }
  off = end;
  tracks.push({ id, notes });
}

const allNotes = tracks.flatMap((track) => track.notes);
const starts = new Map();
for (const note of allNotes) {
  if (!starts.has(note.start)) starts.set(note.start, []);
  starts.get(note.start).push(note);
}

function chooseNote(notes) {
  const upper = notes.filter((n) => n.trackIndex === 1);
  const pool = upper.length ? upper : notes;
  return pool.slice().sort((a, b) => b.note - a.note || b.vel - a.vel || b.end - a.end)[0];
}

const selected = [...starts.entries()]
  .sort((a, b) => a[0] - b[0])
  .map(([tick, notes]) => ({ tick, note: chooseNote(notes) }))
  .filter((event) => event.note.end > event.tick);

const ticksToMs = (ticks) => Math.max(1, Math.round((ticks * usPerQuarter) / (division * 1000)));
const noteNames = [
  "NOTE_C", "NOTE_CS", "NOTE_D", "NOTE_DS", "NOTE_E", "NOTE_F",
  "NOTE_FS", "NOTE_G", "NOTE_GS", "NOTE_A", "NOTE_AS", "NOTE_B",
];
const midiToC = (midi) => {
  while (midi < 48) {
    midi += 12;
  }
  const note = noteNames[midi % 12];
  const octave = Math.floor(midi / 12) - 1;
  return `${note}, ${octave}`;
};

const tones = [];
for (let i = 0; i < selected.length; i += 1) {
  const current = selected[i];
  const nextTick = i + 1 < selected.length ? selected[i + 1].tick : current.note.end;
  const noteTicks = Math.min(current.note.end, nextTick) - current.tick;
  if (noteTicks > 0) {
    tones.push({ kind: "note", midi: current.note.note, ticks: noteTicks, tick: current.tick });
  }
  const restTicks = nextTick - current.tick - Math.max(noteTicks, 0);
  if (restTicks >= 24) {
    tones.push({ kind: "rest", ticks: restTicks, tick: current.tick + noteTicks });
  }
}

const totalMs = tones.reduce((sum, t) => sum + ticksToMs(t.ticks), 0);
const lines = [];
lines.push(`/* ${title}.`);
lines.push(" * Reduced to one monophonic note stream for a passive PWM buzzer.");
lines.push(" * Bass notes below C3 are raised one octave to keep them audible. */");
lines.push(`const Tone_t ${arrayName}[] = {`);
let barTick = 0;
let countInLine = 0;
const measureTicks = division * Number(measureQuartersArg);
for (const tone of tones) {
  while (tone.tick >= barTick) {
    if (tone.tick === barTick) {
      const measureNo = Math.floor(barTick / measureTicks) + 1;
      if (measureNo === 1 || measureNo % 8 === 1) {
        if (countInLine) {
          lines[lines.length - 1] = lines.at(-1).replace(/,\s*$/, ",");
          countInLine = 0;
        }
        lines.push(`    /* ~measure ${measureNo} */`);
      }
    }
    barTick += measureTicks;
  }
  const dur = ticksToMs(tone.ticks);
  const item = tone.kind === "rest"
    ? `{NOTE_REST, 0, ${dur}}`
    : `{${midiToC(tone.midi)}, ${dur}}`;
  if (countInLine === 0) {
    lines.push(`    ${item}, `);
  } else if (countInLine < 3) {
    lines[lines.length - 1] += `${item}, `;
  } else {
    lines.push(`    ${item}, `);
    countInLine = 0;
  }
  countInLine += 1;
}
if (lines.at(-1).endsWith(", ")) {
  lines[lines.length - 1] = lines.at(-1).slice(0, -2);
}
lines.push("};");
lines.push("");
lines.push(`/* ${tones.length} buzzer events, about ${Math.round(totalMs / 1000)} seconds at Poco moto. */`);

fs.writeFileSync(outPath, lines.join("\n") + "\n");
console.error(`events=${tones.length} selectedStarts=${selected.length} durationMs=${totalMs}`);
