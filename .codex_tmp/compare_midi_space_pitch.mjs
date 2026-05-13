import fs from "node:fs";

const [midiPath, spacePath] = process.argv.slice(2);
if (!midiPath || !spacePath) {
  console.error("usage: node compare_midi_space_pitch.mjs <midi> <space.json>");
  process.exit(2);
}

const data = fs.readFileSync(midiPath);
let off = 0;
const readStr = (n) => data.subarray((off += n) - n, off).toString("ascii");
const readU16 = () => (data[off++] << 8) | data[off++];
const readU32 = () =>
  ((data[off++] << 24) | (data[off++] << 16) | (data[off++] << 8) | data[off++]) >>> 0;
const readVar = () => {
  let value = 0;
  for (;;) {
    const b = data[off++];
    value = (value << 7) | (b & 0x7f);
    if ((b & 0x80) === 0) return value;
  }
};

if (readStr(4) !== "MThd") throw new Error("not MIDI");
const headerLen = readU32();
readU16();
const trackCount = readU16();
const division = readU16();
off += headerLen - 6;

const tempos = [{ tick: 0, usPerQuarter: 500000 }];
const notes = [];
for (let trackIndex = 0; trackIndex < trackCount; trackIndex += 1) {
  if (readStr(4) !== "MTrk") throw new Error("missing track");
  const len = readU32();
  const end = off + len;
  let tick = 0;
  let status = 0;
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
      if (type === 0x51 && n === 3) {
        tempos.push({ tick, usPerQuarter: (payload[0] << 16) | (payload[1] << 8) | payload[2] });
      }
      continue;
    }
    if (b === 0xf0 || b === 0xf7) {
      off += readVar();
      continue;
    }
    const type = b & 0xf0;
    const ch = b & 0x0f;
    const d1 = data[off++];
    const d2 = type === 0xc0 || type === 0xd0 ? 0 : data[off++];
    if (type === 0x90 && d2 > 0) {
      const key = `${ch}:${d1}`;
      if (!active.has(key)) active.set(key, []);
      active.get(key).push({ tick, pitch: d1, trackIndex });
    } else if (type === 0x80 || (type === 0x90 && d2 === 0)) {
      const key = `${ch}:${d1}`;
      const stack = active.get(key);
      if (stack?.length) notes.push(stack.shift());
    }
  }
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
  let i = tempoMap.length - 1;
  while (i > 0 && tempoMap[i].tick > tick) i -= 1;
  const t = tempoMap[i];
  return t.ms + ((tick - t.tick) * t.usPerQuarter) / division / 1000;
}

const midiByMs = new Map();
for (const note of notes.filter((n) => n.trackIndex === 1)) {
  const ms = Math.round(tickToMs(note.tick));
  const list = midiByMs.get(ms) ?? [];
  list.push(note.pitch);
  midiByMs.set(ms, list);
}

const score = JSON.parse(fs.readFileSync(spacePath, "utf8"));
const timestamps = [];
function collect(value) {
  if (Array.isArray(value)) {
    for (const child of value) collect(child);
    return;
  }
  if (!value || typeof value !== "object") return;
  if (Array.isArray(value.noteTimestamp)) timestamps.push(...value.noteTimestamp);
  for (const child of Object.values(value)) if (child && typeof child === "object") collect(child);
}
collect(score);

const points = timestamps
  .map((entry) => ({
    start: Math.round(Number(entry.starTime ?? entry.startTime)),
    pitch: Array.isArray(entry.key?.[1]) ? Math.max(...entry.key[1]) : null,
  }))
  .filter((entry) => Number.isFinite(entry.start) && Number.isFinite(entry.pitch))
  .sort((a, b) => a.start - b.start);

let checked = 0;
let mismatched = 0;
for (const point of points) {
  const midi = midiByMs.get(point.start);
  if (!midi) continue;
  checked += 1;
  if (!midi.includes(point.pitch)) {
    mismatched += 1;
    if (mismatched <= 20) {
      console.log(`mismatch ms=${point.start} space=${point.pitch} midi=${midi.join(",")}`);
    }
  }
}

console.log(`checked=${checked}`);
console.log(`mismatched=${mismatched}`);
