import fs from "node:fs";

const file = process.argv[2];
if (!file) {
  console.error("usage: node inspect_midi.mjs <midi>");
  process.exit(2);
}

const data = fs.readFileSync(file);
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

const header = readStr(4);
if (header !== "MThd") throw new Error("not a MIDI file");
const hlen = readU32();
const format = readU16();
const tracks = readU16();
const division = readU16();
off += hlen - 6;
console.log({ format, tracks, division });

for (let t = 0; t < tracks; t += 1) {
  const id = readStr(4);
  const len = readU32();
  const end = off + len;
  let tick = 0;
  let status = 0;
  const active = new Map();
  const notes = [];
  const tempos = [];
  const programs = new Set();
  let name = "";
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
      if (type === 0x03 || type === 0x04) {
        const s = payload.toString("utf8").trim();
        if (s) name += `${name ? " / " : ""}${s}`;
      } else if (type === 0x51 && n === 3) {
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
    const needs2 = type !== 0xc0 && type !== 0xd0;
    const d2 = needs2 ? data[off++] : 0;
    if (type === 0xc0) {
      programs.add(`${ch}:${d1}`);
    } else if (type === 0x90 && d2 > 0) {
      const key = `${ch}:${d1}`;
      if (!active.has(key)) active.set(key, []);
      active.get(key).push({ tick, vel: d2, note: d1, ch });
    } else if (type === 0x80 || (type === 0x90 && d2 === 0)) {
      const key = `${ch}:${d1}`;
      const stack = active.get(key);
      if (stack && stack.length) {
        const start = stack.shift();
        notes.push({ note: d1, ch, start: start.tick, end: tick, vel: start.vel });
      }
    }
  }
  off = end;
  notes.sort((a, b) => a.start - b.start || b.note - a.note || a.end - b.end);
  const minNote = notes.length ? Math.min(...notes.map((n) => n.note)) : null;
  const maxNote = notes.length ? Math.max(...notes.map((n) => n.note)) : null;
  const maxTick = notes.length ? Math.max(...notes.map((n) => n.end)) : 0;
  console.log(`track ${t}: id=${id} len=${len} name="${name}" notes=${notes.length} range=${minNote}-${maxNote} maxTick=${maxTick}`);
  if (tempos.length) console.log("  tempos", tempos.slice(0, 8));
  if (programs.size) console.log("  programs", [...programs].join(", "));
  console.log("  first", notes.slice(0, 20).map(n => `${n.note}@${n.start}+${n.end-n.start}`).join(" "));
}
