import fs from "node:fs";

const [htmlPath] = process.argv.slice(2);
if (!htmlPath) {
  console.error("usage: node extract_piastudy_detail.mjs <html>");
  process.exit(2);
}

const html = fs.readFileSync(htmlPath, "utf8");

function htmlDecode(s) {
  return s
    .replaceAll("&quot;", '"')
    .replaceAll("&amp;", "&")
    .replaceAll("&lt;", "<")
    .replaceAll("&gt;", ">")
    .replaceAll("&#38;", "&");
}

function decodeAstro(value) {
  if (!Array.isArray(value) || value.length < 2 || typeof value[0] !== "number") {
    return value;
  }
  const [type, data] = value;
  switch (type) {
    case 0:
      return decodeObject(data);
    case 1:
      return data.map(decodeAstro);
    case 2:
      return new RegExp(data);
    case 3:
      return new Date(data);
    case 4:
      return new Map(data.map(decodeAstro));
    case 5:
      return new Set(data.map(decodeAstro));
    case 6:
      return BigInt(data);
    case 7:
      return new URL(data);
    case 8:
      return new Uint8Array(data);
    case 9:
      return new Uint16Array(data);
    case 10:
      return new Uint32Array(data);
    case 11:
      return data === 1 ? Infinity : -Infinity;
    default:
      return data;
  }
}

function decodeObject(value) {
  if (Array.isArray(value)) {
    return value.map(decodeAstro);
  }
  if (value === null || typeof value !== "object") {
    return value;
  }
  const out = {};
  for (const [key, entry] of Object.entries(value)) {
    out[key] = decodeAstro(entry);
  }
  return out;
}

const re = /component-export="default"[^>]+props="([^"]+)"/g;
for (const match of html.matchAll(re)) {
  const raw = htmlDecode(match[1]);
  let props;
  try {
    props = decodeObject(JSON.parse(raw));
  } catch {
    continue;
  }
  if (props.sheetData) {
    const s = props.sheetData;
    console.log(JSON.stringify({
      sheetCode: s.sheetCode,
      title: s.title,
      artist: s.artist,
      author: s.author,
      compose: s.compose,
      keynote: s.keynote,
      difficulty: s.difficulty,
      pagesCount: s.pagesCount,
      imageType: s.imageType,
      imagePathUrl: s.imagePathUrl,
      scoreUrl: s.scoreUrl,
      midiUrl: s.midiUrl,
      spaceUrl: s.spaceUrl,
      totalImitationCount: s.totalImitationCount,
    }, null, 2));
    process.exit(0);
  }
}

console.error("sheetData not found");
process.exit(1);
