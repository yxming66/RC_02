import fs from "node:fs";

const [htmlPath] = process.argv.slice(2);
if (!htmlPath) {
  console.error("usage: node extract_piastudy_search_records.mjs <html>");
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

const records = [];
const re = /component-export="default"[^>]+props="([^"]+)"/g;
for (const match of html.matchAll(re)) {
  const raw = htmlDecode(match[1]);
  let props;
  try {
    props = decodeObject(JSON.parse(raw));
  } catch {
    continue;
  }
  const recs = props.searchInfo?.records;
  if (Array.isArray(recs)) {
    records.push(...recs);
  }
}

for (const rec of records) {
  console.log(
    JSON.stringify({
      sheetCode: rec.sheetCode,
      title: rec.title,
      artist: rec.artist,
      author: rec.author,
      keynote: rec.keynote,
      difficulty: rec.difficulty,
      pagesCount: rec.pagesCount,
      views: rec.totalViews,
      screenshotUrl: rec.screenshotUrl,
    })
  );
}
