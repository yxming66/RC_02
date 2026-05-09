import fs from "node:fs";
import https from "node:https";

const [url, out] = process.argv.slice(2);
if (!url || !out) {
  console.error("usage: node download_url.mjs <url> <out>");
  process.exit(2);
}

function download(src, dest, redirects = 0) {
  https.get(src, { headers: { "User-Agent": "codex-local" } }, (res) => {
    if ([301, 302, 303, 307, 308].includes(res.statusCode) && res.headers.location) {
      if (redirects > 5) {
        console.error("too many redirects");
        process.exit(1);
      }
      download(new URL(res.headers.location, src).href, dest, redirects + 1);
      return;
    }
    if (res.statusCode !== 200) {
      console.error(`HTTP ${res.statusCode}`);
      process.exit(1);
    }
    const file = fs.createWriteStream(dest);
    res.pipe(file);
    file.on("finish", () => file.close(() => console.log(dest)));
  }).on("error", (err) => {
    console.error(err);
    process.exit(1);
  });
}

download(url, out);
