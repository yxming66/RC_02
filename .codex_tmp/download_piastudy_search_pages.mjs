import fs from "node:fs";
import https from "node:https";

const [query, pageCountArg, outPrefix] = process.argv.slice(2);
const pageCount = Number(pageCountArg);
if (!query || !Number.isFinite(pageCount) || !outPrefix) {
  console.error(
    "usage: node download_piastudy_search_pages.mjs <query> <page-count> <out-prefix>"
  );
  process.exit(2);
}

function get(url) {
  return new Promise((resolve, reject) => {
    https
      .get(
        url,
        {
          headers: {
            "User-Agent": "Mozilla/5.0 codex-local",
            Accept: "text/html",
          },
        },
        (res) => {
          if (
            [301, 302, 303, 307, 308].includes(res.statusCode) &&
            res.headers.location
          ) {
            res.resume();
            resolve(get(new URL(res.headers.location, url).href));
            return;
          }
          if (res.statusCode !== 200) {
            res.resume();
            reject(new Error(`HTTP ${res.statusCode} ${url}`));
            return;
          }
          const chunks = [];
          res.on("data", (chunk) => chunks.push(chunk));
          res.on("end", () => resolve(Buffer.concat(chunks)));
        }
      )
      .on("error", reject);
  });
}

for (let page = 1; page <= pageCount; page += 1) {
  const url = `https://piastudy.com/searchResults/1/${encodeURIComponent(
    query
  )}/0/0/${page}`;
  const body = await get(url);
  const outPath = `${outPrefix}_${page}.html`;
  fs.writeFileSync(outPath, body);
  console.log(outPath);
}
