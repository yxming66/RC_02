import fs from "node:fs";
import path from "node:path";
import { createRequire } from "node:module";
import * as pdfjsLib from "pdfjs-dist/legacy/build/pdf.mjs";

const require = createRequire(import.meta.url);
const pdfjsPath = require.resolve("pdfjs-dist/legacy/build/pdf.mjs");
const pdfjsRequire = createRequire(pdfjsPath);
const { createCanvas } = pdfjsRequire("@napi-rs/canvas");

const [pdfPath, outDir] = process.argv.slice(2);
if (!pdfPath || !outDir) {
  console.error("usage: node render_pdf_pages.mjs <pdf> <out-dir>");
  process.exit(2);
}

fs.mkdirSync(outDir, { recursive: true });

const loadingTask = pdfjsLib.getDocument({
  data: new Uint8Array(fs.readFileSync(pdfPath)),
  useWorkerFetch: false,
  isEvalSupported: false,
  disableFontFace: true,
});

const pdf = await loadingTask.promise;
console.log(`pages=${pdf.numPages}`);

for (let pageNo = 1; pageNo <= pdf.numPages; pageNo += 1) {
  const page = await pdf.getPage(pageNo);
  const viewport = page.getViewport({ scale: 2.0 });
  const canvas = createCanvas(Math.ceil(viewport.width), Math.ceil(viewport.height));
  const context = canvas.getContext("2d");
  const proto = Object.getPrototypeOf(context);
  if (!proto.__codexClipPatched) {
    const originalClip = proto.clip;
    const originalFill = proto.fill;
    const originalStroke = proto.stroke;
    proto.clip = function patchedClip(...args) {
      try {
        return originalClip.apply(this, args);
      } catch (err) {
        if (args.length > 0) {
          return originalClip.call(this);
        }
        throw err;
      }
    };
    proto.fill = function patchedFill(...args) {
      try {
        return originalFill.apply(this, args);
      } catch (err) {
        if (args.length > 0) {
          return originalFill.call(this);
        }
        throw err;
      }
    };
    proto.stroke = function patchedStroke(...args) {
      try {
        return originalStroke.apply(this, args);
      } catch (err) {
        if (args.length > 0) {
          return originalStroke.call(this);
        }
        throw err;
      }
    };
    proto.__codexClipPatched = true;
  }
  await page.render({ canvasContext: context, viewport }).promise;
  const outPath = path.join(outDir, `page-${String(pageNo).padStart(2, "0")}.png`);
  fs.writeFileSync(outPath, canvas.toBuffer("image/png"));
  console.log(outPath);
}
