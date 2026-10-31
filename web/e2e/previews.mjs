// The welcome's previews of the featured examples (`src/examples.ts`):
// each example opened in the built app, in headless Chrome, as a shared
// link opens it (`?example=<name>`), and the view's canvas saved, cropped
// and scaled, to `dist/examples/<name>.webp`. Rendered from what the
// example builds now, when the app is built: never committed, never stale.
//
// `npm run build:previews` after `npm run build`; the end-to-end checks
// render them when they are missing or older than the build, and deploying
// renders them before it uploads `dist/`.

import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { FEATURED } from "../src/examples.ts";
import { launch, preview, settle, WEB } from "./lib.mjs";

/** Where the previews go. */
export const PREVIEWS = path.join(WEB, "dist", "examples");
/** A preview's size, in pixels: the welcome's, 80 × 50, three times over for a sharp screen. */
const WIDTH = 240;
const HEIGHT = 150;
/** The most bytes a preview may take: the welcome loads all of them on a first visit. */
export const BUDGET = 20_000;

/** Where the preview of `name` goes. */
const previewPath = (name) => path.join(PREVIEWS, `${name}.webp`);

/** Whether a preview is missing, or older than the build it shows. */
export function previewsStale() {
  const built = path.join(WEB, "dist", "index.html");
  if (!fs.existsSync(built)) return true;
  const since = fs.statSync(built).mtimeMs;
  return FEATURED.some(({ name }) => !fs.existsSync(previewPath(name)) || fs.statSync(previewPath(name)).mtimeMs < since);
}

/** Wait until the camera has stopped moving: the view framed what was opened. */
async function still(page) {
  let last = null;
  for (let i = 0; i < 100; i++) {
    const now = JSON.stringify(await page.evaluate(() => window.geopView?.pose()));
    if (now === last) return;
    last = now;
    await page.waitForTimeout(150);
  }
}

/** Render every featured example's preview with `browser`, the built app served at `url`. */
export async function renderPreviews(browser, url) {
  fs.mkdirSync(PREVIEWS, { recursive: true });
  const written = [];
  for (const { name } of FEATURED) {
    // A context of its own: each example as a first visit opens it.
    const context = await browser.newContext({ viewport: { width: 1000, height: 760 } });
    const page = await context.newPage();
    await page.goto(`${url}?example=${name}`);
    await settle(page);
    const steps = await page.locator(".timeline .step-box").count();
    if (steps === 0) throw new Error(`the example ${name} did not open`);
    await still(page);
    // Read back right after the view renders the frame (see `canvasPixels`
    // in `firstrun.mjs`): the canvas alone, without what lies over it.
    const data = await page.evaluate(
      ([width, height]) =>
        new Promise((resolve) =>
          requestAnimationFrame(() => {
            const canvas = document.querySelector(".viewport canvas");
            // The middle of the view, at the preview's aspect.
            const scale = Math.min(canvas.width / width, canvas.height / height) * 0.9;
            const [w, h] = [width * scale, height * scale];
            const out = document.createElement("canvas");
            out.width = width;
            out.height = height;
            const context = out.getContext("2d");
            context.imageSmoothingQuality = "high";
            context.drawImage(canvas, (canvas.width - w) / 2, (canvas.height - h) / 2, w, h, 0, 0, width, height);
            resolve(out.toDataURL("image/webp", 0.8));
          }),
        ),
      [WIDTH, HEIGHT],
    );
    await context.close();
    const bytes = Buffer.from(data.slice(data.indexOf(",") + 1), "base64");
    if (!data.startsWith("data:image/webp")) throw new Error(`the browser wrote ${name}'s preview as ${data.slice(0, 30)}`);
    if (bytes.length > BUDGET) throw new Error(`${name}'s preview takes ${bytes.length} bytes, over the budget of ${BUDGET}`);
    fs.writeFileSync(previewPath(name), bytes);
    written.push(`${name} (${(bytes.length / 1000).toFixed(1)} kB)`);
  }
  console.log(`previews: ${written.join(", ")}`);
}

// On its own: render them from `dist/`, which must be built.
if (process.argv[1] === fileURLToPath(import.meta.url)) {
  if (!fs.existsSync(path.join(WEB, "dist", "index.html"))) throw new Error("dist/ is not built: npm run build first");
  const browser = await launch();
  const server = await preview();
  try {
    await renderPreviews(browser, server.url);
  } finally {
    await browser.close();
    server.stop();
  }
}
