// Records the README's demo: a new part, a rectangle sketched by clicks,
// extruded, one edge filleted, and the extrusion's distance changed so the
// fillet follows. A screenshot of each state lands in `e2e/out/demo/`, and
// `ffmpeg` (if installed) assembles them into `docs/demo.gif`.
//
//   npm run demo
//
// Drives the built app (`dist/`, rebuilt if stale) in a headless Chrome,
// as `npm run e2e` does; skips (exit code 0) if there is no Chrome. Flags:
// `--build` rebuilds dist/ even if it looks current.

import { spawnSync } from "node:child_process";
import fs from "node:fs";
import path from "node:path";

import { ROOT, WEB, clickOperation, launch, preview, run, settle, stale, watchErrors } from "./lib.mjs";

const OUT = path.join(WEB, "e2e", "out", "demo");
const GIF = path.join(ROOT, "docs", "demo.gif");

if (stale(path.join(WEB, "dist", "index.html"))) run("npm", ["run", "build"]);
fs.rmSync(OUT, { recursive: true, force: true });
fs.mkdirSync(OUT, { recursive: true });

const browser = await launch();
const server = await preview();
// The viewport the e2e checks click in: the plane and the rectangle's
// corners below are pixel offsets from its centre.
const context = await browser.newContext({ viewport: { width: 1400, height: 900 }, deviceScaleFactor: 1 });
const page = await context.newPage();
const errors = watchErrors(page);

/** The frames taken so far, with how long each is shown, in seconds. */
const frames = [];
async function frame(name, seconds = 1.6) {
  await settle(page);
  // A moment for the viewer to draw what the kernel last sent.
  await page.waitForTimeout(400);
  const file = path.join(OUT, `${String(frames.length + 1).padStart(2, "0")}-${name}.png`);
  await page.screenshot({ path: file });
  frames.push({ file, seconds });
  console.log(`frame ${path.relative(WEB, file)}`);
}

const popup = page.locator(".desktop-only .popup");

/** Click the viewport `dx`, `dy` pixels from its centre, hovering there first so the kernel knows where the pointer is. */
async function clickView(dx, dy) {
  const box = await page.locator("main.viewport canvas").first().boundingBox();
  const [x, y] = [box.x + box.width / 2 + dx, box.y + box.height / 2 + dy];
  await page.mouse.move(x, y);
  await settle(page);
  await page.mouse.click(x, y);
  await settle(page);
}

/** The step being edited, as the kernel presents it. */
const presented = () => page.evaluate(async () => (await window.geopCommand({ command: "show" }))?.step?.presentation);

/** Type `value` into the popup's field labelled `label`. */
async function fill(label, value) {
  const input = popup.locator(".field", { hasText: label }).locator("input[type=number], input[type=text]").first();
  await input.fill(String(value));
  await input.press("Enter");
  await settle(page);
}

async function commit() {
  await popup.locator(".button-row button.primary").click();
  await settle(page);
}

/**
 * The part's edges as drawn, by name with their polylines: the scene is
 * sent only when it changes, so the program is run to its first step and
 * back, which changes it twice and leaves everything as it was.
 */
async function edges() {
  return page.evaluate(async () => {
    await window.geopCommand({ command: "seek", marker: 1 });
    const update = await window.geopCommand({ command: "seek", marker: null });
    return update.scene.part.edges.map((e) => ({ name: e.name, polyline: e.polyline }));
  });
}

/**
 * Click an edge of the part for the fillet: the candidates are the edges
 * whose middle projects into the view (`geopView.project` gives client
 * coordinates), the longest on screen first, so the fillet shows; each is
 * hovered, and the first the kernel says a click would pick is clicked.
 */
async function pickAnEdge() {
  const box = await page.locator("main.viewport canvas").first().boundingBox();
  const candidates = [];
  for (const edge of await edges()) {
    const middle = edge.polyline[Math.floor(edge.polyline.length / 2)];
    const [x, y] = await page.evaluate((p) => window.geopView.project(p), middle);
    const [[x0, y0], [x1, y1]] = await page.evaluate(
      (ends) => ends.map((p) => window.geopView.project(p)),
      [edge.polyline[0], edge.polyline[edge.polyline.length - 1]],
    );
    candidates.push({ name: edge.name, x, y, length: Math.hypot(x1 - x0, y1 - y0) });
  }
  candidates.sort((a, b) => b.length - a.length);
  for (const c of candidates) {
    if (c.x < box.x || c.x > box.x + box.width || c.y < box.y || c.y > box.y + box.height) continue;
    await page.mouse.move(c.x, c.y);
    await settle(page);
    const lit = await page.evaluate(() => window.geopLit ?? {});
    if (!Object.values(lit).some((l) => l.edges.length > 0)) continue;
    await page.mouse.click(c.x, c.y);
    await settle(page);
    const edges = (await presented())?.dialog.find((f) => f.key === "edges")?.value ?? [];
    if (edges.length === 1) return edges[0].element.name;
  }
  throw new Error("no edge of the part could be picked");
}

try {
  // A new visitor: nothing kept, the robust operations offered.
  await page.goto(server.url);
  await page.evaluate(() => localStorage.clear());
  await page.reload();
  await settle(page);
  await frame("new-part", 1.2);

  // The sketch: the zx plane picked below and right of the origin, a
  // rectangle by its two corners.
  await clickOperation(page, "Sketch");
  await settle(page);
  await clickView(8, 23);
  await page.keyboard.press("r");
  await settle(page);
  await clickView(-270, -190);
  await clickView(-120, -70);
  await frame("sketch");
  await commit();

  await clickOperation(page, "Extrude");
  await settle(page);
  await fill("distance", 2);
  await frame("extrude");
  await commit();
  await frame("solid", 1.2);

  await clickOperation(page, "Fillet");
  await settle(page);
  const edge = await pickAnEdge();
  console.log(`filleting ${edge}`);
  await fill("radius", 1.2);
  await frame("fillet");
  await commit();
  await frame("filleted");

  // The extrusion's distance changed: the fillet follows.
  await page.evaluate(() => window.geopCommand({ command: "open", id: "extrude1" }));
  await settle(page);
  await fill("distance", 6);
  await frame("rebuilt");
  await commit();
  await frame("done", 2.4);

  const shown = await page.locator(".error, .op-error-text").allInnerTexts();
  if (shown.length > 0) throw new Error(`errors shown: ${shown.join(" | ")}`);
  if (errors.length > 0) throw new Error(`page errors:\n${errors.join("\n")}`);
} finally {
  await browser.close();
  server.stop();
}

// The GIF: the frames in order, each for its time, 960 px wide, one palette.
const ffmpeg = spawnSync("ffmpeg", ["-version"], { stdio: "ignore" });
if (ffmpeg.status !== 0) {
  console.log(`no ffmpeg: the frames are in ${path.relative(ROOT, OUT)}; assemble ${path.relative(ROOT, GIF)} from them`);
} else {
  const list = path.join(OUT, "frames.txt");
  const lines = frames.flatMap((f) => [`file '${f.file}'`, `duration ${f.seconds}`]);
  // The concat demuxer shows the last frame only if it is named again.
  lines.push(`file '${frames[frames.length - 1].file}'`);
  fs.writeFileSync(list, lines.join("\n") + "\n");
  fs.mkdirSync(path.dirname(GIF), { recursive: true });
  const filter =
    "fps=5,scale=960:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle";
  run("ffmpeg", ["-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", list, "-vf", filter, "-loop", "0", GIF], ROOT);
  console.log(`${path.relative(ROOT, GIF)}: ${(fs.statSync(GIF).size / 1e6).toFixed(2)} MB, ${frames.length} frames`);
}
