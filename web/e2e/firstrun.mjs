// The first visit, as a new visitor makes it: a browser profile that has
// never seen the app (a fresh context is an incognito window: no storage,
// no cache), the page loaded once, nothing reloaded or cleared, in several
// windows. The origin's coordinate system must be held by the view, drawn —
// in the canvas's pixels where its axes project — and seen, in the page's
// pixels there, clear of the view cube, with the welcome open, and after it
// is closed. The view must keep its room (`VIEW_MIN` of `src/layout.ts`)
// beside the panels docked by it, in the build as it is deployed: with the
// assistant offered. The welcome must show each example with its preview,
// its name and what it is, and the keys must open one.
//
// Part of `npm run e2e` (it runs first), and on its own: `npm run
// e2e:first-run`, which CI runs on every push. Both build `dist/` as it is
// deployed when it is stale (`buildAsDeployed`); `--build` rebuilds it.

import path from "node:path";
import { fileURLToPath } from "node:url";
import { Checks, expect, buildAsDeployed, launch, pixelAt, preview, settle, stale, watchErrors, WEB } from "./lib.mjs";
import { previewsStale, renderPreviews } from "./previews.mjs";
import { VIEW_MIN } from "../src/layout.ts";

/** The datums of the origin every part has: its coordinate system. */
const ORIGIN = "origin";

/**
 * The pixels of the view's canvas at the client points `at`, as drawn in
 * the next frame: read back right after the view renders it (a callback of
 * the same frame, registered after the view's), so what lies over the
 * canvas — the welcome — does not count. `[r, g, b, a]` each, `a` 0 where
 * nothing is drawn: the canvas is transparent over the viewport's backdrop.
 */
export function canvasPixels(page, at) {
  return page.evaluate(
    (at) =>
      new Promise((resolve) =>
        requestAnimationFrame(() => {
          const canvas = document.querySelector(".viewport canvas");
          const rect = canvas.getBoundingClientRect();
          const copy = document.createElement("canvas");
          copy.width = canvas.width;
          copy.height = canvas.height;
          const context = copy.getContext("2d");
          context.drawImage(canvas, 0, 0);
          const scale = canvas.width / rect.width;
          resolve(
            at.map(([x, y]) => [
              ...context.getImageData(Math.round((x - rect.left) * scale), Math.round((y - rect.top) * scale), 1, 1).data,
            ]),
          );
        }),
      ),
    at,
  );
}

/** How long the origin's axes are drawn, in pixels: `FRAME_PX` of `datums3d.ts`. */
const FRAME_PX = 90;

/**
 * Where the origin's axes are drawn on screen: for each of x, y and z,
 * client points along it, from a third to two thirds of its arrow, and a
 * little either side of it — the shaft is a few pixels thick. With
 * `whole`, points along all of it instead, from the origin to its tip.
 */
async function axisPoints(page, whole = false) {
  return page.evaluate(([whole, frame]) => {
    const { project } = window.geopView;
    const o = project([0, 0, 0]);
    const out = {};
    for (const [axis, dir] of [
      ["x", [1e-3, 0, 0]],
      ["y", [0, 1e-3, 0]],
      ["z", [0, 0, 1e-3]],
    ]) {
      const p = project(dir);
      const d = [p[0] - o[0], p[1] - o[1]];
      const n = Math.hypot(d[0], d[1]);
      if (!(n > 0)) {
        out[axis] = [];
        continue;
      }
      const u = [d[0] / n, d[1] / n];
      const points = [];
      const alongs = whole ? Array.from({ length: frame / 5 + 1 }, (_, i) => i * 5) : [30, 40, 50, 60];
      for (const along of alongs) {
        for (const across of whole ? [0] : [-1, 0, 1]) points.push([o[0] + u[0] * along - u[1] * across, o[1] + u[1] * along + u[0] * across]);
      }
      out[axis] = points;
    }
    return out;
  }, [whole, FRAME_PX]);
}

/** Whether the pixel `[r, g, b, a]` is the axis `axis`'s colour: drawn, and that channel ahead of the others. */
function axisColoured([r, g, b, a], axis) {
  if (a < 128) return false;
  const [own, others] = axis === "x" ? [r, [g, b]] : axis === "y" ? [g, [r, b]] : [b, [r, g]];
  return own > 120 && others.every((c) => own > c + 30);
}

/**
 * Throw unless the origin is held by the view, drawn — its axes in the
 * canvas's own pixels — and seen: its axes in the page's pixels, with
 * whatever lies over the view. Returns what it saw.
 */
export async function expectOriginDrawn(page) {
  const drawn = await page.evaluate(() => window.geopView?.datums?.() ?? null);
  if (drawn == null) {
    const canvases = await page.locator(".viewport canvas").count();
    throw new Error(
      canvases === 0
        ? "there is no view: the page drew no scene, so it has no canvas (it was sent none)"
        : "the view does not say what it draws: window.geopView.datums is missing",
    );
  }
  expect(drawn.includes(ORIGIN), `the view draws no origin; its datums: [${drawn.join(", ")}]`);
  const points = await axisPoints(page);
  const seen = [];
  for (const axis of ["x", "y", "z"]) {
    expect(points[axis].length > 0, `the origin's ${axis} axis points straight at the camera`);
    const pixels = await canvasPixels(page, points[axis]);
    const inCanvas = pixels.filter((p) => axisColoured(p, axis)).length;
    expect(inCanvas >= 3, `the origin's ${axis} axis is not in the canvas: of ${pixels.length} pixels along it ${inCanvas} are its colour (${JSON.stringify(pixels.slice(0, 6))}…)`);
    const shown = [];
    for (const [x, y] of points[axis]) shown.push([...(await pixelAt(page, Math.round(x), Math.round(y))), 255]);
    const onScreen = shown.filter((p) => axisColoured(p, axis)).length;
    if (onScreen < 3) {
      const [x, y] = points[axis][4];
      const over = await page.evaluate(([x, y]) => {
        const el = document.elementFromPoint(x, y);
        return el?.tagName === "CANVAS" ? null : `${el?.tagName}.${el?.className}`;
      }, [x, y]);
      throw new Error(`the origin's ${axis} axis is drawn but not seen: of ${shown.length} pixels on screen along it ${onScreen} are its colour (${JSON.stringify(shown.slice(0, 6))}…)${over ? `; over it lies ${over}` : ""}`);
    }
    seen.push(`${axis} ${inCanvas}/${onScreen}/${pixels.length}`);
  }
  // The view cube is drawn into the same canvas: it lies over none of the origin.
  const cube = await page.locator(".view-cube-area").boundingBox();
  if (cube) {
    const whole = await axisPoints(page, true);
    for (const axis of ["x", "y", "z"]) {
      const under = whole[axis].filter(([x, y]) => x >= cube.x && x <= cube.x + cube.width && y >= cube.y && y <= cube.y + cube.height);
      expect(under.length === 0, `the view cube (${JSON.stringify(cube)}) lies over the origin's ${axis} axis, at ${JSON.stringify(under[0])}`);
    }
  }
  return `datums [${drawn.join(", ")}], axis pixels drawn/seen ${seen.join(", ")}`;
}

/**
 * Throw unless the view keeps its room beside the panels, as the build is
 * deployed: it offers the assistant, the view is `VIEW_MIN` wide or wider,
 * and nothing lies over it on a first visit. Returns what it saw.
 */
async function expectViewRoom(page) {
  const offered = (await page.locator(".assistant-toggle").count()) > 0;
  expect(
    offered,
    "this build does not offer the assistant, as the deployed one does: build it with VITE_TURNSTILE_SITE_KEY set (npm run e2e:first-run -- --build does)",
  );
  const view = await page.locator("main.viewport").boundingBox();
  expect(view != null && view.width >= VIEW_MIN, `the view is ${view?.width} px wide, less than ${VIEW_MIN}`);
  const over = await page.locator(".assistant-over").count();
  expect(over === 0, "the assistant lies over the view on a first visit");
  const docked = await page.evaluate(() =>
    [".explorer-bar", ".assistant-bar"].filter((s) => document.querySelector(s) != null).map((s) => s.slice(1)),
  );
  return `the view ${Math.round(view.width)}×${Math.round(view.height)}, beside [${["sidebar", ...docked].join(", ")}]`;
}

/** The windows a first visit is checked in: desktops from wide to small, and a phone. */
const WINDOWS = [
  { name: "1440×900", viewport: { width: 1440, height: 900 } },
  { name: "1400×900", viewport: { width: 1400, height: 900 } },
  { name: "1366×768", viewport: { width: 1366, height: 768 } },
  { name: "1280×720", viewport: { width: 1280, height: 720 } },
  { name: "1024×640", viewport: { width: 1024, height: 640 } },
  { name: "a phone", viewport: { width: 390, height: 844 }, isMobile: true, hasTouch: true },
];

/**
 * The first-visit checks, each window in a browser context of its own,
 * made afresh: as an incognito window, it has no storage and no cache.
 * Returns the results, as `Checks` keeps them.
 */
export async function firstRunChecks(browser, url) {
  // The welcome shows the examples' previews, rendered from this build.
  if (previewsStale()) await renderPreviews(browser, url);
  const results = [];
  for (const { name: window, ...options } of WINDOWS) {
    const context = await browser.newContext(options);
    const page = await context.newPage();
    const checks = new Checks(page, watchErrors(page), "first-run-");
    await checks.check(`a first visit in ${window}, from a fresh profile, shows the origin with the welcome open`, async () => {
      const stored = await page.evaluate(() => localStorage.length).catch(() => 0);
      expect(stored === 0, `the profile is not fresh: ${stored} items stored`);
      await page.goto(url);
      await settle(page);
      expect(await page.locator(".welcome-card").isVisible(), "no welcome on a first visit");
      const room = options.isMobile ? "" : `${await expectViewRoom(page)}; `;
      return `${room}${await expectOriginDrawn(page)}`;
    });
    await checks.check(`a first visit in ${window}: closing the welcome leaves the origin shown`, async () => {
      await page.locator(".welcome-card button[aria-label='Close']").click();
      expect((await page.locator(".welcome-card").count()) === 0, "the welcome is still open");
      await settle(page);
      return expectOriginDrawn(page);
    });
    if (!options.isMobile) {
      await checks.check(`a first visit in ${window}: the welcome shows each example's preview, name and what it is, and the keys open one`, async () => {
        // The welcome stays closed only for the page that closed it.
        await page.reload();
        await settle(page);
        const cards = page.locator(".welcome-example");
        const n = await cards.count();
        expect(n >= 3, `${n} examples in the welcome`);
        const shown = await cards.evaluateAll((els) =>
          els.map((el) => {
            const img = el.querySelector("img");
            return {
              name: el.querySelector(".welcome-example-name")?.textContent ?? "",
              about: el.querySelector(".welcome-example-about")?.textContent ?? "",
              loaded: img != null && img.complete && img.naturalWidth > 0,
            };
          }),
        );
        const bad = shown.filter((s) => !s.name || !s.about || !s.loaded);
        expect(bad.length === 0, `examples without a name, a description or a loaded preview: ${JSON.stringify(bad)}`);
        // The first example has the focus, the arrows move it, Enter opens it.
        await page.keyboard.press("ArrowRight");
        const focused = await page.evaluate(() => document.activeElement?.querySelector(".welcome-example-name")?.textContent ?? null);
        expect(focused === shown[1].name, `the arrow moved the focus to ${focused}, not ${shown[1].name}`);
        await page.keyboard.press("Enter");
        await settle(page);
        const steps = await page.locator(".timeline .step-box").count();
        expect(steps > 0, `Enter on ${focused} opened no example`);
        expect((await page.locator(".welcome-card").count()) === 0, "the welcome is still open over the example");
        return `${n} examples: ${shown.map((s) => s.name).join(", ")}; Enter opened ${focused} (${steps} steps)`;
      });
    }
    results.push(...checks.results);
    await context.close();
  }
  return results;
}

// On its own: build what is stale, serve it, run the checks.
if (process.argv[1] === fileURLToPath(import.meta.url)) {
  const browser = await launch();
  if (stale(path.join(WEB, "dist", "index.html"))) buildAsDeployed();
  const server = await preview();
  try {
    const checks = new Checks(null, []);
    checks.results.push(...(await firstRunChecks(browser, server.url)));
    checks.report();
  } finally {
    await browser.close();
    server.stop();
  }
}
