// What the end-to-end checks share: finding a Chrome to drive, building
// what is stale, serving a folder on a free port, waiting for the editor to
// settle, and running named checks that save a screenshot when they fail.
//
// The checks drive the real app in a real (headless) browser through
// `playwright-core`, which brings no browser of its own: a system Chrome or
// Chromium is used, from `$CHROME` or the usual places.

import { spawn, spawnSync } from "node:child_process";
import fs from "node:fs";
import net from "node:net";
import path from "node:path";
import { fileURLToPath } from "node:url";
import zlib from "node:zlib";

/** `web/`. */
export const WEB = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
/** The repository. */
export const ROOT = path.resolve(WEB, "..");
/** Where failing checks leave their screenshots (gitignored). */
export const OUT = path.join(WEB, "e2e", "out");

/** The Chrome to drive: `$CHROME`, or the first of the usual places that exists; `null` if none does. */
export function findChrome() {
  if (process.env.CHROME) return process.env.CHROME;
  const candidates = [
    "/usr/bin/google-chrome",
    "/usr/bin/google-chrome-stable",
    "/usr/bin/chromium",
    "/usr/bin/chromium-browser",
    "/snap/bin/chromium",
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
    "/Applications/Chromium.app/Contents/MacOS/Chromium",
    "C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe",
  ];
  return candidates.find((c) => fs.existsSync(c)) ?? null;
}

/** Headless Chrome, rendering WebGL in software (SwiftShader), so it needs no GPU. Exits the process — successfully — when there is no Chrome to drive. */
export async function launch() {
  const executablePath = findChrome();
  if (!executablePath) {
    console.log("SKIPPED: no Chrome or Chromium found. Set CHROME to its executable to run the end-to-end checks.");
    process.exit(0);
  }
  const { chromium } = await import("playwright-core");
  return chromium.launch({ executablePath, args: ["--use-gl=swiftshader", "--enable-unsafe-swiftshader"] });
}

/** The newest modification time of the files under `roots` whose names match `pattern`, skipping build output and dependencies. */
function newest(roots, pattern) {
  let latest = 0;
  const walk = (dir) => {
    for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
      if (["node_modules", "target", "dist", "pkg", ".git", "out"].includes(entry.name)) continue;
      const p = path.join(dir, entry.name);
      if (entry.isDirectory()) walk(p);
      else if (pattern.test(entry.name)) latest = Math.max(latest, fs.statSync(p).mtimeMs);
    }
  };
  for (const root of roots) {
    if (!fs.existsSync(root)) continue;
    if (fs.statSync(root).isDirectory()) walk(root);
    else latest = Math.max(latest, fs.statSync(root).mtimeMs);
  }
  return latest;
}

/** The sources the web app and the CLI are built from: the kernel's crates and the front end. */
export const SOURCES = ["core", "ops", "cad", "web/src"].map((d) => path.join(ROOT, d));
const SOURCE_FILES = /\.(rs|toml|ts|tsx|css|html)$/;

/** Whether `output` is missing or older than any of the sources (or `--build` was given). */
export function stale(output) {
  if (process.argv.includes("--build") || !fs.existsSync(output)) return true;
  return fs.statSync(output).mtimeMs < newest([...SOURCES, path.join(ROOT, "Cargo.lock")], SOURCE_FILES);
}

/** Run `command` with `args` in `cwd`, showing its output; throws if it fails. */
export function run(command, args, cwd = WEB) {
  console.log(`$ ${command} ${args.join(" ")}`);
  const result = spawnSync(command, args, { cwd, stdio: "inherit", env: { CARGO_BUILD_JOBS: "6", ...process.env } });
  if (result.status !== 0) throw new Error(`${command} ${args.join(" ")} failed (${result.status ?? result.signal})`);
}

/** A TCP port nobody listens on right now. */
export function freePort() {
  return new Promise((resolve, reject) => {
    const server = net.createServer();
    server.unref();
    server.on("error", reject);
    server.listen(0, "127.0.0.1", () => {
      const { port } = server.address();
      server.close(() => resolve(port));
    });
  });
}

/** `web/dist` served by `vite preview` on a free port: its URL, and how to stop it. */
export async function preview() {
  const port = await freePort();
  const vite = path.join(WEB, "node_modules", ".bin", "vite");
  const child = spawn(vite, ["preview", "--host", "127.0.0.1", "--port", String(port), "--strictPort"], {
    cwd: WEB,
    stdio: ["ignore", "pipe", "inherit"],
  });
  const url = `http://127.0.0.1:${port}/`;
  for (let i = 0; ; i++) {
    if (child.exitCode != null) throw new Error(`vite preview exited (${child.exitCode})`);
    try {
      if ((await fetch(url)).ok) break;
    } catch {
      // Not up yet.
    }
    if (i > 200) throw new Error("vite preview did not come up");
    await new Promise((r) => setTimeout(r, 100));
  }
  return { url, stop: () => child.kill() };
}

/**
 * Collects the page's errors — uncaught exceptions and console errors — so
 * a check can ask which happened while it ran.
 */
export function watchErrors(page) {
  const errors = [];
  page.on("pageerror", (e) => errors.push(String(e)));
  page.on("console", (m) => {
    if (m.type() === "error") errors.push(`console: ${m.text()}`);
  });
  return errors;
}

/**
 * Wait until the editor is idle: the wasm module loaded, no command in
 * flight (`aria-busy` on the app), and the stats in the toolbar unchanged
 * for a moment — a command's answer can start the next (loading an example
 * opens its file first). Returns the stats.
 */
export async function settle(page, timeout = 120000) {
  await page.waitForSelector(".stats", { timeout });
  const start = Date.now();
  let last = null;
  let since = Date.now();
  for (;;) {
    const now = await page.evaluate(() => {
      const busy = document.querySelector(".app")?.getAttribute("aria-busy") === "true";
      return busy ? null : (document.querySelector(".stats")?.textContent ?? null);
    });
    if (now === null || now !== last) {
      last = now;
      since = Date.now();
    } else if (Date.now() - since > 200) {
      return now;
    }
    if (Date.now() - start > timeout) throw new Error(`the editor did not settle within ${timeout / 1000} s`);
    await page.waitForTimeout(100);
  }
}

/** The errors the app shows: a refused command, a step's error, a failed field. */
export function shownErrors(page) {
  return page.locator(".error, .op-error-text").allInnerTexts();
}

/** The colour of the page's pixel at `x`, `y`: `[r, g, b]`, from a screenshot of it (a PNG, read here). */
export async function pixelAt(page, x, y) {
  const png = await page.screenshot({ clip: { x, y, width: 1, height: 1 } });
  // Chunks after the 8-byte signature: length, type, data, CRC.
  const data = [];
  for (let at = 8; at < png.length; ) {
    const length = png.readUInt32BE(at);
    if (png.toString("ascii", at + 4, at + 8) === "IDAT") data.push(png.subarray(at + 8, at + 8 + length));
    at += 12 + length;
  }
  // One row: its filter byte — every filter leaves the first pixel of the
  // first row as it is — then RGB(A).
  const row = zlib.inflateSync(Buffer.concat(data));
  return [row[1], row[2], row[3]];
}

/** Pick the entry `label` — exactly that — from the app's File menu. */
export async function fileMenu(page, label) {
  const trigger = page.locator(".file-menu .dropdown-trigger");
  if ((await trigger.getAttribute("aria-expanded")) !== "true") await trigger.click();
  await page
    .locator(".file-menu .dropdown-item", { has: page.locator(`.dropdown-item-label:text-is("${label}")`) })
    .click();
}

/** `text`, matched literally in a regular expression. */
const literal = (text) => text.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");

/** The labels of every operation (and editor tool) the app offers, as the phone's tab of them lists them all. */
export async function operationLabels(page) {
  return (await page.locator(".mobile-bottom .operation-grid button.op-button").allInnerTexts()).map((l) => l.trim());
}

/**
 * Click the operation `label` in the desktop toolbar: its button, or — where
 * the window has no room for one — its entry in the menu of its group, or
 * of the overflow.
 */
export async function clickOperation(page, label) {
  const button = page.locator(".operation-ribbon button.op-button", { hasText: new RegExp(`^${literal(label)}$`) });
  if ((await button.count()) > 0) return button.click();
  const triggers = page.locator(".operation-ribbon .dropdown-trigger");
  const n = await triggers.count();
  for (let i = 0; i < n; i++) {
    await triggers.nth(i).click();
    const item = page.locator(".operation-ribbon .dropdown-menu .dropdown-item", {
      has: page.locator(`.dropdown-item-label:text-is("${label}")`),
    });
    if ((await item.count()) > 0) return item.click();
    await page.keyboard.press("Escape");
  }
  throw new Error(`${label} is nowhere in the toolbar`);
}

/** Where a check's screenshot goes: `e2e/out/<prefix><name>.png`. */
function shotPath(prefix, name) {
  fs.mkdirSync(OUT, { recursive: true });
  return path.join(OUT, `${prefix}${name.replace(/[^\w.-]+/g, "_").slice(0, 80)}.png`);
}

/**
 * Named checks, run one after another: each passes unless it throws, or the
 * page reported an error while it ran. A failing check saves a screenshot
 * of the page to `e2e/out/`. `report()` prints the summary and sets the exit
 * code.
 *
 * From the command line, `--only <text>` runs only the checks whose name
 * contains the text, and `--shots` saves a screenshot after every check.
 */
export class Checks {
  constructor(page, errors, prefix = "") {
    this.page = page;
    this.errors = errors;
    this.prefix = prefix;
    this.results = [];
    const only = process.argv.indexOf("--only");
    this.only = only >= 0 ? process.argv[only + 1] : null;
    this.shots = process.argv.includes("--shots");
  }

  async check(name, body) {
    if (this.only && !name.includes(this.only)) return true;
    const before = this.errors.length;
    const start = Date.now();
    let failure = null;
    let label = name;
    try {
      const note = await body();
      if (this.errors.length > before) failure = `page errors:\n    ${this.errors.slice(before).join("\n    ")}`;
      if (!failure && note) label = `${name}: ${note}`;
    } catch (e) {
      failure = e instanceof Error ? e.message : String(e);
      if (this.errors.length > before) failure += `\n    page errors:\n    ${this.errors.slice(before).join("\n    ")}`;
    }
    const seconds = ((Date.now() - start) / 1000).toFixed(1);
    if (failure || this.shots) await this.page.screenshot({ path: shotPath(this.prefix, name) }).catch(() => {});
    if (failure) {
      const shot = path.relative(process.cwd(), shotPath(this.prefix, name));
      console.log(`FAIL ${label} (${seconds} s)\n    ${failure}\n    screenshot: ${shot}`);
    } else {
      console.log(`ok   ${label} (${seconds} s)`);
    }
    this.results.push({ name: label, failure });
    return !failure;
  }

  /** Print the summary; exit non-zero if any check failed. */
  report() {
    const failed = this.results.filter((r) => r.failure);
    console.log(`\n${this.results.length - failed.length} passed, ${failed.length} failed`);
    for (const f of failed) console.log(`  FAIL ${f.name}`);
    process.exitCode = failed.length > 0 ? 1 : 0;
  }
}

/** Throw `message` unless `condition`. */
export function expect(condition, message) {
  if (!condition) throw new Error(message);
}
