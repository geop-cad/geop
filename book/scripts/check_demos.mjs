#!/usr/bin/env node
// Opens the built book in headless Chrome and checks every page: no console
// errors or uncaught exceptions, every demo started and drew something, no
// sideways scrolling at phone width, in a light and a dark theme, with and
// without reduced motion, and every internal link and anchor resolves.
//
//   mdbook build book
//   NODE_PATH=web/node_modules node book/scripts/check_demos.mjs [--dist book/dist] [page-substring ...]
//
// Needs playwright-core (web/'s dev dependency) and a Chrome; CHROME_BIN overrides
// /usr/bin/google-chrome. Serves the directory itself over http on 127.0.0.1.
import { createServer } from "node:http";
import { readFileSync, existsSync, statSync, readdirSync } from "node:fs";
import { join, dirname, extname, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { createRequire } from "node:module";

const here = dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
let dist = join(here, "..", "dist");
const filters = [];
for (let i = 0; i < args.length; i++) {
  if (args[i] === "--dist") dist = resolve(args[++i]);
  else filters.push(args[i]);
}
const require = createRequire(join(process.env.NODE_PATH || join(here, "..", "..", "web", "node_modules"), "x.js"));
const { chromium } = require("playwright-core");

const types = { ".html": "text/html", ".js": "text/javascript", ".css": "text/css", ".json": "application/json", ".woff2": "font/woff2", ".svg": "image/svg+xml", ".png": "image/png" };
const server = createServer((req, res) => {
  const url = decodeURIComponent(req.url.split("?")[0]);
  let p = join(dist, url);
  if (existsSync(p) && statSync(p).isDirectory()) p = join(p, "index.html");
  if (!existsSync(p)) { res.writeHead(404); res.end(); return; }
  res.writeHead(200, { "content-type": types[extname(p)] || "application/octet-stream" });
  res.end(readFileSync(p));
});
await new Promise((r) => server.listen(0, "127.0.0.1", r));
const base = `http://127.0.0.1:${server.address().port}/`;

function pages(dir, prefix = "") {
  let out = [];
  for (const e of readdirSync(dir, { withFileTypes: true })) {
    if (e.isDirectory()) out = out.concat(pages(join(dir, e.name), prefix + e.name + "/"));
    else if (e.name.endsWith(".html") && !["print.html", "toc.html", "404.html"].includes(e.name)) out.push(prefix + e.name);
  }
  return out;
}
const all = pages(dist).filter((p) => !filters.length || filters.some((f) => p.includes(f)));

const browser = await chromium.launch({ executablePath: process.env.CHROME_BIN || "/usr/bin/google-chrome", args: ["--no-sandbox"] });
const configs = [
  { name: "phone-dark-reduced", viewport: { width: 360, height: 740 }, theme: "navy", reducedMotion: "reduce" },
  { name: "desktop-light", viewport: { width: 1280, height: 900 }, theme: "light", reducedMotion: "no-preference" },
];
let problems = 0;
const report = (page, cfg, msg) => { problems++; console.log(`${page} [${cfg}]: ${msg}`); };
const linkProblems = new Set();

for (const cfg of configs) {
  const context = await browser.newContext({ viewport: cfg.viewport, reducedMotion: cfg.reducedMotion });
  await context.addInitScript((t) => { try { localStorage.setItem("mdbook-theme", JSON.stringify(t)); } catch (e) {} }, cfg.theme);
  for (const pageName of all) {
    const page = await context.newPage();
    page.on("console", (m) => { if (m.type() === "error" || m.type() === "warning") report(pageName, cfg.name, `console ${m.type()}: ${m.text()}`); });
    page.on("pageerror", (e) => report(pageName, cfg.name, `exception: ${e.message}`));
    page.on("response", (r) => { if (r.status() >= 400 && !r.url().endsWith("favicon.ico")) report(pageName, cfg.name, `HTTP ${r.status()}: ${r.url()}`); });
    await page.goto(base + pageName, { waitUntil: "load" });
    // Walk down the page, so that each demo is started and exercised.
    const count = await page.evaluate(async () => {
      const figs = Array.from(document.querySelectorAll("figure.demo"));
      for (const f of figs) { f.scrollIntoView(); await new Promise((r) => setTimeout(r, 120)); }
      return figs.length;
    });
    if (count) {
      await page.waitForTimeout(300);
      const status = await page.evaluate(() => Array.from(document.querySelectorAll("figure.demo")).map((f) => {
        const canvas = f.querySelector("canvas");
        let painted = false;
        if (canvas && canvas.width) {
          const d = canvas.getContext("2d").getImageData(0, 0, canvas.width, canvas.height).data;
          for (let i = 3; i < d.length; i += 4) if (d[i]) { painted = true; break; }
        }
        const svg = f.querySelector("svg");
        return { name: f.dataset.demo, pending: !!f.querySelector(".gb-pending"), err: f.querySelector(".gb-error")?.textContent, painted: painted || !!(svg && svg.childElementCount) };
      }));
      for (const s of status) {
        if (s.pending) report(pageName, cfg.name, `demo ${s.name} never started`);
        if (s.err) report(pageName, cfg.name, `demo ${s.name}: ${s.err}`);
        if (!s.painted) report(pageName, cfg.name, `demo ${s.name} drew nothing`);
      }
      // Press every button and move every control once.
      await page.evaluate(async () => {
        for (const b of document.querySelectorAll("figure.demo button")) { b.click(); await new Promise((r) => setTimeout(r, 20)); }
        for (const b of document.querySelectorAll("figure.demo input[type=range]")) {
          b.value = (Number(b.min) + Number(b.max)) * 0.37; b.dispatchEvent(new Event("input", { bubbles: true }));
        }
        for (const s of document.querySelectorAll("figure.demo select")) {
          for (const o of s.options) { s.value = o.value; s.dispatchEvent(new Event("change", { bubbles: true })); await new Promise((r) => setTimeout(r, 30)); }
        }
        for (const c of document.querySelectorAll("figure.demo input[type=checkbox]")) { c.click(); await new Promise((r) => setTimeout(r, 20)); }
      });
      await page.waitForTimeout(150);
    }
    const overflow = await page.evaluate(() => document.documentElement.scrollWidth - document.documentElement.clientWidth);
    if (overflow > 1) report(pageName, cfg.name, `the page scrolls sideways by ${overflow}px`);
    if (cfg === configs[0]) {
      const links = await page.evaluate(() => Array.from(document.querySelectorAll("main a[href]")).map((a) => a.href));
      for (const l of links) {
        if (!l.startsWith(base)) continue;
        const [file, hash] = l.slice(base.length).split("#");
        const p = join(dist, file || "index.html");
        if (!existsSync(p)) { linkProblems.add(`${pageName}: broken link ${l.slice(base.length)}`); continue; }
        if (hash) {
          const html = readFileSync(p, "utf8");
          if (!html.includes(`id="${decodeURIComponent(hash)}"`)) linkProblems.add(`${pageName}: no anchor ${l.slice(base.length)}`);
        }
      }
    }
    await page.close();
  }
  await context.close();
}
for (const l of linkProblems) { problems++; console.log(l); }
await browser.close();
server.close();
console.log(`${all.length} pages, ${configs.length} configurations, ${problems} problems`);
process.exit(problems ? 1 : 0);
