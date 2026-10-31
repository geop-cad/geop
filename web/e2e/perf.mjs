// The latency budget of the web app: `npm run e2e:perf` (see docs/ux.md).
//
// Measures, in the page, from the user's act to the first frame drawn
// with its answer: a hover, a click in the view, a number typed into a
// dialog (Enter, the preview built), a step opened from the program and
// committed — on a medium part and on a large assembly (a pin placed two
// thousand times by a part pattern). Each is taken several times; the
// median and the slowest are printed, and a check fails where a median is
// over its budget. What the kernel spends on the 2000-part robot arm is
// measured in Rust (`geop-ops-assembly`'s `scale_tests`,
// `latency_on_the_robot_arm`): the robot is built of test-only steps no
// app offers.

import path from "node:path";
import { Checks, expect, fileMenu, launch, preview, run, settle, stale, watchErrors, WEB } from "./lib.mjs";

const browser = await launch();
if (stale(path.join(WEB, "dist", "index.html"))) run("npm", ["run", "build"]);
const server = await preview();
const context = await browser.newContext({ viewport: { width: 1400, height: 900 } });
const page = await context.newPage();
const errors = watchErrors(page);
const checks = new Checks(page, errors);
const check = (name, body) => checks.check(name, body);

/** How many times each act is timed. */
const SAMPLES = 9;

/** The median and the slowest of `times`, in milliseconds. */
function summary(times) {
  const sorted = [...times].sort((a, b) => a - b);
  return { median: sorted[Math.floor(sorted.length / 2)], worst: sorted[sorted.length - 1] };
}

/**
 * In the page: do `act` (`hover`, `click`, `type`, `open`, `commit`) and
 * wait until the kernel's answer to it was shown and drawn — an update
 * more than before, and the frame after — the milliseconds that took.
 */
function timeInPage(page, act, args) {
  return page.evaluate(
    async ({ act, args }) => {
      const frame = () => new Promise((r) => requestAnimationFrame(() => r()));
      const until = async (done) => {
        const start = performance.now();
        while (!done()) {
          if (performance.now() - start > 120000) throw new Error(`no answer to ${act}`);
          await frame();
        }
      };
      const updates = () => window.geopUpdates ?? 0;
      const canvas = document.querySelector("main.viewport canvas");
      const pointer = (type, x, y, buttons) =>
        canvas.dispatchEvent(
          new PointerEvent(type, { clientX: x, clientY: y, bubbles: true, pointerId: 1, pointerType: "mouse", isPrimary: true, button: 0, buttons }),
        );
      const before = updates();
      const t0 = performance.now();
      if (act === "hover") pointer("pointermove", args.x, args.y, 0);
      if (act === "click") {
        pointer("pointerdown", args.x, args.y, 1);
        pointer("pointerup", args.x, args.y, 0);
      }
      if (act === "type") {
        const input = document.querySelector(`.popup .field[data-field-key='${args.key}'] input`);
        const set = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value").set;
        set.call(input, String(args.value));
        input.dispatchEvent(new KeyboardEvent("keydown", { key: "Enter", bubbles: true }));
      }
      if (act === "open") [...document.querySelectorAll(".timeline .step-box")][args.index].click();
      if (act === "commit") document.querySelector(".popup .button-row button.primary").click();
      await until(() => updates() > before);
      if (act === "open") await until(() => document.querySelector(".popup") != null);
      if (act === "commit") await until(() => document.querySelector(".popup") == null);
      await frame();
      return performance.now() - t0;
    },
    { act, args },
  );
}

/** `act` timed [[SAMPLES]] times, `args(k)` its arguments the `k`th time, `between(k)` done untimed after each. */
async function timed(act, args, between = async () => {}) {
  const times = [];
  for (let k = 0; k < SAMPLES; k++) {
    times.push(await timeInPage(page, act, args(k)));
    await between(k);
  }
  return summary(times);
}

/** Where the view's middle is, and a point `dx` pixels right of it, in client coordinates. */
async function middle(dx = 0) {
  const box = await page.locator("main.viewport canvas").first().boundingBox();
  return { x: box.x + box.width / 2 + dx, y: box.y + box.height / 2 };
}

/** What is measured of each part, by act. */
const measured = {};

/**
 * The acts on the part loaded: a hover (alternating between two points,
 * so each is new), a click, a number typed into the step at `step` (its
 * field `key`, the values alternating), and that step opened and committed.
 */
async function measure(name, { step, key, values, click }) {
  const results = {};
  const a = await middle(-30);
  const b = await middle(30);
  results.hover = await timed("hover", (k) => (k % 2 ? a : b));
  if (click) results.click = await timed("click", (k) => (k % 2 ? a : b));
  results.open = await timed("open", () => ({ index: step }), async () => {
    await page.locator(".popup .button-row button", { hasText: /^Cancel$/ }).click();
    await settle(page);
  });
  await timeInPage(page, "open", { index: step });
  await settle(page);
  results.type = await timed("type", (k) => ({ key, value: values[k % 2] }));
  results.commit = await timed("commit", () => ({}), async () => {
    await timeInPage(page, "open", { index: step });
    await settle(page);
  });
  measured[name] = results;
  return Object.entries(results)
    .map(([act, { median, worst }]) => `${act} ${median.toFixed(0)} ms (worst ${worst.toFixed(0)})`)
    .join(", ");
}

/** The index of the program's step `id`. */
async function stepIndex(id) {
  const names = await page.locator(".timeline .step-name").allInnerTexts();
  const index = names.indexOf(id);
  expect(index >= 0, `no step ${id}: ${names.join(", ")}`);
  return index;
}

/** The app as a new visitor has it, all operations offered. */
async function fresh() {
  await page.goto(server.url);
  await page.evaluate(() => {
    localStorage.clear();
    localStorage.setItem("geop.reducedMode", "off");
  });
  await page.reload();
  await settle(page);
}

await check("latency on a medium part (the patterned plate)", async () => {
  await fresh();
  await fileMenu(page, "Patterned plate");
  await settle(page);
  await page.waitForTimeout(800);
  // Its first extrusion: a distance to type.
  const names = await page.locator(".timeline .step-name").allInnerTexts();
  let step = -1;
  for (let i = 0; i < names.length && step < 0; i++) {
    await timeInPage(page, "open", { index: i });
    await settle(page);
    if ((await page.locator(".popup .field[data-field-key='distance'] input").count()) > 0) step = i;
    await page.locator(".popup .button-row button", { hasText: /^Cancel$/ }).click();
    await settle(page);
  }
  expect(step >= 0, "no step of the plate has a distance");
  const said = await measure("medium part", { step, key: "distance", values: [2.5, 3], click: false });
  return said;
});

await check("latency on a large assembly (a pin placed 2000 times)", async () => {
  await fresh();
  await page.evaluate(() => window.geopCommand({ command: "load_workspace_example", name: "pin_in_plate" }));
  await settle(page);
  // A part pattern of the pin, along the origin's x axis.
  const program = await page.evaluate(async () => (await window.geopCommand({ command: "show" })).program.program);
  program.steps.push({
    id: "pins",
    operation: "part_pattern",
    args: { part: "pin", axis: { datum: "origin:x" }, reversed: false, layout: { type: "linear", spacing: { step: 3 } }, count: 2000 },
  });
  const loaded = await page.evaluate((program) => window.geopCommand({ command: "load", program }), program);
  await settle(page, 600000);
  const failed = (loaded?.program?.steps ?? []).filter((s) => s.error).map((s) => `${s.id}: ${s.error}`);
  expect(failed.length === 0, `the pattern fails: ${failed.join(" | ").slice(0, 400)}`);
  await page.evaluate(() => window.geopCommand({ command: "tool", key: "drag" }));
  await settle(page);
  const step = await stepIndex("pins");
  return measure("large assembly", { step, key: "count", values: [1999, 2000], click: true });
});

console.log(JSON.stringify(measured, null, 2));
checks.report();
await browser.close();
server.stop();
