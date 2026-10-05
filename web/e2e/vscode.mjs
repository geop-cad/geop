// End-to-end checks of the VS Code extension's editor: `npm run e2e:vscode`
// (see README.md).
//
// Builds the release CLI and the extension's page (`build:vscode`), writes
// example workspaces with `geop examples --out-dir`, and opens their
// programs through the bridge (`bridge.mjs`), which stands in for VS Code
// and drives a real `geop serve` as the extension does: single parts, an
// assembly of several files with standard parts, and a jointed one. An edit
// in the page must be written back to the document, an undo too, and an
// export must reach VS Code to be saved.
//
// Flags: `--build` rebuilds the page even if it looks current; `--only
// <text>` runs only the checks whose name contains the text; `--shots`
// saves a screenshot after every check.

import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { startBridge } from "./bridge.mjs";
import { Checks, fileMenu, ROOT, expect, launch, run, settle, shownErrors, stale, watchErrors } from "./lib.mjs";

const browser = await launch();
run("cargo", ["build", "--release", "-p", "geop-cad-cli"], ROOT);
// The bridge runs the kernel with the extension's own code (`bridge.mjs`).
run("npm", ["run", "compile"], path.join(ROOT, "vscode-extension"));
const media = path.join(ROOT, "vscode-extension", "media");
if (stale(path.join(media, "index.html"))) run("npm", ["run", "build:vscode"]);
const exe = path.join(ROOT, "target", "release", process.platform === "win32" ? "geop.exe" : "geop");

const folder = fs.mkdtempSync(path.join(os.tmpdir(), "geop-e2e-"));
const examples = ["pin", "hole_plate", "bolted_plate", "arm"];
run(exe, ["examples", "--out-dir", folder, "--quality", "4", ...examples.flatMap((e) => ["--only", e])]);
const bridge = await startBridge({ media, exe, folder });

const context = await browser.newContext({ viewport: { width: 1400, height: 900 } });
const page = await context.newPage();
const errors = watchErrors(page);
const checks = new Checks(page, errors, "vscode_");

const check = (name, body) => checks.check(name, body);

/** Open `doc` as VS Code would, and wait until it is built without an error; its stats. */
async function open(doc) {
  await page.goto(bridge.url(doc));
  // The document arrives after the page is up: wait for its steps.
  const steps = JSON.parse(fs.readFileSync(path.join(folder, doc), "utf8")).steps.length;
  const deadline = Date.now() + 120000;
  let stats = await settle(page);
  while (!stats.startsWith(`${steps} step`)) {
    expect(Date.now() < deadline, `${doc} did not load: ${stats}`);
    await page.waitForTimeout(200);
    stats = await settle(page);
  }
  const shown = await shownErrors(page);
  expect(shown.length === 0, `errors shown: ${shown.join(" | ")}`);
  const failed = await page.locator(".timeline .step-box.op-error").evaluateAll((els) => els.map((e) => e.title));
  expect(failed.length === 0, `steps failed: ${failed.join(" | ")}`);
  expect(!/ 0 tris/.test(stats), `nothing is drawn: ${stats}`);
  return stats;
}

/** The last program the page wrote back to `doc`, once it has written `count` of them. */
async function writtenBack(doc, count) {
  const deadline = Date.now() + 30000;
  while ((bridge.written[doc]?.length ?? 0) < count) {
    expect(Date.now() < deadline, `${doc}: ${bridge.written[doc]?.length ?? 0} programs written back, expected ${count}`);
    await page.waitForTimeout(100);
  }
  return bridge.written[doc].at(-1);
}

for (const doc of ["pin.geop", "hole_plate.geop"]) {
  await check(`opens ${doc}`, () => open(doc));
}

await check("an edit is written back, and its undo", async () => {
  const doc = "hole_plate.geop";
  await open(doc);
  const steps = JSON.parse(fs.readFileSync(path.join(folder, doc), "utf8")).steps;
  const before = bridge.written[doc]?.length ?? 0;
  // Delete the last step, as the timeline's ✕ does.
  await page.locator(".timeline .step-box").last().hover();
  await page.locator(".timeline .step-box").last().locator(".step-remove").click();
  await settle(page);
  const removed = await writtenBack(doc, before + 1);
  expect(removed.steps.length === steps.length - 1, `written back with ${removed.steps.length} steps, expected ${steps.length - 1}`);
  await page.locator("button[aria-label=Undo]").click();
  await settle(page);
  const restored = await writtenBack(doc, before + 2);
  expect(
    JSON.stringify(restored.steps) === JSON.stringify(steps),
    "undo did not write the program back as it was",
  );
});

await check("exports reach VS Code to be saved", async () => {
  const doc = "pin.geop";
  await open(doc);
  /** The file the File menu's `entry` sends to be saved. */
  const exported = async (entry) => {
    const before = bridge.saved[doc]?.length ?? 0;
    await fileMenu(page, entry);
    await settle(page);
    const deadline = Date.now() + 30000;
    while ((bridge.saved[doc]?.length ?? 0) <= before) {
      expect(Date.now() < deadline, `${entry}: no file was sent to be saved`);
      await page.waitForTimeout(100);
    }
    return bridge.saved[doc].at(-1);
  };
  const step = await exported("Download STEP");
  expect(step.name === "pin.step", `saved as ${step.name}`);
  expect(step.text?.startsWith("ISO-10303-21;"), "the STEP file does not start with its header");
  // A binary file comes as base64.
  const stl = await exported("Download STL");
  expect(stl.name === "pin.stl", `saved as ${stl.name}`);
  const bytes = Buffer.from(stl.bytes ?? "", "base64");
  expect(bytes.length > 84 && bytes.readUInt32LE(80) * 50 + 84 === bytes.length, `pin.stl is no binary STL (${bytes.length} bytes)`);
  return `${step.name} ${step.text.length} bytes, ${stl.name} ${bytes.length} bytes`;
});

await check("a STEP file chosen from disk in Import STEP is stored next to the document and imported", async () => {
  const doc = "hole_plate.geop";
  await open(doc);
  // A STEP file elsewhere on disk: the pin, written by geop's exporter.
  const elsewhere = fs.mkdtempSync(path.join(os.tmpdir(), "geop-e2e-step-"));
  const step = path.join(elsewhere, "pin part.step");
  run(exe, ["compile", path.join(folder, "pin.geop"), "--output", step]);
  const before = bridge.written[doc]?.length ?? 0;
  await page.locator(".desktop-only .operation-tools button.op-button", { hasText: /^Import STEP$/ }).click();
  await settle(page);
  const popup = page.locator(".desktop-only .popup");
  const [chooser] = await Promise.all([
    page.waitForEvent("filechooser", { timeout: 10000 }),
    popup.getByRole("button", { name: /Choose a file/ }).click(),
  ]);
  await chooser.setFiles(step);
  await settle(page);
  expect(fs.existsSync(path.join(folder, "pin part.step")), "the STEP file was not stored next to the document");
  expect((await popup.innerText()).includes("pin part.step"), `the dialog does not show the file:\n${await popup.innerText()}`);
  await popup.locator(".button-row button.primary").click();
  await settle(page);
  const deadline = Date.now() + 30000;
  let imported = null;
  while (imported?.operation !== "import_step" || imported.args.file !== "pin part.step") {
    expect(Date.now() < deadline, `the import was not written back: ${JSON.stringify(imported)}`);
    await page.waitForTimeout(100);
    imported = (bridge.written[doc]?.slice(before).at(-1)?.steps ?? []).at(-1) ?? null;
  }
  const shown = await shownErrors(page);
  expect(shown.length === 0, `errors shown: ${shown.join(" | ")}`);
  const structure = await page.locator(".structure-panel").innerText();
  expect(structure.includes(`import(${imported.id},s0)`), `no imported solid:\n${structure}`);
  fs.rmSync(elsewhere, { recursive: true, force: true });
  return `${imported.id}: ${imported.args.file}`;
});

await check("an assembly with standard parts opens", async () => {
  const stats = await open("bolted_plate/bolted_plate.geop");
  // The plate from its own file, the screw and the nut from the standard library (`std:`).
  const placed = await page.locator(".structure-panel .structure-name").allInnerTexts();
  for (const part of ["plate", "screw", "nut"]) {
    expect(placed.includes(part), `${part} is not placed: ${placed.join(", ")}`);
  }
  return stats;
});

await check("a jointed assembly opens, and moving a joint is written back", async () => {
  const doc = "arm/arm.geop";
  const stats = await open(doc);
  // The joints panel follows its heading; each coordinate is typed in.
  const joints = page.locator("h2:text-is('Joints') + .parameters input[type=number]");
  expect((await joints.count()) > 0, "no joint can be moved");
  const before = bridge.written[doc]?.length ?? 0;
  const joint = joints.first();
  const value = Number(await joint.inputValue()) + 10;
  await joint.fill(String(value));
  await joint.press("Enter");
  await settle(page);
  const moved = await writtenBack(doc, before + 1);
  const state = Object.values(moved.state ?? {}).map((v) => JSON.stringify(v));
  expect(state.some((v) => v.includes(String(value))), `no joint is at ${value} in the program written back: ${state.join(", ")}`);
  return stats;
});

await check("a kernel that crashes is restarted with the document", async () => {
  const doc = "bolted_plate/bolted_plate.geop";
  const before = await open(doc);
  const text = fs.readFileSync(path.join(folder, doc), "utf8");
  const logged = errors.length;
  // As a bug would: `geop serve` panics, says so, and ends.
  await page.evaluate(() => window.geopCommand({ command: "crash" }));
  const after = await settle(page);
  const shown = (await shownErrors(page)).join(" | ");
  expect(/kernel crashed/.test(shown) && /asked to crash/.test(shown), `the crash was not shown: ${shown || "nothing shown"}`);
  expect(after === before, `the document came back as ${after}, not ${before}`);
  expect(fs.readFileSync(path.join(folder, doc), "utf8") === text, "the document was changed");
  errors.push(...errors.splice(logged).filter((e) => !/crashed/.test(e)));
  // The standard parts are placed again — the files came back too — and it still edits.
  const placed = await page.locator(".structure-panel .structure-name").allInnerTexts();
  expect(placed.includes("screw"), `the standard parts are gone: ${placed.join(", ")}`);
  await page.locator(".desktop-only .operation-tools button.op-button", { hasText: /^Sketch$/ }).click();
  await settle(page);
  expect((await page.locator(".desktop-only .popup").count()) === 1, "Sketch did not open after the crash");
  return after;
});

checks.report();
await browser.close();
bridge.stop();
fs.rmSync(folder, { recursive: true, force: true });
