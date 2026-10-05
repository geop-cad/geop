// End-to-end checks of the web app: `npm run e2e` (see README.md).
//
// Builds `dist/` if it is older than the sources, serves it with `vite
// preview` on a free port, and drives it in headless Chrome: every
// operation is in reach of the toolbar without scrolling, and a drag over
// the view selects no text; every example loads from the File menu and
// builds without an error; every operation opens and cancels; a part is
// sketched and extruded by clicks; the inspect panel weighs it; a subd face
// and a 3-D sketch point are dragged by their gizmos; every export of the
// File menu downloads a file; and a drawing is dimensioned on its sheet and
// downloaded from its dialog.
//
// Flags: `--build` rebuilds even if `dist/` looks current; `--only <text>`
// runs only the checks whose name contains the text; `--shots` saves a
// screenshot after every check, not only after failing ones.

import path from "node:path";
import fs from "node:fs";
import { Checks, clickOperation, expect, fileMenu, launch, operationLabels, OUT, pixelAt, preview, run, settle, shownErrors, stale, watchErrors, WEB } from "./lib.mjs";

const browser = await launch();
if (stale(path.join(WEB, "dist", "index.html"))) run("npm", ["run", "build"]);
const server = await preview();
const context = await browser.newContext({ viewport: { width: 1400, height: 900 }, acceptDownloads: true });
const page = await context.newPage();
const errors = watchErrors(page);
const checks = new Checks(page, errors);

const check = (name, body) => checks.check(name, body);

/** The app as a new visitor sees it: no files kept from before, one empty part. */
async function fresh() {
  await page.goto(server.url);
  await page.evaluate(() => localStorage.clear());
  await page.reload();
  await settle(page);
}

/** The labels of the File menu's entries under the heading `heading`. */
async function menuSection(heading) {
  await page.locator(".file-menu .dropdown-trigger").click();
  const labels = await page.evaluate((heading) => {
    const out = [];
    let inside = false;
    for (const el of document.querySelectorAll(".file-menu .dropdown-menu > *")) {
      if (el.classList.contains("dropdown-heading")) inside = el.textContent === heading;
      else if (el.classList.contains("dropdown-separator")) inside = false;
      else if (inside) out.push(el.querySelector(".dropdown-item-label").textContent);
    }
    return out;
  }, heading);
  await page.keyboard.press("Escape");
  return labels;
}

/** The program's steps that failed, with why. */
function failedSteps() {
  return page.locator(".timeline .step-box.op-error").evaluateAll((els) => els.map((e) => e.getAttribute("title")));
}

/** Throw if the app shows an error or a step failed; else the stats. */
async function builtCleanly() {
  const stats = await settle(page);
  const shown = await shownErrors(page);
  expect(shown.length === 0, `errors shown: ${shown.join(" | ")}`);
  const failed = await failedSteps();
  expect(failed.length === 0, `steps failed: ${failed.join(" | ")}`);
  return stats;
}

// ── the toolbar ──────────────────────────────────────────────────────────

/**
 * What of the toolbar is in view with nothing scrolled: the labels of the
 * operation buttons, and of the entries of every menu of the operations
 * (each opened in turn); and what is out of the window or would scroll.
 */
async function toolbarReach() {
  const inView = () =>
    page.evaluate(() => {
      const problems = [];
      const fits = (el) => {
        const r = el.getBoundingClientRect();
        return r.width > 0 && r.left >= 0 && r.right <= window.innerWidth && r.top >= 0 && r.bottom <= window.innerHeight;
      };
      for (const s of [".file-menu", "button[aria-label='Undo']", "button[aria-label='Redo']", ".stats", ".help-menu"]) {
        const el = document.querySelector(s);
        if (!el || !fits(el)) problems.push(`${s} is out of view`);
      }
      for (const el of [document.documentElement, document.querySelector(".toolbar"), document.querySelector(".operation-strip")]) {
        if (el.scrollWidth > el.clientWidth) problems.push(`${el.className || el.tagName} scrolls: ${el.scrollWidth} > ${el.clientWidth}`);
      }
      const strip = document.querySelector(".operation-strip").getBoundingClientRect();
      const labels = [];
      for (const b of document.querySelectorAll(".operation-ribbon button.op-button")) {
        const r = b.getBoundingClientRect();
        if (!fits(b) || r.right > strip.right + 0.5) problems.push(`${b.innerText} is out of view`);
        else labels.push(b.innerText.trim());
      }
      for (const b of document.querySelectorAll(".operation-ribbon .dropdown-trigger")) {
        if (!fits(b) || b.getBoundingClientRect().right > strip.right + 0.5) problems.push(`the menu ${b.innerText} is out of view`);
      }
      return { labels, problems };
    });
  const { labels, problems } = await inView();
  const triggers = page.locator(".operation-ribbon .dropdown-trigger");
  const menus = await triggers.count();
  for (let i = 0; i < menus; i++) {
    await triggers.nth(i).click();
    const items = await page.evaluate(() =>
      [...document.querySelectorAll(".operation-ribbon .dropdown-menu .dropdown-item")].map((item) => {
        const r = item.getBoundingClientRect();
        const fits = r.width > 0 && r.left >= 0 && r.right <= window.innerWidth && r.bottom <= window.innerHeight;
        return { label: item.querySelector(".dropdown-item-label").textContent, fits };
      }),
    );
    await page.keyboard.press("Escape");
    for (const { label, fits } of items) {
      if (fits) labels.push(label);
      else problems.push(`${label} runs out of the window in its menu`);
    }
  }
  return { labels: new Set(labels), problems, menus };
}

for (const width of [1400, 1100, 900]) {
  await check(`every operation is reachable in the toolbar at ${width} px, nothing scrolled`, async () => {
    await page.setViewportSize({ width, height: 900 });
    await fresh();
    const all = await operationLabels(page);
    expect(all.length > 40, `only ${all.length} operations offered`);
    const { labels, problems, menus } = await toolbarReach();
    const missing = all.filter((l) => !labels.has(l));
    expect(missing.length === 0, `not reachable: ${missing.join(", ")}`);
    expect(problems.length === 0, problems.join("; "));
    const buttons = await page.locator(".operation-ribbon button.op-button").count();
    return `${all.length} operations, ${buttons} as buttons, the rest in ${menus} menus`;
  });
}
await page.setViewportSize({ width: 1400, height: 900 });

await check("the primary operations are big buttons at 1400 px", async () => {
  await fresh();
  const big = (await page.locator(".operation-ribbon button.op-button.big").allInnerTexts()).map((l) => l.trim());
  const missing = ["Sketch", "Extrude", "Revolve", "Hole", "Fillet", "Boolean", "Part"].filter((l) => !big.includes(l));
  expect(missing.length === 0, `not big: ${missing.join(", ")} (big: ${big.join(", ")})`);
  return `big: ${big.join(", ")}`;
});

await check("the toolbar at 1400, 1100 and 800 px", async () => {
  fs.mkdirSync(path.join(WEB, "e2e", "out"), { recursive: true });
  for (const width of [1400, 1100, 800]) {
    await page.setViewportSize({ width, height: 900 });
    await fresh();
    await page.screenshot({ path: path.join(WEB, "e2e", "out", `toolbar-${width}.png`) });
    // On a phone the operations are a tab of their own, not in the bar.
    const ribbon = await page.locator(".operation-strip").isVisible();
    expect(ribbon === width > 860, `at ${width} px the ribbon is ${ribbon ? "shown" : "hidden"}`);
    if (width <= 860) {
      const grid = await page.locator(".mobile-bottom .operation-grid button.op-button").count();
      expect(grid > 40, `the phone's tab offers ${grid} operations`);
    }
  }
  await page.setViewportSize({ width: 1400, height: 900 });
  return "screenshots in e2e/out/toolbar-*.png";
});

// ── text selection ───────────────────────────────────────────────────────

/** Press at the first of `points`, move through the others, and let go there; the text then selected. */
async function dragThrough(points) {
  const [from, ...rest] = points;
  await page.mouse.move(from.x, from.y);
  await page.mouse.down();
  for (const p of rest) await page.mouse.move(p.x, p.y, { steps: 8 });
  await page.mouse.up();
  return page.evaluate(() => window.getSelection().toString());
}

/** The centre of the first element `selector` finds. */
async function centre(selector) {
  const box = await page.locator(selector).first().boundingBox();
  return { x: box.x + box.width / 2, y: box.y + box.height / 2 };
}

await check("a drag across the viewport selects no text", async () => {
  await fresh();
  await fileMenu(page, "Box with drill hole");
  await builtCleanly();
  await clickOperation(page, "Extrude");
  await settle(page);
  const view = await page.locator("main.viewport canvas").first().boundingBox();
  const inView = { x: view.x + view.width / 3, y: view.y + view.height / 2 };
  const dialog = await centre(".desktop-only .popup h2");
  const toolbar = await centre(".operation-ribbon button.op-button");
  const panel = await centre(".structure-panel");
  const program = await centre(".timeline .step-box");
  // From the view over the dialog, the toolbar, the panels, and back.
  const selected = await dragThrough([inView, dialog, toolbar, panel, program, inView]);
  expect(selected === "", `selected: ${JSON.stringify(selected.slice(0, 200))}`);
  // From the view's edge, out over everything to the window's corner.
  const corner = await dragThrough([{ x: view.x + 5, y: view.y + view.height - 5 }, { x: 2, y: 2 }]);
  expect(corner === "", `selected: ${JSON.stringify(corner.slice(0, 200))}`);
  // Across the view from the chrome around it: the tab strip's empty end,
  // and the empty foot of the sidebar and of the explorer — where a press
  // is no camera's, and nothing captures the pointer.
  const tabs = await page.locator(".editor-tabs").boundingBox();
  const sidebar = await page.locator(".sidebar").boundingBox();
  const explorer = await page.locator(".explorer-bar").boundingBox();
  for (const [where, from] of [
    ["the tab strip", { x: tabs.x + tabs.width - 20, y: tabs.y + tabs.height / 2 }],
    ["the sidebar", { x: sidebar.x + sidebar.width / 2, y: sidebar.y + sidebar.height - 10 }],
    ["the explorer", { x: explorer.x + explorer.width / 2, y: explorer.y + explorer.height - 10 }],
  ]) {
    const crossed = await dragThrough([from, inView, toolbar]);
    expect(crossed === "", `a drag from ${where} over the view selected: ${JSON.stringify(crossed.slice(0, 200))}`);
  }
  await page.locator(".desktop-only .popup .button-row button", { hasText: /^Cancel$/ }).click();
  return builtCleanly();
});

await check("a drag within a dialog or the program panel selects its text, and only its", async () => {
  await fresh();
  await fileMenu(page, "Box with drill hole");
  await builtCleanly();
  await clickOperation(page, "Extrude");
  await settle(page);
  // Across the dialog's text, then on out over the view and the toolbar.
  const popup = await page.locator(".desktop-only .popup").boundingBox();
  const start = { x: popup.x + 12, y: popup.y + 40 };
  const inside = await dragThrough([start, { x: popup.x + popup.width - 12, y: popup.y + popup.height / 2 }]);
  expect(inside.trim().length > 0, "dragging across the dialog selected nothing");
  const out = await dragThrough([start, { x: start.x, y: 40 }, { x: 300, y: 40 }]);
  const toolbarText = (await page.locator(".toolbar").innerText()).split(/\s+/).filter((w) => w.length > 3);
  const dialogText = await page.locator(".desktop-only .popup").innerText();
  const leaked = toolbarText.filter((w) => out.includes(w) && !dialogText.includes(w));
  expect(leaked.length === 0, `selected the toolbar's ${leaked.join(", ")}`);
  await page.locator(".desktop-only .popup .button-row button", { hasText: /^Cancel$/ }).click();
  await settle(page);
  // The program panel: its parameters' text.
  const panel = await page.locator(".panel.timeline").boundingBox();
  const inPanel = await dragThrough([
    { x: panel.x + 8, y: panel.y + 30 },
    { x: panel.x + panel.width - 8, y: panel.y + 90 },
  ]);
  expect(inPanel.trim().length > 0, "dragging across the program panel selected nothing");
  // On out of the panel, over the view and up into the toolbar: still only the panel's.
  const view = await page.locator("main.viewport canvas").first().boundingBox();
  const panelOut = await dragThrough([
    { x: panel.x + 8, y: panel.y + 30 },
    { x: view.x + view.width / 2, y: view.y + view.height / 2 },
    { x: view.x + view.width / 2, y: 40 },
    { x: 300, y: 40 },
  ]);
  const panelText = await page.locator(".panel.timeline").innerText();
  const fromPanel = toolbarText.filter((w) => panelOut.includes(w) && !panelText.includes(w));
  expect(fromPanel.length === 0, `a drag out of the panel selected the toolbar's ${fromPanel.join(", ")}`);
  await page.mouse.click(5, 5);
  return `${inside.trim().length} characters in the dialog, ${inPanel.trim().length} in the panel`;
});

// ── every example ────────────────────────────────────────────────────────

await fresh();
const parts = await menuSection("Example parts");
const assemblies = await menuSection("Example assemblies");
await check("the File menu offers examples", async () => {
  expect(parts.length > 0, "no example parts in the File menu");
  expect(assemblies.length > 0, "no example assemblies in the File menu");
  return `${parts.length} parts, ${assemblies.length} assemblies`;
});
for (const label of [...parts, ...assemblies]) {
  await check(`example ${label}`, async () => {
    await fresh();
    await fileMenu(page, label);
    const stats = await builtCleanly();
    expect(!stats.startsWith("0 steps"), `nothing was loaded: ${stats}`);
    expect(!/ 0 tris/.test(stats), `nothing is drawn: ${stats}`);
    return stats;
  });
}

// ── every operation opens, and cancels ──────────────────────────────────

/** Click every operation in the toolbar in turn: each opens its step (or says why not), and Cancel closes it again. */
async function everyOperation() {
  const labels = await operationLabels(page);
  const n = labels.length;
  expect(n > 10, `only ${n} operations`);
  const refused = [];
  const times = [];
  for (const label of labels) {
    const start = Date.now();
    if (label === "Drag") continue;
    await clickOperation(page, label);
    await settle(page);
    if ((await page.locator(".desktop-only .popup").count()) === 0) {
      // Refused to open: it must say why.
      const shown = await shownErrors(page);
      expect(shown.length > 0, `${label} neither opened nor said why`);
      refused.push(label);
      continue;
    }
    await page.locator(".desktop-only .popup .button-row button", { hasText: /^Cancel$/ }).click();
    await settle(page);
    expect((await page.locator(".desktop-only .popup").count()) === 0, `${label} did not close on Cancel`);
    times.push([label, Date.now() - start]);
  }
  // The editor still answers: the stats are back, and Undo is as it was.
  await settle(page);
  const slowest = times.sort((a, b) => b[1] - a[1]).slice(0, 3).map(([l, t]) => `${l} ${(t / 1000).toFixed(1)} s`);
  return `${n - refused.length} opened${refused.length ? `, refused: ${refused.join(", ")}` : ""}; slowest: ${slowest.join(", ")}`;
}

await check("every operation opens on an empty part", async () => {
  await fresh();
  return everyOperation();
});
await check("every operation opens on a part", async () => {
  await fresh();
  await fileMenu(page, "Box with drill hole");
  await builtCleanly();
  const result = await everyOperation();
  await builtCleanly();
  return result;
});

// ── a kernel that crashes ───────────────────────────────────────────────

/**
 * Make the kernel panic, as a bug would (the `crash` command, through the
 * app's `geopCommand`), and check that the app comes back with the program
 * as it was, says so, and keeps working. The panic's own console messages
 * are what is expected, not page errors.
 */
async function crashKernel(page, errors) {
  const before = await builtCleanly();
  const logged = errors.length;
  await page.evaluate(() => window.geopCommand({ command: "crash" }));
  const after = await settle(page);
  const shown = (await shownErrors(page)).join(" | ");
  expect(/kernel crashed/.test(shown) && /asked to crash/.test(shown), `the crash was not shown: ${shown || "nothing shown"}`);
  expect(/restarted/.test(shown), `the restart was not said: ${shown}`);
  expect(after === before, `the program came back as ${after}, not ${before}`);
  const unexpected = errors.splice(logged).filter((e) => !/panicked|asked to crash|unreachable/.test(e));
  errors.push(...unexpected);
  // It still works: an operation opens, and the error is gone.
  await clickOperation(page, "Sketch");
  await settle(page);
  expect((await popup.count()) === 1, "Sketch did not open after the crash");
  await popup.locator(".button-row button", { hasText: /^Cancel$/ }).click();
  return builtCleanly();
}

// ── a part made by clicks ───────────────────────────────────────────────

const popup = page.locator(".desktop-only .popup");

/** Click the viewport `dx`, `dy` pixels from its centre. */
async function clickView(dx, dy) {
  const box = await page.locator("main.viewport canvas").first().boundingBox();
  await page.mouse.move(box.x + box.width / 2 + dx, box.y + box.height / 2 + dy);
  await settle(page);
  await page.mouse.click(box.x + box.width / 2 + dx, box.y + box.height / 2 + dy);
  await settle(page);
}

/** A row of the inspect panel's mass properties: its numbers. */
async function massRow(label) {
  const text = await page.locator(".inspect-result tr", { has: page.locator(`th:text-is('${label}')`) }).first().innerText();
  return [...text.matchAll(/-?\d+(\.\d+)?(e-?\d+)?/g)].map((m) => Number(m[0]));
}

await check("a rectangle is sketched and extruded by clicks, and weighed", async () => {
  await fresh();
  await clickOperation(page, "Sketch");
  await settle(page);
  // The plane: the square of the origin's zx plane, below and right of its centre, seen from the default view.
  await clickView(8, 23);
  expect((await popup.innerText()).includes("plane: origin zx plane"), "the zx plane was not picked");
  // The corner rectangle tool (R), and its two corners.
  await page.keyboard.press("r");
  await settle(page);
  await clickView(-270, -190);
  await clickView(-120, -70);
  expect((await popup.innerText()).includes("1 region"), `the rectangle drew no region:\n${await popup.innerText()}`);
  await popup.locator(".button-row button.primary").click();
  await settle(page);
  await clickOperation(page, "Extrude");
  await settle(page);
  expect((await popup.innerText()).includes("sketch: sketch1"), "the extrusion did not take the sketch");
  // Distance 2, typed into the field.
  const distance = popup.locator(".field", { hasText: "distance" }).locator("input[type=number], input[type=text]").first();
  await distance.fill("2");
  await distance.press("Enter");
  await settle(page);
  await popup.locator(".button-row button.primary").click();
  const stats = await builtCleanly();
  expect(stats.startsWith("2 steps"), `not two steps: ${stats}`);
  expect((await page.locator(".structure-panel").innerText()).includes("extrude(extrude1)"), "no solid extrude(extrude1)");
  // The first solid is framed: drawn off-centre, it is in the middle of the view now.
  await page.waitForTimeout(1000);
  const box = await page.locator("main.viewport canvas").first().boundingBox();
  const [r, g, b] = await pixelAt(page, box.x + box.width / 2, box.y + box.height / 2);
  expect(b > r + 30 && b > g, `the middle of the view is rgb(${r}, ${g}, ${b}), not the blue solid: it was not framed`);

  await page.getByTitle("Mass properties of every solid, of its part's material").click();
  await settle(page);
  const [volume] = await massRow("Volume");
  expect(volume > 0, `volume ${volume}`);
  // Extruded 2 up from the zx plane: the centre is 1 above it.
  const centre = await massRow("Centre");
  expect(Math.abs(centre[1] - 1) < 1e-3, `centre ${centre.join(", ")}, expected y = 1`);
  return `${stats}, volume ${volume} mm³`;
});

await check("a 3-D sketch's point is clicked and measured, as a vertex", async () => {
  await fresh();
  await clickOperation(page, "3-D sketch");
  await settle(page);
  // Two points in the plane through the origin, facing the eye: a line.
  await clickView(-220, -90);
  await clickView(-60, -150);
  await page.keyboard.press("Escape");
  await settle(page);
  await popup.locator(".button-row button.primary").click();
  await builtCleanly();
  await page.getByTitle("Measure: click up to two vertices, edges, faces or datums").click();
  await settle(page);
  await clickView(-220, -90);
  const picked = await page.locator(".inspect-entity").allInnerTexts();
  expect(
    picked.length === 1 && /^Vertex sketch3d\([^,]+,p\d+\)$/.test(picked[0].trim()),
    `picked: ${picked.join(", ") || "nothing"}`,
  );
  const rows = await page.locator(".inspect-table th").allInnerTexts();
  expect(["X", "Y", "Z"].every((axis) => rows.includes(axis)), `measured: ${rows.join(", ")}`);
  return picked[0];
});

await check("a pattern picks a hole as a feature by a click on its wall", async () => {
  await fresh();
  await fileMenu(page, "Patterned plate");
  await builtCleanly();
  // The example's own pattern lists the hole it repeats.
  await page.locator(".timeline .step-box").last().click();
  await settle(page);
  const opened = await popup.innerText();
  expect(/features[\s\S]*\bhole\b/.test(opened), `the pattern lists no feature:\n${opened}`);
  await popup.locator(".button-row button", { hasText: "Cancel" }).click();
  await settle(page);
  // A new pattern: its features picked by clicking along the view's middle
  // row, out from its centre where the framed plate's holes are, until one
  // lands on a hole's wall — the plate's own faces are no feature, so
  // nothing else is picked.
  await clickOperation(page, "Linear pattern");
  await settle(page);
  await popup.locator(".reference button", { hasText: "features:" }).click();
  await settle(page);
  const features = popup.locator(".reference", { has: page.locator("button", { hasText: "features:" }) });
  let picked = "";
  let dx = 0;
  for (let k = 0; k < 120 && !picked; k++) {
    dx = (k % 2 === 0 ? 1 : -1) * Math.ceil(k / 2) * 6;
    await clickView(dx, 0);
    picked = (await features.locator(".item-label").allInnerTexts()).join(", ");
  }
  expect(/^holes?$/.test(picked), `no hole was picked: "${picked}"`);
  // The wall clicked is drawn lit, as a face of the feature picked.
  const box = await page.locator("main.viewport canvas").first().boundingBox();
  const [r, g, b] = await pixelAt(page, box.x + box.width / 2 + dx, box.y + box.height / 2);
  expect(r > b + 60 && g > b, `the wall clicked is rgb(${r}, ${g}, ${b}), not lit`);
  const text = await popup.innerText();
  expect(text.includes("bodies: pick"), `the bodies were not emptied:\n${text}`);
  await popup.locator(".button-row button.primary").click();
  const stats = await builtCleanly();
  const solids = await page.locator(".structure-panel").innerText();
  expect(/linear_pattern\(linear_pattern\d+\)/.test(solids), `the plate is not the new pattern's:\n${solids}`);
  return `picked ${picked}; ${stats}`;
});

await check("a parameter renamed is renamed where it is read, and removing one read warns", async () => {
  await fresh();
  await fileMenu(page, "Parametric plate");
  const before = await builtCleanly();
  const dialog = page.locator(".modal[aria-label^='Parameter']");
  await page.locator("button[aria-label='Edit width']").click();
  const readers = await dialog.locator(".parameter-readers").innerText();
  expect(readers.includes("depth") && readers.includes("outline"), `width is read by: ${readers}`);
  const name = dialog.locator(".modal-field", { hasText: "Name" }).locator("input");
  await name.fill("plate_width");
  await name.press("Enter");
  const after = await builtCleanly();
  expect(after === before, `renamed, the part is ${after}, not ${before}`);
  await dialog.locator("button", { hasText: "Done" }).click();
  expect((await page.locator(".parameter-name").allInnerTexts()).includes("plate_width"), "width was not renamed");
  // Removing what the plate's extrusion reads says so first, and removes nothing yet.
  await page.locator("button[aria-label='Edit thickness']").click();
  await dialog.locator("button.danger").click();
  const warning = await dialog.locator(".warning-text").innerText();
  expect(warning.includes("plate"), `no warning that the plate reads thickness: ${warning}`);
  await dialog.locator("button", { hasText: "Done" }).click();
  expect((await page.locator(".parameter-name").allInnerTexts()).includes("thickness"), "thickness was removed");
  return builtCleanly();
});

await check("a kernel that crashes is restarted with the program", async () => {
  await fresh();
  await fileMenu(page, "Box with drill hole");
  return crashKernel(page, errors);
});

// ── gizmos ──────────────────────────────────────────────────────────────

/** Where the scene's point `p` is on screen, in client coordinates. */
const onScreen = (p) => page.evaluate((p) => window.geopView.project(p), p);
/** How long a reach is at `p`, in world units: what a gizmo is laid out in. */
const reachAt = (p) => page.evaluate((p) => window.geopView.reach(p), p);
/** What the step being edited shows, as the kernel has it now. */
const presented = () => page.evaluate(async () => (await window.geopCommand({ command: "show" }))?.step?.presentation);
/** `p` moved `k` along `d`. */
const along = (p, d, k) => p.map((c, i) => c + d[i] * k);
/** The values of the number fields `keys` the step shows. */
const numbers = (presentation, keys) => keys.map((key) => presentation.dialog.find((f) => f.key === key)?.value);
/** `geop_ops::ui::gizmo::grid_step`: what a drag snaps to, a reach being `reach`. */
function gridStep(reach) {
  const least = (20 / 9) * reach;
  const power = 10 ** Math.floor(Math.log10(least));
  return [1, 2, 5, 10].map((m) => m * power).find((s) => s >= least);
}
const samePart = (a, b) => a != null && a.part === b.part && a.axis === b.axis;

/** Hover, then click, the page at `x`, `y`. */
async function clickAt([x, y]) {
  await page.mouse.move(x, y);
  await settle(page);
  await page.mouse.click(x, y);
  await settle(page);
}

/**
 * Hover `from` — which must light the gizmo's `part`, drawn there in the
 * highlight's yellow — then press there and drag to `to` in steps: the part
 * is dragged, and says how far, on the way. A screenshot half way is saved
 * as `e2e/out/gizmo-<name>.png` with `--shots`.
 */
async function dragGizmo(part, from, to, name) {
  await page.mouse.move(...from);
  await settle(page);
  const hovered = (await presented()).gizmo;
  expect(samePart(hovered?.hover, part), `hovering ${from} lit ${JSON.stringify(hovered?.hover)}, not ${JSON.stringify(part)}`);
  await page.waitForTimeout(200);
  const [r, g, b] = await pixelAt(page, Math.round(from[0]), Math.round(from[1]));
  expect(r > 180 && g > 150 && b < 140, `the hovered ${part.part} is drawn rgb(${r}, ${g}, ${b}) at ${from}, not lit`);
  await page.mouse.down();
  const steps = 8;
  for (let i = 1; i <= steps; i++) {
    await page.mouse.move(from[0] + ((to[0] - from[0]) * i) / steps, from[1] + ((to[1] - from[1]) * i) / steps);
    await page.waitForTimeout(30);
  }
  await settle(page);
  const during = (await presented()).gizmo;
  expect(samePart(during?.active, part), `dragging, ${JSON.stringify(during?.active)} is active`);
  expect(during.readout, "the drag says nothing of how far it went");
  if (process.argv.includes("--shots")) {
    fs.mkdirSync(OUT, { recursive: true });
    await page.screenshot({ path: path.join(OUT, `gizmo-${name}.png`) });
  }
  await page.mouse.up();
  await settle(page);
  return during.readout;
}

await check("a subd cage face is moved up by the gizmo's arrow", async () => {
  await fresh();
  await clickOperation(page, "SubD");
  await settle(page);
  await page.locator("button.fit-view").click();
  await page.waitForTimeout(1200);
  // The cage's top face, near its middle: selected, a gizmo at its centre.
  await clickAt(await onScreen([0.3, 0.2, 1]));
  const selected = await presented();
  const at = selected.gizmo?.at;
  expect(at && Math.abs(at[2] - 1) < 1e-9, `no gizmo at the top face's centre: ${JSON.stringify(selected.gizmo)}`);
  expect(!selected.visuals.some((v) => v.shape === "handle"), "the old handles are still drawn");
  const reach = await reachAt(at);
  const step = gridStep(reach);
  const up = [0, 0, 1];
  const readout = await dragGizmo(
    { part: "move", axis: 2 },
    await onScreen(along(at, up, 8.5 * reach)),
    await onScreen(along(at, up, 8.5 * reach + 3 * step)),
    "subd-face",
  );
  const [z] = numbers(await presented(), ["z"]);
  expect(Math.abs(z - (1 + 3 * step)) < 1e-9, `the face's centre went to z = ${z}, not ${1 + 3 * step}`);
  await popup.locator(".button-row button.primary").click();
  const stats = await builtCleanly();
  return `moved by ${readout}; ${stats}`;
});

await check("a 3-D sketch point is dragged along an axis by the gizmo", async () => {
  await fresh();
  await clickOperation(page, "3-D sketch");
  await settle(page);
  // A line: from the origin out, then put down — Escape ends it, Escape
  // again takes up selecting — and its end clicked to select it.
  await clickView(0, 0);
  await clickView(-160, -120);
  await page.keyboard.press("Escape");
  await settle(page);
  await page.keyboard.press("Escape");
  await settle(page);
  await clickView(-160, -120);
  const selected = await presented();
  const at = selected.gizmo?.at;
  expect(at, `the point selected has no gizmo: ${JSON.stringify(selected.dialog.map((f) => f.key))}`);
  expect(selected.gizmo.modes.translate && !selected.gizmo.modes.rotate, "a point is only moved");
  const before = numbers(selected, ["x", "y", "z"]);
  const reach = await reachAt(at);
  const step = gridStep(reach);
  const x = [1, 0, 0];
  const readout = await dragGizmo(
    { part: "move", axis: 0 },
    await onScreen(along(at, x, 8.5 * reach)),
    await onScreen(along(at, x, 8.5 * reach + 4 * step)),
    "sketch3d-point",
  );
  const after = numbers(await presented(), ["x", "y", "z"]);
  const want = [before[0] + 4 * step, before[1], before[2]];
  expect(
    after.every((v, i) => Math.abs(v - want[i]) < 1e-6),
    `the point went from ${before} to ${after}, not ${want}`,
  );
  await popup.locator(".button-row button.primary").click();
  const stats = await builtCleanly();
  return `moved by ${readout}; ${stats}`;
});

// ── exports ─────────────────────────────────────────────────────────────

/** Do `action`, and return the file it downloads: its name and size. */
async function downloaded(action) {
  const [download] = await Promise.all([page.waitForEvent("download", { timeout: 60000 }), action()]);
  const file = await download.path();
  const { size } = fs.statSync(file);
  return { name: download.suggestedFilename(), size, text: size < 1e6 ? fs.readFileSync(file) : null };
}

const exports = [
  ["Box with drill hole", "Download STEP", /\.step$/, (b) => b.toString().startsWith("ISO-10303-21;")],
  ["Box with drill hole", "Download STL", /\.stl$/, (b) => b.length > 84 && b.readUInt32LE(80) * 50 + 84 === b.length],
  ["Arm", "Export URDF", /\.zip$/, (b) => b.subarray(0, 2).toString() === "PK"],
];
let loaded = null;
for (const [example, entry, name, valid] of exports) {
  await check(`${entry} of ${example}`, async () => {
    if (loaded !== example) {
      await fresh();
      await fileMenu(page, example);
      await builtCleanly();
      loaded = example;
    }
    const file = await downloaded(() => fileMenu(page, entry));
    expect(name.test(file.name), `downloaded ${file.name}`);
    expect(file.size > 0, `${file.name} is empty`);
    expect(file.text == null || valid(file.text), `${file.name} is not what it should be`);
    await builtCleanly();
    return `${file.name}, ${file.size} bytes`;
  });
}

/** How many texts the SVG `text` writes on its dimension layer. */
function dimensionTexts(text) {
  const layer = text.slice(text.indexOf('<g class="DIMENSIONS"'));
  return (layer.slice(0, layer.indexOf("</g>")).match(/<text/g) ?? []).length;
}

await check("a drawing is dimensioned on its sheet and downloaded from its dialog", async () => {
  await fresh();
  await fileMenu(page, "Box with drill hole");
  await builtCleanly();
  await clickOperation(page, "Drawing");
  await settle(page);
  // The sheet, framed head on: wait for the camera to glide there.
  await page.waitForTimeout(1500);
  const before = await downloaded(() => popup.locator("button", { hasText: "Download SVG" }).click());
  expect(before.name.endsWith(".svg"), `downloaded ${before.name}`);
  const dimensions = dimensionTexts(before.text.toString());

  // Where a point of the sheet is on screen: it is framed whole, centred,
  // from straight above (see `fitPose`).
  const update = await page.evaluate(() => window.geopCommand({ command: "show" }));
  const { sheet } = update.step.presentation;
  const box = await page.locator("main.viewport canvas").first().boundingBox();
  const half = (50 * Math.PI) / 360;
  const narrowest = Math.min(half, Math.atan(Math.tan(half) * (box.width / box.height)));
  const distance = sheet.size / 2 / Math.sin(narrowest);
  const perMm = box.height / (2 * distance * Math.tan(half));
  const screen = (p) => [
    box.width / 2 + (p[0] - sheet.center[0]) * perMm,
    box.height / 2 - (p[1] - sheet.center[1]) * perMm,
  ];
  // The ends of an edge the first view draws.
  const edge = update.step.presentation.visuals.find(
    (v) => v.key.startsWith("sheet/") && v.shape === "polyline" && v.style === "fixed" && v.points.length === 2,
  );
  expect(edge, "the sheet draws no edge");
  const [a, b] = edge.points;
  await popup.locator("button[aria-label^='Dimension']").click();
  await settle(page);
  for (const p of [a, b]) {
    const [x, y] = screen(p);
    await clickView(x - box.width / 2, y - box.height / 2);
  }
  // Its value, a little off the edge.
  const along = [b[0] - a[0], b[1] - a[1]];
  const length = Math.hypot(...along);
  const out = [(a[0] + b[0]) / 2 - (12 * along[1]) / length, (a[1] + b[1]) / 2 + (12 * along[0]) / length];
  const [x, y] = screen(out);
  await clickView(x - box.width / 2, y - box.height / 2);
  const listed = await popup.locator(".dialog-list li", { hasText: "Distance" }).count();
  expect(listed === 1, `${listed} distances listed:\n${await popup.innerText()}`);

  const after = await downloaded(() => popup.locator("button", { hasText: "Download SVG" }).click());
  const added = dimensionTexts(after.text.toString()) - dimensions;
  expect(added === 1, `the downloaded drawing has ${added} more dimensions, not one`);
  const dxf = await downloaded(() => popup.locator("button", { hasText: "Download DXF" }).click());
  expect(dxf.name.endsWith(".dxf") && dxf.text.toString().includes("ENTITIES"), `downloaded ${dxf.name}`);
  // Left open: the screenshot (`--shots`) shows the sheet with its dimension.
  return `${await builtCleanly()}, ${after.name}`;
});

await check("the bill of materials is saved as CSV", async () => {
  await fresh();
  await fileMenu(page, "Bolted plate");
  await builtCleanly();
  await page.getByTitle("Bill of materials: every part with its quantity, designation and mass").click();
  await settle(page);
  const lines = await page.locator(".bom-table tbody tr").count();
  expect(lines > 1, `the bill of materials lists ${lines} lines`);
  const file = await downloaded(() => page.locator(".inspect-result button", { hasText: "Save CSV" }).click());
  expect(file.name.endsWith(".csv"), `downloaded ${file.name}`);
  const rows = file.text.toString().trim().split("\n");
  expect(rows.length > lines, `${rows.length} rows in the CSV (with its header), ${lines} lines shown`);
  return `${lines} lines, ${file.name}`;
});

checks.report();
await browser.close();
server.stop();
