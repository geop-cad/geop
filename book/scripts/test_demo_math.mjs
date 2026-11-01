#!/usr/bin/env node
// Checks each demo's algorithm against its formula, with no browser: loads
// every book/src/demos/*.js with a stub GeopBook, and runs
// book/scripts/demo_tests/<name>.test.mjs with what the demo registered as its
// `algo`. A demo's pure functions are what is tested; its drawing is checked by
// check_demos.mjs.
//
//   node book/scripts/test_demo_math.mjs [name ...]
import { readdirSync, readFileSync, existsSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";
import vm from "node:vm";
import assert from "node:assert/strict";

const here = dirname(fileURLToPath(import.meta.url));
const demos = join(here, "..", "src", "demos");
const tests = join(here, "demo_tests");
const only = process.argv.slice(2);

const algos = {};
const sandbox = { GeopBook: { demo: (name, _init, algo) => { algos[name] = algo; }, draw: {} }, Math, console };
vm.createContext(sandbox);
for (const f of readdirSync(demos).filter((f) => f.endsWith(".js"))) {
  vm.runInContext(readFileSync(join(demos, f), "utf8"), sandbox, { filename: f });
}

let failed = 0, passed = 0;
const missing = [];
for (const name of Object.keys(algos).sort()) {
  if (only.length && !only.includes(name)) continue;
  const file = join(tests, name + ".test.mjs");
  if (!existsSync(file)) { missing.push(name); continue; }
  const mod = await import(pathToFileURL(file));
  for (const [title, fn] of Object.entries(mod)) {
    try {
      fn(algos[name], assert);
      passed++;
      console.log(`ok   ${name}: ${title}`);
    } catch (e) {
      failed++;
      console.log(`FAIL ${name}: ${title}\n     ${e.message}`);
    }
  }
}
if (missing.length) console.log(`demos with no math test: ${missing.join(", ")}`);
console.log(`${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
