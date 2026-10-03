// Copy the release build of `geop` (see `cad/geop-cad-cli`) next to the
// extension, where `geop.serverPath` defaults to.
//
// Written beside it and renamed over it: open editors keep running the
// kernel they started, and an executable that is running cannot be
// written over (ETXTBSY) — but it can be replaced. They pick up the new
// one once reopened.
import { copyFileSync, mkdirSync, renameSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const exe = process.platform === "win32" ? "geop.exe" : "geop";
mkdirSync(join(root, "bin"), { recursive: true });
const dest = join(root, "bin", exe);
copyFileSync(join(root, "..", "target", "release", exe), `${dest}.new`);
renameSync(`${dest}.new`, dest);
