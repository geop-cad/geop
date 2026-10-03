// Copy the release build of `geop` (see `cad/geop-cad-cli`) next to the
// extension, where `geop.serverPath` defaults to.
import { copyFileSync, mkdirSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const exe = process.platform === "win32" ? "geop.exe" : "geop";
mkdirSync(join(root, "bin"), { recursive: true });
copyFileSync(join(root, "..", "target", "release", exe), join(root, "bin", exe));
