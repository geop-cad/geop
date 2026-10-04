#!/usr/bin/env bash
# Downloads a corpus of public STEP files into target/step-corpus/ (inside
# the gitignored target/ directory — never commit them), for the ignored
# corpus test:
#
#   ops/geop-ops-step/scripts/fetch_corpus.sh
#   cargo test -p geop-ops-step --release corpus -- --ignored --nocapture
#
# Sources:
#   - the NIST MBE PMI test models (AP203/AP242, from NIST's STEP File
#     Analyzer repository),
#   - the STEP files of the OCCT, CadQuery and build123d repositories,
#   - a spread of the FreeCAD parts library: the first file (by path) of
#     every folder, under 1.5 MB — fasteners, profiles, bearings, gears,
#     electronics, written by many CAD systems.
#
# Needs curl, unzip and python3. Safe to run again: what is there is kept.
set -euo pipefail

root="$(cd "$(dirname "$0")/../../.." && pwd)"
out="${STEP_CORPUS:-$root/target/step-corpus}"
mkdir -p "$out"
cd "$out"

fetch() { # url file
  if [ ! -s "$2" ]; then
    mkdir -p "$(dirname "$2")"
    curl -sfL --retry 3 -o "$2" "$1" || { echo "failed: $1" >&2; rm -f "$2"; }
  fi
}

raw=https://raw.githubusercontent.com

echo "NIST MBE PMI models"
fetch "$raw/usnistgov/SFA/master/Release/NIST-PMI-STEP-Files.zip" nist.zip
if [ -s nist.zip ] && [ ! -d nist ]; then
  mkdir -p nist && unzip -q -o -j nist.zip -d nist || true
fi

echo "OCCT, CadQuery, build123d"
fetch "$raw/Open-Cascade-SAS/OCCT/master/data/step/linkrods.step" occt/linkrods.step
fetch "$raw/Open-Cascade-SAS/OCCT/master/data/step/screw.step" occt/screw.step
fetch "$raw/CadQuery/cadquery/master/tests/testdata/red_cube_blue_cylinder.step" cadquery/red_cube_blue_cylinder.step
fetch "$raw/gumyr/build123d/dev/docs/M6-1x12-countersunk-screw.step" build123d/M6-1x12-countersunk-screw.step
fetch "$raw/gumyr/build123d/dev/docs/topology_selection/examples/nema-17-bracket.step" build123d/nema-17-bracket.step

echo "FreeCAD parts library"
fetch "https://api.github.com/repos/FreeCAD/FreeCAD-library/git/trees/master?recursive=1" freecad-tree.json
python3 - "$out" <<'EOF'
import json, os, sys, urllib.parse, urllib.request
out = sys.argv[1]
tree = json.load(open(os.path.join(out, "freecad-tree.json")))["tree"]
files = sorted(
    (x for x in tree if x["path"].lower().endswith((".step", ".stp")) and x.get("size", 0) < 1_500_000),
    key=lambda x: x["path"],
)
seen, chosen = set(), []
for x in files:
    folder = os.path.dirname(x["path"])
    if folder not in seen:
        seen.add(folder)
        chosen.append(x["path"])
os.makedirs(os.path.join(out, "freecad"), exist_ok=True)
for path in chosen:
    name = os.path.join(out, "freecad", path.replace("/", "__"))
    if os.path.exists(name) and os.path.getsize(name) > 0:
        continue
    url = "https://raw.githubusercontent.com/FreeCAD/FreeCAD-library/master/" + urllib.parse.quote(path)
    try:
        with urllib.request.urlopen(url, timeout=60) as r, open(name, "wb") as f:
            f.write(r.read())
    except Exception as e:
        print("failed:", path, e, file=sys.stderr)
print(len(chosen), "FreeCAD library files")
EOF

echo "$(find "$out" -type f \( -iname '*.step' -o -iname '*.stp' \) | wc -l) STEP files in $out"
