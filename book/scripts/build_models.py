#!/usr/bin/env python3
"""Turn binary STL files the real kernel wrote into the small JSON meshes the
book's `model-viewer` demo draws. They are committed, so building the book
needs no kernel; run this again when an example changes noticeably:

    geop examples --out-dir OUT --only rounded_box -O quality=2 ...
    book/scripts/build_models.py OUT/rounded_box.stl ...

Each output is {name, triangles, positions: [x,y,z,...], indices: [...]},
vertices merged where they coincide to the rounding of 4 significant decimals
of the part's size. Nothing is simplified: every triangle is the kernel's.
"""
import json
import os
import struct
import sys

OUT = os.path.join(os.path.dirname(__file__), "..", "src", "assets", "models")


def read_stl(path):
    data = open(path, "rb").read()
    count = struct.unpack_from("<I", data, 80)[0]
    tris = []
    for i in range(count):
        off = 84 + i * 50
        v = struct.unpack_from("<9f", data, off + 12)
        tris.append((v[0:3], v[3:6], v[6:9]))
    return tris


def convert(path):
    name = os.path.splitext(os.path.basename(path))[0]
    tris = read_stl(path)
    lo = [min(t[k][a] for t in tris for k in range(3)) for a in range(3)]
    hi = [max(t[k][a] for t in tris for k in range(3)) for a in range(3)]
    size = max(h - l for h, l in zip(hi, lo)) or 1.0
    grid = size / 20000.0
    ids = {}
    positions = []
    indices = []
    for t in tris:
        for p in t:
            key = tuple(round(c / grid) for c in p)
            if key not in ids:
                ids[key] = len(ids)
                positions.extend(round(c, 4) for c in p)
            indices.append(ids[key])
    out = {"name": name, "triangles": len(tris), "positions": positions, "indices": indices}
    target = os.path.join(OUT, name + ".json")
    with open(target, "w") as f:
        json.dump(out, f, separators=(",", ":"))
    print(f"{name}: {len(tris)} triangles, {len(ids)} vertices, {os.path.getsize(target)} bytes")


for arg in sys.argv[1:]:
    convert(arg)
