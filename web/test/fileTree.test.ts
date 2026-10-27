import assert from "node:assert/strict";
import { test } from "node:test";
import { fileTree, foldersOf, rowKey, visibleRows, type FileNode } from "../src/fileTree.ts";

const files = [
  { path: "bolt.geop", accepted: true },
  { path: "notes.txt", accepted: false },
  { path: "parts/bracket/plate.geop", accepted: true },
  { path: "parts/base.geop", accepted: true },
  { path: "../shared/hinge.geop", accepted: true },
  { path: "std:iso4032_hex_nut.geop", accepted: true },
  { path: "std:ball_bearing.geop", accepted: true },
];

/** The tree as indented lines: a folder by its name and a `/`, a file greyed by a `~`. */
function lines(rows: { node: FileNode; depth: number }[]): string[] {
  return rows.map(({ node, depth }) => {
    const name = node.kind === "folder" ? `${node.name}/` : node.accepted ? node.name : `~${node.name}`;
    return "  ".repeat(depth) + name;
  });
}

test("the folders a path is in come from its slashes, a shared library's from its scheme", () => {
  assert.deepEqual(foldersOf("parts/bracket/plate.geop"), ["parts/", "parts/bracket/"]);
  assert.deepEqual(foldersOf("bolt.geop"), []);
  assert.deepEqual(foldersOf("../shared/hinge.geop"), ["../", "../shared/"]);
  assert.deepEqual(foldersOf("std:iso4032_hex_nut.geop"), ["std:"]);
});

test("files are a tree: the parent first, folders before files, the shared library last", () => {
  const tree = fileTree(files);
  const all = new Set(["../", "../shared/", "parts/", "parts/bracket/", "std:"]);
  assert.deepEqual(lines(visibleRows(tree, all, "")), [
    "../",
    "  shared/",
    "    hinge.geop",
    "parts/",
    "  bracket/",
    "    plate.geop",
    "  base.geop",
    "bolt.geop",
    "~notes.txt",
    "std:/",
    "  ball_bearing.geop",
    "  iso4032_hex_nut.geop",
  ]);
  // A file is known by the path sent back; a folder by the start of its files' paths.
  const nut = visibleRows(tree, all, "").find((r) => r.node.kind === "file" && r.node.name.startsWith("iso"));
  assert.equal(nut && rowKey(nut.node), "std:iso4032_hex_nut.geop");
});

test("closed folders hide what is in them; a search opens every folder holding a match", () => {
  const tree = fileTree(files);
  assert.deepEqual(lines(visibleRows(tree, new Set(), "")), ["../", "parts/", "bolt.geop", "~notes.txt", "std:/"]);
  assert.deepEqual(lines(visibleRows(tree, new Set(), "plate")), ["parts/", "  bracket/", "    plate.geop"]);
  // Words in any order, ignoring case, matched against the whole path.
  assert.deepEqual(lines(visibleRows(tree, new Set(), "NUT std")), ["std:/", "  iso4032_hex_nut.geop"]);
  assert.deepEqual(visibleRows(tree, new Set(), "nothing like it"), []);
});

test("only the files the field takes, when asked", () => {
  const tree = fileTree(files);
  const rows = lines(visibleRows(tree, new Set(), "", true));
  assert.ok(!rows.includes("~notes.txt"), rows.join("\n"));
  assert.deepEqual(lines(visibleRows(fileTree([{ path: "a/b.txt", accepted: false }]), new Set(["a/"]), "", true)), []);
});
