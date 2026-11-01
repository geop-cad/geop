#!/usr/bin/env python3
"""Checks the book against the code: every path and identifier the lectures
cite has to exist. Run it from anywhere, after the kernel changes too:

    book/scripts/check_citations.py [book/src/lectures/some.md ...]

What it reads: the inline code spans of the Markdown (not math, not fenced
blocks; put those through a compiler or your own eyes).
- A span that looks like a repo path (core/..., lib/..., ops/..., cad/...,
  web/..., docs/..., scripts/..., homepage/..., `AGENTS.md`) has to exist.
- A span that looks like an identifier (snake_case, CamelCase, `a::b`, `f()`,
  `x.y`) must have every name in it appear as a word in the repo's sources
  (.rs, .ts, .tsx, .sh, .toml, .md outside book/).
Spans a lecture quotes on purpose that are not in the code (a name it says is
not there, an example from another system) go in scripts/citation_allow.txt.
Prints what it could not find, exits 1 if anything.
"""
import glob
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BOOK = os.path.dirname(HERE)
REPO = os.path.dirname(BOOK)
SKIP_DIRS = {"target", "node_modules", "dist", ".git", ".claude", "code-extension-playground", "book", "pkg", "outputs"}
EXTS = (".rs", ".ts", ".tsx", ".sh", ".toml", ".md", ".mjs", ".yml")
PATH_ROOTS = ("book/", "core/", "lib/", "ops/", "cad/", "web/", "docs/", "scripts/", "homepage/", ".github/", "vscode-extension/", "cloudflare-functions/")


def source_words():
    words = set()
    for root, dirs, files in os.walk(REPO):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        for f in files:
            if f.endswith(EXTS):
                try:
                    text = open(os.path.join(root, f), encoding="utf-8").read()
                except UnicodeDecodeError:
                    continue
                words.update(re.findall(r"[A-Za-z_][A-Za-z0-9_]*", text))
    return words


def spans(text):
    # Drop fenced blocks, keep inline code.
    text = re.sub(r"^(```|~~~).*?^\1\s*$", "", text, flags=re.S | re.M)
    for m in re.finditer(r"(?<!`)`([^`\n]+?)`(?!`)", text):
        yield m.group(1)


IDENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(?:(?:::|\.)[A-Za-z_][A-Za-z0-9_]*)*(?:!|\(\))?$")


def is_identifier(s):
    if not IDENT.match(s):
        return False
    # Plain lower-case words and short names are English or mathematics.
    return "_" in s or "::" in s or re.search(r"[a-z][A-Z]", s) or re.match(r"^[A-Z][a-z]+[A-Z]", s) or s.endswith("()") or s.endswith("!")


def main():
    allow = set()
    allow_files = [os.path.join(HERE, "citation_allow.txt")] + sorted(glob.glob(os.path.join(HERE, "citation_allow.d", "*.txt")))
    for allow_file in allow_files:
        if os.path.exists(allow_file):
            allow |= {l.strip() for l in open(allow_file) if l.strip() and not l.startswith("#")}
    files = sys.argv[1:]
    if not files:
        for root, _, names in os.walk(os.path.join(BOOK, "src")):
            for n in names:
                if n.endswith(".md"):
                    files.append(os.path.join(root, n))
    words = source_words()
    bad = 0
    for path in sorted(files):
        text = open(path, encoding="utf-8").read()
        for s in spans(text):
            if s.startswith("$") or s in allow:
                continue
            candidate = s.split(":")[0].split("#")[0].rstrip("/")
            if candidate.startswith(PATH_ROOTS) or candidate in ("AGENTS.md", "DEVELOPERS.md", "ARCHITECTURE_PLAN.md"):
                if " " in candidate or "*" in candidate or "<" in candidate:
                    continue
                if not os.path.exists(os.path.join(REPO, candidate)):
                    print(f"{os.path.relpath(path, BOOK)}: no such path: {s}")
                    bad += 1
            elif is_identifier(s):
                names = re.findall(r"[A-Za-z_][A-Za-z0-9_]*", s)
                missing = [n for n in names if n not in words]
                if missing:
                    print(f"{os.path.relpath(path, BOOK)}: not in the sources: {s} ({', '.join(missing)})")
                    bad += 1
    print(f"{len(files)} files checked, {bad} problems")
    sys.exit(1 if bad else 0)


main()
