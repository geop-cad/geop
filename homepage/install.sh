#!/bin/sh
# Install the geop command line tool from its GitHub release:
#
#   curl -fsSL https://geop-cad.dev/install.sh | sh
#
# Downloads the archive for this machine and the release's SHA256SUMS,
# checks the archive's SHA-256 (and stops if it does not match), and puts
# `geop` in ~/.local/bin. Nothing is run with sudo and no shell profile is
# changed: if that folder is not on PATH, the line to add is printed.
#
# Set before `sh`, e.g. `curl -fsSL https://geop-cad.dev/install.sh | GEOP_VERSION=0.2.0 sh`:
#   GEOP_VERSION           the version to install (default: the latest release)
#   GEOP_INSTALL_DIR       where to put geop (default: ~/.local/bin)
#   GEOP_RELEASE_BASE_URL  where releases are downloaded from (default:
#                          https://github.com/geop-cad/geop/releases/download),
#                          a file is <base>/v<version>/<name>
set -eu

repo="geop-cad/geop"
base="${GEOP_RELEASE_BASE_URL:-https://github.com/$repo/releases/download}"
install_dir="${GEOP_INSTALL_DIR:-$HOME/.local/bin}"

say() { printf '%s\n' "$*" >&2; }
fail() {
  say "geop install: $*"
  exit 1
}

# What downloads: curl, else wget.
if command -v curl > /dev/null 2>&1; then
  fetch() { curl --proto '=https,file' -fsSL "$1" -o "$2"; }
  fetch_text() { curl --proto '=https' -fsSL "$1"; }
elif command -v wget > /dev/null 2>&1; then
  fetch() { wget -q -O "$2" "$1"; }
  fetch_text() { wget -q -O - "$1"; }
else
  fail "neither curl nor wget is installed"
fi

# What checks a SHA-256: without one, nothing is installed.
if command -v sha256sum > /dev/null 2>&1; then
  sha256() { sha256sum "$1" | cut -d ' ' -f 1; }
elif command -v shasum > /dev/null 2>&1; then
  sha256() { shasum -a 256 "$1" | cut -d ' ' -f 1; }
else
  fail "neither sha256sum nor shasum is installed: cannot check the download"
fi

# The machine, as the release names its archives.
os="$(uname -s)"
arch="$(uname -m)"
case "$arch" in
  x86_64 | amd64) arch="x86_64" ;;
  aarch64 | arm64) arch="aarch64" ;;
  *) fail "no geop is released for the processor $arch" ;;
esac
case "$os" in
  Linux) target="$arch-unknown-linux-gnu" ;;
  Darwin)
    # A shell under Rosetta says x86_64 on an Apple processor: the native
    # build is the one to run there.
    if [ "$arch" = "x86_64" ] && [ "$(sysctl -n sysctl.proc_translated 2> /dev/null || echo 0)" = "1" ]; then
      arch="aarch64"
    fi
    target="$arch-apple-darwin"
    ;;
  MINGW* | MSYS* | CYGWIN*) fail "on Windows, run in PowerShell: irm https://geop-cad.dev/install.ps1 | iex" ;;
  *) fail "no geop is released for $os" ;;
esac

# The version: given, or the latest release's.
version="${GEOP_VERSION:-}"
if [ -z "$version" ]; then
  version="$(fetch_text "https://api.github.com/repos/$repo/releases/latest" |
    sed -n 's/.*"tag_name"[[:space:]]*:[[:space:]]*"v\{0,1\}\([^"]*\)".*/\1/p' | head -n 1)" ||
    true
  [ -n "$version" ] || fail "could not find the latest release of $repo (set GEOP_VERSION to choose one)"
fi
version="${version#v}"

name="geop-$version-$target"
archive="$name.tar.gz"
tmp="$(mktemp -d 2> /dev/null || mktemp -d -t geop)"
trap 'rm -rf "$tmp"' EXIT INT TERM

say "downloading geop $version for $target"
fetch "$base/v$version/SHA256SUMS" "$tmp/SHA256SUMS" ||
  fail "could not download $base/v$version/SHA256SUMS: is there a release v$version?"
fetch "$base/v$version/$archive" "$tmp/$archive" ||
  fail "could not download $base/v$version/$archive"

expected="$(awk -v f="$archive" '$2 == f || $2 == "*" f { print $1; exit }' "$tmp/SHA256SUMS")"
[ -n "$expected" ] || fail "SHA256SUMS lists no $archive"
actual="$(sha256 "$tmp/$archive")"
expected="$(printf '%s' "$expected" | tr 'A-F' 'a-f')"
[ "$actual" = "$expected" ] ||
  fail "$archive is not the file released: its SHA-256 is $actual, SHA256SUMS says $expected"

tar -xzf "$tmp/$archive" -C "$tmp"
[ -f "$tmp/$name/geop" ] || fail "$archive holds no $name/geop"
mkdir -p "$install_dir"
# Put in place whole: a geop running meanwhile keeps its own file.
cp "$tmp/$name/geop" "$install_dir/.geop.new"
chmod 755 "$install_dir/.geop.new"
mv -f "$install_dir/.geop.new" "$install_dir/geop"
say "installed geop $version to $install_dir/geop"

case ":${PATH:-}:" in
  *":$install_dir:"*) geop="geop" ;;
  *)
    geop="$install_dir/geop"
    say ""
    say "$install_dir is not on your PATH. Add it, e.g. in ~/.profile (or your shell's rc file):"
    say "  export PATH=\"$install_dir:\$PATH\""
    ;;
esac

say ""
say "Next:"
say "  $geop open                          edit the .geop files of this folder in your browser"
say "  claude mcp add geop -- $geop mcp    let Claude Code model parts with geop"
say "  codex mcp add geop -- $geop mcp     let Codex model parts with geop"
say "  $geop vscode                        install the VS Code extension"
