#!/usr/bin/env bash
# Cut a release: set the workspace version, commit, tag `v<version>` and
# push the tag. The tag starts .github/workflows/release.yml, which builds
# the `geop` CLI for every platform (with the page `geop open` serves), the
# VS Code extension for every platform and the web app, and publishes them.
#
#   scripts/release.sh 0.2.0
#
# Runs from a clean checkout of main. Nothing is pushed until the version
# builds here (`cargo build -p geop-cad-cli`), so a typo in Cargo.toml is
# found before the tag exists.
set -euo pipefail

version="${1:-}"
if [[ ! "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[0-9A-Za-z.]+)?$ ]]; then
  echo "usage: scripts/release.sh <version>   e.g. 0.2.0 or 0.2.0-rc1" >&2
  exit 2
fi

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

if [ -n "$(git status --porcelain)" ]; then
  echo "the working tree is not clean; commit or discard first" >&2
  exit 1
fi
branch="$(git rev-parse --abbrev-ref HEAD)"
if [ "$branch" != "main" ]; then
  echo "releases are cut from main, not $branch" >&2
  exit 1
fi
if git rev-parse -q --verify "refs/tags/v$version" > /dev/null; then
  echo "v$version is already tagged" >&2
  exit 1
fi

# The one version every crate shares: [workspace.package] in Cargo.toml.
current="$(sed -n '/^\[workspace\.package\]/,/^\[/{s/^version = "\(.*\)"/\1/p}' Cargo.toml)"
echo "version $current -> $version"
sed -i '/^\[workspace\.package\]/,/^\[/{s/^version = ".*"/version = "'"$version"'"/}' Cargo.toml
# Cargo.lock records the workspace crates' versions too.
timeout 1800 prlimit --as=16000000000 cargo build --release --locked -p geop-cad-cli 2> /dev/null \
  || timeout 1800 prlimit --as=16000000000 cargo build --release -p geop-cad-cli

# The extension carries the same version: `geop vscode` installs the
# extension of its own.
(cd vscode-extension && npm version "$version" --no-git-tag-version --allow-same-version > /dev/null)

git add Cargo.toml Cargo.lock vscode-extension/package.json vscode-extension/package-lock.json
git commit -m "Release v$version"
git tag -a "v$version" -m "geop v$version"
git push origin main "v$version"
echo "pushed v$version: the Release workflow builds and publishes it"
