#!/usr/bin/env bash
# Tests only the crates a change can affect (ARCHITECTURE_PLAN.md §8).
#
# Usage: scripts/test-affected.sh [--dry-run] [base-ref] [-- cargo test args]
#
#   base-ref   what the change is measured against: the files of
#              `git diff --name-only <base>...HEAD`, plus uncommitted and
#              untracked files. Defaults to `origin/main` when that ref
#              exists, otherwise `HEAD~1`.
#   --dry-run  print the crates that would be tested, run nothing.
#
# Files map to the crate whose directory holds them. A crate is affected
# when it is changed or depends, through normal and build dependencies, on
# an affected crate; a crate whose tests dev-depend on an affected crate is
# tested too. A change under core/, or to the workspace's Cargo.toml or
# Cargo.lock, affects the whole workspace and the script says so instead of
# listing every crate. Files outside any crate (scripts, docs, web/) are
# listed and tested by nothing here.

set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"

dry_run=0
base=""
cargo_args=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run) dry_run=1 ;;
        --) shift; cargo_args=("$@"); break ;;
        -*) echo "test-affected: unknown option $1" >&2; exit 2 ;;
        *) [[ -z "$base" ]] || { echo "test-affected: one base-ref only" >&2; exit 2; }
           base="$1" ;;
    esac
    shift
done
if [[ -z "$base" ]]; then
    if git rev-parse --verify --quiet origin/main >/dev/null; then base=origin/main; else base=HEAD~1; fi
fi

mapfile -t changed_files < <(
    {
        git diff --name-only "$base"...HEAD
        git diff --name-only HEAD
        git ls-files --others --exclude-standard
    } | sort -u)
echo "test-affected: ${#changed_files[@]} files changed against $base"

run() {
    echo "+ $*"
    [[ $dry_run -eq 1 ]] || exec timeout 1800 prlimit --as=16000000000 "$@"
}

run_workspace() {
    echo "test-affected: $1 affects the whole workspace"
    if [[ $dry_run -eq 1 ]]; then echo "workspace"; exit 0; fi
    run cargo test --workspace "${cargo_args[@]}"
}

metadata="$(cargo metadata --format-version 1 --no-deps)"

# "<name> <directory relative to the root>", longest directory first so the
# first match of a file is its innermost crate.
mapfile -t crates < <(jq -r --arg root "$root/" '
    .packages[] | "\(.name) \(.manifest_path | ltrimstr($root) | rtrimstr("/Cargo.toml"))"' \
    <<<"$metadata" | awk '{ print length($2), $0 }' | sort -rn | cut -d' ' -f2-)

declare -A affected=()
outside=()
for file in "${changed_files[@]}"; do
    case "$file" in
        Cargo.toml|Cargo.lock) run_workspace "$file" ;;
    esac
    crate=""
    for entry in "${crates[@]}"; do
        dir="${entry#* }"
        if [[ "$file" == "$dir"/* ]]; then crate="${entry%% *}"; break; fi
    done
    if [[ -z "$crate" ]]; then
        outside+=("$file")
    elif [[ "$file" == core/* ]]; then
        run_workspace "a change to $crate ($file)"
    else
        affected["$crate"]=1
    fi
done

for file in "${outside[@]}"; do echo "test-affected: $file is in no crate"; done
if [[ ${#affected[@]} -eq 0 ]]; then
    echo "test-affected: no crate changed, nothing to test"
    exit 0
fi
echo "test-affected: changed crates: ${!affected[*]}"

# "<kind> <from> <to>" for every dependency between workspace crates.
edges="$(jq -r '
    .packages[] | .name as $from
    | .dependencies[] | select(.path != null)
    | "\(.kind // "normal") \($from) \(.name)"' <<<"$metadata" | sort -u)"

# Builds: what depends on an affected crate is affected, transitively.
grew=1
while [[ $grew -eq 1 ]]; do
    grew=0
    while read -r kind from to; do
        [[ "$kind" != dev && -n "${affected["$to"]:-}" && -z "${affected["$from"]:-}" ]] || continue
        affected["$from"]=1
        grew=1
    done <<<"$edges"
done
# Tests: a crate whose tests use an affected crate is tested, but nothing
# builds on its tests.
declare -A tested=()
for crate in "${!affected[@]}"; do tested["$crate"]=1; done
while read -r kind from to; do
    [[ "$kind" == dev && -n "${affected["$to"]:-}" ]] && tested["$from"]=1
done <<<"$edges"

mapfile -t to_test < <(printf '%s\n' "${!tested[@]}" | sort)
echo "test-affected: ${#to_test[@]} crates to test"
if [[ $dry_run -eq 1 ]]; then
    printf '%s\n' "${to_test[@]}"
    exit 0
fi
packages=()
for crate in "${to_test[@]}"; do packages+=(-p "$crate"); done
run cargo test "${packages[@]}" "${cargo_args[@]}"
