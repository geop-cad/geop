#!/usr/bin/env bash
# Checks the dependency rules of the crate tiers (ARCHITECTURE_PLAN.md §2.1):
#
#   geop-core-*              normal deps only on geop-core-*
#   geop-core-program        workspace deps only on geop-core-math
#   geop-core-editor         workspace deps only on geop-core-math and
#                            geop-core-program (editing a step is editing a
#                            step of a program)
#   geop-lib-*               deps only on geop-core-*, geop-ops-model, geop-lib-*
#   geop-lib-stdlib          also on the geop-ops-* whose Args its programs are
#                            written in (ARCHITECTURE_PLAN.md §5.4)
#   geop-ops-*, geop-lib-*   no deps on geop-cad-*
#   core/                    no source names geop_ops_*, geop_lib_*, geop_cad_*
#   cad/geop-cad-base/src    names geop_ops_* only in src/registry.rs (one
#                            `register` line per crate) and src/tests.rs
#                            (the composition's tests)
#
# Every violation is printed as one line. Lines listed in
# scripts/layering-allowed.txt are known violations that later phases burn
# down; anything else fails the check. Dev-dependencies breaking a rule are
# printed as warnings and never fail it.
#
# Usage: scripts/check-layering.sh [--write-allowed]
#   --write-allowed  rewrite the allow-list from the current violations
#                    (only to record the baseline; burn it down by hand).

set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
allowed_file="$root/scripts/layering-allowed.txt"
cd "$root"

for tool in cargo jq grep; do
    command -v "$tool" >/dev/null || { echo "check-layering: needs $tool" >&2; exit 2; }
done

metadata="$(cargo metadata --format-version 1 --no-deps)"

# One line per workspace dependency edge: "<kind> <from> <to>", kind being
# "normal", "dev" or "build".
edges="$(jq -r '
    .packages[] | .name as $from
    | .dependencies[] | select(.path != null)
    | "\(.kind // "normal") \($from) \(.name)"' <<<"$metadata" | sort -u)"

# Prints the rule an edge breaks, or nothing.
rule_broken() {
    local from="$1" to="$2"
    case "$from" in
        geop-core-program)
            [[ "$to" == geop-core-math ]] || echo "geop-core-program depends on geop-core-math only"
            ;;
        geop-core-editor)
            [[ "$to" == geop-core-math || "$to" == geop-core-program ]] \
                || echo "geop-core-editor depends on geop-core-math and geop-core-program only"
            ;;
        geop-core-*)
            [[ "$to" == geop-core-* ]] || echo "core depends on core only"
            ;;
        geop-lib-stdlib)
            [[ "$to" == geop-core-* || "$to" == geop-ops-* || "$to" == geop-lib-* ]] \
                || echo "geop-lib-stdlib depends on core, ops and lib only"
            ;;
        geop-lib-*)
            [[ "$to" == geop-core-* || "$to" == geop-ops-model || "$to" == geop-lib-* ]] \
                || echo "lib depends on core, geop-ops-model and lib only"
            ;;
        geop-ops-*)
            [[ "$to" != geop-cad-* ]] || echo "ops never depend on cad"
            ;;
    esac
}

violations=()
warnings=()
while read -r kind from to; do
    [[ -n "$kind" ]] || continue
    rule="$(rule_broken "$from" "$to")"
    [[ -n "$rule" ]] || continue
    if [[ "$kind" == dev ]]; then
        warnings+=("dev-dep $from -> $to: $rule")
    else
        violations+=("dep $from -> $to: $rule")
    fi
done <<<"$edges"

# Sources: one line per file and name, not per line, so that the allow-list
# does not go stale when a file is edited.
while IFS=: read -r file name; do
    [[ -n "$file" ]] || continue
    violations+=("source $file: core names $name")
done < <(grep -rEo --exclude-dir=target 'geop_(ops|lib|cad)_[A-Za-z0-9_]*' core/ | sort -u)

while IFS=: read -r file name; do
    [[ -n "$file" ]] || continue
    case "$file" in
        cad/geop-cad-base/src/registry.rs | cad/geop-cad-base/src/tests.rs) continue ;;
    esac
    violations+=("source $file: cad-base names $name outside registry.rs and tests.rs")
done < <(grep -rEo 'geop_ops_[A-Za-z0-9_]*' cad/geop-cad-base/src/ | sort -u)

if [[ "${1:-}" == --write-allowed ]]; then
    {
        echo "# Known layering violations, one per line as scripts/check-layering.sh prints"
        echo "# them. Phases of ARCHITECTURE_PLAN.md remove lines; nothing adds them."
        printf '%s\n' "${violations[@]}"
    } >"$allowed_file"
    echo "check-layering: wrote ${#violations[@]} violations to ${allowed_file#"$root"/}"
    exit 0
fi

allowed=()
if [[ -f "$allowed_file" ]]; then
    mapfile -t allowed < <(grep -vE '^[[:space:]]*(#|$)' "$allowed_file")
fi
declare -A is_allowed=()
for line in "${allowed[@]}"; do is_allowed["$line"]=1; done
declare -A is_current=()
for line in "${violations[@]}"; do is_current["$line"]=1; done

new=()
for line in "${violations[@]}"; do
    [[ -n "${is_allowed["$line"]:-}" ]] || new+=("$line")
done
fixed=()
for line in "${allowed[@]}"; do
    [[ -n "${is_current["$line"]:-}" ]] || fixed+=("$line")
done

for line in "${warnings[@]}"; do echo "warning: $line"; done
for line in "${fixed[@]}"; do
    echo "fixed (remove from the allow-list): $line"
done
for line in "${new[@]}"; do echo "error: $line"; done

echo "check-layering: ${#violations[@]} violations, $(( ${#violations[@]} - ${#new[@]} )) of them allowed" \
    "(${#allowed[@]} on the allow-list), ${#new[@]} new, ${#warnings[@]} dev-dependency warnings"
[[ ${#new[@]} -eq 0 ]]
