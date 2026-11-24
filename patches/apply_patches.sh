#!/usr/bin/env bash
# apply_patches.sh - apply our local patches to the third-party submodule(s).
#
# Usage (from anywhere):  bash patches/apply_patches.sh
#
# Patches live in patches/<submodule name>/*.patch and are applied, in name
# order, to src/third_party/<submodule name>. Safe to run again and again:
#   - already applied   -> prints "already applied", changes nothing
#   - not yet applied   -> applies it, prints "applied"
#   - neither works     -> prints an ERROR and exits 1 (e.g. the submodule is
#                          missing or at a different commit than 326cf8a)
# Exit code 0 = every patch is in place.
#
# The patched files make the submodule "dirty" (changed files inside it).
# .gitmodules has "ignore = dirty" for it, so "git status" in the main repo
# does not list it, and nothing about it can be committed by accident. The
# submodule pointer (commit 326cf8a) does not change.
#
# After a patch is newly applied, rebuild the package that contains it, e.g.
#   colcon build --packages-select explore_lite
# (run_sim.sh does this by itself when needed).

set -uo pipefail

PATCH_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$PATCH_ROOT/.." && pwd)"
status=0

# git apply inside the submodule. safe.directory='*' (only for this command):
# the repo is a Windows bind mount owned by another user than the container's,
# which newer git otherwise refuses with "detected dubious ownership".
# --ignore-whitespace: also works if Windows checked the submodule out with
# CRLF line endings.
if ! command -v git >/dev/null 2>&1; then
  echo "ERROR: git is not installed, cannot apply patches/ (sudo apt install git)." >&2
  exit 1
fi
gapply() { git -c safe.directory='*' -C "$target" apply --ignore-whitespace "$@"; }

for dir in "$PATCH_ROOT"/*/; do
  [ -d "$dir" ] || continue
  name="$(basename "$dir")"
  target="$REPO_ROOT/src/third_party/$name"
  for patch in "$dir"*.patch; do
    [ -f "$patch" ] || continue
    label="$name/$(basename "$patch")"
    if [ -z "$(ls -A "$target" 2>/dev/null)" ]; then
      echo "ERROR: patch $label: $target is empty. Run 'git submodule update --init' (on Windows) first." >&2
      status=1
      continue
    fi
    if gapply --reverse --check "$patch" >/dev/null 2>&1; then
      echo "patch $label: already applied"
    elif gapply --check "$patch" >/dev/null 2>&1; then
      if gapply "$patch"; then
        echo "patch $label: applied now (rebuild the patched package, e.g. colcon build --packages-select explore_lite)"
      else
        echo "ERROR: patch $label: git apply failed." >&2
        status=1
      fi
    else
      echo "ERROR: patch $label does not apply to $target (submodule at another commit or edited by hand?)." >&2
      gapply --check "$patch" >&2 || true
      status=1
    fi
  done
done

exit "$status"
