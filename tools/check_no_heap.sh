#!/usr/bin/env sh
# Fails if the freestanding binary references the allocator or the unwinder.
# Run after building target j1939_freestanding.
set -e

BIN="${1:-build/j1939_freestanding}"
if [ ! -f "$BIN" ]; then
  echo "no binary at $BIN; build target j1939_freestanding first" >&2
  exit 2
fi

FOUND=$(nm -C "$BIN" 2>/dev/null \
  | grep -E ' (U|T|W) ' \
  | grep -E 'operator new|operator delete|\bmalloc\b|\bcalloc\b|\brealloc\b|_Unwind_|__cxa_throw' \
  || true)

if [ -n "$FOUND" ]; then
  echo "FAIL: allocator or unwinder symbols present:" >&2
  echo "$FOUND" >&2
  exit 1
fi

echo "PASS: no allocator or unwinder symbols in $BIN"
