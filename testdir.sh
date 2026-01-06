#!/bin/bash

set -u

TEST_SCRIPT="./testn.sh"
ROOT_DIR="${1:-.}"

MODES=(4 5 8)

if [[ ! -x "$TEST_SCRIPT" ]]; then
  echo "Error: $TEST_SCRIPT not executable"
  exit 1
fi

echo "DIR: $ROOT_DIR"
echo "MODES: ${MODES[*]}"

# Use -print0 to avoid exploding on spaces, tabs, or human creativity
find "$ROOT_DIR" -size -512k -type f \( -iname "*.ppm" \) -print0 |
while IFS= read -r -d '' PPM; do
  STUB=$(dirname "$PPM")/$(basename "$PPM" .ppm)
  for MODE in "${MODES[@]}"; do
    if ! "$TEST_SCRIPT" "$MODE" "$STUB"; then
      echo
      echo "FAILURE detected"
      echo "File: $PPM"
      echo "Mode: $MODE"
      exit 1
    fi
  done
done

exit 0
