#!/usr/bin/env bash
# Print the per-suite and total test/assertion counts.
# Every test-count figure quoted in the documentation comes from this script.
set -euo pipefail

cd "$(dirname "$0")"

if [ ! -d build ]; then
  echo "no build directory - run ./run_tests.sh first" >&2
  exit 1
fi

cd build

total_tests=0
total_assertions=0
total_failures=0

printf '%-26s %8s %12s %10s\n' "SUITE" "TESTS" "ASSERTIONS" "FAILURES"
printf '%s\n' "------------------------------------------------------------"

for binary in test_*; do
  [ -x "$binary" ] || continue
  [ -f "$binary" ] || continue

  summary=$("./$binary" | grep -E '^[0-9]+ assertion')
  assertions=$(echo "$summary" | awk '{print $1}')
  failures=$(echo "$summary" | awk '{print $3}')
  tests=$(echo "$summary" | awk '{print $6}')

  printf '%-26s %8s %12s %10s\n' "$binary" "$tests" "$assertions" "$failures"

  total_tests=$((total_tests + tests))
  total_assertions=$((total_assertions + assertions))
  total_failures=$((total_failures + failures))
done

printf '%s\n' "------------------------------------------------------------"
printf '%-26s %8s %12s %10s\n' "TOTAL" "$total_tests" "$total_assertions" "$total_failures"
