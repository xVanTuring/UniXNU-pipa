#!/bin/bash
# Run a pipa kernel round and retry when the compositor never came up (the flaky
# SpringBoard stall: swaps stay in single digits, gestures deliver but nothing draws).
set -u
attempts=${ATTEMPTS:-3}
min_swaps=${MIN_SWAPS:-30}
shift_args="$*"
for attempt in $(seq 1 "$attempts"); do
  echo "=== attempt $attempt/$attempts ==="
  cd /Users/xvan/agents-home/UniXNU || exit 1
  before=$(ls -1t out/pipa/device-logs | head -1)
  caffeinate -d env PIPA_GPU_USERSPACE=/Users/xvan/agents-home/UniXNU/out/pipa/gpu-metal \
    bash -c "cd ../UniXNU-pipa && scripts/kernel-round.sh $shift_args" 2>&1 | tail -4
  after=$(ls -1t out/pipa/device-logs | head -1)
  [ "$after" = "$before" ] && { echo "no new log dir"; continue; }
  swaps=$(tr -d '\000' < "out/pipa/device-logs/$after/pstore/console-ramoops-0" 2>/dev/null |
    grep -a "display: stage 10" | tail -1 | sed -E 's/.*swaps ([0-9]+).*/\1/')
  swaps=${swaps:-0}
  echo "log=$after swaps=$swaps (need >= $min_swaps)"
  if [ "$swaps" -ge "$min_swaps" ]; then echo "OK: compositor came up"; exit 0; fi
done
echo "GAVE UP: compositor never came up in $attempts attempts"
exit 1
