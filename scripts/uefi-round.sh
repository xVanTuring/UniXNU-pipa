#!/bin/bash
# One firmware round on a pipa in fastboot: build the firmware and its menu boot image (unless --no-build), `fastboot boot` it,
# wait for the tablet to come back to fastboot by itself (RamoopsLogLib of uefi-pipa restarts into fastboot with a warm reset
# on an ASSERT), fetch the firmware log with the log reader and print its end.
#
# usage: scripts/uefi-round.sh [--no-build] [TAIL_LINES]      (run from anywhere; UniXNU and uefi-pipa sit beside this repository)
# The build shares the UniXNU checkout with other boards: it waits while another unixnu runs, and puts out/state.json back.
set -u
here=$(cd "$(dirname "$0")/.." && pwd)
unixnu=$(cd "$here/../UniXNU" && pwd)
uefi=$(cd "$here/../uefi-pipa" && pwd)
serial=${PIPA_SERIAL:-68b7676d}
build=1
[ "${1:-}" = --no-build ] && { build=0; shift; }
lines=${1:-60}
image=$unixnu/out/pipa/boot/boot-menu-round.img

if [ $build = 1 ]; then
  while pgrep -f "unixnu (build|bootimg|fetch|defconfig)" > /dev/null; do echo "another unixnu is running, waiting"; sleep 10; done
  state=$(cat "$unixnu/out/state.json" 2>/dev/null)
  log=$unixnu/out/pipa/logs/uefi-round-build.log
  (cd "$unixnu" && UNIXNU_UEFI=$uefi ./unixnu build firmware -D "$here" > "$log" 2>&1 &&
    UNIXNU_UEFI=$uefi ./unixnu bootimg -D "$here" --dtb "$uefi/Resources/DTBs/pipa.dtb" --ramdisk none -o "$image" >> "$log" 2>&1)
  rc=$?
  [ -n "$state" ] && printf '%s\n' "$state" > "$unixnu/out/state.json"
  if [ $rc != 0 ]; then
    grep -E "error|Error" "$log" | grep -v PcdDebug | head -20
    echo "BUILD FAILED, see $log"
    exit 1
  fi
fi

fastboot devices | grep -q "^$serial" || { echo "pipa ($serial) is not in fastboot"; exit 2; }
[ "$(timeout 20 fastboot -s "$serial" getvar product 2>&1 | head -1)" = "product: pipa" ] || { echo "$serial is not a pipa"; exit 2; }
echo "image $(shasum -a 256 "$image" | cut -c1-16)"
start=$(date +%s)
fastboot -s "$serial" boot "$image" > /dev/null 2>&1
while fastboot devices | grep -q "^$serial"; do sleep 1; done
for _ in $(seq 1 180); do fastboot devices | grep -q "^$serial" && break; sleep 1; done
if ! fastboot devices | grep -q "^$serial"; then
  echo "NOT back in fastboot after $(( $(date +%s) - start )) s: a hang or a hard reset; hold power + volume down"
  exit 3
fi
echo "back in fastboot after $(( $(date +%s) - start )) s"
cd "$unixnu" || exit 1
PIPA_SERIAL=$serial timeout 300 python3 "$here/scripts/read-pipa-log.py" 2>&1 | grep -E "Power-o|saved|no PIPALOG"
run=$(ls -td "$unixnu"/out/pipa/device-logs/2* | head -1)
console=$run/pstore/console-ramoops-0
[ -f "$console" ] || { echo "no console-ramoops-0 in $run"; exit 4; }
echo "== $console ($(wc -l < "$console") lines)"
tail -n "$lines" "$console"
