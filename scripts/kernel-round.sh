#!/bin/bash
# One kernel round on a pipa in fastboot: build the kernel, payload and boot image (unless --no-build), `fastboot boot` it, wait
# for the tablet to come back to fastboot by itself (package/BootRamoops.c restarts into fastboot with a warm reset 10 s after a
# panic and pipa-restart-after seconds after boot), fetch the log with the log reader and print the kernel's own lines and the end.
#
# usage: scripts/kernel-round.sh [--no-build | --build-only] [--twice] [--arg NAME=VALUE]... [TAIL_LINES]   (run from anywhere;
#                                                                       UniXNU and uefi-pipa sit beside this repository)
# --build-only stops after the boot image (no tablet needed).
# --arg adds a kernel argument for this round only (it replaces board.toml's argument of the same name), e.g.
# --arg sheng-data-reset=1 to start the Data overlay over.
# --twice builds the kernel twice, for a change of pipa_defconfig or of kernel headers (the boot progress objects compile against
# the previous build's headers). The build reads the local IPSW (UNIXNU_IPSW), never Apple's server; it shares the UniXNU checkout
# with other boards: it waits while another unixnu runs, and puts out/state.json back. The payload carries UniXNU-tb128fu's 8 GiB
# Data seed with Setup marked done and the original static + System trust caches (UniXNU's build/, see UniXNU-tb128fu's README).
set -u
here=$(cd "$(dirname "$0")/.." && pwd)
unixnu=$(cd "$here/../UniXNU" && pwd)
uefi=$(cd "$here/../uefi-pipa" && pwd)
serial=${PIPA_SERIAL:-68b7676d}
build=1
build_only=0
twice=0
extra_args=""
while [ $# -gt 0 ]; do
  case $1 in
    --no-build) build=0; shift ;;
    --build-only) build_only=1; shift ;;
    --twice) twice=1; shift ;;
    --arg) extra_args="$extra_args --kernel-arg $2"; shift 2 ;;
    *) break ;;
  esac
done
lines=${1:-20}
image=$unixnu/out/pipa/boot/boot-kernel-round.img
ipsw=$unixnu/out/ipsw/iPad_Pro_HFR_17.7.11_21H461_Restore.ipsw

if [ $build = 1 ]; then
  [ -f "$ipsw" ] || { echo "the local IPSW $ipsw is missing"; exit 1; }
  while pgrep -f "unixnu (build|bootimg|fetch|defconfig)" > /dev/null; do echo "another unixnu is running, waiting"; sleep 10; done
  state=$(cat "$unixnu/out/state.json" 2>/dev/null)
  log=$unixnu/out/pipa/logs/kernel-round-build.log
  # $extra_args is split on purpose: --kernel-arg NAME=VALUE words without spaces.
  # shellcheck disable=SC2086
  (
    cd "$unixnu" || exit 1
    export UNIXNU_UEFI=$uefi UNIXNU_IPSW=$ipsw
    ./unixnu defconfig pipa_defconfig -D "$here" > "$log" 2>&1 &&
      ./unixnu build kernel -D "$here" >> "$log" 2>&1 &&
      { [ $twice = 0 ] || ./unixnu build kernel -D "$here" >> "$log" 2>&1; } &&
      ./unixnu build payload -D "$here" --unique-chip-id "$(cat "$here/.cache/platform-seed")" $extra_args \
        --data-ramdisk build/owner-data-fixture/owner-data-8g.dbr --skip-setup \
        --trust-cache build/original-static-system-trust/original-static-system.bundle >> "$log" 2>&1 &&
      ./unixnu bootimg -D "$here" --dtb "$uefi/Resources/DTBs/pipa.dtb" -o "$image" >> "$log" 2>&1
  )
  rc=$?
  [ -n "$state" ] && printf '%s\n' "$state" > "$unixnu/out/state.json"
  if [ $rc != 0 ]; then
    grep -E "error|Error|failed" "$log" | head -20
    echo "BUILD FAILED, see $log"
    exit 1
  fi
fi
if [ $build_only = 1 ]; then
  echo "built $image ($(shasum -a 256 "$image" | cut -c1-16))"
  exit 0
fi

fastboot devices | grep -q "^$serial" || { echo "pipa ($serial) is not in fastboot"; exit 2; }
[ "$(timeout 20 fastboot -s "$serial" getvar product 2>&1 | head -1)" = "product: pipa" ] || { echo "$serial is not a pipa"; exit 2; }
echo "image $(shasum -a 256 "$image" | cut -c1-16)"
start=$(date +%s)
fastboot -s "$serial" boot "$image" > /dev/null 2>&1
while fastboot devices | grep -q "^$serial"; do sleep 1; done
# pipa-restart-after (300 s in board.toml) plus the firmware and the kernel's start
for _ in $(seq 1 480); do fastboot devices | grep -q "^$serial" && break; sleep 1; done
if ! fastboot devices | grep -q "^$serial"; then
  echo "NOT back in fastboot after $(( $(date +%s) - start )) s: a hang or a hard reset; hold power + volume down"
  for _ in $(seq 1 1800); do fastboot devices | grep -q "^$serial" && break; sleep 1; done
  fastboot devices | grep -q "^$serial" || exit 3
fi
echo "back in fastboot after $(( $(date +%s) - start )) s"
cd "$unixnu" || exit 1
PIPA_SERIAL=$serial timeout 300 python3 "$here/scripts/read-pipa-log.py" 2>&1 | grep -E "Power-o|saved|no PIPALOG"
# shellcheck disable=SC2012  # the run directories are timestamps
run=$(ls -td "$unixnu"/out/pipa/device-logs/2* | head -1)
console=$run/pstore/console-ramoops-0
[ -f "$console" ] || { echo "no console-ramoops-0 in $run"; exit 4; }
echo "== $console ($(wc -l < "$console") lines)"
grep -a -o -E "\[xnu\] (restart|panic|boot cpu|cpufreq).*|NATIVE_UFS.*" "$console" | cut -c1-300
grep -a -o "\[xnu\] display: .*" "$console" | tail -3
grep -a -o "\[xnu\] cpus: .*" "$console" | tail -1
echo "processes started: $(grep -a -c 'exec process signature accepted' "$console")"
echo "failed exits (count, name, status):"
grep -a "^\[xnu\] exit " "$console" | grep -v "status 0x0 " | awk '{print $4, $6}' | sort | uniq -c | sort -rn | head -8
echo "== end"
tail -n "$lines" "$console" | cut -c1-300
