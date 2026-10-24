#!/usr/bin/env python3
"""Fetch what a pipa in fastboot kept in its ramoops area: boot the log reader (package/linux-log/build-log-reader.py) with
`fastboot boot`, wait for its read-only USB disk (volume PIPALOG), copy LOG.BIN and STATUS.TXT, eject the disk (the reader then
restarts into fastboot) and wait for fastboot again. The TB128FU reader's disk is UNIXNULOG; a disk whose STATUS.TXT is not the pipa
reader's is left alone.

usage: read-pipa-log.py [--reader reader.img] [--out DIR] [--no-boot] [--wait 120]
Output: DIR/<time>/LOG.BIN and STATUS.TXT (what the reader did), and LOG.BIN's records as files: pstore/console-ramoops-0 and
pstore/pmsg-ramoops-0 (what the previous system left, when the memory survived), dmesg and cmdline (the reader's own kernel)."""
import argparse
import glob
import hashlib
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

UNIXNU = Path(__file__).resolve().parents[2] / 'UniXNU'
# The tablet's fastboot serial (PIPA_SERIAL); without it, the only device in fastboot counts, and two or more are refused.
SERIAL = os.environ.get('PIPA_SERIAL', '')


def fastboot_serials():
    out = subprocess.run(['fastboot', 'devices'], capture_output=True, text=True).stdout
    return [line.split()[0] for line in out.splitlines() if line.strip().endswith('fastboot')]


def in_fastboot():
    serials = fastboot_serials()
    return SERIAL in serials if SERIAL else len(serials) == 1


def fastboot(*args):
    return ['fastboot'] + (['-s', SERIAL] if SERIAL else []) + list(args)


def find_volume():
    for volume in sorted(glob.glob('/Volumes/PIPALOG*')):
        status = os.path.join(volume, 'STATUS.TXT')
        if not (os.path.isfile(os.path.join(volume, 'LOG.BIN')) and os.path.isfile(status)):
            continue
        with open(status, 'rb') as f:
            if f.read(16) == b'pipa log reader\n':
                return volume
    return None


def unpack(blob, out):
    """LOG.BIN (package/linux-log/init.c): 64-byte records ("UXFILE01", signed size or -errno, name), data padded to 8."""
    at = 0
    while at + 64 <= len(blob) and blob[at:at + 8] == b'UXFILE01':
        size = int.from_bytes(blob[at + 8:at + 16], 'little', signed=True)
        name = blob[at + 16:at + 64].split(b'\0')[0].decode()
        if '..' in name.split('/') or name.startswith('/'):
            sys.exit(f'LOG.BIN: bad record name {name!r}')
        if size < 0:
            print(f'{name}: error {-size}')
            at += 64
            continue
        data = blob[at + 64:at + 64 + size]
        target = out / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        print(f'{name}: {len(data)} bytes, sha256 {hashlib.sha256(data).hexdigest()[:16]}')
        at += 64 + (size + 7) // 8 * 8
    if blob[at:at + 8] != b'UXEND000':
        print('LOG.BIN: no end record (the reader stopped early)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--reader', type=Path, default=UNIXNU / 'out/pipa/log-reader/reader.img')
    ap.add_argument('--out', type=Path, default=UNIXNU / 'out/pipa/device-logs')
    ap.add_argument('--no-boot', action='store_true', help='the reader is already running')
    ap.add_argument('--wait', type=int, default=120, help='seconds to wait for the disk')
    a = ap.parse_args()
    if find_volume():
        sys.exit('a PIPALOG volume is already mounted; eject it first')
    if not a.no_boot:
        if not SERIAL and len(fastboot_serials()) > 1:
            sys.exit('more than one device in fastboot; set PIPA_SERIAL')
        if not in_fastboot():
            sys.exit('the tablet is not in fastboot')
        print(' '.join(fastboot('boot', str(a.reader))))
        subprocess.run(fastboot('boot', str(a.reader)), check=True)
    start = time.time()
    volume = None
    while time.time() - start < a.wait:
        volume = find_volume()
        if volume:
            break
        time.sleep(1)
    if not volume:
        sys.exit(f'no PIPALOG disk within {a.wait} s (the reader returns to fastboot by itself within about 2 minutes)')
    print(f'disk {volume} after {time.time() - start:.0f} s')
    run = a.out / time.strftime('%Y%m%d-%H%M%S')
    run.mkdir(parents=True)
    for name in ('STATUS.TXT', 'LOG.BIN'):
        shutil.copyfile(os.path.join(volume, name), run / name)
    print((run / 'STATUS.TXT').read_text(errors='replace').rstrip())
    unpack((run / 'LOG.BIN').read_bytes(), run)
    subprocess.run(['diskutil', 'eject', volume], check=True)
    start = time.time()
    while time.time() - start < 120 and not in_fastboot():
        time.sleep(1)
    print(f'saved {run}; ' + ('back in fastboot' if in_fastboot() else 'not back in fastboot within 120 s'))


if __name__ == '__main__':
    main()
