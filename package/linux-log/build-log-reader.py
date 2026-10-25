#!/usr/bin/env python3
"""Build the fastboot-boot-only pipa log reader: the tablet's own kernel (from its header v3 boot image) and device trees (from
the header v3 vendor_boot image) in a header v2 image, so the reader does not depend on the vendor_boot partition of the active
slot; the ramdisk is an initramfs holding init (init.c next to this file, static and libc-free), a FAT image with LOG.BIN (8 MiB)
and STATUS.TXT, and /log.offsets, where those two files' data lie inside the image. The reader copies what the stock kernel's
ramoops area (0xb0000000) kept from the previous system, offers the image to the host as a read-only USB disk and restarts into
fastboot when the host ejects it (scripts/read-pipa-log.py).

Every device tree that has /soc/restart@c264000 gets qcom,force-warm-reboot unless --cold is given: the stock kernel then
restarts with a PMIC warm reset (msm-poweroff), which keeps the memory, instead of the hard reset it uses with the PON's
qcom,store-hard-reset-reason. The kernel command line is the vendor_boot one.

usage: build-log-reader.py --boot boot_b.img --vendor-boot vendor_boot_b.img --out reader.img [--cold]"""
import argparse
import gzip
import hashlib
import os
import shutil
import struct
import subprocess
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
LLVM = Path('/opt/homebrew/opt/llvm/bin')
LOG_BYTES = 8 * 1024 * 1024
STATUS_BYTES = 4096
IMAGE_SECTORS = 10 * 2048          # 10 MiB: LOG.BIN, STATUS.TXT and the FAT16 metadata
VOLUME = 'PIPALOG'                 # not the TB128FU reader's UNIXNULOG: both tablets can be read on the same Mac at once
PAGE = 4096                        # header v3 images have a fixed 4096-byte page; the output keeps it
RESTART = '/soc/restart@c264000'
WARM = 'qcom,force-warm-reboot'


def tool(name):
    # LLVM's own clang first: Xcode's links only Mach-O, and the Linux target needs ld.lld.
    found = (LLVM / name if (LLVM / name).exists() else None) or shutil.which(name)
    if not found:
        raise SystemExit(f'{name} not found')
    return str(found)


def build_init(work, power_off=False):
    out = work / 'init'
    subprocess.run([tool('clang'), '--target=aarch64-linux-gnu', '-O2', '-ffreestanding', '-fno-builtin', '-fno-stack-protector',
                    '-fno-pic', '-static', '-nostdlib', '-fuse-ld=lld', '-Wl,--no-pie', '-Wl,-e,start', '-Wall', '-Werror',
                    *(['-DREADER_POWER_OFF'] if power_off else []), '-o', out, HERE / 'init.c'], check=True)
    header = subprocess.run([tool('llvm-readelf'), '-h', out], capture_output=True, text=True, check=True).stdout
    if 'EXEC (Executable file)' not in header or 'AArch64' not in header:
        raise SystemExit('init is not a static AArch64 executable')
    if subprocess.run([tool('llvm-readelf'), '-d', out], capture_output=True, text=True).stdout.strip() not in ('', 'There is no dynamic section in this file.'):
        raise SystemExit('init has a dynamic section')
    return out.read_bytes()


def contiguous_file(image, name83):
    """Byte offset and size of a root-directory file of a FAT16 image whose clusters are contiguous."""
    bps, spc, reserved, fats, root_entries, _, _, fat_sectors = struct.unpack_from('<HBHBHHBH', image, 11)
    if image[54:62] != b'FAT16   ':
        raise SystemExit('expected a FAT16 image')
    fat = reserved * bps
    root = fat + fats * fat_sectors * bps
    data = root + root_entries * 32
    cluster_bytes = spc * bps
    for at in range(root, data, 32):
        if image[at:at + 11] == name83:
            first, size = struct.unpack_from('<HI', image, at + 26)
            cluster, count = first, 0
            while cluster < 0xFFF8:
                nxt = struct.unpack_from('<H', image, fat + 2 * cluster)[0]
                count += 1
                if nxt < 0xFFF8 and nxt != cluster + 1:
                    raise SystemExit(f'{name83!r} is fragmented')
                cluster = nxt
            if count * cluster_bytes < size:
                raise SystemExit(f'{name83!r} chain is short')
            return data + (first - 2) * cluster_bytes, size
    raise SystemExit(f'{name83!r} not in the root directory')


def build_image(work):
    image = work / 'log.img'
    env = dict(os.environ, MTOOLS_SKIP_CHECK='1')
    image.write_bytes(b'\0' * IMAGE_SECTORS * 512)
    # 1 KiB clusters: with the TB128FU reader's 4 KiB ones, 10 MiB has fewer than 4085 clusters and mformat picks FAT12.
    subprocess.run(['mformat', '-i', image, '-T', str(IMAGE_SECTORS), '-h', '64', '-s', '32', '-c', '2', '-v', VOLUME, '::'],
                   env=env, check=True)
    (work / 'LOG.BIN').write_bytes(b'\0' * LOG_BYTES)
    (work / 'STATUS.TXT').write_bytes(b' ' * (STATUS_BYTES - 1) + b'\n')
    for name in ('LOG.BIN', 'STATUS.TXT'):
        subprocess.run(['mcopy', '-i', image, work / name, '::' + name], env=env, check=True)
    data = image.read_bytes()
    log_off, log_size = contiguous_file(data, b'LOG     BIN')
    status_off, status_size = contiguous_file(data, b'STATUS  TXT')
    if log_size != LOG_BYTES or status_size != STATUS_BYTES:
        raise SystemExit('unexpected file sizes in the FAT image')
    return data, f'{log_off} {log_size} {status_off} {status_size}\n'.encode()


def cpio(entries):
    """newc archive; entries are (path, mode, rdev major, rdev minor, data) with directories first."""
    out = bytearray()
    for ino, (name, mode, major, minor, data) in enumerate(entries + [('TRAILER!!!', 0, 0, 0, b'')], 1):
        path = name.encode() + b'\0'
        fields = (ino, mode, 0, 0, 1, 0, len(data), 0, 0, major, minor, len(path), 0)
        out += b'070701' + b''.join(b'%08x' % v for v in fields) + path
        out += b'\0' * (-len(out) % 4) + data
        out += b'\0' * (-len(out) % 4)
    return bytes(out)


def pages(n):
    return (n + PAGE - 1) // PAGE * PAGE


def read_boot_v3(path):
    """Kernel and os_version of a header v3 boot image."""
    boot = path.read_bytes()
    if boot[:8] != b'ANDROID!':
        raise SystemExit(f'{path}: not an Android boot image')
    kernel_size, ramdisk_size, os_version, header_size = struct.unpack_from('<4I', boot, 8)
    version, = struct.unpack_from('<I', boot, 40)
    if version != 3:
        raise SystemExit(f'{path}: expected header v3, found v{version}')
    kernel = boot[PAGE:PAGE + kernel_size]
    if len(kernel) != kernel_size or kernel[0x38:0x3c] != b'ARMd':
        raise SystemExit(f'{path}: truncated, or the kernel is not an arm64 Image')
    return kernel, os_version


def read_vendor_boot_v3(path):
    """Command line, load addresses and the device tree section of a header v3 vendor_boot image."""
    vb = path.read_bytes()
    if vb[:8] != b'VNDRBOOT':
        raise SystemExit(f'{path}: not a vendor_boot image')
    version, page, kernel_addr, ramdisk_addr, ramdisk_size = struct.unpack_from('<5I', vb, 8)
    if version != 3 or page != PAGE:
        raise SystemExit(f'{path}: expected header v3 with 4096-byte pages')
    cmdline = vb[28:28 + 2048].split(b'\0')[0]
    tags_addr, = struct.unpack_from('<I', vb, 2076)
    header_size, dtb_size = struct.unpack_from('<II', vb, 2096)
    dtb_addr, = struct.unpack_from('<Q', vb, 2104)
    dtb_at = pages(header_size) + pages(ramdisk_size)
    dtb = vb[dtb_at:dtb_at + dtb_size]
    if len(dtb) != dtb_size or dtb[:4] != b'\xd0\x0d\xfe\xed':
        raise SystemExit(f'{path}: truncated, or no device tree section')
    return cmdline, kernel_addr, ramdisk_addr, tags_addr, dtb_addr, dtb


def split_dtbs(blob):
    out, at = [], 0
    while at < len(blob):
        if blob[at:at + 4] != b'\xd0\x0d\xfe\xed':
            raise SystemExit(f'device tree section: no FDT header at {at:#x}')
        size, = struct.unpack_from('>I', blob, at + 4)
        out.append(blob[at:at + size])
        at += size
    return out


def warm_dtbs(dtbs, work):
    """The device trees with qcom,force-warm-reboot on the restart node; trees without that node (the RTIC blob) stay as they are."""
    out, changed = [], 0
    for i, dtb in enumerate(dtbs):
        path = work / f'{i}.dtb'
        path.write_bytes(dtb)
        if subprocess.run([tool('fdtget'), '-t', 's', path, RESTART, 'compatible'], capture_output=True, text=True).stdout.strip() != 'qcom,pshold':
            out.append(dtb)
            continue
        subprocess.run([tool('fdtput'), '-t', 'x', path, RESTART, WARM], check=True)
        out.append(path.read_bytes())
        changed += 1
    if not changed:
        raise SystemExit(f'no device tree has {RESTART}')
    return out, changed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--boot', required=True, type=Path, help="the tablet's own boot image (header v3)")
    ap.add_argument('--vendor-boot', required=True, type=Path, help="the tablet's own vendor_boot image (header v3)")
    ap.add_argument('--out', required=True, type=Path)
    ap.add_argument('--cold', action='store_true', help='keep the stock hard reset when the reader restarts')
    ap.add_argument('--poweroff', action='store_true', help='an init that powers the tablet off at once (nothing is read)')
    a = ap.parse_args()
    kernel, os_version = read_boot_v3(a.boot)
    cmdline, kernel_addr, ramdisk_addr, tags_addr, dtb_addr, dtb_section = read_vendor_boot_v3(a.vendor_boot)
    if len(cmdline) > 512 + 1024 - 1:
        raise SystemExit('the vendor_boot command line does not fit a header v2 image')

    with tempfile.TemporaryDirectory() as tmp:
        work = Path(tmp)
        dtbs = split_dtbs(dtb_section)
        changed = 0
        if not a.cold:
            dtbs, changed = warm_dtbs(dtbs, work)
        dtb = b''.join(dtbs)
        init = build_init(work, a.poweroff)
        image, offsets = build_image(work)
    entries = [(d, 0o40755, 0, 0, b'') for d in ('dev', 'sys', 'proc')]
    entries += [('dev/console', 0o20600, 5, 1, b''), ('init', 0o100755, 0, 0, init),
                ('log.img', 0o100644, 0, 0, image), ('log.offsets', 0o100644, 0, 0, offsets)]
    ramdisk = gzip.compress(cpio(entries), 9, mtime=0)

    # mkbootimg's header v2: the v1 fields, then dtb_size and dtb_addr (1660 bytes).
    header = bytearray(PAGE)
    second_addr = 0xf00000
    struct.pack_into('<8s10I', header, 0, b'ANDROID!', len(kernel), kernel_addr, len(ramdisk), ramdisk_addr, 0, second_addr,
                     tags_addr, PAGE, 2, os_version)
    header[64:64 + 512] = cmdline[:511].ljust(512, b'\0')
    header[608:608 + 1024] = cmdline[511:].ljust(1024, b'\0')
    struct.pack_into('<IQIIQ', header, 1632, 0, 0, 1660, len(dtb), dtb_addr)
    digest = hashlib.sha1()
    for blob in (kernel, ramdisk, b'', b'', dtb):      # kernel, ramdisk, second, recovery dtbo, dtb (mkbootimg's id for v2)
        digest.update(blob)
        digest.update(struct.pack('<I', len(blob)))
    header[576:608] = digest.digest().ljust(32, b'\0')
    pad = lambda b: b + b'\0' * (-len(b) % PAGE)
    out = bytes(header) + pad(kernel) + pad(ramdisk) + pad(dtb)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_bytes(out)
    print(f'BUILT {a.out} ({len(out)} bytes) sha256 {hashlib.sha256(out).hexdigest()} ramdisk {len(ramdisk)} init {len(init)} '
          f'dtbs {len(dtbs)} warm {changed} offsets {offsets.decode().strip()}')


if __name__ == '__main__':
    main()
