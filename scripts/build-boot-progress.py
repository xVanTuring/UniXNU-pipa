#!/usr/bin/env python3
"""Archive the board's boot progress hooks for the kernel link: kits/pipa/BootRamoops.c, the kernel log in the stock ramoops console
zone (XNU_BOOT_PROGRESS with XNU_BOOT_PROGRESS_STANDALONE). build-xnu-4k-baseline.sh runs this before the kernel build and
force-loads build/boot-progress/libBootProgress.a. The source declares the few kernel interfaces it calls, so it builds without
the kernel's headers."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / 'kits/pipa/BootRamoops.c'
out = ROOT / 'build/boot-progress'
out.mkdir(parents=True, exist_ok=True)
obj, archive = out / 'BootRamoops.o', out / 'libBootProgress.a'
subprocess.run(['xcrun', '--sdk', 'iphoneos', 'clang', '-arch', 'arm64', '-mkernel', '-miphoneos-version-min=17.0',
                '-mgeneral-regs-only', '-Os', '-Wall', '-Wextra', '-Werror', '-c', source, '-o', obj], check=True)
archive.unlink(missing_ok=True)
subprocess.run(['xcrun', 'ar', 'rcs', archive, obj], check=True)
print(archive)
