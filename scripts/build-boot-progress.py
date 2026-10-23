#!/usr/bin/env python3
"""Archive the board's boot progress hooks for the kernel link: kits/pipa/BootRamoops.c, the kernel log in the stock ramoops console
zone (XNU_BOOT_PROGRESS with XNU_BOOT_PROGRESS_STANDALONE), and kits/pipa/BootWatchdogPolicy.c (UniXNU-tb128fu's, from sheng):
sheng_boot_watchdog_policy, which load_init_program calls with the owner root chain (sheng-disable-watchdogd, sheng-disable-jobs).
build-xnu-4k-baseline.sh runs this before the kernel build and force-loads build/boot-progress/libBootProgress.a. BootRamoops.c
declares the few kernel interfaces it calls, so it builds without the kernel's headers; the policy builds with kern_exec.o's
compile record of the previous XNU pass."""
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / 'kits/pipa/BootRamoops.c'
out = ROOT / 'build/boot-progress'
out.mkdir(parents=True, exist_ok=True)
obj, archive = out / 'BootRamoops.o', out / 'libBootProgress.a'
subprocess.run(['xcrun', '--sdk', 'iphoneos', 'clang', '-arch', 'arm64', '-mkernel', '-miphoneos-version-min=17.0',
                '-mgeneral-regs-only', '-Os', '-Wall', '-Wextra', '-Werror', '-c', source, '-o', obj], check=True)

# kern_exec.o's own command with the policy source and object in place of its own, warnings of the record dropped.
policy_source, policy_obj = ROOT / 'kits/pipa/BootWatchdogPolicy.c', out / 'BootWatchdogPolicy.o'
record = json.loads((ROOT / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/bsd/DEVELOPMENT/kern_exec.o.json').read_text().strip().rstrip(','))
command, args, i = [], record['arguments'], 0
while i < len(args):
    arg = args[i]
    if arg == record['file']:
        command.append(str(policy_source))
    elif arg in ('-o', '-MF', '-MT', '-MJ'):
        if arg == '-o':
            command += ['-o', str(policy_obj)]
        i += 1
    elif arg == '-Xclang' and args[i + 1].startswith('-W'):
        i += 1
    elif not arg.startswith('-W'):
        command.append(arg)
    i += 1
command += ['-mgeneral-regs-only', '-Wall', '-Wextra', '-Werror']
subprocess.run(command, cwd=record['directory'], check=True, timeout=90)

archive.unlink(missing_ok=True)
subprocess.run(['xcrun', 'ar', 'rcs', archive, obj, policy_obj], check=True)
print(archive)
