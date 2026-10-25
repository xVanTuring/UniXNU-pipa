#!/usr/bin/env python3
"""Decode the render thread's profile rows (PR*, NativeBridgeMetalFrontend.mm summarizeProfile) of a pipa console log and symbolize them.

  decode-render-profile.py CONSOLE BINDIR [--top N] [SECTION...]

CONSOLE is pstore/console-ramoops-0 of a round (NUL padding is fine). BINDIR holds that round's NativeAdrenoMetal.dSYM and
libvulkan_freedreno.full.dylib (make-gpu-metal.sh keeps them per tag): the tablet's copies are stripped, so the plugin's and Turnip's
leaf samples arrive as 64-byte buckets and are folded into functions here. The shared cache's addresses resolve against its local
symbols (UniXNU's symbolicate-cache-crash.py, the SystemOS volume UNIXNU_SYSTEMOS_CACHE or UniXNU-qemu's mount); the exported ones
come from the tablet's own dladdr (PRn rows). SECTION picks PRs, PRc, PRk, PRw, PRx or fold (the self samples by function).
"""
import importlib.util
import re
import struct
import subprocess
import sys
from pathlib import Path

spec = importlib.util.spec_from_file_location('scc', '/Users/xvan/agents-home/UniXNU/scripts/symbolicate-cache-crash.py')
scc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scc)

import os
CACHE = Path(os.environ.get('UNIXNU_SYSTEMOS_CACHE', '/Users/xvan/agents-home/UniXNU-qemu/.cache/firmware/target-volumes/systemos-mount/System/Library/Caches/com.apple.dyld'))
bindir = Path(sys.argv[2])
LOCAL = {
    'NativeAdrenoMetal': bindir / 'NativeAdrenoMetal.dSYM/Contents/Resources/DWARF/NativeAdrenoMetal',
    'libvulkan_freedreno.dylib': bindir / 'libvulkan_freedreno.full.dylib',
}
top = int(sys.argv[4]) if len(sys.argv) > 4 and sys.argv[3] == '--top' else 40

text = Path(sys.argv[1]).read_bytes().replace(b'\0', b'').decode('utf-8', 'replace')
rows, names, head = {}, {}, []
for line in text.splitlines():
    m = re.search(r'metal table: (PR[a-z]\d*)( .*)$', line)
    if not m:
        continue
    label, body = m.group(1), m.group(2)
    if label == 'PRi':
        if ' run' in body:
            head = [body]
        else:
            head.append(body)
    elif label == 'PRn':
        k = re.match(r' (\w)([0-9a-f]+)=(.*)', body)
        if k:
            names[(k[1], int(k[2], 16))] = k[3]
    else:
        rows[label] = body

images = {}
totals = re.search(r'run(\d+) wait(\d+)', head[0]) if head else None
run, wait = (int(totals.group(1)), int(totals.group(2))) if totals else (0, 0)
for part in head:
    for letter, name, count in re.findall(r' (\w)=(\S+?):(\d+)', part):
        images[letter] = (name, int(count))

cache = None


def cache_tables():
    global cache
    if cache is None:
        imgs = scc.cache_images(CACHE)
        with (CACHE / 'dyld_shared_cache_arm64').open('rb') as stream:
            region = struct.unpack_from('<Q', stream.read(1024), scc.SHARED_REGION_START)[0]
        cache = (imgs, region, scc.cache_symbols(CACHE))
    return cache


atos_cache = {}


def symbolize(letter, offset):
    name = images.get(letter, ('?', 0))[0]
    if (letter, offset) in names:
        return f'{name}: {names[(letter, offset)]} (exported)'
    full = next((k for k in LOCAL if k.startswith(name)), None)
    if full:
        key = (full, offset)
        if key not in atos_cache:
            out = subprocess.run(['atos', '-arch', 'arm64', '-o', str(LOCAL[full]), '-l', '0x0', hex(offset)],
                                 capture_output=True, text=True).stdout.strip()
            out = re.sub(r'_RNv[A-Za-z0-9_]*?metal2vulkan_runtime', '', out).replace(f' (in {full.split(".")[0]})', '')
            atos_cache[key] = out
        return f'{full.split(".")[0][:12]}: {atos_cache[key]}'
    imgs, region, symbols = cache_tables()
    for load, size, path in imgs:
        if path.rsplit('/', 1)[-1].startswith(name):
            unslid = load + offset
            hit = scc.nearest(symbols.get(load - region, []), unslid) if symbols else None
            if hit:
                return f'{name}: {hit[1]}+{unslid - hit[0]:#x}'
            return f'{name}+{offset:#x}'
    return f'{name}+{offset:#x}'


def entries(prefix):
    out = []
    for label in sorted((k for k in rows if re.fullmatch(prefix + r'\d+', k)), key=lambda k: int(k[len(prefix):])):
        for letter, off, count in re.findall(r' (\w)([0-9a-f]+)=(\d+)', rows[label]):
            out.append((int(count), letter, int(off, 16)))
    return out


print(f'running samples {run}, waiting {wait}')
print('images (leaf samples):')
for letter, (name, count) in sorted(images.items(), key=lambda kv: -kv[1][1]):
    if count >= run / 200:
        print(f'  {letter} {name:28} {count:6} {100.0 * count / max(run, 1):5.1f}%')
sections = (('self (function start)', 'PRs', run), ('inclusive (call site)', 'PRc', run),
            ('under system calls and malloc: first plugin/Turnip frame', 'PRk', run),
            ('waits: system call site', 'PRw', wait), ('waits: QuartzCore/plugin frames', 'PRx', wait))
want = sys.argv[5:] if len(sys.argv) > 5 else None
if not want or 'fold' in want:
    folded = {}
    for count, letter, off in entries('PRs'):
        name = symbolize(letter, off)
        name = re.sub(r' \(in [^)]*\)', '', name)
        name = re.sub(r' \([^()]*:\d+\)$', '', name)
        name = re.sub(r' \+ \d+$', '', name)
        name = re.sub(r'\+0x[0-9a-f]+$', '', name)
        folded[name] = folded.get(name, 0) + count
    print('\nself, folded by function:')
    for name, count in sorted(folded.items(), key=lambda kv: -kv[1])[:top]:
        print(f'  {count:6} {100.0 * count / max(run, 1):5.1f}%  {name[:190]}')
for title, prefix, base in sections:
    if want and prefix not in want:
        continue
    print(f'\n{title}:')
    for count, letter, off in entries(prefix)[:top]:
        print(f'  {count:6} {100.0 * count / max(base, 1):5.1f}%  {symbolize(letter, off)[:200]}')
