#!/usr/bin/env python3
"""The swap timelines (`swaps:` lines, Gestures.c anim_timeline) of a pipa console log: per swap the gap since the one before, how long
it waited for its offer to the GPU copy and when the copy finished.

  swap-timeline.py CONSOLE
"""
import re
import sys

text = open(sys.argv[1], 'rb').read().replace(b'\0', b'').decode('utf-8', 'replace').splitlines()
blocks, current = [], []
for line in text:
    m = re.search(r'swaps: W(\d+) t=([\d.]+)s \+(\d+)ms offer=(-?\d+)ms done=(-?\d+)ms', line)
    if m:
        current.append(tuple(int(m[i]) if i != 2 else float(m[i]) for i in range(1, 6)))
    elif current:
        blocks.append(current)
        current = []
if current:
    blocks.append(current)
for block in blocks:
    print(f'== swaps {block[0][0]}..{block[-1][0]} at {block[0][1]:.3f}s')
    print('   gap ms:   ', ' '.join(f'{b[2]:3d}' for b in block))
    print('   offer ms: ', ' '.join(f'{b[3]:3d}' for b in block))
    print('   done ms:  ', ' '.join(f'{b[4]:3d}' for b in block))
