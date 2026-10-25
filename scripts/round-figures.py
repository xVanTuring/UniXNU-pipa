#!/usr/bin/env python3
"""Per-round figures over the scripted gestures (pipa-gesture), for comparing rounds: the render thread's CPU and wall time a frame (U),
the draws a frame and their cost (A0), the GPU time of a render commit and of the scanout copy (A2), the D and A5 rows after the script,
and every animation step's frames and gap classes summed by step name ("gesture: anim").

  round-figures.py CONSOLE...
"""
import re, sys
for path in sys.argv[1:]:
    text = open(path, 'rb').read().replace(b'\0', b'').decode('utf-8', 'replace').splitlines()
    start = next(i for i, l in enumerate(text) if 'gesture: start' in l)
    done = next((i for i, l in enumerate(text) if 'gesture: done' in l), len(text) - 1)
    def last(pattern, before):
        for i in range(before, -1, -1):
            m = re.search(pattern, text[i])
            if m: return m
    def first(pattern, after):
        for i in range(after, len(text)):
            m = re.search(pattern, text[i])
            if m: return m
    U = r'U n=(\d+) cpu=(\d+)ms wall=(\d+)ms'
    A0 = r'A0 draw (\d+) (\d+)ms/(\d+)us'
    A2 = r'A2 gpu pass n=(\d+) (\d+)ms/\d+us \| copy n=(\d+) (\d+)ms'
    A5 = r'A5 imp(\d+) tex(\d+) enc(\d+) sub(\d+) wait(\d+)'
    D = r'D draw us settle(\d+) cmd(\d+) heap(\d+) rec(\d+)'
    u0, u1 = last(U, start), first(U, done)
    a0, a1 = last(A0, start), first(A0, done)
    g0, g1 = last(A2, start), first(A2, done)
    n = int(u1[1]) - int(u0[1])
    cpu = int(u1[2]) - int(u0[2]); wall = int(u1[3]) - int(u0[3])
    draws = int(a1[1]) - int(a0[1]); drawms = int(a1[2]) - int(a0[2])
    rp = int(g1[1]) - int(g0[1]); rpms = int(g1[2]) - int(g0[2]); cp = int(g1[3]) - int(g0[3]); cpms = int(g1[4]) - int(g0[4])
    print(f'{path.split("/")[-1]}: {n} frames, cpu {cpu/n:.1f} ms/frame, wall {wall/n:.1f}, draws {draws/n:.0f}/frame at {1000*drawms/max(draws,1):.0f} us,'
          f' gpu render {rpms/max(rp,1):.1f} ms x{rp/n:.2f}/frame, copy {cpms/max(cp,1):.2f} ms')
    print('   ', first(D, done)[0], '|', (first(A5, done) or [''])[0])
    anims = [l.split('gesture: ')[1] for l in text[start:done+1] if 'gesture: anim' in l]
    agg = {}
    for a in anims:
        m = re.match(r'anim (.+?): (\d+) frames (\d+) ms .*gaps 17:(\d+) 34:(\d+) 50:(\d+) more:(\d+), max (\d+)', a)
        k = m[1]; v = agg.setdefault(k, [0, 0, 0, 0, 0, 0, 0])
        for j in range(6): v[j] += int(m[j + 2])
        v[6] = max(v[6], int(m[8]))
    for k, v in agg.items():
        print(f'    {k:22} {v[0]:4} frames {1000*v[0]/max(v[1],1):5.1f}/s  <=17 {v[2]} <=34 {v[3]} <=50 {v[4]} more {v[5]} max {v[6]} ms')
