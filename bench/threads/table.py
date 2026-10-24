# SPDX-FileCopyrightText: 2026 Erin Catto
# SPDX-License-Identifier: MIT
# Turns a bench/threads/run.sh session log into markdown tables. Native ms/step is the mean over a repeat's timed
# steps, min over repeats; AnimaScript's is the mean over the run's timed steps. Both take the min over rounds, and
# the stage tables show the round that gave it.
import re
import sys
from collections import defaultdict

STAGES = ['pairs', 'collide', 'solve', 'constraints', 'prepareConstraints', 'warmStart', 'solveImpulses',
          'relaxImpulses', 'storeImpulses', 'transforms']
LANE_FIELDS = ['lanePack', 'laneBlob', 'lanePartition0', 'laneWait', 'laneApply', 'part_exchange',
               'part_firstWait', 'laneMBin', 'laneMBout', 'laneShared', 'laneExchanges', 'collidePack',
               'collidePartition0', 'collideWait', 'collideApply', 'collideMBin', 'collideMBout']

load = None
config = None
lanes = None
runs = defaultdict(list)
bad = []
for line in open(sys.argv[1]):
    m = re.search(r'load averages?: ([\d.]+)', line)
    if line.startswith('UPTIME') and m:
        load = float(m.group(1))
    elif line.startswith('AS '):
        kv = dict(p.split('=', 1) for p in line.split()[1:])
        config, lanes = kv['config'], int(kv['lanes'])
    elif line.startswith('RESULT'):
        kv = dict(p.split('=', 1) for p in line.split()[1:])
        kv['ms'] = float(kv['min'])
        kv['load'] = load
        runs[(kv['scene'], 'native simd', int(kv['workers']))].append(kv)
    elif 'BENCH RESULT' in line:
        kv = dict(p.split('=', 1) for p in line.split('BENCH RESULT ')[1].split())
        kv['ms'] = float(kv['mean'])
        kv['load'] = load
        runs[(kv['scene'], 'AS ' + config, lanes)].append(kv)
    elif 'trapped' in line or 'failed' in line or 'gave up' in line:
        bad.append(line.strip())

scenes, impls, counts = [], [], []
for scene, impl, n in runs:
    if scene not in scenes:
        scenes.append(scene)
    if impl not in impls:
        impls.append(impl)
    if n not in counts and n > 0:
        counts.append(n)
counts.sort()


def best(scene, impl, n):
    rs = runs.get((scene, impl, n))
    return min(rs, key=lambda r: r['ms']) if rs else None


print('ms per step, min over rounds (1 minute load of that run). AS serial is the port without lanes; x serial divides it by the cell.')
print()
print('| scene | impl | ' + ' | '.join(f'{n} lanes' for n in counts) + ' |')
print('|---|---|' + '---|' * len(counts))
for scene in scenes:
    serial = best(scene, 'AS serial', 0)
    if serial:
        print(f"| {scene} | AS serial | **{serial['ms']:.3f}** ({serial['load']:.0f}) |" + ' |' * (len(counts) - 1))
    for impl in impls:
        if impl == 'AS serial':
            continue
        cells = []
        for n in counts:
            b = best(scene, impl, n)
            if not b:
                cells.append('')
                continue
            ratio = f", x{serial['ms'] / b['ms']:.2f}" if serial and impl.startswith('AS') else ''
            cells.append(f"{b['ms']:.3f} ({b['load']:.0f}{ratio})")
        print(f'| {scene} | {impl} | ' + ' | '.join(cells) + ' |')

print()
print('Stages of the best run, ms per step:')
print()
print('| scene | impl | lanes | step | ' + ' | '.join(STAGES) + ' |')
print('|---|---|---|---|' + '---|' * len(STAGES))
for scene in scenes:
    for impl in impls:
        for n in ([0] if impl == 'AS serial' else counts):
            b = best(scene, impl, n)
            if not b:
                continue
            step = b.get('step', b.get('mean', ''))
            cells = [f"{float(b[s]):.3f}" if s in b else '' for s in STAGES]
            print(f'| {scene} | {impl} | {n} | {float(step):.3f} | ' + ' | '.join(cells) + ' |')

print()
print('Lane costs of the best run, ms per step (MB per step for the byte columns):')
print()
print('| scene | impl | lanes | ' + ' | '.join(LANE_FIELDS) + ' |')
print('|---|---|---|' + '---|' * len(LANE_FIELDS))
for scene in scenes:
    for impl in impls:
        if not impl.startswith('AS') or impl == 'AS serial':
            continue
        for n in counts:
            b = best(scene, impl, n)
            if not b:
                continue
            cells = [b.get(f, '') for f in LANE_FIELDS]
            print(f'| {scene} | {impl} | {n} | ' + ' | '.join(cells) + ' |')

if bad:
    print()
    print('Problems:')
    for line in bad:
        print('- ' + line)
