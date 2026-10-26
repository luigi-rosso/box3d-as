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
PARALLEL_FIELDS = ['parContacts', 'parDeferredContacts', 'parDeferredBodies', 'parts']
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


def stage_table(stage, impls_shown):
    print()
    print(f'{stage}, ms per step: the mean over the run, then the fastest step, each min over rounds; x serial on the mean.')
    print()
    print('| scene | impl | ' + ' | '.join(f'{n} lanes' for n in counts) + ' |')
    print('|---|---|' + '---|' * len(counts))
    for scene in scenes:
        serial_runs = runs.get((scene, 'AS serial', 0), [])
        serial = min((float(r[stage]) for r in serial_runs if stage in r), default=None)
        if serial_runs:
            fastest = min(float(r['min_' + stage]) for r in serial_runs)
            print(f'| {scene} | AS serial | **{serial:.3f}** / {fastest:.3f} |' + ' |' * (len(counts) - 1))
        for impl in impls:
            if impl == 'AS serial' or not any(k in impl for k in impls_shown):
                continue
            cells = []
            for n in counts:
                rs = [r for r in runs.get((scene, impl, n), []) if stage in r]
                if not rs:
                    cells.append('')
                    continue
                mean = min(float(r[stage]) for r in rs)
                fastest = min(float(r.get('min_' + stage, r[stage])) for r in rs)
                ratio = f' (x{serial / mean:.2f})' if serial and impl.startswith('AS') else ''
                cells.append(f'{mean:.3f} / {fastest:.3f}{ratio}' if impl.startswith('AS') else f'{mean:.3f}')
            print(f'| {scene} | {impl} | ' + ' | '.join(cells) + ' |')


stage_table('constraints', ['native', 'solve', 'all', 'parallel'])
stage_table('collide', ['native', 'all', 'parallel'])
stage_table('pairs', ['native', 'parallel'])
stage_table('transforms', ['native', 'parallel'])

# Where a step's time goes: the narrow phase, the solver stages and the rest, which on parallel calls is what the main
# lane runs alone between the calls (pair finding, body finalization and the step's own serial parts included in it)
print()
print('Step breakdown of the best run, ms per step: collide, constraints, and the rest of the step.')
print()
print('| scene | impl | ' + ' | '.join(f'{n}' for n in counts) + ' |')
print('|---|---|' + '---|' * len(counts))
for scene in scenes:
    for impl in impls:
        if not (impl == 'native simd' or 'parallel' in impl or impl == 'AS serial'):
            continue
        cells = []
        for n in counts:
            b = best(scene, impl, 0 if impl == 'AS serial' else n)
            if not b or (impl == 'AS serial' and n != counts[0]):
                cells.append('')
                continue
            step = float(b.get('step', b.get('mean')))
            collide = float(b['collide'])
            constraints = float(b['constraints'])
            cells.append(f'{step:.2f} = {collide:.2f} + {constraints:.2f} + {step - collide - constraints:.2f}')
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

print()
print('Parallel call counts of the best run, per step: awake contacts, those the parts left to the main lane, fast bodies swept on the main lane, and Parallel.size().')
print()
print('| scene | impl | n | ' + ' | '.join(PARALLEL_FIELDS) + ' |')
print('|---|---|---|' + '---|' * len(PARALLEL_FIELDS))
for scene in scenes:
    for impl in impls:
        if 'parallel' not in impl:
            continue
        for n in counts:
            b = best(scene, impl, n)
            if not b:
                continue
            print(f'| {scene} | {impl} | {n} | ' + ' | '.join(b.get(f, '') for f in PARALLEL_FIELDS) + ' |')

if bad:
    print()
    print('Problems:')
    for line in bad:
        print('- ' + line)
