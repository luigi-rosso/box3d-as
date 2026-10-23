# SPDX-FileCopyrightText: 2026 Erin Catto
# SPDX-License-Identifier: MIT
# Turns a bench/as/run.sh session log into markdown tables. Native ms/step is the mean over a repeat's
# timed steps, min over repeats; AnimaScript's is the mean over the run's timed steps. Both then take the
# min over rounds; stages are means per step, min over rounds.
import re
import sys
from collections import defaultdict

STAGES = ['step', 'pairs', 'collide', 'solve', 'solverSetup', 'constraints', 'prepareConstraints',
          'integrateVelocities', 'warmStart', 'solveImpulses', 'integratePositions', 'relaxImpulses',
          'storeImpulses', 'transforms', 'refit']
ORDER = ['native simd', 'native scalar', 'AS aot', 'AS interp']

load = None
build = None
tier = None
runs = defaultdict(list)
for line in open(sys.argv[1]):
    m = re.search(r'load averages?: ([\d.]+)', line)
    if line.startswith('UPTIME') and m:
        load = float(m.group(1))
    elif line.startswith('BUILD'):
        build = line.split()[1]
    elif line.startswith('AS '):
        tier = line.split()[1].split('=')[1]
    elif line.startswith('RESULT'):
        kv = dict(p.split('=', 1) for p in line.split()[1:])
        kv['ms'] = float(kv['min'])
        kv['load'] = load
        runs[(kv['scene'], 'native ' + build)].append(kv)
    elif 'BENCH RESULT' in line:
        kv = dict(p.split('=', 1) for p in line.split('BENCH RESULT ')[1].split())
        kv['ms'] = float(kv['mean'])
        kv['load'] = load
        runs[(kv['scene'], 'AS ' + tier)].append(kv)

scenes = []
for scene, _ in runs:
    if scene not in scenes:
        scenes.append(scene)

print('| scene | impl | best ms/step | x scalar | per round ms/step (load) | AS step min / median of best run |')
print('|---|---|---|---|---|---|')
for scene in scenes:
    scalar = min((r['ms'] for r in runs.get((scene, 'native scalar'), [])), default=None)
    for impl in ORDER:
        rs = runs.get((scene, impl))
        if not rs:
            continue
        best = min(rs, key=lambda r: r['ms'])
        ratio = f"{best['ms'] / scalar:.1f}" if scalar else ''
        rounds = ', '.join(f"{r['ms']:.2f} ({r['load']:.1f})" for r in rs)
        extra = f"{float(best['min']):.2f} / {float(best['median']):.2f}" if impl.startswith('AS') else ''
        print(f"| {scene} | {impl} | **{best['ms']:.3f}** | {ratio} | {rounds} | {extra} |")

print()
print('| scene | impl | ' + ' | '.join(STAGES) + ' |')
print('|---|---|' + '---|' * len(STAGES))
for scene in scenes:
    for impl in ORDER:
        rs = runs.get((scene, impl))
        if not rs:
            continue
        cells = [f"{min(float(r[s]) for r in rs):.3f}" for s in STAGES]
        print(f"| {scene} | {impl} | " + ' | '.join(cells) + ' |')
