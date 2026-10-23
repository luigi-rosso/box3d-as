# SPDX-FileCopyrightText: 2026 Erin Catto
# SPDX-License-Identifier: MIT
# Turns a run.sh log into markdown tables
import re
import sys

load = None
build = None
rows = []
upstream = []
current = None
for line in open(sys.argv[1]):
    m = re.search(r'load averages?: ([\d.]+)[, ]+([\d.]+)[, ]+([\d.]+)', line)
    if line.startswith('UPTIME') and m:
        load = m.group(1)
    elif line.startswith('BUILD'):
        build = line.split()[1]
    elif line.startswith('RESULT'):
        kv = dict(p.split('=', 1) for p in line.split()[1:])
        kv['build'] = build
        kv['load'] = load
        rows.append(kv)
    elif line.startswith('UPSTREAM'):
        kv = dict(p.split('=', 1) for p in line.split()[1:])
        kv['load'] = load
        kv['runs'] = []
        upstream.append(kv)
        current = kv
    elif current is not None:
        m = re.match(r'run \d+ : ([\d.e+-]+) \(ms\)', line)
        if m:
            current['runs'].append(float(m.group(1)))

stages = ['step', 'pairs', 'collide', 'solve', 'constraints', 'transforms', 'refit', 'bullets', 'sleepIslands', 'solverSetup', 'splitIslands']
print('| scene | build | workers | load | median ms/step | min ms/step | ' + ' | '.join(stages) + ' |')
print('|' + '---|' * (6 + len(stages)))
for r in rows:
    print(f"| {r['scene']} | {r['build']} | {r['workers']} | {r['load']} | {float(r['median']):.3f} | {float(r['min']):.3f} | " +
          ' | '.join(f"{float(r[s]):.3f}" for s in stages) + ' |')

if upstream:
    print()
    print('| upstream benchmark | build | workers | load | min total ms (199 steps) | min ms/step |')
    print('|---|---|---|---|---|---|')
    for u in upstream:
        if u['runs']:
            best = min(u['runs'])
            print(f"| {u['benchmark']} | {u['build']} | {u['workers']} | {u['load']} | {best:.1f} | {best / 199:.3f} |")
