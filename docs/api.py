#!/usr/bin/env python3
# Generates docs/api.md: every public Box3D function, by area, against what lib/box3d.as exports.
#   docs/api.py            write docs/api.md
#   docs/api.py --summary  print the area table the README carries
# Public means declared B3_API, B3_INLINE, B3_FORCE_INLINE or B3_ID_INLINE in the upstream headers below.
import os, re, sys

here = os.path.dirname(os.path.abspath(__file__))
root = os.path.dirname(here)
box3d = os.environ.get('BOX3D', os.path.expanduser('~/Projects/box3d'))
include = os.path.join(box3d, 'include', 'box3d')
headers = ['box3d.h', 'collision.h', 'math_functions.h', 'types.h', 'base.h', 'id.h']

AREAS = [
	'world', 'bodies', 'shapes', 'mesh, height field and compound shapes', 'joints, common', 'distance joint',
	'filter joint', 'motor joint', 'parallel joint', 'prismatic joint', 'revolute joint', 'spherical joint',
	'weld joint', 'wheel joint', 'queries', 'events', 'character mover', 'geometry and collision', 'dynamic tree',
	'math', 'ids and defs', 'base', 'debug draw', 'recording and replay',
]

MOVER = {
	'b3World_CastMover', 'b3World_CollideMover', 'b3Body_CollideMover', 'b3Body_TimeOfImpactMover', 'b3SolvePlanes',
	'b3ClipVector',
}
QUERIES = {
	'b3World_CastRay', 'b3World_CastRayClosest', 'b3World_CastShape', 'b3World_OverlapAABB', 'b3World_OverlapShape',
	'b3Shape_RayCast', 'b3Shape_GetClosestPoint',
}
DEBUG_DRAW = {'b3World_Draw', 'b3DefaultDebugDraw', 'b3GetGraphColor', 'b3MakeDebugColor'}

# Why each function that lib/box3d.as does not export is left out
REASONS = [
	(lambda n: area_of(n) == 'debug draw', 'debug draw'),
	(lambda n: area_of(n) == 'recording and replay', 'recording and replay'),
	(lambda n: n in {'b3DumpHeightData', 'b3LoadHeightField', 'b3DynamicTree_Save', 'b3DynamicTree_Load',
		'b3World_DumpShapeBounds'}, 'other: file IO'),
	(lambda n: n == 'b3ValidateCompound', 'other: validates serialized bytes'),
]

# Exported, but with nothing to do here: the runtime owns memory and threads
NO_EFFECT = {
	'b3SetAllocator': 'the runtime allocates',
	'b3GetByteCount': 'returns 0, the runtime allocates',
	'b3IsAVX2Available': 'returns false',
	'b3Yield': 'returns at once',
	'b3Sleep': 'returns at once',
	'b3DestroyHull': 'the collector frees it',
	'b3DestroyMesh': 'the collector frees it',
	'b3DestroyHeightField': 'the collector frees it',
	'b3DestroyCompound': 'the collector frees it',
	'b3World_DumpMemoryStats': 'prints nothing, the runtime allocates',
}


def area_of(name, header=None):
	header = header or declared.get(name)
	if name in DEBUG_DRAW:
		return 'debug draw'
	if re.match(r'b3Rec|b3(Create|Destroy)(Recording|Player)$|b3\w*Recording|b3ValidateReplay', name):
		return 'recording and replay'
	if name in MOVER:
		return 'character mover'
	if name in QUERIES:
		return 'queries'
	if re.match(r'b3World_Get\w*Events$|b3Contact_|b3(Shape|Body)_\w*(Events|ContactData|ContactCapacity|SensorData|SensorCapacity)', name):
		return 'events'
	m = re.match(r'b3(Create|Default)?(Distance|Filter|Motor|Parallel|Prismatic|Revolute|Spherical|Weld|Wheel)Joint', name)
	if m:
		return m.group(2).lower() + ' joint'
	if re.match(r'b3Joint_|b3DestroyJoint$', name):
		return 'joints, common'
	if re.match(r'b3Create(Mesh|HeightField|BakedCompound)Shape$|b3Shape_\w*(Mesh|HeightField|Compound)', name):
		return 'mesh, height field and compound shapes'
	if re.match(r'b3Create\w*Shape$|b3DestroyShape$|b3Shape_', name):
		return 'shapes'
	if re.match(r'b3(Create|Destroy)Body$|b3Body_', name):
		return 'bodies'
	if re.match(r'b3(Create|Destroy)World$|b3World_|b3Get(Max)?WorldCount$', name):
		return 'world'
	if header == 'id.h' or re.match(r'b3Default\w*$', name):
		return 'ids and defs'
	if name.startswith('b3DynamicTree_'):
		return 'dynamic tree'
	if header == 'math_functions.h':
		return 'math'
	if header == 'collision.h':
		return 'geometry and collision'
	if header == 'base.h':
		return 'base'
	return 'world'


def strip(text):
	text = re.sub(r'/\*.*?\*/', '', text, flags=re.S)
	text = re.sub(r'//[^\n]*', '', text)
	return re.sub(r'^\s*#.*$', '', text, flags=re.M)


declared = {}
for h in headers:
	body = strip(open(os.path.join(include, h)).read())
	for m in re.finditer(r'\bB3_(?:API|INLINE|FORCE_INLINE|ID_INLINE)\b[^;{(]*?\b([A-Za-z_]\w*)\s*\(', body):
		declared.setdefault(m.group(1), h)

exported = {}
text = open(os.path.join(root, 'lib', 'box3d.as')).read()
for m in re.finditer(r'export\s*\{([^}]*)\}\s*from\s*"\./(\w+)"', text):
	for part in m.group(1).split(','):
		part = part.strip()
		if part:
			exported[part.split(' as ')[-1].strip()] = m.group(2)


def reason_of(name):
	for test, reason in REASONS:
		if test(name):
			return reason
	return 'other'


areas = {a: [] for a in AREAS}
for name in sorted(declared, key=str.lower):
	areas[area_of(name)].append(name)


def summary_rows():
	rows = []
	for area in AREAS:
		names = areas[area]
		if not names:
			continue
		missing = [n for n in names if n not in exported]
		reasons = sorted({reason_of(n) for n in missing})
		note = ''
		if missing and len(missing) == len(names):
			note = 'all: ' + ', '.join(reasons)
		elif missing:
			note = ', '.join('`%s` (%s)' % (n, reason_of(n)) for n in missing)
		rows.append((area, len(names) - len(missing), len(names), note))
	return rows


def summary_table():
	out = ['| area | ported | not ported, and why |', '|---|---|---|']
	total = ported = 0
	for area, p, n, note in summary_rows():
		out.append('| %s | %d / %d | %s |' % (area, p, n, note))
		total += n
		ported += p
	out.append('| **all** | **%d / %d** | |' % (ported, total))
	return out


def write_doc():
	out = [
		'# API availability',
		'',
		'Generated by `docs/api.py` from the public functions of upstream Box3D at f36d061 (`B3_API` and the',
		'inline functions of `include/box3d/{%s}`) against the names `lib/box3d.as` exports. Regenerate it after' %
		','.join(h[:-2] for h in headers),
		'porting more of the API. Types, enums and constants are exported with the same names and are not listed.',
		'',
	]
	out += summary_table()
	out += ['', '## Not ported', '', '| function | area | why |', '|---|---|---|']
	for area in AREAS:
		for name in areas[area]:
			if name not in exported:
				out.append('| `%s` | %s | %s |' % (name, area, reason_of(name)))
	out += ['', '## Exported with no effect', '', '| function | why |', '|---|---|']
	for name in sorted(NO_EFFECT):
		if name in exported:
			out.append('| `%s` | %s |' % (name, NO_EFFECT[name]))
	out += ['', '## By area', '', 'Each ported function with the `lib/` module that defines it.']
	for area in AREAS:
		names = areas[area]
		if not names:
			continue
		ported = [n for n in names if n in exported]
		out += ['', '### %s (%d / %d)' % (area, len(ported), len(names)), '']
		out += ['| function | module |', '|---|---|']
		for name in names:
			out.append('| `%s` | %s |' % (name, ('`lib/%s.as`' % exported[name]) if name in exported else 'not ported'))
	open(os.path.join(here, 'api.md'), 'w').write('\n'.join(out) + '\n')


if '--summary' in sys.argv:
	print('\n'.join(summary_table()))
else:
	write_doc()
	rows = summary_rows()
	print('wrote docs/api.md: %d of %d functions ported' % (sum(r[1] for r in rows), sum(r[2] for r in rows)))
