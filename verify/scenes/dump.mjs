// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reads trajectory files written by traj.c (format in traj.c and notes/scenes.md).
//   node dump.mjs print <file> [--step=N] [--body=I]
//   node dump.mjs diff <a> <b> [--all]
// diff compares bodies, mover state and the physics counters; --all also compares the build and worker
// dependent counters. Exit code 0 when identical, 1 when they differ.

import { readFileSync } from 'node:fs';

export const counterNames = [
	'bodyCount', 'shapeCount', 'contactCount', 'jointCount', 'islandCount', 'awakeBodyCount',
	'awakeContactCount', 'recycledContactCount', 'staticTreeHeight', 'treeHeight', 'satCallCount',
	'satCacheHitCount', 'distanceIterations', 'pushBackIterations', 'rootIterations', 'contactBeginCount',
	'contactEndCount', 'contactHitCount', 'bodyMoveCount',
	...Array.from( { length: 24 }, ( _, i ) => `colorCounts[${i}]` ),
	...Array.from( { length: 8 }, ( _, i ) => `manifoldCounts[${i}]` ),
	'taskCount', 'stackUsed', 'arenaCapacity', 'byteCountLo', 'byteCountHi',
];
export const envCounterStart = 51;

export const bodyFields = [
	'index', 'flags', 'id.index1', 'id.generation', 'p.x', 'p.y', 'p.z', 'q.x', 'q.y', 'q.z', 'q.s',
	'v.x', 'v.y', 'v.z', 'w.x', 'w.y', 'w.z',
];
const bodyFloatStart = 4;

const planeFields = [ 'n.x', 'n.y', 'n.z', 'offset', 'pushLimit', 'push', 'clipVelocity', 'point.x', 'point.y',
	'point.z', 'shape.index1', 'shape.generation' ];
export const moverFields = [
	'p.x', 'p.y', 'p.z', 'v.x', 'v.y', 'v.z', 'pogoVelocity', 'onGround', 'sprint', 'planeCount',
	'totalIterations', 'moveIterations', 'rayHit', 'rayFraction', 'castFraction', 'reserved',
	...Array.from( { length: 8 }, ( _, i ) => planeFields.map( ( f ) => `plane[${i}].${f}` ) ).flat(),
];
const moverFloatFields = new Set( [ 0, 1, 2, 3, 4, 5, 6, 13, 14 ] );
for ( let i = 0; i < 8; ++i )
{
	for ( const k of [ 0, 1, 2, 3, 4, 5, 7, 8, 9 ] )
	{
		moverFloatFields.add( 16 + 12 * i + k );
	}
}

export function readTrajectory( path )
{
	const buffer = readFileSync( path );
	const u32 = new Uint32Array( buffer.buffer, buffer.byteOffset, buffer.byteLength >> 2 );
	const f32 = new Float32Array( buffer.buffer, buffer.byteOffset, buffer.byteLength >> 2 );
	if ( u32[0] !== 0x52543342 || u32[1] !== 1 )
	{
		throw new Error( `${path}: not a version 1 trajectory` );
	}
	const nameBytes = buffer.subarray( 8, 40 );
	const name = nameBytes.subarray( 0, nameBytes.indexOf( 0 ) < 0 ? 32 : nameBytes.indexOf( 0 ) ).toString();
	const header = {
		name,
		stepCount: u32[10],
		workerCount: u32[11],
		timeStep: f32[12],
		subStepCount: u32[13],
		counterCount: u32[14],
		moverWords: u32[15],
	};

	const steps = [];
	let o = 16;
	while ( o < u32.length )
	{
		const stepIndex = u32[o];
		const bodyCount = u32[o + 1];
		const counterOffset = o + 2;
		const bodyOffset = counterOffset + header.counterCount;
		const moverOffset = bodyOffset + bodyCount * bodyFields.length;
		steps.push( { stepIndex, bodyCount, counterOffset, bodyOffset, moverOffset } );
		o = moverOffset + header.moverWords;
	}
	return { path, header, steps, u32, f32 };
}

const hex = ( x ) => x.toString( 16 ).padStart( 8, '0' );

function formatBody( t, s, b )
{
	const base = s.bodyOffset + b * bodyFields.length;
	const u = t.u32;
	const f = t.f32;
	const flags = u[base + 1];
	const vec = ( k, n ) => Array.from( { length: n }, ( _, i ) => f[base + k + i].toPrecision( 9 ) ).join( ' ' );
	return `body ${u[base]} id ${u[base + 2]}.${u[base + 3]} valid ${flags & 1} awake ${( flags >> 1 ) & 1} type ${( flags >> 4 ) & 15}` +
		` p [${vec( 4, 3 )}] q [${vec( 7, 4 )}] v [${vec( 11, 3 )}] w [${vec( 14, 3 )}]`;
}

function print( t, stepFilter, bodyFilter )
{
	const h = t.header;
	console.log( `${h.name}: steps ${h.stepCount} workers ${h.workerCount} dt ${h.timeStep} substeps ${h.subStepCount}` +
		` counters ${h.counterCount} mover words ${h.moverWords}` );
	for ( const s of t.steps )
	{
		if ( stepFilter !== undefined && s.stepIndex !== stepFilter )
		{
			continue;
		}
		const c = counterNames.slice( 0, 19 ).map( ( n, i ) => `${n} ${t.u32[s.counterOffset + i] | 0}` ).join( ', ' );
		console.log( `step ${s.stepIndex} bodies ${s.bodyCount}: ${c}` );
		for ( let b = 0; b < s.bodyCount; ++b )
		{
			if ( bodyFilter === undefined || bodyFilter === b )
			{
				console.log( '  ' + formatBody( t, s, b ) );
			}
		}
		if ( h.moverWords > 0 )
		{
			const m = s.moverOffset;
			const parts = [];
			for ( let i = 0; i < 16; ++i )
			{
				const v = moverFloatFields.has( i ) ? t.f32[m + i].toPrecision( 9 ) : t.u32[m + i];
				parts.push( `${moverFields[i]} ${v}` );
			}
			console.log( '  mover ' + parts.join( ', ' ) );
			const planeCount = t.u32[m + 9];
			for ( let p = 0; p < planeCount; ++p )
			{
				const k = m + 16 + 12 * p;
				const n = [ 0, 1, 2 ].map( ( i ) => t.f32[k + i].toPrecision( 9 ) ).join( ' ' );
				const pt = [ 7, 8, 9 ].map( ( i ) => t.f32[k + i].toPrecision( 9 ) ).join( ' ' );
				console.log( `    plane ${p} n [${n}] offset ${t.f32[k + 3].toPrecision( 9 )} push ${t.f32[k + 5].toPrecision( 9 )}` +
					` clip ${t.u32[k + 6]} point [${pt}] shape ${t.u32[k + 10]}.${t.u32[k + 11]}` );
			}
		}
	}
}

function valueText( t, index, isFloat )
{
	return isFloat ? `${t.f32[index].toPrecision( 9 )} (${hex( t.u32[index] )})` : `${t.u32[index] | 0}`;
}

// Returns a summary; prints the first differing body field, counter and mover field
export function diff( a, b, all, quiet )
{
	const result = { identical: true, firstBody: null, firstCounter: null, firstMover: null, differingSteps: 0 };
	const report = ( ...args ) => quiet || console.log( ...args );
	if ( a.header.counterCount !== b.header.counterCount || a.header.moverWords !== b.header.moverWords )
	{
		report( 'layouts differ' );
		result.identical = false;
		return result;
	}
	const counterLimit = all ? a.header.counterCount : envCounterStart;
	const stepCount = Math.min( a.steps.length, b.steps.length );
	if ( a.steps.length !== b.steps.length )
	{
		report( `step counts differ: ${a.steps.length} vs ${b.steps.length}` );
		result.identical = false;
	}
	for ( let si = 0; si < stepCount; ++si )
	{
		const sa = a.steps[si];
		const sb = b.steps[si];
		let stepDiffers = false;

		if ( sa.bodyCount !== sb.bodyCount )
		{
			stepDiffers = true;
			if ( result.firstBody === null )
			{
				result.firstBody = { step: sa.stepIndex, body: -1, field: 'bodyCount' };
				report( `first body difference: step ${sa.stepIndex} body count ${sa.bodyCount} vs ${sb.bodyCount}` );
			}
		}

		for ( let i = 0; i < counterLimit; ++i )
		{
			const va = a.u32[sa.counterOffset + i];
			const vb = b.u32[sb.counterOffset + i];
			if ( va !== vb )
			{
				stepDiffers = true;
				if ( result.firstCounter === null )
				{
					result.firstCounter = { step: sa.stepIndex, counter: counterNames[i] };
					report( `first counter difference: step ${sa.stepIndex} ${counterNames[i]} ${va | 0} vs ${vb | 0}` );
				}
				break;
			}
		}

		const bodyCount = Math.min( sa.bodyCount, sb.bodyCount );
		const words = bodyCount * bodyFields.length;
		for ( let k = 0; k < words; ++k )
		{
			if ( a.u32[sa.bodyOffset + k] !== b.u32[sb.bodyOffset + k] )
			{
				stepDiffers = true;
				if ( result.firstBody === null )
				{
					const body = Math.floor( k / bodyFields.length );
					const field = k % bodyFields.length;
					const isFloat = field >= bodyFloatStart;
					result.firstBody = { step: sa.stepIndex, body, field: bodyFields[field] };
					report( `first body difference: step ${sa.stepIndex} body ${body} ${bodyFields[field]} ` +
						`${valueText( a, sa.bodyOffset + k, isFloat )} vs ${valueText( b, sb.bodyOffset + k, isFloat )}` );
					report( '  a ' + formatBody( a, sa, body ) );
					report( '  b ' + formatBody( b, sb, body ) );
				}
				break;
			}
		}

		for ( let k = 0; k < a.header.moverWords; ++k )
		{
			if ( a.u32[sa.moverOffset + k] !== b.u32[sb.moverOffset + k] )
			{
				stepDiffers = true;
				if ( result.firstMover === null )
				{
					const isFloat = moverFloatFields.has( k );
					result.firstMover = { step: sa.stepIndex, field: moverFields[k] };
					report( `first mover difference: step ${sa.stepIndex} ${moverFields[k]} ` +
						`${valueText( a, sa.moverOffset + k, isFloat )} vs ${valueText( b, sb.moverOffset + k, isFloat )}` );
				}
				break;
			}
		}

		if ( stepDiffers )
		{
			result.identical = false;
			result.differingSteps += 1;
		}
	}

	if ( result.identical )
	{
		report( `identical over ${stepCount} steps${all ? '' : ' (environment counters ignored)'}` );
	}
	else
	{
		report( `${result.differingSteps} of ${stepCount} steps differ` );
	}
	return result;
}

function main()
{
	const args = process.argv.slice( 2 );
	const flags = Object.fromEntries( args.filter( ( a ) => a.startsWith( '--' ) ).map( ( a ) =>
	{
		const [ k, v ] = a.slice( 2 ).split( '=' );
		return [ k, v === undefined ? true : v ];
	} ) );
	const positional = args.filter( ( a ) => !a.startsWith( '--' ) );
	const command = positional[0];

	if ( command === 'print' && positional.length === 2 )
	{
		const step = flags.step === undefined ? undefined : Number( flags.step );
		const body = flags.body === undefined ? undefined : Number( flags.body );
		print( readTrajectory( positional[1] ), step, body );
		return 0;
	}

	if ( command === 'diff' && positional.length === 3 )
	{
		const result = diff( readTrajectory( positional[1] ), readTrajectory( positional[2] ), flags.all === true, false );
		return result.identical ? 0 : 1;
	}

	console.error( 'usage: node dump.mjs print <file> [--step=N] [--body=I]\n       node dump.mjs diff <a> <b> [--all]' );
	return 2;
}

if ( import.meta.url === `file://${process.argv[1]}` )
{
	process.exitCode = main();
}
