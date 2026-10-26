// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Loads one web module (C through emscripten, or the port's rasc bake) and runs a scene, timing each step
// with performance.now and recording its b3Profile fields and state hash. Shared by node and the browser.

export const SCENES = [ 'pyramid', 'large_pyramid', 'card_house', 'chains', 'capsule_mesh', 'heightfield', 'bullets',
	'mover', 'many_bodies', 'plank_pile', 'mesh_drop', 'village', 'queries' ];
export const STAGES = [ 'step', 'pairs', 'collide', 'solve', 'solverSetup', 'constraints', 'prepareConstraints',
	'integrateVelocities', 'warmStart', 'solveImpulses', 'integratePositions', 'relaxImpulses', 'restitution',
	'storeImpulses', 'splitIslands', 'transforms', 'sensorHits', 'jointEvents', 'hitEvents', 'refit', 'bullets',
	'sleepIslands', 'sensors' ];
export const QUERY_KINDS = 5;

// fetchBytes(name) returns the bytes of an AnimaScript module; importGlue(name) the emscripten factory
export async function load( impl, { fetchBytes, importGlue } )
{
	if ( impl.startsWith( 'c_' ) )
	{
		const factory = await importGlue( impl );
		const m = await factory();
		return {
			create: m._create, step: m._step, stateHash: () => m._stateHash() >>> 0, profileField: m._profileField,
			queryMs: m._queryMs, queryHash: ( k ) => m._queryHash( k ) >>> 0, frameBoundary: () => {},
		};
	}
	const bytes = await fetchBytes( impl );
	const imports = { rive_rt_v1: { now: () => performance.now() } };
	const { instance } = await WebAssembly.instantiate( bytes, imports );
	const e = instance.exports;
	return {
		create: e.create, step: e.step, stateHash: () => e.stateHash() >>> 0, profileField: e.profileField,
		queryMs: e.queryMs, queryHash: ( k ) => e.queryHash( k ) >>> 0, frameBoundary: () => e.frameBoundary(),
	};
}

export function run( m, scene, steps, workers )
{
	const sceneIndex = SCENES.indexOf( scene );
	if ( sceneIndex < 0 || !m.create( sceneIndex, workers ) )
	{
		throw new Error( `create failed for ${scene}` );
	}
	m.frameBoundary();
	const ms = new Array( steps );
	const stages = new Array( steps * STAGES.length );
	const hashes = new Array( steps );
	for ( let i = 0; i < steps; ++i )
	{
		const t0 = performance.now();
		m.step( i );
		ms[i] = performance.now() - t0;
		for ( let k = 0; k < STAGES.length; ++k )
		{
			stages[i * STAGES.length + k] = m.profileField( k );
		}
		hashes[i] = m.stateHash();
		// The frame collector runs between steps, outside the timed step, as in rive
		m.frameBoundary();
	}
	const queryMs = [];
	const queryHash = [];
	for ( let k = 0; k < QUERY_KINDS; ++k )
	{
		queryMs.push( m.queryMs( k ) );
		queryHash.push( m.queryHash( k ) );
	}
	return { ms, stages, hashes, queryMs, queryHash };
}

// The per step state hashes of a version 1 trajectory, as verify/scenes/hash.mjs computes the last one
export function recordingHashes( bytes )
{
	const w = new Uint32Array( bytes.buffer, bytes.byteOffset, bytes.byteLength >> 2 );
	const counters = w[14];
	const moverWords = w[15];
	const hashes = [];
	let i = 16;
	while ( i < w.length )
	{
		const n = w[i + 1];
		i += 2 + counters;
		let h = 2166136261;
		for ( let b = 0; b < n; ++b )
		{
			const r = i + b * 17 + 4;
			for ( let k = 0; k < 13; ++k )
			{
				const bits = w[r + k];
				for ( let s = 0; s < 4; ++s )
				{
					h = Math.imul( h ^ ( ( bits >>> ( 8 * s ) ) & 0xff ), 16777619 ) >>> 0;
				}
			}
		}
		hashes.push( h );
		i += n * 17 + moverWords;
	}
	return hashes;
}

// First step whose hash differs, or -1; steps past the recording are not compared
export function firstMismatch( hashes, reference )
{
	const n = Math.min( hashes.length, reference.length );
	for ( let i = 0; i < n; ++i )
	{
		if ( hashes[i] !== reference[i] )
		{
			return i;
		}
	}
	return -1;
}

function median( sorted )
{
	const n = sorted.length;
	return n & 1 ? sorted[n >> 1] : 0.5 * ( sorted[n / 2 - 1] + sorted[n / 2] );
}

// Steps from warmup on: min and median ms per step, and each stage's mean. mean1 is the mean over steps 1 on,
// what bench/native/bench.c reports.
export function summarize( raw, warmup )
{
	const timed = raw.ms.slice( warmup );
	const sorted = [ ...timed ].sort( ( a, b ) => a - b );
	const stages = {};
	for ( let k = 0; k < STAGES.length; ++k )
	{
		let sum = 0;
		for ( let i = warmup; i < raw.ms.length; ++i )
		{
			sum += raw.stages[i * STAGES.length + k];
		}
		stages[STAGES[k]] = sum / timed.length;
	}
	const rest = raw.ms.slice( 1 );
	return {
		min: sorted[0], median: median( sorted ), mean: timed.reduce( ( a, b ) => a + b, 0 ) / timed.length,
		mean1: rest.reduce( ( a, b ) => a + b, 0 ) / rest.length, stages,
	};
}

// A bench/threads BENCH RESULT line, as a run record keyed like the node runs; name is the bake's
// <scene>__<config>_<lanes>, and the hash is checked against reference, the recording's per step hashes
export function parseBench( line, name, reference )
{
	const fields = Object.fromEntries( line.split( ' ' ).filter( ( w ) => w.includes( '=' ) ).map( ( w ) => w.split( '=' ) ) );
	const [ scene, rest ] = name.split( '__' );
	const config = rest.slice( 0, rest.lastIndexOf( '_' ) );
	const lanes = Number( rest.slice( rest.lastIndexOf( '_' ) + 1 ) );
	const steps = Number( fields.steps );
	const finalHash = Number( fields.hash );
	const compared = reference.length >= steps ? 1 : 0;
	return {
		impl: `as_${config}`, scene, steps, workers: Math.max( lanes, 1 ), median: Number( fields.median ),
		min: Number( fields.min ), mean: Number( fields.mean ), finalHash, compared,
		mismatch: compared && reference[steps - 1] === finalHash ? -1 : steps - 1,
	};
}
