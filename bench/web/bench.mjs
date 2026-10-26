// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Interleaved rounds of run.mjs, one process per run, then the tables:
//   ROUNDS=5 IMPLS="..." SCENES="large_pyramid:200 ..." WORKERS=1 node bench/web/bench.mjs
//   node bench/web/bench.mjs --table=out/bench_<stamp>.jsonl
// Each round runs every scene on every implementation, rotating the order. The table takes, per scene and
// implementation, the min over rounds of each run's median and min ms per step.

import { execFileSync } from 'node:child_process';
import { appendFileSync, readFileSync, mkdirSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname( fileURLToPath( import.meta.url ) );
const tableArg = process.argv.find( ( a ) => a.startsWith( '--table=' ) );

const DEFAULT_SCENES = 'large_pyramid:200 many_bodies:100 mesh_drop:120 village:120 plank_pile:300 chains:300 pyramid:300 queries:300';
const DEFAULT_IMPLS = 'native_simd native_scalar c_simd c_scalar as_simd as_scalar';

let log;
if ( tableArg )
{
	log = tableArg.slice( 8 );
}
else
{
	const rounds = Number( process.env.ROUNDS ?? 5 );
	const impls = ( process.env.IMPLS ?? DEFAULT_IMPLS ).split( /\s+/ ).filter( Boolean );
	const scenes = ( process.env.SCENES ?? DEFAULT_SCENES ).split( /\s+/ ).filter( Boolean );
	const workers = ( process.env.WORKERS ?? '1' ).split( /\s+/ ).filter( Boolean );
	const nodeFlags = ( process.env.NODE_FLAGS ?? '' ).split( /\s+/ ).filter( Boolean );
	mkdirSync( join( here, 'out' ), { recursive: true } );
	const stamp = new Date().toISOString().replace( /[-:]/g, '' ).slice( 0, 15 );
	log = join( here, 'out', `bench_${stamp}.jsonl` );
	appendFileSync( log, JSON.stringify( { node: process.version, v8: process.versions.v8, nodeFlags,
		machine: execFileSync( 'sysctl', [ '-n', 'machdep.cpu.brand_string' ] ).toString().trim() } ) + '\n' );
	console.log( `log ${log}` );
	for ( let r = 0; r < rounds; ++r )
	{
		for ( const entry of scenes )
		{
			const [ scene, steps ] = entry.split( ':' );
			for ( const w of workers )
			{
				for ( let j = 0; j < impls.length; ++j )
				{
					const impl = impls[( j + r ) % impls.length];
					const uptime = execFileSync( 'uptime' ).toString().trim();
					const load = Number( uptime.match( /load averages?: ([\d.]+)/ )[1] );
					let line;
					try
					{
						line = execFileSync( process.execPath, [ ...nodeFlags, join( here, 'run.mjs' ), impl, scene, steps,
							`--workers=${w}` ], { maxBuffer: 1 << 26 } ).toString().trim().split( '\n' ).pop();
					}
					catch ( err )
					{
						line = JSON.stringify( { impl, scene, workers: Number( w ), error: String( err.stdout ?? err ) } );
					}
					const result = { round: r, load, ...JSON.parse( line ) };
					appendFileSync( log, JSON.stringify( result ) + '\n' );
					console.log( `round ${r} ${scene} ${impl} w${w} load ${load} median ${result.median?.toFixed( 3 )} ` +
						`${result.mismatch === -1 ? 'hash ok' : 'HASH MISMATCH ' + result.mismatch}` );
				}
			}
		}
	}
}

table( log );

function table( path )
{
	const lines = readFileSync( path, 'utf8' ).trim().split( '\n' ).map( ( l ) => JSON.parse( l ) );
	const meta = lines[0];
	const runs = lines.slice( 1 ).filter( ( r ) => !r.error );
	const key = ( r ) => `${r.scene}|${r.workers}|${r.impl}`;
	const best = new Map();
	for ( const r of runs )
	{
		const b = best.get( key( r ) ) ?? { runs: [], median: Infinity, min: Infinity, mean: Infinity, mean1: Infinity,
			stages: null };
		b.runs.push( r );
		if ( r.median < b.median )
		{
			b.median = r.median;
			b.stages = r.stages;
			b.queryMs = r.queryMs;
		}
		b.min = Math.min( b.min, r.min );
		b.mean = Math.min( b.mean, r.mean );
		b.mean1 = Math.min( b.mean1, r.mean1 );
		best.set( key( r ), b );
	}
	const scenes = [ ...new Set( runs.map( ( r ) => `${r.scene}|${r.workers}` ) ) ];
	const impls = [ ...new Set( runs.map( ( r ) => r.impl ) ) ];
	const loads = runs.map( ( r ) => r.load );
	console.log( `\nnode ${meta.node}, V8 ${meta.v8}, ${meta.machine}, ${runs.length} runs, load ${Math.min( ...loads )} to ${Math.max( ...loads )}` );
	const ref = ( s, impl ) => best.get( `${s}|${impl}` );
	console.log( '\nms per step, min over rounds of the run median (run min), steps after warmup' );
	console.log( `| scene | workers | ${impls.join( ' | ' )} | as_simd / c_simd | as_scalar / c_scalar | hashes |` );
	console.log( `|---|---|${impls.map( () => '---|' ).join( '' )}---|---|---|` );
	for ( const s of scenes )
	{
		const [ scene, w ] = s.split( '|' );
		const cells = impls.map( ( i ) =>
		{
			const b = ref( s, i );
			return b ? `${fmt( b.median )} (${fmt( b.min )})` : '-';
		} );
		const ratio = ( a, b ) => ( ref( s, a ) && ref( s, b ) ? ( ref( s, a ).median / ref( s, b ).median ).toFixed( 2 ) : '-' );
		const all = impls.flatMap( ( i ) => ref( s, i )?.runs ?? [] );
		const ok = all.filter( ( r ) => r.mismatch === -1 && r.compared > 0 ).length;
		console.log( `| ${scene} | ${w} | ${cells.join( ' | ' )} | ${ratio( 'as_simd', 'c_simd' )} | ${ratio( 'as_scalar', 'c_scalar' )} | ${ok}/${all.length} |` );
	}
	console.log( '\nms per step, min over rounds of the run mean over steps after warmup (resolves steps below a coarse timer)' );
	console.log( `| scene | workers | ${impls.join( ' | ' )} | as_simd / c_simd | as_scalar / c_scalar |` );
	console.log( `|---|---|${impls.map( () => '---|' ).join( '' )}---|---|` );
	for ( const s of scenes )
	{
		const [ scene, w ] = s.split( '|' );
		const ratio = ( a, b ) => ( ref( s, a ) && ref( s, b ) ? ( ref( s, a ).mean / ref( s, b ).mean ).toFixed( 2 ) : '-' );
		console.log( `| ${scene} | ${w} | ${impls.map( ( i ) => ( ref( s, i ) ? fmt( ref( s, i ).mean ) : '-' ) ).join( ' | ' )} | ` +
			`${ratio( 'as_simd', 'c_simd' )} | ${ratio( 'as_scalar', 'c_scalar' )} |` );
	}
	const STAGES = [ 'step', 'pairs', 'collide', 'solve', 'solverSetup', 'prepareConstraints', 'integrateVelocities',
		'warmStart', 'solveImpulses', 'integratePositions', 'relaxImpulses', 'storeImpulses', 'splitIslands', 'transforms',
		'refit', 'bullets', 'sleepIslands' ];
	console.log( '\nb3Profile stages, mean ms per step after warmup, from the run with the lowest median' );
	console.log( `| scene | impl | ${STAGES.join( ' | ' )} |` );
	console.log( `|---|---|${STAGES.map( () => '---|' ).join( '' )}` );
	for ( const s of scenes )
	{
		for ( const i of impls )
		{
			const b = ref( s, i );
			if ( b )
			{
				console.log( `| ${s.replace( '|', ' w' )} | ${i} | ${STAGES.map( ( k ) => fmt( b.stages[k] ) ).join( ' | ' )} |` );
			}
		}
	}
}

function fmt( x )
{
	if ( x === undefined )
	{
		return '-';
	}
	return x >= 1 ? x.toFixed( 2 ) : x >= 0.1 ? x.toFixed( 3 ) : x.toFixed( 4 );
}
