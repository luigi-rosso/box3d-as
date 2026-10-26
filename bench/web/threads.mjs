// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Threaded web bench in node: native Box3D and its pthreads wasm build at each worker count, and the port's
// bench/threads bakes (out/lanes, from lanes_bake.sh) on rive's threaded web runtime at each lane count,
// interleaved over rounds. Appends to a jsonl that bench.mjs --table= reads.
//   HARNESS_NODE=<web_scripting_harness.mjs> ROUNDS=3 node bench/web/threads.mjs
//   BAKE=1 also bakes every scene and config first.
//   node bench/web/threads.mjs --table=a.jsonl[,b.jsonl] prints ms per step by worker or lane count
// Every run reports the median over steps 1 on, which is what bench/threads times; the C runs take run.mjs's
// untimed pass first, the AS bakes run once, as a page would.

import { execFileSync } from 'node:child_process';
import { appendFileSync, readFileSync, existsSync, mkdirSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { parseBench, recordingHashes } from './core.mjs';

const here = dirname( fileURLToPath( import.meta.url ) );
const tableArg = process.argv.find( ( a ) => a.startsWith( '--table=' ) );
if ( tableArg )
{
	table( tableArg.slice( 8 ).split( ',' ) );
	process.exit( 0 );
}
const rounds = Number( process.env.ROUNDS ?? 3 );
const counts = ( process.env.COUNTS ?? '1 2 4 8' ).split( /\s+/ ).filter( Boolean ).map( Number );
// scene:steps:bench/threads SCENE
const scenes = ( process.env.SCENES ?? 'large_pyramid:200:1 many_bodies:100:2 mesh_drop:120:5 village:120:6' )
	.split( /\s+/ ).filter( Boolean ).map( ( e ) => e.split( ':' ) );
const impls = ( process.env.IMPLS ?? 'native_simd c_simd_mt' ).split( /\s+/ ).filter( Boolean );
// config:TASKS:BACKEND
const configs = ( process.env.CONFIGS ?? 'solve:2:2 both:3:2 solve-jobs:2:1' ).split( /\s+/ ).filter( Boolean )
	.map( ( e ) => e.split( ':' ) );

const rivs = [];
for ( const [ scene, steps, sceneIndex ] of scenes )
{
	rivs.push( { name: `${scene}__serial_0`, options: `SCENE=${sceneIndex} STEPS=${steps} LANES=0` } );
	for ( const [ config, tasks, backend ] of configs )
	{
		for ( const n of counts )
		{
			rivs.push( { name: `${scene}__${config}_${n}`,
				options: `SCENE=${sceneIndex} STEPS=${steps} LANES=${n} TASKS=${tasks} BACKEND=${backend} IDLEFRAMES=30` } );
		}
	}
}
if ( process.env.BAKE === '1' )
{
	for ( const r of rivs )
	{
		execFileSync( join( here, 'lanes_bake.sh' ), [ r.name, ...r.options.split( ' ' ) ], { stdio: 'inherit' } );
		console.log( `baked ${r.name}` );
	}
}

mkdirSync( join( here, 'out' ), { recursive: true } );
const stamp = new Date().toISOString().replace( /[-:]/g, '' ).slice( 0, 15 );
const log = join( here, 'out', `threads_${stamp}.jsonl` );
appendFileSync( log, JSON.stringify( { node: process.version, v8: process.versions.v8,
	machine: execFileSync( 'sysctl', [ '-n', 'machdep.cpu.brand_string' ] ).toString().trim() } ) + '\n' );
console.log( `log ${log}` );
const loadNow = () => Number( execFileSync( 'uptime' ).toString().match( /load averages?: ([\d.]+)/ )[1] );
const reference = ( scene ) =>
{
	const path = join( here, '..', '..', 'verify', 'scenes', 'out', `${scene}_scalar_w1.bin` );
	return existsSync( path ) ? recordingHashes( readFileSync( path ) ) : [];
};

for ( let round = 0; round < rounds; ++round )
{
	for ( const [ scene, steps ] of scenes )
	{
		const ref = reference( scene );
		const jobs = [];
		for ( const impl of impls )
		{
			for ( const n of counts )
			{
				jobs.push( () => JSON.parse( execFileSync( process.execPath, [ join( here, 'run.mjs' ), impl, scene, steps,
					`--workers=${n}`, '--warmup=1' ], { maxBuffer: 1 << 26 } ).toString().trim().split( '\n' ).pop() ) );
			}
		}
		for ( const r of rivs.filter( ( r ) => r.name.startsWith( `${scene}__` ) ) )
		{
			jobs.push( () =>
			{
				let text;
				try
				{
					text = execFileSync( process.execPath, [ join( here, 'lanes.mjs' ), process.env.HARNESS_NODE,
						join( here, 'out', 'lanes', `${r.name}.riv` ), '--seconds=120' ], { maxBuffer: 1 << 26 } ).toString();
				}
				catch ( err )
				{
					text = String( err.stdout ?? err );
				}
				const line = text.split( '\n' ).find( ( l ) => l.includes( 'BENCH RESULT' ) );
				return line ? parseBench( line, r.name, ref ) : { impl: r.name, scene, error: text.slice( -400 ) };
			} );
		}
		// Rotate the order each round so no implementation always runs first
		for ( let j = 0; j < jobs.length; ++j )
		{
			const load = loadNow();
			const result = { round, load, ...jobs[( j + round * 3 ) % jobs.length]() };
			appendFileSync( log, JSON.stringify( result ) + '\n' );
			console.log( `round ${round} ${scene} ${result.impl} n${result.workers} load ${load} median ` +
				`${result.median?.toFixed( 3 )} ${result.error ? 'ERROR' : result.mismatch === -1 ? 'hash ok' : 'HASH MISMATCH'}` );
		}
	}
}

// Per scene, the min over rounds of each run's median ms per step, by worker or lane count, with the speed up
// over the same implementation at 1; logs from several sessions (node, Chrome) join as separate rows
function table( paths )
{
	const runs = [];
	for ( const path of paths )
	{
		const lines = readFileSync( path, 'utf8' ).trim().split( '\n' ).map( ( l ) => JSON.parse( l ) );
		const host = lines[0].v8 === 'chrome' ? lines[0].node : `node ${lines[0].node}`;
		runs.push( ...lines.slice( 1 ).filter( ( r ) => !r.error ).map( ( r ) => ( { ...r, host } ) ) );
	}
	const best = new Map();
	for ( const r of runs )
	{
		const key = `${r.scene}|${r.host}|${r.impl}|${r.workers}`;
		const b = best.get( key ) ?? { median: Infinity, n: 0, ok: 0 };
		b.median = Math.min( b.median, r.median );
		b.n += 1;
		b.ok += r.mismatch === -1 && r.compared > 0 ? 1 : 0;
		best.set( key, b );
	}
	const counts = [ 1, 2, 4, 8 ];
	const loads = runs.map( ( r ) => r.load );
	console.log( `${runs.length} runs, load ${Math.min( ...loads )} to ${Math.max( ...loads )}, hashes ok ` +
		`${[ ...best.values() ].reduce( ( a, b ) => a + b.ok, 0 )} of ${runs.length}` );
	console.log( '| scene | host | impl | 1 | 2 | 4 | 8 |' );
	console.log( '|---|---|---|---|---|---|---|' );
	const rows = [ ...new Set( runs.map( ( r ) => `${r.scene}|${r.host}|${r.impl}` ) ) ];
	for ( const row of rows )
	{
		const one = best.get( `${row}|1` )?.median;
		const cells = counts.map( ( n ) =>
		{
			const b = best.get( `${row}|${n}` );
			if ( !b )
			{
				return '';
			}
			return n === 1 || !one ? b.median.toFixed( 3 ) : `${b.median.toFixed( 3 )} (x${( one / b.median ).toFixed( 2 )})`;
		} );
		console.log( `| ${row.split( '|' ).join( ' | ' )} | ${cells.join( ' | ' )} |` );
	}
}
