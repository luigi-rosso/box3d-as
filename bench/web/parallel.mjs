// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// The port's parallel calls backend (bench/threads BACKEND=3) in a browser, as rive's parallel-calls docs say a page
// hosts it: librive's threads build on a cross origin isolated page, prepare() before the scripts start, the job
// pool sized before the file loads. One browser session, a fresh page per run.
//   PLAYWRIGHT=<dir holding node_modules/playwright-core> HARNESS=<wasm_threads_release> ROUNDS=3 node bench/web/parallel.mjs
//   BAKE=1 bakes each scene's two rivs first (lanes_bake.sh): <scene>__parallel_0 and <scene>__parallelhash_0 (HASHLOG).
//   MODE=bench (default) interleaves AS parts with C's pthreads build (out/c_simd_mt) at the same worker counts.
//   MODE=parity runs the HASHLOG bakes at every part count and checks every step against the native recording,
//     then on a page without isolation: HARNESS_PLAIN's build without threads, and HARNESS's threads build.
//   MODE=stages STAGE_RIV=<rive>/packages/runtime/tests/web_scripting/fixture/parallel_bench.riv runs rive's web stage
//     benchmark on the same page at 8 parts and prints what an empty call and a sync cost.
//   node bench/web/parallel.mjs --table=a.jsonl[,b.jsonl] prints min over rounds of each run's median ms per step.
// PARTS="1 2 4 8 default": 1 is parallelSafe, n is n - 1 job threads, default leaves the pool to librive.
// BROWSER=chromium (Chrome), firefox or webkit; the last two come from npx playwright-core install.

import { appendFileSync, copyFileSync, mkdirSync, readFileSync, existsSync } from 'node:fs';
import { execFileSync } from 'node:child_process';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { parseBench, recordingHashes, firstMismatch } from './core.mjs';
import { serve } from './serve.mjs';

const here = dirname( fileURLToPath( import.meta.url ) );
const root = join( here, '..', '..' );
const tableArg = process.argv.find( ( a ) => a.startsWith( '--table=' ) );
if ( tableArg )
{
	table( tableArg.slice( 8 ).split( ',' ) );
	process.exit( 0 );
}

const mode = process.env.MODE ?? 'bench';
const rounds = Number( process.env.ROUNDS ?? 3 );
const parts = ( process.env.PARTS ?? '1 2 4 8 default' ).split( /\s+/ ).filter( Boolean );
// scene:steps:bench/threads SCENE
const scenes = ( process.env.SCENES ?? 'large_pyramid:200:1 many_bodies:100:2 mesh_drop:120:5 village:120:6' )
	.split( /\s+/ ).filter( Boolean ).map( ( e ) => e.split( ':' ) );

if ( process.env.BAKE === '1' )
{
	for ( const [ scene, steps, index ] of scenes )
	{
		const options = [ `SCENE=${index}`, `STEPS=${steps}`, 'LANES=1', 'BACKEND=3', 'TASKS=3' ];
		execFileSync( join( here, 'lanes_bake.sh' ), [ `${scene}__parallel_0`, ...options ], { stdio: 'inherit' } );
		execFileSync( join( here, 'lanes_bake.sh' ), [ `${scene}__parallelhash_0`, ...options, 'HASHLOG=1' ],
			{ stdio: 'inherit' } );
	}
}

const browserName = process.env.BROWSER ?? 'chromium';
const pw = await import( pathToFileURL( join( process.env.PLAYWRIGHT, 'node_modules', 'playwright-core', 'index.mjs' ) ) );
const browser = await pw[browserName].launch( browserName === 'chromium' ? { channel: 'chrome', headless: true }
	: { headless: true } );
const isolated = await serve( root, { harness: process.env.HARNESS } );
const servers = [ isolated ];

mkdirSync( join( here, 'out' ), { recursive: true } );
const stamp = new Date().toISOString().replace( /[-:]/g, '' ).slice( 0, 15 );
const log = join( here, 'out', `parallel_${mode}_${browserName}_${stamp}.jsonl` );
appendFileSync( log, JSON.stringify( { browser: `${browserName} ${browser.version()}`, mode,
	machine: execFileSync( 'sysctl', [ '-n', 'machdep.cpu.brand_string' ] ).toString().trim() } ) + '\n' );
console.log( `log ${log}` );

const loadNow = () => Number( execFileSync( 'uptime' ).toString().match( /load averages?: ([\d.]+)/ )[1] );
const reference = ( scene ) =>
{
	const path = join( root, 'verify', 'scenes', 'out', `${scene}_scalar_w1.bin` );
	return existsSync( path ) ? recordingHashes( readFileSync( path ) ) : [];
};
const hostOptions = ( p ) => p === '1' ? { parallelSafe: true } : p === 'default' ? {} : { jobThreads: Number( p ) - 1 };

// Runs fn on a fresh page, giving up after seconds; a page that never answers reports its last console lines
async function onPage( origin, page, fn, arg, seconds = 240 )
{
	const tab = await browser.newPage();
	const consoleLines = [];
	tab.on( 'console', ( m ) => consoleLines.push( m.text() ) );
	tab.on( 'pageerror', ( e ) => consoleLines.push( `pageerror: ${e.message}` ) );
	let timer;
	try
	{
		await tab.goto( `${origin}/bench/web/${page}` );
		await tab.waitForFunction( () => window.ready === true );
		const timeout = new Promise( ( settle ) =>
		{
			timer = setTimeout( () => settle( { error: `no answer in ${seconds} s`, lines: consoleLines.slice( -6 ) } ),
				seconds * 1000 );
		} );
		return await Promise.race( [ tab.evaluate( fn, arg ), timeout ] );
	}
	catch ( err )
	{
		return { error: String( err ), lines: consoleLines.slice( -6 ) };
	}
	finally
	{
		clearTimeout( timer );
		await tab.close();
	}
}

// One AS bake; parts is the label, options what the page gives the harness
async function runAs( origin, name, parts, options, seconds = 180 )
{
	const out = await onPage( origin, 'lanes.html', ( [ n, s, o ] ) => window.runRiv( n, s, o ), [ name, seconds, options ],
		seconds + 60 );
	const scene = name.split( '__' )[0];
	const ref = reference( scene );
	if ( !out.result )
	{
		return { impl: 'as_parallel', scene, parts, error: `${out.error ?? ''} ${( out.lines ?? [] ).slice( -6 ).join( ' / ' )}` };
	}
	const result = { ...parseBench( out.result, name, ref ), impl: 'as_parallel', parts, workers: parts === 'default'
		? out.size : Number( parts ), size: out.size, team: out.team, isolated: out.crossOriginIsolated };
	if ( out.hashes.length > 0 )
	{
		result.hashSteps = out.hashes.length;
		result.stepMismatch = firstMismatch( out.hashes, ref );
	}
	return result;
}

async function runC( scene, steps, workers )
{
	const out = await onPage( isolated.origin, 'page.html', ( [ s, n, w ] ) => window.runOne( 'c_simd_mt', s, n, w, 1 ),
		[ scene, steps, workers ] );
	return out.error ? { impl: 'c_simd_mt', scene, workers, error: out.error } : out;
}

function record( round, load, result )
{
	appendFileSync( log, JSON.stringify( { round, load, ...result } ) + '\n' );
	const hash = result.error ? `ERROR ${result.error.slice( 0, 300 )}`
		: result.stepMismatch !== undefined ? ( result.stepMismatch === -1 ? `every step ok (${result.hashSteps})`
			: `STEP MISMATCH at ${result.stepMismatch}` )
			: result.mismatch === -1 ? 'hash ok' : 'HASH MISMATCH';
	console.log( `round ${round} ${result.scene} ${result.impl} ${result.parts ?? 'w' + result.workers} load ${load} ` +
		`median ${result.median?.toFixed?.( 3 ) ?? result.median} size ${result.size ?? '-'} ${hash}` );
}

try
{
	if ( mode === 'bench' )
	{
		for ( let round = 0; round < rounds; ++round )
		{
			for ( const [ scene, steps ] of scenes )
			{
				const jobs = [];
				for ( const p of parts )
				{
					jobs.push( () => runAs( isolated.origin, `${scene}__parallel_0`, p, hostOptions( p ) ) );
					if ( p !== 'default' )
					{
						jobs.push( () => runC( scene, Number( steps ), Number( p ) ) );
					}
				}
				// Rotate the order each round so no run always comes first
				for ( let j = 0; j < jobs.length; ++j )
				{
					const load = loadNow();
					record( round, load, await jobs[( j + round * 3 ) % jobs.length]() );
				}
			}
		}
	}
	else if ( mode === 'parity' )
	{
		for ( const [ scene ] of scenes )
		{
			for ( const p of parts )
			{
				record( 0, loadNow(), await runAs( isolated.origin, `${scene}__parallelhash_0`, p, hostOptions( p ) ) );
			}
		}
		// Without isolation a page cannot share memory, so it loads the build without threads; the threads build is
		// tried too, to see how it fails there
		const plain = await serve( root, { harness: process.env.HARNESS_PLAIN, isolated: false } );
		const plainThreads = await serve( root, { harness: process.env.HARNESS, isolated: false } );
		servers.push( plain, plainThreads );
		for ( const [ scene ] of scenes )
		{
			for ( const p of [ 'default', '8' ] )
			{
				const r = await runAs( plain.origin, `${scene}__parallelhash_0`, p, hostOptions( p ) );
				record( 0, loadNow(), { ...r, impl: 'as_parallel_no_isolation' } );
			}
		}
		const [ scene ] = scenes[0];
		const r = await runAs( plainThreads.origin, `${scene}__parallelhash_0`, 'default', {}, 20 );
		record( 0, loadNow(), { ...r, impl: 'as_parallel_threads_build_no_isolation' } );
	}
	else if ( mode === 'stages' )
	{
		copyFileSync( process.env.STAGE_RIV, join( here, 'out', 'lanes', 'stage_bench.riv' ) );
		for ( let round = 0; round < rounds; ++round )
		{
			const load = loadNow();
			const out = await onPage( isolated.origin, 'lanes.html', ( o ) => window.runRiv( 'stage_bench', 300, o ),
				{ jobThreads: 7, until: 'bench done' }, 360 );
			const lines = ( out.lines ?? [] ).filter( ( l ) => /(bench size|empty parts|stage [0-9])/.test( l ) );
			appendFileSync( log, JSON.stringify( { round, load, lines, error: out.error } ) + '\n' );
			console.log( `round ${round} load ${load}\n${lines.join( '\n' )}${out.error ? '\n' + out.error : ''}` );
		}
	}
}
finally
{
	await browser.close();
	servers.forEach( ( s ) => s.close() );
}

// Per scene, min over rounds of each run's median ms per step, AS parts against C workers, with the ratio and each
// side's speed up over its own 1
function table( paths )
{
	const runs = [];
	for ( const path of paths )
	{
		runs.push( ...readFileSync( path, 'utf8' ).trim().split( '\n' ).slice( 1 ).map( ( l ) => JSON.parse( l ) )
			.filter( ( r ) => !r.error ) );
	}
	const best = new Map();
	for ( const r of runs )
	{
		const key = `${r.scene}|${r.impl}|${r.impl === 'c_simd_mt' ? r.workers : r.parts}`;
		const ok = r.mismatch === -1 && r.compared > 0;
		const b = best.get( key ) ?? { median: Infinity, n: 0, ok: 0, size: r.size };
		b.median = Math.min( b.median, r.median );
		b.n += 1;
		b.ok += ok ? 1 : 0;
		best.set( key, b );
	}
	const loads = runs.map( ( r ) => r.load );
	const ok = [ ...best.values() ].reduce( ( a, b ) => a + b.ok, 0 );
	console.log( `${runs.length} runs, load ${Math.min( ...loads )} to ${Math.max( ...loads )}, hashes ok ${ok} of ${runs.length}` );
	console.log( '| scene | parts | C pthreads | AS parallel calls | AS / C | C speed up | AS speed up |' );
	console.log( '|---|---|---|---|---|---|---|' );
	for ( const scene of [ ...new Set( runs.map( ( r ) => r.scene ) ) ] )
	{
		const c1 = best.get( `${scene}|c_simd_mt|1` )?.median;
		const a1 = best.get( `${scene}|as_parallel|1` )?.median;
		for ( const p of [ '1', '2', '4', '8', 'default' ] )
		{
			const c = best.get( `${scene}|c_simd_mt|${p}` )?.median;
			const a = best.get( `${scene}|as_parallel|${p}` );
			const label = p === 'default' && a ? `default (${a.size})` : p;
			const cell = ( x ) => x === undefined ? '' : x.toFixed( 3 );
			const ratio = a && c ? ( a.median / c ).toFixed( 2 ) : '';
			const up = ( one, x ) => one && x ? `${( one / x ).toFixed( 2 )}x` : '';
			console.log( `| ${scene} | ${label} | ${cell( c )} | ${cell( a?.median )} | ${ratio} | ${up( c1, c )} | ` +
				`${up( a1, a?.median )} |` );
		}
	}
}
