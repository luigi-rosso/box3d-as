// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// One timed run in node, checked against the native scalar recording step by step:
//   node bench/web/run.mjs <impl> <scene> <steps> [--workers=1] [--warmup=n]
// impl: c_simd, c_scalar, as_simd, as_scalar (bench/web/out), native_simd, native_scalar (bench/web/out/bin).
// Prints one JSON line.

import { readFileSync, existsSync } from 'node:fs';
import { execFileSync } from 'node:child_process';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { load, run, summarize, recordingHashes, firstMismatch, SCENES } from './core.mjs';

const here = dirname( fileURLToPath( import.meta.url ) );
const out = join( here, 'out' );
const args = process.argv.slice( 2 ).filter( ( a ) => !a.startsWith( '--' ) );
const flags = Object.fromEntries( process.argv.slice( 2 ).filter( ( a ) => a.startsWith( '--' ) ).map( ( a ) =>
	a.slice( 2 ).split( '=' ) ) );
const [ impl, scene ] = args;
const steps = Number( args[2] );
const workers = Number( flags.workers ?? 1 );
const warmup = Number( flags.warmup ?? Math.floor( steps / 10 ) );

let raw;
if ( impl.startsWith( 'native_' ) )
{
	const bin = join( out, 'bin', impl );
	raw = JSON.parse( execFileSync( bin, [ String( SCENES.indexOf( scene ) ), String( steps ), String( workers ) ],
		{ maxBuffer: 1 << 28 } ).toString() );
}
else
{
	const m = await load( impl, {
		fetchBytes: async ( name ) => readFileSync( join( out, `${name}.wasm` ) ),
		importGlue: async ( name ) => ( await import( pathToFileURL( join( out, `${name}.mjs` ) ) ) ).default,
	} );
	raw = run( m, scene, steps, workers );
}

const refPath = join( here, '..', '..', 'verify', 'scenes', 'out', `${scene}_scalar_w1.bin` );
const reference = existsSync( refPath ) ? recordingHashes( readFileSync( refPath ) ) : [];
const mismatch = firstMismatch( raw.hashes, reference );
const compared = Math.min( raw.hashes.length, reference.length );
console.log( JSON.stringify( {
	impl, scene, steps, workers, warmup, compared, mismatch, ...summarize( raw, warmup ),
	queryMs: raw.queryMs.map( ( x ) => x / steps ), queryHash: raw.queryHash.map( ( h ) => h.toString( 16 ).padStart( 8, '0' ) ),
	finalHash: raw.hashes[steps - 1],
} ) );
// process.exit can hang joining a busy V8 compile worker, so let the loop drain instead
process.exitCode = mismatch < 0 && compared > 0 ? 0 : 1;
