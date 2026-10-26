// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Runs a bench/threads bake on rive's web runtime under node: librive's web scripting harness (the rive repo's
// packages/runtime/tests/web_scripting, built with --web_threads) loads the riv as a page would, then frames run
// until the script logs its BENCH RESULT. Prints the script's log lines as they come.
//   node bench/web/lanes.mjs <web_scripting_harness.mjs> <file.riv> [--seconds=120]

import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';

const [ harnessPath, rivPath ] = process.argv.slice( 2 ).filter( ( a ) => !a.startsWith( '--' ) );
const seconds = Number( process.argv.find( ( a ) => a.startsWith( '--seconds=' ) )?.slice( 10 ) ?? 120 );

let done = false;
const print = ( text ) =>
{
	console.log( text );
	if ( text.includes( 'BENCH RESULT' ) )
	{
		done = true;
	}
};
const createHarness = ( await import( pathToFileURL( resolve( harnessPath ) ) ) ).default;
const Module = await createHarness( { print, printErr: print } );

const bytes = readFileSync( rivPath );
const ptr = Module._malloc( bytes.length );
Module.HEAPU8.set( bytes, ptr );
if ( !Module._harness_import( ptr, bytes.length ) )
{
	console.error( 'import failed' );
	process.exit( 1 );
}
Module._free( ptr );
await Module.riveScripting.prepare();
Module._harness_run( 1 );

// Yield between frames so the pthread workers' messages run, as a page's event loop does
const deadline = Date.now() + seconds * 1000;
while ( !done && Date.now() < deadline )
{
	await new Promise( ( settle ) => setTimeout( settle, 0 ) );
	Module._harness_frames( 1 );
}
if ( !done )
{
	console.log( `LANES TIMEOUT after ${seconds} s` );
}
// Releasing a file whose lanes still run can wait on them, and the result is already out
process.exit( done ? 0 : 1 );
