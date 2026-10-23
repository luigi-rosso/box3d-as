// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Runs verify/scenes/traj.as for one scene, writes a version 1 trajectory (format in
// notes/scenes.md) and diffs it against the native scalar recording.
//   node verify/traj.mjs <scene> [--steps=N] [--wasm=path] [--out=path] [--ref=path]
// Steps default to the reference's step count. Prints one RESULT line:
//   RESULT <scene> <steps compared> <pass|fail> <first divergence or ->

import { writeFileSync, existsSync } from 'node:fs';
import { join } from 'node:path';
import { loadWasm, verifyDir } from './lib.mjs';
import { readTrajectory, diff } from './scenes/dump.mjs';

const sceneNames = [ 'pyramid', 'large_pyramid', 'card_house', 'chains', 'capsule_mesh', 'heightfield', 'bullets',
	'mover', 'many_bodies', 'mesh_drop', 'village', 'queries' ];

const args = process.argv.slice( 2 );
const flags = Object.fromEntries( args.filter( ( a ) => a.startsWith( '--' ) ).map( ( a ) =>
{
	const [ k, v ] = a.slice( 2 ).split( '=' );
	return [ k, v === undefined ? true : v ];
} ) );
const scene = args.find( ( a ) => !a.startsWith( '--' ) );
const sceneIndex = sceneNames.indexOf( scene );
if ( sceneIndex < 0 )
{
	console.error( `usage: node traj.mjs <scene> [--steps=N]; scenes: ${sceneNames.join( ' ' )}` );
	process.exit( 2 );
}

const outDir = join( verifyDir, 'scenes', 'out' );
const refPath = flags.ref ?? join( outDir, `${scene}_scalar_w1.bin` );
const outPath = flags.out ?? join( outDir, 'as', `${scene}.bin` );
const wasmPath = flags.wasm ?? join( outDir, 'as', 'traj.wasm' );

const ref = existsSync( refPath ) ? readTrajectory( refPath ) : null;
const stepCount = flags.steps !== undefined ? Number( flags.steps ) : ref ? ref.header.stepCount : 300;

const { exports: e, frameBoundary } = loadWasm( wasmPath );
const workerCount = 1;
if ( !e.create( sceneIndex, workerCount ) )
{
	console.error( `create failed for ${scene}` );
	process.exit( 2 );
}
frameBoundary();

const header = new Uint32Array( 16 );
const headerF32 = new Float32Array( header.buffer );
header[0] = 0x52543342;
header[1] = 1;
const nameBytes = Buffer.alloc( 32 );
nameBytes.write( scene.slice( 0, 31 ) );
for ( let i = 0; i < 8; ++i )
{
	header[2 + i] = nameBytes.readUInt32LE( 4 * i );
}
header[10] = stepCount;
header[11] = workerCount;
headerF32[12] = Math.fround( 1 / 60 );
header[13] = 4;
header[14] = e.counterCount();
header[15] = e.moverWords();

const chunks = [ Buffer.from( header.buffer ) ];
let trapped = null;
let stepsRun = 0;
const t0 = Date.now();
for ( let i = 0; i < stepCount; ++i )
{
	try
	{
		e.step( i );
	}
	catch ( err )
	{
		trapped = `trap at step ${i}: ${err.message}`;
		break;
	}
	if ( e.recordError() !== 0 )
	{
		trapped = `record layout error ${e.recordError()} at step ${i}`;
		break;
	}
	const ptr = Number( e.recordPointer() );
	const len = e.recordLength();
	chunks.push( Buffer.from( new Uint8Array( e.memory.buffer, ptr, len * 4 ) ) );
	stepsRun += 1;
	frameBoundary();
}
const ms = Date.now() - t0;
writeFileSync( outPath, Buffer.concat( chunks ) );

let summary;
if ( ref === null )
{
	summary = `${scene}: no reference at ${refPath}`;
	console.log( summary );
	console.log( `RESULT ${scene} 0 fail no-reference` );
	process.exitCode = 1;
}
else
{
	report();
}

function report()
{
	const mine = readTrajectory( outPath );
	const result = diff( ref, mine, false, true );
	const compared = Math.min( ref.steps.length, mine.steps.length );

	// First divergence in step order across counters, bodies and mover
	let first = null;
	const consider = ( step, text ) =>
	{
		if ( first === null || step < first.step )
		{
			first = { step, text };
		}
	};
	if ( result.firstCounter )
	{
		const s = result.firstCounter.step;
		const si = ref.steps.findIndex( ( x ) => x.stepIndex === s );
		const k = [ ...Array( 51 ).keys() ].find( ( i ) => ref.u32[ref.steps[si].counterOffset + i] !== mine.u32[mine.steps[si].counterOffset + i] );
		consider( s, `counter ${result.firstCounter.counter} c ${ref.u32[ref.steps[si].counterOffset + k] | 0} as ${mine.u32[mine.steps[si].counterOffset + k] | 0}` );
	}
	if ( result.firstBody )
	{
		consider( result.firstBody.step, `body ${result.firstBody.body} ${result.firstBody.field}` );
	}
	if ( result.firstMover )
	{
		consider( result.firstMover.step, `mover ${result.firstMover.field}` );
	}

	const pass = result.identical && trapped === null && stepsRun === stepCount;
	console.log( `${scene}: ${stepsRun} steps run in ${ms} ms, ${compared} compared, ${result.differingSteps} differ` );
	if ( trapped )
	{
		console.log( `${scene}: ${trapped}` );
	}
	if ( !pass )
	{
		diff( ref, mine, false, false );
	}
	const firstText = first ? `step ${first.step} ${first.text}` : trapped ? trapped : '-';
	console.log( `RESULT ${scene} ${compared} ${pass ? 'pass' : 'fail'} ${firstText}` );
	// process.exit can hang joining a busy V8 compile worker, so let the loop drain instead
	process.exitCode = pass ? 0 : 1;
}
