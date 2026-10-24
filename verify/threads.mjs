// Compares the per step state hashes a threaded rive-cli run of bench/threads logged (--define=HASHLOG) with the
// native recording's, which the run cannot write itself: lanes run only inside rive-cli, where node cannot follow.
//   node verify/threads.mjs <recording.bin> <run.log> <label>
// Prints one RESULT line: RESULT <label> <steps compared> <pass|fail> <first divergence or ->
import fs from 'fs';

const [file, logFile, label] = process.argv.slice( 2 );
const buf = fs.readFileSync( file );
const w = new Uint32Array( buf.buffer, buf.byteOffset, buf.length >> 2 );
const counters = w[14], moverWords = w[15];

// bench/as/main.as's stateHash over each recorded step
const native = [];
for ( let i = 16; i < w.length; )
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
	native.push( h );
	i += n * 17 + moverWords;
}

const run = new Map();
for ( const line of fs.readFileSync( logFile, 'utf8' ).split( '\n' ) )
{
	const m = line.match( /HASH step=(\d+) hash=(\d+)/ );
	if ( m )
	{
		run.set( Number( m[1] ), Number( m[2] ) );
	}
}
const problems = fs.readFileSync( logFile, 'utf8' ).split( '\n' ).filter( ( l ) => /trapped|failed|gave up| error /.test( l ) );

let compared = 0;
let divergence = '-';
for ( let step = 0; step < native.length; ++step )
{
	if ( !run.has( step ) )
	{
		divergence = `step ${step} missing`;
		break;
	}
	if ( run.get( step ) !== native[step] )
	{
		divergence = `step ${step} hash ${run.get( step )} native ${native[step]}`;
		break;
	}
	compared += 1;
}
if ( problems.length > 0 && divergence === '-' )
{
	divergence = problems[0].replace( /^.*?\] /, '' ).slice( 0, 120 );
}
const pass = compared === native.length && problems.length === 0;
console.log( `RESULT ${label} ${compared} ${pass ? 'pass' : 'fail'} ${divergence}` );
process.exit( pass ? 0 : 1 );
