// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// node determinism.mjs <dir> <scene> <workers...>: one table row per recording against scalar at 1 worker

import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { diff, readTrajectory } from './dump.mjs';

const [ dir, scene, ...workers ] = process.argv.slice( 2 );
const path = ( flavor, w ) => join( dir, `${scene}_${flavor}_w${w}.bin` );
const reference = readTrajectory( path( 'scalar', workers[0] ) );
const referenceBytes = readFileSync( reference.path );

for ( const flavor of [ 'scalar', 'simd' ] )
{
	for ( const w of workers )
	{
		if ( flavor === 'scalar' && w === workers[0] )
		{
			continue;
		}
		const other = readTrajectory( path( flavor, w ) );
		const result = diff( reference, other, false, true );
		const sameFile = referenceBytes.equals( readFileSync( other.path ) );
		let first = '';
		if ( result.firstBody !== null )
		{
			first = `body ${result.firstBody.body} ${result.firstBody.field} at step ${result.firstBody.step}`;
		}
		else if ( result.firstCounter !== null )
		{
			first = `${result.firstCounter.counter} at step ${result.firstCounter.step}`;
		}
		if ( result.firstMover !== null )
		{
			first += `${first ? '; ' : ''}mover ${result.firstMover.field} at step ${result.firstMover.step}`;
		}
		const verdict = result.identical ? ( sameFile ? 'identical file' : 'identical state' ) : 'differs';
		console.log( `| ${scene} | ${flavor} w${w} | ${verdict} | ${result.differingSteps}/${reference.steps.length} | ${first} |` );
	}
}
