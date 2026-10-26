// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Self time by function from a node --cpu-prof profile, wasm functions by their name section names:
//   node bench/web/cpuprof.mjs <file.cpuprofile> [--top=30]

import { readFileSync } from 'node:fs';

const path = process.argv[2];
const top = Number( process.argv.find( ( a ) => a.startsWith( '--top=' ) )?.slice( 6 ) ?? 30 );
const profile = JSON.parse( readFileSync( path, 'utf8' ) );
const byId = new Map( profile.nodes.map( ( n ) => [ n.id, n ] ) );
const self = new Map();
const deltas = profile.timeDeltas;
let total = 0;
for ( let i = 0; i < profile.samples.length; ++i )
{
	const node = byId.get( profile.samples[i] );
	const name = node.callFrame.functionName || `(${node.callFrame.url.split( '/' ).pop() || 'anonymous'})`;
	const us = deltas[i] ?? 0;
	self.set( name, ( self.get( name ) ?? 0 ) + us );
	total += us;
}
const rows = [ ...self.entries() ].sort( ( a, b ) => b[1] - a[1] ).slice( 0, top );
console.log( `| function | self ms | self % |` );
console.log( `|---|---|---|` );
for ( const [ name, us ] of rows )
{
	console.log( `| \`${name}\` | ${( us / 1000 ).toFixed( 1 )} | ${( 100 * us / total ).toFixed( 1 )} |` );
}
console.log( `total ${( total / 1000 ).toFixed( 1 )} ms in ${profile.samples.length} samples` );
