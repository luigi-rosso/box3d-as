// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// The node bench's runs in headless Chrome, one fresh page per run, served cross origin isolated so the
// pthreads builds can run too. Writes a jsonl that bench.mjs --table= reads.
//   PLAYWRIGHT=<dir holding node_modules/playwright-core> ROUNDS=3 node bench/web/browser.mjs
// Same IMPLS, SCENES and WORKERS variables as bench.mjs; native runs are left out.

import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { appendFileSync, mkdirSync } from 'node:fs';
import { execFileSync } from 'node:child_process';
import { dirname, join, normalize, extname } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const here = dirname( fileURLToPath( import.meta.url ) );
const root = join( here, '..', '..' );
const { chromium } = await import( pathToFileURL( join( process.env.PLAYWRIGHT, 'node_modules', 'playwright-core', 'index.mjs' ) ) );

const types = { '.html': 'text/html', '.mjs': 'text/javascript', '.js': 'text/javascript', '.wasm': 'application/wasm' };
const server = createServer( async ( req, res ) =>
{
	const path = normalize( join( root, decodeURIComponent( new URL( req.url, 'http://x' ).pathname ) ) );
	try
	{
		const body = await readFile( path );
		res.writeHead( 200, { 'Content-Type': types[extname( path )] ?? 'application/octet-stream',
			'Cross-Origin-Opener-Policy': 'same-origin', 'Cross-Origin-Embedder-Policy': 'require-corp',
			'Cache-Control': 'no-store' } );
		res.end( body );
	}
	catch
	{
		res.writeHead( 404 );
		res.end();
	}
} );
await new Promise( ( resolve ) => server.listen( 0, '127.0.0.1', resolve ) );
const url = `http://127.0.0.1:${server.address().port}/bench/web/page.html`;

const browser = await chromium.launch( { channel: 'chrome', headless: true } );
const rounds = Number( process.env.ROUNDS ?? 3 );
const impls = ( process.env.IMPLS ?? 'c_simd c_scalar as_simd as_scalar' ).split( /\s+/ ).filter( Boolean );
const scenes = ( process.env.SCENES ??
	'large_pyramid:200 many_bodies:100 mesh_drop:120 village:120 plank_pile:300 chains:300 pyramid:300 queries:300' )
	.split( /\s+/ ).filter( Boolean );
const workers = ( process.env.WORKERS ?? '1' ).split( /\s+/ ).filter( Boolean );
mkdirSync( join( here, 'out' ), { recursive: true } );
const stamp = new Date().toISOString().replace( /[-:]/g, '' ).slice( 0, 15 );
const log = join( here, 'out', `chrome_${stamp}.jsonl` );
appendFileSync( log, JSON.stringify( { node: `Chrome ${browser.version()}`, v8: 'chrome', machine:
	execFileSync( 'sysctl', [ '-n', 'machdep.cpu.brand_string' ] ).toString().trim() } ) + '\n' );
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
				const load = Number( execFileSync( 'uptime' ).toString().match( /load averages?: ([\d.]+)/ )[1] );
				const page = await browser.newPage();
				let result;
				try
				{
					await page.goto( url );
					await page.waitForFunction( () => window.ready === true );
					result = await page.evaluate( ( [ i, s, n, k ] ) => window.runOne( i, s, n, k, Math.floor( n / 10 ) ),
						[ impl, scene, Number( steps ), Number( w ) ] );
				}
				catch ( err )
				{
					result = { impl, scene, workers: Number( w ), error: String( err ) };
				}
				await page.close();
				appendFileSync( log, JSON.stringify( { round: r, load, ...result } ) + '\n' );
				console.log( `round ${r} ${scene} ${impl} w${w} load ${load} median ${result.median?.toFixed( 3 )} ` +
					`${result.error ?? ( result.mismatch === -1 ? 'hash ok' : 'HASH MISMATCH ' + result.mismatch )}` );
			}
		}
	}
}
await browser.close();
server.close();
