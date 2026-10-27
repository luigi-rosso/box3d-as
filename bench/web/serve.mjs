// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// The static server the browser benches load their pages from: the repo root, with /harness/ mapped to a librive
// web scripting build when one is given. isolated adds the cross origin isolation headers a threads build needs.

import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { join, normalize, extname } from 'node:path';

const types = { '.html': 'text/html', '.mjs': 'text/javascript', '.js': 'text/javascript', '.wasm': 'application/wasm' };

// Resolves to { origin, close }
export async function serve( root, { harness, isolated = true } = {} )
{
	const isolation = isolated
		? { 'Cross-Origin-Opener-Policy': 'same-origin', 'Cross-Origin-Embedder-Policy': 'require-corp' }
		: {};
	const server = createServer( async ( req, res ) =>
	{
		const pathname = decodeURIComponent( new URL( req.url, 'http://x' ).pathname );
		const path = pathname.startsWith( '/harness/' ) && harness
			? normalize( join( harness, pathname.slice( 9 ) ) )
			: normalize( join( root, pathname ) );
		try
		{
			const body = await readFile( path );
			res.writeHead( 200, { 'Content-Type': types[extname( path )] ?? 'application/octet-stream', ...isolation,
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
	return { origin: `http://127.0.0.1:${server.address().port}`, close: () => server.close() };
}
