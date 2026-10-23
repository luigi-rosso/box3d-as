// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/name_cache.as: b3Hash32 over UTF-8 names, add and find.

#include "common.h"

#include "name_cache.h"

#include <string.h>

#define MAX_NAME 64

// Code points the AS side builds its string from; written here as UTF-8.
static int rname( char* buffer )
{
	int length = vrandint( 0, 24 );
	int n = 0;
	for ( int i = 0; i < length; ++i )
	{
		int kind = vrandint( 0, 8 );
		uint32_t c;
		if ( kind == 0 )
		{
			c = (uint32_t)vrandint( 0x80, 0x800 );
		}
		else if ( kind == 1 )
		{
			c = (uint32_t)vrandint( 0x800, 0xD800 );
		}
		else
		{
			c = (uint32_t)vrandint( 0x20, 0x7F );
		}

		if ( c < 0x80 )
		{
			buffer[n++] = (char)c;
		}
		else if ( c < 0x800 )
		{
			buffer[n++] = (char)( 0xC0 | ( c >> 6 ) );
			buffer[n++] = (char)( 0x80 | ( c & 0x3F ) );
		}
		else
		{
			buffer[n++] = (char)( 0xE0 | ( c >> 12 ) );
			buffer[n++] = (char)( 0x80 | ( ( c >> 6 ) & 0x3F ) );
			buffer[n++] = (char)( 0x80 | ( c & 0x3F ) );
		}
	}
	buffer[n] = 0;
	return n;
}

int main( void )
{
	vseed( 0x6e616d6573ull );

	b3NameCache cache = b3CreateNameCache();
	uint32_t ids[256];
	int idCount = 0;

	char buffer[3 * MAX_NAME + 1];
	for ( int i = 0; i < 3000; ++i )
	{
		int byteCount = rname( buffer );

		vtag( "hash" );
		vi32( byteCount );
		vu32( b3Hash32( buffer, (size_t)byteCount ) );
		vend();

		uint32_t id = b3AddName( &cache, buffer );
		if ( idCount < 256 )
		{
			ids[idCount++] = id;
		}

		vtag( "add" );
		vu32( id );
		const char* found = b3FindName( &cache, id );
		vi32( found == NULL ? -1 : (int)strlen( found ) );
		vbool( found != NULL && strcmp( found, buffer ) == 0 );
		vend();

		// Adding again returns the same id
		if ( ( vrand32() & 7 ) == 0 )
		{
			vtag( "again" );
			vu32( b3AddName( &cache, buffer ) );
			vend();
		}
	}

	for ( int i = 0; i < 500; ++i )
	{
		uint32_t id = ( vrand32() & 1 ) != 0 ? ids[vrandint( 0, idCount )] : vrand32();
		const char* found = b3FindName( &cache, id );
		vtag( "find" );
		vu32( id );
		vi32( found == NULL ? -1 : (int)strlen( found ) );
		vend();
	}

	b3DestroyNameCache( &cache );
	return 0;
}
