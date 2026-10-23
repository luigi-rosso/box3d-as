// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/id_pool.as

#include "id_pool.h"

#include "common.h"

static int live[1 << 16];
static int liveCount;

static void emitPool( const b3IdPool* pool )
{
	vtag( "s" );
	vi32( b3GetIdCount( pool ) );
	vi32( b3GetIdCapacity( pool ) );
	vi32( b3GetIdBytes( pool ) );
	vi32( pool->freeArray.count );
	vend();

	vtag( "free" );
	for ( int i = 0; i < pool->freeArray.count; ++i )
	{
		vi32( pool->freeArray.data[i] );
	}
	vend();
}

int main( void )
{
	vseed( 2 );
	b3IdPool pool = b3CreateIdPool();
	emitPool( &pool );

	for ( int phase = 0; phase < 60; ++phase )
	{
		int ops = vrandint( 1, 300 );
		for ( int k = 0; k < ops; ++k )
		{
			uint32_t r = vrand32() % 5;
			if ( r < 3 || liveCount == 0 )
			{
				int id = b3AllocId( &pool );
				live[liveCount++] = id;
				vtag( "a" );
				vi32( id );
				vend();
			}
			else
			{
				int index = vrandint( 0, liveCount );
				int id = live[index];
				live[index] = live[--liveCount];
				b3FreeId( &pool, id );
				vtag( "f" );
				vi32( id );
				vend();
			}
		}

		if ( phase % 7 == 3 )
		{
			// The id == nextIndex branch
			b3FreeId( &pool, pool.nextIndex );
			vtag( "n" );
			vi32( pool.nextIndex );
			vend();
		}

		emitPool( &pool );
	}

	b3DestroyIdPool( &pool );
	emitPool( &pool );

	for ( int k = 0; k < 40; ++k )
	{
		int id = b3AllocId( &pool );
		vtag( "a" );
		vi32( id );
		vend();
		if ( k % 3 == 0 )
		{
			b3FreeId( &pool, id / 2 );
		}
	}
	emitPool( &pool );
	return 0;
}
