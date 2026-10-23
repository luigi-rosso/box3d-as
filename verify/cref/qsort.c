// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/qsort.as

#include "qsort.h"

#include "common.h"

typedef struct Item
{
	float key;
	int index;
} Item;

static Item items[2048];
static uint64_t pairKeys[2048];
static int lessCount;

// Ties and signed zeros compare equal, so the output order exposes the exact algorithm.
static void sortItems( int n )
{
	lessCount = 0;
#define LESS( i, j ) ( ++lessCount, items[(int)( i )].key < items[(int)( j )].key )
#define SWAP( i, j )                                                                               \
	do                                                                                             \
	{                                                                                              \
		Item tmp = items[(int)( i )];                                                              \
		items[(int)( i )] = items[(int)( j )];                                                     \
		items[(int)( j )] = tmp;                                                                   \
	}                                                                                              \
	while ( 0 )
	QSORT( n, LESS, SWAP );
#undef LESS
#undef SWAP
}

static void sortKeys( int n )
{
#define LESS( i, j ) ( pairKeys[(int)( i )] < pairKeys[(int)( j )] )
#define SWAP( i, j )                                                                               \
	do                                                                                             \
	{                                                                                              \
		uint64_t tmp_ = pairKeys[(int)( i )];                                                      \
		pairKeys[(int)( i )] = pairKeys[(int)( j )];                                               \
		pairKeys[(int)( j )] = tmp_;                                                               \
	}                                                                                              \
	while ( 0 )
	QSORT( n, LESS, SWAP );
#undef LESS
#undef SWAP
}

static void emitItems( const char* tag, int n )
{
	vtag( tag );
	vi32( n );
	vi32( lessCount );
	for ( int i = 0; i < n; ++i )
	{
		vi32( items[i].index );
		vf32( items[i].key );
	}
	vend();
}

int main( void )
{
	vseed( 5 );

	int sizes[] = { 0, 1, 2, 3, 4, 5, 15, 16, 17, 18, 31, 32, 33, 100, 257, 1000, 2048 };
	int sizeCount = (int)( sizeof( sizes ) / sizeof( sizes[0] ) );
	for ( int shape = 0; shape < 6; ++shape )
	{
		for ( int s = 0; s < sizeCount; ++s )
		{
			int n = sizes[s];
			int range = vrandint( 1, 2 * n + 2 );
			for ( int i = 0; i < n; ++i )
			{
				float key;
				if ( shape == 0 )
				{
					key = (float)vrandint( 0, range );
				}
				else if ( shape == 1 )
				{
					key = (float)i;
				}
				else if ( shape == 2 )
				{
					key = (float)( n - i );
				}
				else if ( shape == 3 )
				{
					key = 7.0f;
				}
				else if ( shape == 4 )
				{
					key = ( vrand32() & 1 ) ? 0.0f : -0.0f;
				}
				else
				{
					key = vrandf( -1.0f, 1.0f );
				}
				items[i].key = key;
				items[i].index = i;
			}

			Item copy[2048];
			memcpy( copy, items, n * sizeof( Item ) );
			sortItems( n );
			emitItems( "closure", n );
			memcpy( items, copy, n * sizeof( Item ) );
			sortItems( n );
			emitItems( "sorter", n );
		}
	}

	for ( int round = 0; round < 50; ++round )
	{
		int n = vrandint( 0, 600 );
		for ( int i = 0; i < n; ++i )
		{
			pairKeys[i] = vrand32() % 4 == 0 && i > 0 ? pairKeys[vrandint( 0, i )]
													  : vrand64() >> ( vrand32() % 64 );
		}
		sortKeys( n );
		vtag( "keys" );
		vi32( n );
		for ( int i = 0; i < n; ++i )
		{
			vu64( pairKeys[i] );
		}
		vend();
	}
	return 0;
}
