// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/container.as and lib/algorithm.as

#include "container.h"

#include "algorithm.h"
#include "common.h"

typedef struct Pair
{
	float a;
	int b;
} Pair;

b3DeclareArrayNative( Pair );

static void emitArrays( b3Array( int ) * ints, b3Array( Pair ) * pairs )
{
	vtag( "ints" );
	vi32( ints->count );
	for ( int i = 0; i < ints->count; ++i )
	{
		vi32( ints->data[i] );
	}
	vend();
	vtag( "pairs" );
	vi32( pairs->count );
	for ( int i = 0; i < pairs->count; ++i )
	{
		vf32( pairs->data[i].a );
		vi32( pairs->data[i].b );
	}
	vend();
}

int main( void )
{
	vseed( 6 );
	b3Array( int ) ints = { 0 };
	b3Array( Pair ) pairs = { 0 };
	int src[64];

	for ( int phase = 0; phase < 200; ++phase )
	{
		int ops = vrandint( 1, 20 );
		for ( int k = 0; k < ops; ++k )
		{
			uint32_t op = vrand32() % 6;
			if ( op == 0 )
			{
				int index = b3Array_AddIndex( ints );
				ints.data[index] = vrandint( -1000, 1000 );
				int p = b3Array_AddIndex( pairs );
				pairs.data[p] = (Pair){ vrandf( -1.0f, 1.0f ), index };
				vtag( "add" );
				vi32( index );
				vi32( p );
				vend();
			}
			else if ( op == 1 && ints.count > 0 )
			{
				int moved = b3Array_RemoveSwap( ints, vrandint( 0, ints.count ) );
				vtag( "rs" );
				vi32( moved );
				vend();
			}
			else if ( op == 2 && pairs.count > 0 )
			{
				int moved = b3Array_RemoveSwap( pairs, vrandint( 0, pairs.count ) );
				vtag( "rsp" );
				vi32( moved );
				vend();
			}
			else if ( op == 3 )
			{
				int n = vrandint( 0, 8 );
				for ( int i = 0; i < n; ++i )
				{
					src[i] = vrandint( 0, 50 );
				}
				b3Array_Append( ints, src, n );
			}
			else if ( op == 4 )
			{
				int oldCount = pairs.count;
				int n = vrandint( 0, oldCount + 4 );
				b3Array_Resize( pairs, n );
				for ( int i = oldCount; i < n; ++i )
				{
					pairs.data[i] = (Pair){ (float)i, -i };
				}
			}
			else if ( pairs.count > 1 )
			{
				int i = vrandint( 0, pairs.count );
				int j = vrandint( 0, pairs.count );
				B3_SWAP( pairs.data[i], pairs.data[j] );
			}
		}
		emitArrays( &ints, &pairs );
	}

	b3Array_Destroy( ints );
	b3Array_Destroy( pairs );
	return 0;
}
