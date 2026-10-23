// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/bitset.as

#include "bitset.h"

#include "common.h"

// Every block up to capacity, so stale blocks past the count are compared too.
static void emitSet( const char* tag, b3BitSet* set )
{
	vtag( tag );
	vu32( set->blockCapacity );
	vu32( set->blockCount );
	vi32( b3GetBitSetBytes( set ) );
	vi32( b3CountSetBits( set ) );
	for ( uint32_t i = 0; i < set->blockCapacity; ++i )
	{
		vu64( set->bits[i] );
	}
	vend();
}

int main( void )
{
	vseed( 3 );

	for ( int round = 0; round < 40; ++round )
	{
		uint32_t capacity = vrand32() % 700;
		b3BitSet a = b3CreateBitSet( capacity );
		b3BitSet b = b3CreateBitSet( vrand32() % 300 );
		emitSet( "create", &a );

		uint32_t bitCount = vrand32() % 900;
		b3SetBitCountAndClear( &a, bitCount );
		b3SetBitCountAndClear( &b, bitCount );
		emitSet( "count", &a );

		for ( int phase = 0; phase < 8; ++phase )
		{
			int ops = vrandint( 1, 120 );
			for ( int k = 0; k < ops; ++k )
			{
				uint32_t op = vrand32() % 6;
				uint32_t limit = a.blockCount * 64 + 200;
				uint32_t bit = vrand32() % limit;
				if ( op == 0 && a.blockCount > 0 )
				{
					uint32_t inside = vrand32() % ( a.blockCount * 64 );
					b3SetBit( &a, inside );
				}
				else if ( op == 1 )
				{
					b3SetBitGrow( &a, bit );
				}
				else if ( op == 2 )
				{
					b3ClearBit( &a, bit );
				}
				else if ( op == 3 )
				{
					vtag( "get" );
					vu32( bit );
					vbool( b3GetBit( &a, bit ) );
					vend();
				}
				else if ( op == 4 )
				{
					b3SetBitGrow( &b, vrand32() % ( a.blockCount * 64 + 64 ) );
				}
				else
				{
					b3ClearBit( &b, bit );
				}
			}
			emitSet( "a", &a );
			emitSet( "b", &b );

			if ( phase == 4 )
			{
				b3GrowBitSet( &a, a.blockCount + 1 + vrand32() % 5 );
				emitSet( "grow", &a );
			}

			if ( b.blockCount < a.blockCount )
			{
				b3GrowBitSet( &b, a.blockCount );
			}
			if ( a.blockCount < b.blockCount )
			{
				b3GrowBitSet( &a, b.blockCount );
			}
			b3InPlaceUnion( &a, &b );
			emitSet( "union", &a );
		}

		b3SetBitCountAndClear( &a, vrand32() % 2000 );
		emitSet( "recount", &a );

		b3DestroyBitSet( &a );
		b3DestroyBitSet( &b );
		emitSet( "destroy", &a );
	}
	return 0;
}
