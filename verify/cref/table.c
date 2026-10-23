// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/table.as

#include "table.h"

#include "common.h"

static uint64_t keys[1 << 16];
static int keyCount;

// One line per occupied slot, so slot placement and backward shift deletion are compared exactly.
static void emitSet( const char* tag, const b3HashSet* set )
{
	vtag( tag );
	vu32( set->capacity );
	vu32( set->count );
	vi32( b3GetHashSetBytes( (b3HashSet*)set ) );
	vend();
	for ( uint32_t i = 0; i < set->capacity; ++i )
	{
		if ( set->items[i].hash != 0 || set->items[i].key != 0 )
		{
			vtag( "slot" );
			vu32( i );
			vu64( set->items[i].key );
			vu32( set->items[i].hash );
			vend();
		}
	}
}

static uint64_t makeKey( void )
{
	uint32_t kind = vrand32() % 4;
	if ( kind == 3 )
	{
		uint64_t key = vrand64();
		return key == 0 ? 1 : key;
	}

	// Pair keys from small increasing shape indices, as the broad phase makes them
	int s1 = vrandint( 0, 64 + keyCount / 4 );
	int s2 = vrandint( 0, 64 + keyCount / 4 );
	int c = kind == 0 ? vrandint( 0, 8 ) : 0;
	if ( s1 == s2 && c == 0 )
	{
		c = 1;
	}
	return b3ShapePairKey( s1, s2, c );
}

int main( void )
{
	vseed( 4 );

	int capacities[] = { -3, 0, 1, 16, 17, 31, 32, 33, 100, 1000 };
	for ( int i = 0; i < (int)( sizeof( capacities ) / sizeof( capacities[0] ) ); ++i )
	{
		b3HashSet set = b3CreateSet( capacities[i] );
		vtag( "cap" );
		vi32( capacities[i] );
		vu32( set.capacity );
		vend();
		b3DestroySet( &set );
	}

	int pairs[][3] = { { 0, 1, 0 },	  { 1, 0, 0 },		  { 5, 5, 3 }, { 4194303, 0, 1048575 },
					   { -1, 7, -1 }, { 100, 4194304, 2 } };
	for ( int i = 0; i < 6; ++i )
	{
		vtag( "pair" );
		vu64( b3ShapePairKey( pairs[i][0], pairs[i][1], pairs[i][2] ) );
		vend();
	}

	for ( int round = 0; round < 6; ++round )
	{
		b3HashSet set = b3CreateSet( vrandint( 0, 64 ) );
		keyCount = 0;
		emitSet( "create", &set );

		for ( int phase = 0; phase < 12; ++phase )
		{
			int adds = vrandint( 0, 400 );
			for ( int k = 0; k < adds; ++k )
			{
				uint64_t key = makeKey();
				if ( keyCount > 0 && vrand32() % 8 == 0 )
				{
					key = keys[vrandint( 0, keyCount )];
				}
				bool found = b3AddKey( &set, key );
				if ( keyCount < ( 1 << 16 ) )
				{
					keys[keyCount++] = key;
				}
				vtag( "add" );
				vu64( key );
				vu32( b3KeyHash( key ) );
				vbool( found );
				vend();
			}
			emitSet( "added", &set );

			int removes = keyCount > 0 ? vrandint( 0, adds / 2 + 2 ) : 0;
			for ( int k = 0; k < removes; ++k )
			{
				uint64_t key = vrand32() % 10 == 0 ? makeKey() : keys[vrandint( 0, keyCount )];
				bool found = b3RemoveKey( &set, key );
				vtag( "remove" );
				vu64( key );
				vbool( found );
				vend();
			}
			emitSet( "removed", &set );

			int queries = vrandint( 0, 100 );
			for ( int k = 0; k < queries; ++k )
			{
				uint64_t key =
					vrand32() % 3 == 0 || keyCount == 0 ? makeKey() : keys[vrandint( 0, keyCount )];
				vtag( "has" );
				vu64( key );
				vbool( b3ContainsKey( &set, key ) );
				vbool( b3ContainsHashedKey( &set, key, b3KeyHash( key ) ) );
				vend();
			}
		}

		if ( round % 2 == 1 )
		{
			b3ClearSet( &set );
			emitSet( "clear", &set );
			for ( int k = 0; k < 50; ++k )
			{
				uint64_t key = makeKey();
				vtag( "add" );
				vu64( key );
				vbool( b3AddKey( &set, key ) );
				vend();
			}
			emitSet( "refill", &set );
		}

		b3DestroySet( &set );
	}
	return 0;
}
