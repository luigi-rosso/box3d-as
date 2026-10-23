// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Vector output and seeded inputs for the C reference drivers. verify/as/common.as
// mirrors every function here, so both sides draw the same inputs and print the
// same tokens.

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int v_column;

static void v_separate( void )
{
	if ( v_column++ > 0 )
	{
		putchar( ' ' );
	}
}

static void vtag( const char* s )
{
	v_separate();
	fputs( s, stdout );
}

static void vi32( int32_t x )
{
	v_separate();
	printf( "%d", x );
}

static void vu32( uint32_t x )
{
	v_separate();
	printf( "%u", x );
}

static void vbool( bool x )
{
	vi32( x ? 1 : 0 );
}

static void vf32( float x )
{
	uint32_t bits;
	memcpy( &bits, &x, sizeof( bits ) );
	v_separate();
	printf( "%08x", bits );
}

static void vu64( uint64_t x )
{
	v_separate();
	printf( "%016llx", (unsigned long long)x );
}

static void vf64( double x )
{
	uint64_t bits;
	memcpy( &bits, &x, sizeof( bits ) );
	vu64( bits );
}

static void vend( void )
{
	putchar( '\n' );
	v_column = 0;
}

// splitmix64
static uint64_t v_state;

static void vseed( uint64_t seed )
{
	v_state = seed;
}

static uint64_t vrand64( void )
{
	v_state += 0x9e3779b97f4a7c15ull;
	uint64_t z = v_state;
	z = ( z ^ ( z >> 30 ) ) * 0xbf58476d1ce4e5b9ull;
	z = ( z ^ ( z >> 27 ) ) * 0x94d049bb133111ebull;
	return z ^ ( z >> 31 );
}

static uint32_t vrand32( void )
{
	return (uint32_t)( vrand64() >> 32 );
}

// [lo, hi)
static int vrandint( int lo, int hi )
{
	return lo + (int)( vrand32() % (uint32_t)( hi - lo ) );
}

// [lo, hi]
static float vrandf( float lo, float hi )
{
	float t = (float)( vrand32() >> 8 ) * ( 1.0f / 16777216.0f );
	return lo + t * ( hi - lo );
}

static float vbitsf( uint32_t bits )
{
	float x;
	memcpy( &x, &bits, sizeof( x ) );
	return x;
}

#define V_SPECIAL_COUNT 32

static const uint32_t v_specialBits[V_SPECIAL_COUNT] = {
	0x00000000, 0x80000000, 0x3f800000, 0xbf800000, 0x3f000000, 0xbf000000, 0x40000000, 0x40400000,
	0x00000001, 0x80000001, 0x007fffff, 0x807fffff, 0x00400000, 0x00800000, 0x80800000, 0x7f7fffff,
	0xff7fffff, 0x7f800000, 0xff800000, 0x7fc00000, 0xffc00000, 0x7fc12345, 0x7fa00001, 0xffa00001,
	0x7149f2ca, 0xf149f2ca, 0x0da24260, 0x8da24260, 0x34000000, 0x33800000, 0x4b800000, 0xcb800001,
};

// A quarter special values (signed zeros, denormals, infinities, NaNs with
// payloads, huge and tiny), a quarter raw bit patterns, half ordinary values.
static float vspecialf( void )
{
	uint32_t pick = vrand32() & 3;
	if ( pick == 0 )
	{
		return vbitsf( v_specialBits[vrand32() % V_SPECIAL_COUNT] );
	}

	if ( pick == 1 )
	{
		return vbitsf( vrand32() );
	}

	return vrandf( -100.0f, 100.0f );
}
