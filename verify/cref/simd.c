// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/simd.as: every scalar lane op over specials and seeded draws.
// C leaves the order of initializer and argument evaluation open, so every draw is its own
// statement.

// The width wide4.c compiles the wide solver and SAT at
#define B3_SIMD_WIDTH 4

#include "simd.h"

#include "common.h"
#include "simd_wide.h"

#define N 3000

static b3FloatW4 randW( void )
{
	b3FloatW4 r;
	r.x = vspecialf();
	r.y = vspecialf();
	r.z = vspecialf();
	r.w = vspecialf();
	return r;
}

// Masks and logic inputs: mostly 0 and 1, sometimes any special.
static float maskf( void )
{
	uint32_t pick = vrand32() % 4;
	if ( pick == 0 )
	{
		return vspecialf();
	}
	return ( pick & 1 ) ? 1.0f : 0.0f;
}

static b3FloatW4 randMask( void )
{
	b3FloatW4 r;
	r.x = maskf();
	r.y = maskf();
	r.z = maskf();
	r.w = maskf();
	return r;
}

// Geometry inputs: mostly ordinary values so tests go both ways, sometimes specials.
static float geomf( void )
{
	if ( vrand32() % 8 == 0 )
	{
		return vspecialf();
	}
	return vrandf( -10.0f, 10.0f );
}

static b3V32 randV32( void )
{
	b3V32 v;
	v.x = geomf();
	v.y = geomf();
	v.z = geomf();
	return v;
}

static b3Vec3 randVec3( void )
{
	b3Vec3 v;
	v.x = geomf();
	v.y = geomf();
	v.z = geomf();
	return v;
}

static b3AABB randAABB( void )
{
	b3AABB a;
	a.lowerBound = randVec3();
	b3Vec3 extent;
	extent.x = vrandf( 0.0f, 8.0f );
	extent.y = vrandf( 0.0f, 8.0f );
	extent.z = vrandf( 0.0f, 8.0f );
	a.upperBound = b3Add( a.lowerBound, extent );
	return a;
}

static b3Vec3W4 randVec3W( void )
{
	b3Vec3W4 v;
	v.X = randW();
	v.Y = randW();
	v.Z = randW();
	return v;
}

static void emitW( b3FloatW4 a )
{
	vf32( a.x );
	vf32( a.y );
	vf32( a.z );
	vf32( a.w );
}

static void emitV32( b3V32 a )
{
	vf32( a.x );
	vf32( a.y );
	vf32( a.z );
}

static void emitVec3( b3Vec3 a )
{
	vf32( a.x );
	vf32( a.y );
	vf32( a.z );
}

static void emitAABB( b3AABB a )
{
	emitVec3( a.lowerBound );
	emitVec3( a.upperBound );
}

static void emitVec3W( b3Vec3W4 a )
{
	emitW( a.X );
	emitW( a.Y );
	emitW( a.Z );
}

typedef b3FloatW4 ( *UnaryW )( b3FloatW4 );
typedef b3FloatW4 ( *BinaryW )( b3FloatW4, b3FloatW4 );

static b3FloatW4 negW( b3FloatW4 a )
{
	return b3NegW( a );
}
static b3FloatW4 absW( b3FloatW4 a )
{
	return b3AbsW( a );
}
static b3FloatW4 sqrtW( b3FloatW4 a )
{
	return b3SqrtW( a );
}
static b3FloatW4 addW( b3FloatW4 a, b3FloatW4 b )
{
	return b3AddW( a, b );
}
static b3FloatW4 subW( b3FloatW4 a, b3FloatW4 b )
{
	return b3SubW( a, b );
}
static b3FloatW4 mulW( b3FloatW4 a, b3FloatW4 b )
{
	return b3MulW( a, b );
}
static b3FloatW4 divW( b3FloatW4 a, b3FloatW4 b )
{
	return b3DivW( a, b );
}
static b3FloatW4 minW( b3FloatW4 a, b3FloatW4 b )
{
	return b3MinW( a, b );
}
static b3FloatW4 maxW( b3FloatW4 a, b3FloatW4 b )
{
	return b3MaxW( a, b );
}
static b3FloatW4 symClampW( b3FloatW4 a, b3FloatW4 b )
{
	return b3SymClampW( a, b );
}
static b3FloatW4 andW( b3FloatW4 a, b3FloatW4 b )
{
	return b3AndW( a, b );
}
static b3FloatW4 orW( b3FloatW4 a, b3FloatW4 b )
{
	return b3OrW( a, b );
}
static b3FloatW4 andNotW( b3FloatW4 a, b3FloatW4 b )
{
	return b3AndNotW( a, b );
}
static b3FloatW4 greaterW( b3FloatW4 a, b3FloatW4 b )
{
	return b3GreaterThanW( a, b );
}
static b3FloatW4 lessW( b3FloatW4 a, b3FloatW4 b )
{
	return b3LessThanW( a, b );
}
static b3FloatW4 equalsW( b3FloatW4 a, b3FloatW4 b )
{
	return b3EqualsW( a, b );
}

static void sweepUnary( const char* tag, UnaryW f )
{
	for ( int i = 0; i < N; ++i )
	{
		b3FloatW4 a = randW();
		vtag( tag );
		emitW( a );
		emitW( f( a ) );
		vend();
	}
}

static void sweepBinary( const char* tag, BinaryW f, bool masks )
{
	for ( int i = 0; i < N; ++i )
	{
		b3FloatW4 a = masks ? randMask() : randW();
		b3FloatW4 b = masks ? randMask() : randW();
		if ( vrand32() % 8 == 0 )
		{
			b = a;
		}
		vtag( tag );
		emitW( a );
		emitW( b );
		emitW( f( a, b ) );
		vend();
	}
}

int main( void )
{
	vseed( 7 );

	vtag( "zero" );
	emitW( b3ZeroW() );
	vend();

	for ( int i = 0; i < 64; ++i )
	{
		float s = vspecialf();
		vtag( "splat" );
		emitW( b3SplatW( s ) );
		vend();
		float a = vspecialf();
		float b = vspecialf();
		float c = vspecialf();
		float d = vspecialf();
		vtag( "set" );
		emitW( b3SetW( a, b, c, d ) );
		vend();
	}

	for ( int i = 0; i < 200; ++i )
	{
		float data[16];
		float out[16] = { 0 };
		for ( int k = 0; k < 16; ++k )
		{
			data[k] = vspecialf();
		}
		int index = vrandint( 0, 13 );
		b3FloatW4 v = b3LoadW( data + index );
		b3StoreW( out + vrandint( 0, 13 ), v );
		vtag( "loadstore" );
		vi32( index );
		emitW( v );
		for ( int k = 0; k < 16; ++k )
		{
			vf32( out[k] );
		}
		vend();
	}

	sweepUnary( "neg", negW );
	sweepUnary( "abs", absW );
	sweepUnary( "sqrt", sqrtW );
	sweepBinary( "add", addW, false );
	sweepBinary( "sub", subW, false );
	sweepBinary( "mul", mulW, false );
	sweepBinary( "div", divW, false );
	sweepBinary( "min", minW, false );
	sweepBinary( "max", maxW, false );
	sweepBinary( "symclamp", symClampW, false );
	sweepBinary( "and", andW, true );
	sweepBinary( "or", orW, true );
	sweepBinary( "andnot", andNotW, true );
	sweepBinary( "gt", greaterW, false );
	sweepBinary( "lt", lessW, false );
	sweepBinary( "eq", equalsW, false );

	for ( int i = 0; i < N; ++i )
	{
		b3FloatW4 a = randW();
		b3FloatW4 b = randW();
		b3FloatW4 c = randW();
		vtag( "muladd" );
		emitW( a );
		emitW( b );
		emitW( c );
		emitW( b3MulAddW( a, b, c ) );
		vend();
	}

	for ( int i = 0; i < N; ++i )
	{
		b3FloatW4 m = randMask();
		b3FloatW4 a = randW();
		b3FloatW4 b = randW();
		vtag( "mask" );
		emitW( m );
		vbool( b3AllZeroW( m ) );
		vbool( b3AnyTrueW( m ) );
		vi32( b3MoveMaskW4( m ) );
		emitW( b3BlendW( a, b, m ) );
		vend();
	}

	for ( int i = 0; i < N; ++i )
	{
		int indexA[4];
		int indexB[4];
		for ( int k = 0; k < 4; ++k )
		{
			indexA[k] = vrandint( -1, 3 );
			indexB[k] = vrandint( -1, 3 );
		}
		vtag( "softmask" );
		for ( int k = 0; k < 4; ++k )
		{
			vi32( indexA[k] );
			vi32( indexB[k] );
		}
		emitW( b3SoftMaskW( indexA, indexB ) );
		vend();
	}

	for ( int i = 0; i < N; ++i )
	{
		b3FloatW4 a = randW();
		int base = vrandint( 0, 128 );
		int bits = i % 2 == 0 ? 7 : vrandint( 1, 24 );
		b3FloatW4 e = b3EmbedIndexW( a, base, bits );
		vtag( "embed" );
		emitW( a );
		vi32( base );
		vi32( bits );
		emitW( e );
		vi32( b3MinIndexW( e, bits ) );
		vi32( b3MinIndexW( a, bits ) );
		vend();
	}

	for ( int i = 0; i < N / 4; ++i )
	{
		b3FloatW4 r0 = randW();
		b3FloatW4 r1 = randW();
		b3FloatW4 r2 = randW();
		b3FloatW4 r3 = randW();
		b3FloatW4 c0, c1, c2, c3;
		b3TransposeW( r0, r1, r2, r3, &c0, &c1, &c2, &c3 );
		vtag( "transpose" );
		emitW( c0 );
		emitW( c1 );
		emitW( c2 );
		emitW( c3 );
		vend();

		// Aliased outputs, as the contact solver's pad lanes are
		b3FloatW4 pad = b3ZeroW();
		b3TransposeW( r3, r2, r1, r0, &c0, &pad, &c1, &pad );
		vtag( "transpose2" );
		emitW( c0 );
		emitW( c1 );
		emitW( pad );
		vend();
	}

	for ( int i = 0; i < N; ++i )
	{
		b3AABB a = randAABB();
		b3AABB b = randAABB();
		if ( vrand32() % 16 == 0 )
		{
			a.upperBound.x = vspecialf();
		}
		b3TreeNode nodes[2] = { 0 };
		nodes[0].aabb = a;
		nodes[1].aabb = b;
		b3AABBV av = b3LoadAABBV( &a );
		b3AABB stored = randAABB();
		bool condition = vrand32() % 2 == 0;
		b3StoreAABBV( &stored, b3UnionAABBV( av, b3LoadAABBV( &b ) ), condition );
		vtag( "aabb" );
		emitAABB( a );
		emitAABB( b );
		vbool( b3OverlapAABBV( av, b3LoadAABBV( &b ) ) );
		vbool( b3OverlapNode( av, nodes + 1 ) );
		vbool( b3OverlapV( &a, &b ) );
		emitAABB( b3UnionPairV( nodes ) );
		emitAABB( b3UnionV( a, b ) );
		vbool( condition );
		emitAABB( stored );
		vend();
	}

	for ( int i = 0; i < N; ++i )
	{
		b3V32 a = randV32();
		b3V32 b = randV32();
		if ( vrand32() % 8 == 0 )
		{
			b = a;
		}
		int index = vrandint( 0, 3 );
		float s = geomf();
		vtag( "v32" );
		emitV32( a );
		emitV32( b );
		emitV32( b3AddV( a, b ) );
		emitV32( b3SubV( a, b ) );
		emitV32( b3MulV( a, b ) );
		emitV32( b3DivV( a, b ) );
		emitV32( b3NegV( a ) );
		emitV32( b3AbsV( a ) );
		emitV32( b3MinV( a, b ) );
		emitV32( b3MaxV( a, b ) );
		emitV32( b3CrossV( a, b ) );
		emitV32( b3ModifiedCrossV( a, b ) );
		emitV32( b3SplatV( s ) );
		emitV32( b3ZeroV() );
		vf32( b3GetXV( a ) );
		vf32( b3GetYV( a ) );
		vf32( b3GetZV( a ) );
		vi32( index );
		vf32( b3GetV( a, index ) );
		vbool( b3AnyLess3V( a, b ) );
		vbool( b3AnyLessEq3V( a, b ) );
		vbool( b3AnyGreater3V( a, b ) );
		vbool( b3AllLessEq3V( a, b ) );
		vend();

		b3Vec3 p = randVec3();
		vtag( "loadv" );
		emitV32( b3LoadV( &p.x ) );
		vend();
	}

	for ( int i = 0; i < N; ++i )
	{
		b3Vec3W4 a = randVec3W();
		b3Vec3W4 b = randVec3W();
		b3FloatW4 s = randW();
		b3Vec3 v = randVec3();
		vtag( "vec3w" );
		emitVec3W( a );
		emitVec3W( b );
		emitW( s );
		emitVec3W( b3SplatVW( v ) );
		emitVec3W( b3NegVW( a ) );
		emitVec3W( b3MulSVW( s, a ) );
		emitVec3W( b3MulSubSVW( a, s, b ) );
		emitVec3W( b3MulAddSVW( a, s, b ) );
		emitVec3W( b3SubVW( a, b ) );
		emitVec3W( b3AddVW( a, b ) );
		emitW( b3DotW( a, b ) );
		emitVec3W( b3CrossW( a, b ) );
		vend();
	}

	for ( int i = 0; i < 200; ++i )
	{
		float x[12];
		float y[12];
		float z[12];
		float ox[12] = { 0 };
		float oy[12] = { 0 };
		float oz[12] = { 0 };
		for ( int k = 0; k < 12; ++k )
		{
			x[k] = vspecialf();
			y[k] = vspecialf();
			z[k] = vspecialf();
		}
		int xi = vrandint( 0, 9 );
		int yi = vrandint( 0, 9 );
		int zi = vrandint( 0, 9 );
		b3Vec3W4 v = b3LoadVW( x + xi, y + yi, z + zi );
		b3StoreVW( ox + zi, oy + xi, oz + yi, v );
		vtag( "loadstorev" );
		emitVec3W( v );
		for ( int k = 0; k < 12; ++k )
		{
			vf32( ox[k] );
			vf32( oy[k] );
			vf32( oz[k] );
		}
		vend();
	}

	for ( int i = 0; i < 4 * N; ++i )
	{
		b3V32 min1 = randV32();
		b3V32 max1 = b3AddV( min1, b3SplatV( vrandf( 0.0f, 6.0f ) ) );
		b3V32 min2 = randV32();
		b3V32 max2 = b3AddV( min2, b3SplatV( vrandf( 0.0f, 6.0f ) ) );
		b3V32 start = randV32();
		b3V32 delta = randV32();
		b3V32 v1 = randV32();
		b3V32 v2 = randV32();
		b3V32 v3 = randV32();
		if ( i % 3 == 0 )
		{
			// Triangles near the box so the SAT axes all get exercised
			v1 = b3AddV( min1, b3MulV( b3_halfV, v1 ) );
			v2 = b3AddV( min1, b3MulV( b3_halfV, v2 ) );
			v3 = b3AddV( min1, b3MulV( b3_halfV, v3 ) );
		}
		b3V32 center = b3MulV( b3_halfV, b3AddV( min1, max1 ) );
		b3V32 extent = b3SubV( max1, center );
		vtag( "bounds" );
		emitV32( min1 );
		emitV32( max1 );
		emitV32( min2 );
		emitV32( max2 );
		emitV32( start );
		emitV32( delta );
		emitV32( v1 );
		emitV32( v2 );
		emitV32( v3 );
		vbool( b3TestBoundsOverlap( min1, max1, min2, max2 ) );
		vbool( b3TestBoundsRayOverlap( min1, max1, start, delta ) );
		vbool( b3TestBoundsTriangleOverlap( center, extent, v1, v2, v3 ) );
		vf32( b3IntersectRayTriangle( start, delta, v1, v2, v3 ) );
		vf32( b3IntersectRayTriangle( start, b3SubV( v1, start ), v1, v2, v3 ) );
		vf32( b3IntersectRayTriangle( start, b3SubV( b3MulV( b3_halfV, b3AddV( v2, v3 ) ), start ),
									  v1, v2, v3 ) );
		b3V32 centroid = b3MulV( b3SplatV( 0.333333f ), b3AddV( v1, b3AddV( v2, v3 ) ) );
		vf32( b3IntersectRayTriangle( start, b3MulV( b3SplatV( 2.0f ), b3SubV( centroid, start ) ),
									  v1, v2, v3 ) );
		vf32( b3IntersectRayTriangle( start, b3MulV( b3SplatV( 2.0f ), b3SubV( centroid, start ) ),
									  v1, v3, v2 ) );
		vend();
	}

	vtag( "width" );
	vi32( b3GetSIMDWidth() );
	b3SetSIMDWidth( 0 );
	vi32( b3GetSIMDWidth() );
	b3SetSIMDWidth( 8 );
	vi32( b3GetSIMDWidth() );
	b3SetSIMDWidth( 4 );
	vi32( b3GetSIMDWidth() );
	vend();
	return 0;
}
