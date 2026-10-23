// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/convex_manifold.as: sphere, capsule and hull pairs, the separating
// axis test and hull clipping, with the caches fed back across calls as the contact update does.
// With WIDE_REF defined (verify/cref/convex_manifold_wide.c) the hull phases run against a copy
// of convex_manifold.c and convex_manifold_wide.inl compiled with the lane loop for edge pairs.

#if defined( WIDE_REF )
	#define B3_SIMD_WIDTH 4
	#define b3CollideSpheres W_b3CollideSpheres
	#define b3CollideCapsuleAndSphere W_b3CollideCapsuleAndSphere
	#define b3CollideHullAndSphere W_b3CollideHullAndSphere
	#define b3CollideCapsules W_b3CollideCapsules
	#define b3CollideHullAndCapsule W_b3CollideHullAndCapsule
	#define b3CollideHulls W_b3CollideHulls
	#define b3CollideHullsAtWidth W_b3CollideHullsAtWidth
	#define b3ComputeSeparatingAxis W_b3ComputeSeparatingAxis
	#define b3ComputeSeparatingAxisAtWidth W_b3ComputeSeparatingAxisAtWidth
	#define b3ComputeSeparatingAxisW4 W_b3ComputeSeparatingAxisW4
	#include "../src/convex_manifold.c"
	#undef B3_SIMD_NONE
	#include "../src/convex_manifold_wide.inl"
	#define B3_SIMD_NONE
#endif

#include "manifold_common.h"

#include "math_internal.h"

#define STEP_COUNT 6

#if defined( WIDE_REF )

#include <stdio.h>

// The library build of the same functions, with the scalar edge pair loop, to count where the
// two loops part. Reported on stderr only, the AS side has no twin.
#pragma push_macro( "b3ComputeSeparatingAxis" )
#pragma push_macro( "b3CollideHulls" )
#undef b3ComputeSeparatingAxis
#undef b3CollideHulls
b3AxisQuery b3ComputeSeparatingAxis( const b3HullData* hullA, const b3HullData* hullB, b3Transform xfB, bool earlyReturn );
void b3CollideHulls( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,
					 b3Transform transformBtoA, b3SATCache* cache );

static b3AxisQuery scalarSeparatingAxis( const b3HullData* hullA, const b3HullData* hullB, b3Transform xf, bool earlyReturn )
{
	return b3ComputeSeparatingAxis( hullA, hullB, xf, earlyReturn );
}

static void scalarCollideHulls( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,
								b3Transform xf, b3SATCache* cache )
{
	b3CollideHulls( manifold, capacity, hullA, hullB, xf, cache );
}
#pragma pop_macro( "b3CollideHulls" )
#pragma pop_macro( "b3ComputeSeparatingAxis" )

static int g_queryCount, g_queryDiffs, g_collideCount, g_collideDiffs;
static b3LocalManifoldPoint g_scalarBuffer[POINT_BUFFER];

static bool sameManifold( const b3LocalManifold* a, const b3LocalManifold* b )
{
	b3LocalManifold c = *a;
	c.points = b->points;
	return memcmp( &c, b, sizeof( b3LocalManifold ) ) == 0 &&
		   memcmp( a->points, b->points, sizeof( b3LocalManifoldPoint ) * 8 ) == 0;
}

#endif

static void pcall( const char* tag, int step, int capacity, int offset, b3Transform xf )
{
	vtag( tag );
	vi32( step );
	vi32( capacity );
	vi32( offset );
	pxf( xf );
	vend();
}

static void runHulls( int phase, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		int ia = vrandint( 0, g_hullCount );
		int ib = vrandint( 0, g_hullCount );
		int mode = vrandint( 0, POSE_MODE_COUNT );
		const b3HullData* hullA = g_hulls[ia];
		const b3HullData* hullB = g_hulls[ib];
		b3Transform xf = hullPose( hullA, hullB, mode );

		vtag( "hh" );
		vi32( phase );
		vi32( i );
		vi32( ia );
		vi32( ib );
		vi32( mode );
		vend();

		b3AxisQuery early = b3ComputeSeparatingAxis( hullA, hullB, xf, true );
		vtag( "sat1" );
		pquery( early );
		vend();
		b3AxisQuery full = b3ComputeSeparatingAxis( hullA, hullB, xf, false );
		vtag( "sat0" );
		pquery( full );
		vend();

#if defined( WIDE_REF )
		b3AxisQuery scalarEarly = scalarSeparatingAxis( hullA, hullB, xf, true );
		b3AxisQuery scalarFull = scalarSeparatingAxis( hullA, hullB, xf, false );
		g_queryCount += 2;
		g_queryDiffs += memcmp( &scalarEarly, &early, sizeof( b3AxisQuery ) ) != 0;
		g_queryDiffs += memcmp( &scalarFull, &full, sizeof( b3AxisQuery ) ) != 0;
#endif

		b3SATCache cache = { 0 };
		for ( int step = 0; step < STEP_COUNT; ++step )
		{
			xf = advancePose( xf, step );
			int capacity = pickCapacity();
			int offset = pickOffset();
			int manual = vrandint( 0, 24 );
			if ( manual < 3 )
			{
				cache.type = (uint8_t)( b3_manualFaceAxisA + manual );
			}
			int width = ( vrand32() & 1 ) ? 4 : 8;

			resetManifold( offset );
			pcall( "hhc", step, capacity, offset, xf );
			vtag( "in" );
			vi32( width );
			psat( cache );
			vend();

#if defined( WIDE_REF )
			memcpy( g_scalarBuffer, g_pointBuffer + offset, sizeof( b3LocalManifoldPoint ) * ( POINT_BUFFER - offset ) );
			b3LocalManifold scalarManifold = g_manifold;
			scalarManifold.points = g_scalarBuffer;
			b3SATCache scalarCache = cache;
			scalarCollideHulls( &scalarManifold, capacity, hullA, hullB, xf, &scalarCache );
#endif

			if ( width == 4 )
			{
				b3CollideHulls( &g_manifold, capacity, hullA, hullB, xf, &cache );
			}
			else
			{
				b3CollideHullsAtWidth( &g_manifold, capacity, hullA, hullB, xf, &cache, width );
			}

			pmanifold( offset + capacity );
			vtag( "out" );
			psat( cache );
			vend();

#if defined( WIDE_REF )
			g_collideCount += 1;
			g_collideDiffs += sameManifold( &scalarManifold, &g_manifold ) == false ||
							  memcmp( &scalarCache, &cache, sizeof( b3SATCache ) ) != 0;
#endif
		}
	}
}

// The test_manifold.c style slide: two boxes moved along x through touch, cache fed back.
static void runSlide( int phase )
{
	for ( int k = 0; k < 6; ++k )
	{
		const b3HullData* hullA = g_hulls[k];
		const b3HullData* hullB = g_hulls[( k + 1 ) % 6];
		b3Quat q = k < 3 ? b3Quat_identity : rquatEdge();
		b3Vec3 u = axisDirection( 2 * ( k % 3 ) );
		float eA = hullExtent( hullA, b3Quat_identity, u );
		float eB = hullExtent( hullB, q, b3Neg( u ) );
		b3SATCache cache = { 0 };
		for ( int s = 0; s < 80; ++s )
		{
			float gap = -0.2f + 0.005f * (float)s;
			b3Transform xf;
			xf.q = q;
			xf.p = b3MulSV( eA + eB + gap, u );
			resetManifold( 0 );
			vtag( "slide" );
			vi32( phase );
			vi32( k );
			vi32( s );
			pxf( xf );
			psat( cache );
			vend();
			b3CollideHulls( &g_manifold, B3_MAX_MANIFOLD_POINTS, hullA, hullB, xf, &cache );
			pmanifold( B3_MAX_MANIFOLD_POINTS );
			vtag( "out" );
			psat( cache );
			vend();
		}
	}
}

#if !defined( WIDE_REF )

// A lerp that may leave [0, 1]
static b3Vec3 extend( b3Vec3 a, b3Vec3 b, float t )
{
	return b3MulAdd( a, t, b3Sub( b, a ) );
}

static b3Transform spherePose( b3Vec3 centerA, float extentA, b3Sphere sphereB, int mode )
{
	b3Transform xf;
	xf.q = rquatOrIdentity();
	if ( mode == 5 )
	{
		// The rotated center of B lands near center A
		b3Vec3 rotated = b3RotateVector( xf.q, sphereB.center );
		xf.p = b3Sub( centerA, rotated );
		return xf;
	}
	b3Vec3 u = runit();
	float gap = poseGap( mode );
	b3Vec3 rotated = b3RotateVector( xf.q, sphereB.center );
	b3Vec3 target = b3MulAdd( centerA, extentA + sphereB.radius + gap, u );
	xf.p = b3Sub( target, rotated );
	if ( mode == 7 )
	{
		xf.p = b3Add( xf.p, ( b3Vec3 ){ 900.0f, -1300.0f, 400.0f } );
	}
	return xf;
}

static void runSpheres( int count )
{
	for ( int i = 0; i < count; ++i )
	{
		b3Sphere sphereA = randomSphere();
		b3Sphere sphereB = randomSphere();
		int mode = vrandint( 0, POSE_MODE_COUNT );
		b3Transform xf = spherePose( sphereA.center, sphereA.radius, sphereB, mode );
		int capacity = vrandint( 0, 3 );
		int offset = pickOffset();

		resetManifold( offset );
		vtag( "ss" );
		vi32( i );
		vi32( mode );
		pv3( sphereA.center );
		vf32( sphereA.radius );
		pv3( sphereB.center );
		vf32( sphereB.radius );
		vend();
		pcall( "ssc", 0, capacity, offset, xf );
		b3CollideSpheres( &g_manifold, capacity, &sphereA, &sphereB, xf );
		pmanifold( offset + 2 );
	}
}

static void runCapsuleSphere( int count )
{
	for ( int i = 0; i < count; ++i )
	{
		b3Capsule capsuleA = randomCapsule();
		b3Sphere sphereB = randomSphere();
		int mode = vrandint( 0, POSE_MODE_COUNT );
		float t = vrandf( -0.3f, 1.3f );
		b3Vec3 onAxis = extend( capsuleA.center1, capsuleA.center2, t );
		b3Transform xf = spherePose( onAxis, capsuleA.radius, sphereB, mode );
		int capacity = vrandint( 0, 3 );
		int offset = pickOffset();

		resetManifold( offset );
		vtag( "cs" );
		vi32( i );
		vi32( mode );
		pv3( capsuleA.center1 );
		pv3( capsuleA.center2 );
		vf32( capsuleA.radius );
		pv3( sphereB.center );
		vf32( sphereB.radius );
		vend();
		pcall( "csc", 0, capacity, offset, xf );
		b3CollideCapsuleAndSphere( &g_manifold, capacity, &capsuleA, &sphereB, xf );
		pmanifold( offset + 2 );
	}
}

static void runCapsules( int count )
{
	for ( int i = 0; i < count; ++i )
	{
		b3Capsule capsuleA = randomCapsule();
		b3Capsule capsuleB = randomCapsule();
		int kind = vrandint( 0, 6 );
		b3Transform xf;

		if ( kind == 0 )
		{
			// Parallel: B along A's axis, offset sideways by the radii plus a gap
			b3Vec3 axisA = b3Normalize( b3Sub( capsuleA.center2, capsuleA.center1 ) );
			b3Vec3 axisB = b3Sub( capsuleB.center2, capsuleB.center1 );
			float lengthB = b3Length( axisB );
			b3Vec3 side = b3Normalize( b3Cross( axisA, runit() ) );
			float gap = vrandf( -0.05f, 0.05f );
			float slide = vrandf( -0.5f, 1.5f );
			float tilt = vrandf( -0.04f, 0.04f );
			b3Vec3 dirB = b3Normalize( b3MulAdd( axisA, tilt, side ) );
			b3Vec3 start = extend( capsuleA.center1, capsuleA.center2, slide );
			start = b3MulAdd( start, capsuleA.radius + capsuleB.radius + gap, side );
			capsuleB.center1 = b3Vec3_zero;
			capsuleB.center2 = b3MulSV( lengthB, dirB );
			xf.q = b3Quat_identity;
			xf.p = start;
		}
		else if ( kind == 1 )
		{
			// Coincident
			capsuleB = capsuleA;
			xf = b3Transform_identity;
		}
		else
		{
			int mode = vrandint( 0, POSE_MODE_COUNT );
			float t = vrandf( 0.0f, 1.0f );
			b3Sphere probe = { b3Lerp( capsuleB.center1, capsuleB.center2, t ), capsuleB.radius };
			float s = vrandf( -0.2f, 1.2f );
			b3Vec3 onAxis = extend( capsuleA.center1, capsuleA.center2, s );
			xf = spherePose( onAxis, capsuleA.radius, probe, mode );
		}

		int capacity = vrandint( 0, 5 );
		int offset = pickOffset();

		resetManifold( offset );
		vtag( "cc" );
		vi32( i );
		vi32( kind );
		pv3( capsuleA.center1 );
		pv3( capsuleA.center2 );
		vf32( capsuleA.radius );
		pv3( capsuleB.center1 );
		pv3( capsuleB.center2 );
		vf32( capsuleB.radius );
		vend();
		pcall( "ccc", 0, capacity, offset, xf );
		b3CollideCapsules( &g_manifold, capacity, &capsuleA, &capsuleB, xf );
		pmanifold( offset + 4 );
	}
}

static void runHullSphere( int count )
{
	for ( int i = 0; i < count; ++i )
	{
		int ia = vrandint( 0, g_hullCount );
		const b3HullData* hullA = g_hulls[ia];
		b3Sphere sphereB = randomSphere();
		int mode = vrandint( 0, POSE_MODE_COUNT );
		b3Transform xf;
		if ( mode == 5 )
		{
			// Deep inside
			xf.q = rquatOrIdentity();
			b3Vec3 inside = b3MulAdd( hullA->center, 0.1f, rv( -1.0f, 1.0f ) );
			xf.p = b3Sub( inside, b3RotateVector( xf.q, sphereB.center ) );
		}
		else
		{
			b3Vec3 u = poseDirection( mode );
			float eA = hullExtent( hullA, b3Quat_identity, u );
			xf = spherePose( b3MulSV( eA, u ), 0.0f, sphereB, mode );
		}

		vtag( "hs" );
		vi32( i );
		vi32( ia );
		vi32( mode );
		pv3( sphereB.center );
		vf32( sphereB.radius );
		vend();

		b3SimplexCache cache = { 0 };
		for ( int step = 0; step < STEP_COUNT; ++step )
		{
			xf = advancePose( xf, step );
			int capacity = vrandint( 0, 3 );
			int offset = pickOffset();
			resetManifold( offset );
			pcall( "hsc", step, capacity, offset, xf );
			vtag( "in" );
			psimplexCache( cache );
			vend();
			b3CollideHullAndSphere( &g_manifold, capacity, hullA, &sphereB, xf, &cache );
			pmanifold( offset + 2 );
			vtag( "out" );
			psimplexCache( cache );
			vend();
		}
	}
}

static void runHullCapsule( int count )
{
	for ( int i = 0; i < count; ++i )
	{
		int ia = vrandint( 0, g_hullCount );
		const b3HullData* hullA = g_hulls[ia];
		b3Capsule capsuleB = randomCapsule();
		int mode = vrandint( 0, POSE_MODE_COUNT );
		b3Transform xf;
		if ( mode == 5 )
		{
			xf.q = rquatOrIdentity();
			xf.p = b3Sub( hullA->center, b3RotateVector( xf.q, b3Lerp( capsuleB.center1, capsuleB.center2, 0.5f ) ) );
		}
		else
		{
			b3Quat q = poseRotation( mode );
			b3Vec3 u = poseDirection( mode );
			float gap = poseGap( mode );
			int lie = vrandint( 0, 3 );
			if ( lie == 0 )
			{
				// Lay the capsule flat on the support plane
				b3Vec3 axisB = b3RotateVector( q, b3Sub( capsuleB.center2, capsuleB.center1 ) );
				b3Vec3 flat = b3MulSub( axisB, b3Dot( axisB, u ), u );
				b3Vec3 local = b3InvRotateVector( q, flat );
				capsuleB.center2 = b3Add( capsuleB.center1, local );
			}
			float eA = hullExtent( hullA, b3Quat_identity, u );
			b3Vec3 down = b3Neg( u );
			float e1 = b3Dot( b3RotateVector( q, capsuleB.center1 ), down );
			float e2 = b3Dot( b3RotateVector( q, capsuleB.center2 ), down );
			float eB = b3MaxFloat( e1, e2 ) + capsuleB.radius;
			xf = placePose( eA, eB, u, q, gap, mode );
		}

		vtag( "hc" );
		vi32( i );
		vi32( ia );
		vi32( mode );
		pv3( capsuleB.center1 );
		pv3( capsuleB.center2 );
		vf32( capsuleB.radius );
		vend();

		b3SimplexCache cache = { 0 };
		for ( int step = 0; step < STEP_COUNT; ++step )
		{
			xf = advancePose( xf, step );
			int capacity = vrandint( 0, 5 );
			int offset = pickOffset();
			resetManifold( offset );
			pcall( "hcc", step, capacity, offset, xf );
			vtag( "in" );
			psimplexCache( cache );
			vend();
			b3CollideHullAndCapsule( &g_manifold, capacity, hullA, &capsuleB, xf, &cache );
			pmanifold( offset + 4 );
			vtag( "out" );
			psimplexCache( cache );
			vend();
		}
	}
}

#endif

int main( void )
{
	buildHulls();
	vseed( 0x636f6e766578ull );

#if !defined( WIDE_REF )
	runSpheres( 6000 );
	runCapsuleSphere( 6000 );
	runCapsules( 12000 );
	for ( int batch = 0; batch < 4; ++batch )
	{
		runHullSphere( 1000 );
	}
	for ( int batch = 0; batch < 6; ++batch )
	{
		runHullCapsule( 1000 );
	}
#endif

	runSlide( 0 );
	for ( int batch = 0; batch < 8; ++batch )
	{
		runHulls( batch, 500 );
	}

	b3SetLengthUnitsPerMeter( 0.37f );
	runSlide( 1 );
	runHulls( 100, 500 );
	b3SetLengthUnitsPerMeter( 1.0f );

#if defined( WIDE_REF )
	fprintf( stderr, "wide vs scalar edge loop: %d of %d queries and %d of %d hull collisions differ\n", g_queryDiffs,
			 g_queryCount, g_collideDiffs, g_collideCount );
#endif

	return 0;
}
