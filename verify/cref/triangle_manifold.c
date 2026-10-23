// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/triangle_manifold.as: triangle versus sphere, capsule and hull,
// with the simplex and SAT caches fed back across calls and every edge flag combination.

#include "manifold_common.h"

#include "math_internal.h"

#define STEP_COUNT 6

static b3Vec3 g_triangle[3];
static b3Vec3 g_world[3];

static void ptri( const b3Vec3* t )
{
	pv3( t[0] );
	pv3( t[1] );
	pv3( t[2] );
}

// kinds: 0 random, 1 near equilateral, 2 sliver, 3 large, 4 tiny, 5 axis aligned right triangle
static int randomTriangle( b3Vec3* t )
{
	int kind = vrandint( 0, 6 );
	switch ( kind )
	{
		case 0:
			t[0] = rv( -1.0f, 1.0f );
			t[1] = rv( -1.0f, 1.0f );
			t[2] = rv( -1.0f, 1.0f );
			break;

		case 1:
		{
			b3Vec3 center = rv( -0.5f, 0.5f );
			b3Vec3 axis = runit();
			b3Vec3 tangent = b3Normalize( b3Cross( axis, runit() ) );
			b3Vec3 bitangent = b3Cross( axis, tangent );
			float radius = vrandf( 0.2f, 2.0f );
			for ( int k = 0; k < 3; ++k )
			{
				float jitter = vrandf( -0.1f, 0.1f );
				float angle = ( 2.0f / 3.0f ) * B3_PI * (float)k + jitter;
				b3CosSin cs = b3ComputeCosSin( angle );
				b3Vec3 p = b3MulAdd( center, radius * cs.cosine, tangent );
				t[k] = b3MulAdd( p, radius * cs.sine, bitangent );
			}
		}
		break;

		case 2:
		{
			t[0] = rv( -1.0f, 1.0f );
			b3Vec3 d = runit();
			float length = vrandf( 0.5f, 2.0f );
			t[1] = b3MulAdd( t[0], length, d );
			float s = vrandf( 0.1f, 0.9f );
			b3Vec3 off = runit();
			t[2] = b3MulAdd( b3Lerp( t[0], t[1], s ), 0.01f, off );
		}
		break;

		case 3:
			t[0] = rv( -10.0f, 10.0f );
			t[1] = rv( -10.0f, 10.0f );
			t[2] = rv( -10.0f, 10.0f );
			break;

		case 4:
			t[0] = rv( -0.05f, 0.05f );
			t[1] = rv( -0.05f, 0.05f );
			t[2] = rv( -0.05f, 0.05f );
			break;

		default:
		{
			float s = vrandf( 0.5f, 3.0f );
			t[0] = b3Vec3_zero;
			t[1] = ( b3Vec3 ){ 0.0f, 0.0f, s };
			t[2] = ( b3Vec3 ){ s, 0.0f, 0.0f };
		}
		break;
	}
	return kind;
}

static b3Vec3 triangleNormal( const b3Vec3* t )
{
	return b3Normalize( b3Cross( b3Sub( t[1], t[0] ), b3Sub( t[2], t[0] ) ) );
}

// A point in the triangle plane, sometimes outside the triangle.
static b3Vec3 trianglePoint( const b3Vec3* t )
{
	float b1 = vrandf( -0.3f, 1.3f );
	float b2 = vrandf( -0.3f, 1.3f );
	if ( b1 + b2 > 1.6f )
	{
		b1 = 1.0f - b1;
		b2 = 1.0f - b2;
	}
	b3Vec3 p = b3MulAdd( t[0], b1, b3Sub( t[1], t[0] ) );
	return b3MulAdd( p, b2, b3Sub( t[2], t[0] ) );
}

static float triangleGap( int mode )
{
	if ( mode == 5 )
	{
		// Below the plane
		return vrandf( -1.5f, -0.2f );
	}
	return poseGap( mode );
}

static int edgeFlags( int combo )
{
	int flags = 0;
	if ( combo & 1 )
	{
		flags |= b3_concaveEdge1;
	}
	if ( combo & 2 )
	{
		flags |= b3_concaveEdge2;
	}
	if ( combo & 4 )
	{
		flags |= b3_concaveEdge3;
	}
	if ( combo & 8 )
	{
		flags |= b3_inverseConcaveEdge1;
	}
	if ( combo & 16 )
	{
		flags |= b3_inverseConcaveEdge2;
	}
	if ( combo & 32 )
	{
		flags |= b3_inverseConcaveEdge3;
	}
	return flags;
}

static void runSpheres( int count )
{
	for ( int i = 0; i < count; ++i )
	{
		int kind = randomTriangle( g_triangle );
		b3Vec3 n = triangleNormal( g_triangle );
		b3Vec3 p = trianglePoint( g_triangle );
		int mode = vrandint( 0, POSE_MODE_COUNT );
		b3Sphere sphere;
		sphere.radius = vrandf( 0.0f, 0.6f );
		float gap = triangleGap( mode );
		sphere.center = b3MulAdd( p, sphere.radius + gap, n );
		if ( mode == 7 )
		{
			// Center on the triangle
			sphere.center = p;
		}
		int capacity = vrandint( 0, 3 );
		int offset = pickOffset();

		resetManifold( offset );
		vtag( "ts" );
		vi32( i );
		vi32( kind );
		vi32( mode );
		vi32( capacity );
		vi32( offset );
		ptri( g_triangle );
		pv3( sphere.center );
		vf32( sphere.radius );
		vend();
		b3CollideTriangleAndSphere( &g_manifold, capacity, g_triangle, &sphere );
		pmanifold( offset + 2 );
	}
}

static void runCapsules( int count )
{
	for ( int i = 0; i < count; ++i )
	{
		int kind = randomTriangle( g_triangle );
		b3Vec3 n = triangleNormal( g_triangle );
		b3Vec3 p = trianglePoint( g_triangle );
		int mode = vrandint( 0, POSE_MODE_COUNT );
		int pose = vrandint( 0, 5 );
		b3Capsule capsule;
		capsule.radius = vrandf( 0.02f, 0.5f );
		float length = vrandf( 0.0f, 2.0f );
		if ( vrandint( 0, 16 ) == 0 )
		{
			// A point capsule leaves every edge axis undefined
			length = 0.0f;
		}
		float gap = triangleGap( mode );
		b3Vec3 axis;
		if ( pose == 0 )
		{
			// Lying on the plane
			axis = b3Normalize( b3Cross( n, runit() ) );
		}
		else if ( pose == 1 )
		{
			// Standing
			axis = n;
		}
		else if ( pose == 2 )
		{
			// Across a triangle edge
			int e = vrandint( 0, 3 );
			b3Vec3 edge = b3Sub( g_triangle[( e + 1 ) % 3], g_triangle[e] );
			b3Vec3 side = b3Normalize( b3Cross( edge, n ) );
			axis = b3Normalize( b3MulAdd( side, vrandf( -0.5f, 0.5f ), n ) );
			p = b3Lerp( g_triangle[e], g_triangle[( e + 1 ) % 3], vrandf( 0.0f, 1.0f ) );
			p = b3MulSub( p, 0.5f * length, axis );
		}
		else if ( pose == 3 )
		{
			axis = runit();
		}
		else
		{
			// Parallel to a triangle edge
			int e = vrandint( 0, 3 );
			axis = b3Normalize( b3Sub( g_triangle[( e + 1 ) % 3], g_triangle[e] ) );
		}
		capsule.center1 = b3MulAdd( p, capsule.radius + gap, n );
		capsule.center2 = b3MulAdd( capsule.center1, length, axis );
		if ( mode == 7 )
		{
			capsule.center1 = b3MulSub( p, 0.5f * length, axis );
			capsule.center2 = b3MulAdd( p, 0.5f * length, axis );
		}

		vtag( "tc" );
		vi32( i );
		vi32( kind );
		vi32( mode );
		vi32( pose );
		ptri( g_triangle );
		vend();

		b3SimplexCache cache = { 0 };
		for ( int step = 0; step < STEP_COUNT; ++step )
		{
			if ( step > 0 )
			{
				int jump = vrandint( 0, 8 );
				float scale = jump == 0 ? 0.1f : 0.005f;
				b3Vec3 delta = rv( -scale, scale );
				capsule.center1 = b3Add( capsule.center1, delta );
				b3Vec3 delta2 = rv( -scale, scale );
				capsule.center2 = b3Add( capsule.center2, delta2 );
			}
			int capacity = vrandint( 0, 5 );
			int offset = pickOffset();
			resetManifold( offset );
			vtag( "tcc" );
			vi32( step );
			vi32( capacity );
			vi32( offset );
			pv3( capsule.center1 );
			pv3( capsule.center2 );
			vf32( capsule.radius );
			psimplexCache( cache );
			vend();
			b3CollideTriangleAndCapsule( &g_manifold, capacity, g_triangle, &capsule, &cache );
			pmanifold( offset + 4 );
			vtag( "out" );
			psimplexCache( cache );
			vend();
		}
	}
}

static void runHulls( int phase, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		int kind = randomTriangle( g_world );
		b3Vec3 n = triangleNormal( g_world );
		b3Vec3 p = trianglePoint( g_world );
		int ih = vrandint( 0, g_hullCount );
		const b3HullData* hull = g_hulls[ih];
		int mode = vrandint( 0, POSE_MODE_COUNT + 1 );
		b3Transform xf;
		if ( mode == POSE_MODE_COUNT )
		{
			// Hull center near the plane, for the back side hysteresis
			xf.q = rquatOrIdentity();
			float height = vrandf( -0.02f, 0.005f );
			b3Vec3 center = b3MulAdd( p, height, n );
			xf.p = b3Sub( center, b3RotateVector( xf.q, hull->center ) );
		}
		else
		{
			b3Quat q = poseRotation( mode );
			float gap = triangleGap( mode );
			float eB = hullExtent( hull, q, b3Neg( n ) );
			xf.q = q;
			xf.p = b3MulAdd( p, eB + gap, n );
			if ( mode == 7 )
			{
				xf.p = b3Sub( p, b3RotateVector( q, hull->center ) );
			}
		}

		vtag( "th" );
		vi32( phase );
		vi32( i );
		vi32( kind );
		vi32( ih );
		vi32( mode );
		ptri( g_world );
		vend();

		b3SATCache cache = { 0 };
		for ( int step = 0; step < STEP_COUNT; ++step )
		{
			xf = advancePose( xf, step );
			for ( int k = 0; k < 3; ++k )
			{
				g_triangle[k] = b3InvTransformPoint( xf, g_world[k] );
			}

			int capacity = pickCapacity();
			int offset = pickOffset();
			int combo = ( 7 * i + step ) & 63;
			int flags = edgeFlags( combo );
			bool enableSpeculative = vrandint( 0, 4 ) != 0;
			int manual = vrandint( 0, 24 );
			if ( manual < 3 )
			{
				cache.type = (uint8_t)( b3_manualFaceAxisA + manual );
			}

			resetManifold( offset );
			vtag( "thc" );
			vi32( step );
			vi32( capacity );
			vi32( offset );
			vi32( flags );
			vbool( enableSpeculative );
			ptri( g_triangle );
			psat( cache );
			vend();
			b3CollideTriangleAndHull( &g_manifold, capacity, g_triangle[0], g_triangle[1], g_triangle[2], flags, hull, &cache,
									  enableSpeculative );
			pmanifold( offset + capacity );
			vtag( "out" );
			psat( cache );
			vend();
		}
	}
}

int main( void )
{
	buildHulls();
	vseed( 0x747269616e676c65ull );

	runSpheres( 20000 );
	for ( int batch = 0; batch < 4; ++batch )
	{
		runCapsules( 2000 );
	}
	for ( int batch = 0; batch < 10; ++batch )
	{
		runHulls( batch, 1000 );
	}

	b3SetLengthUnitsPerMeter( 0.37f );
	runHulls( 100, 1000 );
	runCapsules( 1000 );
	b3SetLengthUnitsPerMeter( 1.0f );

	return 0;
}
