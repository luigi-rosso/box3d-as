// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/distance.as: support, GJK with warm starts and debug
// simplexes, shape casts, sweeps and time of impact.

#include "distance_common.h"

#include "math_internal.h"

#include "box3d/constants.h"

#include <float.h>

#define SIMPLEX_CAPACITY 40

static void pcache( const b3SimplexCache* c )
{
	vf32( c->metric );
	vi32( c->count );
	for ( int i = 0; i < 4; ++i )
	{
		vi32( c->indexA[i] );
	}
	for ( int i = 0; i < 4; ++i )
	{
		vi32( c->indexB[i] );
	}
}

static void pdist( b3DistanceOutput o )
{
	pv3( o.pointA );
	pv3( o.pointB );
	pv3( o.normal );
	vf32( o.distance );
	vi32( o.iterations );
	vi32( o.simplexCount );
}

static void psimplex( const b3Simplex* s )
{
	vi32( s->count );
	for ( int i = 0; i < 4; ++i )
	{
		const b3SimplexVertex* v = s->vertices + i;
		pv3( v->wA );
		pv3( v->wB );
		pv3( v->w );
		vf32( v->a );
		vi32( v->indexA );
		vi32( v->indexB );
	}
}

static void ptoi( b3TOIOutput o )
{
	vi32( o.state );
	pv3( o.point );
	pv3( o.normal );
	vf32( o.fraction );
	vf32( o.distance );
	vi32( o.distanceIterations );
	vi32( o.pushBackIterations );
	vi32( o.rootIterations );
	vbool( o.usedFallback );
}

static b3Vec3 rposition( void )
{
	int mode = vrandint( 0, 5 );
	if ( mode == 0 )
	{
		return rv( -1.0f, 1.0f );
	}
	if ( mode == 1 )
	{
		return rv( -0.05f, 0.05f );
	}
	if ( mode == 2 )
	{
		b3Vec3 p = rv( -3.0f, 3.0f );
		return b3Add( p, ( b3Vec3 ){ 900.0f, -1300.0f, 400.0f } );
	}
	return rv( -3.0f, 3.0f );
}

static b3Transform rxf( void )
{
	b3Transform xf;
	xf.p = rposition();
	xf.q = rquatOrIdentity();
	return xf;
}

static TestProxy g_a, g_b;
static b3Simplex g_simplexes[SIMPLEX_CAPACITY];

static void runSupport( void )
{
	for ( int i = 0; i < 3000; ++i )
	{
		int kind = vrandint( 0, PROXY_KIND_COUNT );
		makeTestProxy( &g_a, kind );
		b3ShapeProxy proxy = testProxy( &g_a );

		b3Vec3 axis;
		if ( vrand32() & 1 )
		{
			axis = rv( -1.0f, 1.0f );
		}
		else
		{
			axis.x = vspecialf();
			axis.y = vspecialf();
			axis.z = vspecialf();
		}

		vtag( "sup" );
		vi32( i );
		pproxy( &g_a );
		pv3( axis );
		vi32( b3GetProxySupport( &proxy, axis ) );
		vi32( b3GetPointSupport( g_a.points, g_a.count, axis ) );
		vend();
	}
}

static void runDistance( int count )
{
	for ( int i = 0; i < count; ++i )
	{
		int kindA = vrandint( 0, PROXY_KIND_COUNT );
		makeTestProxy( &g_a, kindA );
		int kindB = vrandint( 0, PROXY_KIND_COUNT );
		makeTestProxy( &g_b, kindB );

		b3DistanceInput input;
		input.proxyA = testProxy( &g_a );
		input.proxyB = testProxy( &g_b );
		input.transform = rxf();
		input.useRadii = ( vrand32() & 1 ) != 0;

		int simplexMode = vrandint( 0, 3 );
		b3Simplex* simplexes = simplexMode == 0 ? NULL : g_simplexes;
		int capacity = simplexMode == 0 ? 0 : ( simplexMode == 1 ? SIMPLEX_CAPACITY : 3 );

		vtag( "din" );
		vi32( i );
		pproxy( &g_a );
		pproxy( &g_b );
		pxf( input.transform );
		vbool( input.useRadii );
		vi32( capacity );
		vend();

		b3SimplexCache cache = { 0 };
		for ( int w = 0; w < 5; ++w )
		{
			b3DistanceOutput out = b3ShapeDistance( &input, &cache, simplexes, capacity );

			vtag( "d" );
			vi32( i );
			vi32( w );
			pdist( out );
			pcache( &cache );
			vend();

			if ( simplexes != NULL )
			{
				for ( int k = 0; k < out.simplexCount; ++k )
				{
					vtag( "s" );
					vi32( k );
					psimplex( simplexes + k );
					vend();
				}
			}

			// Warm start: same pose, then small steps, then a jump
			if ( w == 1 || w == 2 )
			{
				b3Vec3 dp = rv( -0.05f, 0.05f );
				input.transform.p = b3Add( input.transform.p, dp );
				input.transform.q = rotateBy( input.transform.q, 0.1f );
			}
			else if ( w == 3 )
			{
				input.transform = rxf();
			}
		}
	}
}

static void runCast( int count )
{
	float linearSlop = B3_LINEAR_SLOP;
	static const float gaps[7] = { 0.25f, 0.75f, 1.0f, 1.2f, 1.5f, 2.5f, 5.0f };

	for ( int i = 0; i < count; ++i )
	{
		int kindA = vrandint( 0, PROXY_KIND_COUNT );
		makeTestProxy( &g_a, kindA );
		int kindB = vrandint( 0, PROXY_KIND_COUNT );
		makeTestProxy( &g_b, kindB );

		b3ShapeCastPairInput input;
		input.proxyA = testProxy( &g_a );
		input.proxyB = testProxy( &g_b );
		input.transform.p = rv( -4.0f, 4.0f );
		input.transform.q = rquatOrIdentity();

		b3Vec3 aim = rv( -1.0f, 1.0f );
		float scale = vrandf( 0.2f, 2.5f );
		input.translationB = b3MulSV( scale, b3Sub( aim, input.transform.p ) );

		int mode = vrandint( 0, 6 );
		if ( mode == 1 )
		{
			input.translationB = rv( -6.0f, 6.0f );
		}
		else if ( mode == 2 )
		{
			input.transform.p = rv( -0.3f, 0.3f );
		}
		else if ( mode == 3 || mode == 4 )
		{
			b3DistanceInput di;
			di.proxyA = input.proxyA;
			di.proxyB = input.proxyB;
			di.transform = input.transform;
			di.useRadii = false;
			b3SimplexCache cache = { 0 };
			b3DistanceOutput d = b3ShapeDistance( &di, &cache, NULL, 0 );

			if ( mode == 3 )
			{
				// Start near touching
				int pick = vrandint( 0, 7 );
				float desired = input.proxyA.radius + input.proxyB.radius + gaps[pick] * linearSlop;
				input.transform.p = b3MulAdd( input.transform.p, desired - d.distance, d.normal );
			}
			else
			{
				// Graze: keep only the tangential part
				float along = b3Dot( input.translationB, d.normal );
				input.translationB = b3MulSub( input.translationB, along, d.normal );
			}
		}
		else if ( mode == 5 )
		{
			input.translationB = b3Vec3_zero;
		}

		int fractionMode = vrandint( 0, 4 );
		input.maxFraction = fractionMode < 2 ? 1.0f : ( fractionMode == 2 ? vrandf( 0.0f, 1.0f ) : vrandf( 0.0f, 0.01f ) );
		input.canEncroach = ( vrand32() & 1 ) != 0;

		b3CastOutput out = b3ShapeCast( &input );

		vtag( "c" );
		vi32( i );
		vi32( mode );
		pproxy( &g_a );
		pproxy( &g_b );
		pxf( input.transform );
		pv3( input.translationB );
		vf32( input.maxFraction );
		vbool( input.canEncroach );
		pcast( out );
		vend();
	}
}

static void runSweep( void )
{
	for ( int i = 0; i < 2000; ++i )
	{
		b3Sweep sweep;
		sweep.localCenter = rv( -1.0f, 1.0f );
		sweep.c1 = rv( -10.0f, 10.0f );
		sweep.c2 = rv( -10.0f, 10.0f );
		sweep.q1 = rquat();
		sweep.q2 = rotateBy( sweep.q1, 3.0f );
		int mode = vrandint( 0, 4 );
		float time = mode == 0 ? 0.0f : ( mode == 1 ? 1.0f : vrandf( 0.0f, 1.0f ) );

		vtag( "sw" );
		vi32( i );
		vf32( time );
		pxf( b3GetSweepTransform( &sweep, time ) );
		vend();
	}
}

static b3Sweep rsweepA( void )
{
	b3Sweep s;
	s.localCenter = rv( -0.2f, 0.2f );
	s.c1 = rv( -0.5f, 0.5f );
	s.c2 = s.c1;
	if ( vrand32() & 1 )
	{
		b3Vec3 d = rv( -1.0f, 1.0f );
		s.c2 = b3Add( s.c1, d );
	}
	s.q1 = rquatOrIdentity();
	s.q2 = rotateBy( s.q1, 0.5f );
	return s;
}

static void runToi( int count, int forcedMode )
{
	static const float angles[4] = { 0.0f, 0.2f, 1.0f, 3.0f };

	for ( int i = 0; i < count; ++i )
	{
		int mode = forcedMode >= 0 ? forcedMode : vrandint( 0, 6 );

		// Mode 5 rotates a segment fast around a triangle vertex, where progress can stall
		int kindA = mode == 5 ? 2 : vrandint( 0, PROXY_KIND_COUNT );
		makeTestProxy( &g_a, kindA );
		int kindB = mode == 5 ? 1 : vrandint( 0, PROXY_KIND_COUNT );
		makeTestProxy( &g_b, kindB );

		b3TOIInput input;
		input.proxyA = testProxy( &g_a );
		input.proxyB = testProxy( &g_b );
		input.sweepA = rsweepA();

		b3Sweep sb;
		sb.localCenter = rv( -0.2f, 0.2f );
		sb.c1 = rv( -5.0f, 5.0f );
		b3Vec3 aim = rv( -1.0f, 1.0f );
		float scale = vrandf( 1.0f, 2.0f );
		sb.c2 = b3Add( sb.c1, b3MulSV( scale, b3Sub( aim, sb.c1 ) ) );
		sb.q1 = rquatOrIdentity();
		int pick = vrandint( 0, 4 );
		sb.q2 = rotateBy( sb.q1, angles[pick] );

		if ( mode == 1 )
		{
			// Mostly misses
			sb.c2 = rv( -5.0f, 5.0f );
		}
		else if ( mode == 2 )
		{
			// Start overlapped
			sb.c1 = rv( -0.2f, 0.2f );
		}
		else if ( mode == 3 )
		{
			// Fast rotation in place near A
			b3Vec3 dir = runit();
			float dist = vrandf( 0.5f, 2.5f );
			sb.c1 = b3MulSV( dist, dir );
			sb.c2 = sb.c1;
			sb.q2 = rotateBy( sb.q1, 3.0f );
		}
		else if ( mode == 4 )
		{
			// Translate far in one step, tunneling candidates
			sb.c1 = rv( -20.0f, -10.0f );
			sb.c2 = rv( 10.0f, 20.0f );
			b3Vec3 offset = rv( -0.5f, 0.5f );
			sb.c2 = b3Add( b3Neg( sb.c1 ), offset );
		}
		else if ( mode == 5 )
		{
			input.sweepA = ( b3Sweep ){ 0 };
			input.sweepA.q1 = b3Quat_identity;
			input.sweepA.q2 = b3Quat_identity;
			sb.localCenter = b3Vec3_zero;
			int v = vrandint( 0, g_a.count );
			b3Vec3 offset = rv( -0.3f, 0.3f );
			sb.c1 = b3Add( g_a.points[v], offset );
			b3Vec3 move = rv( -0.5f, 0.5f );
			sb.c2 = b3Add( sb.c1, move );
			sb.q1 = rquat();
			sb.q2 = rotateBy( sb.q1, 3.1f );
		}
		input.sweepB = sb;
		input.maxFraction = ( vrand32() & 1 ) ? 1.0f : vrandf( 0.1f, 1.0f );

		b3TOIOutput out = b3TimeOfImpact( &input );

		vtag( "t" );
		vi32( i );
		vi32( mode );
		pproxy( &g_a );
		pproxy( &g_b );
		pv3( input.sweepB.c1 );
		pv3( input.sweepB.c2 );
		pq( input.sweepB.q2 );
		vf32( input.maxFraction );
		ptoi( out );
		vend();
	}
}

static void fixedDistance( int id, const b3Vec3* pa, int ca, float ra, const b3Vec3* pb, int cb, float rb, b3Transform xf )
{
	for ( int useRadii = 0; useRadii < 2; ++useRadii )
	{
		b3DistanceInput input;
		input.proxyA = ( b3ShapeProxy ){ pa, ca, ra };
		input.proxyB = ( b3ShapeProxy ){ pb, cb, rb };
		input.transform = xf;
		input.useRadii = useRadii == 1;

		b3SimplexCache cache = { 0 };
		for ( int w = 0; w < 2; ++w )
		{
			b3DistanceOutput out = b3ShapeDistance( &input, &cache, g_simplexes, SIMPLEX_CAPACITY );
			vtag( "fd" );
			vi32( id );
			vi32( useRadii );
			vi32( w );
			pdist( out );
			pcache( &cache );
			vend();
			for ( int k = 0; k < out.simplexCount; ++k )
			{
				vtag( "s" );
				vi32( k );
				psimplex( g_simplexes + k );
				vend();
			}
		}
	}
}

static void runFixed( void )
{
	b3Vec3 origin[1] = { { 0.0f, 0.0f, 0.0f } };
	b3Vec3 corner[1] = { { 1.0f, 1.0f, 1.0f } };
	b3Vec3 segX[2] = { { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } };
	b3Vec3 segY[2] = { { 0.0f, -1.0f, 0.0f }, { 0.0f, 1.0f, 0.0f } };
	b3Vec3 tri[3] = { { -1.0f, -1.0f, 0.0f }, { 1.0f, -1.0f, 0.0f }, { 0.0f, 1.0f, 0.0f } };
	b3Vec3 box[8];
	for ( int i = 0; i < 8; ++i )
	{
		box[i].x = ( i & 1 ) ? 1.0f : -1.0f;
		box[i].y = ( i & 2 ) ? 1.0f : -1.0f;
		box[i].z = ( i & 4 ) ? 1.0f : -1.0f;
	}

	b3Transform id = b3Transform_identity;
	b3Transform faceTouch = { { 2.0f, 0.0f, 0.0f }, b3Quat_identity };
	b3Transform faceGap = { { 2.5f, 0.0f, 0.0f }, b3Quat_identity };
	b3Transform deep = { { 0.5f, 0.25f, 0.0f }, b3Quat_identity };
	b3Transform edgeTouch = { { 2.0f, 2.0f, 0.0f }, b3Quat_identity };
	b3Transform cornerTouch = { { 2.0f, 2.0f, 2.0f }, b3Quat_identity };
	b3Transform turned = { { 0.0f, 0.0f, 1.0f }, b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI ) };

	fixedDistance( 0, origin, 1, 0.0f, origin, 1, 0.0f, id );
	fixedDistance( 1, origin, 1, 0.5f, origin, 1, 0.25f, faceTouch );
	fixedDistance( 2, segX, 2, 0.0f, origin, 1, 0.0f, id );
	fixedDistance( 3, segX, 2, 0.1f, segY, 2, 0.1f, id );
	fixedDistance( 4, segX, 2, 0.0f, segX, 2, 0.0f, turned );
	fixedDistance( 5, tri, 3, 0.0f, origin, 1, 0.0f, id );
	fixedDistance( 6, tri, 3, 0.0f, segY, 2, 0.0f, turned );
	fixedDistance( 7, box, 8, 0.0f, box, 8, 0.0f, faceTouch );
	fixedDistance( 8, box, 8, 0.0f, box, 8, 0.0f, faceGap );
	fixedDistance( 9, box, 8, 0.0f, box, 8, 0.0f, deep );
	fixedDistance( 10, box, 8, 0.0f, box, 8, 0.0f, edgeTouch );
	fixedDistance( 11, box, 8, 0.0f, box, 8, 0.0f, cornerTouch );
	fixedDistance( 12, box, 8, 0.0f, corner, 1, 0.0f, id );
	fixedDistance( 13, box, 8, 0.0f, origin, 1, 0.0f, id );
	fixedDistance( 14, box, 8, 0.0f, tri, 3, 0.0f, turned );
	fixedDistance( 15, tri, 3, 0.0f, tri, 3, 0.0f, id );
}

int main( void )
{
	vseed( 0x5eed0d15ull );
	runFixed();
	runSupport();
	runSweep();
	for ( int batch = 0; batch < 8; ++batch )
	{
		runDistance( 1000 );
	}
	for ( int batch = 0; batch < 8; ++batch )
	{
		runCast( 1000 );
	}
	for ( int batch = 0; batch < 8; ++batch )
	{
		runToi( 1000, -1 );
	}

	// Again at another length scale
	b3SetLengthUnitsPerMeter( 0.37f );
	runCast( 1000 );
	runToi( 1000, -1 );
	b3SetLengthUnitsPerMeter( 1.0f );

	for ( int batch = 0; batch < 4; ++batch )
	{
		runToi( 5000, 5 );
	}
	return 0;
}
