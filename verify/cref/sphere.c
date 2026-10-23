// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/sphere.as: mass, bounds, overlap, ray casts, shape cast
// and mover collision.

#include "distance_common.h"

#include "shape.h"

#include "box3d/constants.h"

static TestProxy g_proxy;

static b3Sphere rsphere( void )
{
	b3Sphere s;
	s.center = rv( -2.0f, 2.0f );
	s.radius = ( vrand32() & 7 ) == 0 ? 0.0f : vrandf( 0.01f, 2.0f );
	return s;
}

static b3Transform rxf( float extent )
{
	b3Transform xf;
	xf.p = rv( -extent, extent );
	xf.q = rquatOrIdentity();
	return xf;
}

static b3RayCastInput rray( b3Vec3 center, float radius )
{
	b3RayCastInput input;
	int mode = vrandint( 0, 6 );
	if ( mode == 0 )
	{
		// Inside
		b3Vec3 d = runit();
		float t = vrandf( 0.0f, 0.99f );
		input.origin = b3MulAdd( center, t * radius, d );
	}
	else if ( mode == 1 )
	{
		// Near the surface
		b3Vec3 d = runit();
		float t = vrandf( 0.98f, 1.02f );
		input.origin = b3MulAdd( center, t * radius, d );
	}
	else
	{
		b3Vec3 offset = rv( -6.0f, 6.0f );
		input.origin = b3Add( center, offset );
	}

	int aim = vrandint( 0, 5 );
	if ( aim == 0 )
	{
		input.translation = rv( -8.0f, 8.0f );
	}
	else if ( aim == 1 )
	{
		input.translation = b3Vec3_zero;
	}
	else
	{
		b3Vec3 target = rv( -radius, radius );
		target = b3Add( center, target );
		float scale = vrandf( 0.3f, 3.0f );
		input.translation = b3MulSV( scale, b3Sub( target, input.origin ) );
	}

	int fraction = vrandint( 0, 4 );
	input.maxFraction = fraction < 2 ? 1.0f : ( fraction == 2 ? vrandf( 0.0f, 1.0f ) : 0.0f );
	return input;
}

static b3Capsule rmover( void )
{
	b3Capsule mover;
	mover.center1 = rv( -2.0f, 2.0f );
	int mode = vrandint( 0, 4 );
	if ( mode == 0 )
	{
		mover.center2 = mover.center1;
	}
	else
	{
		b3Vec3 d = rv( -1.5f, 1.5f );
		mover.center2 = b3Add( mover.center1, d );
	}
	mover.radius = vrandf( 0.05f, 1.0f );
	return mover;
}

int main( void )
{
	vseed( 0x5be7e5ull );

	for ( int i = 0; i < 1000; ++i )
	{
		b3Sphere s = rsphere();
		float density = vrandf( 0.0f, 10.0f );
		vtag( "m" );
		vi32( i );
		pmass( b3ComputeSphereMass( &s, density ) );
		vend();
	}

	for ( int i = 0; i < 2000; ++i )
	{
		b3Sphere s = rsphere();
		b3Transform xf1 = rxf( 10.0f );
		b3Transform xf2 = rxf( 10.0f );
		vtag( "ab" );
		vi32( i );
		pab( b3ComputeSphereAABB( &s, xf1 ) );
		pab( b3ComputeSweptSphereAABB( &s, xf1, xf2 ) );
		vend();
	}

	for ( int i = 0; i < 5000; ++i )
	{
		b3Sphere s = rsphere();
		b3Transform xf = rxf( 2.5f );
		int kind = vrandint( 0, PROXY_KIND_COUNT );
		makeTestProxy( &g_proxy, kind );
		b3ShapeProxy proxy = testProxy( &g_proxy );
		vtag( "o" );
		vi32( i );
		vbool( b3OverlapSphere( &s, xf, &proxy ) );
		vend();
	}

	for ( int i = 0; i < 10000; ++i )
	{
		b3Sphere s = rsphere();
		b3RayCastInput input = rray( s.center, s.radius );
		vtag( "r" );
		vi32( i );
		pv3( input.origin );
		pv3( input.translation );
		vf32( input.maxFraction );
		pcast( b3RayCastSphere( &s, &input ) );
		pcast( b3RayCastHollowSphere( &s, &input ) );
		vend();
	}

	for ( int i = 0; i < 5000; ++i )
	{
		b3Sphere s = rsphere();
		int kind = vrandint( 0, PROXY_KIND_COUNT );
		makeTestProxy( &g_proxy, kind );
		b3Vec3 offset = rv( -4.0f, 4.0f );
		for ( int k = 0; k < g_proxy.count; ++k )
		{
			g_proxy.points[k] = b3Add( g_proxy.points[k], offset );
		}

		b3ShapeCastInput input;
		input.proxy = testProxy( &g_proxy );
		b3Vec3 aim = rv( -1.0f, 1.0f );
		float scale = vrandf( 0.2f, 2.5f );
		input.translation = b3MulSV( scale, b3Sub( b3Add( s.center, aim ), offset ) );
		if ( ( vrand32() & 7 ) == 0 )
		{
			input.translation = rv( -6.0f, 6.0f );
		}
		input.maxFraction = ( vrand32() & 1 ) ? 1.0f : vrandf( 0.0f, 1.0f );
		input.canEncroach = ( vrand32() & 1 ) != 0;

		vtag( "sc" );
		vi32( i );
		pcast( b3ShapeCastSphere( &s, &input ) );
		vend();
	}

	for ( int i = 0; i < 5000; ++i )
	{
		b3Sphere s = rsphere();
		b3Capsule mover = rmover();
		int mode = vrandint( 0, 4 );
		if ( mode == 0 )
		{
			// Sphere center on the mover axis
			float t = vrandf( 0.0f, 1.0f );
			s.center = b3Lerp( mover.center1, mover.center2, t );
		}
		else if ( mode == 1 )
		{
			b3Vec3 d = rv( -0.003f, 0.003f );
			s.center = b3Add( mover.center1, d );
		}

		b3PlaneResult result;
		int count = b3CollideMoverAndSphere( &result, &s, &mover );
		vtag( "mv" );
		vi32( i );
		vi32( count );
		if ( count > 0 )
		{
			pplaneResult( result );
		}
		vend();
	}

	return 0;
}
