// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/capsule.as: mass, bounds, overlap, ray cast, shape cast
// and mover collision.

#include "distance_common.h"

#include "shape.h"

#include "box3d/constants.h"

static TestProxy g_proxy;

static b3Capsule rcapsule( void )
{
	b3Capsule c;
	c.center1 = rv( -2.0f, 2.0f );
	int mode = vrandint( 0, 8 );
	if ( mode == 0 )
	{
		c.center2 = c.center1;
	}
	else if ( mode == 1 )
	{
		// Shorter than the ray cast sphere fallback
		b3Vec3 d = rv( -1.0e-5f, 1.0e-5f );
		c.center2 = b3Add( c.center1, d );
	}
	else if ( mode == 2 )
	{
		b3Vec3 d = rv( -1.0e-20f, 1.0e-20f );
		c.center2 = b3Add( c.center1, d );
	}
	else
	{
		b3Vec3 d = rv( -2.0f, 2.0f );
		c.center2 = b3Add( c.center1, d );
	}
	c.radius = ( vrand32() & 7 ) == 0 ? 0.0f : vrandf( 0.01f, 1.5f );
	return c;
}

static b3Transform rxf( float extent )
{
	b3Transform xf;
	xf.p = rv( -extent, extent );
	xf.q = rquatOrIdentity();
	return xf;
}

static b3RayCastInput rray( b3Capsule c )
{
	b3Vec3 axis = b3Sub( c.center2, c.center1 );
	b3RayCastInput input;
	int mode = vrandint( 0, 8 );
	if ( mode == 0 )
	{
		// Inside the capsule
		float t = vrandf( -0.2f, 1.2f );
		b3Vec3 d = runit();
		float s = vrandf( 0.0f, 0.99f );
		input.origin = b3MulAdd( b3MulAdd( c.center1, t, axis ), s * c.radius, d );
	}
	else if ( mode == 1 )
	{
		// Inside the infinite cylinder beyond the ends
		float t = ( vrand32() & 1 ) ? vrandf( -3.0f, -0.1f ) : vrandf( 1.1f, 4.0f );
		b3Vec3 d = runit();
		float s = vrandf( 0.0f, 0.9f );
		input.origin = b3MulAdd( b3MulAdd( c.center1, t, axis ), s * c.radius, d );
	}
	else
	{
		b3Vec3 offset = rv( -5.0f, 5.0f );
		input.origin = b3Add( c.center1, offset );
	}

	int aim = vrandint( 0, 7 );
	if ( aim == 0 )
	{
		input.translation = rv( -8.0f, 8.0f );
	}
	else if ( aim == 1 )
	{
		input.translation = b3Vec3_zero;
	}
	else if ( aim == 2 )
	{
		// Parallel to the axis, maybe slightly off
		float scale = vrandf( -3.0f, 3.0f );
		b3Vec3 noise = rv( -1.0e-4f, 1.0e-4f );
		input.translation = b3Add( b3MulSV( scale, axis ), noise );
	}
	else
	{
		float t = vrandf( -0.3f, 1.3f );
		b3Vec3 target = b3MulAdd( c.center1, t, axis );
		b3Vec3 jitter = rv( -c.radius, c.radius );
		target = b3Add( target, jitter );
		float scale = vrandf( 0.3f, 3.0f );
		input.translation = b3MulSV( scale, b3Sub( target, input.origin ) );
	}

	int fraction = vrandint( 0, 4 );
	input.maxFraction = fraction < 2 ? 1.0f : ( fraction == 2 ? vrandf( 0.0f, 1.0f ) : 0.0f );
	return input;
}

int main( void )
{
	vseed( 0xca95e1ull );

	for ( int i = 0; i < 2000; ++i )
	{
		b3Capsule c = rcapsule();
		float density = vrandf( 0.0f, 10.0f );
		vtag( "m" );
		vi32( i );
		pmass( b3ComputeCapsuleMass( &c, density ) );
		vend();
	}

	for ( int i = 0; i < 2000; ++i )
	{
		b3Capsule c = rcapsule();
		b3Transform xf1 = rxf( 10.0f );
		b3Transform xf2 = rxf( 10.0f );
		vtag( "ab" );
		vi32( i );
		pab( b3ComputeCapsuleAABB( &c, xf1 ) );
		pab( b3ComputeSweptCapsuleAABB( &c, xf1, xf2 ) );
		vend();
	}

	for ( int i = 0; i < 5000; ++i )
	{
		b3Capsule c = rcapsule();
		b3Transform xf = rxf( 2.5f );
		int kind = vrandint( 0, PROXY_KIND_COUNT );
		makeTestProxy( &g_proxy, kind );
		b3ShapeProxy proxy = testProxy( &g_proxy );
		vtag( "o" );
		vi32( i );
		vbool( b3OverlapCapsule( &c, xf, &proxy ) );
		vend();
	}

	for ( int i = 0; i < 20000; ++i )
	{
		b3Capsule c = rcapsule();
		b3RayCastInput input = rray( c );
		vtag( "r" );
		vi32( i );
		pv3( input.origin );
		pv3( input.translation );
		vf32( input.maxFraction );
		pcast( b3RayCastCapsule( &c, &input ) );
		vend();
	}

	for ( int i = 0; i < 5000; ++i )
	{
		b3Capsule c = rcapsule();
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
		input.translation = b3MulSV( scale, b3Sub( b3Add( c.center1, aim ), offset ) );
		if ( ( vrand32() & 7 ) == 0 )
		{
			input.translation = rv( -6.0f, 6.0f );
		}
		input.maxFraction = ( vrand32() & 1 ) ? 1.0f : vrandf( 0.0f, 1.0f );
		input.canEncroach = ( vrand32() & 1 ) != 0;

		vtag( "sc" );
		vi32( i );
		pcast( b3ShapeCastCapsule( &c, &input ) );
		vend();
	}

	for ( int i = 0; i < 5000; ++i )
	{
		b3Capsule c = rcapsule();
		b3Capsule mover = rcapsule();
		mover.radius = vrandf( 0.05f, 1.0f );
		int mode = vrandint( 0, 5 );
		if ( mode == 0 )
		{
			// Core segments cross
			float t = vrandf( 0.0f, 1.0f );
			b3Vec3 p = b3Lerp( c.center1, c.center2, t );
			b3Vec3 d = rv( -1.0f, 1.0f );
			mover.center1 = b3Sub( p, d );
			mover.center2 = b3Add( p, d );
		}
		else if ( mode == 1 )
		{
			// Parallel and close
			b3Vec3 d = rv( -0.5f, 0.5f );
			mover.center1 = b3Add( c.center1, d );
			mover.center2 = b3Add( c.center2, d );
		}
		else if ( mode == 2 )
		{
			mover.center1 = c.center1;
			mover.center2 = c.center1;
		}

		b3PlaneResult result;
		int count = b3CollideMoverAndCapsule( &result, &c, &mover );
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
