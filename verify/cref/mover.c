// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/mover.as: b3SolvePlanes and b3ClipVector.

#include "common.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>

#define PLANE_CAPACITY 8

static b3Vec3 rvec( float lo, float hi )
{
	b3Vec3 v;
	v.x = vrandf( lo, hi );
	v.y = vrandf( lo, hi );
	v.z = vrandf( lo, hi );
	return v;
}

static void rplane( b3CollisionPlane* plane )
{
	b3Vec3 n = rvec( -1.0f, 1.0f );
	plane->plane.normal = b3Normalize( n );
	plane->plane.offset = vrandf( -1.0f, 1.0f );

	int limit = vrandint( 0, 4 );
	if ( limit == 0 )
	{
		plane->pushLimit = FLT_MAX;
	}
	else if ( limit == 1 )
	{
		plane->pushLimit = 0.0f;
	}
	else
	{
		plane->pushLimit = vrandf( 0.0f, 0.5f );
	}

	plane->push = vrandf( -1.0f, 1.0f );
	plane->clipVelocity = ( vrand32() & 1 ) != 0;
}

static void run( float lengthUnits, int caseCount )
{
	b3SetLengthUnitsPerMeter( lengthUnits );

	b3CollisionPlane planes[PLANE_CAPACITY];
	for ( int c = 0; c < caseCount; ++c )
	{
		int count = vrandint( 0, PLANE_CAPACITY + 1 );
		for ( int i = 0; i < count; ++i )
		{
			rplane( planes + i );
		}

		b3Vec3 targetDelta = rvec( -2.0f, 2.0f );
		b3PlaneSolverResult result = b3SolvePlanes( targetDelta, planes, count );

		vtag( "solve" );
		vi32( count );
		vf32( result.delta.x );
		vf32( result.delta.y );
		vf32( result.delta.z );
		vi32( result.iterationCount );
		for ( int i = 0; i < count; ++i )
		{
			vf32( planes[i].push );
		}
		vend();

		// Unsolved planes too, so the push == 0 skip sees both signs
		if ( ( vrand32() & 3 ) == 0 )
		{
			for ( int i = 0; i < count; ++i )
			{
				planes[i].push = ( vrand32() & 1 ) != 0 ? 0.0f : -0.0f;
			}
		}

		b3Vec3 vector = rvec( -5.0f, 5.0f );
		b3Vec3 clipped = b3ClipVector( vector, planes, count );

		vtag( "clip" );
		vf32( clipped.x );
		vf32( clipped.y );
		vf32( clipped.z );
		vend();
	}

	b3SetLengthUnitsPerMeter( 1.0f );
}

int main( void )
{
	vseed( 0x6d6f766572ull );
	run( 1.0f, 4000 );
	run( 0.37f, 2000 );
	run( 100.0f, 2000 );
	return 0;
}
