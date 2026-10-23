// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Printers and seeded proxies shared by the distance, sphere and capsule drivers.
// verify/as/distance_common.as mirrors every function here.

#pragma once

#include "common.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#define PROXY_CAPACITY 40
#define PROXY_KIND_COUNT 8

static void pv3( b3Vec3 v )
{
	vf32( v.x );
	vf32( v.y );
	vf32( v.z );
}

static void pq( b3Quat q )
{
	pv3( q.v );
	vf32( q.s );
}

static void pxf( b3Transform t )
{
	pv3( t.p );
	pq( t.q );
}

static void pab( b3AABB a )
{
	pv3( a.lowerBound );
	pv3( a.upperBound );
}

static void pm3( b3Matrix3 m )
{
	pv3( m.cx );
	pv3( m.cy );
	pv3( m.cz );
}

static void pcast( b3CastOutput o )
{
	pv3( o.normal );
	pv3( o.point );
	vf32( o.fraction );
	vi32( o.iterations );
	vi32( o.triangleIndex );
	vi32( o.childIndex );
	vi32( o.materialIndex );
	vbool( o.hit );
}

static void pmass( b3MassData m )
{
	vf32( m.mass );
	pv3( m.center );
	pm3( m.inertia );
}

static void pplaneResult( b3PlaneResult r )
{
	pv3( r.plane.normal );
	vf32( r.plane.offset );
	pv3( r.point );
	vi32( r.triangleIndex );
	vi32( r.childIndex );
	vi32( r.materialIndex );
}

static b3Vec3 rv( float lo, float hi )
{
	b3Vec3 v;
	v.x = vrandf( lo, hi );
	v.y = vrandf( lo, hi );
	v.z = vrandf( lo, hi );
	return v;
}

static b3Vec3 runit( void )
{
	b3Vec3 v = rv( -1.0f, 1.0f );
	v = b3Normalize( v );
	if ( b3LengthSquared( v ) == 0.0f )
	{
		v = b3Vec3_axisX;
	}
	return v;
}

static b3Quat rquat( void )
{
	b3Quat q;
	q.v = rv( -1.0f, 1.0f );
	q.s = vrandf( -1.0f, 1.0f );
	return b3NormalizeQuat( q );
}

// A quarter identity rotations so axis aligned degeneracies show up.
static b3Quat rquatOrIdentity( void )
{
	if ( ( vrand32() & 3 ) == 0 )
	{
		return b3Quat_identity;
	}
	return rquat();
}

static b3Quat rotateBy( b3Quat q, float maxAngle )
{
	b3Vec3 axis = runit();
	float angle = vrandf( -maxAngle, maxAngle );
	b3Quat r = b3MakeQuatFromAxisAngle( axis, angle );
	return b3NormalizeQuat( b3MulQuat( r, q ) );
}

typedef struct TestProxy
{
	b3Vec3 points[PROXY_CAPACITY];
	int count;
	float radius;
	int kind;
} TestProxy;

// kinds: 0 point, 1 segment, 2 triangle, 3 box, 4 cloud, 5 duplicates, 6 quad, 7 collinear
static void makeTestProxy( TestProxy* tp, int kind )
{
	tp->kind = kind;
	float s = vrandf( 0.1f, 1.5f );

	switch ( kind )
	{
		case 0:
			tp->count = 1;
			tp->points[0] = rv( -s, s );
			break;

		case 1:
			tp->count = 2;
			tp->points[0] = rv( -s, s );
			tp->points[1] = rv( -s, s );
			break;

		case 2:
			tp->count = 3;
			for ( int i = 0; i < 3; ++i )
			{
				tp->points[i] = rv( -s, s );
			}
			break;

		case 3:
		{
			b3Vec3 h = rv( 0.05f, s );
			b3Vec3 c = rv( -0.5f * s, 0.5f * s );
			tp->count = 8;
			for ( int i = 0; i < 8; ++i )
			{
				b3Vec3 p;
				p.x = ( i & 1 ) ? c.x + h.x : c.x - h.x;
				p.y = ( i & 2 ) ? c.y + h.y : c.y - h.y;
				p.z = ( i & 4 ) ? c.z + h.z : c.z - h.z;
				tp->points[i] = p;
			}
		}
		break;

		case 4:
		{
			int count = vrandint( 4, 33 );
			tp->count = count;
			for ( int i = 0; i < count; ++i )
			{
				tp->points[i] = rv( -s, s );
			}
		}
		break;

		case 5:
		{
			b3Vec3 base[3];
			for ( int i = 0; i < 3; ++i )
			{
				base[i] = rv( -s, s );
			}
			int count = vrandint( 2, 9 );
			tp->count = count;
			for ( int i = 0; i < count; ++i )
			{
				int pick = vrandint( 0, 3 );
				tp->points[i] = base[pick];
			}
		}
		break;

		case 6:
		{
			float hx = vrandf( 0.05f, s );
			float hy = vrandf( 0.05f, s );
			tp->count = 4;
			tp->points[0] = ( b3Vec3 ){ -hx, -hy, 0.0f };
			tp->points[1] = ( b3Vec3 ){ hx, -hy, 0.0f };
			tp->points[2] = ( b3Vec3 ){ hx, hy, 0.0f };
			tp->points[3] = ( b3Vec3 ){ -hx, hy, 0.0f };
		}
		break;

		default:
		{
			b3Vec3 p0 = rv( -s, s );
			b3Vec3 d = runit();
			int count = vrandint( 3, 6 );
			tp->count = count;
			for ( int i = 0; i < count; ++i )
			{
				float t = vrandf( -s, s );
				tp->points[i] = b3MulAdd( p0, t, d );
			}
		}
		break;
	}

	if ( vrand32() & 1 )
	{
		tp->radius = 0.0f;
	}
	else
	{
		tp->radius = s * vrandf( 0.01f, 0.5f );
	}
}

static b3ShapeProxy testProxy( const TestProxy* tp )
{
	return ( b3ShapeProxy ){ tp->points, tp->count, tp->radius };
}

static void pproxy( const TestProxy* tp )
{
	vi32( tp->kind );
	vi32( tp->count );
	vf32( tp->radius );
	pv3( tp->points[0] );
	pv3( tp->points[tp->count - 1] );
}
