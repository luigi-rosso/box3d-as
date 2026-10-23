// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Hull vectors: quickhull over sample, procedural, random and degenerate point clouds, every
// field and array of each hull and its raw blob, queries, clones, transforms, boxes, the hull map
// and the 2D hull. Phases mirror the exports of verify/as/hull.as, each with its own seed.

#include "common.h"
#include "hull_data.h"

#include "hull.h"
#include "math_internal.h"
#include "shape.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <float.h>

static void vvec( b3Vec3 v )
{
	vf32( v.x );
	vf32( v.y );
	vf32( v.z );
}

static void vplane( b3Plane p )
{
	vvec( p.normal );
	vf32( p.offset );
}

static void vaabb( b3AABB a )
{
	vvec( a.lowerBound );
	vvec( a.upperBound );
}

static void vmat( b3Matrix3 m )
{
	vvec( m.cx );
	vvec( m.cy );
	vvec( m.cz );
}

static b3Vec3 vrandvec( float lo, float hi )
{
	b3Vec3 v;
	v.x = vrandf( lo, hi );
	v.y = vrandf( lo, hi );
	v.z = vrandf( lo, hi );
	return v;
}

static b3Quat vrandquat( void )
{
	b3Quat q;
	q.v.x = vrandf( -1.0f, 1.0f );
	q.v.y = vrandf( -1.0f, 1.0f );
	q.v.z = vrandf( -1.0f, 1.0f );
	q.s = vrandf( -1.0f, 1.0f );
	return b3NormalizeQuat( q );
}

static b3Transform vrandxf( float range )
{
	b3Transform xf;
	xf.p = vrandvec( -range, range );
	xf.q = vrandquat();
	return xf;
}

static b3Vec3 bitsVec( int pointIndex )
{
	b3Vec3 v;
	v.x = vbitsf( hullPointBits[3 * pointIndex + 0] );
	v.y = vbitsf( hullPointBits[3 * pointIndex + 1] );
	v.z = vbitsf( hullPointBits[3 * pointIndex + 2] );
	return v;
}

// B3_ENABLE_VALIDATION body of b3IsValidHull, which the reference build compiles out.
static bool refValidateHull( const b3HullData* hull )
{
	if ( hull->version != B3_HULL_VERSION )
	{
		return false;
	}

	int v = hull->vertexCount;
	int e = hull->edgeCount / 2;
	int f = hull->faceCount;

	if ( v - e + f != 2 )
	{
		return false;
	}

	const b3HullVertex* vertices = b3GetHullVertices( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	for ( int index = 0; index < hull->vertexCount; ++index )
	{
		const b3HullVertex* vertex = vertices + index;
		const b3HullHalfEdge* edge = edges + vertex->edge;

		if ( edge->origin != index )
		{
			return false;
		}
	}

	for ( int index = 0; index < hull->edgeCount; index += 2 )
	{
		const b3HullHalfEdge* edge = edges + index + 0;
		const b3HullHalfEdge* twin = edges + index + 1;

		if ( edge->twin != index + 1 )
		{
			return false;
		}

		if ( twin->twin != index + 0 )
		{
			return false;
		}
	}

	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );
	for ( int faceIndex = 0; faceIndex < hull->faceCount; ++faceIndex )
	{
		const b3HullFace* face = faces + faceIndex;

		int baseEdgeIndex = face->edge;
		const b3HullHalfEdge* edge = edges + baseEdgeIndex;

		b3Plane plane = planes[faceIndex];
		if ( b3PlaneSeparation( plane, hull->center ) >= 0.0f )
		{
			return false;
		}

		int edgeIndex = baseEdgeIndex;
		do
		{
			edge = edges + edgeIndex;
			const b3HullHalfEdge* next = edges + edge->next;
			const b3HullHalfEdge* twin = edges + edge->twin;

			if ( edge->face != faceIndex )
			{
				return false;
			}

			if ( twin->twin != edgeIndex )
			{
				return false;
			}

			if ( next->origin != twin->origin )
			{
				return false;
			}

			edgeIndex = edge->next;
		}
		while ( edgeIndex != baseEdgeIndex );
	}

	if ( hull->volume <= 0.0f )
	{
		return false;
	}

	if ( hull->surfaceArea <= 0.0f )
	{
		return false;
	}

	if ( hull->innerRadius <= 0.0f )
	{
		return false;
	}

	return true;
}

// Log lines become vectors, so the hull limit messages are compared too.
static void logToVectors( const char* message )
{
	vtag( "log" );
	vtag( message );
	vend();
}

static void destroyHull( b3HullData* hull )
{
	if ( hull != NULL )
	{
		b3DestroyHull( hull );
	}
}

static void printHull( const char* tag, const b3HullData* hull )
{
	vtag( tag );
	if ( hull == NULL )
	{
		vtag( "null" );
		vend();
		return;
	}

	vu64( hull->version );
	vu64( hull->hash );
	vaabb( hull->aabb );
	vf32( hull->surfaceArea );
	vf32( hull->volume );
	vf32( hull->innerRadius );
	vvec( hull->center );
	vmat( hull->centralInertia );
	vi32( hull->vertexCount );
	vi32( hull->vertexOffset );
	vi32( hull->pointOffset );
	vi32( hull->edgeCount );
	vi32( hull->edgeOffset );
	vi32( hull->faceCount );
	vi32( hull->planeOffset );
	vi32( hull->faceOffset );
	vi32( hull->soaVertexOffset );
	vi32( hull->soaNormalOffset );
	vi32( hull->edgeCosineOffset );
	vi32( hull->byteCount );
	vi32( hull->padding );
	vend();

	const b3HullVertex* vertices = b3GetHullVertices( hull );
	const b3Vec3* points = b3GetHullPoints( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );
	const b3HullFace* faces = b3GetHullFaces( hull );
	const float* soaVertices = b3GetHullSoaVertices( hull );
	const float* soaNormals = b3GetHullSoaNormals( hull );
	const float* cosines = b3GetHullEdgeCosines( hull );

	vtag( "v" );
	for ( int i = 0; i < hull->vertexCount; ++i )
	{
		vi32( vertices[i].edge );
	}
	vend();

	vtag( "p" );
	for ( int i = 0; i < hull->vertexCount; ++i )
	{
		vvec( points[i] );
	}
	vend();

	vtag( "e" );
	for ( int i = 0; i < hull->edgeCount; ++i )
	{
		vi32( edges[i].next );
		vi32( edges[i].twin );
		vi32( edges[i].origin );
		vi32( edges[i].face );
	}
	vend();

	vtag( "pl" );
	for ( int i = 0; i < hull->faceCount; ++i )
	{
		vplane( planes[i] );
	}
	vend();

	vtag( "f" );
	for ( int i = 0; i < hull->faceCount; ++i )
	{
		vi32( faces[i].edge );
	}
	vend();

	vtag( "sv" );
	int soaVertexCount = 3 * b3GetHullSoaStride( hull->vertexCount );
	for ( int i = 0; i < soaVertexCount; ++i )
	{
		vf32( soaVertices[i] );
	}
	vend();

	vtag( "sn" );
	int soaNormalCount = 3 * b3GetHullSoaStride( hull->faceCount );
	for ( int i = 0; i < soaNormalCount; ++i )
	{
		vf32( soaNormals[i] );
	}
	vend();

	vtag( "ec" );
	for ( int i = 0; i < hull->edgeCount / 2; ++i )
	{
		vf32( cosines[i] );
	}
	vend();

	// The raw blob, which the hash and the hull map compare.
	vtag( "blob" );
	const uint32_t* words = (const uint32_t*)hull;
	for ( int i = 0; i < hull->byteCount / 4; ++i )
	{
		vu32( words[i] );
	}
	vend();

	vtag( "valid" );
	vbool( b3IsValidHull( hull ) );
	vbool( refValidateHull( hull ) );
	vu64( b3HashHullData( hull ) );
	vend();
}

static void printCast( b3CastOutput o )
{
	vvec( o.normal );
	vvec( o.point );
	vf32( o.fraction );
	vi32( o.iterations );
	vi32( o.triangleIndex );
	vi32( o.childIndex );
	vi32( o.materialIndex );
	vbool( o.hit );
}

static void queryRay( const b3HullData* hull, b3Vec3 origin, b3Vec3 translation, float maxFraction )
{
	b3RayCastInput input = { origin, translation, maxFraction };
	b3CastOutput o = b3RayCastHull( hull, &input );
	vtag( "ray" );
	printCast( o );
	vend();
}

static void querySupport( const b3HullData* hull, b3Vec3 d )
{
	vi32( b3FindHullSupportVertex( hull, d ) );
	vi32( b3FindHullSupportFace( hull, d ) );
}

static void queryHull( const b3HullData* hull, int directionCount, int rayCount )
{
	b3Vec3 lower = hull->aabb.lowerBound;
	b3Vec3 upper = hull->aabb.upperBound;
	b3Vec3 center = b3MulSV( 0.5f, b3Add( lower, upper ) );
	b3Vec3 extent = b3Sub( upper, lower );

	vtag( "mass" );
	b3MassData md = b3ComputeHullMass( hull, 1.0f );
	vf32( md.mass );
	vvec( md.center );
	vmat( md.inertia );
	md = b3ComputeHullMass( hull, 0.37f );
	vf32( md.mass );
	vvec( md.center );
	vmat( md.inertia );
	float density = vrandf( 0.1f, 10.0f );
	md = b3ComputeHullMass( hull, density );
	vf32( md.mass );
	vvec( md.center );
	vmat( md.inertia );
	vend();

	vtag( "aabb" );
	for ( int i = 0; i < 3; ++i )
	{
		b3Transform xf1 = vrandxf( 10.0f );
		b3Transform xf2 = vrandxf( 10.0f );
		vaabb( b3ComputeHullAABB( hull, xf1 ) );
		vaabb( b3ComputeSweptHullAABB( hull, xf1, xf2 ) );
	}
	vend();

	vtag( "support" );
	querySupport( hull, b3Vec3_axisX );
	querySupport( hull, b3Neg( b3Vec3_axisX ) );
	querySupport( hull, b3Vec3_axisY );
	querySupport( hull, b3Neg( b3Vec3_axisY ) );
	querySupport( hull, b3Vec3_axisZ );
	querySupport( hull, b3Neg( b3Vec3_axisZ ) );
	querySupport( hull, b3Vec3_zero );
	for ( int i = 0; i < directionCount; ++i )
	{
		b3Vec3 d = vrandvec( -1.0f, 1.0f );
		querySupport( hull, d );
	}
	// NaN directions break the support vertex precondition, so an assert build skips them.
#ifndef HULL_NO_SPECIALS
	for ( int i = 0; i < 3; ++i )
	{
		b3Vec3 d;
		d.x = vspecialf();
		d.y = vspecialf();
		d.z = vspecialf();
		querySupport( hull, d );
	}
#endif
	vend();

	for ( int i = 0; i < rayCount; ++i )
	{
		b3Vec3 o = vrandvec( -1.5f, 1.5f );
		b3Vec3 t = vrandvec( -3.0f, 3.0f );
		float maxFraction = vrandf( 0.0f, 1.5f );
		queryRay( hull, b3Add( center, b3Mul( o, extent ) ), b3Mul( t, extent ), maxFraction );
	}

	// Point queries and rays from the centroid
	queryRay( hull, center, b3Vec3_zero, 1.0f );
	queryRay( hull, hull->center, b3Vec3_axisX, 1.0f );
	queryRay( hull, b3Add( upper, b3Vec3_one ), b3Vec3_zero, 1.0f );
	queryRay( hull, b3Add( upper, b3Vec3_one ), b3Neg( b3Add( extent, b3Add( b3Vec3_one, b3Vec3_one ) ) ), 1.0f );

	// GJK based queries
	for ( int i = 0; i < 4; ++i )
	{
		b3Vec3 proxyPoints[8];
		int proxyCount = vrandint( 1, 9 );
		for ( int j = 0; j < proxyCount; ++j )
		{
			b3Vec3 o = vrandvec( -1.0f, 1.0f );
			proxyPoints[j] = b3Add( center, b3Mul( o, extent ) );
		}
		float proxyRadius = vrandf( 0.0f, 0.3f );
		b3ShapeProxy proxy = { proxyPoints, proxyCount, proxyRadius };
		b3Transform xf = vrandxf( 0.5f );

		vtag( "overlap" );
		vbool( b3OverlapHull( hull, xf, &proxy ) );
		vend();

		b3ShapeCastInput castInput;
		castInput.proxy = proxy;
		b3Vec3 t = vrandvec( -2.0f, 2.0f );
		castInput.translation = b3Mul( t, extent );
		castInput.maxFraction = vrandf( 0.0f, 1.0f );
		castInput.canEncroach = vrandint( 0, 2 ) == 1;
		vtag( "shapecast" );
		printCast( b3ShapeCastHull( hull, &castInput ) );
		vend();

		b3Capsule mover;
		b3Vec3 m1 = vrandvec( -1.0f, 1.0f );
		b3Vec3 m2 = vrandvec( -1.0f, 1.0f );
		mover.center1 = b3Add( center, b3Mul( m1, extent ) );
		mover.center2 = b3Add( center, b3Mul( m2, extent ) );
		mover.radius = vrandf( 0.01f, 1.0f );
		b3PlaneResult result = { 0 };
		int planeCount = b3CollideMoverAndHull( &result, hull, &mover );
		vtag( "mover" );
		vi32( planeCount );
		vplane( result.plane );
		vvec( result.point );
		vi32( result.triangleIndex );
		vi32( result.childIndex );
		vi32( result.materialIndex );
		vend();
	}

	vtag( "extent" );
	for ( int i = 0; i < 2; ++i )
	{
		b3Vec3 origin = vrandvec( -2.0f, 2.0f );
		b3ShapeExtent ex = b3ComputeHullExtent( hull, origin );
		vf32( ex.minExtent );
		vvec( ex.maxExtent );
	}
	vend();

	vtag( "area" );
	for ( int i = 0; i < 6; ++i )
	{
		b3Vec3 d = vrandvec( -1.0f, 1.0f );
		vf32( b3ComputeHullProjectedArea( hull, d ) );
	}
	vf32( b3ComputeHullProjectedArea( hull, b3Vec3_axisY ) );
	vend();
}

static b3Vec3 randomScale( int kind )
{
	b3Vec3 s;
	if ( kind == 0 )
	{
		float u = vrandf( 0.25f, 3.0f );
		s = (b3Vec3){ u, u, u };
	}
	else if ( kind == 1 )
	{
		s = vrandvec( 0.2f, 3.0f );
	}
	else if ( kind == 2 )
	{
		s = vrandvec( 0.2f, 3.0f );
		int axis = vrandint( 0, 3 );
		if ( axis == 0 )
		{
			s.x = -s.x;
		}
		else if ( axis == 1 )
		{
			s.y = -s.y;
		}
		else
		{
			s.z = -s.z;
		}
	}
	else
	{
		s = vrandvec( -3.0f, -0.2f );
		s.y = vrandf( 0.0f, 0.02f );
	}
	return s;
}

// Prints the hull, runs the queries, clones and transforms it.
static void checkHull( const char* tag, const b3HullData* hull )
{
	printHull( tag, hull );
	if ( hull == NULL )
	{
		return;
	}

	queryHull( hull, 40, 40 );

	b3HullData* clone = b3CloneHull( hull );
	printHull( "clone", clone );
	vtag( "same" );
	vbool( b3CompareHullData( clone, hull ) );
	vbool( b3CompareHullData( hull, hull ) );
	vend();
	destroyHull( clone );

	for ( int kind = 0; kind < 4; ++kind )
	{
		b3Transform xf = vrandxf( 5.0f );
		b3Vec3 scale = randomScale( kind );
		b3HullData* t = b3CloneAndTransformHull( hull, xf, scale );
		printHull( "xf", t );
		if ( t != NULL )
		{
			queryHull( t, 8, 8 );
			vtag( "same" );
			vbool( b3CompareHullData( t, hull ) );
			vend();
			destroyHull( t );
		}
	}
}

static int pickMaxCount( int n )
{
	int pick = vrandint( 0, 10 );
	switch ( pick )
	{
		case 0:
			return 4;
		case 1:
			return 8;
		case 2:
			return 16;
		case 3:
			return 32;
		case 4:
			return 1;
		case 5:
			return 1000;
		case 6:
			return vrandint( 4, 130 );
		default:
			return n;
	}
}

static b3Vec3 g_points[4096];

static void phaseSample( int set )
{
	vseed( 100 + set );
	int offset = hullSets[3 * set + 0];
	int count = hullSets[3 * set + 1];
	int maxCount = hullSets[3 * set + 2];
	for ( int i = 0; i < count; ++i )
	{
		g_points[i] = bitsVec( offset + i );
	}

	vtag( "sample" );
	vi32( set );
	vi32( count );
	vi32( maxCount );
	vend();

	b3HullData* hull = b3CreateHull( g_points, count, maxCount );
	checkHull( "hull", hull );
	destroyHull( hull );

	// Other limits on the same cloud
	int limits[4] = { 4, 6, 12, 1000 };
	for ( int i = 0; i < 4; ++i )
	{
		b3HullData* h = b3CreateHull( g_points, count, limits[i] );
		printHull( "limit", h );
		destroyHull( h );
	}
}

// The WheelStack sample wraps the 37 wheel pieces in one hull.
static void phaseWheel( void )
{
	vseed( 99 );
	int bufferCount = 0;
	for ( int h = 0; h < 37; ++h )
	{
		int set = 4 + h;
		int offset = hullSets[3 * set + 0];
		int count = hullSets[3 * set + 1];
		for ( int i = 0; i < count; ++i )
		{
			g_points[i] = bitsVec( offset + i );
		}
		b3HullData* piece = b3CreateHull( g_points, count, count );
		const b3Vec3* points = b3GetHullPoints( piece );
		for ( int j = 0; j < piece->vertexCount && bufferCount < 512; ++j )
		{
			g_points[2048 + bufferCount] = points[j];
			++bufferCount;
		}
		destroyHull( piece );
	}

	vtag( "wheel" );
	vi32( bufferCount );
	vend();
	b3HullData* hull = b3CreateHull( g_points + 2048, bufferCount, bufferCount );
	checkHull( "hull", hull );
	destroyHull( hull );
}

static void phaseRandom( int k )
{
	vseed( 5000 + k );
	int type = k % 12;
	int n = 0;

	if ( type == 0 )
	{
		n = vrandint( 4, 400 );
		for ( int i = 0; i < n; ++i )
		{
			g_points[i] = vrandvec( -1.0f, 1.0f );
		}
	}
	else if ( type == 1 )
	{
		n = vrandint( 4, 300 );
		for ( int i = 0; i < n; ++i )
		{
			b3Vec3 v = vrandvec( -1.0f, 1.0f );
			g_points[i] = b3Normalize( v );
		}
	}
	else if ( type == 2 )
	{
		// HullReduction box cloud
		n = vrandint( 8, 256 );
		float a = 0.001f;
		for ( int i = 0; i < n; ++i )
		{
			b3Vec3 p = vrandvec( -2.0f, 2.0f );
			p = b3Clamp( p, (b3Vec3){ -1.0f, -1.0f, -1.0f }, (b3Vec3){ 1.0f, 1.0f, 1.0f } );
			b3Vec3 f = vrandvec( -a, a );
			g_points[i] = b3Add( p, f );
		}
	}
	else if ( type == 3 )
	{
		// Nearly coplanar
		n = vrandint( 4, 100 );
		int pick = vrandint( 0, 5 );
		float eps = pick == 0 ? 0.0f : ( pick == 1 ? 1e-7f : ( pick == 2 ? 1e-6f : ( pick == 3 ? 1e-5f : 1e-3f ) ) );
		for ( int i = 0; i < n; ++i )
		{
			b3Vec3 p;
			p.x = vrandf( -1.0f, 1.0f );
			p.y = vrandf( -1.0f, 1.0f );
			p.z = vrandf( -eps, eps );
			g_points[i] = p;
		}
	}
	else if ( type == 4 )
	{
		// Duplicates
		int m = vrandint( 4, 40 );
		int r = vrandint( 2, 5 );
		for ( int i = 0; i < m; ++i )
		{
			b3Vec3 p = vrandvec( -1.0f, 1.0f );
			for ( int j = 0; j < r; ++j )
			{
				g_points[n++] = p;
			}
		}
	}
	else if ( type == 5 )
	{
		// Nearly collinear
		n = vrandint( 4, 60 );
		b3Vec3 d = vrandvec( -1.0f, 1.0f );
		float eps = vrandint( 0, 2 ) == 0 ? 0.0f : 1e-6f;
		for ( int i = 0; i < n; ++i )
		{
			float t = vrandf( -2.0f, 2.0f );
			b3Vec3 noise = vrandvec( -eps, eps );
			g_points[i] = b3Add( b3MulSV( t, d ), noise );
		}
	}
	else if ( type == 6 )
	{
		// One point repeated
		n = vrandint( 4, 20 );
		b3Vec3 p = vrandvec( -1.0f, 1.0f );
		for ( int i = 0; i < n; ++i )
		{
			g_points[i] = p;
		}
	}
	else if ( type == 7 )
	{
		// Scaled cloud
		n = vrandint( 4, 200 );
		int e = vrandint( -4, 5 );
		float s = 1.0f;
		for ( int i = 0; i < e; ++i )
		{
			s *= 10.0f;
		}
		for ( int i = 0; i > e; --i )
		{
			s *= 0.1f;
		}
		for ( int i = 0; i < n; ++i )
		{
			b3Vec3 p = vrandvec( -1.0f, 1.0f );
			g_points[i] = b3MulSV( s, p );
		}
	}
	else if ( type == 8 )
	{
		// Far from the origin
		n = vrandint( 4, 200 );
		b3Vec3 offset = vrandvec( -5000.0f, 5000.0f );
		for ( int i = 0; i < n; ++i )
		{
			b3Vec3 p = vrandvec( -1.0f, 1.0f );
			g_points[i] = b3Add( offset, p );
		}
	}
	else if ( type == 9 )
	{
		// Transformed box corners
		b3Vec3 h = vrandvec( 0.01f, 3.0f );
		b3Transform xf = vrandxf( 10.0f );
		n = 8;
		g_points[0] = b3TransformPoint( xf, (b3Vec3){ h.x, h.y, h.z } );
		g_points[1] = b3TransformPoint( xf, (b3Vec3){ h.x, h.y, -h.z } );
		g_points[2] = b3TransformPoint( xf, (b3Vec3){ h.x, -h.y, h.z } );
		g_points[3] = b3TransformPoint( xf, (b3Vec3){ h.x, -h.y, -h.z } );
		g_points[4] = b3TransformPoint( xf, (b3Vec3){ -h.x, h.y, h.z } );
		g_points[5] = b3TransformPoint( xf, (b3Vec3){ -h.x, h.y, -h.z } );
		g_points[6] = b3TransformPoint( xf, (b3Vec3){ -h.x, -h.y, h.z } );
		g_points[7] = b3TransformPoint( xf, (b3Vec3){ -h.x, -h.y, -h.z } );
	}
	else if ( type == 10 )
	{
		// Few points
		n = vrandint( 0, 7 );
		for ( int i = 0; i < n; ++i )
		{
			g_points[i] = vrandvec( -1.0f, 1.0f );
		}
	}
	else
	{
		// Lattice, many coplanar faces, optional jitter
		int a = vrandint( 2, 6 );
		float jitter = vrandint( 0, 2 ) == 0 ? 0.0f : 1e-4f;
		for ( int i = 0; i < a; ++i )
		{
			for ( int j = 0; j < a; ++j )
			{
				for ( int l = 0; l < a; ++l )
				{
					b3Vec3 noise = vrandvec( -jitter, jitter );
					b3Vec3 p = { (float)i, (float)j, (float)l };
					g_points[n++] = b3Add( p, noise );
				}
			}
		}
	}

	int maxCount = pickMaxCount( n );

	// Upstream b3CreateHull crashes (3, 27) or never returns (the rest) on these clouds within
	// 1e-5 of a plane. See notes/hull.md.
	static const int skip[6] = { 3, 27, 267, 303, 627, 699 };
	for ( int i = 0; i < 6; ++i )
	{
		if ( k == skip[i] )
		{
			vtag( "skip" );
			vi32( k );
			vend();
			return;
		}
	}

	vtag( "random" );
	vi32( k );
	vi32( n );
	vi32( maxCount );
	vend();

	b3HullData* hull = b3CreateHull( g_points, n, maxCount );
	checkHull( "hull", hull );
	destroyHull( hull );
}

static void phaseShapes( int k )
{
	vseed( 9000 + k );

	int sides = 3 + k % 30;
	float height = vrandf( 0.05f, 4.0f );
	float radius = vrandf( 0.05f, 3.0f );
	float yOffset = vrandf( -2.0f, 2.0f );
	vtag( "cylinder" );
	vi32( sides );
	vend();
	b3HullData* cylinder = b3CreateCylinder( height, radius, yOffset, sides );
	checkHull( "hull", cylinder );
	destroyHull( cylinder );

	int slices = 4 + k % 29;
	float radius1 = vrandf( 0.05f, 3.0f );
	float radius2 = vrandf( 0.05f, 3.0f );
	vtag( "cone" );
	vi32( slices );
	vend();
	b3HullData* cone = b3CreateCone( height, radius1, radius2, slices );
	checkHull( "hull", cone );
	destroyHull( cone );

	float rockRadius = vrandf( 0.01f, 10.0f );
	b3HullData* rock = b3CreateRock( rockRadius );
	checkHull( "rock", rock );
	destroyHull( rock );

	float complexRadius = vrandf( 0.01f, 10.0f );
	b3HullData* complex = b3CreateComplexHull( complexRadius );
	checkHull( "complex", complex );
	destroyHull( complex );
}

static void boxQueries( const b3HullData* hull )
{
	// Axis aligned rays hit exactly zero denominators.
	b3Vec3 axes[3] = { b3Vec3_axisX, b3Vec3_axisY, b3Vec3_axisZ };
	for ( int i = 0; i < 3; ++i )
	{
		b3Vec3 o = vrandvec( -3.0f, 3.0f );
		queryRay( hull, o, b3MulSV( 6.0f, axes[i] ), 1.0f );
		queryRay( hull, o, b3MulSV( -6.0f, axes[i] ), 1.0f );
		queryRay( hull, b3Vec3_zero, axes[i], 1.0f );
	}
}

static void phaseBoxes( int k )
{
	vseed( 12000 + k );

	b3Vec3 h = vrandvec( 0.01f, 3.0f );
	if ( k % 7 == 0 )
	{
		h.y = 0.0001f;
	}
	b3Transform xf = vrandxf( 5.0f );
	b3Vec3 offset = vrandvec( -5.0f, 5.0f );

	b3BoxHull box = b3MakeBoxHull( h.x, h.y, h.z );
	checkHull( "box", &box.base );
	boxQueries( &box.base );

	b3BoxHull cube = b3MakeCubeHull( h.x );
	printHull( "cube", &cube.base );

	b3BoxHull offsetBox = b3MakeOffsetBoxHull( h.x, h.y, h.z, offset );
	printHull( "offset", &offsetBox.base );
	boxQueries( &offsetBox.base );

	b3BoxHull xfBox = b3MakeTransformedBoxHull( h.x, h.y, h.z, xf );
	checkHull( "xfbox", &xfBox.base );

	b3Vec3 postScale = randomScale( k % 4 );
	b3BoxHull scaled = b3MakeScaledBoxHull( h, xf, postScale );
	printHull( "scaled", &scaled.base );
	queryHull( &scaled.base, 8, 8 );

	b3Vec3 sh = h;
	b3Transform sxf = xf;
	float minHalfWidth = vrandf( 0.001f, 0.5f );
	b3ScaleBox( &sh, &sxf, postScale, minHalfWidth );
	vtag( "scalebox" );
	vvec( sh );
	vvec( sxf.p );
	vvec( sxf.q.v );
	vf32( sxf.q.s );
	vend();
}

static void phaseMap( void )
{
	vseed( 777 );

	b3HullData* hulls[24];
	int hullCount = 0;
	for ( int i = 0; i < 6; ++i )
	{
		int n = vrandint( 8, 40 );
		for ( int j = 0; j < n; ++j )
		{
			g_points[j] = vrandvec( -1.0f, 1.0f );
		}
		hulls[hullCount++] = b3CreateHull( g_points, n, n );
		// Same content, distinct allocation
		hulls[hullCount++] = b3CreateHull( g_points, n, n );
		b3HullData* clone = b3CloneHull( hulls[hullCount - 1] );
		hulls[hullCount++] = clone;
	}
	b3BoxHull box1 = b3MakeBoxHull( 1.0f, 2.0f, 3.0f );
	b3BoxHull box2 = b3MakeBoxHull( 1.0f, 2.0f, 3.0f );
	b3BoxHull box3 = b3MakeBoxHull( 1.0f, 2.0f, 3.5f );

	b3HullMap map;
	b3HullMap_init( &map );

	// The world hull database: reference counts
	vtag( "db" );
	const b3HullData* keys[24];
	int keyCount = 0;
	for ( int i = 0; i < hullCount; ++i )
	{
		b3HullMap_itr itr = b3HullMap_get( &map, hulls[i] );
		if ( b3HullMap_is_end( itr ) == false )
		{
			itr.data->val += 1;
			vi32( 1 );
			vi32( itr.data->val );
			vbool( itr.data->key == hulls[i] );
		}
		else
		{
			b3HullMap_insert( &map, hulls[i], 1 );
			keys[keyCount++] = hulls[i];
			vi32( 0 );
		}
	}
	const b3HullData* boxes[3] = { &box1.base, &box2.base, &box3.base };
	for ( int i = 0; i < 3; ++i )
	{
		b3HullMap_itr itr = b3HullMap_get_or_insert( &map, boxes[i], 1 );
		vi32( itr.data->val );
		vbool( itr.data->key == boxes[i] );
		itr.data->val += 1;
	}
	vi32( (int)b3HullMap_size( &map ) );
	vend();

	vtag( "iterate" );
	int count = 0;
	int sum = 0;
	for ( b3HullMap_itr itr = b3HullMap_first( &map ); b3HullMap_is_end( itr ) == false; itr = b3HullMap_next( itr ) )
	{
		count += 1;
		sum += itr.data->val;
	}
	vi32( count );
	vi32( sum );
	vend();

	// Remove references
	vtag( "remove" );
	for ( int i = 0; i < hullCount; i += 2 )
	{
		b3HullMap_itr itr = b3HullMap_get( &map, hulls[i] );
		if ( b3HullMap_is_end( itr ) )
		{
			vi32( -1 );
			continue;
		}
		if ( --itr.data->val == 0 )
		{
			b3HullMap_erase_itr( &map, itr );
			vi32( 0 );
		}
		else
		{
			vi32( itr.data->val );
		}
	}
	vi32( (int)b3HullMap_size( &map ) );
	for ( int i = 0; i < hullCount; ++i )
	{
		b3HullMap_itr itr = b3HullMap_get( &map, hulls[i] );
		vbool( b3HullMap_is_end( itr ) );
		if ( b3HullMap_is_end( itr ) == false )
		{
			vi32( itr.data->val );
		}
	}
	vend();

	// Compound baking: unique hull indices
	vtag( "compound" );
	b3HullMap shared;
	b3HullMap_init( &shared );
	b3HullMap_reserve( &shared, hullCount );
	int sharedCount = 0;
	for ( int i = 0; i < hullCount; ++i )
	{
		b3HullMap_itr itr = b3HullMap_get_or_insert( &shared, hulls[i], sharedCount );
		int index = itr.data->val;
		vi32( index );
		if ( index == sharedCount )
		{
			sharedCount += 1;
		}
	}
	vi32( sharedCount );
	vi32( (int)b3HullMap_size( &shared ) );
	vend();
	b3HullMap_cleanup( &shared );
	b3HullMap_cleanup( &map );

	vtag( "compare" );
	for ( int i = 0; i < hullCount; ++i )
	{
		for ( int j = 0; j < hullCount; ++j )
		{
			vbool( b3CompareHullData( hulls[i], hulls[j] ) );
		}
	}
	vbool( b3CompareHullData( &box1.base, &box2.base ) );
	vbool( b3CompareHullData( &box1.base, &box3.base ) );
	vend();

	for ( int i = 0; i < hullCount; ++i )
	{
		destroyHull( hulls[i] );
	}
	(void)keys;
}

static void phase2D( int k )
{
	vseed( 15000 + k );
	b3Point2D pts[64];
	b3Point2D hull[130];
	int count = vrandint( 0, 25 );
	int grid = vrandint( 0, 2 );
	for ( int i = 0; i < count; ++i )
	{
		if ( grid == 1 )
		{
			int gx = vrandint( 0, 4 );
			int gy = vrandint( 0, 4 );
			float jx = vrandf( -0.001f, 0.001f );
			float jy = vrandf( -0.001f, 0.001f );
			pts[i].p.x = 0.1f * (float)gx + jx;
			pts[i].p.y = 0.1f * (float)gy + jy;
		}
		else
		{
			pts[i].p.x = vrandf( -1.0f, 1.0f );
			pts[i].p.y = vrandf( -1.0f, 1.0f );
		}
		pts[i].separation = vrandf( -0.1f, 0.1f );
		pts[i].originalIndex = i;
	}

	int hullCount = b3Hull2D( pts, count, hull );
	vtag( "hull2d" );
	vi32( count );
	vi32( hullCount );
	for ( int i = 0; i < hullCount; ++i )
	{
		vf32( hull[i].p.x );
		vf32( hull[i].p.y );
		vf32( hull[i].separation );
		vi32( hull[i].originalIndex );
	}
	vend();

	int target = 3 + vrandint( 0, 6 );
	int simplified = b3SimplifyHull2D( hull, hullCount, target );
	vtag( "simplify" );
	vi32( target );
	vi32( simplified );
	for ( int i = 0; i < simplified; ++i )
	{
		vf32( hull[i].p.x );
		vf32( hull[i].p.y );
		vi32( hull[i].originalIndex );
	}
	vend();
}

static void phaseLimits( int k )
{
	vseed( 18000 + k );

	// Dense sphere clouds overflow the face or edge limits at high vertex budgets.
	int n = 200 + 100 * k;
	for ( int i = 0; i < n; ++i )
	{
		b3Vec3 v = vrandvec( -1.0f, 1.0f );
		g_points[i] = b3Normalize( v );
	}
	int limits[5] = { 128, 100, 80, 66, 64 };
	for ( int i = 0; i < 5; ++i )
	{
		b3HullData* hull = b3CreateHull( g_points, n, limits[i] );
		vtag( "limit" );
		vi32( n );
		vi32( limits[i] );
		vend();
		printHull( "hull", hull );
		destroyHull( hull );
	}

	printHull( "null", b3CloneHull( NULL ) );
	b3Transform xf = vrandxf( 1.0f );
	printHull( "null", b3CloneAndTransformHull( NULL, xf, b3Vec3_one ) );
}

// Corrupts a clone and compares the validating b3IsValidHull body against the C copy.
static void phaseValidate( int k )
{
	vseed( 21000 + k );
	int n = vrandint( 8, 60 );
	for ( int i = 0; i < n; ++i )
	{
		g_points[i] = vrandvec( -1.0f, 1.0f );
	}
	b3HullData* hull = b3CreateHull( g_points, n, n );
	b3HullData* clone = b3CloneHull( hull );
	b3HullVertex* vertices = (b3HullVertex*)b3GetHullVertices( clone );
	b3HullHalfEdge* edges = (b3HullHalfEdge*)b3GetHullEdges( clone );
	b3Plane* planes = (b3Plane*)b3GetHullPlanes( clone );
	int kind = k % 10;
	int v = vrandint( 0, clone->vertexCount );
	int e = vrandint( 0, clone->edgeCount );
	int f = vrandint( 0, clone->faceCount );
	if ( kind == 0 )
	{
		clone->version ^= 1;
	}
	else if ( kind == 1 )
	{
		vertices[v].edge = (uint8_t)( ( vertices[v].edge + 1 ) % clone->edgeCount );
	}
	else if ( kind == 2 )
	{
		edges[e].twin = (uint8_t)( ( edges[e].twin + 2 ) % clone->edgeCount );
	}
	else if ( kind == 3 )
	{
		edges[e].next = edges[e].twin;
	}
	else if ( kind == 4 )
	{
		planes[f].offset = -planes[f].offset - 10.0f;
	}
	else if ( kind == 5 )
	{
		clone->volume = 0.0f;
	}
	else if ( kind == 6 )
	{
		clone->surfaceArea = -1.0f;
	}
	else if ( kind == 7 )
	{
		clone->innerRadius = 0.0f;
	}
	else if ( kind == 8 )
	{
		edges[e].origin = (uint8_t)( ( edges[e].origin + 1 ) % clone->vertexCount );
	}
	else
	{
		edges[e].face = (uint8_t)( ( edges[e].face + 1 ) % clone->faceCount );
	}
	vtag( "validate" );
	vi32( kind );
	vbool( refValidateHull( hull ) );
	vbool( refValidateHull( clone ) );
	vbool( b3CompareHullData( hull, clone ) );
	vend();
	destroyHull( clone );
	destroyHull( hull );
}

int main( void )
{
	b3SetLogFcn( logToVectors );

	for ( int set = 0; set < HULL_SET_COUNT; ++set )
	{
		phaseSample( set );
	}

	phaseWheel();

	for ( int k = 0; k < 720; ++k )
	{
		phaseRandom( k );
	}

	for ( int k = 0; k < 60; ++k )
	{
		phaseShapes( k );
	}

	for ( int k = 0; k < 60; ++k )
	{
		phaseBoxes( k );
	}

	phaseMap();

	for ( int k = 0; k < 400; ++k )
	{
		phase2D( k );
	}

	for ( int k = 0; k < 8; ++k )
	{
		phaseLimits( k );
	}

	for ( int k = 0; k < 200; ++k )
	{
		phaseValidate( k );
	}

	return 0;
}
