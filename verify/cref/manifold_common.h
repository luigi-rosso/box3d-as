// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Printers, the hull library and pose generators shared by the manifold drivers.
// verify/as/manifold_common.as mirrors every function here.

#pragma once

#include "distance_common.h"
#include "hull_data.h"

#include "hull.h"
#include "manifold.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <float.h>

#define POINT_BUFFER 16
#define HULL_CAPACITY 40
#define BOX_CAPACITY 8

static b3LocalManifoldPoint g_pointBuffer[POINT_BUFFER];
static b3LocalManifold g_manifold;

// Sentinels everywhere, so a field or slot a collide function skips prints the same on both sides.
static void resetManifold( int offset )
{
	g_manifold.normal = ( b3Vec3 ){ 11.0f, 12.0f, 13.0f };
	g_manifold.triangleNormal = ( b3Vec3 ){ 14.0f, 15.0f, 16.0f };
	g_manifold.points = g_pointBuffer + offset;
	g_manifold.pointCount = 77;
	g_manifold.triangleIndex = 78;
	g_manifold.i1 = 79;
	g_manifold.i2 = 80;
	g_manifold.i3 = 81;
	g_manifold.squaredDistance = 82.0f;
	g_manifold.feature = 83;
	g_manifold.triangleFlags = 84;

	for ( int k = 0; k < POINT_BUFFER; ++k )
	{
		b3LocalManifoldPoint* p = g_pointBuffer + k;
		p->point = ( b3Vec3 ){ 100.0f + (float)k, 200.0f + (float)k, 300.0f + (float)k };
		p->separation = 400.0f + (float)k;
		p->pair.owner1 = (uint8_t)k;
		p->pair.index1 = (uint8_t)( k + 1 );
		p->pair.owner2 = (uint8_t)( k + 2 );
		p->pair.index2 = (uint8_t)( k + 3 );
		p->triangleIndex = 500 + k;
	}
}

static void ppair( b3FeaturePair pair )
{
	vi32( pair.owner1 );
	vi32( pair.index1 );
	vi32( pair.owner2 );
	vi32( pair.index2 );
	vu32( b3MakeFeatureId( pair ) );
}

// The manifold fields, then every buffer slot below slotCount.
static void pmanifold( int slotCount )
{
	vtag( "m" );
	vi32( g_manifold.pointCount );
	pv3( g_manifold.normal );
	pv3( g_manifold.triangleNormal );
	vi32( g_manifold.triangleIndex );
	vi32( g_manifold.i1 );
	vi32( g_manifold.i2 );
	vi32( g_manifold.i3 );
	vf32( g_manifold.squaredDistance );
	vi32( g_manifold.feature );
	vi32( g_manifold.triangleFlags );
	vend();

	for ( int k = 0; k < slotCount; ++k )
	{
		const b3LocalManifoldPoint* p = g_pointBuffer + k;
		vtag( "p" );
		vi32( k );
		pv3( p->point );
		vf32( p->separation );
		ppair( p->pair );
		vi32( p->triangleIndex );
		vend();
	}
}

static void psat( b3SATCache c )
{
	vf32( c.separation );
	vi32( c.type );
	vi32( c.indexA );
	vi32( c.indexB );
	vi32( c.hit );
}

static void psimplexCache( b3SimplexCache c )
{
	vf32( c.metric );
	vi32( c.count );
	for ( int i = 0; i < 4; ++i )
	{
		vi32( c.indexA[i] );
	}
	for ( int i = 0; i < 4; ++i )
	{
		vi32( c.indexB[i] );
	}
}

static void paxis( b3SeparatingAxis a )
{
	pv3( a.normal );
	vf32( a.separation );
	vi32( a.indexA );
	vi32( a.indexB );
	vi32( a.type );
}

static void pquery( b3AxisQuery q )
{
	paxis( q.faceA );
	paxis( q.faceB );
	paxis( q.edge );
	vi32( q.separatedFeature );
}

static void pplane( b3Plane p )
{
	pv3( p.normal );
	vf32( p.offset );
}

static void phull( int index, const b3HullData* hull )
{
	vtag( "hull" );
	vi32( index );
	vi32( hull->vertexCount );
	vi32( hull->edgeCount );
	vi32( hull->faceCount );
	pab( hull->aabb );
	vf32( hull->surfaceArea );
	vf32( hull->volume );
	vf32( hull->innerRadius );
	pv3( hull->center );
	pm3( hull->centralInertia );
	vend();

	const b3Vec3* points = b3GetHullPoints( hull );
	const b3HullVertex* vertices = b3GetHullVertices( hull );
	for ( int i = 0; i < hull->vertexCount; ++i )
	{
		vtag( "hv" );
		pv3( points[i] );
		vi32( vertices[i].edge );
		vend();
	}

	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const float* cosines = b3GetHullEdgeCosines( hull );
	for ( int i = 0; i < hull->edgeCount; ++i )
	{
		vtag( "he" );
		vi32( edges[i].next );
		vi32( edges[i].twin );
		vi32( edges[i].origin );
		vi32( edges[i].face );
		if ( ( i & 1 ) == 0 )
		{
			vf32( cosines[i >> 1] );
		}
		vend();
	}

	const b3Plane* planes = b3GetHullPlanes( hull );
	const b3HullFace* faces = b3GetHullFaces( hull );
	for ( int i = 0; i < hull->faceCount; ++i )
	{
		vtag( "hf" );
		pplane( planes[i] );
		vi32( faces[i].edge );
		vend();
	}

	int vertexStride = b3GetHullSoaStride( hull->vertexCount );
	const float* soaVertices = b3GetHullSoaVertices( hull );
	vtag( "hsv" );
	for ( int i = 0; i < 3 * vertexStride; ++i )
	{
		vf32( soaVertices[i] );
	}
	vend();

	int faceStride = b3GetHullSoaStride( hull->faceCount );
	const float* soaNormals = b3GetHullSoaNormals( hull );
	vtag( "hsn" );
	for ( int i = 0; i < 3 * faceStride; ++i )
	{
		vf32( soaNormals[i] );
	}
	vend();
}

static const b3HullData* g_hulls[HULL_CAPACITY];
static int g_hullCount;
static b3BoxHull g_boxes[BOX_CAPACITY];
static int g_boxCount;

static void addBox( b3BoxHull box )
{
	g_boxes[g_boxCount] = box;
	g_hulls[g_hullCount] = &g_boxes[g_boxCount].base;
	g_boxCount += 1;
	g_hullCount += 1;
}

static void addHull( b3HullData* hull )
{
	vtag( "add" );
	vi32( g_hullCount );
	vbool( hull != NULL );
	vend();
	if ( hull != NULL )
	{
		g_hulls[g_hullCount] = hull;
		g_hullCount += 1;
	}
}

static b3Vec3 g_cloud[B3_MAX_HULL_VERTICES * 16];

static b3HullData* hullFromSet( int set )
{
	int offset = hullSets[3 * set + 0];
	int count = hullSets[3 * set + 1];
	int maxCount = hullSets[3 * set + 2];
	for ( int i = 0; i < count; ++i )
	{
		int k = offset + i;
		g_cloud[i].x = vbitsf( hullPointBits[3 * k + 0] );
		g_cloud[i].y = vbitsf( hullPointBits[3 * k + 1] );
		g_cloud[i].z = vbitsf( hullPointBits[3 * k + 2] );
	}
	return b3CreateHull( g_cloud, count, maxCount );
}

static b3HullData* randomCloudHull( int count, float s )
{
	for ( int i = 0; i < count; ++i )
	{
		g_cloud[i] = rv( -s, s );
	}
	return b3CreateHull( g_cloud, count, B3_MAX_HULL_VERTICES );
}

// The AS default log handler is silent too
static void silentLog( const char* message )
{
	(void)message;
}

static void buildHulls( void )
{
	b3SetLogFcn( silentLog );
	vseed( 0x4d414e49464f4c44ull );
	g_hullCount = 0;
	g_boxCount = 0;

	addBox( b3MakeCubeHull( 0.5f ) );
	addBox( b3MakeBoxHull( 1.0f, 0.1f, 2.0f ) );
	addBox( b3MakeBoxHull( 0.05f, 0.5f, 0.05f ) );
	addBox( b3MakeOffsetBoxHull( 0.3f, 0.4f, 0.5f, ( b3Vec3 ){ 0.2f, -0.1f, 0.3f } ) );

	b3Transform boxTransform;
	boxTransform.p = ( b3Vec3 ){ 0.1f, 0.2f, -0.3f };
	boxTransform.q = b3MakeQuatFromAxisAngle( b3Normalize( ( b3Vec3 ){ 1.0f, 1.0f, 0.0f } ), 0.7f );
	addBox( b3MakeTransformedBoxHull( 0.6f, 0.2f, 0.4f, boxTransform ) );

	b3Transform scaleTransform;
	scaleTransform.p = ( b3Vec3 ){ 0.0f, 0.1f, 0.0f };
	scaleTransform.q = b3MakeQuatFromAxisAngle( ( b3Vec3 ){ 0.0f, 0.0f, 1.0f }, 0.5f );
	addBox( b3MakeScaledBoxHull( ( b3Vec3 ){ 0.5f, 0.4f, 0.3f }, scaleTransform, ( b3Vec3 ){ 1.0f, 2.0f, 0.5f } ) );

	addHull( b3CreateCylinder( 1.0f, 0.4f, 0.0f, 12 ) );
	addHull( b3CreateCylinder( 0.2f, 1.0f, 0.1f, 24 ) );
	addHull( b3CreateCone( 1.0f, 0.5f, 0.1f, 10 ) );
	addHull( b3CreateCone( 0.6f, 0.3f, 0.3f, 6 ) );

	int rockIndex = g_hullCount;
	addHull( b3CreateRock( 0.5f ) );
	addHull( b3CreateComplexHull( 0.6f ) );

	static const int sets[] = { 0, 1, 2, 3, 4, 41, 42, 43, 44 };
	int setBase = g_hullCount;
	for ( int i = 0; i < 9; ++i )
	{
		addHull( hullFromSet( sets[i] ) );
	}

	b3Transform cloneTransform;
	cloneTransform.p = ( b3Vec3 ){ -0.2f, 0.1f, 0.05f };
	cloneTransform.q = b3MakeQuatFromAxisAngle( b3Normalize( ( b3Vec3 ){ 1.0f, -2.0f, 0.5f } ), 1.1f );
	addHull( b3CloneAndTransformHull( g_hulls[rockIndex], cloneTransform, ( b3Vec3 ){ 1.5f, 0.5f, 1.0f } ) );
	addHull( b3CloneAndTransformHull( g_hulls[setBase + 4], b3Transform_identity, ( b3Vec3 ){ 0.3f, 2.0f, 1.0f } ) );

	addHull( randomCloudHull( 8, 0.6f ) );
	addHull( randomCloudHull( 20, 0.7f ) );
	addHull( randomCloudHull( 60, 1.2f ) );

	for ( int i = 0; i < g_hullCount; ++i )
	{
		phull( i, g_hulls[i] );
	}
}

// Largest dot of the rotated hull points with d.
static float hullExtent( const b3HullData* hull, b3Quat q, b3Vec3 d )
{
	const b3Vec3* points = b3GetHullPoints( hull );
	float best = -FLT_MAX;
	for ( int i = 0; i < hull->vertexCount; ++i )
	{
		float e = b3Dot( b3RotateVector( q, points[i] ), d );
		if ( e > best )
		{
			best = e;
		}
	}
	return best;
}

static b3Vec3 axisDirection( int k )
{
	b3Vec3 v = b3Vec3_zero;
	float s = ( k & 1 ) ? -1.0f : 1.0f;
	if ( k >> 1 == 0 )
	{
		v.x = s;
	}
	else if ( k >> 1 == 1 )
	{
		v.y = s;
	}
	else
	{
		v.z = s;
	}
	return v;
}

// Identity or a quarter or half turn about a principal axis.
static b3Quat rquatAligned( void )
{
	int k = vrandint( 0, 7 );
	if ( k == 0 )
	{
		return b3Quat_identity;
	}
	int axis = vrandint( 0, 3 );
	float angle = k < 4 ? 0.5f * B3_PI : B3_PI;
	return b3MakeQuatFromAxisAngle( axisDirection( 2 * axis ), angle );
}

// Two 45 degree turns, so edges cross.
static b3Quat rquatEdge( void )
{
	int a = vrandint( 0, 3 );
	int b = vrandint( 0, 3 );
	float jitter = vrandf( -0.05f, 0.05f );
	b3Quat qa = b3MakeQuatFromAxisAngle( axisDirection( 2 * a ), 0.25f * B3_PI );
	b3Quat qb = b3MakeQuatFromAxisAngle( axisDirection( 2 * b ), 0.25f * B3_PI + jitter );
	return b3NormalizeQuat( b3MulQuat( qa, qb ) );
}

#define POSE_MODE_COUNT 8

// Pose of a shape with extent eB along -u (already rotated by q) against a hull A extent along u.
// modes: 0 near touch, 1 deep, 2 separated, 3 aligned face, 4 crossing edges, 5 coincident, 6 exact touch, 7 far
static b3Transform placePose( float eA, float eB, b3Vec3 u, b3Quat q, float gap, int mode )
{
	b3Transform xf;
	xf.q = q;
	xf.p = b3MulSV( eA + eB + gap, u );
	if ( mode == 3 )
	{
		b3Vec3 lateral = rv( -0.2f, 0.2f );
		lateral = b3MulSub( lateral, b3Dot( lateral, u ), u );
		xf.p = b3Add( xf.p, lateral );
	}
	if ( mode == 7 )
	{
		xf.p = b3Add( xf.p, ( b3Vec3 ){ 900.0f, -1300.0f, 400.0f } );
	}
	return xf;
}

static b3Quat poseRotation( int mode )
{
	if ( mode == 3 )
	{
		return rquatAligned();
	}
	if ( mode == 4 )
	{
		return rquatEdge();
	}
	return rquatOrIdentity();
}

static b3Vec3 poseDirection( int mode )
{
	if ( mode == 3 )
	{
		int k = vrandint( 0, 6 );
		return axisDirection( k );
	}
	return runit();
}

static float poseGap( int mode )
{
	switch ( mode )
	{
		case 0:
			return vrandf( -0.05f, 0.05f );
		case 1:
			return vrandf( -0.6f, -0.05f );
		case 2:
			return vrandf( 0.03f, 0.8f );
		case 3:
		case 4:
			return vrandf( -0.03f, 0.03f );
		case 6:
			return 0.0f;
		default:
			return vrandf( -0.1f, 0.1f );
	}
}

static b3Transform hullPose( const b3HullData* hullA, const b3HullData* hullB, int mode )
{
	if ( mode == 5 )
	{
		b3Transform xf;
		xf.q = rquatOrIdentity();
		xf.p = rv( -0.01f, 0.01f );
		return xf;
	}

	b3Quat q = poseRotation( mode );
	b3Vec3 u = poseDirection( mode );
	float gap = poseGap( mode );
	float eA = hullExtent( hullA, b3Quat_identity, u );
	float eB = hullExtent( hullB, q, b3Neg( u ) );
	return placePose( eA, eB, u, q, gap, mode );
}

// Small motion most steps, a repeat at step 1, a jump now and then.
static b3Transform advancePose( b3Transform xf, int step )
{
	if ( step == 0 )
	{
		return xf;
	}
	int kind = vrandint( 0, 8 );
	if ( kind == 0 )
	{
		xf.p = b3Add( xf.p, rv( -0.1f, 0.1f ) );
		xf.q = rotateBy( xf.q, 0.3f );
	}
	else
	{
		xf.p = b3Add( xf.p, rv( -0.005f, 0.005f ) );
		xf.q = rotateBy( xf.q, 0.01f );
	}
	return xf;
}

// Mostly the contact capacity, sometimes the edges of each capacity test.
static int pickCapacity( void )
{
	int k = vrandint( 0, 16 );
	if ( k < 11 )
	{
		return B3_MAX_MANIFOLD_POINTS;
	}
	if ( k < 13 )
	{
		return 8;
	}
	return vrandint( 0, 4 );
}

static int pickOffset( void )
{
	if ( vrand32() & 1 )
	{
		return 0;
	}
	return vrandint( 1, 5 );
}

static b3Capsule randomCapsule( void )
{
	b3Capsule c;
	c.center1 = rv( -0.5f, 0.5f );
	int kind = vrandint( 0, 8 );
	float length;
	if ( kind == 0 )
	{
		length = 0.0f;
	}
	else if ( kind == 1 )
	{
		length = vrandf( 0.0f, 0.01f );
	}
	else
	{
		length = vrandf( 0.1f, 2.0f );
	}
	b3Vec3 axis = runit();
	c.center2 = b3MulAdd( c.center1, length, axis );
	c.radius = vrandf( 0.02f, 0.5f );
	return c;
}

static b3Sphere randomSphere( void )
{
	b3Sphere s;
	s.center = rv( -0.5f, 0.5f );
	int kind = vrandint( 0, 8 );
	if ( kind == 0 )
	{
		s.radius = 0.0f;
	}
	else
	{
		s.radius = vrandf( 0.01f, 1.0f );
	}
	return s;
}
