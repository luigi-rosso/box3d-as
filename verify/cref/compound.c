// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for the compound part of lib/compound.as

#include "common.h"

#include "compound.h"
#include "shape.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <stdlib.h>

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

static void pcast( b3CastOutput o )
{
	vbool( o.hit );
	vf32( o.fraction );
	pv3( o.normal );
	pv3( o.point );
	vi32( o.iterations );
	vi32( o.triangleIndex );
	vi32( o.childIndex );
	vi32( o.materialIndex );
}

static void pmat( b3SurfaceMaterial m )
{
	vf32( m.friction );
	vf32( m.restitution );
	vf32( m.rollingResistance );
	pv3( m.tangentVelocity );
	vu64( m.userMaterialId );
	vu32( m.customColor );
	vu32( m.padding );
}

static void phull( const b3HullData* h )
{
	vu64( h->version );
	vu64( h->hash );
	vi32( h->byteCount );
	vi32( h->vertexCount );
	vi32( h->edgeCount );
	vi32( h->faceCount );
	pab( h->aabb );
	vf32( h->volume );
}

static void pmesh( const b3MeshData* m )
{
	vu64( m->version );
	vu64( m->hash );
	vi32( m->byteCount );
	vi32( m->vertexCount );
	vi32( m->triangleCount );
	vi32( m->materialCount );
	pab( m->bounds );
}

static b3Vec3 randVec( float lo, float hi )
{
	b3Vec3 v;
	v.x = vrandf( lo, hi );
	v.y = vrandf( lo, hi );
	v.z = vrandf( lo, hi );
	return v;
}

static b3Vec3 randInBox( b3AABB box, float pad )
{
	b3Vec3 v;
	v.x = vrandf( box.lowerBound.x - pad, box.upperBound.x + pad );
	v.y = vrandf( box.lowerBound.y - pad, box.upperBound.y + pad );
	v.z = vrandf( box.lowerBound.z - pad, box.upperBound.z + pad );
	return v;
}

static b3Quat randQuat( void )
{
	b3Quat q;
	q.v = randVec( -1.0f, 1.0f );
	q.s = vrandf( -1.0f, 1.0f );
	return b3NormalizeQuat( q );
}

static b3Transform randTransform( float range )
{
	b3Transform t;
	t.p = randVec( -range, range );
	t.q = randQuat();
	return t;
}

#define HULL_COUNT 10
#define MESH_COUNT 7
#define MATERIAL_COUNT 7

static b3BoxHull g_boxHulls[3];
static const b3HullData* g_hulls[HULL_COUNT];
static b3MeshData* g_meshes[MESH_COUNT];
static b3SurfaceMaterial g_materials[MATERIAL_COUNT];

static void buildPools( void )
{
	vseed( 77 );

	g_boxHulls[0] = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	g_boxHulls[1] = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	g_boxHulls[2] = b3MakeOffsetBoxHull( 0.3f, 0.6f, 0.2f, (b3Vec3){ 0.1f, -0.2f, 0.3f } );
	g_hulls[0] = &g_boxHulls[0].base;
	g_hulls[1] = &g_boxHulls[1].base;
	g_hulls[2] = &g_boxHulls[2].base;
	g_hulls[3] = b3CreateCylinder( 1.0f, 0.4f, 0.0f, 12 );
	g_hulls[4] = b3CreateCone( 1.0f, 0.5f, 0.2f, 10 );

	b3Vec3 points[40];
	for ( int i = 5; i < HULL_COUNT; ++i )
	{
		int count = vrandint( 8, 40 );
		for ( int j = 0; j < count; ++j )
		{
			points[j] = randVec( -0.8f, 0.8f );
		}
		g_hulls[i] = b3CreateHull( points, count, 32 );
	}

	// the same point set twice gives identical content
	g_hulls[9] = b3CreateHull( b3GetHullPoints( g_hulls[8] ), g_hulls[8]->vertexCount, 32 );

	g_meshes[0] = b3CreateBoxMesh( (b3Vec3){ 0.0f, 0.0f, 0.0f }, (b3Vec3){ 1.0f, 0.5f, 1.0f }, true );
	g_meshes[1] = b3CreateBoxMesh( (b3Vec3){ 0.0f, 0.0f, 0.0f }, (b3Vec3){ 1.0f, 0.5f, 1.0f }, true );
	g_meshes[2] = b3CreateGridMesh( 6, 5, 0.5f, 3, true );
	g_meshes[3] = b3CreateTorusMesh( 12, 8, 1.0f, 0.25f );
	g_meshes[4] = b3CreateWaveMesh( 8, 8, 0.4f, 0.3f, 0.5f, 0.7f );
	g_meshes[5] = b3CreatePlatformMesh( (b3Vec3){ 0.0f, -0.5f, 0.0f }, 0.5f, 1.0f, 2.0f );
	g_meshes[6] = b3CreateGridMesh( 4, 4, 0.7f, 5, false );

	for ( int i = 0; i < 5; ++i )
	{
		b3SurfaceMaterial m = { 0 };
		m.friction = vrandf( 0.0f, 1.0f );
		m.restitution = vrandf( 0.0f, 1.0f );
		m.rollingResistance = vrandf( 0.0f, 0.2f );
		m.tangentVelocity = randVec( -1.0f, 1.0f );
		m.userMaterialId = vrand64();
		m.customColor = vrand32();
		g_materials[i] = m;
	}

	// a repeat and a signed zero friction, which memcmp tells apart from +0
	g_materials[0].friction = 0.0f;
	g_materials[5] = g_materials[2];
	g_materials[6] = g_materials[0];
	g_materials[6].friction = -0.0f;

	for ( int i = 0; i < HULL_COUNT; ++i )
	{
		vtag( "poolhull" );
		vi32( i );
		phull( g_hulls[i] );
		vend();
	}

	for ( int i = 0; i < MESH_COUNT; ++i )
	{
		vtag( "poolmesh" );
		vi32( i );
		pmesh( g_meshes[i] );
		vend();
	}
}

static int align8( int x )
{
	return ( x + 7 ) & ~7;
}

static void printChild( const b3CompoundData* compound, int childIndex )
{
	b3ChildShape child = b3GetCompoundChild( compound, childIndex );
	vtag( "child" );
	vi32( childIndex );
	vi32( child.type );
	pxf( child.transform );
	vi32( child.materialCount );
	for ( int i = 0; i < child.materialCount; ++i )
	{
		vu32( child.materialIndices[i] );
	}

	switch ( child.type )
	{
		case b3_capsuleShape:
			pv3( child.capsule.center1 );
			pv3( child.capsule.center2 );
			vf32( child.capsule.radius );
			break;
		case b3_hullShape:
			phull( child.hull );
			break;
		case b3_meshShape:
			pv3( child.mesh.scale );
			pmesh( child.mesh.data );
			break;
		case b3_sphereShape:
			pv3( child.sphere.center );
			vf32( child.sphere.radius );
			break;
		default:
			break;
	}
	vend();
}

static void printCompound( const b3CompoundData* c )
{
	vtag( "compound" );
	vbool( c != NULL );
	vend();
	if ( c == NULL )
	{
		return;
	}

	vtag( "head" );
	vu64( c->version );
	vi32( c->byteCount );
	pab( c->bounds );
	vi32( c->treeHeight );
	vi32( c->nodeOffset );
	vi32( c->nodeCount );
	vi32( c->materialOffset );
	vi32( c->materialCount );
	vi32( c->capsuleOffset );
	vi32( c->capsuleCount );
	vi32( c->hullOffset );
	vi32( c->hullCount );
	vi32( c->sharedHullCount );
	vi32( c->meshOffset );
	vi32( c->meshCount );
	vi32( c->sharedMeshCount );
	vi32( c->sphereOffset );
	vi32( c->sphereCount );
	vend();

	const b3TreeNode* nodes = (const b3TreeNode*)( (intptr_t)c + c->nodeOffset );
	for ( int i = 0; i < c->nodeCount; ++i )
	{
		vtag( "node" );
		pab( nodes[i].aabb );
		vu32( nodes[i].flagIndex );
		vi32( nodes[i].height );
		vend();
	}

	const b3SurfaceMaterial* materials = b3GetCompoundMaterials( c );
	for ( int i = 0; i < c->materialCount; ++i )
	{
		vtag( "mat" );
		pmat( materials[i] );
		vend();
	}

	for ( int i = 0; i < c->capsuleCount; ++i )
	{
		b3CompoundCapsule cc = b3GetCompoundCapsule( c, i );
		vtag( "cap" );
		pv3( cc.capsule.center1 );
		pv3( cc.capsule.center2 );
		vf32( cc.capsule.radius );
		vu32( cc.materialIndex );
		vu32( cc.padding );
		vend();
	}

	// Shared hull blobs follow the instance array in shared index order
	int sharedHullOffsets[256];
	{
		int offset = c->hullOffset + align8( c->hullCount * 36 );
		for ( int i = 0; i < c->sharedHullCount; ++i )
		{
			sharedHullOffsets[i] = offset;
			const b3HullData* h = (const b3HullData*)( (intptr_t)c + offset );
			vtag( "shull" );
			phull( h );
			vend();
			offset += align8( h->byteCount );
		}
	}

	for ( int i = 0; i < c->hullCount; ++i )
	{
		b3CompoundHull ch = b3GetCompoundHull( c, i );
		int offset = (int)( (intptr_t)ch.hull - (intptr_t)c );
		int shared = -1;
		for ( int j = 0; j < c->sharedHullCount; ++j )
		{
			if ( sharedHullOffsets[j] == offset )
			{
				shared = j;
			}
		}
		vtag( "hull" );
		pxf( ch.transform );
		vi32( shared );
		vu32( ch.materialIndex );
		vu32( ch.padding );
		vend();
	}

	int meshMaterialTotal = 0;
	for ( int i = 0; i < c->meshCount; ++i )
	{
		meshMaterialTotal += b3GetCompoundMesh( c, i ).materialCount;
	}
	int meshMaterialMapOffset = c->meshOffset + align8( c->meshCount * 52 );

	int sharedMeshOffsets[256];
	{
		int offset = meshMaterialMapOffset + align8( meshMaterialTotal * 2 );
		for ( int i = 0; i < c->sharedMeshCount; ++i )
		{
			sharedMeshOffsets[i] = offset;
			const b3MeshData* m = (const b3MeshData*)( (intptr_t)c + offset );
			vtag( "smesh" );
			pmesh( m );
			vend();
			offset += align8( m->byteCount );
		}
		if ( c->sharedMeshCount > 0 )
		{
			vtag( "sphereoff" );
			vbool( offset == c->sphereOffset );
			vend();
		}
	}

	const uint16_t* mmap = (const uint16_t*)( (intptr_t)c + meshMaterialMapOffset );
	vtag( "mmap" );
	vi32( meshMaterialTotal );
	for ( int i = 0; i < meshMaterialTotal; ++i )
	{
		vu32( mmap[i] );
	}
	vend();

	for ( int i = 0; i < c->meshCount; ++i )
	{
		b3CompoundMesh cm = b3GetCompoundMesh( c, i );
		int offset = (int)( (intptr_t)cm.meshData - (intptr_t)c );
		int shared = -1;
		for ( int j = 0; j < c->sharedMeshCount; ++j )
		{
			if ( sharedMeshOffsets[j] == offset )
			{
				shared = j;
			}
		}
		vtag( "mesh" );
		pxf( cm.transform );
		pv3( cm.scale );
		vi32( shared );
		vi32( (int)( cm.materialIndices - mmap ) );
		vi32( cm.materialCount );
		vend();
	}

	for ( int i = 0; i < c->sphereCount; ++i )
	{
		b3CompoundSphere cs = b3GetCompoundSphere( c, i );
		vtag( "sph" );
		pv3( cs.sphere.center );
		vf32( cs.sphere.radius );
		vu32( cs.materialIndex );
		vu32( cs.padding );
		vend();
	}

	int childCount = c->capsuleCount + c->hullCount + c->meshCount + c->sphereCount;
	for ( int i = 0; i < childCount; ++i )
	{
		printChild( c, i );
	}
}

static b3SurfaceMaterial randMaterial( void )
{
	int index = vrandint( 0, MATERIAL_COUNT );
	return g_materials[index];
}

typedef struct QueryContext
{
	int stopAfter;
	int count;
} QueryContext;

static bool queryFcn( const b3CompoundData* compound, int childIndex, void* context )
{
	(void)compound;
	QueryContext* qc = context;
	qc->count += 1;
	vtag( "qhit" );
	vi32( childIndex );
	vend();
	return qc->stopAfter == 0 || qc->count < qc->stopAfter;
}

// Proxies in a given buffer: a point, a segment, or the corners of an oriented box
static b3ShapeProxy randProxy( b3Vec3* buffer, b3Vec3 center )
{
	int kind = vrandint( 0, 3 );
	b3ShapeProxy proxy;
	proxy.points = buffer;
	if ( kind == 0 )
	{
		buffer[0] = center;
		proxy.count = 1;
		proxy.radius = vrandf( 0.05f, 0.8f );
	}
	else if ( kind == 1 )
	{
		buffer[0] = center;
		b3Vec3 d = randVec( -1.0f, 1.0f );
		buffer[1] = b3Add( center, d );
		proxy.count = 2;
		proxy.radius = vrandf( 0.0f, 0.5f );
	}
	else
	{
		b3Vec3 h = randVec( 0.05f, 0.7f );
		b3Quat q = randQuat();
		for ( int i = 0; i < 8; ++i )
		{
			b3Vec3 corner = { ( i & 1 ) ? h.x : -h.x, ( i & 2 ) ? h.y : -h.y, ( i & 4 ) ? h.z : -h.z };
			buffer[i] = b3Add( center, b3RotateVector( q, corner ) );
		}
		proxy.count = 8;
		int round = vrandint( 0, 2 );
		proxy.radius = round == 1 ? vrandf( 0.0f, 0.2f ) : 0.0f;
	}
	return proxy;
}

// A point on or inside a child, in the compound frame
static b3Vec3 childPoint( const b3CompoundData* compound, int childIndex )
{
	b3ChildShape child = b3GetCompoundChild( compound, childIndex );
	if ( child.type == b3_capsuleShape )
	{
		return child.capsule.center1;
	}
	if ( child.type == b3_sphereShape )
	{
		return child.sphere.center;
	}
	return child.transform.p;
}

static b3Vec3 randTarget( const b3CompoundData* compound, int childCount, b3AABB bounds )
{
	int aimed = vrandint( 0, 2 );
	if ( aimed == 0 )
	{
		return randInBox( bounds, 0.0f );
	}
	int childIndex = vrandint( 0, childCount );
	b3Vec3 p = childPoint( compound, childIndex );
	b3Vec3 jitter = randVec( -0.3f, 0.3f );
	return b3Add( p, jitter );
}

static void runCompound( int index )
{
	vseed( 5000 + index );

	b3CompoundCapsuleDef capsules[64];
	b3CompoundHullDef hulls[64];
	b3CompoundMeshDef meshes[8];
	b3CompoundSphereDef spheres[64];
	b3SurfaceMaterial meshMaterials[8][8];

	b3CompoundDef def = { 0 };
	bool big = index % 10 == 9;
	def.capsuleCount = big ? 40 : vrandint( 0, 6 );
	def.hullCount = big ? 30 : vrandint( 0, 6 );
	def.meshCount = big ? 4 : vrandint( 0, 3 );
	def.sphereCount = big ? 60 : vrandint( 0, 6 );
	if ( def.capsuleCount + def.hullCount + def.meshCount + def.sphereCount == 0 )
	{
		def.sphereCount = 1;
	}

	// The first compound shares hulls and meshes by pointer and by identical content
	static const int forcedHulls[4] = { 0, 1, 8, 9 };
	if ( index == 0 )
	{
		def.hullCount = 4;
		def.meshCount = 2;
	}
	def.capsules = capsules;
	def.hulls = hulls;
	def.meshes = meshes;
	def.spheres = spheres;

	float range = big ? 10.0f : 4.0f;
	for ( int i = 0; i < def.capsuleCount; ++i )
	{
		b3Vec3 c1 = randVec( -range, range );
		b3Vec3 d = randVec( -1.0f, 1.0f );
		capsules[i].capsule.center1 = c1;
		capsules[i].capsule.center2 = b3Add( c1, d );
		capsules[i].capsule.radius = vrandf( 0.05f, 0.5f );
		capsules[i].material = randMaterial();
	}

	for ( int i = 0; i < def.hullCount; ++i )
	{
		int h = vrandint( 0, HULL_COUNT );
		if ( index == 0 )
		{
			h = forcedHulls[i];
		}
		hulls[i].hull = g_hulls[h];
		hulls[i].transform = randTransform( range );
		hulls[i].material = randMaterial();
	}

	for ( int i = 0; i < def.meshCount; ++i )
	{
		int m = vrandint( 0, MESH_COUNT );
		if ( index == 0 )
		{
			m = i;
		}
		meshes[i].meshData = g_meshes[m];
		meshes[i].transform = randTransform( range );
		b3Vec3 scale;
		scale.x = vrandf( 0.5f, 2.0f );
		scale.y = vrandf( 0.5f, 2.0f );
		scale.z = vrandf( 0.5f, 2.0f );
		int flip = vrandint( 0, 4 );
		if ( flip == 0 )
		{
			scale.y = -scale.y;
		}
		meshes[i].scale = scale;
		meshes[i].materialCount = g_meshes[m]->materialCount;
		for ( int j = 0; j < meshes[i].materialCount; ++j )
		{
			meshMaterials[i][j] = randMaterial();
		}
		meshes[i].materials = meshMaterials[i];
	}

	for ( int i = 0; i < def.sphereCount; ++i )
	{
		spheres[i].sphere.center = randVec( -range, range );
		spheres[i].sphere.radius = vrandf( 0.05f, 0.6f );
		spheres[i].material = randMaterial();
	}

	b3CompoundData* compound = b3CreateCompound( &def );
	vtag( "build" );
	vi32( index );
	vend();
	printCompound( compound );
	if ( compound == NULL )
	{
		return;
	}

	b3AABB bounds = compound->bounds;
	int childCount = def.capsuleCount + def.hullCount + def.meshCount + def.sphereCount;

	for ( int i = 0; i < 4; ++i )
	{
		b3Transform xf = randTransform( 5.0f );
		vtag( "aabb" );
		pab( b3ComputeCompoundAABB( compound, xf ) );
		vend();
	}

	for ( int i = 0; i < 12; ++i )
	{
		b3Vec3 center = randInBox( bounds, 1.0f );
		b3Vec3 h = randVec( 0.1f, 3.0f );
		b3AABB box = { b3Sub( center, h ), b3Add( center, h ) };
		QueryContext qc = { 0 };
		qc.stopAfter = vrandint( 0, 6 );
		b3QueryCompound( compound, box, queryFcn, &qc );
		vtag( "query" );
		vi32( qc.count );
		vend();
	}

	for ( int i = 0; i < 40; ++i )
	{
		b3RayCastInput input;
		input.origin = randInBox( bounds, 3.0f );
		b3Vec3 target = randTarget( compound, childCount, bounds );
		float stretch = vrandf( 0.5f, 2.0f );
		input.translation = b3MulSV( stretch, b3Sub( target, input.origin ) );
		int partial = vrandint( 0, 3 );
		input.maxFraction = partial == 0 ? vrandf( 0.1f, 1.0f ) : 1.0f;
		b3CastOutput output = b3RayCastCompound( compound, &input );
		vtag( "ray" );
		pcast( output );
		vend();
	}

	for ( int i = 0; i < 30; ++i )
	{
		b3Vec3 buffer[8];
		b3Vec3 center = randInBox( bounds, 2.0f );
		b3ShapeCastInput input = { 0 };
		input.proxy = randProxy( buffer, center );
		b3Vec3 target = randTarget( compound, childCount, bounds );
		input.translation = b3Sub( target, center );
		int partial = vrandint( 0, 3 );
		input.maxFraction = partial == 0 ? vrandf( 0.1f, 1.0f ) : 1.0f;
		int encroach = vrandint( 0, 2 );
		input.canEncroach = encroach == 1;
		b3CastOutput output = b3ShapeCastCompound( compound, &input );
		vtag( "cast" );
		pcast( output );
		vend();
	}

	{
		b3ShapeCastInput empty = { 0 };
		empty.translation = (b3Vec3){ 1.0f, 0.0f, 0.0f };
		empty.maxFraction = 1.0f;
		vtag( "castempty" );
		pcast( b3ShapeCastCompound( compound, &empty ) );
		vend();
	}

	for ( int i = 0; i < 30; ++i )
	{
		b3Vec3 buffer[8];
		b3Transform xf = randTransform( 2.0f );
		b3Vec3 local = randTarget( compound, childCount, bounds );
		b3Vec3 jitter = randVec( -0.7f, 0.7f );
		b3Vec3 center = b3Add( b3TransformPoint( xf, local ), jitter );
		b3ShapeProxy proxy = randProxy( buffer, center );
		bool overlap = b3OverlapCompound( compound, xf, &proxy );
		vtag( "overlap" );
		vbool( overlap );
		vend();
	}

	for ( int i = 0; i < 30; ++i )
	{
		b3Capsule mover;
		mover.center1 = randTarget( compound, childCount, bounds );
		b3Vec3 d = randVec( -1.0f, 1.0f );
		mover.center2 = b3Add( mover.center1, d );
		mover.radius = vrandf( 0.05f, 1.0f );
		int capacity = vrandint( 1, 17 );
		b3PlaneResult planes[16];
		memset( planes, 0, sizeof( planes ) );
		int count = b3CollideMoverAndCompound( planes, capacity, compound, &mover );
		vtag( "mover" );
		vi32( capacity );
		vi32( count );
		vend();
		for ( int j = 0; j < count; ++j )
		{
			vtag( "plane" );
			pv3( planes[j].plane.normal );
			vf32( planes[j].plane.offset );
			pv3( planes[j].point );
			vi32( planes[j].triangleIndex );
			vi32( planes[j].childIndex );
			vi32( planes[j].materialIndex );
			vend();
		}
	}

	for ( int i = 0; i < 20; ++i )
	{
		b3Transform xf = randTransform( 3.0f );
		int childIndex = vrandint( 0, childCount );
		b3ChildShape child = b3GetCompoundChild( compound, childIndex );
		b3Sweep sweepA = b3MakeCompoundChildSweep( xf, child.transform );
		vtag( "sweep" );
		vi32( childIndex );
		pv3( sweepA.localCenter );
		pv3( sweepA.c1 );
		pv3( sweepA.c2 );
		pq( sweepA.q1 );
		pq( sweepA.q2 );
		vend();

		b3TOIInput input = { 0 };
		if ( child.type == b3_capsuleShape )
		{
			input.proxyA.points = &child.capsule.center1;
			input.proxyA.count = 2;
			input.proxyA.radius = child.capsule.radius;
		}
		else if ( child.type == b3_hullShape )
		{
			input.proxyA.points = b3GetHullPoints( child.hull );
			input.proxyA.count = child.hull->vertexCount;
			input.proxyA.radius = 0.0f;
		}
		else if ( child.type == b3_sphereShape )
		{
			input.proxyA.points = &child.sphere.center;
			input.proxyA.count = 1;
			input.proxyA.radius = child.sphere.radius;
		}
		else
		{
			continue;
		}

		b3Vec3 buffer[8];
		input.proxyB = randProxy( buffer, b3Vec3_zero );
		input.sweepA = sweepA;
		input.sweepB.localCenter = randVec( -0.2f, 0.2f );
		b3Vec3 near = b3TransformPoint( xf, child.transform.p );
		b3Vec3 offset = randVec( -3.0f, 3.0f );
		input.sweepB.c1 = b3Add( near, offset );
		b3Vec3 miss = randVec( -0.5f, 0.5f );
		input.sweepB.c2 = b3Add( b3Sub( near, offset ), miss );
		input.sweepB.q1 = randQuat();
		input.sweepB.q2 = randQuat();
		input.maxFraction = 1.0f;
		b3TOIOutput output = b3TimeOfImpact( &input );
		vtag( "toi" );
		vi32( output.state );
		pv3( output.point );
		pv3( output.normal );
		vf32( output.fraction );
		vf32( output.distance );
		vend();
	}

	b3DestroyCompound( compound );
}

int main( int argc, char** argv )
{
	int count = argc > 1 ? atoi( argv[1] ) : 60;
	buildPools();
	for ( int i = 0; i < count; ++i )
	{
		runCompound( i );
	}

	// Too many children
	{
		b3CompoundDef def = { 0 };
		def.sphereCount = B3_MAX_CHILD_SHAPES;
		b3CompoundSphereDef* spheres = calloc( def.sphereCount, sizeof( b3CompoundSphereDef ) );
		def.spheres = spheres;
		b3CompoundData* compound = b3CreateCompound( &def );
		vtag( "toomany" );
		vbool( compound != NULL );
		vend();
		free( spheres );
	}
	return 0;
}
