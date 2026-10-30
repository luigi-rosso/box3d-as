// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/mesh_contact.as: b3ComputeMeshManifolds driven directly over pose sequences,
// with the contact and its triangle cache carried from call to call.

#include "common.h"

#include "contact.h"
#include "physics_world.h"
#include "shape.h"

#include "box3d/box3d.h"
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

static void pab( b3AABB a )
{
	pv3( a.lowerBound );
	pv3( a.upperBound );
}

static b3Vec3 randVec( float lo, float hi )
{
	b3Vec3 v;
	v.x = vrandf( lo, hi );
	v.y = vrandf( lo, hi );
	v.z = vrandf( lo, hi );
	return v;
}

static b3Quat randQuat( void )
{
	b3Quat q;
	q.v = randVec( -1.0f, 1.0f );
	q.s = vrandf( -1.0f, 1.0f );
	return b3NormalizeQuat( q );
}

// A small rotation about a random axis
static b3Quat smallRotation( float maxAngle )
{
	b3Vec3 axis = randVec( -1.0f, 1.0f );
	float angle = vrandf( -maxAngle, maxAngle );
	return b3MakeQuatFromAxisAngle( b3Normalize( axis ), angle );
}

#define MESH_COUNT 6
#define FIELD_COUNT 4
#define HULL_COUNT 5

static b3MeshData* g_meshes[MESH_COUNT];
static b3HeightFieldData* g_fields[FIELD_COUNT];
static b3BoxHull g_boxHulls[2];
static const b3HullData* g_hulls[HULL_COUNT];
static b3SurfaceMaterial g_materials[4];

static void buildPools( void )
{
	vseed( 91 );

	g_meshes[0] = b3CreateGridMesh( 12, 12, 0.5f, 3, true );
	g_meshes[1] = b3CreateWaveMesh( 16, 16, 0.4f, 0.4f, 0.5f, 0.7f );
	g_meshes[2] = b3CreateTorusMesh( 16, 10, 2.0f, 0.6f );
	g_meshes[3] = b3CreateBoxMesh( (b3Vec3){ 0.0f, 0.0f, 0.0f }, (b3Vec3){ 2.0f, 0.5f, 2.0f }, true );
	g_meshes[4] = b3CreateGridMesh( 60, 60, 0.1f, 0, true );
	g_meshes[5] = b3CreatePlatformMesh( (b3Vec3){ 0.0f, 0.0f, 0.0f }, 1.0f, 2.0f, 4.0f );

	g_fields[0] = b3CreateWave( 20, 20, (b3Vec3){ 0.5f, 1.0f, 0.5f }, 0.3f, 0.4f, true );
	g_fields[1] = b3CreateGrid( 16, 16, (b3Vec3){ 0.6f, 1.0f, 0.6f }, false );
	{
		int countX = 14;
		int countZ = 12;
		float heights[14 * 12];
		uint8_t materials[13 * 11];
		for ( int i = 0; i < countX * countZ; ++i )
		{
			heights[i] = vrandf( -0.4f, 0.4f );
		}
		for ( int i = 0; i < ( countX - 1 ) * ( countZ - 1 ); ++i )
		{
			int m = vrandint( 0, 12 );
			materials[i] = m == 11 ? B3_HEIGHT_FIELD_HOLE : (uint8_t)( m % 3 );
		}
		b3HeightFieldDef def = { 0 };
		def.heights = heights;
		def.materialIndices = materials;
		def.scale = (b3Vec3){ 0.5f, 1.0f, 0.5f };
		def.countX = countX;
		def.countZ = countZ;
		def.globalMinimumHeight = -0.5f;
		def.globalMaximumHeight = 0.5f;
		g_fields[2] = b3CreateHeightField( &def );
	}

	// Fine enough that a large hull overlaps more triangles than the cache holds
	g_fields[3] = b3CreateWave( 90, 90, (b3Vec3){ 0.08f, 0.3f, 0.08f }, 0.2f, 0.3f, false );

	g_boxHulls[0] = b3MakeBoxHull( 0.4f, 0.3f, 0.5f );
	g_boxHulls[1] = b3MakeBoxHull( 1.2f, 0.4f, 1.2f );
	g_hulls[0] = &g_boxHulls[0].base;
	g_hulls[1] = &g_boxHulls[1].base;
	g_hulls[2] = b3CreateCylinder( 0.8f, 0.4f, 0.0f, 10 );
	g_hulls[3] = b3CreateCone( 0.8f, 0.5f, 0.1f, 8 );
	{
		b3Vec3 points[24];
		for ( int i = 0; i < 24; ++i )
		{
			points[i] = randVec( -0.5f, 0.5f );
		}
		g_hulls[4] = b3CreateHull( points, 24, 32 );
	}

	for ( int i = 0; i < 4; ++i )
	{
		b3SurfaceMaterial m = { 0 };
		m.friction = vrandf( 0.0f, 1.0f );
		m.restitution = vrandf( 0.0f, 1.0f );
		m.rollingResistance = vrandf( 0.0f, 0.3f );
		m.tangentVelocity = randVec( -1.0f, 1.0f );
		m.userMaterialId = vrand64();
		g_materials[i] = m;
	}
}

static b3World* g_world;

static void printContact( int seq, int step, bool touching, b3Contact* contact, int shapeTypeB )
{
	b3TaskContext* context = g_world->taskContexts.data + 0;
	vtag( "step" );
	vi32( seq );
	vi32( step );
	vbool( touching );
	vi32( contact->manifoldCount );
	vf32( contact->friction );
	vf32( contact->restitution );
	vf32( contact->rollingResistance );
	pv3( contact->tangentVelocity );
	pab( contact->meshContact.queryBounds );
	vi32( contact->meshContact.triangleCache.count );
	vi32( context->satCallCount );
	vi32( context->satCacheHitCount );
	vend();

	for ( int i = 0; i < contact->manifoldCount; ++i )
	{
		b3Manifold* m = contact->manifolds + i;
		vtag( "m" );
		pv3( m->normal );
		vi32( m->pointCount );
		vf32( m->twistImpulse );
		pv3( m->frictionImpulse );
		pv3( m->rollingImpulse );
		vend();
		for ( int j = 0; j < m->pointCount; ++j )
		{
			b3ManifoldPoint* p = m->points + j;
			vtag( "p" );
			pv3( p->anchorA );
			pv3( p->anchorB );
			vf32( p->separation );
			vf32( p->normalImpulse );
			vf32( p->totalNormalImpulse );
			vf32( p->normalVelocity );
			vf32( p->baseSeparation );
			vu32( p->featureId );
			vi32( p->triangleIndex );
			vbool( p->persisted );
			vend();
		}
	}

	for ( int i = 0; i < contact->meshContact.triangleCache.count; ++i )
	{
		b3TriangleCache* tc = contact->meshContact.triangleCache.data + i;
		vtag( "c" );
		vi32( tc->triangleIndex );
		if ( shapeTypeB == b3_hullShape )
		{
			vf32( tc->cache.satCache.separation );
			vu32( tc->cache.satCache.type );
			vu32( tc->cache.satCache.indexA );
			vu32( tc->cache.satCache.indexB );
			vu32( tc->cache.satCache.hit );
		}
		else if ( shapeTypeB == b3_capsuleShape )
		{
			vf32( tc->cache.simplexCache.metric );
			vu32( tc->cache.simplexCache.count );
			// Indices past count are whatever the cache's memory held, which C never clears
			for ( int k = 0; k < tc->cache.simplexCache.count; ++k )
			{
				vu32( tc->cache.simplexCache.indexA[k] );
				vu32( tc->cache.simplexCache.indexB[k] );
			}
		}
		vend();
	}
}

static b3AABB computeAABB( const b3Shape* shape, b3Transform xf )
{
	if ( shape->type == b3_sphereShape )
	{
		return b3ComputeSphereAABB( &shape->sphere, xf );
	}
	if ( shape->type == b3_capsuleShape )
	{
		return b3ComputeCapsuleAABB( &shape->capsule, xf );
	}
	return b3ComputeHullAABB( shape->hull, xf );
}

static void runSequence( int seq )
{
	vseed( 9000 + seq );

	static b3Shape shapeA;
	static b3Shape shapeB;
	memset( &shapeA, 0, sizeof( shapeA ) );
	memset( &shapeB, 0, sizeof( shapeB ) );

	b3SurfaceMaterial materialsA[3];
	b3AABB localBounds;

	int kindA = vrandint( 0, 4 );
	bool isMesh = kindA != 0;
	if ( isMesh )
	{
		int m = vrandint( 0, MESH_COUNT );
		b3Vec3 scale;
		scale.x = vrandf( 0.6f, 1.6f );
		scale.y = vrandf( 0.6f, 1.6f );
		scale.z = vrandf( 0.6f, 1.6f );
		int flip = vrandint( 0, 4 );
		if ( flip == 0 )
		{
			scale.x = -scale.x;
		}
		shapeA.type = b3_meshShape;
		shapeA.mesh.data = g_meshes[m];
		shapeA.mesh.scale = scale;
		localBounds = b3ComputeMeshAABB( g_meshes[m], b3Transform_identity, scale );
	}
	else
	{
		int f = vrandint( 0, FIELD_COUNT );
		shapeA.type = b3_heightShape;
		shapeA.heightField = g_fields[f];
		localBounds = b3ComputeHeightFieldAABB( g_fields[f], b3Transform_identity );
	}

	// 0: no materials (the else branch), 1: inline, 2: three materials, 3: three materials through a map
	int materialMode = vrandint( 0, 4 );
	uint16_t materialMap[8] = { 2, 0, 1, 2, 1, 0, 2, 1 };
	const uint16_t* map = NULL;
	for ( int i = 0; i < 3; ++i )
	{
		int k = vrandint( 0, 4 );
		materialsA[i] = g_materials[k];
	}
	if ( materialMode == 0 )
	{
		shapeA.materialCount = 0;
		shapeA.material = materialsA[0];
	}
	else if ( materialMode == 1 )
	{
		shapeA.materialCount = 1;
		shapeA.material = materialsA[0];
	}
	else
	{
		shapeA.materialCount = 3;
		shapeA.materials = materialsA;
		if ( materialMode == 3 && isMesh )
		{
			map = materialMap;
		}
	}

	bool flat = false;
	int kindB = vrandint( 0, 3 );
	if ( kindB == 0 )
	{
		shapeB.type = b3_sphereShape;
		shapeB.sphere.center = randVec( -0.1f, 0.1f );
		shapeB.sphere.radius = vrandf( 0.2f, 0.8f );
	}
	else if ( kindB == 1 )
	{
		shapeB.type = b3_capsuleShape;
		b3Vec3 c = randVec( -0.1f, 0.1f );
		b3Vec3 h = randVec( -0.7f, 0.7f );
		shapeB.capsule.center1 = b3Sub( c, h );
		shapeB.capsule.center2 = b3Add( c, h );
		shapeB.capsule.radius = vrandf( 0.1f, 0.5f );

		// A capsule lying flat along the A frame x axis gives collinear cluster points
		int lying = vrandint( 0, 3 );
		if ( lying == 0 )
		{
			flat = true;
			shapeB.capsule.center1 = (b3Vec3){ -h.x - 0.5f, 0.0f, 0.0f };
			shapeB.capsule.center2 = (b3Vec3){ h.x + 0.5f, 0.0f, 0.0f };
		}
	}
	else
	{
		int h = vrandint( 0, HULL_COUNT );
		shapeB.type = b3_hullShape;
		shapeB.hull = g_hulls[h];
	}
	{
		int k = vrandint( 0, 4 );
		shapeB.material = g_materials[k];
		shapeB.materialCount = 1;
	}

	b3Transform xfA = b3Transform_identity;
	int rotatedA = vrandint( 0, 3 );
	if ( rotatedA == 0 )
	{
		xfA.p = randVec( -2.0f, 2.0f );
		xfA.q = randQuat();
	}
	else if ( rotatedA == 1 )
	{
		xfA.p = randVec( -2.0f, 2.0f );
	}

	static b3Contact contact;
	b3Array_Destroy( contact.meshContact.triangleCache );
	if ( contact.manifoldCount > 0 )
	{
		b3FreeManifolds( g_world, contact.manifolds, contact.manifoldCount );
	}
	memset( &contact, 0, sizeof( contact ) );
	int speculative = vrandint( 0, 3 );
	contact.flags = speculative != 0 ? b3_enableSpeculativePoints : 0;

	// Start over the surface in the A frame
	b3Vec3 local;
	local.x = vrandf( localBounds.lowerBound.x, localBounds.upperBound.x );
	local.z = vrandf( localBounds.lowerBound.z, localBounds.upperBound.z );
	local.y = vrandf( localBounds.lowerBound.y - 0.2f, localBounds.upperBound.y + 0.5f );
	b3Quat qB = randQuat();
	if ( flat )
	{
		qB = xfA.q;
	}
	b3Vec3 velocity = randVec( -0.05f, 0.05f );
	velocity.y = vrandf( -0.06f, 0.0f );

	vtag( "seq" );
	vi32( seq );
	vi32( shapeA.type );
	vi32( shapeB.type );
	vi32( materialMode );
	vi32( speculative );
	vend();

	int stepCount = 40;
	for ( int step = 0; step < stepCount; ++step )
	{
		int jump = vrandint( 0, 25 );
		if ( jump == 0 )
		{
			local.x = vrandf( localBounds.lowerBound.x, localBounds.upperBound.x );
			local.z = vrandf( localBounds.lowerBound.z, localBounds.upperBound.z );
			local.y = vrandf( localBounds.lowerBound.y - 0.2f, localBounds.upperBound.y + 0.5f );
		}
		else
		{
			b3Vec3 noise = randVec( -0.02f, 0.02f );
			local = b3Add( local, b3Add( velocity, noise ) );
		}
		b3Quat turn = smallRotation( 0.15f );
		if ( flat == false )
		{
			qB = b3NormalizeQuat( b3MulQuat( turn, qB ) );
		}

		b3Transform xfB;
		xfB.p = b3TransformPoint( xfA, local );
		xfB.q = qB;
		shapeB.aabb = computeAABB( &shapeB, xfB );

		int fast = vrandint( 0, 4 );
		bool isFast = fast == 0;

		bool touching = b3ComputeMeshManifolds( g_world, 0, &contact, &shapeA, map, xfA, &shapeB, xfB, isFast,
												g_world->taskContexts.data[0].arena );
		printContact( seq, step, touching, &contact, shapeB.type );

		// Fake solver impulses so the next call has something to preserve
		for ( int i = 0; i < contact.manifoldCount; ++i )
		{
			b3Manifold* m = contact.manifolds + i;
			m->twistImpulse = vrandf( -1.0f, 1.0f );
			m->frictionImpulse = randVec( -1.0f, 1.0f );
			m->rollingImpulse = randVec( -1.0f, 1.0f );
			for ( int j = 0; j < m->pointCount; ++j )
			{
				m->points[j].normalImpulse = vrandf( 0.0f, 2.0f );
			}
		}
	}
}

// The port's default log handler is silent; keep the vectors clean of the capacity warning
static void silentLog( const char* message )
{
	(void)message;
}

int main( int argc, char** argv )
{
	int count = argc > 1 ? atoi( argv[1] ) : 120;
	b3SetLogFcn( silentLog );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	g_world = b3GetWorldFromId( worldId );

	buildPools();
	for ( int i = 0; i < count; ++i )
	{
		runSequence( i );
	}

	b3DestroyWorld( worldId );
	return 0;
}
