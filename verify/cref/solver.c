// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for the step solver: lib/solver.as, lib/contact_solver.as, lib/contact_solver_wide.as,
// lib/solver_set.as, lib/island.as and lib/constraint_graph.as. Four scenes stepped through the public API;
// after every step the bodies, the solver sets, the graph colors, the islands and every live contact's
// manifold impulses are printed from the world internals.

#include "common.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "island.h"
#include "joint.h"
#include "physics_world.h"
#include "solver_set.h"

#define MAX_BODIES 128

static b3WorldId g_worldId;
static b3BodyId g_bodies[MAX_BODIES];
static int g_bodyCount;
static b3MeshData* g_mesh;
static b3JointId g_joint;

static b3BodyId addBody( const b3BodyDef* def )
{
	b3BodyId id = b3CreateBody( g_worldId, def );
	g_bodies[g_bodyCount] = id;
	g_bodyCount += 1;
	return id;
}

static void pv3( b3Vec3 v )
{
	vf32( v.x );
	vf32( v.y );
	vf32( v.z );
}

static void pquat( b3Quat q )
{
	pv3( q.v );
	vf32( q.s );
}

static void printBodies( void )
{
	for ( int i = 0; i < g_bodyCount; ++i )
	{
		b3BodyId id = g_bodies[i];
		vtag( "body" );
		vi32( i );
		bool valid = b3Body_IsValid( id );
		vbool( valid );
		if ( valid )
		{
			pv3( b3Body_GetPosition( id ) );
			pquat( b3Body_GetRotation( id ) );
			pv3( b3Body_GetLinearVelocity( id ) );
			pv3( b3Body_GetAngularVelocity( id ) );
			vbool( b3Body_IsAwake( id ) );
		}
		vend();
	}
}

static void printSolver( void )
{
	b3World* world = b3GetWorldFromId( g_worldId );

	vtag( "sets" );
	vi32( world->solverSets.count );
	vi32( world->splitIslandId );
	vend();
	for ( int i = 0; i < world->solverSets.count; ++i )
	{
		b3SolverSet* set = world->solverSets.data + i;
		vtag( "set" );
		vi32( i );
		vi32( set->setIndex );
		vi32( set->bodySims.count );
		vi32( set->bodyStates.count );
		vi32( set->jointSims.count );
		vi32( set->contactIndices.count );
		vi32( set->islandSims.count );
		vend();

		if ( set->setIndex == B3_NULL_INDEX )
		{
			continue;
		}

		vtag( "setBodies" );
		for ( int j = 0; j < set->bodySims.count; ++j )
		{
			vi32( set->bodySims.data[j].bodyId );
		}
		vend();

		vtag( "setFlags" );
		for ( int j = 0; j < set->bodyStates.count; ++j )
		{
			vu32( set->bodyStates.data[j].flags );
		}
		for ( int j = 0; j < set->bodySims.count; ++j )
		{
			vu32( set->bodySims.data[j].flags );
		}
		vend();

		vtag( "setContacts" );
		for ( int j = 0; j < set->contactIndices.count; ++j )
		{
			vi32( set->contactIndices.data[j] );
		}
		vend();

		vtag( "setJoints" );
		for ( int j = 0; j < set->jointSims.count; ++j )
		{
			vi32( set->jointSims.data[j].jointId );
		}
		vend();

		vtag( "setIslands" );
		for ( int j = 0; j < set->islandSims.count; ++j )
		{
			vi32( set->islandSims.data[j].islandId );
		}
		vend();
	}

	for ( int i = 0; i < B3_GRAPH_COLOR_COUNT; ++i )
	{
		b3GraphColor* color = world->constraintGraph.colors + i;
		vtag( "color" );
		vi32( i );
		for ( int j = 0; j < color->convexContacts.count; ++j )
		{
			vi32( color->convexContacts.data[j] );
		}
		vi32( -2 );
		for ( int j = 0; j < color->contacts.count; ++j )
		{
			vi32( color->contacts.data[j].contactId );
			vi32( color->contacts.data[j].manifoldCount );
		}
		vi32( -2 );
		for ( int j = 0; j < color->jointSims.count; ++j )
		{
			vi32( color->jointSims.data[j].jointId );
		}
		vend();
	}

	vtag( "islands" );
	vi32( world->islands.count );
	vend();
	for ( int i = 0; i < world->islands.count; ++i )
	{
		b3Island* island = world->islands.data + i;
		if ( island->setIndex == B3_NULL_INDEX )
		{
			continue;
		}

		vtag( "island" );
		vi32( i );
		vi32( island->islandId );
		vi32( island->setIndex );
		vi32( island->localIndex );
		vi32( island->constraintRemoveCount );
		vi32( -2 );
		for ( int j = 0; j < island->bodies.count; ++j )
		{
			vi32( island->bodies.data[j] );
		}
		vi32( -2 );
		for ( int j = 0; j < island->contacts.count; ++j )
		{
			vi32( island->contacts.data[j].contactId );
		}
		vi32( -2 );
		for ( int j = 0; j < island->joints.count; ++j )
		{
			vi32( island->joints.data[j].jointId );
		}
		vend();
	}

	for ( int i = 0; i < world->bodies.count; ++i )
	{
		b3Body* body = world->bodies.data + i;
		if ( body->setIndex == B3_NULL_INDEX )
		{
			continue;
		}

		vtag( "b" );
		vi32( i );
		vi32( body->setIndex );
		vi32( body->localIndex );
		vi32( body->islandId );
		vi32( body->islandIndex );
		vi32( body->bodyMoveIndex );
		vf32( body->sleepTime );
		vf32( body->sleepVelocity );
		vu32( body->flags );
		vend();
	}

	for ( int i = 0; i < world->joints.count; ++i )
	{
		b3Joint* joint = world->joints.data + i;
		if ( joint->setIndex == B3_NULL_INDEX )
		{
			continue;
		}

		vtag( "j" );
		vi32( i );
		vi32( joint->setIndex );
		vi32( joint->colorIndex );
		vi32( joint->localIndex );
		vi32( joint->islandId );
		vi32( joint->islandIndex );
		vend();
	}

	for ( int i = 0; i < world->contacts.count; ++i )
	{
		b3Contact* contact = world->contacts.data + i;
		if ( contact->setIndex == B3_NULL_INDEX )
		{
			continue;
		}

		vtag( "c" );
		vi32( i );
		vi32( contact->setIndex );
		vi32( contact->colorIndex );
		vi32( contact->localIndex );
		vi32( contact->islandId );
		vi32( contact->islandIndex );
		vi32( contact->encodedBodySimA );
		vi32( contact->encodedBodySimB );
		vu32( contact->flags );
		vi32( contact->manifoldCount );
		for ( int m = 0; m < contact->manifoldCount; ++m )
		{
			b3Manifold* manifold = contact->manifolds + m;
			vi32( manifold->pointCount );
			pv3( manifold->normal );
			vf32( manifold->twistImpulse );
			pv3( manifold->frictionImpulse );
			pv3( manifold->rollingImpulse );
			for ( int p = 0; p < manifold->pointCount; ++p )
			{
				b3ManifoldPoint* mp = manifold->points + p;
				vf32( mp->normalImpulse );
				vf32( mp->totalNormalImpulse );
				vf32( mp->normalVelocity );
			}
		}
		vend();
	}
}

static void printEvents( void )
{
	b3ContactEvents contactEvents = b3World_GetContactEvents( g_worldId );
	vtag( "contactEvents" );
	vi32( contactEvents.beginCount );
	vi32( contactEvents.endCount );
	vi32( contactEvents.hitCount );
	vend();
	for ( int i = 0; i < contactEvents.hitCount; ++i )
	{
		b3ContactHitEvent* e = contactEvents.hitEvents + i;
		vtag( "hit" );
		vi32( e->shapeIdA.index1 );
		vi32( e->shapeIdB.index1 );
		vi32( e->contactId.index1 );
		vu32( e->contactId.generation );
		pv3( e->point );
		pv3( e->normal );
		vf32( e->approachSpeed );
		vu64( e->userMaterialIdA );
		vu64( e->userMaterialIdB );
		vend();
	}

	b3JointEvents jointEvents = b3World_GetJointEvents( g_worldId );
	vtag( "jointEvents" );
	vi32( jointEvents.count );
	for ( int i = 0; i < jointEvents.count; ++i )
	{
		vi32( jointEvents.jointEvents[i].jointId.index1 );
		vi32( jointEvents.jointEvents[i].jointId.generation );
	}
	vend();

	b3SensorEvents sensorEvents = b3World_GetSensorEvents( g_worldId );
	vtag( "sensorEvents" );
	vi32( sensorEvents.beginCount );
	for ( int i = 0; i < sensorEvents.beginCount; ++i )
	{
		vi32( sensorEvents.beginEvents[i].sensorShapeId.index1 );
		vi32( sensorEvents.beginEvents[i].visitorShapeId.index1 );
	}
	vi32( sensorEvents.endCount );
	vend();
}

static b3BodyId createGround( float hx, float hy, float hz )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = (b3Pos){ 0.0f, -hy, 0.0f };
	b3BodyId groundId = addBody( &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.6f;
	b3BoxHull box = b3MakeBoxHull( hx, hy, hz );
	b3CreateHullShape( groundId, &shapeDef, &box.base );
	return groundId;
}

// Scene 0: a plank on a 5 x 5 grid of boxes puts 25 dynamic contacts on one body, so most of them
// overflow the graph colors. Restitution, rolling resistance, gyroscopic torque and motion locks
// ride along. The pile falls asleep, is woken by an impulse, and loses a box to force an island split.
static void createOverflow( void )
{
	createGround( 20.0f, 1.0f, 20.0f );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.6f;

	b3BoxHull small = b3MakeBoxHull( 0.25f, 0.25f, 0.25f );
	for ( int i = 0; i < 5; ++i )
	{
		for ( int j = 0; j < 5; ++j )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = (b3Pos){ -1.2f + 0.6f * (float)i, 0.25f, -1.2f + 0.6f * (float)j };
			b3BodyId bodyId = addBody( &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &small.base );
		}
	}

	// plank, index 26
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 1.4f, 0.0f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3ShapeDef plankDef = b3DefaultShapeDef();
		plankDef.baseMaterial.friction = 0.6f;
		plankDef.baseMaterial.restitution = 0.3f;
		plankDef.enableHitEvents = true;
		b3BoxHull plank = b3MakeBoxHull( 1.6f, 0.1f, 1.6f );
		b3CreateHullShape( bodyId, &plankDef, &plank.base );
	}

	// bouncing sphere with hit events, index 27
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 4.0f, 3.0f, 0.0f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3ShapeDef ballDef = b3DefaultShapeDef();
		ballDef.baseMaterial.restitution = 0.8f;
		ballDef.enableHitEvents = true;
		ballDef.baseMaterial.userMaterialId = 7;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.3f };
		b3CreateSphereShape( bodyId, &ballDef, &sphere );
	}

	// rolling capsule with rolling resistance, index 28
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ -4.0f, 0.25f, 0.0f };
		bodyDef.linearVelocity = (b3Vec3){ 0.0f, 0.0f, 2.0f };
		bodyDef.angularVelocity = (b3Vec3){ 8.0f, 0.0f, 0.0f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3ShapeDef capsuleDef = b3DefaultShapeDef();
		capsuleDef.baseMaterial.rollingResistance = 0.3f;
		b3Capsule capsule = { { -0.4f, 0.0f, 0.0f }, { 0.4f, 0.0f, 0.0f }, 0.25f };
		b3CreateCapsuleShape( bodyId, &capsuleDef, &capsule );
	}

	// tumbling rod for gyroscopic torque, index 29
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 4.0f, 4.0f };
		bodyDef.angularVelocity = (b3Vec3){ 0.3f, 6.0f, 0.5f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3BoxHull rod = b3MakeBoxHull( 1.0f, 0.05f, 0.1f );
		b3CreateHullShape( bodyId, &shapeDef, &rod.base );
	}

	// sliding box with angular locks, index 30
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 3.0f, 0.5f, 4.0f };
		bodyDef.linearVelocity = (b3Vec3){ 3.0f, 0.0f, 1.0f };
		bodyDef.angularVelocity = (b3Vec3){ 1.0f, 2.0f, 3.0f };
		bodyDef.motionLocks.angularX = true;
		bodyDef.motionLocks.angularZ = true;
		b3BodyId bodyId = addBody( &bodyDef );
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}
}

static void stepOverflow( int stepIndex )
{
	if ( stepIndex == 140 )
	{
		b3Body_ApplyLinearImpulse( g_bodies[26], (b3Vec3){ 0.0f, 2.0f, 0.5f }, (b3Pos){ 1.0f, 0.7f, 1.0f }, true );
	}

	if ( stepIndex == 150 )
	{
		b3DestroyBody( g_bodies[13] );
	}

	if ( stepIndex == 200 )
	{
		b3Body_SetAwake( g_bodies[27], true );
		b3Body_ApplyLinearImpulse( g_bodies[27], (b3Vec3){ 0.0f, 3.0f, 0.0f }, (b3Pos){ 4.0f, 0.3f, 0.0f }, true );
	}
}

// Scene 1: boxes, capsules and spheres on a wave mesh give mesh contacts with several manifolds,
// solved by the wide mesh path. A fast sphere sweeps into the mesh.
static void createMesh( void )
{
	b3BodyDef groundDef = b3DefaultBodyDef();
	b3BodyId groundId = addBody( &groundDef );
	g_mesh = b3CreateWaveMesh( 16, 16, 0.5f, 0.3f, 0.5f, 0.7f );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3CreateMeshShape( groundId, &groundShapeDef, g_mesh, b3Vec3_one );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.5f;
	shapeDef.baseMaterial.rollingResistance = 0.05f;
	shapeDef.baseMaterial.restitution = 0.4f;
	shapeDef.enableHitEvents = true;

	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			float x = 1.0f + 1.5f * (float)i;
			float z = 1.0f + 1.5f * (float)j;
			int kind = ( i + j ) % 3;

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = (b3Pos){ x, 1.5f + 0.5f * (float)j, z };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Normalize( (b3Vec3){ 1.0f, 0.5f, 0.2f * (float)i } ), 0.3f * (float)j );
			b3BodyId bodyId = addBody( &bodyDef );

			if ( kind == 0 )
			{
				b3BoxHull box = b3MakeBoxHull( 0.6f, 0.2f, 0.4f );
				b3CreateHullShape( bodyId, &shapeDef, &box.base );
			}
			else if ( kind == 1 )
			{
				b3Capsule capsule = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.2f };
				b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			}
			else
			{
				b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.35f };
				b3CreateSphereShape( bodyId, &shapeDef, &sphere );
			}
		}
	}

	// fast sphere
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 6.0f, 6.0f, 6.0f };
		bodyDef.linearVelocity = (b3Vec3){ 1.0f, -70.0f, 0.5f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.1f };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	}
}

static void stepMesh( int stepIndex )
{
	if ( stepIndex == 60 )
	{
		b3World_EnableRestitutionPropagation( g_worldId, true );
	}

	if ( stepIndex == 100 )
	{
		b3Body_ApplyLinearImpulse( g_bodies[1], (b3Vec3){ 1.0f, 2.0f, 0.0f }, (b3Pos){ 1.0f, 1.0f, 1.0f }, true );
	}
}

// Scene 2: joints. Two boxes fall asleep apart and are joined while asleep, which merges their solver
// sets; an impulse then wakes the merged set. A hub with 24 distance joints overflows the joint colors,
// a revolute chain reports joint events, a body is disabled and enabled, and a link is destroyed.
static void createJoints( void )
{
	createGround( 20.0f, 1.0f, 20.0f );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	// sleepers, indices 1 and 2
	for ( int i = 0; i < 2; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ -6.0f + 3.0f * (float)i, 0.5f, -6.0f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	// hub, index 3, with 24 spokes, indices 4 to 27
	b3BodyId hubId;
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 6.0f, 0.0f };
		bodyDef.gravityScale = 0.0f;
		bodyDef.angularVelocity = (b3Vec3){ 0.0f, 1.0f, 0.2f };
		hubId = addBody( &bodyDef );
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		b3CreateSphereShape( hubId, &shapeDef, &sphere );
	}

	for ( int i = 0; i < 24; ++i )
	{
		float angle = 2.0f * B3_PI * (float)i / 24.0f;
		b3CosSin cs = b3ComputeCosSin( angle );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 2.0f * cs.cosine, 6.0f + 0.1f * (float)( i % 3 ), 2.0f * cs.sine };
		bodyDef.gravityScale = 0.0f;
		b3BodyId bodyId = addBody( &bodyDef );
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.15f };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );

		b3DistanceJointDef jointDef = b3DefaultDistanceJointDef();
		jointDef.base.bodyIdA = hubId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.length = 2.0f;
		jointDef.enableSpring = ( i % 2 ) == 0;
		jointDef.hertz = 2.0f;
		jointDef.dampingRatio = 0.3f;
		b3CreateDistanceJoint( g_worldId, &jointDef );
	}

	// revolute chain hanging from a static anchor, indices 28 to 34
	{
		b3BodyDef anchorDef = b3DefaultBodyDef();
		anchorDef.position = (b3Pos){ 6.0f, 8.0f, 0.0f };
		b3BodyId prevId = addBody( &anchorDef );

		b3Capsule capsule = { { -0.4f, 0.0f, 0.0f }, { 0.4f, 0.0f, 0.0f }, 0.1f };
		for ( int i = 0; i < 6; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = (b3Pos){ 6.5f + 1.0f * (float)i, 8.0f, 0.0f };
			b3BodyId bodyId = addBody( &bodyDef );
			b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );

			b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = prevId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = i == 0 ? (b3Vec3){ 0.0f, 0.0f, 0.0f } : (b3Vec3){ 0.5f, 0.0f, 0.0f };
			jointDef.base.localFrameB.p = (b3Vec3){ -0.5f, 0.0f, 0.0f };
			jointDef.base.forceThreshold = 20.0f;
			jointDef.base.torqueThreshold = 100.0f;
			b3JointId jointId = b3CreateRevoluteJoint( g_worldId, &jointDef );
			if ( i == 3 )
			{
				g_joint = jointId;
			}
			prevId = bodyId;
		}
	}

	// spherical pendulum and a welded pair, indices 35 to 37
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ -4.0f, 4.0f, 3.0f };
		b3BodyId ballId = addBody( &bodyDef );
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.3f };
		b3CreateSphereShape( ballId, &shapeDef, &sphere );

		b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
		jointDef.base.bodyIdA = g_bodies[0];
		jointDef.base.bodyIdB = ballId;
		jointDef.base.localFrameA.p = (b3Vec3){ -4.0f, 6.0f, 2.0f };
		jointDef.base.localFrameB.p = (b3Vec3){ 0.0f, 0.0f, 0.0f };
		b3CreateSphericalJoint( g_worldId, &jointDef );

		bodyDef.position = (b3Pos){ 4.0f, 0.5f, 4.0f };
		b3BodyId lowerId = addBody( &bodyDef );
		b3CreateHullShape( lowerId, &shapeDef, &box.base );

		bodyDef.position = (b3Pos){ 4.0f, 1.5f, 4.0f };
		bodyDef.angularVelocity = (b3Vec3){ 0.0f, 2.0f, 0.0f };
		b3BodyId upperId = addBody( &bodyDef );
		b3CreateHullShape( upperId, &shapeDef, &box.base );

		b3WeldJointDef weldDef = b3DefaultWeldJointDef();
		weldDef.base.bodyIdA = lowerId;
		weldDef.base.bodyIdB = upperId;
		weldDef.base.localFrameA.p = (b3Vec3){ 0.0f, 0.5f, 0.0f };
		weldDef.base.localFrameB.p = (b3Vec3){ 0.0f, -0.5f, 0.0f };
		weldDef.angularHertz = 5.0f;
		weldDef.angularDampingRatio = 0.5f;
		b3CreateWeldJoint( g_worldId, &weldDef );
	}
}

static void stepJoints( int stepIndex )
{
	if ( stepIndex == 70 )
	{
		b3DistanceJointDef jointDef = b3DefaultDistanceJointDef();
		jointDef.base.bodyIdA = g_bodies[1];
		jointDef.base.bodyIdB = g_bodies[2];
		jointDef.length = 3.0f;
		b3CreateDistanceJoint( g_worldId, &jointDef );
	}

	if ( stepIndex == 100 )
	{
		b3Body_ApplyLinearImpulse( g_bodies[1], (b3Vec3){ 0.0f, 4.0f, 1.0f }, (b3Pos){ -6.0f, 0.8f, -6.2f }, true );
	}

	if ( stepIndex == 130 )
	{
		b3Body_Disable( g_bodies[3] );
	}

	if ( stepIndex == 150 )
	{
		b3Body_Enable( g_bodies[3] );
	}

	if ( stepIndex == 170 )
	{
		b3DestroyJoint( g_joint, true );
	}

	if ( stepIndex == 190 )
	{
		b3DestroyBody( g_bodies[30] );
	}
}

// Scene 3: a bullet against a dynamic stack and a fast capsule against a thin wall, through a sensor.
static void createBullets( void )
{
	createGround( 30.0f, 1.0f, 30.0f );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enableSensorEvents = true;
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	for ( int i = 0; i < 4; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 0.5f + 1.0f * (float)i, 0.0f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	// thin wall
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 0.0f, 2.0f, -10.0f };
		b3BodyId wallId = addBody( &bodyDef );
		b3BoxHull wall = b3MakeBoxHull( 3.0f, 2.0f, 0.05f );
		b3CreateHullShape( wallId, &shapeDef, &wall.base );
	}

	// sensor in the capsule path
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 0.0f, 1.0f, -6.0f };
		b3BodyId sensorId = addBody( &bodyDef );
		b3ShapeDef sensorDef = b3DefaultShapeDef();
		sensorDef.isSensor = true;
		sensorDef.enableSensorEvents = true;
		b3BoxHull slab = b3MakeBoxHull( 2.0f, 1.0f, 0.02f );
		b3CreateHullShape( sensorId, &sensorDef, &slab.base );
	}

	// fast capsule, not a bullet
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.3f, 1.0f, 2.0f };
		bodyDef.linearVelocity = (b3Vec3){ 0.0f, 0.5f, -150.0f };
		bodyDef.angularVelocity = (b3Vec3){ 3.0f, 0.0f, 0.0f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3Capsule capsule = { { -0.3f, 0.0f, 0.0f }, { 0.3f, 0.0f, 0.0f }, 0.1f };
		b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
	}

	// bullet
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.position = (b3Pos){ 0.1f, 1.5f, 12.0f };
		bodyDef.linearVelocity = (b3Vec3){ 0.0f, 0.0f, -120.0f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.15f };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	}
}

static void stepBullets( int stepIndex )
{
	if ( stepIndex == 40 )
	{
		b3World_EnableWarmStarting( g_worldId, false );
	}
}

static const int g_stepCounts[4] = { 240, 160, 220, 80 };

static void runScene( int scene )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	g_worldId = b3CreateWorld( &worldDef );
	g_bodyCount = 0;
	g_mesh = NULL;

	if ( scene == 0 )
	{
		createOverflow();
	}
	else if ( scene == 1 )
	{
		createMesh();
	}
	else if ( scene == 2 )
	{
		createJoints();
	}
	else
	{
		createBullets();
	}

	vtag( "scene" );
	vi32( scene );
	vi32( g_bodyCount );
	vend();

	int stepCount = g_stepCounts[scene];
	for ( int i = 0; i < stepCount; ++i )
	{
		if ( scene == 0 )
		{
			stepOverflow( i );
		}
		else if ( scene == 1 )
		{
			stepMesh( i );
		}
		else if ( scene == 2 )
		{
			stepJoints( i );
		}
		else
		{
			stepBullets( i );
		}

		b3World_Step( g_worldId, 1.0f / 60.0f, 4 );

		vtag( "step" );
		vi32( scene );
		vi32( i );
		vend();

		printBodies();
		printSolver();
		printEvents();
	}

	b3DestroyWorld( g_worldId );
	if ( g_mesh != NULL )
	{
		b3DestroyMesh( g_mesh );
	}
}

int main( void )
{
	for ( int scene = 0; scene < 4; ++scene )
	{
		runScene( scene );
	}
	return 0;
}
