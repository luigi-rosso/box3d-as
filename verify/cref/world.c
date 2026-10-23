// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/physics_world.as: the world registry, settings, stepping a small
// mixed scene with events, every query, explosions and counters.

#include "common.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>

#define MAX_BODIES 64
#define PLANE_CAPACITY 16

static b3WorldId g_worldId;
static b3BodyId g_bodies[MAX_BODIES];
static int g_bodyCount;
static b3CollisionPlane g_planes[PLANE_CAPACITY];
static int g_planeCount;

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

static void pworldId( b3WorldId id )
{
	vi32( id.index1 );
	vi32( id.generation );
}

static void pshapeId( b3ShapeId id )
{
	vi32( id.index1 );
	vi32( id.world0 );
	vi32( id.generation );
}

static void pbodyId( b3BodyId id )
{
	vi32( id.index1 );
	vi32( id.world0 );
	vi32( id.generation );
}

static void pcontactId( b3ContactId id )
{
	vi32( id.index1 );
	vi32( id.world0 );
	vu32( id.generation );
}

static void pstats( b3TreeStats stats )
{
	vi32( stats.nodeVisits );
	vi32( stats.leafVisits );
}

static void paabb( b3AABB a )
{
	pv3( a.lowerBound );
	pv3( a.upperBound );
}

static b3BodyId addBody( const b3BodyDef* def )
{
	b3BodyId id = b3CreateBody( g_worldId, def );
	g_bodies[g_bodyCount] = id;
	g_bodyCount += 1;
	return id;
}

static float minFriction( float frictionA, uint64_t materialA, float frictionB, uint64_t materialB )
{
	(void)materialA;
	(void)materialB;
	return frictionA < frictionB ? frictionA : frictionB;
}

static void registry( void )
{
	vtag( "registry" );
	vi32( b3GetWorldCount() );
	vend();

	b3WorldDef def = b3DefaultWorldDef();
	b3WorldId a = b3CreateWorld( &def );
	b3WorldId b = b3CreateWorld( &def );

	vtag( "created" );
	pworldId( a );
	pworldId( b );
	vbool( b3World_IsValid( a ) );
	vbool( b3World_IsValid( b ) );
	vi32( b3GetWorldCount() );
	vi32( b3GetMaxWorldCount() );
	vend();

	b3DestroyWorld( b );
	b3WorldId c = b3CreateWorld( &def );

	vtag( "recreated" );
	vbool( b3World_IsValid( b ) );
	pworldId( c );
	vbool( b3World_IsValid( c ) );
	vi32( b3GetWorldCount() );
	vend();

	b3WorldId bad = { 0, 0 };
	b3WorldId far = { 200, 0 };
	vtag( "invalid" );
	vbool( b3World_IsValid( bad ) );
	vbool( b3World_IsValid( far ) );
	vend();

	// Setters clamp
	b3World_SetRestitutionThreshold( c, -1.0f );
	b3World_SetRestitutionIterations( c, 100 );
	b3World_SetHitEventThreshold( c, 2.5f );
	b3World_SetContactTuning( c, -3.0f, 4.0f, 5.0f );
	b3World_SetContactRecycleDistance( c, -0.1f );
	b3World_SetMaximumLinearSpeed( c, 123.0f );
	b3World_SetIsotropicInertiaTolerance( c, 3.0f );
	b3World_EnableSleeping( c, false );
	b3World_EnableWarmStarting( c, false );
	b3World_EnableContinuous( c, false );
	b3World_EnableRestitutionPropagation( c, true );
	b3World_SetUserData( c, (void*)(uintptr_t)77 );
	b3World_SetGravity( c, (b3Vec3){ 1.0f, -2.0f, 3.0f } );

	vtag( "settings" );
	vf32( b3World_GetRestitutionThreshold( c ) );
	vi32( b3World_GetRestitutionIterations( c ) );
	vf32( b3World_GetHitEventThreshold( c ) );
	vf32( b3World_GetContactRecycleDistance( c ) );
	vf32( b3World_GetMaximumLinearSpeed( c ) );
	vf32( b3World_GetIsotropicInertiaTolerance( c ) );
	vbool( b3World_IsSleepingEnabled( c ) );
	vbool( b3World_IsWarmStartingEnabled( c ) );
	vbool( b3World_IsContinuousEnabled( c ) );
	vbool( b3World_IsRestitutionPropagationEnabled( c ) );
	vu64( (uint64_t)(uintptr_t)b3World_GetUserData( c ) );
	pv3( b3World_GetGravity( c ) );
	vi32( b3World_GetWorkerCount( c ) );
	vi32( b3World_GetAwakeBodyCount( c ) );
	paabb( b3World_GetBounds( c ) );
	vend();

	b3DestroyWorld( c );
	b3DestroyWorld( a );

	vtag( "destroyed" );
	vbool( b3World_IsValid( a ) );
	vbool( b3World_IsValid( c ) );
	vi32( b3GetWorldCount() );
	vi32( b3GetMaxWorldCount() );
	vend();
}

static void createScene( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	g_worldId = b3CreateWorld( &worldDef );
	g_bodyCount = 0;

	b3World_SetFrictionCallback( g_worldId, minFriction );

	// Ground
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.8f;
		shapeDef.baseMaterial.userMaterialId = 11;
		b3BoxHull box = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
		b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enableContactEvents = true;
	shapeDef.enableSensorEvents = true;
	shapeDef.baseMaterial.userMaterialId = 22;

	// Box stack
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3BodyId hingeA = { 0 };
	b3BodyId hingeB = { 0 };
	for ( int i = 0; i < 5; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.02f * (float)i, 0.5f + 1.0f * (float)i, 0.0f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
		if ( i == 3 )
		{
			hingeA = bodyId;
		}
		if ( i == 4 )
		{
			hingeB = bodyId;
		}
	}

	// Spheres with hit events
	shapeDef.enableHitEvents = true;
	for ( int i = 0; i < 4; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 3.0f + 1.1f * (float)i, 2.0f + (float)i, 0.3f * (float)i };
		bodyDef.linearVelocity = (b3Vec3){ -1.0f, -2.0f * (float)i, 0.0f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.4f };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	}
	shapeDef.enableHitEvents = false;

	// Capsules, rotated
	for ( int i = 0; i < 3; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ -3.0f, 1.0f + 0.8f * (float)i, 0.1f * (float)i };
		b3Vec3 axis = b3Normalize( (b3Vec3){ 0.3f, 1.0f, 0.2f * (float)i } );
		bodyDef.rotation = b3MakeQuatFromAxisAngle( axis, 0.4f * (float)i );
		b3BodyId bodyId = addBody( &bodyDef );
		b3Capsule capsule = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.25f };
		b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
	}

	// Kinematic box sweeping through the capsules
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_kinematicBody;
		bodyDef.position = (b3Pos){ -8.0f, 0.6f, 0.0f };
		bodyDef.linearVelocity = (b3Vec3){ 2.0f, 0.0f, 0.0f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3BoxHull kbox = b3MakeBoxHull( 0.5f, 0.4f, 0.5f );
		b3CreateHullShape( bodyId, &shapeDef, &kbox.base );
	}

	// Bullet
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.position = (b3Pos){ 0.0f, 3.0f, -12.0f };
		bodyDef.linearVelocity = (b3Vec3){ 0.0f, 0.0f, 60.0f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.1f };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	}

	// Sensor
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 5.0f, 0.5f, 0.0f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3ShapeDef sensorDef = b3DefaultShapeDef();
		sensorDef.isSensor = true;
		sensorDef.enableSensorEvents = true;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 1.5f };
		b3CreateSphereShape( bodyId, &sensorDef, &sphere );
	}

	// Hinge between the top two boxes
	{
		b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = hingeA;
		jointDef.base.bodyIdB = hingeB;
		jointDef.base.localFrameA.p = (b3Vec3){ 0.5f, 0.5f, 0.0f };
		jointDef.base.localFrameB.p = (b3Vec3){ 0.5f, -0.5f, 0.0f };
		jointDef.base.forceThreshold = 10.0f;
		jointDef.base.torqueThreshold = 10.0f;
		b3CreateRevoluteJoint( g_worldId, &jointDef );
	}
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

static void printCounters( void )
{
	b3Counters s = b3World_GetCounters( g_worldId );
	vtag( "counters" );
	vi32( s.bodyCount );
	vi32( s.shapeCount );
	vi32( s.contactCount );
	vi32( s.jointCount );
	vi32( s.islandCount );
	vi32( s.staticTreeHeight );
	vi32( s.treeHeight );
	vi32( s.satCallCount );
	vi32( s.satCacheHitCount );
	vi32( s.awakeContactCount );
	vi32( s.recycledContactCount );
	vi32( s.distanceIterations );
	vi32( s.pushBackIterations );
	vi32( s.rootIterations );
	for ( int i = 0; i < B3_GRAPH_COLOR_COUNT; ++i )
	{
		vi32( s.colorCounts[i] );
	}
	for ( int i = 0; i < B3_CONTACT_MANIFOLD_COUNT_BUCKETS; ++i )
	{
		vi32( s.manifoldCounts[i] );
	}
	vi32( b3World_GetAwakeBodyCount( g_worldId ) );
	vend();

	b3Capacity c = b3World_GetMaxCapacity( g_worldId );
	vtag( "capacity" );
	vi32( c.staticShapeCount );
	vi32( c.dynamicShapeCount );
	vi32( c.staticBodyCount );
	vi32( c.dynamicBodyCount );
	vi32( c.contactCount );
	vend();
}

static void printEvents( void )
{
	b3ContactEvents contactEvents = b3World_GetContactEvents( g_worldId );
	for ( int i = 0; i < contactEvents.beginCount; ++i )
	{
		b3ContactBeginTouchEvent* e = contactEvents.beginEvents + i;
		vtag( "begin" );
		pshapeId( e->shapeIdA );
		pshapeId( e->shapeIdB );
		pcontactId( e->contactId );
		vbool( b3Contact_IsValid( e->contactId ) );
		vend();
	}
	for ( int i = 0; i < contactEvents.endCount; ++i )
	{
		b3ContactEndTouchEvent* e = contactEvents.endEvents + i;
		vtag( "end" );
		pshapeId( e->shapeIdA );
		pshapeId( e->shapeIdB );
		pcontactId( e->contactId );
		vend();
	}
	for ( int i = 0; i < contactEvents.hitCount; ++i )
	{
		b3ContactHitEvent* e = contactEvents.hitEvents + i;
		vtag( "hit" );
		pshapeId( e->shapeIdA );
		pshapeId( e->shapeIdB );
		pcontactId( e->contactId );
		pv3( e->point );
		pv3( e->normal );
		vf32( e->approachSpeed );
		vu64( e->userMaterialIdA );
		vu64( e->userMaterialIdB );
		vend();
	}

	b3SensorEvents sensorEvents = b3World_GetSensorEvents( g_worldId );
	for ( int i = 0; i < sensorEvents.beginCount; ++i )
	{
		vtag( "sensorBegin" );
		pshapeId( sensorEvents.beginEvents[i].sensorShapeId );
		pshapeId( sensorEvents.beginEvents[i].visitorShapeId );
		vend();
	}
	for ( int i = 0; i < sensorEvents.endCount; ++i )
	{
		vtag( "sensorEnd" );
		pshapeId( sensorEvents.endEvents[i].sensorShapeId );
		pshapeId( sensorEvents.endEvents[i].visitorShapeId );
		vend();
	}

	b3BodyEvents bodyEvents = b3World_GetBodyEvents( g_worldId );
	vtag( "moves" );
	vi32( bodyEvents.moveCount );
	for ( int i = 0; i < bodyEvents.moveCount; ++i )
	{
		pbodyId( bodyEvents.moveEvents[i].bodyId );
		vbool( bodyEvents.moveEvents[i].fellAsleep );
		pv3( bodyEvents.moveEvents[i].transform.p );
	}
	vend();

	b3JointEvents jointEvents = b3World_GetJointEvents( g_worldId );
	vtag( "joints" );
	vi32( jointEvents.count );
	vend();
}

static bool overlapFcn( b3ShapeId shapeId, void* context )
{
	int* count = context;
	*count += 1;
	vtag( "overlap" );
	pshapeId( shapeId );
	vend();
	return true;
}

static float castFcn( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
					  int triangleIndex, int childIndex, void* context )
{
	int* mode = context;
	vtag( "cast" );
	pshapeId( shapeId );
	pv3( point );
	pv3( normal );
	vf32( fraction );
	vu64( userMaterialId );
	vi32( triangleIndex );
	vi32( childIndex );
	vend();

	if ( *mode == 0 )
	{
		return fraction;
	}

	if ( *mode == 1 )
	{
		return 1.0f;
	}

	// Skip the first hit, then clip
	*mode = 0;
	return -1.0f;
}

static bool moverFilter( b3ShapeId shapeId, void* context )
{
	(void)context;
	vtag( "moverFilter" );
	pshapeId( shapeId );
	vend();
	return shapeId.index1 != 1;
}

static bool planeFcn( b3ShapeId shapeId, const b3PlaneResult* planes, int count, void* context )
{
	(void)context;
	vtag( "planes" );
	pshapeId( shapeId );
	vi32( count );
	for ( int i = 0; i < count; ++i )
	{
		pv3( planes[i].plane.normal );
		vf32( planes[i].plane.offset );
		pv3( planes[i].point );
		vi32( planes[i].triangleIndex );
		vi32( planes[i].childIndex );
		vi32( planes[i].materialIndex );

		if ( g_planeCount < PLANE_CAPACITY )
		{
			b3CollisionPlane* p = g_planes + g_planeCount;
			p->plane = planes[i].plane;
			p->pushLimit = FLT_MAX;
			p->push = 0.0f;
			p->clipVelocity = true;
			g_planeCount += 1;
		}
	}
	vend();
	return true;
}

static void queries( void )
{
	b3QueryFilter filter = b3DefaultQueryFilter();

	{
		int count = 0;
		b3AABB box = { { -2.0f, -0.5f, -2.0f }, { 2.0f, 3.0f, 2.0f } };
		b3TreeStats stats = b3World_OverlapAABB( g_worldId, box, filter, overlapFcn, &count );
		vtag( "overlapAABB" );
		vi32( count );
		pstats( stats );
		vend();
	}

	{
		int count = 0;
		b3Vec3 center = { 0.0f, 0.0f, 0.0f };
		b3ShapeProxy proxy = { .points = &center, .count = 1, .radius = 1.0f };
		b3TreeStats stats = b3World_OverlapShape( g_worldId, (b3Pos){ 2.0f, 0.5f, 0.0f }, &proxy, filter, overlapFcn, &count );
		vtag( "overlapShape" );
		vi32( count );
		pstats( stats );
		vend();
	}

	for ( int mode = 0; mode < 3; ++mode )
	{
		int state = mode;
		b3TreeStats stats = b3World_CastRay( g_worldId, (b3Pos){ -9.0f, 0.5f, 0.1f }, (b3Vec3){ 18.0f, 0.0f, 0.0f }, filter,
											 castFcn, &state );
		vtag( "castRay" );
		vi32( mode );
		pstats( stats );
		vend();
	}

	{
		b3RayResult result =
			b3World_CastRayClosest( g_worldId, (b3Pos){ 0.05f, 10.0f, 0.02f }, (b3Vec3){ 0.0f, -20.0f, 0.0f }, filter );
		vtag( "rayClosest" );
		pshapeId( result.shapeId );
		pv3( result.point );
		pv3( result.normal );
		vu64( result.userMaterialId );
		vf32( result.fraction );
		vi32( result.triangleIndex );
		vi32( result.childIndex );
		vi32( result.nodeVisits );
		vi32( result.leafVisits );
		vbool( result.hit );
		vend();
	}

	{
		b3Vec3 points[8];
		for ( int i = 0; i < 8; ++i )
		{
			points[i].x = ( i & 1 ) ? 0.3f : -0.3f;
			points[i].y = ( i & 2 ) ? 0.2f : -0.2f;
			points[i].z = ( i & 4 ) ? 0.3f : -0.3f;
		}
		b3ShapeProxy proxy = { .points = points, .count = 8, .radius = 0.05f };
		int state = 0;
		b3TreeStats stats = b3World_CastShape( g_worldId, (b3Pos){ -9.0f, 2.0f, 0.0f }, &proxy, (b3Vec3){ 18.0f, -1.5f, 0.0f },
											   filter, castFcn, &state );
		vtag( "castShape" );
		pstats( stats );
		vend();
	}

	{
		b3Capsule mover = { { 0.0f, -0.3f, 0.0f }, { 0.0f, 0.3f, 0.0f }, 0.3f };
		float fraction = b3World_CastMover( g_worldId, (b3Pos){ -6.0f, 1.0f, 0.0f }, &mover, (b3Vec3){ 12.0f, -0.5f, 0.0f },
											filter, moverFilter, NULL );
		float unfiltered =
			b3World_CastMover( g_worldId, (b3Pos){ 0.0f, 6.0f, 0.0f }, &mover, (b3Vec3){ 0.0f, -8.0f, 0.0f }, filter, NULL, NULL );
		vtag( "castMover" );
		vf32( fraction );
		vf32( unfiltered );
		vend();
	}

	{
		g_planeCount = 0;
		b3Capsule mover = { { 0.0f, -0.4f, 0.0f }, { 0.0f, 0.4f, 0.0f }, 0.35f };
		b3World_CollideMover( g_worldId, (b3Pos){ 0.6f, 0.7f, 0.2f }, &mover, filter, planeFcn, NULL );
		b3PlaneSolverResult result = b3SolvePlanes( (b3Vec3){ -0.3f, -0.2f, 0.1f }, g_planes, g_planeCount );
		b3Vec3 clipped = b3ClipVector( (b3Vec3){ -2.0f, -1.0f, 0.5f }, g_planes, g_planeCount );
		vtag( "collideMover" );
		vi32( g_planeCount );
		pv3( result.delta );
		vi32( result.iterationCount );
		pv3( clipped );
		vend();
	}

	vtag( "bounds" );
	paabb( b3World_GetBounds( g_worldId ) );
	vend();
}

static void step( int stepIndex )
{
	if ( stepIndex == 40 )
	{
		b3ExplosionDef def = b3DefaultExplosionDef();
		def.position = (b3Pos){ 0.0f, 0.5f, 0.5f };
		def.radius = 2.0f;
		def.falloff = 1.0f;
		def.impulsePerArea = 5.0f;
		b3World_Explode( g_worldId, &def );
	}

	if ( stepIndex == 60 )
	{
		b3World_SetFrictionCallback( g_worldId, NULL );
		b3World_SetGravity( g_worldId, (b3Vec3){ 0.5f, -9.0f, 0.0f } );
		b3DestroyBody( g_bodies[6] );
	}

	if ( stepIndex == 80 )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 1.0f, 4.0f, 0.5f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.3f };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	}

	if ( stepIndex == 100 )
	{
		b3World_SetContactTuning( g_worldId, 20.0f, 5.0f, 2.0f );
		b3World_EnableWarmStarting( g_worldId, false );
	}

	b3World_Step( g_worldId, 1.0f / 60.0f, 4 );

	vtag( "step" );
	vi32( stepIndex );
	vend();

	printBodies();
	printCounters();
	printEvents();

	if ( stepIndex % 20 == 19 )
	{
		queries();
	}
}

int main( void )
{
	registry();

	createScene();
	for ( int i = 0; i < 160; ++i )
	{
		step( i );
	}

	// A zero time step collides without solving
	b3World_Step( g_worldId, 0.0f, 4 );
	printCounters();
	printEvents();

	b3DestroyWorld( g_worldId );
	vtag( "final" );
	vbool( b3World_IsValid( g_worldId ) );
	vi32( b3GetWorldCount() );
	vend();
	return 0;
}
