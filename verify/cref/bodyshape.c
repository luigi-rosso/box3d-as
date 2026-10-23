// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/body.as and lib/shape.as: body and shape creation, every getter
// and setter, type changes, enable and disable, sleep, mass, queries per body and shape,
// destruction with id reuse, then stepping with contact data.

#include "common.h"

#include "body.h"
#include "physics_world.h"
#include "shape.h"
#include "solver_set.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#define BODY_CAPACITY 32
#define SHAPE_CAPACITY 16
#define CONTACT_CAPACITY 32

static b3WorldId g_worldId;
static b3BodyId g_bodies[BODY_CAPACITY];
static int g_bodyCount;
static b3HullData* g_rock;
static b3MeshData* g_mesh;
static b3HeightFieldData* g_heightField;

static b3Vec3 rvec( float lo, float hi )
{
	b3Vec3 v;
	v.x = vrandf( lo, hi );
	v.y = vrandf( lo, hi );
	v.z = vrandf( lo, hi );
	return v;
}

static void pvec( b3Vec3 v )
{
	vf32( v.x );
	vf32( v.y );
	vf32( v.z );
}

static void pquat( b3Quat q )
{
	pvec( q.v );
	vf32( q.s );
}

static void pmat( b3Matrix3 m )
{
	pvec( m.cx );
	pvec( m.cy );
	pvec( m.cz );
}

static void paabb( b3AABB a )
{
	pvec( a.lowerBound );
	pvec( a.upperBound );
}

static void pshapeId( b3ShapeId id )
{
	vi32( id.index1 );
	vu32( id.world0 );
	vu32( id.generation );
}

static void pmaterial( b3SurfaceMaterial m )
{
	vf32( m.friction );
	vf32( m.restitution );
	vf32( m.rollingResistance );
	pvec( m.tangentVelocity );
	vu64( m.userMaterialId );
	vu32( m.customColor );
}

static void pcast( b3CastOutput o )
{
	pvec( o.normal );
	pvec( o.point );
	vf32( o.fraction );
	vi32( o.iterations );
	vi32( o.triangleIndex );
	vi32( o.childIndex );
	vi32( o.materialIndex );
	vbool( o.hit );
}

static void dumpShape( b3ShapeId shapeId )
{
	b3World* world = b3GetWorldFromId( g_worldId );
	b3Shape* shape = world->shapes.data + ( shapeId.index1 - 1 );

	vtag( "shape" );
	pshapeId( shapeId );
	vi32( b3Shape_GetType( shapeId ) );
	paabb( b3Shape_GetAABB( shapeId ) );
	vf32( b3Shape_GetDensity( shapeId ) );
	vf32( b3Shape_GetFriction( shapeId ) );
	vf32( b3Shape_GetRestitution( shapeId ) );
	vi32( b3Shape_GetMeshMaterialCount( shapeId ) );
	b3Filter filter = b3Shape_GetFilter( shapeId );
	vu64( filter.categoryBits );
	vu64( filter.maskBits );
	vi32( filter.groupIndex );
	vbool( b3Shape_IsSensor( shapeId ) );
	vbool( b3Shape_AreSensorEventsEnabled( shapeId ) );
	vbool( b3Shape_AreContactEventsEnabled( shapeId ) );
	vbool( b3Shape_ArePreSolveEventsEnabled( shapeId ) );
	vbool( b3Shape_AreHitEventsEnabled( shapeId ) );
	vu64( (uint64_t)(uintptr_t)b3Shape_GetUserData( shapeId ) );
	vtag( b3Shape_GetName( shapeId ) );
	b3BodyId bodyId = b3Shape_GetBody( shapeId );
	vi32( bodyId.index1 );
	vu32( bodyId.generation );
	vend();

	vtag( "shapemat" );
	int materialCount = b3Shape_GetMeshMaterialCount( shapeId );
	pmaterial( b3Shape_GetSurfaceMaterial( shapeId ) );
	for ( int i = 1; i < materialCount; ++i )
	{
		pmaterial( b3Shape_GetMeshSurfaceMaterial( shapeId, i ) );
	}
	vend();

	vtag( "shapemass" );
	b3MassData massData = b3Shape_ComputeMassData( shapeId );
	vf32( massData.mass );
	pvec( massData.center );
	pmat( massData.inertia );
	vend();

	vtag( "shapeint" );
	vi32( shape->id );
	vi32( shape->bodyId );
	vi32( shape->prevShapeId );
	vi32( shape->nextShapeId );
	vi32( shape->sensorIndex );
	vi32( shape->proxyKey );
	vf32( shape->explosionScale );
	vf32( shape->aabbMargin );
	pvec( shape->localCentroid );
	vu32( shape->flags );
	paabb( world->fatAABBs.data[shape->id] );
	vend();

	b3ShapeType type = shape->type;
	if ( type == b3_sphereShape )
	{
		b3Sphere s = b3Shape_GetSphere( shapeId );
		vtag( "sphere" );
		pvec( s.center );
		vf32( s.radius );
		vend();
	}
	else if ( type == b3_capsuleShape )
	{
		b3Capsule c = b3Shape_GetCapsule( shapeId );
		vtag( "capsule" );
		pvec( c.center1 );
		pvec( c.center2 );
		vf32( c.radius );
		vend();
	}
	else if ( type == b3_hullShape )
	{
		const b3HullData* hull = b3Shape_GetHull( shapeId );
		vtag( "hull" );
		vi32( hull->vertexCount );
		vi32( hull->faceCount );
		pvec( hull->center );
		vf32( hull->innerRadius );
		vf32( hull->surfaceArea );
		vend();
	}
	else if ( type == b3_meshShape )
	{
		b3Mesh mesh = b3Shape_GetMesh( shapeId );
		vtag( "mesh" );
		vi32( mesh.data->triangleCount );
		pvec( mesh.scale );
		vend();
	}
}

static void dumpBody( b3BodyId bodyId )
{
	b3World* world = b3GetWorldFromId( g_worldId );
	b3Body* body = world->bodies.data + ( bodyId.index1 - 1 );

	vtag( "body" );
	vi32( bodyId.index1 );
	vu32( bodyId.generation );
	vi32( b3Body_GetType( bodyId ) );
	pvec( b3Body_GetPosition( bodyId ) );
	pquat( b3Body_GetRotation( bodyId ) );
	pvec( b3Body_GetLinearVelocity( bodyId ) );
	pvec( b3Body_GetAngularVelocity( bodyId ) );
	vf32( b3Body_GetMass( bodyId ) );
	vf32( b3Body_GetInverseMass( bodyId ) );
	pvec( b3Body_GetLocalCenter( bodyId ) );
	pvec( b3Body_GetWorldCenter( bodyId ) );
	vend();

	vtag( "bodyinertia" );
	pmat( b3Body_GetLocalRotationalInertia( bodyId ) );
	pmat( b3Body_GetWorldInverseRotationalInertia( bodyId ) );
	b3MassData massData = b3Body_GetMassData( bodyId );
	vf32( massData.mass );
	pvec( massData.center );
	pmat( massData.inertia );
	vend();

	vtag( "bodyprops" );
	vf32( b3Body_GetMinExtent( bodyId ) );
	pvec( b3Body_GetMaxExtent( bodyId ) );
	pvec( b3Body_GetMaxExtentOrigin( bodyId ) );
	vbool( b3Body_IsAwake( bodyId ) );
	vbool( b3Body_IsEnabled( bodyId ) );
	vbool( b3Body_IsSleepEnabled( bodyId ) );
	vbool( b3Body_IsBullet( bodyId ) );
	vbool( b3Body_IsFastRotationAllowed( bodyId ) );
	vbool( b3Body_IsContactRecyclingEnabled( bodyId ) );
	b3MotionLocks locks = b3Body_GetMotionLocks( bodyId );
	vbool( locks.linearX );
	vbool( locks.linearY );
	vbool( locks.linearZ );
	vbool( locks.angularX );
	vbool( locks.angularY );
	vbool( locks.angularZ );
	vi32( b3Body_GetShapeCount( bodyId ) );
	vi32( b3Body_GetJointCount( bodyId ) );
	vf32( b3Body_GetSleepThreshold( bodyId ) );
	vf32( b3Body_GetSafetyFactor( bodyId ) );
	vf32( b3Body_GetGravityScale( bodyId ) );
	vf32( b3Body_GetLinearDamping( bodyId ) );
	vf32( b3Body_GetAngularDamping( bodyId ) );
	vu64( (uint64_t)(uintptr_t)b3Body_GetUserData( bodyId ) );
	vtag( b3Body_GetName( bodyId ) );
	paabb( b3Body_ComputeAABB( bodyId ) );
	b3WorldId worldId = b3Body_GetWorld( bodyId );
	vu32( worldId.index1 );
	vu32( worldId.generation );
	vend();

	vtag( "bodyint" );
	vi32( body->setIndex );
	vi32( body->localIndex );
	vi32( body->headContactKey );
	vi32( body->contactCount );
	vi32( body->headShapeId );
	vi32( body->headJointKey );
	vi32( body->islandId );
	vi32( body->islandIndex );
	vf32( body->sleepTime );
	vi32( body->bodyMoveIndex );
	vu32( body->flags );
	vu32( body->nameId );
	b3SolverSet* set = world->solverSets.data + body->setIndex;
	b3BodySim* sim = set->bodySims.data + body->localIndex;
	vu32( sim->flags );
	pvec( sim->center0 );
	pquat( sim->rotation0 );
	pvec( sim->force );
	pvec( sim->torque );
	pmat( sim->invInertiaLocal );
	vi32( sim->bodyId );
	if ( body->setIndex == b3_awakeSet )
	{
		b3BodyState* state = set->bodyStates.data + body->localIndex;
		vu32( state->flags );
	}
	vend();

	b3ShapeId shapes[SHAPE_CAPACITY];
	int shapeCount = b3Body_GetShapes( bodyId, shapes, SHAPE_CAPACITY );
	for ( int i = 0; i < shapeCount; ++i )
	{
		dumpShape( shapes[i] );
	}
}

static void dumpWorld( const char* label )
{
	b3World* world = b3GetWorldFromId( g_worldId );
	vtag( "world" );
	vtag( label );
	vi32( world->bodies.count );
	vi32( world->shapes.count );
	vi32( world->solverSets.count );
	vi32( world->islands.count );
	vi32( world->sensors.count );
	vi32( world->compoundShapeCount );
	vi32( b3GetIdCount( &world->bodyIdPool ) );
	vi32( b3GetIdCount( &world->shapeIdPool ) );
	vi32( b3GetIdCount( &world->solverSetIdPool ) );
	vi32( b3GetIdCount( &world->islandIdPool ) );
	vend();

	for ( int i = 0; i < g_bodyCount; ++i )
	{
		if ( b3Body_IsValid( g_bodies[i] ) )
		{
			dumpBody( g_bodies[i] );
		}
		else
		{
			vtag( "dead" );
			vi32( i );
			vend();
		}
	}
}

static b3BodyId addBody( const b3BodyDef* def )
{
	b3BodyId id = b3CreateBody( g_worldId, def );
	g_bodies[g_bodyCount] = id;
	g_bodyCount += 1;
	return id;
}

static void phaseCreate( void )
{
	vseed( 0x626f647973686170 );

	b3WorldDef worldDef = b3DefaultWorldDef();
	g_worldId = b3CreateWorld( &worldDef );

	g_rock = b3CreateRock( 0.6f );
	g_mesh = b3CreateGridMesh( 4, 4, 1.0f, 2, true );
	g_heightField = b3CreateGrid( 5, 5, (b3Vec3){ 1.0f, 0.5f, 1.0f }, false );

	// 0: static ground box, named, with a sensor
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ 0.0f, -1.0f, 0.0f };
		bodyDef.name = "ground";
		bodyDef.userData = (void*)(uintptr_t)7;
		b3BodyId bodyId = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.7f;
		shapeDef.name = "groundbox";
		b3BoxHull box = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );

		b3ShapeDef sensorDef = b3DefaultShapeDef();
		sensorDef.isSensor = true;
		sensorDef.enableSensorEvents = true;
		b3Sphere sensorSphere = { { 0.0f, 3.0f, 0.0f }, 2.0f };
		b3CreateSphereShape( bodyId, &sensorDef, &sensorSphere );
	}

	// 1: static mesh with two materials
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ 30.0f, 0.0f, 0.0f };
		b3BodyId bodyId = addBody( &bodyDef );

		b3SurfaceMaterial materials[2] = { b3DefaultSurfaceMaterial(), b3DefaultSurfaceMaterial() };
		materials[0].friction = 0.3f;
		materials[0].userMaterialId = 11;
		materials[1].friction = 0.9f;
		materials[1].userMaterialId = 12;
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.materials = materials;
		shapeDef.materialCount = 2;
		b3CreateMeshShape( bodyId, &shapeDef, g_mesh, (b3Vec3){ 1.0f, 1.0f, -1.0f } );
	}

	// 2: static height field
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ -30.0f, 0.0f, 0.0f };
		b3BodyId bodyId = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHeightFieldShape( bodyId, &shapeDef, g_heightField );
	}

	// 3: dynamic sphere
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Vec3){ 0.0f, 2.0f, 0.0f };
		bodyDef.linearVelocity = (b3Vec3){ 0.5f, 0.0f, -0.25f };
		bodyDef.name = "ball";
		b3BodyId bodyId = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 2.0f;
		shapeDef.enableHitEvents = true;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	}

	// 4: dynamic capsule, rotated
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Vec3){ 2.0f, 2.0f, 0.0f };
		bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Normalize( (b3Vec3){ 1.0f, 2.0f, 3.0f } ), 0.7f );
		bodyDef.angularVelocity = (b3Vec3){ 0.0f, 1.5f, 0.0f };
		b3BodyId bodyId = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3Capsule capsule = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.1f, 0.0f }, 0.25f };
		b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
	}

	// 5: dynamic box hull
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Vec3){ -2.0f, 2.0f, 0.0f };
		bodyDef.rotation = b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 0.0f, 1.0f }, 0.3f );
		bodyDef.isBullet = true;
		b3BodyId bodyId = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.restitution = 0.4f;
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.4f, 0.3f );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	// 6: transformed hull and a shared rock hull
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Vec3){ 4.0f, 2.0f, 0.0f };
		b3BodyId bodyId = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3Transform xf = { { 0.2f, 0.1f, 0.0f }, b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 1.0f, 0.0f }, 0.5f ) };
		b3CreateTransformedHullShape( bodyId, &shapeDef, g_rock, xf, (b3Vec3){ 1.0f, 2.0f, 0.5f } );
		b3CreateHullShape( bodyId, &shapeDef, g_rock );
	}

	// 7: compound body with a zero density shape
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Vec3){ 0.0f, 4.0f, 2.0f };
		bodyDef.gravityScale = 0.5f;
		bodyDef.linearDamping = 0.1f;
		bodyDef.angularDamping = 0.2f;
		b3BodyId bodyId = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3Sphere sphere = { { 0.5f, 0.0f, 0.0f }, 0.3f };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		shapeDef.density = 3.0f;
		b3Capsule capsule = { { -0.5f, 0.0f, 0.0f }, { -0.5f, 1.0f, 0.0f }, 0.2f };
		b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
		shapeDef.density = 0.0f;
		b3BoxHull box = b3MakeOffsetBoxHull( 0.2f, 0.2f, 0.2f, (b3Vec3){ 0.0f, -0.5f, 0.3f } );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	// 8: kinematic hull
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_kinematicBody;
		bodyDef.position = (b3Vec3){ 6.0f, 1.0f, 0.0f };
		bodyDef.angularVelocity = (b3Vec3){ 0.0f, 0.0f, 1.0f };
		b3BodyId bodyId = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( 1.0f, 0.2f, 0.5f );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	// 9: sleeping sphere in its own set
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Vec3){ 8.0f, 0.5f, 0.0f };
		bodyDef.isAwake = false;
		b3BodyId bodyId = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	}

	// 10: disabled body
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Vec3){ 10.0f, 2.0f, 0.0f };
		bodyDef.isEnabled = false;
		b3BodyId bodyId = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeCubeHull( 0.4f );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	// 11: motion locks, degenerate capsule, deferred mass
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Vec3){ -4.0f, 3.0f, 1.0f };
		bodyDef.motionLocks.angularX = true;
		bodyDef.motionLocks.angularZ = true;
		bodyDef.motionLocks.linearY = true;
		bodyDef.allowFastRotation = true;
		bodyDef.enableContactRecycling = false;
		b3BodyId bodyId = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.updateBodyMass = false;
		b3Capsule capsule = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0001f, 0.0f }, 0.3f };
		b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
		b3BoxHull box = b3MakeBoxHull( 0.3f, 0.6f, 0.2f );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	// 12: fixed rotation body
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Vec3){ -6.0f, 1.0f, -1.0f };
		bodyDef.motionLocks.angularX = true;
		bodyDef.motionLocks.angularY = true;
		bodyDef.motionLocks.angularZ = true;
		bodyDef.enableSleep = false;
		b3BodyId bodyId = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.filter.groupIndex = -3;
		shapeDef.filter.categoryBits = 0x4;
		b3BoxHull box = b3MakeBoxHull( 0.4f, 0.4f, 0.4f );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	dumpWorld( "create" );

	b3Body_ApplyMassFromShapes( g_bodies[11] );
	dumpBody( g_bodies[11] );
}

static void phaseQueries( void )
{
	vseed( 0x7175657279 );
	b3QueryFilter filter = b3DefaultQueryFilter();

	for ( int i = 0; i < g_bodyCount; ++i )
	{
		if ( b3Body_IsValid( g_bodies[i] ) == false )
		{
			continue;
		}

		b3BodyId bodyId = g_bodies[i];
		b3WorldTransform xf = b3Body_GetTransform( bodyId );
		b3Vec3 center = b3Body_GetPosition( bodyId );

		for ( int k = 0; k < 4; ++k )
		{
			b3Vec3 offset = rvec( -3.0f, 3.0f );
			b3Vec3 target = b3Add( center, offset );

			b3Vec3 closest;
			float distance = b3Body_GetClosestPoint( bodyId, &closest, target );
			vtag( "closest" );
			vi32( i );
			vf32( distance );
			pvec( closest );
			vend();

			b3Vec3 translation = b3MulSV( -2.0f, offset );
			b3BodyCastResult ray = b3Body_CastRay( bodyId, target, translation, filter, 1.0f, xf );
			vtag( "ray" );
			pshapeId( ray.shapeId );
			pvec( ray.point );
			pvec( ray.normal );
			vf32( ray.fraction );
			vi32( ray.triangleIndex );
			vu64( ray.userMaterialId );
			vi32( ray.iterations );
			vbool( ray.hit );
			vend();

			b3Vec3 points[2] = { target, b3Add( target, (b3Vec3){ 0.0f, 0.3f, 0.0f } ) };
			b3ShapeProxy proxy = { points, 2, 0.2f };
			b3BodyCastResult cast = b3Body_CastShape( bodyId, target, &proxy, translation, filter, 1.0f, false, xf );
			vtag( "cast" );
			pshapeId( cast.shapeId );
			pvec( cast.point );
			pvec( cast.normal );
			vf32( cast.fraction );
			vi32( cast.triangleIndex );
			vu64( cast.userMaterialId );
			vi32( cast.iterations );
			vbool( cast.hit );
			vend();

			b3Vec3 mid = b3MulAdd( target, 0.75f, translation );
			b3Vec3 overlapPoints[1] = { mid };
			b3ShapeProxy overlapProxy = { overlapPoints, 1, 0.5f };
			vtag( "overlap" );
			vbool( b3Body_OverlapShape( bodyId, b3Vec3_zero, &overlapProxy, filter, xf ) );
			vend();

			b3Capsule mover = { mid, b3Add( mid, (b3Vec3){ 0.0f, 0.5f, 0.0f } ), 0.4f };
			b3BodyPlaneResult planes[4];
			int planeCount = b3Body_CollideMover( bodyId, planes, 4, b3Vec3_zero, &mover, filter, xf );
			vtag( "mover" );
			vi32( planeCount );
			for ( int p = 0; p < planeCount; ++p )
			{
				pshapeId( planes[p].shapeId );
				pvec( planes[p].result.plane.normal );
				vf32( planes[p].result.plane.offset );
				pvec( planes[p].result.point );
				vi32( planes[p].result.triangleIndex );
				vi32( planes[p].result.childIndex );
				vi32( planes[p].result.materialIndex );
			}
			vend();

			b3Capsule moverAtTarget = { target, b3Add( target, (b3Vec3){ 0.0f, 0.5f, 0.0f } ), 0.2f };
			b3WorldTransform xf2 = xf;
			xf2.p = b3Add( xf.p, (b3Vec3){ 0.1f, 0.0f, 0.0f } );
			b3BodyTOIResult toi = b3Body_TimeOfImpactMover( bodyId, b3Vec3_zero, &moverAtTarget, translation, filter, xf, xf2 );
			vtag( "toi" );
			pvec( toi.point );
			pvec( toi.normal );
			vf32( toi.fraction );
			pshapeId( toi.shapeId );
			vend();
		}

		b3ShapeId shapes[SHAPE_CAPACITY];
		int shapeCount = b3Body_GetShapes( bodyId, shapes, SHAPE_CAPACITY );
		for ( int s = 0; s < shapeCount; ++s )
		{
			b3Vec3 origin = b3Add( center, rvec( -3.0f, 3.0f ) );
			b3Vec3 translation = b3MulSV( 2.0f, b3Sub( center, origin ) );
			b3CastOutput output = b3Shape_RayCast( shapes[s], origin, translation );
			vtag( "shaperay" );
			pcast( output );
			vend();

			b3ShapeType type = b3Shape_GetType( shapes[s] );
			if ( type == b3_sphereShape || type == b3_capsuleShape || type == b3_hullShape )
			{
				vtag( "shapeclosest" );
				pvec( b3Shape_GetClosestPoint( shapes[s], origin ) );
				vend();
			}
		}
	}
}

static void dumpContacts( void )
{
	b3ContactData contactData[CONTACT_CAPACITY];
	for ( int i = 0; i < g_bodyCount; ++i )
	{
		if ( b3Body_IsValid( g_bodies[i] ) == false )
		{
			continue;
		}

		vtag( "contacts" );
		vi32( i );
		vi32( b3Body_GetContactCapacity( g_bodies[i] ) );
		int count = b3Body_GetContactData( g_bodies[i], contactData, CONTACT_CAPACITY );
		vi32( count );
		for ( int c = 0; c < count; ++c )
		{
			vi32( contactData[c].contactId.index1 );
			vu32( contactData[c].contactId.generation );
			pshapeId( contactData[c].shapeIdA );
			pshapeId( contactData[c].shapeIdB );
			vi32( contactData[c].manifoldCount );
		}
		vend();

		b3ShapeId shapes[SHAPE_CAPACITY];
		int shapeCount = b3Body_GetShapes( g_bodies[i], shapes, SHAPE_CAPACITY );
		for ( int s = 0; s < shapeCount; ++s )
		{
			vtag( "shapecontacts" );
			vi32( b3Shape_GetContactCapacity( shapes[s] ) );
			int n = b3Shape_GetContactData( shapes[s], contactData, CONTACT_CAPACITY );
			vi32( n );
			for ( int c = 0; c < n; ++c )
			{
				vi32( contactData[c].contactId.index1 );
				vi32( contactData[c].manifoldCount );
			}
			vi32( b3Shape_GetSensorCapacity( shapes[s] ) );
			b3ShapeId visitors[SHAPE_CAPACITY];
			int v = b3Shape_GetSensorData( shapes[s], visitors, SHAPE_CAPACITY );
			vi32( v );
			for ( int k = 0; k < v; ++k )
			{
				pshapeId( visitors[k] );
			}
			vend();
		}
	}
}

static void phaseMutate( void )
{
	vseed( 0x6d757461 );

	b3BodyId ball = g_bodies[3];
	b3BodyId capsuleBody = g_bodies[4];
	b3BodyId boxBody = g_bodies[5];
	b3BodyId rockBody = g_bodies[6];
	b3BodyId compound = g_bodies[7];
	b3BodyId kinematic = g_bodies[8];
	b3BodyId sleeper = g_bodies[9];
	b3BodyId disabled = g_bodies[10];
	b3BodyId locked = g_bodies[11];

	b3Body_SetTransform( ball, (b3Vec3){ 0.5f, 2.5f, -0.5f }, b3MakeQuatFromAxisAngle( (b3Vec3){ 1.0f, 0.0f, 0.0f }, 0.2f ) );
	b3Body_SetLinearVelocity( ball, (b3Vec3){ 1.0f, 2.0f, 3.0f } );
	b3Body_SetAngularVelocity( ball, (b3Vec3){ -1.0f, 0.5f, 0.25f } );
	b3Body_ApplyForce( ball, (b3Vec3){ 10.0f, 0.0f, 0.0f }, (b3Vec3){ 0.5f, 2.6f, -0.5f }, true );
	b3Body_ApplyForceToCenter( ball, (b3Vec3){ 0.0f, 5.0f, 0.0f }, true );
	b3Body_ApplyTorque( ball, (b3Vec3){ 0.0f, 0.0f, 2.0f }, true );
	b3Body_ApplyLinearImpulse( ball, (b3Vec3){ 0.1f, 0.2f, 0.3f }, (b3Vec3){ 0.6f, 2.5f, -0.4f }, true );
	b3Body_ApplyLinearImpulseToCenter( ball, (b3Vec3){ 0.0f, -0.2f, 0.1f }, true );
	b3Body_ApplyAngularImpulse( ball, (b3Vec3){ 0.05f, 0.0f, -0.05f }, true );
	vtag( "pointvel" );
	pvec( b3Body_GetLocalPointVelocity( ball, (b3Vec3){ 0.1f, 0.2f, 0.3f } ) );
	pvec( b3Body_GetWorldPointVelocity( ball, (b3Vec3){ 0.6f, 2.4f, -0.5f } ) );
	pvec( b3Body_GetLocalPoint( ball, (b3Vec3){ 1.0f, 1.0f, 1.0f } ) );
	pvec( b3Body_GetWorldPoint( ball, (b3Vec3){ 1.0f, 1.0f, 1.0f } ) );
	pvec( b3Body_GetLocalVector( ball, (b3Vec3){ 1.0f, 1.0f, 1.0f } ) );
	pvec( b3Body_GetWorldVector( ball, (b3Vec3){ 1.0f, 1.0f, 1.0f } ) );
	vend();

	// Forces on a sleeping body: no wake, then wake
	b3Body_ApplyForceToCenter( sleeper, (b3Vec3){ 1.0f, 0.0f, 0.0f }, false );
	b3Body_ApplyLinearImpulse( sleeper, (b3Vec3){ 1.0f, 0.0f, 0.0f }, (b3Vec3){ 8.0f, 0.6f, 0.0f }, false );
	dumpBody( sleeper );
	b3Body_ApplyTorque( sleeper, (b3Vec3){ 0.0f, 1.0f, 0.0f }, true );
	dumpWorld( "forces" );

	b3MassData massData = { 2.5f, { 0.1f, 0.0f, -0.1f }, { { 0.3f, 0.0f, 0.0f }, { 0.0f, 0.4f, 0.0f }, { 0.0f, 0.0f, 0.5f } } };
	b3Body_SetMassData( boxBody, massData );
	dumpBody( boxBody );
	b3Body_SetMassData( boxBody, (b3MassData){ 0 } );
	dumpBody( boxBody );
	b3Body_ApplyMassFromShapes( boxBody );

	b3Body_SetLinearDamping( compound, 0.3f );
	b3Body_SetAngularDamping( compound, 0.4f );
	b3Body_SetGravityScale( compound, 2.0f );
	b3Body_SetSleepThreshold( compound, 0.02f );
	b3Body_SetSafetyFactor( compound, 0.5f );
	b3Body_SetUserData( compound, (void*)(uintptr_t)99 );
	b3Body_SetName( compound, "compound" );
	b3Body_EnableSleep( compound, false );
	b3Body_EnableSleep( compound, true );
	b3Body_SetBullet( capsuleBody, true );
	b3Body_AllowFastRotation( capsuleBody, true );
	b3Body_EnableContactRecycling( capsuleBody, false );
	b3Body_EnableHitEvents( rockBody, true );

	b3MotionLocks locks = { 0 };
	locks.linearX = true;
	locks.angularY = true;
	b3Body_SetMotionLocks( locked, locks );
	b3MotionLocks allLocks = { true, true, true, true, true, true };
	b3Body_SetMotionLocks( capsuleBody, allLocks );
	dumpWorld( "props" );
	b3Body_SetMotionLocks( capsuleBody, (b3MotionLocks){ 0 } );

	// Type changes
	b3Body_SetType( boxBody, b3_staticBody );
	dumpWorld( "tostatic" );
	b3Body_SetType( boxBody, b3_kinematicBody );
	dumpWorld( "tokinematic" );
	b3Body_SetType( boxBody, b3_dynamicBody );
	b3Body_SetType( kinematic, b3_dynamicBody );
	b3Body_SetType( disabled, b3_kinematicBody );
	dumpWorld( "todynamic" );

	// Enable and disable
	b3Body_Disable( rockBody );
	dumpWorld( "disable" );
	b3Body_Enable( rockBody );
	b3Body_Enable( disabled );
	b3Body_SetType( disabled, b3_dynamicBody );
	dumpWorld( "enable" );

	// Sleep
	b3Body_SetAwake( compound, false );
	dumpWorld( "sleep" );
	b3Body_SetAwake( compound, true );
	b3Body_SetTargetTransform( kinematic, (b3WorldTransform){ { 6.5f, 1.2f, 0.1f }, b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 1.0f, 0.0f }, 0.1f ) }, 1.0f / 60.0f, true );
	dumpWorld( "wake" );

	// Shape setters
	b3ShapeId shapes[SHAPE_CAPACITY];
	int shapeCount = b3Body_GetShapes( compound, shapes, SHAPE_CAPACITY );
	b3Shape_SetDensity( shapes[0], 5.0f, true );
	b3Shape_SetDensity( shapes[1], 1.0f, false );
	b3Shape_SetFriction( shapes[0], 0.25f );
	b3Shape_SetRestitution( shapes[1], 0.75f );
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.rollingResistance = 0.1f;
	material.tangentVelocity = (b3Vec3){ 1.0f, 0.0f, 0.0f };
	material.userMaterialId = 42;
	b3Shape_SetSurfaceMaterial( shapes[2], material );
	b3Shape_SetUserData( shapes[0], (void*)(uintptr_t)5 );
	b3Shape_SetName( shapes[1], "cap" );
	b3Shape_EnableContactEvents( shapes[0], false );
	b3Shape_EnablePreSolveEvents( shapes[1], true );
	b3Shape_EnableHitEvents( shapes[2], true );
	b3Shape_EnableSensorEvents( shapes[2], false );
	b3Filter filter = b3DefaultFilter();
	filter.categoryBits = 0x2;
	b3Shape_SetFilter( shapes[0], filter, true );
	filter.maskBits = 0xF;
	b3Shape_SetFilter( shapes[1], filter, false );
	vtag( "setters" );
	vi32( shapeCount );
	vend();

	b3ShapeId meshShapes[SHAPE_CAPACITY];
	b3Body_GetShapes( g_bodies[1], meshShapes, SHAPE_CAPACITY );
	b3SurfaceMaterial meshMaterial = b3DefaultSurfaceMaterial();
	meshMaterial.friction = 0.1f;
	b3Shape_SetMeshMaterial( meshShapes[0], meshMaterial, 1 );

	b3Sphere sphere = { { 0.1f, 0.2f, 0.3f }, 0.45f };
	b3Shape_SetSphere( shapes[2], &sphere );
	b3Capsule capsule = { { 0.0f, -0.4f, 0.0f }, { 0.0f, 0.4f, 0.0f }, 0.2f };
	b3Shape_SetCapsule( shapes[0], &capsule );
	b3Shape_SetHull( shapes[1], g_rock );
	b3Shape_SetHull( shapes[1], g_rock );
	b3Body_ApplyMassFromShapes( compound );
	dumpWorld( "shapes" );

	// Wind
	b3ShapeId windShapes[SHAPE_CAPACITY];
	int windCount = b3Body_GetShapes( compound, windShapes, SHAPE_CAPACITY );
	for ( int i = 0; i < windCount; ++i )
	{
		b3Shape_ApplyWind( windShapes[i], (b3Vec3){ 5.0f, 0.0f, 1.0f }, 0.5f, 0.2f, 10.0f, true );
	}
	b3ShapeId ballShapes[SHAPE_CAPACITY];
	b3Body_GetShapes( ball, ballShapes, SHAPE_CAPACITY );
	b3Shape_ApplyWind( ballShapes[0], (b3Vec3){ 0.0f, 0.0f, -3.0f }, 1.0f, 0.0f, 10.0f, true );
	dumpBody( compound );
	dumpBody( ball );

	// Destroy and reuse ids
	b3ShapeId rockShapes[SHAPE_CAPACITY];
	b3Body_GetShapes( rockBody, rockShapes, SHAPE_CAPACITY );
	b3DestroyShape( rockShapes[0], true );
	b3DestroyBody( sleeper );
	dumpWorld( "destroy" );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Vec3){ 1.0f, 5.0f, -1.0f };
		b3BodyId bodyId = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( 0.3f, 0.3f, 0.3f );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
		b3Sphere s = { { 0.0f, 0.4f, 0.0f }, 0.2f };
		b3CreateSphereShape( bodyId, &shapeDef, &s );
	}
	dumpWorld( "reuse" );
}

static void phaseStep( int stepCount )
{
	for ( int i = 0; i < stepCount; ++i )
	{
		b3World_Step( g_worldId, 1.0f / 60.0f, 4 );
	}

	dumpWorld( "step" );
	dumpContacts();
}

static void dumpEvents( void )
{
	b3ContactEvents contactEvents = b3World_GetContactEvents( g_worldId );
	vtag( "events" );
	vi32( contactEvents.beginCount );
	vi32( contactEvents.endCount );
	vi32( contactEvents.hitCount );
	for ( int i = 0; i < contactEvents.hitCount; ++i )
	{
		b3ContactHitEvent* e = contactEvents.hitEvents + i;
		pshapeId( e->shapeIdA );
		pshapeId( e->shapeIdB );
		pvec( e->point );
		pvec( e->normal );
		vf32( e->approachSpeed );
		vu64( e->userMaterialIdA );
		vu64( e->userMaterialIdB );
	}
	b3SensorEvents sensorEvents = b3World_GetSensorEvents( g_worldId );
	vi32( sensorEvents.beginCount );
	vi32( sensorEvents.endCount );
	for ( int i = 0; i < sensorEvents.beginCount; ++i )
	{
		pshapeId( sensorEvents.beginEvents[i].sensorShapeId );
		pshapeId( sensorEvents.beginEvents[i].visitorShapeId );
	}
	vend();
}

static void stepExtras( int stepCount )
{
	for ( int i = 0; i < stepCount; ++i )
	{
		b3World_Step( g_worldId, 1.0f / 60.0f, 4 );
		dumpEvents();
	}

	dumpWorld( "extras" );
	dumpContacts();
}

static void phaseExtras( void )
{
	vseed( 0x657874726173 );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.hitEventThreshold = 0.1f;
	g_worldId = b3CreateWorld( &worldDef );
	g_bodyCount = 0;

	b3SurfaceMaterial materialA = b3DefaultSurfaceMaterial();
	materialA.userMaterialId = 21;
	b3SurfaceMaterial materialB = b3DefaultSurfaceMaterial();
	materialB.userMaterialId = 22;
	materialB.friction = 0.4f;

	// 0: static compound with every child type
	b3CompoundCapsuleDef capsules[1];
	capsules[0].capsule = (b3Capsule){ { -3.0f, 0.0f, 0.0f }, { -2.0f, 0.0f, 0.5f }, 0.3f };
	capsules[0].material = materialA;
	b3BoxHull box = b3MakeBoxHull( 1.0f, 0.25f, 1.0f );
	b3CompoundHullDef hulls[1];
	hulls[0].hull = &box.base;
	hulls[0].transform = (b3Transform){ { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
	hulls[0].material = materialB;
	b3SurfaceMaterial meshMaterials[2] = { materialA, materialB };
	b3CompoundMeshDef meshes[1];
	meshes[0].meshData = g_mesh;
	meshes[0].transform = (b3Transform){ { 2.0f, -0.2f, -2.0f }, b3Quat_identity };
	meshes[0].scale = (b3Vec3){ 1.0f, 1.0f, 1.0f };
	meshes[0].materials = meshMaterials;
	meshes[0].materialCount = 2;
	b3CompoundSphereDef spheres[1];
	spheres[0].sphere = (b3Sphere){ { 0.0f, 0.0f, 3.0f }, 0.5f };
	spheres[0].material = materialA;
	b3CompoundDef compoundDef = { 0 };
	compoundDef.capsules = capsules;
	compoundDef.capsuleCount = 1;
	compoundDef.hulls = hulls;
	compoundDef.hullCount = 1;
	compoundDef.meshes = meshes;
	compoundDef.meshCount = 1;
	compoundDef.spheres = spheres;
	compoundDef.sphereCount = 1;
	b3CompoundData* compound = b3CreateCompound( &compoundDef );
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId bodyId = addBody( &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableHitEvents = true;
		b3CreateBakedCompoundShape( bodyId, &shapeDef, compound );
	}

	// 1: static mesh and a sensor
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ 20.0f, 0.0f, 0.0f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3SurfaceMaterial materials[2] = { materialA, materialB };
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.materials = materials;
		shapeDef.materialCount = 2;
		shapeDef.enableHitEvents = true;
		b3CreateMeshShape( bodyId, &shapeDef, g_mesh, (b3Vec3){ 2.0f, 1.0f, 2.0f } );

		b3ShapeDef sensorDef = b3DefaultShapeDef();
		sensorDef.isSensor = true;
		b3BoxHull sensorBox = b3MakeOffsetBoxHull( 2.0f, 2.0f, 2.0f, (b3Vec3){ 4.0f, 2.0f, 4.0f } );
		b3CreateHullShape( bodyId, &sensorDef, &sensorBox.base );
	}

	// 2: static height field
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ -20.0f, 0.0f, 0.0f };
		b3BodyId bodyId = addBody( &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial = materialB;
		b3CreateHeightFieldShape( bodyId, &shapeDef, g_heightField );
	}

	// 3..11: fast bullets onto each static shape
	for ( int i = 0; i < 9; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		int target = i % 3;
		float x = target == 0 ? 0.0f : ( target == 1 ? 24.0f : -18.0f );
		float px = x + vrandf( -1.0f, 1.0f );
		float py = 3.0f + vrandf( 0.0f, 1.0f );
		float pz = vrandf( -1.0f, 1.0f ) + ( target == 0 ? 0.0f : 2.0f );
		bodyDef.position = (b3Vec3){ px, py, pz };
		float vx = vrandf( -2.0f, 2.0f );
		float vz = vrandf( -2.0f, 2.0f );
		bodyDef.linearVelocity = (b3Vec3){ vx, -150.0f, vz };
		bodyDef.isBullet = ( i & 1 ) == 0;
		b3BodyId bodyId = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableHitEvents = true;
		shapeDef.enableSensorEvents = true;
		int kind = i / 3;
		if ( kind == 0 )
		{
			b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.1f };
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		}
		else if ( kind == 1 )
		{
			b3Capsule capsule = { { 0.0f, -0.1f, 0.0f }, { 0.0f, 0.1f, 0.0f }, 0.08f };
			b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
		}
		else
		{
			b3BoxHull bulletBox = b3MakeBoxHull( 0.1f, 0.1f, 0.1f );
			b3CreateHullShape( bodyId, &shapeDef, &bulletBox.base );
		}
	}

	// 12, 13: jointed pair hanging from the compound body
	b3BodyId linkA;
	b3BodyId linkB;
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Vec3){ 0.0f, 3.0f, -3.0f };
		linkA = addBody( &bodyDef );
		bodyDef.position = (b3Vec3){ 1.0f, 3.0f, -3.0f };
		linkB = addBody( &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull linkBox = b3MakeBoxHull( 0.4f, 0.1f, 0.1f );
		b3CreateHullShape( linkA, &shapeDef, &linkBox.base );
		b3CreateHullShape( linkB, &shapeDef, &linkBox.base );

		b3RevoluteJointDef revoluteDef = b3DefaultRevoluteJointDef();
		revoluteDef.base.bodyIdA = linkA;
		revoluteDef.base.bodyIdB = linkB;
		revoluteDef.base.localFrameA.p = (b3Vec3){ 0.5f, 0.0f, 0.0f };
		revoluteDef.base.localFrameB.p = (b3Vec3){ -0.5f, 0.0f, 0.0f };
		b3CreateRevoluteJoint( g_worldId, &revoluteDef );

		b3SphericalJointDef sphericalDef = b3DefaultSphericalJointDef();
		sphericalDef.base.bodyIdA = g_bodies[0];
		sphericalDef.base.bodyIdB = linkA;
		sphericalDef.base.localFrameA.p = (b3Vec3){ -0.5f, 3.0f, -3.0f };
		sphericalDef.base.localFrameB.p = (b3Vec3){ -0.5f, 0.0f, 0.0f };
		b3CreateSphericalJoint( g_worldId, &sphericalDef );
	}

	dumpWorld( "extras" );

	b3JointId joints[4];
	int jointCount = b3Body_GetJoints( linkA, joints, 4 );
	vtag( "joints" );
	vi32( jointCount );
	for ( int i = 0; i < jointCount; ++i )
	{
		vi32( joints[i].index1 );
		vu32( joints[i].generation );
	}
	vend();

	// Queries against the compound
	b3QueryFilter filter = b3DefaultQueryFilter();
	b3ShapeId compoundShapes[SHAPE_CAPACITY];
	b3Body_GetShapes( g_bodies[0], compoundShapes, SHAPE_CAPACITY );
	for ( int k = 0; k < 8; ++k )
	{
		b3Vec3 aims[4] = { { -2.5f, 0.0f, 0.25f }, { 0.0f, 0.0f, 0.0f }, { 3.0f, 0.0f, -1.0f }, { 0.0f, 0.0f, 3.0f } };
		b3Vec3 origin = b3Add( aims[k % 4], rvec( -0.5f, 0.5f ) );
		origin.y = 3.0f;
		float tx = vrandf( -1.0f, 1.0f );
		float tz = vrandf( -1.0f, 1.0f );
		b3Vec3 translation = { tx, -6.0f, tz };
		b3BodyCastResult ray = b3Body_CastRay( g_bodies[0], origin, translation, filter, 1.0f, b3Body_GetTransform( g_bodies[0] ) );
		vtag( "cray" );
		pshapeId( ray.shapeId );
		pvec( ray.point );
		vf32( ray.fraction );
		vi32( ray.triangleIndex );
		vu64( ray.userMaterialId );
		vbool( ray.hit );
		pcast( b3Shape_RayCast( compoundShapes[0], origin, translation ) );
		vend();

		b3Vec3 points[1] = { origin };
		b3ShapeProxy proxy = { points, 1, 0.3f };
		b3BodyCastResult cast = b3Body_CastShape( g_bodies[0], origin, &proxy, translation, filter, 1.0f, false,
												  b3Body_GetTransform( g_bodies[0] ) );
		vtag( "ccast" );
		pshapeId( cast.shapeId );
		pvec( cast.point );
		pvec( cast.normal );
		vf32( cast.fraction );
		vu64( cast.userMaterialId );
		vbool( cast.hit );
		b3Vec3 low = b3MulAdd( origin, 0.5f, translation );
		b3Vec3 lowPoints[1] = { low };
		b3ShapeProxy lowProxy = { lowPoints, 1, 0.5f };
		vbool( b3Body_OverlapShape( g_bodies[0], b3Vec3_zero, &lowProxy, filter, b3Body_GetTransform( g_bodies[0] ) ) );
		vend();
	}

	stepExtras( 12 );

	// Joints across type changes, disable and destroy
	b3Body_SetType( linkA, b3_staticBody );
	dumpWorld( "jstatic" );
	b3Body_SetType( linkA, b3_kinematicBody );
	b3Body_SetType( linkA, b3_dynamicBody );
	dumpWorld( "jdynamic" );
	b3Body_Disable( linkB );
	dumpWorld( "jdisable" );
	stepExtras( 5 );
	b3Body_Enable( linkB );
	b3Body_SetAwake( linkB, false );
	dumpWorld( "jenable" );
	stepExtras( 5 );

	// Mesh swap on the mesh body
	b3ShapeId meshShapes[SHAPE_CAPACITY];
	int meshShapeCount = b3Body_GetShapes( g_bodies[1], meshShapes, SHAPE_CAPACITY );
	for ( int i = 0; i < meshShapeCount; ++i )
	{
		if ( b3Shape_GetType( meshShapes[i] ) == b3_meshShape )
		{
			b3Shape_SetMesh( meshShapes[i], g_mesh, (b3Vec3){ -1.5f, 1.0f, 1.5f } );
		}
	}

	b3DestroyBody( linkA );
	dumpWorld( "jdestroy" );
	stepExtras( 10 );
}

int main( void )
{
	phaseCreate();
	phaseQueries();
	phaseMutate();
	phaseQueries();
	phaseStep( 10 );
	phaseStep( 20 );
	phaseQueries();
	phaseStep( 60 );
	phaseExtras();
	return 0;
}
