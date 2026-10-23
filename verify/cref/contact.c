// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Contacts, sensors and the broad-phase pair update through the public API. After every step the
// driver prints the pair keys of the last pair update, every contact slot with its manifolds and
// cache, the body contact lists, the contact, hit and sensor events, b3Contact_GetData for the
// begin events and the sensor overlap arrays. verify/as/contact.as mirrors it line for line.

#include "common.h"

#include "body.h"
#include "broad_phase.h"
#include "contact.h"
#include "physics_world.h"
#include "sensor.h"
#include "shape.h"
#include "solver.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#define MAX_BODIES 128

typedef struct DriverScene
{
	b3WorldId worldId;
	b3BodyId bodies[MAX_BODIES];
	b3ShapeId shapes[MAX_BODIES];
	int bodyCount;
	b3ShapeId sensorShapeId;
	b3HeightFieldData* heightField;
	b3MeshData* mesh;
	b3CompoundData* compound;
	int moverStart;
	b3Vec3 base[MAX_BODIES];
	b3Vec3 velocity[MAX_BODIES];
	b3Vec3 axis[MAX_BODIES];
	float rate[MAX_BODIES];
} DriverScene;

static void vVec3( b3Vec3 v )
{
	vf32( v.x );
	vf32( v.y );
	vf32( v.z );
}

static void vShapeId( b3ShapeId id )
{
	vi32( id.index1 );
	vu32( id.world0 );
	vu32( id.generation );
}

static void vContactId( b3ContactId id )
{
	vi32( id.index1 );
	vu32( id.world0 );
	vi32( id.padding );
	vu32( id.generation );
}

static b3BodyId AddBody( DriverScene* scene, const b3BodyDef* def )
{
	b3BodyId id = b3CreateBody( scene->worldId, def );
	scene->bodies[scene->bodyCount] = id;
	scene->shapes[scene->bodyCount] = b3_nullShapeId;
	scene->bodyCount += 1;
	return id;
}

// Shape kind 0 box, 1 sphere, 2 capsule
static b3ShapeId AddShape( DriverScene* scene, b3BodyId bodyId, const b3ShapeDef* shapeDef, int kind )
{
	b3ShapeId shapeId;
	if ( kind == 0 )
	{
		b3BoxHull box = b3MakeBoxHull( 0.4f, 0.4f, 0.4f );
		shapeId = b3CreateHullShape( bodyId, shapeDef, &box.base );
	}
	else if ( kind == 1 )
	{
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.4f };
		shapeId = b3CreateSphereShape( bodyId, shapeDef, &sphere );
	}
	else
	{
		b3Capsule capsule = { { 0.0f, -0.3f, 0.0f }, { 0.0f, 0.3f, 0.0f }, 0.3f };
		shapeId = b3CreateCapsuleShape( bodyId, shapeDef, &capsule );
	}
	scene->shapes[scene->bodyCount - 1] = shapeId;
	return shapeId;
}

// Set while creating scripted movers.
static bool s_customFiltering;
static bool s_preSolve;

static bool CustomFilter( b3ShapeId shapeIdA, b3ShapeId shapeIdB, void* context )
{
	(void)context;
	return ( shapeIdA.index1 + shapeIdB.index1 ) % 3 != 0;
}

static bool PreSolve( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3Pos point, b3Vec3 normal, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;
	(void)context;
	return point.y > -0.25f || normal.y < -0.5f;
}

static b3BodyId AddDynamic( DriverScene* scene, float x, float y, float z, int kind, bool sensorEvents, bool hitEvents )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	float dx = vrandf( -0.1f, 0.1f );
	float dz = vrandf( -0.1f, 0.1f );
	float angle = vrandf( -3.0f, 3.0f );
	bodyDef.position = (b3Vec3){ x + dx, y, z + dz };
	bodyDef.rotation = b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 1.0f, 0.0f }, angle );
	b3BodyId bodyId = AddBody( scene, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.6f;
	shapeDef.baseMaterial.rollingResistance = kind == 1 ? 0.1f : 0.0f;
	shapeDef.enableContactEvents = true;
	shapeDef.enableHitEvents = hitEvents;
	shapeDef.enableSensorEvents = sensorEvents;
	shapeDef.enableCustomFiltering = s_customFiltering;
	shapeDef.enablePreSolveEvents = s_preSolve;
	AddShape( scene, bodyId, &shapeDef, kind );
	return bodyId;
}

static void AddGround( DriverScene* scene )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = (b3Vec3){ 0.0f, -1.0f, 0.0f };
	b3BodyId groundId = AddBody( scene, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enableContactEvents = true;
	b3BoxHull box = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	scene->shapes[scene->bodyCount - 1] = b3CreateHullShape( groundId, &shapeDef, &box.base );
}

static b3WorldId CreateDriverWorld( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	return b3CreateWorld( &worldDef );
}

// Every pair of convex types, static and kinematic cross pairs, body destruction, a filter change
// and a body flung away so its contacts end through the broad-phase.
static void CreateMixed( DriverScene* scene )
{
	vseed( 101 );
	scene->worldId = CreateDriverWorld();
	AddGround( scene );

	{
		b3BodyId groundId = scene->bodies[0];
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3Sphere sphere = { { 5.0f, 1.5f, 0.0f }, 1.0f };
		b3CreateSphereShape( groundId, &shapeDef, &sphere );
		b3Capsule capsule = { { -5.0f, 1.5f, -2.0f }, { -5.0f, 1.5f, 2.0f }, 0.5f };
		b3CreateCapsuleShape( groundId, &shapeDef, &capsule );
	}

	for ( int i = 0; i < 4; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			for ( int k = 0; k < 4; ++k )
			{
				float x = -3.0f + 2.0f * (float)i;
				float y = 1.0f + 1.2f * (float)j;
				float z = -3.0f + 2.0f * (float)k;
				int kind = ( i + j + k ) % 3;
				AddDynamic( scene, x, y, z, kind, false, ( i + k ) % 4 == 0 );
			}
		}
	}

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_kinematicBody;
		bodyDef.position = (b3Vec3){ -8.0f, 0.6f, 0.0f };
		bodyDef.linearVelocity = (b3Vec3){ 2.0f, 0.0f, 0.0f };
		b3BodyId bodyId = AddBody( scene, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 2.0f );
		scene->shapes[scene->bodyCount - 1] = b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}
}

static void StepMixed( DriverScene* scene, int stepIndex )
{
	if ( stepIndex == 50 )
	{
		b3DestroyBody( scene->bodies[6] );
		scene->bodies[6] = b3_nullBodyId;
		b3DestroyBody( scene->bodies[18] );
		scene->bodies[18] = b3_nullBodyId;
	}
	else if ( stepIndex == 70 )
	{
		b3Filter filter = b3DefaultFilter();
		filter.maskBits = 0;
		b3Shape_SetFilter( scene->shapes[21], filter, true );
	}
	else if ( stepIndex == 90 )
	{
		b3Body_SetLinearVelocity( scene->bodies[31], (b3Vec3){ 0.0f, 12.0f, 0.0f } );
	}
}

// A static and a kinematic sensor, visitors with and without sensor events, a sensor shape
// destroyed while it holds overlaps and a visitor destroyed inside a sensor.
static void CreateSensor( DriverScene* scene )
{
	vseed( 202 );
	scene->worldId = CreateDriverWorld();
	AddGround( scene );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ 0.0f, 1.5f, 0.0f };
		b3BodyId bodyId = AddBody( scene, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;
		b3BoxHull box = b3MakeBoxHull( 2.0f, 1.0f, 2.0f );
		scene->shapes[scene->bodyCount - 1] = b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_kinematicBody;
		bodyDef.position = (b3Vec3){ -6.0f, 1.0f, 0.0f };
		bodyDef.linearVelocity = (b3Vec3){ 1.5f, 0.0f, 0.0f };
		b3BodyId bodyId = AddBody( scene, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 1.0f };
		scene->sensorShapeId = b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		scene->shapes[scene->bodyCount - 1] = scene->sensorShapeId;
	}

	for ( int i = 0; i < 3; ++i )
	{
		for ( int k = 0; k < 3; ++k )
		{
			for ( int j = 0; j < 2; ++j )
			{
				float x = -1.5f + 1.5f * (float)i;
				float y = 4.0f + 1.5f * (float)j;
				float z = -1.5f + 1.5f * (float)k;
				int kind = ( i + j + k ) % 3;
				AddDynamic( scene, x, y, z, kind, i + k != 2, false );
			}
		}
	}

	for ( int i = 0; i < 4; ++i )
	{
		float x = -4.5f + 0.9f * (float)i;
		AddDynamic( scene, x, 0.4f, 1.2f, i % 3, true, false );
	}
}

static void StepSensor( DriverScene* scene, int stepIndex )
{
	if ( stepIndex == 60 )
	{
		b3DestroyShape( scene->sensorShapeId, false );
		scene->shapes[2] = b3_nullShapeId;
	}
	else if ( stepIndex == 75 )
	{
		b3DestroyBody( scene->bodies[5] );
		scene->bodies[5] = b3_nullBodyId;
	}
}

// Capsule, hull, sphere and mesh children.
static void MakeCompound( DriverScene* scene )
{
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = 0.5f;

	b3CompoundCapsuleDef capsules[2];
	capsules[0].capsule = (b3Capsule){ { -3.0f, 0.5f, -2.0f }, { -3.0f, 0.5f, 2.0f }, 0.5f };
	capsules[0].material = material;
	capsules[1].capsule = (b3Capsule){ { -2.0f, 0.5f, 3.0f }, { 2.0f, 0.5f, 3.0f }, 0.4f };
	capsules[1].material = material;
	capsules[1].material.friction = 0.8f;

	b3BoxHull box = b3MakeBoxHull( 1.0f, 0.5f, 1.0f );
	b3CompoundHullDef hulls[2];
	hulls[0].hull = &box.base;
	hulls[0].transform = (b3Transform){ { 0.0f, 0.5f, 0.0f }, b3Quat_identity };
	hulls[0].material = material;
	hulls[1].hull = &box.base;
	hulls[1].transform = (b3Transform){ { 3.0f, 0.5f, -1.0f }, b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 1.0f, 0.0f }, 0.5f ) };
	hulls[1].material = material;
	hulls[1].material.restitution = 0.3f;

	b3CompoundSphereDef spheres[2];
	spheres[0].sphere = (b3Sphere){ { 3.0f, 0.6f, 3.0f }, 0.6f };
	spheres[0].material = material;
	spheres[1].sphere = (b3Sphere){ { -3.0f, 0.6f, -3.5f }, 0.6f };
	spheres[1].material = material;

	scene->mesh = b3CreateWaveMesh( 10, 10, 1.0f, 0.2f, 0.3f, 0.2f );
	b3CompoundMeshDef meshes[1];
	meshes[0].meshData = scene->mesh;
	meshes[0].transform = (b3Transform){ { -5.0f, -0.5f, -5.0f }, b3Quat_identity };
	meshes[0].scale = (b3Vec3){ 1.0f, 1.0f, 1.0f };
	meshes[0].materials = &material;
	meshes[0].materialCount = 1;

	b3CompoundDef compoundDef = { 0 };
	compoundDef.capsules = capsules;
	compoundDef.capsuleCount = 2;
	compoundDef.hulls = hulls;
	compoundDef.hullCount = 2;
	compoundDef.meshes = meshes;
	compoundDef.meshCount = 1;
	compoundDef.spheres = spheres;
	compoundDef.sphereCount = 2;
	scene->compound = b3CreateCompound( &compoundDef );
}

// A static compound with capsule, hull, sphere and mesh children under a pile of convex bodies.
static void CreateCompound( DriverScene* scene )
{
	vseed( 303 );
	scene->worldId = CreateDriverWorld();
	MakeCompound( scene );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId groundId = AddBody( scene, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		scene->shapes[scene->bodyCount - 1] = b3CreateBakedCompoundShape( groundId, &shapeDef, scene->compound );
	}

	for ( int i = 0; i < 4; ++i )
	{
		for ( int k = 0; k < 4; ++k )
		{
			for ( int j = 0; j < 2; ++j )
			{
				float x = -3.0f + 2.0f * (float)i;
				float y = 2.5f + 1.2f * (float)j;
				float z = -3.0f + 2.0f * (float)k;
				int kind = ( i + j + 2 * k ) % 3;
				AddDynamic( scene, x, y, z, kind, false, i == k );
			}
		}
	}
}

static void StepNone( DriverScene* scene, int stepIndex )
{
	(void)scene;
	(void)stepIndex;
}

// A wave height field under a pile of convex bodies.
static void CreateHeightField( DriverScene* scene )
{
	vseed( 404 );
	scene->worldId = CreateDriverWorld();

	int fieldCount = 13;
	b3Vec3 fieldScale = { 1.0f, 0.6f, 1.0f };
	scene->heightField = b3CreateWave( fieldCount, fieldCount, fieldScale, 0.08f, 0.06f, false );

	{
		float extent = fieldScale.x * (float)( fieldCount - 1 );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position.x = -0.5f * extent;
		bodyDef.position.z = -0.5f * extent;
		b3BodyId groundId = AddBody( scene, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableHitEvents = true;
		scene->shapes[scene->bodyCount - 1] = b3CreateHeightFieldShape( groundId, &shapeDef, scene->heightField );
	}

	for ( int i = 0; i < 4; ++i )
	{
		for ( int k = 0; k < 4; ++k )
		{
			float x = -3.0f + 2.0f * (float)i;
			float z = -3.0f + 2.0f * (float)k;
			int kind = ( i + k ) % 3;
			AddDynamic( scene, x, 2.0f, z, kind, false, false );
		}
	}
}

// Every dynamic body follows a scripted path through b3Body_SetTransform, so contacts begin, persist,
// recycle and end against static convex shapes, a compound, a height field and two sensors no
// matter what the solver does. Built with CONTACT_COLLIDE_ONLY the solver is a no-op.
static void CreateScripted( DriverScene* scene )
{
	vseed( 505 );
	scene->worldId = CreateDriverWorld();
	AddGround( scene );

	{
		b3BodyId groundId = scene->bodies[0];
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		b3Sphere sphere = { { 5.0f, 1.5f, 0.0f }, 1.0f };
		b3CreateSphereShape( groundId, &shapeDef, &sphere );
		b3Capsule capsule = { { -5.0f, 1.5f, -2.0f }, { -5.0f, 1.5f, 2.0f }, 0.5f };
		b3CreateCapsuleShape( groundId, &shapeDef, &capsule );
	}

	MakeCompound( scene );
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ 0.0f, 0.0f, 8.0f };
		b3BodyId bodyId = AddBody( scene, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		scene->shapes[scene->bodyCount - 1] = b3CreateBakedCompoundShape( bodyId, &shapeDef, scene->compound );
	}

	{
		b3Vec3 fieldScale = { 1.0f, 0.6f, 1.0f };
		scene->heightField = b3CreateWave( 13, 13, fieldScale, 0.08f, 0.06f, false );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ -18.0f, -0.5f, -6.0f };
		b3BodyId bodyId = AddBody( scene, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		scene->shapes[scene->bodyCount - 1] = b3CreateHeightFieldShape( bodyId, &shapeDef, scene->heightField );
	}

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ 0.0f, 1.5f, -6.0f };
		b3BodyId bodyId = AddBody( scene, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;
		b3BoxHull box = b3MakeBoxHull( 2.0f, 1.0f, 2.0f );
		scene->shapes[scene->bodyCount - 1] = b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_kinematicBody;
		bodyDef.position = (b3Vec3){ -8.0f, 1.0f, 0.0f };
		b3BodyId bodyId = AddBody( scene, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 1.0f };
		scene->sensorShapeId = b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		scene->shapes[scene->bodyCount - 1] = scene->sensorShapeId;
	}

	scene->moverStart = scene->bodyCount;
	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			for ( int k = 0; k < 3; ++k )
			{
				float x = -4.0f + 4.0f * (float)i;
				float y = 0.5f + 1.5f * (float)j;
				float z = -4.0f + 4.0f * (float)k;
				int kind = ( i + j + k ) % 3;
				int moverIndex = 9 * i + 3 * j + k;
				s_customFiltering = moverIndex % 4 == 1;
				s_preSolve = moverIndex % 4 == 2;
				b3BodyId bodyId = AddDynamic( scene, x, y, z, kind, true, ( i + j ) % 3 == 0 );
				int index = scene->bodyCount - 1;
				scene->base[index] = b3Body_GetPosition( bodyId );
				float vx = vrandf( -2.0f, 2.0f );
				float vy = vrandf( -1.0f, 1.0f );
				float vz = vrandf( -2.0f, 2.0f );
				scene->velocity[index] = (b3Vec3){ vx, vy, vz };
				float ax = vrandf( 0.2f, 1.0f );
				float ay = vrandf( 0.2f, 1.0f );
				float az = vrandf( 0.2f, 1.0f );
				scene->axis[index] = b3Normalize( (b3Vec3){ ax, ay, az } );
				scene->rate[index] = vrandf( -2.0f, 2.0f );
			}
		}
	}

	s_customFiltering = false;
	s_preSolve = false;
	b3World_SetCustomFilterCallback( scene->worldId, CustomFilter, NULL );
	b3World_SetPreSolveCallback( scene->worldId, PreSolve, NULL );
}

static void StepScripted( DriverScene* scene, int stepIndex )
{
	float one = 1.0f;
	float hertz = 60.0f;
	float t = (float)( stepIndex + 1 ) * ( one / hertz );

	if ( B3_IS_NON_NULL( scene->sensorShapeId ) )
	{
		b3Vec3 p = { -8.0f + 4.0f * t, 1.0f, 0.0f };
		b3Body_SetTransform( scene->bodies[4], p, b3Quat_identity );
	}

	for ( int i = scene->moverStart; i < scene->bodyCount; ++i )
	{
		if ( B3_IS_NULL( scene->bodies[i] ) )
		{
			continue;
		}

		b3Vec3 p = b3MulAdd( scene->base[i], t, scene->velocity[i] );
		b3Quat q = b3MakeQuatFromAxisAngle( scene->axis[i], scene->rate[i] * t );
		b3Body_SetTransform( scene->bodies[i], p, q );
	}

	int mover = scene->moverStart;
	if ( stepIndex == 80 )
	{
		b3DestroyBody( scene->bodies[mover + 4] );
		scene->bodies[mover + 4] = b3_nullBodyId;
	}
	else if ( stepIndex == 100 )
	{
		b3Filter filter = b3DefaultFilter();
		filter.maskBits = 0;
		b3Shape_SetFilter( scene->shapes[mover + 10], filter, true );
	}
	else if ( stepIndex == 120 )
	{
		b3DestroyShape( scene->shapes[3], false );
		scene->shapes[3] = b3_nullShapeId;
	}
	else if ( stepIndex == 140 )
	{
		b3Shape_SetFilter( scene->shapes[mover + 10], b3DefaultFilter(), true );
	}
	else if ( stepIndex == 160 )
	{
		b3DestroyShape( scene->sensorShapeId, false );
		scene->sensorShapeId = b3_nullShapeId;
		scene->shapes[4] = b3_nullShapeId;
	}
}


static bool UsesSatCache( b3World* world, b3Contact* contact )
{
	const b3Shape* shapeA = b3Array_Get( world->shapes, contact->shapeIdA );
	const b3Shape* shapeB = b3Array_Get( world->shapes, contact->shapeIdB );
	b3ShapeType typeA = shapeA->type;
	if ( typeA == b3_compoundShape )
	{
		b3ChildShape child = b3GetCompoundChild( shapeA->compound, contact->childIndex );
		typeA = child.type;
	}
	return typeA == b3_hullShape && shapeB->type == b3_hullShape;
}

static void DumpWorld( DriverScene* scene )
{
	b3World* world = b3GetWorldFromId( scene->worldId );
	b3BroadPhase* bp = &world->broadPhase;

	vtag( "bp" );
	for ( int i = 0; i < b3_bodyTypeCount; ++i )
	{
		vi32( bp->trees[i].proxyCount );
		vi32( bp->trees[i].nodeEnd );
		vbool( b3HasTreeMoved( bp->trees + i ) );
	}
	vu32( bp->pairSet.count );
	vend();

	{
		b3TaskContext* taskContext = world->taskContexts.data + 0;
		vtag( "pairs" );
		vi32( taskContext->pairKeys.count );
		for ( int i = 0; i < taskContext->pairKeys.count; ++i )
		{
			vu64( taskContext->pairKeys.data[i] );
		}
		vend();
	}

	vtag( "contacts" );
	vi32( world->contacts.count );
	vi32( b3GetIdCount( &world->contactIdPool ) );
	vend();

	for ( int i = 0; i < world->contacts.count; ++i )
	{
		b3Contact* c = world->contacts.data + i;
		vtag( "c" );
		vi32( i );
		vu32( c->generation );
		vi32( c->contactId );
		if ( c->contactId == B3_NULL_INDEX )
		{
			vend();
			continue;
		}

		vi32( c->setIndex );
		vi32( c->colorIndex );
		vi32( c->localIndex );
		vi32( c->shapeIdA );
		vi32( c->shapeIdB );
		vi32( c->childIndex );
		vi32( c->islandId );
		vi32( c->encodedBodySimA );
		vi32( c->encodedBodySimB );
		vu32( c->flags );
		for ( int e = 0; e < 2; ++e )
		{
			vi32( c->edges[e].bodyId );
			vi32( c->edges[e].prevKey );
			vi32( c->edges[e].nextKey );
		}
		vf32( c->friction );
		vf32( c->restitution );
		vf32( c->rollingResistance );
		vVec3( c->tangentVelocity );
		vi32( c->manifoldCount );
		vend();

		if ( c->flags & b3_simMeshContact )
		{
			vtag( "mesh" );
			vi32( c->meshContact.triangleCache.count );
			vend();
		}
		else if ( UsesSatCache( world, c ) )
		{
			b3SATCache* cache = &c->convexContact.cache.satCache;
			vtag( "sat" );
			vf32( cache->separation );
			vu32( cache->type );
			vu32( cache->indexA );
			vu32( cache->indexB );
			vu32( cache->hit );
			vend();
		}
		else
		{
			b3SimplexCache* cache = &c->convexContact.cache.simplexCache;
			vtag( "simplex" );
			vu32( cache->count );
			for ( int k = 0; k < cache->count; ++k )
			{
				vu32( cache->indexA[k] );
				vu32( cache->indexB[k] );
			}
			vend();
		}

		for ( int m = 0; m < c->manifoldCount; ++m )
		{
			b3Manifold* manifold = c->manifolds + m;
			vtag( "m" );
			vVec3( manifold->normal );
			vi32( manifold->pointCount );
			vf32( manifold->twistImpulse );
			vVec3( manifold->frictionImpulse );
			vVec3( manifold->rollingImpulse );
			vend();
			for ( int p = 0; p < manifold->pointCount; ++p )
			{
				b3ManifoldPoint* mp = manifold->points + p;
				vtag( "p" );
				vVec3( mp->anchorA );
				vVec3( mp->anchorB );
				vf32( mp->separation );
				vf32( mp->normalImpulse );
				vf32( mp->totalNormalImpulse );
				vf32( mp->normalVelocity );
				vu32( mp->featureId );
				vi32( mp->triangleIndex );
				vbool( mp->persisted );
				vend();
			}
		}
	}

	vtag( "bodies" );
	vi32( world->bodies.count );
	vend();
	for ( int i = 0; i < world->bodies.count; ++i )
	{
		b3Body* body = world->bodies.data + i;
		vtag( "b" );
		vi32( i );
		vi32( body->id );
		if ( body->id == B3_NULL_INDEX )
		{
			vend();
			continue;
		}
		vi32( body->setIndex );
		vi32( body->headContactKey );
		vi32( body->contactCount );
		b3BodyId bodyId = { i + 1, world->worldId, body->generation };
		vVec3( b3Body_GetPosition( bodyId ) );
		b3Quat q = b3Body_GetRotation( bodyId );
		vVec3( q.v );
		vf32( q.s );
		vend();
	}

	b3ContactEvents contactEvents = b3World_GetContactEvents( scene->worldId );
	vtag( "begin" );
	vi32( contactEvents.beginCount );
	vend();
	for ( int i = 0; i < contactEvents.beginCount; ++i )
	{
		b3ContactBeginTouchEvent* event = contactEvents.beginEvents + i;
		vtag( "cb" );
		vShapeId( event->shapeIdA );
		vShapeId( event->shapeIdB );
		vContactId( event->contactId );
		bool valid = b3Contact_IsValid( event->contactId );
		vbool( valid );
		if ( valid )
		{
			b3ContactData data = b3Contact_GetData( event->contactId );
			vShapeId( data.shapeIdA );
			vShapeId( data.shapeIdB );
			vi32( data.manifoldCount );
			if ( data.manifoldCount > 0 )
			{
				vi32( data.manifolds[0].pointCount );
				vVec3( data.manifolds[0].normal );
			}
		}
		vend();
	}

	vtag( "end" );
	vi32( contactEvents.endCount );
	vend();
	for ( int i = 0; i < contactEvents.endCount; ++i )
	{
		b3ContactEndTouchEvent* event = contactEvents.endEvents + i;
		vtag( "ce" );
		vShapeId( event->shapeIdA );
		vShapeId( event->shapeIdB );
		vContactId( event->contactId );
		vend();
	}

	vtag( "hit" );
	vi32( contactEvents.hitCount );
	vend();
	for ( int i = 0; i < contactEvents.hitCount; ++i )
	{
		b3ContactHitEvent* event = contactEvents.hitEvents + i;
		vtag( "ch" );
		vShapeId( event->shapeIdA );
		vShapeId( event->shapeIdB );
		vContactId( event->contactId );
		vVec3( event->point );
		vVec3( event->normal );
		vf32( event->approachSpeed );
		vend();
	}

	b3SensorEvents sensorEvents = b3World_GetSensorEvents( scene->worldId );
	vtag( "sensor" );
	vi32( sensorEvents.beginCount );
	vi32( sensorEvents.endCount );
	vend();
	for ( int i = 0; i < sensorEvents.beginCount; ++i )
	{
		vtag( "sb" );
		vShapeId( sensorEvents.beginEvents[i].sensorShapeId );
		vShapeId( sensorEvents.beginEvents[i].visitorShapeId );
		vend();
	}
	for ( int i = 0; i < sensorEvents.endCount; ++i )
	{
		vtag( "se" );
		vShapeId( sensorEvents.endEvents[i].sensorShapeId );
		vShapeId( sensorEvents.endEvents[i].visitorShapeId );
		vend();
	}

	vtag( "sensors" );
	vi32( world->sensors.count );
	vend();
	for ( int i = 0; i < world->sensors.count; ++i )
	{
		b3Sensor* sensor = world->sensors.data + i;
		vtag( "s" );
		vi32( sensor->shapeId );
		vi32( sensor->hits.count );
		vi32( sensor->overlaps1.count );
		vi32( sensor->overlaps2.count );
		for ( int k = 0; k < sensor->overlaps2.count; ++k )
		{
			vi32( sensor->overlaps2.data[k].shapeId );
			vu32( sensor->overlaps2.data[k].generation );
		}
		vend();
	}
}

typedef void SceneCreateFcn( DriverScene* scene );
typedef void SceneStepFcn( DriverScene* scene, int stepIndex );

static void RunScene( const char* name, SceneCreateFcn* create, SceneStepFcn* step, int stepCount )
{
	DriverScene scene = { 0 };
	create( &scene );

	vtag( "scene" );
	vtag( name );
	vi32( scene.bodyCount );
	vend();
	DumpWorld( &scene );

	for ( int i = 0; i < stepCount; ++i )
	{
		step( &scene, i );
		b3World_Step( scene.worldId, 1.0f / 60.0f, 4 );
		vtag( "step" );
		vi32( i );
		vend();
		DumpWorld( &scene );
	}

	b3DestroyWorld( scene.worldId );

	if ( scene.heightField != NULL )
	{
		b3DestroyHeightField( scene.heightField );
	}

	if ( scene.compound != NULL )
	{
		b3DestroyCompound( scene.compound );
	}

	if ( scene.mesh != NULL )
	{
		b3DestroyMesh( scene.mesh );
	}
}

#ifdef CONTACT_COLLIDE_ONLY
// Overrides the archive's solver.c, which defines nothing else.
void b3Solve( b3World* world, b3StepContext* stepContext )
{
	(void)world;
	(void)stepContext;
}
#endif

int main( void )
{
#ifndef CONTACT_COLLIDE_ONLY
	RunScene( "mixed", CreateMixed, StepMixed, 150 );
	RunScene( "sensor", CreateSensor, StepSensor, 120 );
	RunScene( "compound", CreateCompound, StepNone, 120 );
	RunScene( "heightfield", CreateHeightField, StepNone, 120 );
#endif
	RunScene( "scripted", CreateScripted, StepScripted, 200 );
	return 0;
}
