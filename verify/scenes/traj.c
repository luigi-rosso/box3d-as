// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Trajectory recorder: traj <scene> <steps> <workers> <out.bin>
//
// File format, little endian, every field 4 bytes (floats as raw IEEE 754 bit patterns):
//
// header, 16 words
//   0 magic 'B3TR' (0x52543342)   1 version (1)        2..9 scene name, 32 bytes, zero padded
//   10 step count                 11 worker count       12 time step (f32)   13 sub-step count
//   14 counter count C            15 mover word count M (0 when the scene has no mover)
//
// per step
//   step index, body count N
//   C counters (i32, see s_counterNames in dump.mjs; counters from index 51 on depend on the build
//   and workers) N bodies in creation order, 17 words each:
//     body index, flags (bit 0 valid, bit 1 awake, bits 4..7 body type), id index1, id generation,
//     position xyz, rotation xyzw, linear velocity xyz, angular velocity xyz (f32)
//   M mover words after the bodies (see WriteMover)
//
// Destroyed bodies keep their slot with flags 0 and zero data.

#include "scenes.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
	TRAJ_HEADER_WORDS = 16,
	TRAJ_BODY_WORDS = 17,
	TRAJ_COUNTER_COUNT = 56,
	TRAJ_PLANE_WORDS = 12,
	TRAJ_MOVER_WORDS = 16 + TRAJ_PLANE_WORDS * SCENE_MOVER_PLANE_CAPACITY,
};

typedef struct Writer
{
	FILE* file;
	uint32_t* words;
	int count;
	int capacity;
} Writer;

static void PushWord( Writer* w, uint32_t value )
{
	if ( w->count == w->capacity )
	{
		w->capacity = w->capacity < 1024 ? 1024 : 2 * w->capacity;
		w->words = realloc( w->words, w->capacity * sizeof( uint32_t ) );
	}
	w->words[w->count] = value;
	w->count += 1;
}

static void PushFloat( Writer* w, float value )
{
	uint32_t bits;
	memcpy( &bits, &value, sizeof( bits ) );
	PushWord( w, bits );
}

static void PushInt( Writer* w, int value )
{
	PushWord( w, (uint32_t)value );
}

static void Flush( Writer* w )
{
	fwrite( w->words, sizeof( uint32_t ), w->count, w->file );
	w->count = 0;
}

static void WriteCounters( Writer* w, b3WorldId worldId )
{
	b3Counters c = b3World_GetCounters( worldId );
	b3ContactEvents contactEvents = b3World_GetContactEvents( worldId );
	b3BodyEvents bodyEvents = b3World_GetBodyEvents( worldId );
	int start = w->count;

	PushInt( w, c.bodyCount );
	PushInt( w, c.shapeCount );
	PushInt( w, c.contactCount );
	PushInt( w, c.jointCount );
	PushInt( w, c.islandCount );
	PushInt( w, b3World_GetAwakeBodyCount( worldId ) );
	PushInt( w, c.awakeContactCount );
	PushInt( w, c.recycledContactCount );
	PushInt( w, c.staticTreeHeight );
	PushInt( w, c.treeHeight );
	PushInt( w, c.satCallCount );
	PushInt( w, c.satCacheHitCount );
	PushInt( w, c.distanceIterations );
	PushInt( w, c.pushBackIterations );
	PushInt( w, c.rootIterations );
	PushInt( w, contactEvents.beginCount );
	PushInt( w, contactEvents.endCount );
	PushInt( w, contactEvents.hitCount );
	PushInt( w, bodyEvents.moveCount );
	for ( int i = 0; i < 24; ++i )
	{
		PushInt( w, c.colorCounts[i] );
	}
	for ( int i = 0; i < B3_CONTACT_MANIFOLD_COUNT_BUCKETS; ++i )
	{
		PushInt( w, c.manifoldCounts[i] );
	}

	// Build and worker dependent
	PushInt( w, c.taskCount );
	PushInt( w, c.stackUsed );
	PushInt( w, c.arenaCapacity );
	PushWord( w, (uint32_t)( (uint64_t)c.byteCount & 0xFFFFFFFFu ) );
	PushWord( w, (uint32_t)( (uint64_t)c.byteCount >> 32 ) );

	if ( w->count - start != TRAJ_COUNTER_COUNT )
	{
		fprintf( stderr, "counter layout mismatch %d\n", w->count - start );
		exit( 1 );
	}
}

static void WriteBody( Writer* w, int index, b3BodyId bodyId )
{
	PushInt( w, index );

	if ( b3Body_IsValid( bodyId ) == false )
	{
		for ( int i = 1; i < TRAJ_BODY_WORDS; ++i )
		{
			PushWord( w, 0 );
		}
		return;
	}

	uint32_t flags = 1;
	if ( b3Body_IsAwake( bodyId ) )
	{
		flags |= 2;
	}
	flags |= (uint32_t)b3Body_GetType( bodyId ) << 4;
	PushWord( w, flags );
	PushInt( w, bodyId.index1 );
	PushWord( w, bodyId.generation );

	b3Pos p = b3Body_GetPosition( bodyId );
	b3Quat q = b3Body_GetRotation( bodyId );
	b3Vec3 v = b3Body_GetLinearVelocity( bodyId );
	b3Vec3 omega = b3Body_GetAngularVelocity( bodyId );

	PushFloat( w, p.x );
	PushFloat( w, p.y );
	PushFloat( w, p.z );
	PushFloat( w, q.v.x );
	PushFloat( w, q.v.y );
	PushFloat( w, q.v.z );
	PushFloat( w, q.s );
	PushFloat( w, v.x );
	PushFloat( w, v.y );
	PushFloat( w, v.z );
	PushFloat( w, omega.x );
	PushFloat( w, omega.y );
	PushFloat( w, omega.z );
}

// position xyz, velocity xyz, pogo velocity, on ground, sprint, plane count, total solver
// iterations, move loop iterations, ray hit, ray fraction, last cast fraction, reserved, then per
// plane slot: normal xyz, offset, push limit, push, clip velocity, world point xyz, shape index1,
// shape generation
static void WriteMover( Writer* w, const SceneMover* m )
{
	int start = w->count;
	PushFloat( w, m->transform.p.x );
	PushFloat( w, m->transform.p.y );
	PushFloat( w, m->transform.p.z );
	PushFloat( w, m->velocity.x );
	PushFloat( w, m->velocity.y );
	PushFloat( w, m->velocity.z );
	PushFloat( w, m->pogoVelocity );
	PushWord( w, m->onGround ? 1 : 0 );
	PushWord( w, m->sprint ? 1 : 0 );
	PushInt( w, m->planeCount );
	PushInt( w, m->totalIterations );
	PushInt( w, m->moveIterations );
	PushWord( w, m->rayHit ? 1 : 0 );
	PushFloat( w, m->rayFraction );
	PushFloat( w, m->castFraction );
	PushWord( w, 0 );

	for ( int i = 0; i < SCENE_MOVER_PLANE_CAPACITY; ++i )
	{
		if ( i >= m->planeCount )
		{
			for ( int k = 0; k < TRAJ_PLANE_WORDS; ++k )
			{
				PushWord( w, 0 );
			}
			continue;
		}

		const b3CollisionPlane* plane = m->planes + i;
		PushFloat( w, plane->plane.normal.x );
		PushFloat( w, plane->plane.normal.y );
		PushFloat( w, plane->plane.normal.z );
		PushFloat( w, plane->plane.offset );
		PushFloat( w, plane->pushLimit );
		PushFloat( w, plane->push );
		PushWord( w, plane->clipVelocity ? 1 : 0 );
		PushFloat( w, m->planePoints[i].x );
		PushFloat( w, m->planePoints[i].y );
		PushFloat( w, m->planePoints[i].z );
		PushInt( w, m->planeShapes[i].index1 );
		PushWord( w, m->planeShapes[i].generation );
	}

	if ( w->count - start != TRAJ_MOVER_WORDS )
	{
		fprintf( stderr, "mover layout mismatch %d\n", w->count - start );
		exit( 1 );
	}
}

int main( int argc, char** argv )
{
	if ( argc != 5 )
	{
		fprintf( stderr, "usage: traj <scene> <steps> <workers> <out.bin>\nscenes:" );
		for ( int i = 0; i < Scene_GetCount(); ++i )
		{
			fprintf( stderr, " %s", Scene_GetName( i ) );
		}
		fprintf( stderr, "\n" );
		return 1;
	}

	const char* sceneName = argv[1];
	int stepCount = atoi( argv[2] );
	int workerCount = atoi( argv[3] );
	const char* path = argv[4];

	Scene scene;
	if ( Scene_Create( &scene, sceneName, workerCount ) == false )
	{
		fprintf( stderr, "unknown scene %s\n", sceneName );
		return 1;
	}

	b3Counters initial = b3World_GetCounters( scene.worldId );
	printf( "%s bodies %d shapes %d joints %d\n", sceneName, initial.bodyCount, initial.shapeCount,
			initial.jointCount );

	Writer w = { 0 };
	w.file = fopen( path, "wb" );
	if ( w.file == NULL )
	{
		fprintf( stderr, "cannot open %s\n", path );
		return 1;
	}

	char name[32] = { 0 };
	strncpy( name, sceneName, sizeof( name ) - 1 );

	PushWord( &w, 0x52543342u );
	PushWord( &w, 1 );
	for ( int i = 0; i < 8; ++i )
	{
		uint32_t word;
		memcpy( &word, name + 4 * i, sizeof( word ) );
		PushWord( &w, word );
	}
	PushInt( &w, stepCount );
	PushInt( &w, workerCount );
	PushFloat( &w, 1.0f / 60.0f );
	PushInt( &w, SCENE_SUBSTEP_COUNT );
	PushInt( &w, TRAJ_COUNTER_COUNT );
	PushInt( &w, scene.hasMover ? TRAJ_MOVER_WORDS : 0 );
	Flush( &w );

	for ( int stepIndex = 0; stepIndex < stepCount; ++stepIndex )
	{
		Scene_Step( &scene, stepIndex );

		PushInt( &w, stepIndex );
		PushInt( &w, scene.bodyCount );
		WriteCounters( &w, scene.worldId );
		for ( int i = 0; i < scene.bodyCount; ++i )
		{
			WriteBody( &w, i, scene.bodies[i] );
		}
		if ( scene.hasMover )
		{
			WriteMover( &w, &scene.mover );
		}
		Flush( &w );
	}

	fclose( w.file );
	free( w.words );

	printf( "%s steps %d workers %d bodies %d\n", sceneName, stepCount, workerCount,
			scene.bodyCount );
	Scene_Destroy( &scene );
	return 0;
}
