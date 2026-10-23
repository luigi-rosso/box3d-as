// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Trajectory parity scenes, public Box3D API only. The AnimaScript side mirrors these line for
// line.

#pragma once

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <stdbool.h>
#include <stdint.h>

enum
{
	SCENE_MOVER_PLANE_CAPACITY = 8,
	SCENE_SUBSTEP_COUNT = 4,
	SCENE_QUERY_MOVER_COUNT = 8,
};

// Query kinds timed and hashed by the village and queries scenes
enum
{
	SCENE_QUERY_RAY = 0,
	SCENE_QUERY_RAY_ALL = 1,
	SCENE_QUERY_SHAPE_CAST = 2,
	SCENE_QUERY_OVERLAP = 3,
	SCENE_QUERY_MOVER = 4,
	SCENE_QUERY_KIND_COUNT = 5,
};

// Port of CharacterMover in samples/mover.h, minus the camera and drawing
typedef struct SceneMover
{
	b3WorldTransform transform;
	b3Vec3 velocity;
	b3Capsule capsule;
	b3CollisionPlane planes[SCENE_MOVER_PLANE_CAPACITY];
	b3Pos planePoints[SCENE_MOVER_PLANE_CAPACITY];
	b3ShapeId planeShapes[SCENE_MOVER_PLANE_CAPACITY];
	int planeCount;
	int totalIterations;
	float pogoVelocity;
	bool onGround;
	bool sprint;

	// Recorded per step, not used by the mover itself
	bool rayHit;
	float rayFraction;
	float castFraction;
	int moveIterations;
} SceneMover;

typedef struct Scene
{
	const char* name;
	b3WorldId worldId;

	// Every body the scene created, in creation order. Destroyed bodies stay in the list.
	b3BodyId* bodies;
	int bodyCount;
	int bodyCapacity;

	uint32_t seed;

	b3MeshData* mesh;
	b3HeightFieldData* heightField;
	b3BodyId bulletId;

	bool hasMover;
	SceneMover mover;

	// Village and queries scenes
	b3CompoundData* compound;
	b3Quat compoundRotation;
	float halfSpan;
	int moverCount;
	SceneMover movers[SCENE_QUERY_MOVER_COUNT];

	// Milliseconds spent in each query kind, summed over steps, and a hash and hit count of the
	// results
	double queryMs[SCENE_QUERY_KIND_COUNT];
	uint32_t queryHash[SCENE_QUERY_KIND_COUNT];
	int queryHits[SCENE_QUERY_KIND_COUNT];
	bool queryNested;
} Scene;

int Scene_GetCount( void );
const char* Scene_GetName( int index );

// Returns false for an unknown scene name
bool Scene_Create( Scene* scene, const char* name, int workerCount );

// Scripted input for this step, then b3World_Step at 60 Hz with 4 sub-steps
void Scene_Step( Scene* scene, int stepIndex );

void Scene_Destroy( Scene* scene );
