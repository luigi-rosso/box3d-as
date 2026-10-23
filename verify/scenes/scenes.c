// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Trajectory parity scenes. Plain C over the public Box3D API so the AnimaScript side can mirror
// each builder line for line. Every float expression is written as separate statements where C
// leaves the evaluation order open (function arguments, initializer lists), so the mirror has one
// valid order.

#include "scenes.h"

#include <float.h>
#include <stdlib.h>
#include <string.h>

static const char* s_sceneNames[] = {
	"pyramid",	   "large_pyramid", "card_house", "chains",		 "capsule_mesh",
	"heightfield", "bullets",		"mover",	  "many_bodies", "mesh_drop",
	"village",	   "queries",
};

int Scene_GetCount( void )
{
	return (int)( sizeof( s_sceneNames ) / sizeof( s_sceneNames[0] ) );
}

const char* Scene_GetName( int index )
{
	return s_sceneNames[index];
}

// Numerical Recipes LCG
static uint32_t Scene_Random( Scene* scene )
{
	scene->seed = scene->seed * 1664525u + 1013904223u;
	return scene->seed;
}

// Uniform in [lo, hi] from the top 24 bits, exact up to the final multiply add
static float Scene_RandomFloat( Scene* scene, float lo, float hi )
{
	uint32_t bits = Scene_Random( scene ) >> 8;
	float r = (float)bits * ( 1.0f / 16777216.0f );
	float range = hi - lo;
	float scaled = range * r;
	return lo + scaled;
}

static b3BodyId Scene_AddBody( Scene* scene, const b3BodyDef* bodyDef )
{
	b3BodyId bodyId = b3CreateBody( scene->worldId, bodyDef );

	if ( scene->bodyCount == scene->bodyCapacity )
	{
		int capacity = scene->bodyCapacity < 64 ? 64 : 2 * scene->bodyCapacity;
		scene->bodies = realloc( scene->bodies, capacity * sizeof( b3BodyId ) );
		scene->bodyCapacity = capacity;
	}

	scene->bodies[scene->bodyCount] = bodyId;
	scene->bodyCount += 1;
	return bodyId;
}

// samples/sample.cpp Sample::AddGroundBox
static b3BodyId Scene_AddGroundBox( Scene* scene, float extent )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
	b3BodyId groundId = Scene_AddBody( scene, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull hull = b3MakeBoxHull( extent, 1.0f, extent );
	b3CreateHullShape( groundId, &shapeDef, &hull.base );

	return groundId;
}

// shared/benchmarks.c CreateLargePyramid with the base count as a parameter
static void CreatePyramid( Scene* scene, int baseCount )
{
	b3World_EnableSleeping( scene->worldId, false );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = Scene_AddBody( scene, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( 400.0f, 1.0f, 400.0f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 100.0f;

	float h = 0.5f;
	b3BoxHull box = b3MakeBoxHull( h, h, h );

	float shift = 1.0f * h;

	for ( int i = 0; i < baseCount; ++i )
	{
		float y = ( 2.0f * (float)i + 1.0f ) * shift;

		for ( int j = i; j < baseCount; ++j )
		{
			float x = ( (float)i + 1.0f ) * shift + 2.0f * (float)( j - i ) * shift -
					  h * (float)baseCount;

			bodyDef.position = (b3Pos){ x, y, 0.0f };

			b3BodyId bodyId = Scene_AddBody( scene, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
	}
}

// samples/sample_stacking.cpp CardHouse
static void CreateCardHouse( Scene* scene )
{
	Scene_AddGroundBox( scene, 10.0f );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.rollingResistance = 0.05f;
	shapeDef.baseMaterial.friction = 0.7f;

	float cardHeight = 0.2f;
	float cardThickness = 0.001f;
	float cardDepth = 0.1f;

	float angle0 = 25.0f * B3_PI / 180.0f;
	float angle1 = -25.0f * B3_PI / 180.0f;
	float angle2 = 0.5f * B3_PI;

	b3BoxHull cardBox = b3MakeBoxHull( cardThickness, cardHeight, cardDepth );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	int Nb = 5;
	float z0 = 0.0f;
	float y = cardHeight - 0.02f;
	while ( Nb > 0 )
	{
		float z = z0;
		for ( int i = 0; i < Nb; i++ )
		{
			if ( i != Nb - 1 )
			{
				bodyDef.position = (b3Pos){ z + 0.25f, y + cardHeight - 0.015f, 0.0f };
				bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, angle2 );
				b3BodyId bodyId = Scene_AddBody( scene, &bodyDef );
				b3CreateHullShape( bodyId, &shapeDef, &cardBox.base );
			}

			bodyDef.position = (b3Pos){ z, y, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, angle1 );
			b3BodyId bodyId = Scene_AddBody( scene, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &cardBox.base );

			z += 0.175f;

			bodyDef.position = (b3Pos){ z, y, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, angle0 );
			bodyId = Scene_AddBody( scene, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &cardBox.base );

			z += 0.175f;
		}
		y += cardHeight * 2.0f - 0.03f;
		z0 += 0.175f;
		Nb -= 1;
	}
}

// shared/human.h
typedef enum BoneId
{
	bone_pelvis,
	bone_spine_01,
	bone_spine_03,
	bone_neck,
	bone_thigh_l,
	bone_calf_l,
	bone_thigh_r,
	bone_calf_r,
	bone_upper_arm_l,
	bone_lower_arm_l,
	bone_upper_arm_r,
	bone_lower_arm_r,
	bone_count,
} BoneId;

typedef struct SceneBone
{
	b3BodyId bodyId;
	b3JointId jointId;
	b3Transform localFrameA;
	b3Transform localFrameB;
	b3Transform referenceFrame;
	b3JointType jointType;
	float swingLimit;
	b3Vec2 twistLimit;
	float jointFriction;
	int parentIndex;
} SceneBone;

// shared/human.c CreateHuman without colors, names and user data
static void Scene_CreateHuman( Scene* scene, b3Pos position, float frictionTorque, float hertz,
							   float dampingRatio, int groupIndex )
{
	SceneBone bones[bone_count] = { 0 };

	for ( int i = 0; i < bone_count; ++i )
	{
		bones[i].bodyId = b3_nullBodyId;
		bones[i].jointId = b3_nullJointId;
		bones[i].jointFriction = 1.0f;
		bones[i].parentIndex = -1;
	}

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	float defaultDensity = shapeDef.density;

	{
		SceneBone* bone = bones + bone_pelvis;
		bone->parentIndex = -1;

		bone->referenceFrame = (b3Transform){ { 0.000000f, 0.996219f, -0.023868f },
											  { { 1.000000f, 0.000000f, 0.000000f }, 0.000000f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = Scene_AddBody( scene, &bodyDef );

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.6f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = {
			{ 0.040000f, 0.000001f, 0.000000f }, { -0.040000f, -0.000001f, 0.000000f }, 0.150000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );
	}

	{
		SceneBone* bone = bones + bone_spine_01;
		bone->parentIndex = bone_pelvis;

		bone->referenceFrame = (b3Transform){ { 0.000000f, 1.017288f, -0.024882f },
											  { { 1.000000f, 0.000000f, 0.000000f }, 0.000000f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = Scene_AddBody( scene, &bodyDef );

		shapeDef.filter.groupIndex = -groupIndex;

		shapeDef.baseMaterial.friction = 0.5f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = { { 0.029876f, -0.146581f, 0.006260f },
							  { -0.029876f, -0.146574f, 0.006260f },
							  0.145663f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { 0.000000f, -0.021069f, 0.001014f },
										   { { -0.642737f, 0.000000f, 0.000000f }, -0.766087f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f },
										   { { -0.707107f, 0.000000f, 0.000000f }, -0.707107f } };
		bone->swingLimit = 35.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -17.5f * B3_DEG_TO_RAD, 17.5f * B3_DEG_TO_RAD };
	}

	{
		SceneBone* bone = bones + bone_spine_03;
		bone->parentIndex = bone_spine_01;

		bone->referenceFrame = (b3Transform){ { 0.000000f, 1.267766f, -0.022320f },
											  { { 1.000000f, 0.000000f, 0.000000f }, 0.000000f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = Scene_AddBody( scene, &bodyDef );

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.5f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = { { 0.063996f, -0.117434f, -0.040199f },
							  { -0.063996f, -0.117432f, -0.040199f },
							  0.165004f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { 0.000000f, -0.250478f, -0.002562f },
										   { { -0.642766f, 0.000000f, 0.000000f }, -0.766063f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f },
										   { { -0.707107f, 0.000000f, 0.000000f }, -0.707107f } };
		bone->swingLimit = 35.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -17.5f * B3_DEG_TO_RAD, 17.5f * B3_DEG_TO_RAD };
	}

	{
		SceneBone* bone = bones + bone_neck;
		bone->parentIndex = bone_spine_03;

		bone->referenceFrame = (b3Transform){ { 0.000000f, 1.498783f, 0.024462f },
											  { { 0.987879f, 0.000000f, 0.000000f }, 0.155228f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = Scene_AddBody( scene, &bodyDef );

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.2f;
		shapeDef.baseMaterial.rollingResistance = 0.05f;
		b3Capsule head = {
			{ 0.000000f, -0.212340f, 0.000000f }, { 0.000000f, -0.087340f, 0.000000f }, 0.110000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &head );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { 0.000000f, -0.231017f, -0.046781f },
										   { { -0.516157f, -0.000002f, 0.000003f }, -0.856494f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f },
										   { { -0.707108f, -0.000002f, 0.000002f }, -0.707106f } };
		bone->swingLimit = 30.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -17.5f * B3_DEG_TO_RAD, 17.5f * B3_DEG_TO_RAD };
	}

	{
		SceneBone* bone = bones + bone_thigh_l;
		bone->parentIndex = bone_pelvis;

		bone->referenceFrame =
			(b3Transform){ { 0.092175f, 0.971562f, -0.011177f },
						   { { 0.471412f, 0.529510f, -0.499888f }, -0.497495f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = Scene_AddBody( scene, &bodyDef );

		shapeDef.filter.groupIndex = -groupIndex;

		shapeDef.baseMaterial.friction = 0.6f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = {
			{ -0.047269f, 0.000001f, 0.000000f }, { -0.379769f, 0.000005f, 0.000000f }, 0.091539f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { 0.092175f, 0.024656f, -0.012691f },
										   { { -0.460186f, -0.365368f, -0.637662f }, 0.498118f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f },
										   { { -0.499997f, -0.500003f, 0.500002f }, 0.499998f } };
		bone->swingLimit = 30.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -10.0f * B3_DEG_TO_RAD, 10.0f * B3_DEG_TO_RAD };
	}

	{
		SceneBone* bone = bones + bone_calf_l;
		bone->parentIndex = bone_thigh_l;

		bone->referenceFrame =
			(b3Transform){ { 0.118720f, 0.534541f, -0.035362f },
						   { { 0.521188f, 0.480597f, -0.546380f }, -0.445935f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = Scene_AddBody( scene, &bodyDef );

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.1f;
		shapeDef.baseMaterial.rollingResistance = 0.0f;
		b3Capsule shin = {
			{ -0.445000f, 0.000001f, 0.000000f }, { 0.005000f, -0.000001f, 0.000000f }, 0.080000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &shin );

		shapeDef.density = 0.5f * defaultDensity;
		b3Capsule foot = {
			{ -0.456371f, 0.129321f, 0.000013f }, { -0.359977f, 0.014394f, 0.000013f }, 0.070000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &foot );
		shapeDef.density = defaultDensity;

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { -0.438494f, -0.000175f, 0.000000f },
										   { { 0.344866f, -0.938652f, 0.000002f }, -0.000008f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f },
										   { { -0.008182f, -0.999967f, 0.000001f }, -0.000007f } };
		bone->twistLimit = (b3Vec2){ -30.0f * B3_DEG_TO_RAD, 30.0f * B3_DEG_TO_RAD };
	}

	{
		SceneBone* bone = bones + bone_thigh_r;
		bone->parentIndex = bone_pelvis;

		bone->referenceFrame =
			(b3Transform){ { -0.092175f, 0.971562f, -0.011177f },
						   { { -0.497495f, 0.499888f, 0.529510f }, -0.471412f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = Scene_AddBody( scene, &bodyDef );

		shapeDef.filter.groupIndex = -groupIndex;

		shapeDef.baseMaterial.friction = 0.6f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = {
			{ 0.047269f, -0.000001f, 0.000000f }, { 0.379769f, -0.000005f, 0.000000f }, 0.092587f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { -0.092175f, 0.024656f, -0.012691f },
										   { { 0.380666f, 0.491846f, 0.488644f }, -0.611889f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f },
										   { { -0.500002f, 0.499998f, -0.499998f }, 0.500002f } };
		bone->swingLimit = 30.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -10.0f * B3_DEG_TO_RAD, 10.0f * B3_DEG_TO_RAD };
	}

	{
		SceneBone* bone = bones + bone_calf_r;
		bone->parentIndex = bone_thigh_r;

		bone->referenceFrame =
			(b3Transform){ { -0.118720f, 0.534541f, -0.035362f },
						   { { -0.445935f, 0.546380f, 0.480597f }, -0.521188f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = Scene_AddBody( scene, &bodyDef );

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.1f;
		shapeDef.baseMaterial.rollingResistance = 0.0f;
		b3Capsule shin = {
			{ 0.445000f, -0.000002f, 0.000003f }, { -0.005000f, 0.000000f, 0.000003f }, 0.080000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &shin );

		shapeDef.density = 0.5f * defaultDensity;
		b3Capsule foot = {
			{ 0.369113f, -0.006830f, 0.000017f }, { 0.465507f, -0.121757f, 0.000017f }, 0.070000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &foot );
		shapeDef.density = defaultDensity;

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { 0.438494f, 0.000175f, 0.000000f },
										   { { 0.000007f, 0.000005f, -0.344860f }, -0.938654f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f },
										   { { 0.000006f, 0.000005f, 0.008188f }, -0.999966f } };
		bone->twistLimit = (b3Vec2){ -30.0f * B3_DEG_TO_RAD, 30.0f * B3_DEG_TO_RAD };
	}

	{
		SceneBone* bone = bones + bone_upper_arm_l;
		bone->parentIndex = bone_spine_03;

		bone->referenceFrame =
			(b3Transform){ { 0.185817f, 1.443630f, 0.031306f },
						   { { 0.560220f, -0.307179f, 0.366233f }, -0.676512f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = Scene_AddBody( scene, &bodyDef );

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.6f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = { { 0.296038f, 0.004386f, -0.000794f },
							  { -0.016086f, 0.004384f, -0.016127f },
							  0.065000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { 0.185817f, -0.175865f, -0.053625f },
										   { { -0.559466f, -0.213784f, -0.670199f }, 0.438323f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f },
										   { { -0.500953f, -0.502617f, -0.509318f }, -0.486844f } };
		bone->swingLimit = 45.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -20.0f * B3_DEG_TO_RAD, 20.0f * B3_DEG_TO_RAD };
		bone->jointFriction = 0.8f;
	}

	{
		SceneBone* bone = bones + bone_lower_arm_l;
		bone->parentIndex = bone_upper_arm_l;

		bone->referenceFrame =
			(b3Transform){ { 0.347683f, 1.193455f, 0.029378f },
						   { { 0.406736f, -0.493012f, 0.089135f }, -0.763911f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = Scene_AddBody( scene, &bodyDef );

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.3f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = { { 0.312329f, -0.008256f, -0.016454f },
							  { 0.033153f, 0.008259f, -0.002741f },
							  0.060000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { 0.297979f, 0.000361f, 0.000000f },
										   { { 0.916726f, 0.399352f, 0.000839f }, -0.011456f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f },
										   { { 0.999922f, -0.003857f, 0.004659f }, -0.010908f } };
		bone->twistLimit = (b3Vec2){ -35.0f * B3_DEG_TO_RAD, 35.0f * B3_DEG_TO_RAD };
		bone->jointFriction = 0.06f;
	}

	{
		SceneBone* bone = bones + bone_upper_arm_r;
		bone->parentIndex = bone_spine_03;

		bone->referenceFrame = (b3Transform){ { -0.185817f, 1.443630f, 0.031306f },
											  { { 0.676512f, 0.366233f, 0.307179f }, 0.560220f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = Scene_AddBody( scene, &bodyDef );

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.6f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = { { -0.300237f, -0.000001f, -0.000319f },
							  { 0.011887f, 0.000001f, 0.015013f },
							  0.065000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { -0.185817f, -0.175865f, -0.053625f },
										   { { 0.213793f, 0.559449f, 0.438334f }, -0.670204f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f },
										   { { -0.509319f, 0.486843f, 0.500954f }, -0.502617f } };
		bone->swingLimit = 45.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -20.0f * B3_DEG_TO_RAD, 20.0f * B3_DEG_TO_RAD };
		bone->jointFriction = 0.8f;
	}

	{
		SceneBone* bone = bones + bone_lower_arm_r;
		bone->parentIndex = bone_upper_arm_r;

		bone->referenceFrame = (b3Transform){ { -0.347683f, 1.193455f, 0.029378f },
											  { { 0.763911f, 0.089135f, 0.493012f }, 0.406736f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = Scene_AddBody( scene, &bodyDef );

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.3f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = { { -0.312649f, -0.000003f, 0.016471f },
							  { -0.032986f, 0.000000f, 0.002732f },
							  0.060000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { -0.297979f, -0.000361f, 0.000000f },
										   { { -0.405444f, 0.914059f, 0.010541f }, 0.000852f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f },
										   { { -0.002794f, 0.999936f, 0.010042f }, 0.004364f } };
		bone->twistLimit = (b3Vec2){ -35.0f * B3_DEG_TO_RAD, 35.0f * B3_DEG_TO_RAD };
		bone->jointFriction = 0.06f;
	}

	for ( int i = 1; i < bone_count; ++i )
	{
		SceneBone* bone = bones + i;
		SceneBone* parent = bones + bone->parentIndex;

		b3BodyId bodyIdA = parent->bodyId;
		b3BodyId bodyIdB = bone->bodyId;

		bone->localFrameA.q = b3NormalizeQuat( bone->localFrameA.q );
		bone->localFrameB.q = b3NormalizeQuat( bone->localFrameB.q );

		if ( bone->jointType == b3_revoluteJoint )
		{
			b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = bodyIdA;
			jointDef.base.bodyIdB = bodyIdB;
			jointDef.base.localFrameA = bone->localFrameA;
			jointDef.base.localFrameB = bone->localFrameB;
			jointDef.enableLimit = true;
			jointDef.lowerAngle = bone->twistLimit.x;
			jointDef.upperAngle = bone->twistLimit.y;
			jointDef.enableSpring = hertz > 0.0f;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = bone->jointFriction * frictionTorque;
			bone->jointId = b3CreateRevoluteJoint( scene->worldId, &jointDef );
		}
		else if ( bone->jointType == b3_sphericalJoint )
		{
			b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
			jointDef.base.bodyIdA = bodyIdA;
			jointDef.base.bodyIdB = bodyIdB;
			jointDef.base.localFrameA = bone->localFrameA;
			jointDef.base.localFrameB = bone->localFrameB;
			jointDef.enableConeLimit = true;
			jointDef.coneAngle = bone->swingLimit;
			jointDef.enableTwistLimit = true;
			jointDef.lowerTwistAngle = bone->twistLimit.x;
			jointDef.upperTwistAngle = bone->twistLimit.y;
			jointDef.enableSpring = hertz > 0.0f;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = bone->jointFriction * frictionTorque;
			bone->jointId = b3CreateSphericalJoint( scene->worldId, &jointDef );
		}
	}

	// Disable some collisions
	b3FilterJointDef filterDef = b3DefaultFilterJointDef();
	filterDef.base.bodyIdA = bones[bone_thigh_l].bodyId;
	filterDef.base.bodyIdB = bones[bone_thigh_r].bodyId;
	b3CreateFilterJoint( scene->worldId, &filterDef );
	filterDef.base.bodyIdA = bones[bone_neck].bodyId;
	filterDef.base.bodyIdB = bones[bone_upper_arm_l].bodyId;
	b3CreateFilterJoint( scene->worldId, &filterDef );
	filterDef.base.bodyIdA = bones[bone_neck].bodyId;
	filterDef.base.bodyIdB = bones[bone_upper_arm_r].bodyId;
	b3CreateFilterJoint( scene->worldId, &filterDef );
}

// samples/sample_joint.cpp BallAndChain, plus four ragdolls from shared/human.c dropped on a box as
// in samples/sample_ragdoll.cpp RagdollOnBox and RagdollPile. The ragdoll box sits at z = -30,
// clear of the chain.
static void CreateChains( Scene* scene )
{
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId groundBody = Scene_AddBody( scene, &bodyDef );

		float linkRadius = 0.125f;
		float linkExtent = 0.5f;
		b3Capsule capsule = { { -linkExtent, 0.0f, 0.0f }, { linkExtent, 0.0f, 0.0f }, linkRadius };
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		int linkCount = 32;

		bodyDef.type = b3_dynamicBody;
		b3BodyId parent = groundBody;
		b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
		jointDef.base.localFrameA = b3Transform_identity;
		jointDef.base.localFrameB = (b3Transform){ { -linkExtent, 0.0f, 0.0f }, b3Quat_identity };
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 10.0f;

		for ( int i = 0; i < linkCount; ++i )
		{
			bodyDef.position = (b3Pos){ ( 1.0f + 2.0f * (float)i ) * linkExtent, 0.0f, 0.0f };
			b3BodyId childId = Scene_AddBody( scene, &bodyDef );

			b3CreateCapsuleShape( childId, &shapeDef, &capsule );

			jointDef.base.bodyIdA = parent;
			jointDef.base.bodyIdB = childId;
			b3CreateSphericalJoint( scene->worldId, &jointDef );

			jointDef.base.localFrameA =
				(b3Transform){ { linkExtent, 0.0f, 0.0f }, b3Quat_identity };
			parent = childId;
		}

		float sphereRadius = 2.0f;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, sphereRadius };

		bodyDef.position =
			(b3Pos){ ( 1.0f + 2.0f * (float)linkCount ) * linkExtent + sphereRadius - linkExtent,
					 0.0f, 0.0f };

		b3BodyId childId = Scene_AddBody( scene, &bodyDef );

		b3CreateSphereShape( childId, &shapeDef, &sphere );
		jointDef.base.bodyIdA = parent;
		jointDef.base.bodyIdB = childId;
		jointDef.base.localFrameB = (b3Transform){ { -sphereRadius, 0.0f, 0.0f }, b3Quat_identity };
		b3CreateSphericalJoint( scene->worldId, &jointDef );
	}

	{
		float boxZ = -30.0f;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 0.0f, -1.0f, boxZ };
		b3BodyId groundId = Scene_AddBody( scene, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull hull = b3MakeBoxHull( 10.0f, 1.0f, 10.0f );
		b3CreateHullShape( groundId, &shapeDef, &hull.base );

		int humanCount = 4;
		for ( int i = 0; i < humanCount; ++i )
		{
			b3Pos position = { 0.2f * (float)i, 2.0f * (float)i, boxZ + 0.1f * (float)i };
			float torque = 10.0f;
			float hertz = 0.5f;
			float damping = 0.7f;
			int groupIndex = i + 1;
			Scene_CreateHuman( scene, position, torque, hertz, damping, groupIndex );
		}
	}
}

// samples/sample_continuous.cpp MeshDrop: wave mesh with walls, a 16 x 16 grid (sample: 32 x 32) of
// capsules with every seventh body a sphere and every eighth a box, random velocities from the
// scene LCG
static void CreateCapsuleMesh( Scene* scene )
{
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId groundId = Scene_AddBody( scene, &bodyDef );

		int gridCount = 40;
		float cellWidth = 1.0f;
		float amplitude = 0.5f;
		float rowHz = 0.1f;
		float columnHz = 0.2f;

		scene->mesh =
			b3CreateWaveMesh( gridCount, gridCount, cellWidth, amplitude, rowHz, columnHz );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.filter.categoryBits = 1;
		b3CreateMeshShape( groundId, &shapeDef, scene->mesh, b3Vec3_one );

		float extent = 0.5f * (float)gridCount * cellWidth;
		float halfHeight = 1.0f;

		{
			b3Transform transform;
			transform.p = (b3Vec3){ 0.0f, halfHeight, -extent };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( extent, halfHeight, 0.1f, transform );
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = (b3Vec3){ 0.0f, halfHeight, extent };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( extent, halfHeight, 0.1f, transform );
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = (b3Vec3){ -extent, halfHeight, 0.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, halfHeight, extent, transform );
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = (b3Vec3){ extent, halfHeight, 0.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, halfHeight, extent, transform );
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}
	}

	b3BoxHull box = b3MakeBoxHull( 0.02f, 0.2f, 0.04f );
	b3Capsule capsule = { { 0.0f, -0.2f, 0.0f }, { 0.0f, 0.2f, 0.0f }, 0.05f };
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.05f };

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	b3ShapeDef capsuleShapeDef = b3DefaultShapeDef();
	capsuleShapeDef.baseMaterial.rollingResistance = 0.4f;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.rollingResistance = 0.1f;

	scene->seed = 12345;

	int gridCount = 16;
	float halfGrid = 0.5f * (float)gridCount;
	int bodyIndex = 0;

	for ( int i = 0; i < gridCount; ++i )
	{
		for ( int j = 0; j < gridCount; ++j )
		{
			b3Vec3 linearVelocity;
			linearVelocity.x = Scene_RandomFloat( scene, -1.0f, 1.0f );
			linearVelocity.y = Scene_RandomFloat( scene, -1.0f, 1.0f );
			linearVelocity.z = Scene_RandomFloat( scene, -1.0f, 1.0f );

			b3Vec3 angularVelocity;
			angularVelocity.x = Scene_RandomFloat( scene, -5.0f, 5.0f );
			angularVelocity.y = Scene_RandomFloat( scene, -5.0f, 5.0f );
			angularVelocity.z = Scene_RandomFloat( scene, -5.0f, 5.0f );

			bodyDef.position =
				(b3Pos){ 0.5f * ( (float)i - halfGrid ), 5.0f, 0.5f * ( (float)j - halfGrid ) };
			bodyDef.linearVelocity = linearVelocity;
			bodyDef.angularVelocity = angularVelocity;
			b3BodyId bodyId = Scene_AddBody( scene, &bodyDef );

			int kind = bodyIndex % 8;
			if ( kind == 6 )
			{
				b3CreateSphereShape( bodyId, &shapeDef, &sphere );
			}
			else if ( kind == 7 )
			{
				b3CreateHullShape( bodyId, &shapeDef, &box.base );
			}
			else
			{
				b3CreateCapsuleShape( bodyId, &capsuleShapeDef, &capsule );
			}

			bodyIndex += 1;
		}
	}
}

// shared/determinism.c CreateWavePile: a convex pile on a wave height field (the height field API
// of samples/sample_mesh.cpp HeightField). RandomVec3Uniform and RandomQuat are replaced by the
// scene LCG.
static void CreateHeightField( Scene* scene )
{
	scene->seed = 52977;

	int fieldCount = 21;
	b3Vec3 fieldScale = { 1.0f, 0.6f, 1.0f };
	scene->heightField = b3CreateWave( fieldCount, fieldCount, fieldScale, 0.08f, 0.06f, false );

	{
		float extent = fieldScale.x * (float)( fieldCount - 1 );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position.x = -0.5f * extent;
		bodyDef.position.z = -0.5f * extent;
		b3BodyId groundId = Scene_AddBody( scene, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHeightFieldShape( groundId, &shapeDef, scene->heightField );
	}

	b3HullData* rock = b3CreateRock( 0.55f );
	b3BoxHull box = b3MakeBoxHull( 0.45f, 0.3f, 0.55f );
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	b3Capsule capsule = { { 0.0f, -0.3f, 0.0f }, { 0.0f, 0.3f, 0.0f }, 0.35f };

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.rollingResistance = 0.3f;

	int gridCount = 5;
	int layerCount = 4;
	float spacing = 1.7f;
	float halfSpan = 0.5f * (float)( gridCount - 1 );
	int index = 0;
	for ( int layer = 0; layer < layerCount; ++layer )
	{
		for ( int i = 0; i < gridCount; ++i )
		{
			for ( int j = 0; j < gridCount; ++j )
			{
				b3Vec3 jitter;
				jitter.x = Scene_RandomFloat( scene, -0.3f, 0.3f );
				jitter.y = Scene_RandomFloat( scene, -0.3f, 0.3f );
				jitter.z = Scene_RandomFloat( scene, -0.3f, 0.3f );

				b3Vec3 axis;
				axis.x = Scene_RandomFloat( scene, -1.0f, 1.0f );
				axis.y = Scene_RandomFloat( scene, -1.0f, 1.0f );
				axis.z = Scene_RandomFloat( scene, -1.0f, 1.0f );
				float angle = Scene_RandomFloat( scene, -B3_PI, B3_PI );

				bodyDef.position.x = spacing * ( (float)i - halfSpan ) + jitter.x;
				bodyDef.position.y = 2.5f + 1.6f * (float)layer + 0.3f * jitter.y;
				bodyDef.position.z = spacing * ( (float)j - halfSpan ) + jitter.z;
				bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Normalize( axis ), angle );

				b3BodyId bodyId = Scene_AddBody( scene, &bodyDef );

				int kind = index % 4;
				if ( kind == 0 )
				{
					b3CreateSphereShape( bodyId, &shapeDef, &sphere );
				}
				else if ( kind == 1 )
				{
					b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
				}
				else if ( kind == 2 )
				{
					b3CreateHullShape( bodyId, &shapeDef, &box.base );
				}
				else
				{
					b3CreateHullShape( bodyId, &shapeDef, rock );
				}

				index += 1;
			}
		}
	}

	// The world keeps its own copy
	b3DestroyHull( rock );
}

// samples/sample_continuous.cpp BulletVersusStack, plus ThinWall moved to z = -40 on the same
// ground
static void CreateBullets( Scene* scene )
{
	Scene_AddGroundBox( scene, 50.0f );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
		b3BodyId groundBodyId = Scene_AddBody( scene, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3Transform transform = { { -1.0f, 5.0f, 0.0f }, b3Quat_identity };
		b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, 5.0f, 10.0f, transform );
		b3CreateHullShape( groundBodyId, &shapeDef, &wallBox.base );
	}

	{
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		for ( int row = 0; row < 10; ++row )
		{
			bodyDef.position = (b3Pos){ 0.0f, 0.5f + 1.1f * (float)row, 0.0f };
			b3BodyId bodyId = Scene_AddBody( scene, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
	}

	scene->bulletId = b3_nullBodyId;

	{
		float wallZ = -40.0f;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		bodyDef.position = (b3Pos){ 0.0f, 10.0f, wallZ };
		bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 90.0f * B3_DEG_TO_RAD );
		b3BodyId wallId = Scene_AddBody( scene, &bodyDef );
		b3BoxHull wallBox = b3MakeBoxHull( 10.0f, 0.1f, 10.0f );
		b3CreateHullShape( wallId, &shapeDef, &wallBox.base );

		bodyDef.type = b3_dynamicBody;
		bodyDef.rotation = b3Quat_identity;
		shapeDef.baseMaterial.rollingResistance = 0.1f;

		bodyDef.position = (b3Pos){ -5.0f, 10.0f, wallZ + 20.0f };
		bodyDef.linearVelocity = (b3Vec3){ 0.0f, 0.0f, -180.0f };
		bodyDef.angularVelocity = (b3Vec3){ 20.0f, 0.0f, 0.0f };
		b3BodyId sphereBodyId = Scene_AddBody( scene, &bodyDef );

		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.1f };
		b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );

		bodyDef.position = (b3Pos){ 0.0f, 10.0f, wallZ + 20.0f };
		bodyDef.linearVelocity = (b3Vec3){ 0.0f, 0.0f, -180.0f };
		bodyDef.angularVelocity = (b3Vec3){ 20.0f, -5.0f, 0.0f };
		b3BodyId capsuleBodyId = Scene_AddBody( scene, &bodyDef );

		b3Capsule capsule = { { -0.3f, 0.0f, 0.0f }, { 0.3f, 0.0f, 0.0f }, 0.1f };
		b3CreateCapsuleShape( capsuleBodyId, &shapeDef, &capsule );

		bodyDef.position = (b3Pos){ 5.0f, 10.0f, wallZ + 20.0f };
		bodyDef.linearVelocity = (b3Vec3){ 0.0f, 0.0f, -180.0f };
		bodyDef.angularVelocity = (b3Vec3){ 20.0f, 5.0f, 0.0f };
		b3BodyId boxBodyId = Scene_AddBody( scene, &bodyDef );
		b3BoxHull box = b3MakeBoxHull( 0.4f, 0.1f, 0.1f );
		b3CreateHullShape( boxBodyId, &shapeDef, &box.base );
	}
}

// samples/sample_continuous.cpp BulletVersusStack::Launch
static void LaunchBullet( Scene* scene )
{
	if ( B3_IS_NON_NULL( scene->bulletId ) )
	{
		b3DestroyBody( scene->bulletId );
	}

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.isBullet = true;
	bodyDef.position = (b3Pos){ 20.5f, 5.5f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ -500.0f, 0.0f, 0.0f };
	scene->bulletId = Scene_AddBody( scene, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density *= 10.0f;
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.25f };
	b3CreateSphereShape( scene->bulletId, &shapeDef, &sphere );
}

// samples/mover.cpp CharacterMover constants
static const float s_moverJumpSpeed = 5.0f;
static const float s_moverMaxSpeed = 6.0f;
static const float s_moverMinSpeed = 0.01f;
static const float s_moverStopSpeed = 1.0f;
static const float s_moverAccelerate = 30.0f;
static const float s_moverFriction = 4.0f;
static const float s_moverGravity = 15.0f;

// samples/mover.cpp CharacterMover::Initialize
static void Scene_InitializeMover( SceneMover* mover, b3Pos position )
{
	memset( mover, 0, sizeof( SceneMover ) );
	mover->transform.p = position;
	mover->transform.q = b3Quat_identity;
	mover->velocity = (b3Vec3){ 0.0f, 0.0f, 0.0f };
	mover->capsule = (b3Capsule){ { 0.0f, -0.5f, 0.0f }, { 0.0f, 0.5f, 0.0f }, 0.3f };
	mover->planeCount = 0;
	mover->totalIterations = 0;
	mover->pogoVelocity = 0.0f;
	mover->onGround = false;
	mover->sprint = false;
}

// The static level of samples/sample_character.cpp GeometricMover is an obj mesh, so this builds a
// small level from boxes instead: floor, step, ramp, wall, and one dynamic box in the path for the
// push code.
static void CreateMover( Scene* scene )
{
	Scene_AddGroundBox( scene, 20.0f );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId levelId = Scene_AddBody( scene, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	{
		b3Transform transform = { { 4.0f, 0.15f, 0.0f }, b3Quat_identity };
		b3BoxHull box = b3MakeTransformedBoxHull( 1.0f, 0.15f, 3.0f, transform );
		b3CreateHullShape( levelId, &shapeDef, &box.base );
	}

	{
		b3Transform transform;
		transform.p = (b3Vec3){ 9.0f, 0.5f, 0.0f };
		transform.q = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 20.0f * B3_DEG_TO_RAD );
		b3BoxHull box = b3MakeTransformedBoxHull( 3.0f, 0.1f, 3.0f, transform );
		b3CreateHullShape( levelId, &shapeDef, &box.base );
	}

	{
		b3Transform transform = { { 14.0f, 1.5f, 0.0f }, b3Quat_identity };
		b3BoxHull box = b3MakeTransformedBoxHull( 0.2f, 1.5f, 6.0f, transform );
		b3CreateHullShape( levelId, &shapeDef, &box.base );
	}

	{
		b3BodyDef boxDef = b3DefaultBodyDef();
		boxDef.type = b3_dynamicBody;
		boxDef.position = (b3Pos){ 1.5f, 0.4f, 0.5f };
		b3BodyId boxId = Scene_AddBody( scene, &boxDef );
		b3BoxHull box = b3MakeBoxHull( 0.4f, 0.4f, 0.4f );
		b3CreateHullShape( boxId, &shapeDef, &box.base );
	}

	scene->hasMover = true;
	Scene_InitializeMover( &scene->mover, (b3Pos){ 0.0f, 1.4f, 0.0f } );
}

static bool Scene_MoverFilter( b3ShapeId shapeId, void* context )
{
	(void)shapeId;
	(void)context;
	return true;
}

// samples/mover.cpp PlaneResultFcn; no shape carries MoverShapeUserData so every plane is rigid and
// clips
static bool Scene_MoverPlaneResult( b3ShapeId shapeId, const b3PlaneResult* planeResults,
									int planeCount, void* context )
{
	SceneMover* mover = context;
	float maxPush = FLT_MAX;
	bool clipVelocity = true;

	for ( int i = 0; i < planeCount && mover->planeCount < SCENE_MOVER_PLANE_CAPACITY; ++i )
	{
		int index = mover->planeCount;
		mover->planes[index].plane = planeResults[i].plane;
		mover->planes[index].pushLimit = maxPush;
		mover->planes[index].push = 0.0f;
		mover->planes[index].clipVelocity = clipVelocity;
		mover->planePoints[index] = b3OffsetPos( mover->transform.p, planeResults[i].point );
		mover->planeShapes[index] = shapeId;
		mover->planeCount += 1;
	}

	return true;
}

// samples/mover.cpp CharacterMover::SolveMove
static void Scene_SolveMove( Scene* scene, SceneMover* mover, float timeStep, b3Vec3 forward,
							 b3Vec3 right, b3Vec2 throttle, bool clipVelocity )
{
	// Friction
	float speed = b3Length( mover->velocity );
	if ( speed < s_moverMinSpeed )
	{
		mover->velocity.x = 0.0f;
		mover->velocity.z = 0.0f;
	}
	else
	{
		// Linear damping above stopSpeed and fixed reduction below stopSpeed
		float control = speed < s_moverStopSpeed ? s_moverStopSpeed : speed;

		// friction has units of 1/time
		float drop = control * s_moverFriction * timeStep;
		float newSpeed = b3MaxFloat( 0.0f, speed - drop );
		float ratio = newSpeed / speed;
		mover->velocity.x *= ratio;
		mover->velocity.z *= ratio;
	}

	float maxSpeed = mover->sprint ? 1.5f * s_moverMaxSpeed : s_moverMaxSpeed;

	b3Vec3 desiredForward = b3MulSV( maxSpeed * throttle.x, forward );
	b3Vec3 desiredRight = b3MulSV( maxSpeed * throttle.y, right );
	b3Vec3 desiredVelocity = b3Add( desiredForward, desiredRight );
	float desiredSpeed = 0.0f;
	b3Vec3 desiredDirection = b3GetLengthAndNormalize( &desiredSpeed, desiredVelocity );

	if ( desiredSpeed > maxSpeed )
	{
		desiredVelocity = b3MulSV( maxSpeed / desiredSpeed, desiredVelocity );
		desiredSpeed = maxSpeed;
	}

	if ( mover->onGround )
	{
		mover->velocity.y = 0.0f;
	}

	// Accelerate
	float currentSpeed = b3Dot( mover->velocity, desiredDirection );
	float addSpeed = desiredSpeed - currentSpeed;
	if ( addSpeed > 0.0f )
	{
		float accelSpeed = s_moverAccelerate * maxSpeed * timeStep;
		if ( accelSpeed > addSpeed )
		{
			accelSpeed = addSpeed;
		}

		mover->velocity = b3Add( mover->velocity, b3MulSV( accelSpeed, desiredDirection ) );
	}

	mover->velocity.y -= s_moverGravity * timeStep;

	b3WorldId worldId = scene->worldId;

	float pogoRestLength = 3.0f * mover->capsule.radius;
	float rayLength = pogoRestLength + mover->capsule.radius;
	b3Pos rayOrigin = b3TransformWorldPoint( mover->transform, mover->capsule.center1 );
	b3Vec3 rayTranslation = b3MulSV( -rayLength, b3Vec3_axisY );

	// The sample writes { 1, ~2u }, which zero extends the mask
	b3QueryFilter skipTeamFilter = b3DefaultQueryFilter();
	skipTeamFilter.categoryBits = 1;
	skipTeamFilter.maskBits = 0xFFFFFFFDull;
	b3RayResult rayResult =
		b3World_CastRayClosest( worldId, rayOrigin, rayTranslation, skipTeamFilter );
	mover->rayHit = rayResult.hit;
	mover->rayFraction = rayResult.fraction;

	// After gravity was applied, disable pogo when still moving up.
	// Avoids getting pulled back to the ground when jumping.
	bool suppressPogo = mover->velocity.y > 0.0f;

	if ( rayResult.hit == false || suppressPogo )
	{
		mover->onGround = false;
		mover->pogoVelocity = 0.0f;
	}
	else
	{
		mover->onGround = true;
		float pogoCurrentLength = rayResult.fraction * rayLength;

		float zeta = 0.7f;
		float hertz = 4.0f;
		float omega = 2.0f * B3_PI * hertz;
		float omegaH = omega * timeStep;

		mover->pogoVelocity =
			( mover->pogoVelocity - omega * omegaH * ( pogoCurrentLength - pogoRestLength ) ) /
			( 1.0f + 2.0f * zeta * omegaH + omegaH * omegaH );
	}

	b3Pos startPosition = mover->transform.p;
	b3Vec3 velocityDelta = b3MulSV( timeStep, mover->velocity );
	b3Vec3 pogoDelta = b3MulSV( timeStep * mover->pogoVelocity, b3Vec3_axisY );
	b3Pos target = b3Add( b3Add( mover->transform.p, velocityDelta ), pogoDelta );

	// Want the mover to collide with allies
	b3QueryFilter moverFilter = b3DefaultQueryFilter();
	moverFilter.categoryBits = 1;
	moverFilter.maskBits = 0xFFFFFFFFull;
	moverFilter.id = 1;

	// The cast should ignore allies
	b3QueryFilter castFilter = b3DefaultQueryFilter();
	castFilter.categoryBits = 1;
	castFilter.maskBits = 0xFFFFFFFDull;
	castFilter.id = 1;

	mover->totalIterations = 0;
	mover->moveIterations = 0;
	mover->castFraction = 1.0f;
	float tolerance = 0.01f;

	for ( int iteration = 0; iteration < 5; ++iteration )
	{
		mover->planeCount = 0;
		mover->moveIterations = iteration + 1;

		b3Capsule capsule;
		capsule.center1 = mover->capsule.center1;
		capsule.center2 = mover->capsule.center2;
		capsule.radius = mover->capsule.radius;

		b3World_CollideMover( worldId, mover->transform.p, &capsule, moverFilter,
							  Scene_MoverPlaneResult, mover );

		b3Vec3 targetDelta = b3Sub( target, mover->transform.p );
		b3PlaneSolverResult result = b3SolvePlanes( targetDelta, mover->planes, mover->planeCount );

		mover->totalIterations += result.iterationCount;

		b3Vec3 delta = result.delta;

		float fraction = b3World_CastMover( worldId, mover->transform.p, &capsule, delta,
											castFilter, Scene_MoverFilter, mover );
		mover->castFraction = fraction;

		delta = b3MulSV( fraction, delta );
		mover->transform.p = b3Add( mover->transform.p, delta );

		if ( b3LengthSquared( delta ) < tolerance * tolerance )
		{
			break;
		}
	}

	for ( int i = 0; i < mover->planeCount; ++i )
	{
		b3BodyId bodyId = b3Shape_GetBody( mover->planeShapes[i] );
		b3BodyType bodyType = b3Body_GetType( bodyId );
		if ( bodyType != b3_dynamicBody )
		{
			continue;
		}

		b3Pos point = mover->planePoints[i];
		b3Vec3 normal = b3Neg( mover->planes[i].plane.normal );

		float invMassA = 0.0f;
		float invMassB = b3Body_GetInverseMass( bodyId );
		b3Matrix3 invIB = b3Body_GetWorldInverseRotationalInertia( bodyId );

		b3Pos pB = b3Body_GetWorldCenter( bodyId );
		b3Vec3 rB = b3SubPos( point, pB );

		b3Vec3 rnB = b3Cross( rB, normal );
		float kNormal = invMassA + invMassB + b3Dot( rnB, b3MulMV( invIB, rnB ) );
		float normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

		b3Vec3 vB = b3Body_GetLinearVelocity( bodyId );
		b3Vec3 omegaB = b3Body_GetAngularVelocity( bodyId );
		b3Vec3 vrB = b3Add( vB, b3Cross( omegaB, rB ) );
		float vn = b3Dot( b3Sub( vrB, mover->velocity ), normal );
		float impulse = b3MaxFloat( -normalMass * vn, 0.0f );

		b3Vec3 P = b3MulSV( impulse, normal );
		mover->velocity = b3MulSub( mover->velocity, invMassA, P );

		b3Body_ApplyLinearImpulse( bodyId, P, point, true );
	}

	if ( clipVelocity )
	{
		// Using the velocity clipper can avoid picking up velocity from depenetration.
		// This allows the mover to avoid velocity from soft collision depenetration.
		mover->velocity = b3ClipVector( mover->velocity, mover->planes, mover->planeCount );
	}
	else if ( timeStep > 0.0f )
	{
		// Using the position delta is more holistic and intuitive in some cases.
		mover->velocity = b3MulSV( 1.0f / timeStep, b3Sub( mover->transform.p, startPosition ) );
	}
}

// samples/mover.cpp CharacterMover::Step with scripted keys in place of the keyboard and camera
static void Scene_StepMover( Scene* scene, int stepIndex, float timeStep )
{
	SceneMover* mover = &scene->mover;

	b3Vec2 throttle = { 0.0f, 0.0f };
	b3Vec3 forward = { 1.0f, 0.0f, 0.0f };
	b3Vec3 right = { 0.0f, 0.0f, 1.0f };
	bool jumpKey = false;
	bool sprintKey = false;

	if ( stepIndex < 140 )
	{
		throttle.x = 1.0f;
	}
	else if ( stepIndex < 200 )
	{
		throttle.x = 1.0f;
		throttle.y = 1.0f;
		jumpKey = stepIndex == 170;
	}
	else if ( stepIndex < 260 )
	{
		throttle.x = -1.0f;
		sprintKey = true;
	}

	if ( jumpKey && mover->onGround == true )
	{
		mover->velocity.y = s_moverJumpSpeed;
		mover->onGround = false;
	}

	if ( mover->onGround == true )
	{
		mover->sprint = sprintKey;
	}
	else
	{
		mover->sprint = false;
	}

	bool clipVelocity = true;
	Scene_SolveMove( scene, mover, timeStep, forward, right, throttle, clipVelocity );
}

// shared/benchmarks.c CreateSmallPyramid
static void CreateSmallPyramid( Scene* scene, int baseCount, float extent, float centerX,
								float baseZ )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 100.0f;

	b3BoxHull box = b3MakeBoxHull( extent, extent, extent );

	for ( int i = 0; i < baseCount; ++i )
	{
		float y = ( 2.0f * (float)i + 1.0f ) * extent;

		for ( int j = i; j < baseCount; ++j )
		{
			float x =
				( (float)i + 1.0f ) * extent + 2.0f * (float)( j - i ) * extent + centerX - 0.5f;
			bodyDef.position = (b3Pos){ x, y, baseZ };

			b3BodyId bodyId = Scene_AddBody( scene, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
	}
}

// shared/benchmarks.c CreateManyPyramids, release sizes
static void CreateManyPyramids( Scene* scene )
{
	int baseCount = 10;
	float extent = 0.5f;
	int rowCount = 14;
	int columnCount = 14;
	float groundExtent = extent * (float)columnCount * ( (float)baseCount + 1.0f );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = Scene_AddBody( scene, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( groundExtent, 1.0f, groundExtent );
		b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	float baseWidth = 2.0f * extent * (float)baseCount;
	float baseZ = -groundExtent + 2.0f * extent;
	float deltaZ = 2.0f * ( groundExtent - 2.0f * extent ) / ( (float)rowCount - 1.0f );

	for ( int i = 0; i < rowCount; ++i )
	{
		for ( int j = 0; j < columnCount; ++j )
		{
			float centerX =
				-groundExtent + (float)j * ( baseWidth + 2.0f * extent ) + 2.0f * extent;
			CreateSmallPyramid( scene, baseCount, extent, centerX, baseZ );
		}

		baseZ += deltaZ;
	}
}

// shared/benchmarks.c CreateMeshDropBenchmark at release size: 100 x 100 boxes that only collide
// with a 120 x 120 wave mesh. RandomVec3Uniform is replaced by the scene LCG.
static void CreateMeshDrop( Scene* scene )
{
	int meshGridCount = 120;
	int borderCount = 10;
	float cellWidth = 1.0f;

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId groundId = Scene_AddBody( scene, &bodyDef );

		float rowHz = 0.1f;
		float columnHz = 0.2f;
		float groundAmplitude = 0.5f;

		scene->mesh = b3CreateWaveMesh( meshGridCount, meshGridCount, cellWidth, groundAmplitude,
										rowHz, columnHz );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.filter.categoryBits = 1;
		b3CreateMeshShape( groundId, &shapeDef, scene->mesh, b3Vec3_one );
	}

	b3BoxHull box = b3MakeBoxHull( 0.02f, 0.2f, 0.04f );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.rollingResistance = 0.1f;

	// Don't allow shapes to collide with each other.
	shapeDef.filter.categoryBits = 2;
	shapeDef.filter.maskBits = 1;

	scene->seed = 3963634789u;

	int gridCount = 100;
	float spacing = cellWidth * (float)( meshGridCount - 2 * borderCount ) / (float)gridCount;
	float offset = 0.5f * spacing * (float)( 1 - gridCount );

	for ( int i = 0; i < gridCount; ++i )
	{
		for ( int j = 0; j < gridCount; ++j )
		{
			b3Vec3 linearVelocity;
			linearVelocity.x = Scene_RandomFloat( scene, -1.0f, 1.0f );
			linearVelocity.y = Scene_RandomFloat( scene, -1.0f, 1.0f );
			linearVelocity.z = Scene_RandomFloat( scene, -1.0f, 1.0f );

			b3Vec3 angularVelocity;
			angularVelocity.x = Scene_RandomFloat( scene, -5.0f, 5.0f );
			angularVelocity.y = Scene_RandomFloat( scene, -5.0f, 5.0f );
			angularVelocity.z = Scene_RandomFloat( scene, -5.0f, 5.0f );

			bodyDef.position.x = offset + spacing * (float)i;
			bodyDef.position.y = 5.0f;
			bodyDef.position.z = offset + spacing * (float)j;
			bodyDef.linearVelocity = linearVelocity;
			bodyDef.angularVelocity = angularVelocity;
			b3BodyId bodyId = Scene_AddBody( scene, &bodyDef );

			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
	}
}

static void Scene_HashQuery( Scene* scene, int kind, uint32_t value )
{
	uint32_t h = scene->queryHash[kind] ^ value;
	scene->queryHash[kind] = h * 16777619u;
}

static uint32_t Scene_FloatBits( float value )
{
	uint32_t bits;
	memcpy( &bits, &value, sizeof( bits ) );
	return bits;
}

static void Scene_HashVec3( Scene* scene, int kind, b3Vec3 v )
{
	Scene_HashQuery( scene, kind, Scene_FloatBits( v.x ) );
	Scene_HashQuery( scene, kind, Scene_FloatBits( v.y ) );
	Scene_HashQuery( scene, kind, Scene_FloatBits( v.z ) );
}

// The static compound of shared/benchmarks.c CreateVillage: a grid of box tiles with a sparse set
// of wave mesh patches, and a capsule or sphere prop on every eighth tile
static void Scene_CreateVillageGround( Scene* scene, int gridCount, float angle )
{
	float tileWidth = 4.0f;
	float halfSpan = 0.5f * tileWidth * (float)gridCount;
	scene->halfSpan = halfSpan;

	int tileCount = gridCount * gridCount;
	int propCapacity = tileCount / 8 + 1;

	b3CompoundHullDef* hulls = malloc( tileCount * sizeof( b3CompoundHullDef ) );
	b3CompoundMeshDef* meshes = malloc( tileCount * sizeof( b3CompoundMeshDef ) );
	b3CompoundCapsuleDef* capsules = malloc( propCapacity * sizeof( b3CompoundCapsuleDef ) );
	b3CompoundSphereDef* spheres = malloc( propCapacity * sizeof( b3CompoundSphereDef ) );

	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	b3BoxHull tile = b3MakeBoxHull( 0.5f * tileWidth, 0.5f, 0.5f * tileWidth );

	// A wave patch replaces a sparse set of tiles so the compound carries mesh children
	b3MeshData* wave = b3CreateWaveMesh( 8, 8, tileWidth / 8.0f, 0.25f, 0.25f, 0.25f );

	int hullCount = 0;
	int meshCount = 0;
	int capsuleCount = 0;
	int sphereCount = 0;

	for ( int i = 0; i < gridCount; ++i )
	{
		float x = -halfSpan + ( (float)i + 0.5f ) * tileWidth;

		for ( int j = 0; j < gridCount; ++j )
		{
			float z = -halfSpan + ( (float)j + 0.5f ) * tileWidth;

			if ( i % 16 == 8 && j % 16 == 8 )
			{
				b3CompoundMeshDef* meshDef = meshes + meshCount;
				memset( meshDef, 0, sizeof( b3CompoundMeshDef ) );
				meshDef->meshData = wave;
				meshDef->transform.p = (b3Vec3){ x, 0.0f, z };
				meshDef->transform.q = b3Quat_identity;
				meshDef->scale = b3Vec3_one;
				meshDef->materials = &material;
				meshDef->materialCount = 1;
				meshCount += 1;
				continue;
			}

			b3CompoundHullDef* hullDef = hulls + hullCount;
			memset( hullDef, 0, sizeof( b3CompoundHullDef ) );
			hullDef->hull = &tile.base;
			hullDef->transform.p = (b3Vec3){ x, -0.5f, z };
			hullDef->transform.q = b3Quat_identity;
			hullDef->material = material;
			hullCount += 1;

			int tileIndex = i * gridCount + j;
			if ( tileIndex % 8 != 0 )
			{
				continue;
			}

			if ( ( tileIndex / 8 ) & 1 )
			{
				float radius = 0.25f;
				b3CompoundCapsuleDef* capsuleDef = capsules + capsuleCount;
				memset( capsuleDef, 0, sizeof( b3CompoundCapsuleDef ) );
				capsuleDef->capsule.center1 = (b3Vec3){ x - 1.0f, radius, z };
				capsuleDef->capsule.center2 = (b3Vec3){ x + 1.0f, radius, z };
				capsuleDef->capsule.radius = radius;
				capsuleDef->material = material;
				capsuleCount += 1;
			}
			else
			{
				float radius = 0.5f;
				b3CompoundSphereDef* sphereDef = spheres + sphereCount;
				memset( sphereDef, 0, sizeof( b3CompoundSphereDef ) );
				sphereDef->sphere.center = (b3Vec3){ x, radius, z };
				sphereDef->sphere.radius = radius;
				sphereDef->material = material;
				sphereCount += 1;
			}
		}
	}

	b3CompoundDef def = { 0 };
	def.capsules = capsules;
	def.capsuleCount = capsuleCount;
	def.hulls = hulls;
	def.hullCount = hullCount;
	def.meshes = meshes;
	def.meshCount = meshCount;
	def.spheres = spheres;
	def.sphereCount = sphereCount;

	scene->compound = b3CreateCompound( &def );

	// The compound clones its inputs
	b3DestroyMesh( wave );
	free( hulls );
	free( meshes );
	free( capsules );
	free( spheres );

	// Rotated so broad-phase lookups transform into the compound frame
	scene->compoundRotation = b3MakeQuatFromAxisAngle( b3Vec3_axisY, angle );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.rotation = scene->compoundRotation;
	b3BodyId groundId = Scene_AddBody( scene, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateBakedCompoundShape( groundId, &shapeDef, scene->compound );
}

// shared/benchmarks.c CreateVillage at release size: 70 x 70 mixed bodies, every twentieth fast
// enough to need continuous, onto a rotated 100 x 100 tile compound. RandomFloatRange and
// RandomVec3Uniform are replaced by the scene LCG.
static void CreateVillage( Scene* scene )
{
	scene->seed = 2718281828u;

	Scene_CreateVillageGround( scene, 100, -1.15f * B3_PI );

	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.3f };
	b3Capsule capsule = { { -0.3f, 0.0f, 0.0f }, { 0.3f, 0.0f, 0.0f }, 0.2f };
	b3BoxHull box = b3MakeCubeHull( 0.3f );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	int bodyGrid = 70;
	float spacing = 1.6f;
	float offset = 0.5f * spacing * (float)( 1 - bodyGrid );

	for ( int i = 0; i < bodyGrid; ++i )
	{
		for ( int j = 0; j < bodyGrid; ++j )
		{
			int index = i * bodyGrid + j;

			bodyDef.position.x = offset + spacing * (float)i;
			bodyDef.position.y = Scene_RandomFloat( scene, 2.0f, 6.0f );
			bodyDef.position.z = offset + spacing * (float)j;
			bodyDef.linearVelocity = b3Vec3_zero;
			bodyDef.angularVelocity.x = Scene_RandomFloat( scene, -2.0f, 2.0f );
			bodyDef.angularVelocity.y = Scene_RandomFloat( scene, -2.0f, 2.0f );
			bodyDef.angularVelocity.z = Scene_RandomFloat( scene, -2.0f, 2.0f );

			// Fast enough to need continuous collision against the compound
			if ( index % 20 == 0 )
			{
				bodyDef.position.y = 20.0f;
				bodyDef.linearVelocity = (b3Vec3){ 0.0f, -80.0f, 0.0f };
			}

			b3BodyId bodyId = Scene_AddBody( scene, &bodyDef );

			int kind = index % 3;
			if ( kind == 0 )
			{
				b3CreateSphereShape( bodyId, &shapeDef, &sphere );
			}
			else if ( kind == 1 )
			{
				b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			}
			else
			{
				b3CreateHullShape( bodyId, &shapeDef, &box.base );
			}
		}
	}
}

typedef struct SceneCast
{
	b3Pos point;
	b3Vec3 normal;
	float fraction;
	int shapeIndex;
	bool hit;
} SceneCast;

static float Scene_ClosestCastCallback( b3ShapeId shapeId, b3Pos point, b3Vec3 normal,
										float fraction, uint64_t userMaterialId, int triangleIndex,
										int childIndex, void* context )
{
	(void)userMaterialId;
	(void)triangleIndex;
	(void)childIndex;

	// Clipping makes each report closer than the last, so the final one is the closest
	SceneCast* cast = context;
	cast->point = point;
	cast->normal = normal;
	cast->fraction = fraction;
	cast->shapeIndex = shapeId.index1;
	cast->hit = true;
	return fraction;
}

static void Scene_HashCast( Scene* scene, int kind, SceneCast* cast )
{
	if ( cast->hit == false )
	{
		Scene_HashQuery( scene, kind, 0 );
		return;
	}

	scene->queryHits[kind] += 1;
	Scene_HashQuery( scene, kind, (uint32_t)cast->shapeIndex );
	Scene_HashQuery( scene, kind, Scene_FloatBits( cast->fraction ) );
	Scene_HashVec3( scene, kind, cast->point );
	Scene_HashVec3( scene, kind, cast->normal );
}

// shared/benchmarks.c StepVillage: a curtain of tilted sphere casts, one per meter, sweeps across
// the compound once per 600 steps
static void Scene_StepVillage( Scene* scene, int stepIndex )
{
	uint64_t ticks = b3GetTicks();

	int villageStepCount = 600;
	float castRadius = 0.05f;
	float castHeight = 10.0f;
	float castLength = 12.0f;

	float halfSpan = scene->halfSpan;
	float t = (float)( stepIndex % villageStepCount ) / (float)( villageStepCount - 1 );
	float x = -halfSpan + 2.0f * halfSpan * t;

	b3CosSin tilt = b3ComputeCosSin( ( 20.0f / 180.0f ) * B3_PI );
	b3Vec3 localTranslation;
	localTranslation.x = castLength * tilt.sine;
	localTranslation.y = -castLength * tilt.cosine;
	localTranslation.z = 0.0f;

	// The curtain is laid out in the compound frame
	b3Quat q = scene->compoundRotation;
	b3Vec3 translation = b3RotateVector( q, localTranslation );

	b3Vec3 point = b3Vec3_zero;
	b3ShapeProxy proxy = { &point, 1, castRadius };
	b3QueryFilter filter = b3DefaultQueryFilter();

	int castCount = (int)( 2.0f * halfSpan ) + 1;

	for ( int i = 0; i < castCount; ++i )
	{
		b3Vec3 localOrigin = { x, castHeight, -halfSpan + (float)i };

		SceneCast cast = { 0 };
		cast.fraction = 1.0f;
		b3Pos origin = b3ToPos( b3RotateVector( q, localOrigin ) );

		b3World_CastShape( scene->worldId, origin, &proxy, translation, filter,
						   Scene_ClosestCastCallback, &cast );
		Scene_HashCast( scene, SCENE_QUERY_SHAPE_CAST, &cast );
	}

	scene->queryMs[SCENE_QUERY_SHAPE_CAST] += b3GetMilliseconds( ticks );
}

// A query workload over a static world with every shape type: a rotated village compound, a wave
// mesh, a height field, single hulls, capsules and spheres on a ground box, and a few dynamic
// bodies resting on them. Each step runs closest ray casts, all-hit ray casts, sphere and box
// casts, capsule overlaps and capsule movers, timing and hashing each kind.
static void CreateQueries( Scene* scene )
{
	scene->seed = 1234567u;

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 0.0f, -2.0f, 10.0f };
		b3BodyId groundId = Scene_AddBody( scene, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( 100.0f, 0.5f, 70.0f );
		b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	Scene_CreateVillageGround( scene, 16, 0.3f );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 52.0f, 0.0f, 0.0f };
		b3BodyId meshBodyId = Scene_AddBody( scene, &bodyDef );
		scene->mesh = b3CreateWaveMesh( 32, 32, 1.0f, 0.5f, 0.1f, 0.2f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateMeshShape( meshBodyId, &shapeDef, scene->mesh, b3Vec3_one );
	}

	{
		b3Vec3 fieldScale = { 1.0f, 0.6f, 1.0f };
		scene->heightField = b3CreateWave( 21, 21, fieldScale, 0.08f, 0.06f, false );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ -72.0f, 0.0f, -10.0f };
		b3BodyId fieldBodyId = Scene_AddBody( scene, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHeightFieldShape( fieldBodyId, &shapeDef, scene->heightField );
	}

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3Capsule capsule = { { 0.0f, -0.5f, 0.0f }, { 0.0f, 0.5f, 0.0f }, 0.4f };
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };

	{
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		for ( int i = 0; i < 8; ++i )
		{
			for ( int j = 0; j < 4; ++j )
			{
				b3BodyDef bodyDef = b3DefaultBodyDef();
				bodyDef.position.x = -28.0f + 8.0f * (float)i;
				bodyDef.position.y = -1.0f;
				bodyDef.position.z = 44.0f + 6.0f * (float)j;
				b3BodyId bodyId = Scene_AddBody( scene, &bodyDef );

				int kind = ( i + j ) % 3;
				if ( kind == 0 )
				{
					b3CreateHullShape( bodyId, &shapeDef, &box.base );
				}
				else if ( kind == 1 )
				{
					b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
				}
				else
				{
					b3CreateSphereShape( bodyId, &shapeDef, &sphere );
				}
			}
		}
	}

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		for ( int i = 0; i < 10; ++i )
		{
			for ( int j = 0; j < 10; ++j )
			{
				bodyDef.position.x = -27.0f + 6.0f * (float)i;
				bodyDef.position.y = Scene_RandomFloat( scene, 2.0f, 5.0f );
				bodyDef.position.z = -27.0f + 6.0f * (float)j;
				b3BodyId bodyId = Scene_AddBody( scene, &bodyDef );

				int kind = ( i * 10 + j ) % 3;
				if ( kind == 0 )
				{
					b3CreateHullShape( bodyId, &shapeDef, &box.base );
				}
				else if ( kind == 1 )
				{
					b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
				}
				else
				{
					b3CreateSphereShape( bodyId, &shapeDef, &sphere );
				}
			}
		}
	}

	scene->moverCount = SCENE_QUERY_MOVER_COUNT;
	for ( int m = 0; m < SCENE_QUERY_MOVER_COUNT; ++m )
	{
		b3Pos position;
		position.x = -62.0f + 16.0f * (float)m;
		position.y = 3.0f;
		position.z = -4.0f + 2.0f * (float)m;
		Scene_InitializeMover( scene->movers + m, position );
	}
}

typedef struct SceneRayAll
{
	Scene* scene;
} SceneRayAll;

static float Scene_RayAllCallback( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction,
								   uint64_t userMaterialId, int triangleIndex, int childIndex,
								   void* context )
{
	(void)point;
	(void)userMaterialId;
	Scene* scene = context;
	scene->queryHits[SCENE_QUERY_RAY_ALL] += 1;
	Scene_HashQuery( scene, SCENE_QUERY_RAY_ALL, (uint32_t)shapeId.index1 );
	Scene_HashQuery( scene, SCENE_QUERY_RAY_ALL, (uint32_t)triangleIndex );
	Scene_HashQuery( scene, SCENE_QUERY_RAY_ALL, (uint32_t)childIndex );
	Scene_HashQuery( scene, SCENE_QUERY_RAY_ALL, Scene_FloatBits( fraction ) );
	Scene_HashVec3( scene, SCENE_QUERY_RAY_ALL, normal );
	return 1.0f;
}

static bool Scene_OverlapCallback( b3ShapeId shapeId, void* context )
{
	Scene* scene = context;
	scene->queryHits[SCENE_QUERY_OVERLAP] += 1;
	Scene_HashQuery( scene, SCENE_QUERY_OVERLAP, (uint32_t)shapeId.index1 );
	return true;
}

static void Scene_StepQueries( Scene* scene, int stepIndex, float timeStep )
{
	b3WorldId worldId = scene->worldId;
	b3QueryFilter filter = b3DefaultQueryFilter();
	float phase = (float)( stepIndex % 64 ) / 64.0f;

	{
		uint64_t ticks = b3GetTicks();
		b3Vec3 translation = { 3.0f, -30.0f, 2.0f };
		for ( int i = 0; i < 16; ++i )
		{
			for ( int j = 0; j < 16; ++j )
			{
				b3Pos origin;
				origin.x = -80.0f + 10.0f * ( (float)i + phase );
				origin.y = 15.0f;
				origin.z = -40.0f + 7.0f * ( (float)j + phase );
				b3RayResult result = b3World_CastRayClosest( worldId, origin, translation, filter );

				SceneCast cast = { 0 };
				cast.point = result.point;
				cast.normal = result.normal;
				cast.fraction = result.fraction;
				cast.shapeIndex = result.shapeId.index1;
				cast.hit = result.hit;
				Scene_HashCast( scene, SCENE_QUERY_RAY, &cast );
			}
		}
		scene->queryMs[SCENE_QUERY_RAY] += b3GetMilliseconds( ticks );
	}

	{
		uint64_t ticks = b3GetTicks();
		b3Vec3 translation = { 180.0f, 0.0f, 0.0f };
		for ( int k = 0; k < 32; ++k )
		{
			b3Pos origin;
			origin.x = -90.0f;
			origin.y = 0.3f + 0.1f * (float)( k % 8 );
			origin.z = -40.0f + 3.5f * ( (float)k + phase );
			b3World_CastRay( worldId, origin, translation, filter, Scene_RayAllCallback, scene );
		}
		scene->queryMs[SCENE_QUERY_RAY_ALL] += b3GetMilliseconds( ticks );
	}

	{
		uint64_t ticks = b3GetTicks();
		b3Vec3 spherePoint = b3Vec3_zero;
		b3ShapeProxy sphereProxy = { &spherePoint, 1, 0.3f };

		b3Vec3 boxPoints[8];
		for ( int n = 0; n < 8; ++n )
		{
			boxPoints[n].x = ( n & 1 ) ? 0.25f : -0.25f;
			boxPoints[n].y = ( n & 2 ) ? 0.25f : -0.25f;
			boxPoints[n].z = ( n & 4 ) ? 0.25f : -0.25f;
		}
		b3ShapeProxy boxProxy = { boxPoints, 8, 0.0f };

		b3Vec3 translation = { 0.0f, -20.0f, 1.5f };
		for ( int k = 0; k < 128; ++k )
		{
			b3Pos origin;
			origin.x = -80.0f + 1.25f * (float)k + 5.0f * phase;
			origin.y = 8.0f;
			origin.z = -30.0f + 0.7f * (float)k;

			SceneCast cast = { 0 };
			cast.fraction = 1.0f;
			const b3ShapeProxy* proxy = ( k & 1 ) ? &boxProxy : &sphereProxy;
			b3World_CastShape( worldId, origin, proxy, translation, filter,
							   Scene_ClosestCastCallback, &cast );
			Scene_HashCast( scene, SCENE_QUERY_SHAPE_CAST, &cast );
		}
		scene->queryMs[SCENE_QUERY_SHAPE_CAST] += b3GetMilliseconds( ticks );
	}

	{
		uint64_t ticks = b3GetTicks();
		b3Vec3 capsulePoints[2] = { { 0.0f, -0.4f, 0.0f }, { 0.0f, 0.4f, 0.0f } };
		b3ShapeProxy proxy = { capsulePoints, 2, 0.3f };
		for ( int k = 0; k < 64; ++k )
		{
			b3Pos origin;
			origin.x = -78.0f + 2.4f * (float)k + 3.0f * phase;
			origin.y = 0.5f;
			origin.z = -20.0f + 1.1f * (float)k;
			b3World_OverlapShape( worldId, origin, &proxy, filter, Scene_OverlapCallback, scene );
		}
		scene->queryMs[SCENE_QUERY_OVERLAP] += b3GetMilliseconds( ticks );
	}

	{
		uint64_t ticks = b3GetTicks();
		for ( int m = 0; m < scene->moverCount; ++m )
		{
			SceneMover* mover = scene->movers + m;
			b3CosSin cs = b3ComputeCosSin( 0.785f * (float)m );
			b3Vec3 forward = { cs.cosine, 0.0f, cs.sine };
			b3Vec3 right = { -cs.sine, 0.0f, cs.cosine };
			b3Vec2 throttle = { 1.0f, 0.0f };
			if ( stepIndex % 120 >= 60 )
			{
				throttle.x = 0.5f;
				throttle.y = 1.0f;
			}

			if ( stepIndex % 90 == 45 && mover->onGround == true )
			{
				mover->velocity.y = s_moverJumpSpeed;
				mover->onGround = false;
			}

			Scene_SolveMove( scene, mover, timeStep, forward, right, throttle, true );

			Scene_HashVec3( scene, SCENE_QUERY_MOVER, mover->transform.p );
			Scene_HashVec3( scene, SCENE_QUERY_MOVER, mover->velocity );
			Scene_HashQuery( scene, SCENE_QUERY_MOVER, (uint32_t)mover->totalIterations );
			scene->queryHits[SCENE_QUERY_MOVER] += mover->planeCount;
		}
		scene->queryMs[SCENE_QUERY_MOVER] += b3GetMilliseconds( ticks );
	}
}

bool Scene_Create( Scene* scene, const char* name, int workerCount )
{
	memset( scene, 0, sizeof( Scene ) );
	scene->name = name;
	scene->bulletId = b3_nullBodyId;

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;
	scene->worldId = b3CreateWorld( &worldDef );

	if ( strcmp( name, "pyramid" ) == 0 )
	{
		CreatePyramid( scene, 10 );
	}
	else if ( strcmp( name, "large_pyramid" ) == 0 )
	{
		CreatePyramid( scene, 100 );
	}
	else if ( strcmp( name, "card_house" ) == 0 )
	{
		CreateCardHouse( scene );
	}
	else if ( strcmp( name, "chains" ) == 0 )
	{
		CreateChains( scene );
	}
	else if ( strcmp( name, "capsule_mesh" ) == 0 )
	{
		CreateCapsuleMesh( scene );
	}
	else if ( strcmp( name, "heightfield" ) == 0 )
	{
		CreateHeightField( scene );
	}
	else if ( strcmp( name, "bullets" ) == 0 )
	{
		CreateBullets( scene );
	}
	else if ( strcmp( name, "mover" ) == 0 )
	{
		CreateMover( scene );
	}
	else if ( strcmp( name, "many_bodies" ) == 0 )
	{
		CreateManyPyramids( scene );
	}
	else if ( strcmp( name, "mesh_drop" ) == 0 )
	{
		CreateMeshDrop( scene );
	}
	else if ( strcmp( name, "village" ) == 0 )
	{
		CreateVillage( scene );
	}
	else if ( strcmp( name, "queries" ) == 0 )
	{
		CreateQueries( scene );
	}
	else
	{
		b3DestroyWorld( scene->worldId );
		scene->worldId = b3_nullWorldId;
		return false;
	}

	return true;
}

void Scene_Step( Scene* scene, int stepIndex )
{
	float timeStep = 1.0f / 60.0f;

	if ( strcmp( scene->name, "bullets" ) == 0 && ( stepIndex == 20 || stepIndex == 80 ) )
	{
		LaunchBullet( scene );
	}

	if ( scene->hasMover )
	{
		Scene_StepMover( scene, stepIndex, timeStep );
	}

	if ( strcmp( scene->name, "village" ) == 0 )
	{
		Scene_StepVillage( scene, stepIndex );
	}
	else if ( strcmp( scene->name, "queries" ) == 0 )
	{
		Scene_StepQueries( scene, stepIndex, timeStep );
	}

	b3World_Step( scene->worldId, timeStep, SCENE_SUBSTEP_COUNT );
}

void Scene_Destroy( Scene* scene )
{
	b3DestroyWorld( scene->worldId );
	scene->worldId = b3_nullWorldId;

	if ( scene->mesh != NULL )
	{
		b3DestroyMesh( scene->mesh );
		scene->mesh = NULL;
	}

	if ( scene->heightField != NULL )
	{
		b3DestroyHeightField( scene->heightField );
		scene->heightField = NULL;
	}

	if ( scene->compound != NULL )
	{
		b3DestroyCompound( scene->compound );
		scene->compound = NULL;
	}

	free( scene->bodies );
	scene->bodies = NULL;
	scene->bodyCount = 0;
	scene->bodyCapacity = 0;
}
