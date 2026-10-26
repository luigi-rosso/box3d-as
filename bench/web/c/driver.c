// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Native Box3D on wasm, driven from JS: the same exports as bench/web/as/web.as.

#include "scenes.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

#include <string.h>

static Scene s_scene;
static bool s_live = false;

EMSCRIPTEN_KEEPALIVE int sceneCount( void )
{
	return Scene_GetCount();
}

EMSCRIPTEN_KEEPALIVE int create( int sceneIndex, int workerCount )
{
	if ( s_live )
	{
		Scene_Destroy( &s_scene );
		s_live = false;
	}
	if ( sceneIndex < 0 || sceneIndex >= Scene_GetCount() )
	{
		return 0;
	}
	s_live = Scene_Create( &s_scene, Scene_GetName( sceneIndex ), workerCount );
	return s_live ? 1 : 0;
}

EMSCRIPTEN_KEEPALIVE void step( int stepIndex )
{
	Scene_Step( &s_scene, stepIndex );
}

static uint32_t HashWord( uint32_t h, uint32_t bits )
{
	for ( int s = 0; s < 4; ++s )
	{
		h = ( h ^ ( ( bits >> ( 8 * s ) ) & 0xff ) ) * 16777619u;
	}
	return h;
}

static uint32_t HashFloat( uint32_t h, float x )
{
	uint32_t bits;
	memcpy( &bits, &x, sizeof( bits ) );
	return HashWord( h, bits );
}

// FNV-1a over each body's pose and velocities as verify/scenes/hash.mjs reads a recording; a destroyed
// body hashes the recording's zero words
EMSCRIPTEN_KEEPALIVE uint32_t stateHash( void )
{
	uint32_t h = 2166136261u;
	for ( int i = 0; i < s_scene.bodyCount; ++i )
	{
		b3BodyId id = s_scene.bodies[i];
		if ( b3Body_IsValid( id ) == false )
		{
			for ( int k = 0; k < 13; ++k )
			{
				h = HashWord( h, 0 );
			}
			continue;
		}
		b3Vec3 p = b3Body_GetPosition( id );
		b3Quat q = b3Body_GetRotation( id );
		b3Vec3 v = b3Body_GetLinearVelocity( id );
		b3Vec3 w = b3Body_GetAngularVelocity( id );
		float words[13] = { p.x, p.y, p.z, q.v.x, q.v.y, q.v.z, q.s, v.x, v.y, v.z, w.x, w.y, w.z };
		for ( int k = 0; k < 13; ++k )
		{
			h = HashFloat( h, words[k] );
		}
	}
	return h;
}

// The b3Profile fields in the order of bench/native/bench.c
EMSCRIPTEN_KEEPALIVE float profileField( int k )
{
	b3Profile p = b3World_GetProfile( s_scene.worldId );
	float fields[23] = {
		p.step,		  p.pairs,		 p.collide,			   p.solve,
		p.solverSetup, p.constraints, p.prepareConstraints, p.integrateVelocities,
		p.warmStart,   p.solveImpulses, p.integratePositions, p.relaxImpulses,
		p.restitution, p.storeImpulses, p.splitIslands,		   p.transforms,
		p.sensorHits,  p.jointEvents,  p.hitEvents,			   p.refit,
		p.bullets,	   p.sleepIslands, p.sensors,
	};
	return 0 <= k && k < 23 ? fields[k] : 0.0f;
}

EMSCRIPTEN_KEEPALIVE double queryMs( int k )
{
	return s_scene.queryMs[k];
}

EMSCRIPTEN_KEEPALIVE uint32_t queryHash( int k )
{
	return s_scene.queryHash[k];
}

EMSCRIPTEN_KEEPALIVE int bodyCount( void )
{
	return s_scene.bodyCount;
}
