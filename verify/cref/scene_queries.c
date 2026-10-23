// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// The queries and village scenes of verify/scenes: after every step, each query kind's result hash
// and hit count, and the movers' poses.

#include "common.h"
#include "scenes.h"

static void PrintQueries( const Scene* scene, int stepIndex )
{
	vtag( "step" );
	vi32( stepIndex );
	for ( int k = 0; k < SCENE_QUERY_KIND_COUNT; ++k )
	{
		vu32( scene->queryHash[k] );
		vi32( scene->queryHits[k] );
	}
	vend();

	for ( int m = 0; m < scene->moverCount; ++m )
	{
		const SceneMover* mover = scene->movers + m;
		vtag( "mover" );
		vi32( m );
		vf32( mover->transform.p.x );
		vf32( mover->transform.p.y );
		vf32( mover->transform.p.z );
		vf32( mover->velocity.x );
		vf32( mover->velocity.y );
		vf32( mover->velocity.z );
		vbool( mover->onGround );
		vi32( mover->planeCount );
		vend();
	}
}

int main( void )
{
	const char* names[2] = { "queries", "village" };
	int steps[2] = { 200, 40 };
	for ( int s = 0; s < 2; ++s )
	{
		Scene scene;
		Scene_Create( &scene, names[s], 1 );
		for ( int i = 0; i < steps[s]; ++i )
		{
			Scene_Step( &scene, i );
			PrintQueries( &scene, i );
		}
		Scene_Destroy( &scene );
	}
	return 0;
}
