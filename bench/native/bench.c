// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Native step timing: bench <scene> <steps> <workers> <repeats>
// Each repeat builds the scene, takes one untimed step as upstream's benchmark does, then times the
// rest. Reports ms per step (median and min over repeats) and the b3Profile stages, mean per step,
// min over repeats.

#include "scenes.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
	STAGE_COUNT = 23,
	MAX_REPEATS = 64,
};

static const char* s_stageNames[STAGE_COUNT] = {
	"step",
	"pairs",
	"collide",
	"solve",
	"solverSetup",
	"constraints",
	"prepareConstraints",
	"integrateVelocities",
	"warmStart",
	"solveImpulses",
	"integratePositions",
	"relaxImpulses",
	"restitution",
	"storeImpulses",
	"splitIslands",
	"transforms",
	"sensorHits",
	"jointEvents",
	"hitEvents",
	"refit",
	"bullets",
	"sleepIslands",
	"sensors",
};

static void ProfileToArray( const b3Profile* p, float* out )
{
	out[0] = p->step;
	out[1] = p->pairs;
	out[2] = p->collide;
	out[3] = p->solve;
	out[4] = p->solverSetup;
	out[5] = p->constraints;
	out[6] = p->prepareConstraints;
	out[7] = p->integrateVelocities;
	out[8] = p->warmStart;
	out[9] = p->solveImpulses;
	out[10] = p->integratePositions;
	out[11] = p->relaxImpulses;
	out[12] = p->restitution;
	out[13] = p->storeImpulses;
	out[14] = p->splitIslands;
	out[15] = p->transforms;
	out[16] = p->sensorHits;
	out[17] = p->jointEvents;
	out[18] = p->hitEvents;
	out[19] = p->refit;
	out[20] = p->bullets;
	out[21] = p->sleepIslands;
	out[22] = p->sensors;
}

static int CompareDouble( const void* a, const void* b )
{
	double x = *(const double*)a;
	double y = *(const double*)b;
	return x < y ? -1 : ( x > y ? 1 : 0 );
}

int main( int argc, char** argv )
{
	if ( argc != 5 )
	{
		fprintf( stderr, "usage: bench <scene> <steps> <workers> <repeats>\n" );
		return 1;
	}

	const char* sceneName = argv[1];
	int stepCount = atoi( argv[2] );
	int workerCount = atoi( argv[3] );
	int repeatCount = atoi( argv[4] );
	if ( stepCount < 2 || repeatCount < 1 || repeatCount > MAX_REPEATS )
	{
		fprintf( stderr, "need steps >= 2 and 1 <= repeats <= %d\n", MAX_REPEATS );
		return 1;
	}

	double msPerStep[MAX_REPEATS];
	double stageMin[STAGE_COUNT];
	for ( int k = 0; k < STAGE_COUNT; ++k )
	{
		stageMin[k] = 1e30;
	}

	int bodyCount = 0;
	int contactCount = 0;

	for ( int r = 0; r < repeatCount; ++r )
	{
		Scene scene;
		if ( Scene_Create( &scene, sceneName, workerCount ) == false )
		{
			fprintf( stderr, "unknown scene %s\n", sceneName );
			return 1;
		}

		Scene_Step( &scene, 0 );

		double stageSum[STAGE_COUNT] = { 0 };
		uint64_t ticks = b3GetTicks();
		for ( int i = 1; i < stepCount; ++i )
		{
			Scene_Step( &scene, i );

			b3Profile profile = b3World_GetProfile( scene.worldId );
			float stages[STAGE_COUNT];
			ProfileToArray( &profile, stages );
			for ( int k = 0; k < STAGE_COUNT; ++k )
			{
				stageSum[k] += stages[k];
			}
		}
		double totalMs = b3GetMilliseconds( ticks );

		int timedSteps = stepCount - 1;
		msPerStep[r] = totalMs / timedSteps;
		for ( int k = 0; k < STAGE_COUNT; ++k )
		{
			double mean = stageSum[k] / timedSteps;
			stageMin[k] = mean < stageMin[k] ? mean : stageMin[k];
		}

		b3Counters counters = b3World_GetCounters( scene.worldId );
		bodyCount = counters.bodyCount;
		contactCount = counters.contactCount;

		printf( "run %d: %.3f ms total, %.4f ms/step\n", r, totalMs, msPerStep[r] );
		Scene_Destroy( &scene );
	}

	double sorted[MAX_REPEATS];
	memcpy( sorted, msPerStep, repeatCount * sizeof( double ) );
	qsort( sorted, repeatCount, sizeof( double ), CompareDouble );
	double median = ( repeatCount & 1 )
						? sorted[repeatCount / 2]
						: 0.5 * ( sorted[repeatCount / 2 - 1] + sorted[repeatCount / 2] );

	printf(
		"RESULT scene=%s workers=%d steps=%d repeats=%d bodies=%d contacts=%d median=%.4f min=%.4f",
		sceneName, workerCount, stepCount, repeatCount, bodyCount, contactCount, median,
		sorted[0] );
	for ( int k = 0; k < STAGE_COUNT; ++k )
	{
		printf( " %s=%.4f", s_stageNames[k], stageMin[k] );
	}
	printf( "\n" );
	return 0;
}
