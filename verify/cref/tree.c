// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/dynamic_tree.as, mirrored by verify/as/tree.as

#include "common.h"

#include "aabb.h"
#include "dynamic_tree.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>

#define MAX_LIVE 4096
#define PHASE_COUNT 10

static b3DynamicTree tree;
static bool treeAlive;
static int live[MAX_LIVE];
static int liveCount;
static int lastLeafCount;

static int boxMode;
static float worldSize;
static float boxSize;
static int createWeight;
static int destroyWeight;
static int opsLo;
static int opsHi;
static int phaseIndex;

// FNV-1a over the tree state
static uint32_t h_state;

static void hmix( uint32_t x )
{
	h_state ^= x;
	h_state *= 16777619u;
}

static uint32_t fbits( float x )
{
	uint32_t bits;
	memcpy( &bits, &x, sizeof( bits ) );
	return bits;
}

static void hmixf( float x )
{
	hmix( fbits( x ) );
}

static void hmix64( uint64_t x )
{
	hmix( (uint32_t)x );
	hmix( (uint32_t)( x >> 32 ) );
}

static void hmixAABB( b3AABB a )
{
	hmixf( a.lowerBound.x );
	hmixf( a.lowerBound.y );
	hmixf( a.lowerBound.z );
	hmixf( a.upperBound.x );
	hmixf( a.upperBound.y );
	hmixf( a.upperBound.z );
}

static uint32_t treeHash( void )
{
	h_state = 2166136261u;
	hmix( (uint32_t)tree.nodeEnd );
	hmix( (uint32_t)tree.nodeCapacity );
	hmix( (uint32_t)tree.pairFreeList );
	hmix( (uint32_t)tree.proxyCount );
	hmix( (uint32_t)tree.proxyCapacity );
	hmix( (uint32_t)tree.proxyFreeList );
	hmix( tree.dfsOrdered ? 1u : 0u );
	for ( int i = 0; i < tree.nodeEnd; ++i )
	{
		const b3TreeNode* n = tree.nodes + i;
		hmixAABB( n->aabb );
		hmix( n->flagIndex );
		hmix( (uint32_t)n->height );
		hmix( (uint32_t)tree.parents[i] );
	}
	for ( int i = 0; i < tree.proxyCapacity; ++i )
	{
		const b3TreeProxy* p = tree.proxies + i;
		hmix64( p->userData );
		hmix64( p->categoryBits );
		hmix( (uint32_t)p->node );
		hmix( (uint32_t)p->next );
	}
	return h_state;
}

static void vaabb( b3AABB a )
{
	vf32( a.lowerBound.x );
	vf32( a.lowerBound.y );
	vf32( a.lowerBound.z );
	vf32( a.upperBound.x );
	vf32( a.upperBound.y );
	vf32( a.upperBound.z );
}

static void vvec( b3Vec3 v )
{
	vf32( v.x );
	vf32( v.y );
	vf32( v.z );
}

static void vstats( const char* tag, b3TreeStats s )
{
	vtag( tag );
	vi32( s.nodeVisits );
	vi32( s.leafVisits );
	vend();
}

// The union reads as height; the AS side prints the member the node uses and counts the other one
static void vnode( const b3TreeNode* n )
{
	vaabb( n->aabb );
	vu32( n->flagIndex );
	vi32( n->height );
}

static void emitHeader( const char* tag )
{
	vtag( "tree" );
	vtag( tag );
	vu64( tree.version );
	vi32( tree.nodeEnd );
	vi32( tree.nodeCapacity );
	vi32( tree.pairFreeList );
	vi32( tree.proxyCount );
	vi32( tree.proxyCapacity );
	vi32( tree.proxyFreeList );
	vi32( tree.rebuildCapacity );
	vbool( tree.dfsOrdered );
	vbool( tree.swapNodes != NULL );
	vend();
}

static void emitStats( void )
{
	vtag( "stats" );
	vi32( b3DynamicTree_GetHeight( &tree ) );
	vf32( b3DynamicTree_GetAreaRatio( &tree ) );
	vaabb( b3DynamicTree_GetRootBounds( &tree ) );
	vi32( b3DynamicTree_GetProxyCount( &tree ) );
	vi32( b3DynamicTree_GetByteCount( &tree ) );
	vbool( b3HasTreeMoved( &tree ) );
	vbool( b3NeedsRebuild( &tree ) );
	vend();
}

// Every slot of every array, the stale tails included
static void emitTree( const char* tag )
{
	emitHeader( tag );
	emitStats();

	for ( int i = 0; i < tree.nodeCapacity; ++i )
	{
		vtag( "n" );
		vi32( i );
		vnode( tree.nodes + i );
		vi32( tree.parents[i] );
		vend();
	}

	if ( tree.swapNodes != NULL )
	{
		for ( int i = 0; i < tree.nodeCapacity; ++i )
		{
			vtag( "s" );
			vi32( i );
			vnode( tree.swapNodes + i );
			vend();
		}
	}

	for ( int i = 0; i < tree.proxyCapacity; ++i )
	{
		const b3TreeProxy* p = tree.proxies + i;
		vtag( "p" );
		vi32( i );
		vu64( p->userData );
		vu64( p->categoryBits );
		vi32( p->node );
		vi32( p->next );
		vend();
	}

	for ( int i = 0; i < lastLeafCount; ++i )
	{
		vtag( "l" );
		vi32( i );
		vi32( tree.leafIndices[i] );
		vnode( tree.leafNodes + i );
		vvec( tree.leafCenters[i] );
		vend();
	}

	vtag( "alias" );
	vi32( 0 );
	vend();
}

static void emitOp( const char* tag, int a, int b )
{
	vtag( tag );
	vi32( a );
	vi32( b );
	vu32( treeHash() );
	vend();
}

static b3AABB makeBox( b3Vec3 c, b3Vec3 e )
{
	b3AABB a = { b3Sub( c, e ), b3Add( c, e ) };
	return a;
}

static b3AABB randomBox( void )
{
	b3Vec3 c, e;
	int mode = boxMode;
	if ( mode == 5 && vrandint( 0, 8 ) == 0 )
	{
		mode = 0;
	}

	if ( mode == 0 )
	{
		c.x = vrandf( -worldSize, worldSize );
		c.y = vrandf( -worldSize, worldSize );
		c.z = vrandf( -worldSize, worldSize );
		e.x = vrandf( 0.05f, boxSize );
		e.y = vrandf( 0.05f, boxSize );
		e.z = vrandf( 0.05f, boxSize );
	}
	else if ( mode == 1 )
	{
		c.x = (float)vrandint( -6, 7 );
		c.y = (float)vrandint( -6, 7 );
		c.z = (float)vrandint( -6, 7 );
		e.x = 0.5f * (float)vrandint( 1, 3 );
		e.y = 0.5f * (float)vrandint( 1, 3 );
		e.z = 0.5f * (float)vrandint( 1, 3 );
	}
	else if ( mode == 2 )
	{
		c.x = vrandf( -worldSize, worldSize );
		c.y = vrandf( -worldSize, worldSize );
		c.z = vrandf( -worldSize, worldSize );
		e.x = vrandint( 0, 2 ) == 0 ? 0.0f : vrandf( 0.0f, boxSize );
		e.y = vrandint( 0, 2 ) == 0 ? 0.0f : vrandf( 0.0f, boxSize );
		e.z = vrandint( 0, 2 ) == 0 ? 0.0f : vrandf( 0.0f, boxSize );
	}
	else if ( mode == 3 )
	{
		c.x = vrandf( -worldSize, worldSize );
		c.y = 0.0f;
		c.z = 0.0f;
		e.x = vrandf( 0.05f, boxSize );
		e.y = vrandf( 0.05f, boxSize );
		e.z = vrandf( 0.05f, boxSize );
	}
	else if ( mode == 4 )
	{
		c.x = 100000.0f + vrandf( -worldSize, worldSize );
		c.y = -50000.0f + vrandf( -worldSize, worldSize );
		c.z = vrandf( -worldSize, worldSize );
		e.x = vrandf( 0.01f, boxSize );
		e.y = vrandf( 0.01f, boxSize );
		e.z = vrandf( 0.01f, boxSize );
	}
	else
	{
		c = b3Vec3_zero;
		e = (b3Vec3){ 1.0f, 1.0f, 1.0f };
	}

	return makeBox( c, e );
}

static bool aimAtProxy( b3Vec3* target )
{
	if ( liveCount == 0 || vrandint( 0, 2 ) == 0 )
	{
		return false;
	}

	*target = b3AABB_Center( b3DynamicTree_GetAABB( &tree, live[vrandint( 0, liveCount )] ) );
	return true;
}

static b3AABB randomQueryBox( void )
{
	b3Vec3 c, e;
	if ( aimAtProxy( &c ) == false )
	{
		c.x = vrandf( -worldSize, worldSize );
		c.y = vrandf( -worldSize, worldSize );
		c.z = vrandf( -worldSize, worldSize );
		if ( boxMode == 4 )
		{
			c.x += 100000.0f;
			c.y -= 50000.0f;
		}
	}
	float s = vrandf( 0.0f, 0.5f * worldSize );
	e.x = vrandf( 0.0f, s );
	e.y = vrandf( 0.0f, s );
	e.z = vrandf( 0.0f, s );
	return makeBox( c, e );
}

static b3Vec3 randomPoint( void )
{
	b3Vec3 p;
	p.x = vrandf( -1.2f * worldSize, 1.2f * worldSize );
	p.y = vrandf( -1.2f * worldSize, 1.2f * worldSize );
	p.z = vrandf( -1.2f * worldSize, 1.2f * worldSize );
	if ( boxMode == 4 )
	{
		p.x += 100000.0f;
		p.y -= 50000.0f;
	}
	return p;
}

static uint64_t randomBits( void )
{
	int pick = vrandint( 0, 4 );
	if ( pick == 0 )
	{
		return B3_DEFAULT_CATEGORY_BITS;
	}
	if ( pick == 1 )
	{
		int shift = vrandint( 0, 64 );
		return 1ull << shift;
	}
	if ( pick == 2 )
	{
		return vrand64();
	}
	return 3;
}

static b3AABB enlarged( b3AABB a )
{
	float g = 0.25f * boxSize;
	float d0 = vrandf( 0.0f, g );
	float d1 = vrandf( 0.0f, g );
	float d2 = vrandf( 0.0f, g );
	float d3 = vrandf( 0.0f, g );
	float d4 = vrandf( 0.0f, g );
	float d5 = vrandf( 0.0f, g );
	int skip = vrandint( 0, 6 );
	a.lowerBound.x -= skip == 0 ? 0.0f : d0;
	a.lowerBound.y -= skip == 1 ? 0.0f : d1;
	a.lowerBound.z -= skip == 2 ? 0.0f : d2;
	a.upperBound.x += skip == 3 ? 0.0f : d3 + 0.001f;
	a.upperBound.y += skip == 4 ? 0.0f : d4;
	a.upperBound.z += skip == 5 ? 0.0f : d5;
	return a;
}

// Query callbacks. All context is global so both sides read it the same way.
static int q_calls;
static int q_stopAfter;
static bool q_nest;
static int q_depth;
static b3Vec3 q_point;
static int q_policy;

static bool nestedQueryCallback( int proxyId, uint64_t userData, void* context )
{
	(void)context;
	vtag( "nq" );
	vi32( proxyId );
	vu64( userData );
	vend();
	return true;
}

static void nestedQuery( void )
{
	q_depth += 1;
	b3AABB box = randomQueryBox();
	b3TreeStats s = b3DynamicTree_Query( &tree, box, B3_DEFAULT_MASK_BITS, false, nestedQueryCallback, NULL );
	vstats( "nqs", s );
	q_depth -= 1;
}

static bool queryCallback( int proxyId, uint64_t userData, void* context )
{
	(void)context;
	vtag( "q" );
	vi32( proxyId );
	vu64( userData );
	vend();
	q_calls += 1;
	if ( q_nest && q_depth == 0 && q_calls % 3 == 1 )
	{
		nestedQuery();
	}
	return q_stopAfter < 0 || q_calls < q_stopAfter;
}

static float nestedClosestCallback( float minSqr, int proxyId, uint64_t userData, void* context )
{
	(void)context;
	vtag( "nk" );
	vf32( minSqr );
	vi32( proxyId );
	vu64( userData );
	vend();
	return FLT_MAX;
}

static float closestCallback( float minSqr, int proxyId, uint64_t userData, void* context )
{
	(void)context;
	b3AABB a = b3DynamicTree_GetAABB( &tree, proxyId );
	float d = b3DistanceSquared( q_point, b3AABB_Center( a ) );
	int pick = vrandint( 0, 8 );
	float value = pick == 0 ? FLT_MAX : d;
	vtag( "k" );
	vf32( minSqr );
	vi32( proxyId );
	vu64( userData );
	vf32( value );
	vend();
	q_calls += 1;
	if ( q_nest && q_depth == 0 && q_calls % 4 == 1 )
	{
		q_depth += 1;
		b3Vec3 p = randomPoint();
		float m = 0.25f * worldSize * worldSize;
		b3TreeStats s = b3DynamicTree_QueryClosest( &tree, p, B3_DEFAULT_MASK_BITS, false, nestedClosestCallback, NULL, &m );
		vtag( "nks" );
		vf32( m );
		vend();
		vstats( "nkst", s );
		q_depth -= 1;
	}
	return value;
}

static float castValue( float maxFraction, bool hit, float fraction )
{
	if ( q_policy == 1 )
	{
		return hit ? fraction : -1.0f;
	}

	if ( q_policy == 2 )
	{
		return maxFraction;
	}

	int pick = vrandint( 0, 16 );
	if ( pick == 0 )
	{
		return 0.0f;
	}
	if ( pick < 4 )
	{
		return -1.0f;
	}
	if ( pick < 8 )
	{
		return maxFraction;
	}
	float t = vrandf( 0.0f, 1.0f );
	return maxFraction * t;
}

static float rayCallback( const b3RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)context;
	float fraction = 0.0f;
	bool hit = false;
	if ( q_policy == 1 )
	{
		b3AABB a = b3DynamicTree_GetAABB( &tree, proxyId );
		float minF = 0.0f;
		float maxF = 0.0f;
		hit = b3RayCastAABB( a, input->origin, b3Add( input->origin, input->translation ), &minF, &maxF );
		fraction = minF;
	}
	float value = castValue( input->maxFraction, hit, fraction );
	vtag( "r" );
	vi32( proxyId );
	vu64( userData );
	vvec( input->origin );
	vvec( input->translation );
	vf32( input->maxFraction );
	vf32( value );
	vend();
	q_calls += 1;
	if ( q_nest && q_depth == 0 && q_calls % 3 == 1 )
	{
		nestedQuery();
	}
	return value;
}

static float boxCallback( const b3BoxCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)context;
	float value = castValue( input->maxFraction, false, 0.0f );
	vtag( "b" );
	vi32( proxyId );
	vu64( userData );
	vaabb( input->box );
	vvec( input->translation );
	vf32( input->maxFraction );
	vf32( value );
	vend();
	q_calls += 1;
	if ( q_nest && q_depth == 0 && q_calls % 3 == 1 )
	{
		nestedQuery();
	}
	return value;
}

// 0 runs through the tree API, 1 through a view without proxies, 2 through a view with proxies
static int pickPath( void )
{
	int pick = vrandint( 0, 8 );
	return pick < 5 ? 0 : ( pick < 7 ? 1 : 2 );
}

static void runQuery( void )
{
	b3AABB box = randomQueryBox();
	uint64_t mask = randomBits();
	bool requireAll = vrandint( 0, 3 ) == 0;
	q_stopAfter = vrandint( 0, 4 ) == 0 ? vrandint( 1, 6 ) : -1;
	q_nest = vrandint( 0, 4 ) == 0;
	q_calls = 0;
	int path = pickPath();
	vtag( "Q" );
	vaabb( box );
	vu64( mask );
	vbool( requireAll );
	vi32( q_stopAfter );
	vbool( q_nest );
	vi32( path );
	vend();
	b3TreeStats s;
	if ( path == 0 )
	{
		s = b3DynamicTree_Query( &tree, box, mask, requireAll, queryCallback, NULL );
	}
	else
	{
		b3TreeView view = { tree.nodes, path == 2 ? tree.proxies : NULL };
		s = b3TreeView_Query( &view, box, mask, requireAll, queryCallback, NULL );
	}
	vstats( "Qs", s );
}

static void runClosest( void )
{
	q_point = randomPoint();
	uint64_t mask = randomBits();
	bool requireAll = vrandint( 0, 3 ) == 0;
	float minSqr = vrandint( 0, 2 ) == 0 ? FLT_MAX : vrandf( 0.0f, worldSize * worldSize );
	q_nest = vrandint( 0, 4 ) == 0;
	q_calls = 0;
	int path = pickPath();
	vtag( "K" );
	vvec( q_point );
	vu64( mask );
	vbool( requireAll );
	vf32( minSqr );
	vbool( q_nest );
	vi32( path );
	vend();
	b3TreeStats s;
	if ( path == 0 )
	{
		s = b3DynamicTree_QueryClosest( &tree, q_point, mask, requireAll, closestCallback, NULL, &minSqr );
	}
	else
	{
		b3TreeView view = { tree.nodes, path == 2 ? tree.proxies : NULL };
		s = b3TreeView_QueryClosest( &view, q_point, mask, requireAll, closestCallback, NULL, &minSqr );
	}
	vtag( "Km" );
	vf32( minSqr );
	vend();
	vstats( "Ks", s );
}

static void runRayCast( void )
{
	b3RayCastInput input;
	input.origin = randomPoint();
	float span = 2.4f * worldSize;
	input.translation.x = vrandf( -span, span );
	input.translation.y = vrandf( -span, span );
	input.translation.z = vrandf( -span, span );
	b3Vec3 target;
	if ( aimAtProxy( &target ) )
	{
		input.translation = b3MulSV( 1.5f, b3Sub( target, input.origin ) );
	}
	int axis = vrandint( 0, 6 );
	if ( axis == 0 )
	{
		// axis aligned rays hit the parallel slab paths
		input.translation.y = 0.0f;
		input.translation.z = 0.0f;
	}
	else if ( axis == 1 )
	{
		input.translation = b3Vec3_zero;
	}
	input.maxFraction = vrandint( 0, 4 ) == 0 ? vrandf( 0.1f, 1.0f ) : 1.0f;
	uint64_t mask = randomBits();
	bool requireAll = vrandint( 0, 3 ) == 0;
	q_policy = vrandint( 0, 3 );
	q_nest = vrandint( 0, 4 ) == 0;
	q_calls = 0;
	int path = pickPath();
	vtag( "R" );
	vvec( input.origin );
	vvec( input.translation );
	vf32( input.maxFraction );
	vu64( mask );
	vbool( requireAll );
	vi32( q_policy );
	vbool( q_nest );
	vi32( path );
	vend();
	b3TreeStats s;
	if ( path == 0 )
	{
		s = b3DynamicTree_RayCast( &tree, &input, mask, requireAll, rayCallback, NULL );
	}
	else
	{
		b3TreeView view = { tree.nodes, path == 2 ? tree.proxies : NULL };
		s = b3TreeView_RayCast( &view, &input, mask, requireAll, rayCallback, NULL );
	}
	vstats( "Rs", s );
}

static void runBoxCast( void )
{
	b3BoxCastInput input;
	b3Vec3 c = randomPoint();
	b3Vec3 e;
	e.x = vrandf( 0.0f, boxSize );
	e.y = vrandf( 0.0f, boxSize );
	e.z = vrandf( 0.0f, boxSize );
	input.box = makeBox( c, e );
	float span = 2.4f * worldSize;
	input.translation.x = vrandf( -span, span );
	input.translation.y = vrandf( -span, span );
	input.translation.z = vrandf( -span, span );
	b3Vec3 target;
	if ( aimAtProxy( &target ) )
	{
		input.translation = b3MulSV( 1.5f, b3Sub( target, c ) );
	}
	if ( vrandint( 0, 6 ) == 0 )
	{
		input.translation = b3Vec3_zero;
	}
	input.maxFraction = vrandint( 0, 4 ) == 0 ? vrandf( 0.1f, 1.0f ) : 1.0f;
	uint64_t mask = randomBits();
	bool requireAll = vrandint( 0, 3 ) == 0;
	q_policy = vrandint( 0, 2 ) == 0 ? 2 : 0;
	q_nest = vrandint( 0, 4 ) == 0;
	q_calls = 0;
	int path = pickPath();
	vtag( "B" );
	vaabb( input.box );
	vvec( input.translation );
	vf32( input.maxFraction );
	vu64( mask );
	vbool( requireAll );
	vi32( q_policy );
	vbool( q_nest );
	vi32( path );
	vend();
	b3TreeStats s;
	if ( path == 0 )
	{
		s = b3DynamicTree_BoxCast( &tree, &input, mask, requireAll, boxCallback, NULL );
	}
	else
	{
		b3TreeView view = { tree.nodes, path == 2 ? tree.proxies : NULL };
		s = b3TreeView_BoxCast( &view, &input, mask, requireAll, boxCallback, NULL );
	}
	vstats( "Bs", s );
}

static void runQueries( int count )
{
	for ( int i = 0; i < count; ++i )
	{
		int kind = vrandint( 0, 4 );
		if ( kind == 0 )
		{
			runQuery();
		}
		else if ( kind == 1 )
		{
			runClosest();
		}
		else if ( kind == 2 )
		{
			runRayCast();
		}
		else
		{
			runBoxCast();
		}
	}
}

static void doRebuild( bool full )
{
	int count = b3DynamicTree_Rebuild( &tree, full );
	if ( count > 0 )
	{
		lastLeafCount = count;
	}
	b3DynamicTree_Validate( &tree );
	emitOp( full ? "rebuildFull" : "rebuild", count, 0 );
}

static void doGather( void )
{
	static int ids[MAX_LIVE];
	int count = b3DynamicTree_GatherMovedProxies( &tree, ids );
	vtag( "gather" );
	vi32( count );
	for ( int i = 0; i < count; ++i )
	{
		vi32( ids[i] );
	}
	vend();
}

static void doCreate( void )
{
	b3AABB box = randomBox();
	uint64_t category = randomBits();
	uint64_t userData = vrand64();
	int internal = vrandint( 0, 3 );
	int proxyId;
	if ( internal == 0 )
	{
		bool mark = vrandint( 0, 2 ) == 1;
		proxyId = b3CreateTreeProxyInternal( &tree, box, category, userData, mark );
	}
	else
	{
		proxyId = b3DynamicTree_CreateProxy( &tree, box, category, userData );
	}
	live[liveCount++] = proxyId;
	emitOp( "create", proxyId, internal );
}

static void doDestroy( int index )
{
	int proxyId = live[index];
	live[index] = live[liveCount - 1];
	liveCount -= 1;
	b3DynamicTree_DestroyProxy( &tree, proxyId );
	emitOp( "destroy", proxyId, liveCount );
}

static void doMove( int proxyId )
{
	b3AABB box = randomBox();
	int internal = vrandint( 0, 3 );
	if ( internal == 0 )
	{
		bool mark = vrandint( 0, 2 ) == 1;
		b3DynamicTree_MoveProxyInternal( &tree, proxyId, box, mark );
	}
	else
	{
		b3DynamicTree_MoveProxy( &tree, proxyId, box );
	}
	emitOp( "move", proxyId, internal );
}

static void doEnlarge( int proxyId )
{
	b3AABB box = enlarged( b3DynamicTree_GetAABB( &tree, proxyId ) );
	b3DynamicTree_EnlargeProxy( &tree, proxyId, box );
	emitOp( "enlarge", proxyId, 0 );
}

static void doMarkMoved( int proxyId )
{
	b3AABB box = enlarged( b3DynamicTree_GetAABB( &tree, proxyId ) );
	b3DynamicTree_MarkProxyMoved( &tree, proxyId, box );
	emitOp( "mark", proxyId, 0 );
}

static void doMarkSerial( int proxyId )
{
	b3DynamicTree_MarkProxyMovedSerial( &tree, proxyId );
	emitOp( "markSerial", proxyId, 0 );
}

static void doSetCategory( int proxyId )
{
	uint64_t bits = randomBits();
	b3DynamicTree_SetCategoryBits( &tree, proxyId, bits );
	vtag( "category" );
	vi32( proxyId );
	vu64( b3DynamicTree_GetCategoryBits( &tree, proxyId ) );
	vu64( b3DynamicTree_GetUserData( &tree, proxyId ) );
	vaabb( b3DynamicTree_GetAABB( &tree, proxyId ) );
	vend();
}

static void createTree( int capacity )
{
	tree = b3DynamicTree_Create( capacity );
	treeAlive = true;
	liveCount = 0;
	lastLeafCount = 0;
}

static void destroyTree( void )
{
	b3DynamicTree_Destroy( &tree );
	treeAlive = false;
	liveCount = 0;
	lastLeafCount = 0;
	emitHeader( "destroyed" );
}

static void createFixed( b3AABB box, uint64_t userData )
{
	int proxyId = b3DynamicTree_CreateProxy( &tree, box, B3_DEFAULT_CATEGORY_BITS, userData );
	live[liveCount++] = proxyId;
	emitOp( "create", proxyId, 0 );
}

static b3AABB unitBox( float x, float y, float z )
{
	return makeBox( (b3Vec3){ x, y, z }, (b3Vec3){ 0.5f, 0.5f, 0.5f } );
}

static void scripted( void )
{
	vseed( 77 );
	boxMode = 0;
	worldSize = 10.0f;
	boxSize = 1.0f;

	// Capacities
	int capacities[] = { -5, 0, 1, 15, 16, 17, 100 };
	for ( int i = 0; i < 7; ++i )
	{
		createTree( capacities[i] );
		emitHeader( "cap" );
		emitStats();
		destroyTree();
	}

	// Empty tree
	createTree( 0 );
	emitTree( "empty" );
	runQueries( 12 );
	doRebuild( false );
	doRebuild( true );
	b3DynamicTree_Refit( &tree );
	b3DynamicTree_ClearMoved( &tree );
	doGather();
	emitTree( "empty2" );

	// One proxy: the root is a leaf
	createFixed( unitBox( 1.0f, 2.0f, 3.0f ), 0xabcdef0123456789ull );
	emitTree( "one" );
	runQueries( 12 );
	doRebuild( false );
	doRebuild( true );
	emitTree( "oneRebuilt" );
	doMarkSerial( live[0] );
	doGather();
	doRebuild( false );
	doEnlarge( live[0] );
	doGather();
	b3DynamicTree_Refit( &tree );
	emitOp( "refit", 0, 0 );
	b3DynamicTree_ClearMoved( &tree );
	emitOp( "clear", 0, 0 );
	doMarkMoved( live[0] );
	b3DynamicTree_Refit( &tree );
	emitOp( "refit", 0, 0 );
	emitTree( "oneMoved" );
	doDestroy( 0 );
	emitTree( "oneDestroyed" );

	// A handful of proxies, rotations and removals by hand
	for ( int i = 0; i < 6; ++i )
	{
		createFixed( unitBox( (float)i, 0.0f, 0.0f ), (uint64_t)i );
		emitTree( "row" );
	}
	runQueries( 16 );
	doDestroy( 2 );
	doDestroy( 0 );
	emitTree( "rowRemoved" );
	createFixed( unitBox( 2.5f, 0.0f, 0.0f ), 100 );
	createFixed( unitBox( 2.5f, 0.0f, 0.0f ), 101 );
	emitTree( "rowReuse" );
	doRebuild( false );
	emitTree( "rowRebuilt" );
	doMove( live[1] );
	doMove( live[3] );
	doRebuild( false );
	emitTree( "rowPartial" );
	doRebuild( true );
	emitTree( "rowFull" );
	while ( liveCount > 0 )
	{
		doDestroy( liveCount - 1 );
	}
	emitTree( "rowEmpty" );
	destroyTree();

	// Growth of the node and proxy pools, with the spare dropped on node growth
	createTree( 1 );
	for ( int i = 0; i < 60; ++i )
	{
		doCreate();
		if ( i % 8 == 7 )
		{
			doRebuild( i % 16 == 15 );
			emitTree( "grow" );
		}
	}
	runQueries( 24 );
	for ( int i = 0; i < 40; ++i )
	{
		doDestroy( vrandint( 0, liveCount ) );
	}
	emitTree( "shrunk" );
	for ( int i = 0; i < 40; ++i )
	{
		doCreate();
	}
	emitTree( "regrown" );
	doRebuild( false );
	doRebuild( false );
	emitTree( "settled" );
	destroyTree();

	// Identical boxes: every partition and sibling cost ties
	createTree( 8 );
	for ( int i = 0; i < 33; ++i )
	{
		createFixed( unitBox( 0.0f, 0.0f, 0.0f ), (uint64_t)( 1000 + i ) );
	}
	emitTree( "same" );
	runQueries( 12 );
	doRebuild( true );
	emitTree( "sameRebuilt" );
	destroyTree();

	// Compound style: create then a full build, then queries
	createTree( 50 );
	for ( int i = 0; i < 50; ++i )
	{
		doCreate();
	}
	doRebuild( true );
	emitTree( "compound" );
	runQueries( 40 );
	destroyTree();
}

static void beginRound( int round )
{
	vseed( 1000 + (uint64_t)round );
	boxMode = round % 6;
	worldSize = vrandf( 5.0f, 60.0f );
	boxSize = vrandf( 0.2f, 4.0f );
	int capacity = vrandint( 0, 300 );
	createWeight = vrandint( 35, 55 );
	destroyWeight = vrandint( 10, 20 );
	opsLo = round % 3 == 0 ? 300 : 60;
	opsHi = round % 3 == 0 ? 700 : 200;
	phaseIndex = 0;
	createTree( capacity );
	vtag( "round" );
	vi32( round );
	vi32( boxMode );
	vf32( worldSize );
	vf32( boxSize );
	vi32( capacity );
	vend();
	emitTree( "begin" );
}

static void phase( void )
{
	bool growing = phaseIndex < PHASE_COUNT / 2 + 1;
	int cw = growing ? createWeight : destroyWeight;
	int dw = growing ? destroyWeight : createWeight;
	int ops = vrandint( opsLo, opsHi );
	for ( int k = 0; k < ops; ++k )
	{
		int kind = vrandint( 0, 100 );
		if ( liveCount == 0 && kind >= cw )
		{
			kind = 0;
		}

		if ( kind < cw )
		{
			if ( liveCount < MAX_LIVE )
			{
				doCreate();
			}
			continue;
		}
		kind -= cw;

		if ( kind < dw )
		{
			doDestroy( vrandint( 0, liveCount ) );
			continue;
		}
		kind -= dw;

		int target = live[vrandint( 0, liveCount )];
		if ( kind < 14 )
		{
			doMove( target );
		}
		else if ( kind < 22 )
		{
			doEnlarge( target );
		}
		else if ( kind < 26 )
		{
			doMarkMoved( target );
		}
		else if ( kind < 28 )
		{
			doMarkSerial( target );
		}
		else if ( kind < 30 )
		{
			doSetCategory( target );
		}
		else if ( kind < 33 )
		{
			runQueries( 1 );
		}
		else if ( kind < 35 )
		{
			doRebuild( vrandint( 0, 4 ) == 0 );
		}
		else if ( kind < 37 )
		{
			b3DynamicTree_Refit( &tree );
			emitOp( "refit", 0, 0 );
		}
		else if ( kind < 38 )
		{
			// Clearing without a refit leaves stale ancestors that validation rejects
			b3DynamicTree_Refit( &tree );
			b3DynamicTree_ClearMoved( &tree );
			emitOp( "clear", 0, 0 );
		}
		else if ( kind < 39 )
		{
			doGather();
		}
		else
		{
			emitOp( "noop", target, 0 );
		}
	}

	emitTree( "phase" );
	runQueries( vrandint( 10, 40 ) );

	// The solver pattern: refit what moved, then a partial rebuild
	if ( phaseIndex % 3 == 2 )
	{
		doGather();
		b3DynamicTree_Refit( &tree );
		emitOp( "refit", 0, 0 );
		doRebuild( false );
		emitTree( "settled" );
		runQueries( vrandint( 5, 20 ) );
	}

	phaseIndex += 1;
}

static void endRound( void )
{
	doRebuild( true );
	emitTree( "end" );
	runQueries( 20 );
	destroyTree();
}

int main( void )
{
	scripted();
	for ( int round = 0; round < 12; ++round )
	{
		beginRound( round );
		for ( int p = 0; p < PHASE_COUNT; ++p )
		{
			phase();
		}
		endRound();
	}
	return 0;
}
