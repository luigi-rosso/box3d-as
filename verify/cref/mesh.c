// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/mesh.as

#include "common.h"

#include "shape.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <stdlib.h>

static void pv3( b3Vec3 v )
{
	vf32( v.x );
	vf32( v.y );
	vf32( v.z );
}

static void pab( b3AABB a )
{
	pv3( a.lowerBound );
	pv3( a.upperBound );
}

static void pcast( b3CastOutput o )
{
	vbool( o.hit );
	vf32( o.fraction );
	pv3( o.normal );
	pv3( o.point );
	vi32( o.iterations );
	vi32( o.triangleIndex );
	vi32( o.childIndex );
	vi32( o.materialIndex );
}

static b3Vec3 randVec( float lo, float hi )
{
	b3Vec3 v;
	v.x = vrandf( lo, hi );
	v.y = vrandf( lo, hi );
	v.z = vrandf( lo, hi );
	return v;
}

static b3Vec3 randInBox( b3AABB box, float pad )
{
	b3Vec3 v;
	v.x = vrandf( box.lowerBound.x - pad, box.upperBound.x + pad );
	v.y = vrandf( box.lowerBound.y - pad, box.upperBound.y + pad );
	v.z = vrandf( box.lowerBound.z - pad, box.upperBound.z + pad );
	return v;
}

static b3Transform randTransform( float range )
{
	b3Transform t;
	t.p = randVec( -range, range );
	b3Quat q;
	q.v = randVec( -1.0f, 1.0f );
	q.s = vrandf( -1.0f, 1.0f );
	t.q = b3NormalizeQuat( q );
	return t;
}

static void printMesh( const b3MeshData* mesh )
{
	vtag( "mesh" );
	vbool( mesh != NULL );
	vbool( b3IsValidMesh( mesh ) );
	vend();
	if ( mesh == NULL )
	{
		return;
	}

	vtag( "head" );
	vu64( mesh->version );
	vu64( mesh->hash );
	vi32( mesh->byteCount );
	pab( mesh->bounds );
	vf32( mesh->surfaceArea );
	vi32( mesh->treeHeight );
	vi32( mesh->degenerateCount );
	vi32( mesh->nodeOffset );
	vi32( mesh->nodeCount );
	vi32( mesh->vertexOffset );
	vi32( mesh->vertexCount );
	vi32( mesh->triangleOffset );
	vi32( mesh->triangleCount );
	vi32( mesh->materialOffset );
	vi32( mesh->materialCount );
	vi32( mesh->flagsOffset );
	vi32( mesh->padding );
	vi32( b3GetHeight( mesh ) );
	vend();

	const b3MeshNode* nodes = b3GetMeshNodes( mesh );
	for ( int i = 0; i < mesh->nodeCount; ++i )
	{
		uint32_t bits;
		memcpy( &bits, &nodes[i].data, sizeof( bits ) );
		vtag( "n" );
		pv3( nodes[i].lowerBound );
		vu32( bits );
		pv3( nodes[i].upperBound );
		vu32( nodes[i].triangleOffset );
		vend();
	}

	const b3Vec3* vertices = b3GetMeshVertices( mesh );
	for ( int i = 0; i < mesh->vertexCount; ++i )
	{
		vtag( "v" );
		pv3( vertices[i] );
		vend();
	}

	const b3MeshTriangle* triangles = b3GetMeshTriangles( mesh );
	const uint8_t* materials = b3GetMeshMaterialIndices( mesh );
	const uint8_t* flags = b3GetMeshFlags( mesh );
	for ( int i = 0; i < mesh->triangleCount; ++i )
	{
		vtag( "t" );
		vi32( triangles[i].index1 );
		vi32( triangles[i].index2 );
		vi32( triangles[i].index3 );
		vu32( materials[i] );
		vu32( flags[i] );
		vend();
	}
}

#define MAX_MESHES 128
static b3MeshData* g_meshes[MAX_MESHES];
static int g_meshCount;

static void addMesh( b3MeshData* mesh )
{
	printMesh( mesh );
	if ( mesh != NULL && g_meshCount < MAX_MESHES )
	{
		g_meshes[g_meshCount++] = mesh;
	}
}

// A seeded soup: random vertices, optional exact and near duplicates, random and degenerate triangles.
static void soup( int seed )
{
	vseed( 1000 + seed );

	int baseCount = vrandint( 3, 120 );
	int dupCount = vrandint( 0, 40 );
	int vertexCount = baseCount + dupCount;
	float extent = vrandf( 0.5f, 20.0f );
	float jitter = vrandf( 0.0f, 0.02f );

	b3Vec3* vertices = malloc( vertexCount * sizeof( b3Vec3 ) );
	for ( int i = 0; i < baseCount; ++i )
	{
		vertices[i] = randVec( -extent, extent );
	}

	for ( int i = 0; i < dupCount; ++i )
	{
		int src = vrandint( 0, baseCount );
		b3Vec3 v = vertices[src];
		int mode = vrandint( 0, 3 );
		if ( mode == 1 )
		{
			v.x += vrandf( -jitter, jitter );
			v.y += vrandf( -jitter, jitter );
			v.z += vrandf( -jitter, jitter );
		}
		else if ( mode == 2 )
		{
			// collinear with two others
			int other = vrandint( 0, baseCount );
			float t = vrandf( 0.0f, 1.0f );
			v = b3Lerp( vertices[src], vertices[other], t );
		}
		vertices[baseCount + i] = v;
	}

	int triangleCount = vrandint( 1, 260 );
	int* indices = malloc( 3 * triangleCount * sizeof( int ) );
	uint8_t* materials = malloc( triangleCount );
	for ( int i = 0; i < triangleCount; ++i )
	{
		int kind = vrandint( 0, 10 );
		int a = vrandint( 0, vertexCount );
		int b = vrandint( 0, vertexCount );
		int c = vrandint( 0, vertexCount );
		if ( kind == 0 )
		{
			b = a;
		}
		else if ( kind == 1 )
		{
			c = b;
		}
		indices[3 * i + 0] = a;
		indices[3 * i + 1] = b;
		indices[3 * i + 2] = c;
		materials[i] = (uint8_t)vrandint( 0, 7 );
	}

	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.vertexCount = vertexCount;
	def.indices = indices;
	def.triangleCount = triangleCount;
	int useMaterials = vrandint( 0, 2 );
	def.materialIndices = useMaterials ? materials : NULL;
	def.weldVertices = vrandint( 0, 2 ) == 1;
	def.weldTolerance = vrandf( 0.0f, 0.05f );
	def.useMedianSplit = vrandint( 0, 2 ) == 1;
	def.identifyEdges = vrandint( 0, 2 ) == 1;
	def.clockWiseWinding = vrandint( 0, 2 ) == 1;

	int degenerateCapacity = vrandint( 0, 8 );
	int degenerates[8] = { 0 };

	b3MeshData* mesh = b3CreateMesh( &def, degenerateCapacity > 0 ? degenerates : NULL, degenerateCapacity );
	vtag( "soup" );
	vi32( seed );
	for ( int i = 0; i < 8; ++i )
	{
		vi32( degenerates[i] );
	}
	vend();
	addMesh( mesh );

	free( vertices );
	free( indices );
	free( materials );
}

// A grid where every triangle has its own vertices, welded back together, plus noise in height.
static void weldedGrid( int seed )
{
	vseed( 5000 + seed );
	int nx = vrandint( 1, 14 );
	int nz = vrandint( 1, 14 );
	float cell = vrandf( 0.2f, 2.0f );
	float noise = vrandf( 0.0f, 0.5f );

	int triangleCount = 2 * nx * nz;
	b3Vec3* vertices = malloc( 3 * triangleCount * sizeof( b3Vec3 ) );
	int* indices = malloc( 3 * triangleCount * sizeof( int ) );
	float* heights = malloc( ( nx + 1 ) * ( nz + 1 ) * sizeof( float ) );
	for ( int i = 0; i < ( nx + 1 ) * ( nz + 1 ); ++i )
	{
		heights[i] = vrandf( -noise, noise );
	}

	int k = 0;
	for ( int ix = 0; ix < nx; ++ix )
	{
		for ( int iz = 0; iz < nz; ++iz )
		{
			b3Vec3 p11 = { cell * ix, heights[ix * ( nz + 1 ) + iz], cell * iz };
			b3Vec3 p12 = { cell * ix, heights[ix * ( nz + 1 ) + iz + 1], cell * ( iz + 1 ) };
			b3Vec3 p21 = { cell * ( ix + 1 ), heights[( ix + 1 ) * ( nz + 1 ) + iz], cell * iz };
			b3Vec3 p22 = { cell * ( ix + 1 ), heights[( ix + 1 ) * ( nz + 1 ) + iz + 1], cell * ( iz + 1 ) };
			vertices[k + 0] = p11;
			vertices[k + 1] = p12;
			vertices[k + 2] = p22;
			vertices[k + 3] = p22;
			vertices[k + 4] = p21;
			vertices[k + 5] = p11;
			for ( int j = 0; j < 6; ++j )
			{
				indices[k + j] = k + j;
			}
			k += 6;
		}
	}

	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.vertexCount = 3 * triangleCount;
	def.indices = indices;
	def.triangleCount = triangleCount;
	def.weldVertices = true;
	def.weldTolerance = vrandf( 0.001f, 0.1f );
	def.useMedianSplit = vrandint( 0, 2 ) == 1;
	def.identifyEdges = true;
	def.clockWiseWinding = vrandint( 0, 2 ) == 1;

	vtag( "welded" );
	vi32( seed );
	vend();
	addMesh( b3CreateMesh( &def, NULL, 0 ) );

	free( vertices );
	free( indices );
	free( heights );
}

static void failures( void )
{
	b3Vec3 vertices[4] = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 0, 1 }, { 1, 0, 1 } };
	int indices[6] = { 0, 1, 2, 2, 1, 3 };

	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.vertexCount = 4;
	def.indices = indices;
	def.triangleCount = 2;

	int strides[6] = { 0, 12, 5, 8, 14, 5000 };
	for ( int i = 0; i < 6; ++i )
	{
		def.stride = strides[i];
		b3MeshData* mesh = b3CreateMesh( &def, NULL, 0 );
		vtag( "stride" );
		vi32( strides[i] );
		vend();
		printMesh( mesh );
		if ( mesh )
			b3DestroyMesh( mesh );
	}
	def.stride = 0;

	def.vertexCount = 2;
	printMesh( b3CreateMesh( &def, NULL, 0 ) );
	def.vertexCount = 4;
	def.triangleCount = 0;
	printMesh( b3CreateMesh( &def, NULL, 0 ) );

	// all degenerate
	int degenerateIndices[6] = { 0, 0, 1, 2, 2, 3 };
	def.indices = degenerateIndices;
	def.triangleCount = 2;
	printMesh( b3CreateMesh( &def, NULL, 0 ) );
}

static void buildAll( void )
{
	addMesh( b3CreateGridMesh( 4, 4, 1.0f, 0, false ) );
	addMesh( b3CreateGridMesh( 4, 4, 1.0f, 0, true ) );
	addMesh( b3CreateGridMesh( 10, 7, 0.5f, 3, true ) );
	addMesh( b3CreateGridMesh( 1, 1, 2.0f, 1, true ) );
	addMesh( b3CreateGridMesh( 16, 16, 1.0f, 0, true ) );
	addMesh( b3CreateGridMesh( 33, 20, 0.25f, 5, false ) );
	addMesh( b3CreateWaveMesh( 10, 10, 1.0f, 0.5f, 0.2f, 0.3f ) );
	addMesh( b3CreateWaveMesh( 20, 13, 0.7f, 2.0f, 0.1f, 0.05f ) );
	addMesh( b3CreateWaveMesh( 32, 32, 0.5f, 1.5f, 0.17f, 0.23f ) );
	addMesh( b3CreateTorusMesh( 8, 6, 2.0f, 0.5f ) );
	addMesh( b3CreateTorusMesh( 16, 12, 3.0f, 1.0f ) );
	addMesh( b3CreateTorusMesh( 32, 16, 5.0f, 1.5f ) );
	addMesh( b3CreateTorusMesh( 3, 3, 1.0f, 0.3f ) );
	addMesh( b3CreateBoxMesh( (b3Vec3){ 0, 0, 0 }, (b3Vec3){ 1, 1, 1 }, true ) );
	addMesh( b3CreateBoxMesh( (b3Vec3){ 0, 0, 0 }, (b3Vec3){ 1, 1, 1 }, false ) );
	addMesh( b3CreateBoxMesh( (b3Vec3){ 1, -2, 3 }, (b3Vec3){ 0.5f, 2, 0.1f }, true ) );
	addMesh( b3CreateHollowBoxMesh( (b3Vec3){ 0, 0, 0 }, (b3Vec3){ 2, 1, 3 } ) );
	addMesh( b3CreateHollowBoxMesh( (b3Vec3){ -1, 0.5f, 2 }, (b3Vec3){ 0.3f, 0.3f, 0.3f } ) );
	addMesh( b3CreatePlatformMesh( (b3Vec3){ 0, 0, 0 }, 1.0f, 2.0f, 4.0f ) );
	addMesh( b3CreatePlatformMesh( (b3Vec3){ 3, 1, -2 }, 0.5f, 6.0f, 6.5f ) );

	for ( int i = 0; i < 40; ++i )
	{
		soup( i );
	}

	for ( int i = 0; i < 12; ++i )
	{
		weldedGrid( i );
	}

	failures();
}

static const b3Vec3 g_scales[6] = {
	{ 1, 1, 1 }, { 2, 0.5f, 1.5f }, { -1, 1, 1 }, { 1, -2, -0.5f }, { -1, -1, -1 }, { 0.3f, 3, -0.7f },
};

static int g_queryLimit;
static int g_queryCount;

static bool queryCallback( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	(void)context;
	vtag( "q" );
	vi32( triangleIndex );
	pv3( a );
	pv3( b );
	pv3( c );
	vend();
	g_queryCount += 1;
	return g_queryLimit < 0 || g_queryCount < g_queryLimit;
}

// Everything that does not need GJK
static void queries( int meshIndex, int scaleIndex )
{
	b3MeshData* data = g_meshes[meshIndex];
	b3Mesh mesh = { data, g_scales[scaleIndex] };
	vseed( 20000 + 16 * meshIndex + scaleIndex );

	vtag( "queries" );
	vi32( meshIndex );
	vi32( scaleIndex );
	vend();

	for ( int i = 0; i < data->triangleCount; ++i )
	{
		b3Triangle t = b3GetMeshTriangle( &mesh, i );
		vtag( "tri" );
		pv3( t.vertices[0] );
		pv3( t.vertices[1] );
		pv3( t.vertices[2] );
		vi32( t.i1 );
		vi32( t.i2 );
		vi32( t.i3 );
		vi32( t.flags );
		vend();
	}

	for ( int i = 0; i < 4; ++i )
	{
		b3Transform xf = randTransform( 10.0f );
		vtag( "aabb" );
		pab( b3ComputeMeshAABB( data, xf, mesh.scale ) );
		vend();
	}

	b3Vec3 s = mesh.scale;
	b3Vec3 l = b3Mul( s, data->bounds.lowerBound );
	b3Vec3 u = b3Mul( s, data->bounds.upperBound );
	b3AABB box = { b3Min( l, u ), b3Max( l, u ) };
	b3Vec3 size = b3Sub( box.upperBound, box.lowerBound );
	float pad = 0.25f * ( size.x + size.y + size.z ) + 0.5f;

	for ( int i = 0; i < 40; ++i )
	{
		b3RayCastInput input;
		int kind = vrandint( 0, 3 );
		if ( kind == 0 )
		{
			input.origin = randInBox( box, pad );
			input.translation = randVec( -3.0f * pad, 3.0f * pad );
		}
		else
		{
			// aim at a triangle centroid
			int ti = vrandint( 0, data->triangleCount );
			b3Triangle t = b3GetMeshTriangle( &mesh, ti );
			b3Vec3 c = b3MulSV( 1.0f / 3.0f, b3Add( b3Add( t.vertices[0], t.vertices[1] ), t.vertices[2] ) );
			input.origin = b3Add( c, randVec( -pad, pad ) );
			input.translation = b3MulSV( vrandf( 1.0f, 2.5f ), b3Sub( c, input.origin ) );
		}
		input.maxFraction = vrandint( 0, 3 ) == 0 ? vrandf( 0.1f, 1.0f ) : 1.0f;

		vtag( "ray" );
		pcast( b3RayCastMesh( &mesh, &input ) );
		vend();
	}

	for ( int i = 0; i < 12; ++i )
	{
		b3Vec3 c = randInBox( box, 0.2f * pad );
		b3Vec3 e = randVec( 0.0f, 0.4f * pad );
		b3AABB bounds = { b3Sub( c, e ), b3Add( c, e ) };
		g_queryLimit = vrandint( 0, 4 ) == 0 ? vrandint( 1, 6 ) : -1;
		g_queryCount = 0;
		vtag( "query" );
		pab( bounds );
		vi32( g_queryLimit );
		vend();
		b3QueryMesh( &mesh, bounds, queryCallback, NULL );
		vtag( "queryEnd" );
		vi32( g_queryCount );
		vend();
	}
}

static b3ShapeProxy randProxy( b3Vec3* points, float scale )
{
	int kind = vrandint( 0, 4 );
	int count = kind == 0 ? 1 : ( kind == 1 ? 2 : vrandint( 3, 9 ) );
	for ( int i = 0; i < count; ++i )
	{
		points[i] = randVec( -scale, scale );
	}
	float radius = kind == 3 ? 0.0f : vrandf( 0.0f, scale );
	return (b3ShapeProxy){ points, count, radius };
}

// Overlap, shape cast and mover, which need GJK
static void casts( int meshIndex, int scaleIndex )
{
	b3MeshData* data = g_meshes[meshIndex];
	b3Mesh mesh = { data, g_scales[scaleIndex] };
	vseed( 40000 + 16 * meshIndex + scaleIndex );

	vtag( "casts" );
	vi32( meshIndex );
	vi32( scaleIndex );
	vend();

	b3Vec3 s = mesh.scale;
	b3Vec3 l = b3Mul( s, data->bounds.lowerBound );
	b3Vec3 u = b3Mul( s, data->bounds.upperBound );
	b3AABB box = { b3Min( l, u ), b3Max( l, u ) };
	b3Vec3 size = b3Sub( box.upperBound, box.lowerBound );
	float pad = 0.25f * ( size.x + size.y + size.z ) + 0.5f;
	float shapeScale = 0.05f * ( size.x + size.y + size.z ) + 0.1f;

	b3Vec3 points[B3_MAX_SHAPE_CAST_POINTS];

	for ( int i = 0; i < 12; ++i )
	{
		b3ShapeProxy proxy = randProxy( points, shapeScale );
		b3Transform xf = randTransform( pad );
		b3Vec3 p = randInBox( box, 0.3f * pad );
		// place the proxy in mesh space through the inverse of xf
		for ( int k = 0; k < proxy.count; ++k )
		{
			points[k] = b3TransformPoint( xf, b3Add( points[k], p ) );
		}
		vtag( "overlap" );
		vbool( b3OverlapMesh( &mesh, xf, &proxy ) );
		vend();
	}

	for ( int i = 0; i < 16; ++i )
	{
		b3ShapeCastInput input = { 0 };
		input.proxy = randProxy( points, shapeScale );
		b3Vec3 origin;
		b3Vec3 target;
		if ( vrandint( 0, 2 ) == 0 )
		{
			origin = randInBox( box, pad );
			target = randInBox( box, 0.0f );
		}
		else
		{
			int ti = vrandint( 0, data->triangleCount );
			b3Triangle t = b3GetMeshTriangle( &mesh, ti );
			target = b3MulSV( 1.0f / 3.0f, b3Add( b3Add( t.vertices[0], t.vertices[1] ), t.vertices[2] ) );
			origin = b3Add( target, randVec( -pad, pad ) );
		}
		for ( int k = 0; k < input.proxy.count; ++k )
		{
			points[k] = b3Add( points[k], origin );
		}
		input.translation = b3MulSV( vrandf( 0.5f, 2.0f ), b3Sub( target, origin ) );
		input.maxFraction = vrandint( 0, 3 ) == 0 ? vrandf( 0.2f, 1.0f ) : 1.0f;
		input.canEncroach = vrandint( 0, 2 ) == 1;

		vtag( "cast" );
		pcast( b3ShapeCastMesh( &mesh, &input ) );
		vend();
	}

	b3PlaneResult planes[16];
	for ( int i = 0; i < 16; ++i )
	{
		b3Capsule mover;
		mover.center1 = randInBox( box, 0.2f * pad );
		mover.center2 = b3Add( mover.center1, randVec( -shapeScale, shapeScale ) );
		mover.radius = vrandf( 0.05f, 2.0f * shapeScale );
		int offset = vrandint( 0, 4 );
		int capacity = vrandint( 0, 12 );
		memset( planes, 0, sizeof( planes ) );
		int count = b3CollideMoverAndMesh( planes + offset, capacity, &mesh, &mover );
		vtag( "mover" );
		vi32( offset );
		vi32( capacity );
		vi32( count );
		vend();
		for ( int k = 0; k < count; ++k )
		{
			b3PlaneResult r = planes[offset + k];
			vtag( "plane" );
			pv3( r.plane.normal );
			vf32( r.plane.offset );
			pv3( r.point );
			vi32( r.triangleIndex );
			vi32( r.childIndex );
			vi32( r.materialIndex );
			vend();
		}
	}
}

int main( int argc, char** argv )
{
	int stage = argc > 1 ? atoi( argv[1] ) : 3;

	buildAll();
	vtag( "built" );
	vi32( g_meshCount );
	vend();

	if ( stage & 1 )
	{
		for ( int m = 0; m < g_meshCount; ++m )
		{
			for ( int s = 0; s < 6; ++s )
			{
				queries( m, s );
			}
		}
	}

	if ( stage & 2 )
	{
		for ( int m = 0; m < g_meshCount; ++m )
		{
			for ( int s = 0; s < 6; ++s )
			{
				casts( m, s );
			}
		}
	}

	return 0;
}
