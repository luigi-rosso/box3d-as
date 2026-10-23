// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/height_field.as

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

static void printHeightField( const b3HeightFieldData* hf )
{
	vtag( "hf" );
	vu64( hf->version );
	vu64( hf->hash );
	vi32( hf->byteCount );
	pab( hf->aabb );
	vf32( hf->minHeight );
	vf32( hf->maxHeight );
	vf32( hf->heightScale );
	pv3( hf->scale );
	vi32( hf->columnCount );
	vi32( hf->rowCount );
	vi32( hf->heightsOffset );
	vi32( hf->materialOffset );
	vi32( hf->flagsOffset );
	vu32( hf->clockwise );
	vi32( b3GetHeightFieldTriangleCount( hf ) );
	vend();

	const uint16_t* heights = b3GetHeightFieldCompressedHeights( hf );
	int heightCount = hf->rowCount * hf->columnCount;
	for ( int i = 0; i < heightCount; i += 16 )
	{
		vtag( "h" );
		for ( int j = i; j < i + 16 && j < heightCount; ++j )
		{
			vu32( heights[j] );
		}
		vend();
	}

	const uint8_t* materials = b3GetHeightFieldMaterialIndices( hf );
	const uint8_t* flags = b3GetHeightFieldFlags( hf );
	int cellCount = ( hf->rowCount - 1 ) * ( hf->columnCount - 1 );
	for ( int i = 0; i < cellCount; ++i )
	{
		vtag( "c" );
		vu32( materials[i] );
		vu32( flags[2 * i] );
		vu32( flags[2 * i + 1] );
		vend();
	}

	for ( int i = 0; i < 2 * cellCount; ++i )
	{
		if ( materials[i >> 1] == B3_HEIGHT_FIELD_HOLE )
		{
			continue;
		}

		b3Triangle t = b3GetHeightFieldTriangle( hf, i );
		vtag( "tri" );
		pv3( t.vertices[0] );
		pv3( t.vertices[1] );
		pv3( t.vertices[2] );
		vi32( t.i1 );
		vi32( t.i2 );
		vi32( t.i3 );
		vi32( t.flags );
		vi32( b3GetHeightFieldMaterial( hf, i ) );
		vend();
	}
}

#define MAX_FIELDS 64
static b3HeightFieldData* g_fields[MAX_FIELDS];
static int g_fieldCount;

static void addField( b3HeightFieldData* hf )
{
	printHeightField( hf );
	if ( g_fieldCount < MAX_FIELDS )
	{
		g_fields[g_fieldCount++] = hf;
	}
}

// Seeded heights with holes. special adds infinities and NaN, clamped by construction.
static b3HeightFieldData* randomField( int seed, bool special )
{
	vseed( 3000 + seed );
	int columns = vrandint( 2, 40 );
	int rows = vrandint( 2, 40 );
	b3HeightFieldDef def = { 0 };
	def.countX = columns;
	def.countZ = rows;
	def.scale.x = vrandf( 0.2f, 3.0f );
	def.scale.y = vrandf( 0.1f, 4.0f );
	def.scale.z = vrandf( 0.2f, 3.0f );
	float amplitude = vrandf( 0.0f, 10.0f );
	def.globalMinimumHeight = vrandf( -12.0f, 0.0f );
	def.globalMaximumHeight = vrandf( 0.0f, 12.0f );
	if ( vrandint( 0, 5 ) == 0 )
	{
		// degenerate range clamps to the linear slop
		def.globalMaximumHeight = def.globalMinimumHeight;
	}
	def.clockwiseWinding = vrandint( 0, 2 ) == 1;

	float* heights = malloc( rows * columns * sizeof( float ) );
	for ( int i = 0; i < rows * columns; ++i )
	{
		heights[i] = vrandf( -amplitude, amplitude );
		if ( special && vrandint( 0, 8 ) == 0 )
		{
			heights[i] = vspecialf();
		}
	}
	def.heights = heights;

	int cellCount = ( rows - 1 ) * ( columns - 1 );
	uint8_t* materials = malloc( cellCount );
	int holeChance = vrandint( 0, 6 );
	for ( int i = 0; i < cellCount; ++i )
	{
		materials[i] = (uint8_t)vrandint( 0, 5 );
		if ( holeChance > 0 && vrandint( 0, holeChance * 3 ) == 0 )
		{
			materials[i] = B3_HEIGHT_FIELD_HOLE;
		}
	}
	def.materialIndices = vrandint( 0, 4 ) == 0 ? NULL : materials;

	b3HeightFieldData* hf = b3CreateHeightField( &def );
	free( heights );
	free( materials );
	return hf;
}

static void buildAll( void )
{
	addField( b3CreateGrid( 2, 2, (b3Vec3){ 1, 1, 1 }, false ) );
	addField( b3CreateGrid( 20, 20, (b3Vec3){ 1, 1, 1 }, true ) );
	addField( b3CreateGrid( 9, 33, (b3Vec3){ 0.5f, 2, 1.5f }, true ) );
	addField( b3CreateWave( 20, 20, (b3Vec3){ 1, 1, 1 }, 0.1f, 0.07f, false ) );
	addField( b3CreateWave( 40, 25, (b3Vec3){ 0.75f, 3, 0.5f }, 0.05f, 0.13f, true ) );
	addField( b3CreateWave( 17, 64, (b3Vec3){ 2, 0.5f, 2 }, 0.21f, 0.03f, true ) );

	for ( int i = 0; i < 30; ++i )
	{
		addField( randomField( i, false ) );
	}

	// construction only
	for ( int i = 0; i < 6; ++i )
	{
		b3HeightFieldData* hf = randomField( 100 + i, true );
		printHeightField( hf );
		b3DestroyHeightField( hf );
	}
}

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
	return true;
}

// A hole triangle has no vertices; aim at the field center instead.
static b3Vec3 triangleCenter( const b3HeightFieldData* hf, int ti )
{
	if ( b3GetHeightFieldMaterialIndices( hf )[ti >> 1] == B3_HEIGHT_FIELD_HOLE )
	{
		return b3AABB_Center( hf->aabb );
	}

	b3Triangle t = b3GetHeightFieldTriangle( hf, ti );
	return b3MulSV( 1.0f / 3.0f, b3Add( b3Add( t.vertices[0], t.vertices[1] ), t.vertices[2] ) );
}

static float boxPad( b3AABB box )
{
	b3Vec3 size = b3Sub( box.upperBound, box.lowerBound );
	return 0.1f * ( size.x + size.y + size.z ) + 0.5f;
}

static void queries( int fieldIndex )
{
	b3HeightFieldData* hf = g_fields[fieldIndex];
	vseed( 70000 + fieldIndex );

	vtag( "queries" );
	vi32( fieldIndex );
	vend();

	for ( int i = 0; i < 4; ++i )
	{
		b3Transform xf = randTransform( 10.0f );
		vtag( "aabb" );
		pab( b3ComputeHeightFieldAABB( hf, xf ) );
		vend();
	}

	b3AABB box = hf->aabb;
	float pad = boxPad( box );
	int triangleCount = b3GetHeightFieldTriangleCount( hf );

	for ( int i = 0; i < 200; ++i )
	{
		b3RayCastInput input;
		int kind = vrandint( 0, 4 );
		if ( kind == 0 )
		{
			input.origin = randInBox( box, pad );
			input.translation = randVec( -3.0f * pad, 3.0f * pad );
		}
		else if ( kind == 1 )
		{
			// vertical
			input.origin = randInBox( box, 0.0f );
			input.origin.y = box.upperBound.y + vrandf( 0.1f, pad );
			input.translation = (b3Vec3){ 0.0f, -( box.upperBound.y - box.lowerBound.y ) - 2.0f * pad, 0.0f };
		}
		else
		{
			b3Vec3 c = triangleCenter( hf, vrandint( 0, triangleCount ) );
			input.origin = b3Add( c, randVec( -pad, pad ) );
			input.translation = b3MulSV( vrandf( 1.0f, 2.5f ), b3Sub( c, input.origin ) );
		}
		input.maxFraction = vrandint( 0, 3 ) == 0 ? vrandf( 0.1f, 1.0f ) : 1.0f;

		vtag( "ray" );
		pcast( b3RayCastHeightField( hf, &input ) );
		vend();
	}

	for ( int i = 0; i < 10; ++i )
	{
		b3Vec3 c = randInBox( box, 0.2f * pad );
		b3Vec3 e = randVec( 0.0f, pad );
		b3AABB bounds = { b3Sub( c, e ), b3Add( c, e ) };
		g_queryCount = 0;
		vtag( "query" );
		pab( bounds );
		vend();
		b3QueryHeightField( hf, bounds, queryCallback, NULL );
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

static void casts( int fieldIndex )
{
	b3HeightFieldData* hf = g_fields[fieldIndex];
	vseed( 90000 + fieldIndex );

	vtag( "casts" );
	vi32( fieldIndex );
	vend();

	b3AABB box = hf->aabb;
	float pad = boxPad( box );
	b3Vec3 size = b3Sub( box.upperBound, box.lowerBound );
	float shapeScale = 0.03f * ( size.x + size.y + size.z ) + 0.1f;
	int triangleCount = b3GetHeightFieldTriangleCount( hf );

	b3Vec3 points[B3_MAX_SHAPE_CAST_POINTS];

	for ( int i = 0; i < 40; ++i )
	{
		b3ShapeProxy proxy = randProxy( points, shapeScale );
		b3Transform xf = randTransform( pad );
		b3Vec3 p = vrandint( 0, 2 ) == 0 ? randInBox( box, 0.3f * pad ) : triangleCenter( hf, vrandint( 0, triangleCount ) );
		for ( int k = 0; k < proxy.count; ++k )
		{
			points[k] = b3TransformPoint( xf, b3Add( points[k], p ) );
		}
		vtag( "overlap" );
		vbool( b3OverlapHeightField( hf, xf, &proxy ) );
		vend();
	}

	for ( int i = 0; i < 60; ++i )
	{
		b3ShapeCastInput input = { 0 };
		input.proxy = randProxy( points, shapeScale );
		b3Vec3 origin;
		b3Vec3 target;
		int kind = vrandint( 0, 3 );
		if ( kind == 0 )
		{
			origin = randInBox( box, pad );
			target = randInBox( box, 0.0f );
		}
		else if ( kind == 1 )
		{
			target = triangleCenter( hf, vrandint( 0, triangleCount ) );
			origin = target;
			origin.y = box.upperBound.y + vrandf( shapeScale, pad );
		}
		else
		{
			target = triangleCenter( hf, vrandint( 0, triangleCount ) );
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
		pcast( b3ShapeCastHeightField( hf, &input ) );
		vend();
	}

	b3PlaneResult planes[16];
	for ( int i = 0; i < 40; ++i )
	{
		b3Capsule mover;
		b3Vec3 c = vrandint( 0, 2 ) == 0 ? randInBox( box, 0.2f * pad ) : triangleCenter( hf, vrandint( 0, triangleCount ) );
		mover.center1 = b3Add( c, randVec( -shapeScale, shapeScale ) );
		mover.center2 = b3Add( mover.center1, randVec( -shapeScale, shapeScale ) );
		mover.radius = vrandf( 0.05f, 2.0f * shapeScale );
		int offset = vrandint( 0, 4 );
		int capacity = vrandint( 1, 12 );
		memset( planes, 0, sizeof( planes ) );
		int count = b3CollideMoverAndHeightField( planes + offset, capacity, hf, &mover );
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
	vi32( g_fieldCount );
	vend();

	if ( stage & 1 )
	{
		for ( int f = 0; f < g_fieldCount; ++f )
		{
			queries( f );
		}
	}

	if ( stage & 2 )
	{
		for ( int f = 0; f < g_fieldCount; ++f )
		{
			casts( f );
		}
	}

	return 0;
}
