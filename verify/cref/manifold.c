// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for the manifold.c part of lib/manifold.as: feature pairs, incident faces and polygon clipping.

#include "manifold_common.h"

#include "math_internal.h"

#define CLIP_CAPACITY ( 2 * B3_MAX_CLIP_POINTS + 2 )

static b3ClipVertex g_polygon[CLIP_CAPACITY];
static b3ClipVertex g_out[CLIP_CAPACITY];

static void pclip( const b3ClipVertex* v )
{
	pv3( v->position );
	vf32( v->separation );
	ppair( v->pair );
}

static void runPairs( void )
{
	static const int indices[] = { 0, 1, 2, 7, 127, 128, 200, 255 };
	for ( int o1 = 0; o1 < 2; ++o1 )
	{
		for ( int o2 = 0; o2 < 2; ++o2 )
		{
			for ( int i = 0; i < 8; ++i )
			{
				for ( int j = 0; j < 8; ++j )
				{
					b3FeaturePair pair = b3MakeFeaturePair( (b3FeatureOwner)o1, indices[i], (b3FeatureOwner)o2, indices[j] );
					b3FeaturePair flip = b3FlipPair( pair );
					vtag( "pair" );
					ppair( pair );
					ppair( flip );
					ppair( b3FlipPair( flip ) );
					vend();
				}
			}
		}
	}

	vtag( "single" );
	ppair( b3FeaturePair_single );
	vend();
}

static void runIncidentFaces( void )
{
	for ( int h = 0; h < g_hullCount; ++h )
	{
		const b3HullData* hull = g_hulls[h];
		for ( int k = 0; k < 40; ++k )
		{
			b3Vec3 normal;
			if ( k < 6 )
			{
				normal = axisDirection( k );
			}
			else
			{
				normal = runit();
			}
			vtag( "inc" );
			vi32( h );
			pv3( normal );
			for ( int v = 0; v < hull->vertexCount; ++v )
			{
				vi32( b3FindIncidentFace( hull, normal, v ) );
			}
			vend();
		}
	}
}

static void runClip( int count )
{
	for ( int i = 0; i < count; ++i )
	{
		int n = vrandint( 3, 24 );
		if ( ( i & 15 ) == 0 )
		{
			n = vrandint( 60, 2 * B3_MAX_CLIP_POINTS );
		}
		b3Vec3 center = rv( -1.0f, 1.0f );
		b3Vec3 axis = runit();
		b3Vec3 tangent = b3Normalize( b3Cross( axis, runit() ) );
		b3Vec3 bitangent = b3Cross( axis, tangent );
		float radius = vrandf( 0.05f, 2.0f );

		for ( int k = 0; k < n; ++k )
		{
			float angle = 2.0f * B3_PI * (float)k / (float)n;
			b3CosSin cs = b3ComputeCosSin( angle );
			float jitter = vrandf( 0.9f, 1.0f );
			if ( n > 24 )
			{
				// Large polygons stay convex so the output fits the clip buffer
				jitter = 1.0f;
			}
			b3Vec3 p = b3MulAdd( center, radius * jitter * cs.cosine, tangent );
			p = b3MulAdd( p, radius * jitter * cs.sine, bitangent );
			g_polygon[k].position = p;
			g_polygon[k].separation = vrandf( -1.0f, 1.0f );
			g_polygon[k].pair.owner1 = (uint8_t)( vrand32() & 1 );
			g_polygon[k].pair.index1 = (uint8_t)vrand32();
			g_polygon[k].pair.owner2 = (uint8_t)( vrand32() & 1 );
			g_polygon[k].pair.index2 = (uint8_t)vrand32();
		}

		b3Plane clipPlane;
		clipPlane.normal = runit();
		int kind = vrandint( 0, 4 );
		if ( n > 24 && kind == 1 )
		{
			// Noise around a coplanar plane alternates sides and would overflow the buffer
			kind = 2;
		}
		if ( kind == 0 )
		{
			// Through a polygon vertex
			int k = vrandint( 0, n );
			clipPlane.offset = b3Dot( clipPlane.normal, g_polygon[k].position );
		}
		else if ( kind == 1 )
		{
			// In the polygon plane, so every distance is about zero
			clipPlane.normal = axis;
			clipPlane.offset = b3Dot( axis, center );
		}
		else
		{
			float shift = vrandf( -radius, radius );
			clipPlane.offset = b3Dot( clipPlane.normal, center ) + shift;
		}

		int edge = vrandint( 0, 256 );
		b3Plane refPlane;
		refPlane.normal = runit();
		refPlane.offset = vrandf( -1.0f, 1.0f );

		int outCount = b3ClipPolygon( g_out, g_polygon, n, clipPlane, edge, refPlane );

		vtag( "clip" );
		vi32( i );
		vi32( n );
		pplane( clipPlane );
		vi32( edge );
		pplane( refPlane );
		vi32( outCount );
		vend();
		for ( int k = 0; k < outCount; ++k )
		{
			vtag( "cv" );
			pclip( g_out + k );
			vend();
		}
	}
}

int main( void )
{
	buildHulls();
	vseed( 0x6d616e69666f6c64ull );
	runPairs();
	runIncidentFaces();
	runClip( 20000 );
	return 0;
}
