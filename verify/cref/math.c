// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/math_functions.as, lib/math_internal.as, lib/core.as,
// lib/constants.as, lib/types.as, lib/collision.as and lib/aabb.as

#include "common.h"

#include "aabb.h"
#include "core.h"
#include "math_internal.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"
#include "box3d/types.h"

#include <float.h>
#include <stddef.h>

uint32_t b3Hash( uint32_t hash, const uint8_t* data, int count );

static void pv2( b3Vec2 v )
{
	vf32( v.x );
	vf32( v.y );
}

static void pv3( b3Vec3 v )
{
	vf32( v.x );
	vf32( v.y );
	vf32( v.z );
}

static void pq( b3Quat q )
{
	pv3( q.v );
	vf32( q.s );
}

static void pm3( b3Matrix3 m )
{
	pv3( m.cx );
	pv3( m.cy );
	pv3( m.cz );
}

static void pm2( b3Matrix2 m )
{
	pv2( m.cx );
	pv2( m.cy );
}

static void pab( b3AABB a )
{
	pv3( a.lowerBound );
	pv3( a.upperBound );
}

static void pxf( b3Transform t )
{
	pv3( t.p );
	pq( t.q );
}

static void ppl( b3Plane p )
{
	pv3( p.normal );
	vf32( p.offset );
}

static void pseg( b3SegmentDistanceResult r )
{
	pv3( r.point1 );
	vf32( r.fraction1 );
	pv3( r.point2 );
	vf32( r.fraction2 );
}

// clang lowers a > b ? a : b against a constant to fmaxnm, which quiets a signaling NaN,
// so signaling inputs are quieted where a function feeds one into such a select.
static b3Vec3 quietVec( b3Vec3 v )
{
	uint32_t bits[3];
	memcpy( bits, &v, sizeof( bits ) );
	for ( int i = 0; i < 3; ++i )
	{
		if ( ( bits[i] & 0x7f800000 ) == 0x7f800000 && ( bits[i] & 0x007fffff ) != 0 )
		{
			bits[i] |= 0x00400000;
		}
	}
	memcpy( &v, bits, sizeof( bits ) );
	return v;
}

// mode 0 special, 1 tiny, 2 large, 3 ordinary
static float rcomp( int mode )
{
	if ( mode == 0 )
	{
		return vspecialf();
	}
	if ( mode == 1 )
	{
		return vrandf( -1.0e-3f, 1.0e-3f );
	}
	if ( mode == 2 )
	{
		return vrandf( -1.0e4f, 1.0e4f );
	}
	return vrandf( -10.0f, 10.0f );
}

static float rscalar( void )
{
	int mode = (int)( vrand32() % 4 );
	return rcomp( mode );
}

static b3Vec3 rvec( void )
{
	int mode = (int)( vrand32() % 4 );
	b3Vec3 v;
	v.x = rcomp( mode );
	v.y = rcomp( mode );
	v.z = rcomp( mode );
	return v;
}

static b3Vec3 rvecn( void )
{
	b3Vec3 v;
	v.x = vrandf( -10.0f, 10.0f );
	v.y = vrandf( -10.0f, 10.0f );
	v.z = vrandf( -10.0f, 10.0f );
	return v;
}

static b3Vec2 rvec2( void )
{
	int mode = (int)( vrand32() % 4 );
	b3Vec2 v;
	v.x = rcomp( mode );
	v.y = rcomp( mode );
	return v;
}

static b3Vec3 runit( void )
{
	b3Vec3 v = rvecn();
	v = b3Normalize( v );
	if ( b3LengthSquared( v ) == 0.0f )
	{
		v = b3Vec3_axisX;
	}
	return v;
}

static b3Quat rquat( void )
{
	b3Quat q;
	q.v.x = vrandf( -1.0f, 1.0f );
	q.v.y = vrandf( -1.0f, 1.0f );
	q.v.z = vrandf( -1.0f, 1.0f );
	q.s = vrandf( -1.0f, 1.0f );
	return b3NormalizeQuat( q );
}

static b3Quat rquatAny( void )
{
	uint32_t mode = vrand32() % 4;
	if ( mode == 0 )
	{
		b3Quat q;
		q.v.x = vspecialf();
		q.v.y = vspecialf();
		q.v.z = vspecialf();
		q.s = vspecialf();
		return q;
	}
	if ( mode == 1 )
	{
		b3Quat q;
		q.v = rvecn();
		q.s = vrandf( -10.0f, 10.0f );
		return q;
	}
	return rquat();
}

static b3Matrix3 rmat( void )
{
	uint32_t mode = vrand32() % 8;
	b3Matrix3 m;
	if ( mode == 0 )
	{
		m.cx = rvec();
		m.cy = rvec();
		m.cz = rvec();
		return m;
	}
	if ( mode == 1 )
	{
		m.cx = rvecn();
		m.cy = rvecn();
		m.cz = b3Add( m.cx, m.cy );
		return m;
	}
	if ( mode == 2 )
	{
		b3Quat q = rquat();
		return b3MakeMatrixFromQuat( q );
	}
	if ( mode == 3 )
	{
		float s = vrandf( 1.0e-15f, 1.0e-12f );
		m.cx = b3MulSV( s, rvecn() );
		m.cy = b3MulSV( s, rvecn() );
		m.cz = b3MulSV( s, rvecn() );
		return m;
	}
	m.cx = rvecn();
	m.cy = rvecn();
	m.cz = rvecn();
	return m;
}

static b3Transform rxf( void )
{
	b3Transform t;
	t.p = rvecn();
	t.q = rquat();
	return t;
}

static b3AABB raabb( void )
{
	uint32_t mode = vrand32() % 6;
	b3AABB a;
	if ( mode == 0 )
	{
		a.lowerBound = rvec();
		a.upperBound = rvec();
		return a;
	}
	b3Vec3 c = rvecn();
	b3Vec3 e = b3Abs( rvecn() );
	if ( mode == 1 )
	{
		e = b3Neg( e );
	}
	a.lowerBound = b3Sub( c, e );
	a.upperBound = b3Add( c, e );
	return a;
}

static b3Plane rplane( void )
{
	b3Plane p;
	uint32_t mode = vrand32() % 4;
	if ( mode == 0 )
	{
		p.normal = rvec();
		p.offset = vspecialf();
		return p;
	}
	p.normal = runit();
	p.offset = vrandf( -10.0f, 10.0f );
	return p;
}

static void checkConstants( void )
{
	vtag( "const" );
	vf32( B3_PI );
	vf32( B3_DEG_TO_RAD );
	vf32( B3_RAD_TO_DEG );
	vf32( B3_MIN_SCALE );
	vf32( B3_TWO_PI );
	vf32( B3_PI_OVER_TWO );
	vf32( B3_PI_OVER_FOUR );
	vf32( B3_SQRT3 );
	vf32( FLT_MIN );
	vf32( FLT_MAX );
	vf32( FLT_EPSILON );
	vf32( B3_MAX_ROTATION );
	vf32( B3_MIN_FRICTION_WEIGHT );
	vf32( B3_CONTACT_RECYCLE_ANGULAR_DISTANCE );
	vf32( B3_PARALLEL_EDGE_TOL );
	vf32( B3_AABB_MARGIN_FRACTION );
	vf32( B3_TIME_TO_SLEEP );
	vend();

	vtag( "iconst" );
	vi32( B3_MAX_WORKERS );
	vi32( B3_MAX_TASKS );
	vi32( B3_GRAPH_COLOR_COUNT );
	vi32( B3_CONTACT_MANIFOLD_COUNT_BUCKETS );
	vi32( B3_MAX_WORLDS );
	vi32( B3_MAX_RESTITUTION_ITERATIONS );
	vi32( B3_MAX_MANIFOLD_POINTS );
	vi32( B3_GYROSCOPIC_ITERATIONS );
	vi32( B3_MAX_HULL_VERTICES );
	vi32( B3_MAX_HULL_FACES );
	vi32( B3_MAX_HULL_EDGES );
	vi32( B3_MAX_SHAPE_CAST_POINTS );
	vi32( B3_SHAPE_POWER );
	vi32( B3_CHILD_POWER );
	vi32( B3_MAX_SHAPES );
	vi32( B3_MAX_CHILD_SHAPES );
	vi32( B3_MAX_MESH_CONTACT_TRIANGLES );
	vi32( B3_NULL_INDEX );
	vi32( B3_SECRET_COOKIE );
	vi32( B3_ALIGNMENT );
	vi32( B3_HEIGHT_FIELD_HOLE );
	vu32( B3_HASH_INIT );
	vend();

	vtag( "versions" );
	vu64( B3_HULL_VERSION );
	vu64( B3_MESH_VERSION );
	vu64( B3_HEIGHT_FIELD_VERSION );
	vu64( B3_COMPOUND_VERSION );
	vu64( B3_DYNAMIC_TREE_VERSION );
	vu64( B3_DEFAULT_CATEGORY_BITS );
	vu64( B3_DEFAULT_MASK_BITS );
	b3Version version = b3GetVersion();
	vi32( version.major );
	vi32( version.minor );
	vi32( version.revision );
	vbool( b3IsDoublePrecision() );
	vi32( b3GetMaxManifoldPoints() );
	vend();

	vtag( "statics" );
	pv3( b3Vec3_zero );
	pv3( b3Vec3_one );
	pv3( b3Vec3_axisX );
	pv3( b3Vec3_axisY );
	pv3( b3Vec3_axisZ );
	pq( b3Quat_identity );
	pxf( b3Transform_identity );
	pm3( b3Mat3_zero );
	pm3( b3Mat3_identity );
	pv3( b3Pos_zero );
	pxf( b3WorldTransform_identity );
	pab( B3_BOUNDS3_EMPTY );
	vf32( b3_emptyDistanceCache.metric );
	vu32( b3_emptyDistanceCache.count );
	for ( int i = 0; i < 4; ++i )
	{
		vu32( b3_emptyDistanceCache.indexA[i] );
		vu32( b3_emptyDistanceCache.indexB[i] );
	}
	vend();

	vtag( "edgeflags" );
	vi32( b3_concaveEdge1 );
	vi32( b3_concaveEdge2 );
	vi32( b3_concaveEdge3 );
	vi32( b3_inverseConcaveEdge1 );
	vi32( b3_inverseConcaveEdge2 );
	vi32( b3_inverseConcaveEdge3 );
	vi32( b3_allConcaveEdges );
	vi32( b3_flatEdge1 );
	vi32( b3_flatEdge2 );
	vi32( b3_flatEdge3 );
	vi32( b3_allFlatEdges );
	vi32( b3_bodyTypeCount );
	vi32( b3_shapeTypeCount );
	vi32( b3_sphereShape );
	vi32( b3_wheelJoint );
	vi32( b3_toiStateSeparated );
	vi32( b3_manualEdgePairAxis );
	vi32( b3_featureVertex3 );
	vend();
}

static void checkUnitConstants( float units )
{
	b3SetLengthUnitsPerMeter( units );
	vtag( "units" );
	vf32( b3GetLengthUnitsPerMeter() );
	vf32( B3_HUGE );
	vf32( B3_LINEAR_SLOP );
	vf32( B3_MIN_CAPSULE_LENGTH );
	vf32( B3_OVERLAP_SLOP );
	vf32( B3_SPECULATIVE_DISTANCE );
	vf32( B3_MESH_REST_OFFSET );
	vf32( B3_CONTACT_RECYCLE_DISTANCE );
	vf32( B3_MAX_AABB_MARGIN );
	vend();
}

static void pjointdef( b3JointDef d )
{
	vu64( (uint64_t)(uintptr_t)d.userData );
	vi32( d.bodyIdA.index1 );
	vu32( d.bodyIdA.world0 );
	vu32( d.bodyIdA.generation );
	vi32( d.bodyIdB.index1 );
	vu32( d.bodyIdB.world0 );
	vu32( d.bodyIdB.generation );
	pxf( d.localFrameA );
	pxf( d.localFrameB );
	vf32( d.forceThreshold );
	vf32( d.torqueThreshold );
	vf32( d.constraintHertz );
	vf32( d.constraintDampingRatio );
	vf32( d.drawScale );
	vbool( d.collideConnected );
	vi32( d.internalValue );
}

static void psurface( b3SurfaceMaterial m )
{
	vf32( m.friction );
	vf32( m.restitution );
	vf32( m.rollingResistance );
	pv3( m.tangentVelocity );
	vu64( m.userMaterialId );
	vu32( m.customColor );
	vu32( m.padding );
}

static void pfilter( b3Filter f )
{
	vu64( f.categoryBits );
	vu64( f.maskBits );
	vi32( f.groupIndex );
}

static void checkDefaults( float units )
{
	b3SetLengthUnitsPerMeter( units );

	{
		b3WorldDef d = b3DefaultWorldDef();
		vtag( "worlddef" );
		pv3( d.gravity );
		vf32( d.restitutionThreshold );
		vi32( d.restitutionIterations );
		vbool( d.enableRestitutionPropagation );
		vf32( d.hitEventThreshold );
		vf32( d.contactHertz );
		vf32( d.contactDampingRatio );
		vf32( d.contactSpeed );
		vf32( d.maximumLinearSpeed );
		vf32( d.isotropicInertiaTolerance );
		vbool( d.frictionCallback != NULL );
		vbool( d.restitutionCallback != NULL );
		vbool( d.enableSleep );
		vbool( d.enableContinuous );
		vu32( d.workerCount );
		vbool( d.enqueueTask != NULL );
		vbool( d.finishTask != NULL );
		vu64( (uint64_t)(uintptr_t)d.userTaskContext );
		vu64( (uint64_t)(uintptr_t)d.userData );
		vbool( d.createDebugShape != NULL );
		vbool( d.destroyDebugShape != NULL );
		vu64( (uint64_t)(uintptr_t)d.userDebugShapeContext );
		vi32( d.capacity.staticShapeCount );
		vi32( d.capacity.dynamicShapeCount );
		vi32( d.capacity.staticBodyCount );
		vi32( d.capacity.dynamicBodyCount );
		vi32( d.capacity.contactCount );
		vi32( d.internalValue );
		vend();
	}

	{
		b3BodyDef d = b3DefaultBodyDef();
		vtag( "bodydef" );
		vi32( d.type );
		pv3( d.position );
		pq( d.rotation );
		pv3( d.linearVelocity );
		pv3( d.angularVelocity );
		vf32( d.linearDamping );
		vf32( d.angularDamping );
		vf32( d.gravityScale );
		vf32( d.sleepThreshold );
		vf32( d.safetyFactor );
		vbool( d.name != NULL && d.name[0] != 0 );
		vu64( (uint64_t)(uintptr_t)d.userData );
		vbool( d.motionLocks.linearX );
		vbool( d.motionLocks.linearY );
		vbool( d.motionLocks.linearZ );
		vbool( d.motionLocks.angularX );
		vbool( d.motionLocks.angularY );
		vbool( d.motionLocks.angularZ );
		vbool( d.enableSleep );
		vbool( d.isAwake );
		vbool( d.isBullet );
		vbool( d.isEnabled );
		vbool( d.allowFastRotation );
		vbool( d.enableContactRecycling );
		vi32( d.internalValue );
		vend();
	}

	vtag( "filter" );
	pfilter( b3DefaultFilter() );
	vend();

	{
		b3QueryFilter f = b3DefaultQueryFilter();
		vtag( "queryfilter" );
		vu64( f.categoryBits );
		vu64( f.maskBits );
		vu64( f.id );
		vbool( f.name != NULL );
		vend();
	}

	vtag( "surface" );
	psurface( b3DefaultSurfaceMaterial() );
	vend();

	{
		b3ShapeDef d = b3DefaultShapeDef();
		vtag( "shapedef" );
		vbool( d.name != NULL && d.name[0] != 0 );
		vu64( (uint64_t)(uintptr_t)d.userData );
		vbool( d.materials != NULL );
		vi32( d.materialCount );
		psurface( d.baseMaterial );
		vf32( d.density );
		vf32( d.explosionScale );
		pfilter( d.filter );
		vbool( d.enableCustomFiltering );
		vbool( d.isSensor );
		vbool( d.enableSensorEvents );
		vbool( d.enableContactEvents );
		vbool( d.enableHitEvents );
		vbool( d.enablePreSolveEvents );
		vbool( d.invokeContactCreation );
		vbool( d.updateBodyMass );
		vbool( d.enableSpeculativeContact );
		vi32( d.internalValue );
		vend();
	}

	{
		b3ParallelJointDef d = b3DefaultParallelJointDef();
		vtag( "parallel" );
		pjointdef( d.base );
		vf32( d.hertz );
		vf32( d.dampingRatio );
		vf32( d.maxTorque );
		vend();
	}

	{
		b3DistanceJointDef d = b3DefaultDistanceJointDef();
		vtag( "distance" );
		pjointdef( d.base );
		vf32( d.length );
		vbool( d.enableSpring );
		vf32( d.lowerSpringForce );
		vf32( d.upperSpringForce );
		vf32( d.hertz );
		vf32( d.dampingRatio );
		vbool( d.enableLimit );
		vf32( d.minLength );
		vf32( d.maxLength );
		vbool( d.enableMotor );
		vf32( d.maxMotorForce );
		vf32( d.motorSpeed );
		vend();
	}

	{
		b3MotorJointDef d = b3DefaultMotorJointDef();
		vtag( "motor" );
		pjointdef( d.base );
		pv3( d.linearVelocity );
		vf32( d.maxVelocityForce );
		pv3( d.angularVelocity );
		vf32( d.maxVelocityTorque );
		vf32( d.linearHertz );
		vf32( d.linearDampingRatio );
		vf32( d.maxSpringForce );
		vf32( d.angularHertz );
		vf32( d.angularDampingRatio );
		vf32( d.maxSpringTorque );
		vend();
	}

	{
		b3FilterJointDef d = b3DefaultFilterJointDef();
		vtag( "filterjoint" );
		pjointdef( d.base );
		vend();
	}

	{
		b3PrismaticJointDef d = b3DefaultPrismaticJointDef();
		vtag( "prismatic" );
		pjointdef( d.base );
		vbool( d.enableSpring );
		vf32( d.hertz );
		vf32( d.dampingRatio );
		vf32( d.targetTranslation );
		vbool( d.enableLimit );
		vf32( d.lowerTranslation );
		vf32( d.upperTranslation );
		vbool( d.enableMotor );
		vf32( d.maxMotorForce );
		vf32( d.motorSpeed );
		vend();
	}

	{
		b3RevoluteJointDef d = b3DefaultRevoluteJointDef();
		vtag( "revolute" );
		pjointdef( d.base );
		vf32( d.targetAngle );
		vbool( d.enableSpring );
		vf32( d.hertz );
		vf32( d.dampingRatio );
		vbool( d.enableLimit );
		vf32( d.lowerAngle );
		vf32( d.upperAngle );
		vbool( d.enableMotor );
		vf32( d.maxMotorTorque );
		vf32( d.motorSpeed );
		vend();
	}

	{
		b3SphericalJointDef d = b3DefaultSphericalJointDef();
		vtag( "spherical" );
		pjointdef( d.base );
		vbool( d.enableSpring );
		vf32( d.hertz );
		vf32( d.dampingRatio );
		pq( d.targetRotation );
		vbool( d.enableConeLimit );
		vf32( d.coneAngle );
		vbool( d.enableTwistLimit );
		vf32( d.lowerTwistAngle );
		vf32( d.upperTwistAngle );
		vbool( d.enableMotor );
		vf32( d.maxMotorTorque );
		pv3( d.motorVelocity );
		vend();
	}

	{
		b3WeldJointDef d = b3DefaultWeldJointDef();
		vtag( "weld" );
		pjointdef( d.base );
		vf32( d.linearHertz );
		vf32( d.angularHertz );
		vf32( d.linearDampingRatio );
		vf32( d.angularDampingRatio );
		vend();
	}

	{
		b3WheelJointDef d = b3DefaultWheelJointDef();
		vtag( "wheel" );
		pjointdef( d.base );
		vbool( d.enableSuspensionSpring );
		vf32( d.suspensionHertz );
		vf32( d.suspensionDampingRatio );
		vbool( d.enableSuspensionLimit );
		vf32( d.lowerSuspensionLimit );
		vf32( d.upperSuspensionLimit );
		vbool( d.enableSpinMotor );
		vf32( d.maxSpinTorque );
		vf32( d.spinSpeed );
		vbool( d.enableSteering );
		vf32( d.steeringHertz );
		vf32( d.steeringDampingRatio );
		vf32( d.targetSteeringAngle );
		vf32( d.maxSteeringTorque );
		vbool( d.enableSteeringLimit );
		vf32( d.lowerSteeringLimit );
		vf32( d.upperSteeringLimit );
		vend();
	}

	{
		b3ExplosionDef d = b3DefaultExplosionDef();
		vtag( "explosion" );
		vu64( d.maskBits );
		pv3( d.position );
		vf32( d.radius );
		vf32( d.falloff );
		vf32( d.impulsePerArea );
		vend();
	}
}


static void checkScalars( void )
{
	for ( int i = 0; i < 6000; ++i )
	{
		float a = vspecialf();
		float b = vspecialf();
		float c = vspecialf();
		float alpha = vrandf( 0.0f, 1.0f );
		float r = vrandf( -20.0f, 20.0f );
		float r2 = vrandf( -20.0f, 20.0f );
		int ia = (int)vrand32();
		int ib = vrandint( -100, 100 );
		int ic = vrandint( -100, 100 );
		int n = vrandint( 0, 10000 );
		int e = vrandint( 1, 10 );
		int d = vrandint( 1, 100 );

		vtag( "s1" );
		vf32( a );
		vf32( b );
		vf32( c );
		vi32( b3MinInt( ia, ib ) );
		vi32( b3MaxInt( ia, ib ) );
		vi32( b3ClampInt( ia, b3MinInt( ib, ic ), b3MaxInt( ib, ic ) ) );
		vi32( b3ClampInt( ib, -10, 10 ) );
		vf32( b3AbsFloat( a ) );
		vf32( b3MinFloat( a, b ) );
		vf32( b3MaxFloat( a, b ) );
		vf32( b3ClampFloat( a, b, c ) );
		vf32( b3LerpFloat( a, b, alpha ) );
		vbool( b3IsValidFloat( a ) );
		vi32( (int)a );
		vu32( (uint32_t)a );
		vend();

		vtag( "s2" );
		vf32( b3Atan2( a, b ) );
		vf32( b3Atan2( r, r2 ) );
		vf32( b3Atan2( r, 0.0f ) );
		vf32( b3Atan2( 0.0f, r2 ) );
		b3CosSin cs = b3ComputeCosSin( a );
		vf32( cs.cosine );
		vf32( cs.sine );
		cs = b3ComputeCosSin( r );
		vf32( cs.cosine );
		vf32( cs.sine );
		vf32( b3Sin( r2 ) );
		vf32( b3Cos( r2 ) );
		vf32( b3UnwindAngle( a ) );
		vf32( b3UnwindAngle( r ) );
		vi32( (int)b3AlignUp8( (size_t)n ) );
		vi32( b3CeilingInt( n, d ) );
		vi32( b3CeilingPow2( n, 1 << e, e ) );
		vend();
	}
}

static void checkVectors( void )
{
	for ( int i = 0; i < 6000; ++i )
	{
		b3Vec3 a = rvec();
		b3Vec3 b = rvec();
		b3Vec3 c = rvec();
		float s = rscalar();
		float t = rscalar();
		float alpha = vrandf( 0.0f, 1.0f );
		int k = (int)( vrand32() % 3 );
		b3Vec3 u = runit();

		vtag( "v1" );
		pv3( a );
		pv3( b );
		pv3( c );
		vf32( s );
		vf32( t );
		pv3( b3Add( a, b ) );
		pv3( b3Sub( a, b ) );
		pv3( b3Mul( a, b ) );
		pv3( b3Neg( a ) );
		vf32( b3Dot( a, b ) );
		vf32( b3Length( a ) );
		vf32( b3LengthSquared( a ) );
		vf32( b3Distance( a, b ) );
		vf32( b3DistanceSquared( a, b ) );
		pv3( b3Normalize( a ) );
		float length = -7.0f;
		b3Vec3 n = b3GetLengthAndNormalize( &length, a );
		vf32( length );
		pv3( n );
		pv3( b3Perp( a ) );
		vbool( b3IsNormalized( a ) );
		vbool( b3IsNormalized( u ) );
		vend();

		vtag( "v2" );
		pv3( b3MulAdd( a, s, b ) );
		pv3( b3MulSub( a, s, b ) );
		pv3( b3MulSV( s, a ) );
		pv3( b3Cross( a, b ) );
		pv3( b3Lerp( a, b, alpha ) );
		pv3( b3Blend2( s, a, t, b ) );
		pv3( b3Abs( a ) );
		pv3( b3Sign( a ) );
		pv3( b3Min( a, b ) );
		pv3( b3Max( a, b ) );
		pv3( b3Clamp( a, b, c ) );
		pv3( b3SafeScale( quietVec( a ) ) );
		vbool( b3IsValidVec3( a ) );
		pv3( b3ClampLength( a, b3AbsFloat( s ) ) );
		vend();

		vtag( "v3" );
		vf32( b3ScalarTripleProduct( a, b, c ) );
		vf32( b3GetByIndex( a, k ) );
		vi32( b3MajorAxis( a ) );
		vf32( b3MinElement( a ) );
		vf32( b3MaxElement( a ) );
		vi32( b3MaxElementIndex( a ) );
		pv3( b3Blend3( s, a, t, b, alpha, c ) );
		pv3( b3ModifiedCross( a, b ) );
		pv3( b3ToPos( a ) );
		pv3( b3ToVec3( a ) );
		pv3( b3SubPos( a, b ) );
		pv3( b3OffsetPos( a, b ) );
		pv3( b3LerpPosition( a, b, alpha ) );
		vf32( b3RoundDownFloat( a.x ) );
		vf32( b3RoundUpFloat( a.y ) );
		vbool( b3IsValidPosition( a ) );
		pv3( b3ArbitraryPerp( u ) );
		vend();
	}
}

static void checkVec2( void )
{
	for ( int i = 0; i < 3000; ++i )
	{
		b3Vec2 a = rvec2();
		b3Vec2 b = rvec2();
		float s = rscalar();
		b3Matrix2 m;
		m.cx = rvec2();
		m.cy = rvec2();
		b3Matrix2 m2;
		m2.cx = rvec2();
		m2.cy = rvec2();

		// positive definite
		b3Matrix2 p;
		float px = vrandf( 0.1f, 10.0f );
		float py = vrandf( 0.1f, 10.0f );
		float pxy = vrandf( -0.05f, 0.05f );
		p.cx = (b3Vec2){ px, pxy };
		p.cy = (b3Vec2){ pxy, py };

		vtag( "w" );
		pv2( a );
		pv2( b );
		vf32( s );
		vf32( b3Dot2( a, b ) );
		vf32( b3Length2( a ) );
		vf32( b3LengthSquared2( a ) );
		pv2( b3MinVec2( a, b ) );
		pv2( b3MaxVec2( a, b ) );
		pv2( b3Add2( a, b ) );
		pv2( b3Sub2( a, b ) );
		pv2( b3Neg2( a ) );
		pv2( b3MulSV2( s, a ) );
		pv2( b3MulAdd2( a, s, b ) );
		pv2( b3MulSub2( a, s, b ) );
		vf32( b3Cross2( a, b ) );
		vf32( b3DistanceSquared2( a, b ) );
		pv2( b3MulMV2( m, a ) );
		pm2( b3MulMM2( m, m2 ) );
		vf32( b3Det2( m ) );
		pm2( b3Invert2( m ) );
		pv2( b3Solve2( m, a ) );
		pv2( b3Solve2( p, a ) );
		vend();
	}
}

static void checkQuats( void )
{
	for ( int i = 0; i < 4000; ++i )
	{
		b3Quat q1 = rquatAny();
		b3Quat q2 = rquatAny();
		b3Quat n1 = rquat();
		b3Quat n2 = rquat();
		b3Vec3 v = rvec();
		b3Vec3 axis = runit();
		float angle = vrandf( -10.0f, 10.0f );
		float alpha = vrandf( 0.0f, 1.0f );
		b3Vec3 u1 = runit();
		b3Vec3 u2 = runit();
		if ( vrand32() % 4 == 0 )
		{
			u2 = b3Neg( u1 );
		}
		b3Vec3 smallRot = b3MulSV( vrandf( 0.0f, 0.01f ), runit() );
		b3Vec3 bigRot = rvecn();
		b3Vec3 dr = b3MulSV( vrandf( 0.0f, 0.5f ), rvecn() );
		b3Matrix3 m = rmat();

		vtag( "q1" );
		pq( q1 );
		pq( q2 );
		vbool( b3IsNormalizedQuat( q1 ) );
		vbool( b3IsNormalizedQuat( n1 ) );
		vbool( b3IsValidQuat( q1 ) );
		pv3( b3RotateVector( q1, v ) );
		pv3( b3InvRotateVector( q1, v ) );
		vf32( b3DotQuat( q1, q2 ) );
		pq( b3MulQuat( q1, q2 ) );
		pq( b3InvMulQuat( q1, q2 ) );
		pq( b3Conjugate( q1 ) );
		pq( b3NegateQuat( q1 ) );
		pq( b3NormalizeQuat( q1 ) );
		vend();

		vtag( "q2" );
		pq( b3MakeQuatFromAxisAngle( axis, angle ) );
		float radians = -7.0f;
		b3Vec3 ax = b3GetAxisAngle( &radians, n1 );
		vf32( radians );
		pv3( ax );
		vf32( b3GetQuatAngle( n1 ) );
		vf32( b3GetTwistAngle( n1 ) );
		vf32( b3GetSwingAngle( n1 ) );
		pq( b3NLerp( n1, n2, alpha ) );
		b3Matrix3 r = b3MakeMatrixFromQuat( n1 );
		pm3( r );
		pq( b3MakeQuatFromMatrix( &r ) );
		pq( b3MakeQuatFromMatrix( &m ) );
		pm3( b3MakeMatrixFromQuat( q1 ) );
		vend();

		vtag( "q3" );
		pq( b3ComputeQuatBetweenUnitVectors( u1, u2 ) );
		pq( b3QuatFromExponentialMap( smallRot ) );
		pq( b3QuatFromExponentialMap( bigRot ) );
		pq( b3IntegrateRotation( n1, dr ) );
		pq( b3IntegrateRotation( q1, v ) );
		pv3( b3DeltaQuatToRotation( n1, n2 ) );
		pv3( b3DeltaQuatToRotation( q1, q2 ) );
		vend();
	}
}

static void checkTransforms( void )
{
	for ( int i = 0; i < 3000; ++i )
	{
		b3Transform A = rxf();
		b3Transform B = rxf();
		b3Vec3 v = rvec();
		b3Vec3 base = rvecn();
		b3Transform T;
		T.p = rvec();
		T.q = rquatAny();

		vtag( "x" );
		pxf( b3MulTransforms( A, B ) );
		pxf( b3InvMulTransforms( A, B ) );
		pxf( b3InvertTransform( A ) );
		pv3( b3TransformPoint( A, v ) );
		pv3( b3InvTransformPoint( A, v ) );
		pv3( b3TransformWorldPoint( A, v ) );
		pv3( b3InvTransformWorldPoint( A, v ) );
		pxf( b3MulWorldTransforms( A, B ) );
		pxf( b3InvMulWorldTransforms( A, B ) );
		pxf( b3ToRelativeTransform( A, base ) );
		pxf( b3MakeWorldTransform( T ) );
		pxf( b3MulTransforms( T, A ) );
		vbool( b3IsValidTransform( A ) );
		vbool( b3IsValidTransform( T ) );
		vbool( b3IsValidWorldTransform( T ) );
		vend();
	}
}

static void checkMatrices( void )
{
	for ( int i = 0; i < 3000; ++i )
	{
		b3Matrix3 m = rmat();
		b3Matrix3 n = rmat();
		b3Vec3 v = rvec();
		float s = rscalar();
		float mass = vrandf( 0.1f, 10.0f );
		b3Vec3 o = rvecn();
		float radius = vrandf( 0.01f, 5.0f );
		float height = vrandf( 0.01f, 5.0f );
		b3Vec3 lo = rvecn();
		b3Vec3 hi = b3Add( lo, b3Abs( rvecn() ) );
		b3Quat q = rquat();
		b3Transform xf = rxf();

		vtag( "m1" );
		pm3( m );
		pm3( n );
		vf32( b3Det( m ) );
		pv3( b3MulMV( m, v ) );
		pm3( b3NegateMat3( m ) );
		pm3( b3AddMM( m, n ) );
		pm3( b3SubMM( m, n ) );
		pm3( b3MulSM( s, m ) );
		pm3( b3MulMM( m, n ) );
		pm3( b3Transpose( m ) );
		vend();

		vtag( "m2" );
		pm3( b3InvertMatrix( m ) );
		pv3( b3Solve3( m, v ) );
		pm3( b3InvertT( m ) );
		pm3( b3AbsMatrix3( m ) );
		pm3( b3Steiner( mass, o ) );
		vbool( b3IsValidMatrix3( m ) );
		pm3( b3MakeDiagonalMatrix( s, mass, radius ) );
		pm3( b3SphereInertia( mass, radius ) );
		pm3( b3CylinderInertia( mass, radius, height ) );
		pm3( b3BoxInertia( mass, lo, hi ) );
		pm3( b3RotateInertia( q, m ) );
		pm3( b3TransformInertia( xf, m, mass ) );
		vend();
	}
}

static void checkAABBs( void )
{
	for ( int i = 0; i < 4000; ++i )
	{
		b3AABB a = raabb();
		b3AABB b = raabb();
		b3Transform xf = rxf();
		b3Vec3 p = rvec();
		float ext = rscalar();
		b3Vec3 origin = rvecn();
		int count = vrandint( 1, 9 );
		b3Vec3 points[8];
		for ( int j = 0; j < count; ++j )
		{
			points[j] = rvecn();
		}
		float radius = vrandf( 0.0f, 1.0f );
		float scale = vrandf( 1.0f, 40000.0f );
		b3AABB big = { b3MulSV( scale, a.lowerBound ), b3MulSV( scale, a.upperBound ) };

		vtag( "a1" );
		pab( a );
		pab( b );
		vbool( b3AABB_Contains( a, b ) );
		vf32( b3AABB_Area( a ) );
		pv3( b3AABB_Center( a ) );
		pv3( b3AABB_Extents( a ) );
		pab( b3AABB_Union( a, b ) );
		pab( b3AABB_Inflate( a, ext ) );
		vbool( b3AABB_Overlaps( a, b ) );
		pab( b3AABB_Transform( xf, a ) );
		pv3( b3ClosestPointToAABB( p, a ) );
		pab( b3OffsetAABB( a, origin ) );
		vend();

		vtag( "a2" );
		vbool( b3IsValidAABB( a ) );
		vbool( b3IsBoundedAABB( a ) );
		vbool( b3IsSaneAABB( a ) );
		vbool( b3IsBoundedAABB( big ) );
		vbool( b3IsSaneAABB( big ) );
		pab( b3AABB_AddPoint( a, p ) );
		vf32( b3Perimeter( a ) );
		b3AABB c = a;
		vbool( b3EnlargeAABB( &c, b ) );
		pab( c );
		pv3( b3FarthestPointOnAABB( a, p ) );
		pab( b3MakeAABB( points, count, radius ) );
		vend();

		b3AABB box;
		b3Vec3 center = rvecn();
		b3Vec3 extent = b3Add( b3Abs( rvecn() ), (b3Vec3){ 0.1f, 0.1f, 0.1f } );
		box.lowerBound = b3Sub( center, extent );
		box.upperBound = b3Add( center, extent );
		b3Vec3 p1 = b3MulSV( 2.0f, rvecn() );
		b3Vec3 p2;
		uint32_t kind = vrand32() % 5;
		if ( kind == 0 )
		{
			p2 = p1;
		}
		else if ( kind == 1 )
		{
			p2 = b3Add( p1, (b3Vec3){ 0.0f, 0.0f, vrandf( -30.0f, 30.0f ) } );
		}
		else if ( kind == 2 )
		{
			p2 = b3Add( center, b3MulSV( 0.5f, rvecn() ) );
			p1 = b3Add( center, b3MulSV( 0.1f, rvecn() ) );
		}
		else if ( kind == 3 )
		{
			p2 = b3Add( center, b3MulSV( 0.5f, rvecn() ) );
		}
		else
		{
			p2 = b3MulSV( 2.0f, rvecn() );
		}

		float minFraction = -7.0f;
		float maxFraction = -7.0f;
		bool hit = b3RayCastAABB( box, p1, p2, &minFraction, &maxFraction );
		vtag( "a3" );
		vbool( hit );
		vf32( minFraction );
		vf32( maxFraction );
		vend();
	}
}

static void checkPlanes( void )
{
	for ( int i = 0; i < 3000; ++i )
	{
		b3Plane pl = rplane();
		b3Transform xf = rxf();
		b3Vec3 p = rvec();
		b3Vec3 p1 = rvecn();
		b3Vec3 p2 = rvecn();
		b3Vec3 p3 = rvecn();
		if ( vrand32() % 5 == 0 )
		{
			p3 = b3MulAdd( p1, vrandf( -2.0f, 2.0f ), b3Sub( p2, p1 ) );
		}
		b3Vec3 n = runit();
		b3Plane raw;
		raw.normal = rvecn();
		raw.offset = vrandf( -10.0f, 10.0f );

		vtag( "p" );
		ppl( pl );
		vbool( b3IsValidPlane( pl ) );
		ppl( b3NormalizePlane( raw ) );
		ppl( b3MakePlaneFromNormalAndPoint( n, p ) );
		ppl( b3MakePlaneFromPoints( p1, p2, p3 ) );
		pv3( b3MakeNormalFromPoints( p1, p2, p3 ) );
		ppl( b3TransformPlane( xf, pl ) );
		vf32( b3PlaneSeparation( pl, p ) );
		vf32( b3SignedVolume( p1, p2, p3, p ) );
		vend();
	}
}

static void checkSegments( void )
{
	for ( int i = 0; i < 4000; ++i )
	{
		uint32_t mode = vrand32() % 8;
		b3Vec3 p1, q1, p2, q2;
		if ( mode == 0 )
		{
			p1 = rvec();
			q1 = rvec();
			p2 = rvec();
			q2 = rvec();
		}
		else
		{
			p1 = rvecn();
			q1 = rvecn();
			p2 = rvecn();
			q2 = rvecn();
			if ( mode == 1 )
			{
				q1 = p1;
			}
			else if ( mode == 2 )
			{
				q2 = p2;
			}
			else if ( mode == 3 )
			{
				q2 = b3MulAdd( p2, vrandf( -2.0f, 2.0f ), b3Sub( q1, p1 ) );
			}
			else if ( mode == 4 )
			{
				q1 = b3MulAdd( p1, 1.0e-4f, rvecn() );
				q2 = b3MulAdd( p2, 1.0e-4f, rvecn() );
			}
			else if ( mode == 5 )
			{
				q1 = p1;
				q2 = p2;
			}
		}

		vtag( "g" );
		b3SegmentDistanceResult r = b3SegmentDistance( p1, q1, p2, q2 );
		pseg( r );
		vbool( b3IsWithinSegments( &r ) );
		b3SegmentDistanceResult l = b3LineDistance( p1, b3Sub( q1, p1 ), p2, b3Sub( q2, p2 ) );
		pseg( l );
		vbool( b3IsWithinSegments( &l ) );
		pv3( b3PointToSegmentDistance( p1, q1, p2 ) );
		vend();
	}
}

static void checkTriangles( void )
{
	for ( int i = 0; i < 4000; ++i )
	{
		b3Vec3 a = rvecn();
		b3Vec3 b = rvecn();
		b3Vec3 c = rvecn();
		uint32_t mode = vrand32() % 6;
		if ( mode == 0 )
		{
			c = b3MulAdd( a, vrandf( -2.0f, 2.0f ), b3Sub( b, a ) );
		}
		float s = vrandf( -1.0f, 2.0f );
		float t = vrandf( -1.0f, 2.0f );
		float h = vrandf( -3.0f, 3.0f );
		b3Vec3 q;
		if ( mode == 1 )
		{
			q = rvec();
		}
		else
		{
			b3Vec3 n = b3Normalize( b3Cross( b3Sub( b, a ), b3Sub( c, a ) ) );
			q = b3MulAdd( a, s, b3Sub( b, a ) );
			q = b3MulAdd( q, t, b3Sub( c, a ) );
			q = b3MulAdd( q, h, n );
		}

		b3TrianglePoint tp = b3ClosestPointOnTriangle( a, b, c, q );
		vtag( "t" );
		pv3( q );
		pv3( tp.point );
		vi32( tp.feature );
		vend();
	}
}

static void checkRays( void )
{
	for ( int i = 0; i < 2000; ++i )
	{
		b3RayCastInput input;
		input.origin = rvec();
		input.translation = rvec();
		if ( vrand32() % 2 == 0 )
		{
			input.maxFraction = vspecialf();
		}
		else
		{
			input.maxFraction = vrandf( -0.5f, 1.5f );
		}

		vtag( "r" );
		vbool( b3IsValidRay( &input ) );
		vend();
	}
}

static void checkHash( void )
{
	uint8_t buffer[512];
	for ( int i = 0; i < 400; ++i )
	{
		int len;
		if ( i < 260 )
		{
			len = i;
		}
		else
		{
			len = vrandint( 0, 512 );
		}

		for ( int j = 0; j < len; ++j )
		{
			buffer[j] = (uint8_t)( vrand32() & 0xff );
		}

		vtag( "h" );
		vi32( len );
		vu32( b3Hash( B3_HASH_INIT, buffer, len ) );
		vu64( b3Hash64NonZero( buffer, len ) );
		vend();
	}
}

static void checkIds( void )
{
	for ( int i = 0; i < 2000; ++i )
	{
		uint32_t x32 = vrand32();
		uint64_t x64 = vrand64();
		uint64_t y64 = vrand64();
		uint64_t z64 = vrand64();
		uint32_t values[3];
		values[0] = vrand32();
		values[1] = vrand32();
		values[2] = vrand32();

		vtag( "id" );
		b3WorldId w = b3LoadWorldId( x32 );
		vu32( w.index1 );
		vu32( w.generation );
		vu32( b3StoreWorldId( w ) );
		b3BodyId bo = b3LoadBodyId( x64 );
		vi32( bo.index1 );
		vu32( bo.world0 );
		vu32( bo.generation );
		vu64( b3StoreBodyId( bo ) );
		b3ShapeId sh = b3LoadShapeId( y64 );
		vi32( sh.index1 );
		vu32( sh.world0 );
		vu32( sh.generation );
		vu64( b3StoreShapeId( sh ) );
		b3JointId jo = b3LoadJointId( z64 );
		vi32( jo.index1 );
		vu32( jo.world0 );
		vu32( jo.generation );
		vu64( b3StoreJointId( jo ) );
		b3ContactId co = b3LoadContactId( values );
		vi32( co.index1 );
		vu32( co.world0 );
		vi32( co.padding );
		vu32( co.generation );
		uint32_t out[3];
		b3StoreContactId( co, out );
		vu32( out[0] );
		vu32( out[1] );
		vu32( out[2] );
		vbool( B3_ID_EQUALS( bo, sh ) );
		vbool( B3_ID_EQUALS( bo, bo ) );
		vbool( B3_IS_NULL( bo ) );
		vend();
	}
}

// Mirrors the blob layout code in b3CreateHull, b3CreateMesh and b3CreateHeightField.
static int hullSoaStride( int count )
{
	return ( count + 8 - 1 ) & ~( 8 - 1 );
}

static void hullLayout( int vertexCount, int edgeCount, int faceCount )
{
	int soaVertexCount = hullSoaStride( vertexCount );
	int soaNormalCount = hullSoaStride( faceCount );
	size_t byteCount = b3AlignUp8( sizeof( b3HullData ) );
	int vertexOffset = (int)byteCount;
	byteCount += b3AlignUp8( vertexCount * (int)sizeof( b3HullVertex ) );
	int pointOffset = (int)byteCount;
	byteCount += b3AlignUp8( vertexCount * (int)sizeof( b3Vec3 ) );
	int edgeOffset = (int)byteCount;
	byteCount += b3AlignUp8( edgeCount * (int)sizeof( b3HullHalfEdge ) );
	int planeOffset = (int)byteCount;
	byteCount += b3AlignUp8( faceCount * (int)sizeof( b3Plane ) );
	int faceOffset = (int)byteCount;
	byteCount += b3AlignUp8( faceCount * (int)sizeof( b3HullFace ) );
	int soaVertexOffset = (int)byteCount;
	byteCount += b3AlignUp8( 3 * soaVertexCount * (int)sizeof( float ) );
	int soaNormalOffset = (int)byteCount;
	byteCount += b3AlignUp8( 3 * soaNormalCount * (int)sizeof( float ) );
	int edgeCosineOffset = (int)byteCount;
	byteCount += b3AlignUp8( ( edgeCount / 2 ) * (int)sizeof( float ) );

	vtag( "hull" );
	vi32( vertexCount );
	vi32( edgeCount );
	vi32( faceCount );
	vi32( vertexOffset );
	vi32( pointOffset );
	vi32( edgeOffset );
	vi32( planeOffset );
	vi32( faceOffset );
	vi32( soaVertexOffset );
	vi32( soaNormalOffset );
	vi32( edgeCosineOffset );
	vi32( (int)byteCount );
	vi32( vertexCount );
	vi32( vertexCount );
	vi32( edgeCount );
	vi32( faceCount );
	vi32( faceCount );
	vi32( 3 * soaVertexCount );
	vi32( 3 * soaNormalCount );
	vi32( edgeCount / 2 );
	vend();
}

static void meshLayout( int nodeCount, int vertexCount, int triangleCount )
{
	size_t byteCount = b3AlignUp8( sizeof( b3MeshData ) );
	int nodeOffset = (int)byteCount;
	byteCount += b3AlignUp8( nodeCount * sizeof( b3MeshNode ) );
	int vertexOffset = (int)byteCount;
	byteCount += b3AlignUp8( vertexCount * sizeof( b3Vec3 ) );
	int triangleOffset = (int)byteCount;
	byteCount += b3AlignUp8( triangleCount * sizeof( b3MeshTriangle ) );
	int materialIndicesOffset = (int)byteCount;
	byteCount += b3AlignUp8( triangleCount * sizeof( uint8_t ) );
	int flagsOffset = (int)byteCount;
	byteCount += b3AlignUp8( triangleCount * sizeof( uint8_t ) );

	vtag( "mesh" );
	vi32( nodeCount );
	vi32( vertexCount );
	vi32( triangleCount );
	vi32( nodeOffset );
	vi32( vertexOffset );
	vi32( triangleOffset );
	vi32( materialIndicesOffset );
	vi32( flagsOffset );
	vi32( (int)byteCount );
	vend();
}

static void heightLayout( int columnCount, int rowCount )
{
	int heightCount = columnCount * rowCount;
	int cellCount = ( columnCount - 1 ) * ( rowCount - 1 );
	int triangleCount = 2 * cellCount;
	size_t byteCount = b3AlignUp8( sizeof( b3HeightFieldData ) );
	int heightsOffset = (int)byteCount;
	byteCount += b3AlignUp8( heightCount * sizeof( uint16_t ) );
	int materialOffset = (int)byteCount;
	byteCount += b3AlignUp8( cellCount * sizeof( uint8_t ) );
	int flagsOffset = (int)byteCount;
	byteCount += b3AlignUp8( triangleCount * sizeof( uint8_t ) );

	vtag( "height" );
	vi32( columnCount );
	vi32( rowCount );
	vi32( heightsOffset );
	vi32( materialOffset );
	vi32( flagsOffset );
	vi32( (int)byteCount );
	vi32( heightCount );
	vi32( cellCount );
	vi32( triangleCount );
	vend();
}

static void checkLayout( void )
{
	vtag( "sizes" );
	vi32( (int)sizeof( b3HullData ) );
	vi32( (int)sizeof( b3MeshData ) );
	vi32( (int)sizeof( b3MeshNode ) );
	vi32( (int)sizeof( b3HeightFieldData ) );
	vi32( (int)sizeof( b3CompoundData ) );
	vend();

	vtag( "box" );
	vi32( (int)sizeof( b3BoxHull ) );
	vi32( (int)offsetof( b3BoxHull, boxVertices ) );
	vi32( (int)offsetof( b3BoxHull, boxPoints ) );
	vi32( (int)offsetof( b3BoxHull, boxEdges ) );
	vi32( (int)offsetof( b3BoxHull, boxPlanes ) );
	vi32( (int)offsetof( b3BoxHull, boxFaces ) );
	vi32( (int)offsetof( b3BoxHull, vx ) );
	vi32( (int)offsetof( b3BoxHull, nx ) );
	vi32( (int)offsetof( b3BoxHull, edgeCosines ) );
	vi32( B3_ARRAY_COUNT( ( (b3BoxHull*)0 )->boxVertices ) );
	vi32( B3_ARRAY_COUNT( ( (b3BoxHull*)0 )->boxPoints ) );
	vi32( B3_ARRAY_COUNT( ( (b3BoxHull*)0 )->boxEdges ) );
	vi32( B3_ARRAY_COUNT( ( (b3BoxHull*)0 )->boxPlanes ) );
	vi32( B3_ARRAY_COUNT( ( (b3BoxHull*)0 )->boxFaces ) );
	vi32( 3 * B3_ARRAY_COUNT( ( (b3BoxHull*)0 )->vx ) );
	vi32( 3 * B3_ARRAY_COUNT( ( (b3BoxHull*)0 )->nx ) );
	vi32( B3_ARRAY_COUNT( ( (b3BoxHull*)0 )->edgeCosines ) );
	vend();

	hullLayout( 4, 12, 4 );
	hullLayout( 8, 24, 6 );
	hullLayout( 128, 254, 128 );
	for ( int i = 0; i < 50; ++i )
	{
		int v = vrandint( 4, 129 );
		int e = 2 * vrandint( 6, 128 );
		int f = vrandint( 4, 129 );
		hullLayout( v, e, f );
	}

	for ( int i = 0; i < 50; ++i )
	{
		int n = vrandint( 1, 1000 );
		int v = vrandint( 3, 1000 );
		int t = vrandint( 1, 1000 );
		meshLayout( n, v, t );
	}

	for ( int i = 0; i < 50; ++i )
	{
		int c = vrandint( 2, 100 );
		int r = vrandint( 2, 100 );
		heightLayout( c, r );
	}
}

int main( void )
{
	vseed( 0x5eed );
	checkConstants();
	checkUnitConstants( 1.0f );
	checkDefaults( 1.0f );
	checkUnitConstants( 0.37f );
	checkDefaults( 0.37f );
	checkUnitConstants( 100.0f );
	b3SetLengthUnitsPerMeter( 1.0f );
	checkScalars();
	checkVectors();
	checkVec2();
	checkQuats();
	checkTransforms();
	checkMatrices();
	checkAABBs();
	checkPlanes();
	checkSegments();
	checkTriangles();
	checkRays();
	checkHash();
	checkIds();
	checkLayout();
	return 0;
}
