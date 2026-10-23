// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Joint parity driver: one world with every joint type, stepped with mid run API changes, joint
// creation into sleeping, merged and disabled sets, destruction and joint events. Prints body
// motion, the public joint getters and the internal joint sim state. Mirrored by verify/as/joints.as.

#include "common.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include "joint.h"
#include "physics_world.h"

static b3WorldId g_world;
static b3BodyId g_bodies[32];
static int g_bodyCount;
static b3JointId g_joints[32];
static bool g_alive[32];
static int g_jointCount;

#define STEP_COUNT 130

static void vvec( b3Vec3 v )
{
	vf32( v.x );
	vf32( v.y );
	vf32( v.z );
}

static void vquat( b3Quat q )
{
	vvec( q.v );
	vf32( q.s );
}

static void vsoft( b3Softness s )
{
	vf32( s.biasRate );
	vf32( s.massScale );
	vf32( s.impulseScale );
}

static void vxf( b3Transform t )
{
	vvec( t.p );
	vquat( t.q );
}

static b3BodyId AddBody( b3BodyType type, b3Vec3 position )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = type;
	bodyDef.position = position;
	b3BodyId id = b3CreateBody( g_world, &bodyDef );
	g_bodies[g_bodyCount] = id;
	g_bodyCount += 1;
	return id;
}

static b3BodyId AddBall( b3BodyType type, b3Vec3 position, float radius )
{
	b3BodyId id = AddBody( type, position );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { b3Vec3_zero, radius };
	b3CreateSphereShape( id, &shapeDef, &sphere );
	return id;
}

static b3BodyId AddBox( b3BodyType type, b3Vec3 position, float hx, float hy, float hz )
{
	b3BodyId id = AddBody( type, position );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( hx, hy, hz );
	b3CreateHullShape( id, &shapeDef, &box.base );
	return id;
}

static void AddJoint( b3JointId id )
{
	g_joints[g_jointCount] = id;
	g_alive[g_jointCount] = true;
	g_jointCount += 1;
}

static b3JointDef MakeBase( b3JointDef base, b3BodyId bodyIdA, b3BodyId bodyIdB, b3Vec3 pivot )
{
	base.bodyIdA = bodyIdA;
	base.bodyIdB = bodyIdB;
	b3Vec3 pA = b3Body_GetPosition( bodyIdA );
	b3Vec3 pB = b3Body_GetPosition( bodyIdB );
	base.localFrameA.p = b3Sub( pivot, pA );
	base.localFrameB.p = b3Sub( pivot, pB );
	return base;
}

static b3BodyId g_ground;
static b3BodyId g_sleepA, g_sleepB, g_sleepC, g_disabled;

static void Setup( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	g_world = b3CreateWorld( &worldDef );

	g_ground = AddBox( b3_staticBody, b3Vec3_zero, 20.0f, 1.0f, 20.0f );

	// 0: rigid distance pendulum
	b3BodyId ball1 = AddBall( b3_dynamicBody, (b3Vec3){ -14.0f, 8.0f, 0.0f }, 0.25f );
	{
		b3DistanceJointDef def = b3DefaultDistanceJointDef();
		def.base = MakeBase( def.base, g_ground, ball1, (b3Vec3){ -16.0f, 8.0f, 0.0f } );
		def.base.localFrameB.p = b3Vec3_zero;
		def.length = 2.0f;
		AddJoint( b3CreateDistanceJoint( g_world, &def ) );
	}

	// 1: soft distance with limit and motor
	b3BodyId ball2 = AddBall( b3_dynamicBody, (b3Vec3){ -14.0f, 6.0f, 0.5f }, 0.25f );
	{
		b3DistanceJointDef def = b3DefaultDistanceJointDef();
		def.base.bodyIdA = ball1;
		def.base.bodyIdB = ball2;
		def.base.forceThreshold = 1.0f;
		def.base.userData = (void*)(uintptr_t)7;
		def.length = 2.0f;
		def.enableSpring = true;
		def.hertz = 2.0f;
		def.dampingRatio = 0.5f;
		def.lowerSpringForce = -100.0f;
		def.upperSpringForce = 100.0f;
		def.enableLimit = true;
		def.minLength = 1.0f;
		def.maxLength = 3.0f;
		def.enableMotor = true;
		def.maxMotorForce = 5.0f;
		def.motorSpeed = 0.5f;
		AddJoint( b3CreateDistanceJoint( g_world, &def ) );
	}

	// 2: motor joint
	b3BodyId box3 = AddBox( b3_dynamicBody, (b3Vec3){ -10.0f, 5.0f, 0.0f }, 0.5f, 0.5f, 0.5f );
	{
		b3MotorJointDef def = b3DefaultMotorJointDef();
		def.base = MakeBase( def.base, g_ground, box3, (b3Vec3){ -10.0f, 5.0f, 0.0f } );
		def.linearVelocity = (b3Vec3){ 0.5f, 0.0f, 0.0f };
		def.maxVelocityForce = 50.0f;
		def.angularVelocity = (b3Vec3){ 0.0f, 1.0f, 0.0f };
		def.maxVelocityTorque = 20.0f;
		def.linearHertz = 2.0f;
		def.linearDampingRatio = 1.0f;
		def.maxSpringForce = 20.0f;
		def.angularHertz = 2.0f;
		def.angularDampingRatio = 1.0f;
		def.maxSpringTorque = 10.0f;
		AddJoint( b3CreateMotorJoint( g_world, &def ) );
	}

	// 3: parallel
	b3BodyId box4 = AddBox( b3_dynamicBody, (b3Vec3){ -6.0f, 5.0f, 0.0f }, 0.5f, 0.25f, 0.5f );
	b3Body_SetAngularVelocity( box4, (b3Vec3){ 1.0f, 0.5f, 0.0f } );
	{
		b3ParallelJointDef def = b3DefaultParallelJointDef();
		def.base = MakeBase( def.base, g_ground, box4, (b3Vec3){ -6.0f, 5.0f, 0.0f } );
		def.hertz = 5.0f;
		def.dampingRatio = 1.0f;
		def.maxTorque = 100.0f;
		AddJoint( b3CreateParallelJoint( g_world, &def ) );
	}

	// 4: prismatic on a tilted axis with limit, motor and spring
	b3BodyId box5 = AddBox( b3_dynamicBody, (b3Vec3){ -2.0f, 5.0f, 0.0f }, 0.5f, 0.5f, 0.5f );
	{
		b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
		def.base = MakeBase( def.base, g_ground, box5, (b3Vec3){ -2.0f, 5.0f, 0.0f } );
		def.base.localFrameA.q = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.25f * B3_PI );
		def.base.localFrameB.q = def.base.localFrameA.q;
		def.enableLimit = true;
		def.lowerTranslation = -1.0f;
		def.upperTranslation = 1.0f;
		def.enableMotor = true;
		def.maxMotorForce = 20.0f;
		def.motorSpeed = 1.0f;
		def.enableSpring = true;
		def.hertz = 1.0f;
		def.dampingRatio = 0.5f;
		def.targetTranslation = 0.25f;
		AddJoint( b3CreatePrismaticJoint( g_world, &def ) );
	}

	// 5: revolute hinged at a box edge
	b3BodyId box6 = AddBox( b3_dynamicBody, (b3Vec3){ 2.5f, 5.0f, 0.0f }, 0.5f, 0.25f, 0.25f );
	{
		b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
		def.base = MakeBase( def.base, g_ground, box6, (b3Vec3){ 2.0f, 5.0f, 0.0f } );
		def.base.torqueThreshold = 0.5f;
		def.base.userData = (void*)(uintptr_t)11;
		def.enableLimit = true;
		def.lowerAngle = -0.5f;
		def.upperAngle = 0.8f;
		def.enableMotor = true;
		def.maxMotorTorque = 5.0f;
		def.motorSpeed = 2.0f;
		def.enableSpring = true;
		def.hertz = 1.0f;
		def.dampingRatio = 0.3f;
		def.targetAngle = 0.2f;
		AddJoint( b3CreateRevoluteJoint( g_world, &def ) );
	}

	// 6: spherical with cone, twist, motor and spring
	b3BodyId box7 = AddBox( b3_dynamicBody, (b3Vec3){ 6.0f, 4.4f, 0.0f }, 0.25f, 0.6f, 0.25f );
	b3Body_SetAngularVelocity( box7, (b3Vec3){ 2.0f, 1.0f, 3.0f } );
	{
		b3SphericalJointDef def = b3DefaultSphericalJointDef();
		def.base = MakeBase( def.base, g_ground, box7, (b3Vec3){ 6.0f, 5.0f, 0.0f } );
		def.base.localFrameA.q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
		def.base.localFrameB.q = def.base.localFrameA.q;
		def.enableConeLimit = true;
		def.coneAngle = 0.1f;
		def.enableTwistLimit = true;
		def.lowerTwistAngle = 0.2f;
		def.upperTwistAngle = 0.3f;
		def.enableMotor = true;
		def.maxMotorTorque = 20.0f;
		def.motorVelocity = (b3Vec3){ 0.0f, 3.0f, 0.0f };
		def.enableSpring = true;
		def.hertz = 1.0f;
		def.dampingRatio = 0.2f;
		def.targetRotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.3f );
		AddJoint( b3CreateSphericalJoint( g_world, &def ) );
	}

	// 7: rigid weld between two dynamic bodies
	b3BodyId box8 = AddBox( b3_dynamicBody, (b3Vec3){ 6.0f, 3.4f, 0.0f }, 0.3f, 0.3f, 0.3f );
	{
		b3WeldJointDef def = b3DefaultWeldJointDef();
		def.base = MakeBase( def.base, box7, box8, (b3Vec3){ 6.0f, 3.7f, 0.0f } );
		AddJoint( b3CreateWeldJoint( g_world, &def ) );
	}

	// 8: soft weld
	b3BodyId box9 = AddBox( b3_dynamicBody, (b3Vec3){ 10.5f, 5.0f, 0.0f }, 0.5f, 0.2f, 0.2f );
	{
		b3WeldJointDef def = b3DefaultWeldJointDef();
		def.base = MakeBase( def.base, g_ground, box9, (b3Vec3){ 10.0f, 5.0f, 0.0f } );
		def.linearHertz = 3.0f;
		def.linearDampingRatio = 0.7f;
		def.angularHertz = 2.0f;
		def.angularDampingRatio = 0.5f;
		AddJoint( b3CreateWeldJoint( g_world, &def ) );
	}

	// 9: wheel under a chassis, suspension along world y
	b3BodyId chassis = AddBox( b3_dynamicBody, (b3Vec3){ 14.0f, 2.6f, 0.0f }, 1.0f, 0.25f, 0.5f );
	b3BodyId wheel = AddBall( b3_dynamicBody, (b3Vec3){ 14.0f, 1.8f, 0.0f }, 0.5f );
	{
		b3WheelJointDef def = b3DefaultWheelJointDef();
		def.base = MakeBase( def.base, chassis, wheel, (b3Vec3){ 14.0f, 1.8f, 0.0f } );
		def.base.localFrameA.q = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.5f * B3_PI );
		def.base.localFrameB.q = def.base.localFrameA.q;
		def.enableSuspensionLimit = true;
		def.lowerSuspensionLimit = -0.3f;
		def.upperSuspensionLimit = 0.3f;
		def.enableSpinMotor = true;
		def.maxSpinTorque = 10.0f;
		def.spinSpeed = -3.0f;
		def.enableSteering = true;
		def.targetSteeringAngle = 0.8f;
		def.maxSteeringTorque = 50.0f;
		def.steeringHertz = 5.0f;
		def.enableSteeringLimit = true;
		def.lowerSteeringLimit = -0.2f;
		def.upperSteeringLimit = 0.2f;
		AddJoint( b3CreateWheelJoint( g_world, &def ) );
	}

	// 10: filter joint between overlapping balls
	b3BodyId ball12 = AddBall( b3_dynamicBody, (b3Vec3){ 18.0f, 1.5f, 0.0f }, 0.3f );
	b3BodyId ball13 = AddBall( b3_dynamicBody, (b3Vec3){ 18.1f, 1.9f, 0.0f }, 0.3f );
	{
		b3FilterJointDef def = b3DefaultFilterJointDef();
		def.base.bodyIdA = ball12;
		def.base.bodyIdB = ball13;
		AddJoint( b3CreateFilterJoint( g_world, &def ) );
	}

	// 11: motor joint driven by a kinematic body
	b3BodyId kin = AddBox( b3_kinematicBody, (b3Vec3){ -18.0f, 3.0f, 0.0f }, 0.3f, 0.3f, 0.3f );
	b3Body_SetLinearVelocity( kin, (b3Vec3){ 0.0f, 0.0f, 0.5f } );
	b3Body_SetAngularVelocity( kin, (b3Vec3){ 0.0f, 1.0f, 0.0f } );
	b3BodyId ball15 = AddBall( b3_dynamicBody, (b3Vec3){ -18.0f, 2.0f, 0.0f }, 0.25f );
	{
		b3MotorJointDef def = b3DefaultMotorJointDef();
		def.base = MakeBase( def.base, kin, ball15, (b3Vec3){ -18.0f, 2.5f, 0.0f } );
		def.maxVelocityForce = 10.0f;
		def.maxVelocityTorque = 5.0f;
		def.linearHertz = 4.0f;
		def.linearDampingRatio = 0.7f;
		def.maxSpringForce = 50.0f;
		AddJoint( b3CreateMotorJoint( g_world, &def ) );
	}

	// resting balls that fall asleep, plus a disabled one
	g_sleepA = AddBall( b3_dynamicBody, (b3Vec3){ -4.0f, 1.3f, 8.0f }, 0.3f );
	g_sleepB = AddBall( b3_dynamicBody, (b3Vec3){ 4.0f, 1.3f, 8.0f }, 0.3f );
	g_sleepC = AddBall( b3_dynamicBody, (b3Vec3){ 0.0f, 1.3f, 8.0f }, 0.3f );
	g_disabled = AddBall( b3_dynamicBody, (b3Vec3){ 8.0f, 3.0f, 8.0f }, 0.3f );
	b3Body_Disable( g_disabled );
}

static void PrintJointSim( b3JointSim* base )
{
	vtag( "sim" );
	vi32( base->type );
	vf32( base->invMassA );
	vf32( base->invMassB );
	vbool( base->fixedRotation );
	vsoft( base->constraintSoftness );
	vxf( base->localFrameA );
	vxf( base->localFrameB );
	vend();

	switch ( base->type )
	{
		case b3_distanceJoint:
		{
			b3DistanceJoint* j = &base->distanceJoint;
			vtag( "distance" );
			vf32( j->impulse );
			vf32( j->lowerImpulse );
			vf32( j->upperImpulse );
			vf32( j->motorImpulse );
			vf32( j->axialMass );
			vvec( j->anchorA );
			vvec( j->anchorB );
			vvec( j->deltaCenter );
			vsoft( j->distanceSoftness );
			vf32( j->length );
			vf32( j->minLength );
			vf32( j->maxLength );
			vend();
			break;
		}
		case b3_motorJoint:
		{
			b3MotorJoint* j = &base->motorJoint;
			vtag( "motor" );
			vvec( j->linearVelocityImpulse );
			vvec( j->angularVelocityImpulse );
			vvec( j->linearSpringImpulse );
			vvec( j->angularSpringImpulse );
			vsoft( j->linearSpring );
			vsoft( j->angularSpring );
			vxf( j->frameA );
			vxf( j->frameB );
			vvec( j->angularMass.cx );
			vvec( j->angularMass.cy );
			vvec( j->angularMass.cz );
			vend();
			break;
		}
		case b3_parallelJoint:
		{
			b3ParallelJoint* j = &base->parallelJoint;
			vtag( "parallel" );
			vf32( j->perpImpulse.x );
			vf32( j->perpImpulse.y );
			vvec( j->perpAxisX );
			vvec( j->perpAxisY );
			vquat( j->quatA );
			vquat( j->quatB );
			vsoft( j->softness );
			vend();
			break;
		}
		case b3_prismaticJoint:
		{
			b3PrismaticJoint* j = &base->prismaticJoint;
			vtag( "prismatic" );
			vf32( j->perpImpulse.x );
			vf32( j->perpImpulse.y );
			vvec( j->angularImpulse );
			vf32( j->springImpulse );
			vf32( j->motorImpulse );
			vf32( j->lowerImpulse );
			vf32( j->upperImpulse );
			vvec( j->jointAxis );
			vvec( j->perpAxisY );
			vvec( j->perpAxisZ );
			vvec( j->rotationMass.cx );
			vsoft( j->springSoftness );
			vend();
			break;
		}
		case b3_revoluteJoint:
		{
			b3RevoluteJoint* j = &base->revoluteJoint;
			vtag( "revolute" );
			vvec( j->linearImpulse );
			vf32( j->perpImpulse.x );
			vf32( j->perpImpulse.y );
			vf32( j->springImpulse );
			vf32( j->motorImpulse );
			vf32( j->lowerImpulse );
			vf32( j->upperImpulse );
			vvec( j->rotationAxisZ );
			vvec( j->perpAxisX );
			vvec( j->perpAxisY );
			vf32( j->axialMass );
			vsoft( j->springSoftness );
			vend();
			break;
		}
		case b3_sphericalJoint:
		{
			b3SphericalJoint* j = &base->sphericalJoint;
			vtag( "spherical" );
			vvec( j->linearImpulse );
			vvec( j->springImpulse );
			vvec( j->motorImpulse );
			vf32( j->lowerTwistImpulse );
			vf32( j->upperTwistImpulse );
			vf32( j->swingImpulse );
			vvec( j->swingAxis );
			vvec( j->twistJacobian );
			vf32( j->swingMass );
			vf32( j->twistMass );
			vvec( j->rotationMass.cy );
			vend();
			break;
		}
		case b3_weldJoint:
		{
			b3WeldJoint* j = &base->weldJoint;
			vtag( "weld" );
			vvec( j->linearImpulse );
			vvec( j->angularImpulse );
			vsoft( j->linearSpring );
			vsoft( j->angularSpring );
			vvec( j->angularMass.cz );
			vend();
			break;
		}
		case b3_wheelJoint:
		{
			b3WheelJoint* j = &base->wheelJoint;
			vtag( "wheel" );
			vf32( j->linearImpulse.x );
			vf32( j->linearImpulse.y );
			vf32( j->angularImpulse.x );
			vf32( j->angularImpulse.y );
			vf32( j->spinImpulse );
			vf32( j->suspensionSpringImpulse );
			vf32( j->lowerSuspensionImpulse );
			vf32( j->upperSuspensionImpulse );
			vf32( j->steeringSpringImpulse );
			vf32( j->lowerSteeringImpulse );
			vf32( j->upperSteeringImpulse );
			vf32( j->spinMass );
			vf32( j->suspensionMass );
			vf32( j->steeringMass );
			vsoft( j->suspensionSoftness );
			vsoft( j->steeringSoftness );
			vend();
			break;
		}
		default:
			break;
	}
}

static void PrintJoint( int i )
{
	b3JointId id = g_joints[i];
	b3JointType type = b3Joint_GetType( id );

	vtag( "joint" );
	vi32( i );
	vi32( type );
	vi32( id.index1 );
	vu32( id.generation );
	vbool( b3Joint_IsAwake( id ) );
	vi32( b3Joint_GetBodyA( id ).index1 );
	vi32( b3Joint_GetBodyB( id ).index1 );
	vbool( b3Joint_GetCollideConnected( id ) );
	vu64( (uint64_t)(uintptr_t)b3Joint_GetUserData( id ) );
	vvec( b3Joint_GetConstraintForce( id ) );
	vvec( b3Joint_GetConstraintTorque( id ) );
	vf32( b3Joint_GetLinearSeparation( id ) );
	if ( type != b3_wheelJoint )
	{
		vf32( b3Joint_GetAngularSeparation( id ) );
	}
	vf32( b3Joint_GetForceThreshold( id ) );
	vf32( b3Joint_GetTorqueThreshold( id ) );
	float tuning[2];
	b3Joint_GetConstraintTuning( id, tuning + 0, tuning + 1 );
	vf32( tuning[0] );
	vf32( tuning[1] );
	vend();

	b3JointSim* base = b3GetJointSimCheckType( id, type );
	b3World* world = b3GetWorld( id.world0 );
	float reaction[2];
	b3GetJointReaction( world, base, world->inv_h, reaction + 0, reaction + 1 );
	vtag( "reaction" );
	vf32( reaction[0] );
	vf32( reaction[1] );
	vend();

	vtag( "api" );
	switch ( type )
	{
		case b3_distanceJoint:
		{
			vf32( b3DistanceJoint_GetLength( id ) );
			vbool( b3DistanceJoint_IsLimitEnabled( id ) );
			vf32( b3DistanceJoint_GetMinLength( id ) );
			vf32( b3DistanceJoint_GetMaxLength( id ) );
			vf32( b3DistanceJoint_GetCurrentLength( id ) );
			vbool( b3DistanceJoint_IsSpringEnabled( id ) );
			float range[2];
			b3DistanceJoint_GetSpringForceRange( id, range + 0, range + 1 );
			vf32( range[0] );
			vf32( range[1] );
			vf32( b3DistanceJoint_GetSpringHertz( id ) );
			vf32( b3DistanceJoint_GetSpringDampingRatio( id ) );
			vbool( b3DistanceJoint_IsMotorEnabled( id ) );
			vf32( b3DistanceJoint_GetMotorSpeed( id ) );
			vf32( b3DistanceJoint_GetMotorForce( id ) );
			vf32( b3DistanceJoint_GetMaxMotorForce( id ) );
			break;
		}
		case b3_motorJoint:
			vvec( b3MotorJoint_GetLinearVelocity( id ) );
			vvec( b3MotorJoint_GetAngularVelocity( id ) );
			vf32( b3MotorJoint_GetMaxVelocityTorque( id ) );
			vf32( b3MotorJoint_GetMaxVelocityForce( id ) );
			vf32( b3MotorJoint_GetLinearHertz( id ) );
			vf32( b3MotorJoint_GetLinearDampingRatio( id ) );
			vf32( b3MotorJoint_GetAngularHertz( id ) );
			vf32( b3MotorJoint_GetAngularDampingRatio( id ) );
			vf32( b3MotorJoint_GetMaxSpringForce( id ) );
			vf32( b3MotorJoint_GetMaxSpringTorque( id ) );
			break;
		case b3_parallelJoint:
			vf32( b3ParallelJoint_GetSpringHertz( id ) );
			vf32( b3ParallelJoint_GetSpringDampingRatio( id ) );
			vf32( b3ParallelJoint_GetMaxTorque( id ) );
			break;
		case b3_prismaticJoint:
			vbool( b3PrismaticJoint_IsLimitEnabled( id ) );
			vf32( b3PrismaticJoint_GetLowerLimit( id ) );
			vf32( b3PrismaticJoint_GetUpperLimit( id ) );
			vf32( b3PrismaticJoint_GetTranslation( id ) );
			vbool( b3PrismaticJoint_IsSpringEnabled( id ) );
			vf32( b3PrismaticJoint_GetTargetTranslation( id ) );
			vf32( b3PrismaticJoint_GetSpringHertz( id ) );
			vf32( b3PrismaticJoint_GetSpringDampingRatio( id ) );
			vbool( b3PrismaticJoint_IsMotorEnabled( id ) );
			vf32( b3PrismaticJoint_GetMotorSpeed( id ) );
			vf32( b3PrismaticJoint_GetMaxMotorForce( id ) );
			vf32( b3PrismaticJoint_GetMotorForce( id ) );
			vf32( b3PrismaticJoint_GetSpeed( id ) );
			break;
		case b3_revoluteJoint:
			vbool( b3RevoluteJoint_IsLimitEnabled( id ) );
			vf32( b3RevoluteJoint_GetLowerLimit( id ) );
			vf32( b3RevoluteJoint_GetUpperLimit( id ) );
			vf32( b3RevoluteJoint_GetAngle( id ) );
			vbool( b3RevoluteJoint_IsSpringEnabled( id ) );
			vf32( b3RevoluteJoint_GetTargetAngle( id ) );
			vf32( b3RevoluteJoint_GetSpringHertz( id ) );
			vf32( b3RevoluteJoint_GetSpringDampingRatio( id ) );
			vbool( b3RevoluteJoint_IsMotorEnabled( id ) );
			vf32( b3RevoluteJoint_GetMotorSpeed( id ) );
			vf32( b3RevoluteJoint_GetMaxMotorTorque( id ) );
			vf32( b3RevoluteJoint_GetMotorTorque( id ) );
			break;
		case b3_sphericalJoint:
			vbool( b3SphericalJoint_IsConeLimitEnabled( id ) );
			vf32( b3SphericalJoint_GetConeLimit( id ) );
			vf32( b3SphericalJoint_GetConeAngle( id ) );
			vbool( b3SphericalJoint_IsTwistLimitEnabled( id ) );
			vf32( b3SphericalJoint_GetLowerTwistLimit( id ) );
			vf32( b3SphericalJoint_GetUpperTwistLimit( id ) );
			vf32( b3SphericalJoint_GetTwistAngle( id ) );
			vbool( b3SphericalJoint_IsSpringEnabled( id ) );
			vquat( b3SphericalJoint_GetTargetRotation( id ) );
			vf32( b3SphericalJoint_GetSpringHertz( id ) );
			vf32( b3SphericalJoint_GetSpringDampingRatio( id ) );
			vbool( b3SphericalJoint_IsMotorEnabled( id ) );
			vvec( b3SphericalJoint_GetMotorVelocity( id ) );
			vf32( b3SphericalJoint_GetMaxMotorTorque( id ) );
			vvec( b3SphericalJoint_GetMotorTorque( id ) );
			break;
		case b3_weldJoint:
			vf32( b3WeldJoint_GetLinearHertz( id ) );
			vf32( b3WeldJoint_GetLinearDampingRatio( id ) );
			vf32( b3WeldJoint_GetAngularHertz( id ) );
			vf32( b3WeldJoint_GetAngularDampingRatio( id ) );
			break;
		case b3_wheelJoint:
			vbool( b3WheelJoint_IsSuspensionEnabled( id ) );
			vf32( b3WheelJoint_GetSuspensionHertz( id ) );
			vf32( b3WheelJoint_GetSuspensionDampingRatio( id ) );
			vbool( b3WheelJoint_IsSuspensionLimitEnabled( id ) );
			vf32( b3WheelJoint_GetLowerSuspensionLimit( id ) );
			vf32( b3WheelJoint_GetUpperSuspensionLimit( id ) );
			vbool( b3WheelJoint_IsSpinMotorEnabled( id ) );
			vf32( b3WheelJoint_GetSpinMotorSpeed( id ) );
			vf32( b3WheelJoint_GetMaxSpinTorque( id ) );
			vbool( b3WheelJoint_IsSteeringEnabled( id ) );
			vf32( b3WheelJoint_GetSteeringHertz( id ) );
			vf32( b3WheelJoint_GetSteeringDampingRatio( id ) );
			vf32( b3WheelJoint_GetMaxSteeringTorque( id ) );
			vbool( b3WheelJoint_IsSteeringLimitEnabled( id ) );
			vf32( b3WheelJoint_GetLowerSteeringLimit( id ) );
			vf32( b3WheelJoint_GetUpperSteeringLimit( id ) );
			vf32( b3WheelJoint_GetTargetSteeringAngle( id ) );
			vf32( b3WheelJoint_GetSpinSpeed( id ) );
			vf32( b3WheelJoint_GetSpinTorque( id ) );
			vf32( b3WheelJoint_GetSteeringAngle( id ) );
			vf32( b3WheelJoint_GetSteeringTorque( id ) );
			break;
		default:
			break;
	}
	vend();

	PrintJointSim( base );
}

static void PrintBodies( void )
{
	for ( int i = 0; i < g_bodyCount; ++i )
	{
		b3BodyId id = g_bodies[i];
		vtag( "body" );
		vi32( i );
		vvec( b3Body_GetPosition( id ) );
		vquat( b3Body_GetRotation( id ) );
		vvec( b3Body_GetLinearVelocity( id ) );
		vvec( b3Body_GetAngularVelocity( id ) );
		vbool( b3Body_IsAwake( id ) );
		vend();
	}
}

static void PrintEvents( void )
{
	b3JointEvents events = b3World_GetJointEvents( g_world );
	vtag( "events" );
	vi32( events.count );
	for ( int i = 0; i < events.count; ++i )
	{
		vi32( events.jointEvents[i].jointId.index1 );
		vu32( events.jointEvents[i].jointId.generation );
		vu64( (uint64_t)(uintptr_t)events.jointEvents[i].userData );
	}
	vend();
}

static void Destroy( int i, bool wake )
{
	b3DestroyJoint( g_joints[i], wake );
	g_alive[i] = false;
}

static void Script( int step )
{
	if ( step == 40 )
	{
		b3DistanceJoint_SetLength( g_joints[0], 2.5f );
		b3DistanceJoint_SetLengthRange( g_joints[1], 0.5f, 2.5f );
		b3DistanceJoint_EnableMotor( g_joints[1], false );
		b3DistanceJoint_SetSpringForceRange( g_joints[1], -50.0f, 60.0f );
		b3MotorJoint_SetLinearVelocity( g_joints[2], (b3Vec3){ -0.5f, 0.2f, 0.0f } );
		b3MotorJoint_SetMaxSpringTorque( g_joints[2], -1.0f );
		b3ParallelJoint_SetMaxTorque( g_joints[3], 20.0f );
		b3PrismaticJoint_SetLimits( g_joints[4], 1.5f, -0.5f );
		b3PrismaticJoint_SetMotorSpeed( g_joints[4], -1.0f );
		b3PrismaticJoint_EnableSpring( g_joints[4], false );
		b3RevoluteJoint_SetLimits( g_joints[5], -0.2f, 0.3f );
		b3RevoluteJoint_SetMotorSpeed( g_joints[5], -2.0f );
		b3RevoluteJoint_EnableSpring( g_joints[5], false );
		b3Joint_SetConstraintTuning( g_joints[5], 30.0f, 1.0f );
		b3SphericalJoint_SetConeLimit( g_joints[6], 0.3f );
		b3SphericalJoint_SetTwistLimits( g_joints[6], 0.2f, -0.2f );
		b3SphericalJoint_SetMotorVelocity( g_joints[6], (b3Vec3){ 0.5f, 0.0f, 0.0f } );
		b3WeldJoint_SetAngularHertz( g_joints[8], 0.0f );
		b3WheelJoint_SetSpinMotorSpeed( g_joints[9], 2.0f );
		b3WheelJoint_SetTargetSteeringAngle( g_joints[9], -0.9f );
		b3WheelJoint_SetSuspensionLimits( g_joints[9], -0.1f, 0.2f );
		b3Joint_SetForceThreshold( g_joints[1], 3.0f );
		b3Joint_SetTorqueThreshold( g_joints[5], 0.25f );
	}

	if ( step == 50 )
	{
		b3Joint_SetCollideConnected( g_joints[10], true );
		b3Joint_SetLocalFrameA( g_joints[8], (b3Transform){ (b3Vec3){ 10.0f, 5.5f, 0.0f }, b3Quat_identity } );
		b3SphericalJoint_EnableConeLimit( g_joints[6], false );
		b3WheelJoint_EnableSteering( g_joints[9], false );
	}

	if ( step == 70 )
	{
		b3Joint_SetCollideConnected( g_joints[10], false );
		b3PrismaticJoint_EnableLimit( g_joints[4], false );
		b3RevoluteJoint_EnableMotor( g_joints[5], false );
		b3SphericalJoint_EnableMotor( g_joints[6], false );
		b3SphericalJoint_SetTargetRotation( g_joints[6], b3MakeQuatFromAxisAngle( b3Vec3_axisY, -0.4f ) );
	}

	if ( step == 80 )
	{
		Destroy( 7, true );
		b3Joint_SetUserData( g_joints[2], (void*)(uintptr_t)42 );
	}

	if ( step == 90 )
	{
		// both bodies sleep in their own sets, so the sets merge
		vtag( "asleep" );
		vbool( b3Body_IsAwake( g_sleepA ) );
		vbool( b3Body_IsAwake( g_sleepB ) );
		vbool( b3Body_IsAwake( g_sleepC ) );
		vend();

		b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
		def.base = MakeBase( def.base, g_sleepA, g_sleepB, (b3Vec3){ 0.0f, 1.3f, 8.0f } );
		def.base.collideConnected = true;
		AddJoint( b3CreateRevoluteJoint( g_world, &def ) );

		b3DistanceJointDef ddef = b3DefaultDistanceJointDef();
		ddef.base = MakeBase( ddef.base, g_ground, g_sleepC, (b3Vec3){ 0.0f, 3.0f, 8.0f } );
		ddef.base.localFrameB.p = b3Vec3_zero;
		ddef.length = 1.7f;
		AddJoint( b3CreateDistanceJoint( g_world, &ddef ) );

		b3WeldJointDef wdef = b3DefaultWeldJointDef();
		wdef.base = MakeBase( wdef.base, g_ground, g_disabled, (b3Vec3){ 8.0f, 3.0f, 8.0f } );
		AddJoint( b3CreateWeldJoint( g_world, &wdef ) );
	}

	if ( step == 100 )
	{
		b3Joint_WakeBodies( g_joints[12] );
		b3World_EnableWarmStarting( g_world, false );
		b3Body_Enable( g_disabled );
	}

	if ( step == 105 )
	{
		b3World_EnableWarmStarting( g_world, true );
	}

	if ( step == 110 )
	{
		Destroy( 10, false );
		Destroy( 12, true );
		b3MotorJoint_SetAngularVelocity( g_joints[11], (b3Vec3){ 1.0f, 0.0f, 0.0f } );
	}
}

int main( void )
{
	Setup();
	for ( int i = 0; i < g_jointCount; ++i )
	{
		PrintJoint( i );
	}

	for ( int step = 0; step < STEP_COUNT; ++step )
	{
		Script( step );
		b3World_Step( g_world, 1.0f / 60.0f, 4 );

		vtag( "step" );
		vi32( step );
		vend();
		PrintEvents();
		for ( int i = 0; i < g_jointCount; ++i )
		{
			if ( g_alive[i] )
			{
				PrintJoint( i );
			}
		}
		PrintBodies();
	}

	b3DestroyWorld( g_world );
	return 0;
}
