// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for the joint modules, mirrors verify/cref/joints.c. The world lives in module globals and
// the host steps it one export call at a time.

import { vtag, vi32, vu32, vu64, vbool, vf32, vend } from "./common";
import { B3_PI, b3Quat, b3Quat_identity, b3MakeQuatFromAxisAngle, b3Sub, b3Transform, b3Vec3, b3Vec3_axisX, b3Vec3_axisY, b3Vec3_axisZ, b3Vec3_zero } from "../../lib/math_functions";
import { b3Sphere } from "../../lib/collision";
import {
	b3BodyId,
	b3BodyType,
	b3JointDef,
	b3JointId,
	b3JointType,
	b3WorldId,
	b3DefaultBodyDef,
	b3DefaultShapeDef,
	b3DefaultWorldDef,
	b3_distanceJoint,
	b3_dynamicBody,
	b3_kinematicBody,
	b3_motorJoint,
	b3_parallelJoint,
	b3_prismaticJoint,
	b3_revoluteJoint,
	b3_sphericalJoint,
	b3_staticBody,
	b3_weldJoint,
	b3_wheelJoint,
} from "../../lib/types";
import { b3Softness } from "../../lib/solver";
import { b3CreateWorld, b3DestroyWorld, b3GetWorld, b3World_EnableWarmStarting, b3World_GetJointEvents, b3World_Step } from "../../lib/physics_world";
import {
	b3Body_Disable,
	b3Body_Enable,
	b3Body_GetAngularVelocity,
	b3Body_GetLinearVelocity,
	b3Body_GetPosition,
	b3Body_GetRotation,
	b3Body_IsAwake,
	b3Body_SetAngularVelocity,
	b3Body_SetLinearVelocity,
	b3CreateBody,
} from "../../lib/body";
import { b3CreateHullShape, b3CreateSphereShape } from "../../lib/shape";
import { b3MakeBoxHull } from "../../lib/hull";
import {
	b3CreateDistanceJoint,
	b3CreateFilterJoint,
	b3CreateMotorJoint,
	b3CreateParallelJoint,
	b3CreatePrismaticJoint,
	b3CreateRevoluteJoint,
	b3CreateSphericalJoint,
	b3CreateWeldJoint,
	b3CreateWheelJoint,
	b3DefaultDistanceJointDef,
	b3DefaultFilterJointDef,
	b3DefaultMotorJointDef,
	b3DefaultParallelJointDef,
	b3DefaultPrismaticJointDef,
	b3DefaultRevoluteJointDef,
	b3DefaultSphericalJointDef,
	b3DefaultWeldJointDef,
	b3DefaultWheelJointDef,
	b3DestroyJoint,
	b3GetJointReaction,
	b3GetJointSimCheckType,
	b3JointSim,
	b3Joint_GetAngularSeparation,
	b3Joint_GetBodyA,
	b3Joint_GetBodyB,
	b3Joint_GetCollideConnected,
	b3Joint_GetConstraintForce,
	b3Joint_GetConstraintTorque,
	b3Joint_GetConstraintTuning,
	b3Joint_GetForceThreshold,
	b3Joint_GetLinearSeparation,
	b3Joint_GetTorqueThreshold,
	b3Joint_GetType,
	b3Joint_GetUserData,
	b3Joint_IsAwake,
	b3Joint_SetCollideConnected,
	b3Joint_SetConstraintTuning,
	b3Joint_SetForceThreshold,
	b3Joint_SetLocalFrameA,
	b3Joint_SetTorqueThreshold,
	b3Joint_SetUserData,
	b3Joint_WakeBodies,
} from "../../lib/joint";
import * as dj from "../../lib/distance_joint";
import * as mj from "../../lib/motor_joint";
import * as pj from "../../lib/parallel_joint";
import * as xj from "../../lib/prismatic_joint";
import * as rj from "../../lib/revolute_joint";
import * as sj from "../../lib/spherical_joint";
import * as wj from "../../lib/weld_joint";
import * as hj from "../../lib/wheel_joint";

export { frameBoundary } from "./common";

const STEP_COUNT: i32 = 130;

let g_world: b3WorldId = b3WorldId();
let g_bodies = new StaticArray<b3BodyId>(32);
let g_bodyCount: i32 = 0;
let g_joints = new StaticArray<b3JointId>(32);
let g_alive = new StaticArray<bool>(32);
let g_jointCount: i32 = 0;

let g_ground: b3BodyId = b3BodyId();
let g_sleepA: b3BodyId = b3BodyId();
let g_sleepB: b3BodyId = b3BodyId();
let g_sleepC: b3BodyId = b3BodyId();
let g_disabled: b3BodyId = b3BodyId();

function vvec(v: b3Vec3): void {
	vf32(v.x);
	vf32(v.y);
	vf32(v.z);
}

function vquat(q: b3Quat): void {
	vvec(q.v);
	vf32(q.s);
}

function vsoft(s: b3Softness): void {
	vf32(s.biasRate);
	vf32(s.massScale);
	vf32(s.impulseScale);
}

function vxf(t: b3Transform): void {
	vvec(t.p);
	vquat(t.q);
}

function AddBody(type: b3BodyType, position: b3Vec3): b3BodyId {
	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = type;
	bodyDef.position = position;
	let id = b3CreateBody(g_world, bodyDef);
	g_bodies[g_bodyCount] = id;
	g_bodyCount += 1;
	return id;
}

function AddBall(type: b3BodyType, position: b3Vec3, radius: f32): b3BodyId {
	let id = AddBody(type, position);
	let shapeDef = b3DefaultShapeDef();
	let sphere = b3Sphere(b3Vec3_zero, radius);
	b3CreateSphereShape(id, shapeDef, sphere);
	return id;
}

function AddBox(type: b3BodyType, position: b3Vec3, hx: f32, hy: f32, hz: f32): b3BodyId {
	let id = AddBody(type, position);
	let shapeDef = b3DefaultShapeDef();
	let box = b3MakeBoxHull(hx, hy, hz);
	b3CreateHullShape(id, shapeDef, box.base);
	return id;
}

function AddJoint(id: b3JointId): void {
	g_joints[g_jointCount] = id;
	g_alive[g_jointCount] = true;
	g_jointCount += 1;
}

function MakeBase(base: b3JointDef, bodyIdA: b3BodyId, bodyIdB: b3BodyId, pivot: b3Vec3): b3JointDef {
	base.bodyIdA = bodyIdA;
	base.bodyIdB = bodyIdB;
	let pA = b3Body_GetPosition(bodyIdA);
	let pB = b3Body_GetPosition(bodyIdB);
	base.localFrameA.p = b3Sub(pivot, pA);
	base.localFrameB.p = b3Sub(pivot, pB);
	return base;
}

export function setup(): void {
	let worldDef = b3DefaultWorldDef();
	g_world = b3CreateWorld(worldDef);

	g_ground = AddBox(b3_staticBody, b3Vec3_zero, 20.0, 1.0, 20.0);

	// 0: rigid distance pendulum
	let ball1 = AddBall(b3_dynamicBody, b3Vec3(-14.0, 8.0, 0.0), 0.25);
	{
		let def = b3DefaultDistanceJointDef();
		def.base = MakeBase(def.base, g_ground, ball1, b3Vec3(-16.0, 8.0, 0.0));
		def.base.localFrameB.p = b3Vec3_zero;
		def.length = 2.0;
		AddJoint(b3CreateDistanceJoint(g_world, def));
	}

	// 1: soft distance with limit and motor
	let ball2 = AddBall(b3_dynamicBody, b3Vec3(-14.0, 6.0, 0.5), 0.25);
	{
		let def = b3DefaultDistanceJointDef();
		def.base.bodyIdA = ball1;
		def.base.bodyIdB = ball2;
		def.base.forceThreshold = 1.0;
		def.base.userData = 7;
		def.length = 2.0;
		def.enableSpring = true;
		def.hertz = 2.0;
		def.dampingRatio = 0.5;
		def.lowerSpringForce = -100.0;
		def.upperSpringForce = 100.0;
		def.enableLimit = true;
		def.minLength = 1.0;
		def.maxLength = 3.0;
		def.enableMotor = true;
		def.maxMotorForce = 5.0;
		def.motorSpeed = 0.5;
		AddJoint(b3CreateDistanceJoint(g_world, def));
	}

	// 2: motor joint
	let box3 = AddBox(b3_dynamicBody, b3Vec3(-10.0, 5.0, 0.0), 0.5, 0.5, 0.5);
	{
		let def = b3DefaultMotorJointDef();
		def.base = MakeBase(def.base, g_ground, box3, b3Vec3(-10.0, 5.0, 0.0));
		def.linearVelocity = b3Vec3(0.5, 0.0, 0.0);
		def.maxVelocityForce = 50.0;
		def.angularVelocity = b3Vec3(0.0, 1.0, 0.0);
		def.maxVelocityTorque = 20.0;
		def.linearHertz = 2.0;
		def.linearDampingRatio = 1.0;
		def.maxSpringForce = 20.0;
		def.angularHertz = 2.0;
		def.angularDampingRatio = 1.0;
		def.maxSpringTorque = 10.0;
		AddJoint(b3CreateMotorJoint(g_world, def));
	}

	// 3: parallel
	let box4 = AddBox(b3_dynamicBody, b3Vec3(-6.0, 5.0, 0.0), 0.5, 0.25, 0.5);
	b3Body_SetAngularVelocity(box4, b3Vec3(1.0, 0.5, 0.0));
	{
		let def = b3DefaultParallelJointDef();
		def.base = MakeBase(def.base, g_ground, box4, b3Vec3(-6.0, 5.0, 0.0));
		def.hertz = 5.0;
		def.dampingRatio = 1.0;
		def.maxTorque = 100.0;
		AddJoint(b3CreateParallelJoint(g_world, def));
	}

	// 4: prismatic on a tilted axis with limit, motor and spring
	let box5 = AddBox(b3_dynamicBody, b3Vec3(-2.0, 5.0, 0.0), 0.5, 0.5, 0.5);
	{
		let def = b3DefaultPrismaticJointDef();
		def.base = MakeBase(def.base, g_ground, box5, b3Vec3(-2.0, 5.0, 0.0));
		def.base.localFrameA.q = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.25 * B3_PI);
		def.base.localFrameB.q = def.base.localFrameA.q;
		def.enableLimit = true;
		def.lowerTranslation = -1.0;
		def.upperTranslation = 1.0;
		def.enableMotor = true;
		def.maxMotorForce = 20.0;
		def.motorSpeed = 1.0;
		def.enableSpring = true;
		def.hertz = 1.0;
		def.dampingRatio = 0.5;
		def.targetTranslation = 0.25;
		AddJoint(b3CreatePrismaticJoint(g_world, def));
	}

	// 5: revolute hinged at a box edge
	let box6 = AddBox(b3_dynamicBody, b3Vec3(2.5, 5.0, 0.0), 0.5, 0.25, 0.25);
	{
		let def = b3DefaultRevoluteJointDef();
		def.base = MakeBase(def.base, g_ground, box6, b3Vec3(2.0, 5.0, 0.0));
		def.base.torqueThreshold = 0.5;
		def.base.userData = 11;
		def.enableLimit = true;
		def.lowerAngle = -0.5;
		def.upperAngle = 0.8;
		def.enableMotor = true;
		def.maxMotorTorque = 5.0;
		def.motorSpeed = 2.0;
		def.enableSpring = true;
		def.hertz = 1.0;
		def.dampingRatio = 0.3;
		def.targetAngle = 0.2;
		AddJoint(b3CreateRevoluteJoint(g_world, def));
	}

	// 6: spherical with cone, twist, motor and spring
	let box7 = AddBox(b3_dynamicBody, b3Vec3(6.0, 4.4, 0.0), 0.25, 0.6, 0.25);
	b3Body_SetAngularVelocity(box7, b3Vec3(2.0, 1.0, 3.0));
	{
		let def = b3DefaultSphericalJointDef();
		def.base = MakeBase(def.base, g_ground, box7, b3Vec3(6.0, 5.0, 0.0));
		def.base.localFrameA.q = b3MakeQuatFromAxisAngle(b3Vec3_axisX, 0.5 * B3_PI);
		def.base.localFrameB.q = def.base.localFrameA.q;
		def.enableConeLimit = true;
		def.coneAngle = 0.1;
		def.enableTwistLimit = true;
		def.lowerTwistAngle = 0.2;
		def.upperTwistAngle = 0.3;
		def.enableMotor = true;
		def.maxMotorTorque = 20.0;
		def.motorVelocity = b3Vec3(0.0, 3.0, 0.0);
		def.enableSpring = true;
		def.hertz = 1.0;
		def.dampingRatio = 0.2;
		def.targetRotation = b3MakeQuatFromAxisAngle(b3Vec3_axisX, 0.3);
		AddJoint(b3CreateSphericalJoint(g_world, def));
	}

	// 7: rigid weld between two dynamic bodies
	let box8 = AddBox(b3_dynamicBody, b3Vec3(6.0, 3.4, 0.0), 0.3, 0.3, 0.3);
	{
		let def = b3DefaultWeldJointDef();
		def.base = MakeBase(def.base, box7, box8, b3Vec3(6.0, 3.7, 0.0));
		AddJoint(b3CreateWeldJoint(g_world, def));
	}

	// 8: soft weld
	let box9 = AddBox(b3_dynamicBody, b3Vec3(10.5, 5.0, 0.0), 0.5, 0.2, 0.2);
	{
		let def = b3DefaultWeldJointDef();
		def.base = MakeBase(def.base, g_ground, box9, b3Vec3(10.0, 5.0, 0.0));
		def.linearHertz = 3.0;
		def.linearDampingRatio = 0.7;
		def.angularHertz = 2.0;
		def.angularDampingRatio = 0.5;
		AddJoint(b3CreateWeldJoint(g_world, def));
	}

	// 9: wheel under a chassis, suspension along world y
	let chassis = AddBox(b3_dynamicBody, b3Vec3(14.0, 2.6, 0.0), 1.0, 0.25, 0.5);
	let wheel = AddBall(b3_dynamicBody, b3Vec3(14.0, 1.8, 0.0), 0.5);
	{
		let def = b3DefaultWheelJointDef();
		def.base = MakeBase(def.base, chassis, wheel, b3Vec3(14.0, 1.8, 0.0));
		def.base.localFrameA.q = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.5 * B3_PI);
		def.base.localFrameB.q = def.base.localFrameA.q;
		def.enableSuspensionLimit = true;
		def.lowerSuspensionLimit = -0.3;
		def.upperSuspensionLimit = 0.3;
		def.enableSpinMotor = true;
		def.maxSpinTorque = 10.0;
		def.spinSpeed = -3.0;
		def.enableSteering = true;
		def.targetSteeringAngle = 0.8;
		def.maxSteeringTorque = 50.0;
		def.steeringHertz = 5.0;
		def.enableSteeringLimit = true;
		def.lowerSteeringLimit = -0.2;
		def.upperSteeringLimit = 0.2;
		AddJoint(b3CreateWheelJoint(g_world, def));
	}

	// 10: filter joint between overlapping balls
	let ball12 = AddBall(b3_dynamicBody, b3Vec3(18.0, 1.5, 0.0), 0.3);
	let ball13 = AddBall(b3_dynamicBody, b3Vec3(18.1, 1.9, 0.0), 0.3);
	{
		let def = b3DefaultFilterJointDef();
		def.base.bodyIdA = ball12;
		def.base.bodyIdB = ball13;
		AddJoint(b3CreateFilterJoint(g_world, def));
	}

	// 11: motor joint driven by a kinematic body
	let kin = AddBox(b3_kinematicBody, b3Vec3(-18.0, 3.0, 0.0), 0.3, 0.3, 0.3);
	b3Body_SetLinearVelocity(kin, b3Vec3(0.0, 0.0, 0.5));
	b3Body_SetAngularVelocity(kin, b3Vec3(0.0, 1.0, 0.0));
	let ball15 = AddBall(b3_dynamicBody, b3Vec3(-18.0, 2.0, 0.0), 0.25);
	{
		let def = b3DefaultMotorJointDef();
		def.base = MakeBase(def.base, kin, ball15, b3Vec3(-18.0, 2.5, 0.0));
		def.maxVelocityForce = 10.0;
		def.maxVelocityTorque = 5.0;
		def.linearHertz = 4.0;
		def.linearDampingRatio = 0.7;
		def.maxSpringForce = 50.0;
		AddJoint(b3CreateMotorJoint(g_world, def));
	}

	// resting balls that fall asleep, plus a disabled one
	g_sleepA = AddBall(b3_dynamicBody, b3Vec3(-4.0, 1.3, 8.0), 0.3);
	g_sleepB = AddBall(b3_dynamicBody, b3Vec3(4.0, 1.3, 8.0), 0.3);
	g_sleepC = AddBall(b3_dynamicBody, b3Vec3(0.0, 1.3, 8.0), 0.3);
	g_disabled = AddBall(b3_dynamicBody, b3Vec3(8.0, 3.0, 8.0), 0.3);
	b3Body_Disable(g_disabled);

	for (let i = 0; i < g_jointCount; ++i) {
		PrintJoint(i);
	}
}

function PrintJointSim(base: b3JointSim): void {
	vtag("sim");
	vi32(base.type);
	vf32(base.invMassA);
	vf32(base.invMassB);
	vbool(base.fixedRotation);
	vsoft(base.constraintSoftness);
	vxf(base.localFrameA);
	vxf(base.localFrameB);
	vend();

	switch (base.type) {
		case b3_distanceJoint: {
			ref j = base.distanceJoint;
			vtag("distance");
			vf32(j.impulse);
			vf32(j.lowerImpulse);
			vf32(j.upperImpulse);
			vf32(j.motorImpulse);
			vf32(j.axialMass);
			vvec(j.anchorA);
			vvec(j.anchorB);
			vvec(j.deltaCenter);
			vsoft(j.distanceSoftness);
			vf32(j.length);
			vf32(j.minLength);
			vf32(j.maxLength);
			vend();
			break;
		}
		case b3_motorJoint: {
			ref j = base.motorJoint;
			vtag("motor");
			vvec(j.linearVelocityImpulse);
			vvec(j.angularVelocityImpulse);
			vvec(j.linearSpringImpulse);
			vvec(j.angularSpringImpulse);
			vsoft(j.linearSpring);
			vsoft(j.angularSpring);
			vxf(j.frameA);
			vxf(j.frameB);
			vvec(j.angularMass.cx);
			vvec(j.angularMass.cy);
			vvec(j.angularMass.cz);
			vend();
			break;
		}
		case b3_parallelJoint: {
			ref j = base.parallelJoint;
			vtag("parallel");
			vf32(j.perpImpulse.x);
			vf32(j.perpImpulse.y);
			vvec(j.perpAxisX);
			vvec(j.perpAxisY);
			vquat(j.quatA);
			vquat(j.quatB);
			vsoft(j.softness);
			vend();
			break;
		}
		case b3_prismaticJoint: {
			ref j = base.prismaticJoint;
			vtag("prismatic");
			vf32(j.perpImpulse.x);
			vf32(j.perpImpulse.y);
			vvec(j.angularImpulse);
			vf32(j.springImpulse);
			vf32(j.motorImpulse);
			vf32(j.lowerImpulse);
			vf32(j.upperImpulse);
			vvec(j.jointAxis);
			vvec(j.perpAxisY);
			vvec(j.perpAxisZ);
			vvec(j.rotationMass.cx);
			vsoft(j.springSoftness);
			vend();
			break;
		}
		case b3_revoluteJoint: {
			ref j = base.revoluteJoint;
			vtag("revolute");
			vvec(j.linearImpulse);
			vf32(j.perpImpulse.x);
			vf32(j.perpImpulse.y);
			vf32(j.springImpulse);
			vf32(j.motorImpulse);
			vf32(j.lowerImpulse);
			vf32(j.upperImpulse);
			vvec(j.rotationAxisZ);
			vvec(j.perpAxisX);
			vvec(j.perpAxisY);
			vf32(j.axialMass);
			vsoft(j.springSoftness);
			vend();
			break;
		}
		case b3_sphericalJoint: {
			ref j = base.sphericalJoint;
			vtag("spherical");
			vvec(j.linearImpulse);
			vvec(j.springImpulse);
			vvec(j.motorImpulse);
			vf32(j.lowerTwistImpulse);
			vf32(j.upperTwistImpulse);
			vf32(j.swingImpulse);
			vvec(j.swingAxis);
			vvec(j.twistJacobian);
			vf32(j.swingMass);
			vf32(j.twistMass);
			vvec(j.rotationMass.cy);
			vend();
			break;
		}
		case b3_weldJoint: {
			ref j = base.weldJoint;
			vtag("weld");
			vvec(j.linearImpulse);
			vvec(j.angularImpulse);
			vsoft(j.linearSpring);
			vsoft(j.angularSpring);
			vvec(j.angularMass.cz);
			vend();
			break;
		}
		case b3_wheelJoint: {
			ref j = base.wheelJoint;
			vtag("wheel");
			vf32(j.linearImpulse.x);
			vf32(j.linearImpulse.y);
			vf32(j.angularImpulse.x);
			vf32(j.angularImpulse.y);
			vf32(j.spinImpulse);
			vf32(j.suspensionSpringImpulse);
			vf32(j.lowerSuspensionImpulse);
			vf32(j.upperSuspensionImpulse);
			vf32(j.steeringSpringImpulse);
			vf32(j.lowerSteeringImpulse);
			vf32(j.upperSteeringImpulse);
			vf32(j.spinMass);
			vf32(j.suspensionMass);
			vf32(j.steeringMass);
			vsoft(j.suspensionSoftness);
			vsoft(j.steeringSoftness);
			vend();
			break;
		}
		default:
			break;
	}
}

function PrintJoint(i: i32): void {
	let id = g_joints[i];
	let type: b3JointType = b3Joint_GetType(id);

	vtag("joint");
	vi32(i);
	vi32(type);
	vi32(id.index1);
	vu32(id.generation);
	vbool(b3Joint_IsAwake(id));
	vi32(b3Joint_GetBodyA(id).index1);
	vi32(b3Joint_GetBodyB(id).index1);
	vbool(b3Joint_GetCollideConnected(id));
	vu64(b3Joint_GetUserData(id));
	vvec(b3Joint_GetConstraintForce(id));
	vvec(b3Joint_GetConstraintTorque(id));
	vf32(b3Joint_GetLinearSeparation(id));
	if (type != b3_wheelJoint) {
		vf32(b3Joint_GetAngularSeparation(id));
	}
	vf32(b3Joint_GetForceThreshold(id));
	vf32(b3Joint_GetTorqueThreshold(id));
	let tuning: FixedArray<f32, 2>;
	b3Joint_GetConstraintTuning(id, tuning[0], tuning[1]);
	vf32(tuning[0]);
	vf32(tuning[1]);
	vend();

	let base = b3GetJointSimCheckType(id, type);
	let world = b3GetWorld(id.world0);
	let reaction: FixedArray<f32, 2>;
	b3GetJointReaction(world, base, world.inv_h, reaction[0], reaction[1]);
	vtag("reaction");
	vf32(reaction[0]);
	vf32(reaction[1]);
	vend();

	vtag("api");
	switch (type) {
		case b3_distanceJoint: {
			vf32(dj.b3DistanceJoint_GetLength(id));
			vbool(dj.b3DistanceJoint_IsLimitEnabled(id));
			vf32(dj.b3DistanceJoint_GetMinLength(id));
			vf32(dj.b3DistanceJoint_GetMaxLength(id));
			vf32(dj.b3DistanceJoint_GetCurrentLength(id));
			vbool(dj.b3DistanceJoint_IsSpringEnabled(id));
			let range: FixedArray<f32, 2>;
			dj.b3DistanceJoint_GetSpringForceRange(id, range[0], range[1]);
			vf32(range[0]);
			vf32(range[1]);
			vf32(dj.b3DistanceJoint_GetSpringHertz(id));
			vf32(dj.b3DistanceJoint_GetSpringDampingRatio(id));
			vbool(dj.b3DistanceJoint_IsMotorEnabled(id));
			vf32(dj.b3DistanceJoint_GetMotorSpeed(id));
			vf32(dj.b3DistanceJoint_GetMotorForce(id));
			vf32(dj.b3DistanceJoint_GetMaxMotorForce(id));
			break;
		}
		case b3_motorJoint:
			vvec(mj.b3MotorJoint_GetLinearVelocity(id));
			vvec(mj.b3MotorJoint_GetAngularVelocity(id));
			vf32(mj.b3MotorJoint_GetMaxVelocityTorque(id));
			vf32(mj.b3MotorJoint_GetMaxVelocityForce(id));
			vf32(mj.b3MotorJoint_GetLinearHertz(id));
			vf32(mj.b3MotorJoint_GetLinearDampingRatio(id));
			vf32(mj.b3MotorJoint_GetAngularHertz(id));
			vf32(mj.b3MotorJoint_GetAngularDampingRatio(id));
			vf32(mj.b3MotorJoint_GetMaxSpringForce(id));
			vf32(mj.b3MotorJoint_GetMaxSpringTorque(id));
			break;
		case b3_parallelJoint:
			vf32(pj.b3ParallelJoint_GetSpringHertz(id));
			vf32(pj.b3ParallelJoint_GetSpringDampingRatio(id));
			vf32(pj.b3ParallelJoint_GetMaxTorque(id));
			break;
		case b3_prismaticJoint:
			vbool(xj.b3PrismaticJoint_IsLimitEnabled(id));
			vf32(xj.b3PrismaticJoint_GetLowerLimit(id));
			vf32(xj.b3PrismaticJoint_GetUpperLimit(id));
			vf32(xj.b3PrismaticJoint_GetTranslation(id));
			vbool(xj.b3PrismaticJoint_IsSpringEnabled(id));
			vf32(xj.b3PrismaticJoint_GetTargetTranslation(id));
			vf32(xj.b3PrismaticJoint_GetSpringHertz(id));
			vf32(xj.b3PrismaticJoint_GetSpringDampingRatio(id));
			vbool(xj.b3PrismaticJoint_IsMotorEnabled(id));
			vf32(xj.b3PrismaticJoint_GetMotorSpeed(id));
			vf32(xj.b3PrismaticJoint_GetMaxMotorForce(id));
			vf32(xj.b3PrismaticJoint_GetMotorForce(id));
			vf32(xj.b3PrismaticJoint_GetSpeed(id));
			break;
		case b3_revoluteJoint:
			vbool(rj.b3RevoluteJoint_IsLimitEnabled(id));
			vf32(rj.b3RevoluteJoint_GetLowerLimit(id));
			vf32(rj.b3RevoluteJoint_GetUpperLimit(id));
			vf32(rj.b3RevoluteJoint_GetAngle(id));
			vbool(rj.b3RevoluteJoint_IsSpringEnabled(id));
			vf32(rj.b3RevoluteJoint_GetTargetAngle(id));
			vf32(rj.b3RevoluteJoint_GetSpringHertz(id));
			vf32(rj.b3RevoluteJoint_GetSpringDampingRatio(id));
			vbool(rj.b3RevoluteJoint_IsMotorEnabled(id));
			vf32(rj.b3RevoluteJoint_GetMotorSpeed(id));
			vf32(rj.b3RevoluteJoint_GetMaxMotorTorque(id));
			vf32(rj.b3RevoluteJoint_GetMotorTorque(id));
			break;
		case b3_sphericalJoint:
			vbool(sj.b3SphericalJoint_IsConeLimitEnabled(id));
			vf32(sj.b3SphericalJoint_GetConeLimit(id));
			vf32(sj.b3SphericalJoint_GetConeAngle(id));
			vbool(sj.b3SphericalJoint_IsTwistLimitEnabled(id));
			vf32(sj.b3SphericalJoint_GetLowerTwistLimit(id));
			vf32(sj.b3SphericalJoint_GetUpperTwistLimit(id));
			vf32(sj.b3SphericalJoint_GetTwistAngle(id));
			vbool(sj.b3SphericalJoint_IsSpringEnabled(id));
			vquat(sj.b3SphericalJoint_GetTargetRotation(id));
			vf32(sj.b3SphericalJoint_GetSpringHertz(id));
			vf32(sj.b3SphericalJoint_GetSpringDampingRatio(id));
			vbool(sj.b3SphericalJoint_IsMotorEnabled(id));
			vvec(sj.b3SphericalJoint_GetMotorVelocity(id));
			vf32(sj.b3SphericalJoint_GetMaxMotorTorque(id));
			vvec(sj.b3SphericalJoint_GetMotorTorque(id));
			break;
		case b3_weldJoint:
			vf32(wj.b3WeldJoint_GetLinearHertz(id));
			vf32(wj.b3WeldJoint_GetLinearDampingRatio(id));
			vf32(wj.b3WeldJoint_GetAngularHertz(id));
			vf32(wj.b3WeldJoint_GetAngularDampingRatio(id));
			break;
		case b3_wheelJoint:
			vbool(hj.b3WheelJoint_IsSuspensionEnabled(id));
			vf32(hj.b3WheelJoint_GetSuspensionHertz(id));
			vf32(hj.b3WheelJoint_GetSuspensionDampingRatio(id));
			vbool(hj.b3WheelJoint_IsSuspensionLimitEnabled(id));
			vf32(hj.b3WheelJoint_GetLowerSuspensionLimit(id));
			vf32(hj.b3WheelJoint_GetUpperSuspensionLimit(id));
			vbool(hj.b3WheelJoint_IsSpinMotorEnabled(id));
			vf32(hj.b3WheelJoint_GetSpinMotorSpeed(id));
			vf32(hj.b3WheelJoint_GetMaxSpinTorque(id));
			vbool(hj.b3WheelJoint_IsSteeringEnabled(id));
			vf32(hj.b3WheelJoint_GetSteeringHertz(id));
			vf32(hj.b3WheelJoint_GetSteeringDampingRatio(id));
			vf32(hj.b3WheelJoint_GetMaxSteeringTorque(id));
			vbool(hj.b3WheelJoint_IsSteeringLimitEnabled(id));
			vf32(hj.b3WheelJoint_GetLowerSteeringLimit(id));
			vf32(hj.b3WheelJoint_GetUpperSteeringLimit(id));
			vf32(hj.b3WheelJoint_GetTargetSteeringAngle(id));
			vf32(hj.b3WheelJoint_GetSpinSpeed(id));
			vf32(hj.b3WheelJoint_GetSpinTorque(id));
			vf32(hj.b3WheelJoint_GetSteeringAngle(id));
			vf32(hj.b3WheelJoint_GetSteeringTorque(id));
			break;
		default:
			break;
	}
	vend();

	PrintJointSim(base);
}

function PrintBodies(): void {
	for (let i = 0; i < g_bodyCount; ++i) {
		let id = g_bodies[i];
		vtag("body");
		vi32(i);
		vvec(b3Body_GetPosition(id));
		vquat(b3Body_GetRotation(id));
		vvec(b3Body_GetLinearVelocity(id));
		vvec(b3Body_GetAngularVelocity(id));
		vbool(b3Body_IsAwake(id));
		vend();
	}
}

function PrintEvents(): void {
	let events = b3World_GetJointEvents(g_world);
	vtag("events");
	vi32(events.count);
	for (let i = 0; i < events.count; ++i) {
		vi32(events.jointEvents[i].jointId.index1);
		vu32(events.jointEvents[i].jointId.generation);
		vu64(events.jointEvents[i].userData);
	}
	vend();
}

function Destroy(i: i32, wake: bool): void {
	b3DestroyJoint(g_joints[i], wake);
	g_alive[i] = false;
}

function Script(step: i32): void {
	if (step == 40) {
		dj.b3DistanceJoint_SetLength(g_joints[0], 2.5);
		dj.b3DistanceJoint_SetLengthRange(g_joints[1], 0.5, 2.5);
		dj.b3DistanceJoint_EnableMotor(g_joints[1], false);
		dj.b3DistanceJoint_SetSpringForceRange(g_joints[1], -50.0, 60.0);
		mj.b3MotorJoint_SetLinearVelocity(g_joints[2], b3Vec3(-0.5, 0.2, 0.0));
		mj.b3MotorJoint_SetMaxSpringTorque(g_joints[2], -1.0);
		pj.b3ParallelJoint_SetMaxTorque(g_joints[3], 20.0);
		xj.b3PrismaticJoint_SetLimits(g_joints[4], 1.5, -0.5);
		xj.b3PrismaticJoint_SetMotorSpeed(g_joints[4], -1.0);
		xj.b3PrismaticJoint_EnableSpring(g_joints[4], false);
		rj.b3RevoluteJoint_SetLimits(g_joints[5], -0.2, 0.3);
		rj.b3RevoluteJoint_SetMotorSpeed(g_joints[5], -2.0);
		rj.b3RevoluteJoint_EnableSpring(g_joints[5], false);
		b3Joint_SetConstraintTuning(g_joints[5], 30.0, 1.0);
		sj.b3SphericalJoint_SetConeLimit(g_joints[6], 0.3);
		sj.b3SphericalJoint_SetTwistLimits(g_joints[6], 0.2, -0.2);
		sj.b3SphericalJoint_SetMotorVelocity(g_joints[6], b3Vec3(0.5, 0.0, 0.0));
		wj.b3WeldJoint_SetAngularHertz(g_joints[8], 0.0);
		hj.b3WheelJoint_SetSpinMotorSpeed(g_joints[9], 2.0);
		hj.b3WheelJoint_SetTargetSteeringAngle(g_joints[9], -0.9);
		hj.b3WheelJoint_SetSuspensionLimits(g_joints[9], -0.1, 0.2);
		b3Joint_SetForceThreshold(g_joints[1], 3.0);
		b3Joint_SetTorqueThreshold(g_joints[5], 0.25);
	}

	if (step == 50) {
		b3Joint_SetCollideConnected(g_joints[10], true);
		b3Joint_SetLocalFrameA(g_joints[8], b3Transform(b3Vec3(10.0, 5.5, 0.0), b3Quat_identity));
		sj.b3SphericalJoint_EnableConeLimit(g_joints[6], false);
		hj.b3WheelJoint_EnableSteering(g_joints[9], false);
	}

	if (step == 70) {
		b3Joint_SetCollideConnected(g_joints[10], false);
		xj.b3PrismaticJoint_EnableLimit(g_joints[4], false);
		rj.b3RevoluteJoint_EnableMotor(g_joints[5], false);
		sj.b3SphericalJoint_EnableMotor(g_joints[6], false);
		sj.b3SphericalJoint_SetTargetRotation(g_joints[6], b3MakeQuatFromAxisAngle(b3Vec3_axisY, -0.4));
	}

	if (step == 80) {
		Destroy(7, true);
		b3Joint_SetUserData(g_joints[2], 42);
	}

	if (step == 90) {
		// both bodies sleep in their own sets, so the sets merge
		vtag("asleep");
		vbool(b3Body_IsAwake(g_sleepA));
		vbool(b3Body_IsAwake(g_sleepB));
		vbool(b3Body_IsAwake(g_sleepC));
		vend();

		let def = b3DefaultRevoluteJointDef();
		def.base = MakeBase(def.base, g_sleepA, g_sleepB, b3Vec3(0.0, 1.3, 8.0));
		def.base.collideConnected = true;
		AddJoint(b3CreateRevoluteJoint(g_world, def));

		let ddef = b3DefaultDistanceJointDef();
		ddef.base = MakeBase(ddef.base, g_ground, g_sleepC, b3Vec3(0.0, 3.0, 8.0));
		ddef.base.localFrameB.p = b3Vec3_zero;
		ddef.length = 1.7;
		AddJoint(b3CreateDistanceJoint(g_world, ddef));

		let wdef = b3DefaultWeldJointDef();
		wdef.base = MakeBase(wdef.base, g_ground, g_disabled, b3Vec3(8.0, 3.0, 8.0));
		AddJoint(b3CreateWeldJoint(g_world, wdef));
	}

	if (step == 100) {
		b3Joint_WakeBodies(g_joints[12]);
		b3World_EnableWarmStarting(g_world, false);
		b3Body_Enable(g_disabled);
	}

	if (step == 105) {
		b3World_EnableWarmStarting(g_world, true);
	}

	if (step == 110) {
		Destroy(10, false);
		Destroy(12, true);
		mj.b3MotorJoint_SetAngularVelocity(g_joints[11], b3Vec3(1.0, 0.0, 0.0));
	}
}

export function stepCount(): i32 {
	return STEP_COUNT;
}

export function step(stepIndex: i32): void {
	Script(stepIndex);
	// a constant 1.0 / 60.0 folds in f64
	let timeStep: f32 = 1.0;
	timeStep = timeStep / 60.0;
	b3World_Step(g_world, timeStep, 4);

	vtag("step");
	vi32(stepIndex);
	vend();
	PrintEvents();
	for (let i = 0; i < g_jointCount; ++i) {
		if (g_alive[i]) {
			PrintJoint(i);
		}
	}
	PrintBodies();
}

export function teardown(): void {
	b3DestroyWorld(g_world);
}
