// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/joint.c

import { B3_ASSERT, B3_NULL_INDEX, B3_SECRET_COOKIE, FLT_MIN } from "./core";
import { B3_GRAPH_COLOR_COUNT, B3_LINEAR_SLOP } from "./constants";
import {
	B3_PI,
	b3AbsFloat,
	b3Add,
	b3ClampFloat,
	b3Cross,
	b3Dot,
	b3GetQuatAngle,
	b3GetSwingAngle,
	b3GetTwistAngle,
	b3InvMulQuat,
	b3IsValidFloat,
	b3IsValidQuat,
	b3IsValidTransform,
	b3Length,
	b3Matrix3,
	b3MaxFloat,
	b3MaxInt,
	b3MinFloat,
	b3MulAdd,
	b3MulQuat,
	b3MulSV,
	b3Normalize,
	b3Perp,
	b3Quat,
	b3RotateVector,
	b3Sub,
	b3SubPos,
	b3Transform,
	b3TransformWorldPoint,
	b3Vec2,
	b3Vec3,
	b3Vec3_axisX,
	b3Vec3_axisZ,
	b3Vec3_zero,
} from "./math_functions";
import {
	b3BodyId,
	b3DistanceJointDef,
	b3FilterJointDef,
	b3JointDef,
	b3JointId,
	b3JointType,
	b3MotorJointDef,
	b3ParallelJointDef,
	b3PrismaticJointDef,
	b3RevoluteJointDef,
	b3SphericalJointDef,
	b3WeldJointDef,
	b3WheelJointDef,
	b3WorldId,
	b3_distanceJoint,
	b3_dynamicBody,
	b3_filterJoint,
	b3_motorJoint,
	b3_parallelJoint,
	b3_prismaticJoint,
	b3_revoluteJoint,
	b3_sphericalJoint,
	b3_weldJoint,
	b3_wheelJoint,
} from "./types";
import { b3AllocId, b3FreeId } from "./id_pool";
import { b3Array_RemoveSwap } from "./container";
import { b3MakeSoft, b3Softness, b3StepContext } from "./solver";
import {
	b3GetUnlockedWorld,
	b3GetUnlockedWorldFromId,
	b3GetWorld,
	b3World,
	b3_awakeSet,
	b3_disabledSet,
	b3_firstSleepingSet,
	b3_staticSet,
} from "./physics_world";
import { b3GetBodyFullId, b3GetBodyTransform, b3MakeBodyId, b3WakeBody } from "./body";
import { b3DestroyContact } from "./contact";
import { b3BroadPhase_MarkProxyMovedSerial } from "./broad_phase";
import { b3MergeSolverSets, b3WakeSolverSet } from "./solver_set";
import { b3LinkJoint, b3UnlinkJoint } from "./island";
import { B3_OVERFLOW_INDEX, b3CreateJointInGraph, b3RemoveJointFromGraph } from "./constraint_graph";
import {
	b3GetDistanceJointForce,
	b3PrepareDistanceJoint,
	b3SolveDistanceJoint,
	b3WarmStartDistanceJoint,
} from "./distance_joint";
import {
	b3GetMotorJointForce,
	b3GetMotorJointTorque,
	b3PrepareMotorJoint,
	b3SolveMotorJoint,
	b3WarmStartMotorJoint,
} from "./motor_joint";
import {
	b3GetParallelJointTorque,
	b3PrepareParallelJoint,
	b3SolveParallelJoint,
	b3WarmStartParallelJoint,
} from "./parallel_joint";
import {
	b3GetPrismaticJointForce,
	b3GetPrismaticJointTorque,
	b3PreparePrismaticJoint,
	b3SolvePrismaticJoint,
	b3WarmStartPrismaticJoint,
} from "./prismatic_joint";
import {
	b3GetRevoluteJointForce,
	b3GetRevoluteJointTorque,
	b3PrepareRevoluteJoint,
	b3SolveRevoluteJoint,
	b3WarmStartRevoluteJoint,
} from "./revolute_joint";
import {
	b3GetSphericalJointForce,
	b3GetSphericalJointTorque,
	b3PrepareSphericalJoint,
	b3SolveSphericalJoint,
	b3WarmStartSphericalJoint,
} from "./spherical_joint";
import {
	b3GetWeldJointForce,
	b3GetWeldJointTorque,
	b3PrepareWeldJoint,
	b3SolveWeldJoint,
	b3WarmStartWeldJoint,
} from "./weld_joint";
import {
	b3GetWheelJointForce,
	b3GetWheelJointTorque,
	b3PrepareWheelJoint,
	b3SolveWheelJoint,
	b3WarmStartWheelJoint,
} from "./wheel_joint";

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
export struct b3JointEdge {
	bodyId: i32 = 0;
	prevKey: i32 = 0;
	nextKey: i32 = 0;
}

// Map from b3JointId to b3Joint in the solver sets
export struct b3Joint {
	userData: u64 = 0;

	// index of simulation set stored in b3World
	// B3_NULL_INDEX when slot is free
	setIndex: i32 = 0;

	// index into the constraint graph color array, may be B3_NULL_INDEX for sleeping/disabled joints
	// B3_NULL_INDEX when slot is free
	colorIndex: i32 = 0;

	// joint index within set or graph color
	// B3_NULL_INDEX when slot is free
	localIndex: i32 = 0;

	edges: FixedArray<b3JointEdge, 2>;

	jointId: i32 = 0;
	islandId: i32 = 0;

	// Index into the island's joints array for O(1) swap-removal.
	// B3_NULL_INDEX when not in an island.
	islandIndex: i32 = 0;

	drawScale: f32 = 0;

	type: b3JointType = 0;

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b3JointId
	generation: u16 = 0;

	collideConnected: bool = false;
}

export struct b3DistanceJoint {
	length: f32 = 0;
	hertz: f32 = 0;
	dampingRatio: f32 = 0;
	lowerSpringForce: f32 = 0;
	upperSpringForce: f32 = 0;
	minLength: f32 = 0;
	maxLength: f32 = 0;

	maxMotorForce: f32 = 0;
	motorSpeed: f32 = 0;

	impulse: f32 = 0;
	lowerImpulse: f32 = 0;
	upperImpulse: f32 = 0;
	motorImpulse: f32 = 0;

	indexA: i32 = 0;
	indexB: i32 = 0;
	anchorA: b3Vec3 = b3Vec3();
	anchorB: b3Vec3 = b3Vec3();
	deltaCenter: b3Vec3 = b3Vec3();
	distanceSoftness: b3Softness = b3Softness();
	axialMass: f32 = 0;

	enableSpring: bool = false;
	enableLimit: bool = false;
	enableMotor: bool = false;
}

export struct b3MotorJoint {
	linearVelocity: b3Vec3 = b3Vec3();
	angularVelocity: b3Vec3 = b3Vec3();
	maxVelocityForce: f32 = 0;
	maxVelocityTorque: f32 = 0;
	linearHertz: f32 = 0;
	linearDampingRatio: f32 = 0;
	maxSpringForce: f32 = 0;
	angularHertz: f32 = 0;
	angularDampingRatio: f32 = 0;
	maxSpringTorque: f32 = 0;

	linearVelocityImpulse: b3Vec3 = b3Vec3();
	angularVelocityImpulse: b3Vec3 = b3Vec3();
	linearSpringImpulse: b3Vec3 = b3Vec3();
	angularSpringImpulse: b3Vec3 = b3Vec3();

	linearSpring: b3Softness = b3Softness();
	angularSpring: b3Softness = b3Softness();

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b3Transform = b3Transform();
	frameB: b3Transform = b3Transform();
	deltaCenter: b3Vec3 = b3Vec3();
	angularMass: b3Matrix3 = b3Matrix3();
}

export struct b3ParallelJoint {
	hertz: f32 = 0;
	dampingRatio: f32 = 0;
	maxTorque: f32 = 0;

	perpImpulse: b3Vec2 = b3Vec2();
	perpAxisX: b3Vec3 = b3Vec3();
	perpAxisY: b3Vec3 = b3Vec3();

	quatA: b3Quat = b3Quat();
	quatB: b3Quat = b3Quat();
	indexA: i32 = 0;
	indexB: i32 = 0;
	softness: b3Softness = b3Softness();
}

export struct b3PrismaticJoint {
	perpImpulse: b3Vec2 = b3Vec2();
	angularImpulse: b3Vec3 = b3Vec3();
	springImpulse: f32 = 0;
	motorImpulse: f32 = 0;
	lowerImpulse: f32 = 0;
	upperImpulse: f32 = 0;
	hertz: f32 = 0;
	dampingRatio: f32 = 0;
	maxMotorForce: f32 = 0;
	motorSpeed: f32 = 0;
	targetTranslation: f32 = 0;
	lowerTranslation: f32 = 0;
	upperTranslation: f32 = 0;

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b3Transform = b3Transform();
	frameB: b3Transform = b3Transform();
	jointAxis: b3Vec3 = b3Vec3();
	perpAxisY: b3Vec3 = b3Vec3();
	perpAxisZ: b3Vec3 = b3Vec3();
	deltaCenter: b3Vec3 = b3Vec3();
	deltaAngle: f32 = 0;
	rotationMass: b3Matrix3 = b3Matrix3();
	springSoftness: b3Softness = b3Softness();

	enableSpring: bool = false;
	enableLimit: bool = false;
	enableMotor: bool = false;
}

export struct b3RevoluteJoint {
	linearImpulse: b3Vec3 = b3Vec3();
	perpImpulse: b3Vec2 = b3Vec2();
	springImpulse: f32 = 0;
	motorImpulse: f32 = 0;
	lowerImpulse: f32 = 0;
	upperImpulse: f32 = 0;
	hertz: f32 = 0;
	dampingRatio: f32 = 0;
	maxMotorTorque: f32 = 0;
	motorSpeed: f32 = 0;
	targetAngle: f32 = 0;
	lowerAngle: f32 = 0;
	upperAngle: f32 = 0;

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b3Transform = b3Transform();
	frameB: b3Transform = b3Transform();
	rotationAxisZ: b3Vec3 = b3Vec3();
	perpAxisX: b3Vec3 = b3Vec3();
	perpAxisY: b3Vec3 = b3Vec3();
	deltaCenter: b3Vec3 = b3Vec3();
	deltaAngle: f32 = 0;
	axialMass: f32 = 0;
	springSoftness: b3Softness = b3Softness();

	enableSpring: bool = false;
	enableMotor: bool = false;
	enableLimit: bool = false;
}

export struct b3SphericalJoint {
	linearImpulse: b3Vec3 = b3Vec3();
	springImpulse: b3Vec3 = b3Vec3();
	motorImpulse: b3Vec3 = b3Vec3();
	lowerTwistImpulse: f32 = 0;
	upperTwistImpulse: f32 = 0;
	swingImpulse: f32 = 0;
	hertz: f32 = 0;
	dampingRatio: f32 = 0;
	maxMotorTorque: f32 = 0;
	motorVelocity: b3Vec3 = b3Vec3();
	lowerTwistAngle: f32 = 0;
	upperTwistAngle: f32 = 0;
	coneAngle: f32 = 0;
	targetRotation: b3Quat = b3Quat();

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b3Transform = b3Transform();
	frameB: b3Transform = b3Transform();
	deltaCenter: b3Vec3 = b3Vec3();
	swingAxis: b3Vec3 = b3Vec3();
	twistJacobian: b3Vec3 = b3Vec3();

	rotationMass: b3Matrix3 = b3Matrix3();
	swingMass: f32 = 0;
	twistMass: f32 = 0;
	springSoftness: b3Softness = b3Softness();

	enableSpring: bool = false;
	enableMotor: bool = false;
	enableConeLimit: bool = false;
	enableTwistLimit: bool = false;
}

export struct b3WeldJoint {
	linearHertz: f32 = 0;
	linearDampingRatio: f32 = 0;
	angularHertz: f32 = 0;
	angularDampingRatio: f32 = 0;

	linearSpring: b3Softness = b3Softness();
	angularSpring: b3Softness = b3Softness();
	linearImpulse: b3Vec3 = b3Vec3();
	angularImpulse: b3Vec3 = b3Vec3();

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b3Transform = b3Transform();
	frameB: b3Transform = b3Transform();
	deltaCenter: b3Vec3 = b3Vec3();

	angularMass: b3Matrix3 = b3Matrix3();
}

export struct b3WheelJoint {
	linearImpulse: b3Vec2 = b3Vec2();
	angularImpulse: b3Vec2 = b3Vec2();
	spinImpulse: f32 = 0;
	maxSpinTorque: f32 = 0;
	spinSpeed: f32 = 0;
	suspensionSpringImpulse: f32 = 0;
	lowerSuspensionImpulse: f32 = 0;
	upperSuspensionImpulse: f32 = 0;
	lowerSuspensionLimit: f32 = 0;
	upperSuspensionLimit: f32 = 0;
	suspensionHertz: f32 = 0;
	suspensionDampingRatio: f32 = 0;
	steeringSpringImpulse: f32 = 0;
	lowerSteeringImpulse: f32 = 0;
	upperSteeringImpulse: f32 = 0;
	lowerSteeringLimit: f32 = 0;
	upperSteeringLimit: f32 = 0;
	targetSteeringAngle: f32 = 0;
	maxSteeringTorque: f32 = 0;
	steeringHertz: f32 = 0;
	steeringDampingRatio: f32 = 0;

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b3Transform = b3Transform();
	frameB: b3Transform = b3Transform();
	deltaCenter: b3Vec3 = b3Vec3();
	spinMass: f32 = 0;
	suspensionMass: f32 = 0;
	steeringMass: f32 = 0;
	suspensionSoftness: b3Softness = b3Softness();
	steeringSoftness: b3Softness = b3Softness();

	enableSpinMotor: bool = false;
	enableSuspensionSpring: bool = false;
	enableSuspensionLimit: bool = false;
	enableSteering: bool = false;
	enableSteeringLimit: bool = false;
	enableSteeringMotor: bool = false;
}

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
/// A reference class, not a struct: every C use goes through a b3JointSim* into a growable
/// array, and C's copies between arrays are moves.
export class b3JointSim {
	jointId: i32 = 0;

	bodyIdA: i32 = 0;
	bodyIdB: i32 = 0;

	type: b3JointType = 0;

	// Joint frames local to body origin
	localFrameA: b3Transform = b3Transform();
	localFrameB: b3Transform = b3Transform();

	invMassA: f32 = 0;
	invMassB: f32 = 0;
	invIA: b3Matrix3 = b3Matrix3();
	invIB: b3Matrix3 = b3Matrix3();

	constraintHertz: f32 = 0;
	constraintDampingRatio: f32 = 0;

	constraintSoftness: b3Softness = b3Softness();

	forceThreshold: f32 = 0;
	torqueThreshold: f32 = 0;

	fixedRotation: bool = false;

	// The C union becomes one field per member; they do not alias.
	distanceJoint: b3DistanceJoint = b3DistanceJoint();
	motorJoint: b3MotorJoint = b3MotorJoint();
	parallelJoint: b3ParallelJoint = b3ParallelJoint();
	revoluteJoint: b3RevoluteJoint = b3RevoluteJoint();
	sphericalJoint: b3SphericalJoint = b3SphericalJoint();
	prismaticJoint: b3PrismaticJoint = b3PrismaticJoint();
	weldJoint: b3WeldJoint = b3WeldJoint();
	wheelJoint: b3WheelJoint = b3WheelJoint();
}

// K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
export function b3SolvePointConstraint(
	mA: f32,
	mB: f32,
	iA: b3Matrix3,
	iB: b3Matrix3,
	rA: b3Vec3,
	rB: b3Vec3,
	rhs: b3Vec3
): b3Vec3 {
	let axy: f32 = rA.z * iA.cy.y - rA.y * iA.cz.y;
	let axz: f32 = rA.z * iA.cy.z - rA.y * iA.cz.z;
	let ay = b3Sub(b3MulSV(rA.x, iA.cz), b3MulSV(rA.z, iA.cx));
	let az = b3Sub(b3MulSV(rA.y, iA.cx), b3MulSV(rA.x, iA.cy));

	let bxy: f32 = rB.z * iB.cy.y - rB.y * iB.cz.y;
	let bxz: f32 = rB.z * iB.cy.z - rB.y * iB.cz.z;
	let by = b3Sub(b3MulSV(rB.x, iB.cz), b3MulSV(rB.z, iB.cx));
	let bz = b3Sub(b3MulSV(rB.y, iB.cx), b3MulSV(rB.x, iB.cy));

	let m: f32 = mA + mB;
	let kxx: f32 = (axy * rA.z - axz * rA.y) + (bxy * rB.z - bxz * rB.y) + m;
	let kyy: f32 = (ay.z * rA.x - ay.x * rA.z) + (by.z * rB.x - by.x * rB.z) + m;
	let kzz: f32 = (az.x * rA.y - az.y * rA.x) + (bz.x * rB.y - bz.y * rB.x) + m;
	let kxy: f32 = (ay.y * rA.z - ay.z * rA.y) + (by.y * rB.z - by.z * rB.y);
	let kxz: f32 = (az.y * rA.z - az.z * rA.y) + (bz.y * rB.z - bz.z * rB.y);
	let kyz: f32 = (az.z * rA.x - az.x * rA.z) + (bz.z * rB.x - bz.x * rB.z);

	let cxx: f32 = kyy * kzz - kyz * kyz;
	let cyy: f32 = kxx * kzz - kxz * kxz;
	let czz: f32 = kxx * kyy - kxy * kxy;
	let cxy: f32 = kxz * kyz - kxy * kzz;
	let cxz: f32 = kxy * kyz - kyy * kxz;
	let cyz: f32 = kxy * kxz - kxx * kyz;

	let det: f32 = kxx * cxx + kxy * cxy + kxz * cxz;
	if (b3AbsFloat(det) > 1000.0 * FLT_MIN) {
		let invDet: f32 = 1.0 / det;
		let b = b3Vec3(
			invDet * (cxx * rhs.x + cxy * rhs.y + cxz * rhs.z),
			invDet * (cxy * rhs.x + cyy * rhs.y + cyz * rhs.z),
			invDet * (cxz * rhs.x + cyz * rhs.y + czz * rhs.z)
		);
		return b;
	}

	return b3Vec3_zero;
}

// functions from joint.c follow

// The joint and explosion defaults live in types.as.
export {
	b3DefaultParallelJointDef,
	b3DefaultDistanceJointDef,
	b3DefaultMotorJointDef,
	b3DefaultFilterJointDef,
	b3DefaultPrismaticJointDef,
	b3DefaultRevoluteJointDef,
	b3DefaultSphericalJointDef,
	b3DefaultWeldJointDef,
	b3DefaultWheelJointDef,
	b3DefaultExplosionDef,
} from "./types";

export function b3GetJointFullId(world: b3World, jointId: b3JointId): i32 {
	let id = jointId.index1 - 1;
	B3_ASSERT(world.joints[id].jointId == id && world.joints[id].generation == jointId.generation);
	return id;
}

export function b3GetJointSim(world: b3World, jointId: i32): b3JointSim {
	let joint = world.joints[jointId];
	if (joint.setIndex == b3_awakeSet) {
		B3_ASSERT(0 <= joint.colorIndex && joint.colorIndex < B3_GRAPH_COLOR_COUNT);
		let color = world.constraintGraph.colors[joint.colorIndex];
		return color.jointSims[joint.localIndex];
	}

	let set = world.solverSets[joint.setIndex];
	return set.jointSims[joint.localIndex];
}

export function b3GetJointSimCheckType(jointId: b3JointId, type: b3JointType): b3JointSim {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	B3_ASSERT(world.joints[joint].type == type);
	let jointSim = b3GetJointSim(world, joint);
	B3_ASSERT(jointSim.type == type);
	return jointSim;
}

function b3DestroyContactsBetweenBodies(world: b3World, bodyIdA: i32, bodyIdB: i32): void {
	let contactKey: i32;
	let otherBodyId: i32;

	let bodyA = world.bodies[bodyIdA];
	let bodyB = world.bodies[bodyIdB];

	// use the smaller of the two contact lists
	if (bodyA.contactCount < bodyB.contactCount) {
		contactKey = bodyA.headContactKey;
		otherBodyId = bodyB.id;
	} else {
		contactKey = bodyB.headContactKey;
		otherBodyId = bodyA.id;
	}

	// no need to wake bodies when a joint removes collision between them
	let wakeBodies = false;

	// destroy the contacts
	while (contactKey != B3_NULL_INDEX) {
		let contactId = contactKey >> 1;
		let edgeIndex = contactKey & 1;

		let contact = world.contacts[contactId];
		contactKey = contact.edges[edgeIndex].nextKey;

		let otherEdgeIndex = edgeIndex ^ 1;
		if (contact.edges[otherEdgeIndex].bodyId == otherBodyId) {
			// Careful, this removes the contact from the current doubly linked list
			b3DestroyContact(world, contact, wakeBodies);
		}
	}
}

// C's b3Joint* is the joint index.
class b3JointPair {
	joint: i32;
	jointSim: b3JointSim;

	constructor(joint: i32, jointSim: b3JointSim) {
		this.joint = joint;
		this.jointSim = jointSim;
	}
}

function b3CreateJoint(world: b3World, def: b3JointDef, type: b3JointType): b3JointPair {
	B3_ASSERT(b3IsValidTransform(def.localFrameA));
	B3_ASSERT(b3IsValidTransform(def.localFrameB));
	B3_ASSERT(world.worldId == def.bodyIdA.world0);
	B3_ASSERT(world.worldId == def.bodyIdB.world0);
	B3_ASSERT((def.bodyIdA == def.bodyIdB) == false);

	let bodyIdA = b3GetBodyFullId(world, def.bodyIdA);
	let bodyIdB = b3GetBodyFullId(world, def.bodyIdB);

	// If the joint prevents collisions, then destroy all contacts between attached bodies.
	// Do this first so world arrays are not disturbed in the following code.
	if (def.collideConnected == false) {
		b3DestroyContactsBetweenBodies(world, bodyIdA, bodyIdB);
	}

	let maxSetIndex = b3MaxInt(world.bodies[bodyIdA].setIndex, world.bodies[bodyIdB].setIndex);

	// Create joint id and joint
	let jointId = b3AllocId(world.jointIdPool);
	if (jointId == world.joints.length) {
		world.joints.push(b3Joint());
	}

	world.joints[jointId].jointId = jointId;
	world.joints[jointId].userData = def.userData;
	world.joints[jointId].generation += 1;
	world.joints[jointId].setIndex = B3_NULL_INDEX;
	world.joints[jointId].colorIndex = B3_NULL_INDEX;
	world.joints[jointId].localIndex = B3_NULL_INDEX;
	world.joints[jointId].islandId = B3_NULL_INDEX;
	world.joints[jointId].islandIndex = B3_NULL_INDEX;
	world.joints[jointId].drawScale = def.drawScale;
	world.joints[jointId].type = type;
	world.joints[jointId].collideConnected = def.collideConnected;

	// Doubly linked list on bodyA
	world.joints[jointId].edges[0].bodyId = bodyIdA;
	world.joints[jointId].edges[0].prevKey = B3_NULL_INDEX;
	world.joints[jointId].edges[0].nextKey = world.bodies[bodyIdA].headJointKey;

	let keyA = (jointId << 1) | 0;
	let headKeyA = world.bodies[bodyIdA].headJointKey;
	if (headKeyA != B3_NULL_INDEX) {
		world.joints[headKeyA >> 1].edges[headKeyA & 1].prevKey = keyA;
	}
	world.bodies[bodyIdA].headJointKey = keyA;
	world.bodies[bodyIdA].jointCount += 1;

	// Doubly linked list on bodyB
	world.joints[jointId].edges[1].bodyId = bodyIdB;
	world.joints[jointId].edges[1].prevKey = B3_NULL_INDEX;
	world.joints[jointId].edges[1].nextKey = world.bodies[bodyIdB].headJointKey;

	let keyB = (jointId << 1) | 1;
	let headKeyB = world.bodies[bodyIdB].headJointKey;
	if (headKeyB != B3_NULL_INDEX) {
		world.joints[headKeyB >> 1].edges[headKeyB & 1].prevKey = keyB;
	}
	world.bodies[bodyIdB].headJointKey = keyB;
	world.bodies[bodyIdB].jointCount += 1;

	let jointSim: b3JointSim;

	let setIndexA = world.bodies[bodyIdA].setIndex;
	let setIndexB = world.bodies[bodyIdB].setIndex;

	if (setIndexA == b3_disabledSet || setIndexB == b3_disabledSet) {
		// if either body is disabled, create in disabled set
		let set = world.solverSets[b3_disabledSet];
		world.joints[jointId].setIndex = b3_disabledSet;
		world.joints[jointId].localIndex = set.jointSims.length;

		jointSim = new b3JointSim();
		set.jointSims.push(jointSim);

		jointSim.jointId = jointId;
		jointSim.bodyIdA = bodyIdA;
		jointSim.bodyIdB = bodyIdB;
	} else if (world.bodies[bodyIdA].type != b3_dynamicBody && world.bodies[bodyIdB].type != b3_dynamicBody) {
		// joint is not attached to a dynamic body
		let set = world.solverSets[b3_staticSet];
		world.joints[jointId].setIndex = b3_staticSet;
		world.joints[jointId].localIndex = set.jointSims.length;

		jointSim = new b3JointSim();
		set.jointSims.push(jointSim);

		jointSim.jointId = jointId;
		jointSim.bodyIdA = bodyIdA;
		jointSim.bodyIdB = bodyIdB;
	} else if (setIndexA == b3_awakeSet || setIndexB == b3_awakeSet) {
		// if either body is sleeping, wake it
		if (maxSetIndex >= b3_firstSleepingSet) {
			b3WakeSolverSet(world, maxSetIndex);
		}

		world.joints[jointId].setIndex = b3_awakeSet;

		jointSim = b3CreateJointInGraph(world, jointId);
		jointSim.jointId = jointId;
		jointSim.bodyIdA = bodyIdA;
		jointSim.bodyIdB = bodyIdB;
	} else {
		// joint connected between sleeping and/or static bodies
		B3_ASSERT(setIndexA >= b3_firstSleepingSet || setIndexB >= b3_firstSleepingSet);
		B3_ASSERT(setIndexA != b3_staticSet || setIndexB != b3_staticSet);

		// joint should go into the sleeping set (not static set)
		let setIndex = maxSetIndex;

		let set = world.solverSets[setIndex];
		world.joints[jointId].setIndex = setIndex;
		world.joints[jointId].localIndex = set.jointSims.length;

		jointSim = new b3JointSim();
		set.jointSims.push(jointSim);

		jointSim.jointId = jointId;
		jointSim.bodyIdA = bodyIdA;
		jointSim.bodyIdB = bodyIdB;

		if (setIndexA != setIndexB && setIndexA >= b3_firstSleepingSet && setIndexB >= b3_firstSleepingSet) {
			// merge sleeping sets
			b3MergeSolverSets(world, setIndexA, setIndexB);
			B3_ASSERT(world.bodies[bodyIdA].setIndex == world.bodies[bodyIdB].setIndex);

			// fix potentially invalid set index
			setIndex = world.bodies[bodyIdA].setIndex;

			let mergedSet = world.solverSets[setIndex];

			// Careful! The joint sim pointer was orphaned by the set merge.
			jointSim = mergedSet.jointSims[world.joints[jointId].localIndex];
		}

		B3_ASSERT(world.joints[jointId].setIndex == setIndex);
	}

	jointSim.localFrameA = def.localFrameA;
	jointSim.localFrameB = def.localFrameB;
	jointSim.type = type;
	jointSim.constraintHertz = def.constraintHertz;
	jointSim.constraintDampingRatio = def.constraintDampingRatio;
	jointSim.constraintSoftness = b3Softness(0.0, 1.0, 0.0);

	B3_ASSERT(b3IsValidFloat(def.forceThreshold) && def.forceThreshold >= 0.0);
	B3_ASSERT(b3IsValidFloat(def.torqueThreshold) && def.torqueThreshold >= 0.0);

	jointSim.forceThreshold = def.forceThreshold;
	jointSim.torqueThreshold = def.torqueThreshold;

	B3_ASSERT(jointSim.jointId == jointId);
	B3_ASSERT(jointSim.bodyIdA == bodyIdA);
	B3_ASSERT(jointSim.bodyIdB == bodyIdB);

	if (world.joints[jointId].setIndex > b3_disabledSet) {
		// Add edge to island graph
		b3LinkJoint(world, jointId);
	}

	return new b3JointPair(jointId, jointSim);
}

export function b3Joint_SetConstraintTuning(jointId: b3JointId, hertz: f32, dampingRatio: f32): void {
	B3_ASSERT(b3IsValidFloat(hertz) && hertz >= 0.0);
	B3_ASSERT(b3IsValidFloat(dampingRatio) && dampingRatio >= 0.0);

	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	let base = b3GetJointSim(world, joint);
	base.constraintHertz = hertz;
	base.constraintDampingRatio = dampingRatio;
}

export function b3Joint_GetConstraintTuning(jointId: b3JointId, hertz: ref f32, dampingRatio: ref f32): void {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	let base = b3GetJointSim(world, joint);
	hertz = base.constraintHertz;
	dampingRatio = base.constraintDampingRatio;
}

export function b3Joint_SetForceThreshold(jointId: b3JointId, threshold: f32): void {
	B3_ASSERT(b3IsValidFloat(threshold) && threshold >= 0.0);

	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	let base = b3GetJointSim(world, joint);
	base.forceThreshold = threshold;
}

export function b3Joint_GetForceThreshold(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	let base = b3GetJointSim(world, joint);
	return base.forceThreshold;
}

export function b3Joint_SetTorqueThreshold(jointId: b3JointId, threshold: f32): void {
	B3_ASSERT(b3IsValidFloat(threshold) && threshold >= 0.0);

	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	let base = b3GetJointSim(world, joint);
	base.torqueThreshold = threshold;
}

export function b3Joint_GetTorqueThreshold(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	let base = b3GetJointSim(world, joint);
	return base.torqueThreshold;
}

function b3MakeJointId(world: b3World, pair: b3JointPair): b3JointId {
	let jointId = b3JointId();
	jointId.index1 = pair.jointSim.jointId + 1;
	jointId.world0 = world.worldId;
	jointId.generation = world.joints[pair.joint].generation;
	return jointId;
}

export function b3CreateDistanceJoint(worldId: b3WorldId, def: b3DistanceJointDef): b3JointId {
	B3_ASSERT(def.base.internalValue == B3_SECRET_COOKIE);
	let w = b3GetUnlockedWorldFromId(worldId);
	if (w == null) {
		return b3JointId();
	}
	let world = w!;

	B3_ASSERT(b3IsValidFloat(def.length) && def.length > 0.0);
	B3_ASSERT(def.lowerSpringForce <= def.upperSpringForce);

	let pair = b3CreateJoint(world, def.base, b3_distanceJoint);

	let joint = pair.jointSim;

	joint.distanceJoint = b3DistanceJoint();
	joint.distanceJoint.length = b3MaxFloat(def.length, B3_LINEAR_SLOP());
	joint.distanceJoint.hertz = def.hertz;
	joint.distanceJoint.dampingRatio = def.dampingRatio;
	joint.distanceJoint.lowerSpringForce = def.lowerSpringForce;
	joint.distanceJoint.upperSpringForce = def.upperSpringForce;
	joint.distanceJoint.minLength = b3MaxFloat(def.minLength, B3_LINEAR_SLOP());
	joint.distanceJoint.maxLength = b3MaxFloat(def.minLength, def.maxLength);
	joint.distanceJoint.maxMotorForce = def.maxMotorForce;
	joint.distanceJoint.motorSpeed = def.motorSpeed;
	joint.distanceJoint.enableSpring = def.enableSpring;
	joint.distanceJoint.enableLimit = def.enableLimit;
	joint.distanceJoint.enableMotor = def.enableMotor;
	joint.distanceJoint.impulse = 0.0;
	joint.distanceJoint.lowerImpulse = 0.0;
	joint.distanceJoint.upperImpulse = 0.0;
	joint.distanceJoint.motorImpulse = 0.0;

	return b3MakeJointId(world, pair);
}

export function b3CreateMotorJoint(worldId: b3WorldId, def: b3MotorJointDef): b3JointId {
	B3_ASSERT(def.base.internalValue == B3_SECRET_COOKIE);
	let w = b3GetUnlockedWorldFromId(worldId);
	if (w == null) {
		return b3JointId();
	}
	let world = w!;

	let pair = b3CreateJoint(world, def.base, b3_motorJoint);
	let joint = pair.jointSim;

	joint.motorJoint = b3MotorJoint();
	joint.motorJoint.linearVelocity = def.linearVelocity;
	joint.motorJoint.maxVelocityForce = def.maxVelocityForce;
	joint.motorJoint.angularVelocity = def.angularVelocity;
	joint.motorJoint.maxVelocityTorque = def.maxVelocityTorque;
	joint.motorJoint.linearHertz = def.linearHertz;
	joint.motorJoint.linearDampingRatio = def.linearDampingRatio;
	joint.motorJoint.maxSpringForce = def.maxSpringForce;
	joint.motorJoint.angularHertz = def.angularHertz;
	joint.motorJoint.angularDampingRatio = def.angularDampingRatio;
	joint.motorJoint.maxSpringTorque = def.maxSpringTorque;

	return b3MakeJointId(world, pair);
}

export function b3CreateFilterJoint(worldId: b3WorldId, def: b3FilterJointDef): b3JointId {
	B3_ASSERT(def.base.internalValue == B3_SECRET_COOKIE);
	let w = b3GetUnlockedWorldFromId(worldId);
	if (w == null) {
		return b3JointId();
	}
	let world = w!;

	let pair = b3CreateJoint(world, def.base, b3_filterJoint);

	return b3MakeJointId(world, pair);
}

export function b3CreateParallelJoint(worldId: b3WorldId, def: b3ParallelJointDef): b3JointId {
	B3_ASSERT(def.base.internalValue == B3_SECRET_COOKIE);
	let w = b3GetUnlockedWorldFromId(worldId);
	if (w == null) {
		return b3JointId();
	}
	let world = w!;

	B3_ASSERT(b3IsValidFloat(def.hertz) && def.hertz >= 0.0);
	B3_ASSERT(b3IsValidFloat(def.dampingRatio) && def.dampingRatio >= 0.0);
	B3_ASSERT(b3IsValidFloat(def.maxTorque) && def.maxTorque >= 0.0);

	let pair = b3CreateJoint(world, def.base, b3_parallelJoint);

	let joint = pair.jointSim;

	joint.parallelJoint = b3ParallelJoint();
	joint.parallelJoint.hertz = def.hertz;
	joint.parallelJoint.dampingRatio = def.dampingRatio;
	joint.parallelJoint.maxTorque = def.maxTorque;

	return b3MakeJointId(world, pair);
}

export function b3CreatePrismaticJoint(worldId: b3WorldId, def: b3PrismaticJointDef): b3JointId {
	B3_ASSERT(def.base.internalValue == B3_SECRET_COOKIE);
	B3_ASSERT(def.lowerTranslation <= def.upperTranslation);

	let w = b3GetUnlockedWorldFromId(worldId);
	if (w == null) {
		return b3JointId();
	}
	let world = w!;

	let pair = b3CreateJoint(world, def.base, b3_prismaticJoint);

	let joint = pair.jointSim;

	joint.prismaticJoint = b3PrismaticJoint();
	joint.prismaticJoint.hertz = def.hertz;
	joint.prismaticJoint.dampingRatio = def.dampingRatio;
	joint.prismaticJoint.targetTranslation = def.targetTranslation;
	joint.prismaticJoint.lowerTranslation = def.lowerTranslation;
	joint.prismaticJoint.upperTranslation = def.upperTranslation;
	joint.prismaticJoint.maxMotorForce = def.maxMotorForce;
	joint.prismaticJoint.motorSpeed = def.motorSpeed;
	joint.prismaticJoint.enableSpring = def.enableSpring;
	joint.prismaticJoint.enableLimit = def.enableLimit;
	joint.prismaticJoint.enableMotor = def.enableMotor;

	return b3MakeJointId(world, pair);
}

export function b3CreateRevoluteJoint(worldId: b3WorldId, def: b3RevoluteJointDef): b3JointId {
	B3_ASSERT(def.base.internalValue == B3_SECRET_COOKIE);

	let w = b3GetUnlockedWorldFromId(worldId);
	if (w == null) {
		return b3JointId();
	}
	let world = w!;

	let pair = b3CreateJoint(world, def.base, b3_revoluteJoint);

	let joint = pair.jointSim;

	joint.revoluteJoint = b3RevoluteJoint();
	joint.revoluteJoint.hertz = def.hertz;
	joint.revoluteJoint.dampingRatio = def.dampingRatio;
	joint.revoluteJoint.targetAngle = b3ClampFloat(def.targetAngle, -B3_PI, B3_PI);

	let lowerAngle = b3MinFloat(def.lowerAngle, def.upperAngle);
	let upperAngle = b3MaxFloat(def.lowerAngle, def.upperAngle);
	joint.revoluteJoint.lowerAngle = b3ClampFloat(lowerAngle, -0.99 * B3_PI, 0.99 * B3_PI);
	joint.revoluteJoint.upperAngle = b3ClampFloat(upperAngle, -0.99 * B3_PI, 0.99 * B3_PI);

	joint.revoluteJoint.maxMotorTorque = def.maxMotorTorque;
	joint.revoluteJoint.motorSpeed = def.motorSpeed;
	joint.revoluteJoint.enableSpring = def.enableSpring;
	joint.revoluteJoint.enableLimit = def.enableLimit;
	joint.revoluteJoint.enableMotor = def.enableMotor;

	return b3MakeJointId(world, pair);
}

export function b3CreateSphericalJoint(worldId: b3WorldId, def: b3SphericalJointDef): b3JointId {
	B3_ASSERT(def.base.internalValue == B3_SECRET_COOKIE);
	B3_ASSERT(0.0 <= def.coneAngle && def.coneAngle <= 0.99 * B3_PI);
	B3_ASSERT(b3IsValidQuat(def.targetRotation));

	let w = b3GetUnlockedWorldFromId(worldId);
	if (w == null) {
		return b3JointId();
	}
	let world = w!;

	let pair = b3CreateJoint(world, def.base, b3_sphericalJoint);

	let joint = pair.jointSim;

	joint.sphericalJoint = b3SphericalJoint();
	joint.sphericalJoint.hertz = def.hertz;
	joint.sphericalJoint.dampingRatio = def.dampingRatio;
	joint.sphericalJoint.targetRotation = def.targetRotation;
	joint.sphericalJoint.coneAngle = b3ClampFloat(def.coneAngle, 0.0, 0.5 * B3_PI);

	let lowerAngle = b3MinFloat(def.lowerTwistAngle, def.upperTwistAngle);
	let upperAngle = b3MaxFloat(def.lowerTwistAngle, def.upperTwistAngle);
	joint.sphericalJoint.lowerTwistAngle = b3ClampFloat(lowerAngle, -0.99 * B3_PI, 0.99 * B3_PI);
	joint.sphericalJoint.upperTwistAngle = b3ClampFloat(upperAngle, -0.99 * B3_PI, 0.99 * B3_PI);

	joint.sphericalJoint.maxMotorTorque = def.maxMotorTorque;
	joint.sphericalJoint.motorVelocity = def.motorVelocity;
	joint.sphericalJoint.enableSpring = def.enableSpring;
	joint.sphericalJoint.enableConeLimit = def.enableConeLimit;
	joint.sphericalJoint.enableTwistLimit = def.enableTwistLimit;
	joint.sphericalJoint.enableMotor = def.enableMotor;

	return b3MakeJointId(world, pair);
}

export function b3CreateWeldJoint(worldId: b3WorldId, def: b3WeldJointDef): b3JointId {
	B3_ASSERT(def.base.internalValue == B3_SECRET_COOKIE);
	B3_ASSERT(0.0 <= def.angularHertz);
	B3_ASSERT(0.0 <= def.angularDampingRatio);
	B3_ASSERT(0.0 <= def.linearHertz);
	B3_ASSERT(0.0 <= def.linearDampingRatio);

	let w = b3GetUnlockedWorldFromId(worldId);
	if (w == null) {
		return b3JointId();
	}
	let world = w!;

	let pair = b3CreateJoint(world, def.base, b3_weldJoint);

	let joint = pair.jointSim;

	joint.weldJoint = b3WeldJoint();
	joint.weldJoint.linearHertz = def.linearHertz;
	joint.weldJoint.linearDampingRatio = def.linearDampingRatio;
	joint.weldJoint.angularHertz = def.angularHertz;
	joint.weldJoint.angularDampingRatio = def.angularDampingRatio;

	return b3MakeJointId(world, pair);
}

export function b3CreateWheelJoint(worldId: b3WorldId, def: b3WheelJointDef): b3JointId {
	B3_ASSERT(def.base.internalValue == B3_SECRET_COOKIE);
	B3_ASSERT(def.lowerSuspensionLimit <= def.upperSuspensionLimit);

	let w = b3GetUnlockedWorldFromId(worldId);
	if (w == null) {
		return b3JointId();
	}
	let world = w!;

	let pair = b3CreateJoint(world, def.base, b3_wheelJoint);

	let joint = pair.jointSim;

	joint.wheelJoint = b3WheelJoint();
	joint.wheelJoint.enableSuspensionSpring = def.enableSuspensionSpring;
	joint.wheelJoint.suspensionHertz = def.suspensionHertz;
	joint.wheelJoint.suspensionDampingRatio = def.suspensionDampingRatio;
	joint.wheelJoint.enableSuspensionLimit = def.enableSuspensionLimit;
	joint.wheelJoint.lowerSuspensionLimit = def.lowerSuspensionLimit;
	joint.wheelJoint.upperSuspensionLimit = def.upperSuspensionLimit;
	joint.wheelJoint.enableSpinMotor = def.enableSpinMotor;
	joint.wheelJoint.maxSpinTorque = def.maxSpinTorque;
	joint.wheelJoint.spinSpeed = def.spinSpeed;

	joint.wheelJoint.enableSteering = def.enableSteering;
	joint.wheelJoint.steeringHertz = def.steeringHertz;
	joint.wheelJoint.steeringDampingRatio = def.steeringDampingRatio;
	joint.wheelJoint.targetSteeringAngle = def.targetSteeringAngle;
	joint.wheelJoint.maxSteeringTorque = def.maxSteeringTorque;
	joint.wheelJoint.enableSteeringLimit = def.enableSteeringLimit;
	joint.wheelJoint.lowerSteeringLimit = def.lowerSteeringLimit;
	joint.wheelJoint.upperSteeringLimit = def.upperSteeringLimit;

	return b3MakeJointId(world, pair);
}

export function b3DestroyJointInternal(world: b3World, jointId: i32, wakeBodies: bool): void {
	// The edges of other joints never alias this joint's edges, so copies stay valid below.
	let edgeA = world.joints[jointId].edges[0];
	let edgeB = world.joints[jointId].edges[1];

	let idA = edgeA.bodyId;
	let idB = edgeB.bodyId;

	// Remove from body A
	if (edgeA.prevKey != B3_NULL_INDEX) {
		world.joints[edgeA.prevKey >> 1].edges[edgeA.prevKey & 1].nextKey = edgeA.nextKey;
	}

	if (edgeA.nextKey != B3_NULL_INDEX) {
		world.joints[edgeA.nextKey >> 1].edges[edgeA.nextKey & 1].prevKey = edgeA.prevKey;
	}

	let edgeKeyA = (jointId << 1) | 0;
	if (world.bodies[idA].headJointKey == edgeKeyA) {
		world.bodies[idA].headJointKey = edgeA.nextKey;
	}

	world.bodies[idA].jointCount -= 1;

	// Remove from body B
	if (edgeB.prevKey != B3_NULL_INDEX) {
		world.joints[edgeB.prevKey >> 1].edges[edgeB.prevKey & 1].nextKey = edgeB.nextKey;
	}

	if (edgeB.nextKey != B3_NULL_INDEX) {
		world.joints[edgeB.nextKey >> 1].edges[edgeB.nextKey & 1].prevKey = edgeB.prevKey;
	}

	let edgeKeyB = (jointId << 1) | 1;
	if (world.bodies[idB].headJointKey == edgeKeyB) {
		world.bodies[idB].headJointKey = edgeB.nextKey;
	}

	world.bodies[idB].jointCount -= 1;

	if (world.joints[jointId].islandId != B3_NULL_INDEX) {
		B3_ASSERT(world.joints[jointId].setIndex > b3_disabledSet);
		b3UnlinkJoint(world, jointId);
	} else {
		B3_ASSERT(world.joints[jointId].setIndex <= b3_disabledSet);
	}

	// Remove joint from solver set that owns it
	let setIndex = world.joints[jointId].setIndex;
	let localIndex = world.joints[jointId].localIndex;

	if (setIndex == b3_awakeSet) {
		b3RemoveJointFromGraph(
			world,
			world.joints[jointId].edges[0].bodyId,
			world.joints[jointId].edges[1].bodyId,
			world.joints[jointId].colorIndex,
			localIndex
		);
	} else {
		let set = world.solverSets[setIndex];
		let movedIndex = b3Array_RemoveSwap(set.jointSims, localIndex);
		if (movedIndex != B3_NULL_INDEX) {
			// Fix moved joint
			let movedJointSim = set.jointSims[localIndex];
			let movedId = movedJointSim.jointId;
			B3_ASSERT(world.joints[movedId].localIndex == movedIndex);
			world.joints[movedId].localIndex = localIndex;
		}
	}

	// Free joint and id (preserve joint revision)
	world.joints[jointId].setIndex = B3_NULL_INDEX;
	world.joints[jointId].localIndex = B3_NULL_INDEX;
	world.joints[jointId].colorIndex = B3_NULL_INDEX;
	world.joints[jointId].jointId = B3_NULL_INDEX;
	b3FreeId(world.jointIdPool, jointId);

	if (wakeBodies) {
		b3WakeBody(world, idA);
		b3WakeBody(world, idB);
	}
}

export function b3DestroyJoint(jointId: b3JointId, wakeAttached: bool): void {
	// C checks for a NULL world here, but b3GetWorld never returns one.
	let world = b3GetWorld(jointId.world0);

	let joint = b3GetJointFullId(world, jointId);

	b3DestroyJointInternal(world, joint, wakeAttached);
}

export function b3Joint_GetType(jointId: b3JointId): b3JointType {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	return world.joints[joint].type;
}

export function b3Joint_GetBodyA(jointId: b3JointId): b3BodyId {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	return b3MakeBodyId(world, world.joints[joint].edges[0].bodyId);
}

export function b3Joint_GetBodyB(jointId: b3JointId): b3BodyId {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	return b3MakeBodyId(world, world.joints[joint].edges[1].bodyId);
}

export function b3Joint_GetWorld(jointId: b3JointId): b3WorldId {
	let world = b3GetWorld(jointId.world0);
	let worldId = b3WorldId();
	worldId.index1 = <u16>(jointId.world0 + 1);
	worldId.generation = world.generation;
	return worldId;
}

export function b3Joint_SetLocalFrameA(jointId: b3JointId, localFrame: b3Transform): void {
	B3_ASSERT(b3IsValidTransform(localFrame));

	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	let jointSim = b3GetJointSim(world, joint);
	jointSim.localFrameA = localFrame;
}

export function b3Joint_GetLocalFrameA(jointId: b3JointId): b3Transform {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	let jointSim = b3GetJointSim(world, joint);
	return jointSim.localFrameA;
}

export function b3Joint_SetLocalFrameB(jointId: b3JointId, localFrame: b3Transform): void {
	B3_ASSERT(b3IsValidTransform(localFrame));

	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	let jointSim = b3GetJointSim(world, joint);
	jointSim.localFrameB = localFrame;
}

export function b3Joint_GetLocalFrameB(jointId: b3JointId): b3Transform {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	let jointSim = b3GetJointSim(world, joint);
	return jointSim.localFrameB;
}

export function b3Joint_SetCollideConnected(jointId: b3JointId, shouldCollide: bool): void {
	let w = b3GetUnlockedWorld(jointId.world0);
	if (w == null) {
		return;
	}
	let world = w!;

	let joint = b3GetJointFullId(world, jointId);
	if (world.joints[joint].collideConnected == shouldCollide) {
		return;
	}

	world.joints[joint].collideConnected = shouldCollide;

	let bodyIdA = world.joints[joint].edges[0].bodyId;
	let bodyIdB = world.joints[joint].edges[1].bodyId;

	if (shouldCollide) {
		// need to tell the broad-phase to look for new pairs for one of the
		// two bodies. Pick the one with the fewest shapes.
		let shapeCountA = world.bodies[bodyIdA].shapeCount;
		let shapeCountB = world.bodies[bodyIdB].shapeCount;

		let shapeId = shapeCountA < shapeCountB ? world.bodies[bodyIdA].headShapeId : world.bodies[bodyIdB].headShapeId;
		while (shapeId != B3_NULL_INDEX) {
			let shape = world.shapes[shapeId];

			if (shape.proxyKey != B3_NULL_INDEX) {
				b3BroadPhase_MarkProxyMovedSerial(world.broadPhase, shape.proxyKey);
			}

			shapeId = shape.nextShapeId;
		}
	} else {
		b3DestroyContactsBetweenBodies(world, bodyIdA, bodyIdB);
	}
}

export function b3Joint_GetCollideConnected(jointId: b3JointId): bool {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	return world.joints[joint].collideConnected;
}

export function b3Joint_SetUserData(jointId: b3JointId, userData: u64): void {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	world.joints[joint].userData = userData;
}

export function b3Joint_GetUserData(jointId: b3JointId): u64 {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	return world.joints[joint].userData;
}

export function b3Joint_WakeBodies(jointId: b3JointId): void {
	let w = b3GetUnlockedWorld(jointId.world0);
	if (w == null) {
		return;
	}
	let world = w!;

	world.locked = true;

	let joint = b3GetJointFullId(world, jointId);
	let bodyIdA = world.joints[joint].edges[0].bodyId;
	let bodyIdB = world.joints[joint].edges[1].bodyId;

	b3WakeBody(world, bodyIdA);
	b3WakeBody(world, bodyIdB);

	world.locked = false;
}

export function b3Joint_IsAwake(jointId: b3JointId): bool {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);

	// Cheaper than checking if the bodies are awake.
	return world.joints[joint].setIndex == b3_awakeSet;
}

export function b3GetJointReaction(world: b3World, sim: b3JointSim, invTimeStep: f32, force: ref f32, torque: ref f32): void {
	let linearImpulse: f32 = 0.0;
	let angularImpulse: f32 = 0.0;

	switch (sim.type) {
		case b3_parallelJoint: {
			ref joint = sim.parallelJoint;
			let impulse = b3Vec3(joint.perpImpulse.x, joint.perpImpulse.y, 0.0);
			angularImpulse = b3Length(impulse);
			break;
		}

		case b3_distanceJoint: {
			ref joint = sim.distanceJoint;
			linearImpulse = b3AbsFloat(joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse);
			break;
		}

		case b3_motorJoint: {
			ref joint = sim.motorJoint;
			linearImpulse = b3Length(b3Add(joint.linearVelocityImpulse, joint.linearSpringImpulse));
			angularImpulse = b3Length(b3Add(joint.angularVelocityImpulse, joint.angularSpringImpulse));
			break;
		}

		case b3_prismaticJoint: {
			ref joint = sim.prismaticJoint;
			let impulse = b3Vec3(
				joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse,
				joint.perpImpulse.x,
				joint.perpImpulse.y
			);
			linearImpulse = b3Length(impulse);
			angularImpulse = b3Length(joint.angularImpulse);
			break;
		}

		case b3_revoluteJoint: {
			ref joint = sim.revoluteJoint;
			linearImpulse = b3Length(joint.linearImpulse);
			let impulse = b3Vec3(
				joint.perpImpulse.x,
				joint.perpImpulse.y,
				joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse
			);
			angularImpulse = b3Length(impulse);
			break;
		}

		case b3_sphericalJoint: {
			// todo improve performance
			ref joint = sim.sphericalJoint;
			linearImpulse = b3Length(joint.linearImpulse);

			let xfA = b3GetBodyTransform(world, sim.bodyIdA);
			let xfB = b3GetBodyTransform(world, sim.bodyIdB);
			let qA = b3MulQuat(xfA.q, sim.localFrameA.q);
			let qB = b3MulQuat(xfB.q, sim.localFrameB.q);

			// Cone axis is the z-axis of body A.
			let coneAxis = b3RotateVector(qA, b3Vec3_axisZ);
			let twistAxis = b3RotateVector(qB, b3Vec3_axisZ);
			let swingAxis = b3Normalize(b3Cross(coneAxis, twistAxis));

			let impulse = b3Add(joint.springImpulse, joint.motorImpulse);
			impulse = b3MulAdd(impulse, joint.lowerTwistImpulse - joint.upperTwistImpulse, twistAxis);
			impulse = b3MulAdd(impulse, joint.swingImpulse, swingAxis);

			angularImpulse = b3Length(impulse);
			break;
		}

		case b3_weldJoint: {
			ref joint = sim.weldJoint;
			linearImpulse = b3Length(joint.linearImpulse);
			angularImpulse = b3Length(joint.angularImpulse);
			break;
		}

		case b3_wheelJoint: {
			// todo probably wrong
			ref joint = sim.wheelJoint;
			let perpImpulse = joint.linearImpulse;
			let axialImpulse: f32 = joint.suspensionSpringImpulse + joint.lowerSuspensionImpulse - joint.upperSuspensionImpulse;
			linearImpulse = Mathf.sqrt(perpImpulse.x * perpImpulse.x + perpImpulse.y * perpImpulse.y + axialImpulse * axialImpulse);
			angularImpulse = b3AbsFloat(joint.spinImpulse);
			break;
		}

		default:
			break;
	}

	force = linearImpulse * invTimeStep;
	torque = angularImpulse * invTimeStep;
}

function b3GetJointConstraintForce(world: b3World, jointId: i32): b3Vec3 {
	let base = b3GetJointSim(world, jointId);

	switch (world.joints[jointId].type) {
		case b3_parallelJoint:
			return b3Vec3_zero;

		case b3_distanceJoint:
			return b3GetDistanceJointForce(world, base);

		case b3_filterJoint:
			return b3Vec3_zero;

		case b3_motorJoint:
			return b3GetMotorJointForce(world, base);

		case b3_prismaticJoint:
			return b3GetPrismaticJointForce(world, base);

		case b3_revoluteJoint:
			return b3GetRevoluteJointForce(world, base);

		case b3_sphericalJoint:
			return b3GetSphericalJointForce(world, base);

		case b3_weldJoint:
			return b3GetWeldJointForce(world, base);

		case b3_wheelJoint:
			return b3GetWheelJointForce(world, base);

		default:
			B3_ASSERT(false);
			return b3Vec3_zero;
	}
}

function b3GetJointConstraintTorque(world: b3World, jointId: i32): b3Vec3 {
	let base = b3GetJointSim(world, jointId);

	switch (world.joints[jointId].type) {
		case b3_parallelJoint:
			return b3GetParallelJointTorque(world, base);

		case b3_distanceJoint:
			return b3Vec3_zero;

		case b3_filterJoint:
			return b3Vec3_zero;

		case b3_motorJoint:
			return b3GetMotorJointTorque(world, base);

		case b3_prismaticJoint:
			return b3GetPrismaticJointTorque(world, base);

		case b3_revoluteJoint:
			return b3GetRevoluteJointTorque(world, base);

		case b3_sphericalJoint:
			return b3GetSphericalJointTorque(world, base);

		case b3_weldJoint:
			return b3GetWeldJointTorque(world, base);

		case b3_wheelJoint:
			return b3GetWheelJointTorque(world, base);

		default:
			B3_ASSERT(false);
			return b3Vec3_zero;
	}
}

export function b3Joint_GetConstraintForce(jointId: b3JointId): b3Vec3 {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	return b3GetJointConstraintForce(world, joint);
}

export function b3Joint_GetConstraintTorque(jointId: b3JointId): b3Vec3 {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	return b3GetJointConstraintTorque(world, joint);
}

export function b3Joint_GetLinearSeparation(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	let base = b3GetJointSim(world, joint);

	let xfA = b3GetBodyTransform(world, world.joints[joint].edges[0].bodyId);
	let xfB = b3GetBodyTransform(world, world.joints[joint].edges[1].bodyId);

	let pA = b3TransformWorldPoint(xfA, base.localFrameA.p);
	let pB = b3TransformWorldPoint(xfB, base.localFrameB.p);
	let dp = b3SubPos(pB, pA);

	switch (world.joints[joint].type) {
		case b3_parallelJoint:
			return 0.0;

		case b3_distanceJoint: {
			ref distanceJoint = base.distanceJoint;
			let length = b3Length(dp);
			if (distanceJoint.enableSpring) {
				if (distanceJoint.enableLimit) {
					if (length < distanceJoint.minLength) {
						return distanceJoint.minLength - length;
					}

					if (length > distanceJoint.maxLength) {
						return length - distanceJoint.maxLength;
					}

					return 0.0;
				}

				return 0.0;
			}

			return b3AbsFloat(length - distanceJoint.length);
		}

		case b3_motorJoint:
			return 0.0;

		case b3_filterJoint:
			return 0.0;

		case b3_prismaticJoint: {
			ref prismaticJoint = base.prismaticJoint;
			let axisA = b3RotateVector(xfA.q, b3Vec3_axisX);
			let perpA = b3Perp(axisA);
			let perpendicularSeparation = b3AbsFloat(b3Dot(perpA, dp));
			let limitSeparation: f32 = 0.0;

			if (prismaticJoint.enableLimit) {
				let translation = b3Dot(axisA, dp);
				if (translation < prismaticJoint.lowerTranslation) {
					limitSeparation = prismaticJoint.lowerTranslation - translation;
				}

				if (prismaticJoint.upperTranslation < translation) {
					limitSeparation = translation - prismaticJoint.upperTranslation;
				}
			}

			return Mathf.sqrt(perpendicularSeparation * perpendicularSeparation + limitSeparation * limitSeparation);
		}

		case b3_revoluteJoint:
			return b3Length(dp);

		case b3_sphericalJoint:
			return b3Length(dp);

		case b3_weldJoint: {
			ref weldJoint = base.weldJoint;
			if (weldJoint.linearHertz == 0.0) {
				return b3Length(dp);
			}

			return 0.0;
		}

		case b3_wheelJoint: {
			ref wheelJoint = base.wheelJoint;
			let axisA = b3RotateVector(xfA.q, b3Vec3_axisX);
			let perpA = b3Perp(axisA);
			let perpendicularSeparation = b3AbsFloat(b3Dot(perpA, dp));
			let limitSeparation: f32 = 0.0;

			if (wheelJoint.enableSuspensionLimit) {
				let translation = b3Dot(axisA, dp);
				if (translation < wheelJoint.lowerSuspensionLimit) {
					limitSeparation = wheelJoint.lowerSuspensionLimit - translation;
				}

				if (wheelJoint.upperSuspensionLimit < translation) {
					limitSeparation = translation - wheelJoint.upperSuspensionLimit;
				}
			}

			return Mathf.sqrt(perpendicularSeparation * perpendicularSeparation + limitSeparation * limitSeparation);
		}

		default:
			B3_ASSERT(false);
			return 0.0;
	}
}

export function b3Joint_GetAngularSeparation(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointFullId(world, jointId);
	let base = b3GetJointSim(world, joint);

	let xfA = b3GetBodyTransform(world, world.joints[joint].edges[0].bodyId);
	let xfB = b3GetBodyTransform(world, world.joints[joint].edges[1].bodyId);

	let relQ = b3InvMulQuat(xfA.q, xfB.q);

	switch (world.joints[joint].type) {
		case b3_parallelJoint: {
			// Remove hinge angle
			relQ.v.z = 0.0;
			return b3GetQuatAngle(relQ);
		}

		case b3_distanceJoint:
			return 0.0;

		case b3_motorJoint:
			return 0.0;

		case b3_filterJoint:
			return 0.0;

		case b3_prismaticJoint:
			return b3GetQuatAngle(relQ);

		case b3_revoluteJoint: {
			ref revoluteJoint = base.revoluteJoint;
			if (revoluteJoint.enableLimit) {
				let angle = b3GetTwistAngle(relQ);
				if (angle < revoluteJoint.lowerAngle) {
					return b3GetQuatAngle(relQ);
				}

				if (revoluteJoint.upperAngle < angle) {
					return b3GetQuatAngle(relQ);
				}
			}

			// Remove hinge angle
			relQ.v.z = 0.0;
			return b3GetQuatAngle(relQ);
		}

		case b3_sphericalJoint: {
			ref sphericalJoint = base.sphericalJoint;
			let sum: f32 = 0.0;
			if (sphericalJoint.enableConeLimit) {
				let swingAngle = b3GetSwingAngle(relQ);
				sum += b3MaxFloat(0.0, swingAngle - sphericalJoint.coneAngle);
			}

			if (sphericalJoint.enableTwistLimit) {
				let twistAngle = b3GetTwistAngle(relQ);
				sum += b3MaxFloat(0.0, sphericalJoint.lowerTwistAngle - twistAngle);
				sum += b3MaxFloat(0.0, twistAngle - sphericalJoint.upperTwistAngle);
			}

			return sum;
		}

		case b3_weldJoint: {
			ref weldJoint = base.weldJoint;
			if (weldJoint.angularHertz == 0.0) {
				return b3GetQuatAngle(relQ);
			}

			return 0.0;
		}

		case b3_wheelJoint:
			// todo
			B3_ASSERT(false);
			return 0.0;

		default:
			B3_ASSERT(false);
			return 0.0;
	}
}

export function b3PrepareJoint(joint: b3JointSim, context: b3StepContext): void {
	// Clamp joint hertz based on the time step to reduce jitter.
	let hertz = b3MinFloat(joint.constraintHertz, 0.25 * context.inv_h);
	joint.constraintSoftness = b3MakeSoft(hertz, joint.constraintDampingRatio, context.h);

	switch (joint.type) {
		case b3_parallelJoint:
			b3PrepareParallelJoint(joint, context);
			break;

		case b3_distanceJoint:
			b3PrepareDistanceJoint(joint, context);
			break;

		case b3_filterJoint:
			break;

		case b3_motorJoint:
			b3PrepareMotorJoint(joint, context);
			break;

		case b3_prismaticJoint:
			b3PreparePrismaticJoint(joint, context);
			break;

		case b3_revoluteJoint:
			b3PrepareRevoluteJoint(joint, context);
			break;

		case b3_sphericalJoint:
			b3PrepareSphericalJoint(joint, context);
			break;

		case b3_weldJoint:
			b3PrepareWeldJoint(joint, context);
			break;

		case b3_wheelJoint:
			b3PrepareWheelJoint(joint, context);
			break;

		default:
			B3_ASSERT(false);
	}
}

export function b3WarmStartJoint(joint: b3JointSim, context: b3StepContext): void {
	switch (joint.type) {
		case b3_parallelJoint:
			b3WarmStartParallelJoint(joint, context);
			break;

		case b3_distanceJoint:
			b3WarmStartDistanceJoint(joint, context);
			break;

		case b3_filterJoint:
			break;

		case b3_motorJoint:
			b3WarmStartMotorJoint(joint, context);
			break;

		case b3_prismaticJoint:
			b3WarmStartPrismaticJoint(joint, context);
			break;

		case b3_revoluteJoint:
			b3WarmStartRevoluteJoint(joint, context);
			break;

		case b3_sphericalJoint:
			b3WarmStartSphericalJoint(joint, context);
			break;

		case b3_weldJoint:
			b3WarmStartWeldJoint(joint, context);
			break;

		case b3_wheelJoint:
			b3WarmStartWheelJoint(joint, context);
			break;

		default:
			B3_ASSERT(false);
	}
}

export function b3SolveJoint(joint: b3JointSim, context: b3StepContext, useBias: bool): void {
	switch (joint.type) {
		case b3_parallelJoint:
			b3SolveParallelJoint(joint, context);
			break;

		case b3_distanceJoint:
			b3SolveDistanceJoint(joint, context, useBias);
			break;

		case b3_filterJoint:
			break;

		case b3_motorJoint:
			b3SolveMotorJoint(joint, context);
			break;

		case b3_prismaticJoint:
			b3SolvePrismaticJoint(joint, context, useBias);
			break;

		case b3_revoluteJoint:
			b3SolveRevoluteJoint(joint, context, useBias);
			break;

		case b3_sphericalJoint:
			b3SolveSphericalJoint(joint, context, useBias);
			break;

		case b3_weldJoint:
			b3SolveWeldJoint(joint, context, useBias);
			break;

		case b3_wheelJoint:
			b3SolveWheelJoint(joint, context, useBias);
			break;

		default:
			B3_ASSERT(false);
	}
}

export function b3PrepareJoints_Overflow(context: b3StepContext): void {
	let graph = context.graph;
	let joints = graph.colors[B3_OVERFLOW_INDEX].jointSims;
	let jointCount = joints.length;

	for (let i = 0; i < jointCount; ++i) {
		let joint = joints[i];
		b3PrepareJoint(joint, context);
	}
}

export function b3WarmStartJoints_Overflow(context: b3StepContext): void {
	let graph = context.graph;
	let joints = graph.colors[B3_OVERFLOW_INDEX].jointSims;
	let jointCount = joints.length;

	for (let i = 0; i < jointCount; ++i) {
		let joint = joints[i];
		b3WarmStartJoint(joint, context);
	}
}

export function b3SolveJoints_Overflow(context: b3StepContext, useBias: bool): void {
	let graph = context.graph;
	let joints = graph.colors[B3_OVERFLOW_INDEX].jointSims;
	let jointCount = joints.length;

	for (let i = 0; i < jointCount; ++i) {
		let joint = joints[i];
		b3SolveJoint(joint, context, useBias);
	}
}
