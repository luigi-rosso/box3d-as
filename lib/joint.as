// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/joint.c

import { FLT_MIN } from "./core";
import { b3AbsFloat, b3Matrix3, b3MulSV, b3Quat, b3Sub, b3Transform, b3Vec2, b3Vec3, b3Vec3_zero } from "./math_functions";
import { b3JointType } from "./types";
import { b3Softness } from "./solver";

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
