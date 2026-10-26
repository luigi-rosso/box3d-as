// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/prismatic_joint.c

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX, FLT_MIN } from "./core";
import {
	b3Add,
	b3AddMM,
	b3Blend2,
	b3ClampFloat,
	b3Cross,
	b3Det,
	b3Dot,
	b3InvMulQuat,
	b3InvertMatrix,
	b3IsValidFloat,
	b3IsValidVec3,
	b3MakeMatrixFromQuat,
	b3MaxFloat,
	b3MinFloat,
	b3MulAdd,
	b3MulMV,
	b3MulQuat,
	b3MulSV,
	b3MulSub,
	b3Neg,
	b3Quat_identity,
	b3RotateVector,
	b3Sub,
	b3SubPos,
	b3Vec2,
	b3Vec3,
	b3Vec3_axisX,
	b3Vec3_zero,
} from "./math_functions";
import { b3Add2, b3Blend3, b3DeltaQuatToRotation, b3Matrix2, b3MulSV2, b3Solve2, b3Sub2 } from "./math_internal";
import { b3JointId, b3_prismaticJoint } from "./types";
import { b3GetBodySim, b3GetBodyState, b3GetBodyTransform, b3_dynamicFlag, b3BodyState, b3StateAddress, b3StoreStateVelocities } from "./body";
import { b3GetWorld, b3World, b3_awakeSet } from "./physics_world";
import { b3MakeSoft, b3StepContext } from "./solver";
import { b3GetJointSimCheckType, b3JointSim } from "./joint";

// Linear constraint (point-to-line)
// joint axis is along joint frame A local z-axis
// perpX and perpY are world vectors fixed in A
//
// d = pB - pA = xB + rB - xA - rA
// Cx = dot(perpX, d)
// Cy = dot(perpY, d)

// CdotX = dot(d, cross(wA, perpX)) + dot(perpX, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(perpX, vA) - dot(cross(d + rA, perpX), wA) + dot(perpX, vB) + dot(cross(rB, perpX), vB)
// Jx = [-perpX, -cross(d + rA, perpX), perpX, cross(rB, perpX)]
// similar for perpY
//
// Simplification dropping dot(d, cross(wA, perpX)) (todo needs testing)
// CdotXs = dot(perpX, vB + cross(wB, rB) - vA - cross(wA, rA))
// Jxs = [-perpX, -cross(rA, perpX), perpX, cross(rB, perpX)]

// Motor/limit/spring linear constraint
// axis is the world joint axis fixed in A

// C = dot(axis, d)
// Cdot = dot(d, cross(wA, axis)) + dot(axis, vB + cross(wB, rB) - vA - cross(wA, rA))
// Cdot = -dot(axis, vA) - dot(cross(d + rA, axis), wA) + dot(axis, vB) + dot(cross(rB, axis), vB)
// J = [-axis -cross(d + rA, axis) axis cross(rB, axis)]
//
// Simplified (todo needs testing)
// Cdot = -dot(axis, vA) - dot(cross(rA, axis), wA) + dot(axis, vB) + dot(cross(rB, axis), vB)
// J = [-axis -cross(rA, axis) axis cross(rB, axis)]

// Predictive limit is applied even when the limit is not active.
// Prevents a constraint speed that can lead to a constraint error in one time step.
// Want C2 = C1 + h * Cdot >= 0
// Or:
// Cdot + C1/h >= 0
// I do not apply a negative constraint error because that is handled in position correction.
// So:
// Cdot + max(C1, 0)/h >= 0

export function b3PrismaticJoint_EnableLimit(jointId: b3JointId, enableLimit: bool): void {
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	if (enableLimit != base.prismaticJoint.enableLimit) {
		base.prismaticJoint.lowerImpulse = 0.0;
		base.prismaticJoint.upperImpulse = 0.0;
	}
	base.prismaticJoint.enableLimit = enableLimit;
}

export function b3PrismaticJoint_IsLimitEnabled(jointId: b3JointId): bool {
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	return base.prismaticJoint.enableLimit;
}

export function b3PrismaticJoint_GetLowerLimit(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	return base.prismaticJoint.lowerTranslation;
}

export function b3PrismaticJoint_GetUpperLimit(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	return base.prismaticJoint.upperTranslation;
}

export function b3PrismaticJoint_SetLimits(jointId: b3JointId, lower: f32, upper: f32): void {
	B3_ASSERT(b3IsValidFloat(lower) && b3IsValidFloat(upper));
	let lowerAngle = b3MinFloat(lower, upper);
	let upperAngle = b3MaxFloat(lower, upper);

	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	base.prismaticJoint.lowerTranslation = lowerAngle;
	base.prismaticJoint.upperTranslation = upperAngle;
}

export function b3PrismaticJoint_GetTranslation(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	let transformA = b3GetBodyTransform(world, base.bodyIdA);
	let transformB = b3GetBodyTransform(world, base.bodyIdB);

	let jointAxis = b3RotateVector(base.localFrameA.q, b3Vec3_axisX);
	jointAxis = b3RotateVector(transformA.q, jointAxis);

	let anchorA = b3RotateVector(transformA.q, base.localFrameA.p);
	let anchorB = b3RotateVector(transformB.q, base.localFrameB.p);
	let d = b3Add(b3SubPos(transformB.p, transformA.p), b3Sub(anchorB, anchorA));
	let translation = b3Dot(d, jointAxis);
	return translation;
}

export function b3PrismaticJoint_EnableSpring(jointId: b3JointId, enableSpring: bool): void {
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	if (enableSpring != base.prismaticJoint.enableSpring) {
		base.prismaticJoint.springImpulse = 0.0;
	}
	base.prismaticJoint.enableSpring = enableSpring;
}

export function b3PrismaticJoint_IsSpringEnabled(jointId: b3JointId): bool {
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	return base.prismaticJoint.enableSpring;
}

export function b3PrismaticJoint_SetTargetTranslation(jointId: b3JointId, targetTranslation: f32): void {
	B3_ASSERT(b3IsValidFloat(targetTranslation));
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	base.prismaticJoint.targetTranslation = targetTranslation;
}

export function b3PrismaticJoint_GetTargetTranslation(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	return base.prismaticJoint.targetTranslation;
}

export function b3PrismaticJoint_SetSpringHertz(jointId: b3JointId, hertz: f32): void {
	B3_ASSERT(b3IsValidFloat(hertz) && hertz >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	base.prismaticJoint.hertz = hertz;
}

export function b3PrismaticJoint_GetSpringHertz(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	return base.prismaticJoint.hertz;
}

export function b3PrismaticJoint_SetSpringDampingRatio(jointId: b3JointId, dampingRatio: f32): void {
	B3_ASSERT(b3IsValidFloat(dampingRatio) && dampingRatio >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	base.prismaticJoint.dampingRatio = dampingRatio;
}

export function b3PrismaticJoint_GetSpringDampingRatio(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	return base.prismaticJoint.dampingRatio;
}

export function b3PrismaticJoint_EnableMotor(jointId: b3JointId, enableMotor: bool): void {
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	if (enableMotor != base.prismaticJoint.enableMotor) {
		base.prismaticJoint.motorImpulse = 0.0;
	}
	base.prismaticJoint.enableMotor = enableMotor;
}

export function b3PrismaticJoint_IsMotorEnabled(jointId: b3JointId): bool {
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	return base.prismaticJoint.enableMotor;
}

export function b3PrismaticJoint_SetMotorSpeed(jointId: b3JointId, motorSpeed: f32): void {
	B3_ASSERT(b3IsValidFloat(motorSpeed));
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	base.prismaticJoint.motorSpeed = motorSpeed;
}

export function b3PrismaticJoint_GetMotorSpeed(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	return base.prismaticJoint.motorSpeed;
}

export function b3PrismaticJoint_SetMaxMotorForce(jointId: b3JointId, maxForce: f32): void {
	B3_ASSERT(b3IsValidFloat(maxForce) && maxForce >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	base.prismaticJoint.maxMotorForce = maxForce;
}

export function b3PrismaticJoint_GetMaxMotorForce(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	return base.prismaticJoint.maxMotorForce;
}

export function b3PrismaticJoint_GetMotorForce(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);
	return world.inv_h * base.prismaticJoint.motorImpulse;
}

export function b3PrismaticJoint_GetSpeed(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let base = b3GetJointSimCheckType(jointId, b3_prismaticJoint);

	let bodySimA = b3GetBodySim(world, base.bodyIdA);
	let bodySimB = b3GetBodySim(world, base.bodyIdB);
	let stateIndexA = b3GetBodyState(world, base.bodyIdA);
	let stateIndexB = b3GetBodyState(world, base.bodyIdB);
	let states = world.solverSets[b3_awakeSet].bodyStates;

	let qA = bodySimA.transform.q;
	let qB = bodySimB.transform.q;

	let axisA = b3RotateVector(qA, b3RotateVector(base.localFrameA.q, b3Vec3_axisX));
	let rA = b3RotateVector(qA, b3Sub(base.localFrameA.p, bodySimA.localCenter));
	let rB = b3RotateVector(qB, b3Sub(base.localFrameB.p, bodySimB.localCenter));

	// Difference the centers in double so the speed stays exact far from the origin.
	let d = b3Add(b3SubPos(bodySimB.center, bodySimA.center), b3Sub(rB, rA));

	let vA = stateIndexA != B3_NULL_INDEX ? states[stateIndexA].linearVelocity : b3Vec3_zero;
	let vB = stateIndexB != B3_NULL_INDEX ? states[stateIndexB].linearVelocity : b3Vec3_zero;
	let wA = stateIndexA != B3_NULL_INDEX ? states[stateIndexA].angularVelocity : b3Vec3_zero;
	let wB = stateIndexB != B3_NULL_INDEX ? states[stateIndexB].angularVelocity : b3Vec3_zero;

	let vRel = b3Sub(b3Add(vB, b3Cross(wB, rB)), b3Add(vA, b3Cross(wA, rA)));

	// The axis moves with body A, so account for its rotation.
	let speed: f32 = b3Dot(d, b3Cross(wA, axisA)) + b3Dot(axisA, vRel);
	return speed;
}

export function b3GetPrismaticJointForce(world: b3World, base: b3JointSim): b3Vec3 {
	let transformA = b3GetBodyTransform(world, base.bodyIdA);
	ref joint = base.prismaticJoint;

	// impulse in joint space
	let impulse = b3Vec3(
		joint.perpImpulse.x,
		joint.perpImpulse.y,
		joint.motorImpulse + joint.lowerImpulse + joint.upperImpulse + joint.springImpulse
	);

	// convert impulse to force
	let force = b3MulSV(world.inv_h, impulse);

	// convert to body space
	force = b3RotateVector(base.localFrameA.q, force);

	// convert to world space
	force = b3RotateVector(transformA.q, force);
	return force;
}

export function b3GetPrismaticJointTorque(world: b3World, base: b3JointSim): b3Vec3 {
	let transformA = b3GetBodyTransform(world, base.bodyIdA);
	ref joint = base.prismaticJoint;

	let torque = b3MulSV(world.inv_h, joint.angularImpulse);
	torque = b3RotateVector(base.localFrameA.q, torque);
	torque = b3RotateVector(transformA.q, torque);
	return torque;
}

@noinline
export function b3PreparePrismaticJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_prismaticJoint);

	let world = context.world;

	let bodyA = world.bodies[base.bodyIdA];
	let bodyB = world.bodies[base.bodyIdB];

	if (B3_ASSERT_ENABLED) B3_ASSERT(bodyA.setIndex == b3_awakeSet || bodyB.setIndex == b3_awakeSet);
	let setA = world.solverSets[bodyA.setIndex];
	let setB = world.solverSets[bodyB.setIndex];

	let localIndexA = bodyA.localIndex;
	let localIndexB = bodyB.localIndex;

	let bodySimA = setA.bodySims[localIndexA];
	let bodySimB = setB.bodySims[localIndexB];

	base.invMassA = bodySimA.invMass;
	base.invMassB = bodySimB.invMass;
	base.invIA = bodySimA.invInertiaWorld;
	base.invIB = bodySimB.invInertiaWorld;

	let invInertiaSum = b3AddMM(base.invIA, base.invIB);
	base.fixedRotation = b3Det(invInertiaSum) < 1000.0 * FLT_MIN;

	ref joint = base.prismaticJoint;
	joint.indexA = bodyA.setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint.indexB = bodyB.setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.q = b3MulQuat(bodySimA.transform.q, base.localFrameA.q);
	joint.frameA.p = b3RotateVector(bodySimA.transform.q, b3Sub(base.localFrameA.p, bodySimA.localCenter));
	joint.frameB.q = b3MulQuat(bodySimB.transform.q, base.localFrameB.q);
	joint.frameB.p = b3RotateVector(bodySimB.transform.q, b3Sub(base.localFrameB.p, bodySimB.localCenter));

	joint.deltaCenter = b3SubPos(bodySimB.center, bodySimA.center);
	joint.rotationMass = b3InvertMatrix(invInertiaSum);

	// Initial joint axes in world space
	let matrixA = b3MakeMatrixFromQuat(joint.frameA.q);
	joint.jointAxis = matrixA.cx;
	joint.perpAxisY = matrixA.cy;
	joint.perpAxisZ = matrixA.cz;

	joint.springSoftness = b3MakeSoft(joint.hertz, joint.dampingRatio, context.h);

	if (context.enableWarmStarting == false) {
		joint.perpImpulse = b3Vec2(0.0, 0.0);
		joint.angularImpulse = b3Vec3(0.0, 0.0, 0.0);
		joint.motorImpulse = 0.0;
		joint.springImpulse = 0.0;
		joint.lowerImpulse = 0.0;
		joint.upperImpulse = 0.0;
	}
}

@noinline
export unsafe function b3WarmStartPrismaticJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_prismaticJoint);

	let mA = base.invMassA;
	let mB = base.invMassB;
	let iA = base.invIA;
	let iB = base.invIB;

	ref joint = base.prismaticJoint;

	// dummy state for static bodies
	let states = context.states;
	let stateA = load<b3BodyState>(b3StateAddress(states, joint.indexA));
	let stateB = load<b3BodyState>(b3StateAddress(states, joint.indexB));

	// todo make this code and the wheel joint more similar

	let rA = b3RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b3RotateVector(stateB.deltaRotation, joint.frameB.p);
	let d = b3Add(b3Add(b3Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), b3Sub(rB, rA));
	let jointAxis = b3RotateVector(stateA.deltaRotation, joint.jointAxis);
	let sAx = b3Cross(b3Add(rA, d), jointAxis);
	let sBx = b3Cross(rB, jointAxis);

	let perpY = b3RotateVector(stateA.deltaRotation, joint.perpAxisY);
	let perpZ = b3RotateVector(stateA.deltaRotation, joint.perpAxisZ);
	let sAy = b3Cross(b3Add(rA, d), perpY);
	let sBy = b3Cross(rB, perpY);
	let sAz = b3Cross(b3Add(rA, d), perpZ);
	let sBz = b3Cross(rB, perpZ);

	let axialImpulse: f32 = joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;
	let perpImpulse = joint.perpImpulse;

	let P = b3Blend3(axialImpulse, jointAxis, perpImpulse.x, perpY, perpImpulse.y, perpZ);
	let LA = b3Add(b3Blend3(axialImpulse, sAx, perpImpulse.x, sAy, perpImpulse.y, sAz), joint.angularImpulse);
	let LB = b3Add(b3Blend3(axialImpulse, sBx, perpImpulse.x, sBy, perpImpulse.y, sBz), joint.angularImpulse);

	let vA = stateA.linearVelocity;
	let wA = stateA.angularVelocity;
	let vB = stateB.linearVelocity;
	let wB = stateB.angularVelocity;
	vA = b3MulSub(vA, mA, P);
	wA = b3Sub(wA, b3MulMV(iA, LA));
	vB = b3MulAdd(vB, mB, P);
	wB = b3Add(wB, b3MulMV(iB, LB));

	if (stateA.flags & b3_dynamicFlag) {
		b3StoreStateVelocities(states, joint.indexA, vA, wA);
	}

	if (stateB.flags & b3_dynamicFlag) {
		b3StoreStateVelocities(states, joint.indexB, vB, wB);
	}
}

@noinline
export unsafe function b3SolvePrismaticJoint(base: b3JointSim, context: b3StepContext, useBias: bool): void {
	let mA = base.invMassA;
	let mB = base.invMassB;
	let iA = base.invIA;
	let iB = base.invIB;

	// dummy state for static bodies
	ref joint = base.prismaticJoint;
	let states = context.states;
	let stateA = load<b3BodyState>(b3StateAddress(states, joint.indexA));
	let stateB = load<b3BodyState>(b3StateAddress(states, joint.indexB));

	let vA = stateA.linearVelocity;
	let wA = stateA.angularVelocity;
	let vB = stateB.linearVelocity;
	let wB = stateB.angularVelocity;

	let fixedRotation = base.fixedRotation;
	let rA = b3RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b3RotateVector(stateB.deltaRotation, joint.frameB.p);

	let dcA = stateA.deltaPosition;
	let dcB = stateB.deltaPosition;
	let d = b3Add(b3Add(b3Sub(dcB, dcA), joint.deltaCenter), b3Sub(rB, rA));

	let jointAxis = b3RotateVector(stateA.deltaRotation, joint.jointAxis);
	let sAx = b3Cross(b3Add(rA, d), jointAxis);
	let sBx = b3Cross(rB, jointAxis);
	let jointTranslation = b3Dot(d, jointAxis);
	let targetTranslation = joint.targetTranslation;

	// The axial effective mass must be fresh to avoid divergence when the joint is stressed
	let ka: f32 = mA + mB + b3Dot(sAx, b3MulMV(iA, sAx)) + b3Dot(sBx, b3MulMV(iB, sBx));
	let axialMass: f32 = ka > 0.0 ? 1.0 / ka : 0.0;

	// Solve spring
	if (joint.enableSpring && fixedRotation == false) {
		// Get the substep relative rotation
		let c: f32 = jointTranslation - targetTranslation;

		let bias: f32 = joint.springSoftness.biasRate * c;
		let massScale = joint.springSoftness.massScale;
		let impulseScale = joint.springSoftness.impulseScale;

		let vRel = b3Sub(b3Sub(b3Add(vB, b3Cross(wB, rB)), vA), b3Cross(wA, b3Add(rA, d)));
		let cdot = b3Dot(vRel, jointAxis);
		let deltaImpulse: f32 = -massScale * axialMass * (cdot + bias) - impulseScale * joint.springImpulse;
		joint.springImpulse += deltaImpulse;

		let P = b3MulSV(deltaImpulse, jointAxis);
		let LA = b3MulSV(deltaImpulse, sAx);
		let LB = b3MulSV(deltaImpulse, sBx);

		vA = b3MulSub(vA, mA, P);
		wA = b3Sub(wA, b3MulMV(iA, LA));
		vB = b3MulAdd(vB, mB, P);
		wB = b3Add(wB, b3MulMV(iB, LB));
	}

	if (joint.enableMotor && fixedRotation == false) {
		let vRel = b3Sub(b3Sub(b3Add(vB, b3Cross(wB, rB)), vA), b3Cross(wA, b3Add(rA, d)));
		let cdot: f32 = b3Dot(vRel, jointAxis) - joint.motorSpeed;

		let deltaImpulse: f32 = -axialMass * cdot;
		let newImpulse: f32 = joint.motorImpulse + deltaImpulse;
		let maxImpulse: f32 = joint.maxMotorForce * context.h;
		newImpulse = b3ClampFloat(newImpulse, -maxImpulse, maxImpulse);
		deltaImpulse = newImpulse - joint.motorImpulse;
		joint.motorImpulse = newImpulse;

		let P = b3MulSV(deltaImpulse, jointAxis);
		let LA = b3MulSV(deltaImpulse, sAx);
		let LB = b3MulSV(deltaImpulse, sBx);

		vA = b3MulSub(vA, mA, P);
		wA = b3Sub(wA, b3MulMV(iA, LA));
		vB = b3MulAdd(vB, mB, P);
		wB = b3Add(wB, b3MulMV(iB, LB));
	}

	if (joint.enableLimit && fixedRotation == false) {
		let speculativeDistance: f32 = 0.25 * (joint.upperTranslation - joint.lowerTranslation);

		// Lower limit
		{
			let C: f32 = jointTranslation - joint.lowerTranslation;

			if (C < speculativeDistance) {
				let bias: f32 = 0.0;
				let massScale: f32 = 1.0;
				let impulseScale: f32 = 0.0;
				if (C > 0.0) {
					// speculation
					bias = C * context.inv_h;
				} else if (useBias) {
					bias = base.constraintSoftness.biasRate * C;
					massScale = base.constraintSoftness.massScale;
					impulseScale = base.constraintSoftness.impulseScale;
				}

				let vRel = b3Sub(b3Sub(b3Add(vB, b3Cross(wB, rB)), vA), b3Cross(wA, b3Add(rA, d)));
				let cdot = b3Dot(vRel, jointAxis);
				let oldImpulse = joint.lowerImpulse;
				let deltaImpulse: f32 = -massScale * axialMass * (cdot + bias) - impulseScale * oldImpulse;
				joint.lowerImpulse = b3MaxFloat(oldImpulse + deltaImpulse, 0.0);
				deltaImpulse = joint.lowerImpulse - oldImpulse;

				let P = b3MulSV(deltaImpulse, jointAxis);
				let LA = b3MulSV(deltaImpulse, sAx);
				let LB = b3MulSV(deltaImpulse, sBx);

				vA = b3MulSub(vA, mA, P);
				wA = b3Sub(wA, b3MulMV(iA, LA));
				vB = b3MulAdd(vB, mB, P);
				wB = b3Add(wB, b3MulMV(iB, LB));
			} else {
				joint.lowerImpulse = 0.0;
			}
		}

		// Upper limit
		{
			let C: f32 = joint.upperTranslation - jointTranslation;

			if (C < speculativeDistance) {
				let bias: f32 = 0.0;
				let massScale: f32 = 1.0;
				let impulseScale: f32 = 0.0;
				if (C > 0.0) {
					// speculation
					bias = C * context.inv_h;
				} else if (useBias) {
					bias = base.constraintSoftness.biasRate * C;
					massScale = base.constraintSoftness.massScale;
					impulseScale = base.constraintSoftness.impulseScale;
				}

				// sign flipped on Cdot
				let vRel = b3Sub(b3Sub(b3Add(vB, b3Cross(wB, rB)), vA), b3Cross(wA, b3Add(rA, d)));
				let cdot: f32 = -b3Dot(vRel, jointAxis);
				let oldImpulse = joint.upperImpulse;
				let deltaImpulse: f32 = -massScale * axialMass * (cdot + bias) - impulseScale * oldImpulse;
				joint.upperImpulse = b3MaxFloat(oldImpulse + deltaImpulse, 0.0);

				// sign flipped on applied impulse
				let negDeltaImpulse: f32 = oldImpulse - joint.upperImpulse;
				let P = b3MulSV(negDeltaImpulse, jointAxis);
				let LA = b3MulSV(negDeltaImpulse, sAx);
				let LB = b3MulSV(negDeltaImpulse, sBx);

				vA = b3MulSub(vA, mA, P);
				wA = b3Sub(wA, b3MulMV(iA, LA));
				vB = b3MulAdd(vB, mB, P);
				wB = b3Add(wB, b3MulMV(iB, LB));
			} else {
				joint.upperImpulse = 0.0;
			}
		}
	}

	// Rotation constraint
	if (fixedRotation == false) {
		let bias = b3Vec3(0.0, 0.0, 0.0);
		let massScale: f32 = 1.0;
		let impulseScale: f32 = 0.0;

		if (useBias) {
			let quatA = b3MulQuat(stateA.deltaRotation, joint.frameA.q);
			let quatB = b3MulQuat(stateB.deltaRotation, joint.frameB.q);

			let relQ = b3InvMulQuat(quatA, quatB);
			let targetQuat = b3Quat_identity;
			let deltaRotation = b3DeltaQuatToRotation(relQ, targetQuat);
			let c = b3Neg(b3RotateVector(quatA, deltaRotation));

			bias = b3MulSV(base.constraintSoftness.biasRate, c);
			massScale = base.constraintSoftness.massScale;
			impulseScale = base.constraintSoftness.impulseScale;
		}

		let cdot = b3Sub(wB, wA);
		let impulse = b3Sub(b3MulSV(-massScale, b3MulMV(joint.rotationMass, b3Add(cdot, bias))), b3MulSV(impulseScale, joint.angularImpulse));
		joint.angularImpulse = b3Add(joint.angularImpulse, impulse);

		wA = b3Sub(wA, b3MulMV(iA, impulse));
		wB = b3Add(wB, b3MulMV(iB, impulse));
	}

	// Solve point-to-line constraint
	{
		let perpY = b3RotateVector(stateA.deltaRotation, joint.perpAxisY);
		let perpZ = b3RotateVector(stateA.deltaRotation, joint.perpAxisZ);

		let bias = b3Vec2(0.0, 0.0);
		let massScale: f32 = 1.0;
		let impulseScale: f32 = 0.0;
		if (useBias) {
			let c = b3Vec2(b3Dot(perpY, d), b3Dot(perpZ, d));
			bias = b3MulSV2(base.constraintSoftness.biasRate, c);
			massScale = base.constraintSoftness.massScale;
			impulseScale = base.constraintSoftness.impulseScale;
		}

		let vRel = b3Sub(b3Sub(b3Add(vB, b3Cross(wB, rB)), vA), b3Cross(wA, b3Add(rA, d)));
		let cdot = b3Vec2(b3Dot(perpY, vRel), b3Dot(perpZ, vRel));

		// K = [(1/mA + 1/mB) * eye(2) - skew(rA) * invIA * skew(rA) - skew(rB) * invIB * skew(rB)]
		// Jx = [-perpX, -cross(d + rA, perpX), perpX, cross(rB, perpX)]
		let sAy = b3Cross(b3Add(rA, d), perpY);
		let sBy = b3Cross(rB, perpY);
		let sAz = b3Cross(b3Add(rA, d), perpZ);
		let sBz = b3Cross(rB, perpZ);

		let kyy: f32 = mA + mB + b3Dot(sAy, b3MulMV(iA, sAy)) + b3Dot(sBy, b3MulMV(iB, sBy));
		let kyz: f32 = b3Dot(sAy, b3MulMV(iA, sAz)) + b3Dot(sBy, b3MulMV(iB, sBz));
		let kzz: f32 = mA + mB + b3Dot(sAz, b3MulMV(iA, sAz)) + b3Dot(sBz, b3MulMV(iB, sBz));

		let K = b3Matrix2(b3Vec2(kyy, kyz), b3Vec2(kyz, kzz));

		let oldImpulse = joint.perpImpulse;
		let sol = b3Solve2(K, b3Add2(cdot, bias));
		let deltaImpulse = b3Sub2(b3MulSV2(-massScale, sol), b3MulSV2(impulseScale, oldImpulse));
		joint.perpImpulse = b3Add2(oldImpulse, deltaImpulse);

		let P = b3Blend2(deltaImpulse.x, perpY, deltaImpulse.y, perpZ);

		vA = b3MulSub(vA, mA, P);
		wA = b3Sub(wA, b3MulMV(iA, b3Blend2(deltaImpulse.x, sAy, deltaImpulse.y, sAz)));
		vB = b3MulAdd(vB, mB, P);
		wB = b3Add(wB, b3MulMV(iB, b3Blend2(deltaImpulse.x, sBy, deltaImpulse.y, sBz)));
	}

	if (B3_ASSERT_ENABLED) {
		B3_ASSERT(b3IsValidVec3(vA));
		B3_ASSERT(b3IsValidVec3(wA));
		B3_ASSERT(b3IsValidVec3(vB));
		B3_ASSERT(b3IsValidVec3(wB));
	}

	if (stateA.flags & b3_dynamicFlag) {
		b3StoreStateVelocities(states, joint.indexA, vA, wA);
	}

	if (stateB.flags & b3_dynamicFlag) {
		b3StoreStateVelocities(states, joint.indexB, vB, wB);
	}
}
