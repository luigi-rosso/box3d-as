// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/distance_joint.c

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX } from "./core";
import { B3_HUGE, B3_LINEAR_SLOP } from "./constants";
import {
	b3Add,
	b3ClampFloat,
	b3Cross,
	b3Dot,
	b3Length,
	b3MaxFloat,
	b3MinFloat,
	b3MulAdd,
	b3MulMV,
	b3MulSV,
	b3MulSub,
	b3Normalize,
	b3RotateVector,
	b3Sub,
	b3SubPos,
	b3TransformWorldPoint,
	b3Vec3,
} from "./math_functions";
import { b3JointId, b3_distanceJoint } from "./types";
import { b3_dynamicFlag, b3BodyState, b3StateAddress, b3StoreStateVelocities } from "./body";
import { b3GetBodyTransform } from "./body";
import { b3GetUnlockedWorld, b3GetWorld, b3World, b3_awakeSet } from "./physics_world";
import { b3MakeSoft, b3StepContext } from "./solver";
import { b3GetJointSimCheckType, b3JointSim } from "./joint";

export function b3DistanceJoint_SetLength(jointId: b3JointId, length: f32): void {
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	ref joint = base.distanceJoint;

	joint.length = b3ClampFloat(length, B3_LINEAR_SLOP(), B3_HUGE());
	joint.impulse = 0.0;
	joint.lowerImpulse = 0.0;
	joint.upperImpulse = 0.0;
}

export function b3DistanceJoint_GetLength(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	ref joint = base.distanceJoint;
	return joint.length;
}

export function b3DistanceJoint_EnableLimit(jointId: b3JointId, enableLimit: bool): void {
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	ref joint = base.distanceJoint;
	joint.enableLimit = enableLimit;
}

export function b3DistanceJoint_IsLimitEnabled(jointId: b3JointId): bool {
	let joint = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	return joint.distanceJoint.enableLimit;
}

export function b3DistanceJoint_SetLengthRange(jointId: b3JointId, minLength: f32, maxLength: f32): void {
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	ref joint = base.distanceJoint;

	minLength = b3ClampFloat(minLength, B3_LINEAR_SLOP(), B3_HUGE());
	maxLength = b3ClampFloat(maxLength, B3_LINEAR_SLOP(), B3_HUGE());
	joint.minLength = b3MinFloat(minLength, maxLength);
	joint.maxLength = b3MaxFloat(minLength, maxLength);
	joint.impulse = 0.0;
	joint.lowerImpulse = 0.0;
	joint.upperImpulse = 0.0;
}

export function b3DistanceJoint_GetMinLength(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	ref joint = base.distanceJoint;
	return joint.minLength;
}

export function b3DistanceJoint_GetMaxLength(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	ref joint = base.distanceJoint;
	return joint.maxLength;
}

export function b3DistanceJoint_GetCurrentLength(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);

	let w = b3GetUnlockedWorld(jointId.world0);
	if (w == null) {
		return 0.0;
	}
	let world = w!;

	let transformA = b3GetBodyTransform(world, base.bodyIdA);
	let transformB = b3GetBodyTransform(world, base.bodyIdB);

	let pA = b3TransformWorldPoint(transformA, base.localFrameA.p);
	let pB = b3TransformWorldPoint(transformB, base.localFrameB.p);
	let d = b3SubPos(pB, pA);
	let length = b3Length(d);
	return length;
}

export function b3DistanceJoint_EnableSpring(jointId: b3JointId, enableSpring: bool): void {
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	base.distanceJoint.enableSpring = enableSpring;
}

export function b3DistanceJoint_IsSpringEnabled(jointId: b3JointId): bool {
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	return base.distanceJoint.enableSpring;
}

export function b3DistanceJoint_SetSpringForceRange(jointId: b3JointId, lowerForce: f32, upperForce: f32): void {
	B3_ASSERT(lowerForce <= upperForce);
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	base.distanceJoint.lowerSpringForce = lowerForce;
	base.distanceJoint.upperSpringForce = upperForce;
}

export function b3DistanceJoint_GetSpringForceRange(jointId: b3JointId, lowerForce: ref f32, upperForce: ref f32): void {
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	lowerForce = base.distanceJoint.lowerSpringForce;
	upperForce = base.distanceJoint.upperSpringForce;
}

export function b3DistanceJoint_SetSpringHertz(jointId: b3JointId, hertz: f32): void {
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	base.distanceJoint.hertz = hertz;
}

export function b3DistanceJoint_SetSpringDampingRatio(jointId: b3JointId, dampingRatio: f32): void {
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	base.distanceJoint.dampingRatio = dampingRatio;
}

export function b3DistanceJoint_GetSpringHertz(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	ref joint = base.distanceJoint;
	return joint.hertz;
}

export function b3DistanceJoint_GetSpringDampingRatio(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	ref joint = base.distanceJoint;
	return joint.dampingRatio;
}

export function b3DistanceJoint_EnableMotor(jointId: b3JointId, enableMotor: bool): void {
	let joint = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	if (enableMotor != joint.distanceJoint.enableMotor) {
		joint.distanceJoint.enableMotor = enableMotor;
		joint.distanceJoint.motorImpulse = 0.0;
	}
}

export function b3DistanceJoint_IsMotorEnabled(jointId: b3JointId): bool {
	let joint = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	return joint.distanceJoint.enableMotor;
}

export function b3DistanceJoint_SetMotorSpeed(jointId: b3JointId, motorSpeed: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	joint.distanceJoint.motorSpeed = motorSpeed;
}

export function b3DistanceJoint_GetMotorSpeed(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	return joint.distanceJoint.motorSpeed;
}

export function b3DistanceJoint_GetMotorForce(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let base = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	return world.inv_h * base.distanceJoint.motorImpulse;
}

export function b3DistanceJoint_SetMaxMotorForce(jointId: b3JointId, force: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	joint.distanceJoint.maxMotorForce = force;
}

export function b3DistanceJoint_GetMaxMotorForce(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_distanceJoint);
	return joint.distanceJoint.maxMotorForce;
}

export function b3GetDistanceJointForce(world: b3World, base: b3JointSim): b3Vec3 {
	ref joint = base.distanceJoint;

	let transformA = b3GetBodyTransform(world, base.bodyIdA);
	let transformB = b3GetBodyTransform(world, base.bodyIdB);

	let pA = b3TransformWorldPoint(transformA, base.localFrameA.p);
	let pB = b3TransformWorldPoint(transformB, base.localFrameB.p);
	let d = b3SubPos(pB, pA);
	let axis = b3Normalize(d);
	let force: f32 = (joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse) * world.inv_h;
	return b3MulSV(force, axis);
}

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

export function b3PrepareDistanceJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_distanceJoint);

	// chase body id to the solver set where the body lives
	let idA = base.bodyIdA;
	let idB = base.bodyIdB;

	let world = context.world;
	let bodyA = world.bodies[idA];
	let bodyB = world.bodies[idB];

	if (B3_ASSERT_ENABLED) B3_ASSERT(bodyA.setIndex == b3_awakeSet || bodyB.setIndex == b3_awakeSet);

	let setA = world.solverSets[bodyA.setIndex];
	let setB = world.solverSets[bodyB.setIndex];

	let localIndexA = bodyA.localIndex;
	let localIndexB = bodyB.localIndex;

	let bodySimA = setA.bodySims[localIndexA];
	let bodySimB = setB.bodySims[localIndexB];

	let mA = bodySimA.invMass;
	let iA = bodySimA.invInertiaWorld;
	let mB = bodySimB.invMass;
	let iB = bodySimB.invInertiaWorld;

	base.invMassA = mA;
	base.invMassB = mB;
	base.invIA = iA;
	base.invIB = iB;

	ref joint = base.distanceJoint;

	joint.indexA = bodyA.setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint.indexB = bodyB.setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// initial anchors in world space
	joint.anchorA = b3RotateVector(bodySimA.transform.q, b3Sub(base.localFrameA.p, bodySimA.localCenter));
	joint.anchorB = b3RotateVector(bodySimB.transform.q, b3Sub(base.localFrameB.p, bodySimB.localCenter));
	joint.deltaCenter = b3SubPos(bodySimB.center, bodySimA.center);

	let rA = joint.anchorA;
	let rB = joint.anchorB;
	let separation = b3Add(b3Sub(rB, rA), joint.deltaCenter);
	let axis = b3Normalize(separation);

	// compute effective mass
	let crA = b3Cross(rA, axis);
	let crB = b3Cross(rB, axis);
	let k: f32 = mA + mB + b3Dot(crA, b3MulMV(iA, crA)) + b3Dot(crB, b3MulMV(iB, crB));
	joint.axialMass = k > 0.0 ? 1.0 / k : 0.0;

	joint.distanceSoftness = b3MakeSoft(joint.hertz, joint.dampingRatio, context.h);

	if (context.enableWarmStarting == false) {
		joint.impulse = 0.0;
		joint.lowerImpulse = 0.0;
		joint.upperImpulse = 0.0;
		joint.motorImpulse = 0.0;
	}
}

export unsafe function b3WarmStartDistanceJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_distanceJoint);

	let mA = base.invMassA;
	let mB = base.invMassB;
	let iA = base.invIA;
	let iB = base.invIB;

	// dummy state for static bodies
	ref joint = base.distanceJoint;
	let states = context.states;
	let stateA = load<b3BodyState>(b3StateAddress(states, joint.indexA));
	let stateB = load<b3BodyState>(b3StateAddress(states, joint.indexB));

	let rA = b3RotateVector(stateA.deltaRotation, joint.anchorA);
	let rB = b3RotateVector(stateB.deltaRotation, joint.anchorB);

	let ds = b3Add(b3Sub(stateB.deltaPosition, stateA.deltaPosition), b3Sub(rB, rA));
	let separation = b3Add(joint.deltaCenter, ds);
	let axis = b3Normalize(separation);

	let axialImpulse: f32 = joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse;
	let P = b3MulSV(axialImpulse, axis);

	// the dummy state has no flags, so these only write awake states
	if (stateA.flags & b3_dynamicFlag) {
		states[joint.indexA].linearVelocity = b3MulSub(stateA.linearVelocity, mA, P);
		states[joint.indexA].angularVelocity = b3Sub(stateA.angularVelocity, b3MulMV(iA, b3Cross(rA, P)));
	}

	if (stateB.flags & b3_dynamicFlag) {
		states[joint.indexB].linearVelocity = b3MulAdd(stateB.linearVelocity, mB, P);
		states[joint.indexB].angularVelocity = b3Add(stateB.angularVelocity, b3MulMV(iB, b3Cross(rB, P)));
	}
}

export unsafe function b3SolveDistanceJoint(base: b3JointSim, context: b3StepContext, useBias: bool): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_distanceJoint);

	let mA = base.invMassA;
	let mB = base.invMassB;
	let iA = base.invIA;
	let iB = base.invIB;

	// dummy state for static bodies
	ref joint = base.distanceJoint;
	let states = context.states;
	let stateA = load<b3BodyState>(b3StateAddress(states, joint.indexA));
	let stateB = load<b3BodyState>(b3StateAddress(states, joint.indexB));

	let vA = stateA.linearVelocity;
	let wA = stateA.angularVelocity;
	let vB = stateB.linearVelocity;
	let wB = stateB.angularVelocity;

	// current anchors
	let rA = b3RotateVector(stateA.deltaRotation, joint.anchorA);
	let rB = b3RotateVector(stateB.deltaRotation, joint.anchorB);

	// current separation
	let ds = b3Add(b3Sub(stateB.deltaPosition, stateA.deltaPosition), b3Sub(rB, rA));
	let separation = b3Add(joint.deltaCenter, ds);

	let length = b3Length(separation);
	let axis = b3Normalize(separation);

	// joint is soft if
	// - spring is enabled
	// - and (joint limit is disabled or limits are not equal)
	if (joint.enableSpring && (joint.minLength < joint.maxLength || joint.enableLimit == false)) {
		// spring
		if (joint.hertz > 0.0) {
			// Cdot = dot(u, v + cross(w, r))
			let vr = b3Add(b3Sub(vB, vA), b3Sub(b3Cross(wB, rB), b3Cross(wA, rA)));
			let Cdot = b3Dot(axis, vr);
			let C: f32 = length - joint.length;
			let bias: f32 = joint.distanceSoftness.biasRate * C;

			let m: f32 = joint.distanceSoftness.massScale * joint.axialMass;
			let oldImpulse = joint.impulse;
			let impulse: f32 = -m * (Cdot + bias) - joint.distanceSoftness.impulseScale * oldImpulse;
			let h = context.h;
			joint.impulse = b3ClampFloat(joint.impulse + impulse, joint.lowerSpringForce * h, joint.upperSpringForce * h);
			impulse = joint.impulse - oldImpulse;

			let P = b3MulSV(impulse, axis);
			vA = b3MulSub(vA, mA, P);
			wA = b3Sub(wA, b3MulMV(iA, b3Cross(rA, P)));
			vB = b3MulAdd(vB, mB, P);
			wB = b3Add(wB, b3MulMV(iB, b3Cross(rB, P)));
		}

		if (joint.enableLimit) {
			// lower limit
			{
				let vr = b3Add(b3Sub(vB, vA), b3Sub(b3Cross(wB, rB), b3Cross(wA, rA)));
				let Cdot = b3Dot(axis, vr);

				let C: f32 = length - joint.minLength;

				let bias: f32 = 0.0;
				let massCoeff: f32 = 1.0;
				let impulseCoeff: f32 = 0.0;
				if (C > 0.0) {
					// speculative
					bias = C * context.inv_h;
				} else if (useBias) {
					bias = base.constraintSoftness.biasRate * C;
					massCoeff = base.constraintSoftness.massScale;
					impulseCoeff = base.constraintSoftness.impulseScale;
				}

				let impulse: f32 = -massCoeff * joint.axialMass * (Cdot + bias) - impulseCoeff * joint.lowerImpulse;
				let newImpulse = b3MaxFloat(0.0, joint.lowerImpulse + impulse);
				impulse = newImpulse - joint.lowerImpulse;
				joint.lowerImpulse = newImpulse;

				let P = b3MulSV(impulse, axis);
				vA = b3MulSub(vA, mA, P);
				wA = b3Sub(wA, b3MulMV(iA, b3Cross(rA, P)));
				vB = b3MulAdd(vB, mB, P);
				wB = b3Add(wB, b3MulMV(iB, b3Cross(rB, P)));
			}

			// upper
			{
				let vr = b3Add(b3Sub(vA, vB), b3Sub(b3Cross(wA, rA), b3Cross(wB, rB)));
				let Cdot = b3Dot(axis, vr);

				let C: f32 = joint.maxLength - length;

				let bias: f32 = 0.0;
				let massScale: f32 = 1.0;
				let impulseScale: f32 = 0.0;
				if (C > 0.0) {
					// speculative
					bias = C * context.inv_h;
				} else if (useBias) {
					bias = base.constraintSoftness.biasRate * C;
					massScale = base.constraintSoftness.massScale;
					impulseScale = base.constraintSoftness.impulseScale;
				}

				let impulse: f32 = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.upperImpulse;
				let newImpulse = b3MaxFloat(0.0, joint.upperImpulse + impulse);
				impulse = newImpulse - joint.upperImpulse;
				joint.upperImpulse = newImpulse;

				let P = b3MulSV(-impulse, axis);
				vA = b3MulSub(vA, mA, P);
				wA = b3Sub(wA, b3MulMV(iA, b3Cross(rA, P)));
				vB = b3MulAdd(vB, mB, P);
				wB = b3Add(wB, b3MulMV(iB, b3Cross(rB, P)));
			}
		}

		if (joint.enableMotor) {
			let vr = b3Add(b3Sub(vB, vA), b3Sub(b3Cross(wB, rB), b3Cross(wA, rA)));
			let Cdot = b3Dot(axis, vr);
			let impulse: f32 = joint.axialMass * (joint.motorSpeed - Cdot);
			let oldImpulse = joint.motorImpulse;
			let maxImpulse: f32 = context.h * joint.maxMotorForce;
			joint.motorImpulse = b3ClampFloat(joint.motorImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = joint.motorImpulse - oldImpulse;

			let P = b3MulSV(impulse, axis);
			vA = b3MulSub(vA, mA, P);
			wA = b3Sub(wA, b3MulMV(iA, b3Cross(rA, P)));
			vB = b3MulAdd(vB, mB, P);
			wB = b3Add(wB, b3MulMV(iB, b3Cross(rB, P)));
		}
	} else {
		// rigid constraint
		let vr = b3Add(b3Sub(vB, vA), b3Sub(b3Cross(wB, rB), b3Cross(wA, rA)));
		let Cdot = b3Dot(axis, vr);

		let C: f32 = length - joint.length;

		let bias: f32 = 0.0;
		let massScale: f32 = 1.0;
		let impulseScale: f32 = 0.0;
		if (useBias) {
			bias = base.constraintSoftness.biasRate * C;
			massScale = base.constraintSoftness.massScale;
			impulseScale = base.constraintSoftness.impulseScale;
		}

		let impulse: f32 = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.impulse;
		joint.impulse += impulse;

		let P = b3MulSV(impulse, axis);
		vA = b3MulSub(vA, mA, P);
		wA = b3Sub(wA, b3MulMV(iA, b3Cross(rA, P)));
		vB = b3MulAdd(vB, mB, P);
		wB = b3Add(wB, b3MulMV(iB, b3Cross(rB, P)));
	}

	if (stateA.flags & b3_dynamicFlag) {
		b3StoreStateVelocities(states, joint.indexA, vA, wA);
	}

	if (stateB.flags & b3_dynamicFlag) {
		b3StoreStateVelocities(states, joint.indexB, vB, wB);
	}
}
