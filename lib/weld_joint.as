// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/weld_joint.c

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX, FLT_MIN } from "./core";
import {
	b3Add,
	b3AddMM,
	b3Cross,
	b3Det,
	b3DotQuat,
	b3InvMulQuat,
	b3InvertMatrix,
	b3IsValidFloat,
	b3MulAdd,
	b3MulMV,
	b3MulQuat,
	b3MulSV,
	b3MulSub,
	b3Neg,
	b3NegateQuat,
	b3Quat_identity,
	b3RotateVector,
	b3Sub,
	b3SubPos,
	b3Vec3,
	b3Vec3_zero,
} from "./math_functions";
import { b3DeltaQuatToRotation } from "./math_internal";
import { b3JointId, b3_weldJoint } from "./types";
import { b3_dynamicFlag, b3BodyState, b3StateAddress, b3StoreStateVelocities } from "./body";
import { b3World, b3_awakeSet } from "./physics_world";
import { b3MakeSoft, b3StepContext } from "./solver";
import { b3GetJointSimCheckType, b3JointSim, b3SolvePointConstraint } from "./joint";

export function b3WeldJoint_SetLinearHertz(jointId: b3JointId, hertz: f32): void {
	B3_ASSERT(b3IsValidFloat(hertz) && hertz >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_weldJoint);
	base.weldJoint.linearHertz = hertz;
}

export function b3WeldJoint_GetLinearHertz(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_weldJoint);
	return base.weldJoint.linearHertz;
}

export function b3WeldJoint_SetLinearDampingRatio(jointId: b3JointId, dampingRatio: f32): void {
	B3_ASSERT(b3IsValidFloat(dampingRatio) && dampingRatio >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_weldJoint);
	base.weldJoint.linearDampingRatio = dampingRatio;
}

export function b3WeldJoint_GetLinearDampingRatio(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_weldJoint);
	return base.weldJoint.linearDampingRatio;
}

export function b3WeldJoint_SetAngularHertz(jointId: b3JointId, hertz: f32): void {
	B3_ASSERT(b3IsValidFloat(hertz) && hertz >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_weldJoint);
	base.weldJoint.angularHertz = hertz;
}

export function b3WeldJoint_GetAngularHertz(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_weldJoint);
	return base.weldJoint.angularHertz;
}

export function b3WeldJoint_SetAngularDampingRatio(jointId: b3JointId, dampingRatio: f32): void {
	B3_ASSERT(b3IsValidFloat(dampingRatio) && dampingRatio >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_weldJoint);
	base.weldJoint.angularDampingRatio = dampingRatio;
}

export function b3WeldJoint_GetAngularDampingRatio(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_weldJoint);
	return base.weldJoint.angularDampingRatio;
}

export function b3GetWeldJointForce(world: b3World, base: b3JointSim): b3Vec3 {
	let force = b3MulSV(world.inv_h, base.weldJoint.linearImpulse);
	return force;
}

export function b3GetWeldJointTorque(world: b3World, base: b3JointSim): b3Vec3 {
	return b3MulSV(world.inv_h, base.weldJoint.angularImpulse);
}

@noinline
export function b3PrepareWeldJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_weldJoint);

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

	ref joint = base.weldJoint;
	joint.indexA = bodyA.setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint.indexB = bodyB.setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.q = b3MulQuat(bodySimA.transform.q, base.localFrameA.q);
	joint.frameA.p = b3RotateVector(bodySimA.transform.q, b3Sub(base.localFrameA.p, bodySimA.localCenter));
	joint.frameB.q = b3MulQuat(bodySimB.transform.q, base.localFrameB.q);
	joint.frameB.p = b3RotateVector(bodySimB.transform.q, b3Sub(base.localFrameB.p, bodySimB.localCenter));

	joint.deltaCenter = b3SubPos(bodySimB.center, bodySimA.center);
	joint.angularMass = b3InvertMatrix(invInertiaSum);

	if (joint.linearHertz == 0.0) {
		joint.linearSpring = base.constraintSoftness;
	} else {
		joint.linearSpring = b3MakeSoft(joint.linearHertz, joint.linearDampingRatio, context.h);
	}

	if (joint.angularHertz == 0.0) {
		joint.angularSpring = base.constraintSoftness;
	} else {
		joint.angularSpring = b3MakeSoft(joint.angularHertz, joint.angularDampingRatio, context.h);
	}

	if (context.enableWarmStarting == false) {
		joint.linearImpulse = b3Vec3_zero;
		joint.angularImpulse = b3Vec3_zero;
	}
}

@noinline
export unsafe function b3WarmStartWeldJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_weldJoint);

	let mA = base.invMassA;
	let mB = base.invMassB;
	let iA = base.invIA;
	let iB = base.invIB;

	ref joint = base.weldJoint;

	// dummy state for static bodies
	let states = context.states;
	let stateA = load<b3BodyState>(b3StateAddress(states, joint.indexA));
	let stateB = load<b3BodyState>(b3StateAddress(states, joint.indexB));

	let vA = stateA.linearVelocity;
	let wA = stateA.angularVelocity;
	let vB = stateB.linearVelocity;
	let wB = stateB.angularVelocity;

	let rA = b3RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b3RotateVector(stateB.deltaRotation, joint.frameB.p);

	vA = b3MulSub(vA, mA, joint.linearImpulse);
	wA = b3Sub(wA, b3MulMV(iA, b3Add(b3Cross(rA, joint.linearImpulse), joint.angularImpulse)));

	vB = b3MulAdd(vB, mB, joint.linearImpulse);
	wB = b3Add(wB, b3MulMV(iB, b3Add(b3Cross(rB, joint.linearImpulse), joint.angularImpulse)));

	if (stateA.flags & b3_dynamicFlag) {
		b3StoreStateVelocities(states, joint.indexA, vA, wA);
	}

	if (stateB.flags & b3_dynamicFlag) {
		b3StoreStateVelocities(states, joint.indexB, vB, wB);
	}
}

@noinline
export unsafe function b3SolveWeldJoint(base: b3JointSim, context: b3StepContext, useBias: bool): void {
	let mA = base.invMassA;
	let mB = base.invMassB;
	let iA = base.invIA;
	let iB = base.invIB;

	// dummy state for static bodies
	ref joint = base.weldJoint;
	let states = context.states;
	let stateA = load<b3BodyState>(b3StateAddress(states, joint.indexA));
	let stateB = load<b3BodyState>(b3StateAddress(states, joint.indexB));

	let vA = stateA.linearVelocity;
	let wA = stateA.angularVelocity;
	let vB = stateB.linearVelocity;
	let wB = stateB.angularVelocity;

	let fixedRotation = base.fixedRotation;
	let quatA = b3MulQuat(stateA.deltaRotation, joint.frameA.q);
	let quatB = b3MulQuat(stateB.deltaRotation, joint.frameB.q);

	if (b3DotQuat(quatA, quatB) < 0.0) {
		// this keeps the rotation angle in the range [-pi, pi]
		quatB = b3NegateQuat(quatB);
	}

	let relQ = b3InvMulQuat(quatA, quatB);

	// angular constraint
	if (fixedRotation == false) {
		let bias = b3Vec3_zero;
		let massScale: f32 = 1.0;
		let impulseScale: f32 = 0.0;
		if (useBias || joint.angularHertz > 0.0) {
			let targetQuat = b3Quat_identity;
			let deltaRotation = b3DeltaQuatToRotation(relQ, targetQuat);
			let c = b3Neg(b3RotateVector(quatA, deltaRotation));

			bias = b3MulSV(joint.angularSpring.biasRate, c);
			massScale = joint.angularSpring.massScale;
			impulseScale = joint.angularSpring.impulseScale;
		}

		let cdot = b3Sub(wB, wA);
		let impulse = b3MulSub(b3MulSV(-massScale, b3MulMV(joint.angularMass, b3Add(cdot, bias))), impulseScale, joint.angularImpulse);
		joint.angularImpulse = b3Add(joint.angularImpulse, impulse);

		wA = b3Sub(wA, b3MulMV(iA, impulse));
		wB = b3Add(wB, b3MulMV(iB, impulse));
	}

	// linear constraint
	{
		let rA = b3RotateVector(stateA.deltaRotation, joint.frameA.p);
		let rB = b3RotateVector(stateB.deltaRotation, joint.frameB.p);

		let cdot = b3Sub(b3Add(vB, b3Cross(wB, rB)), b3Add(vA, b3Cross(wA, rA)));

		let bias = b3Vec3_zero;
		let massScale: f32 = 1.0;
		let impulseScale: f32 = 0.0;
		if (useBias || joint.linearHertz > 0.0) {
			let dcA = stateA.deltaPosition;
			let dcB = stateB.deltaPosition;

			let separation = b3Add(b3Add(b3Sub(dcB, dcA), b3Sub(rB, rA)), joint.deltaCenter);

			bias = b3MulSV(joint.linearSpring.biasRate, separation);
			massScale = joint.linearSpring.massScale;
			impulseScale = joint.linearSpring.impulseScale;
		}

		let b = b3SolvePointConstraint(mA, mB, base.invIA, base.invIB, rA, rB, b3Add(cdot, bias));

		let impulse = b3MulSub(b3MulSV(-massScale, b), impulseScale, joint.linearImpulse);
		joint.linearImpulse = b3Add(joint.linearImpulse, impulse);

		vA = b3MulSub(vA, mA, impulse);
		wA = b3Sub(wA, b3MulMV(iA, b3Cross(rA, impulse)));
		vB = b3MulAdd(vB, mB, impulse);
		wB = b3Add(wB, b3MulMV(iB, b3Cross(rB, impulse)));
	}

	if (stateA.flags & b3_dynamicFlag) {
		b3StoreStateVelocities(states, joint.indexA, vA, wA);
	}

	if (stateB.flags & b3_dynamicFlag) {
		b3StoreStateVelocities(states, joint.indexB, vB, wB);
	}
}
