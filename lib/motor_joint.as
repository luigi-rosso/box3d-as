// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/motor_joint.c

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX, FLT_MIN } from "./core";
import {
	b3Add,
	b3AddMM,
	b3Cross,
	b3Det,
	b3DotQuat,
	b3InvMulQuat,
	b3InvertMatrix,
	b3LengthSquared,
	b3MaxFloat,
	b3MulAdd,
	b3MulMV,
	b3MulQuat,
	b3MulSV,
	b3MulSub,
	b3Neg,
	b3NegateQuat,
	b3Normalize,
	b3Quat_identity,
	b3RotateVector,
	b3Sub,
	b3SubPos,
	b3Vec3,
	b3Vec3_zero,
} from "./math_functions";
import { b3DeltaQuatToRotation } from "./math_internal";
import { b3JointId, b3_motorJoint } from "./types";
import { b3_dynamicFlag, b3_identityBodyState } from "./body";
import { b3World, b3_awakeSet } from "./physics_world";
import { b3MakeSoft, b3StepContext } from "./solver";
import { b3GetJointSimCheckType, b3JointSim, b3SolvePointConstraint } from "./joint";

export function b3MotorJoint_SetLinearVelocity(jointId: b3JointId, velocity: b3Vec3): void {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	joint.motorJoint.linearVelocity = velocity;
}

export function b3MotorJoint_GetLinearVelocity(jointId: b3JointId): b3Vec3 {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	return joint.motorJoint.linearVelocity;
}

export function b3MotorJoint_SetAngularVelocity(jointId: b3JointId, velocity: b3Vec3): void {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	joint.motorJoint.angularVelocity = velocity;
}

export function b3MotorJoint_GetAngularVelocity(jointId: b3JointId): b3Vec3 {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	return joint.motorJoint.angularVelocity;
}

export function b3MotorJoint_SetMaxVelocityTorque(jointId: b3JointId, maxTorque: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	joint.motorJoint.maxVelocityTorque = maxTorque;
}

export function b3MotorJoint_GetMaxVelocityTorque(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	return joint.motorJoint.maxVelocityTorque;
}

export function b3MotorJoint_SetMaxVelocityForce(jointId: b3JointId, maxForce: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	joint.motorJoint.maxVelocityForce = maxForce;
}

export function b3MotorJoint_GetMaxVelocityForce(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	return joint.motorJoint.maxVelocityForce;
}

export function b3MotorJoint_SetLinearHertz(jointId: b3JointId, hertz: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	joint.motorJoint.linearHertz = hertz;
}

export function b3MotorJoint_GetLinearHertz(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	return joint.motorJoint.linearHertz;
}

export function b3MotorJoint_SetLinearDampingRatio(jointId: b3JointId, damping: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	joint.motorJoint.linearDampingRatio = damping;
}

export function b3MotorJoint_GetLinearDampingRatio(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	return joint.motorJoint.linearDampingRatio;
}

export function b3MotorJoint_SetAngularHertz(jointId: b3JointId, hertz: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	joint.motorJoint.angularHertz = hertz;
}

export function b3MotorJoint_GetAngularHertz(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	return joint.motorJoint.angularHertz;
}

export function b3MotorJoint_SetAngularDampingRatio(jointId: b3JointId, damping: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	joint.motorJoint.angularDampingRatio = damping;
}

export function b3MotorJoint_GetAngularDampingRatio(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	return joint.motorJoint.angularDampingRatio;
}

export function b3MotorJoint_SetMaxSpringForce(jointId: b3JointId, maxForce: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	joint.motorJoint.maxSpringForce = b3MaxFloat(0.0, maxForce);
}

export function b3MotorJoint_GetMaxSpringForce(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	return joint.motorJoint.maxSpringForce;
}

export function b3MotorJoint_SetMaxSpringTorque(jointId: b3JointId, maxTorque: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	joint.motorJoint.maxSpringTorque = b3MaxFloat(0.0, maxTorque);
}

export function b3MotorJoint_GetMaxSpringTorque(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_motorJoint);
	return joint.motorJoint.maxSpringTorque;
}

export function b3GetMotorJointForce(world: b3World, base: b3JointSim): b3Vec3 {
	let force = b3MulSV(world.inv_h, b3Add(base.motorJoint.linearVelocityImpulse, base.motorJoint.linearSpringImpulse));
	return force;
}

export function b3GetMotorJointTorque(world: b3World, base: b3JointSim): b3Vec3 {
	return b3MulSV(world.inv_h, b3Add(base.motorJoint.angularVelocityImpulse, base.motorJoint.angularSpringImpulse));
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

export function b3PrepareMotorJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_motorJoint);

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

	base.invMassA = bodySimA.invMass;
	base.invMassB = bodySimB.invMass;
	base.invIA = bodySimA.invInertiaWorld;
	base.invIB = bodySimB.invInertiaWorld;

	let invInertiaSum = b3AddMM(base.invIA, base.invIB);
	base.fixedRotation = b3Det(invInertiaSum) < 1000.0 * FLT_MIN;

	ref joint = base.motorJoint;
	joint.indexA = bodyA.setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint.indexB = bodyB.setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.q = b3MulQuat(bodySimA.transform.q, base.localFrameA.q);
	joint.frameA.p = b3RotateVector(bodySimA.transform.q, b3Sub(base.localFrameA.p, bodySimA.localCenter));
	joint.frameB.q = b3MulQuat(bodySimB.transform.q, base.localFrameB.q);
	joint.frameB.p = b3RotateVector(bodySimB.transform.q, b3Sub(base.localFrameB.p, bodySimB.localCenter));

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint.deltaCenter = b3SubPos(bodySimB.center, bodySimA.center);

	joint.linearSpring = b3MakeSoft(joint.linearHertz, joint.linearDampingRatio, context.h);
	joint.angularSpring = b3MakeSoft(joint.angularHertz, joint.angularDampingRatio, context.h);

	joint.angularMass = b3InvertMatrix(invInertiaSum);

	if (context.enableWarmStarting == false) {
		joint.linearVelocityImpulse = b3Vec3_zero;
		joint.angularVelocityImpulse = b3Vec3_zero;
		joint.linearSpringImpulse = b3Vec3_zero;
		joint.angularSpringImpulse = b3Vec3_zero;
	}
}

export function b3WarmStartMotorJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_motorJoint);

	let mA = base.invMassA;
	let mB = base.invMassB;
	let iA = base.invIA;
	let iB = base.invIB;

	ref joint = base.motorJoint;

	// dummy state for static bodies
	let states = context.states;
	let stateA = joint.indexA == B3_NULL_INDEX ? b3_identityBodyState : states[joint.indexA];
	let stateB = joint.indexB == B3_NULL_INDEX ? b3_identityBodyState : states[joint.indexB];

	let rA = b3RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b3RotateVector(stateB.deltaRotation, joint.frameB.p);

	let linearImpulse = b3Add(joint.linearVelocityImpulse, joint.linearSpringImpulse);
	let angularImpulse = b3Add(joint.angularVelocityImpulse, joint.angularSpringImpulse);

	// C writes these without a flags test; its writes to the dummy state are discarded
	if (joint.indexA != B3_NULL_INDEX) {
		states[joint.indexA].linearVelocity = b3MulSub(stateA.linearVelocity, mA, linearImpulse);
		states[joint.indexA].angularVelocity = b3Sub(stateA.angularVelocity, b3MulMV(iA, b3Add(b3Cross(rA, linearImpulse), angularImpulse)));
	}

	if (joint.indexB != B3_NULL_INDEX) {
		states[joint.indexB].linearVelocity = b3MulAdd(stateB.linearVelocity, mB, linearImpulse);
		states[joint.indexB].angularVelocity = b3Add(stateB.angularVelocity, b3MulMV(iB, b3Add(b3Cross(rB, linearImpulse), angularImpulse)));
	}
}

export function b3SolveMotorJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_motorJoint);

	let mA = base.invMassA;
	let mB = base.invMassB;
	let iA = base.invIA;
	let iB = base.invIB;

	// dummy state for static bodies
	ref joint = base.motorJoint;
	let states = context.states;
	let stateA = joint.indexA == B3_NULL_INDEX ? b3_identityBodyState : states[joint.indexA];
	let stateB = joint.indexB == B3_NULL_INDEX ? b3_identityBodyState : states[joint.indexB];

	let vA = stateA.linearVelocity;
	let wA = stateA.angularVelocity;
	let vB = stateB.linearVelocity;
	let wB = stateB.angularVelocity;

	let quatA = b3MulQuat(stateA.deltaRotation, joint.frameA.q);
	let quatB = b3MulQuat(stateB.deltaRotation, joint.frameB.q);

	if (b3DotQuat(quatA, quatB) < 0.0) {
		// this keeps the rotation angle in the range [-pi, pi]
		quatB = b3NegateQuat(quatB);
	}

	let relQ = b3InvMulQuat(quatA, quatB);

	// angular spring
	if (joint.maxSpringTorque > 0.0 && joint.angularHertz > 0.0) {
		let targetQuat = b3Quat_identity;
		let deltaRotation = b3DeltaQuatToRotation(relQ, targetQuat);
		let c = b3Neg(b3RotateVector(quatA, deltaRotation));

		let bias = b3MulSV(joint.angularSpring.biasRate, c);
		let massScale = joint.angularSpring.massScale;
		let impulseScale = joint.angularSpring.impulseScale;

		let cdot = b3Sub(wB, wA);

		let maxImpulse: f32 = context.h * joint.maxSpringTorque;
		let oldImpulse = joint.angularSpringImpulse;
		let impulse = b3MulSub(b3MulSV(-massScale, b3MulMV(joint.angularMass, b3Add(cdot, bias))), impulseScale, oldImpulse);
		joint.angularSpringImpulse = b3Add(oldImpulse, impulse);
		if (b3LengthSquared(joint.angularSpringImpulse) > maxImpulse * maxImpulse) {
			joint.angularSpringImpulse = b3MulSV(maxImpulse, b3Normalize(joint.angularSpringImpulse));
		}
		impulse = b3Sub(joint.angularSpringImpulse, oldImpulse);

		wA = b3Sub(wA, b3MulMV(iA, impulse));
		wB = b3Add(wB, b3MulMV(iB, impulse));
	}

	// angular velocity
	if (joint.maxVelocityTorque > 0.0) {
		let cdot = b3Sub(b3Sub(wB, wA), joint.angularVelocity);
		let impulse = b3Neg(b3MulMV(joint.angularMass, cdot));

		let maxImpulse: f32 = context.h * joint.maxVelocityTorque;
		let oldImpulse = joint.angularVelocityImpulse;
		joint.angularVelocityImpulse = b3Add(oldImpulse, impulse);
		if (b3LengthSquared(joint.angularVelocityImpulse) > maxImpulse * maxImpulse) {
			joint.angularVelocityImpulse = b3MulSV(maxImpulse, b3Normalize(joint.angularVelocityImpulse));
		}
		impulse = b3Sub(joint.angularVelocityImpulse, oldImpulse);

		wA = b3Sub(wA, b3MulMV(iA, impulse));
		wB = b3Add(wB, b3MulMV(iB, impulse));
	}

	let rA = b3RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b3RotateVector(stateB.deltaRotation, joint.frameB.p);

	// linear spring
	if (joint.maxSpringForce > 0.0 && joint.linearHertz > 0.0) {
		let dcA = stateA.deltaPosition;
		let dcB = stateB.deltaPosition;
		let c = b3Add(b3Add(b3Sub(dcB, dcA), b3Sub(rB, rA)), joint.deltaCenter);

		let bias = b3MulSV(joint.linearSpring.biasRate, c);
		let massScale = joint.linearSpring.massScale;
		let impulseScale = joint.linearSpring.impulseScale;

		let cdot = b3Sub(b3Add(vB, b3Cross(wB, rB)), b3Add(vA, b3Cross(wA, rA)));

		let b = b3SolvePointConstraint(mA, mB, base.invIA, base.invIB, rA, rB, b3Add(cdot, bias));

		let oldImpulse = joint.linearSpringImpulse;
		let impulse = b3MulSub(b3MulSV(-massScale, b), impulseScale, oldImpulse);
		let maxImpulse: f32 = context.h * joint.maxSpringForce;
		joint.linearSpringImpulse = b3Add(joint.linearSpringImpulse, impulse);

		if (b3LengthSquared(joint.linearSpringImpulse) > maxImpulse * maxImpulse) {
			joint.linearSpringImpulse = b3MulSV(maxImpulse, b3Normalize(joint.linearSpringImpulse));
		}

		impulse = b3Sub(joint.linearSpringImpulse, oldImpulse);

		vA = b3MulSub(vA, mA, impulse);
		wA = b3Sub(wA, b3MulMV(iA, b3Cross(rA, impulse)));
		vB = b3MulAdd(vB, mB, impulse);
		wB = b3Add(wB, b3MulMV(iB, b3Cross(rB, impulse)));
	}

	// linear velocity
	if (joint.maxVelocityForce > 0.0) {
		let cdot = b3Sub(b3Add(vB, b3Cross(wB, rB)), b3Add(vA, b3Cross(wA, rA)));
		cdot = b3Sub(cdot, joint.linearVelocity);

		let b = b3SolvePointConstraint(mA, mB, base.invIA, base.invIB, rA, rB, cdot);
		let impulse = b3Neg(b);

		let oldImpulse = joint.linearVelocityImpulse;
		let maxImpulse: f32 = context.h * joint.maxVelocityForce;
		joint.linearVelocityImpulse = b3Add(joint.linearVelocityImpulse, impulse);

		if (b3LengthSquared(joint.linearVelocityImpulse) > maxImpulse * maxImpulse) {
			joint.linearVelocityImpulse = b3MulSV(maxImpulse, b3Normalize(joint.linearVelocityImpulse));
		}

		impulse = b3Sub(joint.linearVelocityImpulse, oldImpulse);

		vA = b3MulSub(vA, mA, impulse);
		wA = b3Sub(wA, b3MulMV(iA, b3Cross(rA, impulse)));
		vB = b3MulAdd(vB, mB, impulse);
		wB = b3Add(wB, b3MulMV(iB, b3Cross(rB, impulse)));
	}

	if (stateA.flags & b3_dynamicFlag) {
		states[joint.indexA].linearVelocity = vA;
		states[joint.indexA].angularVelocity = wA;
	}

	if (stateB.flags & b3_dynamicFlag) {
		states[joint.indexB].linearVelocity = vB;
		states[joint.indexB].angularVelocity = wB;
	}
}
