// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/parallel_joint.c

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX, FLT_MIN } from "./core";
import {
	b3Add,
	b3AddMM,
	b3Blend2,
	b3Cross,
	b3Det,
	b3Dot,
	b3DotQuat,
	b3InvMulQuat,
	b3IsValidFloat,
	b3MulMV,
	b3MulQuat,
	b3MulSV,
	b3NegateQuat,
	b3RotateVector,
	b3Sub,
	b3Vec2,
	b3Vec3,
	b3Vec3_axisX,
	b3Vec3_axisY,
} from "./math_functions";
import { b3Length2, b3LengthSquared2, b3Matrix2, b3Solve2, b3Sub2 } from "./math_internal";
import { b3JointId, b3_parallelJoint } from "./types";
import { b3_dynamicFlag, b3_identityBodyState } from "./body";
import { b3World, b3_awakeSet } from "./physics_world";
import { b3MakeSoft, b3StepContext } from "./solver";
import { b3GetJointSimCheckType, b3JointSim } from "./joint";

export function b3ParallelJoint_SetSpringHertz(jointId: b3JointId, hertz: f32): void {
	B3_ASSERT(b3IsValidFloat(hertz) && hertz >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_parallelJoint);
	base.parallelJoint.hertz = hertz;
}

export function b3ParallelJoint_GetSpringHertz(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_parallelJoint);
	return base.parallelJoint.hertz;
}

export function b3ParallelJoint_SetSpringDampingRatio(jointId: b3JointId, dampingRatio: f32): void {
	B3_ASSERT(b3IsValidFloat(dampingRatio) && dampingRatio >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_parallelJoint);
	base.parallelJoint.dampingRatio = dampingRatio;
}

export function b3ParallelJoint_GetSpringDampingRatio(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_parallelJoint);
	return base.parallelJoint.dampingRatio;
}

export function b3ParallelJoint_SetMaxTorque(jointId: b3JointId, maxForce: f32): void {
	B3_ASSERT(b3IsValidFloat(maxForce) && maxForce >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_parallelJoint);
	base.parallelJoint.maxTorque = maxForce;
}

export function b3ParallelJoint_GetMaxTorque(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_parallelJoint);
	return base.parallelJoint.maxTorque;
}

export function b3GetParallelJointTorque(world: b3World, base: b3JointSim): b3Vec3 {
	ref joint = base.parallelJoint;

	let relQ = b3InvMulQuat(joint.quatA, joint.quatB);
	joint.perpAxisX = b3MulSV(0.5, b3RotateVector(joint.quatA, b3Add(b3MulSV(relQ.s, b3Vec3_axisX), b3Cross(relQ.v, b3Vec3_axisX))));
	joint.perpAxisY = b3MulSV(0.5, b3RotateVector(joint.quatA, b3Add(b3MulSV(relQ.s, b3Vec3_axisY), b3Cross(relQ.v, b3Vec3_axisY))));

	let angularImpulse = b3Blend2(joint.perpImpulse.x, joint.perpAxisX, joint.perpImpulse.y, joint.perpAxisY);
	let torque = b3MulSV(world.inv_h, angularImpulse);
	return torque;
}

export function b3PrepareParallelJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_parallelJoint);

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

	ref joint = base.parallelJoint;
	joint.indexA = bodyA.setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint.indexB = bodyB.setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.quatA = b3MulQuat(bodySimA.transform.q, base.localFrameA.q);
	joint.quatB = b3MulQuat(bodySimB.transform.q, base.localFrameB.q);

	let relQ = b3InvMulQuat(joint.quatA, joint.quatB);

	{
		// These are needed for warm starting
		joint.perpAxisX = b3MulSV(0.5, b3RotateVector(joint.quatA, b3Add(b3MulSV(relQ.s, b3Vec3_axisX), b3Cross(relQ.v, b3Vec3_axisX))));
		joint.perpAxisY = b3MulSV(0.5, b3RotateVector(joint.quatA, b3Add(b3MulSV(relQ.s, b3Vec3_axisY), b3Cross(relQ.v, b3Vec3_axisY))));
	}

	joint.softness = b3MakeSoft(joint.hertz, joint.dampingRatio, context.h);

	if (context.enableWarmStarting == false) {
		joint.perpImpulse = b3Vec2(0.0, 0.0);
	}
}

export function b3WarmStartParallelJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_parallelJoint);

	let iA = base.invIA;
	let iB = base.invIB;

	ref joint = base.parallelJoint;

	// dummy state for static bodies
	let states = context.states;
	let stateA = joint.indexA == B3_NULL_INDEX ? b3_identityBodyState : states[joint.indexA];
	let stateB = joint.indexB == B3_NULL_INDEX ? b3_identityBodyState : states[joint.indexB];

	let wA = stateA.angularVelocity;
	let wB = stateB.angularVelocity;

	let angularImpulse = b3Blend2(joint.perpImpulse.x, joint.perpAxisX, joint.perpImpulse.y, joint.perpAxisY);

	wA = b3Sub(wA, b3MulMV(iA, angularImpulse));
	wB = b3Add(wB, b3MulMV(iB, angularImpulse));

	if (stateA.flags & b3_dynamicFlag) {
		states[joint.indexA].angularVelocity = wA;
	}

	if (stateB.flags & b3_dynamicFlag) {
		states[joint.indexB].angularVelocity = wB;
	}
}

export function b3SolveParallelJoint(base: b3JointSim, context: b3StepContext): void {
	let iA = base.invIA;
	let iB = base.invIB;

	// dummy state for static bodies
	ref joint = base.parallelJoint;
	let states = context.states;
	let stateA = joint.indexA == B3_NULL_INDEX ? b3_identityBodyState : states[joint.indexA];
	let stateB = joint.indexB == B3_NULL_INDEX ? b3_identityBodyState : states[joint.indexB];

	let wA = stateA.angularVelocity;
	let wB = stateB.angularVelocity;

	let fixedRotation = base.fixedRotation;
	let quatA = b3MulQuat(stateA.deltaRotation, joint.quatA);
	let quatB = b3MulQuat(stateB.deltaRotation, joint.quatB);

	if (b3DotQuat(quatA, quatB) < 0.0) {
		// this keeps the rotation angle in the range [-pi, pi]
		quatB = b3NegateQuat(quatB);
	}

	let relQ = b3InvMulQuat(quatA, quatB);

	if (fixedRotation == false && joint.maxTorque > 0.0) {
		let c = b3Vec2(relQ.v.x, relQ.v.y);
		let bias = b3Vec2(joint.softness.biasRate * c.x, joint.softness.biasRate * c.y);
		let massScale = joint.softness.massScale;
		let impulseScale = joint.softness.impulseScale;

		// Collinearity constraint as 2-by-2
		let perpAxisX = b3MulSV(0.5, b3RotateVector(quatA, b3Add(b3MulSV(relQ.s, b3Vec3_axisX), b3Cross(relQ.v, b3Vec3_axisX))));
		let perpAxisY = b3MulSV(0.5, b3RotateVector(quatA, b3Add(b3MulSV(relQ.s, b3Vec3_axisY), b3Cross(relQ.v, b3Vec3_axisY))));
		joint.perpAxisX = perpAxisX;
		joint.perpAxisY = perpAxisY;

		let invInertiaSum = b3AddMM(iA, iB);
		let kxx = b3Dot(perpAxisX, b3MulMV(invInertiaSum, perpAxisX));
		let kyy = b3Dot(perpAxisY, b3MulMV(invInertiaSum, perpAxisY));
		let kxy = b3Dot(perpAxisX, b3MulMV(invInertiaSum, perpAxisY));

		let k = b3Matrix2(b3Vec2(kxx, kxy), b3Vec2(kxy, kyy));

		let wRel = b3Sub(wB, wA);
		let cdot = b3Vec2(b3Dot(wRel, perpAxisX), b3Dot(wRel, perpAxisY));

		let maxImpulse: f32 = context.h * joint.maxTorque;
		let oldImpulse = joint.perpImpulse;
		let cdotPlusBias = b3Vec2(cdot.x + bias.x, cdot.y + bias.y);
		let sol = b3Solve2(k, cdotPlusBias);
		let deltaImpulse = b3Vec2(
			-massScale * sol.x - impulseScale * oldImpulse.x,
			-massScale * sol.y - impulseScale * oldImpulse.y
		);
		joint.perpImpulse = b3Vec2(oldImpulse.x + deltaImpulse.x, oldImpulse.y + deltaImpulse.y);
		if (b3LengthSquared2(joint.perpImpulse) > maxImpulse * maxImpulse) {
			let s: f32 = maxImpulse / b3Length2(joint.perpImpulse);
			joint.perpImpulse = b3Vec2(s * joint.perpImpulse.x, s * joint.perpImpulse.y);
		}

		deltaImpulse = b3Sub2(joint.perpImpulse, oldImpulse);

		let angularImpulse = b3Blend2(deltaImpulse.x, perpAxisX, deltaImpulse.y, perpAxisY);
		wA = b3Sub(wA, b3MulMV(iA, angularImpulse));
		wB = b3Add(wB, b3MulMV(iB, angularImpulse));
	}

	if (stateA.flags & b3_dynamicFlag) {
		states[joint.indexA].angularVelocity = wA;
	}

	if (stateB.flags & b3_dynamicFlag) {
		states[joint.indexB].angularVelocity = wB;
	}
}
