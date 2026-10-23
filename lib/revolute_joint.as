// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/revolute_joint.c

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX, FLT_MIN } from "./core";
import {
	B3_PI,
	b3Add,
	b3AddMM,
	b3ClampFloat,
	b3Cross,
	b3Det,
	b3Dot,
	b3DotQuat,
	b3GetTwistAngle,
	b3InvMulQuat,
	b3IsValidFloat,
	b3MaxFloat,
	b3MinFloat,
	b3MulAdd,
	b3MulMV,
	b3MulQuat,
	b3MulSV,
	b3MulSub,
	b3NegateQuat,
	b3RotateVector,
	b3Sub,
	b3SubPos,
	b3Vec2,
	b3Vec3,
	b3Vec3_axisX,
	b3Vec3_axisY,
	b3Vec3_axisZ,
	b3Vec3_zero,
} from "./math_functions";
import { b3Add2, b3Matrix2, b3MulSV2, b3Solve2, b3Sub2 } from "./math_internal";
import { b3JointId, b3_revoluteJoint } from "./types";
import { b3GetBodyTransform, b3_dynamicFlag, b3_identityBodyState } from "./body";
import { b3GetWorld, b3World, b3_awakeSet } from "./physics_world";
import { b3MakeSoft, b3StepContext } from "./solver";
import { b3GetJointSimCheckType, b3JointSim, b3SolvePointConstraint } from "./joint";

// Point-to-point linear constraint
// C = pB - pA
// Cdot = vB - vA
//      = vB + cross(wB, rB) - vA - cross(wA, rA)
// Cdot = J * v
// J = [-E -skew(rA) E skew(rB) ]
//
// K = J * invM * JT
//   = [(1/mA + 1/mB) * E - skew(rA) * invIA * skew(rA) - skew(rB) * invIB * skew(rB)]

// Perpendicularity constraint
// frameA = qA * localFrameA
// frameB = qB * localFrameB
// qRel = conj(frameA) * frameB
// C = [qRel.x; qRel.y]
// qRelDot = 0.5 * conj(frameA) * (wB - wA) * frameB
// Cdot = [qRelDot.x, qRelDot.y]
// Pulling out wB and wA
// sr = qRel.s
// vr = qRel.v
// Jx = 0.5 * rotate(frameA, sr * ex + cross(vr, ex))
// Jy = 0.5 * rotate(frameA, sr * ey + cross(vr, ey))

// Motor constraint
// Cdot = wB - wA
// J = [0 0 -E 0 0 E]
// K = invIA + invIB

export function b3RevoluteJoint_EnableLimit(jointId: b3JointId, enableLimit: bool): void {
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	if (enableLimit != base.revoluteJoint.enableLimit) {
		base.revoluteJoint.lowerImpulse = 0.0;
		base.revoluteJoint.upperImpulse = 0.0;
	}
	base.revoluteJoint.enableLimit = enableLimit;
}

export function b3RevoluteJoint_IsLimitEnabled(jointId: b3JointId): bool {
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	return base.revoluteJoint.enableLimit;
}

export function b3RevoluteJoint_GetLowerLimit(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	return base.revoluteJoint.lowerAngle;
}

export function b3RevoluteJoint_GetUpperLimit(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	return base.revoluteJoint.upperAngle;
}

export function b3RevoluteJoint_SetLimits(jointId: b3JointId, lowerLimitRadians: f32, upperLimitRadians: f32): void {
	B3_ASSERT(b3IsValidFloat(lowerLimitRadians) && b3IsValidFloat(upperLimitRadians));

	let lowerAngle = b3MinFloat(lowerLimitRadians, upperLimitRadians);
	let upperAngle = b3MaxFloat(lowerLimitRadians, upperLimitRadians);

	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	base.revoluteJoint.lowerAngle = b3ClampFloat(lowerAngle, -0.99 * B3_PI, 0.99 * B3_PI);
	base.revoluteJoint.upperAngle = b3ClampFloat(upperAngle, -0.99 * B3_PI, 0.99 * B3_PI);
}

export function b3RevoluteJoint_GetAngle(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	let transformA = b3GetBodyTransform(world, base.bodyIdA);
	let transformB = b3GetBodyTransform(world, base.bodyIdB);

	let quatA = b3MulQuat(transformA.q, base.localFrameA.q);
	let quatB = b3MulQuat(transformB.q, base.localFrameB.q);

	if (b3DotQuat(quatA, quatB) < 0.0) {
		// this keeps the twist angle in the range [-pi, pi]
		quatB = b3NegateQuat(quatB);
	}

	let relQ = b3InvMulQuat(quatA, quatB);

	return b3GetTwistAngle(relQ);
}

export function b3RevoluteJoint_EnableSpring(jointId: b3JointId, enableSpring: bool): void {
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	if (enableSpring != base.revoluteJoint.enableSpring) {
		base.revoluteJoint.springImpulse = 0.0;
	}
	base.revoluteJoint.enableSpring = enableSpring;
}

export function b3RevoluteJoint_IsSpringEnabled(jointId: b3JointId): bool {
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	return base.revoluteJoint.enableSpring;
}

export function b3RevoluteJoint_SetTargetAngle(jointId: b3JointId, targetRadians: f32): void {
	B3_ASSERT(b3IsValidFloat(targetRadians) && -B3_PI <= targetRadians && targetRadians <= B3_PI);
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	base.revoluteJoint.targetAngle = targetRadians;
}

export function b3RevoluteJoint_GetTargetAngle(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	return base.revoluteJoint.targetAngle;
}

export function b3RevoluteJoint_SetSpringHertz(jointId: b3JointId, hertz: f32): void {
	B3_ASSERT(b3IsValidFloat(hertz) && hertz >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	base.revoluteJoint.hertz = hertz;
}

export function b3RevoluteJoint_GetSpringHertz(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	return base.revoluteJoint.hertz;
}

export function b3RevoluteJoint_SetSpringDampingRatio(jointId: b3JointId, dampingRatio: f32): void {
	B3_ASSERT(b3IsValidFloat(dampingRatio) && dampingRatio >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	base.revoluteJoint.dampingRatio = dampingRatio;
}

export function b3RevoluteJoint_GetSpringDampingRatio(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	return base.revoluteJoint.dampingRatio;
}

export function b3RevoluteJoint_EnableMotor(jointId: b3JointId, enableMotor: bool): void {
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	if (enableMotor != base.revoluteJoint.enableMotor) {
		base.revoluteJoint.motorImpulse = 0.0;
	}
	base.revoluteJoint.enableMotor = enableMotor;
}

export function b3RevoluteJoint_IsMotorEnabled(jointId: b3JointId): bool {
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	return base.revoluteJoint.enableMotor;
}

export function b3RevoluteJoint_SetMotorSpeed(jointId: b3JointId, motorSpeed: f32): void {
	B3_ASSERT(b3IsValidFloat(motorSpeed));
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	base.revoluteJoint.motorSpeed = motorSpeed;
}

export function b3RevoluteJoint_GetMotorSpeed(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	return base.revoluteJoint.motorSpeed;
}

export function b3RevoluteJoint_SetMaxMotorTorque(jointId: b3JointId, maxForce: f32): void {
	B3_ASSERT(b3IsValidFloat(maxForce) && maxForce >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	base.revoluteJoint.maxMotorTorque = maxForce;
}

export function b3RevoluteJoint_GetMaxMotorTorque(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	return base.revoluteJoint.maxMotorTorque;
}

export function b3RevoluteJoint_GetMotorTorque(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let base = b3GetJointSimCheckType(jointId, b3_revoluteJoint);
	return world.inv_h * base.revoluteJoint.motorImpulse;
}

export function b3GetRevoluteJointForce(world: b3World, base: b3JointSim): b3Vec3 {
	let force = b3MulSV(world.inv_h, base.revoluteJoint.linearImpulse);
	return force;
}

export function b3GetRevoluteJointTorque(world: b3World, base: b3JointSim): b3Vec3 {
	let transformA = b3GetBodyTransform(world, base.bodyIdA);
	ref joint = base.revoluteJoint;
	let axis = b3RotateVector(base.localFrameA.q, b3Vec3_axisZ);
	axis = b3RotateVector(transformA.q, axis);

	let relQ = b3InvMulQuat(joint.frameA.q, joint.frameB.q);

	// These are needed for warm starting
	joint.perpAxisX = b3MulSV(0.5, b3RotateVector(joint.frameA.q, b3Add(b3MulSV(relQ.s, b3Vec3_axisX), b3Cross(relQ.v, b3Vec3_axisX))));
	joint.perpAxisY = b3MulSV(0.5, b3RotateVector(joint.frameA.q, b3Add(b3MulSV(relQ.s, b3Vec3_axisY), b3Cross(relQ.v, b3Vec3_axisY))));

	let axialImpulse: f32 = joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;
	let angularImpulse = b3Add(b3MulSV(joint.perpImpulse.x, joint.perpAxisX), b3MulSV(joint.perpImpulse.y, joint.perpAxisY));
	angularImpulse = b3MulAdd(angularImpulse, axialImpulse, joint.rotationAxisZ);

	// todo add pivot torque
	let impulse = b3MulAdd(angularImpulse, joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse, axis);
	let torque = b3MulSV(world.inv_h, impulse);
	return torque;
}

export function b3PrepareRevoluteJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_revoluteJoint);

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

	ref joint = base.revoluteJoint;
	joint.indexA = bodyA.setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint.indexB = bodyB.setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	// Avoid round-off here as much as possible.
	// b3Vec3 pf = (xf.p - c) + rot(xf.q, f.p)
	// pf = xf.p - (xf.p + rot(xf.q, lc)) + rot(xf.q, f.p)
	// pf = rot(xf.q, f.p - lc)
	joint.frameA.q = b3MulQuat(bodySimA.transform.q, base.localFrameA.q);
	joint.frameA.p = b3RotateVector(bodySimA.transform.q, b3Sub(base.localFrameA.p, bodySimA.localCenter));
	joint.frameB.q = b3MulQuat(bodySimB.transform.q, base.localFrameB.q);
	joint.frameB.p = b3RotateVector(bodySimB.transform.q, b3Sub(base.localFrameB.p, bodySimB.localCenter));

	joint.deltaCenter = b3SubPos(bodySimB.center, bodySimA.center);

	{
		// Rotation axis is the z-axis of body A.
		let rotationAxisZ = b3RotateVector(joint.frameA.q, b3Vec3_axisZ);
		let k = b3Dot(rotationAxisZ, b3MulMV(invInertiaSum, rotationAxisZ));
		joint.axialMass = k > 0.0 ? 1.0 / k : 0.0;
		joint.rotationAxisZ = rotationAxisZ;
	}

	let relQ = b3InvMulQuat(joint.frameA.q, joint.frameB.q);

	{
		// These are needed for warm starting
		joint.perpAxisX = b3MulSV(0.5, b3RotateVector(joint.frameA.q, b3Add(b3MulSV(relQ.s, b3Vec3_axisX), b3Cross(relQ.v, b3Vec3_axisX))));
		joint.perpAxisY = b3MulSV(0.5, b3RotateVector(joint.frameA.q, b3Add(b3MulSV(relQ.s, b3Vec3_axisY), b3Cross(relQ.v, b3Vec3_axisY))));
	}

	joint.springSoftness = b3MakeSoft(joint.hertz, joint.dampingRatio, context.h);

	if (context.enableWarmStarting == false) {
		joint.linearImpulse = b3Vec3_zero;
		joint.perpImpulse = b3Vec2(0.0, 0.0);
		joint.motorImpulse = 0.0;
		joint.springImpulse = 0.0;
		joint.lowerImpulse = 0.0;
		joint.upperImpulse = 0.0;
	}
}

export function b3WarmStartRevoluteJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_revoluteJoint);

	let mA = base.invMassA;
	let mB = base.invMassB;
	let iA = base.invIA;
	let iB = base.invIB;

	ref joint = base.revoluteJoint;

	// dummy state for static bodies
	let states = context.states;
	let stateA = joint.indexA == B3_NULL_INDEX ? b3_identityBodyState : states[joint.indexA];
	let stateB = joint.indexB == B3_NULL_INDEX ? b3_identityBodyState : states[joint.indexB];

	let vA = stateA.linearVelocity;
	let wA = stateA.angularVelocity;
	let vB = stateB.linearVelocity;
	let wB = stateB.angularVelocity;

	let rA = b3RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b3RotateVector(stateB.deltaRotation, joint.frameB.p);

	let axialImpulse: f32 = joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;
	let angularImpulse = b3Add(b3MulSV(joint.perpImpulse.x, joint.perpAxisX), b3MulSV(joint.perpImpulse.y, joint.perpAxisY));
	angularImpulse = b3MulAdd(angularImpulse, axialImpulse, joint.rotationAxisZ);

	vA = b3MulSub(vA, mA, joint.linearImpulse);
	wA = b3Sub(wA, b3MulMV(iA, b3Add(b3Cross(rA, joint.linearImpulse), angularImpulse)));

	vB = b3MulAdd(vB, mB, joint.linearImpulse);
	wB = b3Add(wB, b3MulMV(iB, b3Add(b3Cross(rB, joint.linearImpulse), angularImpulse)));

	if (stateA.flags & b3_dynamicFlag) {
		states[joint.indexA].linearVelocity = vA;
		states[joint.indexA].angularVelocity = wA;
	}

	if (stateB.flags & b3_dynamicFlag) {
		states[joint.indexB].linearVelocity = vB;
		states[joint.indexB].angularVelocity = wB;
	}
}

export function b3SolveRevoluteJoint(base: b3JointSim, context: b3StepContext, useBias: bool): void {
	let mA = base.invMassA;
	let mB = base.invMassB;
	let iA = base.invIA;
	let iB = base.invIB;

	// dummy state for static bodies
	ref joint = base.revoluteJoint;
	let states = context.states;
	let stateA = joint.indexA == B3_NULL_INDEX ? b3_identityBodyState : states[joint.indexA];
	let stateB = joint.indexB == B3_NULL_INDEX ? b3_identityBodyState : states[joint.indexB];

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

	if (fixedRotation == false && (joint.enableSpring || joint.enableMotor || joint.enableLimit)) {
		let axis = joint.rotationAxisZ;
		let gA = b3MulMV(iA, axis);
		let gB = b3MulMV(iB, axis);
		let k = b3Dot(axis, b3Add(gA, gB));
		let axialCdot = b3Dot(b3Sub(wB, wA), axis);
		let axialImpulse: f32 = 0.0;

		// Solve spring
		if (joint.enableSpring) {
			// Get the substep relative rotation
			let targetAngle = joint.targetAngle;
			let angle = b3GetTwistAngle(relQ);
			let c: f32 = angle - targetAngle;

			let bias: f32 = joint.springSoftness.biasRate * c;
			let massScale = joint.springSoftness.massScale;
			let impulseScale = joint.springSoftness.impulseScale;

			let deltaImpulse: f32 = -massScale * joint.axialMass * (axialCdot + bias) - impulseScale * joint.springImpulse;
			joint.springImpulse += deltaImpulse;

			axialCdot += k * deltaImpulse;
			axialImpulse += deltaImpulse;
		}

		if (joint.enableMotor) {
			let cdot: f32 = axialCdot - joint.motorSpeed;
			let oldImpulse = joint.motorImpulse;
			let maxImpulse: f32 = joint.maxMotorTorque * context.h;
			let deltaImpulse = b3ClampFloat(-joint.axialMass * cdot, -maxImpulse - oldImpulse, maxImpulse - oldImpulse);
			joint.motorImpulse = oldImpulse + deltaImpulse;

			axialCdot += k * deltaImpulse;
			axialImpulse += deltaImpulse;
		}

		if (joint.enableLimit) {
			let angle = b3GetTwistAngle(relQ);

			// todo does an updated twist axis help?

			// Lower limit
			{
				let c: f32 = angle - joint.lowerAngle;
				let bias: f32 = 0.0;
				let massScale: f32 = 1.0;
				let impulseScale: f32 = 0.0;
				if (c > 0.0) {
					// speculation
					bias = c * context.inv_h;
				} else if (useBias) {
					bias = base.constraintSoftness.biasRate * c;
					massScale = base.constraintSoftness.massScale;
					impulseScale = base.constraintSoftness.impulseScale;
				}

				let oldImpulse = joint.lowerImpulse;
				let deltaImpulse = b3MaxFloat(-massScale * joint.axialMass * (axialCdot + bias) - impulseScale * oldImpulse, -oldImpulse);
				joint.lowerImpulse = oldImpulse + deltaImpulse;

				axialCdot += k * deltaImpulse;
				axialImpulse += deltaImpulse;
			}

			// Upper limit
			{
				let c: f32 = joint.upperAngle - angle;
				let bias: f32 = 0.0;
				let massScale: f32 = 1.0;
				let impulseScale: f32 = 0.0;
				if (c > 0.0) {
					// speculation
					bias = c * context.inv_h;
				} else if (useBias) {
					bias = base.constraintSoftness.biasRate * c;
					massScale = base.constraintSoftness.massScale;
					impulseScale = base.constraintSoftness.impulseScale;
				}

				// sign flipped on Cdot
				let cdot: f32 = -axialCdot;
				let oldImpulse = joint.upperImpulse;
				let deltaImpulse = b3MaxFloat(-massScale * joint.axialMass * (cdot + bias) - impulseScale * oldImpulse, -oldImpulse);
				joint.upperImpulse = oldImpulse + deltaImpulse;

				// sign flipped on applied impulse
				axialImpulse -= deltaImpulse;
			}
		}

		wA = b3MulSub(wA, axialImpulse, gA);
		wB = b3MulAdd(wB, axialImpulse, gB);
	}

	// Collinearity constraint
	if (fixedRotation == false) {
		let bias = b3Vec2(0.0, 0.0);
		let massScale: f32 = 1.0;
		let impulseScale: f32 = 0.0;

		if (useBias) {
			let c = b3Vec2(relQ.v.x, relQ.v.y);
			bias = b3MulSV2(base.constraintSoftness.biasRate, c);
			massScale = base.constraintSoftness.massScale;
			impulseScale = base.constraintSoftness.impulseScale;
		}

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
		let oldImpulse = joint.perpImpulse;
		let sol = b3Solve2(k, b3Add2(cdot, bias));
		let deltaImpulse = b3Sub2(b3MulSV2(-massScale, sol), b3MulSV2(impulseScale, oldImpulse));
		joint.perpImpulse = b3Add2(joint.perpImpulse, deltaImpulse);

		let angularImpulse = b3Add(b3MulSV(deltaImpulse.x, perpAxisX), b3MulSV(deltaImpulse.y, perpAxisY));
		wA = b3Sub(wA, b3MulMV(iA, angularImpulse));
		wB = b3Add(wB, b3MulMV(iB, angularImpulse));
	}

	// Solve point-to-point constraint
	{
		let rA = b3RotateVector(stateA.deltaRotation, joint.frameA.p);
		let rB = b3RotateVector(stateB.deltaRotation, joint.frameB.p);

		let cdot = b3Sub(b3Sub(b3Add(vB, b3Cross(wB, rB)), vA), b3Cross(wA, rA));

		let bias = b3Vec3_zero;
		let massScale: f32 = 1.0;
		let impulseScale: f32 = 0.0;
		if (useBias) {
			let dcA = stateA.deltaPosition;
			let dcB = stateB.deltaPosition;

			let separation = b3Add(b3Add(b3Sub(dcB, dcA), b3Sub(rB, rA)), joint.deltaCenter);

			bias = b3MulSV(base.constraintSoftness.biasRate, separation);
			massScale = base.constraintSoftness.massScale;
			impulseScale = base.constraintSoftness.impulseScale;
		}

		let b = b3SolvePointConstraint(mA, mB, base.invIA, base.invIB, rA, rB, b3Add(cdot, bias));

		let impulse = b3Sub(b3MulSV(-massScale, b), b3MulSV(impulseScale, joint.linearImpulse));
		joint.linearImpulse = b3Add(joint.linearImpulse, impulse);

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
