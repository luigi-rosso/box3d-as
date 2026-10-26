// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/spherical_joint.c

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
	b3GetSwingAngle,
	b3GetTwistAngle,
	b3InvMulQuat,
	b3InvertMatrix,
	b3IsValidFloat,
	b3IsValidQuat,
	b3IsValidVec3,
	b3Length,
	b3Mat3_zero,
	b3MaxFloat,
	b3MinFloat,
	b3MulAdd,
	b3MulMV,
	b3MulQuat,
	b3MulSV,
	b3MulSub,
	b3Neg,
	b3NegateQuat,
	b3Normalize,
	b3Quat,
	b3RotateVector,
	b3Sub,
	b3SubPos,
	b3Vec3,
	b3Vec3_axisZ,
	b3Vec3_zero,
} from "./math_functions";
import { b3DeltaQuatToRotation } from "./math_internal";
import { b3JointId, b3_sphericalJoint } from "./types";
import { b3GetBodyTransform, b3_dynamicFlag, b3BodyState, b3StateAddress, b3StoreStateVelocities } from "./body";
import { b3GetWorld, b3World, b3_awakeSet } from "./physics_world";
import { b3MakeSoft, b3StepContext } from "./solver";
import { b3GetJointSimCheckType, b3JointSim, b3SolvePointConstraint } from "./joint";

export function b3SphericalJoint_EnableConeLimit(jointId: b3JointId, enableLimit: bool): void {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	if (enableLimit != base.sphericalJoint.enableConeLimit) {
		base.sphericalJoint.swingImpulse = 0.0;
	}
	base.sphericalJoint.enableConeLimit = enableLimit;
}

export function b3SphericalJoint_IsConeLimitEnabled(jointId: b3JointId): bool {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	return base.sphericalJoint.enableConeLimit;
}

export function b3SphericalJoint_GetConeLimit(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	return base.sphericalJoint.coneAngle;
}

export function b3SphericalJoint_SetConeLimit(jointId: b3JointId, angleRadians: f32): void {
	B3_ASSERT(b3IsValidFloat(angleRadians) && 0 <= angleRadians && angleRadians <= 0.5 * B3_PI);
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	base.sphericalJoint.coneAngle = angleRadians;
}

export function b3SphericalJoint_GetConeAngle(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	let transformA = b3GetBodyTransform(world, base.bodyIdA);
	let transformB = b3GetBodyTransform(world, base.bodyIdB);

	let quatA = b3MulQuat(transformA.q, base.localFrameA.q);
	let quatB = b3MulQuat(transformB.q, base.localFrameB.q);

	if (b3DotQuat(quatA, quatB) < 0.0) {
		// this keeps the swing angle in the range [0, pi]
		quatB = b3NegateQuat(quatB);
	}

	let relQ = b3InvMulQuat(quatA, quatB);

	return b3GetSwingAngle(relQ);
}

export function b3SphericalJoint_EnableTwistLimit(jointId: b3JointId, enableLimit: bool): void {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	if (enableLimit != base.sphericalJoint.enableTwistLimit) {
		base.sphericalJoint.lowerTwistImpulse = 0.0;
		base.sphericalJoint.upperTwistImpulse = 0.0;
	}
	base.sphericalJoint.enableTwistLimit = enableLimit;
}

export function b3SphericalJoint_IsTwistLimitEnabled(jointId: b3JointId): bool {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	return base.sphericalJoint.enableTwistLimit;
}

export function b3SphericalJoint_GetLowerTwistLimit(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	return base.sphericalJoint.lowerTwistAngle;
}

export function b3SphericalJoint_GetUpperTwistLimit(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	return base.sphericalJoint.upperTwistAngle;
}

export function b3SphericalJoint_SetTwistLimits(jointId: b3JointId, lowerLimitRadians: f32, upperLimitRadians: f32): void {
	B3_ASSERT(b3IsValidFloat(lowerLimitRadians) && b3IsValidFloat(upperLimitRadians));

	let lowerAngle = b3MinFloat(lowerLimitRadians, upperLimitRadians);
	let upperAngle = b3MaxFloat(lowerLimitRadians, upperLimitRadians);

	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	base.sphericalJoint.lowerTwistAngle = b3ClampFloat(lowerAngle, -0.99 * B3_PI, 0.99 * B3_PI);
	base.sphericalJoint.upperTwistAngle = b3ClampFloat(upperAngle, -0.99 * B3_PI, 0.99 * B3_PI);
}

export function b3SphericalJoint_GetTwistAngle(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
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

export function b3SphericalJoint_EnableSpring(jointId: b3JointId, enableSpring: bool): void {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	if (enableSpring != base.sphericalJoint.enableSpring) {
		base.sphericalJoint.springImpulse = b3Vec3_zero;
	}
	base.sphericalJoint.enableSpring = enableSpring;
}

export function b3SphericalJoint_IsSpringEnabled(jointId: b3JointId): bool {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	return base.sphericalJoint.enableSpring;
}

export function b3SphericalJoint_SetTargetRotation(jointId: b3JointId, targetRotation: b3Quat): void {
	B3_ASSERT(b3IsValidQuat(targetRotation));
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	base.sphericalJoint.targetRotation = targetRotation;
}

export function b3SphericalJoint_GetTargetRotation(jointId: b3JointId): b3Quat {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	return base.sphericalJoint.targetRotation;
}

export function b3SphericalJoint_SetSpringHertz(jointId: b3JointId, hertz: f32): void {
	B3_ASSERT(b3IsValidFloat(hertz) && hertz >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	base.sphericalJoint.hertz = hertz;
}

export function b3SphericalJoint_GetSpringHertz(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	return base.sphericalJoint.hertz;
}

export function b3SphericalJoint_SetSpringDampingRatio(jointId: b3JointId, dampingRatio: f32): void {
	B3_ASSERT(b3IsValidFloat(dampingRatio) && dampingRatio >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	base.sphericalJoint.dampingRatio = dampingRatio;
}

export function b3SphericalJoint_GetSpringDampingRatio(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	return base.sphericalJoint.dampingRatio;
}

export function b3SphericalJoint_EnableMotor(jointId: b3JointId, enableMotor: bool): void {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	if (enableMotor != base.sphericalJoint.enableMotor) {
		base.sphericalJoint.motorImpulse = b3Vec3_zero;
	}
	base.sphericalJoint.enableMotor = enableMotor;
}

export function b3SphericalJoint_IsMotorEnabled(jointId: b3JointId): bool {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	return base.sphericalJoint.enableMotor;
}

export function b3SphericalJoint_SetMotorVelocity(jointId: b3JointId, motorVelocity: b3Vec3): void {
	B3_ASSERT(b3IsValidVec3(motorVelocity));
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	base.sphericalJoint.motorVelocity = motorVelocity;
}

export function b3SphericalJoint_GetMotorVelocity(jointId: b3JointId): b3Vec3 {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	return base.sphericalJoint.motorVelocity;
}

export function b3SphericalJoint_SetMaxMotorTorque(jointId: b3JointId, maxForce: f32): void {
	B3_ASSERT(b3IsValidFloat(maxForce) && maxForce >= 0.0);
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	base.sphericalJoint.maxMotorTorque = maxForce;
}

export function b3SphericalJoint_GetMaxMotorTorque(jointId: b3JointId): f32 {
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	return base.sphericalJoint.maxMotorTorque;
}

export function b3SphericalJoint_GetMotorTorque(jointId: b3JointId): b3Vec3 {
	let world = b3GetWorld(jointId.world0);
	let base = b3GetJointSimCheckType(jointId, b3_sphericalJoint);
	return b3MulSV(world.inv_h, base.sphericalJoint.motorImpulse);
}

export function b3GetSphericalJointForce(world: b3World, base: b3JointSim): b3Vec3 {
	let force = b3MulSV(world.inv_h, base.sphericalJoint.linearImpulse);
	return force;
}

export function b3GetSphericalJointTorque(world: b3World, base: b3JointSim): b3Vec3 {
	let xfA = b3GetBodyTransform(world, base.bodyIdA);
	let xfB = b3GetBodyTransform(world, base.bodyIdB);
	let qA = b3MulQuat(xfA.q, base.localFrameA.q);
	let qB = b3MulQuat(xfB.q, base.localFrameB.q);

	// Cone axis is the z-axis of body A.
	let coneAxis = b3RotateVector(qA, b3Vec3_axisZ);
	let twistAxis = b3RotateVector(qB, b3Vec3_axisZ);
	let swingAxis = b3Normalize(b3Cross(coneAxis, twistAxis));

	ref joint = base.sphericalJoint;
	let impulse = b3Add(joint.springImpulse, joint.motorImpulse);
	impulse = b3MulAdd(impulse, joint.lowerTwistImpulse - joint.upperTwistImpulse, twistAxis);
	impulse = b3MulAdd(impulse, joint.swingImpulse, swingAxis);
	let torque = b3MulSV(world.inv_h, impulse);
	return torque;
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I r1_skew I -r2_skew ]
// K = J * invM * transpose(J)
// transpose(skew(r)) = -skew(r)
// K = diag(1/m1 + 1/m2) - r1_skew * invI1 * r1_skew - r2_skew * invI2 * r2_skew

// r_skew = R * skew(r_local) * RT
// invI = R * invI_local * RT
// r_skew * invI * r_skew = R * skew(r_local) * RT * R * invI_local * RT * R * r_skew * RT
//                        = R * ( skew(r_local) * invI_local * skew(r_local) ) * RT

@noinline
export function b3PrepareSphericalJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_sphericalJoint);

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

	ref joint = base.sphericalJoint;
	joint.indexA = bodyA.setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint.indexB = bodyB.setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.q = b3MulQuat(bodySimA.transform.q, base.localFrameA.q);
	joint.frameA.p = b3RotateVector(bodySimA.transform.q, b3Sub(base.localFrameA.p, bodySimA.localCenter));
	joint.frameB.q = b3MulQuat(bodySimB.transform.q, base.localFrameB.q);
	joint.frameB.p = b3RotateVector(bodySimB.transform.q, b3Sub(base.localFrameB.p, bodySimB.localCenter));

	joint.deltaCenter = b3SubPos(bodySimB.center, bodySimA.center);

	// Cone axis is the z-axis of body A.
	let coneAxis = b3RotateVector(joint.frameA.q, b3Vec3_axisZ);

	// Twist axis is the z-axis of body B.
	let twistAxis = b3RotateVector(joint.frameB.q, b3Vec3_axisZ);

	if (joint.enableConeLimit) {
		// Swing axis may be zero
		let swingAxis = b3Normalize(b3Cross(coneAxis, twistAxis));
		let k = b3Dot(swingAxis, b3MulMV(invInertiaSum, swingAxis));
		joint.swingMass = k > 0.0 ? 1.0 / k : 0.0;
		joint.swingAxis = swingAxis;
	}

	if (joint.enableTwistLimit) {
		let relQ = b3InvMulQuat(joint.frameA.q, joint.frameB.q);
		let tanThetaOver2 = Mathf.sqrt((relQ.v.x * relQ.v.x + relQ.v.y * relQ.v.y) / (relQ.v.z * relQ.v.z + relQ.s * relQ.s));

		// todo verify this Jacobian using a finite difference, unit test?
		let swingAxis = b3Normalize(b3Cross(coneAxis, twistAxis));
		let perpAxis = b3Cross(swingAxis, coneAxis);
		let twistJacobian = b3MulAdd(coneAxis, tanThetaOver2, perpAxis);
		let k = b3Dot(twistJacobian, b3MulMV(invInertiaSum, twistJacobian));
		joint.twistMass = k > 0.0 ? 1.0 / k : 0.0;
		joint.twistJacobian = twistJacobian;
	}

	if (base.fixedRotation == false) {
		joint.rotationMass = b3InvertMatrix(invInertiaSum);
	} else {
		joint.rotationMass = b3Mat3_zero;
	}

	joint.springSoftness = b3MakeSoft(joint.hertz, joint.dampingRatio, context.h);

	if (context.enableWarmStarting == false) {
		joint.linearImpulse = b3Vec3_zero;
		joint.motorImpulse = b3Vec3_zero;
		joint.springImpulse = b3Vec3_zero;
		joint.swingImpulse = 0.0;
		joint.lowerTwistImpulse = 0.0;
		joint.upperTwistImpulse = 0.0;
	}
}

@noinline
export unsafe function b3WarmStartSphericalJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_sphericalJoint);

	let mA = base.invMassA;
	let mB = base.invMassB;
	let iA = base.invIA;
	let iB = base.invIB;

	ref joint = base.sphericalJoint;

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

	let angularImpulse = b3Add(joint.springImpulse, joint.motorImpulse);
	angularImpulse = b3MulSub(angularImpulse, joint.swingImpulse, joint.swingAxis);
	angularImpulse = b3MulAdd(angularImpulse, joint.lowerTwistImpulse - joint.upperTwistImpulse, joint.twistJacobian);

	vA = b3MulSub(vA, mA, joint.linearImpulse);
	wA = b3Sub(wA, b3MulMV(iA, b3Add(b3Cross(rA, joint.linearImpulse), angularImpulse)));

	vB = b3MulAdd(vB, mB, joint.linearImpulse);
	wB = b3Add(wB, b3MulMV(iB, b3Add(b3Cross(rB, joint.linearImpulse), angularImpulse)));

	if (stateA.flags & b3_dynamicFlag) {
		b3StoreStateVelocities(states, joint.indexA, vA, wA);
	}

	if (stateB.flags & b3_dynamicFlag) {
		b3StoreStateVelocities(states, joint.indexB, vB, wB);
	}
}

@noinline
export unsafe function b3SolveSphericalJoint(base: b3JointSim, context: b3StepContext, useBias: bool): void {
	let mA = base.invMassA;
	let mB = base.invMassB;
	let iA = base.invIA;
	let iB = base.invIB;

	// dummy state for static bodies
	ref joint = base.sphericalJoint;
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

	let relQ = b3InvMulQuat(quatA, quatB);

	if (fixedRotation == false && (joint.enableSpring || joint.enableMotor || joint.enableTwistLimit || joint.enableConeLimit)) {
		let invInertiaSum = b3AddMM(iA, iB);
		let cdot = b3Sub(wB, wA);
		let angularImpulse = b3Vec3_zero;

		// Solve spring
		if (joint.enableSpring) {
			// Rotation constraint error
			let deltaRotation = b3DeltaQuatToRotation(relQ, joint.targetRotation);
			let c = b3Neg(b3RotateVector(quatA, deltaRotation));

			let bias = b3MulSV(joint.springSoftness.biasRate, c);
			let massScale = joint.springSoftness.massScale;
			let impulseScale = joint.springSoftness.impulseScale;

			let impulse = b3MulSub(b3MulSV(-massScale, b3MulMV(joint.rotationMass, b3Add(cdot, bias))), impulseScale, joint.springImpulse);
			joint.springImpulse = b3Add(joint.springImpulse, impulse);

			cdot = b3Add(cdot, b3MulMV(invInertiaSum, impulse));
			angularImpulse = b3Add(angularImpulse, impulse);
		}

		if (joint.enableMotor) {
			// todo it might be more accurate to have the motor velocity in frameA due to sub-stepping.
			let lambda = b3Neg(b3MulMV(joint.rotationMass, b3Sub(cdot, joint.motorVelocity)));
			let newImpulse = b3Add(joint.motorImpulse, lambda);
			let length = b3Length(newImpulse);
			let maxImpulse: f32 = joint.maxMotorTorque * context.h;
			if (length > maxImpulse) {
				newImpulse = b3MulSV(maxImpulse / length, newImpulse);
			}

			lambda = b3Sub(newImpulse, joint.motorImpulse);
			joint.motorImpulse = newImpulse;

			cdot = b3Add(cdot, b3MulMV(invInertiaSum, lambda));
			angularImpulse = b3Add(angularImpulse, lambda);
		}

		if (joint.enableTwistLimit) {
			let twistAngle = b3GetTwistAngle(relQ);

			// todo does an updated twist axis help?

			let twistJacobian = joint.twistJacobian;
			let twistResponse = b3MulMV(invInertiaSum, twistJacobian);

			// Lower limit
			{
				let c: f32 = twistAngle - joint.lowerTwistAngle;
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

				let twistCdot = b3Dot(cdot, twistJacobian);
				let oldImpulse = joint.lowerTwistImpulse;
				let deltaImpulse = b3MaxFloat(-massScale * joint.twistMass * (twistCdot + bias) - impulseScale * oldImpulse, -oldImpulse);
				joint.lowerTwistImpulse = oldImpulse + deltaImpulse;

				cdot = b3MulAdd(cdot, deltaImpulse, twistResponse);
				angularImpulse = b3MulAdd(angularImpulse, deltaImpulse, twistJacobian);
			}

			// Upper limit
			{
				let c: f32 = joint.upperTwistAngle - twistAngle;
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
				let twistCdot: f32 = -b3Dot(cdot, twistJacobian);
				let oldImpulse = joint.upperTwistImpulse;
				let deltaImpulse = b3MaxFloat(-massScale * joint.twistMass * (twistCdot + bias) - impulseScale * oldImpulse, -oldImpulse);
				joint.upperTwistImpulse = oldImpulse + deltaImpulse;

				// sign flipped on applied impulse
				cdot = b3MulSub(cdot, deltaImpulse, twistResponse);
				angularImpulse = b3MulSub(angularImpulse, deltaImpulse, twistJacobian);
			}
		}

		if (joint.enableConeLimit) {
			let swingAngle = b3GetSwingAngle(relQ);

			// todo does an updated swing axis help?
			// b3Vec3 axisA = b3RotateVector( quatA, b3Vec3_axisZ );
			// b3Vec3 axisB = b3RotateVector( quatB, b3Vec3_axisZ );
			// b3Vec3 swingAxis = b3Normalize( b3Cross( axisA, axisB ) );
			// joint->swingAxis = swingAxis;

			let swingAxis = joint.swingAxis;

			let c: f32 = joint.coneAngle - swingAngle;
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
			let swingCdot: f32 = -b3Dot(cdot, swingAxis);
			let oldImpulse = joint.swingImpulse;
			let deltaImpulse = b3MaxFloat(-massScale * joint.swingMass * (swingCdot + bias) - impulseScale * oldImpulse, -oldImpulse);
			joint.swingImpulse = oldImpulse + deltaImpulse;

			// sign flipped on applied impulse
			angularImpulse = b3MulSub(angularImpulse, deltaImpulse, swingAxis);
		}

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

			let separation = b3Add(b3Sub(dcB, dcA), b3Sub(rB, rA));
			separation = b3Add(separation, joint.deltaCenter);

			bias = b3MulSV(base.constraintSoftness.biasRate, separation);
			massScale = base.constraintSoftness.massScale;
			impulseScale = base.constraintSoftness.impulseScale;
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
