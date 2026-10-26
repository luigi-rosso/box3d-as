// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/wheel_joint.c

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX, FLT_MIN } from "./core";
import {
	b3Add,
	b3AddMM,
	b3Atan2,
	b3Blend2,
	b3ClampFloat,
	b3Cross,
	b3Det,
	b3Dot,
	b3DotQuat,
	b3InvMulQuat,
	b3MakeMatrixFromQuat,
	b3MaxFloat,
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
import { b3Blend3, b3Matrix2, b3Solve2 } from "./math_internal";
import { b3JointId, b3_wheelJoint } from "./types";
import { b3_dynamicFlag, b3BodyState, b3StateAddress, b3StoreStateVelocities } from "./body";
import { b3GetBodyState, b3GetBodyTransform } from "./body";
import { b3GetWorld, b3World, b3_awakeSet } from "./physics_world";
import { b3MakeSoft, b3StepContext } from "./solver";
import { b3GetJointSimCheckType, b3JointSim } from "./joint";

export function b3WheelJoint_EnableSuspension(jointId: b3JointId, enableSpring: bool): void {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);

	if (enableSpring != joint.wheelJoint.enableSuspensionSpring) {
		joint.wheelJoint.enableSuspensionSpring = enableSpring;
		joint.wheelJoint.suspensionSpringImpulse = 0.0;
	}
}

export function b3WheelJoint_IsSuspensionEnabled(jointId: b3JointId): bool {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.enableSuspensionSpring;
}

export function b3WheelJoint_SetSuspensionHertz(jointId: b3JointId, hertz: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	joint.wheelJoint.suspensionHertz = hertz;
}

export function b3WheelJoint_GetSuspensionHertz(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.suspensionHertz;
}

export function b3WheelJoint_SetSuspensionDampingRatio(jointId: b3JointId, dampingRatio: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	joint.wheelJoint.suspensionDampingRatio = dampingRatio;
}

export function b3WheelJoint_GetSuspensionDampingRatio(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.suspensionDampingRatio;
}

export function b3WheelJoint_EnableSuspensionLimit(jointId: b3JointId, enableLimit: bool): void {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	if (joint.wheelJoint.enableSuspensionLimit != enableLimit) {
		joint.wheelJoint.lowerSuspensionImpulse = 0.0;
		joint.wheelJoint.upperSuspensionImpulse = 0.0;
		joint.wheelJoint.enableSuspensionLimit = enableLimit;
	}
}

export function b3WheelJoint_IsSuspensionLimitEnabled(jointId: b3JointId): bool {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.enableSuspensionLimit;
}

export function b3WheelJoint_GetLowerSuspensionLimit(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.lowerSuspensionLimit;
}

export function b3WheelJoint_GetUpperSuspensionLimit(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.upperSuspensionLimit;
}

export function b3WheelJoint_SetSuspensionLimits(jointId: b3JointId, lower: f32, upper: f32): void {
	B3_ASSERT(lower <= upper);
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	if (lower != joint.wheelJoint.lowerSuspensionLimit || upper != joint.wheelJoint.upperSuspensionLimit) {
		joint.wheelJoint.lowerSuspensionLimit = lower;
		joint.wheelJoint.upperSuspensionLimit = upper;
		joint.wheelJoint.lowerSuspensionImpulse = 0.0;
		joint.wheelJoint.upperSuspensionImpulse = 0.0;
	}
}

export function b3WheelJoint_EnableSpinMotor(jointId: b3JointId, enableMotor: bool): void {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	if (joint.wheelJoint.enableSpinMotor != enableMotor) {
		joint.wheelJoint.spinImpulse = 0.0;
		joint.wheelJoint.enableSpinMotor = enableMotor;
	}
}

export function b3WheelJoint_IsSpinMotorEnabled(jointId: b3JointId): bool {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.enableSpinMotor;
}

export function b3WheelJoint_SetSpinMotorSpeed(jointId: b3JointId, motorSpeed: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	joint.wheelJoint.spinSpeed = motorSpeed;
}

export function b3WheelJoint_GetSpinMotorSpeed(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.spinSpeed;
}

export function b3WheelJoint_SetMaxSpinTorque(jointId: b3JointId, torque: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	joint.wheelJoint.maxSpinTorque = torque;
}

export function b3WheelJoint_GetMaxSpinTorque(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.maxSpinTorque;
}

export function b3WheelJoint_EnableSteering(jointId: b3JointId, flag: bool): void {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	if (joint.wheelJoint.enableSteering != flag) {
		joint.wheelJoint.angularImpulse = b3Vec2(0.0, 0.0);
		joint.wheelJoint.enableSteering = flag;
	}
}

export function b3WheelJoint_IsSteeringEnabled(jointId: b3JointId): bool {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.enableSteering;
}

export function b3WheelJoint_SetSteeringHertz(jointId: b3JointId, hertz: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	joint.wheelJoint.steeringHertz = hertz;
}

export function b3WheelJoint_GetSteeringHertz(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.steeringHertz;
}

export function b3WheelJoint_SetSteeringDampingRatio(jointId: b3JointId, dampingRatio: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	joint.wheelJoint.steeringDampingRatio = dampingRatio;
}

export function b3WheelJoint_GetSteeringDampingRatio(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.steeringDampingRatio;
}

export function b3WheelJoint_SetMaxSteeringTorque(jointId: b3JointId, maxTorque: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	joint.wheelJoint.maxSteeringTorque = maxTorque;
}

export function b3WheelJoint_GetMaxSteeringTorque(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.maxSteeringTorque;
}

export function b3WheelJoint_EnableSteeringLimit(jointId: b3JointId, flag: bool): void {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	if (joint.wheelJoint.enableSteeringLimit != flag) {
		joint.wheelJoint.lowerSteeringImpulse = 0.0;
		joint.wheelJoint.upperSteeringImpulse = 0.0;
		joint.wheelJoint.enableSteeringLimit = flag;
	}
}

export function b3WheelJoint_IsSteeringLimitEnabled(jointId: b3JointId): bool {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.enableSteeringLimit;
}

export function b3WheelJoint_GetLowerSteeringLimit(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.lowerSteeringLimit;
}

export function b3WheelJoint_GetUpperSteeringLimit(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.upperSteeringLimit;
}

export function b3WheelJoint_SetSteeringLimits(jointId: b3JointId, lowerRadians: f32, upperRadians: f32): void {
	B3_ASSERT(lowerRadians <= upperRadians);
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	joint.wheelJoint.lowerSteeringLimit = lowerRadians;
	joint.wheelJoint.upperSteeringLimit = upperRadians;
}

export function b3WheelJoint_SetTargetSteeringAngle(jointId: b3JointId, radians: f32): void {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	joint.wheelJoint.targetSteeringAngle = radians;
}

export function b3WheelJoint_GetTargetSteeringAngle(jointId: b3JointId): f32 {
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return joint.wheelJoint.targetSteeringAngle;
}

export function b3WheelJoint_GetSpinSpeed(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let base = b3GetJointSimCheckType(jointId, b3_wheelJoint);

	let idA = base.bodyIdA;
	let idB = base.bodyIdB;

	let bodyB = world.bodies[idB];

	let setB = world.solverSets[bodyB.setIndex];
	let localIndexB = bodyB.localIndex;
	let bodySimB = setB.bodySims[localIndexB];

	let quatB = b3MulQuat(bodySimB.transform.q, base.localFrameB.q);
	let spinAxis = b3RotateVector(quatB, b3Vec3_axisZ);

	let wA = b3Vec3_zero;
	let stateA = b3GetBodyState(world, idA);
	if (stateA != B3_NULL_INDEX) {
		wA = world.solverSets[b3_awakeSet].bodyStates[stateA].angularVelocity;
	}

	let wB = b3Vec3_zero;
	let stateB = b3GetBodyState(world, idB);
	if (stateB != B3_NULL_INDEX) {
		wB = world.solverSets[b3_awakeSet].bodyStates[stateB].angularVelocity;
	}

	let speed: f32 = b3Dot(b3Sub(wB, wA), spinAxis);
	return speed;
}

export function b3WheelJoint_GetSpinTorque(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return world.inv_h * joint.wheelJoint.spinImpulse;
}

export function b3WheelJoint_GetSteeringAngle(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let base = b3GetJointSimCheckType(jointId, b3_wheelJoint);

	let idA = base.bodyIdA;
	let idB = base.bodyIdB;

	let bodyA = world.bodies[idA];
	let bodyB = world.bodies[idB];

	let setA = world.solverSets[bodyA.setIndex];
	let setB = world.solverSets[bodyB.setIndex];

	let localIndexA = bodyA.localIndex;
	let localIndexB = bodyB.localIndex;

	let bodySimA = setA.bodySims[localIndexA];
	let bodySimB = setB.bodySims[localIndexB];

	let quatA = b3MulQuat(bodySimA.transform.q, base.localFrameA.q);
	let quatB = b3MulQuat(bodySimB.transform.q, base.localFrameB.q);

	let matrixA = b3MakeMatrixFromQuat(quatA);
	let matrixB = b3MakeMatrixFromQuat(quatB);

	// Twist around x-axis
	let cs: f32 = b3Dot(matrixB.cz, matrixA.cz);
	let ss: f32 = -b3Dot(matrixB.cz, matrixA.cy);

	return b3Atan2(ss, cs);
}

export function b3WheelJoint_GetSteeringTorque(jointId: b3JointId): f32 {
	let world = b3GetWorld(jointId.world0);
	let joint = b3GetJointSimCheckType(jointId, b3_wheelJoint);
	return world.inv_h * joint.wheelJoint.steeringSpringImpulse;
}

export function b3GetWheelJointForce(world: b3World, base: b3JointSim): b3Vec3 {
	let transformA = b3GetBodyTransform(world, base.bodyIdA);
	ref joint = base.wheelJoint;

	// impulse in joint space
	let impulse = b3Vec3(
		joint.linearImpulse.x,
		joint.linearImpulse.y,
		joint.lowerSuspensionLimit + joint.upperSuspensionImpulse + joint.suspensionSpringImpulse,
	);

	// convert impulse to force
	let force = b3MulSV(world.inv_h, impulse);

	// convert to body space
	force = b3RotateVector(base.localFrameA.q, force);

	// convert to world space
	force = b3RotateVector(transformA.q, force);
	return force;
}

export function b3GetWheelJointTorque(world: b3World, base: b3JointSim): b3Vec3 {
	B3_ASSERT(base.type == b3_wheelJoint);

	// chase body id to the solver set where the body lives
	let idA = base.bodyIdA;

	let bodyA = world.bodies[idA];

	let setA = world.solverSets[bodyA.setIndex];

	let localIndexA = bodyA.localIndex;

	let bodySimA = setA.bodySims[localIndexA];

	let qA = b3MulQuat(bodySimA.transform.q, base.localFrameA.q);

	let matrixA = b3MakeMatrixFromQuat(qA);

	return b3MulSV(world.inv_h * base.wheelJoint.spinImpulse, matrixA.cz);
}

// See constraints.pdf

@noinline
export function b3PrepareWheelJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_wheelJoint);

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

	ref joint = base.wheelJoint;

	joint.indexA = bodyA.setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint.indexB = bodyB.setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.q = b3MulQuat(bodySimA.transform.q, base.localFrameA.q);
	joint.frameA.p = b3RotateVector(bodySimA.transform.q, b3Sub(base.localFrameA.p, bodySimA.localCenter));
	joint.frameB.q = b3MulQuat(bodySimB.transform.q, base.localFrameB.q);
	joint.frameB.p = b3RotateVector(bodySimB.transform.q, b3Sub(base.localFrameB.p, bodySimB.localCenter));

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint.deltaCenter = b3SubPos(bodySimB.center, bodySimA.center);

	let rA = joint.frameA.p;
	let rB = joint.frameB.p;

	let matrixA = b3MakeMatrixFromQuat(joint.frameA.q);
	let matrixB = b3MakeMatrixFromQuat(joint.frameB.q);

	// todo use fresh effective masses in the sub-step to avoid divergence like I saw for the prismatic joint

	{
		let suspensionAxis = matrixA.cx;
		let rAn = b3Cross(rA, suspensionAxis);
		let rBn = b3Cross(rB, suspensionAxis);

		let k: f32 = base.invMassA + base.invMassB + b3Dot(rAn, b3MulMV(base.invIA, rAn)) +
			b3Dot(rBn, b3MulMV(base.invIB, rBn));
		joint.suspensionMass = k > 0.0 ? 1.0 / k : 0.0;
	}

	joint.suspensionSoftness = b3MakeSoft(joint.suspensionHertz, joint.suspensionDampingRatio, context.h);
	joint.steeringSoftness = b3MakeSoft(joint.steeringHertz, joint.steeringDampingRatio, context.h);

	{
		// Rotation axis is the z-axis of body A.
		let spinAxis = matrixB.cz;
		let k: f32 = b3Dot(spinAxis, b3MulMV(invInertiaSum, spinAxis));
		joint.spinMass = k > 0.0 ? 1.0 / k : 0.0;
	}

	{
		// Twist constraint around x-axis
		let cs: f32 = b3Dot(matrixB.cz, matrixA.cz);
		let ss: f32 = -b3Dot(matrixB.cz, matrixA.cy);
		let den: f32 = cs * cs + ss * ss;
		den = den > 0.0 ? 1.0 / den : 0.0;
		let steeringAxis = b3MulSV(den, b3Cross(matrixB.cz, b3Sub(b3MulSV(-cs, matrixA.cy), b3MulSV(ss, matrixA.cz))));

		let k: f32 = b3Dot(steeringAxis, b3MulMV(invInertiaSum, steeringAxis));
		joint.steeringMass = k > 0.0 ? 1.0 / k : 0.0;
	}

	if (context.enableWarmStarting == false) {
		joint.linearImpulse = b3Vec2(0.0, 0.0);
		joint.angularImpulse = b3Vec2(0.0, 0.0);
		joint.spinImpulse = 0.0;
		joint.suspensionSpringImpulse = 0.0;
		joint.lowerSuspensionImpulse = 0.0;
		joint.upperSuspensionImpulse = 0.0;
		joint.steeringSpringImpulse = 0.0;
		joint.lowerSteeringImpulse = 0.0;
		joint.upperSteeringImpulse = 0.0;
	}
}

@noinline
export unsafe function b3WarmStartWheelJoint(base: b3JointSim, context: b3StepContext): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_wheelJoint);

	let mA = base.invMassA;
	let mB = base.invMassB;
	let iA = base.invIA;
	let iB = base.invIB;

	// dummy state for static bodies
	ref joint = base.wheelJoint;
	let states = context.states;
	let stateA = load<b3BodyState>(b3StateAddress(states, joint.indexA));
	let stateB = load<b3BodyState>(b3StateAddress(states, joint.indexB));

	let rA = b3RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b3RotateVector(stateB.deltaRotation, joint.frameB.p);

	let d = b3Add(b3Add(b3Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), b3Sub(rB, rA));

	let quatA = b3MulQuat(stateA.deltaRotation, joint.frameA.q);
	let quatB = b3MulQuat(stateB.deltaRotation, joint.frameB.q);
	if (b3DotQuat(quatA, quatB) < 0.0) {
		// this keeps the rotation angle in the range [-pi, pi]
		quatB = b3NegateQuat(quatB);
	}

	let matrixA = b3MakeMatrixFromQuat(quatA);
	let matrixB = b3MakeMatrixFromQuat(quatB);

	let sAx = b3Cross(b3Add(d, rA), matrixA.cx);
	let sBx = b3Cross(rB, matrixA.cx);
	let sAy = b3Cross(b3Add(d, rA), matrixA.cy);
	let sBy = b3Cross(rB, matrixA.cy);
	let sAz = b3Cross(b3Add(d, rA), matrixA.cz);
	let sBz = b3Cross(rB, matrixA.cz);

	let suspensionImpulse: f32 = joint.suspensionSpringImpulse + joint.lowerSuspensionImpulse - joint.upperSuspensionImpulse;

	let linearImpulseY: f32 = joint.linearImpulse.x;
	let linearImpulseZ: f32 = joint.linearImpulse.y;
	let angularImpulseX: f32 = joint.angularImpulse.x;
	let angularImpulseY: f32 = joint.angularImpulse.y;

	let linearImpulse = b3Blend3(suspensionImpulse, matrixA.cx, linearImpulseY, matrixA.cy, linearImpulseZ, matrixA.cz);
	let angularImpulseA = b3Blend3(suspensionImpulse, sAx, linearImpulseY, sAy, linearImpulseZ, sAz);
	let angularImpulseB = b3Blend3(suspensionImpulse, sBx, linearImpulseY, sBy, linearImpulseZ, sBz);
	let angularImpulse = b3MulSV(joint.spinImpulse, matrixA.cz);

	let spinAxis = matrixB.cz;

	if (joint.enableSteering) {
		// Twist constraint around x-axis
		let cs: f32 = b3Dot(matrixB.cz, matrixA.cz);
		let ss: f32 = -b3Dot(matrixB.cz, matrixA.cy);
		let den: f32 = cs * cs + ss * ss;
		den = den > 0.0 ? 1.0 / den : 0.0;
		let steeringAxis = b3MulSV(den, b3Cross(matrixB.cz, b3Sub(b3MulSV(-cs, matrixA.cy), b3MulSV(ss, matrixA.cz))));

		let perpAxis = b3Cross(spinAxis, matrixA.cx);
		let steeringImpulse: f32 = joint.steeringSpringImpulse + joint.lowerSteeringImpulse - joint.upperSteeringImpulse;
		angularImpulse = b3Blend3(angularImpulseX, perpAxis, joint.spinImpulse, spinAxis, steeringImpulse, steeringAxis);
	} else {
		let relQ = b3InvMulQuat(quatA, quatB);
		let perpAxisX = b3MulSV(0.5, b3RotateVector(quatA, b3Add(b3MulSV(relQ.s, b3Vec3_axisX), b3Cross(relQ.v, b3Vec3_axisX))));
		let perpAxisY = b3MulSV(0.5, b3RotateVector(quatA, b3Add(b3MulSV(relQ.s, b3Vec3_axisY), b3Cross(relQ.v, b3Vec3_axisY))));
		angularImpulse = b3Add(
			angularImpulse,
			b3Blend3(angularImpulseX, perpAxisX, angularImpulseY, perpAxisY, joint.spinImpulse, spinAxis),
		);
	}

	// the dummy state has no flags, so these only write awake states
	if (stateA.flags & b3_dynamicFlag) {
		states[joint.indexA].linearVelocity = b3MulSub(stateA.linearVelocity, mA, linearImpulse);
		states[joint.indexA].angularVelocity = b3Sub(stateA.angularVelocity, b3MulMV(iA, b3Add(angularImpulseA, angularImpulse)));
	}

	if (stateB.flags & b3_dynamicFlag) {
		states[joint.indexB].linearVelocity = b3MulAdd(stateB.linearVelocity, mB, linearImpulse);
		states[joint.indexB].angularVelocity = b3Add(stateB.angularVelocity, b3MulMV(iB, b3Add(angularImpulseB, angularImpulse)));
	}
}

@noinline
export unsafe function b3SolveWheelJoint(base: b3JointSim, context: b3StepContext, useBias: bool): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(base.type == b3_wheelJoint);

	let mA = base.invMassA;
	let mB = base.invMassB;
	let iA = base.invIA;
	let iB = base.invIB;

	// dummy state for static bodies
	ref joint = base.wheelJoint;
	let states = context.states;
	let stateA = load<b3BodyState>(b3StateAddress(states, joint.indexA));
	let stateB = load<b3BodyState>(b3StateAddress(states, joint.indexB));

	let vA = stateA.linearVelocity;
	let wA = stateA.angularVelocity;
	let vB = stateB.linearVelocity;
	let wB = stateB.angularVelocity;

	let fixedRotation = base.fixedRotation;

	// current anchors
	let rA = b3RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b3RotateVector(stateB.deltaRotation, joint.frameB.p);

	let quatA = b3MulQuat(stateA.deltaRotation, joint.frameA.q);
	let quatB = b3MulQuat(stateB.deltaRotation, joint.frameB.q);

	if (b3DotQuat(quatA, quatB) < 0.0) {
		// this keeps the rotation angle in the range [-pi, pi]
		quatB = b3NegateQuat(quatB);
	}

	let relQ = b3InvMulQuat(quatA, quatB);
	let matrixA = b3MakeMatrixFromQuat(quatA);
	let matrixB = b3MakeMatrixFromQuat(quatB);

	let d = b3Add(b3Add(b3Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), b3Sub(rB, rA));
	let sAx = b3Cross(b3Add(d, rA), matrixA.cx);
	let sBx = b3Cross(rB, matrixA.cx);
	let sAy = b3Cross(b3Add(d, rA), matrixA.cy);
	let sBy = b3Cross(rB, matrixA.cy);
	let sAz = b3Cross(b3Add(d, rA), matrixA.cz);
	let sBz = b3Cross(rB, matrixA.cz);

	let translation: f32 = b3Dot(matrixA.cx, d);

	// Steering param ib = cz_b, ia = cz_a, ja = -cy_a
	let cs: f32 = b3Dot(matrixB.cz, matrixA.cz);
	let ss: f32 = -b3Dot(matrixB.cz, matrixA.cy);
	let den: f32 = cs * cs + ss * ss;
	den = den > 0.0 ? 1.0 / den : 0.0;
	let steeringAxis = b3MulSV(den, b3Cross(matrixB.cz, b3Sub(b3MulSV(-cs, matrixA.cy), b3MulSV(ss, matrixA.cz))));

	// motor constraint
	if (joint.enableSpinMotor && fixedRotation == false) {
		let spinAxis = matrixB.cz;
		let cdot: f32 = b3Dot(b3Sub(wB, wA), spinAxis) - joint.spinSpeed;
		let impulse: f32 = -joint.spinMass * cdot;
		let oldImpulse: f32 = joint.spinImpulse;
		let maxImpulse: f32 = context.h * joint.maxSpinTorque;
		joint.spinImpulse = b3ClampFloat(joint.spinImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = joint.spinImpulse - oldImpulse;

		wA = b3Sub(wA, b3MulMV(iA, b3MulSV(impulse, spinAxis)));
		wB = b3Add(wB, b3MulMV(iB, b3MulSV(impulse, spinAxis)));
	}

	// suspension
	if (joint.enableSuspensionSpring) {
		// This is a real spring and should be applied even during relax
		let c: f32 = translation;
		let bias: f32 = joint.suspensionSoftness.biasRate * c;
		let massScale: f32 = joint.suspensionSoftness.massScale;
		let impulseScale: f32 = joint.suspensionSoftness.impulseScale;

		let cdot: f32 = b3Dot(matrixA.cx, b3Sub(vB, vA)) + b3Dot(sBx, wB) - b3Dot(sAx, wA);
		let impulse: f32 = -massScale * joint.suspensionMass * (cdot + bias) - impulseScale * joint.suspensionSpringImpulse;
		joint.suspensionSpringImpulse += impulse;

		let linearImpulse = b3MulSV(impulse, matrixA.cx);
		let angularImpulseA = b3MulSV(impulse, sAx);
		let angularImpulseB = b3MulSV(impulse, sBx);

		vA = b3MulSub(vA, mA, linearImpulse);
		wA = b3Sub(wA, b3MulMV(iA, angularImpulseA));
		vB = b3MulAdd(vB, mB, linearImpulse);
		wB = b3Add(wB, b3MulMV(iB, angularImpulseB));
	}

	// steering
	if (joint.enableSteering && fixedRotation == false) {
		let steeringAngle: f32 = b3Atan2(ss, cs);

		{
			// This is a real spring and should be applied even during relax
			let c: f32 = steeringAngle - joint.targetSteeringAngle;
			let bias: f32 = joint.steeringSoftness.biasRate * c;
			let massScale: f32 = joint.steeringSoftness.massScale;
			let impulseScale: f32 = joint.steeringSoftness.impulseScale;

			let cdot: f32 = b3Dot(steeringAxis, b3Sub(wB, wA));
			let oldImpulse: f32 = joint.steeringSpringImpulse;
			let impulse: f32 = -massScale * joint.steeringMass * (cdot + bias) - impulseScale * oldImpulse;
			let maxImpulse: f32 = context.h * joint.maxSteeringTorque;
			joint.steeringSpringImpulse = b3ClampFloat(oldImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = joint.steeringSpringImpulse - oldImpulse;

			wA = b3Sub(wA, b3MulMV(iA, b3MulSV(impulse, steeringAxis)));
			wB = b3Add(wB, b3MulMV(iB, b3MulSV(impulse, steeringAxis)));
		}

		if (joint.enableSteeringLimit) {
			// Lower limit
			{
				let c: f32 = steeringAngle - joint.lowerSteeringLimit;
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

				let cdot: f32 = b3Dot(steeringAxis, b3Sub(wB, wA));
				let oldImpulse: f32 = joint.lowerSteeringImpulse;
				let impulse: f32 = -massScale * joint.steeringMass * (cdot + bias) - impulseScale * oldImpulse;
				joint.lowerSteeringImpulse = b3MaxFloat(oldImpulse + impulse, 0.0);
				impulse = joint.lowerSteeringImpulse - oldImpulse;

				wA = b3Sub(wA, b3MulMV(iA, b3MulSV(impulse, steeringAxis)));
				wB = b3Add(wB, b3MulMV(iB, b3MulSV(impulse, steeringAxis)));
			}

			// Upper limit
			// Note: signs are flipped to keep c positive when the constraint is satisfied.
			// This also keeps the impulse positive when the limit is active.
			{
				// sign flipped
				let c: f32 = joint.upperSteeringLimit - steeringAngle;
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

				// sign flipped on cdot
				let cdot: f32 = b3Dot(steeringAxis, b3Sub(wA, wB));
				let oldImpulse: f32 = joint.upperSteeringImpulse;
				let impulse: f32 = -massScale * joint.steeringMass * (cdot + bias) - impulseScale * oldImpulse;
				joint.upperSteeringImpulse = b3MaxFloat(oldImpulse + impulse, 0.0);
				impulse = joint.upperSteeringImpulse - oldImpulse;

				// sign flipped on applied impulse
				wA = b3Add(wA, b3MulMV(iA, b3MulSV(impulse, steeringAxis)));
				wB = b3Sub(wB, b3MulMV(iB, b3MulSV(impulse, steeringAxis)));
			}
		}
	}

	if (joint.enableSuspensionLimit) {
		// Lower limit
		{
			let c: f32 = translation - joint.lowerSuspensionLimit;
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

			let cdot: f32 = b3Dot(matrixA.cx, b3Sub(vB, vA)) + b3Dot(sBx, wB) - b3Dot(sAx, wA);
			let impulse: f32 = -massScale * joint.suspensionMass * (cdot + bias) - impulseScale * joint.lowerSuspensionImpulse;
			let oldImpulse: f32 = joint.lowerSuspensionImpulse;
			joint.lowerSuspensionImpulse = b3MaxFloat(oldImpulse + impulse, 0.0);
			impulse = joint.lowerSuspensionImpulse - oldImpulse;

			let linearImpulse = b3MulSV(impulse, matrixA.cx);
			let angularImpulseA = b3MulSV(impulse, sAx);
			let angularImpulseB = b3MulSV(impulse, sBx);

			vA = b3MulSub(vA, mA, linearImpulse);
			wA = b3Sub(wA, b3MulMV(iA, angularImpulseA));
			vB = b3MulAdd(vB, mB, linearImpulse);
			wB = b3Add(wB, b3MulMV(iB, angularImpulseB));
		}

		// Upper limit
		// Note: signs are flipped to keep c positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			// sign flipped
			let c: f32 = joint.upperSuspensionLimit - translation;
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

			// sign flipped on cdot
			let cdot: f32 = b3Dot(matrixA.cx, b3Sub(vA, vB)) + b3Dot(sAx, wA) - b3Dot(sBx, wB);
			let impulse: f32 = -massScale * joint.suspensionMass * (cdot + bias) - impulseScale * joint.upperSuspensionImpulse;
			let oldImpulse: f32 = joint.upperSuspensionImpulse;
			joint.upperSuspensionImpulse = b3MaxFloat(oldImpulse + impulse, 0.0);
			impulse = joint.upperSuspensionImpulse - oldImpulse;

			let linearImpulse = b3MulSV(impulse, matrixA.cx);
			let angularImpulseA = b3MulSV(impulse, sAx);
			let angularImpulseB = b3MulSV(impulse, sBx);

			// sign flipped on applied impulse
			vA = b3MulAdd(vA, mA, linearImpulse);
			wA = b3Add(wA, b3MulMV(iA, angularImpulseA));
			vB = b3MulSub(vB, mB, linearImpulse);
			wB = b3Sub(wB, b3MulMV(iB, angularImpulseB));
		}
	}

	// Collinearity constraint
	if (fixedRotation == false) {
		if (joint.enableSteering == true) {
			let bias: f32 = 0.0;
			let massScale: f32 = 1.0;
			let impulseScale: f32 = 0.0;
			if (useBias) {
				let c: f32 = b3Dot(matrixA.cx, matrixB.cz);

				bias = base.constraintSoftness.biasRate * c;
				massScale = base.constraintSoftness.massScale;
				impulseScale = base.constraintSoftness.impulseScale;
			}

			let u = b3Cross(matrixB.cz, matrixA.cx);
			let cdot: f32 = b3Dot(b3Sub(wB, wA), u);

			let invInertiaSum = b3AddMM(iA, iB);
			let k: f32 = b3Dot(u, b3MulMV(invInertiaSum, u));
			let perpMass: f32 = k > 0.0 ? 1.0 / k : 0.0;

			let deltaImpulse: f32 = -massScale * perpMass * (cdot + bias) - impulseScale * joint.angularImpulse.x;
			joint.angularImpulse.x += deltaImpulse;

			wA = b3MulSub(wA, deltaImpulse, b3MulMV(iA, u));
			wB = b3MulAdd(wB, deltaImpulse, b3MulMV(iB, u));
		} else {
			let bias = b3Vec2(0.0, 0.0);
			let massScale: f32 = 1.0;
			let impulseScale: f32 = 0.0;

			if (useBias) {
				let c = b3Vec2(relQ.v.x, relQ.v.y);
				bias = b3Vec2(base.constraintSoftness.biasRate * c.x, base.constraintSoftness.biasRate * c.y);
				massScale = base.constraintSoftness.massScale;
				impulseScale = base.constraintSoftness.impulseScale;
			}

			// Collinearity constraint as 2-by-2
			let perpAxisX = b3MulSV(0.5, b3RotateVector(quatA, b3Add(b3MulSV(relQ.s, b3Vec3_axisX), b3Cross(relQ.v, b3Vec3_axisX))));
			let perpAxisY = b3MulSV(0.5, b3RotateVector(quatA, b3Add(b3MulSV(relQ.s, b3Vec3_axisY), b3Cross(relQ.v, b3Vec3_axisY))));

			let invInertiaSum = b3AddMM(iA, iB);
			let kxx: f32 = b3Dot(perpAxisX, b3MulMV(invInertiaSum, perpAxisX));
			let kyy: f32 = b3Dot(perpAxisY, b3MulMV(invInertiaSum, perpAxisY));
			let kxy: f32 = b3Dot(perpAxisX, b3MulMV(invInertiaSum, perpAxisY));

			let k = b3Matrix2(b3Vec2(kxx, kxy), b3Vec2(kxy, kyy));

			let wRel = b3Sub(wB, wA);
			let cdot = b3Vec2(b3Dot(wRel, perpAxisX), b3Dot(wRel, perpAxisY));
			let oldImpulse = joint.angularImpulse;
			let cdotPlusBias = b3Vec2(cdot.x + bias.x, cdot.y + bias.y);
			let sol = b3Solve2(k, cdotPlusBias);
			let deltaImpulse = b3Vec2(
				-massScale * sol.x - impulseScale * oldImpulse.x,
				-massScale * sol.y - impulseScale * oldImpulse.y,
			);
			joint.angularImpulse = b3Vec2(oldImpulse.x + deltaImpulse.x, oldImpulse.y + deltaImpulse.y);

			let angularImpulse = b3Blend2(deltaImpulse.x, perpAxisX, deltaImpulse.y, perpAxisY);
			wA = b3Sub(wA, b3MulMV(iA, angularImpulse));
			wB = b3Add(wB, b3MulMV(iB, angularImpulse));
		}
	}

	// Solve point-to-line constraint
	{
		let perpY = matrixA.cy;
		let perpZ = matrixA.cz;

		let bias = b3Vec2(0.0, 0.0);
		let massScale: f32 = 1.0;
		let impulseScale: f32 = 0.0;
		if (useBias) {
			let c = b3Vec2(b3Dot(perpY, d), b3Dot(perpZ, d));
			bias = b3Vec2(base.constraintSoftness.biasRate * c.x, base.constraintSoftness.biasRate * c.y);
			massScale = base.constraintSoftness.massScale;
			impulseScale = base.constraintSoftness.impulseScale;
		}

		let vRel = b3Sub(b3Sub(b3Add(vB, b3Cross(wB, rB)), vA), b3Cross(wA, b3Add(rA, d)));
		let cdot = b3Vec2(b3Dot(perpY, vRel), b3Dot(perpZ, vRel));

		//// K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
		///// Jx = [-perpX, -cross(d + rA, perpX), perpX, cross(rB, perpX)]

		let kyy: f32 = mA + mB + b3Dot(sAy, b3MulMV(iA, sAy)) + b3Dot(sBy, b3MulMV(iB, sBy));
		let kyz: f32 = b3Dot(sAy, b3MulMV(iA, sAz)) + b3Dot(sBy, b3MulMV(iB, sBz));
		let kzz: f32 = mA + mB + b3Dot(sAz, b3MulMV(iA, sAz)) + b3Dot(sBz, b3MulMV(iB, sBz));

		let k = b3Matrix2(b3Vec2(kyy, kyz), b3Vec2(kyz, kzz));

		let oldImpulse = joint.linearImpulse;
		let cdotPlusBias = b3Vec2(cdot.x + bias.x, cdot.y + bias.y);
		let sol = b3Solve2(k, cdotPlusBias);
		let deltaImpulse = b3Vec2(
			-massScale * sol.x - impulseScale * oldImpulse.x,
			-massScale * sol.y - impulseScale * oldImpulse.y,
		);
		joint.linearImpulse = b3Vec2(oldImpulse.x + deltaImpulse.x, oldImpulse.y + deltaImpulse.y);

		let linearImpulse = b3Blend2(deltaImpulse.x, perpY, deltaImpulse.y, perpZ);

		vA = b3MulSub(vA, mA, linearImpulse);
		wA = b3Sub(wA, b3MulMV(iA, b3Blend2(deltaImpulse.x, sAy, deltaImpulse.y, sAz)));
		vB = b3MulAdd(vB, mB, linearImpulse);
		wB = b3Add(wB, b3MulMV(iB, b3Blend2(deltaImpulse.x, sBy, deltaImpulse.y, sBz)));
	}

	if (stateA.flags & b3_dynamicFlag) {
		b3StoreStateVelocities(states, joint.indexA, vA, wA);
	}

	if (stateB.flags & b3_dynamicFlag) {
		b3StoreStateVelocities(states, joint.indexB, vB, wB);
	}
}
