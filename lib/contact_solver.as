// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/contact_solver.c
// The width 4 constraint types of src/contact_solver_wide.inl live here too, because the graph
// colors and the step context hold them. Its functions go to the wide4 port.

import { B3_NULL_INDEX, FLT_EPSILON, UINT16_MAX, B3_ASSERT } from "./core";
import {
	b3Add,
	b3AddMM,
	b3Blend2,
	b3ClampFloat,
	b3Cross,
	b3Distance,
	b3Dot,
	b3InvertMatrix,
	b3Mat3_zero,
	b3Matrix3,
	b3MaxFloat,
	b3MinFloat,
	b3MinInt,
	b3MulAdd,
	b3MulMV,
	b3MulSV,
	b3MulSub,
	b3Neg,
	b3Perp,
	b3RotateVector,
	b3Sub,
	b3Vec2,
	b3Vec3,
	b3Vec3_zero,
} from "./math_functions";
import { b3Dot2, b3Invert2, b3Matrix2, b3MulMV2, b3Sub2 } from "./math_internal";
import { B3_MIN_FRICTION_WEIGHT, B3_SPECULATIVE_DISTANCE } from "./constants";
import { b3SetBit } from "./bitset";
import { b3FloatW4, b3Vec3W4 } from "./simd";
import { b3Softness, b3SolverBlock, b3StepContext, b3_overflowBlock } from "./solver";
import { b3DecodeAwakeIndex, b3_dynamicFlag, b3_identityBodyState } from "./body";
import { b3_contactStaticFlag, b3_simEnableHitEvent } from "./contact";
import { b3_awakeSet } from "./physics_world";
import { B3_OVERFLOW_INDEX } from "./constraint_graph";
import {
	b3PrepareContacts_ConvexW4,
	b3WarmStartContacts_ConvexW4,
	b3PushContacts_ConvexW4,
	b3SolveContacts_ConvexW4,
	b3ApplyRestitution_ConvexW4,
	b3StoreImpulses_ConvexW4,
	b3PrepareContacts_MeshWideW4,
	b3WarmStartContacts_MeshWideW4,
	b3PushContacts_MeshWideW4,
	b3SolveContacts_MeshWideW4,
	b3ApplyRestitution_MeshWideW4,
	b3StoreImpulses_MeshWideW4,
} from "./contact_solver_wide";

export struct b3ManifoldConstraintPoint {
	rA: b3Vec3 = b3Vec3();
	rB: b3Vec3 = b3Vec3();
	baseSeparation: f32 = 0;
	relativeVelocity: f32 = 0;
	normalImpulse: f32 = 0;
	totalNormalImpulse: f32 = 0;
	restitutionImpulse: f32 = 0;
	normalMass: f32 = 0;
	leverArm: f32 = 0;
}

export struct b3ManifoldConstraint {
	points: FixedArray<b3ManifoldConstraintPoint, 4>;
	pointCount: i32 = 0;
	normal: b3Vec3 = b3Vec3();
	tangent1: b3Vec3 = b3Vec3();
	tangent2: b3Vec3 = b3Vec3();
	// Friction centers
	centerA: b3Vec3 = b3Vec3();
	centerB: b3Vec3 = b3Vec3();
	twistMass: f32 = 0;
	twistImpulse: f32 = 0;
	tangentMass: b3Matrix2 = b3Matrix2();
	frictionImpulse: b3Vec2 = b3Vec2();
	rollingImpulse: b3Vec3 = b3Vec3();
	tangentVelocity1: f32 = 0;
	tangentVelocity2: f32 = 0;
}

export struct b3ContactConstraint {
	// Index of the first b3ManifoldConstraint in the overflow color's manifoldConstraints
	constraints: i32 = 0;
	// Contact id, an index into b3World.contacts
	contact: i32 = 0;
	indexA: i32 = 0;
	indexB: i32 = 0;
	invMassA: f32 = 0;
	invMassB: f32 = 0;
	invIA: b3Matrix3 = b3Matrix3();
	invIB: b3Matrix3 = b3Matrix3();
	softness: b3Softness = b3Softness();
	rollingMass: b3Matrix3 = b3Matrix3();
	friction: f32 = 0;
	restitution: f32 = 0;
	rollingResistance: f32 = 0;
	manifoldCount: i32 = 0;
}

// contact_solver_wide.inl types at B3_SIMD_WIDTH 4

// Wide vec2
export struct b3Vec2W4 {
	x: b3FloatW4 = b3FloatW4();
	y: b3FloatW4 = b3FloatW4();
}

// Wide quaternion
export struct b3QuatW4 {
	V: b3Vec3W4 = b3Vec3W4();
	S: b3FloatW4 = b3FloatW4();
}

// Wide symmetric matrix2
export struct b3SymMatrix2W4 {
	cxx: b3FloatW4 = b3FloatW4();
	cxy: b3FloatW4 = b3FloatW4();
	cyy: b3FloatW4 = b3FloatW4();
}

// Wide symmetric matrix3
export struct b3SymMatrix3W4 {
	cxx: b3FloatW4 = b3FloatW4();
	cxy: b3FloatW4 = b3FloatW4();
	cxz: b3FloatW4 = b3FloatW4();
	cyy: b3FloatW4 = b3FloatW4();
	cyz: b3FloatW4 = b3FloatW4();
	czz: b3FloatW4 = b3FloatW4();
}

export struct b3Matrix3W4 {
	cx: b3Vec3W4 = b3Vec3W4();
	cy: b3Vec3W4 = b3Vec3W4();
	cz: b3Vec3W4 = b3Vec3W4();
}

// Soft contact constraints with sub-stepping support
// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles & capsules)
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

export struct b3ContactConstraintPointW4 {
	anchorAs: b3Vec3W4 = b3Vec3W4();
	anchorBs: b3Vec3W4 = b3Vec3W4();
	baseSeparations: b3FloatW4 = b3FloatW4();
	normalImpulses: b3FloatW4 = b3FloatW4();
	totalNormalImpulses: b3FloatW4 = b3FloatW4();
	normalMasses: b3FloatW4 = b3FloatW4();
	leverArms: b3FloatW4 = b3FloatW4();
	relativeVelocities: b3FloatW4 = b3FloatW4();
	restitutionImpulses: b3FloatW4 = b3FloatW4();
}

export struct b3ContactManifoldW4 {
	normal: b3Vec3W4 = b3Vec3W4();

	tangent1: b3Vec3W4 = b3Vec3W4();
	tangent2: b3Vec3W4 = b3Vec3W4();

	// Friction centers
	centerA: b3Vec3W4 = b3Vec3W4();
	centerB: b3Vec3W4 = b3Vec3W4();
	twistMass: b3FloatW4 = b3FloatW4();
	twistImpulse: b3FloatW4 = b3FloatW4();
	tangentMass: b3SymMatrix2W4 = b3SymMatrix2W4();
	frictionImpulse: b3Vec2W4 = b3Vec2W4();
	rollingImpulse: b3Vec3W4 = b3Vec3W4();
	rollingMask: b3FloatW4 = b3FloatW4();
	tangentVelocity1: b3FloatW4 = b3FloatW4();
	tangentVelocity2: b3FloatW4 = b3FloatW4();

	pointCount: i32 = 0;

	points: FixedArray<b3ContactConstraintPointW4, 4>;
}

// Solves one contact per lane
export struct b3ContactConstraintW4 {
	// These are base 1
	indexA: FixedArray<i32, 4>;
	indexB: FixedArray<i32, 4>;

	// Contact ids, B3_NULL_INDEX for an empty lane where C stores NULL
	contacts: FixedArray<i32, 4>;

	invMassA: b3FloatW4 = b3FloatW4();
	invMassB: b3FloatW4 = b3FloatW4();
	invIA: b3SymMatrix3W4 = b3SymMatrix3W4();
	invIB: b3SymMatrix3W4 = b3SymMatrix3W4();
	rollingMass: b3SymMatrix3W4 = b3SymMatrix3W4();
	friction: b3FloatW4 = b3FloatW4();
	rollingResistance: b3FloatW4 = b3FloatW4();
	restitution: b3FloatW4 = b3FloatW4();

	manifoldCount: i32 = 0;
}

// wide version of b3BodyState
export struct b3BodyStateW4 {
	v: b3Vec3W4 = b3Vec3W4();
	w: b3Vec3W4 = b3Vec3W4();
	dp: b3Vec3W4 = b3Vec3W4();
	dq: b3QuatW4 = b3QuatW4();
}

// The width generic names of the .inl typedefs
export {
	b3Vec2W4 as b3Vec2W,
	b3QuatW4 as b3QuatW,
	b3SymMatrix2W4 as b3SymMatrix2W,
	b3SymMatrix3W4 as b3SymMatrix3W,
	b3Matrix3W4 as b3Matrix3W,
	b3ContactConstraintPointW4 as b3ContactConstraintPointWide,
	b3ContactManifoldW4 as b3ContactManifoldWide,
	b3ContactConstraintW4 as b3ContactConstraintWide,
	b3BodyStateW4 as b3BodyStateW,
};

// functions from contact_solver.c follow

// todo clean this file up because it is only used for overflow now

// contact separation for sub-stepping
// s = s0 + dot(cB + rB - cA - rA, normal)
// normal is held constant
// body positions c can translation and anchors r can rotate
// s(t) = s0 + dot(cB(t) + rB(t) - cA(t) - rA(t), normal)
// s(t) = s0 + dot(cB0 + dpB + rot(dqB, rB0) - cA0 - dpA - rot(dqA, rA0), normal)
// s(t) = s0 + dot(cB0 - cA0, normal) + dot(dpB - dpA + rot(dqB, rB0) - rot(dqA, rA0), normal)
// s_base = s0 + dot(cB0 - cA0, normal)

// Prepare overflow constraints
export function b3PrepareContacts_Mesh(block: b3SolverBlock, context: b3StepContext): void {
	let world = context.world;
	let bodySims = context.sims;
	let states = context.states;

	let warmStartScale: f32 = world.enableWarmStarting ? 1.0 : 0.0;
	let anyRestitution = false;

	// Used for friction center weighting.
	let invTau: f32 = 1.0 / B3_SPECULATIVE_DISTANCE();

	B3_ASSERT(block.blockType == b3_overflowBlock);
	let overflow = world.constraintGraph.colors[B3_OVERFLOW_INDEX];
	let spans = context.overflowSpans;
	let manifoldBase = overflow.manifoldConstraints;
	let base = overflow.contactConstraints;

	let index = block.startIndex;
	let endIndex = block.startIndex + <i32>block.count;

	// Find color for start index. Linear search but fast.
	let colorIndex = 0;
	while (spans[colorIndex + 1].start <= index) {
		colorIndex += 1;
	}

	// Loop over block
	while (index < endIndex) {
		let colorStart = spans[colorIndex].start;
		let colorEndIndex = b3MinInt(spans[colorIndex + 1].start, endIndex);
		let specs = spans[colorIndex].contacts;

		// Loop over color
		for (; index < colorEndIndex; ++index) {
			ref contactConstraint = base[index];

			let localIndex = index - colorStart;
			B3_ASSERT(0 <= localIndex && localIndex < spans[colorIndex].count);
			let contactId = specs[localIndex].contactId;
			let contact = world.contacts[contactId];
			B3_ASSERT(contact.contactId == contactId);

			let indexA = b3DecodeAwakeIndex(contact.encodedBodySimA);
			let indexB = b3DecodeAwakeIndex(contact.encodedBodySimB);

			// Body A data
			let mA: f32 = 0.0;
			let iA = b3Mat3_zero;

			if (indexA != B3_NULL_INDEX) {
				mA = bodySims[indexA].invMass;
				iA = bodySims[indexA].invInertiaWorld;
			}

			// Body B data
			let mB: f32 = 0.0;
			let iB = b3Mat3_zero;

			if (indexB != B3_NULL_INDEX) {
				mB = bodySims[indexB].invMass;
				iB = bodySims[indexB].invInertiaWorld;
			}

			let manifoldCount = contact.manifoldCount;
			contactConstraint.contact = contactId;
			contactConstraint.manifoldCount = manifoldCount;
			contactConstraint.indexA = indexA;
			contactConstraint.indexB = indexB;
			contactConstraint.invIA = iA;
			contactConstraint.invMassA = mA;
			contactConstraint.invIB = iB;
			contactConstraint.invMassB = mB;
			contactConstraint.rollingMass = b3InvertMatrix(b3AddMM(iA, iB));
			contactConstraint.softness =
				(contact.flags & b3_contactStaticFlag) != 0 ? context.staticSoftness : context.contactSoftness;
			contactConstraint.friction = contact.friction;
			contactConstraint.restitution = contact.restitution;
			contactConstraint.rollingResistance = contact.rollingResistance;

			// Only sample contact point normal velocity if needed.
			let haveRestitution = contact.restitution > 0.0;
			let hitEvents = (contact.flags & b3_simEnableHitEvent) != 0;
			let sampleVelocity = haveRestitution || hitEvents;
			anyRestitution = anyRestitution || haveRestitution;

			let vA = b3Vec3_zero;
			let wA = b3Vec3_zero;
			let vB = b3Vec3_zero;
			let wB = b3Vec3_zero;

			if (sampleVelocity) {
				if (indexA != B3_NULL_INDEX) {
					vA = states[indexA].linearVelocity;
					wA = states[indexA].angularVelocity;
				}

				if (indexB != B3_NULL_INDEX) {
					vB = states[indexB].linearVelocity;
					wB = states[indexB].angularVelocity;
				}
			}

			let manifoldStart = specs[localIndex].manifoldStart;
			contactConstraint.constraints = manifoldStart;

			for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
				ref manifold = contact.manifolds[manifoldIndex];
				ref constraint = manifoldBase[manifoldStart + manifoldIndex];
				let pointCount = manifold.pointCount;
				let normal = manifold.normal;
				let tangent1 = b3Perp(normal);
				let tangent2 = b3Cross(tangent1, normal);

				constraint.pointCount = pointCount;
				constraint.normal = normal;
				constraint.tangent1 = tangent1;
				constraint.tangent2 = tangent2;

				// Stiffer for static contacts to avoid bodies getting pushed through the ground
				constraint.tangentVelocity1 = b3Dot(contact.tangentVelocity, constraint.tangent1);
				constraint.tangentVelocity2 = b3Dot(contact.tangentVelocity, constraint.tangent2);

				let centerA = b3Vec3_zero;
				let centerB = b3Vec3_zero;
				let totalFrictionWeight: f32 = 0.0;

				for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
					ref cp = constraint.points[pointIndex];

					// Copy data from manifold point
					ref mp = manifold.points[pointIndex];
					cp.rA = mp.anchorA;
					cp.rB = mp.anchorB;

					let s = mp.separation;
					cp.baseSeparation = s - b3Dot(b3Sub(cp.rB, cp.rA), normal);
					cp.normalImpulse = warmStartScale * mp.normalImpulse;
					cp.totalNormalImpulse = 0.0;
					cp.restitutionImpulse = 0.0;

					let rA = cp.rA;
					let rB = cp.rB;

					let rnA = b3Cross(rA, normal);
					let rnB = b3Cross(rB, normal);
					let kNormal = mA + mB + b3Dot(rnA, b3MulMV(iA, rnA)) + b3Dot(rnB, b3MulMV(iB, rnB));
					cp.normalMass = kNormal > 0.0 ? <f32>1.0 / kNormal : <f32>0.0;

					// Only compute the normal velocity terms if needed.
					if (sampleVelocity) {
						let vrA = b3Add(vA, b3Cross(wA, rA));
						let vrB = b3Add(vB, b3Cross(wB, rB));
						let vn = b3Dot(normal, b3Sub(vrB, vrA));

						cp.relativeVelocity = vn;
						mp.normalVelocity = hitEvents ? vn : <f32>0.0;
					} else {
						cp.relativeVelocity = 0.0;
						mp.normalVelocity = 0.0;
					}

					// C0 friction center decay. Needed to prevent spinning top drift (GyroscopicPrecession sample).
					// Contacts with separation greater than twice the speculative distance only matter for CCD and
					// should not contribute to the friction center. They are not important for jitter reduction. Closer
					// points may begin to touch on and off, so the friction center needs to move smoothly.
					// Epsilon to avoid a branch below (or divide by zero). Small enough to get washed out normally.
					//
					// Dirk suggested to weight the points by their normal impulse. This would use the total impulse from
					// the previous time step. I suspect this could jump around if the active point set jitters, leading
					// to a limit cycle.
					let weight = b3ClampFloat(2.0 - s * invTau, B3_MIN_FRICTION_WEIGHT, 1.0);
					centerA = b3MulAdd(centerA, weight, rA);
					centerB = b3MulAdd(centerB, weight, rB);
					totalFrictionWeight += weight;
				}

				let invWeight: f32 = 1.0 / totalFrictionWeight;
				centerA = b3MulSV(invWeight, centerA);
				centerB = b3MulSV(invWeight, centerB);
				constraint.centerA = centerA;
				constraint.centerB = centerB;

				for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
					ref cp = constraint.points[pointIndex];
					cp.leverArm = b3Distance(cp.rA, centerA);
				}

				let rtA1 = b3Cross(centerA, tangent1);
				let rtA2 = b3Cross(centerA, tangent2);
				let rtB1 = b3Cross(centerB, tangent1);
				let rtB2 = b3Cross(centerB, tangent2);

				{
					let k = b3Matrix2();
					k.cx.x = mA + mB + b3Dot(rtA1, b3MulMV(iA, rtA1)) + b3Dot(rtB1, b3MulMV(iB, rtB1));
					k.cy.y = mA + mB + b3Dot(rtA2, b3MulMV(iA, rtA2)) + b3Dot(rtB2, b3MulMV(iB, rtB2));
					k.cy.x = b3Dot(rtA1, b3MulMV(iA, rtA2)) + b3Dot(rtB1, b3MulMV(iB, rtB2));
					k.cx.y = k.cy.x;

					constraint.tangentMass = b3Invert2(k);
					constraint.frictionImpulse.x = warmStartScale * b3Dot(manifold.frictionImpulse, tangent1);
					constraint.frictionImpulse.y = warmStartScale * b3Dot(manifold.frictionImpulse, tangent2);
				}

				{
					let k = b3Dot(normal, b3MulMV(b3AddMM(iA, iB), normal));
					constraint.twistMass = k > 0.0 ? <f32>1.0 / k : <f32>0.0;
					constraint.twistImpulse = warmStartScale * manifold.twistImpulse;
				}

				{
					constraint.rollingImpulse = b3MulSV(warmStartScale, manifold.rollingImpulse);
				}
			}
		}

		// Advance to next color
		colorIndex += 1;
	}

	if (anyRestitution) {
		context.anyRestitution.value = 1;
	}
}

export function b3WarmStartContacts_Mesh(block: b3SolverBlock, context: b3StepContext): void {
	let world = context.world;
	let color = world.constraintGraph.colors[block.colorIndex];
	let awakeSet = world.solverSets[b3_awakeSet];
	let states = awakeSet.bodyStates;
	let constraints = color.contactConstraints;
	let manifoldConstraints = color.manifoldConstraints;

	// This is a dummy state to represent a static body because static bodies don't have a solver body.
	let dummyState = b3_identityBodyState;

	let startIndex = block.startIndex;
	let endIndex = startIndex + <i32>block.count;

	for (let constraintIndex = startIndex; constraintIndex < endIndex; ++constraintIndex) {
		ref contactConstraint = constraints[constraintIndex];

		let linearImpulse = b3Vec3_zero;
		let angularImpulseA = b3Vec3_zero;
		let angularImpulseB = b3Vec3_zero;

		let manifoldCount = contactConstraint.manifoldCount;
		for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
			ref constraint = manifoldConstraints[contactConstraint.constraints + manifoldIndex];

			// Normal impulses
			let normal = constraint.normal;
			let totalNormalImpulse: f32 = 0.0;
			let momentA = b3Vec3_zero;
			let momentB = b3Vec3_zero;

			let pointCount = constraint.pointCount;
			for (let j = 0; j < pointCount; ++j) {
				ref cp = constraint.points[j];

				// fixed anchors
				totalNormalImpulse += cp.normalImpulse;
				momentA = b3MulAdd(momentA, cp.normalImpulse, cp.rA);
				momentB = b3MulAdd(momentB, cp.normalImpulse, cp.rB);
			}

			// Central friction
			let frictionImpulse = b3MulSV(constraint.frictionImpulse.x, constraint.tangent1);
			frictionImpulse = b3MulAdd(frictionImpulse, constraint.frictionImpulse.y, constraint.tangent2);

			linearImpulse = b3Add(linearImpulse, b3MulAdd(frictionImpulse, totalNormalImpulse, normal));

			// Central twist friction
			let twistImpulse = b3MulSV(constraint.twistImpulse, normal);

			let angularA = b3Add(b3Add(b3Cross(momentA, normal), b3Cross(constraint.centerA, frictionImpulse)), twistImpulse);
			let angularB = b3Add(b3Add(b3Cross(momentB, normal), b3Cross(constraint.centerB, frictionImpulse)), twistImpulse);

			// Rolling resistance
			angularImpulseA = b3Add(angularImpulseA, b3Add(angularA, constraint.rollingImpulse));
			angularImpulseB = b3Add(angularImpulseB, b3Add(angularB, constraint.rollingImpulse));
		}

		let indexA = contactConstraint.indexA;
		let indexB = contactConstraint.indexB;

		// The dummy state has no dynamic flag, so only real states are written.
		let stateA = indexA == B3_NULL_INDEX ? dummyState : states[indexA];
		let stateB = indexB == B3_NULL_INDEX ? dummyState : states[indexB];

		if ((stateA.flags & b3_dynamicFlag) != 0) {
			states[indexA].linearVelocity = b3MulSub(stateA.linearVelocity, contactConstraint.invMassA, linearImpulse);
			states[indexA].angularVelocity = b3Sub(stateA.angularVelocity, b3MulMV(contactConstraint.invIA, angularImpulseA));
		}

		if ((stateB.flags & b3_dynamicFlag) != 0) {
			states[indexB].linearVelocity = b3MulAdd(stateB.linearVelocity, contactConstraint.invMassB, linearImpulse);
			states[indexB].angularVelocity = b3Add(stateB.angularVelocity, b3MulMV(contactConstraint.invIB, angularImpulseB));
		}
	}
}

// Merged normal and friction loops. This is much more stable for the Jenga stack.
// Solve the non-penetration constraints with the soft bias. No friction and no restitution.
export function b3PushContacts_Mesh(block: b3SolverBlock, context: b3StepContext): void {
	let world = context.world;
	let color = world.constraintGraph.colors[block.colorIndex];
	let contactConstraints = color.contactConstraints;
	let manifoldConstraints = color.manifoldConstraints;
	let states = context.states;

	// This is a dummy state to represent a static body because static bodies have no solver body.
	let dummyState = b3_identityBodyState;

	// The last block might not be full
	let startIndex = block.startIndex;
	let endIndex = startIndex + <i32>block.count;

	let inv_h = context.inv_h;
	let contactSpeed = context.world.contactSpeed;

	for (let i = startIndex; i < endIndex; ++i) {
		ref contactConstraint = contactConstraints[i];
		let manifoldCount = contactConstraint.manifoldCount;

		let indexA = contactConstraint.indexA;
		let indexB = contactConstraint.indexB;

		let mA = contactConstraint.invMassA;
		let iA = contactConstraint.invIA;
		let mB = contactConstraint.invMassB;
		let iB = contactConstraint.invIB;

		let stateA = indexA == B3_NULL_INDEX ? dummyState : states[indexA];
		let vA = stateA.linearVelocity;
		let wA = stateA.angularVelocity;
		let dqA = stateA.deltaRotation;

		let stateB = indexB == B3_NULL_INDEX ? dummyState : states[indexB];
		let vB = stateB.linearVelocity;
		let wB = stateB.angularVelocity;
		let dqB = stateB.deltaRotation;

		let dp = b3Sub(stateB.deltaPosition, stateA.deltaPosition);
		let softness = contactConstraint.softness;

		for (let j = 0; j < manifoldCount; ++j) {
			ref constraint = manifoldConstraints[contactConstraint.constraints + j];

			let pointCount = constraint.pointCount;
			let normal = constraint.normal;

			for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
				ref cp = constraint.points[pointIndex];

				// Fixed anchor points for applying impulses
				let rA = cp.rA;
				let rB = cp.rB;

				// compute current separation
				// this is subject to round-off error if the anchor is far from the body center of mass
				let ds = b3Add(dp, b3Sub(b3RotateVector(dqB, rB), b3RotateVector(dqA, rA)));
				let s = b3Dot(ds, normal) + cp.baseSeparation;

				let velocityBias: f32;
				let massScale: f32;
				let impulseScale: f32;
				if (s > 0.0) {
					// speculative bias is positive
					velocityBias = s * inv_h;
					massScale = 1.0;
					impulseScale = 0.0;
				} else {
					// overlap bias is negative
					velocityBias = b3MaxFloat(softness.massScale * softness.biasRate * s, -contactSpeed);
					massScale = softness.massScale;
					impulseScale = softness.impulseScale;
				}

				// relative normal velocity at contact
				let vrA = b3Add(vA, b3Cross(wA, rA));
				let vrB = b3Add(vB, b3Cross(wB, rB));
				let vn = b3Dot(b3Sub(vrB, vrA), normal);

				// incremental normal impulse
				let deltaImpulse = -cp.normalMass * (massScale * vn + velocityBias) - impulseScale * cp.normalImpulse;

				// clamp the accumulated impulse
				let newImpulse = b3MaxFloat(cp.normalImpulse + deltaImpulse, 0.0);
				deltaImpulse = newImpulse - cp.normalImpulse;
				cp.normalImpulse = newImpulse;

				// apply normal impulse
				let P = b3MulSV(deltaImpulse, normal);
				vA = b3MulSub(vA, mA, P);
				wA = b3Sub(wA, b3MulMV(iA, b3Cross(rA, P)));

				vB = b3MulAdd(vB, mB, P);
				wB = b3Add(wB, b3MulMV(iB, b3Cross(rB, P)));
			}
		}

		if ((stateA.flags & b3_dynamicFlag) != 0) {
			states[indexA].linearVelocity = vA;
			states[indexA].angularVelocity = wA;
		}

		if ((stateB.flags & b3_dynamicFlag) != 0) {
			states[indexB].linearVelocity = vB;
			states[indexB].angularVelocity = wB;
		}
	}
}

// Solve contacts: normal, friction and rolling resistance.
export function b3SolveContacts_Mesh(block: b3SolverBlock, context: b3StepContext): void {
	let world = context.world;
	let color = world.constraintGraph.colors[block.colorIndex];
	let contactConstraints = color.contactConstraints;
	let manifoldConstraints = color.manifoldConstraints;
	let states = context.states;

	// This is a dummy state to represent a static body because static bodies have no solver body.
	let dummyState = b3_identityBodyState;

	// The last block might not be full
	let startIndex = block.startIndex;
	let endIndex = startIndex + <i32>block.count;

	let inv_h = context.inv_h;

	for (let i = startIndex; i < endIndex; ++i) {
		ref contactConstraint = contactConstraints[i];
		let manifoldCount = contactConstraint.manifoldCount;

		let indexA = contactConstraint.indexA;
		let indexB = contactConstraint.indexB;

		let mA = contactConstraint.invMassA;
		let iA = contactConstraint.invIA;
		let mB = contactConstraint.invMassB;
		let iB = contactConstraint.invIB;

		let stateA = indexA == B3_NULL_INDEX ? dummyState : states[indexA];
		let vA = stateA.linearVelocity;
		let wA = stateA.angularVelocity;
		let dqA = stateA.deltaRotation;

		let stateB = indexB == B3_NULL_INDEX ? dummyState : states[indexB];
		let vB = stateB.linearVelocity;
		let wB = stateB.angularVelocity;
		let dqB = stateB.deltaRotation;

		let dp = b3Sub(stateB.deltaPosition, stateA.deltaPosition);
		let friction = contactConstraint.friction;
		let rollingResistance = contactConstraint.rollingResistance;

		for (let j = 0; j < manifoldCount; ++j) {
			ref constraint = manifoldConstraints[contactConstraint.constraints + j];

			let pointCount = constraint.pointCount;
			let normal = constraint.normal;

			let totalNormalImpulse: f32 = 0.0;
			let totalTwistLimit: f32 = 0.0;

			for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
				ref cp = constraint.points[pointIndex];

				// Fixed anchor points for applying impulses
				let rA = cp.rA;
				let rB = cp.rB;

				// compute current separation
				// this is subject to round-off error if the anchor is far from the body center of mass
				let ds = b3Add(dp, b3Sub(b3RotateVector(dqB, rB), b3RotateVector(dqA, rA)));
				let s = b3Dot(ds, normal) + cp.baseSeparation;

				// speculative bias, zero when overlapped
				let velocityBias: f32 = s > 0.0 ? s * inv_h : <f32>0.0;

				// relative normal velocity at contact
				let vrA = b3Add(vA, b3Cross(wA, rA));
				let vrB = b3Add(vB, b3Cross(wB, rB));
				let vn = b3Dot(b3Sub(vrB, vrA), normal);

				// incremental normal impulse
				let deltaImpulse = -cp.normalMass * (vn + velocityBias);

				// clamp the accumulated impulse
				let newImpulse = b3MaxFloat(cp.normalImpulse + deltaImpulse, 0.0);
				deltaImpulse = newImpulse - cp.normalImpulse;
				cp.normalImpulse = newImpulse;
				cp.totalNormalImpulse += newImpulse;
				totalNormalImpulse += newImpulse;
				totalTwistLimit += cp.leverArm * newImpulse;

				// apply normal impulse
				let P = b3MulSV(deltaImpulse, normal);
				vA = b3MulSub(vA, mA, P);
				wA = b3Sub(wA, b3MulMV(iA, b3Cross(rA, P)));

				vB = b3MulAdd(vB, mB, P);
				wB = b3Add(wB, b3MulMV(iB, b3Cross(rB, P)));
			}

			// Central twist friction
			{
				let twistSpeed = b3Dot(constraint.normal, b3Sub(wB, wA));
				let maxImpulse = friction * totalTwistLimit;
				let deltaImpulse = -constraint.twistMass * twistSpeed;
				let oldImpulse = constraint.twistImpulse;
				constraint.twistImpulse = b3ClampFloat(oldImpulse + deltaImpulse, -maxImpulse, maxImpulse);
				deltaImpulse = constraint.twistImpulse - oldImpulse;

				wA = b3Sub(wA, b3MulMV(iA, b3MulSV(deltaImpulse, constraint.normal)));
				wB = b3Add(wB, b3MulMV(iB, b3MulSV(deltaImpulse, constraint.normal)));
			}

			// Rolling resistance
			if (rollingResistance > 0.0) {
				let deltaImpulse = b3Neg(b3MulMV(contactConstraint.rollingMass, b3Sub(wB, wA)));
				let oldImpulse = constraint.rollingImpulse;
				constraint.rollingImpulse = b3Add(oldImpulse, deltaImpulse);

				let maxImpulse = rollingResistance * totalNormalImpulse;
				let magSqr = b3Dot(constraint.rollingImpulse, constraint.rollingImpulse);
				if (magSqr > maxImpulse * maxImpulse + FLT_EPSILON) {
					constraint.rollingImpulse = b3MulSV(maxImpulse / Mathf.sqrt(magSqr), constraint.rollingImpulse);
				}

				deltaImpulse = b3Sub(constraint.rollingImpulse, oldImpulse);

				wA = b3Sub(wA, b3MulMV(iA, deltaImpulse));
				wB = b3Add(wB, b3MulMV(iB, deltaImpulse));
			}

			// Central friction
			{
				let tangent1 = constraint.tangent1;
				let tangent2 = constraint.tangent2;

				// Fixed anchor points for applying impulses
				let rA = constraint.centerA;
				let rB = constraint.centerB;

				// Relative tangent velocity at contact
				let vrA = b3Add(vA, b3Cross(wA, rA));
				let vrB = b3Add(vB, b3Cross(wB, rB));
				let vr = b3Sub(vrB, vrA);
				let vt = b3Vec2(
					b3Dot(vr, tangent1) - constraint.tangentVelocity1,
					b3Dot(vr, tangent2) - constraint.tangentVelocity2
				);

				// Incremental tangent impulse
				let tm = b3MulMV2(constraint.tangentMass, vt);
				let deltaImpulse = b3Vec2(-tm.x, -tm.y);
				let newImpulse = b3Vec2(
					constraint.frictionImpulse.x + deltaImpulse.x,
					constraint.frictionImpulse.y + deltaImpulse.y
				);

				let maxImpulse = friction * totalNormalImpulse;

				// Clamp the accumulated impulse
				let lengthSquared = b3Dot2(newImpulse, newImpulse);
				if (lengthSquared > maxImpulse * maxImpulse) {
					let scale = maxImpulse / Mathf.sqrt(lengthSquared);
					newImpulse.x *= scale;
					newImpulse.y *= scale;
				}
				deltaImpulse = b3Sub2(newImpulse, constraint.frictionImpulse);
				constraint.frictionImpulse = newImpulse;

				// Apply delta impulse
				let P = b3Blend2(deltaImpulse.x, tangent1, deltaImpulse.y, tangent2);
				vA = b3MulSub(vA, mA, P);
				wA = b3Sub(wA, b3MulMV(iA, b3Cross(rA, P)));
				vB = b3MulAdd(vB, mB, P);
				wB = b3Add(wB, b3MulMV(iB, b3Cross(rB, P)));
			}
		}

		if ((stateA.flags & b3_dynamicFlag) != 0) {
			states[indexA].linearVelocity = vA;
			states[indexA].angularVelocity = wA;
		}

		if ((stateB.flags & b3_dynamicFlag) != 0) {
			states[indexB].linearVelocity = vB;
			states[indexB].angularVelocity = wB;
		}
	}
}

export function b3ApplyRestitution_Mesh(block: b3SolverBlock, context: b3StepContext): void {
	let world = context.world;
	let color = world.constraintGraph.colors[block.colorIndex];
	let contactConstraints = color.contactConstraints;
	let manifoldConstraints = color.manifoldConstraints;
	let states = context.states;

	let threshold = world.restitutionThreshold;
	let inv_h = context.inv_h;
	let propagate = world.enableRestitutionPropagation;

	let dummyState = b3_identityBodyState;

	let startIndex = block.startIndex;
	let endIndex = startIndex + <i32>block.count;

	for (let i = startIndex; i < endIndex; ++i) {
		ref contactConstraint = contactConstraints[i];
		let restitution = contactConstraint.restitution;
		if (propagate == false && restitution == 0.0) {
			continue;
		}

		let manifoldCount = contactConstraint.manifoldCount;

		let indexA = contactConstraint.indexA;
		let indexB = contactConstraint.indexB;

		let mA = contactConstraint.invMassA;
		let iA = contactConstraint.invIA;
		let mB = contactConstraint.invMassB;
		let iB = contactConstraint.invIB;

		let stateA = indexA == B3_NULL_INDEX ? dummyState : states[indexA];
		let vA = stateA.linearVelocity;
		let wA = stateA.angularVelocity;
		let dqA = stateA.deltaRotation;

		let stateB = indexB == B3_NULL_INDEX ? dummyState : states[indexB];
		let vB = stateB.linearVelocity;
		let wB = stateB.angularVelocity;
		let dqB = stateB.deltaRotation;

		let dp = b3Sub(stateB.deltaPosition, stateA.deltaPosition);

		for (let j = 0; j < manifoldCount; ++j) {
			ref constraint = manifoldConstraints[contactConstraint.constraints + j];

			let pointCount = constraint.pointCount;
			let normal = constraint.normal;

			for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
				ref cp = constraint.points[pointIndex];

				let rA = cp.rA;
				let rB = cp.rB;

				// The total normal impulse is 0 for speculative points.
				let compressionImpulse = cp.totalNormalImpulse - cp.restitutionImpulse;
				let armed = restitution > 0.0 && cp.relativeVelocity < -threshold && compressionImpulse > 0.0;

				let velocityBias: f32;
				if (armed) {
					velocityBias = restitution * cp.relativeVelocity;
				} else {
					let ds = b3Add(dp, b3Sub(b3RotateVector(dqB, rB), b3RotateVector(dqA, rA)));
					let s = b3Dot(ds, normal) + cp.baseSeparation;

					velocityBias = s > 0.0 ? s * inv_h : <f32>0.0;
				}

				let vrA = b3Add(vA, b3Cross(wA, rA));
				let vrB = b3Add(vB, b3Cross(wB, rB));
				let vn = b3Dot(b3Sub(vrB, vrA), normal);

				let impulse = -cp.normalMass * (vn + velocityBias);

				let newImpulse = b3MaxFloat(cp.normalImpulse + impulse, 0.0);
				impulse = newImpulse - cp.normalImpulse;

				let approachImpulse = b3MinFloat(b3MaxFloat(-cp.normalMass * vn, 0.0), b3MaxFloat(impulse, 0.0));

				if (armed) {
					// Poisson kinetic restitution guarantees no energy gain.
					let allowance = restitution * (compressionImpulse + approachImpulse) - cp.restitutionImpulse;
					impulse = b3MinFloat(impulse, approachImpulse + b3MaxFloat(allowance, 0.0));
				}

				cp.normalImpulse += impulse;
				cp.restitutionImpulse += impulse - approachImpulse;
				cp.totalNormalImpulse += impulse;

				let P = b3MulSV(impulse, normal);
				vA = b3MulSub(vA, mA, P);
				wA = b3Sub(wA, b3MulMV(iA, b3Cross(rA, P)));

				vB = b3MulAdd(vB, mB, P);
				wB = b3Add(wB, b3MulMV(iB, b3Cross(rB, P)));
			}
		}

		if ((stateA.flags & b3_dynamicFlag) != 0) {
			states[indexA].linearVelocity = vA;
			states[indexA].angularVelocity = wA;
		}

		if ((stateB.flags & b3_dynamicFlag) != 0) {
			states[indexB].linearVelocity = vB;
			states[indexB].angularVelocity = wB;
		}
	}
}

// Don't need to use spans for colors for this because the constraint to contact association
// is already linked by pointer.
export function b3StoreImpulses_Mesh(block: b3SolverBlock, context: b3StepContext, workerIndex: i32): void {
	let world = context.world;

	B3_ASSERT(block.blockType == b3_overflowBlock);
	let overflow = world.constraintGraph.colors[B3_OVERFLOW_INDEX];
	let spans = context.overflowSpans;
	let base = overflow.contactConstraints;
	let manifoldBase = overflow.manifoldConstraints;

	let taskContext = world.taskContexts[workerIndex];
	let hitEventBitSet = taskContext.hitEventBitSet;
	let hasHitEvents = taskContext.hasHitEvents;
	let negHitThreshold = -world.hitEventThreshold;

	let index = block.startIndex;
	let endIndex = block.startIndex + <i32>block.count;

	// Find color for start index. Linear search but fast.
	let colorIndex = 0;
	while (spans[colorIndex + 1].start <= index) {
		colorIndex += 1;
	}

	// Loop over block
	while (index < endIndex) {
		let colorStart = spans[colorIndex].start;
		let colorEndIndex = b3MinInt(spans[colorIndex + 1].start, endIndex);

		// Loop over color
		for (; index < colorEndIndex; ++index) {
			ref contactConstraint = base[index];

			let localIndex = index - colorStart;
			B3_ASSERT(0 <= localIndex && localIndex < spans[colorIndex].count);

			// Having this contact pointer simplifies impulse storage
			let contact = world.contacts[contactConstraint.contact];

			let manifoldCount = contactConstraint.manifoldCount;
			B3_ASSERT(manifoldCount == contact.manifoldCount);

			let checkHitEvents = (contact.flags & b3_simEnableHitEvent) != 0;
			let flagged = false;

			for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
				ref manifold = contact.manifolds[manifoldIndex];
				ref constraint = manifoldBase[contactConstraint.constraints + manifoldIndex];
				manifold.twistImpulse = constraint.twistImpulse;
				manifold.frictionImpulse = b3Blend2(
					constraint.frictionImpulse.x,
					constraint.tangent1,
					constraint.frictionImpulse.y,
					constraint.tangent2
				);
				manifold.rollingImpulse = constraint.rollingImpulse;

				let count = constraint.pointCount;
				B3_ASSERT(count == manifold.pointCount);
				for (let pointIndex = 0; pointIndex < count; ++pointIndex) {
					ref cp = constraint.points[pointIndex];
					ref mp = manifold.points[pointIndex];
					mp.normalImpulse = cp.normalImpulse;
					mp.totalNormalImpulse = cp.totalNormalImpulse;

					if (checkHitEvents && flagged == false && mp.normalVelocity < negHitThreshold && mp.totalNormalImpulse > 0.0) {
						b3SetBit(hitEventBitSet, <u32>contact.contactId);
						hasHitEvents = true;
						flagged = true;
					}
				}
			}
		}

		// Advance to next color
		colorIndex += 1;
	}

	taskContext.hasHitEvents = hasHitEvents;
}

function b3MakeOverflowBlock(count: u16): b3SolverBlock {
	let block = b3SolverBlock();
	block.startIndex = 0;
	block.count = count;
	block.blockType = <u8>b3_overflowBlock;
	block.colorIndex = <u8>B3_OVERFLOW_INDEX;
	return block;
}

export function b3PrepareContacts_Overflow(context: b3StepContext): void {
	let graph = context.graph;
	let color = graph.colors[B3_OVERFLOW_INDEX];

	B3_ASSERT(color.contacts.length <= UINT16_MAX);
	let count = <u16>color.contacts.length;
	if (count == 0) {
		return;
	}

	b3PrepareContacts_Mesh(b3MakeOverflowBlock(count), context);
}

export function b3WarmStartContacts_Overflow(context: b3StepContext): void {
	let graph = context.graph;
	let color = graph.colors[B3_OVERFLOW_INDEX];

	let count = <u16>color.contacts.length;
	if (count == 0) {
		return;
	}

	b3WarmStartContacts_Mesh(b3MakeOverflowBlock(count), context);
}

export function b3PushContacts_Overflow(context: b3StepContext): void {
	let graph = context.graph;
	let color = graph.colors[B3_OVERFLOW_INDEX];

	let count = <u16>color.contacts.length;
	if (count == 0) {
		return;
	}

	b3PushContacts_Mesh(b3MakeOverflowBlock(count), context);
}

export function b3SolveContacts_Overflow(context: b3StepContext): void {
	let graph = context.graph;
	let color = graph.colors[B3_OVERFLOW_INDEX];

	let count = <u16>color.contacts.length;
	if (count == 0) {
		return;
	}

	b3SolveContacts_Mesh(b3MakeOverflowBlock(count), context);
}

export function b3ApplyRestitution_Overflow(context: b3StepContext): void {
	let graph = context.graph;
	let color = graph.colors[B3_OVERFLOW_INDEX];

	let count = <u16>color.contacts.length;
	if (count == 0) {
		return;
	}

	b3ApplyRestitution_Mesh(b3MakeOverflowBlock(count), context);
}

export function b3StoreImpulses_Overflow(context: b3StepContext): void {
	let graph = context.graph;
	let color = graph.colors[B3_OVERFLOW_INDEX];

	let count = <u16>color.contacts.length;
	if (count == 0) {
		return;
	}

	b3StoreImpulses_Mesh(b3MakeOverflowBlock(count), context, 0);
}

// world.simdWidth is always 4, so the width dispatchers call the W4 versions.

export function b3PrepareContacts_Convex(block: b3SolverBlock, context: b3StepContext): void {
	b3PrepareContacts_ConvexW4(block, context);
}

export function b3WarmStartContacts_Convex(block: b3SolverBlock, context: b3StepContext): void {
	b3WarmStartContacts_ConvexW4(block, context);
}

export function b3PushContacts_Convex(block: b3SolverBlock, context: b3StepContext): void {
	b3PushContacts_ConvexW4(block, context);
}

export function b3SolveContacts_Convex(block: b3SolverBlock, context: b3StepContext): void {
	b3SolveContacts_ConvexW4(block, context);
}

export function b3ApplyRestitution_Convex(block: b3SolverBlock, context: b3StepContext): void {
	b3ApplyRestitution_ConvexW4(block, context);
}

export function b3StoreImpulses_Convex(block: b3SolverBlock, context: b3StepContext, workerIndex: i32): void {
	b3StoreImpulses_ConvexW4(block, context, workerIndex);
}

export function b3PrepareContacts_MeshWide(block: b3SolverBlock, context: b3StepContext): void {
	b3PrepareContacts_MeshWideW4(block, context);
}

export function b3WarmStartContacts_MeshWide(block: b3SolverBlock, context: b3StepContext): void {
	b3WarmStartContacts_MeshWideW4(block, context);
}

export function b3PushContacts_MeshWide(block: b3SolverBlock, context: b3StepContext): void {
	b3PushContacts_MeshWideW4(block, context);
}

export function b3SolveContacts_MeshWide(block: b3SolverBlock, context: b3StepContext): void {
	b3SolveContacts_MeshWideW4(block, context);
}

export function b3ApplyRestitution_MeshWide(block: b3SolverBlock, context: b3StepContext): void {
	b3ApplyRestitution_MeshWideW4(block, context);
}

export function b3StoreImpulses_MeshWide(block: b3SolverBlock, context: b3StepContext, workerIndex: i32): void {
	b3StoreImpulses_MeshWideW4(block, context, workerIndex);
}
