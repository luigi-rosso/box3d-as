// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/contact_solver_wide.inl at B3_SIMD_WIDTH 4, as wide4.c compiles it with
// BOX3D_DISABLE_SIMD. The wide types live in lib/contact_solver.as.

import { B3_ASSERT, B3_NULL_INDEX, FLT_EPSILON, FLT_MIN } from "./core";
import { b3Matrix3, b3Mat3_zero, b3MaxInt, b3MinInt, b3Vec3 } from "./math_functions";
import { b3Manifold, b3ManifoldPoint } from "./collision";
import { B3_MIN_FRICTION_WEIGHT, B3_SPECULATIVE_DISTANCE } from "./constants";
import { b3SetBit } from "./bitset";
import {
	b3FloatW,
	b3Vec3W,
	b3ZeroW,
	b3SplatW,
	b3SetW,
	b3NegW,
	b3AbsW,
	b3AddW,
	b3SubW,
	b3MulW,
	b3DivW,
	b3SqrtW,
	b3MulAddW,
	b3MinW,
	b3MaxW,
	b3AndW,
	b3OrW,
	b3SoftMaskW,
	b3GreaterThanW,
	b3LessThanW,
	b3AllZeroW,
	b3AnyTrueW,
	b3BlendW,
	b3MulSVW,
	b3MulSubSVW,
	b3MulAddSVW,
	b3SubVW,
	b3AddVW,
	b3DotW,
	b3CrossW,
	b3AddV4,
	b3SubV4,
	b3MulV4,
	b3DivV4,
	b3SqrtV4,
	b3NegV4,
	b3MulAddV4,
	b3MulSubV4,
	b3MinV4,
	b3MaxV4,
	b3BlendV4,
	b3MaskV4,
	b3AllZeroV4,
	b3DotV4,
	b3CrossV4,
	b3MulRowV4,
	b3Transpose0V4,
	b3Transpose1V4,
	b3Transpose2V4,
	b3Transpose3V4,
	b3AbsV4,
	b3SetV4,
	b3LaneV4,
} from "./simd";
import { b3BodySim, b3BodyState, b3DecodeAwakeIndex, b3_dynamicFlag, b3_identityBodyState } from "./body";
import { b3Contact, b3_simEnableHitEvent } from "./contact";
import { b3SolverBlock, b3StepContext } from "./solver";
import {
	b3Vec2W,
	b3QuatW,
	b3SymMatrix2W,
	b3SymMatrix3W,
	b3Matrix3W,
	b3ContactManifoldWide,
	b3ContactConstraintWide,
	b3ContactConstraintPointWide,
	b3BodyStateW,
} from "./contact_solver";

// a + b
function b3AddV2W(a: b3Vec2W, b: b3Vec2W): b3Vec2W {
	let r = b3Vec2W();
	r.x = b3AddW(a.x, b.x);
	r.y = b3AddW(a.y, b.y);
	return r;
}

// m * a
function b3MulMV2W(m: b3SymMatrix2W, a: b3Vec2W): b3Vec2W {
	let b = b3Vec2W();
	b.x = b3AddW(b3MulW(m.cxx, a.x), b3MulW(m.cxy, a.y));
	b.y = b3AddW(b3MulW(m.cxy, a.x), b3MulW(m.cyy, a.y));
	return b;
}

// m * a
function b3MulMVW(m: b3SymMatrix3W, a: b3Vec3W): b3Vec3W {
	let b = b3Vec3W(
		b3AddW(b3MulW(m.cxx, a.X), b3AddW(b3MulW(m.cxy, a.Y), b3MulW(m.cxz, a.Z))),
		b3AddW(b3MulW(m.cxy, a.X), b3AddW(b3MulW(m.cyy, a.Y), b3MulW(m.cyz, a.Z))),
		b3AddW(b3MulW(m.cxz, a.X), b3AddW(b3MulW(m.cyz, a.Y), b3MulW(m.czz, a.Z)))
	);

	return b;
}

// a - m * b
function b3MulSubMVW(a: b3Vec3W, m: b3SymMatrix3W, b: b3Vec3W): b3Vec3W {
	let c = b3Vec3W(
		b3AddW(b3MulW(m.cxx, b.X), b3AddW(b3MulW(m.cxy, b.Y), b3MulW(m.cxz, b.Z))),
		b3AddW(b3MulW(m.cxy, b.X), b3AddW(b3MulW(m.cyy, b.Y), b3MulW(m.cyz, b.Z))),
		b3AddW(b3MulW(m.cxz, b.X), b3AddW(b3MulW(m.cyz, b.Y), b3MulW(m.czz, b.Z)))
	);

	return b3Vec3W(b3SubW(a.X, c.X), b3SubW(a.Y, c.Y), b3SubW(a.Z, c.Z));
}

// a + m * b
function b3MulAddMVW(a: b3Vec3W, m: b3SymMatrix3W, b: b3Vec3W): b3Vec3W {
	let c = b3Vec3W(
		b3AddW(b3MulW(m.cxx, b.X), b3AddW(b3MulW(m.cxy, b.Y), b3MulW(m.cxz, b.Z))),
		b3AddW(b3MulW(m.cxy, b.X), b3AddW(b3MulW(m.cyy, b.Y), b3MulW(m.cyz, b.Z))),
		b3AddW(b3MulW(m.cxz, b.X), b3AddW(b3MulW(m.cyz, b.Y), b3MulW(m.czz, b.Z)))
	);

	return b3Vec3W(b3AddW(a.X, c.X), b3AddW(a.Y, c.Y), b3AddW(a.Z, c.Z));
}

function b3MulM3VW(m: b3Matrix3W, a: b3Vec3W): b3Vec3W {
	let b = b3Vec3W(
		b3AddW(b3MulW(m.cx.X, a.X), b3AddW(b3MulW(m.cy.X, a.Y), b3MulW(m.cz.X, a.Z))),
		b3AddW(b3MulW(m.cx.Y, a.X), b3AddW(b3MulW(m.cy.Y, a.Y), b3MulW(m.cz.Y, a.Z))),
		b3AddW(b3MulW(m.cx.Z, a.X), b3AddW(b3MulW(m.cy.Z, a.Y), b3MulW(m.cz.Z, a.Z)))
	);

	return b;
}

function b3InvRotateVectorW(q: b3QuatW, a: b3Vec3W): b3Vec3W {
	let t = b3CrossW(q.V, a);
	t = b3Vec3W(b3AddW(t.X, t.X), b3AddW(t.Y, t.Y), b3AddW(t.Z, t.Z));
	let u = b3CrossW(q.V, t);
	return b3Vec3W(
		b3AddW(b3SubW(a.X, b3MulW(q.S, t.X)), u.X),
		b3AddW(b3SubW(a.Y, b3MulW(q.S, t.Y)), u.Y),
		b3AddW(b3SubW(a.Z, b3MulW(q.S, t.Z)), u.Z)
	);
}

// The B3_SIMD_NONE gather: indices are base 1, 0 for a static body.
function b3GatherBodies(states: Array<b3BodyState>, indices: FixedArray<i32, 4>): b3BodyStateW {
	let identity = b3_identityBodyState;

	let s1 = indices[0] == 0 ? identity : states[indices[0] - 1];
	let s2 = indices[1] == 0 ? identity : states[indices[1] - 1];
	let s3 = indices[2] == 0 ? identity : states[indices[2] - 1];
	let s4 = indices[3] == 0 ? identity : states[indices[3] - 1];

	let simdBody = b3BodyStateW();
	simdBody.v.X = b3FloatW(s1.linearVelocity.x, s2.linearVelocity.x, s3.linearVelocity.x, s4.linearVelocity.x);
	simdBody.v.Y = b3FloatW(s1.linearVelocity.y, s2.linearVelocity.y, s3.linearVelocity.y, s4.linearVelocity.y);
	simdBody.v.Z = b3FloatW(s1.linearVelocity.z, s2.linearVelocity.z, s3.linearVelocity.z, s4.linearVelocity.z);
	simdBody.w.X = b3FloatW(s1.angularVelocity.x, s2.angularVelocity.x, s3.angularVelocity.x, s4.angularVelocity.x);
	simdBody.w.Y = b3FloatW(s1.angularVelocity.y, s2.angularVelocity.y, s3.angularVelocity.y, s4.angularVelocity.y);
	simdBody.w.Z = b3FloatW(s1.angularVelocity.z, s2.angularVelocity.z, s3.angularVelocity.z, s4.angularVelocity.z);
	simdBody.dp.X = b3FloatW(s1.deltaPosition.x, s2.deltaPosition.x, s3.deltaPosition.x, s4.deltaPosition.x);
	simdBody.dp.Y = b3FloatW(s1.deltaPosition.y, s2.deltaPosition.y, s3.deltaPosition.y, s4.deltaPosition.y);
	simdBody.dp.Z = b3FloatW(s1.deltaPosition.z, s2.deltaPosition.z, s3.deltaPosition.z, s4.deltaPosition.z);
	simdBody.dq.V.X = b3FloatW(s1.deltaRotation.v.x, s2.deltaRotation.v.x, s3.deltaRotation.v.x, s4.deltaRotation.v.x);
	simdBody.dq.V.Y = b3FloatW(s1.deltaRotation.v.y, s2.deltaRotation.v.y, s3.deltaRotation.v.y, s4.deltaRotation.v.y);
	simdBody.dq.V.Z = b3FloatW(s1.deltaRotation.v.z, s2.deltaRotation.v.z, s3.deltaRotation.v.z, s4.deltaRotation.v.z);
	simdBody.dq.S = b3FloatW(s1.deltaRotation.s, s2.deltaRotation.s, s3.deltaRotation.s, s4.deltaRotation.s);

	return simdBody;
}

// This writes only the velocities back to the solver bodies
function b3ScatterBodies(states: Array<b3BodyState>, indices: FixedArray<i32, 4>, simdBody: b3BodyStateW): void {
	let index1 = indices[0] - 1;
	if (index1 != -1 && (states[index1].flags & b3_dynamicFlag) != 0) {
		states[index1].linearVelocity.x = simdBody.v.X.x;
		states[index1].linearVelocity.y = simdBody.v.Y.x;
		states[index1].linearVelocity.z = simdBody.v.Z.x;
		states[index1].angularVelocity.x = simdBody.w.X.x;
		states[index1].angularVelocity.y = simdBody.w.Y.x;
		states[index1].angularVelocity.z = simdBody.w.Z.x;
	}

	let index2 = indices[1] - 1;
	if (index2 != -1 && (states[index2].flags & b3_dynamicFlag) != 0) {
		states[index2].linearVelocity.x = simdBody.v.X.y;
		states[index2].linearVelocity.y = simdBody.v.Y.y;
		states[index2].linearVelocity.z = simdBody.v.Z.y;
		states[index2].angularVelocity.x = simdBody.w.X.y;
		states[index2].angularVelocity.y = simdBody.w.Y.y;
		states[index2].angularVelocity.z = simdBody.w.Z.y;
	}

	let index3 = indices[2] - 1;
	if (index3 != -1 && (states[index3].flags & b3_dynamicFlag) != 0) {
		states[index3].linearVelocity.x = simdBody.v.X.z;
		states[index3].linearVelocity.y = simdBody.v.Y.z;
		states[index3].linearVelocity.z = simdBody.v.Z.z;
		states[index3].angularVelocity.x = simdBody.w.X.z;
		states[index3].angularVelocity.y = simdBody.w.Y.z;
		states[index3].angularVelocity.z = simdBody.w.Z.z;
	}

	let index4 = indices[3] - 1;
	if (index4 != -1 && (states[index4].flags & b3_dynamicFlag) != 0) {
		states[index4].linearVelocity.x = simdBody.v.X.w;
		states[index4].linearVelocity.y = simdBody.v.Y.w;
		states[index4].linearVelocity.z = simdBody.v.Z.w;
		states[index4].angularVelocity.x = simdBody.w.X.w;
		states[index4].angularVelocity.y = simdBody.w.Y.w;
		states[index4].angularVelocity.z = simdBody.w.Z.w;
	}
}

// C points empty lanes at zeroed statics. Nothing writes to these.
const b3_zeroContact = new b3Contact();
const b3_zeroManifolds = new StaticArray<b3Manifold>(1);

// Lane scratch for C's pointer arrays, which a FixedArray cannot hold.
const b3_contactLanes = new StaticArray<b3Contact>(4);
const b3_manifoldLanes = new StaticArray<StaticArray<b3Manifold>>(4);

// Never read: the convex paths have a fixed manifold count.
const b3_noManifoldStarts = new Int32Array(0);

// One lane of a wide float, as C reads ( (const float*)&w )[lane].
@inline function b3LaneW(a: b3FloatW, lane: i32): f32 {
	if (lane == 0) {
		return a.x;
	}
	if (lane == 1) {
		return a.y;
	}
	if (lane == 2) {
		return a.z;
	}
	return a.w;
}

// B3_GATHER_LANES over body sims, with B3_NULL_INDEX standing for C's b3_zeroBodySim.
function b3GatherInvMassW(sims: Array<b3BodySim>, simLanes: FixedArray<i32, 4>): b3FloatW {
	let m0: f32 = simLanes[0] == B3_NULL_INDEX ? <f32>0.0 : sims[simLanes[0]].invMass;
	let m1: f32 = simLanes[1] == B3_NULL_INDEX ? <f32>0.0 : sims[simLanes[1]].invMass;
	let m2: f32 = simLanes[2] == B3_NULL_INDEX ? <f32>0.0 : sims[simLanes[2]].invMass;
	let m3: f32 = simLanes[3] == B3_NULL_INDEX ? <f32>0.0 : sims[simLanes[3]].invMass;
	return b3SetW(m0, m1, m2, m3);
}

// C transposes the first nine floats of each lane's invInertiaWorld; the result is these lane sets.
function b3GatherInvInertiaW(sims: Array<b3BodySim>, simLanes: FixedArray<i32, 4>): b3SymMatrix3W {
	let i0: b3Matrix3 = simLanes[0] == B3_NULL_INDEX ? b3Mat3_zero : sims[simLanes[0]].invInertiaWorld;
	let i1: b3Matrix3 = simLanes[1] == B3_NULL_INDEX ? b3Mat3_zero : sims[simLanes[1]].invInertiaWorld;
	let i2: b3Matrix3 = simLanes[2] == B3_NULL_INDEX ? b3Mat3_zero : sims[simLanes[2]].invInertiaWorld;
	let i3: b3Matrix3 = simLanes[3] == B3_NULL_INDEX ? b3Mat3_zero : sims[simLanes[3]].invInertiaWorld;

	let m = b3SymMatrix3W();
	m.cxx = b3SetW(i0.cx.x, i1.cx.x, i2.cx.x, i3.cx.x);
	m.cxy = b3SetW(i0.cx.y, i1.cx.y, i2.cx.y, i3.cx.y);
	m.cxz = b3SetW(i0.cx.z, i1.cx.z, i2.cx.z, i3.cx.z);
	m.cyy = b3SetW(i0.cy.y, i1.cy.y, i2.cy.y, i3.cy.y);
	m.cyz = b3SetW(i0.cy.z, i1.cy.z, i2.cy.z, i3.cy.z);
	m.czz = b3SetW(i0.cz.z, i1.cz.z, i2.cz.z, i3.cz.z);
	return m;
}

function b3AddSymW(a: b3SymMatrix3W, b: b3SymMatrix3W): b3SymMatrix3W {
	let r = b3SymMatrix3W();
	r.cxx = b3AddW(a.cxx, b.cxx);
	r.cxy = b3AddW(a.cxy, b.cxy);
	r.cxz = b3AddW(a.cxz, b.cxz);
	r.cyy = b3AddW(a.cyy, b.cyy);
	r.cyz = b3AddW(a.cyz, b.cyz);
	r.czz = b3AddW(a.czz, b.czz);
	return r;
}

function b3InvertSymW(m: b3SymMatrix3W): b3SymMatrix3W {
	let cxx = b3SubW(b3MulW(m.cyy, m.czz), b3MulW(m.cyz, m.cyz));
	let cxy = b3SubW(b3MulW(m.cxz, m.cyz), b3MulW(m.cxy, m.czz));
	let cxz = b3SubW(b3MulW(m.cxy, m.cyz), b3MulW(m.cxz, m.cyy));
	let cyy = b3SubW(b3MulW(m.cxx, m.czz), b3MulW(m.cxz, m.cxz));
	let cyz = b3SubW(b3MulW(m.cxy, m.cxz), b3MulW(m.cxx, m.cyz));
	let czz = b3SubW(b3MulW(m.cxx, m.cyy), b3MulW(m.cxy, m.cxy));

	let det = b3AddW(b3MulW(m.cxx, cxx), b3AddW(b3MulW(m.cxy, cxy), b3MulW(m.cxz, cxz)));
	let valid = b3GreaterThanW(b3AbsW(det), b3SplatW(1000.0 * FLT_MIN));
	let invDet = b3BlendW(b3ZeroW(), b3DivW(b3SplatW(1.0), det), valid);

	let r = b3SymMatrix3W();
	r.cxx = b3MulW(invDet, cxx);
	r.cxy = b3MulW(invDet, cxy);
	r.cxz = b3MulW(invDet, cxz);
	r.cyy = b3MulW(invDet, cyy);
	r.cyz = b3MulW(invDet, cyz);
	r.czz = b3MulW(invDet, czz);
	return r;
}

function b3PerpW(a: b3Vec3W): b3Vec3W {
	let zero = b3ZeroW();
	let half = b3SplatW(0.5);
	let mask = b3OrW(b3LessThanW(a.X, b3NegW(half)), b3GreaterThanW(a.X, half));

	let p = b3Vec3W();
	p.X = b3BlendW(zero, a.Y, mask);
	p.Y = b3BlendW(a.Z, b3NegW(a.X), mask);
	p.Z = b3BlendW(b3NegW(a.Y), zero, mask);

	let lengthSquared = b3DotW(p, p);
	let valid = b3GreaterThanW(lengthSquared, b3SplatW(1000.0 * FLT_MIN));
	let s = b3BlendW(zero, b3DivW(b3SplatW(1.0), b3SqrtW(lengthSquared)), valid);
	return b3MulSVW(s, p);
}

function b3IntsToFloatW(v: FixedArray<i32, 4>): b3FloatW {
	return b3SetW(<f32>v[0], <f32>v[1], <f32>v[2], <f32>v[3]);
}

// C stores the constraint stream [c1 m11 m12 c2 m21 ...] and walks it by byte offsets. The port splits it into a
// constraint array and a manifold array: a constraint at index ci owns the manifolds from its first manifold index.
// b3GetManifoldsW, b3GetConstraintW and b3NextConstraintW become that index arithmetic.

// Contact constraint data is stored in a heterogious stream. Constraints followed by an array of constraint manifolds.
// [c1 m11 m12 c2 m21 c3 m31 m32 m33 ... ]
// contactIds holds B3_NULL_INDEX where C passes a NULL contact.
function b3PrepareConstraintW(
	c: ref b3ContactConstraintWide,
	manifolds: StaticArray<b3ContactManifoldWide>,
	manifoldBase: i32,
	contactIds: FixedArray<i32, 4>,
	manifoldCount: i32,
	context: b3StepContext,
	warmStartScale: b3FloatW
): bool {
	let world = context.world;
	let sims = context.sims;
	let states = context.states;

	let zeroW = b3ZeroW();
	let oneW = b3SplatW(1.0);
	let twoW = b3SplatW(2.0);
	let invTau = b3SplatW(<f32>1.0 / B3_SPECULATIVE_DISTANCE());
	let minFrictionWeight = b3SplatW(B3_MIN_FRICTION_WEIGHT);
	let minDet = b3SplatW(1000.0 * FLT_MIN);

	let contactLanes = b3_contactLanes;
	let manifoldLanes = b3_manifoldLanes;
	let simLanesA: FixedArray<i32, 4>;
	let simLanesB: FixedArray<i32, 4>;
	let manifoldCounts: FixedArray<i32, 4>;
	let hitEventLanes = 0;
	let maxManifoldCount = 0;

	for (let lane = 0; lane < 4; ++lane) {
		let contactId = contactIds[lane];
		c.contacts[lane] = contactId;

		if (contactId != B3_NULL_INDEX) {
			let contact = world.contacts[contactId];
			let indexA = b3DecodeAwakeIndex(contact.encodedBodySimA);
			let indexB = b3DecodeAwakeIndex(contact.encodedBodySimB);

			c.indexA[lane] = indexA + 1;
			c.indexB[lane] = indexB + 1;
			manifoldCounts[lane] = contact.manifoldCount;

			contactLanes[lane] = contact;
			simLanesA[lane] = indexA;
			simLanesB[lane] = indexB;
			hitEventLanes |= (contact.flags & b3_simEnableHitEvent) != 0 ? 1 << lane : 0;
			maxManifoldCount = b3MaxInt(maxManifoldCount, contact.manifoldCount);
		} else {
			c.indexA[lane] = 0;
			c.indexB[lane] = 0;
			manifoldCounts[lane] = 0;

			contactLanes[lane] = b3_zeroContact;
			simLanesA[lane] = B3_NULL_INDEX;
			simLanesB[lane] = B3_NULL_INDEX;
		}
	}

	c.manifoldCount = manifoldCount;

	let mA = b3GatherInvMassW(sims, simLanesA);
	let mB = b3GatherInvMassW(sims, simLanesB);
	let iA = b3GatherInvInertiaW(sims, simLanesA);
	let iB = b3GatherInvInertiaW(sims, simLanesB);
	c.invMassA = mA;
	c.invMassB = mB;
	c.invIA = iA;
	c.invIB = iB;

	let cl0 = contactLanes[0];
	let cl1 = contactLanes[1];
	let cl2 = contactLanes[2];
	let cl3 = contactLanes[3];
	let tangentVelocity = b3Vec3W();
	c.friction = b3SetW(cl0.friction, cl1.friction, cl2.friction, cl3.friction);
	c.rollingResistance = b3SetW(cl0.rollingResistance, cl1.rollingResistance, cl2.rollingResistance, cl3.rollingResistance);
	c.restitution = b3SetW(cl0.restitution, cl1.restitution, cl2.restitution, cl3.restitution);
	tangentVelocity.X = b3SetW(cl0.tangentVelocity.x, cl1.tangentVelocity.x, cl2.tangentVelocity.x, cl3.tangentVelocity.x);
	tangentVelocity.Y = b3SetW(cl0.tangentVelocity.y, cl1.tangentVelocity.y, cl2.tangentVelocity.y, cl3.tangentVelocity.y);
	tangentVelocity.Z = b3SetW(cl0.tangentVelocity.z, cl1.tangentVelocity.z, cl2.tangentVelocity.z, cl3.tangentVelocity.z);

	let invIAB = b3AddSymW(iA, iB);
	if (b3AllZeroW(c.rollingResistance) == false) {
		c.rollingMass = b3InvertSymW(invIAB);
	} else {
		c.rollingMass = b3SymMatrix3W();
	}

	let rollingResistanceMask = b3GreaterThanW(c.rollingResistance, zeroW);

	for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
		ref cm = manifolds[manifoldBase + manifoldIndex];

		let manifoldIndices: FixedArray<i32, 4>;
		let activeLanes: FixedArray<i32, 4>;
		let pointCounts: FixedArray<i32, 4>;
		let pointCount = 0;
		for (let lane = 0; lane < 4; ++lane) {
			if (manifoldIndex < manifoldCounts[lane]) {
				let laneManifolds = contactLanes[lane].manifolds;
				manifoldLanes[lane] = laneManifolds;
				manifoldIndices[lane] = manifoldIndex;
				activeLanes[lane] = 1;
				pointCounts[lane] = laneManifolds[manifoldIndex].pointCount;
			} else {
				manifoldLanes[lane] = b3_zeroManifolds;
				manifoldIndices[lane] = 0;
				activeLanes[lane] = 0;
				pointCounts[lane] = 0;
			}

			pointCount = b3MaxInt(pointCount, pointCounts[lane]);
		}

		cm.pointCount = pointCount;
		cm.rollingMask = b3AndW(rollingResistanceMask, b3GreaterThanW(b3IntsToFloatW(activeLanes), zeroW));

		let ma0 = manifoldLanes[0];
		let ma1 = manifoldLanes[1];
		let ma2 = manifoldLanes[2];
		let ma3 = manifoldLanes[3];
		ref m0 = ma0[manifoldIndices[0]];
		ref m1 = ma1[manifoldIndices[1]];
		ref m2 = ma2[manifoldIndices[2]];
		ref m3 = ma3[manifoldIndices[3]];

		// C transposes the normal, twist, friction and rolling impulse floats of each lane
		let normal = b3Vec3W(
			b3SetW(m0.normal.x, m1.normal.x, m2.normal.x, m3.normal.x),
			b3SetW(m0.normal.y, m1.normal.y, m2.normal.y, m3.normal.y),
			b3SetW(m0.normal.z, m1.normal.z, m2.normal.z, m3.normal.z)
		);
		let twistImpulse = b3SetW(m0.twistImpulse, m1.twistImpulse, m2.twistImpulse, m3.twistImpulse);
		let frictionImpulse = b3Vec3W(
			b3SetW(m0.frictionImpulse.x, m1.frictionImpulse.x, m2.frictionImpulse.x, m3.frictionImpulse.x),
			b3SetW(m0.frictionImpulse.y, m1.frictionImpulse.y, m2.frictionImpulse.y, m3.frictionImpulse.y),
			b3SetW(m0.frictionImpulse.z, m1.frictionImpulse.z, m2.frictionImpulse.z, m3.frictionImpulse.z)
		);
		let rollingImpulse = b3Vec3W(
			b3SetW(m0.rollingImpulse.x, m1.rollingImpulse.x, m2.rollingImpulse.x, m3.rollingImpulse.x),
			b3SetW(m0.rollingImpulse.y, m1.rollingImpulse.y, m2.rollingImpulse.y, m3.rollingImpulse.y),
			b3SetW(m0.rollingImpulse.z, m1.rollingImpulse.z, m2.rollingImpulse.z, m3.rollingImpulse.z)
		);

		let tangent1 = b3PerpW(normal);
		let tangent2 = b3CrossW(tangent1, normal);
		cm.normal = normal;
		cm.tangent1 = tangent1;
		cm.tangent2 = tangent2;
		cm.tangentVelocity1 = b3DotW(tangentVelocity, tangent1);
		cm.tangentVelocity2 = b3DotW(tangentVelocity, tangent2);

		cm.twistImpulse = b3MulW(warmStartScale, twistImpulse);
		cm.rollingImpulse.X = b3BlendW(zeroW, b3MulW(warmStartScale, rollingImpulse.X), cm.rollingMask);
		cm.rollingImpulse.Y = b3BlendW(zeroW, b3MulW(warmStartScale, rollingImpulse.Y), cm.rollingMask);
		cm.rollingImpulse.Z = b3BlendW(zeroW, b3MulW(warmStartScale, rollingImpulse.Z), cm.rollingMask);
		cm.frictionImpulse.x = b3MulW(warmStartScale, b3DotW(frictionImpulse, tangent1));
		cm.frictionImpulse.y = b3MulW(warmStartScale, b3DotW(frictionImpulse, tangent2));

		let pointCountW = b3IntsToFloatW(pointCounts);

		let centerA = b3Vec3W(zeroW, zeroW, zeroW);
		let centerB = b3Vec3W(zeroW, zeroW, zeroW);
		let totalFrictionWeight = zeroW;

		for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
			ref cp = cm.points[pointIndex];
			let pointMask = b3GreaterThanW(pointCountW, b3SplatW(<f32>pointIndex));

			ref p0 = m0.points[pointIndex];
			ref p1 = m1.points[pointIndex];
			ref p2 = m2.points[pointIndex];
			ref p3 = m3.points[pointIndex];

			// C transposes anchorA, anchorB, separation and normalImpulse of each lane
			let rA = b3Vec3W(
				b3SetW(p0.anchorA.x, p1.anchorA.x, p2.anchorA.x, p3.anchorA.x),
				b3SetW(p0.anchorA.y, p1.anchorA.y, p2.anchorA.y, p3.anchorA.y),
				b3SetW(p0.anchorA.z, p1.anchorA.z, p2.anchorA.z, p3.anchorA.z)
			);
			let rB = b3Vec3W(
				b3SetW(p0.anchorB.x, p1.anchorB.x, p2.anchorB.x, p3.anchorB.x),
				b3SetW(p0.anchorB.y, p1.anchorB.y, p2.anchorB.y, p3.anchorB.y),
				b3SetW(p0.anchorB.z, p1.anchorB.z, p2.anchorB.z, p3.anchorB.z)
			);
			let separation = b3SetW(p0.separation, p1.separation, p2.separation, p3.separation);
			let normalImpulse = b3SetW(p0.normalImpulse, p1.normalImpulse, p2.normalImpulse, p3.normalImpulse);

			rA.X = b3BlendW(zeroW, rA.X, pointMask);
			rA.Y = b3BlendW(zeroW, rA.Y, pointMask);
			rA.Z = b3BlendW(zeroW, rA.Z, pointMask);
			rB.X = b3BlendW(zeroW, rB.X, pointMask);
			rB.Y = b3BlendW(zeroW, rB.Y, pointMask);
			rB.Z = b3BlendW(zeroW, rB.Z, pointMask);
			separation = b3BlendW(zeroW, separation, pointMask);
			normalImpulse = b3BlendW(zeroW, normalImpulse, pointMask);

			// C0 friction center decay. Needed to prevent spinning top drift (GyroscopicPrecession sample).
			// See details in b3PrepareContacts_Mesh. This code should stay in sync.
			let weight = b3MinW(b3MaxW(b3SubW(twoW, b3MulW(separation, invTau)), minFrictionWeight), oneW);
			weight = b3BlendW(zeroW, weight, pointMask);
			centerA = b3MulAddSVW(centerA, weight, rA);
			centerB = b3MulAddSVW(centerB, weight, rB);
			totalFrictionWeight = b3AddW(totalFrictionWeight, weight);

			cp.anchorAs = rA;
			cp.anchorBs = rB;
			cp.baseSeparations = b3SubW(separation, b3DotW(b3SubVW(rB, rA), normal));
			cp.normalImpulses = b3MulW(warmStartScale, normalImpulse);
			cp.totalNormalImpulses = zeroW;
			cp.relativeVelocities = zeroW;
			cp.restitutionImpulses = zeroW;

			let rnA = b3CrossW(rA, normal);
			let rnB = b3CrossW(rB, normal);
			let kNormal = b3AddW(mA, mB);
			kNormal = b3AddW(kNormal, b3DotW(rnA, b3MulMVW(iA, rnA)));
			kNormal = b3AddW(kNormal, b3DotW(rnB, b3MulMVW(iB, rnB)));
			let valid = b3AndW(b3GreaterThanW(kNormal, zeroW), pointMask);
			cp.normalMasses = b3BlendW(zeroW, b3DivW(oneW, kNormal), valid);
		}

		let frictionValid = b3GreaterThanW(totalFrictionWeight, zeroW);
		let invWeight = b3BlendW(zeroW, b3DivW(oneW, totalFrictionWeight), frictionValid);
		centerA = b3MulSVW(invWeight, centerA);
		centerB = b3MulSVW(invWeight, centerB);
		cm.centerA = centerA;
		cm.centerB = centerB;

		for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
			ref cp = cm.points[pointIndex];
			let pointMask = b3GreaterThanW(pointCountW, b3SplatW(<f32>pointIndex));
			let d = b3SubVW(cp.anchorAs, centerA);
			cp.leverArms = b3BlendW(zeroW, b3SqrtW(b3DotW(d, d)), pointMask);
		}

		{
			let rtA1 = b3CrossW(centerA, tangent1);
			let rtA2 = b3CrossW(centerA, tangent2);
			let rtB1 = b3CrossW(centerB, tangent1);
			let rtB2 = b3CrossW(centerB, tangent2);
			let iArtA1 = b3MulMVW(iA, rtA1);
			let iArtA2 = b3MulMVW(iA, rtA2);
			let iBrtB1 = b3MulMVW(iB, rtB1);
			let iBrtB2 = b3MulMVW(iB, rtB2);

			let kxx = b3AddW(b3AddW(b3AddW(mA, mB), b3DotW(rtA1, iArtA1)), b3DotW(rtB1, iBrtB1));
			let kyy = b3AddW(b3AddW(b3AddW(mA, mB), b3DotW(rtA2, iArtA2)), b3DotW(rtB2, iBrtB2));
			let kxy = b3AddW(b3DotW(rtA1, iArtA2), b3DotW(rtB1, iBrtB2));

			let det = b3SubW(b3MulW(kxx, kyy), b3MulW(kxy, kxy));
			let valid = b3AndW(b3GreaterThanW(b3AbsW(det), minDet), frictionValid);
			let invDet = b3BlendW(zeroW, b3DivW(oneW, det), valid);
			cm.tangentMass.cxx = b3MulW(invDet, kyy);
			cm.tangentMass.cxy = b3NegW(b3MulW(invDet, kxy));
			cm.tangentMass.cyy = b3MulW(invDet, kxx);
		}

		{
			let kTwist = b3DotW(normal, b3MulMVW(invIAB, normal));
			cm.twistMass = b3BlendW(zeroW, b3DivW(oneW, kTwist), b3GreaterThanW(kTwist, zeroW));
		}
	}

	// Only sample contact point normal velocity if needed.
	let haveRestitution = b3AnyTrueW(b3GreaterThanW(c.restitution, zeroW));
	if (hitEventLanes != 0 || haveRestitution) {
		let bA = b3GatherBodies(states, c.indexA);
		let bB = b3GatherBodies(states, c.indexB);

		for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
			ref cm = manifolds[manifoldBase + manifoldIndex];

			for (let pointIndex = 0; pointIndex < cm.pointCount; ++pointIndex) {
				ref cp = cm.points[pointIndex];

				let vrA = b3AddVW(bA.v, b3CrossW(bA.w, cp.anchorAs));
				let vrB = b3AddVW(bB.v, b3CrossW(bB.w, cp.anchorBs));
				let vn = b3DotW(cm.normal, b3SubVW(vrB, vrA));
				cp.relativeVelocities = vn;

				if (hitEventLanes != 0) {
					for (let lane = 0; lane < 4; ++lane) {
						if ((hitEventLanes & (1 << lane)) == 0 || manifoldIndex >= manifoldCounts[lane]) {
							continue;
						}

						let laneManifolds = contactLanes[lane].manifolds;
						ref manifold = laneManifolds[manifoldIndex];
						if (pointIndex < manifold.pointCount) {
							manifold.points[pointIndex].normalVelocity = b3LaneW(vn, lane);
						}
					}
				}
			}
		}
	}

	return haveRestitution;
}

export function b3PrepareContacts_ConvexW4(block: b3SolverBlock, context: b3StepContext): void {
	let world = context.world;
	let spans = context.widePrepareSpans;
	let constraints = context.wideConstraints;
	let manifolds = context.wideManifolds;
	let warmStartScale = world.enableWarmStarting ? b3SplatW(1.0) : b3ZeroW();
	let warmStartScaleValue: f32 = world.enableWarmStarting ? 1.0 : 0.0;
	let anyRestitution = false;

	let wideIndex = block.startIndex;
	let endWideIndex = block.startIndex + <i32>block.count;

	// Find color for start index. Linear search but fast.
	let colorIndex = 0;
	while (spans[colorIndex + 1].start <= wideIndex) {
		colorIndex += 1;
	}

	// Loop over block
	while (wideIndex < endWideIndex) {
		let colorWideStart = spans[colorIndex].start;
		let colorWideEndIndex = b3MinInt(spans[colorIndex + 1].start, endWideIndex);
		let colorContactCount = spans[colorIndex].count;
		let contactIds = spans[colorIndex].contacts;

		// Loop over color
		for (; wideIndex < colorWideEndIndex; ++wideIndex) {
			ref c = constraints[wideIndex];
			let localWideIndex = wideIndex - colorWideStart;

			let contacts: FixedArray<i32, 4>;
			for (let lane = 0; lane < 4; ++lane) {
				let contactIndex = 4 * localWideIndex + lane;
				if (contactIndex < colorContactCount) {
					contacts[lane] = contactIds[contactIndex];
					B3_ASSERT(world.contacts[contacts[lane]].manifoldCount == 1);
				} else {
					contacts[lane] = B3_NULL_INDEX;
				}
			}

			let haveRestitution = false;
			if (ASC_FEATURE_SIMD) {
				haveRestitution = b3PrepareConstraintV4<v128>(
					constraints,
					wideIndex,
					manifolds,
					wideIndex,
					contacts,
					1,
					context,
					warmStartScaleValue
				);
			} else {
				haveRestitution = b3PrepareConstraintW(c, manifolds, wideIndex, contacts, 1, context, warmStartScale);
			}
			anyRestitution = anyRestitution || haveRestitution;
		}

		// Advance to next color
		colorIndex += 1;
	}

	if (anyRestitution) {
		context.anyRestitution.value = 1;
	}
}

export function b3PrepareContacts_MeshWideW4(block: b3SolverBlock, context: b3StepContext): void {
	let world = context.world;
	let spans = context.meshPrepareSpans;
	let constraints = context.wideMeshConstraints;
	let manifolds = context.wideMeshManifolds;
	let manifoldStarts = context.wideMeshManifoldStarts;
	let warmStartScale = world.enableWarmStarting ? b3SplatW(1.0) : b3ZeroW();
	let warmStartScaleValue: f32 = world.enableWarmStarting ? 1.0 : 0.0;
	let anyRestitution = false;

	let wideIndex = block.startIndex;
	let endWideIndex = block.startIndex + <i32>block.count;

	let colorIndex = 0;
	while (spans[colorIndex + 1].start <= wideIndex) {
		colorIndex += 1;
	}

	while (wideIndex < endWideIndex) {
		let colorWideStart = spans[colorIndex].start;
		let colorWideEndIndex = b3MinInt(spans[colorIndex + 1].start, endWideIndex);
		let colorContactCount = spans[colorIndex].count;
		let specs = spans[colorIndex].contacts;

		// This order is used to group contacts with similar manifold counts to keep the lanes full.
		let order = spans[colorIndex].order;
		let orderBase = spans[colorIndex].orderBase;

		for (; wideIndex < colorWideEndIndex; ++wideIndex) {
			ref c = constraints[wideIndex];
			let localWideIndex = wideIndex - colorWideStart;

			let contacts: FixedArray<i32, 4>;
			for (let lane = 0; lane < 4; ++lane) {
				let contactIndex = 4 * localWideIndex + lane;
				if (contactIndex < colorContactCount) {
					let spec = specs[order[orderBase + contactIndex]];
					B3_ASSERT(world.contacts[spec.contactId].contactId == spec.contactId);
					B3_ASSERT(world.contacts[spec.contactId].manifoldCount == <i32>spec.manifoldCount);
					contacts[lane] = spec.contactId;
				} else {
					contacts[lane] = B3_NULL_INDEX;
				}
			}

			let manifoldCount = manifoldStarts[wideIndex + 1] - manifoldStarts[wideIndex];
			let haveRestitution = false;
			if (ASC_FEATURE_SIMD) {
				haveRestitution = b3PrepareConstraintV4<v128>(
					constraints,
					wideIndex,
					manifolds,
					manifoldStarts[wideIndex],
					contacts,
					manifoldCount,
					context,
					warmStartScaleValue
				);
			} else {
				haveRestitution = b3PrepareConstraintW(
					c,
					manifolds,
					manifoldStarts[wideIndex],
					contacts,
					manifoldCount,
					context,
					warmStartScale
				);
			}
			anyRestitution = anyRestitution || haveRestitution;
		}

		colorIndex += 1;
	}

	if (anyRestitution) {
		context.anyRestitution.value = 1;
	}
}

// C's base pointer is the constraint array from constraintBase. The first manifold of the block is at the constraint
// index for convex contacts (one manifold each) and at manifoldStarts[block.startIndex] for mesh groups.
function b3WarmStartContactsW(
	block: b3SolverBlock,
	context: b3StepContext,
	constraints: StaticArray<b3ContactConstraintWide>,
	manifolds: StaticArray<b3ContactManifoldWide>,
	constraintBase: i32,
	manifoldStarts: Int32Array,
	fixedManifoldCount: i32
): void {
	if (ASC_FEATURE_SIMD) {
		b3WarmStartContactsV4(block, context, constraints, manifolds, constraintBase, manifoldStarts, fixedManifoldCount);
		return;
	}

	let states = context.states;
	let zeroW = b3ZeroW();

	let ci = constraintBase + block.startIndex;
	let mi = fixedManifoldCount > 0 ? ci : manifoldStarts[block.startIndex];
	for (let i = 0; i < <i32>block.count; ++i) {
		ref c = constraints[ci];
		let manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : c.manifoldCount;
		let anyRolling = b3AllZeroW(c.rollingResistance) == false;

		let linearImpulse = b3Vec3W(zeroW, zeroW, zeroW);
		let angularImpulseA = b3Vec3W(zeroW, zeroW, zeroW);
		let angularImpulseB = b3Vec3W(zeroW, zeroW, zeroW);

		for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
			ref cm = manifolds[mi + manifoldIndex];

			let totalNormalImpulse = zeroW;
			let momentA = b3Vec3W(zeroW, zeroW, zeroW);
			let momentB = b3Vec3W(zeroW, zeroW, zeroW);

			for (let pointIndex = 0; pointIndex < cm.pointCount; ++pointIndex) {
				ref cp = cm.points[pointIndex];
				let normalImpulse = cp.normalImpulses;
				totalNormalImpulse = b3AddW(totalNormalImpulse, normalImpulse);
				momentA = b3MulAddSVW(momentA, normalImpulse, cp.anchorAs);
				momentB = b3MulAddSVW(momentB, normalImpulse, cp.anchorBs);
			}

			let normal = cm.normal;
			let frictionImpulse = b3MulSVW(cm.frictionImpulse.x, cm.tangent1);
			frictionImpulse = b3MulAddSVW(frictionImpulse, cm.frictionImpulse.y, cm.tangent2);

			let linear = b3MulAddSVW(frictionImpulse, totalNormalImpulse, normal);

			let twistImpulse = b3MulSVW(cm.twistImpulse, normal);
			let angularA = b3AddVW(b3AddVW(b3CrossW(momentA, normal), b3CrossW(cm.centerA, frictionImpulse)), twistImpulse);
			let angularB = b3AddVW(b3AddVW(b3CrossW(momentB, normal), b3CrossW(cm.centerB, frictionImpulse)), twistImpulse);

			if (anyRolling) {
				angularA = b3AddVW(angularA, cm.rollingImpulse);
				angularB = b3AddVW(angularB, cm.rollingImpulse);
			}

			linearImpulse = b3AddVW(linearImpulse, linear);
			angularImpulseA = b3AddVW(angularImpulseA, angularA);
			angularImpulseB = b3AddVW(angularImpulseB, angularB);
		}

		let bA = b3GatherBodies(states, c.indexA);
		let bB = b3GatherBodies(states, c.indexB);

		bA.w = b3MulSubMVW(bA.w, c.invIA, angularImpulseA);
		bA.v = b3MulSubSVW(bA.v, c.invMassA, linearImpulse);
		bB.w = b3MulAddMVW(bB.w, c.invIB, angularImpulseB);
		bB.v = b3MulAddSVW(bB.v, c.invMassB, linearImpulse);

		b3ScatterBodies(states, c.indexA, bA);
		b3ScatterBodies(states, c.indexB, bB);

		ci += 1;
		mi += manifoldCount;
	}
}

// Solve the non-penetration constraints with the soft bias. No friction and no restitution.
function b3PushContactsW(
	block: b3SolverBlock,
	context: b3StepContext,
	constraints: StaticArray<b3ContactConstraintWide>,
	manifolds: StaticArray<b3ContactManifoldWide>,
	constraintBase: i32,
	manifoldStarts: Int32Array,
	fixedManifoldCount: i32
): void {
	if (ASC_FEATURE_SIMD) {
		b3PushContactsV4(block, context, constraints, manifolds, constraintBase, manifoldStarts, fixedManifoldCount);
		return;
	}

	let states = context.states;
	let inv_h = b3SplatW(context.inv_h);
	let contactSpeed = b3SplatW(-context.world.contactSpeed);
	let oneW = b3SplatW(1.0);
	let zeroW = b3ZeroW();

	// Stiffer for static contacts to avoid bodies getting pushed through the ground. Selected per
	// lane from the null body index instead of stored per constraint.
	let dynamicBiasRate = b3SplatW(context.contactSoftness.massScale * context.contactSoftness.biasRate);
	let dynamicMassScale = b3SplatW(context.contactSoftness.massScale);
	let dynamicImpulseScale = b3SplatW(context.contactSoftness.impulseScale);
	let staticBiasRate = b3SplatW(context.staticSoftness.massScale * context.staticSoftness.biasRate);
	let staticMassScale = b3SplatW(context.staticSoftness.massScale);
	let staticImpulseScale = b3SplatW(context.staticSoftness.impulseScale);

	let ci = constraintBase + block.startIndex;
	let mi = fixedManifoldCount > 0 ? ci : manifoldStarts[block.startIndex];
	for (let i = 0; i < <i32>block.count; ++i) {
		ref c = constraints[ci];
		let manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : c.manifoldCount;
		let bA = b3GatherBodies(states, c.indexA);
		let bB = b3GatherBodies(states, c.indexB);

		let softMask = b3SoftMaskW(c.indexA, c.indexB);
		let biasRate = b3BlendW(dynamicBiasRate, staticBiasRate, softMask);
		let massScale = b3BlendW(dynamicMassScale, staticMassScale, softMask);
		let impulseScale = b3BlendW(dynamicImpulseScale, staticImpulseScale, softMask);

		let dp = b3SubVW(bB.dp, bA.dp);
		let invMass = b3AddW(c.invMassA, c.invMassB);

		for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
			ref cm = manifolds[mi + manifoldIndex];

			// Convert to normals to local space to reduce transform math.
			// Read the normal rather than put on stack to avoid register spills.
			let normalSeparation = b3DotW(cm.normal, dp);
			let normalA = b3InvRotateVectorW(bA.dq, cm.normal);
			let normalB = b3InvRotateVectorW(bB.dq, cm.normal);

			let linearNormalVelocity = b3DotW(cm.normal, b3SubVW(bB.v, bA.v));
			let manifoldImpulse = zeroW;

			for (let pointIndex = 0; pointIndex < cm.pointCount; ++pointIndex) {
				ref cp = cm.points[pointIndex];

				// Fixed anchor points for applying impulses
				let rA = cp.anchorAs;
				let rB = cp.anchorBs;

				let s = b3AddW(b3AddW(normalSeparation, b3SubW(b3DotW(normalB, rB), b3DotW(normalA, rA))), cp.baseSeparations);

				// Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
				let separated = b3GreaterThanW(s, zeroW);

				// Speculative bias - positive
				let specBias = b3MulW(s, inv_h);

				// Overlap bias - negative
				let overlapBias = b3MaxW(b3MulW(biasRate, s), contactSpeed);
				let velocityBias = b3BlendW(overlapBias, specBias, separated);

				let pointMassScale = b3BlendW(massScale, oneW, separated);
				let pointImpulseScale = b3BlendW(impulseScale, zeroW, separated);

				let uA = b3CrossW(rA, cm.normal);
				let uB = b3CrossW(rB, cm.normal);
				let gA = b3MulMVW(c.invIA, uA);
				let gB = b3MulMVW(c.invIB, uB);

				let vn = b3AddW(linearNormalVelocity, b3SubW(b3DotW(uB, bB.w), b3DotW(uA, bA.w)));

				let negImpulse = b3AddW(
					b3MulW(cp.normalMasses, b3AddW(b3MulW(pointMassScale, vn), velocityBias)),
					b3MulW(pointImpulseScale, cp.normalImpulses)
				);
				let impulse = b3MaxW(b3NegW(negImpulse), b3NegW(cp.normalImpulses));
				cp.normalImpulses = b3AddW(cp.normalImpulses, impulse);

				linearNormalVelocity = b3MulAddW(linearNormalVelocity, invMass, impulse);
				manifoldImpulse = b3AddW(manifoldImpulse, impulse);
				bA.w = b3MulSubSVW(bA.w, impulse, gA);
				bB.w = b3MulAddSVW(bB.w, impulse, gB);
			}

			let P = b3MulSVW(manifoldImpulse, cm.normal);
			bA.v = b3MulSubSVW(bA.v, c.invMassA, P);
			bB.v = b3MulAddSVW(bB.v, c.invMassB, P);
		}

		b3ScatterBodies(states, c.indexA, bA);
		b3ScatterBodies(states, c.indexB, bB);

		ci += 1;
		mi += manifoldCount;
	}
}

// Solve the normal constraint, friction, and rolling resistance.
function b3SolveContactsW(
	block: b3SolverBlock,
	context: b3StepContext,
	constraints: StaticArray<b3ContactConstraintWide>,
	manifolds: StaticArray<b3ContactManifoldWide>,
	constraintBase: i32,
	manifoldStarts: Int32Array,
	fixedManifoldCount: i32
): void {
	if (ASC_FEATURE_SIMD) {
		b3SolveContactsV4(block, context, constraints, manifolds, constraintBase, manifoldStarts, fixedManifoldCount);
		return;
	}

	let states = context.states;
	let inv_h = b3SplatW(context.inv_h);
	let oneW = b3SplatW(1.0);
	let zeroW = b3ZeroW();
	let epsilonW = b3SplatW(FLT_EPSILON);

	let ci = constraintBase + block.startIndex;
	let mi = fixedManifoldCount > 0 ? ci : manifoldStarts[block.startIndex];
	for (let i = 0; i < <i32>block.count; ++i) {
		ref c = constraints[ci];
		let manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : c.manifoldCount;
		let bA = b3GatherBodies(states, c.indexA);
		let bB = b3GatherBodies(states, c.indexB);

		let anyRolling = b3AllZeroW(c.rollingResistance) == false;
		let dp = b3SubVW(bB.dp, bA.dp);
		let invMass = b3AddW(c.invMassA, c.invMassB);

		for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
			ref cm = manifolds[mi + manifoldIndex];

			// Read the normal rather than put on stack to avoid register spills.
			let normalSeparation = b3DotW(cm.normal, dp);
			let normalA = b3InvRotateVectorW(bA.dq, cm.normal);
			let normalB = b3InvRotateVectorW(bB.dq, cm.normal);

			let dv = b3SubVW(bB.v, bA.v);
			let linearNormalVelocity = b3DotW(cm.normal, dv);
			let linearTangentVelocity1 = b3SubW(b3DotW(cm.tangent1, dv), cm.tangentVelocity1);
			let linearTangentVelocity2 = b3SubW(b3DotW(cm.tangent2, dv), cm.tangentVelocity2);
			let manifoldImpulse = zeroW;

			let totalNormalImpulse = zeroW;
			let totalTwistLimit = zeroW;

			// Normal contraints
			for (let pointIndex = 0; pointIndex < cm.pointCount; ++pointIndex) {
				ref cp = cm.points[pointIndex];

				// Fixed anchor points for applying impulses
				let rA = cp.anchorAs;
				let rB = cp.anchorBs;

				let s = b3AddW(b3AddW(normalSeparation, b3SubW(b3DotW(normalB, rB), b3DotW(normalA, rA))), cp.baseSeparations);

				// Speculative bias, positive if separated and zero if overlapped
				let velocityBias = b3MaxW(zeroW, b3MulW(s, inv_h));

				let uA = b3CrossW(rA, cm.normal);
				let uB = b3CrossW(rB, cm.normal);
				let gA = b3MulMVW(c.invIA, uA);
				let gB = b3MulMVW(c.invIB, uB);

				let vn = b3AddW(linearNormalVelocity, b3SubW(b3DotW(uB, bB.w), b3DotW(uA, bA.w)));

				let negImpulse = b3MulW(cp.normalMasses, b3AddW(vn, velocityBias));
				let impulse = b3MaxW(b3NegW(negImpulse), b3NegW(cp.normalImpulses));
				let newImpulse = b3AddW(cp.normalImpulses, impulse);
				cp.normalImpulses = newImpulse;
				cp.totalNormalImpulses = b3AddW(cp.totalNormalImpulses, newImpulse);
				totalNormalImpulse = b3AddW(totalNormalImpulse, newImpulse);
				totalTwistLimit = b3AddW(totalTwistLimit, b3MulW(cp.leverArms, newImpulse));

				linearNormalVelocity = b3MulAddW(linearNormalVelocity, invMass, impulse);
				manifoldImpulse = b3AddW(manifoldImpulse, impulse);
				bA.w = b3MulSubSVW(bA.w, impulse, gA);
				bB.w = b3MulAddSVW(bB.w, impulse, gB);
			}

			// Rolling resistance. After normal constraints because it uses the normal impulse.
			if (anyRolling) {
				// flip A/B order to negate
				let deltaImpulse = b3MulMVW(c.rollingMass, b3SubVW(bA.w, bB.w));
				let oldImpulse = cm.rollingImpulse;
				cm.rollingImpulse = b3AddVW(oldImpulse, deltaImpulse);

				let maxImpulse = b3MulW(c.rollingResistance, totalNormalImpulse);
				let lengthSquared = b3DotW(cm.rollingImpulse, cm.rollingImpulse);

				// if ( magSqr > maxLambda * maxLambda + FLT_EPSILON )
				//{
				//	c->rollingImpulse *= maxLambda / sqrtf( magSqr );
				// }

				let mask = b3GreaterThanW(lengthSquared, b3MulAddW(epsilonW, maxImpulse, maxImpulse));

				// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism
				let normalize = b3DivW(maxImpulse, b3AddW(b3SqrtW(lengthSquared), epsilonW));
				let scale = b3BlendW(oneW, normalize, mask);

				// Blend the result to ensure there are no +/-0 determinism problems, since 0 * -1 == -0.
				let scaledImpulse = b3MulSVW(scale, cm.rollingImpulse);
				cm.rollingImpulse.X = b3BlendW(zeroW, scaledImpulse.X, cm.rollingMask);
				cm.rollingImpulse.Y = b3BlendW(zeroW, scaledImpulse.Y, cm.rollingMask);
				cm.rollingImpulse.Z = b3BlendW(zeroW, scaledImpulse.Z, cm.rollingMask);

				deltaImpulse = b3SubVW(cm.rollingImpulse, oldImpulse);

				bA.w = b3MulSubMVW(bA.w, c.invIA, deltaImpulse);
				bB.w = b3MulAddMVW(bB.w, c.invIB, deltaImpulse);
			}

			// Central twist friction. Friction goes last to improve stacking stability.
			{
				let twistSpeed = b3DotW(cm.normal, b3SubVW(bB.w, bA.w));
				let maxLambda = b3MulW(c.friction, totalTwistLimit);
				let oldImpulse = cm.twistImpulse;
				let impulse = b3NegW(b3MulW(cm.twistMass, twistSpeed));
				impulse = b3MaxW(b3SubW(b3NegW(maxLambda), oldImpulse), b3MinW(impulse, b3SubW(maxLambda, oldImpulse)));
				cm.twistImpulse = b3AddW(oldImpulse, impulse);

				bA.w = b3MulSubSVW(bA.w, impulse, b3MulMVW(c.invIA, cm.normal));
				bB.w = b3MulAddSVW(bB.w, impulse, b3MulMVW(c.invIB, cm.normal));
			}

			// Central friction. Friction goes last to improve stacking stability.
			{
				let tangent1 = cm.tangent1;
				let tangent2 = cm.tangent2;

				// Fixed anchor points for applying impulses
				let rA = cm.centerA;
				let rB = cm.centerB;

				let angularRelativeVelocity = b3SubVW(b3CrossW(bB.w, rB), b3CrossW(bA.w, rA));

				// Relative tangent velocity at contact
				let vt = b3Vec2W();
				vt.x = b3AddW(linearTangentVelocity1, b3DotW(angularRelativeVelocity, tangent1));
				vt.y = b3AddW(linearTangentVelocity2, b3DotW(angularRelativeVelocity, tangent2));

				// Incremental tangent impulse
				let deltaImpulse = b3MulMV2W(cm.tangentMass, vt);
				deltaImpulse.x = b3NegW(deltaImpulse.x);
				deltaImpulse.y = b3NegW(deltaImpulse.y);
				let newImpulse = b3AddV2W(cm.frictionImpulse, deltaImpulse);

				let maxImpulse = b3MulW(c.friction, totalNormalImpulse);

				// Clamp the accumulated impulse
				let lengthSquared = b3AddW(b3MulW(newImpulse.x, newImpulse.x), b3MulW(newImpulse.y, newImpulse.y));

				// Max impulse can be zero
				let mask = b3GreaterThanW(lengthSquared, b3MulW(maxImpulse, maxImpulse));

				// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism. Add epsilon to avoid divide by
				// zero.
				let normalize = b3DivW(maxImpulse, b3AddW(b3SqrtW(lengthSquared), epsilonW));
				let scale = b3BlendW(oneW, normalize, mask);
				newImpulse.x = b3MulW(scale, newImpulse.x);
				newImpulse.y = b3MulW(scale, newImpulse.y);

				deltaImpulse.x = b3SubW(newImpulse.x, cm.frictionImpulse.x);
				deltaImpulse.y = b3SubW(newImpulse.y, cm.frictionImpulse.y);

				cm.frictionImpulse = newImpulse;

				// Apply delta impulse
				let P = b3AddVW(b3MulSVW(deltaImpulse.x, tangent1), b3MulSVW(deltaImpulse.y, tangent2));
				bA.w = b3MulSubMVW(bA.w, c.invIA, b3CrossW(rA, P));
				bB.w = b3MulAddMVW(bB.w, c.invIB, b3CrossW(rB, P));

				P = b3MulAddSVW(P, manifoldImpulse, cm.normal);
				bA.v = b3MulSubSVW(bA.v, c.invMassA, P);
				bB.v = b3MulAddSVW(bB.v, c.invMassB, P);
			}
		}

		b3ScatterBodies(states, c.indexA, bA);
		b3ScatterBodies(states, c.indexB, bB);

		ci += 1;
		mi += manifoldCount;
	}
}

function b3ApplyRestitutionW(
	block: b3SolverBlock,
	context: b3StepContext,
	constraints: StaticArray<b3ContactConstraintWide>,
	manifolds: StaticArray<b3ContactManifoldWide>,
	constraintBase: i32,
	manifoldStarts: Int32Array,
	fixedManifoldCount: i32
): void {
	let states = context.states;
	let inv_h = b3SplatW(context.inv_h);
	let negRestitutionThreshold = b3SplatW(-context.world.restitutionThreshold);
	let zeroW = b3ZeroW();
	let propagate = context.world.enableRestitutionPropagation;

	let nextCi = constraintBase + block.startIndex;
	let nextMi = fixedManifoldCount > 0 ? nextCi : manifoldStarts[block.startIndex];
	for (let i = 0; i < <i32>block.count; ++i) {
		let ci = nextCi;
		let mi = nextMi;
		ref c = constraints[ci];
		let manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : c.manifoldCount;
		nextCi = ci + 1;
		nextMi = mi + manifoldCount;

		if (propagate == false && b3AllZeroW(c.restitution)) {
			continue;
		}

		let bA = b3GatherBodies(states, c.indexA);
		let bB = b3GatherBodies(states, c.indexB);

		let restitutionMask = b3GreaterThanW(c.restitution, zeroW);
		let dp = b3SubVW(bB.dp, bA.dp);

		for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
			ref cm = manifolds[mi + manifoldIndex];

			let normalSeparation = b3DotW(cm.normal, dp);
			let normalA = b3InvRotateVectorW(bA.dq, cm.normal);
			let normalB = b3InvRotateVectorW(bB.dq, cm.normal);

			// Read the point count rather than put on stack to avoid register spills.
			for (let pointIndex = 0; pointIndex < cm.pointCount; ++pointIndex) {
				ref cp = cm.points[pointIndex];

				let rA = cp.anchorAs;
				let rB = cp.anchorBs;

				let s = b3AddW(b3AddW(normalSeparation, b3SubW(b3DotW(normalB, rB), b3DotW(normalA, rA))), cp.baseSeparations);

				let normalMass = propagate ? cp.normalMasses : b3BlendW(zeroW, cp.normalMasses, restitutionMask);

				let compressionImpulse = b3SubW(cp.totalNormalImpulses, cp.restitutionImpulses);
				let armed = b3AndW(
					b3AndW(restitutionMask, b3LessThanW(cp.relativeVelocities, negRestitutionThreshold)),
					b3GreaterThanW(compressionImpulse, zeroW)
				);

				let specBias = b3MaxW(zeroW, b3MulW(s, inv_h));
				let velocityBias = b3BlendW(specBias, b3MulW(c.restitution, cp.relativeVelocities), armed);

				let vrA = b3AddVW(bA.v, b3CrossW(bA.w, rA));
				let vrB = b3AddVW(bB.v, b3CrossW(bB.w, rB));
				let vn = b3DotW(b3SubVW(vrB, vrA), cm.normal);

				let negImpulse = b3MulW(normalMass, b3AddW(vn, velocityBias));

				let newImpulse = b3MaxW(b3SubW(cp.normalImpulses, negImpulse), zeroW);
				let impulse = b3SubW(newImpulse, cp.normalImpulses);

				let approachImpulse = b3MinW(b3MaxW(b3NegW(b3MulW(normalMass, vn)), zeroW), b3MaxW(impulse, zeroW));
				let allowance = b3SubW(b3MulW(c.restitution, b3AddW(compressionImpulse, approachImpulse)), cp.restitutionImpulses);
				let maxImpulse = b3AddW(approachImpulse, b3MaxW(allowance, zeroW));
				impulse = b3BlendW(impulse, b3MinW(impulse, maxImpulse), armed);

				cp.normalImpulses = b3AddW(cp.normalImpulses, impulse);
				cp.restitutionImpulses = b3AddW(cp.restitutionImpulses, b3SubW(impulse, approachImpulse));
				cp.totalNormalImpulses = b3AddW(cp.totalNormalImpulses, impulse);

				let P = b3MulSVW(impulse, cm.normal);
				bA.w = b3MulSubMVW(bA.w, c.invIA, b3CrossW(rA, P));
				bA.v = b3MulSubSVW(bA.v, c.invMassA, P);
				bB.w = b3MulAddMVW(bB.w, c.invIB, b3CrossW(rB, P));
				bB.v = b3MulAddSVW(bB.v, c.invMassB, P);
			}
		}

		b3ScatterBodies(states, c.indexA, bA);
		b3ScatterBodies(states, c.indexB, bB);
	}
}

// Field offsets for the raw lane loads and the v128 kernels

const CW_CONTACTS = offsetof<b3ContactConstraintWide>("contacts");
const CW_INDEXA = offsetof<b3ContactConstraintWide>("indexA");
const CW_INDEXB = offsetof<b3ContactConstraintWide>("indexB");
const CW_INVMASSA = offsetof<b3ContactConstraintWide>("invMassA");
const CW_INVMASSB = offsetof<b3ContactConstraintWide>("invMassB");
const CW_INVIA = offsetof<b3ContactConstraintWide>("invIA");
const CW_INVIB = offsetof<b3ContactConstraintWide>("invIB");
const CW_ROLLINGMASS = offsetof<b3ContactConstraintWide>("rollingMass");
const CW_FRICTION = offsetof<b3ContactConstraintWide>("friction");
const CW_ROLLINGRESISTANCE = offsetof<b3ContactConstraintWide>("rollingResistance");
const CW_RESTITUTION = offsetof<b3ContactConstraintWide>("restitution");
const CW_MANIFOLDCOUNT = offsetof<b3ContactConstraintWide>("manifoldCount");

const MW_NORMAL = offsetof<b3ContactManifoldWide>("normal");
const MW_TANGENT1 = offsetof<b3ContactManifoldWide>("tangent1");
const MW_TANGENT2 = offsetof<b3ContactManifoldWide>("tangent2");
const MW_CENTERA = offsetof<b3ContactManifoldWide>("centerA");
const MW_CENTERB = offsetof<b3ContactManifoldWide>("centerB");
const MW_TWISTMASS = offsetof<b3ContactManifoldWide>("twistMass");
const MW_TWISTIMPULSE = offsetof<b3ContactManifoldWide>("twistImpulse");
const MW_TANGENTMASS = offsetof<b3ContactManifoldWide>("tangentMass");
const MW_FRICTIONIMPULSE = offsetof<b3ContactManifoldWide>("frictionImpulse");
const MW_ROLLINGIMPULSE = offsetof<b3ContactManifoldWide>("rollingImpulse");
const MW_ROLLINGMASK = offsetof<b3ContactManifoldWide>("rollingMask");
const MW_TANGENTVELOCITY1 = offsetof<b3ContactManifoldWide>("tangentVelocity1");
const MW_TANGENTVELOCITY2 = offsetof<b3ContactManifoldWide>("tangentVelocity2");
const MW_POINTCOUNT = offsetof<b3ContactManifoldWide>("pointCount");
const MW_POINTS = offsetof<b3ContactManifoldWide>("points");

const PW_ANCHORA = offsetof<b3ContactConstraintPointWide>("anchorAs");
const PW_ANCHORB = offsetof<b3ContactConstraintPointWide>("anchorBs");
const PW_BASESEPARATION = offsetof<b3ContactConstraintPointWide>("baseSeparations");
const PW_NORMALIMPULSE = offsetof<b3ContactConstraintPointWide>("normalImpulses");
const PW_TOTALNORMALIMPULSE = offsetof<b3ContactConstraintPointWide>("totalNormalImpulses");
const PW_NORMALMASS = offsetof<b3ContactConstraintPointWide>("normalMasses");
const PW_LEVERARM = offsetof<b3ContactConstraintPointWide>("leverArms");
const PW_RELATIVEVELOCITY = offsetof<b3ContactConstraintPointWide>("relativeVelocities");
const PW_RESTITUTIONIMPULSE = offsetof<b3ContactConstraintPointWide>("restitutionImpulses");

const V_X = offsetof<b3Vec3W>("X");
const V_Y = offsetof<b3Vec3W>("Y");
const V_Z = offsetof<b3Vec3W>("Z");
const S_CXX = offsetof<b3SymMatrix3W>("cxx");
const S_CXY = offsetof<b3SymMatrix3W>("cxy");
const S_CXZ = offsetof<b3SymMatrix3W>("cxz");
const S_CYY = offsetof<b3SymMatrix3W>("cyy");
const S_CYZ = offsetof<b3SymMatrix3W>("cyz");
const S_CZZ = offsetof<b3SymMatrix3W>("czz");
const S2_CXX = offsetof<b3SymMatrix2W>("cxx");
const S2_CXY = offsetof<b3SymMatrix2W>("cxy");
const S2_CYY = offsetof<b3SymMatrix2W>("cyy");
const V2_X = offsetof<b3Vec2W>("x");
const V2_Y = offsetof<b3Vec2W>("y");

const BS_V = offsetof<b3BodyState>("linearVelocity");
const BS_W = offsetof<b3BodyState>("angularVelocity");
const BS_DP = offsetof<b3BodyState>("deltaPosition");
const BS_FLAGS = offsetof<b3BodyState>("flags");
const BS_DQ = offsetof<b3BodyState>("deltaRotation");

// Store impulses by contact constraint. C reads each lane through a float pointer; so do these raw loads, in range by
// construction (lane < 4, manifold and point counts from the contact).
unsafe function b3StoreImpulsesW(
	block: b3SolverBlock,
	context: b3StepContext,
	workerIndex: i32,
	constraints: StaticArray<b3ContactConstraintWide>,
	manifolds: StaticArray<b3ContactManifoldWide>,
	constraintBase: i32,
	manifoldStarts: Int32Array,
	fixedManifoldCount: i32
): void {
	let world = context.world;
	let taskContext = world.taskContexts[workerIndex];
	let hitEventBitSet = taskContext.hitEventBitSet;
	let hasHitEvents = taskContext.hasHitEvents;
	let negHitThreshold = -world.hitEventThreshold;
	let constraintData = changetype<usize>(constraints);
	let manifoldData = changetype<usize>(manifolds);

	let ci = constraintBase + block.startIndex;
	let mi = fixedManifoldCount > 0 ? ci : manifoldStarts[block.startIndex];
	for (let i = 0; i < <i32>block.count; ++i) {
		let c = constraintData + <usize>ci * sizeof<b3ContactConstraintWide>();
		let manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : load<i32>(c + CW_MANIFOLDCOUNT);

		for (let lane = 0; lane < 4; ++lane) {
			let contactId = load<i32>(c + CW_CONTACTS + <usize>lane * 4);
			if (contactId == B3_NULL_INDEX) {
				continue;
			}

			let contact = world.contacts[contactId];
			let laneManifolds = changetype<usize>(contact.manifolds);

			let laneManifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : contact.manifoldCount;
			B3_ASSERT(laneManifoldCount == contact.manifoldCount && laneManifoldCount <= manifoldCount);

			let laneOffset = <usize>lane * 4;
			for (let manifoldIndex = 0; manifoldIndex < laneManifoldCount; ++manifoldIndex) {
				let cm = manifoldData + <usize>(mi + manifoldIndex) * sizeof<b3ContactManifoldWide>() + laneOffset;
				let m = laneManifolds + <usize>manifoldIndex * sizeof<b3Manifold>();

				let f1 = load<f32>(cm + MW_FRICTIONIMPULSE + V2_X);
				let f2 = load<f32>(cm + MW_FRICTIONIMPULSE + V2_Y);
				let frictionImpulse = m + offsetof<b3Manifold>("frictionImpulse");
				store<f32>(frictionImpulse, f1 * load<f32>(cm + MW_TANGENT1 + V_X) + f2 * load<f32>(cm + MW_TANGENT2 + V_X));
				store<f32>(frictionImpulse + 4, f1 * load<f32>(cm + MW_TANGENT1 + V_Y) + f2 * load<f32>(cm + MW_TANGENT2 + V_Y));
				store<f32>(frictionImpulse + 8, f1 * load<f32>(cm + MW_TANGENT1 + V_Z) + f2 * load<f32>(cm + MW_TANGENT2 + V_Z));
				store<f32>(m + offsetof<b3Manifold>("twistImpulse"), load<f32>(cm + MW_TWISTIMPULSE));
				let rollingImpulse = m + offsetof<b3Manifold>("rollingImpulse");
				store<f32>(rollingImpulse, load<f32>(cm + MW_ROLLINGIMPULSE + V_X));
				store<f32>(rollingImpulse + 4, load<f32>(cm + MW_ROLLINGIMPULSE + V_Y));
				store<f32>(rollingImpulse + 8, load<f32>(cm + MW_ROLLINGIMPULSE + V_Z));

				let pointCount = load<i32>(m + offsetof<b3Manifold>("pointCount"));
				B3_ASSERT(pointCount <= load<i32>(cm - laneOffset + MW_POINTCOUNT));
				for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
					let cp = cm + MW_POINTS + <usize>pointIndex * sizeof<b3ContactConstraintPointWide>();
					let mp = m + offsetof<b3Manifold>("points") + <usize>pointIndex * sizeof<b3ManifoldPoint>();
					store<f32>(mp + offsetof<b3ManifoldPoint>("normalImpulse"), load<f32>(cp + PW_NORMALIMPULSE));
					store<f32>(mp + offsetof<b3ManifoldPoint>("totalNormalImpulse"), load<f32>(cp + PW_TOTALNORMALIMPULSE));
				}
			}

			if ((contact.flags & b3_simEnableHitEvent) != 0) {
				let contactManifolds = contact.manifolds;
				let flagged = false;
				for (let manifoldIndex = 0; manifoldIndex < laneManifoldCount && flagged == false; ++manifoldIndex) {
					ref m = contactManifolds[manifoldIndex];
					for (let pointIndex = 0; pointIndex < m.pointCount; ++pointIndex) {
						ref mp = m.points[pointIndex];

						// Need to check total impulse because the point may be speculative and not colliding
						if (mp.normalVelocity < negHitThreshold && mp.totalNormalImpulse > 0.0) {
							b3SetBit(hitEventBitSet, <u32>contact.contactId);
							hasHitEvents = true;
							flagged = true;
							break;
						}
					}
				}
			}
		}

		ci += 1;
		mi += manifoldCount;
	}

	taskContext.hasHitEvents = hasHitEvents;
}

// Wasm SIMD versions of the iteration kernels, the same lane math with each b3FloatW in a v128 and the
// constraints read in place. The gather and scatter are C's SSE2 ones: each 64 byte state is four rows,
// transposed into lanes. Body indices come from b3PrepareConstraintW, so they are in range by construction.

function b3MakeIdentityStates(): StaticArray<b3BodyState> {
	let states = new StaticArray<b3BodyState>(1);
	states[0] = b3_identityBodyState;
	return states;
}

// C points static lanes at b3_identityBodyState.
const b3_identityStates = b3MakeIdentityStates();

@inline unsafe function b3StatesDataV4(states: Array<b3BodyState>): usize {
	return load<usize>(changetype<usize>(states) + offsetof<Array<i32>>("dataStart"));
}

// Indices are 0 for null
@inline function b3StatePointerV4(data: usize, identity: usize, index: i32): usize {
	return index == 0 ? identity : data + <usize>(index - 1) * sizeof<b3BodyState>();
}

@inline unsafe function b3StoreBodyVelocityV4<T>(data: usize, index: i32, v: T, w: T): void {
	if (index == 0) {
		return;
	}

	let state = data + <usize>(index - 1) * sizeof<b3BodyState>();
	if ((load<u32>(state + BS_FLAGS) & b3_dynamicFlag) == 0) {
		return;
	}

	v128.store(state + BS_V, v);
	v128.store(state + BS_W, w);
}

// This writes only the velocities back to the solver bodies
@inline unsafe function b3ScatterBodiesV4<T>(data: usize, indices: usize, vX: T, vY: T, vZ: T, wX: T, wY: T, wZ: T): void {
	let zero = f32x4.splat(0.0);
	b3StoreBodyVelocityV4(data, load<i32>(indices), b3Transpose0V4(vX, vY, vZ, zero), b3Transpose0V4(wX, wY, wZ, zero));
	b3StoreBodyVelocityV4(data, load<i32>(indices + 4), b3Transpose1V4(vX, vY, vZ, zero), b3Transpose1V4(wX, wY, wZ, zero));
	b3StoreBodyVelocityV4(data, load<i32>(indices + 8), b3Transpose2V4(vX, vY, vZ, zero), b3Transpose2V4(wX, wY, wZ, zero));
	b3StoreBodyVelocityV4(data, load<i32>(indices + 12), b3Transpose3V4(vX, vY, vZ, zero), b3Transpose3V4(wX, wY, wZ, zero));
}

@inline unsafe function b3GatherXV4<T>(p1: usize, p2: usize, p3: usize, p4: usize, offset: usize): T {
	return b3Transpose0V4(v128.load(p1 + offset), v128.load(p2 + offset), v128.load(p3 + offset), v128.load(p4 + offset));
}

@inline unsafe function b3GatherYV4<T>(p1: usize, p2: usize, p3: usize, p4: usize, offset: usize): T {
	return b3Transpose1V4(v128.load(p1 + offset), v128.load(p2 + offset), v128.load(p3 + offset), v128.load(p4 + offset));
}

@inline unsafe function b3GatherZV4<T>(p1: usize, p2: usize, p3: usize, p4: usize, offset: usize): T {
	return b3Transpose2V4(v128.load(p1 + offset), v128.load(p2 + offset), v128.load(p3 + offset), v128.load(p4 + offset));
}

@inline unsafe function b3GatherWV4<T>(p1: usize, p2: usize, p3: usize, p4: usize, offset: usize): T {
	return b3Transpose3V4(v128.load(p1 + offset), v128.load(p2 + offset), v128.load(p3 + offset), v128.load(p4 + offset));
}

unsafe function b3WarmStartContactsV4(
	block: b3SolverBlock,
	context: b3StepContext,
	constraints: StaticArray<b3ContactConstraintWide>,
	manifolds: StaticArray<b3ContactManifoldWide>,
	constraintBase: i32,
	manifoldStarts: Int32Array,
	fixedManifoldCount: i32
): void {
	let states = b3StatesDataV4(context.states);
	let identity = changetype<usize>(b3_identityStates);
	let constraintData = changetype<usize>(constraints);
	let manifoldData = changetype<usize>(manifolds);
	let zeroW = f32x4.splat(0.0);

	let ci = constraintBase + block.startIndex;
	let mi = fixedManifoldCount > 0 ? ci : manifoldStarts[block.startIndex];
	for (let i = 0; i < <i32>block.count; ++i) {
		let c = constraintData + <usize>ci * sizeof<b3ContactConstraintWide>();
		let manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : load<i32>(c + CW_MANIFOLDCOUNT);
		let anyRolling = b3AllZeroV4(v128.load(c + CW_ROLLINGRESISTANCE)) == false;

		let linearImpulseX = zeroW;
		let linearImpulseY = zeroW;
		let linearImpulseZ = zeroW;
		let angularImpulseAX = zeroW;
		let angularImpulseAY = zeroW;
		let angularImpulseAZ = zeroW;
		let angularImpulseBX = zeroW;
		let angularImpulseBY = zeroW;
		let angularImpulseBZ = zeroW;

		for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
			let cm = manifoldData + <usize>(mi + manifoldIndex) * sizeof<b3ContactManifoldWide>();

			let totalNormalImpulse = zeroW;
			let momentAX = zeroW;
			let momentAY = zeroW;
			let momentAZ = zeroW;
			let momentBX = zeroW;
			let momentBY = zeroW;
			let momentBZ = zeroW;

			let pointCount = load<i32>(cm + MW_POINTCOUNT);
			for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
				let cp = cm + MW_POINTS + <usize>pointIndex * sizeof<b3ContactConstraintPointWide>();
				let normalImpulse = v128.load(cp + PW_NORMALIMPULSE);
				totalNormalImpulse = b3AddV4(totalNormalImpulse, normalImpulse);
				momentAX = b3MulAddV4(momentAX, normalImpulse, v128.load(cp + PW_ANCHORA + V_X));
				momentAY = b3MulAddV4(momentAY, normalImpulse, v128.load(cp + PW_ANCHORA + V_Y));
				momentAZ = b3MulAddV4(momentAZ, normalImpulse, v128.load(cp + PW_ANCHORA + V_Z));
				momentBX = b3MulAddV4(momentBX, normalImpulse, v128.load(cp + PW_ANCHORB + V_X));
				momentBY = b3MulAddV4(momentBY, normalImpulse, v128.load(cp + PW_ANCHORB + V_Y));
				momentBZ = b3MulAddV4(momentBZ, normalImpulse, v128.load(cp + PW_ANCHORB + V_Z));
			}

			let normalX = v128.load(cm + MW_NORMAL + V_X);
			let normalY = v128.load(cm + MW_NORMAL + V_Y);
			let normalZ = v128.load(cm + MW_NORMAL + V_Z);
			let frictionImpulseTx = v128.load(cm + MW_FRICTIONIMPULSE + V2_X);
			let frictionImpulseTy = v128.load(cm + MW_FRICTIONIMPULSE + V2_Y);
			let frictionImpulseX = b3MulV4(frictionImpulseTx, v128.load(cm + MW_TANGENT1 + V_X));
			let frictionImpulseY = b3MulV4(frictionImpulseTx, v128.load(cm + MW_TANGENT1 + V_Y));
			let frictionImpulseZ = b3MulV4(frictionImpulseTx, v128.load(cm + MW_TANGENT1 + V_Z));
			frictionImpulseX = b3MulAddV4(frictionImpulseX, frictionImpulseTy, v128.load(cm + MW_TANGENT2 + V_X));
			frictionImpulseY = b3MulAddV4(frictionImpulseY, frictionImpulseTy, v128.load(cm + MW_TANGENT2 + V_Y));
			frictionImpulseZ = b3MulAddV4(frictionImpulseZ, frictionImpulseTy, v128.load(cm + MW_TANGENT2 + V_Z));

			let linearX = b3MulAddV4(frictionImpulseX, totalNormalImpulse, normalX);
			let linearY = b3MulAddV4(frictionImpulseY, totalNormalImpulse, normalY);
			let linearZ = b3MulAddV4(frictionImpulseZ, totalNormalImpulse, normalZ);

			let twistImpulse = v128.load(cm + MW_TWISTIMPULSE);
			let twistImpulseX = b3MulV4(twistImpulse, normalX);
			let twistImpulseY = b3MulV4(twistImpulse, normalY);
			let twistImpulseZ = b3MulV4(twistImpulse, normalZ);

			let centerAX = v128.load(cm + MW_CENTERA + V_X);
			let centerAY = v128.load(cm + MW_CENTERA + V_Y);
			let centerAZ = v128.load(cm + MW_CENTERA + V_Z);
			let centerBX = v128.load(cm + MW_CENTERB + V_X);
			let centerBY = v128.load(cm + MW_CENTERB + V_Y);
			let centerBZ = v128.load(cm + MW_CENTERB + V_Z);

			let angularAX = b3AddV4(
				b3AddV4(
					b3CrossV4(momentAY, momentAZ, normalY, normalZ),
					b3CrossV4(centerAY, centerAZ, frictionImpulseY, frictionImpulseZ)
				),
				twistImpulseX
			);
			let angularAY = b3AddV4(
				b3AddV4(
					b3CrossV4(momentAZ, momentAX, normalZ, normalX),
					b3CrossV4(centerAZ, centerAX, frictionImpulseZ, frictionImpulseX)
				),
				twistImpulseY
			);
			let angularAZ = b3AddV4(
				b3AddV4(
					b3CrossV4(momentAX, momentAY, normalX, normalY),
					b3CrossV4(centerAX, centerAY, frictionImpulseX, frictionImpulseY)
				),
				twistImpulseZ
			);
			let angularBX = b3AddV4(
				b3AddV4(
					b3CrossV4(momentBY, momentBZ, normalY, normalZ),
					b3CrossV4(centerBY, centerBZ, frictionImpulseY, frictionImpulseZ)
				),
				twistImpulseX
			);
			let angularBY = b3AddV4(
				b3AddV4(
					b3CrossV4(momentBZ, momentBX, normalZ, normalX),
					b3CrossV4(centerBZ, centerBX, frictionImpulseZ, frictionImpulseX)
				),
				twistImpulseY
			);
			let angularBZ = b3AddV4(
				b3AddV4(
					b3CrossV4(momentBX, momentBY, normalX, normalY),
					b3CrossV4(centerBX, centerBY, frictionImpulseX, frictionImpulseY)
				),
				twistImpulseZ
			);

			if (anyRolling) {
				let rollingX = v128.load(cm + MW_ROLLINGIMPULSE + V_X);
				let rollingY = v128.load(cm + MW_ROLLINGIMPULSE + V_Y);
				let rollingZ = v128.load(cm + MW_ROLLINGIMPULSE + V_Z);
				angularAX = b3AddV4(angularAX, rollingX);
				angularAY = b3AddV4(angularAY, rollingY);
				angularAZ = b3AddV4(angularAZ, rollingZ);
				angularBX = b3AddV4(angularBX, rollingX);
				angularBY = b3AddV4(angularBY, rollingY);
				angularBZ = b3AddV4(angularBZ, rollingZ);
			}

			linearImpulseX = b3AddV4(linearImpulseX, linearX);
			linearImpulseY = b3AddV4(linearImpulseY, linearY);
			linearImpulseZ = b3AddV4(linearImpulseZ, linearZ);
			angularImpulseAX = b3AddV4(angularImpulseAX, angularAX);
			angularImpulseAY = b3AddV4(angularImpulseAY, angularAY);
			angularImpulseAZ = b3AddV4(angularImpulseAZ, angularAZ);
			angularImpulseBX = b3AddV4(angularImpulseBX, angularBX);
			angularImpulseBY = b3AddV4(angularImpulseBY, angularBY);
			angularImpulseBZ = b3AddV4(angularImpulseBZ, angularBZ);
		}

		let pA1 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA));
		let pA2 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA + 4));
		let pA3 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA + 8));
		let pA4 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA + 12));
		let pB1 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB));
		let pB2 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB + 4));
		let pB3 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB + 8));
		let pB4 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB + 12));
		let vAX = b3GatherXV4<v128>(pA1, pA2, pA3, pA4, BS_V);
		let vAY = b3GatherYV4<v128>(pA1, pA2, pA3, pA4, BS_V);
		let vAZ = b3GatherZV4<v128>(pA1, pA2, pA3, pA4, BS_V);
		let wAX = b3GatherXV4<v128>(pA1, pA2, pA3, pA4, BS_W);
		let wAY = b3GatherYV4<v128>(pA1, pA2, pA3, pA4, BS_W);
		let wAZ = b3GatherZV4<v128>(pA1, pA2, pA3, pA4, BS_W);
		let vBX = b3GatherXV4<v128>(pB1, pB2, pB3, pB4, BS_V);
		let vBY = b3GatherYV4<v128>(pB1, pB2, pB3, pB4, BS_V);
		let vBZ = b3GatherZV4<v128>(pB1, pB2, pB3, pB4, BS_V);
		let wBX = b3GatherXV4<v128>(pB1, pB2, pB3, pB4, BS_W);
		let wBY = b3GatherYV4<v128>(pB1, pB2, pB3, pB4, BS_W);
		let wBZ = b3GatherZV4<v128>(pB1, pB2, pB3, pB4, BS_W);

		let iA = c + CW_INVIA;
		wAX = b3SubV4(wAX, b3MulRowV4(v128.load(iA + S_CXX), v128.load(iA + S_CXY), v128.load(iA + S_CXZ), angularImpulseAX, angularImpulseAY, angularImpulseAZ));
		wAY = b3SubV4(wAY, b3MulRowV4(v128.load(iA + S_CXY), v128.load(iA + S_CYY), v128.load(iA + S_CYZ), angularImpulseAX, angularImpulseAY, angularImpulseAZ));
		wAZ = b3SubV4(wAZ, b3MulRowV4(v128.load(iA + S_CXZ), v128.load(iA + S_CYZ), v128.load(iA + S_CZZ), angularImpulseAX, angularImpulseAY, angularImpulseAZ));
		let invMassA = v128.load(c + CW_INVMASSA);
		vAX = b3MulSubV4(vAX, invMassA, linearImpulseX);
		vAY = b3MulSubV4(vAY, invMassA, linearImpulseY);
		vAZ = b3MulSubV4(vAZ, invMassA, linearImpulseZ);
		let iB = c + CW_INVIB;
		wBX = b3AddV4(wBX, b3MulRowV4(v128.load(iB + S_CXX), v128.load(iB + S_CXY), v128.load(iB + S_CXZ), angularImpulseBX, angularImpulseBY, angularImpulseBZ));
		wBY = b3AddV4(wBY, b3MulRowV4(v128.load(iB + S_CXY), v128.load(iB + S_CYY), v128.load(iB + S_CYZ), angularImpulseBX, angularImpulseBY, angularImpulseBZ));
		wBZ = b3AddV4(wBZ, b3MulRowV4(v128.load(iB + S_CXZ), v128.load(iB + S_CYZ), v128.load(iB + S_CZZ), angularImpulseBX, angularImpulseBY, angularImpulseBZ));
		let invMassB = v128.load(c + CW_INVMASSB);
		vBX = b3MulAddV4(vBX, invMassB, linearImpulseX);
		vBY = b3MulAddV4(vBY, invMassB, linearImpulseY);
		vBZ = b3MulAddV4(vBZ, invMassB, linearImpulseZ);

		b3ScatterBodiesV4(states, c + CW_INDEXA, vAX, vAY, vAZ, wAX, wAY, wAZ);
		b3ScatterBodiesV4(states, c + CW_INDEXB, vBX, vBY, vBZ, wBX, wBY, wBZ);

		ci += 1;
		mi += manifoldCount;
	}
}

// Solve the non-penetration constraints with the soft bias. No friction and no restitution.
unsafe function b3PushContactsV4(
	block: b3SolverBlock,
	context: b3StepContext,
	constraints: StaticArray<b3ContactConstraintWide>,
	manifolds: StaticArray<b3ContactManifoldWide>,
	constraintBase: i32,
	manifoldStarts: Int32Array,
	fixedManifoldCount: i32
): void {
	let states = b3StatesDataV4(context.states);
	let identity = changetype<usize>(b3_identityStates);
	let constraintData = changetype<usize>(constraints);
	let manifoldData = changetype<usize>(manifolds);
	let inv_h = f32x4.splat(context.inv_h);
	let contactSpeed = f32x4.splat(-context.world.contactSpeed);
	let oneW = f32x4.splat(1.0);
	let zeroW = f32x4.splat(0.0);

	let dynamicBiasRate = f32x4.splat(context.contactSoftness.massScale * context.contactSoftness.biasRate);
	let dynamicMassScale = f32x4.splat(context.contactSoftness.massScale);
	let dynamicImpulseScale = f32x4.splat(context.contactSoftness.impulseScale);
	let staticBiasRate = f32x4.splat(context.staticSoftness.massScale * context.staticSoftness.biasRate);
	let staticMassScale = f32x4.splat(context.staticSoftness.massScale);
	let staticImpulseScale = f32x4.splat(context.staticSoftness.impulseScale);

	let ci = constraintBase + block.startIndex;
	let mi = fixedManifoldCount > 0 ? ci : manifoldStarts[block.startIndex];
	for (let i = 0; i < <i32>block.count; ++i) {
		let c = constraintData + <usize>ci * sizeof<b3ContactConstraintWide>();
		let manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : load<i32>(c + CW_MANIFOLDCOUNT);

		let pA1 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA));
		let pA2 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA + 4));
		let pA3 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA + 8));
		let pA4 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA + 12));
		let pB1 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB));
		let pB2 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB + 4));
		let pB3 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB + 8));
		let pB4 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB + 12));
		let vAX = b3GatherXV4<v128>(pA1, pA2, pA3, pA4, BS_V);
		let vAY = b3GatherYV4<v128>(pA1, pA2, pA3, pA4, BS_V);
		let vAZ = b3GatherZV4<v128>(pA1, pA2, pA3, pA4, BS_V);
		let wAX = b3GatherXV4<v128>(pA1, pA2, pA3, pA4, BS_W);
		let wAY = b3GatherYV4<v128>(pA1, pA2, pA3, pA4, BS_W);
		let wAZ = b3GatherZV4<v128>(pA1, pA2, pA3, pA4, BS_W);
		let dpAX = b3GatherXV4<v128>(pA1, pA2, pA3, pA4, BS_DP);
		let dpAY = b3GatherYV4<v128>(pA1, pA2, pA3, pA4, BS_DP);
		let dpAZ = b3GatherZV4<v128>(pA1, pA2, pA3, pA4, BS_DP);
		let dqAX = b3GatherXV4<v128>(pA1, pA2, pA3, pA4, BS_DQ);
		let dqAY = b3GatherYV4<v128>(pA1, pA2, pA3, pA4, BS_DQ);
		let dqAZ = b3GatherZV4<v128>(pA1, pA2, pA3, pA4, BS_DQ);
		let dqAS = b3GatherWV4<v128>(pA1, pA2, pA3, pA4, BS_DQ);
		let vBX = b3GatherXV4<v128>(pB1, pB2, pB3, pB4, BS_V);
		let vBY = b3GatherYV4<v128>(pB1, pB2, pB3, pB4, BS_V);
		let vBZ = b3GatherZV4<v128>(pB1, pB2, pB3, pB4, BS_V);
		let wBX = b3GatherXV4<v128>(pB1, pB2, pB3, pB4, BS_W);
		let wBY = b3GatherYV4<v128>(pB1, pB2, pB3, pB4, BS_W);
		let wBZ = b3GatherZV4<v128>(pB1, pB2, pB3, pB4, BS_W);
		let dpBX = b3GatherXV4<v128>(pB1, pB2, pB3, pB4, BS_DP);
		let dpBY = b3GatherYV4<v128>(pB1, pB2, pB3, pB4, BS_DP);
		let dpBZ = b3GatherZV4<v128>(pB1, pB2, pB3, pB4, BS_DP);
		let dqBX = b3GatherXV4<v128>(pB1, pB2, pB3, pB4, BS_DQ);
		let dqBY = b3GatherYV4<v128>(pB1, pB2, pB3, pB4, BS_DQ);
		let dqBZ = b3GatherZV4<v128>(pB1, pB2, pB3, pB4, BS_DQ);
		let dqBS = b3GatherWV4<v128>(pB1, pB2, pB3, pB4, BS_DQ);

		// b3SoftMaskW
		let softMask = v128.or(
			i32x4.eq(v128.load(c + CW_INDEXA), i32x4.splat(0)),
			i32x4.eq(v128.load(c + CW_INDEXB), i32x4.splat(0))
		);
		let biasRate = b3BlendV4(dynamicBiasRate, staticBiasRate, softMask);
		let massScale = b3BlendV4(dynamicMassScale, staticMassScale, softMask);
		let impulseScale = b3BlendV4(dynamicImpulseScale, staticImpulseScale, softMask);

		let dpX = b3SubV4(dpBX, dpAX);
		let dpY = b3SubV4(dpBY, dpAY);
		let dpZ = b3SubV4(dpBZ, dpAZ);
		let invMassA = v128.load(c + CW_INVMASSA);
		let invMassB = v128.load(c + CW_INVMASSB);
		let invMass = b3AddV4(invMassA, invMassB);
		let iA = c + CW_INVIA;
		let iB = c + CW_INVIB;

		for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
			let cm = manifoldData + <usize>(mi + manifoldIndex) * sizeof<b3ContactManifoldWide>();
			let normalX = v128.load(cm + MW_NORMAL + V_X);
			let normalY = v128.load(cm + MW_NORMAL + V_Y);
			let normalZ = v128.load(cm + MW_NORMAL + V_Z);

			// Convert to normals to local space to reduce transform math.
			let normalSeparation = b3DotV4(normalX, normalY, normalZ, dpX, dpY, dpZ);

			// b3InvRotateVectorW( bA.dq, normal )
			let tAX = b3CrossV4(dqAY, dqAZ, normalY, normalZ);
			let tAY = b3CrossV4(dqAZ, dqAX, normalZ, normalX);
			let tAZ = b3CrossV4(dqAX, dqAY, normalX, normalY);
			tAX = b3AddV4(tAX, tAX);
			tAY = b3AddV4(tAY, tAY);
			tAZ = b3AddV4(tAZ, tAZ);
			let normalAX = b3AddV4(b3MulSubV4(normalX, dqAS, tAX), b3CrossV4(dqAY, dqAZ, tAY, tAZ));
			let normalAY = b3AddV4(b3MulSubV4(normalY, dqAS, tAY), b3CrossV4(dqAZ, dqAX, tAZ, tAX));
			let normalAZ = b3AddV4(b3MulSubV4(normalZ, dqAS, tAZ), b3CrossV4(dqAX, dqAY, tAX, tAY));

			// b3InvRotateVectorW( bB.dq, normal )
			let tBX = b3CrossV4(dqBY, dqBZ, normalY, normalZ);
			let tBY = b3CrossV4(dqBZ, dqBX, normalZ, normalX);
			let tBZ = b3CrossV4(dqBX, dqBY, normalX, normalY);
			tBX = b3AddV4(tBX, tBX);
			tBY = b3AddV4(tBY, tBY);
			tBZ = b3AddV4(tBZ, tBZ);
			let normalBX = b3AddV4(b3MulSubV4(normalX, dqBS, tBX), b3CrossV4(dqBY, dqBZ, tBY, tBZ));
			let normalBY = b3AddV4(b3MulSubV4(normalY, dqBS, tBY), b3CrossV4(dqBZ, dqBX, tBZ, tBX));
			let normalBZ = b3AddV4(b3MulSubV4(normalZ, dqBS, tBZ), b3CrossV4(dqBX, dqBY, tBX, tBY));

			let linearNormalVelocity = b3DotV4(normalX, normalY, normalZ, b3SubV4(vBX, vAX), b3SubV4(vBY, vAY), b3SubV4(vBZ, vAZ));
			let manifoldImpulse = zeroW;

			let pointCount = load<i32>(cm + MW_POINTCOUNT);
			for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
				let cp = cm + MW_POINTS + <usize>pointIndex * sizeof<b3ContactConstraintPointWide>();

				// Fixed anchor points for applying impulses
				let rAX = v128.load(cp + PW_ANCHORA + V_X);
				let rAY = v128.load(cp + PW_ANCHORA + V_Y);
				let rAZ = v128.load(cp + PW_ANCHORA + V_Z);
				let rBX = v128.load(cp + PW_ANCHORB + V_X);
				let rBY = v128.load(cp + PW_ANCHORB + V_Y);
				let rBZ = v128.load(cp + PW_ANCHORB + V_Z);

				let s = b3AddV4(
					b3AddV4(
						normalSeparation,
						b3SubV4(b3DotV4(normalBX, normalBY, normalBZ, rBX, rBY, rBZ), b3DotV4(normalAX, normalAY, normalAZ, rAX, rAY, rAZ))
					),
					v128.load(cp + PW_BASESEPARATION)
				);

				// Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
				let separated = f32x4.gt(s, zeroW);

				// Speculative bias - positive
				let specBias = b3MulV4(s, inv_h);

				// Overlap bias - negative
				let overlapBias = b3MaxV4(b3MulV4(biasRate, s), contactSpeed);
				let velocityBias = b3BlendV4(overlapBias, specBias, separated);

				let pointMassScale = b3BlendV4(massScale, oneW, separated);
				let pointImpulseScale = b3BlendV4(impulseScale, zeroW, separated);

				let uAX = b3CrossV4(rAY, rAZ, normalY, normalZ);
				let uAY = b3CrossV4(rAZ, rAX, normalZ, normalX);
				let uAZ = b3CrossV4(rAX, rAY, normalX, normalY);
				let uBX = b3CrossV4(rBY, rBZ, normalY, normalZ);
				let uBY = b3CrossV4(rBZ, rBX, normalZ, normalX);
				let uBZ = b3CrossV4(rBX, rBY, normalX, normalY);
				let gAX = b3MulRowV4(v128.load(iA + S_CXX), v128.load(iA + S_CXY), v128.load(iA + S_CXZ), uAX, uAY, uAZ);
				let gAY = b3MulRowV4(v128.load(iA + S_CXY), v128.load(iA + S_CYY), v128.load(iA + S_CYZ), uAX, uAY, uAZ);
				let gAZ = b3MulRowV4(v128.load(iA + S_CXZ), v128.load(iA + S_CYZ), v128.load(iA + S_CZZ), uAX, uAY, uAZ);
				let gBX = b3MulRowV4(v128.load(iB + S_CXX), v128.load(iB + S_CXY), v128.load(iB + S_CXZ), uBX, uBY, uBZ);
				let gBY = b3MulRowV4(v128.load(iB + S_CXY), v128.load(iB + S_CYY), v128.load(iB + S_CYZ), uBX, uBY, uBZ);
				let gBZ = b3MulRowV4(v128.load(iB + S_CXZ), v128.load(iB + S_CYZ), v128.load(iB + S_CZZ), uBX, uBY, uBZ);

				let vn = b3AddV4(linearNormalVelocity, b3SubV4(b3DotV4(uBX, uBY, uBZ, wBX, wBY, wBZ), b3DotV4(uAX, uAY, uAZ, wAX, wAY, wAZ)));

				let normalImpulse = v128.load(cp + PW_NORMALIMPULSE);
				let negImpulse = b3AddV4(
					b3MulV4(v128.load(cp + PW_NORMALMASS), b3AddV4(b3MulV4(pointMassScale, vn), velocityBias)),
					b3MulV4(pointImpulseScale, normalImpulse)
				);
				let impulse = b3MaxV4(b3NegV4(negImpulse), b3NegV4(normalImpulse));
				v128.store(cp + PW_NORMALIMPULSE, b3AddV4(normalImpulse, impulse));

				linearNormalVelocity = b3MulAddV4(linearNormalVelocity, invMass, impulse);
				manifoldImpulse = b3AddV4(manifoldImpulse, impulse);
				wAX = b3MulSubV4(wAX, impulse, gAX);
				wAY = b3MulSubV4(wAY, impulse, gAY);
				wAZ = b3MulSubV4(wAZ, impulse, gAZ);
				wBX = b3MulAddV4(wBX, impulse, gBX);
				wBY = b3MulAddV4(wBY, impulse, gBY);
				wBZ = b3MulAddV4(wBZ, impulse, gBZ);
			}

			let PX = b3MulV4(manifoldImpulse, normalX);
			let PY = b3MulV4(manifoldImpulse, normalY);
			let PZ = b3MulV4(manifoldImpulse, normalZ);
			vAX = b3MulSubV4(vAX, invMassA, PX);
			vAY = b3MulSubV4(vAY, invMassA, PY);
			vAZ = b3MulSubV4(vAZ, invMassA, PZ);
			vBX = b3MulAddV4(vBX, invMassB, PX);
			vBY = b3MulAddV4(vBY, invMassB, PY);
			vBZ = b3MulAddV4(vBZ, invMassB, PZ);
		}

		b3ScatterBodiesV4(states, c + CW_INDEXA, vAX, vAY, vAZ, wAX, wAY, wAZ);
		b3ScatterBodiesV4(states, c + CW_INDEXB, vBX, vBY, vBZ, wBX, wBY, wBZ);

		ci += 1;
		mi += manifoldCount;
	}
}

// Solve the normal constraint, friction, and rolling resistance.
unsafe function b3SolveContactsV4(
	block: b3SolverBlock,
	context: b3StepContext,
	constraints: StaticArray<b3ContactConstraintWide>,
	manifolds: StaticArray<b3ContactManifoldWide>,
	constraintBase: i32,
	manifoldStarts: Int32Array,
	fixedManifoldCount: i32
): void {
	let states = b3StatesDataV4(context.states);
	let identity = changetype<usize>(b3_identityStates);
	let constraintData = changetype<usize>(constraints);
	let manifoldData = changetype<usize>(manifolds);
	let inv_h = f32x4.splat(context.inv_h);
	let oneW = f32x4.splat(1.0);
	let zeroW = f32x4.splat(0.0);
	let epsilonW = f32x4.splat(FLT_EPSILON);

	let ci = constraintBase + block.startIndex;
	let mi = fixedManifoldCount > 0 ? ci : manifoldStarts[block.startIndex];
	for (let i = 0; i < <i32>block.count; ++i) {
		let c = constraintData + <usize>ci * sizeof<b3ContactConstraintWide>();
		let manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : load<i32>(c + CW_MANIFOLDCOUNT);

		let pA1 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA));
		let pA2 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA + 4));
		let pA3 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA + 8));
		let pA4 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA + 12));
		let pB1 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB));
		let pB2 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB + 4));
		let pB3 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB + 8));
		let pB4 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB + 12));
		let vAX = b3GatherXV4<v128>(pA1, pA2, pA3, pA4, BS_V);
		let vAY = b3GatherYV4<v128>(pA1, pA2, pA3, pA4, BS_V);
		let vAZ = b3GatherZV4<v128>(pA1, pA2, pA3, pA4, BS_V);
		let wAX = b3GatherXV4<v128>(pA1, pA2, pA3, pA4, BS_W);
		let wAY = b3GatherYV4<v128>(pA1, pA2, pA3, pA4, BS_W);
		let wAZ = b3GatherZV4<v128>(pA1, pA2, pA3, pA4, BS_W);
		let dpAX = b3GatherXV4<v128>(pA1, pA2, pA3, pA4, BS_DP);
		let dpAY = b3GatherYV4<v128>(pA1, pA2, pA3, pA4, BS_DP);
		let dpAZ = b3GatherZV4<v128>(pA1, pA2, pA3, pA4, BS_DP);
		let dqAX = b3GatherXV4<v128>(pA1, pA2, pA3, pA4, BS_DQ);
		let dqAY = b3GatherYV4<v128>(pA1, pA2, pA3, pA4, BS_DQ);
		let dqAZ = b3GatherZV4<v128>(pA1, pA2, pA3, pA4, BS_DQ);
		let dqAS = b3GatherWV4<v128>(pA1, pA2, pA3, pA4, BS_DQ);
		let vBX = b3GatherXV4<v128>(pB1, pB2, pB3, pB4, BS_V);
		let vBY = b3GatherYV4<v128>(pB1, pB2, pB3, pB4, BS_V);
		let vBZ = b3GatherZV4<v128>(pB1, pB2, pB3, pB4, BS_V);
		let wBX = b3GatherXV4<v128>(pB1, pB2, pB3, pB4, BS_W);
		let wBY = b3GatherYV4<v128>(pB1, pB2, pB3, pB4, BS_W);
		let wBZ = b3GatherZV4<v128>(pB1, pB2, pB3, pB4, BS_W);
		let dpBX = b3GatherXV4<v128>(pB1, pB2, pB3, pB4, BS_DP);
		let dpBY = b3GatherYV4<v128>(pB1, pB2, pB3, pB4, BS_DP);
		let dpBZ = b3GatherZV4<v128>(pB1, pB2, pB3, pB4, BS_DP);
		let dqBX = b3GatherXV4<v128>(pB1, pB2, pB3, pB4, BS_DQ);
		let dqBY = b3GatherYV4<v128>(pB1, pB2, pB3, pB4, BS_DQ);
		let dqBZ = b3GatherZV4<v128>(pB1, pB2, pB3, pB4, BS_DQ);
		let dqBS = b3GatherWV4<v128>(pB1, pB2, pB3, pB4, BS_DQ);

		let anyRolling = b3AllZeroV4(v128.load(c + CW_ROLLINGRESISTANCE)) == false;
		let dpX = b3SubV4(dpBX, dpAX);
		let dpY = b3SubV4(dpBY, dpAY);
		let dpZ = b3SubV4(dpBZ, dpAZ);
		let invMassA = v128.load(c + CW_INVMASSA);
		let invMassB = v128.load(c + CW_INVMASSB);
		let invMass = b3AddV4(invMassA, invMassB);
		let iA = c + CW_INVIA;
		let iB = c + CW_INVIB;

		for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
			let cm = manifoldData + <usize>(mi + manifoldIndex) * sizeof<b3ContactManifoldWide>();
			let normalX = v128.load(cm + MW_NORMAL + V_X);
			let normalY = v128.load(cm + MW_NORMAL + V_Y);
			let normalZ = v128.load(cm + MW_NORMAL + V_Z);

			let normalSeparation = b3DotV4(normalX, normalY, normalZ, dpX, dpY, dpZ);

			// b3InvRotateVectorW( bA.dq, normal )
			let tAX = b3CrossV4(dqAY, dqAZ, normalY, normalZ);
			let tAY = b3CrossV4(dqAZ, dqAX, normalZ, normalX);
			let tAZ = b3CrossV4(dqAX, dqAY, normalX, normalY);
			tAX = b3AddV4(tAX, tAX);
			tAY = b3AddV4(tAY, tAY);
			tAZ = b3AddV4(tAZ, tAZ);
			let normalAX = b3AddV4(b3MulSubV4(normalX, dqAS, tAX), b3CrossV4(dqAY, dqAZ, tAY, tAZ));
			let normalAY = b3AddV4(b3MulSubV4(normalY, dqAS, tAY), b3CrossV4(dqAZ, dqAX, tAZ, tAX));
			let normalAZ = b3AddV4(b3MulSubV4(normalZ, dqAS, tAZ), b3CrossV4(dqAX, dqAY, tAX, tAY));

			// b3InvRotateVectorW( bB.dq, normal )
			let tBX = b3CrossV4(dqBY, dqBZ, normalY, normalZ);
			let tBY = b3CrossV4(dqBZ, dqBX, normalZ, normalX);
			let tBZ = b3CrossV4(dqBX, dqBY, normalX, normalY);
			tBX = b3AddV4(tBX, tBX);
			tBY = b3AddV4(tBY, tBY);
			tBZ = b3AddV4(tBZ, tBZ);
			let normalBX = b3AddV4(b3MulSubV4(normalX, dqBS, tBX), b3CrossV4(dqBY, dqBZ, tBY, tBZ));
			let normalBY = b3AddV4(b3MulSubV4(normalY, dqBS, tBY), b3CrossV4(dqBZ, dqBX, tBZ, tBX));
			let normalBZ = b3AddV4(b3MulSubV4(normalZ, dqBS, tBZ), b3CrossV4(dqBX, dqBY, tBX, tBY));

			let dvX = b3SubV4(vBX, vAX);
			let dvY = b3SubV4(vBY, vAY);
			let dvZ = b3SubV4(vBZ, vAZ);
			let tangent1X = v128.load(cm + MW_TANGENT1 + V_X);
			let tangent1Y = v128.load(cm + MW_TANGENT1 + V_Y);
			let tangent1Z = v128.load(cm + MW_TANGENT1 + V_Z);
			let tangent2X = v128.load(cm + MW_TANGENT2 + V_X);
			let tangent2Y = v128.load(cm + MW_TANGENT2 + V_Y);
			let tangent2Z = v128.load(cm + MW_TANGENT2 + V_Z);
			let linearNormalVelocity = b3DotV4(normalX, normalY, normalZ, dvX, dvY, dvZ);
			let linearTangentVelocity1 = b3SubV4(
				b3DotV4(tangent1X, tangent1Y, tangent1Z, dvX, dvY, dvZ),
				v128.load(cm + MW_TANGENTVELOCITY1)
			);
			let linearTangentVelocity2 = b3SubV4(
				b3DotV4(tangent2X, tangent2Y, tangent2Z, dvX, dvY, dvZ),
				v128.load(cm + MW_TANGENTVELOCITY2)
			);
			let manifoldImpulse = zeroW;

			let totalNormalImpulse = zeroW;
			let totalTwistLimit = zeroW;

			// Normal contraints
			let pointCount = load<i32>(cm + MW_POINTCOUNT);
			for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
				let cp = cm + MW_POINTS + <usize>pointIndex * sizeof<b3ContactConstraintPointWide>();

				// Fixed anchor points for applying impulses
				let rAX = v128.load(cp + PW_ANCHORA + V_X);
				let rAY = v128.load(cp + PW_ANCHORA + V_Y);
				let rAZ = v128.load(cp + PW_ANCHORA + V_Z);
				let rBX = v128.load(cp + PW_ANCHORB + V_X);
				let rBY = v128.load(cp + PW_ANCHORB + V_Y);
				let rBZ = v128.load(cp + PW_ANCHORB + V_Z);

				let s = b3AddV4(
					b3AddV4(
						normalSeparation,
						b3SubV4(b3DotV4(normalBX, normalBY, normalBZ, rBX, rBY, rBZ), b3DotV4(normalAX, normalAY, normalAZ, rAX, rAY, rAZ))
					),
					v128.load(cp + PW_BASESEPARATION)
				);

				// Speculative bias, positive if separated and zero if overlapped
				let velocityBias = b3MaxV4(zeroW, b3MulV4(s, inv_h));

				let uAX = b3CrossV4(rAY, rAZ, normalY, normalZ);
				let uAY = b3CrossV4(rAZ, rAX, normalZ, normalX);
				let uAZ = b3CrossV4(rAX, rAY, normalX, normalY);
				let uBX = b3CrossV4(rBY, rBZ, normalY, normalZ);
				let uBY = b3CrossV4(rBZ, rBX, normalZ, normalX);
				let uBZ = b3CrossV4(rBX, rBY, normalX, normalY);
				let gAX = b3MulRowV4(v128.load(iA + S_CXX), v128.load(iA + S_CXY), v128.load(iA + S_CXZ), uAX, uAY, uAZ);
				let gAY = b3MulRowV4(v128.load(iA + S_CXY), v128.load(iA + S_CYY), v128.load(iA + S_CYZ), uAX, uAY, uAZ);
				let gAZ = b3MulRowV4(v128.load(iA + S_CXZ), v128.load(iA + S_CYZ), v128.load(iA + S_CZZ), uAX, uAY, uAZ);
				let gBX = b3MulRowV4(v128.load(iB + S_CXX), v128.load(iB + S_CXY), v128.load(iB + S_CXZ), uBX, uBY, uBZ);
				let gBY = b3MulRowV4(v128.load(iB + S_CXY), v128.load(iB + S_CYY), v128.load(iB + S_CYZ), uBX, uBY, uBZ);
				let gBZ = b3MulRowV4(v128.load(iB + S_CXZ), v128.load(iB + S_CYZ), v128.load(iB + S_CZZ), uBX, uBY, uBZ);

				let vn = b3AddV4(linearNormalVelocity, b3SubV4(b3DotV4(uBX, uBY, uBZ, wBX, wBY, wBZ), b3DotV4(uAX, uAY, uAZ, wAX, wAY, wAZ)));

				let normalImpulse = v128.load(cp + PW_NORMALIMPULSE);
				let negImpulse = b3MulV4(v128.load(cp + PW_NORMALMASS), b3AddV4(vn, velocityBias));
				let impulse = b3MaxV4(b3NegV4(negImpulse), b3NegV4(normalImpulse));
				let newImpulse = b3AddV4(normalImpulse, impulse);
				v128.store(cp + PW_NORMALIMPULSE, newImpulse);
				v128.store(cp + PW_TOTALNORMALIMPULSE, b3AddV4(v128.load(cp + PW_TOTALNORMALIMPULSE), newImpulse));
				totalNormalImpulse = b3AddV4(totalNormalImpulse, newImpulse);
				totalTwistLimit = b3AddV4(totalTwistLimit, b3MulV4(v128.load(cp + PW_LEVERARM), newImpulse));

				linearNormalVelocity = b3MulAddV4(linearNormalVelocity, invMass, impulse);
				manifoldImpulse = b3AddV4(manifoldImpulse, impulse);
				wAX = b3MulSubV4(wAX, impulse, gAX);
				wAY = b3MulSubV4(wAY, impulse, gAY);
				wAZ = b3MulSubV4(wAZ, impulse, gAZ);
				wBX = b3MulAddV4(wBX, impulse, gBX);
				wBY = b3MulAddV4(wBY, impulse, gBY);
				wBZ = b3MulAddV4(wBZ, impulse, gBZ);
			}

			// Rolling resistance. After normal constraints because it uses the normal impulse.
			if (anyRolling) {
				// flip A/B order to negate
				let rm = c + CW_ROLLINGMASS;
				let dwX = b3SubV4(wAX, wBX);
				let dwY = b3SubV4(wAY, wBY);
				let dwZ = b3SubV4(wAZ, wBZ);
				let deltaImpulseX = b3MulRowV4(v128.load(rm + S_CXX), v128.load(rm + S_CXY), v128.load(rm + S_CXZ), dwX, dwY, dwZ);
				let deltaImpulseY = b3MulRowV4(v128.load(rm + S_CXY), v128.load(rm + S_CYY), v128.load(rm + S_CYZ), dwX, dwY, dwZ);
				let deltaImpulseZ = b3MulRowV4(v128.load(rm + S_CXZ), v128.load(rm + S_CYZ), v128.load(rm + S_CZZ), dwX, dwY, dwZ);
				let oldImpulseX = v128.load(cm + MW_ROLLINGIMPULSE + V_X);
				let oldImpulseY = v128.load(cm + MW_ROLLINGIMPULSE + V_Y);
				let oldImpulseZ = v128.load(cm + MW_ROLLINGIMPULSE + V_Z);
				let rollingImpulseX = b3AddV4(oldImpulseX, deltaImpulseX);
				let rollingImpulseY = b3AddV4(oldImpulseY, deltaImpulseY);
				let rollingImpulseZ = b3AddV4(oldImpulseZ, deltaImpulseZ);

				let maxImpulse = b3MulV4(v128.load(c + CW_ROLLINGRESISTANCE), totalNormalImpulse);
				let lengthSquared = b3DotV4(rollingImpulseX, rollingImpulseY, rollingImpulseZ, rollingImpulseX, rollingImpulseY, rollingImpulseZ);

				let mask = f32x4.gt(lengthSquared, b3MulAddV4(epsilonW, maxImpulse, maxImpulse));

				// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism
				let normalize = b3DivV4(maxImpulse, b3AddV4(b3SqrtV4(lengthSquared), epsilonW));
				let scale = b3BlendV4(oneW, normalize, mask);

				// Blend the result to ensure there are no +/-0 determinism problems, since 0 * -1 == -0.
				let rollingMask = b3MaskV4(v128.load(cm + MW_ROLLINGMASK));
				rollingImpulseX = b3BlendV4(zeroW, b3MulV4(scale, rollingImpulseX), rollingMask);
				rollingImpulseY = b3BlendV4(zeroW, b3MulV4(scale, rollingImpulseY), rollingMask);
				rollingImpulseZ = b3BlendV4(zeroW, b3MulV4(scale, rollingImpulseZ), rollingMask);
				v128.store(cm + MW_ROLLINGIMPULSE + V_X, rollingImpulseX);
				v128.store(cm + MW_ROLLINGIMPULSE + V_Y, rollingImpulseY);
				v128.store(cm + MW_ROLLINGIMPULSE + V_Z, rollingImpulseZ);

				deltaImpulseX = b3SubV4(rollingImpulseX, oldImpulseX);
				deltaImpulseY = b3SubV4(rollingImpulseY, oldImpulseY);
				deltaImpulseZ = b3SubV4(rollingImpulseZ, oldImpulseZ);

				wAX = b3SubV4(wAX, b3MulRowV4(v128.load(iA + S_CXX), v128.load(iA + S_CXY), v128.load(iA + S_CXZ), deltaImpulseX, deltaImpulseY, deltaImpulseZ));
				wAY = b3SubV4(wAY, b3MulRowV4(v128.load(iA + S_CXY), v128.load(iA + S_CYY), v128.load(iA + S_CYZ), deltaImpulseX, deltaImpulseY, deltaImpulseZ));
				wAZ = b3SubV4(wAZ, b3MulRowV4(v128.load(iA + S_CXZ), v128.load(iA + S_CYZ), v128.load(iA + S_CZZ), deltaImpulseX, deltaImpulseY, deltaImpulseZ));
				wBX = b3AddV4(wBX, b3MulRowV4(v128.load(iB + S_CXX), v128.load(iB + S_CXY), v128.load(iB + S_CXZ), deltaImpulseX, deltaImpulseY, deltaImpulseZ));
				wBY = b3AddV4(wBY, b3MulRowV4(v128.load(iB + S_CXY), v128.load(iB + S_CYY), v128.load(iB + S_CYZ), deltaImpulseX, deltaImpulseY, deltaImpulseZ));
				wBZ = b3AddV4(wBZ, b3MulRowV4(v128.load(iB + S_CXZ), v128.load(iB + S_CYZ), v128.load(iB + S_CZZ), deltaImpulseX, deltaImpulseY, deltaImpulseZ));
			}

			let friction = v128.load(c + CW_FRICTION);

			// Central twist friction. Friction goes last to improve stacking stability.
			{
				let twistSpeed = b3DotV4(normalX, normalY, normalZ, b3SubV4(wBX, wAX), b3SubV4(wBY, wAY), b3SubV4(wBZ, wAZ));
				let maxLambda = b3MulV4(friction, totalTwistLimit);
				let oldImpulse = v128.load(cm + MW_TWISTIMPULSE);
				let impulse = b3NegV4(b3MulV4(v128.load(cm + MW_TWISTMASS), twistSpeed));
				impulse = b3MaxV4(b3SubV4(b3NegV4(maxLambda), oldImpulse), b3MinV4(impulse, b3SubV4(maxLambda, oldImpulse)));
				v128.store(cm + MW_TWISTIMPULSE, b3AddV4(oldImpulse, impulse));

				wAX = b3MulSubV4(wAX, impulse, b3MulRowV4(v128.load(iA + S_CXX), v128.load(iA + S_CXY), v128.load(iA + S_CXZ), normalX, normalY, normalZ));
				wAY = b3MulSubV4(wAY, impulse, b3MulRowV4(v128.load(iA + S_CXY), v128.load(iA + S_CYY), v128.load(iA + S_CYZ), normalX, normalY, normalZ));
				wAZ = b3MulSubV4(wAZ, impulse, b3MulRowV4(v128.load(iA + S_CXZ), v128.load(iA + S_CYZ), v128.load(iA + S_CZZ), normalX, normalY, normalZ));
				wBX = b3MulAddV4(wBX, impulse, b3MulRowV4(v128.load(iB + S_CXX), v128.load(iB + S_CXY), v128.load(iB + S_CXZ), normalX, normalY, normalZ));
				wBY = b3MulAddV4(wBY, impulse, b3MulRowV4(v128.load(iB + S_CXY), v128.load(iB + S_CYY), v128.load(iB + S_CYZ), normalX, normalY, normalZ));
				wBZ = b3MulAddV4(wBZ, impulse, b3MulRowV4(v128.load(iB + S_CXZ), v128.load(iB + S_CYZ), v128.load(iB + S_CZZ), normalX, normalY, normalZ));
			}

			// Central friction. Friction goes last to improve stacking stability.
			{
				// Fixed anchor points for applying impulses
				let rAX = v128.load(cm + MW_CENTERA + V_X);
				let rAY = v128.load(cm + MW_CENTERA + V_Y);
				let rAZ = v128.load(cm + MW_CENTERA + V_Z);
				let rBX = v128.load(cm + MW_CENTERB + V_X);
				let rBY = v128.load(cm + MW_CENTERB + V_Y);
				let rBZ = v128.load(cm + MW_CENTERB + V_Z);

				let angularRelativeVelocityX = b3SubV4(b3CrossV4(wBY, wBZ, rBY, rBZ), b3CrossV4(wAY, wAZ, rAY, rAZ));
				let angularRelativeVelocityY = b3SubV4(b3CrossV4(wBZ, wBX, rBZ, rBX), b3CrossV4(wAZ, wAX, rAZ, rAX));
				let angularRelativeVelocityZ = b3SubV4(b3CrossV4(wBX, wBY, rBX, rBY), b3CrossV4(wAX, wAY, rAX, rAY));

				// Relative tangent velocity at contact
				let vtx = b3AddV4(
					linearTangentVelocity1,
					b3DotV4(angularRelativeVelocityX, angularRelativeVelocityY, angularRelativeVelocityZ, tangent1X, tangent1Y, tangent1Z)
				);
				let vty = b3AddV4(
					linearTangentVelocity2,
					b3DotV4(angularRelativeVelocityX, angularRelativeVelocityY, angularRelativeVelocityZ, tangent2X, tangent2Y, tangent2Z)
				);

				// Incremental tangent impulse
				let tm = cm + MW_TANGENTMASS;
				let tmxx = v128.load(tm + S2_CXX);
				let tmxy = v128.load(tm + S2_CXY);
				let tmyy = v128.load(tm + S2_CYY);
				let deltaImpulseX = b3NegV4(b3AddV4(b3MulV4(tmxx, vtx), b3MulV4(tmxy, vty)));
				let deltaImpulseY = b3NegV4(b3AddV4(b3MulV4(tmxy, vtx), b3MulV4(tmyy, vty)));
				let frictionImpulseX = v128.load(cm + MW_FRICTIONIMPULSE + V2_X);
				let frictionImpulseY = v128.load(cm + MW_FRICTIONIMPULSE + V2_Y);
				let newImpulseX = b3AddV4(frictionImpulseX, deltaImpulseX);
				let newImpulseY = b3AddV4(frictionImpulseY, deltaImpulseY);

				let maxImpulse = b3MulV4(friction, totalNormalImpulse);

				// Clamp the accumulated impulse
				let lengthSquared = b3AddV4(b3MulV4(newImpulseX, newImpulseX), b3MulV4(newImpulseY, newImpulseY));

				// Max impulse can be zero
				let mask = f32x4.gt(lengthSquared, b3MulV4(maxImpulse, maxImpulse));

				// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism. Add epsilon to avoid divide by
				// zero.
				let normalize = b3DivV4(maxImpulse, b3AddV4(b3SqrtV4(lengthSquared), epsilonW));
				let scale = b3BlendV4(oneW, normalize, mask);
				newImpulseX = b3MulV4(scale, newImpulseX);
				newImpulseY = b3MulV4(scale, newImpulseY);

				deltaImpulseX = b3SubV4(newImpulseX, frictionImpulseX);
				deltaImpulseY = b3SubV4(newImpulseY, frictionImpulseY);

				v128.store(cm + MW_FRICTIONIMPULSE + V2_X, newImpulseX);
				v128.store(cm + MW_FRICTIONIMPULSE + V2_Y, newImpulseY);

				// Apply delta impulse
				let PX = b3AddV4(b3MulV4(deltaImpulseX, tangent1X), b3MulV4(deltaImpulseY, tangent2X));
				let PY = b3AddV4(b3MulV4(deltaImpulseX, tangent1Y), b3MulV4(deltaImpulseY, tangent2Y));
				let PZ = b3AddV4(b3MulV4(deltaImpulseX, tangent1Z), b3MulV4(deltaImpulseY, tangent2Z));
				let rPAX = b3CrossV4(rAY, rAZ, PY, PZ);
				let rPAY = b3CrossV4(rAZ, rAX, PZ, PX);
				let rPAZ = b3CrossV4(rAX, rAY, PX, PY);
				let rPBX = b3CrossV4(rBY, rBZ, PY, PZ);
				let rPBY = b3CrossV4(rBZ, rBX, PZ, PX);
				let rPBZ = b3CrossV4(rBX, rBY, PX, PY);
				wAX = b3SubV4(wAX, b3MulRowV4(v128.load(iA + S_CXX), v128.load(iA + S_CXY), v128.load(iA + S_CXZ), rPAX, rPAY, rPAZ));
				wAY = b3SubV4(wAY, b3MulRowV4(v128.load(iA + S_CXY), v128.load(iA + S_CYY), v128.load(iA + S_CYZ), rPAX, rPAY, rPAZ));
				wAZ = b3SubV4(wAZ, b3MulRowV4(v128.load(iA + S_CXZ), v128.load(iA + S_CYZ), v128.load(iA + S_CZZ), rPAX, rPAY, rPAZ));
				wBX = b3AddV4(wBX, b3MulRowV4(v128.load(iB + S_CXX), v128.load(iB + S_CXY), v128.load(iB + S_CXZ), rPBX, rPBY, rPBZ));
				wBY = b3AddV4(wBY, b3MulRowV4(v128.load(iB + S_CXY), v128.load(iB + S_CYY), v128.load(iB + S_CYZ), rPBX, rPBY, rPBZ));
				wBZ = b3AddV4(wBZ, b3MulRowV4(v128.load(iB + S_CXZ), v128.load(iB + S_CYZ), v128.load(iB + S_CZZ), rPBX, rPBY, rPBZ));

				PX = b3MulAddV4(PX, manifoldImpulse, normalX);
				PY = b3MulAddV4(PY, manifoldImpulse, normalY);
				PZ = b3MulAddV4(PZ, manifoldImpulse, normalZ);
				vAX = b3MulSubV4(vAX, invMassA, PX);
				vAY = b3MulSubV4(vAY, invMassA, PY);
				vAZ = b3MulSubV4(vAZ, invMassA, PZ);
				vBX = b3MulAddV4(vBX, invMassB, PX);
				vBY = b3MulAddV4(vBY, invMassB, PY);
				vBZ = b3MulAddV4(vBZ, invMassB, PZ);
			}
		}

		b3ScatterBodiesV4(states, c + CW_INDEXA, vAX, vAY, vAZ, wAX, wAY, wAZ);
		b3ScatterBodiesV4(states, c + CW_INDEXB, vBX, vBY, vBZ, wBX, wBY, wBZ);

		ci += 1;
		mi += manifoldCount;
	}
}

const BSIM_INVMASS = offsetof<b3BodySim>("invMass");
const BSIM_INVINERTIA = offsetof<b3BodySim>("invInertiaWorld");
const M3_CX = offsetof<b3Matrix3>("cx");
const M3_CY = offsetof<b3Matrix3>("cy");
const M3_CZ = offsetof<b3Matrix3>("cz");

// A lane's sim, or C's b3_zeroBodySim for B3_NULL_INDEX
const b3_zeroSims = new StaticArray<b3BodySim>(1);

@inline unsafe function b3SimsDataV4(sims: Array<b3BodySim>): usize {
	return load<usize>(changetype<usize>(sims) + offsetof<Array<i32>>("dataStart"));
}

@inline function b3SimPointerV4(data: usize, zero: usize, index: i32): usize {
	return index == B3_NULL_INDEX ? zero : data + <usize>index * sizeof<b3BodySim>();
}

// The lane wise b3PrepareConstraintW. The constraint is constraints[constraintIndex], warmStartScale is the splatted
// scalar. Contacts, sims and manifolds are read as in the scalar path; only the lane math and the stores are wide.
unsafe function b3PrepareConstraintV4<T>(
	constraints: StaticArray<b3ContactConstraintWide>,
	constraintIndex: i32,
	manifolds: StaticArray<b3ContactManifoldWide>,
	manifoldBase: i32,
	contactIds: FixedArray<i32, 4>,
	manifoldCount: i32,
	context: b3StepContext,
	warmStartScaleValue: f32
): bool {
	let world = context.world;
	let simData = b3SimsDataV4(context.sims);
	let zeroSim = changetype<usize>(b3_zeroSims);
	let states = b3StatesDataV4(context.states);
	let identity = changetype<usize>(b3_identityStates);
	let c = changetype<usize>(constraints) + <usize>constraintIndex * sizeof<b3ContactConstraintWide>();
	let manifoldData = changetype<usize>(manifolds);

	let warmStartScale = f32x4.splat(warmStartScaleValue);
	let zeroW = f32x4.splat(0.0);
	let oneW = f32x4.splat(1.0);
	let twoW = f32x4.splat(2.0);
	let halfW = f32x4.splat(0.5);
	let invTau = f32x4.splat(<f32>1.0 / B3_SPECULATIVE_DISTANCE());
	let minFrictionWeight = f32x4.splat(B3_MIN_FRICTION_WEIGHT);
	let minDet = f32x4.splat(1000.0 * FLT_MIN);

	let contactLanes = b3_contactLanes;
	let manifoldLanes = b3_manifoldLanes;
	let simLanesA: FixedArray<i32, 4>;
	let simLanesB: FixedArray<i32, 4>;
	let manifoldCounts: FixedArray<i32, 4>;
	let hitEventLanes = 0;
	let maxManifoldCount = 0;

	for (let lane = 0; lane < 4; ++lane) {
		let contactId = contactIds[lane];
		store<i32>(c + CW_CONTACTS + <usize>lane * 4, contactId);

		if (contactId != B3_NULL_INDEX) {
			let contact = world.contacts[contactId];
			let indexA = b3DecodeAwakeIndex(contact.encodedBodySimA);
			let indexB = b3DecodeAwakeIndex(contact.encodedBodySimB);

			store<i32>(c + CW_INDEXA + <usize>lane * 4, indexA + 1);
			store<i32>(c + CW_INDEXB + <usize>lane * 4, indexB + 1);
			manifoldCounts[lane] = contact.manifoldCount;

			contactLanes[lane] = contact;
			simLanesA[lane] = indexA;
			simLanesB[lane] = indexB;
			hitEventLanes |= (contact.flags & b3_simEnableHitEvent) != 0 ? 1 << lane : 0;
			maxManifoldCount = b3MaxInt(maxManifoldCount, contact.manifoldCount);
		} else {
			store<i32>(c + CW_INDEXA + <usize>lane * 4, 0);
			store<i32>(c + CW_INDEXB + <usize>lane * 4, 0);
			manifoldCounts[lane] = 0;

			contactLanes[lane] = b3_zeroContact;
			simLanesA[lane] = B3_NULL_INDEX;
			simLanesB[lane] = B3_NULL_INDEX;
		}
	}

	store<i32>(c + CW_MANIFOLDCOUNT, manifoldCount);

	let sA0 = b3SimPointerV4(simData, zeroSim, simLanesA[0]);
	let sA1 = b3SimPointerV4(simData, zeroSim, simLanesA[1]);
	let sA2 = b3SimPointerV4(simData, zeroSim, simLanesA[2]);
	let sA3 = b3SimPointerV4(simData, zeroSim, simLanesA[3]);
	let sB0 = b3SimPointerV4(simData, zeroSim, simLanesB[0]);
	let sB1 = b3SimPointerV4(simData, zeroSim, simLanesB[1]);
	let sB2 = b3SimPointerV4(simData, zeroSim, simLanesB[2]);
	let sB3 = b3SimPointerV4(simData, zeroSim, simLanesB[3]);

	let mA = b3SetV4<v128>(load<f32>(sA0 + BSIM_INVMASS), load<f32>(sA1 + BSIM_INVMASS), load<f32>(sA2 + BSIM_INVMASS), load<f32>(sA3 + BSIM_INVMASS));
	let mB = b3SetV4<v128>(load<f32>(sB0 + BSIM_INVMASS), load<f32>(sB1 + BSIM_INVMASS), load<f32>(sB2 + BSIM_INVMASS), load<f32>(sB3 + BSIM_INVMASS));

	// C transposes the first nine floats of each lane's invInertiaWorld
	let iA0 = sA0 + BSIM_INVINERTIA;
	let iA1 = sA1 + BSIM_INVINERTIA;
	let iA2 = sA2 + BSIM_INVINERTIA;
	let iA3 = sA3 + BSIM_INVINERTIA;
	let iB0 = sB0 + BSIM_INVINERTIA;
	let iB1 = sB1 + BSIM_INVINERTIA;
	let iB2 = sB2 + BSIM_INVINERTIA;
	let iB3 = sB3 + BSIM_INVINERTIA;
	let iAxx = b3SetV4<v128>(load<f32>(iA0 + M3_CX), load<f32>(iA1 + M3_CX), load<f32>(iA2 + M3_CX), load<f32>(iA3 + M3_CX));
	let iAxy = b3SetV4<v128>(load<f32>(iA0 + M3_CX + 4), load<f32>(iA1 + M3_CX + 4), load<f32>(iA2 + M3_CX + 4), load<f32>(iA3 + M3_CX + 4));
	let iAxz = b3SetV4<v128>(load<f32>(iA0 + M3_CX + 8), load<f32>(iA1 + M3_CX + 8), load<f32>(iA2 + M3_CX + 8), load<f32>(iA3 + M3_CX + 8));
	let iAyy = b3SetV4<v128>(load<f32>(iA0 + M3_CY + 4), load<f32>(iA1 + M3_CY + 4), load<f32>(iA2 + M3_CY + 4), load<f32>(iA3 + M3_CY + 4));
	let iAyz = b3SetV4<v128>(load<f32>(iA0 + M3_CY + 8), load<f32>(iA1 + M3_CY + 8), load<f32>(iA2 + M3_CY + 8), load<f32>(iA3 + M3_CY + 8));
	let iAzz = b3SetV4<v128>(load<f32>(iA0 + M3_CZ + 8), load<f32>(iA1 + M3_CZ + 8), load<f32>(iA2 + M3_CZ + 8), load<f32>(iA3 + M3_CZ + 8));
	let iBxx = b3SetV4<v128>(load<f32>(iB0 + M3_CX), load<f32>(iB1 + M3_CX), load<f32>(iB2 + M3_CX), load<f32>(iB3 + M3_CX));
	let iBxy = b3SetV4<v128>(load<f32>(iB0 + M3_CX + 4), load<f32>(iB1 + M3_CX + 4), load<f32>(iB2 + M3_CX + 4), load<f32>(iB3 + M3_CX + 4));
	let iBxz = b3SetV4<v128>(load<f32>(iB0 + M3_CX + 8), load<f32>(iB1 + M3_CX + 8), load<f32>(iB2 + M3_CX + 8), load<f32>(iB3 + M3_CX + 8));
	let iByy = b3SetV4<v128>(load<f32>(iB0 + M3_CY + 4), load<f32>(iB1 + M3_CY + 4), load<f32>(iB2 + M3_CY + 4), load<f32>(iB3 + M3_CY + 4));
	let iByz = b3SetV4<v128>(load<f32>(iB0 + M3_CY + 8), load<f32>(iB1 + M3_CY + 8), load<f32>(iB2 + M3_CY + 8), load<f32>(iB3 + M3_CY + 8));
	let iBzz = b3SetV4<v128>(load<f32>(iB0 + M3_CZ + 8), load<f32>(iB1 + M3_CZ + 8), load<f32>(iB2 + M3_CZ + 8), load<f32>(iB3 + M3_CZ + 8));

	v128.store(c + CW_INVMASSA, mA);
	v128.store(c + CW_INVMASSB, mB);
	v128.store(c + CW_INVIA + S_CXX, iAxx);
	v128.store(c + CW_INVIA + S_CXY, iAxy);
	v128.store(c + CW_INVIA + S_CXZ, iAxz);
	v128.store(c + CW_INVIA + S_CYY, iAyy);
	v128.store(c + CW_INVIA + S_CYZ, iAyz);
	v128.store(c + CW_INVIA + S_CZZ, iAzz);
	v128.store(c + CW_INVIB + S_CXX, iBxx);
	v128.store(c + CW_INVIB + S_CXY, iBxy);
	v128.store(c + CW_INVIB + S_CXZ, iBxz);
	v128.store(c + CW_INVIB + S_CYY, iByy);
	v128.store(c + CW_INVIB + S_CYZ, iByz);
	v128.store(c + CW_INVIB + S_CZZ, iBzz);

	let cl0 = contactLanes[0];
	let cl1 = contactLanes[1];
	let cl2 = contactLanes[2];
	let cl3 = contactLanes[3];
	let rollingResistance = b3SetV4<v128>(cl0.rollingResistance, cl1.rollingResistance, cl2.rollingResistance, cl3.rollingResistance);
	let restitution = b3SetV4<v128>(cl0.restitution, cl1.restitution, cl2.restitution, cl3.restitution);
	v128.store(c + CW_FRICTION, b3SetV4<v128>(cl0.friction, cl1.friction, cl2.friction, cl3.friction));
	v128.store(c + CW_ROLLINGRESISTANCE, rollingResistance);
	v128.store(c + CW_RESTITUTION, restitution);
	let tangentVelocityX = b3SetV4<v128>(cl0.tangentVelocity.x, cl1.tangentVelocity.x, cl2.tangentVelocity.x, cl3.tangentVelocity.x);
	let tangentVelocityY = b3SetV4<v128>(cl0.tangentVelocity.y, cl1.tangentVelocity.y, cl2.tangentVelocity.y, cl3.tangentVelocity.y);
	let tangentVelocityZ = b3SetV4<v128>(cl0.tangentVelocity.z, cl1.tangentVelocity.z, cl2.tangentVelocity.z, cl3.tangentVelocity.z);

	let invIABxx = b3AddV4(iAxx, iBxx);
	let invIABxy = b3AddV4(iAxy, iBxy);
	let invIABxz = b3AddV4(iAxz, iBxz);
	let invIAByy = b3AddV4(iAyy, iByy);
	let invIAByz = b3AddV4(iAyz, iByz);
	let invIABzz = b3AddV4(iAzz, iBzz);
	let rm = c + CW_ROLLINGMASS;
	if (b3AllZeroV4(rollingResistance) == false) {
		// b3InvertSymW( invIAB )
		let cxx = b3SubV4(b3MulV4(invIAByy, invIABzz), b3MulV4(invIAByz, invIAByz));
		let cxy = b3SubV4(b3MulV4(invIABxz, invIAByz), b3MulV4(invIABxy, invIABzz));
		let cxz = b3SubV4(b3MulV4(invIABxy, invIAByz), b3MulV4(invIABxz, invIAByy));
		let cyy = b3SubV4(b3MulV4(invIABxx, invIABzz), b3MulV4(invIABxz, invIABxz));
		let cyz = b3SubV4(b3MulV4(invIABxy, invIABxz), b3MulV4(invIABxx, invIAByz));
		let czz = b3SubV4(b3MulV4(invIABxx, invIAByy), b3MulV4(invIABxy, invIABxy));

		let det = b3AddV4(b3MulV4(invIABxx, cxx), b3AddV4(b3MulV4(invIABxy, cxy), b3MulV4(invIABxz, cxz)));
		let valid = f32x4.gt(b3AbsV4(det), minDet);
		let invDet = b3BlendV4(zeroW, b3DivV4(oneW, det), valid);

		v128.store(rm + S_CXX, b3MulV4(invDet, cxx));
		v128.store(rm + S_CXY, b3MulV4(invDet, cxy));
		v128.store(rm + S_CXZ, b3MulV4(invDet, cxz));
		v128.store(rm + S_CYY, b3MulV4(invDet, cyy));
		v128.store(rm + S_CYZ, b3MulV4(invDet, cyz));
		v128.store(rm + S_CZZ, b3MulV4(invDet, czz));
	} else {
		v128.store(rm + S_CXX, zeroW);
		v128.store(rm + S_CXY, zeroW);
		v128.store(rm + S_CXZ, zeroW);
		v128.store(rm + S_CYY, zeroW);
		v128.store(rm + S_CYZ, zeroW);
		v128.store(rm + S_CZZ, zeroW);
	}

	let rollingResistanceMask = f32x4.gt(rollingResistance, zeroW);

	for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
		let cm = manifoldData + <usize>(manifoldBase + manifoldIndex) * sizeof<b3ContactManifoldWide>();

		let manifoldIndices: FixedArray<i32, 4>;
		let activeLanes: FixedArray<i32, 4>;
		let pointCounts: FixedArray<i32, 4>;
		let pointCount = 0;
		for (let lane = 0; lane < 4; ++lane) {
			if (manifoldIndex < manifoldCounts[lane]) {
				let laneManifolds = contactLanes[lane].manifolds;
				manifoldLanes[lane] = laneManifolds;
				manifoldIndices[lane] = manifoldIndex;
				activeLanes[lane] = 1;
				pointCounts[lane] = laneManifolds[manifoldIndex].pointCount;
			} else {
				manifoldLanes[lane] = b3_zeroManifolds;
				manifoldIndices[lane] = 0;
				activeLanes[lane] = 0;
				pointCounts[lane] = 0;
			}

			pointCount = b3MaxInt(pointCount, pointCounts[lane]);
		}

		store<i32>(cm + MW_POINTCOUNT, pointCount);

		// C stores the mask as lanes of 1.0 or 0.0, which the iterations read back
		let activeMask = f32x4.gt(
			b3SetV4<v128>(<f32>activeLanes[0], <f32>activeLanes[1], <f32>activeLanes[2], <f32>activeLanes[3]),
			zeroW
		);
		let rollingMask = v128.and(rollingResistanceMask, activeMask);
		v128.store(cm + MW_ROLLINGMASK, v128.and(rollingMask, oneW));

		let ma0 = manifoldLanes[0];
		let ma1 = manifoldLanes[1];
		let ma2 = manifoldLanes[2];
		let ma3 = manifoldLanes[3];
		ref m0 = ma0[manifoldIndices[0]];
		ref m1 = ma1[manifoldIndices[1]];
		ref m2 = ma2[manifoldIndices[2]];
		ref m3 = ma3[manifoldIndices[3]];

		// C transposes the normal, twist, friction and rolling impulse floats of each lane
		let normalX = b3SetV4<v128>(m0.normal.x, m1.normal.x, m2.normal.x, m3.normal.x);
		let normalY = b3SetV4<v128>(m0.normal.y, m1.normal.y, m2.normal.y, m3.normal.y);
		let normalZ = b3SetV4<v128>(m0.normal.z, m1.normal.z, m2.normal.z, m3.normal.z);
		let twistImpulse = b3SetV4<v128>(m0.twistImpulse, m1.twistImpulse, m2.twistImpulse, m3.twistImpulse);
		let frictionImpulseX = b3SetV4<v128>(m0.frictionImpulse.x, m1.frictionImpulse.x, m2.frictionImpulse.x, m3.frictionImpulse.x);
		let frictionImpulseY = b3SetV4<v128>(m0.frictionImpulse.y, m1.frictionImpulse.y, m2.frictionImpulse.y, m3.frictionImpulse.y);
		let frictionImpulseZ = b3SetV4<v128>(m0.frictionImpulse.z, m1.frictionImpulse.z, m2.frictionImpulse.z, m3.frictionImpulse.z);
		let rollingImpulseX = b3SetV4<v128>(m0.rollingImpulse.x, m1.rollingImpulse.x, m2.rollingImpulse.x, m3.rollingImpulse.x);
		let rollingImpulseY = b3SetV4<v128>(m0.rollingImpulse.y, m1.rollingImpulse.y, m2.rollingImpulse.y, m3.rollingImpulse.y);
		let rollingImpulseZ = b3SetV4<v128>(m0.rollingImpulse.z, m1.rollingImpulse.z, m2.rollingImpulse.z, m3.rollingImpulse.z);

		// b3PerpW( normal )
		let perpMask = v128.or(f32x4.lt(normalX, b3NegV4(halfW)), f32x4.gt(normalX, halfW));
		let pX = b3BlendV4(zeroW, normalY, perpMask);
		let pY = b3BlendV4(normalZ, b3NegV4(normalX), perpMask);
		let pZ = b3BlendV4(b3NegV4(normalY), zeroW, perpMask);
		let perpLengthSquared = b3DotV4(pX, pY, pZ, pX, pY, pZ);
		let perpValid = f32x4.gt(perpLengthSquared, minDet);
		let perpScale = b3BlendV4(zeroW, b3DivV4(oneW, b3SqrtV4(perpLengthSquared)), perpValid);
		let tangent1X = b3MulV4(perpScale, pX);
		let tangent1Y = b3MulV4(perpScale, pY);
		let tangent1Z = b3MulV4(perpScale, pZ);

		let tangent2X = b3CrossV4(tangent1Y, tangent1Z, normalY, normalZ);
		let tangent2Y = b3CrossV4(tangent1Z, tangent1X, normalZ, normalX);
		let tangent2Z = b3CrossV4(tangent1X, tangent1Y, normalX, normalY);
		v128.store(cm + MW_NORMAL + V_X, normalX);
		v128.store(cm + MW_NORMAL + V_Y, normalY);
		v128.store(cm + MW_NORMAL + V_Z, normalZ);
		v128.store(cm + MW_TANGENT1 + V_X, tangent1X);
		v128.store(cm + MW_TANGENT1 + V_Y, tangent1Y);
		v128.store(cm + MW_TANGENT1 + V_Z, tangent1Z);
		v128.store(cm + MW_TANGENT2 + V_X, tangent2X);
		v128.store(cm + MW_TANGENT2 + V_Y, tangent2Y);
		v128.store(cm + MW_TANGENT2 + V_Z, tangent2Z);
		v128.store(cm + MW_TANGENTVELOCITY1, b3DotV4(tangentVelocityX, tangentVelocityY, tangentVelocityZ, tangent1X, tangent1Y, tangent1Z));
		v128.store(cm + MW_TANGENTVELOCITY2, b3DotV4(tangentVelocityX, tangentVelocityY, tangentVelocityZ, tangent2X, tangent2Y, tangent2Z));

		v128.store(cm + MW_TWISTIMPULSE, b3MulV4(warmStartScale, twistImpulse));
		v128.store(cm + MW_ROLLINGIMPULSE + V_X, b3BlendV4(zeroW, b3MulV4(warmStartScale, rollingImpulseX), rollingMask));
		v128.store(cm + MW_ROLLINGIMPULSE + V_Y, b3BlendV4(zeroW, b3MulV4(warmStartScale, rollingImpulseY), rollingMask));
		v128.store(cm + MW_ROLLINGIMPULSE + V_Z, b3BlendV4(zeroW, b3MulV4(warmStartScale, rollingImpulseZ), rollingMask));
		v128.store(
			cm + MW_FRICTIONIMPULSE + V2_X,
			b3MulV4(warmStartScale, b3DotV4(frictionImpulseX, frictionImpulseY, frictionImpulseZ, tangent1X, tangent1Y, tangent1Z))
		);
		v128.store(
			cm + MW_FRICTIONIMPULSE + V2_Y,
			b3MulV4(warmStartScale, b3DotV4(frictionImpulseX, frictionImpulseY, frictionImpulseZ, tangent2X, tangent2Y, tangent2Z))
		);

		let pointCountW = b3SetV4<v128>(<f32>pointCounts[0], <f32>pointCounts[1], <f32>pointCounts[2], <f32>pointCounts[3]);

		let centerAX = zeroW;
		let centerAY = zeroW;
		let centerAZ = zeroW;
		let centerBX = zeroW;
		let centerBY = zeroW;
		let centerBZ = zeroW;
		let totalFrictionWeight = zeroW;

		for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
			let cp = cm + MW_POINTS + <usize>pointIndex * sizeof<b3ContactConstraintPointWide>();
			let pointMask = f32x4.gt(pointCountW, f32x4.splat(<f32>pointIndex));

			ref p0 = m0.points[pointIndex];
			ref p1 = m1.points[pointIndex];
			ref p2 = m2.points[pointIndex];
			ref p3 = m3.points[pointIndex];

			// C transposes anchorA, anchorB, separation and normalImpulse of each lane
			let rAX = b3BlendV4(zeroW, b3SetV4<v128>(p0.anchorA.x, p1.anchorA.x, p2.anchorA.x, p3.anchorA.x), pointMask);
			let rAY = b3BlendV4(zeroW, b3SetV4<v128>(p0.anchorA.y, p1.anchorA.y, p2.anchorA.y, p3.anchorA.y), pointMask);
			let rAZ = b3BlendV4(zeroW, b3SetV4<v128>(p0.anchorA.z, p1.anchorA.z, p2.anchorA.z, p3.anchorA.z), pointMask);
			let rBX = b3BlendV4(zeroW, b3SetV4<v128>(p0.anchorB.x, p1.anchorB.x, p2.anchorB.x, p3.anchorB.x), pointMask);
			let rBY = b3BlendV4(zeroW, b3SetV4<v128>(p0.anchorB.y, p1.anchorB.y, p2.anchorB.y, p3.anchorB.y), pointMask);
			let rBZ = b3BlendV4(zeroW, b3SetV4<v128>(p0.anchorB.z, p1.anchorB.z, p2.anchorB.z, p3.anchorB.z), pointMask);
			let separation = b3BlendV4(zeroW, b3SetV4<v128>(p0.separation, p1.separation, p2.separation, p3.separation), pointMask);
			let normalImpulse = b3BlendV4(
				zeroW,
				b3SetV4<v128>(p0.normalImpulse, p1.normalImpulse, p2.normalImpulse, p3.normalImpulse),
				pointMask
			);

			// C0 friction center decay. Needed to prevent spinning top drift (GyroscopicPrecession sample).
			// See details in b3PrepareContacts_Mesh. This code should stay in sync.
			let weight = b3MinV4(b3MaxV4(b3SubV4(twoW, b3MulV4(separation, invTau)), minFrictionWeight), oneW);
			weight = b3BlendV4(zeroW, weight, pointMask);
			centerAX = b3MulAddV4(centerAX, weight, rAX);
			centerAY = b3MulAddV4(centerAY, weight, rAY);
			centerAZ = b3MulAddV4(centerAZ, weight, rAZ);
			centerBX = b3MulAddV4(centerBX, weight, rBX);
			centerBY = b3MulAddV4(centerBY, weight, rBY);
			centerBZ = b3MulAddV4(centerBZ, weight, rBZ);
			totalFrictionWeight = b3AddV4(totalFrictionWeight, weight);

			v128.store(cp + PW_ANCHORA + V_X, rAX);
			v128.store(cp + PW_ANCHORA + V_Y, rAY);
			v128.store(cp + PW_ANCHORA + V_Z, rAZ);
			v128.store(cp + PW_ANCHORB + V_X, rBX);
			v128.store(cp + PW_ANCHORB + V_Y, rBY);
			v128.store(cp + PW_ANCHORB + V_Z, rBZ);
			v128.store(
				cp + PW_BASESEPARATION,
				b3SubV4(separation, b3DotV4(b3SubV4(rBX, rAX), b3SubV4(rBY, rAY), b3SubV4(rBZ, rAZ), normalX, normalY, normalZ))
			);
			v128.store(cp + PW_NORMALIMPULSE, b3MulV4(warmStartScale, normalImpulse));
			v128.store(cp + PW_TOTALNORMALIMPULSE, zeroW);
			v128.store(cp + PW_RELATIVEVELOCITY, zeroW);
			v128.store(cp + PW_RESTITUTIONIMPULSE, zeroW);

			let rnAX = b3CrossV4(rAY, rAZ, normalY, normalZ);
			let rnAY = b3CrossV4(rAZ, rAX, normalZ, normalX);
			let rnAZ = b3CrossV4(rAX, rAY, normalX, normalY);
			let rnBX = b3CrossV4(rBY, rBZ, normalY, normalZ);
			let rnBY = b3CrossV4(rBZ, rBX, normalZ, normalX);
			let rnBZ = b3CrossV4(rBX, rBY, normalX, normalY);
			let kNormal = b3AddV4(mA, mB);
			kNormal = b3AddV4(
				kNormal,
				b3DotV4(
					rnAX,
					rnAY,
					rnAZ,
					b3MulRowV4(iAxx, iAxy, iAxz, rnAX, rnAY, rnAZ),
					b3MulRowV4(iAxy, iAyy, iAyz, rnAX, rnAY, rnAZ),
					b3MulRowV4(iAxz, iAyz, iAzz, rnAX, rnAY, rnAZ)
				)
			);
			kNormal = b3AddV4(
				kNormal,
				b3DotV4(
					rnBX,
					rnBY,
					rnBZ,
					b3MulRowV4(iBxx, iBxy, iBxz, rnBX, rnBY, rnBZ),
					b3MulRowV4(iBxy, iByy, iByz, rnBX, rnBY, rnBZ),
					b3MulRowV4(iBxz, iByz, iBzz, rnBX, rnBY, rnBZ)
				)
			);
			let valid = v128.and(f32x4.gt(kNormal, zeroW), pointMask);
			v128.store(cp + PW_NORMALMASS, b3BlendV4(zeroW, b3DivV4(oneW, kNormal), valid));
		}

		let frictionValid = f32x4.gt(totalFrictionWeight, zeroW);
		let invWeight = b3BlendV4(zeroW, b3DivV4(oneW, totalFrictionWeight), frictionValid);
		centerAX = b3MulV4(invWeight, centerAX);
		centerAY = b3MulV4(invWeight, centerAY);
		centerAZ = b3MulV4(invWeight, centerAZ);
		centerBX = b3MulV4(invWeight, centerBX);
		centerBY = b3MulV4(invWeight, centerBY);
		centerBZ = b3MulV4(invWeight, centerBZ);
		v128.store(cm + MW_CENTERA + V_X, centerAX);
		v128.store(cm + MW_CENTERA + V_Y, centerAY);
		v128.store(cm + MW_CENTERA + V_Z, centerAZ);
		v128.store(cm + MW_CENTERB + V_X, centerBX);
		v128.store(cm + MW_CENTERB + V_Y, centerBY);
		v128.store(cm + MW_CENTERB + V_Z, centerBZ);

		for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
			let cp = cm + MW_POINTS + <usize>pointIndex * sizeof<b3ContactConstraintPointWide>();
			let pointMask = f32x4.gt(pointCountW, f32x4.splat(<f32>pointIndex));
			let dX = b3SubV4(v128.load(cp + PW_ANCHORA + V_X), centerAX);
			let dY = b3SubV4(v128.load(cp + PW_ANCHORA + V_Y), centerAY);
			let dZ = b3SubV4(v128.load(cp + PW_ANCHORA + V_Z), centerAZ);
			v128.store(cp + PW_LEVERARM, b3BlendV4(zeroW, b3SqrtV4(b3DotV4(dX, dY, dZ, dX, dY, dZ)), pointMask));
		}

		{
			let rtA1X = b3CrossV4(centerAY, centerAZ, tangent1Y, tangent1Z);
			let rtA1Y = b3CrossV4(centerAZ, centerAX, tangent1Z, tangent1X);
			let rtA1Z = b3CrossV4(centerAX, centerAY, tangent1X, tangent1Y);
			let rtA2X = b3CrossV4(centerAY, centerAZ, tangent2Y, tangent2Z);
			let rtA2Y = b3CrossV4(centerAZ, centerAX, tangent2Z, tangent2X);
			let rtA2Z = b3CrossV4(centerAX, centerAY, tangent2X, tangent2Y);
			let rtB1X = b3CrossV4(centerBY, centerBZ, tangent1Y, tangent1Z);
			let rtB1Y = b3CrossV4(centerBZ, centerBX, tangent1Z, tangent1X);
			let rtB1Z = b3CrossV4(centerBX, centerBY, tangent1X, tangent1Y);
			let rtB2X = b3CrossV4(centerBY, centerBZ, tangent2Y, tangent2Z);
			let rtB2Y = b3CrossV4(centerBZ, centerBX, tangent2Z, tangent2X);
			let rtB2Z = b3CrossV4(centerBX, centerBY, tangent2X, tangent2Y);
			let iArtA1X = b3MulRowV4(iAxx, iAxy, iAxz, rtA1X, rtA1Y, rtA1Z);
			let iArtA1Y = b3MulRowV4(iAxy, iAyy, iAyz, rtA1X, rtA1Y, rtA1Z);
			let iArtA1Z = b3MulRowV4(iAxz, iAyz, iAzz, rtA1X, rtA1Y, rtA1Z);
			let iArtA2X = b3MulRowV4(iAxx, iAxy, iAxz, rtA2X, rtA2Y, rtA2Z);
			let iArtA2Y = b3MulRowV4(iAxy, iAyy, iAyz, rtA2X, rtA2Y, rtA2Z);
			let iArtA2Z = b3MulRowV4(iAxz, iAyz, iAzz, rtA2X, rtA2Y, rtA2Z);
			let iBrtB1X = b3MulRowV4(iBxx, iBxy, iBxz, rtB1X, rtB1Y, rtB1Z);
			let iBrtB1Y = b3MulRowV4(iBxy, iByy, iByz, rtB1X, rtB1Y, rtB1Z);
			let iBrtB1Z = b3MulRowV4(iBxz, iByz, iBzz, rtB1X, rtB1Y, rtB1Z);
			let iBrtB2X = b3MulRowV4(iBxx, iBxy, iBxz, rtB2X, rtB2Y, rtB2Z);
			let iBrtB2Y = b3MulRowV4(iBxy, iByy, iByz, rtB2X, rtB2Y, rtB2Z);
			let iBrtB2Z = b3MulRowV4(iBxz, iByz, iBzz, rtB2X, rtB2Y, rtB2Z);

			let kxx = b3AddV4(
				b3AddV4(b3AddV4(mA, mB), b3DotV4(rtA1X, rtA1Y, rtA1Z, iArtA1X, iArtA1Y, iArtA1Z)),
				b3DotV4(rtB1X, rtB1Y, rtB1Z, iBrtB1X, iBrtB1Y, iBrtB1Z)
			);
			let kyy = b3AddV4(
				b3AddV4(b3AddV4(mA, mB), b3DotV4(rtA2X, rtA2Y, rtA2Z, iArtA2X, iArtA2Y, iArtA2Z)),
				b3DotV4(rtB2X, rtB2Y, rtB2Z, iBrtB2X, iBrtB2Y, iBrtB2Z)
			);
			let kxy = b3AddV4(
				b3DotV4(rtA1X, rtA1Y, rtA1Z, iArtA2X, iArtA2Y, iArtA2Z),
				b3DotV4(rtB1X, rtB1Y, rtB1Z, iBrtB2X, iBrtB2Y, iBrtB2Z)
			);

			let det = b3SubV4(b3MulV4(kxx, kyy), b3MulV4(kxy, kxy));
			let valid = v128.and(f32x4.gt(b3AbsV4(det), minDet), frictionValid);
			let invDet = b3BlendV4(zeroW, b3DivV4(oneW, det), valid);
			let tm = cm + MW_TANGENTMASS;
			v128.store(tm + S2_CXX, b3MulV4(invDet, kyy));
			v128.store(tm + S2_CXY, b3NegV4(b3MulV4(invDet, kxy)));
			v128.store(tm + S2_CYY, b3MulV4(invDet, kxx));
		}

		{
			let kTwist = b3DotV4(
				normalX,
				normalY,
				normalZ,
				b3MulRowV4(invIABxx, invIABxy, invIABxz, normalX, normalY, normalZ),
				b3MulRowV4(invIABxy, invIAByy, invIAByz, normalX, normalY, normalZ),
				b3MulRowV4(invIABxz, invIAByz, invIABzz, normalX, normalY, normalZ)
			);
			v128.store(cm + MW_TWISTMASS, b3BlendV4(zeroW, b3DivV4(oneW, kTwist), f32x4.gt(kTwist, zeroW)));
		}
	}

	// Only sample contact point normal velocity if needed.
	let haveRestitution = v128.any_true(f32x4.gt(restitution, zeroW));
	if (hitEventLanes != 0 || haveRestitution) {
		let pA1 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA));
		let pA2 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA + 4));
		let pA3 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA + 8));
		let pA4 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXA + 12));
		let pB1 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB));
		let pB2 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB + 4));
		let pB3 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB + 8));
		let pB4 = b3StatePointerV4(states, identity, load<i32>(c + CW_INDEXB + 12));
		let vAX = b3GatherXV4<v128>(pA1, pA2, pA3, pA4, BS_V);
		let vAY = b3GatherYV4<v128>(pA1, pA2, pA3, pA4, BS_V);
		let vAZ = b3GatherZV4<v128>(pA1, pA2, pA3, pA4, BS_V);
		let wAX = b3GatherXV4<v128>(pA1, pA2, pA3, pA4, BS_W);
		let wAY = b3GatherYV4<v128>(pA1, pA2, pA3, pA4, BS_W);
		let wAZ = b3GatherZV4<v128>(pA1, pA2, pA3, pA4, BS_W);
		let vBX = b3GatherXV4<v128>(pB1, pB2, pB3, pB4, BS_V);
		let vBY = b3GatherYV4<v128>(pB1, pB2, pB3, pB4, BS_V);
		let vBZ = b3GatherZV4<v128>(pB1, pB2, pB3, pB4, BS_V);
		let wBX = b3GatherXV4<v128>(pB1, pB2, pB3, pB4, BS_W);
		let wBY = b3GatherYV4<v128>(pB1, pB2, pB3, pB4, BS_W);
		let wBZ = b3GatherZV4<v128>(pB1, pB2, pB3, pB4, BS_W);

		for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
			let cm = manifoldData + <usize>(manifoldBase + manifoldIndex) * sizeof<b3ContactManifoldWide>();
			let normalX = v128.load(cm + MW_NORMAL + V_X);
			let normalY = v128.load(cm + MW_NORMAL + V_Y);
			let normalZ = v128.load(cm + MW_NORMAL + V_Z);

			let pointCount = load<i32>(cm + MW_POINTCOUNT);
			for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
				let cp = cm + MW_POINTS + <usize>pointIndex * sizeof<b3ContactConstraintPointWide>();
				let rAX = v128.load(cp + PW_ANCHORA + V_X);
				let rAY = v128.load(cp + PW_ANCHORA + V_Y);
				let rAZ = v128.load(cp + PW_ANCHORA + V_Z);
				let rBX = v128.load(cp + PW_ANCHORB + V_X);
				let rBY = v128.load(cp + PW_ANCHORB + V_Y);
				let rBZ = v128.load(cp + PW_ANCHORB + V_Z);

				let vrAX = b3AddV4(vAX, b3CrossV4(wAY, wAZ, rAY, rAZ));
				let vrAY = b3AddV4(vAY, b3CrossV4(wAZ, wAX, rAZ, rAX));
				let vrAZ = b3AddV4(vAZ, b3CrossV4(wAX, wAY, rAX, rAY));
				let vrBX = b3AddV4(vBX, b3CrossV4(wBY, wBZ, rBY, rBZ));
				let vrBY = b3AddV4(vBY, b3CrossV4(wBZ, wBX, rBZ, rBX));
				let vrBZ = b3AddV4(vBZ, b3CrossV4(wBX, wBY, rBX, rBY));
				let vn = b3DotV4(normalX, normalY, normalZ, b3SubV4(vrBX, vrAX), b3SubV4(vrBY, vrAY), b3SubV4(vrBZ, vrAZ));
				v128.store(cp + PW_RELATIVEVELOCITY, vn);

				if (hitEventLanes != 0) {
					for (let lane = 0; lane < 4; ++lane) {
						if ((hitEventLanes & (1 << lane)) == 0 || manifoldIndex >= manifoldCounts[lane]) {
							continue;
						}

						let laneManifolds = contactLanes[lane].manifolds;
						ref manifold = laneManifolds[manifoldIndex];
						if (pointIndex < manifold.pointCount) {
							manifold.points[pointIndex].normalVelocity = b3LaneV4(vn, lane);
						}
					}
				}
			}
		}
	}

	return haveRestitution;
}

// These wrappers allow the implementation be shared between convex and mesh contacts.

export function b3WarmStartContacts_ConvexW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	b3WarmStartContactsW(
		block,
		context,
		color.wideConstraints,
		color.wideManifolds,
		color.wideConstraintStart,
		b3_noManifoldStarts,
		1
	);
}

export function b3PushContacts_ConvexW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	b3PushContactsW(block, context, color.wideConstraints, color.wideManifolds, color.wideConstraintStart, b3_noManifoldStarts, 1);
}

export function b3SolveContacts_ConvexW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	b3SolveContactsW(block, context, color.wideConstraints, color.wideManifolds, color.wideConstraintStart, b3_noManifoldStarts, 1);
}

export function b3ApplyRestitution_ConvexW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	b3ApplyRestitutionW(
		block,
		context,
		color.wideConstraints,
		color.wideManifolds,
		color.wideConstraintStart,
		b3_noManifoldStarts,
		1
	);
}

export function b3StoreImpulses_ConvexW4(block: b3SolverBlock, context: b3StepContext, workerIndex: i32): void {
	b3StoreImpulsesW(block, context, workerIndex, context.wideConstraints, context.wideManifolds, 0, b3_noManifoldStarts, 1);
}

export function b3WarmStartContacts_MeshWideW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	b3WarmStartContactsW(
		block,
		context,
		color.wideMeshConstraints,
		color.wideMeshManifolds,
		color.wideMeshConstraintStart,
		color.wideMeshManifoldStarts,
		0
	);
}

export function b3PushContacts_MeshWideW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	b3PushContactsW(
		block,
		context,
		color.wideMeshConstraints,
		color.wideMeshManifolds,
		color.wideMeshConstraintStart,
		color.wideMeshManifoldStarts,
		0
	);
}

export function b3SolveContacts_MeshWideW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	b3SolveContactsW(
		block,
		context,
		color.wideMeshConstraints,
		color.wideMeshManifolds,
		color.wideMeshConstraintStart,
		color.wideMeshManifoldStarts,
		0
	);
}

export function b3ApplyRestitution_MeshWideW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	b3ApplyRestitutionW(
		block,
		context,
		color.wideMeshConstraints,
		color.wideMeshManifolds,
		color.wideMeshConstraintStart,
		color.wideMeshManifoldStarts,
		0
	);
}

export function b3StoreImpulses_MeshWideW4(block: b3SolverBlock, context: b3StepContext, workerIndex: i32): void {
	b3StoreImpulsesW(
		block,
		context,
		workerIndex,
		context.wideMeshConstraints,
		context.wideMeshManifolds,
		0,
		context.wideMeshManifoldStarts,
		0
	);
}
