// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/contact_solver_wide.inl at B3_SIMD_WIDTH 4, as wide4.c compiles it with
// BOX3D_DISABLE_SIMD. The wide types live in lib/contact_solver.as.

import { B3_ASSERT, B3_NULL_INDEX, FLT_EPSILON, FLT_MIN } from "./core";
import { b3Matrix3, b3Mat3_zero, b3MaxInt, b3MinInt, b3Vec3 } from "./math_functions";
import { b3Manifold } from "./collision";
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

			let haveRestitution = b3PrepareConstraintW(c, manifolds, wideIndex, contacts, 1, context, warmStartScale);
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
			let haveRestitution = b3PrepareConstraintW(
				c,
				manifolds,
				manifoldStarts[wideIndex],
				contacts,
				manifoldCount,
				context,
				warmStartScale
			);
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

// Store impulses by contact constraint
function b3StoreImpulsesW(
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

	let ci = constraintBase + block.startIndex;
	let mi = fixedManifoldCount > 0 ? ci : manifoldStarts[block.startIndex];
	for (let i = 0; i < <i32>block.count; ++i) {
		ref c = constraints[ci];
		let manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : c.manifoldCount;

		for (let lane = 0; lane < 4; ++lane) {
			let contactId = c.contacts[lane];
			if (contactId == B3_NULL_INDEX) {
				continue;
			}

			let contact = world.contacts[contactId];
			let laneManifolds = contact.manifolds;

			let laneManifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : contact.manifoldCount;
			B3_ASSERT(laneManifoldCount == contact.manifoldCount && laneManifoldCount <= manifoldCount);

			for (let manifoldIndex = 0; manifoldIndex < laneManifoldCount; ++manifoldIndex) {
				ref cm = manifolds[mi + manifoldIndex];
				ref m = laneManifolds[manifoldIndex];

				let f1 = b3LaneW(cm.frictionImpulse.x, lane);
				let f2 = b3LaneW(cm.frictionImpulse.y, lane);
				m.frictionImpulse = b3Vec3(
					f1 * b3LaneW(cm.tangent1.X, lane) + f2 * b3LaneW(cm.tangent2.X, lane),
					f1 * b3LaneW(cm.tangent1.Y, lane) + f2 * b3LaneW(cm.tangent2.Y, lane),
					f1 * b3LaneW(cm.tangent1.Z, lane) + f2 * b3LaneW(cm.tangent2.Z, lane)
				);
				m.twistImpulse = b3LaneW(cm.twistImpulse, lane);
				m.rollingImpulse = b3Vec3(
					b3LaneW(cm.rollingImpulse.X, lane),
					b3LaneW(cm.rollingImpulse.Y, lane),
					b3LaneW(cm.rollingImpulse.Z, lane)
				);

				let pointCount = m.pointCount;
				B3_ASSERT(pointCount <= cm.pointCount);
				for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
					ref cp = cm.points[pointIndex];
					ref mp = m.points[pointIndex];
					mp.normalImpulse = b3LaneW(cp.normalImpulses, lane);
					mp.totalNormalImpulse = b3LaneW(cp.totalNormalImpulses, lane);
				}
			}

			if ((contact.flags & b3_simEnableHitEvent) != 0) {
				let flagged = false;
				for (let manifoldIndex = 0; manifoldIndex < laneManifoldCount && flagged == false; ++manifoldIndex) {
					ref m = laneManifolds[manifoldIndex];
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
