// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/contact_solver_wide.inl at B3_SIMD_WIDTH 4. The wide types live in lib/contact_solver.as.
// The kernels are generic over the lane type of lib/simd.as: v128 under ASC_FEATURE_SIMD, as C's SSE2 and NEON
// builds run them, and f32 one lane at a time without it, as wide4.c computes each lane with BOX3D_DISABLE_SIMD.
// They read and write the wide constraints in place through their field offsets, as C does through pointers.

import { B3_ASSERT, B3_NULL_INDEX, FLT_EPSILON, FLT_MIN } from "./core";
import { b3Matrix3, b3MaxInt, b3MinInt } from "./math_functions";
import { b3Manifold, b3ManifoldPoint } from "./collision";
import { B3_MIN_FRICTION_WEIGHT, B3_SPECULATIVE_DISTANCE } from "./constants";
import { b3BitSet, b3SetBit } from "./bitset";
import {
	b3SplatL,
	b3LoadL,
	b3StoreL,
	b3SetL,
	b3SetLoadL,
	b3LaneL,
	b3AddL,
	b3SubL,
	b3MulL,
	b3DivL,
	b3SqrtL,
	b3NegL,
	b3MulAddL,
	b3MulSubL,
	b3MinL,
	b3MaxL,
	b3AbsL,
	b3GreaterThanL,
	b3LessThanL,
	b3AndL,
	b3OrL,
	b3BlendL,
	b3MaskL,
	b3MaskValueL,
	b3DotL,
	b3CrossL,
	b3MulRowL,
	b3Transpose0L,
	b3Transpose1L,
	b3Transpose2L,
	b3Transpose3L,
	b3AllZeroAt,
	b3ColumnsL,
	b3TransposeL,
	b3Vec3W,
} from "./simd";
import { b3ArrayData, b3BodySim, b3BodyState, b3DecodeAwakeIndex, b3_identityStates, b3_dynamicFlag } from "./body";
import { b3Contact, b3_simEnableHitEvent } from "./contact";
import { b3SolverBlock, b3StepContext } from "./solver";
import {
	b3Vec2W,
	b3SymMatrix2W,
	b3SymMatrix3W,
	b3ContactManifoldWide,
	b3ContactConstraintWide,
	b3ContactConstraintPointWide,
} from "./contact_solver";

// C points empty lanes at zeroed statics. Nothing writes to these.
const b3_zeroContact = new b3Contact();
@parallel.shared const b3_zeroManifolds = new StaticArray<b3Manifold>(1);

// Never read: the convex paths have a fixed manifold count.
const b3_noManifoldStarts = new Int32Array(0);

// Field offsets: C reads wide fields through float pointers, lane i of a b3FloatW at 4 i bytes.

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
const CT_FRICTION = offsetof<b3Contact>("friction");
const CT_MANIFOLDS = offsetof<b3Contact>("manifolds");
const CT_MANIFOLDCOUNT = offsetof<b3Contact>("manifoldCount");
const CT_FLAGS = offsetof<b3Contact>("flags");
const CT_CONTACTID = offsetof<b3Contact>("contactId");
const CT_ENCODEDBODYSIMA = offsetof<b3Contact>("encodedBodySimA");
const CT_ENCODEDBODYSIMB = offsetof<b3Contact>("encodedBodySimB");
const CT_ROLLINGRESISTANCE = offsetof<b3Contact>("rollingResistance");
const CT_TANGENTVELOCITY = offsetof<b3Contact>("tangentVelocity");
const M_POINTS = offsetof<b3Manifold>("points");
const M_NORMAL = offsetof<b3Manifold>("normal");
const M_FRICTIONIMPULSE = offsetof<b3Manifold>("frictionImpulse");
const M_ROLLINGIMPULSE = offsetof<b3Manifold>("rollingImpulse");
const M_POINTCOUNT = offsetof<b3Manifold>("pointCount");
const MP_ANCHORA = offsetof<b3ManifoldPoint>("anchorA");
const MP_ANCHORB = offsetof<b3ManifoldPoint>("anchorB");
const MP_NORMALVELOCITY = offsetof<b3ManifoldPoint>("normalVelocity");
const MP_TOTALNORMALIMPULSE = offsetof<b3ManifoldPoint>("totalNormalImpulse");
const BSIM_INVMASS = offsetof<b3BodySim>("invMass");
const BSIM_INVINERTIA = offsetof<b3BodySim>("invInertiaWorld");
const M3_CX = offsetof<b3Matrix3>("cx");
const M3_CY = offsetof<b3Matrix3>("cy");
const M3_CZ = offsetof<b3Matrix3>("cz");

// Lanes per kernel call: a v128 kernel covers the four, an f32 kernel one
@inline function b3LaneCountL<T>(): i32 {
	return isFloat<T>() ? 4 : 1;
}

// C points null sims at b3_zeroBodySim.
const b3_zeroSims = new StaticArray<b3BodySim>(1);

@inline unsafe function b3StatesData(states: Array<b3BodyState>): usize {
	return states.dataStart;
}

@inline unsafe function b3SimsData(sims: Array<b3BodySim>): usize {
	return sims.dataStart;
}

// Indices are 0 for null. They come from b3PrepareConstraintL, so they are in range by construction.
@inline function b3StatePointer(data: usize, identity: usize, index: i32): usize {
	return index == 0 ? identity : data + <usize>(index - 1) * sizeof<b3BodyState>();
}

@inline function b3SimPointer(data: usize, zero: usize, index: i32): usize {
	return index == B3_NULL_INDEX ? zero : data + <usize>index * sizeof<b3BodySim>();
}

// The sim of a lane's body from the constraint's base 1 index, as C's simLanes
@inline unsafe function b3SimLane(data: usize, zero: usize, indices: usize, lane: i32): usize {
	return b3SimPointer(data, zero, load<i32>(indices + <usize>lane * 4) - 1);
}

// A lane's contact as C's contactLanes, the zeroed contact for an empty lane
@inline unsafe function b3ContactLane(c: usize, lane: i32, zero: usize): usize {
	let contact = load<usize>(c + CW_CONTACTS + <usize>lane * 4);
	return contact != 0 ? contact : zero;
}

// A lane's manifold as C's manifoldLanes: zero past the lane's manifold count or for an empty lane
@inline unsafe function b3LaneManifold(contact: usize, manifoldIndex: i32, zero: usize): usize {
	if (contact != 0 && manifoldIndex < load<i32>(contact + CT_MANIFOLDCOUNT)) {
		return load<usize>(contact + CT_MANIFOLDS) + <usize>manifoldIndex * sizeof<b3Manifold>();
	}
	return zero;
}

// Lane k of four values; constant for a v128 kernel, so it folds
@inline function b3PickLane(k: i32, a: usize, b: usize, c: usize, d: usize): usize {
	return k == 0 ? a : k == 1 ? b : k == 2 ? c : d;
}

// The SSE2 gather: each 64 byte state is four rows, transposed into lanes. An f32 kernel reads its lane's state p1.
@inline unsafe function b3GatherXL<T>(p1: usize, p2: usize, p3: usize, p4: usize, offset: usize): T {
	if (isFloat<T>()) {
		return load<f32>(p1 + offset);
	} else {
		return b3Transpose0L(v128.load(p1 + offset), v128.load(p2 + offset), v128.load(p3 + offset), v128.load(p4 + offset));
	}
}

@inline unsafe function b3GatherYL<T>(p1: usize, p2: usize, p3: usize, p4: usize, offset: usize): T {
	if (isFloat<T>()) {
		return load<f32>(p1 + offset + 4);
	} else {
		return b3Transpose1L(v128.load(p1 + offset), v128.load(p2 + offset), v128.load(p3 + offset), v128.load(p4 + offset));
	}
}

@inline unsafe function b3GatherZL<T>(p1: usize, p2: usize, p3: usize, p4: usize, offset: usize): T {
	if (isFloat<T>()) {
		return load<f32>(p1 + offset + 8);
	} else {
		return b3Transpose2L(v128.load(p1 + offset), v128.load(p2 + offset), v128.load(p3 + offset), v128.load(p4 + offset));
	}
}

@inline unsafe function b3GatherWL<T>(p1: usize, p2: usize, p3: usize, p4: usize, offset: usize): T {
	if (isFloat<T>()) {
		return load<f32>(p1 + offset + 12);
	} else {
		return b3Transpose3L(v128.load(p1 + offset), v128.load(p2 + offset), v128.load(p3 + offset), v128.load(p4 + offset));
	}
}

@inline unsafe function b3StoreBodyVelocity(data: usize, index: i32, vX: f32, vY: f32, vZ: f32, wX: f32, wY: f32, wZ: f32): void {
	// Indices are 0 for null
	if (index == 0) {
		return;
	}

	let state = data + <usize>(index - 1) * sizeof<b3BodyState>();
	if ((load<u32>(state + BS_FLAGS) & b3_dynamicFlag) == 0) {
		return;
	}

	store<f32>(state + BS_V, vX);
	store<f32>(state + BS_V + 4, vY);
	store<f32>(state + BS_V + 8, vZ);
	store<f32>(state + BS_W, wX);
	store<f32>(state + BS_W + 4, wY);
	store<f32>(state + BS_W + 8, wZ);
}

@inline unsafe function b3StoreBodyVelocityV<T>(data: usize, index: i32, v: T, w: T): void {
	if (index == 0) {
		return;
	}

	let state = data + <usize>(index - 1) * sizeof<b3BodyState>();
	if ((load<u32>(state + BS_FLAGS) & b3_dynamicFlag) == 0) {
		return;
	}

	// The w lane lands in the padding
	v128.store(state + BS_V, v);
	v128.store(state + BS_W, w);
}

// One 16 byte row of each lane's state transposed into lanes at once, as C's b3TransposeW4: c0 to c2 are x, y and z
// of the row. An f32 kernel reads its lane's three floats.
@inline unsafe function b3GatherRow3L<T>(p1: usize, p2: usize, p3: usize, p4: usize, offset: usize): b3ColumnsL<T> {
	if (isFloat<T>()) {
		let c: b3ColumnsL<T>;
		c.c0 = load<f32>(p1 + offset);
		c.c1 = load<f32>(p1 + offset + 4);
		c.c2 = load<f32>(p1 + offset + 8);
		return c;
	} else {
		return b3TransposeL<T>(v128.load(p1 + offset), v128.load(p2 + offset), v128.load(p3 + offset), v128.load(p4 + offset));
	}
}

// b3GatherRow3L for the row's x and y only, which the v128 path takes from the low zips
@inline unsafe function b3GatherRow2L<T>(p1: usize, p2: usize, p3: usize, p4: usize, offset: usize): b3ColumnsL<T> {
	if (isFloat<T>()) {
		let c: b3ColumnsL<T>;
		c.c0 = load<f32>(p1 + offset);
		c.c1 = load<f32>(p1 + offset + 4);
		return c;
	} else {
		return b3TransposeL<T>(v128.load(p1 + offset), v128.load(p2 + offset), v128.load(p3 + offset), v128.load(p4 + offset));
	}
}

// b3GatherRow3L with the row's w in c3
@inline unsafe function b3GatherRowL<T>(p1: usize, p2: usize, p3: usize, p4: usize, offset: usize): b3ColumnsL<T> {
	if (isFloat<T>()) {
		let c: b3ColumnsL<T>;
		c.c0 = load<f32>(p1 + offset);
		c.c1 = load<f32>(p1 + offset + 4);
		c.c2 = load<f32>(p1 + offset + 8);
		c.c3 = load<f32>(p1 + offset + 12);
		return c;
	} else {
		return b3TransposeL<T>(v128.load(p1 + offset), v128.load(p2 + offset), v128.load(p3 + offset), v128.load(p4 + offset));
	}
}

// This writes only the velocities back to the solver bodies. indices points at the lane's body index.
@inline unsafe function b3ScatterBodiesL<T>(data: usize, indices: usize, vX: T, vY: T, vZ: T, wX: T, wY: T, wZ: T): void {
	if (isFloat<T>()) {
		b3StoreBodyVelocity(data, load<i32>(indices), vX, vY, vZ, wX, wY, wZ);
	} else {
		let zero = f32x4.splat(0.0);
		let v = b3TransposeL<T>(vX, vY, vZ, zero);
		let w = b3TransposeL<T>(wX, wY, wZ, zero);
		b3StoreBodyVelocityV(data, load<i32>(indices), v.c0, w.c0);
		b3StoreBodyVelocityV(data, load<i32>(indices + 4), v.c1, w.c1);
		b3StoreBodyVelocityV(data, load<i32>(indices + 8), v.c2, w.c2);
		b3StoreBodyVelocityV(data, load<i32>(indices + 12), v.c3, w.c3);
	}
}

// b3SoftMaskW over the lane's body indices
@inline unsafe function b3SoftMaskL<T>(indexA: usize, indexB: usize): T {
	if (isFloat<T>()) {
		return load<i32>(indexA) == 0 || load<i32>(indexB) == 0 ? <f32>1.0 : <f32>0.0;
	} else {
		return v128.or(i32x4.eq(v128.load(indexA), i32x4.splat(0)), i32x4.eq(v128.load(indexB), i32x4.splat(0)));
	}
}

// Any of the four floats in memory greater than zero, as b3AnyTrueW( b3GreaterThanW( a, zero ) )
@inline unsafe function b3AnyPositiveAt(p: usize): bool {
	if (ASC_FEATURE_SIMD) {
		return v128.any_true(f32x4.gt(v128.load(p), f32x4.splat(0.0)));
	}
	return load<f32>(p) > 0.0 || load<f32>(p + 4) > 0.0 || load<f32>(p + 8) > 0.0 || load<f32>(p + 12) > 0.0;
}

// C stores the constraint stream [c1 m11 m12 c2 m21 ...] and walks it by byte offsets. The port splits it into a
// constraint array and a manifold array: a constraint at index ci owns the manifolds from its first manifold index.
// b3GetManifoldsW, b3GetConstraintW and b3NextConstraintW become that index arithmetic.

// Contact constraint data is stored in a heterogious stream. Constraints followed by an array of constraint manifolds.
// [c1 m11 m12 c2 m21 c3 m31 m32 m33 ... ]
// The constraint is constraints[constraintIndex], whose contacts the caller has set as C's contacts argument.
// An f32 kernel reads only its lane: l0 to l3 all name that lane, so the lane sets take its value.
unsafe function b3PrepareConstraintL<T>(
	constraints: StaticArray<b3ContactConstraintWide>,
	constraintIndex: i32,
	manifolds: StaticArray<b3ContactManifoldWide>,
	manifoldBase: i32,
	manifoldCount: i32,
	context: b3StepContext,
	warmStartScaleValue: f32
): bool {
	let world = context.world;
	let simData = b3SimsData(context.sims);
	let zeroSim = changetype<usize>(b3_zeroSims);
	let states = b3StatesData(context.states);
	let identity = changetype<usize>(b3_identityStates);
	let c0 = changetype<usize>(constraints) + <usize>constraintIndex * sizeof<b3ContactConstraintWide>();
	let manifoldData = changetype<usize>(manifolds);

	let warmStartScale = b3SplatL<T>(warmStartScaleValue);
	let invTau: f32 = <f32>1.0 / B3_SPECULATIVE_DISTANCE();

	// C's pointer lanes are read back from the constraint, where the lanes' contacts and body indices land first
	let zeroContact = changetype<usize>(b3_zeroContact);
	let zeroManifold = changetype<usize>(b3_zeroManifolds);
	let hitEventLanes = 0;
	let maxManifoldCount = 0;
	let anyRolling = false;

	for (let lane = 0; lane < 4; ++lane) {
		let contact = load<usize>(c0 + CW_CONTACTS + <usize>lane * 4);
		if (contact != 0) {
			let indexA = b3DecodeAwakeIndex(load<i32>(contact + CT_ENCODEDBODYSIMA));
			let indexB = b3DecodeAwakeIndex(load<i32>(contact + CT_ENCODEDBODYSIMB));
			let contactManifoldCount = load<i32>(contact + CT_MANIFOLDCOUNT);

			store<i32>(c0 + CW_INDEXA + <usize>lane * 4, indexA + 1);
			store<i32>(c0 + CW_INDEXB + <usize>lane * 4, indexB + 1);

			hitEventLanes |= (load<u32>(contact + CT_FLAGS) & b3_simEnableHitEvent) != 0 ? 1 << lane : 0;
			maxManifoldCount = b3MaxInt(maxManifoldCount, contactManifoldCount);

			// b3AllZeroW( c->rollingResistance ) == false, which the lanes share
			anyRolling = anyRolling || load<f32>(contact + CT_ROLLINGRESISTANCE) != 0.0;
		} else {
			store<i32>(c0 + CW_INDEXA + <usize>lane * 4, 0);
			store<i32>(c0 + CW_INDEXB + <usize>lane * 4, 0);
		}
	}

	store<i32>(c0 + CW_MANIFOLDCOUNT, manifoldCount);

	for (let lane = 0; lane < b3LaneCountL<T>(); ++lane) {
		let c = c0 + <usize>lane * 4;
		let l0 = isFloat<T>() ? lane : 0;
		let l1 = isFloat<T>() ? lane : 1;
		let l2 = isFloat<T>() ? lane : 2;
		let l3 = isFloat<T>() ? lane : 3;

		let sA0 = b3SimLane(simData, zeroSim, c0 + CW_INDEXA, l0);
		let sA1 = b3SimLane(simData, zeroSim, c0 + CW_INDEXA, l1);
		let sA2 = b3SimLane(simData, zeroSim, c0 + CW_INDEXA, l2);
		let sA3 = b3SimLane(simData, zeroSim, c0 + CW_INDEXA, l3);
		let sB0 = b3SimLane(simData, zeroSim, c0 + CW_INDEXB, l0);
		let sB1 = b3SimLane(simData, zeroSim, c0 + CW_INDEXB, l1);
		let sB2 = b3SimLane(simData, zeroSim, c0 + CW_INDEXB, l2);
		let sB3 = b3SimLane(simData, zeroSim, c0 + CW_INDEXB, l3);

		// B3_GATHER_LANES over the sims
		let mA = b3SetLoadL<T>(sA0 + BSIM_INVMASS, sA1 + BSIM_INVMASS, sA2 + BSIM_INVMASS, sA3 + BSIM_INVMASS);
		let mB = b3SetLoadL<T>(sB0 + BSIM_INVMASS, sB1 + BSIM_INVMASS, sB2 + BSIM_INVMASS, sB3 + BSIM_INVMASS);

		// C transposes the first nine floats of each lane's invInertiaWorld as rows from cx.x and cy.y
		let iA0 = sA0 + BSIM_INVINERTIA;
		let iA1 = sA1 + BSIM_INVINERTIA;
		let iA2 = sA2 + BSIM_INVINERTIA;
		let iA3 = sA3 + BSIM_INVINERTIA;
		let iB0 = sB0 + BSIM_INVINERTIA;
		let iB1 = sB1 + BSIM_INVINERTIA;
		let iB2 = sB2 + BSIM_INVINERTIA;
		let iB3 = sB3 + BSIM_INVINERTIA;
		let iAx = b3GatherRow3L<T>(iA0, iA1, iA2, iA3, M3_CX);
		let iAxx = iAx.c0;
		let iAxy = iAx.c1;
		let iAxz = iAx.c2;
		let iAy = b3GatherRow2L<T>(iA0, iA1, iA2, iA3, M3_CY + 4);
		let iAyy = iAy.c0;
		let iAyz = iAy.c1;
		let iAzz = b3SetLoadL<T>(iA0 + M3_CZ + 8, iA1 + M3_CZ + 8, iA2 + M3_CZ + 8, iA3 + M3_CZ + 8);
		let iBx = b3GatherRow3L<T>(iB0, iB1, iB2, iB3, M3_CX);
		let iBxx = iBx.c0;
		let iBxy = iBx.c1;
		let iBxz = iBx.c2;
		let iBy = b3GatherRow2L<T>(iB0, iB1, iB2, iB3, M3_CY + 4);
		let iByy = iBy.c0;
		let iByz = iBy.c1;
		let iBzz = b3SetLoadL<T>(iB0 + M3_CZ + 8, iB1 + M3_CZ + 8, iB2 + M3_CZ + 8, iB3 + M3_CZ + 8);

		b3StoreL<T>(c + CW_INVMASSA, mA);
		b3StoreL<T>(c + CW_INVMASSB, mB);
		b3StoreL<T>(c + CW_INVIA + S_CXX, iAxx);
		b3StoreL<T>(c + CW_INVIA + S_CXY, iAxy);
		b3StoreL<T>(c + CW_INVIA + S_CXZ, iAxz);
		b3StoreL<T>(c + CW_INVIA + S_CYY, iAyy);
		b3StoreL<T>(c + CW_INVIA + S_CYZ, iAyz);
		b3StoreL<T>(c + CW_INVIA + S_CZZ, iAzz);
		b3StoreL<T>(c + CW_INVIB + S_CXX, iBxx);
		b3StoreL<T>(c + CW_INVIB + S_CXY, iBxy);
		b3StoreL<T>(c + CW_INVIB + S_CXZ, iBxz);
		b3StoreL<T>(c + CW_INVIB + S_CYY, iByy);
		b3StoreL<T>(c + CW_INVIB + S_CYZ, iByz);
		b3StoreL<T>(c + CW_INVIB + S_CZZ, iBzz);

		let cl0 = b3ContactLane(c0, l0, zeroContact);
		let cl1 = b3ContactLane(c0, l1, zeroContact);
		let cl2 = b3ContactLane(c0, l2, zeroContact);
		let cl3 = b3ContactLane(c0, l3, zeroContact);
		// friction, restitution and rolling resistance are one row of each contact
		let materialRow = b3GatherRow3L<T>(cl0, cl1, cl2, cl3, CT_FRICTION);
		let rollingResistance = materialRow.c2;
		b3StoreL<T>(c + CW_FRICTION, materialRow.c0);
		b3StoreL<T>(c + CW_ROLLINGRESISTANCE, rollingResistance);
		b3StoreL<T>(c + CW_RESTITUTION, materialRow.c1);

		// b3AddSymW( iA, iB )
		let invIABxx = b3AddL(iAxx, iBxx);
		let invIABxy = b3AddL(iAxy, iBxy);
		let invIABxz = b3AddL(iAxz, iBxz);
		let invIAByy = b3AddL(iAyy, iByy);
		let invIAByz = b3AddL(iAyz, iByz);
		let invIABzz = b3AddL(iAzz, iBzz);

		let rm = c + CW_ROLLINGMASS;
		if (anyRolling) {

			// b3InvertSymW( invIAB )
			let cxx = b3SubL(b3MulL(invIAByy, invIABzz), b3MulL(invIAByz, invIAByz));
			let cxy = b3SubL(b3MulL(invIABxz, invIAByz), b3MulL(invIABxy, invIABzz));
			let cxz = b3SubL(b3MulL(invIABxy, invIAByz), b3MulL(invIABxz, invIAByy));
			let cyy = b3SubL(b3MulL(invIABxx, invIABzz), b3MulL(invIABxz, invIABxz));
			let cyz = b3SubL(b3MulL(invIABxy, invIABxz), b3MulL(invIABxx, invIAByz));
			let czz = b3SubL(b3MulL(invIABxx, invIAByy), b3MulL(invIABxy, invIABxy));

			let det = b3AddL(b3MulL(invIABxx, cxx), b3AddL(b3MulL(invIABxy, cxy), b3MulL(invIABxz, cxz)));
			let valid = b3GreaterThanL(b3AbsL(det), b3SplatL<T>(1000.0 * FLT_MIN));
			let invDet = b3BlendL(b3SplatL<T>(0.0), b3DivL(b3SplatL<T>(1.0), det), valid);

			b3StoreL<T>(rm + S_CXX, b3MulL(invDet, cxx));
			b3StoreL<T>(rm + S_CXY, b3MulL(invDet, cxy));
			b3StoreL<T>(rm + S_CXZ, b3MulL(invDet, cxz));
			b3StoreL<T>(rm + S_CYY, b3MulL(invDet, cyy));
			b3StoreL<T>(rm + S_CYZ, b3MulL(invDet, cyz));
			b3StoreL<T>(rm + S_CZZ, b3MulL(invDet, czz));
		} else {
			b3StoreL<T>(rm + S_CXX, b3SplatL<T>(0.0));
			b3StoreL<T>(rm + S_CXY, b3SplatL<T>(0.0));
			b3StoreL<T>(rm + S_CXZ, b3SplatL<T>(0.0));
			b3StoreL<T>(rm + S_CYY, b3SplatL<T>(0.0));
			b3StoreL<T>(rm + S_CYZ, b3SplatL<T>(0.0));
			b3StoreL<T>(rm + S_CZZ, b3SplatL<T>(0.0));
		}

		let rollingResistanceMask = b3GreaterThanL(rollingResistance, b3SplatL<T>(0.0));

		for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
			let cm0 = manifoldData + <usize>(manifoldBase + manifoldIndex) * sizeof<b3ContactManifoldWide>();
			let cm = cm0 + <usize>lane * 4;

			// An inactive lane reads the zeroed manifold, whose point count is 0
			let mk0 = b3LaneManifold(load<usize>(c0 + CW_CONTACTS), manifoldIndex, zeroManifold);
			let mk1 = b3LaneManifold(load<usize>(c0 + CW_CONTACTS + 4), manifoldIndex, zeroManifold);
			let mk2 = b3LaneManifold(load<usize>(c0 + CW_CONTACTS + 8), manifoldIndex, zeroManifold);
			let mk3 = b3LaneManifold(load<usize>(c0 + CW_CONTACTS + 12), manifoldIndex, zeroManifold);
			let pointCount = b3MaxInt(
				b3MaxInt(b3MaxInt(load<i32>(mk0 + M_POINTCOUNT), load<i32>(mk1 + M_POINTCOUNT)), load<i32>(mk2 + M_POINTCOUNT)),
				load<i32>(mk3 + M_POINTCOUNT)
			);

			store<i32>(cm0 + MW_POINTCOUNT, pointCount);

			let m0 = b3PickLane(l0, mk0, mk1, mk2, mk3);
			let m1 = b3PickLane(l1, mk0, mk1, mk2, mk3);
			let m2 = b3PickLane(l2, mk0, mk1, mk2, mk3);
			let m3 = b3PickLane(l3, mk0, mk1, mk2, mk3);

			let activeLanesW = b3SetL<T>(
				m0 != zeroManifold ? <f32>1.0 : <f32>0.0,
				m1 != zeroManifold ? <f32>1.0 : <f32>0.0,
				m2 != zeroManifold ? <f32>1.0 : <f32>0.0,
				m3 != zeroManifold ? <f32>1.0 : <f32>0.0
			);
			let rollingMask = b3AndL(rollingResistanceMask, b3GreaterThanL(activeLanesW, b3SplatL<T>(0.0)));
			b3StoreL<T>(cm + MW_ROLLINGMASK, b3MaskValueL(rollingMask));

			// C transposes the normal, twist, friction and rolling impulse floats of each lane as two rows of four,
			// so these fields stay contiguous in that order
			let headerRow = b3GatherRowL<T>(m0, m1, m2, m3, M_NORMAL);
			let normalX = headerRow.c0;
			let normalY = headerRow.c1;
			let normalZ = headerRow.c2;
			let twistImpulse = headerRow.c3;
			let impulseRow = b3GatherRowL<T>(m0, m1, m2, m3, M_FRICTIONIMPULSE);
			let frictionImpulseX = impulseRow.c0;
			let frictionImpulseY = impulseRow.c1;
			let frictionImpulseZ = impulseRow.c2;
			let rollingImpulseX = impulseRow.c3;
			let rollingImpulseY = b3SetLoadL<T>(m0 + M_ROLLINGIMPULSE + 4, m1 + M_ROLLINGIMPULSE + 4, m2 + M_ROLLINGIMPULSE + 4, m3 + M_ROLLINGIMPULSE + 4);
			let rollingImpulseZ = b3SetLoadL<T>(m0 + M_ROLLINGIMPULSE + 8, m1 + M_ROLLINGIMPULSE + 8, m2 + M_ROLLINGIMPULSE + 8, m3 + M_ROLLINGIMPULSE + 8);

			// b3PerpW( normal )
			let perpMask = b3OrL(b3LessThanL(normalX, b3NegL(b3SplatL<T>(0.5))), b3GreaterThanL(normalX, b3SplatL<T>(0.5)));
			let pX = b3BlendL(b3SplatL<T>(0.0), normalY, perpMask);
			let pY = b3BlendL(normalZ, b3NegL(normalX), perpMask);
			let pZ = b3BlendL(b3NegL(normalY), b3SplatL<T>(0.0), perpMask);
			let perpLengthSquared = b3DotL(pX, pY, pZ, pX, pY, pZ);
			let perpValid = b3GreaterThanL(perpLengthSquared, b3SplatL<T>(1000.0 * FLT_MIN));
			let perpScale = b3BlendL(b3SplatL<T>(0.0), b3DivL(b3SplatL<T>(1.0), b3SqrtL(perpLengthSquared)), perpValid);
			let tangent1X = b3MulL(perpScale, pX);
			let tangent1Y = b3MulL(perpScale, pY);
			let tangent1Z = b3MulL(perpScale, pZ);

			let tangent2X = b3CrossL(tangent1Y, tangent1Z, normalY, normalZ);
			let tangent2Y = b3CrossL(tangent1Z, tangent1X, normalZ, normalX);
			let tangent2Z = b3CrossL(tangent1X, tangent1Y, normalX, normalY);
			b3StoreL<T>(cm + MW_NORMAL + V_X, normalX);
			b3StoreL<T>(cm + MW_NORMAL + V_Y, normalY);
			b3StoreL<T>(cm + MW_NORMAL + V_Z, normalZ);
			b3StoreL<T>(cm + MW_TANGENT1 + V_X, tangent1X);
			b3StoreL<T>(cm + MW_TANGENT1 + V_Y, tangent1Y);
			b3StoreL<T>(cm + MW_TANGENT1 + V_Z, tangent1Z);
			b3StoreL<T>(cm + MW_TANGENT2 + V_X, tangent2X);
			b3StoreL<T>(cm + MW_TANGENT2 + V_Y, tangent2Y);
			b3StoreL<T>(cm + MW_TANGENT2 + V_Z, tangent2Z);
			let tangentVelocity = b3GatherRow3L<T>(cl0, cl1, cl2, cl3, CT_TANGENTVELOCITY);
			let tangentVelocityX = tangentVelocity.c0;
			let tangentVelocityY = tangentVelocity.c1;
			let tangentVelocityZ = tangentVelocity.c2;
			b3StoreL<T>(cm + MW_TANGENTVELOCITY1, b3DotL(tangentVelocityX, tangentVelocityY, tangentVelocityZ, tangent1X, tangent1Y, tangent1Z));
			b3StoreL<T>(cm + MW_TANGENTVELOCITY2, b3DotL(tangentVelocityX, tangentVelocityY, tangentVelocityZ, tangent2X, tangent2Y, tangent2Z));

			b3StoreL<T>(cm + MW_TWISTIMPULSE, b3MulL(warmStartScale, twistImpulse));
			b3StoreL<T>(cm + MW_ROLLINGIMPULSE + V_X, b3BlendL(b3SplatL<T>(0.0), b3MulL(warmStartScale, rollingImpulseX), rollingMask));
			b3StoreL<T>(cm + MW_ROLLINGIMPULSE + V_Y, b3BlendL(b3SplatL<T>(0.0), b3MulL(warmStartScale, rollingImpulseY), rollingMask));
			b3StoreL<T>(cm + MW_ROLLINGIMPULSE + V_Z, b3BlendL(b3SplatL<T>(0.0), b3MulL(warmStartScale, rollingImpulseZ), rollingMask));
			b3StoreL<T>(
				cm + MW_FRICTIONIMPULSE + V2_X,
				b3MulL(warmStartScale, b3DotL(frictionImpulseX, frictionImpulseY, frictionImpulseZ, tangent1X, tangent1Y, tangent1Z))
			);
			b3StoreL<T>(
				cm + MW_FRICTIONIMPULSE + V2_Y,
				b3MulL(warmStartScale, b3DotL(frictionImpulseX, frictionImpulseY, frictionImpulseZ, tangent2X, tangent2Y, tangent2Z))
			);

			let pointCountW = b3SetL<T>(
				<f32>load<i32>(m0 + M_POINTCOUNT),
				<f32>load<i32>(m1 + M_POINTCOUNT),
				<f32>load<i32>(m2 + M_POINTCOUNT),
				<f32>load<i32>(m3 + M_POINTCOUNT)
			);

			let centerAX = b3SplatL<T>(0.0);
			let centerAY = b3SplatL<T>(0.0);
			let centerAZ = b3SplatL<T>(0.0);
			let centerBX = b3SplatL<T>(0.0);
			let centerBY = b3SplatL<T>(0.0);
			let centerBZ = b3SplatL<T>(0.0);
			let totalFrictionWeight = b3SplatL<T>(0.0);

			for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
				let cp = cm + MW_POINTS + <usize>pointIndex * sizeof<b3ContactConstraintPointWide>();
				let pointMask = b3GreaterThanL(pointCountW, b3SplatL<T>(<f32>pointIndex));

				let pointOffset = M_POINTS + <usize>pointIndex * sizeof<b3ManifoldPoint>();
				let p0 = m0 + pointOffset;
				let p1 = m1 + pointOffset;
				let p2 = m2 + pointOffset;
				let p3 = m3 + pointOffset;

				// C transposes anchorA, anchorB, separation and normalImpulse of each lane as two rows of four
				let anchorRow = b3GatherRowL<T>(p0, p1, p2, p3, MP_ANCHORA);
				let pointRow = b3GatherRowL<T>(p0, p1, p2, p3, MP_ANCHORB + 4);
				let rAX = b3BlendL(b3SplatL<T>(0.0), anchorRow.c0, pointMask);
				let rAY = b3BlendL(b3SplatL<T>(0.0), anchorRow.c1, pointMask);
				let rAZ = b3BlendL(b3SplatL<T>(0.0), anchorRow.c2, pointMask);
				let rBX = b3BlendL(b3SplatL<T>(0.0), anchorRow.c3, pointMask);
				let rBY = b3BlendL(b3SplatL<T>(0.0), pointRow.c0, pointMask);
				let rBZ = b3BlendL(b3SplatL<T>(0.0), pointRow.c1, pointMask);
				let separation = b3BlendL(b3SplatL<T>(0.0), pointRow.c2, pointMask);
				let normalImpulse = b3BlendL(b3SplatL<T>(0.0), pointRow.c3, pointMask);

				// C0 friction center decay. Needed to prevent spinning top drift (GyroscopicPrecession sample).
				// See details in b3PrepareContacts_Mesh. This code should stay in sync.
				let weight = b3MinL(
					b3MaxL(b3SubL(b3SplatL<T>(2.0), b3MulL(separation, b3SplatL<T>(invTau))), b3SplatL<T>(B3_MIN_FRICTION_WEIGHT)),
					b3SplatL<T>(1.0)
				);
				weight = b3BlendL(b3SplatL<T>(0.0), weight, pointMask);
				centerAX = b3MulAddL(centerAX, weight, rAX);
				centerAY = b3MulAddL(centerAY, weight, rAY);
				centerAZ = b3MulAddL(centerAZ, weight, rAZ);
				centerBX = b3MulAddL(centerBX, weight, rBX);
				centerBY = b3MulAddL(centerBY, weight, rBY);
				centerBZ = b3MulAddL(centerBZ, weight, rBZ);
				totalFrictionWeight = b3AddL(totalFrictionWeight, weight);

				b3StoreL<T>(cp + PW_ANCHORA + V_X, rAX);
				b3StoreL<T>(cp + PW_ANCHORA + V_Y, rAY);
				b3StoreL<T>(cp + PW_ANCHORA + V_Z, rAZ);
				b3StoreL<T>(cp + PW_ANCHORB + V_X, rBX);
				b3StoreL<T>(cp + PW_ANCHORB + V_Y, rBY);
				b3StoreL<T>(cp + PW_ANCHORB + V_Z, rBZ);
				b3StoreL<T>(
					cp + PW_BASESEPARATION,
					b3SubL(separation, b3DotL(b3SubL(rBX, rAX), b3SubL(rBY, rAY), b3SubL(rBZ, rAZ), normalX, normalY, normalZ))
				);
				b3StoreL<T>(cp + PW_NORMALIMPULSE, b3MulL(warmStartScale, normalImpulse));
				b3StoreL<T>(cp + PW_TOTALNORMALIMPULSE, b3SplatL<T>(0.0));
				b3StoreL<T>(cp + PW_RELATIVEVELOCITY, b3SplatL<T>(0.0));
				b3StoreL<T>(cp + PW_RESTITUTIONIMPULSE, b3SplatL<T>(0.0));

				let rnAX = b3CrossL(rAY, rAZ, normalY, normalZ);
				let rnAY = b3CrossL(rAZ, rAX, normalZ, normalX);
				let rnAZ = b3CrossL(rAX, rAY, normalX, normalY);
				let rnBX = b3CrossL(rBY, rBZ, normalY, normalZ);
				let rnBY = b3CrossL(rBZ, rBX, normalZ, normalX);
				let rnBZ = b3CrossL(rBX, rBY, normalX, normalY);
				let kNormal = b3AddL(mA, mB);
				kNormal = b3AddL(
					kNormal,
					b3DotL(
						rnAX,
						rnAY,
						rnAZ,
						b3MulRowL(iAxx, iAxy, iAxz, rnAX, rnAY, rnAZ),
						b3MulRowL(iAxy, iAyy, iAyz, rnAX, rnAY, rnAZ),
						b3MulRowL(iAxz, iAyz, iAzz, rnAX, rnAY, rnAZ)
					)
				);
				kNormal = b3AddL(
					kNormal,
					b3DotL(
						rnBX,
						rnBY,
						rnBZ,
						b3MulRowL(iBxx, iBxy, iBxz, rnBX, rnBY, rnBZ),
						b3MulRowL(iBxy, iByy, iByz, rnBX, rnBY, rnBZ),
						b3MulRowL(iBxz, iByz, iBzz, rnBX, rnBY, rnBZ)
					)
				);
				let valid = b3AndL(b3GreaterThanL(kNormal, b3SplatL<T>(0.0)), pointMask);
				b3StoreL<T>(cp + PW_NORMALMASS, b3BlendL(b3SplatL<T>(0.0), b3DivL(b3SplatL<T>(1.0), kNormal), valid));
			}

			let frictionValid = b3GreaterThanL(totalFrictionWeight, b3SplatL<T>(0.0));
			let invWeight = b3BlendL(b3SplatL<T>(0.0), b3DivL(b3SplatL<T>(1.0), totalFrictionWeight), frictionValid);
			centerAX = b3MulL(invWeight, centerAX);
			centerAY = b3MulL(invWeight, centerAY);
			centerAZ = b3MulL(invWeight, centerAZ);
			centerBX = b3MulL(invWeight, centerBX);
			centerBY = b3MulL(invWeight, centerBY);
			centerBZ = b3MulL(invWeight, centerBZ);
			b3StoreL<T>(cm + MW_CENTERA + V_X, centerAX);
			b3StoreL<T>(cm + MW_CENTERA + V_Y, centerAY);
			b3StoreL<T>(cm + MW_CENTERA + V_Z, centerAZ);
			b3StoreL<T>(cm + MW_CENTERB + V_X, centerBX);
			b3StoreL<T>(cm + MW_CENTERB + V_Y, centerBY);
			b3StoreL<T>(cm + MW_CENTERB + V_Z, centerBZ);

			for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
				let cp = cm + MW_POINTS + <usize>pointIndex * sizeof<b3ContactConstraintPointWide>();
				let pointMask = b3GreaterThanL(pointCountW, b3SplatL<T>(<f32>pointIndex));
				let dX = b3SubL(b3LoadL<T>(cp + PW_ANCHORA + V_X), centerAX);
				let dY = b3SubL(b3LoadL<T>(cp + PW_ANCHORA + V_Y), centerAY);
				let dZ = b3SubL(b3LoadL<T>(cp + PW_ANCHORA + V_Z), centerAZ);
				b3StoreL<T>(cp + PW_LEVERARM, b3BlendL(b3SplatL<T>(0.0), b3SqrtL(b3DotL(dX, dY, dZ, dX, dY, dZ)), pointMask));
			}

			{
				let rtA1X = b3CrossL(centerAY, centerAZ, tangent1Y, tangent1Z);
				let rtA1Y = b3CrossL(centerAZ, centerAX, tangent1Z, tangent1X);
				let rtA1Z = b3CrossL(centerAX, centerAY, tangent1X, tangent1Y);
				let rtA2X = b3CrossL(centerAY, centerAZ, tangent2Y, tangent2Z);
				let rtA2Y = b3CrossL(centerAZ, centerAX, tangent2Z, tangent2X);
				let rtA2Z = b3CrossL(centerAX, centerAY, tangent2X, tangent2Y);
				let rtB1X = b3CrossL(centerBY, centerBZ, tangent1Y, tangent1Z);
				let rtB1Y = b3CrossL(centerBZ, centerBX, tangent1Z, tangent1X);
				let rtB1Z = b3CrossL(centerBX, centerBY, tangent1X, tangent1Y);
				let rtB2X = b3CrossL(centerBY, centerBZ, tangent2Y, tangent2Z);
				let rtB2Y = b3CrossL(centerBZ, centerBX, tangent2Z, tangent2X);
				let rtB2Z = b3CrossL(centerBX, centerBY, tangent2X, tangent2Y);
				let iArtA1X = b3MulRowL(iAxx, iAxy, iAxz, rtA1X, rtA1Y, rtA1Z);
				let iArtA1Y = b3MulRowL(iAxy, iAyy, iAyz, rtA1X, rtA1Y, rtA1Z);
				let iArtA1Z = b3MulRowL(iAxz, iAyz, iAzz, rtA1X, rtA1Y, rtA1Z);
				let iArtA2X = b3MulRowL(iAxx, iAxy, iAxz, rtA2X, rtA2Y, rtA2Z);
				let iArtA2Y = b3MulRowL(iAxy, iAyy, iAyz, rtA2X, rtA2Y, rtA2Z);
				let iArtA2Z = b3MulRowL(iAxz, iAyz, iAzz, rtA2X, rtA2Y, rtA2Z);
				let iBrtB1X = b3MulRowL(iBxx, iBxy, iBxz, rtB1X, rtB1Y, rtB1Z);
				let iBrtB1Y = b3MulRowL(iBxy, iByy, iByz, rtB1X, rtB1Y, rtB1Z);
				let iBrtB1Z = b3MulRowL(iBxz, iByz, iBzz, rtB1X, rtB1Y, rtB1Z);
				let iBrtB2X = b3MulRowL(iBxx, iBxy, iBxz, rtB2X, rtB2Y, rtB2Z);
				let iBrtB2Y = b3MulRowL(iBxy, iByy, iByz, rtB2X, rtB2Y, rtB2Z);
				let iBrtB2Z = b3MulRowL(iBxz, iByz, iBzz, rtB2X, rtB2Y, rtB2Z);

				let kxx = b3AddL(
					b3AddL(b3AddL(mA, mB), b3DotL(rtA1X, rtA1Y, rtA1Z, iArtA1X, iArtA1Y, iArtA1Z)),
					b3DotL(rtB1X, rtB1Y, rtB1Z, iBrtB1X, iBrtB1Y, iBrtB1Z)
				);
				let kyy = b3AddL(
					b3AddL(b3AddL(mA, mB), b3DotL(rtA2X, rtA2Y, rtA2Z, iArtA2X, iArtA2Y, iArtA2Z)),
					b3DotL(rtB2X, rtB2Y, rtB2Z, iBrtB2X, iBrtB2Y, iBrtB2Z)
				);
				let kxy = b3AddL(
					b3DotL(rtA1X, rtA1Y, rtA1Z, iArtA2X, iArtA2Y, iArtA2Z),
					b3DotL(rtB1X, rtB1Y, rtB1Z, iBrtB2X, iBrtB2Y, iBrtB2Z)
				);

				let det = b3SubL(b3MulL(kxx, kyy), b3MulL(kxy, kxy));
				let valid = b3AndL(b3GreaterThanL(b3AbsL(det), b3SplatL<T>(1000.0 * FLT_MIN)), frictionValid);
				let invDet = b3BlendL(b3SplatL<T>(0.0), b3DivL(b3SplatL<T>(1.0), det), valid);
				let tm = cm + MW_TANGENTMASS;
				b3StoreL<T>(tm + S2_CXX, b3MulL(invDet, kyy));
				b3StoreL<T>(tm + S2_CXY, b3NegL(b3MulL(invDet, kxy)));
				b3StoreL<T>(tm + S2_CYY, b3MulL(invDet, kxx));
			}

			{
				let kTwist = b3DotL(
					normalX,
					normalY,
					normalZ,
					b3MulRowL(invIABxx, invIABxy, invIABxz, normalX, normalY, normalZ),
					b3MulRowL(invIABxy, invIAByy, invIAByz, normalX, normalY, normalZ),
					b3MulRowL(invIABxz, invIAByz, invIABzz, normalX, normalY, normalZ)
				);
				b3StoreL<T>(cm + MW_TWISTMASS, b3BlendL(b3SplatL<T>(0.0), b3DivL(b3SplatL<T>(1.0), kTwist), b3GreaterThanL(kTwist, b3SplatL<T>(0.0))));
			}
		}
	}

	// Only sample contact point normal velocity if needed.
	let haveRestitution = b3AnyPositiveAt(c0 + CW_RESTITUTION);
	if (hitEventLanes != 0 || haveRestitution) {
		for (let lane = 0; lane < b3LaneCountL<T>(); ++lane) {
			let c = c0 + <usize>lane * 4;

			// An empty lane only writes its own constraint lanes, which nothing reads
			if (isFloat<T>() && load<usize>(c + CW_CONTACTS) == 0) {
				continue;
			}

			let pA1 = b3StatePointer(states, identity, load<i32>(c + CW_INDEXA));
			let pA2 = isFloat<T>() ? pA1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXA + 4));
			let pA3 = isFloat<T>() ? pA1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXA + 8));
			let pA4 = isFloat<T>() ? pA1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXA + 12));
			let pB1 = b3StatePointer(states, identity, load<i32>(c + CW_INDEXB));
			let pB2 = isFloat<T>() ? pB1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXB + 4));
			let pB3 = isFloat<T>() ? pB1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXB + 8));
			let pB4 = isFloat<T>() ? pB1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXB + 12));
			let vA = b3GatherRow3L<T>(pA1, pA2, pA3, pA4, BS_V);
			let vAX = vA.c0;
			let vAY = vA.c1;
			let vAZ = vA.c2;
			let wA = b3GatherRow3L<T>(pA1, pA2, pA3, pA4, BS_W);
			let wAX = wA.c0;
			let wAY = wA.c1;
			let wAZ = wA.c2;
			let vB = b3GatherRow3L<T>(pB1, pB2, pB3, pB4, BS_V);
			let vBX = vB.c0;
			let vBY = vB.c1;
			let vBZ = vB.c2;
			let wB = b3GatherRow3L<T>(pB1, pB2, pB3, pB4, BS_W);
			let wBX = wB.c0;
			let wBY = wB.c1;
			let wBZ = wB.c2;

			// The lanes this kernel writes hit event velocities for
			let laneBegin = isFloat<T>() ? lane : 0;
			let laneEnd = isFloat<T>() ? lane + 1 : 4;

			for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
				let cm0 = manifoldData + <usize>(manifoldBase + manifoldIndex) * sizeof<b3ContactManifoldWide>();
				let cm = cm0 + <usize>lane * 4;
				let normalX = b3LoadL<T>(cm + MW_NORMAL + V_X);
				let normalY = b3LoadL<T>(cm + MW_NORMAL + V_Y);
				let normalZ = b3LoadL<T>(cm + MW_NORMAL + V_Z);

				let pointCount = load<i32>(cm0 + MW_POINTCOUNT);
				for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
					let cp = cm + MW_POINTS + <usize>pointIndex * sizeof<b3ContactConstraintPointWide>();
					let rAX = b3LoadL<T>(cp + PW_ANCHORA + V_X);
					let rAY = b3LoadL<T>(cp + PW_ANCHORA + V_Y);
					let rAZ = b3LoadL<T>(cp + PW_ANCHORA + V_Z);
					let rBX = b3LoadL<T>(cp + PW_ANCHORB + V_X);
					let rBY = b3LoadL<T>(cp + PW_ANCHORB + V_Y);
					let rBZ = b3LoadL<T>(cp + PW_ANCHORB + V_Z);

					let vrAX = b3AddL(vAX, b3CrossL(wAY, wAZ, rAY, rAZ));
					let vrAY = b3AddL(vAY, b3CrossL(wAZ, wAX, rAZ, rAX));
					let vrAZ = b3AddL(vAZ, b3CrossL(wAX, wAY, rAX, rAY));
					let vrBX = b3AddL(vBX, b3CrossL(wBY, wBZ, rBY, rBZ));
					let vrBY = b3AddL(vBY, b3CrossL(wBZ, wBX, rBZ, rBX));
					let vrBZ = b3AddL(vBZ, b3CrossL(wBX, wBY, rBX, rBY));
					let vn = b3DotL(normalX, normalY, normalZ, b3SubL(vrBX, vrAX), b3SubL(vrBY, vrAY), b3SubL(vrBZ, vrAZ));
					b3StoreL<T>(cp + PW_RELATIVEVELOCITY, vn);

					if (hitEventLanes != 0) {
						for (let k = laneBegin; k < laneEnd; ++k) {
							if ((hitEventLanes & (1 << k)) == 0) {
								continue;
							}

							let manifold = b3LaneManifold(load<usize>(c0 + CW_CONTACTS + <usize>k * 4), manifoldIndex, zeroManifold);
							if (pointIndex < load<i32>(manifold + M_POINTCOUNT)) {
								let point = manifold + M_POINTS + <usize>pointIndex * sizeof<b3ManifoldPoint>();
								store<f32>(point + MP_NORMALVELOCITY, b3LaneL(vn, k));
							}
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
	let warmStartScale: f32 = world.enableWarmStarting ? 1.0 : 0.0;
	let anyRestitution = false;
	let contactData = b3ArrayData(world.contacts);

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
			let localWideIndex = wideIndex - colorWideStart;

			for (let lane = 0; lane < 4; ++lane) {
				let contactIndex = 4 * localWideIndex + lane;
				let contactId = B3_NULL_INDEX;
				if (contactIndex < colorContactCount) {
					contactId = contactIds[contactIndex];
					B3_ASSERT(world.contacts[contactId].manifoldCount == 1);
				}
				b3SetConstraintContact(constraints, wideIndex, lane, contactData, contactId);
			}

			let haveRestitution = false;
			if (ASC_FEATURE_SIMD) {
				haveRestitution = b3PrepareConstraintL<v128>(constraints, wideIndex, manifolds, wideIndex, 1, context, warmStartScale);
			} else {
				haveRestitution = b3PrepareConstraintL<f32>(constraints, wideIndex, manifolds, wideIndex, 1, context, warmStartScale);
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
	let warmStartScale: f32 = world.enableWarmStarting ? 1.0 : 0.0;
	let anyRestitution = false;
	let contactData = b3ArrayData(world.contacts);

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
			let localWideIndex = wideIndex - colorWideStart;

			for (let lane = 0; lane < 4; ++lane) {
				let contactIndex = 4 * localWideIndex + lane;
				let contactId = B3_NULL_INDEX;
				if (contactIndex < colorContactCount) {
					let spec = specs[order[orderBase + contactIndex]];
					B3_ASSERT(world.contacts[spec.contactId].contactId == spec.contactId);
					B3_ASSERT(world.contacts[spec.contactId].manifoldCount == <i32>spec.manifoldCount);
					contactId = spec.contactId;
				}
				b3SetConstraintContact(constraints, wideIndex, lane, contactData, contactId);
			}

			let manifoldCount = manifoldStarts[wideIndex + 1] - manifoldStarts[wideIndex];
			let manifoldBase = manifoldStarts[wideIndex];
			let haveRestitution = false;
			if (ASC_FEATURE_SIMD) {
				haveRestitution = b3PrepareConstraintL<v128>(
					constraints,
					wideIndex,
					manifolds,
					manifoldBase,
					manifoldCount,
					context,
					warmStartScale
				);
			} else {
				haveRestitution = b3PrepareConstraintL<f32>(
					constraints,
					wideIndex,
					manifolds,
					manifoldBase,
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

// C's c->contacts[lane] = contacts[lane]; B3_NULL_INDEX stores C's NULL
@inline unsafe function b3SetConstraintContact(
	constraints: StaticArray<b3ContactConstraintWide>,
	constraintIndex: i32,
	lane: i32,
	contactData: usize,
	contactId: i32
): void {
	let contact = contactId != B3_NULL_INDEX ? load<usize>(contactData + <usize>contactId * sizeof<usize>()) : 0;
	let c = changetype<usize>(constraints) + <usize>constraintIndex * sizeof<b3ContactConstraintWide>();
	store<usize>(c + CW_CONTACTS + <usize>lane * 4, contact);
}

// C's base pointer is the constraint array from constraintBase. The first manifold of the block is at the constraint
// index for convex contacts (one manifold each) and at manifoldStarts[block.startIndex] for mesh groups.
unsafe function b3WarmStartContactsL<T>(
	block: b3SolverBlock,
	context: b3StepContext,
	constraints: StaticArray<b3ContactConstraintWide>,
	manifolds: StaticArray<b3ContactManifoldWide>,
	constraintBase: i32,
	manifoldStarts: Int32Array,
	fixedManifoldCount: i32
): void {
	let states = b3StatesData(context.states);
	let identity = changetype<usize>(b3_identityStates);
	let constraintData = changetype<usize>(constraints);
	let manifoldData = changetype<usize>(manifolds);
	let zeroW = b3SplatL<T>(0.0);

	let ci = constraintBase + block.startIndex;
	let mi = fixedManifoldCount > 0 ? ci : manifoldStarts[block.startIndex];
	for (let i = 0; i < <i32>block.count; ++i) {
		let c0 = constraintData + <usize>ci * sizeof<b3ContactConstraintWide>();
		let manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : load<i32>(c0 + CW_MANIFOLDCOUNT);
		let anyRolling = b3AllZeroAt(c0 + CW_ROLLINGRESISTANCE) == false;
		for (let lane = 0; lane < b3LaneCountL<T>(); ++lane) {
			let c = c0 + <usize>lane * 4;

			// An empty lane only writes its own constraint lanes, which nothing reads
			if (isFloat<T>() && load<usize>(c + CW_CONTACTS) == 0) {
				continue;
			}

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
				let cm0 = manifoldData + <usize>(mi + manifoldIndex) * sizeof<b3ContactManifoldWide>();
				let cm = cm0 + <usize>lane * 4;

				let totalNormalImpulse = zeroW;
				let momentAX = zeroW;
				let momentAY = zeroW;
				let momentAZ = zeroW;
				let momentBX = zeroW;
				let momentBY = zeroW;
				let momentBZ = zeroW;

				let pointCount = load<i32>(cm0 + MW_POINTCOUNT);
				for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
					let cp = cm + MW_POINTS + <usize>pointIndex * sizeof<b3ContactConstraintPointWide>();
					let normalImpulse = b3LoadL<T>(cp + PW_NORMALIMPULSE);
					totalNormalImpulse = b3AddL(totalNormalImpulse, normalImpulse);
					momentAX = b3MulAddL(momentAX, normalImpulse, b3LoadL<T>(cp + PW_ANCHORA + V_X));
					momentAY = b3MulAddL(momentAY, normalImpulse, b3LoadL<T>(cp + PW_ANCHORA + V_Y));
					momentAZ = b3MulAddL(momentAZ, normalImpulse, b3LoadL<T>(cp + PW_ANCHORA + V_Z));
					momentBX = b3MulAddL(momentBX, normalImpulse, b3LoadL<T>(cp + PW_ANCHORB + V_X));
					momentBY = b3MulAddL(momentBY, normalImpulse, b3LoadL<T>(cp + PW_ANCHORB + V_Y));
					momentBZ = b3MulAddL(momentBZ, normalImpulse, b3LoadL<T>(cp + PW_ANCHORB + V_Z));
				}

				let normalX = b3LoadL<T>(cm + MW_NORMAL + V_X);
				let normalY = b3LoadL<T>(cm + MW_NORMAL + V_Y);
				let normalZ = b3LoadL<T>(cm + MW_NORMAL + V_Z);
				let frictionImpulseTx = b3LoadL<T>(cm + MW_FRICTIONIMPULSE + V2_X);
				let frictionImpulseTy = b3LoadL<T>(cm + MW_FRICTIONIMPULSE + V2_Y);
				let frictionImpulseX = b3MulL(frictionImpulseTx, b3LoadL<T>(cm + MW_TANGENT1 + V_X));
				let frictionImpulseY = b3MulL(frictionImpulseTx, b3LoadL<T>(cm + MW_TANGENT1 + V_Y));
				let frictionImpulseZ = b3MulL(frictionImpulseTx, b3LoadL<T>(cm + MW_TANGENT1 + V_Z));
				frictionImpulseX = b3MulAddL(frictionImpulseX, frictionImpulseTy, b3LoadL<T>(cm + MW_TANGENT2 + V_X));
				frictionImpulseY = b3MulAddL(frictionImpulseY, frictionImpulseTy, b3LoadL<T>(cm + MW_TANGENT2 + V_Y));
				frictionImpulseZ = b3MulAddL(frictionImpulseZ, frictionImpulseTy, b3LoadL<T>(cm + MW_TANGENT2 + V_Z));

				let linearX = b3MulAddL(frictionImpulseX, totalNormalImpulse, normalX);
				let linearY = b3MulAddL(frictionImpulseY, totalNormalImpulse, normalY);
				let linearZ = b3MulAddL(frictionImpulseZ, totalNormalImpulse, normalZ);

				let twistImpulse = b3LoadL<T>(cm + MW_TWISTIMPULSE);
				let twistImpulseX = b3MulL(twistImpulse, normalX);
				let twistImpulseY = b3MulL(twistImpulse, normalY);
				let twistImpulseZ = b3MulL(twistImpulse, normalZ);

				let centerAX = b3LoadL<T>(cm + MW_CENTERA + V_X);
				let centerAY = b3LoadL<T>(cm + MW_CENTERA + V_Y);
				let centerAZ = b3LoadL<T>(cm + MW_CENTERA + V_Z);
				let centerBX = b3LoadL<T>(cm + MW_CENTERB + V_X);
				let centerBY = b3LoadL<T>(cm + MW_CENTERB + V_Y);
				let centerBZ = b3LoadL<T>(cm + MW_CENTERB + V_Z);

				let angularAX = b3AddL(
					b3AddL(
						b3CrossL(momentAY, momentAZ, normalY, normalZ),
						b3CrossL(centerAY, centerAZ, frictionImpulseY, frictionImpulseZ)
					),
					twistImpulseX
				);
				let angularAY = b3AddL(
					b3AddL(
						b3CrossL(momentAZ, momentAX, normalZ, normalX),
						b3CrossL(centerAZ, centerAX, frictionImpulseZ, frictionImpulseX)
					),
					twistImpulseY
				);
				let angularAZ = b3AddL(
					b3AddL(
						b3CrossL(momentAX, momentAY, normalX, normalY),
						b3CrossL(centerAX, centerAY, frictionImpulseX, frictionImpulseY)
					),
					twistImpulseZ
				);
				let angularBX = b3AddL(
					b3AddL(
						b3CrossL(momentBY, momentBZ, normalY, normalZ),
						b3CrossL(centerBY, centerBZ, frictionImpulseY, frictionImpulseZ)
					),
					twistImpulseX
				);
				let angularBY = b3AddL(
					b3AddL(
						b3CrossL(momentBZ, momentBX, normalZ, normalX),
						b3CrossL(centerBZ, centerBX, frictionImpulseZ, frictionImpulseX)
					),
					twistImpulseY
				);
				let angularBZ = b3AddL(
					b3AddL(
						b3CrossL(momentBX, momentBY, normalX, normalY),
						b3CrossL(centerBX, centerBY, frictionImpulseX, frictionImpulseY)
					),
					twistImpulseZ
				);

				if (anyRolling) {
					let rollingX = b3LoadL<T>(cm + MW_ROLLINGIMPULSE + V_X);
					let rollingY = b3LoadL<T>(cm + MW_ROLLINGIMPULSE + V_Y);
					let rollingZ = b3LoadL<T>(cm + MW_ROLLINGIMPULSE + V_Z);
					angularAX = b3AddL(angularAX, rollingX);
					angularAY = b3AddL(angularAY, rollingY);
					angularAZ = b3AddL(angularAZ, rollingZ);
					angularBX = b3AddL(angularBX, rollingX);
					angularBY = b3AddL(angularBY, rollingY);
					angularBZ = b3AddL(angularBZ, rollingZ);
				}

				linearImpulseX = b3AddL(linearImpulseX, linearX);
				linearImpulseY = b3AddL(linearImpulseY, linearY);
				linearImpulseZ = b3AddL(linearImpulseZ, linearZ);
				angularImpulseAX = b3AddL(angularImpulseAX, angularAX);
				angularImpulseAY = b3AddL(angularImpulseAY, angularAY);
				angularImpulseAZ = b3AddL(angularImpulseAZ, angularAZ);
				angularImpulseBX = b3AddL(angularImpulseBX, angularBX);
				angularImpulseBY = b3AddL(angularImpulseBY, angularBY);
				angularImpulseBZ = b3AddL(angularImpulseBZ, angularBZ);
			}

			let pA1 = b3StatePointer(states, identity, load<i32>(c + CW_INDEXA));
			let pA2 = isFloat<T>() ? pA1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXA + 4));
			let pA3 = isFloat<T>() ? pA1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXA + 8));
			let pA4 = isFloat<T>() ? pA1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXA + 12));
			let pB1 = b3StatePointer(states, identity, load<i32>(c + CW_INDEXB));
			let pB2 = isFloat<T>() ? pB1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXB + 4));
			let pB3 = isFloat<T>() ? pB1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXB + 8));
			let pB4 = isFloat<T>() ? pB1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXB + 12));
			let vA = b3GatherRow3L<T>(pA1, pA2, pA3, pA4, BS_V);
			let vAX = vA.c0;
			let vAY = vA.c1;
			let vAZ = vA.c2;
			let wA = b3GatherRow3L<T>(pA1, pA2, pA3, pA4, BS_W);
			let wAX = wA.c0;
			let wAY = wA.c1;
			let wAZ = wA.c2;
			let vB = b3GatherRow3L<T>(pB1, pB2, pB3, pB4, BS_V);
			let vBX = vB.c0;
			let vBY = vB.c1;
			let vBZ = vB.c2;
			let wB = b3GatherRow3L<T>(pB1, pB2, pB3, pB4, BS_W);
			let wBX = wB.c0;
			let wBY = wB.c1;
			let wBZ = wB.c2;

			let iAxx = b3LoadL<T>(c + CW_INVIA + S_CXX);
			let iAxy = b3LoadL<T>(c + CW_INVIA + S_CXY);
			let iAxz = b3LoadL<T>(c + CW_INVIA + S_CXZ);
			let iAyy = b3LoadL<T>(c + CW_INVIA + S_CYY);
			let iAyz = b3LoadL<T>(c + CW_INVIA + S_CYZ);
			let iAzz = b3LoadL<T>(c + CW_INVIA + S_CZZ);
			wAX = b3SubL(wAX, b3MulRowL(iAxx, iAxy, iAxz, angularImpulseAX, angularImpulseAY, angularImpulseAZ));
			wAY = b3SubL(wAY, b3MulRowL(iAxy, iAyy, iAyz, angularImpulseAX, angularImpulseAY, angularImpulseAZ));
			wAZ = b3SubL(wAZ, b3MulRowL(iAxz, iAyz, iAzz, angularImpulseAX, angularImpulseAY, angularImpulseAZ));
			let invMassA = b3LoadL<T>(c + CW_INVMASSA);
			vAX = b3MulSubL(vAX, invMassA, linearImpulseX);
			vAY = b3MulSubL(vAY, invMassA, linearImpulseY);
			vAZ = b3MulSubL(vAZ, invMassA, linearImpulseZ);
			let iBxx = b3LoadL<T>(c + CW_INVIB + S_CXX);
			let iBxy = b3LoadL<T>(c + CW_INVIB + S_CXY);
			let iBxz = b3LoadL<T>(c + CW_INVIB + S_CXZ);
			let iByy = b3LoadL<T>(c + CW_INVIB + S_CYY);
			let iByz = b3LoadL<T>(c + CW_INVIB + S_CYZ);
			let iBzz = b3LoadL<T>(c + CW_INVIB + S_CZZ);
			wBX = b3AddL(wBX, b3MulRowL(iBxx, iBxy, iBxz, angularImpulseBX, angularImpulseBY, angularImpulseBZ));
			wBY = b3AddL(wBY, b3MulRowL(iBxy, iByy, iByz, angularImpulseBX, angularImpulseBY, angularImpulseBZ));
			wBZ = b3AddL(wBZ, b3MulRowL(iBxz, iByz, iBzz, angularImpulseBX, angularImpulseBY, angularImpulseBZ));
			let invMassB = b3LoadL<T>(c + CW_INVMASSB);
			vBX = b3MulAddL(vBX, invMassB, linearImpulseX);
			vBY = b3MulAddL(vBY, invMassB, linearImpulseY);
			vBZ = b3MulAddL(vBZ, invMassB, linearImpulseZ);

			b3ScatterBodiesL(states, c + CW_INDEXA, vAX, vAY, vAZ, wAX, wAY, wAZ);
			b3ScatterBodiesL(states, c + CW_INDEXB, vBX, vBY, vBZ, wBX, wBY, wBZ);
		}

		ci += 1;
		mi += manifoldCount;
	}
}

// Solve the non-penetration constraints with the soft bias. No friction and no restitution.
unsafe function b3PushContactsL<T>(
	block: b3SolverBlock,
	context: b3StepContext,
	constraints: StaticArray<b3ContactConstraintWide>,
	manifolds: StaticArray<b3ContactManifoldWide>,
	constraintBase: i32,
	manifoldStarts: Int32Array,
	fixedManifoldCount: i32
): void {
	let states = b3StatesData(context.states);
	let identity = changetype<usize>(b3_identityStates);
	let constraintData = changetype<usize>(constraints);
	let manifoldData = changetype<usize>(manifolds);
	let invH = context.inv_h;
	let contactSpeed = b3SplatL<T>(-context.world.contactSpeed);

	let dynamicBiasRate = b3SplatL<T>(context.contactSoftness.massScale * context.contactSoftness.biasRate);
	let dynamicMassScale = b3SplatL<T>(context.contactSoftness.massScale);
	let dynamicImpulseScale = b3SplatL<T>(context.contactSoftness.impulseScale);
	let staticBiasRate = b3SplatL<T>(context.staticSoftness.massScale * context.staticSoftness.biasRate);
	let staticMassScale = b3SplatL<T>(context.staticSoftness.massScale);
	let staticImpulseScale = b3SplatL<T>(context.staticSoftness.impulseScale);

	let ci = constraintBase + block.startIndex;
	let mi = fixedManifoldCount > 0 ? ci : manifoldStarts[block.startIndex];
	for (let i = 0; i < <i32>block.count; ++i) {
		let c0 = constraintData + <usize>ci * sizeof<b3ContactConstraintWide>();
		let manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : load<i32>(c0 + CW_MANIFOLDCOUNT);
		for (let lane = 0; lane < b3LaneCountL<T>(); ++lane) {
			let c = c0 + <usize>lane * 4;

			// An empty lane only writes its own constraint lanes, which nothing reads
			if (isFloat<T>() && load<usize>(c + CW_CONTACTS) == 0) {
				continue;
			}

			let pA1 = b3StatePointer(states, identity, load<i32>(c + CW_INDEXA));
			let pA2 = isFloat<T>() ? pA1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXA + 4));
			let pA3 = isFloat<T>() ? pA1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXA + 8));
			let pA4 = isFloat<T>() ? pA1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXA + 12));
			let pB1 = b3StatePointer(states, identity, load<i32>(c + CW_INDEXB));
			let pB2 = isFloat<T>() ? pB1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXB + 4));
			let pB3 = isFloat<T>() ? pB1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXB + 8));
			let pB4 = isFloat<T>() ? pB1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXB + 12));
			let vA = b3GatherRow3L<T>(pA1, pA2, pA3, pA4, BS_V);
			let vAX = vA.c0;
			let vAY = vA.c1;
			let vAZ = vA.c2;
			let wA = b3GatherRow3L<T>(pA1, pA2, pA3, pA4, BS_W);
			let wAX = wA.c0;
			let wAY = wA.c1;
			let wAZ = wA.c2;
			let vB = b3GatherRow3L<T>(pB1, pB2, pB3, pB4, BS_V);
			let vBX = vB.c0;
			let vBY = vB.c1;
			let vBZ = vB.c2;
			let wB = b3GatherRow3L<T>(pB1, pB2, pB3, pB4, BS_W);
			let wBX = wB.c0;
			let wBY = wB.c1;
			let wBZ = wB.c2;

			// b3SoftMaskW
			let softMask = b3SoftMaskL<T>(c + CW_INDEXA, c + CW_INDEXB);
			let biasRate = b3BlendL(dynamicBiasRate, staticBiasRate, softMask);
			let massScale = b3BlendL(dynamicMassScale, staticMassScale, softMask);
			let impulseScale = b3BlendL(dynamicImpulseScale, staticImpulseScale, softMask);

			let invMassA = b3LoadL<T>(c + CW_INVMASSA);
			let invMassB = b3LoadL<T>(c + CW_INVMASSB);
			let invMass = b3AddL(invMassA, invMassB);

			for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
				let cm0 = manifoldData + <usize>(mi + manifoldIndex) * sizeof<b3ContactManifoldWide>();
				let cm = cm0 + <usize>lane * 4;

				let dpA = b3GatherRow3L<T>(pA1, pA2, pA3, pA4, BS_DP);
				let dpAX = dpA.c0;
				let dpAY = dpA.c1;
				let dpAZ = dpA.c2;
				let dqA = b3GatherRowL<T>(pA1, pA2, pA3, pA4, BS_DQ);
				let dqAX = dqA.c0;
				let dqAY = dqA.c1;
				let dqAZ = dqA.c2;
				let dqAS = dqA.c3;
				let dpB = b3GatherRow3L<T>(pB1, pB2, pB3, pB4, BS_DP);
				let dpBX = dpB.c0;
				let dpBY = dpB.c1;
				let dpBZ = dpB.c2;
				let dqB = b3GatherRowL<T>(pB1, pB2, pB3, pB4, BS_DQ);
				let dqBX = dqB.c0;
				let dqBY = dqB.c1;
				let dqBZ = dqB.c2;
				let dqBS = dqB.c3;

				let dpX = b3SubL(dpBX, dpAX);
				let dpY = b3SubL(dpBY, dpAY);
				let dpZ = b3SubL(dpBZ, dpAZ);

				let normalX = b3LoadL<T>(cm + MW_NORMAL + V_X);
				let normalY = b3LoadL<T>(cm + MW_NORMAL + V_Y);
				let normalZ = b3LoadL<T>(cm + MW_NORMAL + V_Z);

				// Convert to normals to local space to reduce transform math.
				let normalSeparation = b3DotL(normalX, normalY, normalZ, dpX, dpY, dpZ);

				// b3InvRotateVectorW( bA.dq, normal )
				let tAX = b3CrossL(dqAY, dqAZ, normalY, normalZ);
				let tAY = b3CrossL(dqAZ, dqAX, normalZ, normalX);
				let tAZ = b3CrossL(dqAX, dqAY, normalX, normalY);
				tAX = b3AddL(tAX, tAX);
				tAY = b3AddL(tAY, tAY);
				tAZ = b3AddL(tAZ, tAZ);
				let normalAX = b3AddL(b3MulSubL(normalX, dqAS, tAX), b3CrossL(dqAY, dqAZ, tAY, tAZ));
				let normalAY = b3AddL(b3MulSubL(normalY, dqAS, tAY), b3CrossL(dqAZ, dqAX, tAZ, tAX));
				let normalAZ = b3AddL(b3MulSubL(normalZ, dqAS, tAZ), b3CrossL(dqAX, dqAY, tAX, tAY));

				// b3InvRotateVectorW( bB.dq, normal )
				let tBX = b3CrossL(dqBY, dqBZ, normalY, normalZ);
				let tBY = b3CrossL(dqBZ, dqBX, normalZ, normalX);
				let tBZ = b3CrossL(dqBX, dqBY, normalX, normalY);
				tBX = b3AddL(tBX, tBX);
				tBY = b3AddL(tBY, tBY);
				tBZ = b3AddL(tBZ, tBZ);
				let normalBX = b3AddL(b3MulSubL(normalX, dqBS, tBX), b3CrossL(dqBY, dqBZ, tBY, tBZ));
				let normalBY = b3AddL(b3MulSubL(normalY, dqBS, tBY), b3CrossL(dqBZ, dqBX, tBZ, tBX));
				let normalBZ = b3AddL(b3MulSubL(normalZ, dqBS, tBZ), b3CrossL(dqBX, dqBY, tBX, tBY));

				let linearNormalVelocity = b3DotL(normalX, normalY, normalZ, b3SubL(vBX, vAX), b3SubL(vBY, vAY), b3SubL(vBZ, vAZ));
				let manifoldImpulse = b3SplatL<T>(0.0);

				let pointCount = load<i32>(cm0 + MW_POINTCOUNT);
				for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
					let cp = cm + MW_POINTS + <usize>pointIndex * sizeof<b3ContactConstraintPointWide>();

					// Fixed anchor points for applying impulses
					let rAX = b3LoadL<T>(cp + PW_ANCHORA + V_X);
					let rAY = b3LoadL<T>(cp + PW_ANCHORA + V_Y);
					let rAZ = b3LoadL<T>(cp + PW_ANCHORA + V_Z);
					let rBX = b3LoadL<T>(cp + PW_ANCHORB + V_X);
					let rBY = b3LoadL<T>(cp + PW_ANCHORB + V_Y);
					let rBZ = b3LoadL<T>(cp + PW_ANCHORB + V_Z);

					let s = b3AddL(
						b3AddL(
							normalSeparation,
							b3SubL(b3DotL(normalBX, normalBY, normalBZ, rBX, rBY, rBZ), b3DotL(normalAX, normalAY, normalAZ, rAX, rAY, rAZ))
						),
						b3LoadL<T>(cp + PW_BASESEPARATION)
					);

					// Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
					let separated = b3GreaterThanL(s, b3SplatL<T>(0.0));

					// Speculative bias - positive
					let specBias = b3MulL(s, b3SplatL<T>(invH));

					// Overlap bias - negative
					let overlapBias = b3MaxL(b3MulL(biasRate, s), contactSpeed);
					let velocityBias = b3BlendL(overlapBias, specBias, separated);

					let pointMassScale = b3BlendL(massScale, b3SplatL<T>(1.0), separated);
					let pointImpulseScale = b3BlendL(impulseScale, b3SplatL<T>(0.0), separated);

					let uAX = b3CrossL(rAY, rAZ, normalY, normalZ);
					let uAY = b3CrossL(rAZ, rAX, normalZ, normalX);
					let uAZ = b3CrossL(rAX, rAY, normalX, normalY);
					let uBX = b3CrossL(rBY, rBZ, normalY, normalZ);
					let uBY = b3CrossL(rBZ, rBX, normalZ, normalX);
					let uBZ = b3CrossL(rBX, rBY, normalX, normalY);
					let gAX = b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXX), b3LoadL<T>(c + CW_INVIA + S_CXY), b3LoadL<T>(c + CW_INVIA + S_CXZ), uAX, uAY, uAZ);
					let gAY = b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXY), b3LoadL<T>(c + CW_INVIA + S_CYY), b3LoadL<T>(c + CW_INVIA + S_CYZ), uAX, uAY, uAZ);
					let gAZ = b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXZ), b3LoadL<T>(c + CW_INVIA + S_CYZ), b3LoadL<T>(c + CW_INVIA + S_CZZ), uAX, uAY, uAZ);
					let gBX = b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXX), b3LoadL<T>(c + CW_INVIB + S_CXY), b3LoadL<T>(c + CW_INVIB + S_CXZ), uBX, uBY, uBZ);
					let gBY = b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXY), b3LoadL<T>(c + CW_INVIB + S_CYY), b3LoadL<T>(c + CW_INVIB + S_CYZ), uBX, uBY, uBZ);
					let gBZ = b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXZ), b3LoadL<T>(c + CW_INVIB + S_CYZ), b3LoadL<T>(c + CW_INVIB + S_CZZ), uBX, uBY, uBZ);

					let vn = b3AddL(linearNormalVelocity, b3SubL(b3DotL(uBX, uBY, uBZ, wBX, wBY, wBZ), b3DotL(uAX, uAY, uAZ, wAX, wAY, wAZ)));

					let normalImpulse = b3LoadL<T>(cp + PW_NORMALIMPULSE);
					let negImpulse = b3AddL(
						b3MulL(b3LoadL<T>(cp + PW_NORMALMASS), b3AddL(b3MulL(pointMassScale, vn), velocityBias)),
						b3MulL(pointImpulseScale, normalImpulse)
					);
					let impulse = b3MaxL(b3NegL(negImpulse), b3NegL(normalImpulse));
					b3StoreL<T>(cp + PW_NORMALIMPULSE, b3AddL(normalImpulse, impulse));

					linearNormalVelocity = b3MulAddL(linearNormalVelocity, invMass, impulse);
					manifoldImpulse = b3AddL(manifoldImpulse, impulse);
					wAX = b3MulSubL(wAX, impulse, gAX);
					wAY = b3MulSubL(wAY, impulse, gAY);
					wAZ = b3MulSubL(wAZ, impulse, gAZ);
					wBX = b3MulAddL(wBX, impulse, gBX);
					wBY = b3MulAddL(wBY, impulse, gBY);
					wBZ = b3MulAddL(wBZ, impulse, gBZ);
				}

				let PX = b3MulL(manifoldImpulse, normalX);
				let PY = b3MulL(manifoldImpulse, normalY);
				let PZ = b3MulL(manifoldImpulse, normalZ);
				vAX = b3MulSubL(vAX, invMassA, PX);
				vAY = b3MulSubL(vAY, invMassA, PY);
				vAZ = b3MulSubL(vAZ, invMassA, PZ);
				vBX = b3MulAddL(vBX, invMassB, PX);
				vBY = b3MulAddL(vBY, invMassB, PY);
				vBZ = b3MulAddL(vBZ, invMassB, PZ);
			}

			b3ScatterBodiesL(states, c + CW_INDEXA, vAX, vAY, vAZ, wAX, wAY, wAZ);
			b3ScatterBodiesL(states, c + CW_INDEXB, vBX, vBY, vBZ, wBX, wBY, wBZ);
		}

		ci += 1;
		mi += manifoldCount;
	}
}

// Solve the normal constraint, friction, and rolling resistance.
unsafe function b3SolveContactsL<T>(
	block: b3SolverBlock,
	context: b3StepContext,
	constraints: StaticArray<b3ContactConstraintWide>,
	manifolds: StaticArray<b3ContactManifoldWide>,
	constraintBase: i32,
	manifoldStarts: Int32Array,
	fixedManifoldCount: i32
): void {
	let states = b3StatesData(context.states);
	let identity = changetype<usize>(b3_identityStates);
	let constraintData = changetype<usize>(constraints);
	let manifoldData = changetype<usize>(manifolds);
	let invH = context.inv_h;

	let ci = constraintBase + block.startIndex;
	let mi = fixedManifoldCount > 0 ? ci : manifoldStarts[block.startIndex];
	for (let i = 0; i < <i32>block.count; ++i) {
		let c0 = constraintData + <usize>ci * sizeof<b3ContactConstraintWide>();
		let manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : load<i32>(c0 + CW_MANIFOLDCOUNT);
		let anyRolling = b3AllZeroAt(c0 + CW_ROLLINGRESISTANCE) == false;
		for (let lane = 0; lane < b3LaneCountL<T>(); ++lane) {
			let c = c0 + <usize>lane * 4;

			// An empty lane only writes its own constraint lanes, which nothing reads
			if (isFloat<T>() && load<usize>(c + CW_CONTACTS) == 0) {
				continue;
			}

			let pA1 = b3StatePointer(states, identity, load<i32>(c + CW_INDEXA));
			let pA2 = isFloat<T>() ? pA1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXA + 4));
			let pA3 = isFloat<T>() ? pA1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXA + 8));
			let pA4 = isFloat<T>() ? pA1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXA + 12));
			let pB1 = b3StatePointer(states, identity, load<i32>(c + CW_INDEXB));
			let pB2 = isFloat<T>() ? pB1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXB + 4));
			let pB3 = isFloat<T>() ? pB1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXB + 8));
			let pB4 = isFloat<T>() ? pB1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXB + 12));
			let vA = b3GatherRow3L<T>(pA1, pA2, pA3, pA4, BS_V);
			let vAX = vA.c0;
			let vAY = vA.c1;
			let vAZ = vA.c2;
			let wA = b3GatherRow3L<T>(pA1, pA2, pA3, pA4, BS_W);
			let wAX = wA.c0;
			let wAY = wA.c1;
			let wAZ = wA.c2;
			let vB = b3GatherRow3L<T>(pB1, pB2, pB3, pB4, BS_V);
			let vBX = vB.c0;
			let vBY = vB.c1;
			let vBZ = vB.c2;
			let wB = b3GatherRow3L<T>(pB1, pB2, pB3, pB4, BS_W);
			let wBX = wB.c0;
			let wBY = wB.c1;
			let wBZ = wB.c2;

			let invMassA = b3LoadL<T>(c + CW_INVMASSA);
			let invMassB = b3LoadL<T>(c + CW_INVMASSB);
			let invMass = b3AddL(invMassA, invMassB);

			for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
				let cm0 = manifoldData + <usize>(mi + manifoldIndex) * sizeof<b3ContactManifoldWide>();
				let cm = cm0 + <usize>lane * 4;

				let dpA = b3GatherRow3L<T>(pA1, pA2, pA3, pA4, BS_DP);
				let dpAX = dpA.c0;
				let dpAY = dpA.c1;
				let dpAZ = dpA.c2;
				let dqA = b3GatherRowL<T>(pA1, pA2, pA3, pA4, BS_DQ);
				let dqAX = dqA.c0;
				let dqAY = dqA.c1;
				let dqAZ = dqA.c2;
				let dqAS = dqA.c3;
				let dpB = b3GatherRow3L<T>(pB1, pB2, pB3, pB4, BS_DP);
				let dpBX = dpB.c0;
				let dpBY = dpB.c1;
				let dpBZ = dpB.c2;
				let dqB = b3GatherRowL<T>(pB1, pB2, pB3, pB4, BS_DQ);
				let dqBX = dqB.c0;
				let dqBY = dqB.c1;
				let dqBZ = dqB.c2;
				let dqBS = dqB.c3;

				let dpX = b3SubL(dpBX, dpAX);
				let dpY = b3SubL(dpBY, dpAY);
				let dpZ = b3SubL(dpBZ, dpAZ);

				let normalX = b3LoadL<T>(cm + MW_NORMAL + V_X);
				let normalY = b3LoadL<T>(cm + MW_NORMAL + V_Y);
				let normalZ = b3LoadL<T>(cm + MW_NORMAL + V_Z);

				let normalSeparation = b3DotL(normalX, normalY, normalZ, dpX, dpY, dpZ);

				// b3InvRotateVectorW( bA.dq, normal )
				let tAX = b3CrossL(dqAY, dqAZ, normalY, normalZ);
				let tAY = b3CrossL(dqAZ, dqAX, normalZ, normalX);
				let tAZ = b3CrossL(dqAX, dqAY, normalX, normalY);
				tAX = b3AddL(tAX, tAX);
				tAY = b3AddL(tAY, tAY);
				tAZ = b3AddL(tAZ, tAZ);
				let normalAX = b3AddL(b3MulSubL(normalX, dqAS, tAX), b3CrossL(dqAY, dqAZ, tAY, tAZ));
				let normalAY = b3AddL(b3MulSubL(normalY, dqAS, tAY), b3CrossL(dqAZ, dqAX, tAZ, tAX));
				let normalAZ = b3AddL(b3MulSubL(normalZ, dqAS, tAZ), b3CrossL(dqAX, dqAY, tAX, tAY));

				// b3InvRotateVectorW( bB.dq, normal )
				let tBX = b3CrossL(dqBY, dqBZ, normalY, normalZ);
				let tBY = b3CrossL(dqBZ, dqBX, normalZ, normalX);
				let tBZ = b3CrossL(dqBX, dqBY, normalX, normalY);
				tBX = b3AddL(tBX, tBX);
				tBY = b3AddL(tBY, tBY);
				tBZ = b3AddL(tBZ, tBZ);
				let normalBX = b3AddL(b3MulSubL(normalX, dqBS, tBX), b3CrossL(dqBY, dqBZ, tBY, tBZ));
				let normalBY = b3AddL(b3MulSubL(normalY, dqBS, tBY), b3CrossL(dqBZ, dqBX, tBZ, tBX));
				let normalBZ = b3AddL(b3MulSubL(normalZ, dqBS, tBZ), b3CrossL(dqBX, dqBY, tBX, tBY));

				let dvX = b3SubL(vBX, vAX);
				let dvY = b3SubL(vBY, vAY);
				let dvZ = b3SubL(vBZ, vAZ);
				let tangent1X = b3LoadL<T>(cm + MW_TANGENT1 + V_X);
				let tangent1Y = b3LoadL<T>(cm + MW_TANGENT1 + V_Y);
				let tangent1Z = b3LoadL<T>(cm + MW_TANGENT1 + V_Z);
				let tangent2X = b3LoadL<T>(cm + MW_TANGENT2 + V_X);
				let tangent2Y = b3LoadL<T>(cm + MW_TANGENT2 + V_Y);
				let tangent2Z = b3LoadL<T>(cm + MW_TANGENT2 + V_Z);
				let linearNormalVelocity = b3DotL(normalX, normalY, normalZ, dvX, dvY, dvZ);
				let linearTangentVelocity1 = b3SubL(
					b3DotL(tangent1X, tangent1Y, tangent1Z, dvX, dvY, dvZ),
					b3LoadL<T>(cm + MW_TANGENTVELOCITY1)
				);
				let linearTangentVelocity2 = b3SubL(
					b3DotL(tangent2X, tangent2Y, tangent2Z, dvX, dvY, dvZ),
					b3LoadL<T>(cm + MW_TANGENTVELOCITY2)
				);
				let manifoldImpulse = b3SplatL<T>(0.0);

				let totalNormalImpulse = b3SplatL<T>(0.0);
				let totalTwistLimit = b3SplatL<T>(0.0);

				// Normal contraints
				let pointCount = load<i32>(cm0 + MW_POINTCOUNT);
				for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
					let cp = cm + MW_POINTS + <usize>pointIndex * sizeof<b3ContactConstraintPointWide>();

					// Fixed anchor points for applying impulses
					let rAX = b3LoadL<T>(cp + PW_ANCHORA + V_X);
					let rAY = b3LoadL<T>(cp + PW_ANCHORA + V_Y);
					let rAZ = b3LoadL<T>(cp + PW_ANCHORA + V_Z);
					let rBX = b3LoadL<T>(cp + PW_ANCHORB + V_X);
					let rBY = b3LoadL<T>(cp + PW_ANCHORB + V_Y);
					let rBZ = b3LoadL<T>(cp + PW_ANCHORB + V_Z);

					let s = b3AddL(
						b3AddL(
							normalSeparation,
							b3SubL(b3DotL(normalBX, normalBY, normalBZ, rBX, rBY, rBZ), b3DotL(normalAX, normalAY, normalAZ, rAX, rAY, rAZ))
						),
						b3LoadL<T>(cp + PW_BASESEPARATION)
					);

					// Speculative bias, positive if separated and zero if overlapped
					let velocityBias = b3MaxL(b3SplatL<T>(0.0), b3MulL(s, b3SplatL<T>(invH)));

					let uAX = b3CrossL(rAY, rAZ, normalY, normalZ);
					let uAY = b3CrossL(rAZ, rAX, normalZ, normalX);
					let uAZ = b3CrossL(rAX, rAY, normalX, normalY);
					let uBX = b3CrossL(rBY, rBZ, normalY, normalZ);
					let uBY = b3CrossL(rBZ, rBX, normalZ, normalX);
					let uBZ = b3CrossL(rBX, rBY, normalX, normalY);
					let gAX = b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXX), b3LoadL<T>(c + CW_INVIA + S_CXY), b3LoadL<T>(c + CW_INVIA + S_CXZ), uAX, uAY, uAZ);
					let gAY = b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXY), b3LoadL<T>(c + CW_INVIA + S_CYY), b3LoadL<T>(c + CW_INVIA + S_CYZ), uAX, uAY, uAZ);
					let gAZ = b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXZ), b3LoadL<T>(c + CW_INVIA + S_CYZ), b3LoadL<T>(c + CW_INVIA + S_CZZ), uAX, uAY, uAZ);
					let gBX = b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXX), b3LoadL<T>(c + CW_INVIB + S_CXY), b3LoadL<T>(c + CW_INVIB + S_CXZ), uBX, uBY, uBZ);
					let gBY = b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXY), b3LoadL<T>(c + CW_INVIB + S_CYY), b3LoadL<T>(c + CW_INVIB + S_CYZ), uBX, uBY, uBZ);
					let gBZ = b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXZ), b3LoadL<T>(c + CW_INVIB + S_CYZ), b3LoadL<T>(c + CW_INVIB + S_CZZ), uBX, uBY, uBZ);

					let vn = b3AddL(linearNormalVelocity, b3SubL(b3DotL(uBX, uBY, uBZ, wBX, wBY, wBZ), b3DotL(uAX, uAY, uAZ, wAX, wAY, wAZ)));

					let normalImpulse = b3LoadL<T>(cp + PW_NORMALIMPULSE);
					let negImpulse = b3MulL(b3LoadL<T>(cp + PW_NORMALMASS), b3AddL(vn, velocityBias));
					let impulse = b3MaxL(b3NegL(negImpulse), b3NegL(normalImpulse));
					let newImpulse = b3AddL(normalImpulse, impulse);
					b3StoreL<T>(cp + PW_NORMALIMPULSE, newImpulse);
					b3StoreL<T>(cp + PW_TOTALNORMALIMPULSE, b3AddL(b3LoadL<T>(cp + PW_TOTALNORMALIMPULSE), newImpulse));
					totalNormalImpulse = b3AddL(totalNormalImpulse, newImpulse);
					totalTwistLimit = b3AddL(totalTwistLimit, b3MulL(b3LoadL<T>(cp + PW_LEVERARM), newImpulse));

					linearNormalVelocity = b3MulAddL(linearNormalVelocity, invMass, impulse);
					manifoldImpulse = b3AddL(manifoldImpulse, impulse);
					wAX = b3MulSubL(wAX, impulse, gAX);
					wAY = b3MulSubL(wAY, impulse, gAY);
					wAZ = b3MulSubL(wAZ, impulse, gAZ);
					wBX = b3MulAddL(wBX, impulse, gBX);
					wBY = b3MulAddL(wBY, impulse, gBY);
					wBZ = b3MulAddL(wBZ, impulse, gBZ);
				}

				// Rolling resistance. After normal constraints because it uses the normal impulse.
				if (anyRolling) {
					// flip A/B order to negate
					let rm = c + CW_ROLLINGMASS;
					let dwX = b3SubL(wAX, wBX);
					let dwY = b3SubL(wAY, wBY);
					let dwZ = b3SubL(wAZ, wBZ);
					let deltaImpulseX = b3MulRowL(b3LoadL<T>(rm + S_CXX), b3LoadL<T>(rm + S_CXY), b3LoadL<T>(rm + S_CXZ), dwX, dwY, dwZ);
					let deltaImpulseY = b3MulRowL(b3LoadL<T>(rm + S_CXY), b3LoadL<T>(rm + S_CYY), b3LoadL<T>(rm + S_CYZ), dwX, dwY, dwZ);
					let deltaImpulseZ = b3MulRowL(b3LoadL<T>(rm + S_CXZ), b3LoadL<T>(rm + S_CYZ), b3LoadL<T>(rm + S_CZZ), dwX, dwY, dwZ);
					let oldImpulseX = b3LoadL<T>(cm + MW_ROLLINGIMPULSE + V_X);
					let oldImpulseY = b3LoadL<T>(cm + MW_ROLLINGIMPULSE + V_Y);
					let oldImpulseZ = b3LoadL<T>(cm + MW_ROLLINGIMPULSE + V_Z);
					let rollingImpulseX = b3AddL(oldImpulseX, deltaImpulseX);
					let rollingImpulseY = b3AddL(oldImpulseY, deltaImpulseY);
					let rollingImpulseZ = b3AddL(oldImpulseZ, deltaImpulseZ);

					let maxImpulse = b3MulL(b3LoadL<T>(c + CW_ROLLINGRESISTANCE), totalNormalImpulse);
					let lengthSquared = b3DotL(rollingImpulseX, rollingImpulseY, rollingImpulseZ, rollingImpulseX, rollingImpulseY, rollingImpulseZ);

					let mask = b3GreaterThanL(lengthSquared, b3MulAddL(b3SplatL<T>(FLT_EPSILON), maxImpulse, maxImpulse));

					// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism
					let normalize = b3DivL(maxImpulse, b3AddL(b3SqrtL(lengthSquared), b3SplatL<T>(FLT_EPSILON)));
					let scale = b3BlendL(b3SplatL<T>(1.0), normalize, mask);

					// Blend the result to ensure there are no +/-0 determinism problems, since 0 * -1 == -0.
					let rollingMask = b3MaskL(b3LoadL<T>(cm + MW_ROLLINGMASK));
					rollingImpulseX = b3BlendL(b3SplatL<T>(0.0), b3MulL(scale, rollingImpulseX), rollingMask);
					rollingImpulseY = b3BlendL(b3SplatL<T>(0.0), b3MulL(scale, rollingImpulseY), rollingMask);
					rollingImpulseZ = b3BlendL(b3SplatL<T>(0.0), b3MulL(scale, rollingImpulseZ), rollingMask);
					b3StoreL<T>(cm + MW_ROLLINGIMPULSE + V_X, rollingImpulseX);
					b3StoreL<T>(cm + MW_ROLLINGIMPULSE + V_Y, rollingImpulseY);
					b3StoreL<T>(cm + MW_ROLLINGIMPULSE + V_Z, rollingImpulseZ);

					deltaImpulseX = b3SubL(rollingImpulseX, oldImpulseX);
					deltaImpulseY = b3SubL(rollingImpulseY, oldImpulseY);
					deltaImpulseZ = b3SubL(rollingImpulseZ, oldImpulseZ);

					wAX = b3SubL(wAX, b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXX), b3LoadL<T>(c + CW_INVIA + S_CXY), b3LoadL<T>(c + CW_INVIA + S_CXZ), deltaImpulseX, deltaImpulseY, deltaImpulseZ));
					wAY = b3SubL(wAY, b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXY), b3LoadL<T>(c + CW_INVIA + S_CYY), b3LoadL<T>(c + CW_INVIA + S_CYZ), deltaImpulseX, deltaImpulseY, deltaImpulseZ));
					wAZ = b3SubL(wAZ, b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXZ), b3LoadL<T>(c + CW_INVIA + S_CYZ), b3LoadL<T>(c + CW_INVIA + S_CZZ), deltaImpulseX, deltaImpulseY, deltaImpulseZ));
					wBX = b3AddL(wBX, b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXX), b3LoadL<T>(c + CW_INVIB + S_CXY), b3LoadL<T>(c + CW_INVIB + S_CXZ), deltaImpulseX, deltaImpulseY, deltaImpulseZ));
					wBY = b3AddL(wBY, b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXY), b3LoadL<T>(c + CW_INVIB + S_CYY), b3LoadL<T>(c + CW_INVIB + S_CYZ), deltaImpulseX, deltaImpulseY, deltaImpulseZ));
					wBZ = b3AddL(wBZ, b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXZ), b3LoadL<T>(c + CW_INVIB + S_CYZ), b3LoadL<T>(c + CW_INVIB + S_CZZ), deltaImpulseX, deltaImpulseY, deltaImpulseZ));
				}

				let friction = b3LoadL<T>(c + CW_FRICTION);

				// Central twist friction. Friction goes last to improve stacking stability.
				{
					let twistSpeed = b3DotL(normalX, normalY, normalZ, b3SubL(wBX, wAX), b3SubL(wBY, wAY), b3SubL(wBZ, wAZ));
					let maxLambda = b3MulL(friction, totalTwistLimit);
					let oldImpulse = b3LoadL<T>(cm + MW_TWISTIMPULSE);
					let impulse = b3NegL(b3MulL(b3LoadL<T>(cm + MW_TWISTMASS), twistSpeed));
					impulse = b3MaxL(b3SubL(b3NegL(maxLambda), oldImpulse), b3MinL(impulse, b3SubL(maxLambda, oldImpulse)));
					b3StoreL<T>(cm + MW_TWISTIMPULSE, b3AddL(oldImpulse, impulse));

					wAX = b3MulSubL(wAX, impulse, b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXX), b3LoadL<T>(c + CW_INVIA + S_CXY), b3LoadL<T>(c + CW_INVIA + S_CXZ), normalX, normalY, normalZ));
					wAY = b3MulSubL(wAY, impulse, b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXY), b3LoadL<T>(c + CW_INVIA + S_CYY), b3LoadL<T>(c + CW_INVIA + S_CYZ), normalX, normalY, normalZ));
					wAZ = b3MulSubL(wAZ, impulse, b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXZ), b3LoadL<T>(c + CW_INVIA + S_CYZ), b3LoadL<T>(c + CW_INVIA + S_CZZ), normalX, normalY, normalZ));
					wBX = b3MulAddL(wBX, impulse, b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXX), b3LoadL<T>(c + CW_INVIB + S_CXY), b3LoadL<T>(c + CW_INVIB + S_CXZ), normalX, normalY, normalZ));
					wBY = b3MulAddL(wBY, impulse, b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXY), b3LoadL<T>(c + CW_INVIB + S_CYY), b3LoadL<T>(c + CW_INVIB + S_CYZ), normalX, normalY, normalZ));
					wBZ = b3MulAddL(wBZ, impulse, b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXZ), b3LoadL<T>(c + CW_INVIB + S_CYZ), b3LoadL<T>(c + CW_INVIB + S_CZZ), normalX, normalY, normalZ));
				}

				// Central friction. Friction goes last to improve stacking stability.
				{
					// Fixed anchor points for applying impulses
					let rAX = b3LoadL<T>(cm + MW_CENTERA + V_X);
					let rAY = b3LoadL<T>(cm + MW_CENTERA + V_Y);
					let rAZ = b3LoadL<T>(cm + MW_CENTERA + V_Z);
					let rBX = b3LoadL<T>(cm + MW_CENTERB + V_X);
					let rBY = b3LoadL<T>(cm + MW_CENTERB + V_Y);
					let rBZ = b3LoadL<T>(cm + MW_CENTERB + V_Z);

					let angularRelativeVelocityX = b3SubL(b3CrossL(wBY, wBZ, rBY, rBZ), b3CrossL(wAY, wAZ, rAY, rAZ));
					let angularRelativeVelocityY = b3SubL(b3CrossL(wBZ, wBX, rBZ, rBX), b3CrossL(wAZ, wAX, rAZ, rAX));
					let angularRelativeVelocityZ = b3SubL(b3CrossL(wBX, wBY, rBX, rBY), b3CrossL(wAX, wAY, rAX, rAY));

					// Relative tangent velocity at contact
					let vtx = b3AddL(
						linearTangentVelocity1,
						b3DotL(angularRelativeVelocityX, angularRelativeVelocityY, angularRelativeVelocityZ, tangent1X, tangent1Y, tangent1Z)
					);
					let vty = b3AddL(
						linearTangentVelocity2,
						b3DotL(angularRelativeVelocityX, angularRelativeVelocityY, angularRelativeVelocityZ, tangent2X, tangent2Y, tangent2Z)
					);

					// Incremental tangent impulse
					let tm = cm + MW_TANGENTMASS;
					let tmxx = b3LoadL<T>(tm + S2_CXX);
					let tmxy = b3LoadL<T>(tm + S2_CXY);
					let tmyy = b3LoadL<T>(tm + S2_CYY);
					let deltaImpulseX = b3NegL(b3AddL(b3MulL(tmxx, vtx), b3MulL(tmxy, vty)));
					let deltaImpulseY = b3NegL(b3AddL(b3MulL(tmxy, vtx), b3MulL(tmyy, vty)));
					let frictionImpulseX = b3LoadL<T>(cm + MW_FRICTIONIMPULSE + V2_X);
					let frictionImpulseY = b3LoadL<T>(cm + MW_FRICTIONIMPULSE + V2_Y);
					let newImpulseX = b3AddL(frictionImpulseX, deltaImpulseX);
					let newImpulseY = b3AddL(frictionImpulseY, deltaImpulseY);

					let maxImpulse = b3MulL(friction, totalNormalImpulse);

					// Clamp the accumulated impulse
					let lengthSquared = b3AddL(b3MulL(newImpulseX, newImpulseX), b3MulL(newImpulseY, newImpulseY));

					// Max impulse can be zero
					let mask = b3GreaterThanL(lengthSquared, b3MulL(maxImpulse, maxImpulse));

					// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism. Add epsilon to avoid divide by
					// zero.
					let normalize = b3DivL(maxImpulse, b3AddL(b3SqrtL(lengthSquared), b3SplatL<T>(FLT_EPSILON)));
					let scale = b3BlendL(b3SplatL<T>(1.0), normalize, mask);
					newImpulseX = b3MulL(scale, newImpulseX);
					newImpulseY = b3MulL(scale, newImpulseY);

					deltaImpulseX = b3SubL(newImpulseX, frictionImpulseX);
					deltaImpulseY = b3SubL(newImpulseY, frictionImpulseY);

					b3StoreL<T>(cm + MW_FRICTIONIMPULSE + V2_X, newImpulseX);
					b3StoreL<T>(cm + MW_FRICTIONIMPULSE + V2_Y, newImpulseY);

					// Apply delta impulse
					let PX = b3AddL(b3MulL(deltaImpulseX, tangent1X), b3MulL(deltaImpulseY, tangent2X));
					let PY = b3AddL(b3MulL(deltaImpulseX, tangent1Y), b3MulL(deltaImpulseY, tangent2Y));
					let PZ = b3AddL(b3MulL(deltaImpulseX, tangent1Z), b3MulL(deltaImpulseY, tangent2Z));
					let rPAX = b3CrossL(rAY, rAZ, PY, PZ);
					let rPAY = b3CrossL(rAZ, rAX, PZ, PX);
					let rPAZ = b3CrossL(rAX, rAY, PX, PY);
					let rPBX = b3CrossL(rBY, rBZ, PY, PZ);
					let rPBY = b3CrossL(rBZ, rBX, PZ, PX);
					let rPBZ = b3CrossL(rBX, rBY, PX, PY);
					wAX = b3SubL(wAX, b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXX), b3LoadL<T>(c + CW_INVIA + S_CXY), b3LoadL<T>(c + CW_INVIA + S_CXZ), rPAX, rPAY, rPAZ));
					wAY = b3SubL(wAY, b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXY), b3LoadL<T>(c + CW_INVIA + S_CYY), b3LoadL<T>(c + CW_INVIA + S_CYZ), rPAX, rPAY, rPAZ));
					wAZ = b3SubL(wAZ, b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXZ), b3LoadL<T>(c + CW_INVIA + S_CYZ), b3LoadL<T>(c + CW_INVIA + S_CZZ), rPAX, rPAY, rPAZ));
					wBX = b3AddL(wBX, b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXX), b3LoadL<T>(c + CW_INVIB + S_CXY), b3LoadL<T>(c + CW_INVIB + S_CXZ), rPBX, rPBY, rPBZ));
					wBY = b3AddL(wBY, b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXY), b3LoadL<T>(c + CW_INVIB + S_CYY), b3LoadL<T>(c + CW_INVIB + S_CYZ), rPBX, rPBY, rPBZ));
					wBZ = b3AddL(wBZ, b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXZ), b3LoadL<T>(c + CW_INVIB + S_CYZ), b3LoadL<T>(c + CW_INVIB + S_CZZ), rPBX, rPBY, rPBZ));

					PX = b3MulAddL(PX, manifoldImpulse, normalX);
					PY = b3MulAddL(PY, manifoldImpulse, normalY);
					PZ = b3MulAddL(PZ, manifoldImpulse, normalZ);
					vAX = b3MulSubL(vAX, invMassA, PX);
					vAY = b3MulSubL(vAY, invMassA, PY);
					vAZ = b3MulSubL(vAZ, invMassA, PZ);
					vBX = b3MulAddL(vBX, invMassB, PX);
					vBY = b3MulAddL(vBY, invMassB, PY);
					vBZ = b3MulAddL(vBZ, invMassB, PZ);
				}
			}

			b3ScatterBodiesL(states, c + CW_INDEXA, vAX, vAY, vAZ, wAX, wAY, wAZ);
			b3ScatterBodiesL(states, c + CW_INDEXB, vBX, vBY, vBZ, wBX, wBY, wBZ);
		}

		ci += 1;
		mi += manifoldCount;
	}
}

unsafe function b3ApplyRestitutionL<T>(
	block: b3SolverBlock,
	context: b3StepContext,
	constraints: StaticArray<b3ContactConstraintWide>,
	manifolds: StaticArray<b3ContactManifoldWide>,
	constraintBase: i32,
	manifoldStarts: Int32Array,
	fixedManifoldCount: i32
): void {
	let states = b3StatesData(context.states);
	let identity = changetype<usize>(b3_identityStates);
	let constraintData = changetype<usize>(constraints);
	let manifoldData = changetype<usize>(manifolds);
	let invH = context.inv_h;
	let negRestitutionThreshold = b3SplatL<T>(-context.world.restitutionThreshold);
	let propagate = context.world.enableRestitutionPropagation;

	let nextCi = constraintBase + block.startIndex;
	let nextMi = fixedManifoldCount > 0 ? nextCi : manifoldStarts[block.startIndex];
	for (let i = 0; i < <i32>block.count; ++i) {
		let ci = nextCi;
		let mi = nextMi;
		let c0 = constraintData + <usize>ci * sizeof<b3ContactConstraintWide>();
		let manifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : load<i32>(c0 + CW_MANIFOLDCOUNT);
		nextCi = ci + 1;
		nextMi = mi + manifoldCount;

		if (propagate == false && b3AllZeroAt(c0 + CW_RESTITUTION)) {
			continue;
		}

		for (let lane = 0; lane < b3LaneCountL<T>(); ++lane) {
			let c = c0 + <usize>lane * 4;

			// An empty lane only writes its own constraint lanes, which nothing reads
			if (isFloat<T>() && load<usize>(c + CW_CONTACTS) == 0) {
				continue;
			}

			let pA1 = b3StatePointer(states, identity, load<i32>(c + CW_INDEXA));
			let pA2 = isFloat<T>() ? pA1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXA + 4));
			let pA3 = isFloat<T>() ? pA1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXA + 8));
			let pA4 = isFloat<T>() ? pA1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXA + 12));
			let pB1 = b3StatePointer(states, identity, load<i32>(c + CW_INDEXB));
			let pB2 = isFloat<T>() ? pB1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXB + 4));
			let pB3 = isFloat<T>() ? pB1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXB + 8));
			let pB4 = isFloat<T>() ? pB1 : b3StatePointer(states, identity, load<i32>(c + CW_INDEXB + 12));
			let vA = b3GatherRow3L<T>(pA1, pA2, pA3, pA4, BS_V);
			let vAX = vA.c0;
			let vAY = vA.c1;
			let vAZ = vA.c2;
			let wA = b3GatherRow3L<T>(pA1, pA2, pA3, pA4, BS_W);
			let wAX = wA.c0;
			let wAY = wA.c1;
			let wAZ = wA.c2;
			let vB = b3GatherRow3L<T>(pB1, pB2, pB3, pB4, BS_V);
			let vBX = vB.c0;
			let vBY = vB.c1;
			let vBZ = vB.c2;
			let wB = b3GatherRow3L<T>(pB1, pB2, pB3, pB4, BS_W);
			let wBX = wB.c0;
			let wBY = wB.c1;
			let wBZ = wB.c2;

			let restitution = b3LoadL<T>(c + CW_RESTITUTION);
			let restitutionMask = b3GreaterThanL(restitution, b3SplatL<T>(0.0));
			let invMassA = b3LoadL<T>(c + CW_INVMASSA);
			let invMassB = b3LoadL<T>(c + CW_INVMASSB);

			for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
				let cm0 = manifoldData + <usize>(mi + manifoldIndex) * sizeof<b3ContactManifoldWide>();
				let cm = cm0 + <usize>lane * 4;

				let dpA = b3GatherRow3L<T>(pA1, pA2, pA3, pA4, BS_DP);
				let dpAX = dpA.c0;
				let dpAY = dpA.c1;
				let dpAZ = dpA.c2;
				let dqA = b3GatherRowL<T>(pA1, pA2, pA3, pA4, BS_DQ);
				let dqAX = dqA.c0;
				let dqAY = dqA.c1;
				let dqAZ = dqA.c2;
				let dqAS = dqA.c3;
				let dpB = b3GatherRow3L<T>(pB1, pB2, pB3, pB4, BS_DP);
				let dpBX = dpB.c0;
				let dpBY = dpB.c1;
				let dpBZ = dpB.c2;
				let dqB = b3GatherRowL<T>(pB1, pB2, pB3, pB4, BS_DQ);
				let dqBX = dqB.c0;
				let dqBY = dqB.c1;
				let dqBZ = dqB.c2;
				let dqBS = dqB.c3;

				let dpX = b3SubL(dpBX, dpAX);
				let dpY = b3SubL(dpBY, dpAY);
				let dpZ = b3SubL(dpBZ, dpAZ);

				let normalX = b3LoadL<T>(cm + MW_NORMAL + V_X);
				let normalY = b3LoadL<T>(cm + MW_NORMAL + V_Y);
				let normalZ = b3LoadL<T>(cm + MW_NORMAL + V_Z);

				let normalSeparation = b3DotL(normalX, normalY, normalZ, dpX, dpY, dpZ);

				// b3InvRotateVectorW( bA.dq, normal )
				let tAX = b3CrossL(dqAY, dqAZ, normalY, normalZ);
				let tAY = b3CrossL(dqAZ, dqAX, normalZ, normalX);
				let tAZ = b3CrossL(dqAX, dqAY, normalX, normalY);
				tAX = b3AddL(tAX, tAX);
				tAY = b3AddL(tAY, tAY);
				tAZ = b3AddL(tAZ, tAZ);
				let normalAX = b3AddL(b3MulSubL(normalX, dqAS, tAX), b3CrossL(dqAY, dqAZ, tAY, tAZ));
				let normalAY = b3AddL(b3MulSubL(normalY, dqAS, tAY), b3CrossL(dqAZ, dqAX, tAZ, tAX));
				let normalAZ = b3AddL(b3MulSubL(normalZ, dqAS, tAZ), b3CrossL(dqAX, dqAY, tAX, tAY));

				// b3InvRotateVectorW( bB.dq, normal )
				let tBX = b3CrossL(dqBY, dqBZ, normalY, normalZ);
				let tBY = b3CrossL(dqBZ, dqBX, normalZ, normalX);
				let tBZ = b3CrossL(dqBX, dqBY, normalX, normalY);
				tBX = b3AddL(tBX, tBX);
				tBY = b3AddL(tBY, tBY);
				tBZ = b3AddL(tBZ, tBZ);
				let normalBX = b3AddL(b3MulSubL(normalX, dqBS, tBX), b3CrossL(dqBY, dqBZ, tBY, tBZ));
				let normalBY = b3AddL(b3MulSubL(normalY, dqBS, tBY), b3CrossL(dqBZ, dqBX, tBZ, tBX));
				let normalBZ = b3AddL(b3MulSubL(normalZ, dqBS, tBZ), b3CrossL(dqBX, dqBY, tBX, tBY));

				// Read the point count rather than put on stack to avoid register spills.
				let pointCount = load<i32>(cm0 + MW_POINTCOUNT);
				for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
					let cp = cm + MW_POINTS + <usize>pointIndex * sizeof<b3ContactConstraintPointWide>();

					let rAX = b3LoadL<T>(cp + PW_ANCHORA + V_X);
					let rAY = b3LoadL<T>(cp + PW_ANCHORA + V_Y);
					let rAZ = b3LoadL<T>(cp + PW_ANCHORA + V_Z);
					let rBX = b3LoadL<T>(cp + PW_ANCHORB + V_X);
					let rBY = b3LoadL<T>(cp + PW_ANCHORB + V_Y);
					let rBZ = b3LoadL<T>(cp + PW_ANCHORB + V_Z);

					let s = b3AddL(
						b3AddL(
							normalSeparation,
							b3SubL(b3DotL(normalBX, normalBY, normalBZ, rBX, rBY, rBZ), b3DotL(normalAX, normalAY, normalAZ, rAX, rAY, rAZ))
						),
						b3LoadL<T>(cp + PW_BASESEPARATION)
					);

					let normalMasses = b3LoadL<T>(cp + PW_NORMALMASS);
					let normalMass = propagate ? normalMasses : b3BlendL(b3SplatL<T>(0.0), normalMasses, restitutionMask);

					let normalImpulses = b3LoadL<T>(cp + PW_NORMALIMPULSE);
					let totalNormalImpulses = b3LoadL<T>(cp + PW_TOTALNORMALIMPULSE);
					let restitutionImpulses = b3LoadL<T>(cp + PW_RESTITUTIONIMPULSE);
					let relativeVelocities = b3LoadL<T>(cp + PW_RELATIVEVELOCITY);
					let compressionImpulse = b3SubL(totalNormalImpulses, restitutionImpulses);
					let armed = b3AndL(
						b3AndL(restitutionMask, b3LessThanL(relativeVelocities, negRestitutionThreshold)),
						b3GreaterThanL(compressionImpulse, b3SplatL<T>(0.0))
					);

					let specBias = b3MaxL(b3SplatL<T>(0.0), b3MulL(s, b3SplatL<T>(invH)));
					let velocityBias = b3BlendL(specBias, b3MulL(restitution, relativeVelocities), armed);

					let vrAX = b3AddL(vAX, b3CrossL(wAY, wAZ, rAY, rAZ));
					let vrAY = b3AddL(vAY, b3CrossL(wAZ, wAX, rAZ, rAX));
					let vrAZ = b3AddL(vAZ, b3CrossL(wAX, wAY, rAX, rAY));
					let vrBX = b3AddL(vBX, b3CrossL(wBY, wBZ, rBY, rBZ));
					let vrBY = b3AddL(vBY, b3CrossL(wBZ, wBX, rBZ, rBX));
					let vrBZ = b3AddL(vBZ, b3CrossL(wBX, wBY, rBX, rBY));
					let vn = b3DotL(b3SubL(vrBX, vrAX), b3SubL(vrBY, vrAY), b3SubL(vrBZ, vrAZ), normalX, normalY, normalZ);

					let negImpulse = b3MulL(normalMass, b3AddL(vn, velocityBias));

					let newImpulse = b3MaxL(b3SubL(normalImpulses, negImpulse), b3SplatL<T>(0.0));
					let impulse = b3SubL(newImpulse, normalImpulses);

					let approachImpulse = b3MinL(b3MaxL(b3NegL(b3MulL(normalMass, vn)), b3SplatL<T>(0.0)), b3MaxL(impulse, b3SplatL<T>(0.0)));
					let allowance = b3SubL(b3MulL(restitution, b3AddL(compressionImpulse, approachImpulse)), restitutionImpulses);
					let maxImpulse = b3AddL(approachImpulse, b3MaxL(allowance, b3SplatL<T>(0.0)));
					impulse = b3BlendL(impulse, b3MinL(impulse, maxImpulse), armed);

					b3StoreL<T>(cp + PW_NORMALIMPULSE, b3AddL(normalImpulses, impulse));
					b3StoreL<T>(cp + PW_RESTITUTIONIMPULSE, b3AddL(restitutionImpulses, b3SubL(impulse, approachImpulse)));
					b3StoreL<T>(cp + PW_TOTALNORMALIMPULSE, b3AddL(totalNormalImpulses, impulse));

					let PX = b3MulL(impulse, normalX);
					let PY = b3MulL(impulse, normalY);
					let PZ = b3MulL(impulse, normalZ);
					let rPAX = b3CrossL(rAY, rAZ, PY, PZ);
					let rPAY = b3CrossL(rAZ, rAX, PZ, PX);
					let rPAZ = b3CrossL(rAX, rAY, PX, PY);
					let rPBX = b3CrossL(rBY, rBZ, PY, PZ);
					let rPBY = b3CrossL(rBZ, rBX, PZ, PX);
					let rPBZ = b3CrossL(rBX, rBY, PX, PY);
					wAX = b3SubL(wAX, b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXX), b3LoadL<T>(c + CW_INVIA + S_CXY), b3LoadL<T>(c + CW_INVIA + S_CXZ), rPAX, rPAY, rPAZ));
					wAY = b3SubL(wAY, b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXY), b3LoadL<T>(c + CW_INVIA + S_CYY), b3LoadL<T>(c + CW_INVIA + S_CYZ), rPAX, rPAY, rPAZ));
					wAZ = b3SubL(wAZ, b3MulRowL(b3LoadL<T>(c + CW_INVIA + S_CXZ), b3LoadL<T>(c + CW_INVIA + S_CYZ), b3LoadL<T>(c + CW_INVIA + S_CZZ), rPAX, rPAY, rPAZ));
					vAX = b3MulSubL(vAX, invMassA, PX);
					vAY = b3MulSubL(vAY, invMassA, PY);
					vAZ = b3MulSubL(vAZ, invMassA, PZ);
					wBX = b3AddL(wBX, b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXX), b3LoadL<T>(c + CW_INVIB + S_CXY), b3LoadL<T>(c + CW_INVIB + S_CXZ), rPBX, rPBY, rPBZ));
					wBY = b3AddL(wBY, b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXY), b3LoadL<T>(c + CW_INVIB + S_CYY), b3LoadL<T>(c + CW_INVIB + S_CYZ), rPBX, rPBY, rPBZ));
					wBZ = b3AddL(wBZ, b3MulRowL(b3LoadL<T>(c + CW_INVIB + S_CXZ), b3LoadL<T>(c + CW_INVIB + S_CYZ), b3LoadL<T>(c + CW_INVIB + S_CZZ), rPBX, rPBY, rPBZ));
					vBX = b3MulAddL(vBX, invMassB, PX);
					vBY = b3MulAddL(vBY, invMassB, PY);
					vBZ = b3MulAddL(vBZ, invMassB, PZ);
				}
			}

			b3ScatterBodiesL(states, c + CW_INDEXA, vAX, vAY, vAZ, wAX, wAY, wAZ);
			b3ScatterBodiesL(states, c + CW_INDEXB, vBX, vBY, vBZ, wBX, wBY, wBZ);
		}
	}
}

// Store impulses by contact constraint. C reads each lane through a float pointer; so do these raw loads, in range by
// construction (lane < 4, manifold and point counts from the contact). The same for both lane types.
// One lane of C's store impulses loop, from the lane's contact and its manifold data. Returns whether it set a hit event.
@inline unsafe function b3StoreLaneImpulses(
	manifolds: usize,
	lane: i32,
	contact: usize,
	laneManifolds: usize,
	fixedManifoldCount: i32,
	manifoldCount: i32,
	hitEventBitSet: b3BitSet,
	negHitThreshold: f32
): bool {
	if (contact == 0) {
		return false;
	}

	let contactManifoldCount = load<i32>(contact + CT_MANIFOLDCOUNT);
	let laneManifoldCount = fixedManifoldCount > 0 ? fixedManifoldCount : contactManifoldCount;
	B3_ASSERT(laneManifoldCount == contactManifoldCount && laneManifoldCount <= manifoldCount);

	let laneOffset = <usize>lane * 4;
	for (let manifoldIndex = 0; manifoldIndex < laneManifoldCount; ++manifoldIndex) {
		let cm = manifolds + <usize>manifoldIndex * sizeof<b3ContactManifoldWide>() + laneOffset;
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

	if ((load<u32>(contact + CT_FLAGS) & b3_simEnableHitEvent) != 0) {
		for (let manifoldIndex = 0; manifoldIndex < laneManifoldCount; ++manifoldIndex) {
			let m = laneManifolds + <usize>manifoldIndex * sizeof<b3Manifold>();
			let pointCount = load<i32>(m + M_POINTCOUNT);
			for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
				let mp = m + M_POINTS + <usize>pointIndex * sizeof<b3ManifoldPoint>();

				// Need to check total impulse because the point may be speculative and not colliding
				if (load<f32>(mp + MP_NORMALVELOCITY) < negHitThreshold && load<f32>(mp + MP_TOTALNORMALIMPULSE) > 0.0) {
					b3SetBit(hitEventBitSet, <u32>load<i32>(contact + CT_CONTACTID));
					return true;
				}
			}
		}
	}
	return false;
}

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
		let cm = manifoldData + <usize>mi * sizeof<b3ContactManifoldWide>();

		// Pay the cache misses up front. The lanes run unrolled, as C's do, so their contacts stay in locals.
		let contact0 = load<usize>(c + CW_CONTACTS);
		let contact1 = load<usize>(c + CW_CONTACTS + 4);
		let contact2 = load<usize>(c + CW_CONTACTS + 8);
		let contact3 = load<usize>(c + CW_CONTACTS + 12);
		let manifolds0 = contact0 != 0 ? load<usize>(contact0 + CT_MANIFOLDS) : 0;
		let manifolds1 = contact1 != 0 ? load<usize>(contact1 + CT_MANIFOLDS) : 0;
		let manifolds2 = contact2 != 0 ? load<usize>(contact2 + CT_MANIFOLDS) : 0;
		let manifolds3 = contact3 != 0 ? load<usize>(contact3 + CT_MANIFOLDS) : 0;

		let hit0 = b3StoreLaneImpulses(cm, 0, contact0, manifolds0, fixedManifoldCount, manifoldCount, hitEventBitSet, negHitThreshold);
		let hit1 = b3StoreLaneImpulses(cm, 1, contact1, manifolds1, fixedManifoldCount, manifoldCount, hitEventBitSet, negHitThreshold);
		let hit2 = b3StoreLaneImpulses(cm, 2, contact2, manifolds2, fixedManifoldCount, manifoldCount, hitEventBitSet, negHitThreshold);
		let hit3 = b3StoreLaneImpulses(cm, 3, contact3, manifolds3, fixedManifoldCount, manifoldCount, hitEventBitSet, negHitThreshold);
		hasHitEvents = hasHitEvents || hit0 || hit1 || hit2 || hit3;

		ci += 1;
		mi += manifoldCount;
	}

	taskContext.hasHitEvents = hasHitEvents;
}

// These wrappers allow the implementation be shared between convex and mesh contacts.

export function b3WarmStartContacts_ConvexW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	if (ASC_FEATURE_SIMD) {
		b3WarmStartContactsL<v128>(
			block,
			context,
			color.wideConstraints,
			color.wideManifolds,
			color.wideConstraintStart,
			b3_noManifoldStarts,
			1
		);
	} else {
		b3WarmStartContactsL<f32>(
			block,
			context,
			color.wideConstraints,
			color.wideManifolds,
			color.wideConstraintStart,
			b3_noManifoldStarts,
			1
		);
	}
}

export function b3PushContacts_ConvexW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	if (ASC_FEATURE_SIMD) {
		b3PushContactsL<v128>(
			block,
			context,
			color.wideConstraints,
			color.wideManifolds,
			color.wideConstraintStart,
			b3_noManifoldStarts,
			1
		);
	} else {
		b3PushContactsL<f32>(
			block,
			context,
			color.wideConstraints,
			color.wideManifolds,
			color.wideConstraintStart,
			b3_noManifoldStarts,
			1
		);
	}
}

export function b3SolveContacts_ConvexW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	if (ASC_FEATURE_SIMD) {
		b3SolveContactsL<v128>(
			block,
			context,
			color.wideConstraints,
			color.wideManifolds,
			color.wideConstraintStart,
			b3_noManifoldStarts,
			1
		);
	} else {
		b3SolveContactsL<f32>(
			block,
			context,
			color.wideConstraints,
			color.wideManifolds,
			color.wideConstraintStart,
			b3_noManifoldStarts,
			1
		);
	}
}

export function b3ApplyRestitution_ConvexW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	if (ASC_FEATURE_SIMD) {
		b3ApplyRestitutionL<v128>(
			block,
			context,
			color.wideConstraints,
			color.wideManifolds,
			color.wideConstraintStart,
			b3_noManifoldStarts,
			1
		);
	} else {
		b3ApplyRestitutionL<f32>(
			block,
			context,
			color.wideConstraints,
			color.wideManifolds,
			color.wideConstraintStart,
			b3_noManifoldStarts,
			1
		);
	}
}

export function b3StoreImpulses_ConvexW4(block: b3SolverBlock, context: b3StepContext, workerIndex: i32): void {
	b3StoreImpulsesW(block, context, workerIndex, context.wideConstraints, context.wideManifolds, 0, b3_noManifoldStarts, 1);
}

export function b3WarmStartContacts_MeshWideW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	if (ASC_FEATURE_SIMD) {
		b3WarmStartContactsL<v128>(
			block,
			context,
			color.wideMeshConstraints,
			color.wideMeshManifolds,
			color.wideMeshConstraintStart,
			color.wideMeshManifoldStarts,
			0
		);
	} else {
		b3WarmStartContactsL<f32>(
			block,
			context,
			color.wideMeshConstraints,
			color.wideMeshManifolds,
			color.wideMeshConstraintStart,
			color.wideMeshManifoldStarts,
			0
		);
	}
}

export function b3PushContacts_MeshWideW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	if (ASC_FEATURE_SIMD) {
		b3PushContactsL<v128>(
			block,
			context,
			color.wideMeshConstraints,
			color.wideMeshManifolds,
			color.wideMeshConstraintStart,
			color.wideMeshManifoldStarts,
			0
		);
	} else {
		b3PushContactsL<f32>(
			block,
			context,
			color.wideMeshConstraints,
			color.wideMeshManifolds,
			color.wideMeshConstraintStart,
			color.wideMeshManifoldStarts,
			0
		);
	}
}

export function b3SolveContacts_MeshWideW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	if (ASC_FEATURE_SIMD) {
		b3SolveContactsL<v128>(
			block,
			context,
			color.wideMeshConstraints,
			color.wideMeshManifolds,
			color.wideMeshConstraintStart,
			color.wideMeshManifoldStarts,
			0
		);
	} else {
		b3SolveContactsL<f32>(
			block,
			context,
			color.wideMeshConstraints,
			color.wideMeshManifolds,
			color.wideMeshConstraintStart,
			color.wideMeshManifoldStarts,
			0
		);
	}
}

export function b3ApplyRestitution_MeshWideW4(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	if (ASC_FEATURE_SIMD) {
		b3ApplyRestitutionL<v128>(
			block,
			context,
			color.wideMeshConstraints,
			color.wideMeshManifolds,
			color.wideMeshConstraintStart,
			color.wideMeshManifoldStarts,
			0
		);
	} else {
		b3ApplyRestitutionL<f32>(
			block,
			context,
			color.wideMeshConstraints,
			color.wideMeshManifolds,
			color.wideMeshConstraintStart,
			color.wideMeshManifoldStarts,
			0
		);
	}
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
