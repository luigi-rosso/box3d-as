// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/contact_solver.c
// The width 4 constraint types of src/contact_solver_wide.inl live here too, because the graph
// colors and the step context hold them. Its functions go to the wide4 port.

import { b3Matrix3, b3Vec2, b3Vec3 } from "./math_functions";
import { b3Matrix2 } from "./math_internal";
import { b3FloatW4, b3Vec3W4 } from "./simd";
import { b3Softness } from "./solver";

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
