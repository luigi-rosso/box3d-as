// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of include/box3d/math_functions.h and src/math_functions.c

import { B3_ASSERT, B3_ASSERT_ENABLED, FLT_EPSILON, FLT_MIN } from "./core";
import { B3_HUGE } from "./constants";
import { b3MakeDiagonalMatrix, b3TrianglePoint } from "./math_internal";
import {
	b3RayCastInput,
	b3_featureEdge1,
	b3_featureEdge2,
	b3_featureEdge3,
	b3_featureTriangleFace,
	b3_featureVertex1,
	b3_featureVertex2,
	b3_featureVertex3,
} from "./collision";

/// https://en.wikipedia.org/wiki/Pi
export const B3_PI: f32 = 3.14159265359;

/// Convenience macro to convert from degrees to radians.
export const B3_DEG_TO_RAD: f32 = 0.01745329251;

/// Convenience macro to convert from radians to degrees.
export const B3_RAD_TO_DEG: f32 = 57.2957795131;

/// Minimum scale used for scaling collision meshes, etc.
export const B3_MIN_SCALE: f32 = 0.01;

/// A 2D vector.
export struct b3Vec2 {
	x: f32 = 0;
	y: f32 = 0;

	constructor(x: f32, y: f32) {
		this.x = x;
		this.y = y;
	}
}

/// A 3D vector.
export struct b3Vec3 {
	x: f32 = 0;
	y: f32 = 0;
	z: f32 = 0;

	constructor(x: f32, y: f32, z: f32) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
}

/// Cosine and sine pair.
/// This uses a custom implementation designed for cross-platform determinism.
export struct b3CosSin {
	/// cosine and sine
	cosine: f32 = 0;
	sine: f32 = 0;

	constructor(cosine: f32, sine: f32) {
		this.cosine = cosine;
		this.sine = sine;
	}
}

/// A quaternion.
export struct b3Quat {
	v: b3Vec3 = b3Vec3();
	s: f32 = 0;

	constructor(v: b3Vec3, s: f32) {
		this.v = v;
		this.s = s;
	}
}

/// A rigid transform.
export struct b3Transform {
	p: b3Vec3 = b3Vec3();
	q: b3Quat = b3Quat();

	constructor(p: b3Vec3, q: b3Quat) {
		this.p = p;
		this.q = q;
	}
}

/// In single precision mode these types are the same.
export type b3Pos = b3Vec3;

/// In single precision mode these types are the same.
export type b3WorldTransform = b3Transform;

/// A 3x3 matrix.
export struct b3Matrix3 {
	cx: b3Vec3 = b3Vec3();
	cy: b3Vec3 = b3Vec3();
	cz: b3Vec3 = b3Vec3();

	constructor(cx: b3Vec3, cy: b3Vec3, cz: b3Vec3) {
		this.cx = cx;
		this.cy = cy;
		this.cz = cz;
	}
}

/// Axis aligned bounding box.
export struct b3AABB {
	lowerBound: b3Vec3 = b3Vec3();
	upperBound: b3Vec3 = b3Vec3();

	constructor(lowerBound: b3Vec3, upperBound: b3Vec3) {
		this.lowerBound = lowerBound;
		this.upperBound = upperBound;
	}
}

/// A plane.
/// separation = dot(normal, point) - offset
export struct b3Plane {
	normal: b3Vec3 = b3Vec3();
	offset: f32 = 0;

	constructor(normal: b3Vec3, offset: f32) {
		this.normal = normal;
		this.offset = offset;
	}
}

export const b3Vec3_zero: b3Vec3 = b3Vec3(0.0, 0.0, 0.0);
export const b3Vec3_one: b3Vec3 = b3Vec3(1.0, 1.0, 1.0);
export const b3Vec3_axisX: b3Vec3 = b3Vec3(1.0, 0.0, 0.0);
export const b3Vec3_axisY: b3Vec3 = b3Vec3(0.0, 1.0, 0.0);
export const b3Vec3_axisZ: b3Vec3 = b3Vec3(0.0, 0.0, 1.0);
export const b3Quat_identity: b3Quat = b3Quat(b3Vec3(0.0, 0.0, 0.0), 1.0);
export const b3Transform_identity: b3Transform = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(0.0, 0.0, 0.0), 1.0));
export const b3Mat3_zero: b3Matrix3 = b3Matrix3(b3Vec3(0.0, 0.0, 0.0), b3Vec3(0.0, 0.0, 0.0), b3Vec3(0.0, 0.0, 0.0));
export const b3Mat3_identity: b3Matrix3 = b3Matrix3(b3Vec3(1.0, 0.0, 0.0), b3Vec3(0.0, 1.0, 0.0), b3Vec3(0.0, 0.0, 1.0));

export const b3Pos_zero: b3Pos = b3Vec3(0.0, 0.0, 0.0);
export const b3WorldTransform_identity: b3WorldTransform = b3Transform(
	b3Vec3(0.0, 0.0, 0.0),
	b3Quat(b3Vec3(0.0, 0.0, 0.0), 1.0),
);

/// @return the minimum of two integers.
export function b3MinInt(a: i32, b: i32): i32 {
	return a < b ? a : b;
}

/// @return the maximum of two integers.
export function b3MaxInt(a: i32, b: i32): i32 {
	return a > b ? a : b;
}

/// @return an integer clamped between a lower and upper bound.
export function b3ClampInt(a: i32, lower: i32, upper: i32): i32 {
	return a < lower ? lower : upper < a ? upper : a;
}

/// @return the absolute value of a float.
export function b3AbsFloat(a: f32): f32 {
	return a < 0 ? -a : a;
}

/// @return the minimum of two floats.
export function b3MinFloat(a: f32, b: f32): f32 {
	return a < b ? a : b;
}

/// @return the maximum of two floats.
export function b3MaxFloat(a: f32, b: f32): f32 {
	return a > b ? a : b;
}

/// @return a float clamped between a lower and upper bound.
export function b3ClampFloat(a: f32, lower: f32, upper: f32): f32 {
	return a < lower ? lower : upper < a ? upper : a;
}

/// Interpolate a scalar.
export function b3LerpFloat(a: f32, b: f32, alpha: f32): f32 {
	return (1.0 - alpha) * a + alpha * b;
}

/// @deprecated
export function b3Sin(radians: f32): f32 {
	let cs = b3ComputeCosSin(radians);
	return cs.sine;
}

/// @deprecated
export function b3Cos(radians: f32): f32 {
	let cs = b3ComputeCosSin(radians);
	return cs.cosine;
}

/// Convert any angle into the range [-pi, pi].
export function b3UnwindAngle(radians: f32): f32 {
	// Assuming this is deterministic
	return Mathf.rem(radians, 2.0 * B3_PI);
}

/// Vector addition.
export function b3Add(a: b3Vec3, b: b3Vec3): b3Vec3 {
	return b3Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

/// Vector subtraction.
export function b3Sub(a: b3Vec3, b: b3Vec3): b3Vec3 {
	return b3Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

/// Vector component-wise multiplication.
export function b3Mul(a: b3Vec3, b: b3Vec3): b3Vec3 {
	return b3Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

/// Vector negation.
export function b3Neg(a: b3Vec3): b3Vec3 {
	return b3Vec3(-a.x, -a.y, -a.z);
}

/// Vector dot product.
export function b3Dot(a: b3Vec3, b: b3Vec3): f32 {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Vector length.
export function b3Length(v: b3Vec3): f32 {
	return Mathf.sqrt(b3Dot(v, v));
}

/// Vector length squared.
export function b3LengthSquared(a: b3Vec3): f32 {
	return a.x * a.x + a.y * a.y + a.z * a.z;
}

/// Distance between two points.
export function b3Distance(a: b3Vec3, b: b3Vec3): f32 {
	let dv = b3Vec3(b.x - a.x, b.y - a.y, b.z - a.z);
	return b3Length(dv);
}

/// Squared distance between two points.
export function b3DistanceSquared(a: b3Vec3, b: b3Vec3): f32 {
	let dv = b3Vec3(b.x - a.x, b.y - a.y, b.z - a.z);
	return dv.x * dv.x + dv.y * dv.y + dv.z * dv.z;
}

/// Normalize a vector. Returns a zero vector if the input vector is very small.
@inline
export function b3Normalize(a: b3Vec3): b3Vec3 {
	let lengthSquared = a.x * a.x + a.y * a.y + a.z * a.z;
	if (lengthSquared > 1000.0 * FLT_MIN) {
		let s: f32 = 1.0 / Mathf.sqrt(lengthSquared);
		let u = b3Vec3(s * a.x, s * a.y, s * a.z);
		return u;
	}
	return b3Vec3(0.0, 0.0, 0.0);
}

/// Normalize a vector and return the length. Returns a zero vector
/// if the input is very small.
export function b3GetLengthAndNormalize(length: ref f32, a: b3Vec3): b3Vec3 {
	let lengthSquared = a.x * a.x + a.y * a.y + a.z * a.z;
	if (lengthSquared > 1000.0 * FLT_MIN) {
		length = Mathf.sqrt(lengthSquared);
		let s: f32 = 1.0 / length;
		let u = b3Vec3(s * a.x, s * a.y, s * a.z);
		return u;
	}

	length = 0.0;
	return b3Vec3(0.0, 0.0, 0.0);
}

/// Get a unit vector that is perpendicular to the supplied vector.
export function b3Perp(a: b3Vec3): b3Vec3 {
	// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
	// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component
	// of a unit vector must be greater or equal to 0.57735.
	let p: b3Vec3;
	if (a.x < -0.5 || 0.5 < a.x) {
		p = b3Vec3(a.y, -a.x, 0.0);
	} else {
		p = b3Vec3(0.0, a.z, -a.y);
	}

	return b3Normalize(p);
}

/// Is a vector normalized? In other words, does it have unit length?
export function b3IsNormalized(a: b3Vec3): bool {
	let aa = b3Dot(a, a);
	return b3AbsFloat(1.0 - aa) < 100.0 * FLT_EPSILON;
}

/// a + s * b
export function b3MulAdd(a: b3Vec3, s: f32, b: b3Vec3): b3Vec3 {
	return b3Vec3(a.x + s * b.x, a.y + s * b.y, a.z + s * b.z);
}

/// a - s * b
export function b3MulSub(a: b3Vec3, s: f32, b: b3Vec3): b3Vec3 {
	return b3Vec3(a.x - s * b.x, a.y - s * b.y, a.z - s * b.z);
}

/// s * a
export function b3MulSV(s: f32, a: b3Vec3): b3Vec3 {
	return b3Vec3(s * a.x, s * a.y, s * a.z);
}

/// https://en.wikipedia.org/wiki/Cross_product
export function b3Cross(a: b3Vec3, b: b3Vec3): b3Vec3 {
	let c: b3Vec3;
	c.x = a.y * b.z - a.z * b.y;
	c.y = a.z * b.x - a.x * b.z;
	c.z = a.x * b.y - a.y * b.x;
	return c;
}

/// Linearly interpolate between two vectors.
export function b3Lerp(a: b3Vec3, b: b3Vec3, alpha: f32): b3Vec3 {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0.0 <= alpha && alpha <= 1.0);

	let c = b3Vec3(
		(1.0 - alpha) * a.x + alpha * b.x,
		(1.0 - alpha) * a.y + alpha * b.y,
		(1.0 - alpha) * a.z + alpha * b.z,
	);
	return c;
}

/// Blend two vectors: s * a + t * b
export function b3Blend2(s: f32, a: b3Vec3, t: f32, b: b3Vec3): b3Vec3 {
	let d = b3Vec3(s * a.x + t * b.x, s * a.y + t * b.y, s * a.z + t * b.z);
	return d;
}

/// Component-wise absolute value.
export function b3Abs(a: b3Vec3): b3Vec3 {
	return b3Vec3(b3AbsFloat(a.x), b3AbsFloat(a.y), b3AbsFloat(a.z));
}

/// Component-wise -1 or 1 (1 if zero).
export function b3Sign(a: b3Vec3): b3Vec3 {
	return b3Vec3(<f32>(a.x >= 0.0 ? 1.0 : -1.0), <f32>(a.y >= 0.0 ? 1.0 : -1.0), <f32>(a.z >= 0.0 ? 1.0 : -1.0));
}

/// Component-wise minimum value.
export function b3Min(a: b3Vec3, b: b3Vec3): b3Vec3 {
	return b3Vec3(b3MinFloat(a.x, b.x), b3MinFloat(a.y, b.y), b3MinFloat(a.z, b.z));
}

/// Component-wise maximum value.
export function b3Max(a: b3Vec3, b: b3Vec3): b3Vec3 {
	return b3Vec3(b3MaxFloat(a.x, b.x), b3MaxFloat(a.y, b.y), b3MaxFloat(a.z, b.z));
}

/// Component-wise clamped value.
export function b3Clamp(a: b3Vec3, lower: b3Vec3, upper: b3Vec3): b3Vec3 {
	let b: b3Vec3;
	b.x = b3ClampFloat(a.x, lower.x, upper.x);
	b.y = b3ClampFloat(a.y, lower.y, upper.y);
	b.z = b3ClampFloat(a.z, lower.z, upper.z);
	return b;
}

/// Create a safe scaling value for scaling collision. This allows
/// negative scale, but keeps scale sufficiently far from zero.
export function b3SafeScale(a: b3Vec3): b3Vec3 {
	let absScale = b3Abs(a);
	let minScale = b3Vec3(B3_MIN_SCALE, B3_MIN_SCALE, B3_MIN_SCALE);
	let safeScale = b3Mul(b3Sign(a), b3Max(absScale, minScale));
	return safeScale;
}

/// Does the supplied quaternion have unit length?
export function b3IsNormalizedQuat(q: b3Quat): bool {
	let qq = q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z + q.s * q.s;
	return 1.0 - 20.0 * FLT_EPSILON < qq && qq < 1.0 + 20.0 * FLT_EPSILON;
}

/// Rotate a vector.
export function b3RotateVector(q: b3Quat, v: b3Vec3): b3Vec3 {
	// v + 2 * cross(q.v, cross(q.v, v) + q.s * v)
	let t1 = b3Cross(q.v, v);
	let t2 = b3MulAdd(t1, q.s, v);
	let t3 = b3Cross(q.v, t2);
	return b3MulAdd(v, 2.0, t3);
}

/// Inverse rotate a vector.
export function b3InvRotateVector(q: b3Quat, v: b3Vec3): b3Vec3 {
	// v + 2 * cross(q.v, cross(q.v, v) - q.s * v)
	let t1 = b3Cross(q.v, v);
	let t2 = b3MulSub(t1, q.s, v);
	let t3 = b3Cross(q.v, t2);
	return b3MulAdd(v, 2.0, t3);
}

/// Compute dot product of two quaternions. Useful for polarity tests.
export function b3DotQuat(a: b3Quat, b: b3Quat): f32 {
	return a.v.x * b.v.x + a.v.y * b.v.y + a.v.z * b.v.z + a.s * b.s;
}

/// Multiply two quaternions.
export function b3MulQuat(q1: b3Quat, q2: b3Quat): b3Quat {
	let t1 = b3Cross(q1.v, q2.v);
	let t2 = b3MulAdd(t1, q1.s, q2.v);
	let t3 = b3MulAdd(t2, q2.s, q1.v);
	let q = b3Quat(t3, q1.s * q2.s - b3Dot(q1.v, q2.v));
	return q;
}

/// Compute a relative quaternion.
/// inv(q1) * q2
export function b3InvMulQuat(q1: b3Quat, q2: b3Quat): b3Quat {
	let t1 = b3Cross(q2.v, q1.v);
	let t2 = b3MulAdd(t1, q1.s, q2.v);
	let t3 = b3MulSub(t2, q2.s, q1.v);
	let q = b3Quat(t3, q1.s * q2.s + b3Dot(q1.v, q2.v));
	return q;
}

/// Quaternion conjugate (cheap inverse).
export function b3Conjugate(q: b3Quat): b3Quat {
	return b3Quat(b3Vec3(-q.v.x, -q.v.y, -q.v.z), q.s);
}

/// Component-wise quaternion negation.
export function b3NegateQuat(q: b3Quat): b3Quat {
	return b3Quat(b3Vec3(-q.v.x, -q.v.y, -q.v.z), -q.s);
}

/// Normalize a quaternion.
export function b3NormalizeQuat(q: b3Quat): b3Quat {
	let lengthSq = b3DotQuat(q, q);
	if (lengthSq > 1000.0 * FLT_MIN) {
		let s: f32 = 1.0 / Mathf.sqrt(lengthSq);
		let qn = b3Quat(b3Vec3(s * q.v.x, s * q.v.y, s * q.v.z), s * q.s);
		return qn;
	}

	return b3Quat_identity;
}

/// Make a quaternion that is equivalent to rotating around an axis by a specified angle.
export function b3MakeQuatFromAxisAngle(axis: b3Vec3, radians: f32): b3Quat {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsNormalized(axis));
	let cs = b3ComputeCosSin(0.5 * radians);
	let q = b3Quat(b3Vec3(cs.sine * axis.x, cs.sine * axis.y, cs.sine * axis.z), cs.cosine);
	return q;
}

/// Get the axis and angle from a quaternion. Assumes the quaternion is normalized.
export function b3GetAxisAngle(radians: ref f32, q: b3Quat): b3Vec3 {
	let length = Mathf.sqrt(q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z);
	radians = 2.0 * b3Atan2(length, q.s);
	if (length > 0.0) {
		let invLength: f32 = 1.0 / length;
		let axis = b3Vec3(invLength * q.v.x, invLength * q.v.y, invLength * q.v.z);
		return axis;
	}

	return b3Vec3_zero;
}

/// Get the angle for a quaternion in radians
export function b3GetQuatAngle(q: b3Quat): f32 {
	let length = Mathf.sqrt(q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z);
	return 2.0 * b3Atan2(length, q.s);
}

/// Twist angle around the z-axis, used for twist limit and revolute angle limit
export function b3GetTwistAngle(q: b3Quat): f32 {
	// Account for polarity to keep the twist angle in range.
	// This is simpler than asking the user to check polarity or unwinding.
	let twist = q.s < 0.0 ? b3Atan2(-q.v.z, -q.s) : b3Atan2(q.v.z, q.s);
	twist *= 2.0;
	if (B3_ASSERT_ENABLED) B3_ASSERT(-B3_PI <= twist && twist <= B3_PI);
	return twist;
}

/// Swing angle used for cone limit
@inline
export function b3GetSwingAngle(q: b3Quat): f32 {
	// Polarity should not matter because all terms are squared.
	let x = Mathf.sqrt(q.v.z * q.v.z + q.s * q.s);
	let y = Mathf.sqrt(q.v.x * q.v.x + q.v.y * q.v.y);
	let swing: f32 = 2.0 * b3Atan2(y, x);
	if (B3_ASSERT_ENABLED) B3_ASSERT(0.0 <= swing && swing <= B3_PI);
	return swing;
}

/// Linearly interpolate and normalize between two quaternions
export function b3NLerp(q1: b3Quat, q2: b3Quat, alpha: f32): b3Quat {
	if (b3DotQuat(q1, q2) < 0.0) {
		q1 = b3Quat(b3Vec3(-q1.v.x, -q1.v.y, -q1.v.z), -q1.s);
	}

	let q: b3Quat;
	q.v = b3Lerp(q1.v, q2.v, alpha);
	q.s = (1.0 - alpha) * q1.s + alpha * q2.s;

	return b3NormalizeQuat(q);
}

/// Multiply two transforms. If the result is applied to a point p local to frame B,
/// the transform would first convert p to a point local to frame A, then into a point
/// in the world frame. This is useful if frame B is a child of frame A.
export function b3MulTransforms(a: b3Transform, b: b3Transform): b3Transform {
	let out: b3Transform;
	out.p = b3Add(b3RotateVector(a.q, b.p), a.p);
	out.q = b3MulQuat(a.q, b.q);
	return out;
}

/// Creates a transform that converts a local point in frame B to a local point in frame A.
/// This is useful for transforming points between the local spaces of two frames that are
/// in world space.
@inline
export function b3InvMulTransforms(a: b3Transform, b: b3Transform): b3Transform {
	let out: b3Transform;
	out.p = b3InvRotateVector(a.q, b3Sub(b.p, a.p));
	out.q = b3InvMulQuat(a.q, b.q);
	return out;
}

/// Get the inverse of a transform.
export function b3InvertTransform(t: b3Transform): b3Transform {
	let out: b3Transform;
	out.p = b3InvRotateVector(t.q, b3Neg(t.p));
	out.q = b3Conjugate(t.q);
	return out;
}

/// Transform a point.
@inline
export function b3TransformPoint(t: b3Transform, v: b3Vec3): b3Vec3 {
	let rv = b3RotateVector(t.q, v);
	return b3Add(rv, t.p);
}

/// Inverse transform a point.
export function b3InvTransformPoint(t: b3Transform, v: b3Vec3): b3Vec3 {
	return b3InvRotateVector(t.q, b3Sub(v, t.p));
}

// World position boundary. These cross between the double precision world space at the public
// boundary and the float interior. In single precision the types collapse and the conversions
// are no-ops.

/// Convert a vector to a world position.
export function b3ToPos(v: b3Vec3): b3Pos {
	return b3Vec3(v.x, v.y, v.z);
}

/// Lossy conversion of a world position to a float vector.
export function b3ToVec3(p: b3Pos): b3Vec3 {
	return b3Vec3(p.x, p.y, p.z);
}

/// Narrow a world coordinate to float, rounding toward negative infinity.
/// With large world mode off this is a plain conversion.
export function b3RoundDownFloat(x: f64): f32 {
	return <f32>x;
}

/// Narrow a world coordinate to float, rounding toward positive infinity.
export function b3RoundUpFloat(x: f64): f32 {
	return <f32>x;
}

/// a - b, demoted to float. The primary precision boundary operation.
export function b3SubPos(a: b3Pos, b: b3Pos): b3Vec3 {
	return b3Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

/// p + d
export function b3OffsetPos(p: b3Pos, d: b3Vec3): b3Pos {
	return b3Vec3(p.x + d.x, p.y + d.y, p.z + d.z);
}

/// World position interpolation for sweeps and sampling.
export function b3LerpPosition(a: b3Pos, b: b3Pos, t: f32): b3Pos {
	return b3Vec3((1.0 - t) * a.x + t * b.x, (1.0 - t) * a.y + t * b.y, (1.0 - t) * a.z + t * b.z);
}

/// Transform a local point to a world position. Rotation in float, translation in double.
export function b3TransformWorldPoint(t: b3WorldTransform, p: b3Vec3): b3Pos {
	let r = b3RotateVector(t.q, p);
	return b3Vec3(t.p.x + r.x, t.p.y + r.y, t.p.z + r.z);
}

/// Transform a world position to a local point. One double subtraction, then float.
export function b3InvTransformWorldPoint(t: b3WorldTransform, p: b3Pos): b3Vec3 {
	let d = b3Vec3(p.x - t.p.x, p.y - t.p.y, p.z - t.p.z);
	return b3InvRotateVector(t.q, d);
}

/// Compose a world transform with a local transform.
@inline
export function b3MulWorldTransforms(A: b3WorldTransform, B: b3Transform): b3WorldTransform {
	let C: b3WorldTransform;
	C.q = b3MulQuat(A.q, B.q);
	let r = b3RotateVector(A.q, B.p);
	C.p = b3Vec3(A.p.x + r.x, A.p.y + r.y, A.p.z + r.z);
	return C;
}

/// Relative transform of frame B in frame A. The narrow phase boundary.
@inline
export function b3InvMulWorldTransforms(A: b3WorldTransform, B: b3WorldTransform): b3Transform {
	let C: b3Transform;
	C.q = b3InvMulQuat(A.q, B.q);
	let d = b3Vec3(B.p.x - A.p.x, B.p.y - A.p.y, B.p.z - A.p.z);
	C.p = b3InvRotateVector(A.q, d);
	return C;
}

/// Shift a world transform into the frame of a base position.
export function b3ToRelativeTransform(t: b3WorldTransform, base: b3Pos): b3Transform {
	let r: b3Transform;
	r.q = t.q;
	r.p = b3Vec3(t.p.x - base.x, t.p.y - base.y, t.p.z - base.z);
	return r;
}

/// Promote a float transform to a world transform. Lossless.
export function b3MakeWorldTransform(t: b3Transform): b3WorldTransform {
	let w: b3WorldTransform;
	w.p = b3ToPos(t.p);
	w.q = t.q;
	return w;
}

/// Translate a local AABB by a world position, rounding outward so the single precision box
/// encloses the double precision box.
/// In single precision mode the the rounding is a no-op.
export function b3OffsetAABB(localBox: b3AABB, origin: b3Pos): b3AABB {
	let out: b3AABB;
	out.lowerBound.x = b3RoundDownFloat(origin.x + localBox.lowerBound.x);
	out.lowerBound.y = b3RoundDownFloat(origin.y + localBox.lowerBound.y);
	out.lowerBound.z = b3RoundDownFloat(origin.z + localBox.lowerBound.z);
	out.upperBound.x = b3RoundUpFloat(origin.x + localBox.upperBound.x);
	out.upperBound.y = b3RoundUpFloat(origin.y + localBox.upperBound.y);
	out.upperBound.z = b3RoundUpFloat(origin.z + localBox.upperBound.z);
	return out;
}

/// Compute the determinant of a 3-by-3 matrix.
export function b3Det(m: b3Matrix3): f32 {
	return b3Dot(m.cx, b3Cross(m.cy, m.cz));
}

/// Multiply a matrix times a column vector.
export function b3MulMV(m: b3Matrix3, a: b3Vec3): b3Vec3 {
	let b = b3Vec3(
		m.cx.x * a.x + m.cy.x * a.y + m.cz.x * a.z,
		m.cx.y * a.x + m.cy.y * a.y + m.cz.y * a.z,
		m.cx.z * a.x + m.cy.z * a.y + m.cz.z * a.z,
	);
	return b;
}

/// Negate a matrix.
export function b3NegateMat3(a: b3Matrix3): b3Matrix3 {
	return b3Matrix3(
		b3Vec3(-a.cx.x, -a.cx.y, -a.cx.z),
		b3Vec3(-a.cy.x, -a.cy.y, -a.cy.z),
		b3Vec3(-a.cz.x, -a.cz.y, -a.cz.z),
	);
}

/// Matrix addition.
/// @return a + b
export function b3AddMM(a: b3Matrix3, b: b3Matrix3): b3Matrix3 {
	return b3Matrix3(
		b3Vec3(a.cx.x + b.cx.x, a.cx.y + b.cx.y, a.cx.z + b.cx.z),
		b3Vec3(a.cy.x + b.cy.x, a.cy.y + b.cy.y, a.cy.z + b.cy.z),
		b3Vec3(a.cz.x + b.cz.x, a.cz.y + b.cz.y, a.cz.z + b.cz.z),
	);
}

/// Matrix subtraction.
/// @return a - b
export function b3SubMM(a: b3Matrix3, b: b3Matrix3): b3Matrix3 {
	return b3Matrix3(
		b3Vec3(a.cx.x - b.cx.x, a.cx.y - b.cx.y, a.cx.z - b.cx.z),
		b3Vec3(a.cy.x - b.cy.x, a.cy.y - b.cy.y, a.cy.z - b.cy.z),
		b3Vec3(a.cz.x - b.cz.x, a.cz.y - b.cz.y, a.cz.z - b.cz.z),
	);
}

/// Multiply a matrix by a scalar, component-wise.
export function b3MulSM(s: f32, a: b3Matrix3): b3Matrix3 {
	return b3Matrix3(
		b3Vec3(s * a.cx.x, s * a.cx.y, s * a.cx.z),
		b3Vec3(s * a.cy.x, s * a.cy.y, s * a.cy.z),
		b3Vec3(s * a.cz.x, s * a.cz.y, s * a.cz.z),
	);
}

/// Matrix multiplication.
/// @return a * b
export function b3MulMM(a: b3Matrix3, b: b3Matrix3): b3Matrix3 {
	let out: b3Matrix3;
	out.cx = b3MulMV(a, b.cx);
	out.cy = b3MulMV(a, b.cy);
	out.cz = b3MulMV(a, b.cz);
	return out;
}

/// Matrix transpose.
export function b3Transpose(m: b3Matrix3): b3Matrix3 {
	let out: b3Matrix3;
	out.cx = b3Vec3(m.cx.x, m.cy.x, m.cz.x);
	out.cy = b3Vec3(m.cx.y, m.cy.y, m.cz.y);
	out.cz = b3Vec3(m.cx.z, m.cy.z, m.cz.z);

	return out;
}

/// General matrix inverse.
export function b3InvertMatrix(m: b3Matrix3): b3Matrix3 {
	let det = b3Det(m);
	if (b3AbsFloat(det) > 1000.0 * FLT_MIN) {
		let invDet: f32 = 1.0 / det;
		let out: b3Matrix3;
		out.cx = b3MulSV(invDet, b3Cross(m.cy, m.cz));
		out.cy = b3MulSV(invDet, b3Cross(m.cz, m.cx));
		out.cz = b3MulSV(invDet, b3Cross(m.cx, m.cy));

		return b3Transpose(out);
	}

	return b3Mat3_zero;
}

/// Solve a matrix equation.
/// @return inv(m) * a
@inline
export function b3Solve3(m: b3Matrix3, a: b3Vec3): b3Vec3 {
	let det = b3Det(m);
	if (b3AbsFloat(det) > 1000.0 * FLT_MIN) {
		let invDet: f32 = 1.0 / det;
		let s: b3Matrix3;
		s.cx = b3Cross(m.cy, m.cz);
		s.cy = b3Cross(m.cz, m.cx);
		s.cz = b3Cross(m.cx, m.cy);

		let b = b3Vec3(invDet * b3Dot(s.cx, a), invDet * b3Dot(s.cy, a), invDet * b3Dot(s.cz, a));

		return b;
	}

	return b3Vec3_zero;
}

/// Invert a matrix.
export function b3InvertT(m: b3Matrix3): b3Matrix3 {
	let det = b3Det(m);
	if (b3AbsFloat(det) > 1000.0 * FLT_MIN) {
		let invDet: f32 = 1.0 / det;
		let out: b3Matrix3;
		out.cx = b3MulSV(invDet, b3Cross(m.cy, m.cz));
		out.cy = b3MulSV(invDet, b3Cross(m.cz, m.cx));
		out.cz = b3MulSV(invDet, b3Cross(m.cx, m.cy));
		return out;
	}

	return b3Mat3_zero;
}

/// Get the component-wise absolute value of a matrix.
export function b3AbsMatrix3(m: b3Matrix3): b3Matrix3 {
	let out: b3Matrix3;
	out.cx = b3Abs(m.cx);
	out.cy = b3Abs(m.cy);
	out.cz = b3Abs(m.cz);

	return out;
}

/// Make a matrix from a quaternion. This is useful if you need to
/// rotate many vectors.
/// The force inline improves the performance of b3ShapeDistance.
@inline
export function b3MakeMatrixFromQuat(q: b3Quat): b3Matrix3 {
	let xx = q.v.x * q.v.x;
	let yy = q.v.y * q.v.y;
	let zz = q.v.z * q.v.z;
	let xy = q.v.x * q.v.y;
	let xz = q.v.x * q.v.z;
	let xw = q.v.x * q.s;
	let yz = q.v.y * q.v.z;
	let yw = q.v.y * q.s;
	let zw = q.v.z * q.s;

	return b3Matrix3(
		b3Vec3(1.0 - 2.0 * (yy + zz), 2.0 * (xy + zw), 2.0 * (xz - yw)),
		b3Vec3(2.0 * (xy - zw), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + xw)),
		b3Vec3(2.0 * (xz + yw), 2.0 * (yz - xw), 1.0 - 2.0 * (xx + yy)),
	);
}

/// Get the AABB of a point cloud.
export function b3MakeAABB(points: StaticArray<b3Vec3>, count: i32, radius: f32): b3AABB {
	if (B3_ASSERT_ENABLED) B3_ASSERT(count > 0);
	let a = b3AABB(points[0], points[0]);
	for (let i = 1; i < count; ++i) {
		a.lowerBound = b3Min(a.lowerBound, points[i]);
		a.upperBound = b3Max(a.upperBound, points[i]);
	}

	let r = b3Vec3(radius, radius, radius);
	a.lowerBound = b3Sub(a.lowerBound, r);
	a.upperBound = b3Add(a.upperBound, r);

	return a;
}

/// Does a fully contain b?
export function b3AABB_Contains(a: b3AABB, b: b3AABB): bool {
	if (a.lowerBound.x > b.lowerBound.x || b.upperBound.x > a.upperBound.x) return false;
	if (a.lowerBound.y > b.lowerBound.y || b.upperBound.y > a.upperBound.y) return false;
	if (a.lowerBound.z > b.lowerBound.z || b.upperBound.z > a.upperBound.z) return false;

	return true;
}

/// Get the surface area of an axis-aligned bounding box.
export function b3AABB_Area(a: b3AABB): f32 {
	let delta = b3Sub(a.upperBound, a.lowerBound);
	return 2.0 * (delta.x * delta.y + delta.y * delta.z + delta.z * delta.x);
}

/// Get the center of an axis-aligned bounding box.
export function b3AABB_Center(a: b3AABB): b3Vec3 {
	return b3MulSV(0.5, b3Add(a.upperBound, a.lowerBound));
}

/// Get the extents (half-widths) of an axis-aligned bounding box.
export function b3AABB_Extents(a: b3AABB): b3Vec3 {
	return b3MulSV(0.5, b3Sub(a.upperBound, a.lowerBound));
}

/// Get the union of two axis-aligned bounding boxes.
export function b3AABB_Union(a: b3AABB, b: b3AABB): b3AABB {
	let out: b3AABB;
	out.lowerBound = b3Min(a.lowerBound, b.lowerBound);
	out.upperBound = b3Max(a.upperBound, b.upperBound);
	return out;
}

/// Add uniform padding to an axis-aligned bounding box.
export function b3AABB_Inflate(a: b3AABB, extension: f32): b3AABB {
	let radius = b3Vec3(extension, extension, extension);

	let out: b3AABB;
	out.lowerBound = b3Sub(a.lowerBound, radius);
	out.upperBound = b3Add(a.upperBound, radius);
	return out;
}

/// Do two axis-aligned boxes overlap?
export function b3AABB_Overlaps(a: b3AABB, b: b3AABB): bool {
	// No intersection if separated along one axis
	if (a.upperBound.x < b.lowerBound.x || a.lowerBound.x > b.upperBound.x) return false;
	if (a.upperBound.y < b.lowerBound.y || a.lowerBound.y > b.upperBound.y) return false;
	if (a.upperBound.z < b.lowerBound.z || a.lowerBound.z > b.upperBound.z) return false;

	// Overlapping on all axis means bounds are intersecting
	return true;
}

/// Transform an axis-aligned bounding box. This can create a larger box
/// than if you recomputed the AABB of the original shape with the transform
/// applied.
@inline
export function b3AABB_Transform(transform: b3Transform, a: b3AABB): b3AABB {
	let center = b3TransformPoint(transform, b3AABB_Center(a));
	let m = b3MakeMatrixFromQuat(transform.q);
	let extent = b3MulMV(b3AbsMatrix3(m), b3AABB_Extents(a));
	let out = b3AABB(b3Sub(center, extent), b3Add(center, extent));
	return out;
}

/// Get the closest point on an axis-aligned bounding box.
export function b3ClosestPointToAABB(point: b3Vec3, a: b3AABB): b3Vec3 {
	return b3Clamp(point, a.lowerBound, a.upperBound);
}

/// The closest points between to segments or infinite lines.
export struct b3SegmentDistanceResult {
	point1: b3Vec3 = b3Vec3();
	fraction1: f32 = 0;
	point2: b3Vec3 = b3Vec3();
	fraction2: f32 = 0;
}

// src/math_functions.c

const FLT_INF: f32 = reinterpret<f32>(0x7F800000);

@inline
function isnan(a: f32): bool {
	return isNaN(a);
}

@inline
function isinf(a: f32): bool {
	return a == FLT_INF || a == -FLT_INF;
}

/// Is this a valid number? Not NaN or infinity.
export function b3IsValidFloat(a: f32): bool {
	if (isnan(a)) {
		return false;
	}

	if (isinf(a)) {
		return false;
	}

	return true;
}

/// Is this a valid vector? Not NaN or infinity.
export function b3IsValidVec3(a: b3Vec3): bool {
	if (isnan(a.x) || isnan(a.y) || isnan(a.z)) {
		return false;
	}

	if (isinf(a.x) || isinf(a.y) || isinf(a.z)) {
		return false;
	}

	return true;
}

/// Is this a valid quaternion? Not NaN or infinity. Is normalized.
export function b3IsValidQuat(a: b3Quat): bool {
	if (isnan(a.v.x) || isnan(a.v.y) || isnan(a.v.z) || isnan(a.s)) {
		return false;
	}

	if (isinf(a.v.x) || isinf(a.v.y) || isinf(a.v.z) || isinf(a.s)) {
		return false;
	}

	return b3IsNormalizedQuat(a);
}

/// Is this a valid transform? Not NaN or infinity. Is normalized.
export function b3IsValidTransform(a: b3Transform): bool {
	return b3IsValidVec3(a.p) && b3IsValidQuat(a.q);
}

/// Is this a valid matrix? Not NaN or infinity.
export function b3IsValidMatrix3(a: b3Matrix3): bool {
	return b3IsValidVec3(a.cx) && b3IsValidVec3(a.cy) && b3IsValidVec3(a.cz);
}

/// Is this a valid bounding box? Not Nan or infinity. Upper bound greater than or equal to lower bound.
export function b3IsValidAABB(a: b3AABB): bool {
	if (b3IsValidVec3(a.lowerBound) == false) {
		return false;
	}

	if (b3IsValidVec3(a.upperBound) == false) {
		return false;
	}

	if (a.lowerBound.x > a.upperBound.x) {
		return false;
	}

	if (a.lowerBound.y > a.upperBound.y) {
		return false;
	}

	if (a.lowerBound.z > a.upperBound.z) {
		return false;
	}

	return true;
}

/// Is this AABB reasonably close to the origin? See B3_HUGE.
export function b3IsBoundedAABB(a: b3AABB): bool {
	if (a.lowerBound.x < -B3_HUGE() || a.lowerBound.y < -B3_HUGE() || a.lowerBound.z < -B3_HUGE()) {
		return false;
	}

	if (a.upperBound.x > B3_HUGE() || a.upperBound.y > B3_HUGE() || a.upperBound.z > B3_HUGE()) {
		return false;
	}

	return true;
}

/// Is this AABB valid and reasonable?
export function b3IsSaneAABB(a: b3AABB): bool {
	if (b3IsValidAABB(a) == false) {
		return false;
	}

	if (a.lowerBound.x < -B3_HUGE() || a.lowerBound.y < -B3_HUGE() || a.lowerBound.z < -B3_HUGE()) {
		return false;
	}

	if (a.upperBound.x > B3_HUGE() || a.upperBound.y > B3_HUGE() || a.upperBound.z > B3_HUGE()) {
		return false;
	}

	return true;
}

/// Is this a valid plane? Normal is a unit vector. Not Nan or infinity.
export function b3IsValidPlane(a: b3Plane): bool {
	if (b3IsValidVec3(a.normal) == false) {
		return false;
	}

	if (b3IsNormalized(a.normal) == false) {
		return false;
	}

	return b3IsValidFloat(a.offset);
}

/// Is this a valid world position? Not NaN or infinity.
export function b3IsValidPosition(p: b3Pos): bool {
	if (isnan(p.x) || isnan(p.y) || isnan(p.z)) {
		return false;
	}

	if (isinf(p.x) || isinf(p.y) || isinf(p.z)) {
		return false;
	}

	return true;
}

/// Is this a valid world transform? Not NaN or infinity. Rotation is normalized.
export function b3IsValidWorldTransform(t: b3WorldTransform): bool {
	return b3IsValidPosition(t.p) && b3IsValidQuat(t.q);
}

/// Compute an approximate arctangent in the range [-pi, pi]
/// This is hand coded for cross-platform determinism. The atan2f
/// function in the standard library is not cross-platform deterministic.
///	Accurate to around 0.0023 degrees.
// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4
export function b3Atan2(y: f32, x: f32): f32 {
	// Added check for (0,0) to match atan2f and avoid NaN
	if (x == 0.0 && y == 0.0) {
		return 0.0;
	}

	let ax = b3AbsFloat(x);
	let ay = b3AbsFloat(y);
	let mx = b3MaxFloat(ay, ax);
	let mn = b3MinFloat(ay, ax);
	let a = mn / mx;

	// Minimax polynomial approximation to atan(a) on [0,1]
	let s = a * a;
	let c = s * a;
	let q = s * s;
	let r: f32 = 0.024840285 * q + 0.18681418;
	let t: f32 = -0.094097948 * q - 0.33213072;
	r = r * s + t;
	r = r * c + a;

	// Map to full circle
	if (ay > ax) {
		r = 1.57079637 - r;
	}

	if (x < 0) {
		r = 3.14159274 - r;
	}

	if (y < 0) {
		r = -r;
	}

	return r;
}

/// Compute the cosine and sine of an angle in radians. Implemented
/// for cross-platform determinism.
// Approximate cosine and sine for determinism. In my testing cosf and sinf produced
// the same results on x64 and ARM using MSVC, GCC, and Clang. However, I don't trust
// this result.
// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
export function b3ComputeCosSin(radians: f32): b3CosSin {
	let x = b3UnwindAngle(radians);
	let pi2: f32 = B3_PI * B3_PI;

	// cosine needs angle in [-pi/2, pi/2]
	let c: f32;
	if (x < -0.5 * B3_PI) {
		let y: f32 = x + B3_PI;
		let y2 = y * y;
		c = -(pi2 - 4.0 * y2) / (pi2 + y2);
	} else if (x > 0.5 * B3_PI) {
		let y: f32 = x - B3_PI;
		let y2 = y * y;
		c = -(pi2 - 4.0 * y2) / (pi2 + y2);
	} else {
		let y2 = x * x;
		c = (pi2 - 4.0 * y2) / (pi2 + y2);
	}

	// sine needs angle in [0, pi]
	let s: f32;
	if (x < 0.0) {
		let y: f32 = x + B3_PI;
		s = (-16.0 * y * (B3_PI - y)) / (5.0 * pi2 - 4.0 * y * (B3_PI - y));
	} else {
		s = (16.0 * x * (B3_PI - x)) / (5.0 * pi2 - 4.0 * x * (B3_PI - x));
	}

	let mag = Mathf.sqrt(s * s + c * c);
	let invMag: f32 = mag > 0.0 ? 1.0 / mag : 0.0;
	let cs = b3CosSin(c * invMag, s * invMag);
	return cs;
}

/// Extract a quaternion from a rotation matrix.
export function b3MakeQuatFromMatrix(m: b3Matrix3): b3Quat {
	let c1 = m.cx;
	let c2 = m.cy;
	let c3 = m.cz;

	let q: b3Quat;

	let trace = m.cx.x + m.cy.y + m.cz.z;
	if (trace >= 0.0) {
		q.v.x = c2.z - c3.y;
		q.v.y = c3.x - c1.z;
		q.v.z = c1.y - c2.x;
		q.s = trace + 1.0;
	} else {
		if (c1.x > c2.y && c1.x > c3.z) {
			q.v.x = c1.x - c2.y - c3.z + 1.0;
			q.v.y = c2.x + c1.y;
			q.v.z = c3.x + c1.z;
			q.s = c2.z - c3.y;
		} else if (c2.y > c3.z) {
			q.v.x = c1.y + c2.x;
			q.v.y = c2.y - c3.z - c1.x + 1.0;
			q.v.z = c3.y + c2.z;
			q.s = c3.x - c1.z;
		} else {
			q.v.x = c1.z + c3.x;
			q.v.y = c2.z + c3.y;
			q.v.z = c3.z - c1.x - c2.y + 1.0;
			q.s = c1.y - c2.x;
		}
	}

	// The algorithm is simplified and made more accurate by normalizing at the end
	return b3NormalizeQuat(q);
}

/// Find a quaternion that rotates one vector to another.
export function b3ComputeQuatBetweenUnitVectors(v1: b3Vec3, v2: b3Vec3): b3Quat {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsNormalized(v1));
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsNormalized(v2));

	let out: b3Quat;

	let m = b3Lerp(v1, v2, 0.5);
	let tolerance: f32 = 100.0 * FLT_EPSILON;
	if (b3LengthSquared(m) > tolerance * tolerance) {
		out.v = b3Cross(v1, m);
		out.s = b3Dot(v1, m);
	} else {
		// Anti-parallel: Use a perpendicular vector
		if (b3AbsFloat(v1.x) > 0.5) {
			out.v.x = v1.y;
			out.v.y = -v1.x;
			out.v.z = 0.0;
		} else {
			out.v.x = 0.0;
			out.v.y = v1.z;
			out.v.z = -v1.y;
		}

		out.s = 0.0;
	}

	// The algorithm is simplified and made more accurate by normalizing at the end
	return b3NormalizeQuat(out);
}

/// Compute the closest points on two infinite lines.
export function b3LineDistance(p1: b3Vec3, d1: b3Vec3, p2: b3Vec3, d2: b3Vec3): b3SegmentDistanceResult {
	let result: b3SegmentDistanceResult;

	// Solve A*x = b
	let a11 = b3Dot(d1, d1);
	let a12 = -b3Dot(d1, d2);
	let a21 = b3Dot(d2, d1);
	let a22 = -b3Dot(d2, d2);

	let w = b3Sub(p1, p2);
	let b1 = -b3Dot(d1, w);
	let b2 = -b3Dot(d2, w);

	let det = a11 * a22 - a12 * a21;
	if (det * det < 1000.0 * FLT_MIN) {
		// Lines are parallel - project p2 onto line L1: x1 = p1 + s1 * d1
		let s1 = b3Dot(b3Sub(p2, p1), d1) / b3Dot(d1, d1);
		let s2: f32 = 0.0;

		result.point1 = b3MulAdd(p1, s1, d1);
		result.fraction1 = s1;
		result.point2 = b3MulAdd(p2, s2, d2);
		result.fraction2 = s2;

		return result;
	}

	let s1 = (a22 * b1 - a12 * b2) / det;
	let s2 = (a11 * b2 - a21 * b1) / det;

	result.point1 = b3MulAdd(p1, s1, d1);
	result.fraction1 = s1;
	result.point2 = b3MulAdd(p2, s2, d2);
	result.fraction2 = s2;
	return result;
}

/// Compute the closest points on two line segments.
export function b3SegmentDistance(p1: b3Vec3, q1: b3Vec3, p2: b3Vec3, q2: b3Vec3): b3SegmentDistanceResult {
	let result: b3SegmentDistanceResult;

	let d1 = b3Sub(q1, p1);
	let d2 = b3Sub(q2, p2);
	let r = b3Sub(p1, p2);

	let a = b3Dot(d1, d1);
	let b = b3Dot(d1, d2);
	let c = b3Dot(d1, r);
	let e = b3Dot(d2, d2);
	let f = b3Dot(d2, r);

	// Check if one of the segments degenerates into a point
	if (a < 100.0 * FLT_EPSILON && e < 100.0 * FLT_EPSILON) {
		// Both segments degenerate into points
		result.point1 = p1;
		result.fraction1 = 0.0;
		result.point2 = p2;
		result.fraction2 = 0.0;

		return result;
	}

	if (a < 100.0 * FLT_EPSILON) {
		// First segment degenerates into a point
		let s2 = b3ClampFloat(f / e, 0.0, 1.0);

		result.point1 = p1;
		result.fraction1 = 0.0;
		result.point2 = b3MulAdd(p2, s2, d2);
		result.fraction2 = s2;

		return result;
	}

	if (e < 100.0 * FLT_EPSILON) {
		// Second segment degenerates into a point
		let s1 = b3ClampFloat(-c / a, 0.0, 1.0);

		result.point1 = b3MulAdd(p1, s1, d1);
		result.fraction1 = s1;
		result.point2 = p2;
		result.fraction2 = 0.0;

		return result;
	}

	// Non-degenerate case
	let denom = a * e - b * b;
	let s1: f32 = denom > 1000.0 * FLT_MIN ? b3ClampFloat((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
	let s2 = (b * s1 + f) / e;

	// Clamp lambda2 and recompute lambda1 if necessary
	if (s2 < 0.0) {
		s1 = b3ClampFloat(-c / a, 0.0, 1.0);
		s2 = 0.0;
	} else if (s2 > 1.0) {
		s1 = b3ClampFloat((b - c) / a, 0.0, 1.0);
		s2 = 1.0;
	}

	result.point1 = b3MulAdd(p1, s1, d1);
	result.fraction1 = s1;
	result.point2 = b3MulAdd(p2, s2, d2);
	result.fraction2 = s2;

	return result;
}

/// Compute the closest point on the segment a-b to the target q.
export function b3PointToSegmentDistance(a: b3Vec3, b: b3Vec3, q: b3Vec3): b3Vec3 {
	let ab = b3Sub(b, a);
	let aq = b3Sub(q, a);

	let alpha = b3Dot(ab, aq);

	if (alpha <= 0.0) {
		// q projects outside interval [a, b] on the side of a
		return a;
	} else {
		let denominator = b3Dot(ab, ab);
		if (alpha > denominator) {
			// q projects outside interval [a, b] on the side of b
			return b;
		} else {
			// q projects inside interval [a, b]
			alpha /= denominator;
			return b3MulAdd(a, alpha, ab);
		}
	}
}

export function b3ClosestPointOnTriangle(a: b3Vec3, b: b3Vec3, c: b3Vec3, q: b3Vec3): b3TrianglePoint {
	// Check if P lies in vertex region of A
	let ab = b3Sub(b, a);
	let ac = b3Sub(c, a);
	let aq = b3Sub(q, a);

	let d1 = b3Dot(ab, aq);
	let d2 = b3Dot(ac, aq);
	if (d1 <= 0.0 && d2 <= 0.0) {
		return b3TrianglePoint(a, b3_featureVertex1);
	}

	// Check if P lies in vertex region of B
	let bq = b3Sub(q, b);

	let d3 = b3Dot(ab, bq);
	let d4 = b3Dot(ac, bq);
	if (d3 > 0.0 && d4 <= d3) {
		return b3TrianglePoint(b, b3_featureVertex2);
	}

	// Check if P lies in edge region AB
	let vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
		let t = d1 / (d1 - d3);
		return b3TrianglePoint(b3MulAdd(a, t, ab), b3_featureEdge1);
	}

	// Check if P lies in vertex region of C
	let cq = b3Sub(q, c);

	let d5 = b3Dot(ab, cq);
	let d6 = b3Dot(ac, cq);
	if (d6 >= 0.0 && d5 <= d6) {
		return b3TrianglePoint(c, b3_featureVertex3);
	}

	// Check if P lies in edge region AC
	let vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
		let t = d2 / (d2 - d6);
		return b3TrianglePoint(b3MulAdd(a, t, ac), b3_featureEdge3);
	}

	// Check if P lies in edge region of BC
	let va = d3 * d6 - d5 * d4;
	if (va <= 0.0 && d4 >= d3 && d5 >= d6) {
		let bc = b3Sub(c, b);

		let t = (d4 - d3) / (d4 - d3 + (d5 - d6));
		return b3TrianglePoint(b3MulAdd(b, t, bc), b3_featureEdge2);
	}

	// P inside face region ABC
	let t1 = vb / (va + vb + vc);
	let t2 = vc / (va + vb + vc);

	let p = b3MulAdd(a, t1, ab);
	p = b3MulAdd(p, t2, ac);
	return b3TrianglePoint(p, b3_featureTriangleFace);
}

export function b3SphereInertia(mass: f32, radius: f32): b3Matrix3 {
	let i: f32 = 0.4 * mass * radius * radius;
	return b3MakeDiagonalMatrix(i, i, i);
}

export function b3CylinderInertia(mass: f32, radius: f32, height: f32): b3Matrix3 {
	let ixx: f32 = (mass * (3 * radius * radius + height * height)) / 12.0;
	let iyy: f32 = 0.5 * mass * radius * radius;
	return b3MakeDiagonalMatrix(ixx, iyy, ixx);
}

export function b3BoxInertia(mass: f32, min: b3Vec3, max: b3Vec3): b3Matrix3 {
	let delta = b3Sub(max, min);
	let ixx: f32 = (mass * (delta.y * delta.y + delta.z * delta.z)) / 12.0;
	let iyy: f32 = (mass * (delta.x * delta.x + delta.z * delta.z)) / 12.0;
	let izz: f32 = (mass * (delta.x * delta.x + delta.y * delta.y)) / 12.0;

	return b3MakeDiagonalMatrix(ixx, iyy, izz);
}

/// Get the inertia tensor of an offset point.
// https://en.wikipedia.org/wiki/Parallel_axis_theorem
export function b3Steiner(mass: f32, origin: b3Vec3): b3Matrix3 {
	// Usage: Io = Ic + Is and Ic = Io - Is
	let ixx = mass * (origin.y * origin.y + origin.z * origin.z);
	let iyy = mass * (origin.x * origin.x + origin.z * origin.z);
	let izz = mass * (origin.x * origin.x + origin.y * origin.y);
	let ixy = -mass * origin.x * origin.y;
	let ixz = -mass * origin.x * origin.z;
	let iyz = -mass * origin.y * origin.z;

	// Write
	let out: b3Matrix3;
	out.cx.x = ixx;
	out.cy.x = ixy;
	out.cz.x = ixz;
	out.cx.y = ixy;
	out.cy.y = iyy;
	out.cz.y = iyz;
	out.cx.z = ixz;
	out.cy.z = iyz;
	out.cz.z = izz;

	return out;
}

/// Use this to ensure your ray cast input is valid and avoid internal assertions.
export function b3IsValidRay(input: b3RayCastInput): bool {
	let isValid =
		b3IsValidVec3(input.origin) &&
		b3IsValidVec3(input.translation) &&
		b3IsValidFloat(input.maxFraction) &&
		0.0 <= input.maxFraction &&
		input.maxFraction < B3_HUGE();
	return isValid;
}
