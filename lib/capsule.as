// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/capsule.c

import { B3_ASSERT, B3_ASSERT_ENABLED, FLT_EPSILON, FLT_MIN } from "./core";
import { B3_LINEAR_SLOP, B3_OVERLAP_SLOP } from "./constants";
import {
	B3_PI,
	b3AABB,
	b3Add,
	b3AddMM,
	b3ClampFloat,
	b3ComputeQuatBetweenUnitVectors,
	b3CylinderInertia,
	b3Distance,
	b3Dot,
	b3GetLengthAndNormalize,
	b3InvMulTransforms,
	b3IsValidRay,
	b3LengthSquared,
	b3MakeMatrixFromQuat,
	b3Mat3_identity,
	b3Max,
	b3Min,
	b3MulAdd,
	b3MulMM,
	b3MulSV,
	b3MulSub,
	b3Normalize,
	b3Perp,
	b3Plane,
	b3SegmentDistance,
	b3SphereInertia,
	b3Sub,
	b3Transform,
	b3TransformPoint,
	b3Transform_identity,
	b3Transpose,
	b3Vec3,
	b3Vec3_axisY,
} from "./math_functions";
import {
	b3Capsule,
	b3CastOutput,
	b3DistanceInput,
	b3MassData,
	b3PlaneResult,
	b3RayCastInput,
	b3ShapeCastInput,
	b3ShapeCastPairInput,
	b3ShapeProxy,
	b3SimplexCache,
	b3Sphere,
} from "./collision";
import { b3SetProxy, b3ShapeCast, b3ShapeDistance } from "./distance";
import { b3RayCastSphere } from "./sphere";

/// Compute mass properties of a capsule
export function b3ComputeCapsuleMass(shape: b3Capsule, density: f32): b3MassData {
	let c1 = shape.center1;
	let c2 = shape.center2;
	let r = shape.radius;

	// Cylinder
	let cylinderHeight = b3Distance(c1, c2);
	let cylinderVolume = B3_PI * r * r * cylinderHeight;
	let cylinderMass = cylinderVolume * density;

	// Sphere
	let sphereVolume = (<f32>4.0 / <f32>3.0) * B3_PI * r * r * r;
	let sphereMass = sphereVolume * density;

	// Local accumulated inertia
	let inertia = b3AddMM(b3CylinderInertia(cylinderMass, r, cylinderHeight), b3SphereInertia(sphereMass, r));

	let steiner: f32 = 0.125 * sphereMass * (3.0 * r + 2.0 * cylinderHeight) * cylinderHeight;
	inertia.cx.x += steiner;
	inertia.cz.z += steiner;

	// Align capsule axis with chosen up-axis
	let rotation = b3Mat3_identity;
	if (cylinderHeight * cylinderHeight > 1000.0 * FLT_MIN) {
		let direction = b3Normalize(b3Sub(c2, c1));
		let q = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisY, direction);
		rotation = b3MakeMatrixFromQuat(q);
	}

	let mass = sphereMass + cylinderMass;
	let center = b3MulSV(0.5, b3Add(c1, c2));

	let out: b3MassData;
	out.mass = mass;
	out.center = center;

	// Rotate the central inertia into the shape frame
	out.inertia = b3MulMM(rotation, b3MulMM(inertia, b3Transpose(rotation)));

	return out;
}

/// Compute the bounding box of a transformed capsule
export function b3ComputeCapsuleAABB(shape: b3Capsule, transform: b3Transform): b3AABB {
	let r = shape.radius;

	let center1 = b3TransformPoint(transform, shape.center1);
	let center2 = b3TransformPoint(transform, shape.center2);
	let extent = b3Vec3(r, r, r);

	let aabb: b3AABB;
	aabb.lowerBound = b3Sub(b3Min(center1, center2), extent);
	aabb.upperBound = b3Add(b3Max(center1, center2), extent);
	return aabb;
}

export function b3ComputeSweptCapsuleAABB(shape: b3Capsule, xf1: b3Transform, xf2: b3Transform): b3AABB {
	let r = b3Vec3(shape.radius, shape.radius, shape.radius);
	let a = b3TransformPoint(xf1, shape.center1);
	let b = b3TransformPoint(xf1, shape.center2);
	let c = b3TransformPoint(xf2, shape.center1);
	let d = b3TransformPoint(xf2, shape.center2);

	let aabb: b3AABB;
	aabb.lowerBound = b3Sub(b3Min(b3Min(a, b), b3Min(c, d)), r);
	aabb.upperBound = b3Add(b3Max(b3Max(a, b), b3Max(c, d)), r);
	return aabb;
}

// C builds these inputs on its stack. The overlap and cast below never nest, so they share one set.
const s_overlapInput = new b3DistanceInput();
const s_castInput = new b3ShapeCastPairInput();
const s_capsulePoints = new StaticArray<b3Vec3>(2);
const s_capsuleProxy = new b3ShapeProxy();

/// Overlap shape versus capsule
export function b3OverlapCapsule(shape: b3Capsule, shapeTransform: b3Transform, proxy: b3ShapeProxy): bool {
	let input = s_overlapInput;
	s_capsulePoints[0] = shape.center1;
	s_capsulePoints[1] = shape.center2;
	input.proxyA = b3SetProxy(s_capsuleProxy, s_capsulePoints, 2, shape.radius);
	input.proxyB = proxy;
	input.transform = b3InvMulTransforms(shapeTransform, b3Transform_identity);
	input.useRadii = true;

	let cache = b3SimplexCache();
	let output = b3ShapeDistance(input, cache, null, 0);
	return output.distance < B3_OVERLAP_SLOP();
}

// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
/// Ray cast versus capsule in local space. A zero length ray is a point query. Initial overlap
/// reports a hit at the ray origin with zero fraction and zero normal.
export function b3RayCastCapsule(shape: b3Capsule, input: b3RayCastInput): b3CastOutput {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidRay(input));

	let c1 = shape.center1;
	let c2 = shape.center2;
	let r = shape.radius;

	// Initialize result structure
	let output = b3CastOutput();

	let d = b3Sub(c2, c1);

	// Fall back to sphere if the capsule is short
	let tol: f32 = 0.01 * B3_LINEAR_SLOP();
	let lengthSquared = b3LengthSquared(d);
	if (lengthSquared < tol * tol) {
		let sphereCenter = b3MulSV(0.5, b3Add(shape.center1, shape.center2));
		let sphere = b3Sphere(sphereCenter, shape.radius);
		return b3RayCastSphere(sphere, input);
	}

	// Vector from first center to ray origin.
	let s = b3Sub(input.origin, c1);

	// Capsule axis
	let length = Mathf.sqrt(lengthSquared);
	let axis = b3MulSV(1.0 / length, d);

	// Project ray origin onto capsule axis.
	let u = b3Dot(s, axis);

	// Closest point on infinite capsule axis, relative to c1.
	let c = b3MulSV(u, axis);

	// Vector from closest point to ray origin
	let sc = b3Sub(s, c);

	// Squared distance from ray origin to capsule axis
	let sc2 = b3LengthSquared(sc);

	// Is the ray origin within the infinite cylinder along the capsule axis?
	if (sc2 < r * r) {
		// Clamped barycentric coordinate of ray origin projected onto capsule axis.
		let uClamped = b3ClampFloat(u, 0.0, length);

		// The closest point on the bounded capsule segment, relative to c1.
		let cp = b3MulSV(uClamped, axis);

		// Vector from ray origin to closest point on segment.
		let scp = b3Sub(s, cp);

		// Squared distance of ray origin from capsule segment.
		let scp2 = b3LengthSquared(scp);

		// Is the ray origin within the capsule?
		if (scp2 < r * r) {
			output.hit = true;
			output.point = input.origin;
			return output;
		}

		// The ray can hit an endcap.
		let sphere = b3Sphere(b3Add(c1, cp), r);

		return b3RayCastSphere(sphere, input);
	}

	// Ray axis. A zero length ray reaching here starts outside the capsule, so it misses.
	// Same zero length convention as b3RayCastSphere.
	let dr = input.translation;
	let rayLength: FixedArray<f32, 1>;
	let rayAxis = b3GetLengthAndNormalize(rayLength[0], dr);
	if (rayLength[0] == 0.0) {
		return output;
	}

	// Barycentric coordinate of ray end point.
	let v = u + input.maxFraction * b3Dot(dr, axis);

	// Early out: does the projected ray fall outside the capsule?
	if ((u < -r && v < -r) || (length + r < u && length + r < v)) {
		return output;
	}

	// Compute the closest point between the ray segment and the capsule segment.
	// See Real-Time Collision Detection, section 5.1.9

	// Closest point on capsule : a1 = segment unit axis, t1 = unknown fraction
	// p1 = t1 * a1

	// Closet point on ray : a2 = ray unit axis, t2 = unknown fraction
	// p2 = s + t2 * a2

	// Closest point perpendicularity conditions.
	// dot(p2 - p1, a1) = 0
	// dot(p2 - p1, a2) = 0

	// Expand
	// dot(t1 * a1 - s - t2 * a2, a1) = 0
	// dot(t1 * a1 - s - t2 * a2, a2) = 0

	// Expand
	// t1 - dot(s, a1) - t2 * dot(a1, a2) = 0
	// t1 * dot(a1, a2) - dot(s, a2) - t2 = 0

	// Group : a12 = dot(a1, a2), sa1 = dot(s, a1), sa2 = dot(s, a2)
	// t1       - a12 * t2 = sa1
	// a12 * t1 -       t2 = sa2

	// Solve
	// https://en.wikipedia.org/wiki/Cramer%27s_rule
	// I've flipped the signs of the numerator and denominator to give a positive determinant.
	// det = 1 - a12 * a12
	// t1 = (sa1 - a12 * sa2) / det
	// t2 = (a12 * sa1 - sa2) / det

	let a1 = axis;
	let a2 = rayAxis;
	let a12 = b3Dot(a1, a2);

	// Ray distance to the near intersection with the infinite cylinder. Length units.
	let tr: f32;

	let det: f32 = 1.0 - a12 * a12;
	if (det < FLT_EPSILON) {
		// Solve the 2D problem of ray versus circle starting at the ray origin, where the circle is
		// the axial view of the infinite capsule cylinder. This works well when the ray origin is
		// not too far from the capsule axis.

		// Instead of a cross product, subtract the parallel part to get a perpendicular vector. Non-dimensional.
		let perp = b3MulSub(a2, a12, a1);
		let perp2 = b3LengthSquared(perp);

		// Project to origin to infinite capsule axis vector onto the perpendicular vector. beta has length units.
		let beta = b3Dot(sc, perp);

		// Setup quadratic root finder.
		let gamma = sc2 - r * r;

		// Discriminant
		let disc = beta * beta - perp2 * gamma;

		// Casting away from the axis, or the perpendicular gap never closes to the radius.
		if (beta >= 0.0 || disc < 0.0) {
			return output;
		}

		// Quadratic near root. Expressed in an alternate form to avoid the (-beta - sqrt) cancellation as
		// the ray nears parallel.
		tr = gamma / (-beta + Mathf.sqrt(disc));
	} else {
		// Ray and capsules axes are not parallel.

		// Closest points between the infinite ray and the infinite capsule axis.
		let invDet: f32 = 1.0 / det;
		let sa1 = u;
		let sa2 = b3Dot(s, a2);

		let t1 = (sa1 - a12 * sa2) * invDet;
		let t2 = (a12 * sa1 - sa2) * invDet;

		// Closest points
		let p1 = b3MulSV(t1, a1);
		let p2 = b3MulAdd(s, t2, a2);

		// Vector from closest point on infinite capsule to infinite ray.
		let g = b3Sub(p2, p1);

		let g2 = b3LengthSquared(g);
		if (g2 > r * r) {
			// Early out: closest point on infinite ray is outside infinite cylinder.
			return output;
		}

		// Intersect the infinite ray with the infinite cylinder. Like ray versus sphere this is done
		// relative to the closest point to avoid round-off errors. Length units, not a fraction.
		// https://en.wikipedia.org/wiki/Line-cylinder_intersection
		let h = Mathf.sqrt((r * r - g2) * invDet);

		tr = t2 - h;
	}

	// Outside ray?
	if (tr < 0.0 || input.maxFraction * rayLength[0] < tr) {
		return output;
	}

	// The corresponding distance on the capsule axis. Length units.
	let tc = u + tr * a12;

	// Outside c1 end?
	if (tc < 0.0) {
		// Ray cast sphere 1.
		let sphere = b3Sphere(c1, r);

		return b3RayCastSphere(sphere, input);
	}

	// Outside c2 end?
	if (length < tc) {
		// Ray cast sphere 2.
		let sphere = b3Sphere(c2, r);

		return b3RayCastSphere(sphere, input);
	}

	// Hit point on capsule side, relative to c1.
	let p = b3MulAdd(s, tr, rayAxis);

	// Hit normal.
	let normal = b3MulSub(p, tc, axis);
	normal = b3Normalize(normal);

	output.point = b3Add(c1, p);
	output.normal = normal;
	output.fraction = b3ClampFloat(tr / rayLength[0], 0.0, input.maxFraction);
	output.hit = true;
	return output;
}

/// Shape cast versus a capsule. Initial overlap is treated as a miss.
export function b3ShapeCastCapsule(capsule: b3Capsule, input: b3ShapeCastInput): b3CastOutput {
	let pairInput = s_castInput;
	s_capsulePoints[0] = capsule.center1;
	s_capsulePoints[1] = capsule.center2;
	pairInput.proxyA = b3SetProxy(s_capsuleProxy, s_capsulePoints, 2, capsule.radius);
	pairInput.proxyB = input.proxy;
	pairInput.transform = b3Transform_identity;
	pairInput.translationB = input.translation;
	pairInput.maxFraction = input.maxFraction;
	pairInput.canEncroach = input.canEncroach;

	let output = b3ShapeCast(pairInput);
	return output;
}

export function b3CollideMoverAndCapsule(result: ref b3PlaneResult, shape: b3Capsule, mover: b3Capsule): i32 {
	let totalRadius = mover.radius + shape.radius;

	let approach = b3SegmentDistance(shape.center1, shape.center2, mover.center1, mover.center2);

	// The normal points from the shape toward the mover.
	let distance: FixedArray<f32, 1>;
	let normal = b3GetLengthAndNormalize(distance[0], b3Sub(approach.point2, approach.point1));

	if (distance[0] > totalRadius) {
		return 0;
	}

	let linearSlop = B3_LINEAR_SLOP();
	if (distance[0] < linearSlop) {
		// Deep overlap: the core segments intersect. Pick an arbitrary direction perpendicular
		// the to capsule axis.
		let moverLength: FixedArray<f32, 1>;
		let moverAxis = b3GetLengthAndNormalize(moverLength[0], b3Sub(mover.center2, mover.center1));
		normal = moverLength[0] > linearSlop ? b3Perp(moverAxis) : b3Vec3_axisY;
		distance[0] = 0.0;
	}

	let plane = b3Plane(normal, totalRadius - distance[0]);
	result = b3PlaneResult();
	result.plane = plane;
	result.point = approach.point1;
	return 1;
}
