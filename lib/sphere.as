// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/sphere.c

// Dirk Gregorius contributed portions of this code

import { B3_ASSERT, B3_ASSERT_ENABLED, b3Log } from "./core";
import { B3_LINEAR_SLOP, B3_OVERLAP_SLOP } from "./constants";
import {
	B3_PI,
	b3AABB,
	b3Add,
	b3Dot,
	b3GetLengthAndNormalize,
	b3InvMulTransforms,
	b3IsValidRay,
	b3LengthSquared,
	b3Max,
	b3Min,
	b3MulAdd,
	b3Normalize,
	b3Perp,
	b3Plane,
	b3PointToSegmentDistance,
	b3Sub,
	b3Transform,
	b3TransformPoint,
	b3Transform_identity,
	b3Vec3,
	b3Vec3_axisY,
} from "./math_functions";
import { b3MakeDiagonalMatrix } from "./math_internal";
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
import { b3MakePointProxy, b3ShapeCast, b3ShapeDistance } from "./distance";

/// Compute mass properties of a sphere
export function b3ComputeSphereMass(shape: b3Sphere, density: f32): b3MassData {
	let center = shape.center;
	let radius = shape.radius;

	let volume = (<f32>4.0 / <f32>3.0) * B3_PI * radius * radius * radius;
	let mass = volume * density;
	let ixx: f32 = 0.4 * mass * radius * radius;

	let out: b3MassData;
	out.mass = mass;
	out.center = center;

	// Inertia about the center of mass
	out.inertia = b3MakeDiagonalMatrix(ixx, ixx, ixx);
	return out;
}

/// Compute the bounding box of a transformed sphere
export function b3ComputeSphereAABB(shape: b3Sphere, transform: b3Transform): b3AABB {
	let center = b3TransformPoint(transform, shape.center);
	let radius = shape.radius;
	let extent = b3Vec3(radius, radius, radius);
	return b3AABB(b3Sub(center, extent), b3Add(center, extent));
}

export function b3ComputeSweptSphereAABB(shape: b3Sphere, xf1: b3Transform, xf2: b3Transform): b3AABB {
	let r = b3Vec3(shape.radius, shape.radius, shape.radius);
	let center1 = b3TransformPoint(xf1, shape.center);
	let center2 = b3TransformPoint(xf2, shape.center);
	return b3AABB(b3Sub(b3Min(center1, center2), r), b3Add(b3Max(center1, center2), r));
}

/// Overlap shape versus sphere
export function b3OverlapSphere(shape: b3Sphere, shapeTransform: b3Transform, proxy: b3ShapeProxy): bool {
	let input = new b3DistanceInput();
	input.proxyA = b3MakePointProxy(shape.center, shape.radius);
	input.proxyB = proxy;
	input.transform = b3InvMulTransforms(shapeTransform, b3Transform_identity);
	input.useRadii = true;

	let cache = b3SimplexCache();
	let output = b3ShapeDistance(input, cache, null, 0);
	return output.distance < B3_OVERLAP_SLOP();
}

// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
/// Ray cast versus sphere in local space. A zero length ray is a point query. Initial overlap
/// reports a hit at the ray origin with zero fraction and zero normal.
export function b3RayCastSphere(shape: b3Sphere, input: b3RayCastInput): b3CastOutput {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidRay(input));
	let output = b3CastOutput();

	let p = shape.center;

	// Shift ray so sphere center is the origin
	let s = b3Sub(input.origin, p);

	let r = shape.radius;
	let rr = r * r;

	let length: FixedArray<f32, 1>;
	let d = b3GetLengthAndNormalize(length[0], input.translation);
	if (length[0] == 0.0) {
		// zero length ray

		if (b3LengthSquared(s) < rr) {
			// initial overlap
			output.point = input.origin;
			output.hit = true;
		}

		return output;
	}

	// Find closest point on ray to origin

	// solve: dot(s + t * d, d) = 0
	let t = -b3Dot(s, d);

	// c is the closest point on the line to the origin
	let c = b3MulAdd(s, t, d);

	let cc = b3Dot(c, c);

	if (cc > rr) {
		// closest point is outside the sphere
		return output;
	}

	// Pythagoras
	let h = Mathf.sqrt(rr - cc);

	let fraction = t - h;

	if (fraction < 0.0 || input.maxFraction * length[0] < fraction) {
		// intersection is point outside the range of the ray segment

		if (b3LengthSquared(s) < rr) {
			// initial overlap
			output.point = input.origin;
			output.hit = true;
		}

		return output;
	}

	let hitPoint = b3MulAdd(s, fraction, d);

	output.fraction = fraction / length[0];

	if (output.fraction > input.maxFraction) {
		b3Log("sphere input fraction = " + input.maxFraction.toString() + ", output fraction = " + output.fraction.toString());
		output.fraction = input.maxFraction;
	}

	output.normal = b3Normalize(hitPoint);
	output.point = b3MulAdd(p, shape.radius, output.normal);
	output.hit = true;

	return output;
}

// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
// This will do interior hits.
/// Ray cast versus a hollow sphere shell in local space. Unlike the solid sphere a ray starting
/// inside is not an overlap: it passes through and hits the far wall.
export function b3RayCastHollowSphere(sphere: b3Sphere, input: b3RayCastInput): b3CastOutput {
	let p = sphere.center;

	let output = b3CastOutput();

	// Shift ray so sphere center is the origin
	let s = b3Sub(input.origin, p);
	let d = b3Normalize(input.translation);

	// Find closest point on ray to origin

	// solve: dot(s + t * d, d) = 0
	let t = -b3Dot(s, d);

	// c is the closest point on the line to the origin
	let c = b3MulAdd(s, t, d);

	let cc = b3Dot(c, c);
	let r = sphere.radius;
	let rr = r * r;

	if (cc > rr) {
		// closest point is outside the sphere
		return output;
	}

	// Pythagoras
	let h = Mathf.sqrt(rr - cc);

	let fraction = t - h;

	if (fraction < 0.0) {
		fraction = t + h;
	}

	if (fraction < 0.0) {
		// behind the ray
		return output;
	}

	if (fraction > input.maxFraction) {
		return output;
	}

	let hitPoint = b3MulAdd(s, fraction, d);

	output.fraction = fraction;
	output.normal = b3Normalize(hitPoint);
	output.point = b3MulAdd(p, sphere.radius, output.normal);
	output.hit = true;

	return output;
}

/// Shape cast versus a sphere. Initial overlap is treated as a miss.
export function b3ShapeCastSphere(sphere: b3Sphere, input: b3ShapeCastInput): b3CastOutput {
	let pairInput = new b3ShapeCastPairInput();
	pairInput.proxyA = b3MakePointProxy(sphere.center, sphere.radius);
	pairInput.proxyB = input.proxy;
	pairInput.transform = b3Transform_identity;
	pairInput.translationB = input.translation;
	pairInput.maxFraction = input.maxFraction;
	pairInput.canEncroach = input.canEncroach;

	let output = b3ShapeCast(pairInput);
	return output;
}

export function b3CollideMoverAndSphere(result: ref b3PlaneResult, shape: b3Sphere, mover: b3Capsule): i32 {
	let totalRadius = mover.radius + shape.radius;
	let closest = b3PointToSegmentDistance(mover.center1, mover.center2, shape.center);

	// The normal points from the sphere toward the mover.
	let distance: FixedArray<f32, 1>;
	let normal = b3GetLengthAndNormalize(distance[0], b3Sub(closest, shape.center));

	if (distance[0] > totalRadius) {
		return 0;
	}

	let linearSlop = B3_LINEAR_SLOP();
	if (distance[0] < linearSlop) {
		// Deep overlap: the mover axis passes through the sphere center, so no
		// direction is preferred. Push perpendicular to the mover axis.
		let length: FixedArray<f32, 1>;
		let axis = b3GetLengthAndNormalize(length[0], b3Sub(mover.center2, mover.center1));
		normal = length[0] > linearSlop ? b3Perp(axis) : b3Vec3_axisY;
		distance[0] = 0.0;
	}

	let plane = b3Plane(normal, totalRadius - distance[0]);
	result = b3PlaneResult();
	result.plane = plane;
	result.point = shape.center;
	return 1;
}
