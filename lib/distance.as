// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/distance.c

// Dirk Gregorius contributed portions of this code

import { B3_ASSERT, B3_ASSERT_ENABLED, FLT_EPSILON, FLT_MAX, FLT_MIN, B3_NULL_INDEX, b3Log } from "./core";
import { B3_LINEAR_SLOP } from "./constants";
import {
	b3AbsFloat,
	b3Add,
	b3Blend2,
	b3Cross,
	b3Distance,
	b3Dot,
	b3InvMulTransforms,
	b3InvRotateVector,
	b3IsNormalized,
	b3Length,
	b3LengthSquared,
	b3Lerp,
	b3MakeMatrixFromQuat,
	b3MaxFloat,
	b3MulAdd,
	b3MulMV,
	b3MulSV,
	b3NLerp,
	b3Neg,
	b3Normalize,
	b3RotateVector,
	b3Sub,
	b3Transform,
	b3TransformPoint,
	b3Transpose,
	b3Vec3,
	b3Vec3_zero,
} from "./math_functions";
import { b3Blend3, b3ScalarTripleProduct } from "./math_internal";
import {
	b3CastOutput,
	b3DistanceInput,
	b3DistanceOutput,
	b3ShapeCastPairInput,
	b3ShapeProxy,
	b3Simplex,
	b3SimplexCache,
	b3Sweep,
	b3TOIInput,
	b3TOIOutput,
	b3_toiStateFailed,
	b3_toiStateHit,
	b3_toiStateOverlapped,
	b3_toiStateSeparated,
	b3_toiStateUnknown,
} from "./collision";

const B3_MAX_SIMPLEX_VERTICES: i32 = 4;
const B3_MAX_GJK_ITERATIONS: i32 = 32;

export function b3GetProxySupport(proxy: b3ShapeProxy, axis: b3Vec3): i32 {
	let count = proxy.count;
	let points = proxy.points;

	if (B3_ASSERT_ENABLED) B3_ASSERT(count > 0);

	// We move the first vertex into the origin for improved precision.
	// This is necessary since we don't have shape transforms and
	// vertices can potentially be far away from the origin (large).
	let origin = points[0];
	let maxIndex = 0;
	let maxProjection: f32 = 0.0;

	for (let index = 1; index < count; ++index) {
		// We subtract the first vertex since we are shifting into the origin.
		let projection = b3Dot(axis, b3Sub(points[index], origin));
		if (projection > maxProjection) {
			maxIndex = index;
			maxProjection = projection;
		}
	}

	return maxIndex;
}

export function b3GetPointSupport(points: StaticArray<b3Vec3>, count: i32, axis: b3Vec3): i32 {
	if (B3_ASSERT_ENABLED) B3_ASSERT(count > 0);

	// We move the first vertex into the origin for improved precision.
	// This is necessary since we don't have shape transforms and
	// vertices can potentially be far away from the origin (large).
	let origin = points[0];
	let maxIndex = 0;
	let maxProjection: f32 = 0.0;

	for (let index = 1; index < count; ++index) {
		// We subtract the first vertex since we are shifting into the origin.
		let projection = b3Dot(axis, b3Sub(points[index], origin));
		if (projection > maxProjection) {
			maxIndex = index;
			maxProjection = projection;
		}
	}

	return maxIndex;
}

function b3BarycentricCoordsEdge(a: b3Vec3, b: b3Vec3): FixedArray<f32, 3> {
	let out: FixedArray<f32, 3>;
	let ab = b3Sub(b, a);

	// Last element is divisor
	let divisor = b3Dot(ab, ab);

	out[0] = b3Dot(b, ab);
	out[1] = -b3Dot(a, ab);
	out[2] = divisor;
	return out;
}

function b3BarycentricCoordsTet(a: b3Vec3, b: b3Vec3, c: b3Vec3, d: b3Vec3): FixedArray<f32, 5> {
	let out: FixedArray<f32, 5>;
	let ab = b3Sub(b, a);
	let ac = b3Sub(c, a);
	let ad = b3Sub(d, a);

	// Last element is divisor (forced to be positive)
	let divisor = b3ScalarTripleProduct(ab, ac, ad);

	let sign: f32 = divisor < 0.0 ? -1.0 : 1.0;
	out[0] = sign * b3ScalarTripleProduct(b, c, d);
	out[1] = sign * b3ScalarTripleProduct(a, d, c);
	out[2] = sign * b3ScalarTripleProduct(a, b, d);
	out[3] = sign * b3ScalarTripleProduct(a, c, b);
	out[4] = sign * divisor;
	return out;
}

@inline function b3GetMetric(simplex: ref b3Simplex): f32 {
	let count = simplex.count;
	if (B3_ASSERT_ENABLED) B3_ASSERT(1 <= count && count <= 4);

	switch (count) {
		case 1: {
			return 0.0;
		}

		case 2: {
			let a = simplex.vertices[0].w;
			let b = simplex.vertices[1].w;
			return b3Distance(a, b);
		}

		case 3: {
			let a = simplex.vertices[0].w;
			let b = simplex.vertices[1].w;
			let c = simplex.vertices[2].w;
			return b3Length(b3Cross(b3Sub(b, a), b3Sub(c, a))) / 2.0;
		}

		case 4: {
			let a = simplex.vertices[0].w;
			let b = simplex.vertices[1].w;
			let c = simplex.vertices[2].w;
			let d = simplex.vertices[3].w;
			return b3ScalarTripleProduct(b3Sub(b, a), b3Sub(c, a), b3Sub(d, a)) / 6.0;
		}

		default:
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			break;
	}

	return 0.0;
}

function b3WriteCache(cache: ref b3SimplexCache, simplex: ref b3Simplex): void {
	let count = simplex.count;
	cache.metric = b3GetMetric(simplex);
	cache.count = <u16>count;
	for (let index = 0; index < count; ++index) {
		cache.indexA[index] = <u8>simplex.vertices[index].indexA;
		cache.indexB[index] = <u8>simplex.vertices[index].indexB;
	}
}

function b3SolveSimplex2(simplex: ref b3Simplex): bool {
	if (B3_ASSERT_ENABLED) B3_ASSERT(simplex.count == 2);

	// Vertex regions
	//float wAB[3];

	let a = simplex.vertices[0].w;
	let b = simplex.vertices[1].w;
	let ab = b3Sub(b, a);

	// Last element is divisor
	let divisor = b3Dot(ab, ab);

	let u = b3Dot(b, ab);
	let v = -b3Dot(a, ab);
	//wAB[2] = divisor;

	// V( A )
	if (v <= 0.0) {
		// Reduce simplex
		simplex.count = 1;
		simplex.vertices[0].a = 1.0;

		return true;
	}

	// V( B )
	if (u <= 0.0) {
		// Reduce simplex
		simplex.count = 1;
		simplex.vertices[0] = simplex.vertices[1];
		simplex.vertices[0].a = 1.0;

		return true;
	}

	// Edge region
	if (divisor <= 0.0) {
		return false;
	}

	// VR( AB )
	let denominator: f32 = 1.0 / divisor;
	simplex.vertices[0].a = denominator * u;
	simplex.vertices[1].a = denominator * v;

	return true;
}

function b3SolveSimplex3(simplex: ref b3Simplex): bool {
	if (B3_ASSERT_ENABLED) B3_ASSERT(simplex.count == 3);

	// Get simplex (be aware of aliasing here!)
	let v1 = simplex.vertices[0];
	let v2 = simplex.vertices[1];
	let v3 = simplex.vertices[2];

	// Vertex regions
	let wAB = b3BarycentricCoordsEdge(v1.w, v2.w);
	let wBC = b3BarycentricCoordsEdge(v2.w, v3.w);
	let wCA = b3BarycentricCoordsEdge(v3.w, v1.w);

	// VR( A )
	if (wAB[1] <= 0.0 && wCA[0] <= 0.0) {
		// Reduce simplex
		simplex.count = 1;
		simplex.vertices[0] = v1;
		simplex.vertices[0].a = 1.0;

		return true;
	}

	// VR( B )
	if (wBC[1] <= 0.0 && wAB[0] <= 0.0) {
		// Reduce simplex
		simplex.count = 1;
		simplex.vertices[0] = v2;
		simplex.vertices[0].a = 1.0;

		return true;
	}

	// VR( C )
	if (wCA[1] <= 0.0 && wBC[0] <= 0.0) {
		// Reduce simplex
		simplex.count = 1;
		simplex.vertices[0] = v3;
		simplex.vertices[0].a = 1.0;

		return true;
	}

	// Edge regions
	// wABC = b3BarycentricCoordsTri( v1.w, v2.w, v3.w ) written out: with no calls left in
	// b3ShapeDistance, WAMR keeps the memory base and bound in registers there (notes/perf_3.md)
	let wABC: FixedArray<f32, 4>;
	{
		let ab = b3Sub(v2.w, v1.w);
		let ac = b3Sub(v3.w, v1.w);
		let abXAc = b3Cross(ab, ac);
		wABC[0] = b3Dot(b3Cross(v2.w, v3.w), abXAc);
		wABC[1] = b3Dot(b3Cross(v3.w, v1.w), abXAc);
		wABC[2] = b3Dot(b3Cross(v1.w, v2.w), abXAc);
		wABC[3] = b3Dot(abXAc, abXAc);
	}

	// VR( AB )
	if (wABC[2] <= 0.0 && wAB[0] > 0.0 && wAB[1] > 0.0) {
		// Reduce simplex
		simplex.count = 2;
		simplex.vertices[0] = v1;
		simplex.vertices[1] = v2;

		// Normalize
		let divisor = wAB[2];
		if (divisor <= 0.0) {
			return false;
		}

		simplex.vertices[0].a = wAB[0] / divisor;
		simplex.vertices[1].a = wAB[1] / divisor;

		return true;
	}

	// VR( BC )
	if (wABC[0] <= 0.0 && wBC[0] > 0.0 && wBC[1] > 0.0) {
		// Reduce simplex
		simplex.count = 2;
		simplex.vertices[0] = v2;
		simplex.vertices[1] = v3;

		// Normalize
		let divisor = wBC[2];
		if (divisor <= 0.0) {
			return false;
		}

		simplex.vertices[0].a = wBC[0] / divisor;
		simplex.vertices[1].a = wBC[1] / divisor;

		return true;
	}

	// VR( CA )
	if (wABC[1] <= 0.0 && wCA[0] > 0.0 && wCA[1] > 0.0) {
		// Reduce simplex
		simplex.count = 2;
		simplex.vertices[0] = v3;
		simplex.vertices[1] = v1;

		// Normalize
		let divisor = wCA[2];
		if (divisor <= 0.0) {
			return false;
		}

		simplex.vertices[0].a = wCA[0] / divisor;
		simplex.vertices[1].a = wCA[1] / divisor;

		return true;
	}

	// Face region
	let divisor = wABC[3];
	if (divisor <= 0.0) {
		return false;
	}

	// VR( ABC )
	simplex.vertices[0].a = wABC[0] / divisor;
	simplex.vertices[1].a = wABC[1] / divisor;
	simplex.vertices[2].a = wABC[2] / divisor;

	return true;
}

function b3SolveSimplex4(simplex: ref b3Simplex): bool {
	// Get simplex (be aware of aliasing here!)
	if (B3_ASSERT_ENABLED) B3_ASSERT(simplex.count == 4);
	let vertexA = simplex.vertices[0];
	let vertexB = simplex.vertices[1];
	let vertexC = simplex.vertices[2];
	let vertexD = simplex.vertices[3];

	// Vertex region
	let wAB = b3BarycentricCoordsEdge(vertexA.w, vertexB.w);
	let wAC = b3BarycentricCoordsEdge(vertexA.w, vertexC.w);
	let wAD = b3BarycentricCoordsEdge(vertexA.w, vertexD.w);
	let wBC = b3BarycentricCoordsEdge(vertexB.w, vertexC.w);
	let wCD = b3BarycentricCoordsEdge(vertexC.w, vertexD.w);
	let wDB = b3BarycentricCoordsEdge(vertexD.w, vertexB.w);

	// VR( A )
	if (wAB[1] <= 0.0 && wAC[1] <= 0.0 && wAD[1] <= 0.0) {
		// Reduce simplex
		simplex.count = 1;
		simplex.vertices[0] = vertexA;

		simplex.vertices[0].a = 1.0;

		return true;
	}

	// VR( B )
	if (wAB[0] <= 0.0 && wDB[0] <= 0.0 && wBC[1] <= 0.0) {
		// Reduce simplex
		simplex.count = 1;
		simplex.vertices[0] = vertexB;

		simplex.vertices[0].a = 1.0;

		return true;
	}

	// VR( C )
	if (wAC[0] <= 0.0 && wBC[0] <= 0.0 && wCD[1] <= 0.0) {
		// Reduce simplex
		simplex.count = 1;
		simplex.vertices[0] = vertexC;

		simplex.vertices[0].a = 1.0;

		return true;
	}

	// VR( D )
	if (wAD[0] <= 0.0 && wCD[0] <= 0.0 && wDB[1] <= 0.0) {
		// Reduce simplex
		simplex.count = 1;
		simplex.vertices[0] = vertexD;

		simplex.vertices[0].a = 1.0;

		return true;
	}

	// Edge region
	// wACB = b3BarycentricCoordsTri( vertexA.w, vertexC.w, vertexB.w ) written out
	let wACB: FixedArray<f32, 4>;
	{
		let ab = b3Sub(vertexC.w, vertexA.w);
		let ac = b3Sub(vertexB.w, vertexA.w);
		let abXAc = b3Cross(ab, ac);
		wACB[0] = b3Dot(b3Cross(vertexC.w, vertexB.w), abXAc);
		wACB[1] = b3Dot(b3Cross(vertexB.w, vertexA.w), abXAc);
		wACB[2] = b3Dot(b3Cross(vertexA.w, vertexC.w), abXAc);
		wACB[3] = b3Dot(abXAc, abXAc);
	}
	// wABD = b3BarycentricCoordsTri( vertexA.w, vertexB.w, vertexD.w ) written out
	let wABD: FixedArray<f32, 4>;
	{
		let ab = b3Sub(vertexB.w, vertexA.w);
		let ac = b3Sub(vertexD.w, vertexA.w);
		let abXAc = b3Cross(ab, ac);
		wABD[0] = b3Dot(b3Cross(vertexB.w, vertexD.w), abXAc);
		wABD[1] = b3Dot(b3Cross(vertexD.w, vertexA.w), abXAc);
		wABD[2] = b3Dot(b3Cross(vertexA.w, vertexB.w), abXAc);
		wABD[3] = b3Dot(abXAc, abXAc);
	}
	// wADC = b3BarycentricCoordsTri( vertexA.w, vertexD.w, vertexC.w ) written out
	let wADC: FixedArray<f32, 4>;
	{
		let ab = b3Sub(vertexD.w, vertexA.w);
		let ac = b3Sub(vertexC.w, vertexA.w);
		let abXAc = b3Cross(ab, ac);
		wADC[0] = b3Dot(b3Cross(vertexD.w, vertexC.w), abXAc);
		wADC[1] = b3Dot(b3Cross(vertexC.w, vertexA.w), abXAc);
		wADC[2] = b3Dot(b3Cross(vertexA.w, vertexD.w), abXAc);
		wADC[3] = b3Dot(abXAc, abXAc);
	}
	// wBCD = b3BarycentricCoordsTri( vertexB.w, vertexC.w, vertexD.w ) written out
	let wBCD: FixedArray<f32, 4>;
	{
		let ab = b3Sub(vertexC.w, vertexB.w);
		let ac = b3Sub(vertexD.w, vertexB.w);
		let abXAc = b3Cross(ab, ac);
		wBCD[0] = b3Dot(b3Cross(vertexC.w, vertexD.w), abXAc);
		wBCD[1] = b3Dot(b3Cross(vertexD.w, vertexB.w), abXAc);
		wBCD[2] = b3Dot(b3Cross(vertexB.w, vertexC.w), abXAc);
		wBCD[3] = b3Dot(abXAc, abXAc);
	}

	// VR( AB )
	if (wABD[2] <= 0.0 && wACB[1] <= 0.0 && wAB[0] > 0.0 && wAB[1] > 0.0) {
		// Reduce simplex
		simplex.count = 2;
		simplex.vertices[0] = vertexA;
		simplex.vertices[1] = vertexB;

		// Normalize
		let divisor = wAB[2];
		if (divisor <= 0.0) {
			return false;
		}

		simplex.vertices[0].a = wAB[0] / divisor;
		simplex.vertices[1].a = wAB[1] / divisor;

		return true;
	}

	// VR( AC )
	if (wACB[2] <= 0.0 && wADC[1] <= 0.0 && wAC[0] > 0.0 && wAC[1] > 0.0) {
		// Reduce simplex
		simplex.count = 2;
		simplex.vertices[0] = vertexA;
		simplex.vertices[1] = vertexC;

		// Normalize
		let divisor = wAC[2];
		if (divisor <= 0.0) {
			return false;
		}

		simplex.vertices[0].a = wAC[0] / divisor;
		simplex.vertices[1].a = wAC[1] / divisor;

		return true;
	}

	// VR( AD )
	if (wADC[2] <= 0.0 && wABD[1] <= 0.0 && wAD[0] > 0.0 && wAD[1] > 0.0) {
		// Reduce simplex
		simplex.count = 2;
		simplex.vertices[0] = vertexA;
		simplex.vertices[1] = vertexD;

		// Normalize
		let divisor = wAD[2];
		if (divisor <= 0.0) {
			return false;
		}

		simplex.vertices[0].a = wAD[0] / divisor;
		simplex.vertices[1].a = wAD[1] / divisor;

		return true;
	}

	// VR( BC )
	if (wACB[0] <= 0.0 && wBCD[2] <= 0.0 && wBC[0] > 0.0 && wBC[1] > 0.0) {
		// Reduce simplex
		simplex.count = 2;
		simplex.vertices[0] = vertexB;
		simplex.vertices[1] = vertexC;

		// Normalize
		let divisor = wBC[2];
		if (divisor <= 0.0) {
			return false;
		}

		simplex.vertices[0].a = wBC[0] / divisor;
		simplex.vertices[1].a = wBC[1] / divisor;

		return true;
	}

	// VR( CD )
	if (wADC[0] <= 0.0 && wBCD[0] <= 0.0 && wCD[0] > 0.0 && wCD[1] > 0.0) {
		// Reduce simplex
		simplex.count = 2;
		simplex.vertices[0] = vertexC;
		simplex.vertices[1] = vertexD;

		// Normalize
		let divisor = wCD[2];
		if (divisor <= 0.0) {
			return false;
		}

		simplex.vertices[0].a = wCD[0] / divisor;
		simplex.vertices[1].a = wCD[1] / divisor;

		return true;
	}

	// VR( DB )
	if (wABD[0] <= 0.0 && wBCD[1] <= 0.0 && wDB[0] > 0.0 && wDB[1] > 0.0) {
		// Reduce simplex
		simplex.count = 2;
		simplex.vertices[0] = vertexD;
		simplex.vertices[1] = vertexB;

		// Normalize
		let divisor = wDB[2];
		if (divisor <= 0.0) {
			return false;
		}

		simplex.vertices[0].a = wDB[0] / divisor;
		simplex.vertices[1].a = wDB[1] / divisor;

		return true;
	}

	// Face regions
	let wABCD = b3BarycentricCoordsTet(vertexA.w, vertexB.w, vertexC.w, vertexD.w);

	// VR( ACB )
	if (wABCD[3] < 0.0 && wACB[0] > 0.0 && wACB[1] > 0.0 && wACB[2] > 0.0) {
		// Reduce simplex
		simplex.count = 3;
		simplex.vertices[0] = vertexA;
		simplex.vertices[1] = vertexC;
		simplex.vertices[2] = vertexB;

		// Normalize
		let divisor = wACB[3];
		if (divisor <= 0.0) {
			return false;
		}

		simplex.vertices[0].a = wACB[0] / divisor;
		simplex.vertices[1].a = wACB[1] / divisor;
		simplex.vertices[2].a = wACB[2] / divisor;

		return true;
	}

	// VR( ABD )
	if (wABCD[2] < 0.0 && wABD[0] > 0.0 && wABD[1] > 0.0 && wABD[2] > 0.0) {
		// Reduce simplex
		simplex.count = 3;
		simplex.vertices[0] = vertexA;
		simplex.vertices[1] = vertexB;
		simplex.vertices[2] = vertexD;

		// Normalize
		let divisor = wABD[3];
		if (divisor <= 0.0) {
			return false;
		}

		simplex.vertices[0].a = wABD[0] / divisor;
		simplex.vertices[1].a = wABD[1] / divisor;
		simplex.vertices[2].a = wABD[2] / divisor;

		return true;
	}

	// VR( ADC )
	if (wABCD[1] < 0.0 && wADC[0] > 0.0 && wADC[1] > 0.0 && wADC[2] > 0.0) {
		// Reduce simplex
		simplex.count = 3;
		simplex.vertices[0] = vertexA;
		simplex.vertices[1] = vertexD;
		simplex.vertices[2] = vertexC;

		// Normalize
		let divisor = wADC[3];
		if (divisor <= 0.0) {
			return false;
		}

		simplex.vertices[0].a = wADC[0] / divisor;
		simplex.vertices[1].a = wADC[1] / divisor;
		simplex.vertices[2].a = wADC[2] / divisor;

		return true;
	}

	// VR( BCD )
	if (wABCD[0] < 0.0 && wBCD[0] > 0.0 && wBCD[1] > 0.0 && wBCD[2] > 0.0) {
		// Reduce simplex
		simplex.count = 3;
		simplex.vertices[0] = vertexB;
		simplex.vertices[1] = vertexC;
		simplex.vertices[2] = vertexD;

		// Normalize
		let divisor = wBCD[3];
		if (divisor <= 0.0) {
			return false;
		}

		simplex.vertices[0].a = wBCD[0] / divisor;
		simplex.vertices[1].a = wBCD[1] / divisor;
		simplex.vertices[2].a = wBCD[2] / divisor;

		return true;
	}

	// *** Inside tetrahedron ***
	let divisor = wABCD[4];
	if (divisor <= 0.0) {
		return false;
	}

	// VR( ABCD )
	simplex.vertices[0].a = wABCD[0] / divisor;
	simplex.vertices[1].a = wABCD[1] / divisor;
	simplex.vertices[2].a = wABCD[2] / divisor;
	simplex.vertices[3].a = wABCD[3] / divisor;

	return true;
}

@inline function b3ComputeWitnessPoints(simplex: ref b3Simplex, vertexA: ref b3Vec3, vertexB: ref b3Vec3): void {
	let count = simplex.count;
	if (B3_ASSERT_ENABLED) B3_ASSERT(1 <= count && count <= 4);

	switch (count) {
		case 1:
			vertexA = simplex.vertices[0].wA;
			vertexB = simplex.vertices[0].wB;
			break;

		case 2: {
			let v0 = simplex.vertices[0];
			let v1 = simplex.vertices[1];
			vertexA = b3Blend2(v0.a, v0.wA, v1.a, v1.wA);
			vertexB = b3Blend2(v0.a, v0.wB, v1.a, v1.wB);
			break;
		}

		case 3: {
			let v0 = simplex.vertices[0];
			let v1 = simplex.vertices[1];
			let v2 = simplex.vertices[2];
			vertexA = b3Blend3(v0.a, v0.wA, v1.a, v1.wA, v2.a, v2.wA);
			vertexB = b3Blend3(v0.a, v0.wB, v1.a, v1.wB, v2.a, v2.wB);
			break;
		}

		case 4: {
			// Force identical points and *zero* distance
			let v0 = simplex.vertices[0];
			let v1 = simplex.vertices[1];
			let v2 = simplex.vertices[2];
			let v3 = simplex.vertices[3];
			let sum = b3Add(b3Blend2(v0.a, v0.wA, v1.a, v1.wA), b3Blend2(v2.a, v2.wA, v3.a, v3.wA));
			vertexA = sum;
			vertexB = sum;
			break;
		}

		default:
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			break;
	}
}

// C's simplex and backup locals of b3ShapeDistance, which never nests. Holding both here makes the
// per iteration backup one memory copy instead of a load and store per field.
const B3_GJK_SIMPLEX: i32 = 0;
const B3_GJK_BACKUP: i32 = 1;
const s_gjkSimplexes = new StaticArray<b3Simplex>(2);

unsafe function b3ZeroGJKSimplexes(): void {
	memory.fill(changetype<usize>(s_gjkSimplexes), 0, 2 * sizeof<b3Simplex>());
}

unsafe function b3CopyGJKSimplex(dst: i32, src: i32): void {
	let base = changetype<usize>(s_gjkSimplexes);
	let size = sizeof<b3Simplex>();
	memory.copy(base + <usize>dst * size, base + <usize>src * size, size);
}

// Checked accesses for b3ShapeDistance, which is otherwise unchecked: the cached indices and the
// debug simplex capacity come from the caller and are only asserted in C
function b3GetProxyPoint(proxy: b3ShapeProxy, index: i32): b3Vec3 {
	return proxy.points[index];
}

function b3RecordSimplex(simplexes: StaticArray<b3Simplex>, index: i32, simplex: ref b3Simplex): void {
	simplexes[index] = simplex;
}

/// Compute the closest points between two shapes represented as point clouds.
/// b3SimplexCache cache is input/output. On the first call set b3SimplexCache.count to zero.
/// The query runs in frame A, so the witness points and normal are returned in frame A.
/// The underlying GJK algorithm may be debugged by passing in debug simplexes and capacity. You may pass in null and 0 for these.
@unchecked
export function b3ShapeDistance(
	input: b3DistanceInput,
	cache: ref b3SimplexCache,
	simplexes: StaticArray<b3Simplex>? = null,
	simplexCapacity: i32 = 0,
): b3DistanceOutput {
	// The query runs in frame A using the relative pose of B in A.
	let xf = input.transform;

	// Use matrices for faster math
	let m = b3MakeMatrixFromQuat(xf.q);
	let mt = b3Transpose(m);

	let proxyA = input.proxyA;
	let proxyB = input.proxyB;

	// Compute initial simplex from cache
	if (B3_ASSERT_ENABLED) B3_ASSERT(cache.count <= <u16>B3_MAX_SIMPLEX_VERTICES);

	b3ZeroGJKSimplexes();
	ref simplex = s_gjkSimplexes[B3_GJK_SIMPLEX];

	simplex.count = <i32>cache.count;
	for (let i = 0; i < <i32>cache.count; ++i) {
		let index1 = <i32>cache.indexA[i];
		let index2 = <i32>cache.indexB[i];

		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index1 && index1 < proxyA.count);
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index2 && index2 < proxyB.count);

		let vertex1 = b3GetProxyPoint(proxyA, index1);
		let vertex2 = b3Add(b3MulMV(m, b3GetProxyPoint(proxyB, index2)), xf.p);

		simplex.vertices[i].indexA = index1;
		simplex.vertices[i].indexB = index2;
		simplex.vertices[i].wA = vertex1;
		simplex.vertices[i].wB = vertex2;
		simplex.vertices[i].w = b3Sub(vertex2, vertex1);
		simplex.vertices[i].a = 0.0;
	}

	// Compute the new simplex metric, if it is substantially
	// different than the old metric flush the simplex.
	if (simplex.count > 0) {
		let metric1 = cache.metric;
		let metric2 = b3GetMetric(simplex);

		// todo the tetrahedron metric can be negative
		if (2.0 * metric1 < metric2 || metric2 < 0.5 * metric1 || metric2 < FLT_EPSILON) {
			// Flush the simplex
			simplex.count = 0;
		}
	}

	// If the cache is invalid or empty
	if (simplex.count == 0) {
		let vertex1 = b3GetProxyPoint(proxyA, 0);
		let vertex2 = b3Add(b3MulMV(m, b3GetProxyPoint(proxyB, 0)), xf.p);

		simplex.count = 1;
		simplex.vertices[0].indexA = 0;
		simplex.vertices[0].indexB = 0;
		simplex.vertices[0].wA = vertex1;
		simplex.vertices[0].wB = vertex2;
		simplex.vertices[0].w = b3Sub(vertex2, vertex1);
		simplex.vertices[0].a = 0.0;
	}

	ref backup = s_gjkSimplexes[B3_GJK_BACKUP];

	let simplexIndex = 0;
	if (simplexes != null && simplexIndex < simplexCapacity) {
		b3RecordSimplex(simplexes, simplexIndex, simplex);
		simplexIndex += 1;
	}

	let distanceOutput = b3DistanceOutput();

	// Keep track of squared distance
	let distanceSq = FLT_MAX;

	let normal = b3Vec3_zero;

	// Run GJK
	let iteration = 0;
	for (; iteration < B3_MAX_GJK_ITERATIONS; ++iteration) {
		// Solve simplex
		let solved = false;
		switch (simplex.count) {
			case 1:
				simplex.vertices[0].a = 1.0;
				solved = true;
				break;

			case 2:
				solved = b3SolveSimplex2(simplex);
				break;

			case 3:
				solved = b3SolveSimplex3(simplex);
				break;

			case 4:
				solved = b3SolveSimplex4(simplex);
				break;

			default:
				if (B3_ASSERT_ENABLED) B3_ASSERT(false);
				break;
		}

		if (solved == false) {
			// No progress - reconstruct last simplex
			if (B3_ASSERT_ENABLED) B3_ASSERT(backup.count != 0);
			b3CopyGJKSimplex(B3_GJK_SIMPLEX, B3_GJK_BACKUP);
			break;
		}

		if (simplexes != null && simplexIndex < simplexCapacity) {
			b3RecordSimplex(simplexes, simplexIndex, simplex);
			simplexIndex += 1;
			distanceOutput.iterations = iteration;
			distanceOutput.simplexCount = simplexIndex;
		}

		if (simplex.count == B3_MAX_SIMPLEX_VERTICES) {
			// Overlap
			let localPointA = b3Vec3();
			let localPointB = b3Vec3();
			b3ComputeWitnessPoints(simplex, localPointA, localPointB);
			distanceOutput.pointA = localPointA;
			distanceOutput.pointB = localPointB;
			distanceOutput.normal = b3Vec3_zero;
			distanceOutput.distance = 0.0;
			distanceOutput.iterations = iteration;
			distanceOutput.simplexCount = simplexIndex;
			return distanceOutput;
		}

		// Assure distance progression
		let oldDistanceSq = distanceSq;

		// Compute closest point
		let closestPoint = b3Vec3();

		switch (simplex.count) {
			case 1:
				closestPoint = simplex.vertices[0].w;
				break;

			case 2: {
				let v0 = simplex.vertices[0];
				let v1 = simplex.vertices[1];
				closestPoint = b3Blend2(v0.a, v0.w, v1.a, v1.w);
				break;
			}

			case 3: {
				let v0 = simplex.vertices[0];
				let v1 = simplex.vertices[1];
				let v2 = simplex.vertices[2];
				closestPoint = b3Blend3(v0.a, v0.w, v1.a, v1.w, v2.a, v2.w);
				break;
			}

			case 4: {
				let v0 = simplex.vertices[0];
				let v1 = simplex.vertices[1];
				let v2 = simplex.vertices[2];
				let v3 = simplex.vertices[3];
				closestPoint = b3Add(b3Blend2(v0.a, v0.w, v1.a, v1.w), b3Blend2(v2.a, v2.w, v3.a, v3.w));
				break;
			}

			default:
				if (B3_ASSERT_ENABLED) B3_ASSERT(false);
				break;
		}

		distanceSq = b3Dot(closestPoint, closestPoint);

		if (distanceSq >= oldDistanceSq) {
			// No progress - reconstruct last simplex
			if (B3_ASSERT_ENABLED) B3_ASSERT(backup.count != 0);
			b3CopyGJKSimplex(B3_GJK_SIMPLEX, B3_GJK_BACKUP);
			break;
		}

		// Build new tentative support point
		let searchDirection = b3Vec3();

		switch (simplex.count) {
			case 1: {
				// v = -A
				searchDirection = b3Neg(simplex.vertices[0].w);
				break;
			}

			case 2: {
				// v = (AB x AO) x AB
				let a = simplex.vertices[0].w;
				let b = simplex.vertices[1].w;

				let ab = b3Sub(b, a);

				searchDirection = b3Cross(b3Cross(ab, b3Neg(a)), ab);
				break;
			}

			case 3: {
				// v = AB x AC or v = AC x AB
				let a = simplex.vertices[0].w;
				let b = simplex.vertices[1].w;
				let c = simplex.vertices[2].w;

				let ab = b3Sub(b, a);
				let ac = b3Sub(c, a);

				let n = b3Cross(ab, ac);

				searchDirection = b3Dot(n, a) < 0.0 ? n : b3Neg(n);
				break;
			}

			default:
				if (B3_ASSERT_ENABLED) B3_ASSERT(false);
				break;
		}

		if (b3LengthSquared(searchDirection) < 1000.0 * FLT_MIN) {
			// The origin is probably contained by a line segment or triangle.
			// Thus the shapes are overlapped.
			let localPointA = b3Vec3();
			let localPointB = b3Vec3();
			b3ComputeWitnessPoints(simplex, localPointA, localPointB);
			distanceOutput.pointA = localPointA;
			distanceOutput.pointB = localPointB;
			distanceOutput.normal = b3Vec3_zero;
			distanceOutput.distance = 0.0;
			distanceOutput.iterations = iteration;
			distanceOutput.simplexCount = simplexIndex;
			return distanceOutput;
		}

		normal = b3Neg(searchDirection);

		// Get new support points
		let searchDirection1 = searchDirection;
		let indexA = b3GetProxySupport(proxyA, b3Neg(searchDirection1));
		let supportA = proxyA.points[indexA];
		let searchDirection2 = b3MulMV(mt, searchDirection);
		let indexB = b3GetProxySupport(proxyB, searchDirection2);
		let supportB = b3Add(b3MulMV(m, proxyB.points[indexB]), xf.p);

		// Save current simplex and add new vertex - this can fail if we detect cycling
		b3CopyGJKSimplex(B3_GJK_BACKUP, B3_GJK_SIMPLEX);

		// Check for duplicate support points. This is the main termination criteria.
		let duplicate = false;
		for (let i = 0; i < simplex.count; ++i) {
			if (simplex.vertices[i].indexA == indexA && simplex.vertices[i].indexB == indexB) {
				duplicate = true;
				break;
			}
		}

		if (duplicate) {
			break;
		}

		let count = simplex.count;
		simplex.vertices[count].indexA = indexA;
		simplex.vertices[count].indexB = indexB;
		simplex.vertices[count].wA = supportA;
		simplex.vertices[count].wB = supportB;
		simplex.vertices[count].w = b3Sub(supportB, supportA);
		simplex.count += 1;
	}

	// Build witness points.
	let localPointA = b3Vec3();
	let localPointB = b3Vec3();
	b3ComputeWitnessPoints(simplex, localPointA, localPointB);

	distanceOutput.pointA = localPointA;
	distanceOutput.pointB = localPointB;
	distanceOutput.iterations = iteration;
	distanceOutput.simplexCount = simplexIndex;

	normal = b3Normalize(normal);
	if (b3IsNormalized(normal) == false) {
		// Treat as overlap
		distanceOutput.distance = 0.0;
		distanceOutput.normal = b3Vec3_zero;
		return distanceOutput;
	}

	// Results stay in frame A
	distanceOutput.distance = b3Distance(localPointA, localPointB);
	distanceOutput.normal = normal;

	// Apply radii if requested
	if (input.useRadii) {
		let rA = proxyA.radius;
		let rB = proxyB.radius;
		distanceOutput.distance = b3MaxFloat(0.0, distanceOutput.distance - rA - rB);

		// Keep closest points on perimeter even if overlapped, this way the points move smoothly.
		distanceOutput.pointA = b3Add(distanceOutput.pointA, b3MulSV(rA, normal));
		distanceOutput.pointB = b3Sub(distanceOutput.pointB, b3MulSV(rB, normal));
	}

	// Simplex is useful, cache it.
	b3WriteCache(cache, simplex);

	return distanceOutput;
}

// Separation function:
// f(t) = (c2 + t * dp2 - c1 - t * dp1 ) * n

// Root finding : f(t) - target = 0
// (c2 + t * dp2 - c1 - t * dp1 ) * n - target = 0
// (c2 - c1) * n + t * (dp2 - dp1) * n - target = 0
// t = [target - (c2 - c1) * n] / [(dp2 - dp1) * n]
// t = (target - d) / [(dp2 - dp1) * n]

function b3LogVec3(name: string, p: b3Vec3): void {
	b3Log(name + " = {" + p.x.toString() + ", " + p.y.toString() + ", " + p.z.toString() + "}");
}

/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
/// The query runs in frame A, so the hit point and normal are returned in frame A. Initially touching shapes are a miss.
const s_castDistanceInput = new b3DistanceInput();

export function b3ShapeCast(input: b3ShapeCastPairInput): b3CastOutput {
	// Compute tolerance
	let linearSlop = B3_LINEAR_SLOP();
	let totalRadius = input.proxyA.radius + input.proxyB.radius;
	let target = b3MaxFloat(linearSlop, totalRadius - linearSlop);
	let tolerance: f32 = 0.25 * linearSlop;

	if (B3_ASSERT_ENABLED) B3_ASSERT(target > tolerance);

	// Prepare input for distance query
	let cache = b3SimplexCache();

	let alpha: f32 = 0.0;

	// C's stack value; b3ShapeCast does not nest
	let distanceInput = s_castDistanceInput;
	distanceInput.proxyA = input.proxyA;
	distanceInput.proxyB = input.proxyB;
	distanceInput.useRadii = false;

	// The whole cast runs in frame A. Advance the relative pose of B in float each iteration,
	// which keeps the math near the local origin and avoids re-relativizing world poses.
	distanceInput.transform = input.transform;

	let delta2 = input.translationB;
	let distanceOutput = b3DistanceOutput();
	let output = b3CastOutput();
	output.triangleIndex = B3_NULL_INDEX;

	let iteration = 0;
	const maxIterations = 20;

	for (; iteration < maxIterations; ++iteration) {
		output.iterations += 1;

		distanceOutput = b3ShapeDistance(distanceInput, cache, null, 0);

		if (distanceOutput.distance < target + tolerance) {
			if (iteration == 0) {
				if (input.canEncroach && distanceOutput.distance > 2.0 * linearSlop) {
					target = distanceOutput.distance - linearSlop;
				} else if (input.canEncroach && distanceOutput.distance > 0.0) {
					// This is for box movers and can't normally be reached by capsule
					// movers or regular shape casts.

					// Near-target start without overlap: advance by first order while
					// retaining a rest separation. Sweeps that cannot consume the free
					// gap (receding, tangential, grazing) miss.
					let retained: f32 = 0.5 * linearSlop;
					let needed = distanceOutput.distance - retained;
					let approach = -b3Dot(delta2, distanceOutput.normal);
					if (approach <= b3MaxFloat(needed, 0.0)) {
						// Moving away, no hit.
						return output;
					}

					let fraction: f32 = needed > 0.0 ? needed / approach : 0.0;
					if (fraction >= input.maxFraction) {
						// Still no hit because the max fraction is small.
						return output;
					}

					output.fraction = fraction;
					output.point = b3MulAdd(distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal);
					output.normal = distanceOutput.normal;
					output.hit = true;
					return output;
				} else {
					// Initial overlap
					output.hit = true;

					// Compute a common point
					let c1 = b3MulAdd(distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal);
					let c2 = b3MulAdd(distanceOutput.pointB, -input.proxyB.radius, distanceOutput.normal);
					output.point = b3Lerp(c1, c2, 0.5);
					return output;
				}
			} else {
				// Logging for bad input data
				if (distanceOutput.distance > 0.0 && b3IsNormalized(distanceOutput.normal) == false) {
					for (let i = 0; i < input.proxyA.count; ++i) {
						b3LogVec3("pointA[" + i.toString() + "]", input.proxyA.points[i]);
					}
					b3Log("radiusA = " + input.proxyA.radius.toString());

					for (let i = 0; i < input.proxyB.count; ++i) {
						b3LogVec3("pointB[" + i.toString() + "]", input.proxyB.points[i]);
					}
					b3Log("radiusB = " + input.proxyB.radius.toString());

					{
						let xf = input.transform;
						b3LogVec3("transform.p", xf.p);
						b3LogVec3("transform.q.v", xf.q.v);
						b3Log("transform.q.s = " + xf.q.s.toString());
					}

					b3LogVec3("t", input.translationB);

					b3Log("maxFraction = " + input.maxFraction.toString() + ", canEncroach = " + (input.canEncroach ? "1" : "0"));

					// Numerical problem. Likely extreme input.
					return output;
				}

				// Hitting this assert implies that the algorithm brought the shapes too close.
				// B3_ASSERT( distanceOutput.distance > 0.0f && b3IsNormalized( distanceOutput.normal ) );

				output.fraction = alpha;
				output.point = b3MulAdd(distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal);
				output.normal = distanceOutput.normal;
				output.hit = true;
				return output;
			}
		}

		if (B3_ASSERT_ENABLED) B3_ASSERT(distanceOutput.distance > 0.0);
		if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsNormalized(distanceOutput.normal));

		// Check if shapes are approaching each other
		let denominator = b3Dot(delta2, distanceOutput.normal);
		if (denominator >= 0.0) {
			// Miss
			return output;
		}

		// Advance sweep
		alpha += (target - distanceOutput.distance) / denominator;
		if (alpha >= input.maxFraction) {
			// Success!
			return output;
		}

		distanceInput.transform.p = b3MulAdd(input.transform.p, alpha, delta2);
	}

	// Failure!
	return output;
}

/// Evaluate the transform sweep at a specific time.
export function b3GetSweepTransform(sweep: b3Sweep, time: f32): b3Transform {
	let transform: b3Transform;
	transform.q = b3NLerp(sweep.q1, sweep.q2, time);
	transform.p = b3Sub(b3Lerp(sweep.c1, sweep.c2, time), b3RotateVector(transform.q, sweep.localCenter));
	return transform;
}

function b3GetFinalSweepTransform(sweep: b3Sweep): b3Transform {
	let transform: b3Transform;
	transform.q = sweep.q2;
	transform.p = b3Sub(sweep.c2, b3RotateVector(transform.q, sweep.localCenter));
	return transform;
}

function b3UniqueCount(vertexCount: i32, vertices: FixedArray<i32, 3>): i32 {
	if (B3_ASSERT_ENABLED) B3_ASSERT(1 <= vertexCount && vertexCount <= 3);

	switch (vertexCount) {
		case 1:
			return 1;

		case 2:
			return vertices[0] != vertices[1] ? 2 : 1;

		case 3:
			if (vertices[0] != vertices[1] && vertices[0] != vertices[2] && vertices[1] != vertices[2]) {
				// All different
				return 3;
			}

			if (vertices[0] == vertices[1] && vertices[0] == vertices[2] && vertices[1] == vertices[2]) {
				// All equal
				return 1;
			}

			return 2;

		default:
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			break;
	}

	return 0;
}

// This checks if the cross product of two edges switches direction.
function b3CheckFastEdges(xfA: b3Transform, localEdgeA: b3Vec3, xfB: b3Transform, localEdgeB: b3Vec3, axis0: b3Vec3): bool {
	// By taking the local witness axes we make sure that we
	// get the correct orientations (e.g. if one axis was flipped)!
	let edgeA = b3RotateVector(xfA.q, localEdgeA);
	let edgeB = b3RotateVector(xfB.q, localEdgeB);
	let axis = b3Cross(edgeA, edgeB);
	return b3Dot(axis, axis0) < 0.0;
}

type b3SeparationType = i32;
const b3_separationUnknown: b3SeparationType = 0;
const b3_separationVertices: b3SeparationType = 1;
const b3_separationEdges: b3SeparationType = 2;
const b3_separationFaceA: b3SeparationType = 3;
const b3_separationFaceB: b3SeparationType = 4;

class b3SeparationFunction {
	proxyA: b3ShapeProxy;
	proxyB: b3ShapeProxy;
	sweepA: b3Sweep = b3Sweep();
	sweepB: b3Sweep = b3Sweep();

	// These are associated with different bodies depending on the separation function type.
	// It could be two local vectors/points on the same body (for example, both on bodyA).
	witness1: b3Vec3 = b3Vec3();
	witness2: b3Vec3 = b3Vec3();

	type: b3SeparationType = b3_separationUnknown;

	constructor(proxyA: b3ShapeProxy, proxyB: b3ShapeProxy) {
		this.proxyA = proxyA;
		this.proxyB = proxyB;
	}
}

const s_separationFunction = new b3SeparationFunction(new b3ShapeProxy(), new b3ShapeProxy());

function b3MakeSeparationFunction(
	cache: b3SimplexCache,
	proxyA: b3ShapeProxy,
	sweepA: b3Sweep,
	proxyB: b3ShapeProxy,
	sweepB: b3Sweep,
	worldNormal: b3Vec3,
	t1: f32,
): b3SeparationFunction {
	if (B3_ASSERT_ENABLED) B3_ASSERT(1 <= cache.count && cache.count <= 3);

	// C's stack value; one object serves every call, since b3TimeOfImpact does not nest
	let fcn = s_separationFunction;
	fcn.proxyA = proxyA;
	fcn.proxyB = proxyB;
	fcn.witness1 = b3Vec3();
	fcn.witness2 = b3Vec3();
	fcn.sweepA = sweepA;
	fcn.sweepB = sweepB;
	fcn.type = b3_separationUnknown;

	let indexA: FixedArray<i32, 3>;
	indexA[0] = <i32>cache.indexA[0];
	indexA[1] = <i32>cache.indexA[1];
	indexA[2] = <i32>cache.indexA[2];
	let indexB: FixedArray<i32, 3>;
	indexB[0] = <i32>cache.indexB[0];
	indexB[1] = <i32>cache.indexB[1];
	indexB[2] = <i32>cache.indexB[2];

	let uniqueCountA = b3UniqueCount(<i32>cache.count, indexA);
	let uniqueCountB = b3UniqueCount(<i32>cache.count, indexB);

	let xfA1 = b3GetSweepTransform(sweepA, t1);
	let xfB1 = b3GetSweepTransform(sweepB, t1);

	let qA = xfA1.q;
	let qB = xfB1.q;

	// Minimize round-off
	let deltaP = b3Sub(xfB1.p, xfA1.p);

	switch (<i32>cache.count) {
		case 1: {
			// Witness is the world space direction
			fcn.type = b3_separationVertices;
			fcn.witness1 = worldNormal;
			break;
		}

		case 2: {
			if (uniqueCountA == 2 && uniqueCountB == 2) {
				// Edge/Edge
				let vA1 = proxyA.points[indexA[0]];
				let localEdgeA = b3Sub(proxyA.points[indexA[1]], vA1);
				localEdgeA = b3Normalize(localEdgeA);
				let edgeA = b3RotateVector(qA, localEdgeA);

				let vB1 = proxyB.points[indexB[0]];
				let localEdgeB = b3Sub(proxyB.points[indexB[1]], vB1);
				localEdgeB = b3Normalize(localEdgeB);
				let edgeB = b3RotateVector(qB, localEdgeB);

				let axis = b3Cross(edgeA, edgeB);
				let lengthSquared = b3LengthSquared(axis);

				// Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
				const kToleranceSquared: f32 = <f32>0.05 * <f32>0.05;
				if (lengthSquared < kToleranceSquared) {
					// The axis is not safe to normalize so we use a world axis instead!
					fcn.type = b3_separationVertices;
					fcn.witness1 = worldNormal;
				} else {
					let delta = b3Add(b3Sub(b3RotateVector(qB, vB1), b3RotateVector(qA, vA1)), deltaP);
					if (b3Dot(delta, axis) < 0.0) {
						// Make axis point from A to B
						axis = b3Neg(axis);
						localEdgeB = b3Neg(localEdgeB);
					}

					// Check for possible sign flip in edge/edge cross product
					let xfA2 = b3GetFinalSweepTransform(sweepA);
					let xfB2 = b3GetFinalSweepTransform(sweepB);
					let fastEdges = b3CheckFastEdges(xfA2, localEdgeA, xfB2, localEdgeB, axis);
					if (fastEdges == true) {
						// Not safe to use local edges, fall back to initial world space axis instead
						fcn.type = b3_separationVertices;
						fcn.witness1 = b3Normalize(axis);
					} else {
						// Edge cross product is safe. This converges faster than a fixed axis.
						fcn.type = b3_separationEdges;
						fcn.witness1 = localEdgeA;
						fcn.witness2 = localEdgeB;
					}
				}
			} else {
				// Vertex versus edge, use world axis witness
				fcn.type = b3_separationVertices;
				fcn.witness1 = worldNormal;
			}
			break;
		}

		case 3: {
			if (uniqueCountA == 3) {
				let vA1 = proxyA.points[indexA[0]];
				let vA2 = proxyA.points[indexA[1]];
				let vA3 = proxyA.points[indexA[2]];
				let localAxisA = b3Cross(b3Sub(vA2, vA1), b3Sub(vA3, vA1));
				localAxisA = b3Normalize(localAxisA);
				let axisA = b3RotateVector(qA, localAxisA);

				let localPointA = b3MulSV(<f32>1.0 / <f32>3.0, b3Add(b3Add(vA1, vA2), vA3));
				let localPointB = proxyB.points[indexB[0]];
				let delta = b3Add(b3Sub(b3RotateVector(qB, localPointB), b3RotateVector(qA, localPointA)), deltaP);

				if (b3Dot(delta, axisA) < 0.0) {
					// Make axis point from A to B
					localAxisA = b3Neg(localAxisA);
				}

				// Witness is the local plane of faceA
				fcn.type = b3_separationFaceA;
				fcn.witness1 = localAxisA;
				fcn.witness2 = localPointA;
			} else if (uniqueCountB == 3) {
				let vB1 = proxyB.points[indexB[0]];
				let vB2 = proxyB.points[indexB[1]];
				let vB3 = proxyB.points[indexB[2]];
				let localAxisB = b3Cross(b3Sub(vB2, vB1), b3Sub(vB3, vB1));
				localAxisB = b3Normalize(localAxisB);
				let axisB = b3RotateVector(qB, localAxisB);

				let localPointA = proxyA.points[indexA[0]];
				let localPointB = b3MulSV(<f32>1.0 / <f32>3.0, b3Add(b3Add(vB1, vB2), vB3));
				let delta = b3Sub(b3Sub(b3RotateVector(qA, localPointA), b3RotateVector(qB, localPointB)), deltaP);

				if (b3Dot(delta, axisB) < 0.0) {
					// Make axis point from B to A
					localAxisB = b3Neg(localAxisB);
				}

				// Witness is the local plane of faceB
				fcn.type = b3_separationFaceB;
				fcn.witness1 = localAxisB;
				fcn.witness2 = localPointB;
			} else {
				if (B3_ASSERT_ENABLED) B3_ASSERT(uniqueCountA == 2 && uniqueCountB == 2);

				if (indexA[0] == indexA[1]) {
					// Make first two indices are unique
					indexA[1] = indexA[2];
					if (B3_ASSERT_ENABLED) B3_ASSERT(indexA[0] != indexA[1]);
				}

				let vA1 = proxyA.points[indexA[0]];
				let vA2 = proxyA.points[indexA[1]];
				let localEdgeA = b3Normalize(b3Sub(vA2, vA1));
				let edgeA = b3RotateVector(qA, localEdgeA);

				if (indexB[0] == indexB[1]) {
					// Make first two indices are unique
					indexB[1] = indexB[2];
					if (B3_ASSERT_ENABLED) B3_ASSERT(indexB[0] != indexB[1]);
				}

				let vB1 = proxyB.points[indexB[0]];
				let vB2 = proxyB.points[indexB[1]];
				let localEdgeB = b3Normalize(b3Sub(vB2, vB1));
				let edgeB = b3RotateVector(qB, localEdgeB);

				let axis = b3Cross(edgeA, edgeB);
				let lengthSquared = b3LengthSquared(axis);

				// Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
				const kToleranceSquared: f32 = <f32>0.005 * <f32>0.005;
				if (lengthSquared < kToleranceSquared) {
					// The axis is not safe to normalize so we use a world axis instead!
					fcn.type = b3_separationVertices;
					fcn.witness1 = worldNormal;
				} else {
					let delta = b3Add(b3Sub(b3RotateVector(qB, vB1), b3RotateVector(qA, vA1)), deltaP);
					if (b3Dot(delta, axis) < 0.0) {
						// Make axis point from A to B
						axis = b3Neg(axis);
						localEdgeB = b3Neg(localEdgeB);
					}

					// Check for possible sign flip in edge/edge cross product
					let xfA2 = b3GetFinalSweepTransform(sweepA);
					let xfB2 = b3GetFinalSweepTransform(sweepB);
					let fastEdges = b3CheckFastEdges(xfA2, localEdgeA, xfB2, localEdgeB, axis);
					if (fastEdges) {
						// Not safe to use local edges, fall back to initial world space axis instead
						fcn.type = b3_separationVertices;
						fcn.witness1 = b3Normalize(axis);
					} else {
						// Edge cross product is safe. This converges faster than a fixed axis.
						fcn.type = b3_separationEdges;
						fcn.witness1 = localEdgeA;
						fcn.witness2 = localEdgeB;
					}
				}
			}
			break;
		}

		default:
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			break;
	}

	return fcn;
}

function b3FindMinSeparation(fcn: b3SeparationFunction, indexA: ref i32, indexB: ref i32, t: f32): f32 {
	let xfA = b3GetSweepTransform(fcn.sweepA, t);
	let xfB = b3GetSweepTransform(fcn.sweepB, t);

	switch (fcn.type) {
		case b3_separationVertices: {
			let axis = fcn.witness1;

			let localAxisA = b3InvRotateVector(xfA.q, axis);
			let localAxisB = b3InvRotateVector(xfB.q, b3Neg(axis));

			indexA = b3GetPointSupport(fcn.proxyA.points, fcn.proxyA.count, localAxisA);
			indexB = b3GetPointSupport(fcn.proxyB.points, fcn.proxyB.count, localAxisB);

			let deltaP = b3Sub(xfB.p, xfA.p);
			let localPointA = fcn.proxyA.points[indexA];
			let localPointB = fcn.proxyB.points[indexB];
			let delta = b3Add(b3Sub(b3RotateVector(xfB.q, localPointB), b3RotateVector(xfA.q, localPointA)), deltaP);
			return b3Dot(delta, axis);
		}

		case b3_separationEdges: {
			let edgeA = b3RotateVector(xfA.q, fcn.witness1);
			let edgeB = b3RotateVector(xfB.q, fcn.witness2);
			let axis = b3Cross(edgeA, edgeB);
			if (B3_ASSERT_ENABLED) B3_ASSERT(axis.x != 0.0 || axis.y != 0.0 || axis.z != 0.0);
			axis = b3Normalize(axis);

			let axisA = b3InvRotateVector(xfA.q, axis);
			indexA = b3GetPointSupport(fcn.proxyA.points, fcn.proxyA.count, axisA);

			let axisB = b3InvRotateVector(xfB.q, axis);
			indexB = b3GetPointSupport(fcn.proxyB.points, fcn.proxyB.count, b3Neg(axisB));

			let deltaP = b3Sub(xfB.p, xfA.p);
			let localPointA = fcn.proxyA.points[indexA];
			let localPointB = fcn.proxyB.points[indexB];
			let delta = b3Add(b3Sub(b3RotateVector(xfB.q, localPointB), b3RotateVector(xfA.q, localPointA)), deltaP);

			return b3Dot(delta, axis);
		}

		case b3_separationFaceA: {
			let normal = b3RotateVector(xfA.q, fcn.witness1);
			indexA = -1;
			let pointA = b3TransformPoint(xfA, fcn.witness2);

			let axisB = b3InvRotateVector(xfB.q, normal);
			indexB = b3GetPointSupport(fcn.proxyB.points, fcn.proxyB.count, b3Neg(axisB));
			let pointB = b3TransformPoint(xfB, fcn.proxyB.points[indexB]);

			return b3Dot(b3Sub(pointB, pointA), normal);
		}

		case b3_separationFaceB: {
			let normal = b3RotateVector(xfB.q, fcn.witness1);

			let axisA = b3InvRotateVector(xfA.q, normal);
			indexA = b3GetPointSupport(fcn.proxyA.points, fcn.proxyA.count, b3Neg(axisA));
			let pointA = b3TransformPoint(xfA, fcn.proxyA.points[indexA]);

			indexB = -1;
			let pointB = b3TransformPoint(xfB, fcn.witness2);

			return b3Dot(b3Sub(pointA, pointB), normal);
		}

		default:
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			break;
	}

	return 0.0;
}

function b3EvaluateSeparation(fcn: b3SeparationFunction, index1: i32, index2: i32, beta: f32): f32 {
	let transform1 = b3GetSweepTransform(fcn.sweepA, beta);
	let transform2 = b3GetSweepTransform(fcn.sweepB, beta);

	switch (fcn.type) {
		case b3_separationVertices: {
			let axis = fcn.witness1;

			let point1 = b3TransformPoint(transform1, fcn.proxyA.points[index1]);
			let point2 = b3TransformPoint(transform2, fcn.proxyB.points[index2]);

			return b3Dot(b3Sub(point2, point1), axis);
		}

		case b3_separationEdges: {
			let edge1 = b3RotateVector(transform1.q, fcn.witness1);
			let edge2 = b3RotateVector(transform2.q, fcn.witness2);
			let axis = b3Cross(edge1, edge2);
			axis = b3Normalize(axis);

			let point1 = b3TransformPoint(transform1, fcn.proxyA.points[index1]);
			let point2 = b3TransformPoint(transform2, fcn.proxyB.points[index2]);

			return b3Dot(b3Sub(point2, point1), axis);
		}

		case b3_separationFaceA: {
			let axis = b3RotateVector(transform1.q, fcn.witness1);

			let point1 = b3TransformPoint(transform1, fcn.witness2);
			let point2 = b3TransformPoint(transform2, fcn.proxyB.points[index2]);

			return b3Dot(b3Sub(point2, point1), axis);
		}

		case b3_separationFaceB: {
			let axis = b3RotateVector(transform2.q, fcn.witness1);

			let point1 = b3TransformPoint(transform1, fcn.proxyA.points[index1]);
			let point2 = b3TransformPoint(transform2, fcn.witness2);

			return b3Dot(b3Sub(point1, point2), axis);
		}

		default:
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			break;
	}

	return 0.0;
}

function b3ForceFixedAxis(fcn: b3SeparationFunction, beta: f32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(fcn.type == b3_separationEdges);

	let transform1 = b3GetSweepTransform(fcn.sweepA, beta);
	let transform2 = b3GetSweepTransform(fcn.sweepB, beta);

	let edge1 = b3RotateVector(transform1.q, fcn.witness1);
	let edge2 = b3RotateVector(transform2.q, fcn.witness2);
	let axis = b3Cross(edge1, edge2);
	axis = b3Normalize(axis);

	fcn.type = b3_separationVertices;
	fcn.witness1 = axis;
	fcn.witness2 = b3Vec3_zero;
}

/// Compute the upper bound on time before two shapes penetrate. Time is represented as
/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collisions. If you change the time interval, you should call this function
/// again.
// Time of Impact using root finding
const s_toiDistanceInput = new b3DistanceInput();

export function b3TimeOfImpact(input: b3TOIInput): b3TOIOutput {
	let output = b3TOIOutput();

	// Set these to invalid values so they can be validated on exit
	output.state = b3_toiStateUnknown;
	output.fraction = -1.0;

	let sweepA = input.sweepA;
	let sweepB = input.sweepB;

	// Shift to origin
	let origin = sweepA.c1;
	sweepA.c1 = b3Vec3_zero;
	sweepA.c2 = b3Sub(sweepA.c2, origin);
	sweepB.c1 = b3Sub(sweepB.c1, origin);
	sweepB.c2 = b3Sub(sweepB.c2, origin);

	let proxyA = input.proxyA;
	let proxyB = input.proxyB;

	let maxPushBackIterations = proxyA.count + proxyB.count;
	let tMax = input.maxFraction;

	// Setup target distance and tolerance
	let linearSlop = B3_LINEAR_SLOP();
	let totalRadius = proxyA.radius + proxyB.radius;
	let target = b3MaxFloat(linearSlop, totalRadius - linearSlop);
	let tolerance: f32 = 0.25 * linearSlop;
	if (B3_ASSERT_ENABLED) B3_ASSERT(target > tolerance);

	let t1: f32 = 0.0;
	const maxIterations = 25;
	let distanceIterations = 0;

	// Prepare input for distance query. C's stack value; b3TimeOfImpact does not nest.
	let cache = b3SimplexCache();
	let distanceInput = s_toiDistanceInput;
	distanceInput.proxyA = proxyA;
	distanceInput.proxyB = proxyB;
	distanceInput.useRadii = false;

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for (;;) {
		// Get the distance between shapes. We can also use the results to get a separating axis.
		let xfA = b3GetSweepTransform(sweepA, t1);
		let xfB = b3GetSweepTransform(sweepB, t1);
		distanceInput.transform = b3InvMulTransforms(xfA, xfB);
		let distanceOutput = b3ShapeDistance(distanceInput, cache, null, 0);
		output.distance = distanceOutput.distance;

		// The distance query runs in frame A, project the witness data back to the shifted world
		let worldNormal = b3RotateVector(xfA.q, distanceOutput.normal);
		let worldPointA = b3TransformPoint(xfA, distanceOutput.pointA);
		let worldPointB = b3TransformPoint(xfA, distanceOutput.pointB);

		output.distanceIterations += 1;
		distanceIterations += 1;

		// If the shapes are overlapped, we give up on continuous collision.
		if (distanceOutput.distance <= 0.0) {
			output.state = b3_toiStateOverlapped;
			output.fraction = 0.0;

			// Averaged hit point
			let pA = b3MulAdd(worldPointA, proxyA.radius, worldNormal);
			let pB = b3MulAdd(worldPointB, -proxyB.radius, worldNormal);
			output.point = b3Lerp(pA, pB, 0.5);
			output.point = b3Add(output.point, origin);
			output.normal = worldNormal;

			break;
		}

		if (distanceOutput.distance <= target + tolerance) {
			// Success!
			output.state = b3_toiStateHit;

			// Averaged hit point
			let pA = b3MulAdd(worldPointA, proxyA.radius, worldNormal);
			let pB = b3MulAdd(worldPointB, -proxyB.radius, worldNormal);
			output.point = b3Lerp(pA, pB, 0.5);
			output.point = b3Add(output.point, origin);
			output.normal = worldNormal;
			output.fraction = t1;
			break;
		}

		if (distanceIterations == maxIterations) {
			// Progress too slow. This can happen when a capsule rotates around a
			// triangle vertex.
			output.state = b3_toiStateFailed;
			output.fraction = t1;

			// Averaged hit point
			let pA = b3MulAdd(worldPointA, input.proxyA.radius, worldNormal);
			let pB = b3MulAdd(worldPointB, -input.proxyB.radius, worldNormal);
			output.point = b3Lerp(pA, pB, 0.5);
			output.point = b3Add(output.point, origin);
			output.normal = worldNormal;
			break;
		}

		// Initialize the separating axis.
		let fcn = b3MakeSeparationFunction(cache, proxyA, sweepA, proxyB, sweepB, worldNormal, t1);

		// Compute the TOI on the separating axis. We do this by successively resolving the deepest point.
		let done = false;
		let t2 = tMax;
		let pushBackIterations = 0;
		let indices: FixedArray<i32, 2>;
		for (;;) {
			let s2 = b3FindMinSeparation(fcn, indices[0], indices[1], t2);
			let indexA = indices[0];
			let indexB = indices[1];

			// Is the final configuration separated?
			if (s2 - target > tolerance) {
				// Success!
				output.state = b3_toiStateSeparated;
				output.fraction = input.maxFraction;
				done = true;
				break;
			}

			// Has the separation reached tolerance?
			if (s2 >= target - tolerance) {
				// Advance the sweeps
				t1 = t2;
				break;
			}

			// Compute the initial separation of the witness points
			let s1 = b3EvaluateSeparation(fcn, indexA, indexB, t1);

			// Check for overlap. This might happen if the root finder runs out of iterations.
			if (s1 < target - tolerance) {
				// Failed!
				output.state = b3_toiStateFailed;
				output.fraction = t1;
				done = true;
				break;
			}

			// Has the separation reached tolerance?
			if (s1 <= target + tolerance) {
				// Success! t1 should hold the TOI (could be 0.0)
				output.state = b3_toiStateHit;
				output.fraction = t1;
				done = true;
				break;
			}

			// Compute 1D root of: f(x) - target = 0
			let rootIterationCount = 0;
			let maxRootIterations = 50;
			let a1 = t1;
			let a2 = t2;
			for (;;) {
				// Use a mix of false position and bisection.
				let t: f32;
				if ((rootIterationCount & 1) != 0) {
					// False position to improve convergence.
					t = a1 + ((target - s1) * (a2 - a1)) / (s2 - s1);
				} else {
					// Bisection to guarantee progress.
					t = 0.5 * (a1 + a2);
				}

				output.rootIterations += 1;
				rootIterationCount += 1;

				let s = b3EvaluateSeparation(fcn, indexA, indexB, t);

				// Has the separation reached tolerance?
				if (b3AbsFloat(s - target) <= tolerance) {
					// t2 holds a tentative value for t1
					t2 = t;
					break;
				}

				// Ensure we continue to bracket the root.
				if (s > target) {
					a1 = t;
					s1 = s;
				} else {
					a2 = t;
					s2 = s;
				}

				if (rootIterationCount == maxRootIterations) {
					break;
				}
			}

			// Restart the inner loop if we have a failing edge case.
			if (rootIterationCount == maxRootIterations - 1 && fcn.type == b3_separationEdges) {
				rootIterationCount = 0;
				t2 = input.maxFraction;
				b3ForceFixedAxis(fcn, t1);
				if (B3_ASSERT_ENABLED) B3_ASSERT(fcn.type != b3_separationEdges);
			}

			output.pushBackIterations += 1;
			pushBackIterations += 1;

			if (pushBackIterations == maxPushBackIterations) {
				break;
			}
		}

		if (done) {
			// Averaged hit point
			let pA = b3MulAdd(worldPointA, input.proxyA.radius, worldNormal);
			let pB = b3MulAdd(worldPointB, -input.proxyB.radius, worldNormal);
			output.point = b3Lerp(pA, pB, 0.5);
			output.point = b3Add(output.point, origin);
			output.normal = worldNormal;
			break;
		}
	}

	// It is expected that the state and fraction are set before reaching this
	if (B3_ASSERT_ENABLED) B3_ASSERT(output.state != b3_toiStateUnknown);
	if (B3_ASSERT_ENABLED) B3_ASSERT(output.fraction >= 0.0);

	return output;
}

// AnimaScript helpers, not in C: C builds proxies inline over its own storage
// ((b3ShapeProxy){ &capsule->center1, 2, r }), which here needs an array holding the points.

/// Make a proxy that owns a copy of the first count points.
export function b3MakeProxy(points: StaticArray<b3Vec3>, count: i32, radius: f32): b3ShapeProxy {
	let copy = new StaticArray<b3Vec3>(count);
	for (let i = 0; i < count; ++i) {
		copy[i] = points[i];
	}
	return new b3ShapeProxy(copy, count, radius);
}

/// Make a one point proxy, as C does with &sphere->center.
export function b3MakePointProxy(point: b3Vec3, radius: f32): b3ShapeProxy {
	let points = new StaticArray<b3Vec3>(1);
	points[0] = point;
	return new b3ShapeProxy(points, 1, radius);
}

/// Refill a caller owned proxy, the allocation free form of the helpers here for hot callers that
/// keep one proxy and its point storage.
export function b3SetProxy(proxy: b3ShapeProxy, points: StaticArray<b3Vec3>, count: i32, radius: f32): b3ShapeProxy {
	proxy.points = points;
	proxy.count = count;
	proxy.radius = radius;
	return proxy;
}

/// Make a two point proxy, as C does with &capsule->center1.
export function b3MakeSegmentProxy(point1: b3Vec3, point2: b3Vec3, radius: f32): b3ShapeProxy {
	let points = new StaticArray<b3Vec3>(2);
	points[0] = point1;
	points[1] = point2;
	return new b3ShapeProxy(points, 2, radius);
}
