// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/mover.as, mirrors verify/cref/mover.c

import { vtag, vi32, vf32, vend, vseed, vrand32, vrandint, vrandf } from "./common";
import { FLT_MAX, b3SetLengthUnitsPerMeter } from "../../lib/core";
import { b3Normalize, b3Vec3 } from "../../lib/math_functions";
import { b3CollisionPlane } from "../../lib/collision";
import { b3ClipVector, b3SolvePlanes } from "../../lib/mover";

export { frameBoundary } from "./common";

const PLANE_CAPACITY = 8;

function rvec(lo: f32, hi: f32): b3Vec3 {
	let v: b3Vec3;
	v.x = vrandf(lo, hi);
	v.y = vrandf(lo, hi);
	v.z = vrandf(lo, hi);
	return v;
}

function rplane(plane: ref b3CollisionPlane): void {
	let n = rvec(-1.0, 1.0);
	plane.plane.normal = b3Normalize(n);
	plane.plane.offset = vrandf(-1.0, 1.0);

	let limit = vrandint(0, 4);
	if (limit == 0) {
		plane.pushLimit = FLT_MAX;
	} else if (limit == 1) {
		plane.pushLimit = 0.0;
	} else {
		plane.pushLimit = vrandf(0.0, 0.5);
	}

	plane.push = vrandf(-1.0, 1.0);
	plane.clipVelocity = (vrand32() & 1) != 0;
}

function run(lengthUnits: f32, caseCount: i32): void {
	b3SetLengthUnitsPerMeter(lengthUnits);

	let planes = new StaticArray<b3CollisionPlane>(PLANE_CAPACITY);
	for (let c = 0; c < caseCount; ++c) {
		let count = vrandint(0, PLANE_CAPACITY + 1);
		for (let i = 0; i < count; ++i) {
			rplane(planes[i]);
		}

		let targetDelta = rvec(-2.0, 2.0);
		let result = b3SolvePlanes(targetDelta, planes, count);

		vtag("solve");
		vi32(count);
		vf32(result.delta.x);
		vf32(result.delta.y);
		vf32(result.delta.z);
		vi32(result.iterationCount);
		for (let i = 0; i < count; ++i) {
			vf32(planes[i].push);
		}
		vend();

		if ((vrand32() & 3) == 0) {
			for (let i = 0; i < count; ++i) {
				planes[i].push = (vrand32() & 1) != 0 ? <f32>0.0 : <f32>-0.0;
			}
		}

		let vector = rvec(-5.0, 5.0);
		let clipped = b3ClipVector(vector, planes, count);

		vtag("clip");
		vf32(clipped.x);
		vf32(clipped.y);
		vf32(clipped.z);
		vend();
	}

	b3SetLengthUnitsPerMeter(1.0);
}

export function runAll(): void {
	vseed(0x6d6f766572);
	run(1.0, 4000);
	run(0.37, 2000);
	run(100.0, 2000);
}
