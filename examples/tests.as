// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Runs every example and checks that it shows what it says. examples/check.sh runs this suite with rive --test.

import { Tests } from "rive/test";
import { events } from "./events";
import { helloWorld } from "./hello_world";
import { ANCHOR_HEIGHT, LINK_COUNT, LINK_LENGTH, joints } from "./joints";
import { moverExample } from "./mover";
import { queries } from "./queries";
import { shapes } from "./shapes";
import { RUN_LANES, RUN_PARALLEL, threading } from "./threading";

export class ExampleTests extends Tests {
	override run(): void {
		test("hello world: the box comes to rest on the ground and sleeps", (expect) => {
			let result = helloWorld();
			expect.that(result.height).greaterThan(0.48);
			expect.that(result.height).lessThan(0.52);
			expect.that(result.awake).is(false);
		});

		test("shapes: each body rests on its static shape and sleeps", (expect) => {
			let result = shapes();
			// On the compound table top at y = 4.1, a 0.5 m sphere
			expect.that(result.sphereHeight).greaterThan(4.55);
			expect.that(result.sphereHeight).lessThan(4.65);
			// On the mesh ledge at y = 3, a 1 m box
			expect.that(result.boxHeight).greaterThan(3.45);
			expect.that(result.boxHeight).lessThan(3.55);
			// In the bowl, not through it
			expect.that(result.lowestInBowl).greaterThan(0.0);
			expect.that(result.awakeCount).is(0);
		});

		test("joints: the chain hangs under its anchor and the elevator reaches its limit", (expect) => {
			let result = joints();
			let length: f32 = <f32>LINK_COUNT * LINK_LENGTH;
			expect.that(Mathf.abs(result.chainEnd.x)).lessThan(0.5);
			expect.that(result.chainEnd.y).lessThan(ANCHOR_HEIGHT - length + LINK_LENGTH);
			expect.that(result.chainEnd.y).greaterThan(ANCHOR_HEIGHT - length - 0.1);
			expect.that(result.elevatorTranslation).greaterThan(2.95);
			expect.that(result.elevatorTranslation).lessThan(3.05);
		});

		test("queries: rays, a sphere cast and overlaps find the boxes", (expect) => {
			let result = queries();
			expect.that(result.closestBox).is(1);
			expect.that(result.closestHeight).greaterThan(0.99);
			expect.that(result.closestHeight).lessThan(1.01);
			expect.that(result.rayHitCount).is(3);
			// The sphere's surface meets the first box's face at x = -4.5 when its center is at -4.75
			expect.that(result.castBox).is(0);
			expect.that(result.castFraction).greaterThan(0.26);
			expect.that(result.castFraction).lessThan(0.265);
			expect.that(result.aabbOverlaps).is(1);
			expect.that(result.shapeOverlaps).is(2);
		});

		test("events: the sensor sees the box pass, the ground reports its landing", (expect) => {
			let result = events();
			expect.that(result.sensorBegins).is(1);
			expect.that(result.sensorEnds).is(1);
			expect.that(result.contactBegins).is(1);
			expect.that(result.hits).is(1);
			expect.that(result.fastestHit).greaterThan(5.0);
		});

		test("mover: the capsule lands on the ground and stops at the wall", (expect) => {
			let mover = moverExample();
			// The capsule's bottom on the ground at y = 0
			expect.that(mover.position.y).greaterThan(0.78);
			expect.that(mover.position.y).lessThan(0.82);
			// Its side against the wall's face at x = 4
			expect.that(mover.position.x).greaterThan(3.6);
			expect.that(mover.position.x).lessThan(3.71);
			expect.that(Mathf.abs(mover.velocity.x)).lessThan(0.01);
		});

		test("threading: four lanes end bit for bit where the serial path does", (expect) => {
			let result = threading(RUN_LANES);
			expect.that(result.differ).is(0);
			// Every step ran its solver stages and narrow phase on the lanes
			expect.that(result.solves).is(120);
			expect.that(result.collides).is(120);
		});

		test("threading: parallel calls end bit for bit where the serial path does", (expect) => {
			let result = threading(RUN_PARALLEL);
			expect.that(result.differ).is(0);
			expect.that(result.solves).is(120);
			expect.that(result.collides).is(120);
		});
	}
}
