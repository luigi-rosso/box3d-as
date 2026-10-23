// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Twin of verify/cref/scene_queries.c: the queries and village scenes, one step per call, printing
// each query kind's result hash and hit count and the movers' poses after every step.

import { vbool, vend, vf32, vi32, vtag, vu32 } from "./common";
import { Scene, Scene_Create, Scene_Destroy, Scene_Step, SCENE_QUERY_KIND_COUNT } from "../scenes/scenes";

export { frameBoundary } from "./common";

let g_scene = new Scene();

export function create(index: i32): void {
	g_scene = new Scene();
	Scene_Create(g_scene, index == 0 ? "queries" : "village", 1);
}

export function step(stepIndex: i32): void {
	let scene = g_scene;
	Scene_Step(scene, stepIndex);

	vtag("step");
	vi32(stepIndex);
	for (let k = 0; k < SCENE_QUERY_KIND_COUNT; ++k) {
		vu32(scene.queryHash[k]);
		vi32(scene.queryHits[k]);
	}
	vend();

	for (let m = 0; m < scene.moverCount; ++m) {
		let mover = scene.movers[m];
		vtag("mover");
		vi32(m);
		vf32(mover.transform.p.x);
		vf32(mover.transform.p.y);
		vf32(mover.transform.p.z);
		vf32(mover.velocity.x);
		vf32(mover.velocity.y);
		vf32(mover.velocity.z);
		vbool(mover.onGround);
		vi32(mover.planeCount);
		vend();
	}
}

export function destroy(): void {
	Scene_Destroy(g_scene);
}
