// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for the step solver, mirrors verify/cref/solver.c: four scenes through the public API, with the
// solver sets, graph colors, islands and contact impulses printed from the world internals after each step.

import { vtag, vi32, vu32, vbool, vf32, vu64, vend } from "./common";
import { B3_NULL_INDEX } from "../../lib/core";
import { B3_GRAPH_COLOR_COUNT } from "../../lib/constants";
import {
	B3_PI,
	b3BodyDef,
	b3BodyId,
	b3Capsule,
	b3ComputeCosSin,
	b3CreateBody,
	b3CreateCapsuleShape,
	b3CreateDistanceJoint,
	b3CreateHullShape,
	b3CreateMeshShape,
	b3CreateRevoluteJoint,
	b3CreateSphereShape,
	b3CreateSphericalJoint,
	b3CreateWaveMesh,
	b3CreateWeldJoint,
	b3CreateWorld,
	b3Body_ApplyLinearImpulse,
	b3Body_Disable,
	b3Body_Enable,
	b3Body_GetAngularVelocity,
	b3Body_GetLinearVelocity,
	b3Body_GetPosition,
	b3Body_GetRotation,
	b3Body_IsAwake,
	b3Body_IsValid,
	b3Body_SetAwake,
	b3DefaultBodyDef,
	b3DefaultDistanceJointDef,
	b3DefaultRevoluteJointDef,
	b3DefaultShapeDef,
	b3DefaultSphericalJointDef,
	b3DefaultWeldJointDef,
	b3DefaultWorldDef,
	b3DestroyBody,
	b3DestroyJoint,
	b3DestroyMesh,
	b3DestroyWorld,
	b3JointId,
	b3MakeBoxHull,
	b3MakeQuatFromAxisAngle,
	b3MeshData,
	b3Normalize,
	b3Quat,
	b3Sphere,
	b3Vec3,
	b3Vec3_one,
	b3WorldId,
	b3World_GetContactEvents,
	b3World_GetJointEvents,
	b3World_GetSensorEvents,
	b3World_EnableRestitutionPropagation,
	b3World_EnableWarmStarting,
	b3World_Step,
	b3_dynamicBody,
} from "../../lib/box3d";
import { b3GetWorldFromId } from "../../lib/physics_world";

export { frameBoundary } from "./common";

const MAX_BODIES = 128;

let g_worldId = b3WorldId();
let g_bodies = new StaticArray<b3BodyId>(MAX_BODIES);
let g_bodyCount = 0;
let g_mesh: b3MeshData? = null;
let g_joint = b3JointId();
let g_scene = 0;

function addBody(def: b3BodyDef): b3BodyId {
	let id = b3CreateBody(g_worldId, def);
	g_bodies[g_bodyCount] = id;
	g_bodyCount += 1;
	return id;
}

function pv3(v: b3Vec3): void {
	vf32(v.x);
	vf32(v.y);
	vf32(v.z);
}

function pquat(q: b3Quat): void {
	pv3(q.v);
	vf32(q.s);
}

function printBodies(): void {
	for (let i = 0; i < g_bodyCount; ++i) {
		let id = g_bodies[i];
		vtag("body");
		vi32(i);
		let valid = b3Body_IsValid(id);
		vbool(valid);
		if (valid) {
			pv3(b3Body_GetPosition(id));
			pquat(b3Body_GetRotation(id));
			pv3(b3Body_GetLinearVelocity(id));
			pv3(b3Body_GetAngularVelocity(id));
			vbool(b3Body_IsAwake(id));
		}
		vend();
	}
}

function printSolver(): void {
	let world = b3GetWorldFromId(g_worldId);

	vtag("sets");
	vi32(world.solverSets.length);
	vi32(world.splitIslandId);
	vend();
	for (let i = 0; i < world.solverSets.length; ++i) {
		let set = world.solverSets[i];
		vtag("set");
		vi32(i);
		vi32(set.setIndex);
		vi32(set.bodySims.length);
		vi32(set.bodyStates.length);
		vi32(set.jointSims.length);
		vi32(set.contactIndices.length);
		vi32(set.islandSims.length);
		vend();

		if (set.setIndex == B3_NULL_INDEX) {
			continue;
		}

		vtag("setBodies");
		for (let j = 0; j < set.bodySims.length; ++j) {
			vi32(set.bodySims[j].bodyId);
		}
		vend();

		vtag("setFlags");
		for (let j = 0; j < set.bodyStates.length; ++j) {
			vu32(set.bodyStates[j].flags);
		}
		for (let j = 0; j < set.bodySims.length; ++j) {
			vu32(set.bodySims[j].flags);
		}
		vend();

		vtag("setContacts");
		for (let j = 0; j < set.contactIndices.length; ++j) {
			vi32(set.contactIndices[j]);
		}
		vend();

		vtag("setJoints");
		for (let j = 0; j < set.jointSims.length; ++j) {
			vi32(set.jointSims[j].jointId);
		}
		vend();

		vtag("setIslands");
		for (let j = 0; j < set.islandSims.length; ++j) {
			vi32(set.islandSims[j].islandId);
		}
		vend();
	}

	for (let i = 0; i < B3_GRAPH_COLOR_COUNT; ++i) {
		let color = world.constraintGraph.colors[i];
		vtag("color");
		vi32(i);
		for (let j = 0; j < color.convexContacts.length; ++j) {
			vi32(color.convexContacts[j]);
		}
		vi32(-2);
		for (let j = 0; j < color.contacts.length; ++j) {
			vi32(color.contacts[j].contactId);
			vi32(<i32>color.contacts[j].manifoldCount);
		}
		vi32(-2);
		for (let j = 0; j < color.jointSims.length; ++j) {
			vi32(color.jointSims[j].jointId);
		}
		vend();
	}

	vtag("islands");
	vi32(world.islands.length);
	vend();
	for (let i = 0; i < world.islands.length; ++i) {
		let island = world.islands[i];
		if (island.setIndex == B3_NULL_INDEX) {
			continue;
		}

		vtag("island");
		vi32(i);
		vi32(island.islandId);
		vi32(island.setIndex);
		vi32(island.localIndex);
		vi32(island.constraintRemoveCount);
		vi32(-2);
		for (let j = 0; j < island.bodies.length; ++j) {
			vi32(island.bodies[j]);
		}
		vi32(-2);
		for (let j = 0; j < island.contacts.length; ++j) {
			vi32(island.contacts[j].contactId);
		}
		vi32(-2);
		for (let j = 0; j < island.joints.length; ++j) {
			vi32(island.joints[j].jointId);
		}
		vend();
	}

	for (let i = 0; i < world.bodies.length; ++i) {
		let body = world.bodies[i];
		if (body.setIndex == B3_NULL_INDEX) {
			continue;
		}

		vtag("b");
		vi32(i);
		vi32(body.setIndex);
		vi32(body.localIndex);
		vi32(body.islandId);
		vi32(body.islandIndex);
		vi32(body.bodyMoveIndex);
		vf32(body.sleepTime);
		vf32(body.sleepVelocity);
		vu32(body.flags);
		vend();
	}

	for (let i = 0; i < world.joints.length; ++i) {
		let joint = world.joints[i];
		if (joint.setIndex == B3_NULL_INDEX) {
			continue;
		}

		vtag("j");
		vi32(i);
		vi32(joint.setIndex);
		vi32(joint.colorIndex);
		vi32(joint.localIndex);
		vi32(joint.islandId);
		vi32(joint.islandIndex);
		vend();
	}

	for (let i = 0; i < world.contacts.length; ++i) {
		let contact = world.contacts[i];
		if (contact.setIndex == B3_NULL_INDEX) {
			continue;
		}

		vtag("c");
		vi32(i);
		vi32(contact.setIndex);
		vi32(contact.colorIndex);
		vi32(contact.localIndex);
		vi32(contact.islandId);
		vi32(contact.islandIndex);
		vi32(contact.encodedBodySimA);
		vi32(contact.encodedBodySimB);
		vu32(contact.flags);
		vi32(contact.manifoldCount);
		for (let m = 0; m < contact.manifoldCount; ++m) {
			ref manifold = contact.manifolds[m];
			vi32(manifold.pointCount);
			pv3(manifold.normal);
			vf32(manifold.twistImpulse);
			pv3(manifold.frictionImpulse);
			pv3(manifold.rollingImpulse);
			for (let p = 0; p < manifold.pointCount; ++p) {
				ref mp = manifold.points[p];
				vf32(mp.normalImpulse);
				vf32(mp.totalNormalImpulse);
				vf32(mp.normalVelocity);
			}
		}
		vend();
	}
}

function printEvents(): void {
	let contactEvents = b3World_GetContactEvents(g_worldId);
	vtag("contactEvents");
	vi32(contactEvents.beginCount);
	vi32(contactEvents.endCount);
	vi32(contactEvents.hitCount);
	vend();
	for (let i = 0; i < contactEvents.hitCount; ++i) {
		let e = contactEvents.hitEvents[i];
		vtag("hit");
		vi32(e.shapeIdA.index1);
		vi32(e.shapeIdB.index1);
		vi32(e.contactId.index1);
		vu32(e.contactId.generation);
		pv3(e.point);
		pv3(e.normal);
		vf32(e.approachSpeed);
		vu64(e.userMaterialIdA);
		vu64(e.userMaterialIdB);
		vend();
	}

	let jointEvents = b3World_GetJointEvents(g_worldId);
	vtag("jointEvents");
	vi32(jointEvents.count);
	for (let i = 0; i < jointEvents.count; ++i) {
		vi32(jointEvents.jointEvents[i].jointId.index1);
		vi32(jointEvents.jointEvents[i].jointId.generation);
	}
	vend();

	let sensorEvents = b3World_GetSensorEvents(g_worldId);
	vtag("sensorEvents");
	vi32(sensorEvents.beginCount);
	for (let i = 0; i < sensorEvents.beginCount; ++i) {
		vi32(sensorEvents.beginEvents[i].sensorShapeId.index1);
		vi32(sensorEvents.beginEvents[i].visitorShapeId.index1);
	}
	vi32(sensorEvents.endCount);
	vend();
}

function createGround(hx: f32, hy: f32, hz: f32): b3BodyId {
	let bodyDef = b3DefaultBodyDef();
	bodyDef.position = b3Vec3(0.0, -hy, 0.0);
	let groundId = addBody(bodyDef);

	let shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.6;
	let box = b3MakeBoxHull(hx, hy, hz);
	b3CreateHullShape(groundId, shapeDef, box.base);
	return groundId;
}

function createOverflow(): void {
	createGround(20.0, 1.0, 20.0);

	let shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.6;

	let small = b3MakeBoxHull(0.25, 0.25, 0.25);
	for (let i = 0; i < 5; ++i) {
		for (let j = 0; j < 5; ++j) {
			let bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = b3Vec3(-1.2 + 0.6 * <f32>i, 0.25, -1.2 + 0.6 * <f32>j);
			let bodyId = addBody(bodyDef);
			b3CreateHullShape(bodyId, shapeDef, small.base);
		}
	}

	// plank, index 26
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(0.0, 1.4, 0.0);
		let bodyId = addBody(bodyDef);
		let plankDef = b3DefaultShapeDef();
		plankDef.baseMaterial.friction = 0.6;
		plankDef.baseMaterial.restitution = 0.3;
		plankDef.enableHitEvents = true;
		let plank = b3MakeBoxHull(1.6, 0.1, 1.6);
		b3CreateHullShape(bodyId, plankDef, plank.base);
	}

	// bouncing sphere with hit events, index 27
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(4.0, 3.0, 0.0);
		let bodyId = addBody(bodyDef);
		let ballDef = b3DefaultShapeDef();
		ballDef.baseMaterial.restitution = 0.8;
		ballDef.enableHitEvents = true;
		ballDef.baseMaterial.userMaterialId = 7;
		let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.3);
		b3CreateSphereShape(bodyId, ballDef, sphere);
	}

	// rolling capsule with rolling resistance, index 28
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(-4.0, 0.25, 0.0);
		bodyDef.linearVelocity = b3Vec3(0.0, 0.0, 2.0);
		bodyDef.angularVelocity = b3Vec3(8.0, 0.0, 0.0);
		let bodyId = addBody(bodyDef);
		let capsuleDef = b3DefaultShapeDef();
		capsuleDef.baseMaterial.rollingResistance = 0.3;
		let capsule = b3Capsule(b3Vec3(-0.4, 0.0, 0.0), b3Vec3(0.4, 0.0, 0.0), 0.25);
		b3CreateCapsuleShape(bodyId, capsuleDef, capsule);
	}

	// tumbling rod for gyroscopic torque, index 29
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(0.0, 4.0, 4.0);
		bodyDef.angularVelocity = b3Vec3(0.3, 6.0, 0.5);
		let bodyId = addBody(bodyDef);
		let rod = b3MakeBoxHull(1.0, 0.05, 0.1);
		b3CreateHullShape(bodyId, shapeDef, rod.base);
	}

	// sliding box with angular locks, index 30
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(3.0, 0.5, 4.0);
		bodyDef.linearVelocity = b3Vec3(3.0, 0.0, 1.0);
		bodyDef.angularVelocity = b3Vec3(1.0, 2.0, 3.0);
		bodyDef.motionLocks.angularX = true;
		bodyDef.motionLocks.angularZ = true;
		let bodyId = addBody(bodyDef);
		let box = b3MakeBoxHull(0.5, 0.5, 0.5);
		b3CreateHullShape(bodyId, shapeDef, box.base);
	}
}

function stepOverflow(stepIndex: i32): void {
	if (stepIndex == 140) {
		b3Body_ApplyLinearImpulse(g_bodies[26], b3Vec3(0.0, 2.0, 0.5), b3Vec3(1.0, 0.7, 1.0), true);
	}

	if (stepIndex == 150) {
		b3DestroyBody(g_bodies[13]);
	}

	if (stepIndex == 200) {
		b3Body_SetAwake(g_bodies[27], true);
		b3Body_ApplyLinearImpulse(g_bodies[27], b3Vec3(0.0, 3.0, 0.0), b3Vec3(4.0, 0.3, 0.0), true);
	}
}

function createMesh(): void {
	let groundDef = b3DefaultBodyDef();
	let groundId = addBody(groundDef);
	let mesh = b3CreateWaveMesh(16, 16, 0.5, 0.3, 0.5, 0.7)!;
	g_mesh = mesh;
	let groundShapeDef = b3DefaultShapeDef();
	b3CreateMeshShape(groundId, groundShapeDef, mesh, b3Vec3_one);

	let shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.5;
	shapeDef.baseMaterial.rollingResistance = 0.05;
	shapeDef.baseMaterial.restitution = 0.4;
	shapeDef.enableHitEvents = true;

	for (let i = 0; i < 3; ++i) {
		for (let j = 0; j < 3; ++j) {
			let x: f32 = 1.0 + 1.5 * <f32>i;
			let z: f32 = 1.0 + 1.5 * <f32>j;
			let kind = (i + j) % 3;

			let bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = b3Vec3(x, 1.5 + 0.5 * <f32>j, z);
			bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Normalize(b3Vec3(1.0, 0.5, 0.2 * <f32>i)), 0.3 * <f32>j);
			let bodyId = addBody(bodyDef);

			if (kind == 0) {
				let box = b3MakeBoxHull(0.6, 0.2, 0.4);
				b3CreateHullShape(bodyId, shapeDef, box.base);
			} else if (kind == 1) {
				let capsule = b3Capsule(b3Vec3(-0.5, 0.0, 0.0), b3Vec3(0.5, 0.0, 0.0), 0.2);
				b3CreateCapsuleShape(bodyId, shapeDef, capsule);
			} else {
				let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.35);
				b3CreateSphereShape(bodyId, shapeDef, sphere);
			}
		}
	}

	// fast sphere
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(6.0, 6.0, 6.0);
		bodyDef.linearVelocity = b3Vec3(1.0, -70.0, 0.5);
		let bodyId = addBody(bodyDef);
		let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.1);
		b3CreateSphereShape(bodyId, shapeDef, sphere);
	}
}

function stepMesh(stepIndex: i32): void {
	if (stepIndex == 60) {
		b3World_EnableRestitutionPropagation(g_worldId, true);
	}

	if (stepIndex == 100) {
		b3Body_ApplyLinearImpulse(g_bodies[1], b3Vec3(1.0, 2.0, 0.0), b3Vec3(1.0, 1.0, 1.0), true);
	}
}

function createJoints(): void {
	createGround(20.0, 1.0, 20.0);

	let shapeDef = b3DefaultShapeDef();
	let box = b3MakeBoxHull(0.5, 0.5, 0.5);

	// sleepers, indices 1 and 2
	for (let i = 0; i < 2; ++i) {
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(-6.0 + 3.0 * <f32>i, 0.5, -6.0);
		let bodyId = addBody(bodyDef);
		b3CreateHullShape(bodyId, shapeDef, box.base);
	}

	// hub, index 3, with 24 spokes, indices 4 to 27
	let hubId: b3BodyId;
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(0.0, 6.0, 0.0);
		bodyDef.gravityScale = 0.0;
		bodyDef.angularVelocity = b3Vec3(0.0, 1.0, 0.2);
		hubId = addBody(bodyDef);
		let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.5);
		b3CreateSphereShape(hubId, shapeDef, sphere);
	}

	for (let i = 0; i < 24; ++i) {
		let angle: f32 = (2.0 * B3_PI * <f32>i) / 24.0;
		let cs = b3ComputeCosSin(angle);

		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(2.0 * cs.cosine, 6.0 + 0.1 * <f32>(i % 3), 2.0 * cs.sine);
		bodyDef.gravityScale = 0.0;
		let bodyId = addBody(bodyDef);
		let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.15);
		b3CreateSphereShape(bodyId, shapeDef, sphere);

		let jointDef = b3DefaultDistanceJointDef();
		jointDef.base.bodyIdA = hubId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.length = 2.0;
		jointDef.enableSpring = i % 2 == 0;
		jointDef.hertz = 2.0;
		jointDef.dampingRatio = 0.3;
		b3CreateDistanceJoint(g_worldId, jointDef);
	}

	// revolute chain hanging from a static anchor, indices 28 to 34
	{
		let anchorDef = b3DefaultBodyDef();
		anchorDef.position = b3Vec3(6.0, 8.0, 0.0);
		let prevId = addBody(anchorDef);

		let capsule = b3Capsule(b3Vec3(-0.4, 0.0, 0.0), b3Vec3(0.4, 0.0, 0.0), 0.1);
		for (let i = 0; i < 6; ++i) {
			let bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = b3Vec3(6.5 + 1.0 * <f32>i, 8.0, 0.0);
			let bodyId = addBody(bodyDef);
			b3CreateCapsuleShape(bodyId, shapeDef, capsule);

			let jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = prevId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = i == 0 ? b3Vec3(0.0, 0.0, 0.0) : b3Vec3(0.5, 0.0, 0.0);
			jointDef.base.localFrameB.p = b3Vec3(-0.5, 0.0, 0.0);
			jointDef.base.forceThreshold = 20.0;
			jointDef.base.torqueThreshold = 100.0;
			let jointId = b3CreateRevoluteJoint(g_worldId, jointDef);
			if (i == 3) {
				g_joint = jointId;
			}
			prevId = bodyId;
		}
	}

	// spherical pendulum and a welded pair, indices 35 to 37
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(-4.0, 4.0, 3.0);
		let ballId = addBody(bodyDef);
		let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.3);
		b3CreateSphereShape(ballId, shapeDef, sphere);

		let jointDef = b3DefaultSphericalJointDef();
		jointDef.base.bodyIdA = g_bodies[0];
		jointDef.base.bodyIdB = ballId;
		jointDef.base.localFrameA.p = b3Vec3(-4.0, 6.0, 2.0);
		jointDef.base.localFrameB.p = b3Vec3(0.0, 0.0, 0.0);
		b3CreateSphericalJoint(g_worldId, jointDef);

		bodyDef.position = b3Vec3(4.0, 0.5, 4.0);
		let lowerId = addBody(bodyDef);
		b3CreateHullShape(lowerId, shapeDef, box.base);

		bodyDef.position = b3Vec3(4.0, 1.5, 4.0);
		bodyDef.angularVelocity = b3Vec3(0.0, 2.0, 0.0);
		let upperId = addBody(bodyDef);
		b3CreateHullShape(upperId, shapeDef, box.base);

		let weldDef = b3DefaultWeldJointDef();
		weldDef.base.bodyIdA = lowerId;
		weldDef.base.bodyIdB = upperId;
		weldDef.base.localFrameA.p = b3Vec3(0.0, 0.5, 0.0);
		weldDef.base.localFrameB.p = b3Vec3(0.0, -0.5, 0.0);
		weldDef.angularHertz = 5.0;
		weldDef.angularDampingRatio = 0.5;
		b3CreateWeldJoint(g_worldId, weldDef);
	}
}

function stepJoints(stepIndex: i32): void {
	if (stepIndex == 70) {
		let jointDef = b3DefaultDistanceJointDef();
		jointDef.base.bodyIdA = g_bodies[1];
		jointDef.base.bodyIdB = g_bodies[2];
		jointDef.length = 3.0;
		b3CreateDistanceJoint(g_worldId, jointDef);
	}

	if (stepIndex == 100) {
		b3Body_ApplyLinearImpulse(g_bodies[1], b3Vec3(0.0, 4.0, 1.0), b3Vec3(-6.0, 0.8, -6.2), true);
	}

	if (stepIndex == 130) {
		b3Body_Disable(g_bodies[3]);
	}

	if (stepIndex == 150) {
		b3Body_Enable(g_bodies[3]);
	}

	if (stepIndex == 170) {
		b3DestroyJoint(g_joint, true);
	}

	if (stepIndex == 190) {
		b3DestroyBody(g_bodies[30]);
	}
}

function createBullets(): void {
	createGround(30.0, 1.0, 30.0);

	let shapeDef = b3DefaultShapeDef();
	shapeDef.enableSensorEvents = true;
	let box = b3MakeBoxHull(0.5, 0.5, 0.5);

	for (let i = 0; i < 4; ++i) {
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(0.0, 0.5 + 1.0 * <f32>i, 0.0);
		let bodyId = addBody(bodyDef);
		b3CreateHullShape(bodyId, shapeDef, box.base);
	}

	// thin wall
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(0.0, 2.0, -10.0);
		let wallId = addBody(bodyDef);
		let wall = b3MakeBoxHull(3.0, 2.0, 0.05);
		b3CreateHullShape(wallId, shapeDef, wall.base);
	}

	// sensor in the capsule path
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(0.0, 1.0, -6.0);
		let sensorId = addBody(bodyDef);
		let sensorDef = b3DefaultShapeDef();
		sensorDef.isSensor = true;
		sensorDef.enableSensorEvents = true;
		let slab = b3MakeBoxHull(2.0, 1.0, 0.02);
		b3CreateHullShape(sensorId, sensorDef, slab.base);
	}

	// fast capsule, not a bullet
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(0.3, 1.0, 2.0);
		bodyDef.linearVelocity = b3Vec3(0.0, 0.5, -150.0);
		bodyDef.angularVelocity = b3Vec3(3.0, 0.0, 0.0);
		let bodyId = addBody(bodyDef);
		let capsule = b3Capsule(b3Vec3(-0.3, 0.0, 0.0), b3Vec3(0.3, 0.0, 0.0), 0.1);
		b3CreateCapsuleShape(bodyId, shapeDef, capsule);
	}

	// bullet
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.position = b3Vec3(0.1, 1.5, 12.0);
		bodyDef.linearVelocity = b3Vec3(0.0, 0.0, -120.0);
		let bodyId = addBody(bodyDef);
		let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.15);
		b3CreateSphereShape(bodyId, shapeDef, sphere);
	}
}

function stepBullets(stepIndex: i32): void {
	if (stepIndex == 40) {
		b3World_EnableWarmStarting(g_worldId, false);
	}
}

const g_stepCounts: StaticArray<i32> = [240, 160, 220, 80];

export function stepCount(scene: i32): i32 {
	return g_stepCounts[scene];
}

export function startScene(scene: i32): void {
	g_scene = scene;
	let worldDef = b3DefaultWorldDef();
	g_worldId = b3CreateWorld(worldDef);
	g_bodyCount = 0;
	g_mesh = null;

	if (scene == 0) {
		createOverflow();
	} else if (scene == 1) {
		createMesh();
	} else if (scene == 2) {
		createJoints();
	} else {
		createBullets();
	}

	vtag("scene");
	vi32(scene);
	vi32(g_bodyCount);
	vend();
}

export function stepScene(i: i32): void {
	let scene = g_scene;
	if (scene == 0) {
		stepOverflow(i);
	} else if (scene == 1) {
		stepMesh(i);
	} else if (scene == 2) {
		stepJoints(i);
	} else {
		stepBullets(i);
	}

	b3World_Step(g_worldId, 1.0 / 60.0, 4);

	vtag("step");
	vi32(scene);
	vi32(i);
	vend();

	printBodies();
	printSolver();
	printEvents();
}

export function endScene(): void {
	b3DestroyWorld(g_worldId);
	let mesh = g_mesh;
	if (mesh != null) {
		b3DestroyMesh(mesh);
	}
	g_mesh = null;
}
