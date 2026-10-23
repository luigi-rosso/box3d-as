// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/physics_world.as, mirrors verify/cref/world.c. Uses the public API through
// lib/box3d.as, so it builds once every module the scene touches is ported.

import { vtag, vi32, vu32, vbool, vf32, vu64, vend } from "./common";
import { FLT_MAX } from "../../lib/core";
import {
	B3_CONTACT_MANIFOLD_COUNT_BUCKETS,
	B3_GRAPH_COLOR_COUNT,
	b3AABB,
	b3BodyDef,
	b3BodyId,
	b3Capsule,
	b3CollisionPlane,
	b3ContactId,
	b3CreateBody,
	b3CreateCapsuleShape,
	b3CreateHullShape,
	b3CreateRevoluteJoint,
	b3CreateSphereShape,
	b3CreateWorld,
	b3ClipVector,
	b3Body_GetAngularVelocity,
	b3Body_GetLinearVelocity,
	b3Body_GetPosition,
	b3Body_GetRotation,
	b3Body_IsAwake,
	b3Body_IsValid,
	b3Contact_IsValid,
	b3DefaultBodyDef,
	b3DefaultExplosionDef,
	b3DefaultQueryFilter,
	b3DefaultRevoluteJointDef,
	b3DefaultShapeDef,
	b3DefaultWorldDef,
	b3DestroyBody,
	b3DestroyWorld,
	b3GetMaxWorldCount,
	b3GetWorldCount,
	b3MakeBoxHull,
	b3MakeQuatFromAxisAngle,
	b3Normalize,
	b3PlaneResult,
	b3Quat,
	b3ShapeId,
	b3ShapeProxy,
	b3SolvePlanes,
	b3Sphere,
	b3TreeStats,
	b3Vec3,
	b3WorldId,
	b3World_CastMover,
	b3World_CastRay,
	b3World_CastRayClosest,
	b3World_CastShape,
	b3World_CollideMover,
	b3World_EnableContinuous,
	b3World_EnableRestitutionPropagation,
	b3World_EnableSleeping,
	b3World_EnableWarmStarting,
	b3World_Explode,
	b3World_GetAwakeBodyCount,
	b3World_GetBodyEvents,
	b3World_GetBounds,
	b3World_GetContactEvents,
	b3World_GetContactRecycleDistance,
	b3World_GetCounters,
	b3World_GetGravity,
	b3World_GetHitEventThreshold,
	b3World_GetIsotropicInertiaTolerance,
	b3World_GetJointEvents,
	b3World_GetMaxCapacity,
	b3World_GetMaximumLinearSpeed,
	b3World_GetRestitutionIterations,
	b3World_GetRestitutionThreshold,
	b3World_GetSensorEvents,
	b3World_GetUserData,
	b3World_GetWorkerCount,
	b3World_IsContinuousEnabled,
	b3World_IsRestitutionPropagationEnabled,
	b3World_IsSleepingEnabled,
	b3World_IsValid,
	b3World_IsWarmStartingEnabled,
	b3World_OverlapAABB,
	b3World_OverlapShape,
	b3World_SetContactRecycleDistance,
	b3World_SetContactTuning,
	b3World_SetFrictionCallback,
	b3World_SetGravity,
	b3World_SetHitEventThreshold,
	b3World_SetIsotropicInertiaTolerance,
	b3World_SetMaximumLinearSpeed,
	b3World_SetRestitutionIterations,
	b3World_SetRestitutionThreshold,
	b3World_SetUserData,
	b3World_Step,
	b3_dynamicBody,
	b3_kinematicBody,
} from "../../lib/box3d";

export { frameBoundary } from "./common";

const MAX_BODIES = 64;
const PLANE_CAPACITY = 16;

let g_worldId = b3WorldId();
let g_bodies = new StaticArray<b3BodyId>(MAX_BODIES);
let g_bodyCount = 0;
let g_planes = new StaticArray<b3CollisionPlane>(PLANE_CAPACITY);
let g_planeCount = 0;

// C's int* contexts
class QueryState {
	count: i32 = 0;
	mode: i32 = 0;
}
let g_state = new QueryState();

function pv3(v: b3Vec3): void {
	vf32(v.x);
	vf32(v.y);
	vf32(v.z);
}

function pquat(q: b3Quat): void {
	pv3(q.v);
	vf32(q.s);
}

function pworldId(id: b3WorldId): void {
	vi32(id.index1);
	vi32(id.generation);
}

function pshapeId(id: b3ShapeId): void {
	vi32(id.index1);
	vi32(id.world0);
	vi32(id.generation);
}

function pbodyId(id: b3BodyId): void {
	vi32(id.index1);
	vi32(id.world0);
	vi32(id.generation);
}

function pcontactId(id: b3ContactId): void {
	vi32(id.index1);
	vi32(id.world0);
	vu32(id.generation);
}

function pstats(stats: b3TreeStats): void {
	vi32(stats.nodeVisits);
	vi32(stats.leafVisits);
}

function paabb(a: b3AABB): void {
	pv3(a.lowerBound);
	pv3(a.upperBound);
}

function addBody(def: b3BodyDef): b3BodyId {
	let id = b3CreateBody(g_worldId, def);
	g_bodies[g_bodyCount] = id;
	g_bodyCount += 1;
	return id;
}

function minFriction(frictionA: f32, materialA: u64, frictionB: f32, materialB: u64): f32 {
	return frictionA < frictionB ? frictionA : frictionB;
}

export function registry(): void {
	vtag("registry");
	vi32(b3GetWorldCount());
	vend();

	let def = b3DefaultWorldDef();
	let a = b3CreateWorld(def);
	let b = b3CreateWorld(def);

	vtag("created");
	pworldId(a);
	pworldId(b);
	vbool(b3World_IsValid(a));
	vbool(b3World_IsValid(b));
	vi32(b3GetWorldCount());
	vi32(b3GetMaxWorldCount());
	vend();

	b3DestroyWorld(b);
	let c = b3CreateWorld(def);

	vtag("recreated");
	vbool(b3World_IsValid(b));
	pworldId(c);
	vbool(b3World_IsValid(c));
	vi32(b3GetWorldCount());
	vend();

	let bad = b3WorldId();
	let far = b3WorldId();
	far.index1 = 200;
	vtag("invalid");
	vbool(b3World_IsValid(bad));
	vbool(b3World_IsValid(far));
	vend();

	b3World_SetRestitutionThreshold(c, -1.0);
	b3World_SetRestitutionIterations(c, 100);
	b3World_SetHitEventThreshold(c, 2.5);
	b3World_SetContactTuning(c, -3.0, 4.0, 5.0);
	b3World_SetContactRecycleDistance(c, -0.1);
	b3World_SetMaximumLinearSpeed(c, 123.0);
	b3World_SetIsotropicInertiaTolerance(c, 3.0);
	b3World_EnableSleeping(c, false);
	b3World_EnableWarmStarting(c, false);
	b3World_EnableContinuous(c, false);
	b3World_EnableRestitutionPropagation(c, true);
	b3World_SetUserData(c, 77);
	b3World_SetGravity(c, b3Vec3(1.0, -2.0, 3.0));

	vtag("settings");
	vf32(b3World_GetRestitutionThreshold(c));
	vi32(b3World_GetRestitutionIterations(c));
	vf32(b3World_GetHitEventThreshold(c));
	vf32(b3World_GetContactRecycleDistance(c));
	vf32(b3World_GetMaximumLinearSpeed(c));
	vf32(b3World_GetIsotropicInertiaTolerance(c));
	vbool(b3World_IsSleepingEnabled(c));
	vbool(b3World_IsWarmStartingEnabled(c));
	vbool(b3World_IsContinuousEnabled(c));
	vbool(b3World_IsRestitutionPropagationEnabled(c));
	vu64(b3World_GetUserData(c));
	pv3(b3World_GetGravity(c));
	vi32(b3World_GetWorkerCount(c));
	vi32(b3World_GetAwakeBodyCount(c));
	paabb(b3World_GetBounds(c));
	vend();

	b3DestroyWorld(c);
	b3DestroyWorld(a);

	vtag("destroyed");
	vbool(b3World_IsValid(a));
	vbool(b3World_IsValid(c));
	vi32(b3GetWorldCount());
	vi32(b3GetMaxWorldCount());
	vend();
}

export function createScene(): void {
	let worldDef = b3DefaultWorldDef();
	g_worldId = b3CreateWorld(worldDef);
	g_bodyCount = 0;

	b3World_SetFrictionCallback(g_worldId, minFriction);

	// Ground
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(0.0, -1.0, 0.0);
		let groundId = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.8;
		shapeDef.baseMaterial.userMaterialId = 11;
		let box = b3MakeBoxHull(20.0, 1.0, 20.0);
		b3CreateHullShape(groundId, shapeDef, box.base);
	}

	let shapeDef = b3DefaultShapeDef();
	shapeDef.enableContactEvents = true;
	shapeDef.enableSensorEvents = true;
	shapeDef.baseMaterial.userMaterialId = 22;

	// Box stack
	let box = b3MakeBoxHull(0.5, 0.5, 0.5);
	let hingeA = b3BodyId();
	let hingeB = b3BodyId();
	for (let i = 0; i < 5; ++i) {
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(0.02 * <f32>i, 0.5 + 1.0 * <f32>i, 0.0);
		let bodyId = addBody(bodyDef);
		b3CreateHullShape(bodyId, shapeDef, box.base);
		if (i == 3) {
			hingeA = bodyId;
		}
		if (i == 4) {
			hingeB = bodyId;
		}
	}

	// Spheres with hit events
	shapeDef.enableHitEvents = true;
	for (let i = 0; i < 4; ++i) {
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(3.0 + 1.1 * <f32>i, 2.0 + <f32>i, 0.3 * <f32>i);
		bodyDef.linearVelocity = b3Vec3(-1.0, -2.0 * <f32>i, 0.0);
		let bodyId = addBody(bodyDef);
		let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.4);
		b3CreateSphereShape(bodyId, shapeDef, sphere);
	}
	shapeDef.enableHitEvents = false;

	// Capsules, rotated
	for (let i = 0; i < 3; ++i) {
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(-3.0, 1.0 + 0.8 * <f32>i, 0.1 * <f32>i);
		let axis = b3Normalize(b3Vec3(0.3, 1.0, 0.2 * <f32>i));
		bodyDef.rotation = b3MakeQuatFromAxisAngle(axis, 0.4 * <f32>i);
		let bodyId = addBody(bodyDef);
		let capsule = b3Capsule(b3Vec3(-0.5, 0.0, 0.0), b3Vec3(0.5, 0.0, 0.0), 0.25);
		b3CreateCapsuleShape(bodyId, shapeDef, capsule);
	}

	// Kinematic box sweeping through the capsules
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_kinematicBody;
		bodyDef.position = b3Vec3(-8.0, 0.6, 0.0);
		bodyDef.linearVelocity = b3Vec3(2.0, 0.0, 0.0);
		let bodyId = addBody(bodyDef);
		let kbox = b3MakeBoxHull(0.5, 0.4, 0.5);
		b3CreateHullShape(bodyId, shapeDef, kbox.base);
	}

	// Bullet
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.position = b3Vec3(0.0, 3.0, -12.0);
		bodyDef.linearVelocity = b3Vec3(0.0, 0.0, 60.0);
		let bodyId = addBody(bodyDef);
		let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.1);
		b3CreateSphereShape(bodyId, shapeDef, sphere);
	}

	// Sensor
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(5.0, 0.5, 0.0);
		let bodyId = addBody(bodyDef);
		let sensorDef = b3DefaultShapeDef();
		sensorDef.isSensor = true;
		sensorDef.enableSensorEvents = true;
		let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 1.5);
		b3CreateSphereShape(bodyId, sensorDef, sphere);
	}

	// Hinge between the top two boxes
	{
		let jointDef = b3DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = hingeA;
		jointDef.base.bodyIdB = hingeB;
		jointDef.base.localFrameA.p = b3Vec3(0.5, 0.5, 0.0);
		jointDef.base.localFrameB.p = b3Vec3(0.5, -0.5, 0.0);
		jointDef.base.forceThreshold = 10.0;
		jointDef.base.torqueThreshold = 10.0;
		b3CreateRevoluteJoint(g_worldId, jointDef);
	}
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

function printCounters(): void {
	let s = b3World_GetCounters(g_worldId);
	vtag("counters");
	vi32(s.bodyCount);
	vi32(s.shapeCount);
	vi32(s.contactCount);
	vi32(s.jointCount);
	vi32(s.islandCount);
	vi32(s.staticTreeHeight);
	vi32(s.treeHeight);
	vi32(s.satCallCount);
	vi32(s.satCacheHitCount);
	vi32(s.awakeContactCount);
	vi32(s.recycledContactCount);
	vi32(s.distanceIterations);
	vi32(s.pushBackIterations);
	vi32(s.rootIterations);
	for (let i = 0; i < B3_GRAPH_COLOR_COUNT; ++i) {
		vi32(s.colorCounts[i]);
	}
	for (let i = 0; i < B3_CONTACT_MANIFOLD_COUNT_BUCKETS; ++i) {
		vi32(s.manifoldCounts[i]);
	}
	vi32(b3World_GetAwakeBodyCount(g_worldId));
	vend();

	let c = b3World_GetMaxCapacity(g_worldId);
	vtag("capacity");
	vi32(c.staticShapeCount);
	vi32(c.dynamicShapeCount);
	vi32(c.staticBodyCount);
	vi32(c.dynamicBodyCount);
	vi32(c.contactCount);
	vend();
}

function printEvents(): void {
	let contactEvents = b3World_GetContactEvents(g_worldId);
	for (let i = 0; i < contactEvents.beginCount; ++i) {
		let e = contactEvents.beginEvents[i];
		vtag("begin");
		pshapeId(e.shapeIdA);
		pshapeId(e.shapeIdB);
		pcontactId(e.contactId);
		vbool(b3Contact_IsValid(e.contactId));
		vend();
	}
	for (let i = 0; i < contactEvents.endCount; ++i) {
		let e = contactEvents.endEvents[i];
		vtag("end");
		pshapeId(e.shapeIdA);
		pshapeId(e.shapeIdB);
		pcontactId(e.contactId);
		vend();
	}
	for (let i = 0; i < contactEvents.hitCount; ++i) {
		let e = contactEvents.hitEvents[i];
		vtag("hit");
		pshapeId(e.shapeIdA);
		pshapeId(e.shapeIdB);
		pcontactId(e.contactId);
		pv3(e.point);
		pv3(e.normal);
		vf32(e.approachSpeed);
		vu64(e.userMaterialIdA);
		vu64(e.userMaterialIdB);
		vend();
	}

	let sensorEvents = b3World_GetSensorEvents(g_worldId);
	for (let i = 0; i < sensorEvents.beginCount; ++i) {
		vtag("sensorBegin");
		pshapeId(sensorEvents.beginEvents[i].sensorShapeId);
		pshapeId(sensorEvents.beginEvents[i].visitorShapeId);
		vend();
	}
	for (let i = 0; i < sensorEvents.endCount; ++i) {
		vtag("sensorEnd");
		pshapeId(sensorEvents.endEvents[i].sensorShapeId);
		pshapeId(sensorEvents.endEvents[i].visitorShapeId);
		vend();
	}

	let bodyEvents = b3World_GetBodyEvents(g_worldId);
	vtag("moves");
	vi32(bodyEvents.moveCount);
	for (let i = 0; i < bodyEvents.moveCount; ++i) {
		pbodyId(bodyEvents.moveEvents[i].bodyId);
		vbool(bodyEvents.moveEvents[i].fellAsleep);
		pv3(bodyEvents.moveEvents[i].transform.p);
	}
	vend();

	let jointEvents = b3World_GetJointEvents(g_worldId);
	vtag("joints");
	vi32(jointEvents.count);
	vend();
}

function overlapFcn(shapeId: b3ShapeId): bool {
	g_state.count += 1;
	vtag("overlap");
	pshapeId(shapeId);
	vend();
	return true;
}

function castFcn(
	shapeId: b3ShapeId,
	point: b3Vec3,
	normal: b3Vec3,
	fraction: f32,
	userMaterialId: u64,
	triangleIndex: i32,
	childIndex: i32,
): f32 {
	vtag("cast");
	pshapeId(shapeId);
	pv3(point);
	pv3(normal);
	vf32(fraction);
	vu64(userMaterialId);
	vi32(triangleIndex);
	vi32(childIndex);
	vend();

	if (g_state.mode == 0) {
		return fraction;
	}

	if (g_state.mode == 1) {
		return 1.0;
	}

	// Skip the first hit, then clip
	g_state.mode = 0;
	return -1.0;
}

function moverFilter(shapeId: b3ShapeId): bool {
	vtag("moverFilter");
	pshapeId(shapeId);
	vend();
	return shapeId.index1 != 1;
}

function planeFcn(shapeId: b3ShapeId, planes: StaticArray<b3PlaneResult>, count: i32): bool {
	vtag("planes");
	pshapeId(shapeId);
	vi32(count);
	for (let i = 0; i < count; ++i) {
		pv3(planes[i].plane.normal);
		vf32(planes[i].plane.offset);
		pv3(planes[i].point);
		vi32(planes[i].triangleIndex);
		vi32(planes[i].childIndex);
		vi32(planes[i].materialIndex);

		if (g_planeCount < PLANE_CAPACITY) {
			ref p = g_planes[g_planeCount];
			p.plane = planes[i].plane;
			p.pushLimit = FLT_MAX;
			p.push = 0.0;
			p.clipVelocity = true;
			g_planeCount += 1;
		}
	}
	vend();
	return true;
}

function queries(): void {
	let filter = b3DefaultQueryFilter();

	{
		g_state.count = 0;
		let box = b3AABB(b3Vec3(-2.0, -0.5, -2.0), b3Vec3(2.0, 3.0, 2.0));
		let stats = b3World_OverlapAABB(g_worldId, box, filter, overlapFcn);
		vtag("overlapAABB");
		vi32(g_state.count);
		pstats(stats);
		vend();
	}

	{
		g_state.count = 0;
		let center = new StaticArray<b3Vec3>(1);
		let proxy = new b3ShapeProxy(center, 1, 1.0);
		let stats = b3World_OverlapShape(g_worldId, b3Vec3(2.0, 0.5, 0.0), proxy, filter, overlapFcn);
		vtag("overlapShape");
		vi32(g_state.count);
		pstats(stats);
		vend();
	}

	for (let mode = 0; mode < 3; ++mode) {
		g_state.mode = mode;
		let stats = b3World_CastRay(g_worldId, b3Vec3(-9.0, 0.5, 0.1), b3Vec3(18.0, 0.0, 0.0), filter, castFcn);
		vtag("castRay");
		vi32(mode);
		pstats(stats);
		vend();
	}

	{
		let result = b3World_CastRayClosest(g_worldId, b3Vec3(0.05, 10.0, 0.02), b3Vec3(0.0, -20.0, 0.0), filter);
		vtag("rayClosest");
		pshapeId(result.shapeId);
		pv3(result.point);
		pv3(result.normal);
		vu64(result.userMaterialId);
		vf32(result.fraction);
		vi32(result.triangleIndex);
		vi32(result.childIndex);
		vi32(result.nodeVisits);
		vi32(result.leafVisits);
		vbool(result.hit);
		vend();
	}

	{
		let points = new StaticArray<b3Vec3>(8);
		for (let i = 0; i < 8; ++i) {
			points[i].x = (i & 1) != 0 ? <f32>0.3 : <f32>-0.3;
			points[i].y = (i & 2) != 0 ? <f32>0.2 : <f32>-0.2;
			points[i].z = (i & 4) != 0 ? <f32>0.3 : <f32>-0.3;
		}
		let proxy = new b3ShapeProxy(points, 8, 0.05);
		g_state.mode = 0;
		let stats = b3World_CastShape(g_worldId, b3Vec3(-9.0, 2.0, 0.0), proxy, b3Vec3(18.0, -1.5, 0.0), filter, castFcn);
		vtag("castShape");
		pstats(stats);
		vend();
	}

	{
		let mover = b3Capsule(b3Vec3(0.0, -0.3, 0.0), b3Vec3(0.0, 0.3, 0.0), 0.3);
		let fraction = b3World_CastMover(g_worldId, b3Vec3(-6.0, 1.0, 0.0), mover, b3Vec3(12.0, -0.5, 0.0), filter, moverFilter);
		let unfiltered = b3World_CastMover(g_worldId, b3Vec3(0.0, 6.0, 0.0), mover, b3Vec3(0.0, -8.0, 0.0), filter, null);
		vtag("castMover");
		vf32(fraction);
		vf32(unfiltered);
		vend();
	}

	{
		g_planeCount = 0;
		let mover = b3Capsule(b3Vec3(0.0, -0.4, 0.0), b3Vec3(0.0, 0.4, 0.0), 0.35);
		b3World_CollideMover(g_worldId, b3Vec3(0.6, 0.7, 0.2), mover, filter, planeFcn);
		let result = b3SolvePlanes(b3Vec3(-0.3, -0.2, 0.1), g_planes, g_planeCount);
		let clipped = b3ClipVector(b3Vec3(-2.0, -1.0, 0.5), g_planes, g_planeCount);
		vtag("collideMover");
		vi32(g_planeCount);
		pv3(result.delta);
		vi32(result.iterationCount);
		pv3(clipped);
		vend();
	}

	vtag("bounds");
	paabb(b3World_GetBounds(g_worldId));
	vend();
}

export function step(stepIndex: i32): void {
	if (stepIndex == 40) {
		let def = b3DefaultExplosionDef();
		def.position = b3Vec3(0.0, 0.5, 0.5);
		def.radius = 2.0;
		def.falloff = 1.0;
		def.impulsePerArea = 5.0;
		b3World_Explode(g_worldId, def);
	}

	if (stepIndex == 60) {
		b3World_SetFrictionCallback(g_worldId, null);
		b3World_SetGravity(g_worldId, b3Vec3(0.5, -9.0, 0.0));
		b3DestroyBody(g_bodies[6]);
	}

	if (stepIndex == 80) {
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(1.0, 4.0, 0.5);
		let bodyId = addBody(bodyDef);
		let shapeDef = b3DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.3);
		b3CreateSphereShape(bodyId, shapeDef, sphere);
	}

	if (stepIndex == 100) {
		b3World_SetContactTuning(g_worldId, 20.0, 5.0, 2.0);
		b3World_EnableWarmStarting(g_worldId, false);
	}

	b3World_Step(g_worldId, <f32>1.0 / <f32>60.0, 4);

	vtag("step");
	vi32(stepIndex);
	vend();

	printBodies();
	printCounters();
	printEvents();

	if (stepIndex % 20 == 19) {
		queries();
	}
}

export function finish(): void {
	// A zero time step collides without solving
	b3World_Step(g_worldId, 0.0, 4);
	printCounters();
	printEvents();

	b3DestroyWorld(g_worldId);
	vtag("final");
	vbool(b3World_IsValid(g_worldId));
	vi32(b3GetWorldCount());
	vend();
}
