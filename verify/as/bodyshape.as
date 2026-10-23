// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/body.as and lib/shape.as, mirrors verify/cref/bodyshape.c

import { vtag, vi32, vu32, vbool, vf32, vu64, vend, vseed, vrandf } from "./common";
import {
	b3AABB,
	b3Add,
	b3Matrix3,
	b3MakeQuatFromAxisAngle,
	b3MulAdd,
	b3MulSV,
	b3Normalize,
	b3Quat,
	b3Quat_identity,
	b3Sub,
	b3Transform,
	b3Vec3,
	b3Vec3_zero,
	b3WorldTransform,
} from "../../lib/math_functions";
import {
	b3Capsule,
	b3CastOutput,
	b3CompoundCapsuleDef,
	b3CompoundDef,
	b3CompoundHullDef,
	b3CompoundMeshDef,
	b3CompoundSphereDef,
	b3HeightFieldData,
	b3HullData,
	b3MassData,
	b3MeshData,
	b3ShapeProxy,
	b3Sphere,
} from "../../lib/collision";
import { b3CreateCompound } from "../../lib/compound";
import { b3CreateRevoluteJoint, b3CreateSphericalJoint } from "../../lib/joint";
import {
	b3BodyId,
	b3BodyPlaneResult,
	b3ContactData,
	b3MotionLocks,
	b3ShapeId,
	b3SurfaceMaterial,
	b3WorldId,
	b3DefaultBodyDef,
	b3DefaultFilter,
	b3DefaultRevoluteJointDef,
	b3DefaultSphericalJointDef,
	b3JointId,
	b3DefaultQueryFilter,
	b3DefaultShapeDef,
	b3DefaultSurfaceMaterial,
	b3DefaultWorldDef,
	b3BodyDef,
	b3_capsuleShape,
	b3_dynamicBody,
	b3_hullShape,
	b3_kinematicBody,
	b3_meshShape,
	b3_sphereShape,
	b3_staticBody,
} from "../../lib/types";
import {
	b3Body_IsValid,
	b3CreateWorld,
	b3GetWorldFromId,
	b3World_GetContactEvents,
	b3World_GetSensorEvents,
	b3World_Step,
	b3_awakeSet,
} from "../../lib/physics_world";
import { b3GetIdCount } from "../../lib/id_pool";
import { b3CreateRock, b3MakeBoxHull, b3MakeCubeHull, b3MakeOffsetBoxHull } from "../../lib/hull";
import { b3CreateGridMesh } from "../../lib/mesh";
import { b3CreateGrid } from "../../lib/height_field";
import {
	b3Body_AllowFastRotation,
	b3Body_ApplyAngularImpulse,
	b3Body_ApplyForce,
	b3Body_ApplyForceToCenter,
	b3Body_ApplyLinearImpulse,
	b3Body_ApplyLinearImpulseToCenter,
	b3Body_ApplyMassFromShapes,
	b3Body_ApplyTorque,
	b3Body_CastRay,
	b3Body_CastShape,
	b3Body_CollideMover,
	b3Body_ComputeAABB,
	b3Body_Disable,
	b3Body_Enable,
	b3Body_EnableContactRecycling,
	b3Body_EnableHitEvents,
	b3Body_EnableSleep,
	b3Body_GetAngularDamping,
	b3Body_GetAngularVelocity,
	b3Body_GetClosestPoint,
	b3Body_GetContactCapacity,
	b3Body_GetContactData,
	b3Body_GetGravityScale,
	b3Body_GetInverseMass,
	b3Body_GetJointCount,
	b3Body_GetLinearDamping,
	b3Body_GetLinearVelocity,
	b3Body_GetLocalCenter,
	b3Body_GetLocalPoint,
	b3Body_GetLocalPointVelocity,
	b3Body_GetLocalRotationalInertia,
	b3Body_GetLocalVector,
	b3Body_GetMass,
	b3Body_GetMassData,
	b3Body_GetMaxExtent,
	b3Body_GetMaxExtentOrigin,
	b3Body_GetMinExtent,
	b3Body_GetMotionLocks,
	b3Body_GetName,
	b3Body_GetPosition,
	b3Body_GetRotation,
	b3Body_GetSafetyFactor,
	b3Body_GetShapeCount,
	b3Body_GetShapes,
	b3Body_GetSleepThreshold,
	b3Body_GetTransform,
	b3Body_GetType,
	b3Body_GetUserData,
	b3Body_GetWorld,
	b3Body_GetWorldCenter,
	b3Body_GetWorldInverseRotationalInertia,
	b3Body_GetWorldPoint,
	b3Body_GetWorldPointVelocity,
	b3Body_GetWorldVector,
	b3Body_IsAwake,
	b3Body_IsBullet,
	b3Body_IsContactRecyclingEnabled,
	b3Body_IsEnabled,
	b3Body_IsFastRotationAllowed,
	b3Body_IsSleepEnabled,
	b3Body_OverlapShape,
	b3Body_SetAngularDamping,
	b3Body_SetAngularVelocity,
	b3Body_SetAwake,
	b3Body_SetBullet,
	b3Body_SetGravityScale,
	b3Body_SetLinearDamping,
	b3Body_SetLinearVelocity,
	b3Body_SetMassData,
	b3Body_SetMotionLocks,
	b3Body_SetName,
	b3Body_SetSafetyFactor,
	b3Body_SetSleepThreshold,
	b3Body_SetTargetTransform,
	b3Body_SetTransform,
	b3Body_SetType,
	b3Body_SetUserData,
	b3Body_TimeOfImpactMover,
	b3CreateBody,
	b3DestroyBody,
	b3Body_GetJoints,
} from "../../lib/body";
import {
	b3CreateCapsuleShape,
	b3CreateBakedCompoundShape,
	b3CreateHeightFieldShape,
	b3CreateHullShape,
	b3CreateMeshShape,
	b3CreateSphereShape,
	b3CreateTransformedHullShape,
	b3DestroyShape,
	b3Shape_ApplyWind,
	b3Shape_AreContactEventsEnabled,
	b3Shape_AreHitEventsEnabled,
	b3Shape_ArePreSolveEventsEnabled,
	b3Shape_AreSensorEventsEnabled,
	b3Shape_ComputeMassData,
	b3Shape_EnableContactEvents,
	b3Shape_EnableHitEvents,
	b3Shape_EnablePreSolveEvents,
	b3Shape_EnableSensorEvents,
	b3Shape_GetAABB,
	b3Shape_GetBody,
	b3Shape_GetCapsule,
	b3Shape_GetClosestPoint,
	b3Shape_GetContactCapacity,
	b3Shape_GetContactData,
	b3Shape_GetDensity,
	b3Shape_GetFilter,
	b3Shape_GetFriction,
	b3Shape_GetHull,
	b3Shape_GetMesh,
	b3Shape_GetMeshMaterialCount,
	b3Shape_GetMeshSurfaceMaterial,
	b3Shape_GetName,
	b3Shape_GetRestitution,
	b3Shape_GetSensorCapacity,
	b3Shape_GetSensorData,
	b3Shape_GetSphere,
	b3Shape_GetSurfaceMaterial,
	b3Shape_GetType,
	b3Shape_GetUserData,
	b3Shape_IsSensor,
	b3Shape_RayCast,
	b3Shape_SetCapsule,
	b3Shape_SetDensity,
	b3Shape_SetFilter,
	b3Shape_SetFriction,
	b3Shape_SetHull,
	b3Shape_SetMesh,
	b3Shape_SetMeshMaterial,
	b3Shape_SetName,
	b3Shape_SetRestitution,
	b3Shape_SetSphere,
	b3Shape_SetSurfaceMaterial,
	b3Shape_SetUserData,
} from "../../lib/shape";

export { frameBoundary } from "./common";

const BODY_CAPACITY = 32;
const SHAPE_CAPACITY = 16;
const CONTACT_CAPACITY = 32;

let g_worldId: b3WorldId = b3WorldId();
let g_bodies = new StaticArray<b3BodyId>(BODY_CAPACITY);
let g_bodyCount = 0;
let g_rock: b3HullData = new b3HullData();
let g_mesh: b3MeshData = new b3MeshData();
let g_heightField: b3HeightFieldData = new b3HeightFieldData();

function rvec(lo: f32, hi: f32): b3Vec3 {
	let v: b3Vec3;
	v.x = vrandf(lo, hi);
	v.y = vrandf(lo, hi);
	v.z = vrandf(lo, hi);
	return v;
}

function pvec(v: b3Vec3): void {
	vf32(v.x);
	vf32(v.y);
	vf32(v.z);
}

function pquat(q: b3Quat): void {
	pvec(q.v);
	vf32(q.s);
}

function pmat(m: b3Matrix3): void {
	pvec(m.cx);
	pvec(m.cy);
	pvec(m.cz);
}

function paabb(a: b3AABB): void {
	pvec(a.lowerBound);
	pvec(a.upperBound);
}

function pshapeId(id: b3ShapeId): void {
	vi32(id.index1);
	vu32(id.world0);
	vu32(id.generation);
}

function pmaterial(m: b3SurfaceMaterial): void {
	vf32(m.friction);
	vf32(m.restitution);
	vf32(m.rollingResistance);
	pvec(m.tangentVelocity);
	vu64(m.userMaterialId);
	vu32(m.customColor);
}

function pcast(o: b3CastOutput): void {
	pvec(o.normal);
	pvec(o.point);
	vf32(o.fraction);
	vi32(o.iterations);
	vi32(o.triangleIndex);
	vi32(o.childIndex);
	vi32(o.materialIndex);
	vbool(o.hit);
}

function v3(x: f32, y: f32, z: f32): b3Vec3 {
	return b3Vec3(x, y, z);
}

function dumpShape(shapeId: b3ShapeId): void {
	let world = b3GetWorldFromId(g_worldId);
	let shape = world.shapes[shapeId.index1 - 1];

	vtag("shape");
	pshapeId(shapeId);
	vi32(b3Shape_GetType(shapeId));
	paabb(b3Shape_GetAABB(shapeId));
	vf32(b3Shape_GetDensity(shapeId));
	vf32(b3Shape_GetFriction(shapeId));
	vf32(b3Shape_GetRestitution(shapeId));
	vi32(b3Shape_GetMeshMaterialCount(shapeId));
	let filter = b3Shape_GetFilter(shapeId);
	vu64(filter.categoryBits);
	vu64(filter.maskBits);
	vi32(filter.groupIndex);
	vbool(b3Shape_IsSensor(shapeId));
	vbool(b3Shape_AreSensorEventsEnabled(shapeId));
	vbool(b3Shape_AreContactEventsEnabled(shapeId));
	vbool(b3Shape_ArePreSolveEventsEnabled(shapeId));
	vbool(b3Shape_AreHitEventsEnabled(shapeId));
	vu64(b3Shape_GetUserData(shapeId));
	vtag(b3Shape_GetName(shapeId));
	let bodyId = b3Shape_GetBody(shapeId);
	vi32(bodyId.index1);
	vu32(bodyId.generation);
	vend();

	vtag("shapemat");
	let materialCount = b3Shape_GetMeshMaterialCount(shapeId);
	pmaterial(b3Shape_GetSurfaceMaterial(shapeId));
	for (let i = 1; i < materialCount; ++i) {
		pmaterial(b3Shape_GetMeshSurfaceMaterial(shapeId, i));
	}
	vend();

	vtag("shapemass");
	let massData = b3Shape_ComputeMassData(shapeId);
	vf32(massData.mass);
	pvec(massData.center);
	pmat(massData.inertia);
	vend();

	vtag("shapeint");
	vi32(shape.id);
	vi32(shape.bodyId);
	vi32(shape.prevShapeId);
	vi32(shape.nextShapeId);
	vi32(shape.sensorIndex);
	vi32(shape.proxyKey);
	vf32(shape.explosionScale);
	vf32(shape.aabbMargin);
	pvec(shape.localCentroid);
	vu32(shape.flags);
	paabb(world.fatAABBs[shape.id]);
	vend();

	let type = shape.type;
	if (type == b3_sphereShape) {
		let s = b3Shape_GetSphere(shapeId);
		vtag("sphere");
		pvec(s.center);
		vf32(s.radius);
		vend();
	} else if (type == b3_capsuleShape) {
		let c = b3Shape_GetCapsule(shapeId);
		vtag("capsule");
		pvec(c.center1);
		pvec(c.center2);
		vf32(c.radius);
		vend();
	} else if (type == b3_hullShape) {
		let hull = b3Shape_GetHull(shapeId)!;
		vtag("hull");
		vi32(hull.vertexCount);
		vi32(hull.faceCount);
		pvec(hull.center);
		vf32(hull.innerRadius);
		vf32(hull.surfaceArea);
		vend();
	} else if (type == b3_meshShape) {
		let mesh = b3Shape_GetMesh(shapeId);
		vtag("mesh");
		vi32(mesh.data.triangleCount);
		pvec(mesh.scale);
		vend();
	}
}

function dumpBody(bodyId: b3BodyId): void {
	let world = b3GetWorldFromId(g_worldId);
	let body = world.bodies[bodyId.index1 - 1];

	vtag("body");
	vi32(bodyId.index1);
	vu32(bodyId.generation);
	vi32(b3Body_GetType(bodyId));
	pvec(b3Body_GetPosition(bodyId));
	pquat(b3Body_GetRotation(bodyId));
	pvec(b3Body_GetLinearVelocity(bodyId));
	pvec(b3Body_GetAngularVelocity(bodyId));
	vf32(b3Body_GetMass(bodyId));
	vf32(b3Body_GetInverseMass(bodyId));
	pvec(b3Body_GetLocalCenter(bodyId));
	pvec(b3Body_GetWorldCenter(bodyId));
	vend();

	vtag("bodyinertia");
	pmat(b3Body_GetLocalRotationalInertia(bodyId));
	pmat(b3Body_GetWorldInverseRotationalInertia(bodyId));
	let massData = b3Body_GetMassData(bodyId);
	vf32(massData.mass);
	pvec(massData.center);
	pmat(massData.inertia);
	vend();

	vtag("bodyprops");
	vf32(b3Body_GetMinExtent(bodyId));
	pvec(b3Body_GetMaxExtent(bodyId));
	pvec(b3Body_GetMaxExtentOrigin(bodyId));
	vbool(b3Body_IsAwake(bodyId));
	vbool(b3Body_IsEnabled(bodyId));
	vbool(b3Body_IsSleepEnabled(bodyId));
	vbool(b3Body_IsBullet(bodyId));
	vbool(b3Body_IsFastRotationAllowed(bodyId));
	vbool(b3Body_IsContactRecyclingEnabled(bodyId));
	let locks = b3Body_GetMotionLocks(bodyId);
	vbool(locks.linearX);
	vbool(locks.linearY);
	vbool(locks.linearZ);
	vbool(locks.angularX);
	vbool(locks.angularY);
	vbool(locks.angularZ);
	vi32(b3Body_GetShapeCount(bodyId));
	vi32(b3Body_GetJointCount(bodyId));
	vf32(b3Body_GetSleepThreshold(bodyId));
	vf32(b3Body_GetSafetyFactor(bodyId));
	vf32(b3Body_GetGravityScale(bodyId));
	vf32(b3Body_GetLinearDamping(bodyId));
	vf32(b3Body_GetAngularDamping(bodyId));
	vu64(b3Body_GetUserData(bodyId));
	vtag(b3Body_GetName(bodyId));
	paabb(b3Body_ComputeAABB(bodyId));
	let worldId = b3Body_GetWorld(bodyId);
	vu32(worldId.index1);
	vu32(worldId.generation);
	vend();

	vtag("bodyint");
	vi32(body.setIndex);
	vi32(body.localIndex);
	vi32(body.headContactKey);
	vi32(body.contactCount);
	vi32(body.headShapeId);
	vi32(body.headJointKey);
	vi32(body.islandId);
	vi32(body.islandIndex);
	vf32(body.sleepTime);
	vi32(body.bodyMoveIndex);
	vu32(body.flags);
	vu32(body.nameId);
	let set = world.solverSets[body.setIndex];
	let sim = set.bodySims[body.localIndex];
	vu32(sim.flags);
	pvec(sim.center0);
	pquat(sim.rotation0);
	pvec(sim.force);
	pvec(sim.torque);
	pmat(sim.invInertiaLocal);
	vi32(sim.bodyId);
	if (body.setIndex == b3_awakeSet) {
		vu32(set.bodyStates[body.localIndex].flags);
	}
	vend();

	let shapes = new StaticArray<b3ShapeId>(SHAPE_CAPACITY);
	let shapeCount = b3Body_GetShapes(bodyId, shapes, SHAPE_CAPACITY);
	for (let i = 0; i < shapeCount; ++i) {
		dumpShape(shapes[i]);
	}
}

function dumpWorld(label: string): void {
	let world = b3GetWorldFromId(g_worldId);
	vtag("world");
	vtag(label);
	vi32(world.bodies.length);
	vi32(world.shapes.length);
	vi32(world.solverSets.length);
	vi32(world.islands.length);
	vi32(world.sensors.length);
	vi32(world.compoundShapeCount);
	vi32(b3GetIdCount(world.bodyIdPool));
	vi32(b3GetIdCount(world.shapeIdPool));
	vi32(b3GetIdCount(world.solverSetIdPool));
	vi32(b3GetIdCount(world.islandIdPool));
	vend();

	for (let i = 0; i < g_bodyCount; ++i) {
		if (b3Body_IsValid(g_bodies[i])) {
			dumpBody(g_bodies[i]);
		} else {
			vtag("dead");
			vi32(i);
			vend();
		}
	}
}

function addBody(def: b3BodyDef): b3BodyId {
	let id = b3CreateBody(g_worldId, def);
	g_bodies[g_bodyCount] = id;
	g_bodyCount += 1;
	return id;
}

export function phaseCreate(): void {
	vseed(0x626f647973686170);

	let worldDef = b3DefaultWorldDef();
	g_worldId = b3CreateWorld(worldDef);

	g_rock = b3CreateRock(0.6)!;
	g_mesh = b3CreateGridMesh(4, 4, 1.0, 2, true)!;
	g_heightField = b3CreateGrid(5, 5, v3(1.0, 0.5, 1.0), false);

	// 0: static ground box, named, with a sensor
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = v3(0.0, -1.0, 0.0);
		bodyDef.name = "ground";
		bodyDef.userData = 7;
		let bodyId = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.7;
		shapeDef.name = "groundbox";
		let box = b3MakeBoxHull(20.0, 1.0, 20.0);
		b3CreateHullShape(bodyId, shapeDef, box.base);

		let sensorDef = b3DefaultShapeDef();
		sensorDef.isSensor = true;
		sensorDef.enableSensorEvents = true;
		let sensorSphere = b3Sphere(v3(0.0, 3.0, 0.0), 2.0);
		b3CreateSphereShape(bodyId, sensorDef, sensorSphere);
	}

	// 1: static mesh with two materials
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = v3(30.0, 0.0, 0.0);
		let bodyId = addBody(bodyDef);

		let materials = new StaticArray<b3SurfaceMaterial>(2);
		materials[0] = b3DefaultSurfaceMaterial();
		materials[1] = b3DefaultSurfaceMaterial();
		materials[0].friction = 0.3;
		materials[0].userMaterialId = 11;
		materials[1].friction = 0.9;
		materials[1].userMaterialId = 12;
		let shapeDef = b3DefaultShapeDef();
		shapeDef.materials = materials;
		shapeDef.materialCount = 2;
		b3CreateMeshShape(bodyId, shapeDef, g_mesh, v3(1.0, 1.0, -1.0));
	}

	// 2: static height field
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = v3(-30.0, 0.0, 0.0);
		let bodyId = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		b3CreateHeightFieldShape(bodyId, shapeDef, g_heightField);
	}

	// 3: dynamic sphere
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = v3(0.0, 2.0, 0.0);
		bodyDef.linearVelocity = v3(0.5, 0.0, -0.25);
		bodyDef.name = "ball";
		let bodyId = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		shapeDef.density = 2.0;
		shapeDef.enableHitEvents = true;
		let sphere = b3Sphere(v3(0.0, 0.0, 0.0), 0.5);
		b3CreateSphereShape(bodyId, shapeDef, sphere);
	}

	// 4: dynamic capsule, rotated
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = v3(2.0, 2.0, 0.0);
		bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Normalize(v3(1.0, 2.0, 3.0)), 0.7);
		bodyDef.angularVelocity = v3(0.0, 1.5, 0.0);
		let bodyId = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		let capsule = b3Capsule(v3(-0.5, 0.0, 0.0), v3(0.5, 0.1, 0.0), 0.25);
		b3CreateCapsuleShape(bodyId, shapeDef, capsule);
	}

	// 5: dynamic box hull
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = v3(-2.0, 2.0, 0.0);
		bodyDef.rotation = b3MakeQuatFromAxisAngle(v3(0.0, 0.0, 1.0), 0.3);
		bodyDef.isBullet = true;
		let bodyId = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.restitution = 0.4;
		let box = b3MakeBoxHull(0.5, 0.4, 0.3);
		b3CreateHullShape(bodyId, shapeDef, box.base);
	}

	// 6: transformed hull and a shared rock hull
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = v3(4.0, 2.0, 0.0);
		let bodyId = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		let xf = b3Transform(v3(0.2, 0.1, 0.0), b3MakeQuatFromAxisAngle(v3(0.0, 1.0, 0.0), 0.5));
		b3CreateTransformedHullShape(bodyId, shapeDef, g_rock, xf, v3(1.0, 2.0, 0.5));
		b3CreateHullShape(bodyId, shapeDef, g_rock);
	}

	// 7: compound body with a zero density shape
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = v3(0.0, 4.0, 2.0);
		bodyDef.gravityScale = 0.5;
		bodyDef.linearDamping = 0.1;
		bodyDef.angularDamping = 0.2;
		let bodyId = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		let sphere = b3Sphere(v3(0.5, 0.0, 0.0), 0.3);
		b3CreateSphereShape(bodyId, shapeDef, sphere);
		shapeDef.density = 3.0;
		let capsule = b3Capsule(v3(-0.5, 0.0, 0.0), v3(-0.5, 1.0, 0.0), 0.2);
		b3CreateCapsuleShape(bodyId, shapeDef, capsule);
		shapeDef.density = 0.0;
		let box = b3MakeOffsetBoxHull(0.2, 0.2, 0.2, v3(0.0, -0.5, 0.3));
		b3CreateHullShape(bodyId, shapeDef, box.base);
	}

	// 8: kinematic hull
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_kinematicBody;
		bodyDef.position = v3(6.0, 1.0, 0.0);
		bodyDef.angularVelocity = v3(0.0, 0.0, 1.0);
		let bodyId = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		let box = b3MakeBoxHull(1.0, 0.2, 0.5);
		b3CreateHullShape(bodyId, shapeDef, box.base);
	}

	// 9: sleeping sphere in its own set
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = v3(8.0, 0.5, 0.0);
		bodyDef.isAwake = false;
		let bodyId = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		let sphere = b3Sphere(v3(0.0, 0.0, 0.0), 0.5);
		b3CreateSphereShape(bodyId, shapeDef, sphere);
	}

	// 10: disabled body
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = v3(10.0, 2.0, 0.0);
		bodyDef.isEnabled = false;
		let bodyId = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		let box = b3MakeCubeHull(0.4);
		b3CreateHullShape(bodyId, shapeDef, box.base);
	}

	// 11: motion locks, degenerate capsule, deferred mass
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = v3(-4.0, 3.0, 1.0);
		bodyDef.motionLocks.angularX = true;
		bodyDef.motionLocks.angularZ = true;
		bodyDef.motionLocks.linearY = true;
		bodyDef.allowFastRotation = true;
		bodyDef.enableContactRecycling = false;
		let bodyId = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		shapeDef.updateBodyMass = false;
		let capsule = b3Capsule(v3(0.0, 0.0, 0.0), v3(0.0, 0.0001, 0.0), 0.3);
		b3CreateCapsuleShape(bodyId, shapeDef, capsule);
		let box = b3MakeBoxHull(0.3, 0.6, 0.2);
		b3CreateHullShape(bodyId, shapeDef, box.base);
	}

	// 12: fixed rotation body
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = v3(-6.0, 1.0, -1.0);
		bodyDef.motionLocks.angularX = true;
		bodyDef.motionLocks.angularY = true;
		bodyDef.motionLocks.angularZ = true;
		bodyDef.enableSleep = false;
		let bodyId = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		shapeDef.filter.groupIndex = -3;
		shapeDef.filter.categoryBits = 0x4;
		let box = b3MakeBoxHull(0.4, 0.4, 0.4);
		b3CreateHullShape(bodyId, shapeDef, box.base);
	}

	dumpWorld("create");

	b3Body_ApplyMassFromShapes(g_bodies[11]);
	dumpBody(g_bodies[11]);
}

export function phaseQueries(): void {
	vseed(0x7175657279);
	let filter = b3DefaultQueryFilter();

	for (let i = 0; i < g_bodyCount; ++i) {
		if (b3Body_IsValid(g_bodies[i]) == false) {
			continue;
		}

		let bodyId = g_bodies[i];
		let xf = b3Body_GetTransform(bodyId);
		let center = b3Body_GetPosition(bodyId);

		for (let k = 0; k < 4; ++k) {
			let offset = rvec(-3.0, 3.0);
			let target = b3Add(center, offset);

			let closestHolder = new StaticArray<b3Vec3>(1);
			let distance = b3Body_GetClosestPoint(bodyId, closestHolder[0], target);
			vtag("closest");
			vi32(i);
			vf32(distance);
			pvec(closestHolder[0]);
			vend();

			let translation = b3MulSV(-2.0, offset);
			let ray = b3Body_CastRay(bodyId, target, translation, filter, 1.0, xf);
			vtag("ray");
			pshapeId(ray.shapeId);
			pvec(ray.point);
			pvec(ray.normal);
			vf32(ray.fraction);
			vi32(ray.triangleIndex);
			vu64(ray.userMaterialId);
			vi32(ray.iterations);
			vbool(ray.hit);
			vend();

			let points = new StaticArray<b3Vec3>(2);
			points[0] = target;
			points[1] = b3Add(target, v3(0.0, 0.3, 0.0));
			let proxy = new b3ShapeProxy(points, 2, 0.2);
			let cast = b3Body_CastShape(bodyId, target, proxy, translation, filter, 1.0, false, xf);
			vtag("cast");
			pshapeId(cast.shapeId);
			pvec(cast.point);
			pvec(cast.normal);
			vf32(cast.fraction);
			vi32(cast.triangleIndex);
			vu64(cast.userMaterialId);
			vi32(cast.iterations);
			vbool(cast.hit);
			vend();

			let mid = b3MulAdd(target, 0.75, translation);
			let overlapPoints = new StaticArray<b3Vec3>(1);
			overlapPoints[0] = mid;
			let overlapProxy = new b3ShapeProxy(overlapPoints, 1, 0.5);
			vtag("overlap");
			vbool(b3Body_OverlapShape(bodyId, b3Vec3_zero, overlapProxy, filter, xf));
			vend();

			let mover = b3Capsule(mid, b3Add(mid, v3(0.0, 0.5, 0.0)), 0.4);
			let planes = new StaticArray<b3BodyPlaneResult>(4);
			let planeCount = b3Body_CollideMover(bodyId, planes, 4, b3Vec3_zero, mover, filter, xf);
			vtag("mover");
			vi32(planeCount);
			for (let p = 0; p < planeCount; ++p) {
				pshapeId(planes[p].shapeId);
				pvec(planes[p].result.plane.normal);
				vf32(planes[p].result.plane.offset);
				pvec(planes[p].result.point);
				vi32(planes[p].result.triangleIndex);
				vi32(planes[p].result.childIndex);
				vi32(planes[p].result.materialIndex);
			}
			vend();

			let moverAtTarget = b3Capsule(target, b3Add(target, v3(0.0, 0.5, 0.0)), 0.2);
			let xf2 = xf;
			xf2.p = b3Add(xf.p, v3(0.1, 0.0, 0.0));
			let toi = b3Body_TimeOfImpactMover(bodyId, b3Vec3_zero, moverAtTarget, translation, filter, xf, xf2);
			vtag("toi");
			pvec(toi.point);
			pvec(toi.normal);
			vf32(toi.fraction);
			pshapeId(toi.shapeId);
			vend();
		}

		let shapes = new StaticArray<b3ShapeId>(SHAPE_CAPACITY);
		let shapeCount = b3Body_GetShapes(bodyId, shapes, SHAPE_CAPACITY);
		for (let s = 0; s < shapeCount; ++s) {
			let origin = b3Add(center, rvec(-3.0, 3.0));
			let translation = b3MulSV(2.0, b3Sub(center, origin));
			let output = b3Shape_RayCast(shapes[s], origin, translation);
			vtag("shaperay");
			pcast(output);
			vend();

			let type = b3Shape_GetType(shapes[s]);
			if (type == b3_sphereShape || type == b3_capsuleShape || type == b3_hullShape) {
				vtag("shapeclosest");
				pvec(b3Shape_GetClosestPoint(shapes[s], origin));
				vend();
			}
		}
	}
}

function dumpContacts(): void {
	let contactData = new StaticArray<b3ContactData>(CONTACT_CAPACITY);
	for (let i = 0; i < g_bodyCount; ++i) {
		if (b3Body_IsValid(g_bodies[i]) == false) {
			continue;
		}

		vtag("contacts");
		vi32(i);
		vi32(b3Body_GetContactCapacity(g_bodies[i]));
		let count = b3Body_GetContactData(g_bodies[i], contactData, CONTACT_CAPACITY);
		vi32(count);
		for (let c = 0; c < count; ++c) {
			vi32(contactData[c].contactId.index1);
			vu32(contactData[c].contactId.generation);
			pshapeId(contactData[c].shapeIdA);
			pshapeId(contactData[c].shapeIdB);
			vi32(contactData[c].manifoldCount);
		}
		vend();

		let shapes = new StaticArray<b3ShapeId>(SHAPE_CAPACITY);
		let shapeCount = b3Body_GetShapes(g_bodies[i], shapes, SHAPE_CAPACITY);
		for (let s = 0; s < shapeCount; ++s) {
			vtag("shapecontacts");
			vi32(b3Shape_GetContactCapacity(shapes[s]));
			let n = b3Shape_GetContactData(shapes[s], contactData, CONTACT_CAPACITY);
			vi32(n);
			for (let c = 0; c < n; ++c) {
				vi32(contactData[c].contactId.index1);
				vi32(contactData[c].manifoldCount);
			}
			vi32(b3Shape_GetSensorCapacity(shapes[s]));
			let visitors = new StaticArray<b3ShapeId>(SHAPE_CAPACITY);
			let v = b3Shape_GetSensorData(shapes[s], visitors, SHAPE_CAPACITY);
			vi32(v);
			for (let k = 0; k < v; ++k) {
				pshapeId(visitors[k]);
			}
			vend();
		}
	}
}

export function phaseMutate(): void {
	vseed(0x6d757461);

	let ball = g_bodies[3];
	let capsuleBody = g_bodies[4];
	let boxBody = g_bodies[5];
	let rockBody = g_bodies[6];
	let compound = g_bodies[7];
	let kinematic = g_bodies[8];
	let sleeper = g_bodies[9];
	let disabled = g_bodies[10];
	let locked = g_bodies[11];

	b3Body_SetTransform(ball, v3(0.5, 2.5, -0.5), b3MakeQuatFromAxisAngle(v3(1.0, 0.0, 0.0), 0.2));
	b3Body_SetLinearVelocity(ball, v3(1.0, 2.0, 3.0));
	b3Body_SetAngularVelocity(ball, v3(-1.0, 0.5, 0.25));
	b3Body_ApplyForce(ball, v3(10.0, 0.0, 0.0), v3(0.5, 2.6, -0.5), true);
	b3Body_ApplyForceToCenter(ball, v3(0.0, 5.0, 0.0), true);
	b3Body_ApplyTorque(ball, v3(0.0, 0.0, 2.0), true);
	b3Body_ApplyLinearImpulse(ball, v3(0.1, 0.2, 0.3), v3(0.6, 2.5, -0.4), true);
	b3Body_ApplyLinearImpulseToCenter(ball, v3(0.0, -0.2, 0.1), true);
	b3Body_ApplyAngularImpulse(ball, v3(0.05, 0.0, -0.05), true);
	vtag("pointvel");
	pvec(b3Body_GetLocalPointVelocity(ball, v3(0.1, 0.2, 0.3)));
	pvec(b3Body_GetWorldPointVelocity(ball, v3(0.6, 2.4, -0.5)));
	pvec(b3Body_GetLocalPoint(ball, v3(1.0, 1.0, 1.0)));
	pvec(b3Body_GetWorldPoint(ball, v3(1.0, 1.0, 1.0)));
	pvec(b3Body_GetLocalVector(ball, v3(1.0, 1.0, 1.0)));
	pvec(b3Body_GetWorldVector(ball, v3(1.0, 1.0, 1.0)));
	vend();

	// Forces on a sleeping body: no wake, then wake
	b3Body_ApplyForceToCenter(sleeper, v3(1.0, 0.0, 0.0), false);
	b3Body_ApplyLinearImpulse(sleeper, v3(1.0, 0.0, 0.0), v3(8.0, 0.6, 0.0), false);
	dumpBody(sleeper);
	b3Body_ApplyTorque(sleeper, v3(0.0, 1.0, 0.0), true);
	dumpWorld("forces");

	let massData = b3MassData();
	massData.mass = 2.5;
	massData.center = v3(0.1, 0.0, -0.1);
	massData.inertia = b3Matrix3(v3(0.3, 0.0, 0.0), v3(0.0, 0.4, 0.0), v3(0.0, 0.0, 0.5));
	b3Body_SetMassData(boxBody, massData);
	dumpBody(boxBody);
	b3Body_SetMassData(boxBody, b3MassData());
	dumpBody(boxBody);
	b3Body_ApplyMassFromShapes(boxBody);

	b3Body_SetLinearDamping(compound, 0.3);
	b3Body_SetAngularDamping(compound, 0.4);
	b3Body_SetGravityScale(compound, 2.0);
	b3Body_SetSleepThreshold(compound, 0.02);
	b3Body_SetSafetyFactor(compound, 0.5);
	b3Body_SetUserData(compound, 99);
	b3Body_SetName(compound, "compound");
	b3Body_EnableSleep(compound, false);
	b3Body_EnableSleep(compound, true);
	b3Body_SetBullet(capsuleBody, true);
	b3Body_AllowFastRotation(capsuleBody, true);
	b3Body_EnableContactRecycling(capsuleBody, false);
	b3Body_EnableHitEvents(rockBody, true);

	let locks = b3MotionLocks();
	locks.linearX = true;
	locks.angularY = true;
	b3Body_SetMotionLocks(locked, locks);
	let allLocks = b3MotionLocks();
	allLocks.linearX = true;
	allLocks.linearY = true;
	allLocks.linearZ = true;
	allLocks.angularX = true;
	allLocks.angularY = true;
	allLocks.angularZ = true;
	b3Body_SetMotionLocks(capsuleBody, allLocks);
	dumpWorld("props");
	b3Body_SetMotionLocks(capsuleBody, b3MotionLocks());

	// Type changes
	b3Body_SetType(boxBody, b3_staticBody);
	dumpWorld("tostatic");
	b3Body_SetType(boxBody, b3_kinematicBody);
	dumpWorld("tokinematic");
	b3Body_SetType(boxBody, b3_dynamicBody);
	b3Body_SetType(kinematic, b3_dynamicBody);
	b3Body_SetType(disabled, b3_kinematicBody);
	dumpWorld("todynamic");

	// Enable and disable
	b3Body_Disable(rockBody);
	dumpWorld("disable");
	b3Body_Enable(rockBody);
	b3Body_Enable(disabled);
	b3Body_SetType(disabled, b3_dynamicBody);
	dumpWorld("enable");

	// Sleep
	b3Body_SetAwake(compound, false);
	dumpWorld("sleep");
	b3Body_SetAwake(compound, true);
	let target: b3WorldTransform = b3Transform(v3(6.5, 1.2, 0.1), b3MakeQuatFromAxisAngle(v3(0.0, 1.0, 0.0), 0.1));
	b3Body_SetTargetTransform(kinematic, target, <f32>1.0 / <f32>60.0, true);
	dumpWorld("wake");

	// Shape setters
	let shapes = new StaticArray<b3ShapeId>(SHAPE_CAPACITY);
	let shapeCount = b3Body_GetShapes(compound, shapes, SHAPE_CAPACITY);
	b3Shape_SetDensity(shapes[0], 5.0, true);
	b3Shape_SetDensity(shapes[1], 1.0, false);
	b3Shape_SetFriction(shapes[0], 0.25);
	b3Shape_SetRestitution(shapes[1], 0.75);
	let material = b3DefaultSurfaceMaterial();
	material.rollingResistance = 0.1;
	material.tangentVelocity = v3(1.0, 0.0, 0.0);
	material.userMaterialId = 42;
	b3Shape_SetSurfaceMaterial(shapes[2], material);
	b3Shape_SetUserData(shapes[0], 5);
	b3Shape_SetName(shapes[1], "cap");
	b3Shape_EnableContactEvents(shapes[0], false);
	b3Shape_EnablePreSolveEvents(shapes[1], true);
	b3Shape_EnableHitEvents(shapes[2], true);
	b3Shape_EnableSensorEvents(shapes[2], false);
	let filter = b3DefaultFilter();
	filter.categoryBits = 0x2;
	b3Shape_SetFilter(shapes[0], filter, true);
	filter.maskBits = 0xf;
	b3Shape_SetFilter(shapes[1], filter, false);
	vtag("setters");
	vi32(shapeCount);
	vend();

	let meshShapes = new StaticArray<b3ShapeId>(SHAPE_CAPACITY);
	b3Body_GetShapes(g_bodies[1], meshShapes, SHAPE_CAPACITY);
	let meshMaterial = b3DefaultSurfaceMaterial();
	meshMaterial.friction = 0.1;
	b3Shape_SetMeshMaterial(meshShapes[0], meshMaterial, 1);

	let sphere = b3Sphere(v3(0.1, 0.2, 0.3), 0.45);
	b3Shape_SetSphere(shapes[2], sphere);
	let capsule = b3Capsule(v3(0.0, -0.4, 0.0), v3(0.0, 0.4, 0.0), 0.2);
	b3Shape_SetCapsule(shapes[0], capsule);
	b3Shape_SetHull(shapes[1], g_rock);
	b3Shape_SetHull(shapes[1], g_rock);
	b3Body_ApplyMassFromShapes(compound);
	dumpWorld("shapes");

	// Wind
	let windShapes = new StaticArray<b3ShapeId>(SHAPE_CAPACITY);
	let windCount = b3Body_GetShapes(compound, windShapes, SHAPE_CAPACITY);
	for (let i = 0; i < windCount; ++i) {
		b3Shape_ApplyWind(windShapes[i], v3(5.0, 0.0, 1.0), 0.5, 0.2, 10.0, true);
	}
	let ballShapes = new StaticArray<b3ShapeId>(SHAPE_CAPACITY);
	b3Body_GetShapes(ball, ballShapes, SHAPE_CAPACITY);
	b3Shape_ApplyWind(ballShapes[0], v3(0.0, 0.0, -3.0), 1.0, 0.0, 10.0, true);
	dumpBody(compound);
	dumpBody(ball);

	// Destroy and reuse ids
	let rockShapes = new StaticArray<b3ShapeId>(SHAPE_CAPACITY);
	b3Body_GetShapes(rockBody, rockShapes, SHAPE_CAPACITY);
	b3DestroyShape(rockShapes[0], true);
	b3DestroyBody(sleeper);
	dumpWorld("destroy");

	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = v3(1.0, 5.0, -1.0);
		let bodyId = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		let box = b3MakeBoxHull(0.3, 0.3, 0.3);
		b3CreateHullShape(bodyId, shapeDef, box.base);
		let s = b3Sphere(v3(0.0, 0.4, 0.0), 0.2);
		b3CreateSphereShape(bodyId, shapeDef, s);
	}
	dumpWorld("reuse");
}

export function phaseStep(stepCount: i32): void {
	for (let i = 0; i < stepCount; ++i) {
		b3World_Step(g_worldId, <f32>1.0 / <f32>60.0, 4);
	}

	dumpWorld("step");
	dumpContacts();
}

function dumpEvents(): void {
	let contactEvents = b3World_GetContactEvents(g_worldId);
	vtag("events");
	vi32(contactEvents.beginCount);
	vi32(contactEvents.endCount);
	vi32(contactEvents.hitCount);
	for (let i = 0; i < contactEvents.hitCount; ++i) {
		let e = contactEvents.hitEvents[i];
		pshapeId(e.shapeIdA);
		pshapeId(e.shapeIdB);
		pvec(e.point);
		pvec(e.normal);
		vf32(e.approachSpeed);
		vu64(e.userMaterialIdA);
		vu64(e.userMaterialIdB);
	}
	let sensorEvents = b3World_GetSensorEvents(g_worldId);
	vi32(sensorEvents.beginCount);
	vi32(sensorEvents.endCount);
	for (let i = 0; i < sensorEvents.beginCount; ++i) {
		pshapeId(sensorEvents.beginEvents[i].sensorShapeId);
		pshapeId(sensorEvents.beginEvents[i].visitorShapeId);
	}
	vend();
}

function stepExtras(stepCount: i32): void {
	for (let i = 0; i < stepCount; ++i) {
		b3World_Step(g_worldId, <f32>1.0 / <f32>60.0, 4);
		dumpEvents();
	}

	dumpWorld("extras");
	dumpContacts();
}

let g_linkA: b3BodyId = b3BodyId();
let g_linkB: b3BodyId = b3BodyId();

export function phaseExtras(): void {
	vseed(0x657874726173);

	let worldDef = b3DefaultWorldDef();
	worldDef.hitEventThreshold = 0.1;
	g_worldId = b3CreateWorld(worldDef);
	g_bodyCount = 0;

	let materialA = b3DefaultSurfaceMaterial();
	materialA.userMaterialId = 21;
	let materialB = b3DefaultSurfaceMaterial();
	materialB.userMaterialId = 22;
	materialB.friction = 0.4;

	// 0: static compound with every child type
	let capsules = new StaticArray<b3CompoundCapsuleDef>(1);
	capsules[0].capsule = b3Capsule(v3(-3.0, 0.0, 0.0), v3(-2.0, 0.0, 0.5), 0.3);
	capsules[0].material = materialA;
	let box = b3MakeBoxHull(1.0, 0.25, 1.0);
	let hulls = new StaticArray<b3CompoundHullDef>(1);
	hulls[0] = new b3CompoundHullDef(box.base, b3Transform(v3(0.0, 0.0, 0.0), b3Quat_identity), materialB);
	let meshMaterials = new StaticArray<b3SurfaceMaterial>(2);
	meshMaterials[0] = materialA;
	meshMaterials[1] = materialB;
	let meshes = new StaticArray<b3CompoundMeshDef>(1);
	meshes[0] = new b3CompoundMeshDef(g_mesh);
	meshes[0].transform = b3Transform(v3(2.0, -0.2, -2.0), b3Quat_identity);
	meshes[0].scale = v3(1.0, 1.0, 1.0);
	meshes[0].materials = meshMaterials;
	meshes[0].materialCount = 2;
	let spheres = new StaticArray<b3CompoundSphereDef>(1);
	spheres[0].sphere = b3Sphere(v3(0.0, 0.0, 3.0), 0.5);
	spheres[0].material = materialA;
	let compoundDef = new b3CompoundDef();
	compoundDef.capsules = capsules;
	compoundDef.capsuleCount = 1;
	compoundDef.hulls = hulls;
	compoundDef.hullCount = 1;
	compoundDef.meshes = meshes;
	compoundDef.meshCount = 1;
	compoundDef.spheres = spheres;
	compoundDef.sphereCount = 1;
	let compound = b3CreateCompound(compoundDef)!;
	{
		let bodyDef = b3DefaultBodyDef();
		let bodyId = addBody(bodyDef);
		let shapeDef = b3DefaultShapeDef();
		shapeDef.enableHitEvents = true;
		b3CreateBakedCompoundShape(bodyId, shapeDef, compound);
	}

	// 1: static mesh and a sensor
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = v3(20.0, 0.0, 0.0);
		let bodyId = addBody(bodyDef);
		let materials = new StaticArray<b3SurfaceMaterial>(2);
		materials[0] = materialA;
		materials[1] = materialB;
		let shapeDef = b3DefaultShapeDef();
		shapeDef.materials = materials;
		shapeDef.materialCount = 2;
		shapeDef.enableHitEvents = true;
		b3CreateMeshShape(bodyId, shapeDef, g_mesh, v3(2.0, 1.0, 2.0));

		let sensorDef = b3DefaultShapeDef();
		sensorDef.isSensor = true;
		let sensorBox = b3MakeOffsetBoxHull(2.0, 2.0, 2.0, v3(4.0, 2.0, 4.0));
		b3CreateHullShape(bodyId, sensorDef, sensorBox.base);
	}

	// 2: static height field
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = v3(-20.0, 0.0, 0.0);
		let bodyId = addBody(bodyDef);
		let shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial = materialB;
		b3CreateHeightFieldShape(bodyId, shapeDef, g_heightField);
	}

	// 3..11: fast bullets onto each static shape
	for (let i = 0; i < 9; ++i) {
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		let target = i % 3;
		let x: f32 = target == 0 ? <f32>0.0 : target == 1 ? <f32>24.0 : <f32>-18.0;
		let px = x + vrandf(-1.0, 1.0);
		let py = <f32>3.0 + vrandf(0.0, 1.0);
		let pz = vrandf(-1.0, 1.0) + (target == 0 ? <f32>0.0 : <f32>2.0);
		bodyDef.position = v3(px, py, pz);
		let vx = vrandf(-2.0, 2.0);
		let vz = vrandf(-2.0, 2.0);
		bodyDef.linearVelocity = v3(vx, -150.0, vz);
		bodyDef.isBullet = (i & 1) == 0;
		let bodyId = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		shapeDef.enableHitEvents = true;
		shapeDef.enableSensorEvents = true;
		let kind = i / 3;
		if (kind == 0) {
			let sphere = b3Sphere(v3(0.0, 0.0, 0.0), 0.1);
			b3CreateSphereShape(bodyId, shapeDef, sphere);
		} else if (kind == 1) {
			let capsule = b3Capsule(v3(0.0, -0.1, 0.0), v3(0.0, 0.1, 0.0), 0.08);
			b3CreateCapsuleShape(bodyId, shapeDef, capsule);
		} else {
			let bulletBox = b3MakeBoxHull(0.1, 0.1, 0.1);
			b3CreateHullShape(bodyId, shapeDef, bulletBox.base);
		}
	}

	// 12, 13: jointed pair hanging from the compound body
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = v3(0.0, 3.0, -3.0);
		g_linkA = addBody(bodyDef);
		bodyDef.position = v3(1.0, 3.0, -3.0);
		g_linkB = addBody(bodyDef);

		let shapeDef = b3DefaultShapeDef();
		let linkBox = b3MakeBoxHull(0.4, 0.1, 0.1);
		b3CreateHullShape(g_linkA, shapeDef, linkBox.base);
		b3CreateHullShape(g_linkB, shapeDef, linkBox.base);

		let revoluteDef = b3DefaultRevoluteJointDef();
		revoluteDef.base.bodyIdA = g_linkA;
		revoluteDef.base.bodyIdB = g_linkB;
		revoluteDef.base.localFrameA.p = v3(0.5, 0.0, 0.0);
		revoluteDef.base.localFrameB.p = v3(-0.5, 0.0, 0.0);
		b3CreateRevoluteJoint(g_worldId, revoluteDef);

		let sphericalDef = b3DefaultSphericalJointDef();
		sphericalDef.base.bodyIdA = g_bodies[0];
		sphericalDef.base.bodyIdB = g_linkA;
		sphericalDef.base.localFrameA.p = v3(-0.5, 3.0, -3.0);
		sphericalDef.base.localFrameB.p = v3(-0.5, 0.0, 0.0);
		b3CreateSphericalJoint(g_worldId, sphericalDef);
	}

	dumpWorld("extras");

	let joints = new StaticArray<b3JointId>(4);
	let jointCount = b3Body_GetJoints(g_linkA, joints, 4);
	vtag("joints");
	vi32(jointCount);
	for (let i = 0; i < jointCount; ++i) {
		vi32(joints[i].index1);
		vu32(joints[i].generation);
	}
	vend();

	// Queries against the compound
	let filter = b3DefaultQueryFilter();
	let compoundShapes = new StaticArray<b3ShapeId>(SHAPE_CAPACITY);
	b3Body_GetShapes(g_bodies[0], compoundShapes, SHAPE_CAPACITY);
	let aims = new StaticArray<b3Vec3>(4);
	aims[0] = v3(-2.5, 0.0, 0.25);
	aims[1] = v3(0.0, 0.0, 0.0);
	aims[2] = v3(3.0, 0.0, -1.0);
	aims[3] = v3(0.0, 0.0, 3.0);
	for (let k = 0; k < 8; ++k) {
		let origin = b3Add(aims[k % 4], rvec(-0.5, 0.5));
		origin.y = 3.0;
		let tx = vrandf(-1.0, 1.0);
		let tz = vrandf(-1.0, 1.0);
		let translation = v3(tx, -6.0, tz);
		let ray = b3Body_CastRay(g_bodies[0], origin, translation, filter, 1.0, b3Body_GetTransform(g_bodies[0]));
		vtag("cray");
		pshapeId(ray.shapeId);
		pvec(ray.point);
		vf32(ray.fraction);
		vi32(ray.triangleIndex);
		vu64(ray.userMaterialId);
		vbool(ray.hit);
		pcast(b3Shape_RayCast(compoundShapes[0], origin, translation));
		vend();

		let points = new StaticArray<b3Vec3>(1);
		points[0] = origin;
		let proxy = new b3ShapeProxy(points, 1, 0.3);
		let cast = b3Body_CastShape(g_bodies[0], origin, proxy, translation, filter, 1.0, false, b3Body_GetTransform(g_bodies[0]));
		vtag("ccast");
		pshapeId(cast.shapeId);
		pvec(cast.point);
		pvec(cast.normal);
		vf32(cast.fraction);
		vu64(cast.userMaterialId);
		vbool(cast.hit);
		let low = b3MulAdd(origin, 0.5, translation);
		let lowPoints = new StaticArray<b3Vec3>(1);
		lowPoints[0] = low;
		let lowProxy = new b3ShapeProxy(lowPoints, 1, 0.5);
		vbool(b3Body_OverlapShape(g_bodies[0], b3Vec3_zero, lowProxy, filter, b3Body_GetTransform(g_bodies[0])));
		vend();
	}
}

export function phaseExtrasStep(): void {
	stepExtras(12);
}

export function phaseExtrasJoints(): void {
	let linkA = g_linkA;
	let linkB = g_linkB;

	// Joints across type changes, disable and destroy
	b3Body_SetType(linkA, b3_staticBody);
	dumpWorld("jstatic");
	b3Body_SetType(linkA, b3_kinematicBody);
	b3Body_SetType(linkA, b3_dynamicBody);
	dumpWorld("jdynamic");
	b3Body_Disable(linkB);
	dumpWorld("jdisable");
	stepExtras(5);
	b3Body_Enable(linkB);
	b3Body_SetAwake(linkB, false);
	dumpWorld("jenable");
	stepExtras(5);

	// Mesh swap on the mesh body
	let meshShapes = new StaticArray<b3ShapeId>(SHAPE_CAPACITY);
	let meshShapeCount = b3Body_GetShapes(g_bodies[1], meshShapes, SHAPE_CAPACITY);
	for (let i = 0; i < meshShapeCount; ++i) {
		if (b3Shape_GetType(meshShapes[i]) == b3_meshShape) {
			b3Shape_SetMesh(meshShapes[i], g_mesh, v3(-1.5, 1.0, 1.5));
		}
	}

	b3DestroyBody(linkA);
	dumpWorld("jdestroy");
	stepExtras(10);
}
