// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/math_functions.as, lib/math_internal.as, lib/core.as, lib/constants.as,
// lib/types.as, lib/collision.as and lib/aabb.as, mirrors verify/cref/math.c

import { vtag, vi32, vu32, vbool, vf32, vu64, vend, vseed, vrand32, vrand64, vrandint, vrandf, vspecialf } from "./common";
import * as core from "../../lib/core";
import * as k from "../../lib/constants";
import * as m3 from "../../lib/math_functions";
import * as mi from "../../lib/math_internal";
import * as col from "../../lib/collision";
import * as ty from "../../lib/types";
import * as bb from "../../lib/aabb";
import {
	b3AABB,
	b3Matrix3,
	b3Plane,
	b3Quat,
	b3SegmentDistanceResult,
	b3Transform,
	b3Vec2,
	b3Vec3,
} from "../../lib/math_functions";
import { b3Matrix2 } from "../../lib/math_internal";

import { b3Version, b3AtomicInt, b3AtomicU32, b3AtomicI64 } from "../../lib/core";
import { b3CosSin } from "../../lib/math_functions";
import { b3Triangle, b3TrianglePoint, b3ShapeExtent } from "../../lib/math_internal";
import { b3WorldId, b3BodyId, b3ShapeId, b3JointId, b3ContactId, b3Capacity, b3MotionLocks, b3Filter, b3SurfaceMaterial, b3Profile, b3Counters, b3JointDef, b3DistanceJointDef, b3MotorJointDef, b3FilterJointDef, b3ParallelJointDef, b3PrismaticJointDef, b3RevoluteJointDef, b3SphericalJointDef, b3WeldJointDef, b3WheelJointDef, b3ExplosionDef, b3SensorBeginTouchEvent, b3SensorEndTouchEvent, b3ContactBeginTouchEvent, b3ContactEndTouchEvent, b3ContactHitEvent, b3BodyMoveEvent, b3JointEvent, b3QueryFilter, b3RayResult, b3BodyCastResult, b3BodyPlaneResult, b3BodyTOIResult } from "../../lib/types";
import { b3RayCastInput, b3BoxCastInput, b3CastOutput, b3SimplexCache, b3DistanceOutput, b3SimplexVertex, b3Simplex, b3Sweep, b3TOIOutput, b3TreeNode, b3TreeProxy, b3TreeStats, b3PlaneResult, b3CollisionPlane, b3PlaneSolverResult, b3MassData, b3Sphere, b3Capsule, b3HullVertex, b3HullHalfEdge, b3HullFace, b3MeshTriangle, b3MeshNode, b3CompoundCapsuleDef, b3CompoundSphereDef, b3HullInstance, b3MeshInstance, b3CompoundCapsule, b3CompoundSphere, b3ManifoldPoint, b3Manifold, b3SATCache, b3FeaturePair, b3LocalManifoldPoint, b3Point2D } from "../../lib/collision";

export { frameBoundary } from "./common";

function pv2(v: b3Vec2): void {
	vf32(v.x);
	vf32(v.y);
}

function pv3(v: b3Vec3): void {
	vf32(v.x);
	vf32(v.y);
	vf32(v.z);
}

function pq(q: b3Quat): void {
	pv3(q.v);
	vf32(q.s);
}

function pm3(m: b3Matrix3): void {
	pv3(m.cx);
	pv3(m.cy);
	pv3(m.cz);
}

function pm2(m: b3Matrix2): void {
	pv2(m.cx);
	pv2(m.cy);
}

function pab(a: b3AABB): void {
	pv3(a.lowerBound);
	pv3(a.upperBound);
}

function pxf(t: b3Transform): void {
	pv3(t.p);
	pq(t.q);
}

function ppl(p: b3Plane): void {
	pv3(p.normal);
	vf32(p.offset);
}

function pseg(r: b3SegmentDistanceResult): void {
	pv3(r.point1);
	vf32(r.fraction1);
	pv3(r.point2);
	vf32(r.fraction2);
}

// clang lowers a > b ? a : b against a constant to fmaxnm, which quiets a signaling NaN,
// so signaling inputs are quieted where a function feeds one into such a select.
function quietF(x: f32): f32 {
	let bits = reinterpret<u32>(x);
	if ((bits & 0x7f800000) == 0x7f800000 && (bits & 0x007fffff) != 0) {
		bits |= 0x00400000;
	}
	return reinterpret<f32>(bits);
}

function quietVec(v: b3Vec3): b3Vec3 {
	return b3Vec3(quietF(v.x), quietF(v.y), quietF(v.z));
}

// mode 0 special, 1 tiny, 2 large, 3 ordinary
function rcomp(mode: i32): f32 {
	if (mode == 0) {
		return vspecialf();
	}
	if (mode == 1) {
		return vrandf(-1.0e-3, 1.0e-3);
	}
	if (mode == 2) {
		return vrandf(-1.0e4, 1.0e4);
	}
	return vrandf(-10.0, 10.0);
}

function rscalar(): f32 {
	let mode = <i32>(vrand32() % 4);
	return rcomp(mode);
}

function rvec(): b3Vec3 {
	let mode = <i32>(vrand32() % 4);
	let v: b3Vec3;
	v.x = rcomp(mode);
	v.y = rcomp(mode);
	v.z = rcomp(mode);
	return v;
}

function rvecn(): b3Vec3 {
	let v: b3Vec3;
	v.x = vrandf(-10.0, 10.0);
	v.y = vrandf(-10.0, 10.0);
	v.z = vrandf(-10.0, 10.0);
	return v;
}

function rvec2(): b3Vec2 {
	let mode = <i32>(vrand32() % 4);
	let v: b3Vec2;
	v.x = rcomp(mode);
	v.y = rcomp(mode);
	return v;
}

function runit(): b3Vec3 {
	let v = rvecn();
	v = m3.b3Normalize(v);
	if (m3.b3LengthSquared(v) == 0.0) {
		v = m3.b3Vec3_axisX;
	}
	return v;
}

function rquat(): b3Quat {
	let q: b3Quat;
	q.v.x = vrandf(-1.0, 1.0);
	q.v.y = vrandf(-1.0, 1.0);
	q.v.z = vrandf(-1.0, 1.0);
	q.s = vrandf(-1.0, 1.0);
	return m3.b3NormalizeQuat(q);
}

function rquatAny(): b3Quat {
	let mode = vrand32() % 4;
	if (mode == 0) {
		let q: b3Quat;
		q.v.x = vspecialf();
		q.v.y = vspecialf();
		q.v.z = vspecialf();
		q.s = vspecialf();
		return q;
	}
	if (mode == 1) {
		let q: b3Quat;
		q.v = rvecn();
		q.s = vrandf(-10.0, 10.0);
		return q;
	}
	return rquat();
}

function rmat(): b3Matrix3 {
	let mode = vrand32() % 8;
	let m: b3Matrix3;
	if (mode == 0) {
		m.cx = rvec();
		m.cy = rvec();
		m.cz = rvec();
		return m;
	}
	if (mode == 1) {
		m.cx = rvecn();
		m.cy = rvecn();
		m.cz = m3.b3Add(m.cx, m.cy);
		return m;
	}
	if (mode == 2) {
		let q = rquat();
		return m3.b3MakeMatrixFromQuat(q);
	}
	if (mode == 3) {
		let s = vrandf(1.0e-15, 1.0e-12);
		m.cx = m3.b3MulSV(s, rvecn());
		m.cy = m3.b3MulSV(s, rvecn());
		m.cz = m3.b3MulSV(s, rvecn());
		return m;
	}
	m.cx = rvecn();
	m.cy = rvecn();
	m.cz = rvecn();
	return m;
}

function rxf(): b3Transform {
	let t: b3Transform;
	t.p = rvecn();
	t.q = rquat();
	return t;
}

function raabb(): b3AABB {
	let mode = vrand32() % 6;
	let a: b3AABB;
	if (mode == 0) {
		a.lowerBound = rvec();
		a.upperBound = rvec();
		return a;
	}
	let c = rvecn();
	let e = m3.b3Abs(rvecn());
	if (mode == 1) {
		e = m3.b3Neg(e);
	}
	a.lowerBound = m3.b3Sub(c, e);
	a.upperBound = m3.b3Add(c, e);
	return a;
}

function rplane(): b3Plane {
	let p: b3Plane;
	let mode = vrand32() % 4;
	if (mode == 0) {
		p.normal = rvec();
		p.offset = vspecialf();
		return p;
	}
	p.normal = runit();
	p.offset = vrandf(-10.0, 10.0);
	return p;
}

function checkConstants(): void {
	vtag("const");
	vf32(m3.B3_PI);
	vf32(m3.B3_DEG_TO_RAD);
	vf32(m3.B3_RAD_TO_DEG);
	vf32(m3.B3_MIN_SCALE);
	vf32(mi.B3_TWO_PI);
	vf32(mi.B3_PI_OVER_TWO);
	vf32(mi.B3_PI_OVER_FOUR);
	vf32(mi.B3_SQRT3);
	vf32(core.FLT_MIN);
	vf32(core.FLT_MAX);
	vf32(core.FLT_EPSILON);
	vf32(k.B3_MAX_ROTATION);
	vf32(k.B3_MIN_FRICTION_WEIGHT);
	vf32(k.B3_CONTACT_RECYCLE_ANGULAR_DISTANCE);
	vf32(k.B3_PARALLEL_EDGE_TOL);
	vf32(k.B3_AABB_MARGIN_FRACTION);
	vf32(k.B3_TIME_TO_SLEEP);
	vend();

	vtag("iconst");
	vi32(k.B3_MAX_WORKERS);
	vi32(k.B3_MAX_TASKS);
	vi32(k.B3_GRAPH_COLOR_COUNT);
	vi32(k.B3_CONTACT_MANIFOLD_COUNT_BUCKETS);
	vi32(k.B3_MAX_WORLDS);
	vi32(k.B3_MAX_RESTITUTION_ITERATIONS);
	vi32(k.B3_MAX_MANIFOLD_POINTS);
	vi32(k.B3_GYROSCOPIC_ITERATIONS);
	vi32(k.B3_MAX_HULL_VERTICES);
	vi32(k.B3_MAX_HULL_FACES);
	vi32(k.B3_MAX_HULL_EDGES);
	vi32(k.B3_MAX_SHAPE_CAST_POINTS);
	vi32(k.B3_SHAPE_POWER);
	vi32(k.B3_CHILD_POWER);
	vi32(k.B3_MAX_SHAPES);
	vi32(k.B3_MAX_CHILD_SHAPES);
	vi32(k.B3_MAX_MESH_CONTACT_TRIANGLES);
	vi32(core.B3_NULL_INDEX);
	vi32(core.B3_SECRET_COOKIE);
	vi32(core.B3_ALIGNMENT);
	vi32(col.B3_HEIGHT_FIELD_HOLE);
	vu32(core.B3_HASH_INIT);
	vend();

	vtag("versions");
	vu64(col.B3_HULL_VERSION);
	vu64(col.B3_MESH_VERSION);
	vu64(col.B3_HEIGHT_FIELD_VERSION);
	vu64(col.B3_COMPOUND_VERSION);
	vu64(col.B3_DYNAMIC_TREE_VERSION);
	vu64(ty.B3_DEFAULT_CATEGORY_BITS);
	vu64(ty.B3_DEFAULT_MASK_BITS);
	let version = core.b3GetVersion();
	vi32(version.major);
	vi32(version.minor);
	vi32(version.revision);
	vbool(core.b3IsDoublePrecision());
	vi32(core.b3GetMaxManifoldPoints());
	vend();

	vtag("statics");
	pv3(m3.b3Vec3_zero);
	pv3(m3.b3Vec3_one);
	pv3(m3.b3Vec3_axisX);
	pv3(m3.b3Vec3_axisY);
	pv3(m3.b3Vec3_axisZ);
	pq(m3.b3Quat_identity);
	pxf(m3.b3Transform_identity);
	pm3(m3.b3Mat3_zero);
	pm3(m3.b3Mat3_identity);
	pv3(m3.b3Pos_zero);
	pxf(m3.b3WorldTransform_identity);
	pab(mi.B3_BOUNDS3_EMPTY);
	vf32(col.b3_emptyDistanceCache.metric);
	vu32(col.b3_emptyDistanceCache.count);
	for (let i = 0; i < 4; ++i) {
		vu32(col.b3_emptyDistanceCache.indexA[i]);
		vu32(col.b3_emptyDistanceCache.indexB[i]);
	}
	vend();

	vtag("edgeflags");
	vi32(col.b3_concaveEdge1);
	vi32(col.b3_concaveEdge2);
	vi32(col.b3_concaveEdge3);
	vi32(col.b3_inverseConcaveEdge1);
	vi32(col.b3_inverseConcaveEdge2);
	vi32(col.b3_inverseConcaveEdge3);
	vi32(col.b3_allConcaveEdges);
	vi32(col.b3_flatEdge1);
	vi32(col.b3_flatEdge2);
	vi32(col.b3_flatEdge3);
	vi32(col.b3_allFlatEdges);
	vi32(ty.b3_bodyTypeCount);
	vi32(ty.b3_shapeTypeCount);
	vi32(ty.b3_sphereShape);
	vi32(ty.b3_wheelJoint);
	vi32(col.b3_toiStateSeparated);
	vi32(col.b3_manualEdgePairAxis);
	vi32(col.b3_featureVertex3);
	vend();
}

function checkUnitConstants(units: f32): void {
	k.b3SetLengthUnitsPerMeter(units);
	vtag("units");
	vf32(k.b3GetLengthUnitsPerMeter());
	vf32(k.B3_HUGE());
	vf32(k.B3_LINEAR_SLOP());
	vf32(k.B3_MIN_CAPSULE_LENGTH());
	vf32(k.B3_OVERLAP_SLOP());
	vf32(k.B3_SPECULATIVE_DISTANCE());
	vf32(k.B3_MESH_REST_OFFSET());
	vf32(k.B3_CONTACT_RECYCLE_DISTANCE());
	vf32(k.B3_MAX_AABB_MARGIN());
	vend();
}

function pjointdef(d: ty.b3JointDef): void {
	vu64(d.userData);
	vi32(d.bodyIdA.index1);
	vu32(d.bodyIdA.world0);
	vu32(d.bodyIdA.generation);
	vi32(d.bodyIdB.index1);
	vu32(d.bodyIdB.world0);
	vu32(d.bodyIdB.generation);
	pxf(d.localFrameA);
	pxf(d.localFrameB);
	vf32(d.forceThreshold);
	vf32(d.torqueThreshold);
	vf32(d.constraintHertz);
	vf32(d.constraintDampingRatio);
	vf32(d.drawScale);
	vbool(d.collideConnected);
	vi32(d.internalValue);
}

function psurface(m: ty.b3SurfaceMaterial): void {
	vf32(m.friction);
	vf32(m.restitution);
	vf32(m.rollingResistance);
	pv3(m.tangentVelocity);
	vu64(m.userMaterialId);
	vu32(m.customColor);
	vu32(m.padding);
}

function pfilter(f: ty.b3Filter): void {
	vu64(f.categoryBits);
	vu64(f.maskBits);
	vi32(f.groupIndex);
}

function checkDefaults(units: f32): void {
	k.b3SetLengthUnitsPerMeter(units);

	{
		let d = ty.b3DefaultWorldDef();
		vtag("worlddef");
		pv3(d.gravity);
		vf32(d.restitutionThreshold);
		vi32(d.restitutionIterations);
		vbool(d.enableRestitutionPropagation);
		vf32(d.hitEventThreshold);
		vf32(d.contactHertz);
		vf32(d.contactDampingRatio);
		vf32(d.contactSpeed);
		vf32(d.maximumLinearSpeed);
		vf32(d.isotropicInertiaTolerance);
		vbool(d.frictionCallback != null);
		vbool(d.restitutionCallback != null);
		vbool(d.enableSleep);
		vbool(d.enableContinuous);
		vu32(d.workerCount);
		vbool(d.enqueueTask != null);
		vbool(d.finishTask != null);
		vu64(d.userTaskContext);
		vu64(d.userData);
		vbool(d.createDebugShape != null);
		vbool(d.destroyDebugShape != null);
		vu64(d.userDebugShapeContext);
		vi32(d.capacity.staticShapeCount);
		vi32(d.capacity.dynamicShapeCount);
		vi32(d.capacity.staticBodyCount);
		vi32(d.capacity.dynamicBodyCount);
		vi32(d.capacity.contactCount);
		vi32(d.internalValue);
		vend();
	}

	{
		let d = ty.b3DefaultBodyDef();
		vtag("bodydef");
		vi32(d.type);
		pv3(d.position);
		pq(d.rotation);
		pv3(d.linearVelocity);
		pv3(d.angularVelocity);
		vf32(d.linearDamping);
		vf32(d.angularDamping);
		vf32(d.gravityScale);
		vf32(d.sleepThreshold);
		vf32(d.safetyFactor);
		vbool(d.name.length != 0);
		vu64(d.userData);
		vbool(d.motionLocks.linearX);
		vbool(d.motionLocks.linearY);
		vbool(d.motionLocks.linearZ);
		vbool(d.motionLocks.angularX);
		vbool(d.motionLocks.angularY);
		vbool(d.motionLocks.angularZ);
		vbool(d.enableSleep);
		vbool(d.isAwake);
		vbool(d.isBullet);
		vbool(d.isEnabled);
		vbool(d.allowFastRotation);
		vbool(d.enableContactRecycling);
		vi32(d.internalValue);
		vend();
	}

	vtag("filter");
	pfilter(ty.b3DefaultFilter());
	vend();

	{
		let f = ty.b3DefaultQueryFilter();
		vtag("queryfilter");
		vu64(f.categoryBits);
		vu64(f.maskBits);
		vu64(f.id);
		// the C name field is left out of the port and is NULL by default
		vbool(false);
		vend();
	}

	vtag("surface");
	psurface(ty.b3DefaultSurfaceMaterial());
	vend();

	{
		let d = ty.b3DefaultShapeDef();
		vtag("shapedef");
		vbool(d.name.length != 0);
		vu64(d.userData);
		vbool(d.materials != null);
		vi32(d.materialCount);
		psurface(d.baseMaterial);
		vf32(d.density);
		vf32(d.explosionScale);
		pfilter(d.filter);
		vbool(d.enableCustomFiltering);
		vbool(d.isSensor);
		vbool(d.enableSensorEvents);
		vbool(d.enableContactEvents);
		vbool(d.enableHitEvents);
		vbool(d.enablePreSolveEvents);
		vbool(d.invokeContactCreation);
		vbool(d.updateBodyMass);
		vbool(d.enableSpeculativeContact);
		vi32(d.internalValue);
		vend();
	}

	{
		let d = ty.b3DefaultParallelJointDef();
		vtag("parallel");
		pjointdef(d.base);
		vf32(d.hertz);
		vf32(d.dampingRatio);
		vf32(d.maxTorque);
		vend();
	}

	{
		let d = ty.b3DefaultDistanceJointDef();
		vtag("distance");
		pjointdef(d.base);
		vf32(d.length);
		vbool(d.enableSpring);
		vf32(d.lowerSpringForce);
		vf32(d.upperSpringForce);
		vf32(d.hertz);
		vf32(d.dampingRatio);
		vbool(d.enableLimit);
		vf32(d.minLength);
		vf32(d.maxLength);
		vbool(d.enableMotor);
		vf32(d.maxMotorForce);
		vf32(d.motorSpeed);
		vend();
	}

	{
		let d = ty.b3DefaultMotorJointDef();
		vtag("motor");
		pjointdef(d.base);
		pv3(d.linearVelocity);
		vf32(d.maxVelocityForce);
		pv3(d.angularVelocity);
		vf32(d.maxVelocityTorque);
		vf32(d.linearHertz);
		vf32(d.linearDampingRatio);
		vf32(d.maxSpringForce);
		vf32(d.angularHertz);
		vf32(d.angularDampingRatio);
		vf32(d.maxSpringTorque);
		vend();
	}

	{
		let d = ty.b3DefaultFilterJointDef();
		vtag("filterjoint");
		pjointdef(d.base);
		vend();
	}

	{
		let d = ty.b3DefaultPrismaticJointDef();
		vtag("prismatic");
		pjointdef(d.base);
		vbool(d.enableSpring);
		vf32(d.hertz);
		vf32(d.dampingRatio);
		vf32(d.targetTranslation);
		vbool(d.enableLimit);
		vf32(d.lowerTranslation);
		vf32(d.upperTranslation);
		vbool(d.enableMotor);
		vf32(d.maxMotorForce);
		vf32(d.motorSpeed);
		vend();
	}

	{
		let d = ty.b3DefaultRevoluteJointDef();
		vtag("revolute");
		pjointdef(d.base);
		vf32(d.targetAngle);
		vbool(d.enableSpring);
		vf32(d.hertz);
		vf32(d.dampingRatio);
		vbool(d.enableLimit);
		vf32(d.lowerAngle);
		vf32(d.upperAngle);
		vbool(d.enableMotor);
		vf32(d.maxMotorTorque);
		vf32(d.motorSpeed);
		vend();
	}

	{
		let d = ty.b3DefaultSphericalJointDef();
		vtag("spherical");
		pjointdef(d.base);
		vbool(d.enableSpring);
		vf32(d.hertz);
		vf32(d.dampingRatio);
		pq(d.targetRotation);
		vbool(d.enableConeLimit);
		vf32(d.coneAngle);
		vbool(d.enableTwistLimit);
		vf32(d.lowerTwistAngle);
		vf32(d.upperTwistAngle);
		vbool(d.enableMotor);
		vf32(d.maxMotorTorque);
		pv3(d.motorVelocity);
		vend();
	}

	{
		let d = ty.b3DefaultWeldJointDef();
		vtag("weld");
		pjointdef(d.base);
		vf32(d.linearHertz);
		vf32(d.angularHertz);
		vf32(d.linearDampingRatio);
		vf32(d.angularDampingRatio);
		vend();
	}

	{
		let d = ty.b3DefaultWheelJointDef();
		vtag("wheel");
		pjointdef(d.base);
		vbool(d.enableSuspensionSpring);
		vf32(d.suspensionHertz);
		vf32(d.suspensionDampingRatio);
		vbool(d.enableSuspensionLimit);
		vf32(d.lowerSuspensionLimit);
		vf32(d.upperSuspensionLimit);
		vbool(d.enableSpinMotor);
		vf32(d.maxSpinTorque);
		vf32(d.spinSpeed);
		vbool(d.enableSteering);
		vf32(d.steeringHertz);
		vf32(d.steeringDampingRatio);
		vf32(d.targetSteeringAngle);
		vf32(d.maxSteeringTorque);
		vbool(d.enableSteeringLimit);
		vf32(d.lowerSteeringLimit);
		vf32(d.upperSteeringLimit);
		vend();
	}

	{
		let d = ty.b3DefaultExplosionDef();
		vtag("explosion");
		vu64(d.maskBits);
		pv3(d.position);
		vf32(d.radius);
		vf32(d.falloff);
		vf32(d.impulsePerArea);
		vend();
	}
}

function checkScalars(): void {
	for (let i = 0; i < 6000; ++i) {
		let a = vspecialf();
		let b = vspecialf();
		let c = vspecialf();
		let alpha = vrandf(0.0, 1.0);
		let r = vrandf(-20.0, 20.0);
		let r2 = vrandf(-20.0, 20.0);
		let ia = <i32>vrand32();
		let ib = vrandint(-100, 100);
		let ic = vrandint(-100, 100);
		let n = vrandint(0, 10000);
		let e = vrandint(1, 10);
		let d = vrandint(1, 100);

		vtag("s1");
		vf32(a);
		vf32(b);
		vf32(c);
		vi32(m3.b3MinInt(ia, ib));
		vi32(m3.b3MaxInt(ia, ib));
		vi32(m3.b3ClampInt(ia, m3.b3MinInt(ib, ic), m3.b3MaxInt(ib, ic)));
		vi32(m3.b3ClampInt(ib, -10, 10));
		vf32(m3.b3AbsFloat(a));
		vf32(m3.b3MinFloat(a, b));
		vf32(m3.b3MaxFloat(a, b));
		vf32(m3.b3ClampFloat(a, b, c));
		vf32(m3.b3LerpFloat(a, b, alpha));
		vbool(m3.b3IsValidFloat(a));
		vi32(core.b3FloatToIntSat(a));
		vu32(core.b3FloatToUIntSat(a));
		vend();

		vtag("s2");
		vf32(m3.b3Atan2(a, b));
		vf32(m3.b3Atan2(r, r2));
		vf32(m3.b3Atan2(r, 0.0));
		vf32(m3.b3Atan2(0.0, r2));
		let cs = m3.b3ComputeCosSin(a);
		vf32(cs.cosine);
		vf32(cs.sine);
		cs = m3.b3ComputeCosSin(r);
		vf32(cs.cosine);
		vf32(cs.sine);
		vf32(m3.b3Sin(r2));
		vf32(m3.b3Cos(r2));
		vf32(m3.b3UnwindAngle(a));
		vf32(m3.b3UnwindAngle(r));
		vi32(mi.b3AlignUp8(n));
		vi32(mi.b3CeilingInt(n, d));
		vi32(mi.b3CeilingPow2(n, 1 << e, e));
		vend();
	}
}

function checkVectors(): void {
	for (let i = 0; i < 6000; ++i) {
		let a = rvec();
		let b = rvec();
		let c = rvec();
		let s = rscalar();
		let t = rscalar();
		let alpha = vrandf(0.0, 1.0);
		let kk = <i32>(vrand32() % 3);
		let u = runit();

		vtag("v1");
		pv3(a);
		pv3(b);
		pv3(c);
		vf32(s);
		vf32(t);
		pv3(m3.b3Add(a, b));
		pv3(m3.b3Sub(a, b));
		pv3(m3.b3Mul(a, b));
		pv3(m3.b3Neg(a));
		vf32(m3.b3Dot(a, b));
		vf32(m3.b3Length(a));
		vf32(m3.b3LengthSquared(a));
		vf32(m3.b3Distance(a, b));
		vf32(m3.b3DistanceSquared(a, b));
		pv3(m3.b3Normalize(a));
		let out = b3Vec2(-7.0, -7.0);
		let n = m3.b3GetLengthAndNormalize(out.x, a);
		vf32(out.x);
		pv3(n);
		pv3(m3.b3Perp(a));
		vbool(m3.b3IsNormalized(a));
		vbool(m3.b3IsNormalized(u));
		vend();

		vtag("v2");
		pv3(m3.b3MulAdd(a, s, b));
		pv3(m3.b3MulSub(a, s, b));
		pv3(m3.b3MulSV(s, a));
		pv3(m3.b3Cross(a, b));
		pv3(m3.b3Lerp(a, b, alpha));
		pv3(m3.b3Blend2(s, a, t, b));
		pv3(m3.b3Abs(a));
		pv3(m3.b3Sign(a));
		pv3(m3.b3Min(a, b));
		pv3(m3.b3Max(a, b));
		pv3(m3.b3Clamp(a, b, c));
		pv3(m3.b3SafeScale(quietVec(a)));
		vbool(m3.b3IsValidVec3(a));
		pv3(mi.b3ClampLength(a, m3.b3AbsFloat(s)));
		vend();

		vtag("v3");
		vf32(mi.b3ScalarTripleProduct(a, b, c));
		vf32(mi.b3GetByIndex(a, kk));
		vi32(mi.b3MajorAxis(a));
		vf32(mi.b3MinElement(a));
		vf32(mi.b3MaxElement(a));
		vi32(mi.b3MaxElementIndex(a));
		pv3(mi.b3Blend3(s, a, t, b, alpha, c));
		pv3(mi.b3ModifiedCross(a, b));
		pv3(m3.b3ToPos(a));
		pv3(m3.b3ToVec3(a));
		pv3(m3.b3SubPos(a, b));
		pv3(m3.b3OffsetPos(a, b));
		pv3(m3.b3LerpPosition(a, b, alpha));
		vf32(m3.b3RoundDownFloat(a.x));
		vf32(m3.b3RoundUpFloat(a.y));
		vbool(m3.b3IsValidPosition(a));
		pv3(mi.b3ArbitraryPerp(u));
		vend();
	}
}

function checkVec2(): void {
	for (let i = 0; i < 3000; ++i) {
		let a = rvec2();
		let b = rvec2();
		let s = rscalar();
		let m: b3Matrix2;
		m.cx = rvec2();
		m.cy = rvec2();
		let m2: b3Matrix2;
		m2.cx = rvec2();
		m2.cy = rvec2();

		// positive definite
		let p: b3Matrix2;
		let px = vrandf(0.1, 10.0);
		let py = vrandf(0.1, 10.0);
		let pxy = vrandf(-0.05, 0.05);
		p.cx = b3Vec2(px, pxy);
		p.cy = b3Vec2(pxy, py);

		vtag("w");
		pv2(a);
		pv2(b);
		vf32(s);
		vf32(mi.b3Dot2(a, b));
		vf32(mi.b3Length2(a));
		vf32(mi.b3LengthSquared2(a));
		pv2(mi.b3MinVec2(a, b));
		pv2(mi.b3MaxVec2(a, b));
		pv2(mi.b3Add2(a, b));
		pv2(mi.b3Sub2(a, b));
		pv2(mi.b3Neg2(a));
		pv2(mi.b3MulSV2(s, a));
		pv2(mi.b3MulAdd2(a, s, b));
		pv2(mi.b3MulSub2(a, s, b));
		vf32(mi.b3Cross2(a, b));
		vf32(mi.b3DistanceSquared2(a, b));
		pv2(mi.b3MulMV2(m, a));
		pm2(mi.b3MulMM2(m, m2));
		vf32(mi.b3Det2(m));
		pm2(mi.b3Invert2(m));
		pv2(mi.b3Solve2(m, a));
		pv2(mi.b3Solve2(p, a));
		vend();
	}
}

function checkQuats(): void {
	for (let i = 0; i < 4000; ++i) {
		let q1 = rquatAny();
		let q2 = rquatAny();
		let n1 = rquat();
		let n2 = rquat();
		let v = rvec();
		let axis = runit();
		let angle = vrandf(-10.0, 10.0);
		let alpha = vrandf(0.0, 1.0);
		let u1 = runit();
		let u2 = runit();
		if (vrand32() % 4 == 0) {
			u2 = m3.b3Neg(u1);
		}
		let smallRot = m3.b3MulSV(vrandf(0.0, 0.01), runit());
		let bigRot = rvecn();
		let dr = m3.b3MulSV(vrandf(0.0, 0.5), rvecn());
		let m = rmat();

		vtag("q1");
		pq(q1);
		pq(q2);
		vbool(m3.b3IsNormalizedQuat(q1));
		vbool(m3.b3IsNormalizedQuat(n1));
		vbool(m3.b3IsValidQuat(q1));
		pv3(m3.b3RotateVector(q1, v));
		pv3(m3.b3InvRotateVector(q1, v));
		vf32(m3.b3DotQuat(q1, q2));
		pq(m3.b3MulQuat(q1, q2));
		pq(m3.b3InvMulQuat(q1, q2));
		pq(m3.b3Conjugate(q1));
		pq(m3.b3NegateQuat(q1));
		pq(m3.b3NormalizeQuat(q1));
		vend();

		vtag("q2");
		pq(m3.b3MakeQuatFromAxisAngle(axis, angle));
		let out = b3Vec2(-7.0, -7.0);
		let ax = m3.b3GetAxisAngle(out.x, n1);
		vf32(out.x);
		pv3(ax);
		vf32(m3.b3GetQuatAngle(n1));
		vf32(m3.b3GetTwistAngle(n1));
		vf32(m3.b3GetSwingAngle(n1));
		pq(m3.b3NLerp(n1, n2, alpha));
		let r = m3.b3MakeMatrixFromQuat(n1);
		pm3(r);
		pq(m3.b3MakeQuatFromMatrix(r));
		pq(m3.b3MakeQuatFromMatrix(m));
		pm3(m3.b3MakeMatrixFromQuat(q1));
		vend();

		vtag("q3");
		pq(m3.b3ComputeQuatBetweenUnitVectors(u1, u2));
		pq(mi.b3QuatFromExponentialMap(smallRot));
		pq(mi.b3QuatFromExponentialMap(bigRot));
		pq(mi.b3IntegrateRotation(n1, dr));
		pq(mi.b3IntegrateRotation(q1, v));
		pv3(mi.b3DeltaQuatToRotation(n1, n2));
		pv3(mi.b3DeltaQuatToRotation(q1, q2));
		vend();
	}
}

function checkTransforms(): void {
	for (let i = 0; i < 3000; ++i) {
		let A = rxf();
		let B = rxf();
		let v = rvec();
		let base = rvecn();
		let T: b3Transform;
		T.p = rvec();
		T.q = rquatAny();

		vtag("x");
		pxf(m3.b3MulTransforms(A, B));
		pxf(m3.b3InvMulTransforms(A, B));
		pxf(m3.b3InvertTransform(A));
		pv3(m3.b3TransformPoint(A, v));
		pv3(m3.b3InvTransformPoint(A, v));
		pv3(m3.b3TransformWorldPoint(A, v));
		pv3(m3.b3InvTransformWorldPoint(A, v));
		pxf(m3.b3MulWorldTransforms(A, B));
		pxf(m3.b3InvMulWorldTransforms(A, B));
		pxf(m3.b3ToRelativeTransform(A, base));
		pxf(m3.b3MakeWorldTransform(T));
		pxf(m3.b3MulTransforms(T, A));
		vbool(m3.b3IsValidTransform(A));
		vbool(m3.b3IsValidTransform(T));
		vbool(m3.b3IsValidWorldTransform(T));
		vend();
	}
}

function checkMatrices(): void {
	for (let i = 0; i < 3000; ++i) {
		let m = rmat();
		let n = rmat();
		let v = rvec();
		let s = rscalar();
		let mass = vrandf(0.1, 10.0);
		let o = rvecn();
		let radius = vrandf(0.01, 5.0);
		let height = vrandf(0.01, 5.0);
		let lo = rvecn();
		let hi = m3.b3Add(lo, m3.b3Abs(rvecn()));
		let q = rquat();
		let xf = rxf();

		vtag("m1");
		pm3(m);
		pm3(n);
		vf32(m3.b3Det(m));
		pv3(m3.b3MulMV(m, v));
		pm3(m3.b3NegateMat3(m));
		pm3(m3.b3AddMM(m, n));
		pm3(m3.b3SubMM(m, n));
		pm3(m3.b3MulSM(s, m));
		pm3(m3.b3MulMM(m, n));
		pm3(m3.b3Transpose(m));
		vend();

		vtag("m2");
		pm3(m3.b3InvertMatrix(m));
		pv3(m3.b3Solve3(m, v));
		pm3(m3.b3InvertT(m));
		pm3(m3.b3AbsMatrix3(m));
		pm3(m3.b3Steiner(mass, o));
		vbool(m3.b3IsValidMatrix3(m));
		pm3(mi.b3MakeDiagonalMatrix(s, mass, radius));
		pm3(m3.b3SphereInertia(mass, radius));
		pm3(m3.b3CylinderInertia(mass, radius, height));
		pm3(m3.b3BoxInertia(mass, lo, hi));
		pm3(mi.b3RotateInertia(q, m));
		pm3(mi.b3TransformInertia(xf, m, mass));
		vend();
	}
}

function checkAABBs(): void {
	let points = new StaticArray<b3Vec3>(8);
	for (let i = 0; i < 4000; ++i) {
		let a = raabb();
		let b = raabb();
		let xf = rxf();
		let p = rvec();
		let ext = rscalar();
		let origin = rvecn();
		let count = vrandint(1, 9);
		for (let j = 0; j < count; ++j) {
			points[j] = rvecn();
		}
		let radius = vrandf(0.0, 1.0);
		let scale = vrandf(1.0, 40000.0);
		let big = b3AABB(m3.b3MulSV(scale, a.lowerBound), m3.b3MulSV(scale, a.upperBound));

		vtag("a1");
		pab(a);
		pab(b);
		vbool(m3.b3AABB_Contains(a, b));
		vf32(m3.b3AABB_Area(a));
		pv3(m3.b3AABB_Center(a));
		pv3(m3.b3AABB_Extents(a));
		pab(m3.b3AABB_Union(a, b));
		pab(m3.b3AABB_Inflate(a, ext));
		vbool(m3.b3AABB_Overlaps(a, b));
		pab(m3.b3AABB_Transform(xf, a));
		pv3(m3.b3ClosestPointToAABB(p, a));
		pab(m3.b3OffsetAABB(a, origin));
		vend();

		vtag("a2");
		vbool(m3.b3IsValidAABB(a));
		vbool(m3.b3IsBoundedAABB(a));
		vbool(m3.b3IsSaneAABB(a));
		vbool(m3.b3IsBoundedAABB(big));
		vbool(m3.b3IsSaneAABB(big));
		pab(mi.b3AABB_AddPoint(a, p));
		vf32(bb.b3Perimeter(a));
		let c = a;
		vbool(bb.b3EnlargeAABB(c, b));
		pab(c);
		pv3(bb.b3FarthestPointOnAABB(a, p));
		pab(m3.b3MakeAABB(points, count, radius));
		vend();

		let box: b3AABB;
		let center = rvecn();
		let extent = m3.b3Add(m3.b3Abs(rvecn()), b3Vec3(0.1, 0.1, 0.1));
		box.lowerBound = m3.b3Sub(center, extent);
		box.upperBound = m3.b3Add(center, extent);
		let p1 = m3.b3MulSV(2.0, rvecn());
		let p2: b3Vec3;
		let kind = vrand32() % 5;
		if (kind == 0) {
			p2 = p1;
		} else if (kind == 1) {
			p2 = m3.b3Add(p1, b3Vec3(0.0, 0.0, vrandf(-30.0, 30.0)));
		} else if (kind == 2) {
			p2 = m3.b3Add(center, m3.b3MulSV(0.5, rvecn()));
			p1 = m3.b3Add(center, m3.b3MulSV(0.1, rvecn()));
		} else if (kind == 3) {
			p2 = m3.b3Add(center, m3.b3MulSV(0.5, rvecn()));
		} else {
			p2 = m3.b3MulSV(2.0, rvecn());
		}

		let fractions = b3Vec2(-7.0, -7.0);
		let hit = bb.b3RayCastAABB(box, p1, p2, fractions.x, fractions.y);
		vtag("a3");
		vbool(hit);
		vf32(fractions.x);
		vf32(fractions.y);
		vend();
	}
}

function checkPlanes(): void {
	for (let i = 0; i < 3000; ++i) {
		let pl = rplane();
		let xf = rxf();
		let p = rvec();
		let p1 = rvecn();
		let p2 = rvecn();
		let p3 = rvecn();
		if (vrand32() % 5 == 0) {
			p3 = m3.b3MulAdd(p1, vrandf(-2.0, 2.0), m3.b3Sub(p2, p1));
		}
		let n = runit();
		let raw: b3Plane;
		raw.normal = rvecn();
		raw.offset = vrandf(-10.0, 10.0);

		vtag("p");
		ppl(pl);
		vbool(m3.b3IsValidPlane(pl));
		ppl(mi.b3NormalizePlane(raw));
		ppl(mi.b3MakePlaneFromNormalAndPoint(n, p));
		ppl(mi.b3MakePlaneFromPoints(p1, p2, p3));
		pv3(mi.b3MakeNormalFromPoints(p1, p2, p3));
		ppl(mi.b3TransformPlane(xf, pl));
		vf32(mi.b3PlaneSeparation(pl, p));
		vf32(mi.b3SignedVolume(p1, p2, p3, p));
		vend();
	}
}

function checkSegments(): void {
	for (let i = 0; i < 4000; ++i) {
		let mode = vrand32() % 8;
		let p1: b3Vec3;
		let q1: b3Vec3;
		let p2: b3Vec3;
		let q2: b3Vec3;
		if (mode == 0) {
			p1 = rvec();
			q1 = rvec();
			p2 = rvec();
			q2 = rvec();
		} else {
			p1 = rvecn();
			q1 = rvecn();
			p2 = rvecn();
			q2 = rvecn();
			if (mode == 1) {
				q1 = p1;
			} else if (mode == 2) {
				q2 = p2;
			} else if (mode == 3) {
				q2 = m3.b3MulAdd(p2, vrandf(-2.0, 2.0), m3.b3Sub(q1, p1));
			} else if (mode == 4) {
				q1 = m3.b3MulAdd(p1, 1.0e-4, rvecn());
				q2 = m3.b3MulAdd(p2, 1.0e-4, rvecn());
			} else if (mode == 5) {
				q1 = p1;
				q2 = p2;
			}
		}

		vtag("g");
		let r = m3.b3SegmentDistance(p1, q1, p2, q2);
		pseg(r);
		vbool(mi.b3IsWithinSegments(r));
		let l = m3.b3LineDistance(p1, m3.b3Sub(q1, p1), p2, m3.b3Sub(q2, p2));
		pseg(l);
		vbool(mi.b3IsWithinSegments(l));
		pv3(m3.b3PointToSegmentDistance(p1, q1, p2));
		vend();
	}
}

function checkTriangles(): void {
	for (let i = 0; i < 4000; ++i) {
		let a = rvecn();
		let b = rvecn();
		let c = rvecn();
		let mode = vrand32() % 6;
		if (mode == 0) {
			c = m3.b3MulAdd(a, vrandf(-2.0, 2.0), m3.b3Sub(b, a));
		}
		let s = vrandf(-1.0, 2.0);
		let t = vrandf(-1.0, 2.0);
		let h = vrandf(-3.0, 3.0);
		let q: b3Vec3;
		if (mode == 1) {
			q = rvec();
		} else {
			let n = m3.b3Normalize(m3.b3Cross(m3.b3Sub(b, a), m3.b3Sub(c, a)));
			q = m3.b3MulAdd(a, s, m3.b3Sub(b, a));
			q = m3.b3MulAdd(q, t, m3.b3Sub(c, a));
			q = m3.b3MulAdd(q, h, n);
		}

		let tp = m3.b3ClosestPointOnTriangle(a, b, c, q);
		vtag("t");
		pv3(q);
		pv3(tp.point);
		vi32(tp.feature);
		vend();
	}
}

function checkRays(): void {
	for (let i = 0; i < 2000; ++i) {
		let input: col.b3RayCastInput;
		input.origin = rvec();
		input.translation = rvec();
		if (vrand32() % 2 == 0) {
			input.maxFraction = vspecialf();
		} else {
			input.maxFraction = vrandf(-0.5, 1.5);
		}

		vtag("r");
		vbool(m3.b3IsValidRay(input));
		vend();
	}
}

function checkHash(): void {
	let buffer = new Uint8Array(512);
	for (let i = 0; i < 400; ++i) {
		let len: i32;
		if (i < 260) {
			len = i;
		} else {
			len = vrandint(0, 512);
		}

		for (let j = 0; j < len; ++j) {
			buffer[j] = <u8>(vrand32() & 0xff);
		}

		vtag("h");
		vi32(len);
		vu32(core.b3Hash(core.B3_HASH_INIT, buffer, len));
		vu64(core.b3Hash64NonZero(buffer, len));
		vend();
	}
}

function checkIds(): void {
	let values = new StaticArray<u32>(3);
	let out = new StaticArray<u32>(3);
	for (let i = 0; i < 2000; ++i) {
		let x32 = vrand32();
		let x64 = vrand64();
		let y64 = vrand64();
		let z64 = vrand64();
		values[0] = vrand32();
		values[1] = vrand32();
		values[2] = vrand32();

		vtag("id");
		let w = ty.b3LoadWorldId(x32);
		vu32(w.index1);
		vu32(w.generation);
		vu32(ty.b3StoreWorldId(w));
		let bo = ty.b3LoadBodyId(x64);
		vi32(bo.index1);
		vu32(bo.world0);
		vu32(bo.generation);
		vu64(ty.b3StoreBodyId(bo));
		let sh = ty.b3LoadShapeId(y64);
		vi32(sh.index1);
		vu32(sh.world0);
		vu32(sh.generation);
		vu64(ty.b3StoreShapeId(sh));
		let jo = ty.b3LoadJointId(z64);
		vi32(jo.index1);
		vu32(jo.world0);
		vu32(jo.generation);
		vu64(ty.b3StoreJointId(jo));
		let co = ty.b3LoadContactId(values);
		vi32(co.index1);
		vu32(co.world0);
		vi32(co.padding);
		vu32(co.generation);
		ty.b3StoreContactId(co, out);
		vu32(out[0]);
		vu32(out[1]);
		vu32(out[2]);
		vbool(bo.index1 == sh.index1 && bo.world0 == sh.world0 && bo.generation == sh.generation);
		let bo2 = bo;
		vbool(bo == bo2);
		vbool(bo.index1 == 0);
		vend();
	}
}

function hullLayout(vertexCount: i32, edgeCount: i32, faceCount: i32): void {
	let hull = new col.b3HullData(vertexCount, edgeCount, faceCount);
	vtag("hull");
	vi32(hull.vertexCount);
	vi32(hull.edgeCount);
	vi32(hull.faceCount);
	vi32(hull.vertexOffset);
	vi32(hull.pointOffset);
	vi32(hull.edgeOffset);
	vi32(hull.planeOffset);
	vi32(hull.faceOffset);
	vi32(hull.soaVertexOffset);
	vi32(hull.soaNormalOffset);
	vi32(hull.edgeCosineOffset);
	vi32(hull.byteCount);
	vi32(col.b3GetHullVertices(hull).length);
	vi32(col.b3GetHullPoints(hull).length);
	vi32(col.b3GetHullEdges(hull).length);
	vi32(col.b3GetHullPlanes(hull).length);
	vi32(col.b3GetHullFaces(hull).length);
	vi32(col.b3GetHullSoaVertices(hull).length);
	vi32(col.b3GetHullSoaNormals(hull).length);
	vi32(col.b3GetHullEdgeCosines(hull).length);
	vend();
}

function meshLayout(nodeCount: i32, vertexCount: i32, triangleCount: i32): void {
	let mesh = new col.b3MeshData(nodeCount, vertexCount, triangleCount);
	vtag("mesh");
	vi32(col.b3GetMeshNodes(mesh).length);
	vi32(col.b3GetMeshVertices(mesh).length);
	vi32(col.b3GetMeshTriangles(mesh).length);
	vi32(mesh.nodeOffset);
	vi32(mesh.vertexOffset);
	vi32(mesh.triangleOffset);
	vi32(mesh.materialOffset);
	vi32(mesh.flagsOffset);
	vi32(mesh.byteCount);
	vend();
}

function heightLayout(columnCount: i32, rowCount: i32): void {
	let hf = new col.b3HeightFieldData(columnCount, rowCount);
	vtag("height");
	vi32(hf.columnCount);
	vi32(hf.rowCount);
	vi32(hf.heightsOffset);
	vi32(hf.materialOffset);
	vi32(hf.flagsOffset);
	vi32(hf.byteCount);
	vi32(col.b3GetHeightFieldCompressedHeights(hf).length);
	vi32(col.b3GetHeightFieldMaterialIndices(hf).length);
	vi32(col.b3GetHeightFieldFlags(hf).length);
	vend();
}

function checkLayout(): void {
	vtag("sizes");
	vi32(col.B3_HULL_DATA_SIZE);
	vi32(col.B3_MESH_DATA_SIZE);
	vi32(col.B3_MESH_NODE_SIZE);
	vi32(col.B3_HEIGHT_FIELD_DATA_SIZE);
	vi32(col.B3_COMPOUND_DATA_SIZE);
	vend();

	let box = new col.b3BoxHull();
	vtag("box");
	vi32(box.base.byteCount);
	vi32(box.base.vertexOffset);
	vi32(box.base.pointOffset);
	vi32(box.base.edgeOffset);
	vi32(box.base.planeOffset);
	vi32(box.base.faceOffset);
	vi32(box.base.soaVertexOffset);
	vi32(box.base.soaNormalOffset);
	vi32(box.base.edgeCosineOffset);
	vi32(box.boxVertices.length);
	vi32(box.boxPoints.length);
	vi32(box.boxEdges.length);
	vi32(box.boxPlanes.length);
	vi32(box.boxFaces.length);
	vi32(box.soaVertices.length);
	vi32(box.soaNormals.length);
	vi32(box.edgeCosines.length);
	vend();

	hullLayout(4, 12, 4);
	hullLayout(8, 24, 6);
	hullLayout(128, 254, 128);
	for (let i = 0; i < 50; ++i) {
		let v = vrandint(4, 129);
		let e = 2 * vrandint(6, 128);
		let f = vrandint(4, 129);
		hullLayout(v, e, f);
	}

	for (let i = 0; i < 50; ++i) {
		let n = vrandint(1, 1000);
		let v = vrandint(3, 1000);
		let t = vrandint(1, 1000);
		meshLayout(n, v, t);
	}

	for (let i = 0; i < 50; ++i) {
		let c = vrandint(2, 100);
		let r = vrandint(2, 100);
		heightLayout(c, r);
	}
}

export function run(): void {
	vseed(0x5eed);
	checkConstants();
	checkUnitConstants(1.0);
	checkDefaults(1.0);
	checkUnitConstants(0.37);
	checkDefaults(0.37);
	checkUnitConstants(100.0);
	k.b3SetLengthUnitsPerMeter(1.0);
	checkScalars();
	checkVectors();
	checkVec2();
	checkQuats();
	checkTransforms();
	checkMatrices();
	checkAABBs();
	checkPlanes();
	checkSegments();
	checkTriangles();
	checkRays();
	checkHash();
	checkIds();
	checkLayout();
}

// Builds every struct and class the foundation declares so other modules can rely on them.
// Returns 0 when the defaults read back as zero and the sized sections have their lengths.
export function instantiate(): i32 {
	let errors = 0;

	let version = b3Version(1, 2, 3);
	let atomicInt = b3AtomicInt();
	let atomicU32 = b3AtomicU32();
	let atomicI64 = b3AtomicI64();
	errors += version.minor == 2 ? 0 : 1;
	errors += atomicInt.value + <i32>atomicU32.value + <i32>atomicI64.value;

	let v2 = b3Vec2(1.0, 2.0);
	let v3 = b3Vec3(1.0, 2.0, 3.0);
	let cs = b3CosSin(1.0, 0.0);
	let q = b3Quat(v3, 1.0);
	let xf = b3Transform(v3, q);
	let pos: m3.b3Pos = v3;
	let wxf: m3.b3WorldTransform = xf;
	let mat = b3Matrix3(v3, v3, v3);
	let box = b3AABB(v3, v3);
	let plane = b3Plane(v3, 1.0);
	let seg = b3SegmentDistanceResult();
	errors += v2.y == 2.0 && cs.cosine == 1.0 && pos.z == 3.0 && wxf.q.s == 1.0 ? 0 : 1;
	errors += mat.cz.x == 1.0 && box.upperBound.y == 2.0 && plane.offset == 1.0 && seg.fraction2 == 0.0 ? 0 : 1;

	let m2 = b3Matrix2(v2, v2);
	let tri = b3Triangle();
	tri.vertices[2] = v3;
	let tp = b3TrianglePoint(v3, col.b3_featureEdge2);
	let se = b3ShapeExtent();
	errors += m2.cy.x == 1.0 && tri.vertices[2].z == 3.0 && tp.feature == col.b3_featureEdge2 && se.minExtent == 0.0 ? 0 : 1;

	let worldId = b3WorldId();
	let bodyId = b3BodyId();
	let shapeId = b3ShapeId();
	let jointId = b3JointId();
	let contactId = b3ContactId();
	errors += <i32>worldId.index1 + bodyId.index1 + shapeId.index1 + jointId.index1 + contactId.index1;
	errors += shapeId == ty.b3_nullShapeId ? 0 : 1;

	let capacity = b3Capacity();
	let worldDef = new ty.b3WorldDef();
	let locks = b3MotionLocks();
	let bodyDef = new ty.b3BodyDef();
	let filter = b3Filter();
	let material = b3SurfaceMaterial();
	let shapeDef = new ty.b3ShapeDef();
	let profile = b3Profile();
	let counters = b3Counters();
	counters.colorCounts[23] = 1;
	counters.manifoldCounts[7] = 1;
	errors += capacity.contactCount + worldDef.internalValue + bodyDef.internalValue + shapeDef.internalValue;
	errors += locks.angularZ || filter.groupIndex != 0 || material.friction != 0.0 || profile.sensors != 0.0 ? 1 : 0;
	errors += counters.colorCounts[23] + counters.manifoldCounts[7] == 2 ? 0 : 1;

	let jointDef = b3JointDef();
	let distanceDef = b3DistanceJointDef();
	let motorDef = b3MotorJointDef();
	let filterDef = b3FilterJointDef();
	let parallelDef = b3ParallelJointDef();
	let prismaticDef = b3PrismaticJointDef();
	let revoluteDef = b3RevoluteJointDef();
	let sphericalDef = b3SphericalJointDef();
	let weldDef = b3WeldJointDef();
	let wheelDef = b3WheelJointDef();
	let explosionDef = b3ExplosionDef();
	errors += jointDef.internalValue + distanceDef.base.internalValue + motorDef.base.internalValue;
	errors += filterDef.base.internalValue + parallelDef.base.internalValue + prismaticDef.base.internalValue;
	errors += revoluteDef.base.internalValue + sphericalDef.base.internalValue + weldDef.base.internalValue;
	errors += wheelDef.base.internalValue + <i32>explosionDef.maskBits;

	let sensorBegin = b3SensorBeginTouchEvent();
	let sensorEnd = b3SensorEndTouchEvent();
	let sensorEvents = new ty.b3SensorEvents();
	sensorEvents.beginEvents.push(sensorBegin);
	sensorEvents.endEvents.push(sensorEnd);
	let contactBegin = b3ContactBeginTouchEvent();
	let contactEnd = b3ContactEndTouchEvent();
	let contactHit = b3ContactHitEvent();
	let contactEvents = new ty.b3ContactEvents();
	contactEvents.beginEvents.push(contactBegin);
	contactEvents.endEvents.push(contactEnd);
	contactEvents.hitEvents.push(contactHit);
	let moveEvent = b3BodyMoveEvent();
	let bodyEvents = new ty.b3BodyEvents();
	bodyEvents.moveEvents.push(moveEvent);
	let jointEvent = b3JointEvent();
	let jointEvents = new ty.b3JointEvents();
	jointEvents.jointEvents.push(jointEvent);
	let contactData = new ty.b3ContactData();
	contactData.manifolds = new StaticArray<col.b3Manifold>(2);
	errors += sensorEvents.beginEvents.length + sensorEvents.endEvents.length == 2 ? 0 : 1;
	errors += contactEvents.hitEvents.length + bodyEvents.moveEvents.length + jointEvents.jointEvents.length == 3 ? 0 : 1;
	errors += contactData.manifolds!.length == 2 ? 0 : 1;

	let queryFilter = b3QueryFilter();
	let rayResult = b3RayResult();
	let bodyCast = b3BodyCastResult();
	let bodyPlane = b3BodyPlaneResult();
	let bodyToi = b3BodyTOIResult();
	errors += <i32>queryFilter.id + rayResult.childIndex + bodyCast.iterations + bodyPlane.result.childIndex;
	errors += bodyToi.fraction != 0.0 ? 1 : 0;

	let rayInput = b3RayCastInput(v3, v3, 1.0);
	let points = new StaticArray<b3Vec3>(2);
	points[1] = v3;
	let proxy = new col.b3ShapeProxy(points, 2, 0.5);
	let castInput = new col.b3ShapeCastInput();
	castInput.proxy = proxy;
	let boxCast = b3BoxCastInput(box, v3, 1.0);
	let castOutput = b3CastOutput();
	let worldCast: col.b3WorldCastOutput = castOutput;
	let cache = b3SimplexCache();
	cache.indexB[3] = 9;
	let pairInput = new col.b3ShapeCastPairInput();
	let distanceInput = new col.b3DistanceInput();
	distanceInput.proxyB = proxy;
	let distanceOutput = b3DistanceOutput();
	let simplexVertex = b3SimplexVertex();
	let simplex = b3Simplex();
	simplex.vertices[3] = simplexVertex;
	let sweep = b3Sweep();
	let toiInput = new col.b3TOIInput();
	let toiOutput = b3TOIOutput();
	errors += rayInput.maxFraction == 1.0 && castInput.proxy.points[1].z == 3.0 && boxCast.maxFraction == 1.0 ? 0 : 1;
	errors += worldCast.childIndex + <i32>cache.indexB[3] - 9 + pairInput.proxyA.count + distanceInput.proxyB.count - 2;
	errors += distanceOutput.simplexCount + simplex.count + toiInput.proxyA.count + toiOutput.state;
	errors += sweep.q2.s != 0.0 ? 1 : 0;

	let treeNode = b3TreeNode();
	let treeProxy = b3TreeProxy();
	let tree = new col.b3DynamicTree();
	let treeStats = b3TreeStats();
	errors += treeNode.height + treeNode.shapeIndex + treeProxy.next + tree.nodes.length + tree.leafCenters.length;
	errors += treeStats.leafVisits;

	let planeResult = b3PlaneResult();
	let collisionPlane = b3CollisionPlane();
	let solverResult = b3PlaneSolverResult();
	let massData = b3MassData();
	let sphere = b3Sphere(v3, 1.0);
	let capsule = b3Capsule(v3, v3, 1.0);
	errors += planeResult.materialIndex + solverResult.iterationCount + (collisionPlane.clipVelocity ? 1 : 0);
	errors += massData.mass == 0.0 && sphere.radius == 1.0 && capsule.center2.y == 2.0 ? 0 : 1;

	let hullVertex = b3HullVertex();
	let halfEdge = b3HullHalfEdge(1, 2, 3, 4);
	let hullFace = b3HullFace();
	let hull = new col.b3HullData(8, 24, 6);
	hull.edges[23] = halfEdge;
	let boxHull = new col.b3BoxHull();
	boxHull.boxEdges[5] = halfEdge;
	errors += <i32>hullVertex.edge + <i32>hullFace.edge + <i32>hull.edges[23].face - 4;
	errors += <i32>boxHull.base.edges[5].twin - 2;
	errors += hull.soaVertices.length == 24 && hull.soaNormals.length == 24 && hull.edgeCosines.length == 12 ? 0 : 1;

	let meshDef = new col.b3MeshDef();
	let meshTriangle = b3MeshTriangle(0, 1, 2);
	let meshNode = b3MeshNode();
	meshNode.data.asLeaf.type = 3;
	meshNode.data.asLeaf.triangleCount = 1000000;
	meshNode.data.asNode.childOffset = 77;
	let meshData = new col.b3MeshData(3, 4, 2);
	meshData.nodes[2] = meshNode;
	let mesh = new col.b3Mesh(meshData, v3);
	errors += meshDef.vertexCount + meshTriangle.index3 - 2 + <i32>meshData.nodes[2].data.asLeaf.type - 3;
	errors += <i32>meshData.nodes[2].data.asLeaf.triangleCount - 1000000 + <i32>meshData.nodes[2].data.asNode.axis;
	errors += <i32>meshData.nodes[2].data.asNode.childOffset - 77 + mesh.data.triangles.length - 2;

	let heightDef = new col.b3HeightFieldDef();
	let heightField = new col.b3HeightFieldData(3, 4);
	heightField.padding[6] = 1;
	errors += heightDef.countX + heightField.compressedHeights.length - 12 + heightField.flags.length - 12;
	errors += <i32>heightField.padding[6] - 1;

	let compoundCapsuleDef = b3CompoundCapsuleDef();
	let compoundHullDef = new col.b3CompoundHullDef(hull, xf, material);
	let compoundMeshDef = new col.b3CompoundMeshDef(meshData);
	let compoundSphereDef = b3CompoundSphereDef();
	let compoundDef = new col.b3CompoundDef();
	compoundDef.hulls = [compoundHullDef];
	compoundDef.meshes = [compoundMeshDef];
	compoundDef.capsules = new StaticArray<b3CompoundCapsuleDef>(1);
	compoundDef.capsules[0] = compoundCapsuleDef;
	compoundDef.spheres = new StaticArray<b3CompoundSphereDef>(1);
	compoundDef.spheres[0] = compoundSphereDef;
	let hullInstance = b3HullInstance();
	let meshInstance = b3MeshInstance();
	let compound = new col.b3CompoundData();
	compound.hulls = [hull];
	compound.meshes = [meshData];
	compound.hullInstances = new StaticArray<b3HullInstance>(1);
	compound.hullInstances[0] = hullInstance;
	compound.meshInstances = new StaticArray<b3MeshInstance>(1);
	compound.meshInstances[0] = meshInstance;
	compound.meshMaterialIndices = new Uint16Array(6);
	compound.meshMaterialIndices[4] = 5;
	let compoundCapsule = b3CompoundCapsule();
	let compoundHull = new col.b3CompoundHull();
	compoundHull.hull = compound.hulls[0];
	let compoundMesh = new col.b3CompoundMesh();
	compoundMesh.materialIndices = compound.meshMaterialIndices.subarray(3, 6);
	let compoundSphere = b3CompoundSphere();
	let child = new col.b3ChildShape();
	child.mesh = mesh;
	child.materialIndices = compoundMesh.materialIndices;
	errors += compoundDef.hulls.length + compoundDef.meshes.length + compoundDef.capsules.length + compoundDef.spheres.length - 4;
	errors += <i32>compoundCapsule.materialIndex + <i32>compoundSphere.padding + compound.hulls[0].edges.length - 24;
	errors += <i32>child.materialIndices![1] - 5 + child.type + child.mesh!.data.vertices.length - 4;
	errors += compoundHull.hull!.faceCount - 6;

	let manifoldPoint = b3ManifoldPoint();
	let manifold = b3Manifold();
	manifold.points[3] = manifoldPoint;
	let satCache = b3SATCache();
	let featurePair = b3FeaturePair();
	let localPoint = b3LocalManifoldPoint();
	let localManifold = new col.b3LocalManifold();
	localManifold.points = new StaticArray<col.b3LocalManifoldPoint>(8);
	localManifold.pointOffset = 4;
	localManifold.points[localManifold.pointOffset + 3] = localPoint;
	let point2D = b3Point2D();
	errors += manifold.pointCount + <i32>satCache.hit + <i32>featurePair.index2 + localManifold.points.length - 8;
	errors += point2D.originalIndex + <i32>manifold.points[3].persisted;

	let treeQuery: col.b3TreeQueryCallbackFcn = (proxyId: i32, userData: u64): bool => proxyId == <i32>userData;
	let meshQuery: col.b3MeshQueryFcn = (a: b3Vec3, b: b3Vec3, c: b3Vec3, triangleIndex: i32): bool =>
		a.x + b.y + c.z == <f32>triangleIndex;
	let castResult: ty.b3CastResultFcn = (
		shapeId: ty.b3ShapeId,
		point: m3.b3Pos,
		normal: b3Vec3,
		fraction: f32,
		userMaterialId: u64,
		triangleIndex: i32,
		childIndex: i32,
	): f32 => fraction;
	errors += treeQuery(3, 3) ? 0 : 1;
	errors += meshQuery(v3, v3, v3, 6) ? 0 : 1;
	errors += castResult(shapeId, v3, v3, 0.5, 0, 0, 0) == 0.5 ? 0 : 1;

	return errors;
}
