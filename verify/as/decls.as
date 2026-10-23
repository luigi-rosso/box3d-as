// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Compile only check of the dynamics declarations: builds every struct and class and touches the
// access patterns the porters rely on. instantiate returns the number of failed expectations.

import { B3_GRAPH_COLOR_COUNT } from "../../lib/constants";
import { B3_NULL_INDEX, b3AtomicInt } from "../../lib/core";
import { b3Quat, b3Transform, b3Vec3 } from "../../lib/math_functions";
import {
	b3CompoundData,
	b3FeaturePair,
	b3HeightFieldData,
	b3HullData,
	b3Manifold,
	b3Mesh,
	b3MeshData,
	b3Sweep,
} from "../../lib/collision";
import { b3Filter, b3QueryFilter, b3SurfaceMaterial, b3_dynamicBody, b3_hullShape, b3_meshShape } from "../../lib/types";

import {
	b3AllocateManifolds,
	b3FreeManifolds,
	b3TaskContext,
	b3World,
	b3_awakeSet,
	b3_staticSet,
} from "../../lib/physics_world";
import {
	b3Body,
	b3BodySim,
	b3BodyState,
	b3DecodeAwakeIndex,
	b3EncodeBodySimIndex,
	b3IsStaticSimIndex,
	b3MakeRelativeSweep,
	b3SleepBodySimIndex,
	b3_allLocks,
	b3_identityBodyState,
} from "../../lib/body";
import {
	b3CopyShape,
	b3GetHeightFieldTriangleCount,
	b3GetMeshTriangle,
	b3GetShapeMaterials,
	b3IsConvex,
	b3Shape,
	b3ShouldQueryCollide,
	b3ShouldShapesCollide,
	b3_enableSpeculative,
} from "../../lib/shape";
import {
	b3Contact,
	b3ContactCache,
	b3ContactEdge,
	b3ContactSpec,
	b3ConvexContact,
	b3MeshContact,
	b3TriangleCache,
	b3_emptyManifolds,
	b3_simMeshContact,
} from "../../lib/contact";
import {
	b3ContactPrepareSpan,
	b3JointPrepareSpan,
	b3MakeSoft,
	b3MeshPrepareSpan,
	b3Softness,
	b3SolverBlock,
	b3SolverStage,
	b3StepContext,
	b3SyncBlock,
	b3WidePrepareSpan,
	b3_stageStoreImpulses,
} from "../../lib/solver";
import { b3SolverSet } from "../../lib/solver_set";
import { b3ContactLink, b3Island, b3IslandSim, b3JointLink } from "../../lib/island";
import { B3_DYNAMIC_COLOR_COUNT, B3_OVERFLOW_INDEX, b3ConstraintGraph, b3GraphColor } from "../../lib/constraint_graph";
import {
	b3DistanceJoint,
	b3Joint,
	b3JointEdge,
	b3JointSim,
	b3MotorJoint,
	b3ParallelJoint,
	b3PrismaticJoint,
	b3RevoluteJoint,
	b3SolvePointConstraint,
	b3SphericalJoint,
	b3WeldJoint,
	b3WheelJoint,
} from "../../lib/joint";
import { b3Sensor, b3SensorHit, b3SensorTaskContext, b3Visitor } from "../../lib/sensor";
import { B3_PROXY_ID, B3_PROXY_KEY, B3_PROXY_TYPE, b3BroadPhase } from "../../lib/broad_phase";
import {
	b3BodyStateW,
	b3ContactConstraint,
	b3ContactConstraintPointWide,
	b3ContactConstraintWide,
	b3ContactManifoldWide,
	b3ManifoldConstraint,
	b3ManifoldConstraintPoint,
	b3Matrix3W,
	b3QuatW,
	b3SymMatrix2W,
	b3SymMatrix3W,
	b3Vec2W,
} from "../../lib/contact_solver";
import {
	B3_MAX_CLIP_POINTS,
	b3AxisQuery,
	b3ClipVertex,
	b3FeaturePair_single,
	b3GetBestAxis,
	b3MakeFeatureId,
	b3SeparatingAxis,
	b3_featureShapeB,
} from "../../lib/manifold";
import { b3GetCompoundTreeView } from "../../lib/compound";
import { b3ParallelForCallback } from "../../lib/parallel_for";

export { frameBoundary } from "./common";

function bumpRevolute(joint: ref b3RevoluteJoint): void {
	joint.lowerImpulse += 1.0;
}

function bumpConstraint(c: ref b3ContactConstraintWide): void {
	c.manifoldCount += 1;
}

export function instantiate(): i32 {
	let errors = 0;

	// world and its arrays
	let world = new b3World();
	errors += world.constraintGraph.colors.length == B3_GRAPH_COLOR_COUNT ? 0 : 1;
	errors += world.broadPhase.trees.length == 3 ? 0 : 1;
	errors += world.sensorEndEvents.length == 2 && world.contactEndEvents[1].length == 0 ? 0 : 1;
	errors += world.frictionCallback == null && world.preSolveFcn == null ? 0 : 1;
	world.manifoldCounts[7] = 2;
	errors += world.manifoldCounts[7] == 2 ? 0 : 1;

	let task = new b3TaskContext();
	task.manifoldCounts[3] = 1;
	task.pairKeys.push(5);
	task.sensorHits.push(b3SensorHit(1, 2));
	world.taskContexts.push(task);
	world.sensorTaskContexts.push(new b3SensorTaskContext());
	errors += world.taskContexts[0].sensorHits[0].visitorId == 2 ? 0 : 1;

	// bodies are structs written through the array place
	world.bodies.push(b3Body());
	world.bodies[0].setIndex = b3_awakeSet;
	world.bodies[0].localIndex = 4;
	world.bodies[0].flags = b3_allLocks;
	errors += b3EncodeBodySimIndex(world.bodies[0]) == 4 ? 0 : 1;
	world.bodies[0].setIndex = b3_staticSet;
	let encoded = b3EncodeBodySimIndex(world.bodies[0]);
	errors += encoded == -6 && b3IsStaticSimIndex(encoded) ? 0 : 1;
	errors += b3DecodeAwakeIndex(encoded) == B3_NULL_INDEX && b3SleepBodySimIndex(3) == B3_NULL_INDEX ? 0 : 1;

	let state = b3BodyState();
	errors += state.flags;
	errors += b3_identityBodyState.deltaRotation.s == 1.0 && b3_identityBodyState.linearVelocity.x == 0.0 ? 0 : 1;

	let set = new b3SolverSet();
	set.bodySims.push(b3BodySim());
	set.bodyStates.push(b3_identityBodyState);
	set.bodySims[0].center0 = b3Vec3(1.0, 2.0, 3.0);
	set.bodySims[0].transform = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(0.0, 0.0, 0.0), 1.0));
	set.islandSims.push(b3IslandSim());
	set.contactIndices.push(7);
	world.solverSets.push(set);
	let sweep: b3Sweep = b3MakeRelativeSweep(set.bodySims[0], b3Vec3(1.0, 1.0, 1.0));
	errors += sweep.c1.z == 2.0 && sweep.q2.s == 1.0 ? 0 : 1;

	// shapes
	let shape = new b3Shape();
	shape.materials = new StaticArray<b3SurfaceMaterial>(1);
	shape.materialCount = 1;
	b3GetShapeMaterials(shape)[0].friction = 0.5;
	shape.flags |= b3_enableSpeculative;
	shape.filter = b3Filter(1, 2, 0);
	world.shapes.push(shape);
	world.fatAABBs.push(shape.aabb);
	let copy = b3CopyShape(shape);
	copy.type = b3_meshShape;
	errors += shape.materials[0].friction == 0.5 && copy.materials[0].friction == 0.5 && shape.type == 0 ? 0 : 1;
	errors += shape.hull == null && shape.mesh == null && shape.compound == null && shape.heightField == null ? 0 : 1;
	errors += b3ShouldShapesCollide(b3Filter(1, 2, 0), b3Filter(2, 1, 0)) ? 0 : 1;
	errors += b3ShouldQueryCollide(shape.filter, b3QueryFilter(2, 1, 0)) ? 0 : 1;
	errors += b3IsConvex(b3_hullShape) && !b3IsConvex(b3_meshShape) ? 0 : 1;

	let meshData = new b3MeshData(1, 3, 1);
	meshData.triangleCount = 1;
	let mesh = new b3Mesh(meshData, b3Vec3(1.0, 1.0, -1.0));
	let triangle = b3GetMeshTriangle(mesh, 0);
	errors += triangle.flags == 0 ? 0 : 1;

	// contacts are classes in the world array
	let contact = new b3Contact();
	contact.edges[1].bodyId = 3;
	contact.flags |= b3_simMeshContact;
	contact.manifolds = b3AllocateManifolds(world, 2);
	contact.manifoldCount = 2;
	contact.manifolds[1].pointCount = 2;
	contact.meshContact.triangleCache.push(b3TriangleCache(4, b3ContactCache()));
	contact.convexContact.cache.simplexCache.count = 1;
	world.contacts.push(contact);
	errors += world.contacts[0].manifolds[1].pointCount == 2 && b3AllocateManifolds(world, 0) == b3_emptyManifolds ? 0 : 1;
	b3FreeManifolds(world, contact.manifolds, contact.manifoldCount);
	let edge = b3ContactEdge();
	let convex = b3ConvexContact();
	let meshContact = new b3MeshContact();
	let spec = b3ContactSpec();
	errors += edge.bodyId + convex.cache.satCache.type + meshContact.triangleCache.length + spec.manifoldCount;

	// islands
	let island = new b3Island();
	island.bodies.push(0);
	island.contacts.push(b3ContactLink());
	island.joints.push(b3JointLink());
	world.islands.push(island);

	// joints: b3Joint is a struct, b3JointSim a class whose union members bind by ref
	world.joints.push(b3Joint());
	world.joints[0].edges[0].bodyId = 1;
	let jointEdge = b3JointEdge();
	let jointSim = new b3JointSim();
	ref revolute = jointSim.revoluteJoint;
	revolute.enableLimit = true;
	bumpRevolute(jointSim.revoluteJoint);
	errors += jointSim.revoluteJoint.enableLimit && jointSim.revoluteJoint.lowerImpulse == 1.0 ? 0 : 1;
	let distance = b3DistanceJoint();
	let motor = b3MotorJoint();
	let parallel = b3ParallelJoint();
	let prismatic = b3PrismaticJoint();
	let spherical = b3SphericalJoint();
	let weld = b3WeldJoint();
	let wheel = b3WheelJoint();
	errors += jointEdge.bodyId + distance.indexA + motor.indexA + parallel.indexA + prismatic.indexA;
	errors += spherical.indexA + weld.indexA + wheel.indexA;

	let point = b3SolvePointConstraint(
		1.0,
		1.0,
		jointSim.invIA,
		jointSim.invIB,
		b3Vec3(0.0, 0.0, 0.0),
		b3Vec3(0.0, 0.0, 0.0),
		b3Vec3(2.0, 0.0, 0.0)
	);
	errors += point.x == 1.0 ? 0 : 1;

	// graph
	let graph = new b3ConstraintGraph();
	let color: b3GraphColor = graph.colors[B3_OVERFLOW_INDEX];
	color.jointSims.push(jointSim);
	color.convexContacts.push(0);
	color.contacts.push(spec);
	color.contactConstraints = new StaticArray<b3ContactConstraint>(1);
	color.manifoldConstraints = new StaticArray<b3ManifoldConstraint>(2);
	ref cc = color.contactConstraints[0];
	cc.constraints = 1;
	cc.contact = 0;
	ref mc = color.manifoldConstraints[cc.constraints];
	mc.points[3].normalImpulse = 1.0;
	errors += B3_DYNAMIC_COLOR_COUNT == 20 && color.manifoldConstraints[1].points[3].normalImpulse == 1.0 ? 0 : 1;
	let mcp = b3ManifoldConstraintPoint();
	errors += <i32>mcp.leverArm;

	// solver
	let context = new b3StepContext(world);
	errors += context.graph == world.constraintGraph ? 0 : 1;
	context.states = set.bodyStates;
	context.sims = set.bodySims;
	context.contactSoftness = b3MakeSoft(30.0, 10.0, 1.0 / 240.0);
	let soft = b3Softness(1.0, 0.5, 0.5);
	errors += context.contactSoftness.massScale + context.contactSoftness.impulseScale > 0.0 && soft.biasRate == 1.0 ? 0 : 1;

	context.wideConstraints = new StaticArray<b3ContactConstraintWide>(2);
	context.wideManifolds = new StaticArray<b3ContactManifoldWide>(2);
	ref wc = context.wideConstraints[1];
	wc.contacts[0] = 0;
	wc.contacts[1] = B3_NULL_INDEX;
	wc.indexA[2] = 1;
	bumpConstraint(context.wideConstraints[1]);
	ref wm = context.wideManifolds[1];
	wm.points[2].normalImpulses.z = 3.0;
	errors += context.wideConstraints[1].manifoldCount == 1 && context.wideManifolds[1].points[2].normalImpulses.z == 3.0 ? 0 : 1;
	color.wideConstraints = context.wideConstraints;
	color.wideManifolds = context.wideManifolds;
	color.wideConstraintStart = 1;

	context.wideMeshManifoldStarts = new Int32Array(3);
	color.wideMeshManifoldStarts = context.wideMeshManifoldStarts.subarray(1);
	color.wideMeshManifoldStarts[0] = 2;
	errors += context.wideMeshManifoldStarts[1] == 2 ? 0 : 1;

	let widePrepareSpans = new StaticArray<b3WidePrepareSpan>(2);
	widePrepareSpans[0] = new b3WidePrepareSpan(0, 1, color.convexContacts);
	widePrepareSpans[1] = new b3WidePrepareSpan(2, 0, color.convexContacts);
	context.widePrepareSpans = widePrepareSpans;
	let overflowSpans = new StaticArray<b3ContactPrepareSpan>(1);
	overflowSpans[0] = new b3ContactPrepareSpan(0, 1, color.contacts);
	context.overflowSpans = overflowSpans;
	let meshSpans = new StaticArray<b3MeshPrepareSpan>(1);
	meshSpans[0] = new b3MeshPrepareSpan(0, 1, color.contacts, new Int32Array(1));
	context.meshPrepareSpans = meshSpans;
	let jointSpans = new StaticArray<b3JointPrepareSpan>(1);
	jointSpans[0] = new b3JointPrepareSpan(0, 1, color.jointSims);
	context.jointPrepareSpans = jointSpans;
	errors += context.jointPrepareSpans[0].joints[0] == jointSim ? 0 : 1;

	let stage = new b3SolverStage();
	stage.blocks = new StaticArray<b3SyncBlock>(1);
	stage.blocks[0].block.count = 4;
	stage.blocks[0].syncIndex.value = 1;
	stage.type = b3_stageStoreImpulses;
	let stages = new StaticArray<b3SolverStage>(1);
	stages[0] = stage;
	context.stages = stages;
	let block = b3SolverBlock();
	let sync = b3SyncBlock();
	let atomic = b3AtomicInt();
	errors += block.startIndex + sync.syncIndex.value + atomic.value;
	errors += context.stages[0].blocks[0].block.count == 4 ? 0 : 1;

	let pointW = b3ContactConstraintPointWide();
	let vec2W = b3Vec2W();
	let quatW = b3QuatW();
	let sym2W = b3SymMatrix2W();
	let sym3W = b3SymMatrix3W();
	let mat3W = b3Matrix3W();
	let stateW = b3BodyStateW();
	errors += <i32>(pointW.leverArms.x + vec2W.y.w + quatW.S.x + sym2W.cyy.x + sym3W.czz.x + mat3W.cz.Z.w + stateW.dq.S.y);

	// sensors
	let sensor = new b3Sensor();
	sensor.overlaps2.push(b3Visitor(1, 2));
	world.sensors.push(sensor);
	errors += world.sensors[0].overlaps2[0].generation == 2 ? 0 : 1;

	// broad-phase
	let bp = new b3BroadPhase();
	let key = B3_PROXY_KEY(9, b3_dynamicBody);
	errors += B3_PROXY_ID(key) == 9 && B3_PROXY_TYPE(key) == b3_dynamicBody && bp.movedSiblings.length == 0 ? 0 : 1;

	// manifold
	let query = b3AxisQuery();
	query.faceA.separation = -1.0;
	query.faceB.separation = -2.0;
	query.edge.separation = 0.5;
	let axis: b3SeparatingAxis = b3GetBestAxis(query);
	let clip = b3ClipVertex();
	let pair = b3FeaturePair();
	pair.owner1 = <u8>b3_featureShapeB;
	errors += axis.separation == 0.5 && b3MakeFeatureId(pair) == 0x01000000 && B3_MAX_CLIP_POINTS == 64 ? 0 : 1;
	errors += <i32>b3MakeFeatureId(b3FeaturePair_single) + clip.pair.index2;

	// compound and parallel for
	let compound = new b3CompoundData();
	let view = b3GetCompoundTreeView(compound);
	errors += view.proxies == null && view.nodes.length == 0 ? 0 : 1;
	let callback: b3ParallelForCallback = (startIndex: i32, endIndex: i32, workerIndex: i32): void => {};
	callback(0, 1, 0);

	let manifold = b3Manifold();
	errors += manifold.pointCount + b3_emptyManifolds.length;
	errors += b3GetHeightFieldTriangleCount(new b3HeightFieldData(2, 2)) == 2 ? 0 : 1;

	return errors;
}
