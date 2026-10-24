// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/body.c

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX, B3_SECRET_COOKIE, FLT_MAX } from "./core";
import { B3_HUGE, B3_SPECULATIVE_DISTANCE } from "./constants";
import {
	b3AABB,
	b3AABB_Contains,
	b3AABB_Inflate,
	b3AABB_Union,
	b3Abs,
	b3Add,
	b3AddMM,
	b3ClampInt,
	b3Conjugate,
	b3Cross,
	b3Det,
	b3DotQuat,
	b3InvRotateVector,
	b3InvTransformWorldPoint,
	b3InvertT,
	b3IsValidFloat,
	b3IsValidMatrix3,
	b3IsValidPosition,
	b3IsValidQuat,
	b3IsValidVec3,
	b3IsValidWorldTransform,
	b3Length,
	b3LengthSquared,
	b3MakeMatrixFromQuat,
	b3Mat3_zero,
	b3Matrix3,
	b3Max,
	b3MinFloat,
	b3Mul,
	b3MulAdd,
	b3MulMM,
	b3MulMV,
	b3MulQuat,
	b3MulSV,
	b3NegateQuat,
	b3Normalize,
	b3OffsetPos,
	b3Pos,
	b3Pos_zero,
	b3Quat,
	b3Quat_identity,
	b3RotateVector,
	b3Steiner,
	b3Sub,
	b3SubPos,
	b3ToRelativeTransform,
	b3ToVec3,
	b3Transform,
	b3TransformPoint,
	b3TransformWorldPoint,
	b3Transpose,
	b3Vec3,
	b3Vec3_zero,
	b3WorldTransform,
} from "./math_functions";
import {
	b3Capsule,
	b3DistanceInput,
	b3MassData,
	b3PlaneResult,
	b3RayCastInput,
	b3ShapeCastInput,
	b3ShapeProxy,
	b3SimplexCache,
	b3Sweep,
	b3TOIInput,
} from "./collision";
import { b3ShapeDistance, b3TimeOfImpact } from "./distance";
import {
	b3BodyCastResult,
	b3BodyDef,
	b3BodyId,
	b3BodyPlaneResult,
	b3BodyTOIResult,
	b3BodyType,
	b3ContactData,
	b3ContactId,
	b3JointId,
	b3MotionLocks,
	b3QueryFilter,
	b3ShapeId,
	b3WorldId,
	b3_capsuleShape,
	b3_compoundShape,
	b3_dynamicBody,
	b3_heightShape,
	b3_hullShape,
	b3_kinematicBody,
	b3_nullBodyId,
	b3_sphereShape,
	b3_staticBody,
} from "./types";
import {
	b3Body_IsValid,
	b3GetUnlockedWorld,
	b3GetUnlockedWorldFromId,
	b3GetWorld,
	b3World,
	b3_awakeSet,
	b3_disabledSet,
	b3_firstSleepingSet,
	b3_staticSet,
} from "./physics_world";
import { b3AddName, b3FindNameWithDefault } from "./name_cache";
import { b3BroadPhase_MoveProxy } from "./broad_phase";
import { b3DestroyContact, b3_contactTouchingFlag, b3_relativeTransformValid } from "./contact";
import { b3CreateIsland, b3DestroyIsland, b3LinkJoint, b3SplitIsland, b3UnlinkJoint } from "./island";
import { b3DestroyJointInternal } from "./joint";
import { b3DestroySensor } from "./sensor";
import {
	b3CollideMover,
	b3ComputeFatShapeAABB,
	b3ComputeShapeExtent,
	b3ComputeShapeMass,
	b3CreateShapeProxy,
	b3DestroyShapeAllocations,
	b3DestroyShapeProxy,
	b3GetShapeMaterials,
	b3MakeShapeProxy,
	b3OverlapShape,
	b3RayCastShape,
	b3ShapeCastShape,
	b3ShouldQueryCollide,
	b3_enableHitEvents,
} from "./shape";
import {
	b3DestroySolverSet,
	b3SolverSet,
	b3TransferBody,
	b3TransferJoint,
	b3TrySleepIsland,
	b3WakeSolverSet,
} from "./solver_set";
import { b3AllocId, b3FreeId } from "./id_pool";
import { b3Array_RemoveSwap } from "./container";

export type b3BodyFlags = u32;

// This body has fixed translation along the x-axis
export const b3_lockLinearX: b3BodyFlags = 0x00000001;

// This body has fixed translation along the y-axis
export const b3_lockLinearY: b3BodyFlags = 0x00000002;

// This body has fixed translation along the z-axis
export const b3_lockLinearZ: b3BodyFlags = 0x00000004;

// This body has fixed rotation around the x-axis
export const b3_lockAngularX: b3BodyFlags = 0x00000008;

// This body has fixed rotation around the y-axis
export const b3_lockAngularY: b3BodyFlags = 0x00000010;

// This body has fixed rotation around the z-axis
export const b3_lockAngularZ: b3BodyFlags = 0x00000020;

// This flag is used for debug draw and contact recycling. Only lives on b3BodySim.
export const b3_isFast: b3BodyFlags = 0x00000040;

// This dynamic body does a final CCD pass against all body types, but not other bullets
export const b3_isBullet: b3BodyFlags = 0x00000080;

// This body was speed capped in the current time step
export const b3_isSpeedCapped: b3BodyFlags = 0x00000100;

// This body had a time of impact event in the current time step (body sim only)
export const b3_hadTimeOfImpact: b3BodyFlags = 0x00000200;

// This body has no limit on angular velocity
export const b3_allowFastRotation: b3BodyFlags = 0x00000400;

// This bullet body needs to have its AABB increased. Needed because bullets don't follow
// the standard broad-phase update.
export const b3_enlargeBulletBounds: b3BodyFlags = 0x00000800;

// This body is dynamic so the solver should write to it.
// This prevents writing to kinematic bodies that causes a multithreaded sharing
// cache coherence problem even when the values are not changing.
// Used for b3BodyState flags.
export const b3_dynamicFlag: b3BodyFlags = 0x00001000;

export const b3_enableSleep: b3BodyFlags = 0x00002000;

export const b3_enableContactRecycling: b3BodyFlags = 0x00004000;

// The user deferred mass computation via the updateBodyMass shape option and mass
// data still hasn't been set.
export const b3_dirtyMass: b3BodyFlags = 0x00008000;

export const b3_enableGyroscopicTorque: b3BodyFlags = 0x00010000;

// All lock flags
export const b3_allLocks: b3BodyFlags =
	b3_lockLinearX | b3_lockLinearY | b3_lockLinearZ | b3_lockAngularX | b3_lockAngularY | b3_lockAngularZ;

// If all these flags are set then the body has fixed rotation
export const b3_fixedRotation: b3BodyFlags = b3_lockAngularX | b3_lockAngularY | b3_lockAngularZ;

// These flags are transient per time step. These may be different across b3Body, b3BodySim, and b3BodyState.
export const b3_bodyTransientFlags: b3BodyFlags = b3_isSpeedCapped | b3_hadTimeOfImpact;

// Body organizational details that are not used in the solver.
export struct b3Body {
	userData: u64 = 0;

	// index of solver set stored in b3World
	// may be B3_NULL_INDEX
	setIndex: i32 = 0;

	// body sim and state index within set
	// may be B3_NULL_INDEX
	localIndex: i32 = 0;

	// [31 : contactId | 1 : edgeIndex]
	headContactKey: i32 = 0;
	contactCount: i32 = 0;

	// todo maybe move this to the body sim
	headShapeId: i32 = 0;
	shapeCount: i32 = 0;

	headChainId: i32 = 0;

	// [31 : jointId | 1 : edgeIndex]
	headJointKey: i32 = 0;
	jointCount: i32 = 0;

	// All enabled dynamic and kinematic bodies are in an island.
	islandId: i32 = 0;

	// Index into the island's bodies array for O(1) swap-removal.
	// B3_NULL_INDEX when not in an island.
	islandIndex: i32 = 0;

	sleepThreshold: f32 = 0;
	sleepTime: f32 = 0;
	sleepVelocity: f32 = 0;
	safetyFactor: f32 = 0;
	mass: f32 = 0;

	// this is used to adjust the fellAsleep flag in the body move array
	bodyMoveIndex: i32 = 0;

	id: i32 = 0;

	// b3BodyFlags
	flags: u32 = 0;
	nameId: u32 = 0;

	type: b3BodyType = 0;

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b3BodyId
	generation: u16 = 0;
}

// Body State
// The body state is designed for fast conversion to and from SIMD via scatter-gather.
// Only awake dynamic and kinematic bodies have a body state.
// This is used in the performance critical constraint solver
//
// The solver operates on the body state. The body state array does not hold static bodies. Static bodies are shared
// across worker threads. It would be okay to read their states, but writing to them would cause cache thrashing across
// workers, even if the values don't change.
// This causes some trouble when computing anchors. I rotate joint anchors using the body rotation every sub-step. For static
// bodies the anchor doesn't rotate. Body A or B could be static and this can lead to lots of branching. This branching
// should be minimized.
//
// Solution 1:
// Use delta rotations. This means anchors need to be prepared in world space. The delta rotation for static bodies will be
// identity using a dummy state. Base separation and angles need to be computed. Manifolds will be behind a frame, but that
// is probably best if bodies move fast.
//
// Solution 2:
// Use full rotation. The anchors for static bodies will be in world space while the anchors for dynamic bodies will be in local
// space. Potentially confusing and bug prone.
//
// Note:
// I rotate joint anchors each sub-step but not contact anchors. Joint stability improves a lot by rotating joint anchors
// according to substep progress. Contacts have reduced stability when anchors are rotated during substeps, especially for
// round shapes.
//
// 64 bytes
export struct b3BodyState {
	linearVelocity: b3Vec3 = b3Vec3(); // 12
	padding1: f32 = 0;

	angularVelocity: b3Vec3 = b3Vec3(); // 12
	padding2: f32 = 0;

	// Using delta position reduces round-off error far from the origin
	deltaPosition: b3Vec3 = b3Vec3(); // 12

	// b3BodyFlags
	// Important flags: locking, dynamic
	flags: u32 = 0; // 4

	// Using delta rotation because I cannot access the full rotation on static bodies in
	// the solver and must use zero delta rotation for static bodies (c,s) = (1,0)
	deltaRotation: b3Quat = b3Quat(); // 16
}

function b3MakeIdentityBodyState(): b3BodyState {
	let state = b3BodyState();
	state.deltaRotation.s = 1.0;
	return state;
}

// Identity body state, notice the deltaRotation is identity
export const b3_identityBodyState: b3BodyState = b3MakeIdentityBodyState();

// The element data of an Array of structs, for the solver's pointer walks; rasc refuses dataStart on value class
// arrays. The address holds until the array grows, which no solver stage does.
@inline export unsafe function b3ArrayData<A>(a: A): usize {
	return load<usize>(changetype<usize>(a) + offsetof<Array<i32>>("dataStart"));
}

function b3MakeIdentityStates(): StaticArray<b3BodyState> {
	let states = new StaticArray<b3BodyState>(1);
	states[0] = b3_identityBodyState;
	return states;
}

// b3_identityBodyState in memory, where C points static lanes and joint ends
export const b3_identityStates = b3MakeIdentityStates();

// C's state pointer: the awake state at index, or the identity state for B3_NULL_INDEX. Index comes from a
// constraint the solver prepared, so it is below the awake count.
@inline export unsafe function b3StateAddress(states: Array<b3BodyState>, index: i32): usize {
	return index == B3_NULL_INDEX
		? changetype<usize>(b3_identityStates)
		: b3ArrayData(states) + <usize>index * sizeof<b3BodyState>();
}

@inline export unsafe function b3StoreStateVelocities(states: Array<b3BodyState>, index: i32, v: b3Vec3, w: b3Vec3): void {
	let state = b3ArrayData(states) + <usize>index * sizeof<b3BodyState>();
	store<b3Vec3>(state + offsetof<b3BodyState>("linearVelocity"), v);
	store<b3Vec3>(state + offsetof<b3BodyState>("angularVelocity"), w);
}

@inline export unsafe function b3StoreStateAngularVelocity(states: Array<b3BodyState>, index: i32, w: b3Vec3): void {
	store<b3Vec3>(b3ArrayData(states) + <usize>index * sizeof<b3BodyState>() + offsetof<b3BodyState>("angularVelocity"), w);
}

// Body simulation data used for integration of position and velocity
// Transform data used for collision and solver preparation.
export struct b3BodySim {
	// transform for body origin, double translation in large world mode
	transform: b3WorldTransform = b3Transform();

	// center of mass position in world space
	center: b3Pos = b3Vec3();

	// b3BodyFlags
	flags: u32 = 0;

	minExtent: f32 = 0;
	maxExtent: b3Vec3 = b3Vec3();

	// previous rotation and COM for TOI
	rotation0: b3Quat = b3Quat();
	center0: b3Pos = b3Vec3();

	// location of center of mass relative to the body origin
	localCenter: b3Vec3 = b3Vec3();

	force: b3Vec3 = b3Vec3();
	torque: b3Vec3 = b3Vec3();

	invMass: f32 = 0;

	// Local space inertia
	inertia: b3Matrix3 = b3Matrix3();

	// Rotational inertia about the center of mass. The world space inverse inertia tensor
	// must be updated whenever the body rotation is modified.
	invInertiaLocal: b3Matrix3 = b3Matrix3();
	invInertiaWorld: b3Matrix3 = b3Matrix3();

	linearDamping: f32 = 0;
	angularDamping: f32 = 0;
	gravityScale: f32 = 0;

	// Index of b3Body
	bodyId: i32 = 0;
}

// Encode the body sim index for storage in the contact.
export function b3EncodeBodySimIndex(body: b3Body): i32 {
	if (body.setIndex == b3_awakeSet) {
		return body.localIndex;
	}

	if (body.setIndex == b3_staticSet) {
		return -(body.localIndex + 2);
	}

	return B3_NULL_INDEX;
}

@inline export function b3IsStaticSimIndex(encodedBodySimIndex: i32): bool {
	return encodedBodySimIndex < B3_NULL_INDEX;
}

@inline export function b3DecodeAwakeIndex(encodedBodySimIndex: i32): i32 {
	return encodedBodySimIndex >= 0 ? encodedBodySimIndex : B3_NULL_INDEX;
}

@inline export function b3SleepBodySimIndex(encodedBodySimIndex: i32): i32 {
	return encodedBodySimIndex >= 0 ? B3_NULL_INDEX : encodedBodySimIndex;
}

// Make a sweep relative to a base position to keep TOI in float precision far from the origin.
export function b3MakeRelativeSweep(bodySim: b3BodySim, base: b3Pos): b3Sweep {
	let s: b3Sweep;
	s.c1 = b3SubPos(bodySim.center0, base);
	s.c2 = b3SubPos(bodySim.center, base);
	s.q1 = bodySim.rotation0;
	s.q2 = bodySim.transform.q;
	s.localCenter = bodySim.localCenter;
	return s;
}

// functions from body.c follow

// Get a validated body from a world using an id.
export function b3GetBodyFullId(world: b3World, bodyId: b3BodyId): i32 {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3Body_IsValid(bodyId));

	// id index starts at one so that zero can represent null
	let index = bodyId.index1 - 1;
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index && index < world.bodies.length);
	return index;
}

export function b3GetBodyTransformQuick(world: b3World, bodyId: i32): b3WorldTransform {
	let set = world.solverSets[world.bodies[bodyId].setIndex];
	return set.bodySims[world.bodies[bodyId].localIndex].transform;
}

export function b3GetBodyTransform(world: b3World, bodyId: i32): b3WorldTransform {
	return b3GetBodyTransformQuick(world, bodyId);
}

// Create a b3BodyId from a raw id.
export function b3MakeBodyId(world: b3World, bodyId: i32): b3BodyId {
	let id = b3BodyId();
	id.index1 = bodyId + 1;
	id.world0 = world.worldId;
	id.generation = world.bodies[bodyId].generation;
	return id;
}

// Returns a copy. Write through world.solverSets[body.setIndex].bodySims[body.localIndex].
export function b3GetBodySim(world: b3World, bodyId: i32): b3BodySim {
	let set = world.solverSets[world.bodies[bodyId].setIndex];
	return set.bodySims[world.bodies[bodyId].localIndex];
}

// Returns the index into the awake set's body states, B3_NULL_INDEX where C returns NULL.
export function b3GetBodyState(world: b3World, bodyId: i32): i32 {
	if (world.bodies[bodyId].setIndex == b3_awakeSet) {
		let localIndex = world.bodies[bodyId].localIndex;
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= localIndex && localIndex < world.solverSets[b3_awakeSet].bodyStates.length);
		return localIndex;
	}

	return B3_NULL_INDEX;
}

export function b3RefreshBodyContactIndices(world: b3World, bodyId: i32): void {
	let encodedBodySimIndex = b3EncodeBodySimIndex(world.bodies[bodyId]);

	let contactKey = world.bodies[bodyId].headContactKey;
	while (contactKey != B3_NULL_INDEX) {
		let edgeIndex = contactKey & 1;
		let contactId = contactKey >> 1;

		let contact = world.contacts[contactId];
		contactKey = contact.edges[edgeIndex].nextKey;

		if (edgeIndex == 0) {
			contact.encodedBodySimA = encodedBodySimIndex;
		} else {
			contact.encodedBodySimB = encodedBodySimIndex;
		}
	}
}

export function b3SyncBodyFlags(world: b3World, bodyId: i32): void {
	let bodyFlags = world.bodies[bodyId].flags;
	let set = world.solverSets[world.bodies[bodyId].setIndex];
	let localIndex = world.bodies[bodyId].localIndex;

	// Preserve the sim only flags: fast for contact recycling, time of impact for debug draw.
	set.bodySims[localIndex].flags =
		(set.bodySims[localIndex].flags & (b3_isFast | b3_hadTimeOfImpact)) | (bodyFlags & ~b3_bodyTransientFlags);

	let stateIndex = b3GetBodyState(world, bodyId);
	if (stateIndex != B3_NULL_INDEX) {
		world.solverSets[b3_awakeSet].bodyStates[stateIndex].flags = bodyFlags & ~b3_bodyTransientFlags;
	}
}

function b3CreateIslandForBody(world: b3World, setIndex: i32, bodyId: i32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(world.bodies[bodyId].islandId == B3_NULL_INDEX);
	if (B3_ASSERT_ENABLED) B3_ASSERT(setIndex != b3_disabledSet);

	let island = b3CreateIsland(world, setIndex);
	island.bodies.push(world.bodies[bodyId].id);
	world.bodies[bodyId].islandId = island.islandId;
	world.bodies[bodyId].islandIndex = 0;
}

function b3RemoveBodyFromIsland(world: b3World, bodyId: i32): void {
	if (world.bodies[bodyId].islandId == B3_NULL_INDEX) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(world.bodies[bodyId].islandIndex == B3_NULL_INDEX);
		return;
	}

	let islandId = world.bodies[bodyId].islandId;
	let island = world.islands[islandId];
	{
		let localIndex = world.bodies[bodyId].islandIndex;
		let movedBodyId = island.bodies[island.bodies.length - 1];
		island.bodies[localIndex] = movedBodyId;
		world.bodies[movedBodyId].islandIndex = localIndex;
		island.bodies.length = island.bodies.length - 1;
	}

	if (island.bodies.length == 0) {
		// Destroy empty island
		if (B3_ASSERT_ENABLED) B3_ASSERT(island.contacts.length == 0);
		if (B3_ASSERT_ENABLED) B3_ASSERT(island.joints.length == 0);

		// Free the island
		b3DestroyIsland(world, island.islandId);
	}

	world.bodies[bodyId].islandId = B3_NULL_INDEX;
	world.bodies[bodyId].islandIndex = B3_NULL_INDEX;
}

function b3DestroyBodyContacts(world: b3World, bodyId: i32, wakeBodies: bool): void {
	// Destroy the attached contacts
	let edgeKey = world.bodies[bodyId].headContactKey;
	while (edgeKey != B3_NULL_INDEX) {
		let contactId = edgeKey >> 1;
		let edgeIndex = edgeKey & 1;

		let contact = world.contacts[contactId];
		edgeKey = contact.edges[edgeIndex].nextKey;
		b3DestroyContact(world, contact, wakeBodies);
	}
}

export function b3CreateBody(worldId: b3WorldId, def: b3BodyDef): b3BodyId {
	if (B3_ASSERT_ENABLED) {
		B3_ASSERT(def.internalValue == B3_SECRET_COOKIE);
		B3_ASSERT(b3IsValidPosition(def.position));
		B3_ASSERT(b3IsValidQuat(def.rotation));
		B3_ASSERT(b3IsValidVec3(def.linearVelocity));
		B3_ASSERT(b3IsValidVec3(def.angularVelocity));
		B3_ASSERT(b3IsValidFloat(def.linearDamping) && def.linearDamping >= 0.0);
		B3_ASSERT(b3IsValidFloat(def.angularDamping) && def.angularDamping >= 0.0);
		B3_ASSERT(b3IsValidFloat(def.sleepThreshold) && def.sleepThreshold >= 0.0);
		B3_ASSERT(b3IsValidFloat(def.safetyFactor) && def.safetyFactor >= 0.0);
		B3_ASSERT(b3IsValidFloat(def.gravityScale));
	}

	let world = b3GetUnlockedWorldFromId(worldId);

	if (world == null) {
		return b3_nullBodyId;
	}

	world.locked = true;

	let isAwake = (def.isAwake || def.enableSleep == false) && def.isEnabled;

	// determine the solver set
	let setId: i32;
	if (def.isEnabled == false) {
		// any body type can be disabled
		setId = b3_disabledSet;
	} else if (def.type == b3_staticBody) {
		setId = b3_staticSet;
	} else if (isAwake == true) {
		setId = b3_awakeSet;
	} else {
		// new set for a sleeping body in its own island
		setId = b3AllocId(world.solverSetIdPool);
		if (setId == world.solverSets.length) {
			// Create a zero initialized solver set. All sub-arrays are also zero initialized.
			world.solverSets.push(new b3SolverSet());
		} else {
			if (B3_ASSERT_ENABLED) B3_ASSERT(world.solverSets[setId].setIndex == B3_NULL_INDEX);
		}

		world.solverSets[setId].setIndex = setId;
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= setId && setId < world.solverSets.length);

	let bodyId = b3AllocId(world.bodyIdPool);

	let lockFlags: u32 = 0;
	lockFlags |= def.motionLocks.linearX ? b3_lockLinearX : 0;
	lockFlags |= def.motionLocks.linearY ? b3_lockLinearY : 0;
	lockFlags |= def.motionLocks.linearZ ? b3_lockLinearZ : 0;
	lockFlags |= def.motionLocks.angularX ? b3_lockAngularX : 0;
	lockFlags |= def.motionLocks.angularY ? b3_lockAngularY : 0;
	lockFlags |= def.motionLocks.angularZ ? b3_lockAngularZ : 0;

	let set = world.solverSets[setId];
	let bodySim = b3BodySim();
	bodySim.transform.p = def.position;
	bodySim.transform.q = def.rotation;
	bodySim.center = def.position;
	bodySim.rotation0 = bodySim.transform.q;
	bodySim.center0 = bodySim.center;
	bodySim.localCenter = b3Vec3_zero;
	bodySim.force = b3Vec3_zero;
	bodySim.torque = b3Vec3_zero;
	bodySim.invMass = 0.0;
	bodySim.inertia = b3Mat3_zero;
	bodySim.invInertiaLocal = b3Mat3_zero;
	bodySim.minExtent = B3_HUGE();
	bodySim.maxExtent = b3Vec3_zero;
	bodySim.linearDamping = def.linearDamping;
	bodySim.angularDamping = def.angularDamping;
	bodySim.gravityScale = def.gravityScale;
	bodySim.bodyId = bodyId;
	bodySim.flags = lockFlags;
	bodySim.flags |= def.isBullet ? b3_isBullet : 0;
	bodySim.flags |= def.allowFastRotation ? b3_allowFastRotation : 0;
	bodySim.flags |= def.type == b3_dynamicBody ? b3_dynamicFlag : 0;
	bodySim.flags |= def.enableSleep ? b3_enableSleep : 0;
	bodySim.flags |= def.enableContactRecycling ? b3_enableContactRecycling : 0;
	set.bodySims.push(bodySim);

	if (setId == b3_awakeSet) {
		let bodyState = b3BodyState();
		bodyState.linearVelocity = def.linearVelocity;
		bodyState.angularVelocity = def.angularVelocity;
		bodyState.deltaRotation = b3Quat_identity;
		bodyState.flags = bodySim.flags;
		set.bodyStates.push(bodyState);
	}

	if (bodyId == world.bodies.length) {
		world.bodies.push(b3Body());
	} else {
		if (B3_ASSERT_ENABLED) B3_ASSERT(world.bodies[bodyId].id == B3_NULL_INDEX);
	}

	let body = world.bodies[bodyId];
	body.userData = def.userData;
	body.setIndex = setId;
	body.localIndex = set.bodySims.length - 1;
	body.generation += 1;
	body.headShapeId = B3_NULL_INDEX;
	body.shapeCount = 0;
	body.headChainId = B3_NULL_INDEX;
	body.headContactKey = B3_NULL_INDEX;
	body.contactCount = 0;
	body.headJointKey = B3_NULL_INDEX;
	body.jointCount = 0;
	body.islandId = B3_NULL_INDEX;
	body.islandIndex = B3_NULL_INDEX;
	body.bodyMoveIndex = B3_NULL_INDEX;
	body.id = bodyId;
	body.sleepThreshold = def.sleepThreshold;
	body.sleepTime = 0.0;
	body.sleepVelocity = 0.0;
	body.safetyFactor = def.safetyFactor;
	body.mass = 0.0;
	body.nameId = b3AddName(world.names, def.name);
	body.type = def.type;
	body.flags = bodySim.flags;
	world.bodies[bodyId] = body;

	// dynamic and kinematic bodies that are enabled need a island
	if (setId >= b3_awakeSet) {
		b3CreateIslandForBody(world, setId, bodyId);
	}

	let id = b3BodyId();
	id.index1 = bodyId + 1;
	id.world0 = world.worldId;
	id.generation = world.bodies[bodyId].generation;

	world.locked = false;

	return id;
}

export function b3IsBodyAwake(world: b3World, bodyId: i32): bool {
	return world.bodies[bodyId].setIndex == b3_awakeSet;
}

// Careful: this moves body sims and states, re-read any copy taken before.
export function b3WakeBody(world: b3World, bodyId: i32): bool {
	let setIndex = world.bodies[bodyId].setIndex;
	if (setIndex >= b3_firstSleepingSet) {
		b3WakeSolverSet(world, setIndex);
		return true;
	}

	return false;
}

export function b3WakeBodyWithLock(world: b3World, bodyId: i32): bool {
	if (B3_ASSERT_ENABLED) B3_ASSERT(world.locked == false);
	world.locked = true;
	let woke = b3WakeBody(world, bodyId);
	world.locked = false;
	return woke;
}

export function b3DestroyBody(bodyId: b3BodyId): void {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	world.locked = true;

	let body = b3GetBodyFullId(world, bodyId);

	// Wake bodies attached to this body, even if this body is static.
	let wakeBodies = true;

	// Destroy the attached joints
	let edgeKey = world.bodies[body].headJointKey;
	while (edgeKey != B3_NULL_INDEX) {
		let jointId = edgeKey >> 1;
		let edgeIndex = edgeKey & 1;

		edgeKey = world.joints[jointId].edges[edgeIndex].nextKey;

		// Careful because this modifies the list being traversed
		b3DestroyJointInternal(world, jointId, wakeBodies);
	}

	// Destroy all contacts attached to this body.
	b3DestroyBodyContacts(world, body, wakeBodies);

	// Destroy the attached shapes and their broad-phase proxies.
	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B3_NULL_INDEX) {
		let shape = world.shapes[shapeId];

		if (shape.type == b3_compoundShape) {
			if (B3_ASSERT_ENABLED) B3_ASSERT(world.compoundShapeCount > 0);
			world.compoundShapeCount -= 1;
		}

		if (shape.sensorIndex != B3_NULL_INDEX) {
			b3DestroySensor(world, shape);
		}

		b3DestroyShapeProxy(shape, world.broadPhase);

		b3DestroyShapeAllocations(world, shape);

		// Return shape to free list.
		b3FreeId(world.shapeIdPool, shapeId);
		shape.id = B3_NULL_INDEX;

		shapeId = shape.nextShapeId;
	}

	b3RemoveBodyFromIsland(world, body);

	// Remove body sim from solver set that owns it
	let bodySetIndex = world.bodies[body].setIndex;
	let bodyLocalIndex = world.bodies[body].localIndex;
	let set = world.solverSets[bodySetIndex];
	let movedIndex = b3Array_RemoveSwap(set.bodySims, bodyLocalIndex);
	if (movedIndex != B3_NULL_INDEX) {
		// Fix moved body index
		let movedId = set.bodySims[bodyLocalIndex].bodyId;
		if (B3_ASSERT_ENABLED) B3_ASSERT(world.bodies[movedId].localIndex == movedIndex);
		world.bodies[movedId].localIndex = bodyLocalIndex;
		b3RefreshBodyContactIndices(world, movedId);
	}

	// Remove body state from awake set
	if (bodySetIndex == b3_awakeSet) {
		let result = b3Array_RemoveSwap(set.bodyStates, bodyLocalIndex);
		if (B3_ASSERT_ENABLED) B3_ASSERT(result == movedIndex);
	} else if (set.setIndex >= b3_firstSleepingSet && set.bodySims.length == 0) {
		// Remove solver set if it's now an orphan.
		b3DestroySolverSet(world, set.setIndex);
	}

	// Free body and id (preserve body revision)
	b3FreeId(world.bodyIdPool, world.bodies[body].id);

	world.bodies[body].setIndex = B3_NULL_INDEX;
	world.bodies[body].localIndex = B3_NULL_INDEX;
	world.bodies[body].id = B3_NULL_INDEX;

	world.locked = false;
}

export function b3Body_GetContactCapacity(bodyId: b3BodyId): i32 {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return 0;
	}

	let body = b3GetBodyFullId(world, bodyId);

	// Conservative and fast
	return world.bodies[body].contactCount;
}

export function b3Body_GetContactData(bodyId: b3BodyId, contactData: StaticArray<b3ContactData>, capacity: i32): i32 {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return 0;
	}

	let body = b3GetBodyFullId(world, bodyId);

	let contactKey = world.bodies[body].headContactKey;
	let index = 0;
	while (contactKey != B3_NULL_INDEX && index < capacity) {
		let contactId = contactKey >> 1;
		let edgeIndex = contactKey & 1;

		let contact = world.contacts[contactId];

		// Is contact touching?
		if (contact.flags & b3_contactTouchingFlag) {
			let shapeA = world.shapes[contact.shapeIdA];
			let shapeB = world.shapes[contact.shapeIdB];

			let data = new b3ContactData();
			data.contactId = b3MakeContactIdFrom(contact.contactId + 1, bodyId.world0, contact.generation);
			data.shapeIdA = b3MakeShapeIdFrom(shapeA.id + 1, bodyId.world0, shapeA.generation);
			data.shapeIdB = b3MakeShapeIdFrom(shapeB.id + 1, bodyId.world0, shapeB.generation);
			data.manifolds = contact.manifolds;
			data.manifoldCount = contact.manifoldCount;
			contactData[index] = data;
			index += 1;
		}

		contactKey = contact.edges[edgeIndex].nextKey;
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(index <= capacity);

	return index;
}

export function b3Body_ComputeAABB(bodyId: b3BodyId): b3AABB {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return b3AABB();
	}

	let body = b3GetBodyFullId(world, bodyId);
	if (world.bodies[body].headShapeId == B3_NULL_INDEX) {
		let transform = b3GetBodyTransform(world, world.bodies[body].id);
		let p = b3ToVec3(transform.p);
		return b3AABB(p, p);
	}

	let shape = world.shapes[world.bodies[body].headShapeId];
	let aabb = shape.aabb;
	while (shape.nextShapeId != B3_NULL_INDEX) {
		shape = world.shapes[shape.nextShapeId];
		aabb = b3AABB_Union(aabb, shape.aabb);
	}

	return aabb;
}

export function b3Body_GetClosestPoint(bodyId: b3BodyId, result: ref b3Vec3, target: b3Vec3): f32 {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		result = b3Vec3();
		return 0.0;
	}

	let body = b3GetBodyFullId(world, bodyId);
	let worldTransform = b3GetBodyTransform(world, world.bodies[body].id);
	let transform = b3ToRelativeTransform(worldTransform, b3Pos_zero);

	let closestDistance: f32 = FLT_MAX;
	let closestPoint = transform.p;

	let targetPoint = new StaticArray<b3Vec3>(1);
	targetPoint[0] = target;

	let input = new b3DistanceInput();
	input.proxyA = new b3ShapeProxy(targetPoint, 1, 0.0);

	// Target rides in frame A at the origin, so the relative pose of the shape in A is the body transform
	input.transform = transform;
	input.useRadii = false;

	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B3_NULL_INDEX) {
		let shape = world.shapes[shapeId];
		shapeId = shape.nextShapeId;

		let type = shape.type;
		if (type != b3_sphereShape && type != b3_capsuleShape && type != b3_hullShape) {
			continue;
		}

		input.proxyB = b3MakeShapeProxy(shape);

		let cache = b3SimplexCache();
		let output = b3ShapeDistance(input, cache, null, 0);
		if (output.distance < closestDistance) {
			closestDistance = output.distance;
			closestPoint = output.pointB;
		}
	}

	result = closestPoint;
	return closestDistance;
}

export function b3Body_CastRay(
	bodyId: b3BodyId,
	origin: b3Pos,
	translation: b3Vec3,
	filter: b3QueryFilter,
	maxFraction: f32,
	bodyTransform: b3WorldTransform,
): b3BodyCastResult {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return b3BodyCastResult();
	}

	let result = b3BodyCastResult();
	let body = b3GetBodyFullId(world, bodyId);

	// The consistent framing is to center on the ray origin.
	let shapeInput = b3RayCastInput();
	shapeInput.origin = b3Vec3_zero;
	shapeInput.translation = translation;
	shapeInput.maxFraction = maxFraction;

	let transform = b3ToRelativeTransform(bodyTransform, origin);

	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B3_NULL_INDEX) {
		let shape = world.shapes[shapeId];
		shapeId = shape.nextShapeId;

		if (b3ShouldQueryCollide(shape.filter, filter) == false) {
			continue;
		}

		let shapeOutput = b3RayCastShape(shape, transform, shapeInput);

		if (shapeOutput.hit == false) {
			continue;
		}

		if (shapeOutput.fraction > shapeInput.maxFraction) {
			continue;
		}

		// Careful with id, shapeId is the next shape.
		let id = b3MakeShapeIdFrom(shape.id + 1, bodyId.world0, shape.generation);

		let materialIndex = b3ClampInt(shapeOutput.materialIndex, 0, shape.materialCount - 1);
		let userMaterialId = b3GetShapeMaterials(shape)[materialIndex].userMaterialId;

		result = b3BodyCastResult();
		result.shapeId = id;
		result.point = b3OffsetPos(origin, shapeOutput.point);
		result.normal = shapeOutput.normal;
		result.fraction = shapeOutput.fraction;
		result.triangleIndex = shapeOutput.triangleIndex;
		result.userMaterialId = userMaterialId;
		result.iterations = shapeOutput.iterations;
		result.hit = true;

		shapeInput.maxFraction = shapeOutput.fraction;
	}

	return result;
}

export function b3Body_CastShape(
	bodyId: b3BodyId,
	origin: b3Pos,
	proxy: b3ShapeProxy,
	translation: b3Vec3,
	filter: b3QueryFilter,
	maxFraction: f32,
	canEncroach: bool,
	bodyTransform: b3WorldTransform,
): b3BodyCastResult {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return b3BodyCastResult();
	}

	let result = b3BodyCastResult();
	let body = b3GetBodyFullId(world, bodyId);

	let transform = b3ToRelativeTransform(bodyTransform, origin);

	let shapeInput = new b3ShapeCastInput();
	shapeInput.proxy = new b3ShapeProxy(proxy.points, proxy.count, proxy.radius);
	shapeInput.translation = translation;
	shapeInput.maxFraction = maxFraction;
	shapeInput.canEncroach = canEncroach;

	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B3_NULL_INDEX) {
		let shape = world.shapes[shapeId];
		shapeId = shape.nextShapeId;

		if (b3ShouldQueryCollide(shape.filter, filter) == false) {
			continue;
		}

		let shapeOutput = b3ShapeCastShape(shape, transform, shapeInput);

		if (shapeOutput.hit == false) {
			continue;
		}

		if (shapeOutput.fraction > shapeInput.maxFraction) {
			continue;
		}

		// Careful with id, shapeId is the next shape.
		let id = b3MakeShapeIdFrom(shape.id + 1, bodyId.world0, shape.generation);
		let materialIndex = b3ClampInt(shapeOutput.materialIndex, 0, shape.materialCount - 1);
		let userMaterialId = b3GetShapeMaterials(shape)[materialIndex].userMaterialId;

		result = b3BodyCastResult();
		result.shapeId = id;
		result.point = b3OffsetPos(origin, shapeOutput.point);
		result.normal = shapeOutput.normal;
		result.fraction = shapeOutput.fraction;
		result.triangleIndex = shapeOutput.triangleIndex;
		result.userMaterialId = userMaterialId;
		result.iterations = shapeOutput.iterations;
		result.hit = true;

		shapeInput.maxFraction = shapeOutput.fraction;
	}

	return result;
}

export function b3Body_OverlapShape(
	bodyId: b3BodyId,
	origin: b3Pos,
	proxy: b3ShapeProxy,
	filter: b3QueryFilter,
	bodyTransform: b3WorldTransform,
): bool {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return false;
	}

	let body = b3GetBodyFullId(world, bodyId);
	let transform = b3ToRelativeTransform(bodyTransform, origin);

	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B3_NULL_INDEX) {
		let shape = world.shapes[shapeId];
		shapeId = shape.nextShapeId;

		if (b3ShouldQueryCollide(shape.filter, filter) == false) {
			continue;
		}

		let overlaps = b3OverlapShape(shape, transform, proxy);
		if (overlaps) {
			return true;
		}
	}

	return false;
}

export function b3Body_CollideMover(
	bodyId: b3BodyId,
	bodyPlanes: StaticArray<b3BodyPlaneResult>,
	planeCapacity: i32,
	origin: b3Pos,
	mover: b3Capsule,
	filter: b3QueryFilter,
	bodyTransform: b3WorldTransform,
): i32 {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return 0;
	}

	if (planeCapacity == 0) {
		return 0;
	}

	let resultCount = 0;
	let body = b3GetBodyFullId(world, bodyId);

	let transform = b3ToRelativeTransform(bodyTransform, origin);

	// C's one element stack plane, filled before each use
	let plane = new StaticArray<b3PlaneResult>(1);

	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B3_NULL_INDEX) {
		let shape = world.shapes[shapeId];
		shapeId = shape.nextShapeId;

		if (b3ShouldQueryCollide(shape.filter, filter) == false) {
			continue;
		}

		let type = shape.type;
		if (type != b3_sphereShape && type != b3_capsuleShape && type != b3_hullShape) {
			continue;
		}

		let count = b3CollideMover(plane, 1, shape, transform, mover);

		if (count > 0) {
			let id = b3MakeShapeIdFrom(shape.id + 1, bodyId.world0, shape.generation);
			let bodyPlane = b3BodyPlaneResult();
			bodyPlane.shapeId = id;
			bodyPlane.result = plane[0];
			bodyPlanes[resultCount] = bodyPlane;
			resultCount += 1;
			if (resultCount == planeCapacity) {
				return resultCount;
			}
		}
	}

	return resultCount;
}

export function b3Body_TimeOfImpactMover(
	bodyId: b3BodyId,
	origin: b3Pos,
	mover: b3Capsule,
	moverTranslation: b3Vec3,
	filter: b3QueryFilter,
	bodyTransform1: b3WorldTransform,
	bodyTransform2: b3WorldTransform,
): b3BodyTOIResult {
	let result = b3BodyTOIResult();
	result.fraction = 1.0;

	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return result;
	}

	let xf1 = b3ToRelativeTransform(bodyTransform1, origin);
	let xf2 = b3ToRelativeTransform(bodyTransform2, origin);

	let body = b3GetBodyFullId(world, bodyId);
	let localCenter = b3GetBodySim(world, body).localCenter;

	let capsulePoints = new StaticArray<b3Vec3>(2);
	capsulePoints[0] = mover.center1;
	capsulePoints[1] = mover.center2;
	let input = new b3TOIInput();
	input.proxyB = new b3ShapeProxy(capsulePoints, 2, mover.radius);
	input.sweepA.c1 = b3TransformPoint(xf1, localCenter);
	input.sweepA.c2 = b3TransformPoint(xf2, localCenter);
	input.sweepA.q1 = bodyTransform1.q;
	input.sweepA.q2 = bodyTransform2.q;
	input.sweepA.localCenter = localCenter;

	input.sweepB.c1 = b3Vec3_zero;
	input.sweepB.c2 = moverTranslation;
	input.sweepB.q1 = b3Quat_identity;
	input.sweepB.q2 = b3Quat_identity;
	input.sweepB.localCenter = b3Vec3_zero;

	input.maxFraction = 1.0;

	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B3_NULL_INDEX) {
		let shape = world.shapes[shapeId];
		shapeId = shape.nextShapeId;

		if (b3ShouldQueryCollide(shape.filter, filter) == false) {
			continue;
		}

		let type = shape.type;
		if (type != b3_sphereShape && type != b3_capsuleShape && type != b3_hullShape) {
			continue;
		}

		input.proxyA = b3MakeShapeProxy(shape);

		let output = b3TimeOfImpact(input);

		// Mimic behavior in b3ContinuousQueryCallback. Ignore shapes that initially overlap.
		if (0.0 < output.fraction && output.fraction < result.fraction) {
			input.maxFraction = output.fraction;

			result.point = b3OffsetPos(origin, output.point);
			result.normal = output.normal;
			result.fraction = output.fraction;
			result.shapeId = b3MakeShapeIdFrom(shape.id + 1, world.worldId, shape.generation);
		}
	}

	return result;
}

// This computes the deviation of the principal inertia values without computing eigenvalues.
// It computes:
// ((I1 - I2)^2 + (I1 - I3)^2 + (I2 - I3)^2) / (I1^2 + I2^2 + I3^2)
// https://en.wikipedia.org/wiki/Cauchy_stress_tensor#Stress_deviator_tensor
function b3UpdateIsotropicInertia(world: b3World, bodyId: i32, bodySim: b3BodySim): void {
	let inertia = bodySim.inertia;
	let mean: f32 = (inertia.cx.x + inertia.cy.y + inertia.cz.z) / 3.0;
	let dx: f32 = inertia.cx.x - mean;
	let dy: f32 = inertia.cy.y - mean;
	let dz: f32 = inertia.cz.z - mean;
	let offDiagonal: f32 = inertia.cy.x * inertia.cy.x + inertia.cz.x * inertia.cz.x + inertia.cz.y * inertia.cz.y;

	// This is the square of the Frobenius matrix norm.
	// https://en.wikipedia.org/wiki/Matrix_norm
	let deviation: f32 = dx * dx + dy * dy + dz * dz + 2.0 * offDiagonal;

	let tolerance = world.isotropicInertiaTolerance;

	if (2.0 * deviation > 3.0 * tolerance * tolerance * mean * mean) {
		world.bodies[bodyId].flags |= b3_enableGyroscopicTorque;
	} else {
		world.bodies[bodyId].flags &= ~b3_enableGyroscopicTorque;
	}

	b3SyncBodyFlags(world, bodyId);
}

export function b3UpdateBodyMassData(world: b3World, bodyId: i32): void {
	let set = world.solverSets[world.bodies[bodyId].setIndex];
	let localIndex = world.bodies[bodyId].localIndex;

	// Mass is no longer dirty
	world.bodies[bodyId].flags &= ~b3_dirtyMass;
	world.bodies[bodyId].flags &= ~b3_enableGyroscopicTorque;
	b3SyncBodyFlags(world, bodyId);

	// Copied after the flag sync; stored back before every return and before the inertia sync below.
	let bodySim = set.bodySims[localIndex];

	// Compute mass data from shapes. Each shape has its own density.
	world.bodies[bodyId].mass = 0.0;
	bodySim.inertia = b3Mat3_zero;
	bodySim.invMass = 0.0;
	bodySim.invInertiaLocal = b3Mat3_zero;
	bodySim.invInertiaWorld = b3Mat3_zero;
	bodySim.localCenter = b3Vec3_zero;
	bodySim.minExtent = B3_HUGE();
	bodySim.maxExtent = b3Vec3_zero;

	if (world.bodies[bodyId].headShapeId == B3_NULL_INDEX) {
		set.bodySims[localIndex] = bodySim;
		return;
	}

	// Static and kinematic sims have zero mass.
	let bodyType = world.bodies[bodyId].type;
	if (bodyType != b3_dynamicBody) {
		bodySim.center = bodySim.transform.p;
		bodySim.center0 = bodySim.center;

		// Need extents for kinematic bodies for sleeping to work correctly.
		if (bodyType == b3_kinematicBody) {
			let shapeId = world.bodies[bodyId].headShapeId;
			while (shapeId != B3_NULL_INDEX) {
				let s = world.shapes[shapeId];

				let extent = b3ComputeShapeExtent(s, b3Vec3_zero);
				bodySim.minExtent = b3MinFloat(bodySim.minExtent, extent.minExtent);
				bodySim.maxExtent = b3Max(bodySim.maxExtent, extent.maxExtent);

				shapeId = s.nextShapeId;
			}
		}

		set.bodySims[localIndex] = bodySim;
		return;
	}

	let shapeCount = world.bodies[bodyId].shapeCount;
	let masses = new StaticArray<b3MassData>(shapeCount);

	// Accumulate mass over all shapes.
	let bodyMass: f32 = 0.0;
	let localCenter = b3Vec3_zero;
	let shapeId = world.bodies[bodyId].headShapeId;
	let shapeIndex = 0;
	while (shapeId != B3_NULL_INDEX) {
		let s = world.shapes[shapeId];
		shapeId = s.nextShapeId;

		if (s.density == 0.0) {
			masses[shapeIndex] = b3MassData();
			shapeIndex += 1;
			continue;
		}

		let massData = b3ComputeShapeMass(s);
		bodyMass += massData.mass;
		localCenter = b3MulAdd(localCenter, massData.mass, massData.center);

		masses[shapeIndex] = massData;
		shapeIndex += 1;
	}

	world.bodies[bodyId].mass = bodyMass;

	// Compute center of mass.
	if (bodyMass > 0.0) {
		bodySim.invMass = 1.0 / bodyMass;
		localCenter = b3MulSV(bodySim.invMass, localCenter);
	}

	// Second loop to accumulate the rotational inertia about the center of mass
	for (shapeIndex = 0; shapeIndex < shapeCount; ++shapeIndex) {
		let massData = masses[shapeIndex];
		if (massData.mass == 0.0) {
			continue;
		}

		// Shift to center of mass. This is safe because it can only increase.
		let offset = b3Sub(localCenter, massData.center);
		let inertia = b3AddMM(massData.inertia, b3Steiner(massData.mass, offset));
		bodySim.inertia = b3AddMM(bodySim.inertia, inertia);
	}

	let det = b3Det(bodySim.inertia);
	if (B3_ASSERT_ENABLED) B3_ASSERT(det >= 0.0);

	if (det > 0.0) {
		// This call is faster than b3Invert
		bodySim.invInertiaLocal = b3InvertT(bodySim.inertia);

		let rotationMatrix = b3MakeMatrixFromQuat(bodySim.transform.q);
		bodySim.invInertiaWorld = b3MulMM(b3MulMM(rotationMatrix, bodySim.invInertiaLocal), b3Transpose(rotationMatrix));
	}

	// Move center of mass.
	let oldCenter = bodySim.center;
	bodySim.localCenter = localCenter;
	bodySim.center = b3TransformWorldPoint(bodySim.transform, bodySim.localCenter);
	bodySim.center0 = bodySim.center;

	// Update center of mass velocity
	let stateIndex = b3GetBodyState(world, bodyId);
	if (stateIndex != B3_NULL_INDEX) {
		let states = world.solverSets[b3_awakeSet].bodyStates;
		let deltaLinear = b3Cross(states[stateIndex].angularVelocity, b3SubPos(bodySim.center, oldCenter));
		states[stateIndex].linearVelocity = b3Add(states[stateIndex].linearVelocity, deltaLinear);
	}

	// Compute body extents relative to center of mass
	shapeId = world.bodies[bodyId].headShapeId;
	while (shapeId != B3_NULL_INDEX) {
		let s = world.shapes[shapeId];

		let extent = b3ComputeShapeExtent(s, localCenter);
		bodySim.minExtent = b3MinFloat(bodySim.minExtent, extent.minExtent);
		bodySim.maxExtent = b3Max(bodySim.maxExtent, extent.maxExtent);

		shapeId = s.nextShapeId;
	}

	// When the center of mass changes, any cached contact manifold becomes invalid.
	let edgeKey = world.bodies[bodyId].headContactKey;
	while (edgeKey != B3_NULL_INDEX) {
		let contactId = edgeKey >> 1;
		let edgeIndex = edgeKey & 1;

		let contact = world.contacts[contactId];
		contact.flags &= ~b3_relativeTransformValid;

		edgeKey = contact.edges[edgeIndex].nextKey;
	}

	// Apply fixed rotation
	if ((bodySim.flags & b3_fixedRotation) == b3_fixedRotation) {
		bodySim.inertia = b3Mat3_zero;
		bodySim.invInertiaLocal = b3Mat3_zero;
		bodySim.invInertiaWorld = b3Mat3_zero;
	}

	set.bodySims[localIndex] = bodySim;

	b3UpdateIsotropicInertia(world, bodyId, bodySim);
}

export function b3Body_GetPosition(bodyId: b3BodyId): b3Pos {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	let transform = b3GetBodyTransformQuick(world, body);
	return transform.p;
}

export function b3Body_GetRotation(bodyId: b3BodyId): b3Quat {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	let transform = b3GetBodyTransformQuick(world, body);
	return transform.q;
}

export function b3Body_GetTransform(bodyId: b3BodyId): b3WorldTransform {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return b3GetBodyTransformQuick(world, body);
}

export function b3Body_GetLocalPoint(bodyId: b3BodyId, worldPoint: b3Pos): b3Vec3 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	let transform = b3GetBodyTransformQuick(world, body);
	return b3InvTransformWorldPoint(transform, worldPoint);
}

export function b3Body_GetWorldPoint(bodyId: b3BodyId, localPoint: b3Vec3): b3Pos {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	let transform = b3GetBodyTransformQuick(world, body);
	return b3TransformWorldPoint(transform, localPoint);
}

export function b3Body_GetLocalVector(bodyId: b3BodyId, worldVector: b3Vec3): b3Vec3 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	let transform = b3GetBodyTransformQuick(world, body);
	return b3InvRotateVector(transform.q, worldVector);
}

export function b3Body_GetWorldVector(bodyId: b3BodyId, localVector: b3Vec3): b3Vec3 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	let transform = b3GetBodyTransformQuick(world, body);
	return b3RotateVector(transform.q, localVector);
}

export function b3Body_SetTransform(bodyId: b3BodyId, position: b3Pos, rotation: b3Quat): void {
	if (B3_ASSERT_ENABLED) {
		B3_ASSERT(b3IsValidPosition(position));
		B3_ASSERT(b3IsValidQuat(rotation));
		B3_ASSERT(b3Body_IsValid(bodyId));
	}
	let world = b3GetWorld(bodyId.world0);
	if (B3_ASSERT_ENABLED) B3_ASSERT(world.locked == false);

	let body = b3GetBodyFullId(world, bodyId);
	let set = world.solverSets[world.bodies[body].setIndex];
	let localIndex = world.bodies[body].localIndex;
	let bodySim = set.bodySims[localIndex];

	bodySim.transform.p = position;
	bodySim.transform.q = rotation;
	bodySim.center = b3TransformWorldPoint(bodySim.transform, bodySim.localCenter);

	let rotationMatrix = b3MakeMatrixFromQuat(bodySim.transform.q);
	bodySim.invInertiaWorld = b3MulMM(b3MulMM(rotationMatrix, bodySim.invInertiaLocal), b3Transpose(rotationMatrix));

	bodySim.rotation0 = bodySim.transform.q;
	bodySim.center0 = bodySim.center;
	set.bodySims[localIndex] = bodySim;

	let broadPhase = world.broadPhase;

	let transform = bodySim.transform;
	let speculativeDistance = B3_SPECULATIVE_DISTANCE();

	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B3_NULL_INDEX) {
		let shape = world.shapes[shapeId];
		let aabb = b3ComputeFatShapeAABB(shape, transform, speculativeDistance);
		shape.aabb = aabb;

		if (b3AABB_Contains(world.fatAABBs[shapeId], aabb) == false) {
			let fatAABB = b3AABB_Inflate(aabb, shape.aabbMargin);
			world.fatAABBs[shapeId] = fatAABB;

			// The body could be disabled
			if (shape.proxyKey != B3_NULL_INDEX) {
				b3BroadPhase_MoveProxy(broadPhase, shape.proxyKey, fatAABB);
			}
		}

		shapeId = shape.nextShapeId;
	}
}

export function b3Body_GetLinearVelocity(bodyId: b3BodyId): b3Vec3 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	let state = b3GetBodyState(world, body);
	if (state != B3_NULL_INDEX) {
		return world.solverSets[b3_awakeSet].bodyStates[state].linearVelocity;
	}
	return b3Vec3_zero;
}

export function b3Body_GetAngularVelocity(bodyId: b3BodyId): b3Vec3 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	let state = b3GetBodyState(world, body);
	if (state != B3_NULL_INDEX) {
		return world.solverSets[b3_awakeSet].bodyStates[state].angularVelocity;
	}
	return b3Vec3_zero;
}

export function b3Body_SetLinearVelocity(bodyId: b3BodyId, linearVelocity: b3Vec3): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidVec3(linearVelocity));

	let world = b3GetWorld(bodyId.world0);

	let body = b3GetBodyFullId(world, bodyId);

	if (world.bodies[body].type == b3_staticBody) {
		return;
	}

	if (b3LengthSquared(linearVelocity) > 0.0) {
		b3WakeBodyWithLock(world, body);
	}

	let state = b3GetBodyState(world, body);
	if (state == B3_NULL_INDEX) {
		return;
	}

	world.solverSets[b3_awakeSet].bodyStates[state].linearVelocity = linearVelocity;
}

export function b3Body_SetAngularVelocity(bodyId: b3BodyId, angularVelocity: b3Vec3): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidVec3(angularVelocity));

	let world = b3GetWorld(bodyId.world0);

	let body = b3GetBodyFullId(world, bodyId);

	if (world.bodies[body].type == b3_staticBody) {
		return;
	}

	// Apply locks to avoid waking
	let flags = world.bodies[body].flags;
	let w: b3Vec3;
	w.x = flags & b3_lockAngularX ? 0.0 : angularVelocity.x;
	w.y = flags & b3_lockAngularY ? 0.0 : angularVelocity.y;
	w.z = flags & b3_lockAngularZ ? 0.0 : angularVelocity.z;

	if (b3LengthSquared(w) != 0.0) {
		b3WakeBodyWithLock(world, body);
	}

	let state = b3GetBodyState(world, body);
	if (state == B3_NULL_INDEX) {
		return;
	}

	world.solverSets[b3_awakeSet].bodyStates[state].angularVelocity = w;
}

export function b3Body_SetTargetTransform(bodyId: b3BodyId, target: b3WorldTransform, timeStep: f32, wake: bool): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidWorldTransform(target));

	let world = b3GetWorld(bodyId.world0);

	let body = b3GetBodyFullId(world, bodyId);

	if (world.bodies[body].setIndex == b3_disabledSet) {
		return;
	}

	if (world.bodies[body].type == b3_staticBody || timeStep <= 0.0) {
		return;
	}

	if (world.bodies[body].setIndex != b3_awakeSet && wake == false) {
		return;
	}

	let sim = b3GetBodySim(world, body);

	// Compute linear velocity. The center difference is taken in world precision then demoted.
	let center1 = sim.center;
	let center2 = b3TransformWorldPoint(target, sim.localCenter);
	let invTimeStep: f32 = 1.0 / timeStep;
	let linearVelocity = b3MulSV(invTimeStep, b3SubPos(center2, center1));

	// Compute angular velocity:
	// q' = 0.5 * w * q
	// <~> ( q2 - q1 ) / dt =  0.5 * w * q1
	// <=> w = 2 * ( q2 - q1 ) * Conjugate( q1 ) / dt
	let q1 = sim.transform.q;
	let q2 = target.q;

	// Use the shortest arc quaternion
	if (b3DotQuat(q1, q2) < 0.0) {
		q2 = b3NegateQuat(q2);
	}

	let dq = b3Quat(b3Sub(q2.v, q1.v), q2.s - q1.s);
	let omega = b3MulQuat(dq, b3Conjugate(q1));
	let angularVelocity = b3MulSV(2.0 * invTimeStep, omega.v);

	// Early out if the body is asleep already and the desired movement is small
	if (world.bodies[body].setIndex != b3_awakeSet) {
		let maxVelocity: f32 = b3Length(linearVelocity) + b3Length(b3Mul(angularVelocity, sim.maxExtent));

		// Return if velocity would be sleepy
		if (maxVelocity < world.bodies[body].sleepThreshold) {
			return;
		}

		// Must wake for state to exist
		b3WakeBodyWithLock(world, body);
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(world.bodies[body].setIndex == b3_awakeSet);

	let state = b3GetBodyState(world, body);
	let states = world.solverSets[b3_awakeSet].bodyStates;
	states[state].linearVelocity = linearVelocity;
	states[state].angularVelocity = angularVelocity;
}

export function b3Body_GetLocalPointVelocity(bodyId: b3BodyId, localPoint: b3Vec3): b3Vec3 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	let state = b3GetBodyState(world, body);
	if (state == B3_NULL_INDEX) {
		return b3Vec3_zero;
	}

	let set = world.solverSets[world.bodies[body].setIndex];
	let bodySim = set.bodySims[world.bodies[body].localIndex];
	let bodyState = world.solverSets[b3_awakeSet].bodyStates[state];

	let r = b3RotateVector(bodySim.transform.q, b3Sub(localPoint, bodySim.localCenter));
	let v = b3Add(bodyState.linearVelocity, b3Cross(bodyState.angularVelocity, r));
	return v;
}

export function b3Body_GetWorldPointVelocity(bodyId: b3BodyId, worldPoint: b3Pos): b3Vec3 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	let state = b3GetBodyState(world, body);
	if (state == B3_NULL_INDEX) {
		return b3Vec3_zero;
	}

	let set = world.solverSets[world.bodies[body].setIndex];
	let bodySim = set.bodySims[world.bodies[body].localIndex];
	let bodyState = world.solverSets[b3_awakeSet].bodyStates[state];

	let r = b3SubPos(worldPoint, bodySim.center);
	let v = b3Add(bodyState.linearVelocity, b3Cross(bodyState.angularVelocity, r));
	return v;
}

export function b3Body_ApplyForce(bodyId: b3BodyId, force: b3Vec3, point: b3Pos, wake: bool): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidVec3(force));

	let world = b3GetWorld(bodyId.world0);

	let body = b3GetBodyFullId(world, bodyId);

	if (wake && world.bodies[body].setIndex >= b3_firstSleepingSet) {
		b3WakeBodyWithLock(world, body);
	}

	if (world.bodies[body].setIndex == b3_awakeSet) {
		let sims = world.solverSets[b3_awakeSet].bodySims;
		let localIndex = world.bodies[body].localIndex;
		sims[localIndex].force = b3Add(sims[localIndex].force, force);
		sims[localIndex].torque = b3Add(sims[localIndex].torque, b3Cross(b3SubPos(point, sims[localIndex].center), force));
	}
}

export function b3Body_ApplyForceToCenter(bodyId: b3BodyId, force: b3Vec3, wake: bool): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidVec3(force));

	let world = b3GetWorld(bodyId.world0);

	let body = b3GetBodyFullId(world, bodyId);

	if (wake && world.bodies[body].setIndex >= b3_firstSleepingSet) {
		b3WakeBodyWithLock(world, body);
	}

	if (world.bodies[body].setIndex == b3_awakeSet) {
		let sims = world.solverSets[b3_awakeSet].bodySims;
		let localIndex = world.bodies[body].localIndex;
		sims[localIndex].force = b3Add(sims[localIndex].force, force);
	}
}

export function b3Body_ApplyTorque(bodyId: b3BodyId, torque: b3Vec3, wake: bool): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidVec3(torque));

	let world = b3GetWorld(bodyId.world0);

	let body = b3GetBodyFullId(world, bodyId);

	if (wake && world.bodies[body].setIndex >= b3_firstSleepingSet) {
		b3WakeBodyWithLock(world, body);
	}

	if (world.bodies[body].setIndex == b3_awakeSet) {
		let sims = world.solverSets[b3_awakeSet].bodySims;
		let localIndex = world.bodies[body].localIndex;
		sims[localIndex].torque = b3Add(sims[localIndex].torque, torque);
	}
}

export function b3Body_ApplyLinearImpulse(bodyId: b3BodyId, impulse: b3Vec3, point: b3Pos, wake: bool): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidVec3(impulse));
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidPosition(point));

	let world = b3GetWorld(bodyId.world0);

	let body = b3GetBodyFullId(world, bodyId);

	if (wake && world.bodies[body].setIndex >= b3_firstSleepingSet) {
		b3WakeBodyWithLock(world, body);
	}

	if (world.bodies[body].setIndex == b3_awakeSet) {
		let localIndex = world.bodies[body].localIndex;
		let set = world.solverSets[b3_awakeSet];
		let states = set.bodyStates;
		let bodySim = set.bodySims[localIndex];

		states[localIndex].linearVelocity = b3MulAdd(states[localIndex].linearVelocity, bodySim.invMass, impulse);

		let maxLinearSpeed = world.maxLinearSpeed;
		if (b3LengthSquared(states[localIndex].linearVelocity) > maxLinearSpeed * maxLinearSpeed) {
			states[localIndex].linearVelocity = b3MulSV(maxLinearSpeed, b3Normalize(states[localIndex].linearVelocity));
		}

		let delta = b3MulMV(bodySim.invInertiaWorld, b3Cross(b3SubPos(point, bodySim.center), impulse));
		states[localIndex].angularVelocity = b3Add(states[localIndex].angularVelocity, delta);
	}
}

export function b3Body_ApplyLinearImpulseToCenter(bodyId: b3BodyId, impulse: b3Vec3, wake: bool): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidVec3(impulse));

	let world = b3GetWorld(bodyId.world0);

	let body = b3GetBodyFullId(world, bodyId);

	if (wake && world.bodies[body].setIndex >= b3_firstSleepingSet) {
		b3WakeBodyWithLock(world, body);
	}

	if (world.bodies[body].setIndex == b3_awakeSet) {
		let localIndex = world.bodies[body].localIndex;
		let set = world.solverSets[b3_awakeSet];
		let states = set.bodyStates;
		let invMass = set.bodySims[localIndex].invMass;
		states[localIndex].linearVelocity = b3MulAdd(states[localIndex].linearVelocity, invMass, impulse);

		let maxLinearSpeed = world.maxLinearSpeed;
		if (b3LengthSquared(states[localIndex].linearVelocity) > maxLinearSpeed * maxLinearSpeed) {
			states[localIndex].linearVelocity = b3MulSV(maxLinearSpeed, b3Normalize(states[localIndex].linearVelocity));
		}
	}
}

export function b3Body_ApplyAngularImpulse(bodyId: b3BodyId, impulse: b3Vec3, wake: bool): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidVec3(impulse));
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3Body_IsValid(bodyId));

	let world = b3GetWorld(bodyId.world0);

	let id = bodyId.index1 - 1;
	if (B3_ASSERT_ENABLED) B3_ASSERT(world.bodies[id].generation == bodyId.generation);

	if (wake && world.bodies[id].setIndex >= b3_firstSleepingSet) {
		// this will not invalidate body pointer
		b3WakeBodyWithLock(world, id);
	}

	if (world.bodies[id].setIndex == b3_awakeSet) {
		let localIndex = world.bodies[id].localIndex;
		let set = world.solverSets[b3_awakeSet];
		let states = set.bodyStates;
		let q = set.bodySims[localIndex].transform.q;

		let localImpulse = b3InvRotateVector(q, impulse);
		let localAngularVelocityDelta = b3MulMV(set.bodySims[localIndex].invInertiaLocal, localImpulse);
		states[localIndex].angularVelocity = b3Add(states[localIndex].angularVelocity, b3RotateVector(q, localAngularVelocityDelta));
	}
}

export function b3Body_GetType(bodyId: b3BodyId): b3BodyType {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return world.bodies[body].type;
}

// This should follow similar steps as you would get destroying and recreating the body, shapes, and joints.
// Contacts are difficult to preserve because the broad-phase pairs change, so I just destroy them.
// todo with a bit more effort I could support an option to let the body sleep
//
// Revised steps:
// 1 Skip disabled bodies
// 2 Destroy all contacts on the body
// 3 Wake the body
// 4 For all joints attached to the body
//  - wake attached bodies
//  - remove from island
//  - move to static set temporarily
// 5 Change the body type and transfer the body
// 6 If the body was static
//   - create an island for the body
//   Else if the body is becoming static
//   - remove it from the island
// 7 For all joints
//  - if either body is non-static
//    - link into island
//    - transfer to constraint graph
// 8 For all shapes
//  - Destroy proxy in old tree
//  - Create proxy in new tree
// Notes:
// - the implementation below tries to minimize the number of predicates, so some
//   operations may have no effect, such as transferring a joint to the same set
export function b3Body_SetType(bodyId: b3BodyId, type: b3BodyType): void {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	world.locked = true;
	let body = b3GetBodyFullId(world, bodyId);

	let originalType = world.bodies[body].type;
	if (originalType == type) {
		world.locked = false;
		return;
	}

	if (type != b3_staticBody) {
		let shapeId = world.bodies[body].headShapeId;
		while (shapeId != B3_NULL_INDEX) {
			let shape = world.shapes[shapeId];
			if (shape.type == b3_compoundShape || shape.type == b3_heightShape) {
				// Setting the body type is not supported for bodies with compound shapes
				// C returns here with the world still locked.
				return;
			}

			shapeId = shape.nextShapeId;
		}
	}

	// Stage 1: skip disabled bodies
	if (world.bodies[body].setIndex == b3_disabledSet) {
		// Disabled bodies don't change solver sets or islands when they change type.
		world.bodies[body].type = type;

		if (type == b3_dynamicBody) {
			world.bodies[body].flags |= b3_dynamicFlag;
		} else {
			world.bodies[body].flags &= ~b3_dynamicFlag;
		}

		b3SyncBodyFlags(world, body);

		// Body type affects the mass properties
		b3UpdateBodyMassData(world, body);
		world.locked = false;
		return;
	}

	// Stage 2: destroy all contacts but don't wake bodies (because we don't need to)
	let wakeBodies = false;
	b3DestroyBodyContacts(world, body, wakeBodies);

	// Stage 3: wake this body (does nothing if body is static), otherwise it will also wake
	// all bodies in the same sleeping solver set.
	b3WakeBody(world, body);

	// Stage 4: move joints to temporary storage
	let staticSet = world.solverSets[b3_staticSet];

	let jointKey = world.bodies[body].headJointKey;
	while (jointKey != B3_NULL_INDEX) {
		let jointId = jointKey >> 1;
		let edgeIndex = jointKey & 1;

		jointKey = world.joints[jointId].edges[edgeIndex].nextKey;

		// Joint may be disabled by other body
		if (world.joints[jointId].setIndex == b3_disabledSet) {
			continue;
		}

		// Wake attached bodies. The b3WakeBody call above does not wake bodies
		// attached to a static body. But it is necessary because the body may have
		// no joints.
		let bodyA = world.joints[jointId].edges[0].bodyId;
		let bodyB = world.joints[jointId].edges[1].bodyId;
		b3WakeBody(world, bodyA);
		b3WakeBody(world, bodyB);

		// Remove joint from island
		b3UnlinkJoint(world, jointId);

		// It is necessary to transfer all joints to the static set
		// so they can be added to the constraint graph below and acquire consistent colors.
		let jointSourceSet = world.solverSets[world.joints[jointId].setIndex];
		b3TransferJoint(world, staticSet, jointSourceSet, jointId);
	}

	// Stage 5: change the body type and transfer body
	world.bodies[body].type = type;

	if (type == b3_dynamicBody) {
		world.bodies[body].flags |= b3_dynamicFlag;
	} else {
		world.bodies[body].flags &= ~b3_dynamicFlag;
	}

	let awakeSet = world.solverSets[b3_awakeSet];
	let sourceSet = world.solverSets[world.bodies[body].setIndex];
	let targetSet = type == b3_staticBody ? staticSet : awakeSet;

	// Transfer body
	b3TransferBody(world, targetSet, sourceSet, body);

	// Stage 6: update island participation for the body
	if (originalType == b3_staticBody) {
		// Create island for body
		b3CreateIslandForBody(world, b3_awakeSet, body);
	} else if (type == b3_staticBody) {
		// Remove body from island.
		b3RemoveBodyFromIsland(world, body);
	}

	// Stage 7: Transfer joints to the target set
	jointKey = world.bodies[body].headJointKey;
	while (jointKey != B3_NULL_INDEX) {
		let jointId = jointKey >> 1;
		let edgeIndex = jointKey & 1;

		jointKey = world.joints[jointId].edges[edgeIndex].nextKey;

		// Joint may be disabled by other body
		if (world.joints[jointId].setIndex == b3_disabledSet) {
			continue;
		}

		// All joints were transferred to the static set in an earlier stage
		if (B3_ASSERT_ENABLED) B3_ASSERT(world.joints[jointId].setIndex == b3_staticSet);

		let bodyA = world.joints[jointId].edges[0].bodyId;
		let bodyB = world.joints[jointId].edges[1].bodyId;
		if (B3_ASSERT_ENABLED) {
			B3_ASSERT(world.bodies[bodyA].setIndex == b3_staticSet || world.bodies[bodyA].setIndex == b3_awakeSet);
			B3_ASSERT(world.bodies[bodyB].setIndex == b3_staticSet || world.bodies[bodyB].setIndex == b3_awakeSet);
		}

		if (world.bodies[bodyA].type == b3_dynamicBody || world.bodies[bodyB].type == b3_dynamicBody) {
			b3TransferJoint(world, awakeSet, staticSet, jointId);
		}
	}

	// Recreate shape proxies in broadphase
	let transform = b3GetBodyTransformQuick(world, body);
	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B3_NULL_INDEX) {
		let shape = world.shapes[shapeId];

		// Setting the body type is not supported for bodies with compound shapes
		if (B3_ASSERT_ENABLED) B3_ASSERT(shape.type != b3_compoundShape);

		shapeId = shape.nextShapeId;
		b3DestroyShapeProxy(shape, world.broadPhase);
		let forcePairCreation = true;
		b3CreateShapeProxy(world, shape, type, transform, forcePairCreation);
	}

	// Relink all joints
	jointKey = world.bodies[body].headJointKey;
	while (jointKey != B3_NULL_INDEX) {
		let jointId = jointKey >> 1;
		let edgeIndex = jointKey & 1;

		jointKey = world.joints[jointId].edges[edgeIndex].nextKey;

		let otherEdgeIndex = edgeIndex ^ 1;
		let otherBodyId = world.joints[jointId].edges[otherEdgeIndex].bodyId;

		if (world.bodies[otherBodyId].setIndex == b3_disabledSet) {
			continue;
		}

		if (world.bodies[body].type != b3_dynamicBody && world.bodies[otherBodyId].type != b3_dynamicBody) {
			continue;
		}

		b3LinkJoint(world, jointId);
	}

	b3SyncBodyFlags(world, body);

	// Body type affects the mass
	b3UpdateBodyMassData(world, body);

	world.locked = false;
}

export function b3Body_SetName(bodyId: b3BodyId, name: string): void {
	let world = b3GetWorld(bodyId.world0);

	let body = b3GetBodyFullId(world, bodyId);
	world.bodies[body].nameId = b3AddName(world.names, name);
}

export function b3Body_GetName(bodyId: b3BodyId): string {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return b3FindNameWithDefault(world.names, world.bodies[body].nameId, "");
}

export function b3Body_SetUserData(bodyId: b3BodyId, userData: u64): void {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	world.bodies[body].userData = userData;
}

export function b3Body_GetUserData(bodyId: b3BodyId): u64 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return world.bodies[body].userData;
}

export function b3Body_GetMass(bodyId: b3BodyId): f32 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return world.bodies[body].mass;
}

export function b3Body_GetLocalRotationalInertia(bodyId: b3BodyId): b3Matrix3 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return b3GetBodySim(world, body).inertia;
}

export function b3Body_GetInverseMass(bodyId: b3BodyId): f32 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return b3GetBodySim(world, body).invMass;
}

export function b3Body_GetWorldInverseRotationalInertia(bodyId: b3BodyId): b3Matrix3 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return b3GetBodySim(world, body).invInertiaWorld;
}

export function b3Body_GetLocalCenter(bodyId: b3BodyId): b3Vec3 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return b3GetBodySim(world, body).localCenter;
}

export function b3Body_GetWorldCenter(bodyId: b3BodyId): b3Pos {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return b3GetBodySim(world, body).center;
}

export function b3Body_SetMassData(bodyId: b3BodyId, massData: b3MassData): void {
	if (B3_ASSERT_ENABLED) {
		B3_ASSERT(b3IsValidFloat(massData.mass) && massData.mass >= 0.0);
		B3_ASSERT(b3IsValidMatrix3(massData.inertia));
		B3_ASSERT(b3IsValidVec3(massData.center));
	}

	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b3GetBodyFullId(world, bodyId);
	let set = world.solverSets[world.bodies[body].setIndex];
	let localIndex = world.bodies[body].localIndex;

	// Mass is no longer dirty
	world.bodies[body].flags &= ~b3_dirtyMass;
	b3SyncBodyFlags(world, body);

	// Copied after the flag sync and stored back before the inertia sync below.
	let bodySim = set.bodySims[localIndex];

	world.bodies[body].mass = massData.mass;
	bodySim.localCenter = massData.center;

	let oldCenter = bodySim.center;
	let center = b3TransformWorldPoint(bodySim.transform, massData.center);
	bodySim.center = center;
	bodySim.center0 = center;
	let bodyMass = world.bodies[body].mass;
	bodySim.invMass = bodyMass > 0.0 ? 1.0 / bodyMass : 0.0;
	bodySim.inertia = massData.inertia;

	// Update center of mass velocity
	let stateIndex = b3GetBodyState(world, body);
	if (stateIndex != B3_NULL_INDEX) {
		let states = world.solverSets[b3_awakeSet].bodyStates;
		let deltaLinear = b3Cross(states[stateIndex].angularVelocity, b3SubPos(bodySim.center, oldCenter));
		states[stateIndex].linearVelocity = b3Add(states[stateIndex].linearVelocity, deltaLinear);
	}

	let det = b3Det(bodySim.inertia);
	if (B3_ASSERT_ENABLED) B3_ASSERT(det >= 0.0);

	if (det > 0.0) {
		// This call is faster than b3Invert
		bodySim.invInertiaLocal = b3InvertT(bodySim.inertia);

		let rotationMatrix = b3MakeMatrixFromQuat(bodySim.transform.q);
		bodySim.invInertiaWorld = b3MulMM(b3MulMM(rotationMatrix, bodySim.invInertiaLocal), b3Transpose(rotationMatrix));
	} else {
		bodySim.invInertiaLocal = b3Mat3_zero;
		bodySim.invInertiaWorld = b3Mat3_zero;
	}

	// Apply fixed rotation
	if ((bodySim.flags & b3_fixedRotation) == b3_fixedRotation) {
		bodySim.inertia = b3Mat3_zero;
		bodySim.invInertiaLocal = b3Mat3_zero;
		bodySim.invInertiaWorld = b3Mat3_zero;
	}

	set.bodySims[localIndex] = bodySim;

	b3UpdateIsotropicInertia(world, body, bodySim);

	// Update extents using supplied mass center.
	let minExtent = B3_HUGE();
	let maxExtent = b3Vec3_zero;
	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B3_NULL_INDEX) {
		let s = world.shapes[shapeId];
		let extent = b3ComputeShapeExtent(s, massData.center);
		minExtent = b3MinFloat(minExtent, extent.minExtent);
		maxExtent = b3Max(maxExtent, extent.maxExtent);
		shapeId = s.nextShapeId;
	}
	set.bodySims[localIndex].minExtent = minExtent;
	set.bodySims[localIndex].maxExtent = maxExtent;

	// When the center of mass changes, any cached contact manifold becomes invalid.
	let edgeKey = world.bodies[body].headContactKey;
	while (edgeKey != B3_NULL_INDEX) {
		let contactId = edgeKey >> 1;
		let edgeIndex = edgeKey & 1;

		let contact = world.contacts[contactId];
		contact.flags &= ~b3_relativeTransformValid;

		edgeKey = contact.edges[edgeIndex].nextKey;
	}
}

export function b3Body_GetMassData(bodyId: b3BodyId): b3MassData {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	let bodySim = b3GetBodySim(world, body);
	let massData = b3MassData();
	massData.mass = world.bodies[body].mass;
	massData.center = bodySim.localCenter;
	massData.inertia = bodySim.inertia;
	return massData;
}

export function b3Body_ApplyMassFromShapes(bodyId: b3BodyId): void {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b3GetBodyFullId(world, bodyId);
	b3UpdateBodyMassData(world, body);
}

export function b3Body_SetLinearDamping(bodyId: b3BodyId, linearDamping: f32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidFloat(linearDamping) && linearDamping >= 0.0);

	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b3GetBodyFullId(world, bodyId);
	world.solverSets[world.bodies[body].setIndex].bodySims[world.bodies[body].localIndex].linearDamping = linearDamping;
}

export function b3Body_GetLinearDamping(bodyId: b3BodyId): f32 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return b3GetBodySim(world, body).linearDamping;
}

export function b3Body_SetAngularDamping(bodyId: b3BodyId, angularDamping: f32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidFloat(angularDamping) && angularDamping >= 0.0);

	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b3GetBodyFullId(world, bodyId);
	world.solverSets[world.bodies[body].setIndex].bodySims[world.bodies[body].localIndex].angularDamping = angularDamping;
}

export function b3Body_GetAngularDamping(bodyId: b3BodyId): f32 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return b3GetBodySim(world, body).angularDamping;
}

export function b3Body_SetGravityScale(bodyId: b3BodyId, gravityScale: f32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3Body_IsValid(bodyId));
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidFloat(gravityScale));

	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b3GetBodyFullId(world, bodyId);
	world.solverSets[world.bodies[body].setIndex].bodySims[world.bodies[body].localIndex].gravityScale = gravityScale;
}

export function b3Body_GetGravityScale(bodyId: b3BodyId): f32 {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3Body_IsValid(bodyId));
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return b3GetBodySim(world, body).gravityScale;
}

export function b3Body_IsAwake(bodyId: b3BodyId): bool {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return world.bodies[body].setIndex == b3_awakeSet;
}

export function b3Body_SetAwake(bodyId: b3BodyId, awake: bool): void {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	world.locked = true;

	let body = b3GetBodyFullId(world, bodyId);

	if (awake && world.bodies[body].setIndex >= b3_firstSleepingSet) {
		b3WakeBody(world, body);
	} else if (awake == false && world.bodies[body].setIndex == b3_awakeSet) {
		let island = world.islands[world.bodies[body].islandId];
		if (island.constraintRemoveCount > 0) {
			// Must split the island before sleeping. This is expensive.
			b3SplitIsland(world, world.bodies[body].islandId);
		}

		b3TrySleepIsland(world, world.bodies[body].islandId);
	}

	world.locked = false;
}

export function b3Body_IsEnabled(bodyId: b3BodyId): bool {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return world.bodies[body].setIndex != b3_disabledSet;
}

export function b3Body_IsSleepEnabled(bodyId: b3BodyId): bool {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return (world.bodies[body].flags & b3_enableSleep) == b3_enableSleep;
}

export function b3Body_SetSleepThreshold(bodyId: b3BodyId, sleepThreshold: f32): void {
	let world = b3GetWorld(bodyId.world0);

	let body = b3GetBodyFullId(world, bodyId);
	world.bodies[body].sleepThreshold = sleepThreshold;
}

export function b3Body_GetSleepThreshold(bodyId: b3BodyId): f32 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return world.bodies[body].sleepThreshold;
}

export function b3Body_SetSafetyFactor(bodyId: b3BodyId, safetyFactor: f32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidFloat(safetyFactor) && safetyFactor >= 0.0);

	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b3GetBodyFullId(world, bodyId);
	world.bodies[body].safetyFactor = safetyFactor;
}

export function b3Body_GetSafetyFactor(bodyId: b3BodyId): f32 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return world.bodies[body].safetyFactor;
}

export function b3Body_EnableSleep(bodyId: b3BodyId, enableSleep: bool): void {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b3GetBodyFullId(world, bodyId);

	let flag = (world.bodies[body].flags & b3_enableSleep) == b3_enableSleep;
	if (enableSleep == flag) {
		return;
	}

	world.locked = true;

	let flags = world.bodies[body].flags;
	world.bodies[body].flags = enableSleep ? flags | b3_enableSleep : flags & ~b3_enableSleep;
	b3SyncBodyFlags(world, body);

	if (enableSleep == false) {
		b3WakeBody(world, body);
	}

	world.locked = false;
}

// Disabling a body requires a lot of detailed bookkeeping, but it is a valuable feature.
// The most challenging aspect that joints may connect to bodies that are not disabled.
export function b3Body_Disable(bodyId: b3BodyId): void {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	world.locked = true;

	let body = b3GetBodyFullId(world, bodyId);
	if (world.bodies[body].setIndex == b3_disabledSet) {
		world.locked = false;
		return;
	}

	// Destroy contacts and wake bodies touching this body. This avoid floating bodies.
	// This is necessary even for static bodies.
	let wakeBodies = true;
	b3DestroyBodyContacts(world, body, wakeBodies);

	// The current solver set of the body
	let set = world.solverSets[world.bodies[body].setIndex];

	// Disabled bodies and connected joints are moved to the disabled set
	let disabledSet = world.solverSets[b3_disabledSet];

	// Unlink joints and transfer them to the disabled set
	let jointKey = world.bodies[body].headJointKey;
	while (jointKey != B3_NULL_INDEX) {
		let jointId = jointKey >> 1;
		let edgeIndex = jointKey & 1;

		jointKey = world.joints[jointId].edges[edgeIndex].nextKey;

		// joint may already be disabled by other body
		if (world.joints[jointId].setIndex == b3_disabledSet) {
			continue;
		}

		if (B3_ASSERT_ENABLED) {
			let jointSetIndex = world.joints[jointId].setIndex;
			B3_ASSERT(jointSetIndex == set.setIndex || set.setIndex == b3_staticSet || jointSetIndex == b3_staticSet);
		}

		// Remove joint from island
		b3UnlinkJoint(world, jointId);

		// Transfer joint to disabled set
		let jointSet = world.solverSets[world.joints[jointId].setIndex];
		b3TransferJoint(world, disabledSet, jointSet, jointId);
	}

	// Remove shapes from broad-phase
	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B3_NULL_INDEX) {
		let shape = world.shapes[shapeId];
		shapeId = shape.nextShapeId;
		b3DestroyShapeProxy(shape, world.broadPhase);
	}

	// Disabled bodies are not in an island. If the island becomes empty it will be destroyed.
	b3RemoveBodyFromIsland(world, body);

	// Transfer body sim
	b3TransferBody(world, disabledSet, set, body);

	world.locked = false;
}

export function b3Body_Enable(bodyId: b3BodyId): void {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b3GetBodyFullId(world, bodyId);
	if (world.bodies[body].setIndex != b3_disabledSet) {
		return;
	}

	let disabledSet = world.solverSets[b3_disabledSet];
	let setId = world.bodies[body].type == b3_staticBody ? b3_staticSet : b3_awakeSet;
	let targetSet = world.solverSets[setId];

	b3TransferBody(world, targetSet, disabledSet, body);

	let transform = b3GetBodyTransformQuick(world, body);

	// Add shapes to broad-phase
	let proxyType = world.bodies[body].type;
	let forcePairCreation = true;
	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B3_NULL_INDEX) {
		let shape = world.shapes[shapeId];
		shapeId = shape.nextShapeId;

		b3CreateShapeProxy(world, shape, proxyType, transform, forcePairCreation);
	}

	if (setId != b3_staticSet) {
		b3CreateIslandForBody(world, setId, body);
	}

	// Transfer joints. If the other body is disabled, don't transfer.
	// If the other body is sleeping, wake it.
	let jointKey = world.bodies[body].headJointKey;
	while (jointKey != B3_NULL_INDEX) {
		let jointId = jointKey >> 1;
		let edgeIndex = jointKey & 1;

		if (B3_ASSERT_ENABLED) B3_ASSERT(world.joints[jointId].setIndex == b3_disabledSet);
		if (B3_ASSERT_ENABLED) B3_ASSERT(world.joints[jointId].islandId == B3_NULL_INDEX);

		jointKey = world.joints[jointId].edges[edgeIndex].nextKey;

		let bodyA = world.joints[jointId].edges[0].bodyId;
		let bodyB = world.joints[jointId].edges[1].bodyId;

		if (world.bodies[bodyA].setIndex == b3_disabledSet || world.bodies[bodyB].setIndex == b3_disabledSet) {
			// one body is still disabled
			continue;
		}

		// Transfer joint first
		let jointSetId: i32;
		if (world.bodies[bodyA].type != b3_dynamicBody && world.bodies[bodyB].type != b3_dynamicBody) {
			jointSetId = b3_staticSet;
		} else if (world.bodies[bodyA].setIndex == b3_staticSet) {
			jointSetId = world.bodies[bodyB].setIndex;
		} else {
			jointSetId = world.bodies[bodyA].setIndex;
		}

		let jointSet = world.solverSets[jointSetId];
		b3TransferJoint(world, jointSet, disabledSet, jointId);

		// Now that the joint is in the correct set, I can link the joint in the island.
		if (jointSetId != b3_staticSet) {
			b3LinkJoint(world, jointId);
		}
	}
}

export function b3Body_SetMotionLocks(bodyId: b3BodyId, locks: b3MotionLocks): void {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	let newLocks: u32 = 0;
	newLocks |= locks.linearX ? b3_lockLinearX : 0;
	newLocks |= locks.linearY ? b3_lockLinearY : 0;
	newLocks |= locks.linearZ ? b3_lockLinearZ : 0;
	newLocks |= locks.angularX ? b3_lockAngularX : 0;
	newLocks |= locks.angularY ? b3_lockAngularY : 0;
	newLocks |= locks.angularZ ? b3_lockAngularZ : 0;

	let body = b3GetBodyFullId(world, bodyId);
	if ((world.bodies[body].flags & b3_allLocks) == newLocks) {
		return;
	}

	let fixedRotation1 = (world.bodies[body].flags & b3_fixedRotation) == b3_fixedRotation;
	let fixedRotation2 = (newLocks & b3_fixedRotation) == b3_fixedRotation;

	world.bodies[body].flags &= ~b3_allLocks;
	world.bodies[body].flags |= newLocks;

	b3SyncBodyFlags(world, body);

	let state = b3GetBodyState(world, body);

	if (state != B3_NULL_INDEX) {
		let states = world.solverSets[b3_awakeSet].bodyStates;
		if (locks.linearX) {
			states[state].linearVelocity.x = 0.0;
		}

		if (locks.linearY) {
			states[state].linearVelocity.y = 0.0;
		}

		if (locks.linearZ) {
			states[state].linearVelocity.z = 0.0;
		}

		if (locks.angularX) {
			states[state].angularVelocity.x = 0.0;
		}

		if (locks.angularY) {
			states[state].angularVelocity.y = 0.0;
		}

		if (locks.angularZ) {
			states[state].angularVelocity.z = 0.0;
		}
	}

	if (fixedRotation1 != fixedRotation2) {
		b3UpdateBodyMassData(world, body);
	}
}

export function b3Body_GetMotionLocks(bodyId: b3BodyId): b3MotionLocks {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	let flags = world.bodies[body].flags;

	let locks = b3MotionLocks();
	locks.linearX = (flags & b3_lockLinearX) != 0;
	locks.linearY = (flags & b3_lockLinearY) != 0;
	locks.linearZ = (flags & b3_lockLinearZ) != 0;
	locks.angularX = (flags & b3_lockAngularX) != 0;
	locks.angularY = (flags & b3_lockAngularY) != 0;
	locks.angularZ = (flags & b3_lockAngularZ) != 0;
	return locks;
}

export function b3Body_SetBullet(bodyId: b3BodyId, flag: bool): void {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	let newFlag: u32 = flag ? b3_isBullet : 0;

	let body = b3GetBodyFullId(world, bodyId);
	if ((world.bodies[body].flags & b3_isBullet) == newFlag) {
		return;
	}

	world.bodies[body].flags &= ~b3_isBullet;
	world.bodies[body].flags |= newFlag;

	b3SyncBodyFlags(world, body);
}

export function b3Body_IsBullet(bodyId: b3BodyId): bool {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return (world.bodies[body].flags & b3_isBullet) != 0;
}

export function b3Body_AllowFastRotation(bodyId: b3BodyId, flag: bool): void {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	let newFlag: u32 = flag ? b3_allowFastRotation : 0;

	let body = b3GetBodyFullId(world, bodyId);
	if ((world.bodies[body].flags & b3_allowFastRotation) == newFlag) {
		return;
	}

	world.bodies[body].flags &= ~b3_allowFastRotation;
	world.bodies[body].flags |= newFlag;

	b3SyncBodyFlags(world, body);
}

export function b3Body_IsFastRotationAllowed(bodyId: b3BodyId): bool {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return (world.bodies[body].flags & b3_allowFastRotation) != 0;
}

export function b3Body_EnableContactRecycling(bodyId: b3BodyId, flag: bool): void {
	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return;
	}

	let newFlag: u32 = flag ? b3_enableContactRecycling : 0;

	let body = b3GetBodyFullId(world, bodyId);
	if ((world.bodies[body].flags & b3_enableContactRecycling) == newFlag) {
		return;
	}

	world.bodies[body].flags &= ~b3_enableContactRecycling;
	world.bodies[body].flags |= newFlag;

	b3SyncBodyFlags(world, body);
}

export function b3Body_IsContactRecyclingEnabled(bodyId: b3BodyId): bool {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return (world.bodies[body].flags & b3_enableContactRecycling) != 0;
}

export function b3Body_EnableHitEvents(bodyId: b3BodyId, flag: bool): void {
	let world = b3GetWorld(bodyId.world0);

	let body = b3GetBodyFullId(world, bodyId);
	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B3_NULL_INDEX) {
		let shape = world.shapes[shapeId];
		shape.flags = flag ? shape.flags | b3_enableHitEvents : shape.flags & ~b3_enableHitEvents;
		shapeId = shape.nextShapeId;
	}
}

export function b3Body_GetWorld(bodyId: b3BodyId): b3WorldId {
	let world = b3GetWorld(bodyId.world0);
	let id = b3WorldId();
	id.index1 = <u16>(bodyId.world0 + 1);
	id.generation = world.generation;
	return id;
}

export function b3Body_GetShapeCount(bodyId: b3BodyId): i32 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return world.bodies[body].shapeCount;
}

export function b3Body_GetShapes(bodyId: b3BodyId, shapeArray: StaticArray<b3ShapeId>, capacity: i32): i32 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	let shapeId = world.bodies[body].headShapeId;
	let shapeCount = 0;
	while (shapeId != B3_NULL_INDEX && shapeCount < capacity) {
		let shape = world.shapes[shapeId];
		let id = b3MakeShapeIdFrom(shape.id + 1, bodyId.world0, shape.generation);
		shapeArray[shapeCount] = id;
		shapeCount += 1;

		shapeId = shape.nextShapeId;
	}

	return shapeCount;
}

export function b3Body_GetJointCount(bodyId: b3BodyId): i32 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return world.bodies[body].jointCount;
}

export function b3Body_GetJoints(bodyId: b3BodyId, jointArray: StaticArray<b3JointId>, capacity: i32): i32 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	let jointKey = world.bodies[body].headJointKey;

	let jointCount = 0;
	while (jointKey != B3_NULL_INDEX && jointCount < capacity) {
		let jointId = jointKey >> 1;
		let edgeIndex = jointKey & 1;

		let id = b3JointId();
		id.index1 = jointId + 1;
		id.world0 = bodyId.world0;
		id.generation = world.joints[jointId].generation;
		jointArray[jointCount] = id;
		jointCount += 1;

		jointKey = world.joints[jointId].edges[edgeIndex].nextKey;
	}

	return jointCount;
}

export function b3ShouldBodiesCollide(world: b3World, bodyIdA: i32, bodyIdB: i32): bool {
	let bodies = world.bodies;
	if (bodies[bodyIdA].type != b3_dynamicBody && bodies[bodyIdB].type != b3_dynamicBody) {
		return false;
	}

	let jointKey: i32;
	let otherBodyId: i32;
	if (bodies[bodyIdA].jointCount < bodies[bodyIdB].jointCount) {
		jointKey = bodies[bodyIdA].headJointKey;
		otherBodyId = bodies[bodyIdB].id;
	} else {
		jointKey = bodies[bodyIdB].headJointKey;
		otherBodyId = bodies[bodyIdA].id;
	}

	while (jointKey != B3_NULL_INDEX) {
		let jointId = jointKey >> 1;
		let edgeIndex = jointKey & 1;
		let otherEdgeIndex = edgeIndex ^ 1;

		if (world.joints[jointId].collideConnected == false && world.joints[jointId].edges[otherEdgeIndex].bodyId == otherBodyId) {
			return false;
		}

		jointKey = world.joints[jointId].edges[edgeIndex].nextKey;
	}

	return true;
}

export function b3Body_GetMinExtent(bodyId: b3BodyId): f32 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return b3GetBodySim(world, body).minExtent;
}

export function b3Body_GetMaxExtent(bodyId: b3BodyId): b3Vec3 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	return b3GetBodySim(world, body).maxExtent;
}

export function b3Body_GetMaxExtentOrigin(bodyId: b3BodyId): b3Vec3 {
	let world = b3GetWorld(bodyId.world0);
	let body = b3GetBodyFullId(world, bodyId);
	let bodySim = b3GetBodySim(world, body);
	return b3Add(bodySim.maxExtent, b3Abs(bodySim.localCenter));
}

function b3MakeShapeIdFrom(index1: i32, world0: u16, generation: u16): b3ShapeId {
	let id = b3ShapeId();
	id.index1 = index1;
	id.world0 = world0;
	id.generation = generation;
	return id;
}

function b3MakeContactIdFrom(index1: i32, world0: u16, generation: u32): b3ContactId {
	let id = b3ContactId();
	id.index1 = index1;
	id.world0 = world0;
	id.padding = 0;
	id.generation = generation;
	return id;
}
