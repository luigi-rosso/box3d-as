// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/body.c

import { B3_NULL_INDEX } from "./core";
import { b3Matrix3, b3Pos, b3Quat, b3SubPos, b3Transform, b3Vec3, b3WorldTransform } from "./math_functions";
import { b3Sweep } from "./collision";
import { b3BodyType } from "./types";
import { b3_awakeSet, b3_staticSet } from "./physics_world";

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
