// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/contact.c

import { b3AABB, b3Quat, b3Transform, b3Vec3 } from "./math_functions";
import { b3Manifold, b3SATCache, b3SimplexCache } from "./collision";

export const B3_FORCE_GHOST_COLLISIONS: i32 = 0;

// The C union becomes one field per member; they do not alias. A contact only ever uses the member
// matching its shape B type.
export struct b3ContactCache {
	satCache: b3SATCache = b3SATCache();
	simplexCache: b3SimplexCache = b3SimplexCache();
}

export struct b3TriangleCache {
	triangleIndex: i32 = 0;
	cache: b3ContactCache = b3ContactCache();

	constructor(triangleIndex: i32, cache: b3ContactCache) {
		this.triangleIndex = triangleIndex;
		this.cache = cache;
	}
}

export type b3ContactFlags = u32;

// Set when the solid shapes are touching.
export const b3_contactTouchingFlag: b3ContactFlags = 0x00000001;

// Contact has a hit event
export const b3_contactHitEventFlag: b3ContactFlags = 0x00000002;

// This contact wants contact events
export const b3_contactEnableContactEvents: b3ContactFlags = 0x00000004;

// This is contact is between a dynamic and static body
export const b3_contactStaticFlag: b3ContactFlags = 0x00000008;

export const b3_contactRecycleFlag: b3ContactFlags = 0x00000010;

// Set when the shapes are touching
export const b3_simTouchingFlag: b3ContactFlags = 0x00010000;

// This contact no longer has overlapping AABBs
export const b3_simDisjoint: b3ContactFlags = 0x00020000;

// This contact started touching
export const b3_simStartedTouching: b3ContactFlags = 0x00040000;

// This contact stopped touching
export const b3_simStoppedTouching: b3ContactFlags = 0x00080000;

// This contact has a hit event
export const b3_simEnableHitEvent: b3ContactFlags = 0x00100000;

// This contact wants pre-solve events
export const b3_simEnablePreSolveEvents: b3ContactFlags = 0x00200000;

// This is a mesh contact
export const b3_simMeshContact: b3ContactFlags = 0x00400000;

// Relative transform is cached for contact recycling
export const b3_relativeTransformValid: b3ContactFlags = 0x00800000;

// Enable speculative contact points
export const b3_enableSpeculativePoints: b3ContactFlags = 0x01000000;

// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// edges, one for each attached body.
export struct b3ContactEdge {
	bodyId: i32 = 0;
	prevKey: i32 = 0;
	nextKey: i32 = 0;
}

export class b3MeshContact {
	triangleCache: Array<b3TriangleCache> = new Array<b3TriangleCache>();
	queryBounds: b3AABB = b3AABB();
}

export struct b3ConvexContact {
	cache: b3ContactCache = b3ContactCache();
}

// Shared by every contact without manifolds, where C stores NULL. It has no elements to write.
export const b3_emptyManifolds = new StaticArray<b3Manifold>(0);

// Represents the persistent interaction between two shapes
export class b3Contact {
	// index of simulation set stored in b3World
	// B3_NULL_INDEX when slot is free
	setIndex: i32 = 0;

	// index into the constraint graph color array
	// B3_NULL_INDEX for non-touching or sleeping contacts
	// B3_NULL_INDEX when slot is free
	colorIndex: i32 = 0;

	// contact index within set or graph color
	// B3_NULL_INDEX when slot is free
	localIndex: i32 = 0;

	edges: FixedArray<b3ContactEdge, 2>;
	shapeIdA: i32 = 0;
	shapeIdB: i32 = 0;
	childIndex: i32 = 0;

	// A contact only belongs to an island if touching, otherwise B3_NULL_INDEX.
	islandId: i32 = 0;

	// Index into the island's contacts array for O(1) swap-removal.
	// B3_NULL_INDEX when not in an island.
	islandIndex: i32 = 0;

	// Back index into b3World::contacts
	contactId: i32 = 0;

	// Encoded body sim indices to avoid a cache miss accessing the body in the narrow phase.
	encodedBodySimA: i32 = 0;
	encodedBodySimB: i32 = 0;

	// b3ContactFlags
	flags: u32 = 0;

	// todo embed single manifold with ability to allocate overfloat, use union.
	manifolds: StaticArray<b3Manifold> = b3_emptyManifolds;
	manifoldCount: i32 = 0;

	// Cache for contact recycling.
	cachedRotationA: b3Quat = b3Quat();
	cachedRotationB: b3Quat = b3Quat();
	cachedRelativePose: b3Transform = b3Transform();

	// Mixed friction and restitution
	friction: f32 = 0;

	// Usage determined by b3_simMeshContact in simFlags
	// The C union becomes one field per member.
	convexContact: b3ConvexContact = b3ConvexContact();
	meshContact: b3MeshContact = new b3MeshContact();

	restitution: f32 = 0;
	rollingResistance: f32 = 0;
	tangentVelocity: b3Vec3 = b3Vec3();

	// This is monotonically advanced when a contact is allocated in this slot
	// Used to check for invalid b3ContactId
	generation: u32 = 0;
}

export struct b3ContactSpec {
	contactId: i32 = 0;

	// Start of the global manifold constraint array
	manifoldStart: i32 = 0;
	manifoldCount: u16 = 0;
}

// functions from contact.c follow
