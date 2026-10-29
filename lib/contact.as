// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/contact.c

import { Parallel } from "rive/parallel";
import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX, UINT32_MAX } from "./core";
import { B3_MAX_MANIFOLD_POINTS, B3_MAX_WORKERS } from "./constants";
import {
	b3AABB,
	b3Add,
	b3InvMulWorldTransforms,
	b3MakeMatrixFromQuat,
	b3MaxFloat,
	b3MulMV,
	b3MulWorldTransforms,
	b3Neg,
	b3OffsetPos,
	b3Quat,
	b3RotateVector,
	b3Sub,
	b3SubPos,
	b3Transform,
	b3Vec3,
	b3WorldTransform,
} from "./math_functions";
import { b3ChildShape, b3LocalManifold, b3LocalManifoldPoint, b3Manifold, b3ManifoldPoint, b3SATCache, b3SimplexCache } from "./collision";
import {
	b3ContactData,
	b3ContactEndTouchEvent,
	b3ContactId,
	b3ShapeId,
	b3ShapeType,
	b3SurfaceMaterial,
	b3_capsuleShape,
	b3_compoundShape,
	b3_heightShape,
	b3_hullShape,
	b3_meshShape,
	b3_shapeTypeCount,
	b3_sphereShape,
	b3_staticBody,
} from "./types";
import { b3AllocId, b3FreeId } from "./id_pool";
import { b3Array_RemoveSwap } from "./container";
import { b3AddKey, b3RemoveKey, b3ShapePairKey } from "./table";
import { b3EncodeBodySimIndex, b3WakeBody, b3_enableContactRecycling } from "./body";
import { b3GetCompoundChildInto, b3GetCompoundChildType } from "./compound";
import { b3RemoveContactFromGraph } from "./constraint_graph";
import {
	b3CollideCapsuleAndSphere,
	b3CollideCapsules,
	b3CollideHullAndCapsule,
	b3CollideHullAndSphere,
	b3CollideHullsAtWidth,
	b3CollideSpheres,
	b3ZeroLocalManifold,
} from "./convex_manifold";
import { b3UnlinkContact } from "./island";
import { b3MakeFeatureId } from "./manifold";
import { b3StoreRow } from "./simd";
import { b3ComputeMeshManifoldsAt } from "./mesh_contact";
import {
	b3AllocateManifolds,
	b3FreeManifolds,
	b3GetWorld,
	b3MixFriction,
	b3MixRestitution,
	b3World,
	b3_awakeSet,
	b3_disabledSet,
	b3_staticSet,
} from "./physics_world";
import {
	b3GetShapeMaterials,
	b3Shape,
	b3_enableContactEvents,
	b3_enableHitEvents,
	b3_enablePreSolveEvents,
	b3_enableSpeculative,
} from "./shape";

export const B3_FORCE_GHOST_COLLISIONS: i32 = 0;

// Manifold point writes as 16 byte rows (b3StoreRow). The point fields run anchorA, anchorB, separation, normalImpulse,
// totalNormalImpulse, normalVelocity, baseSeparation, featureId, so the rows below follow that order.
export const MP_ANCHORA = offsetof<b3ManifoldPoint>("anchorA");
export const MP_ANCHORB = offsetof<b3ManifoldPoint>("anchorB");
export const MP_SEPARATION = offsetof<b3ManifoldPoint>("separation");
export const MP_NORMALIMPULSE = offsetof<b3ManifoldPoint>("normalImpulse");
export const MP_TOTALNORMALIMPULSE = offsetof<b3ManifoldPoint>("totalNormalImpulse");
export const MP_BASESEPARATION = offsetof<b3ManifoldPoint>("baseSeparation");
export const MP_TRIANGLEINDEX = offsetof<b3ManifoldPoint>("triangleIndex");
export const MP_PERSISTED = offsetof<b3ManifoldPoint>("persisted");

// The address of point pointIndex of manifold manifoldIndex; both are below the counts the caller holds
@inline export unsafe function b3ManifoldPointAddress(manifolds: StaticArray<b3Manifold>, manifoldIndex: i32, pointIndex: i32): usize {
	return (
		changetype<usize>(manifolds) +
		<usize>manifoldIndex * sizeof<b3Manifold>() +
		offsetof<b3Manifold>("points") +
		<usize>pointIndex * sizeof<b3ManifoldPoint>()
	);
}

// anchorA, anchorB, separation and normalImpulse of the point at mp
@inline export unsafe function b3StorePointAnchors(mp: usize, anchorA: b3Vec3, anchorB: b3Vec3, separation: f32, normalImpulse: f32): void {
	b3StoreRow(mp + MP_ANCHORA, anchorA.x, anchorA.y, anchorA.z, anchorB.x);
	b3StoreRow(mp + MP_ANCHORB + 4, anchorB.y, anchorB.z, separation, normalImpulse);
}

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
	// The solver gathers friction, restitution and rolling resistance as one row, so they stay in this order.
	friction: f32 = 0;
	restitution: f32 = 0;
	rollingResistance: f32 = 0;
	tangentVelocity: b3Vec3 = b3Vec3();

	// Usage determined by b3_simMeshContact in simFlags
	// The C union becomes one field per member.
	convexContact: b3ConvexContact = b3ConvexContact();
	meshContact: b3MeshContact = new b3MeshContact();

	// This is monotonically advanced when a contact is allocated in this slot
	// Used to check for invalid b3ContactId
	generation: u32 = 0;
}

// The zeroed contact of C's (b3Contact){ 0 }, with the mesh contact's cache emptied in place
function b3ResetContact(contact: b3Contact): void {
	contact.setIndex = 0;
	contact.colorIndex = 0;
	contact.localIndex = 0;
	contact.edges[0] = b3ContactEdge();
	contact.edges[1] = b3ContactEdge();
	contact.shapeIdA = 0;
	contact.shapeIdB = 0;
	contact.childIndex = 0;
	contact.islandId = 0;
	contact.islandIndex = 0;
	contact.contactId = 0;
	contact.encodedBodySimA = 0;
	contact.encodedBodySimB = 0;
	contact.flags = 0;
	contact.manifolds = b3_emptyManifolds;
	contact.manifoldCount = 0;
	contact.cachedRotationA = b3Quat();
	contact.cachedRotationB = b3Quat();
	contact.cachedRelativePose = b3Transform();
	contact.friction = 0.0;
	contact.restitution = 0.0;
	contact.rollingResistance = 0.0;
	contact.tangentVelocity = b3Vec3();
	contact.convexContact = b3ConvexContact();
	contact.meshContact.triangleCache.length = 0;
	contact.meshContact.queryBounds = b3AABB();
	contact.generation = 0;
}

export struct b3ContactSpec {
	contactId: i32 = 0;

	// Start of the global manifold constraint array
	manifoldStart: i32 = 0;
	manifoldCount: u16 = 0;
}

// functions from contact.c follow

// Contacts and determinism
// A deterministic simulation requires contacts to exist in the same order in b3Island no matter the thread count.
// The order must reproduce from run to run. This is necessary because the Gauss-Seidel constraint solver is order dependent.
//
// Creation:
// - Contacts are created using results from b3UpdateBroadPhasePairs
// - These results are ordered according to the order of the broad-phase move array
// - The move array is ordered according to the shape creation order using a bitset.
// - The island/shape/body order is determined by creation order
// - Logically contacts are only created for awake bodies, so they are immediately added to the awake contact array (serially)
//
// Island linking:
// - The awake contact array is built from the body-contact graph for all awake bodies in awake islands.
// - Awake contacts are solved in parallel and they generate contact state changes.
// - These state changes may link islands together using union find.
// - The state changes are ordered using a bit array that encompasses all contacts
// - As long as contacts are created in deterministic order, island link order is deterministic.
// - This keeps the order of contacts in islands deterministic

// Manifold functions should compute important results in local space to improve precision. However, this
// interface function takes two world transforms instead of a relative transform for these reasons:
//
// First:
// The anchors need to be computed relative to the shape origin in world space. This is necessary so the
// solver does not need to access static body transforms. Not even in constraint preparation. This approach
// has world space vectors yet retains precision.
//
// Second:
// b3ManifoldPoint::point is very useful for debugging and it is in world space.
//
// Third:
// The user may call the manifold functions directly and they should be easy to use and have easy to use
// results.

function b3GetContactFullId(world: b3World, contactId: b3ContactId): b3Contact {
	let id = contactId.index1 - 1;
	let contact = world.contacts[id];
	B3_ASSERT(contact.contactId == id && contact.generation == contactId.generation);
	return contact;
}

export function b3Contact_GetData(contactId: b3ContactId): b3ContactData {
	let world = b3GetWorld(<i32>contactId.world0);
	let contact = b3GetContactFullId(world, contactId);

	let shapeA = world.shapes[contact.shapeIdA];
	let shapeB = world.shapes[contact.shapeIdB];

	let data = new b3ContactData();
	data.contactId = contactId;
	data.shapeIdA = b3MakeShapeIdInternal(shapeA.id + 1, contactId.world0, shapeA.generation);
	data.shapeIdB = b3MakeShapeIdInternal(shapeB.id + 1, contactId.world0, shapeB.generation);

	if (contact.manifoldCount > 0) {
		data.manifolds = contact.manifolds;
		data.manifoldCount = contact.manifoldCount;
	} else {
		data.manifolds = null;
		data.manifoldCount = 0;
	}

	return data;
}

// C builds these ids with compound literals.
@inline function b3MakeShapeIdInternal(index1: i32, world0: u16, generation: u16): b3ShapeId {
	let id = b3ShapeId();
	id.index1 = index1;
	id.world0 = world0;
	id.generation = generation;
	return id;
}

struct b3ContactRegister {
	supported: bool = false;
	primary: bool = false;
}

// s_registers[type1][type2] is element type1 * b3_shapeTypeCount + type2.
const s_registers = new StaticArray<b3ContactRegister>(b3_shapeTypeCount * b3_shapeTypeCount);
let s_initialized = false;

function b3AddType(type1: b3ShapeType, type2: b3ShapeType): void {
	B3_ASSERT(0 <= type1 && type1 < b3_shapeTypeCount);
	B3_ASSERT(0 <= type2 && type2 < b3_shapeTypeCount);

	s_registers[type1 * b3_shapeTypeCount + type2].supported = true;
	s_registers[type1 * b3_shapeTypeCount + type2].primary = true;

	if (type1 != type2) {
		s_registers[type2 * b3_shapeTypeCount + type1].supported = true;
		s_registers[type2 * b3_shapeTypeCount + type1].primary = false;
	}
}

export function b3InitializeContactRegisters(): void {
	if (s_initialized == false) {
		b3AddType(b3_sphereShape, b3_sphereShape);
		b3AddType(b3_capsuleShape, b3_sphereShape);
		b3AddType(b3_capsuleShape, b3_capsuleShape);
		b3AddType(b3_compoundShape, b3_sphereShape);
		b3AddType(b3_compoundShape, b3_capsuleShape);
		b3AddType(b3_compoundShape, b3_hullShape);
		b3AddType(b3_hullShape, b3_sphereShape);
		b3AddType(b3_hullShape, b3_capsuleShape);
		b3AddType(b3_hullShape, b3_hullShape);
		b3AddType(b3_meshShape, b3_sphereShape);
		b3AddType(b3_meshShape, b3_capsuleShape);
		b3AddType(b3_meshShape, b3_hullShape);
		b3AddType(b3_heightShape, b3_sphereShape);
		b3AddType(b3_heightShape, b3_capsuleShape);
		b3AddType(b3_heightShape, b3_hullShape);
		s_initialized = true;
	}
}

export function b3CreateContact(world: b3World, shapeA: b3Shape, shapeB: b3Shape, childIndex: i32): void {
	let typeA = shapeA.type;
	let typeB = shapeB.type;

	B3_ASSERT(0 <= typeA && typeA < b3_shapeTypeCount);
	B3_ASSERT(0 <= typeB && typeB < b3_shapeTypeCount);

	if (s_registers[typeA * b3_shapeTypeCount + typeB].supported == false) {
		// For example, no mesh vs mesh collision
		return;
	}

	if (s_registers[typeA * b3_shapeTypeCount + typeB].primary == false) {
		// flip order
		b3CreateContact(world, shapeB, shapeA, childIndex);
		return;
	}

	// Copies for reading; the head keys and counts are written through world.bodies.
	let bodyIdA = shapeA.bodyId;
	let bodyIdB = shapeB.bodyId;
	let bodyA = world.bodies[bodyIdA];
	let bodyB = world.bodies[bodyIdB];

	B3_ASSERT(bodyA.setIndex != b3_disabledSet && bodyB.setIndex != b3_disabledSet);
	B3_ASSERT(bodyA.setIndex != b3_staticSet || bodyB.setIndex != b3_staticSet);

	let setIndex: i32;
	if (bodyA.setIndex == b3_awakeSet || bodyB.setIndex == b3_awakeSet) {
		setIndex = b3_awakeSet;
	} else {
		// sleeping and non-touching contacts live in the disabled set
		// later if this set is found to be touching then the sleeping
		// islands will be linked and the contact moved to the merged island

		// This is possible if a shape moves slightly then falls asleep
		setIndex = b3_disabledSet;
	}

	let set = world.solverSets[setIndex];

	// Create contact key and contact
	let contactId = b3AllocId(world.contactIdPool);
	if (contactId == world.contacts.length) {
		world.contacts.push(new b3Contact());
	}

	let shapeIdA = shapeA.id;
	let shapeIdB = shapeB.id;

	// *contact = (b3Contact){ 0 } keeping the generation, in place as C reuses the slot
	let contact = world.contacts[contactId];
	let generation = contact.generation;
	b3ResetContact(contact);
	contact.contactId = contactId;
	contact.generation = generation + 1;
	contact.setIndex = setIndex;
	contact.colorIndex = B3_NULL_INDEX;
	contact.localIndex = set.contactIndices.length;
	contact.islandId = B3_NULL_INDEX;
	contact.islandIndex = B3_NULL_INDEX;
	contact.shapeIdA = shapeIdA;
	contact.shapeIdB = shapeIdB;
	contact.childIndex = childIndex;
	contact.encodedBodySimA = b3EncodeBodySimIndex(bodyA);
	contact.encodedBodySimB = b3EncodeBodySimIndex(bodyB);

	if (shapeA.type == b3_meshShape || shapeA.type == b3_heightShape) {
		contact.flags |= b3_simMeshContact;
	} else if (shapeA.type == b3_compoundShape) {
		if (b3GetCompoundChildType(shapeA.compound!, childIndex) == b3_meshShape) {
			contact.flags |= b3_simMeshContact;
		}
	}

	// A convex contact keeps its one manifold for life, so a parallel part never allocates one
	if ((contact.flags & b3_simMeshContact) == 0) {
		contact.manifolds = b3AllocateManifolds(world, 1);
	}

	// todo impose these restrictions to make life easier
	B3_ASSERT(shapeB.type == b3_sphereShape || shapeB.type == b3_capsuleShape || shapeB.type == b3_hullShape);
	// B3_ASSERT( bodyB->type != b3_staticBody );

	// Is either body static?
	// Note: it is possible to have a dynamic mesh collide with a static convex shape. Maybe I should disallow this.
	if (bodyA.type == b3_staticBody || bodyB.type == b3_staticBody) {
		contact.flags |= b3_contactStaticFlag;
	}

	B3_ASSERT(shapeA.sensorIndex == B3_NULL_INDEX && shapeB.sensorIndex == B3_NULL_INDEX);

	if ((shapeA.flags & b3_enableContactEvents) != 0 || (shapeB.flags & b3_enableContactEvents) != 0) {
		contact.flags |= b3_contactEnableContactEvents;
	}

	if ((shapeA.flags & b3_enableSpeculative) != 0 && (shapeB.flags & b3_enableSpeculative) != 0) {
		contact.flags |= b3_enableSpeculativePoints;
	}

	// Disabling speculative collision also disables recycling.
	if (
		(bodyA.flags & b3_enableContactRecycling) != 0 &&
		(bodyB.flags & b3_enableContactRecycling) != 0 &&
		(contact.flags & b3_enableSpeculativePoints) != 0
	) {
		contact.flags |= b3_contactRecycleFlag;
	}

	// Connect to body A
	{
		contact.edges[0].bodyId = shapeA.bodyId;
		contact.edges[0].prevKey = B3_NULL_INDEX;
		contact.edges[0].nextKey = world.bodies[bodyIdA].headContactKey;

		let keyA = (contactId << 1) | 0;
		let headContactKey = world.bodies[bodyIdA].headContactKey;
		if (headContactKey != B3_NULL_INDEX) {
			let headContact = world.contacts[headContactKey >> 1];
			headContact.edges[headContactKey & 1].prevKey = keyA;
		}
		world.bodies[bodyIdA].headContactKey = keyA;
		world.bodies[bodyIdA].contactCount += 1;
	}

	// Connect to body B
	{
		contact.edges[1].bodyId = shapeB.bodyId;
		contact.edges[1].prevKey = B3_NULL_INDEX;
		contact.edges[1].nextKey = world.bodies[bodyIdB].headContactKey;

		let keyB = (contactId << 1) | 1;
		let headContactKey = world.bodies[bodyIdB].headContactKey;
		if (world.bodies[bodyIdB].headContactKey != B3_NULL_INDEX) {
			let headContact = world.contacts[headContactKey >> 1];
			headContact.edges[headContactKey & 1].prevKey = keyB;
		}
		world.bodies[bodyIdB].headContactKey = keyB;
		world.bodies[bodyIdB].contactCount += 1;
	}

	// Add to pair set for fast lookup
	let pairKey = b3ShapePairKey(shapeIdA, shapeIdB, childIndex);
	b3AddKey(world.broadPhase.pairSet, pairKey);

	// Contacts are created as non-touching. Later if they are found to be touching
	// they will link islands and be moved into the constraint graph.
	set.contactIndices.push(contactId);

	let radiusA: f32 = 0.0;
	if (typeA == b3_sphereShape) {
		radiusA = shapeA.sphere.radius;
	} else if (typeA == b3_capsuleShape) {
		radiusA = shapeA.capsule.radius;
	}

	let radiusB: f32 = 0.0;
	if (typeB == b3_sphereShape) {
		radiusB = shapeB.sphere.radius;
	} else if (typeB == b3_capsuleShape) {
		radiusB = shapeB.capsule.radius;
	}

	let maxRadius = b3MaxFloat(radiusA, radiusB);

	// Assuming the rolling resistance doesn't change
	contact.rollingResistance =
		b3MaxFloat(b3GetShapeMaterials(shapeA)[0].rollingResistance, b3GetShapeMaterials(shapeB)[0].rollingResistance) *
		maxRadius;

	if ((shapeA.flags & b3_enablePreSolveEvents) != 0 || (shapeB.flags & b3_enablePreSolveEvents) != 0) {
		contact.flags |= b3_simEnablePreSolveEvents;
	}
}

// A contact is destroyed when:
// - broad-phase proxies stop overlapping
// - a body is destroyed
// - a body is disabled
// - a body changes type from dynamic to kinematic or static
// - a shape is destroyed
// - contact filtering is modified
export function b3DestroyContact(world: b3World, contact: b3Contact, wakeBodies: bool): void {
	// Remove pair from set
	let pairKey = b3ShapePairKey(contact.shapeIdA, contact.shapeIdB, contact.childIndex);
	b3RemoveKey(world.broadPhase.pairSet, pairKey);

	b3FreeManifolds(world, contact.manifolds, contact.manifoldCount);
	contact.manifolds = b3_emptyManifolds;
	contact.manifoldCount = 0;

	ref edgeA = contact.edges[0];
	ref edgeB = contact.edges[1];

	let bodyIdA = edgeA.bodyId;
	let bodyIdB = edgeB.bodyId;

	let flags = contact.flags;
	let touching = (flags & b3_contactTouchingFlag) != 0;

	// End touch event
	if (touching && (flags & b3_contactEnableContactEvents) != 0) {
		let worldId = world.worldId;
		let shapeA = world.shapes[contact.shapeIdA];
		let shapeB = world.shapes[contact.shapeIdB];
		let shapeIdA = b3MakeShapeIdInternal(shapeA.id + 1, worldId, shapeA.generation);
		let shapeIdB = b3MakeShapeIdInternal(shapeB.id + 1, worldId, shapeB.generation);

		let contactId = b3ContactId();
		contactId.index1 = contact.contactId + 1;
		contactId.world0 = world.worldId;
		contactId.padding = 0;
		contactId.generation = contact.generation;

		let event = b3ContactEndTouchEvent();
		event.shapeIdA = shapeIdA;
		event.shapeIdB = shapeIdB;
		event.contactId = contactId;

		world.contactEndEvents[world.endEventArrayIndex].push(event);
	}

	// Remove from body A
	if (edgeA.prevKey != B3_NULL_INDEX) {
		let prevContact = world.contacts[edgeA.prevKey >> 1];
		prevContact.edges[edgeA.prevKey & 1].nextKey = edgeA.nextKey;
	}

	if (edgeA.nextKey != B3_NULL_INDEX) {
		let nextContact = world.contacts[edgeA.nextKey >> 1];
		nextContact.edges[edgeA.nextKey & 1].prevKey = edgeA.prevKey;
	}

	let contactId = contact.contactId;

	let edgeKeyA = (contactId << 1) | 0;
	if (world.bodies[bodyIdA].headContactKey == edgeKeyA) {
		world.bodies[bodyIdA].headContactKey = edgeA.nextKey;
	}

	world.bodies[bodyIdA].contactCount -= 1;

	// Remove from body B
	if (edgeB.prevKey != B3_NULL_INDEX) {
		let prevContact = world.contacts[edgeB.prevKey >> 1];
		prevContact.edges[edgeB.prevKey & 1].nextKey = edgeB.nextKey;
	}

	if (edgeB.nextKey != B3_NULL_INDEX) {
		let nextContact = world.contacts[edgeB.nextKey >> 1];
		nextContact.edges[edgeB.nextKey & 1].prevKey = edgeB.prevKey;
	}

	let edgeKeyB = (contactId << 1) | 1;
	if (world.bodies[bodyIdB].headContactKey == edgeKeyB) {
		world.bodies[bodyIdB].headContactKey = edgeB.nextKey;
	}

	world.bodies[bodyIdB].contactCount -= 1;

	if ((contact.flags & b3_simMeshContact) != 0) {
		// C destroys the cache array; the slot keeps its storage for the next contact
		contact.meshContact.triangleCache.length = 0;
	}

	// Remove contact from the array that owns it
	if (contact.islandId != B3_NULL_INDEX) {
		b3UnlinkContact(world, contact);
	}

	if (contact.colorIndex != B3_NULL_INDEX) {
		// contact is an active constraint
		B3_ASSERT(contact.setIndex == b3_awakeSet);
		let meshContact = (contact.flags & b3_simMeshContact) != 0;
		b3RemoveContactFromGraph(world, bodyIdA, bodyIdB, contact.colorIndex, contact.localIndex, meshContact);
	} else {
		// contact is non-touching or is sleeping or is a sensor
		B3_ASSERT(contact.setIndex != b3_awakeSet || (contact.flags & b3_contactTouchingFlag) == 0);
		let set = world.solverSets[contact.setIndex];

		let localIndex = contact.localIndex;
		let movedIndex = b3Array_RemoveSwap(set.contactIndices, localIndex);
		if (movedIndex != B3_NULL_INDEX) {
			let movedContactIndex = set.contactIndices[localIndex];
			let movedContact = world.contacts[movedContactIndex];
			movedContact.localIndex = localIndex;
		}
	}

	// Free contact and id (preserve generation)
	contact.contactId = B3_NULL_INDEX;
	contact.setIndex = B3_NULL_INDEX;
	contact.colorIndex = B3_NULL_INDEX;
	contact.localIndex = B3_NULL_INDEX;
	b3FreeId(world.contactIdPool, contactId);

	if (wakeBodies && touching) {
		b3WakeBody(world, bodyIdA);
		b3WakeBody(world, bodyIdB);
	}
}

// C's stack buffers for b3ComputeConvexManifold, one per worker since parallel parts collide at once, and for the
// compound child of b3UpdateContact, which only the main lane updates. Neither call nests in itself.
class b3ConvexContactScratch {
	points: StaticArray<b3LocalManifoldPoint> = new StaticArray<b3LocalManifoldPoint>(B3_MAX_MANIFOLD_POINTS);
	manifold: b3LocalManifold = new b3LocalManifold();

	constructor() {
		this.manifold.points = this.points;
	}
}

function b3MakeConvexContactScratch(): StaticArray<b3ConvexContactScratch> {
	let scratch = new StaticArray<b3ConvexContactScratch>(B3_MAX_WORKERS);
	for (let i = 0; i < B3_MAX_WORKERS; ++i) {
		scratch[i] = new b3ConvexContactScratch();
	}
	return scratch;
}

// One per worker, and each part uses its worker's
@parallel.shared const s_convexContactScratch = b3MakeConvexContactScratch();

// Temporary child shape to match existing function signatures. C copies the whole shape; the paths below read only its
// identity, flags, type, materials and the child geometry, so those are what is copied. childMaterials holds a convex
// child's one material.
function b3SetCompoundChildShape(
	childShapeA: b3Shape,
	shapeA: b3Shape,
	child: b3ChildShape,
	childMaterials: StaticArray<b3SurfaceMaterial>,
): void {
	childShapeA.id = shapeA.id;
	childShapeA.generation = shapeA.generation;
	childShapeA.flags = shapeA.flags;
	childShapeA.materialCount = shapeA.materialCount;
	childShapeA.materials = shapeA.materials;

	childShapeA.type = child.type;

	// Handle child material for non-meshes.
	if (child.type != b3_meshShape) {
		let materialIndex = <i32>child.materialIndices![0];
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= materialIndex && materialIndex < shapeA.materialCount);
		let parentMaterials = b3GetShapeMaterials(shapeA);
		childMaterials[0] = parentMaterials[materialIndex];
		childShapeA.materials = childMaterials;
		childShapeA.materialCount = 1;
	}

	if (child.type == b3_capsuleShape) {
		childShapeA.capsule = child.capsule;
	} else if (child.type == b3_hullShape) {
		childShapeA.hull = child.hull;
	} else if (child.type == b3_meshShape) {
		childShapeA.mesh = child.mesh;
	} else {
		childShapeA.sphere = child.sphere;
	}
}

// Whether the identity, flags and materials compound children copy from their shape are as they copied them
unsafe function b3CompoundParentUnchanged(shape: b3Shape): bool {
	let materials = shape.materials;
	let copy = shape.compoundParentMaterialCopy;
	if (shape.compoundParentGeneration != shape.generation || shape.compoundParentFlags != shape.flags ||
		shape.compoundParentMaterials != materials || copy.length != materials.length) {
		return false;
	}

	let a = changetype<usize>(copy);
	let b = changetype<usize>(materials);
	let end = <usize>materials.length * sizeof<b3SurfaceMaterial>();
	for (let offset: usize = 0; offset < end; offset += 4) {
		if (load<u32>(a + offset) != load<u32>(b + offset)) {
			return false;
		}
	}
	return true;
}

// Builds each compound shape's children as shapes, which parallel parts read in place of the scratch child shape
export unsafe function b3PrepareCompoundContacts(world: b3World): void {
	if (world.compoundShapeCount == 0) {
		return;
	}

	let shapes = world.shapes;
	for (let i = 0; i < shapes.length; ++i) {
		let shape = shapes[i];
		if (shape.id != i || shape.type != b3_compoundShape) {
			continue;
		}

		let compound = shape.compound!;
		if (shape.compoundChildData == compound && b3CompoundParentUnchanged(shape)) {
			continue;
		}

		let childCount = compound.capsuleCount + compound.hullCount + compound.meshCount + compound.sphereCount;
		if (shape.compoundChildData != compound) {
			let children = new StaticArray<b3ChildShape>(childCount);
			let childShapes = new StaticArray<b3Shape>(childCount);
			let childMaterials = new StaticArray<StaticArray<b3SurfaceMaterial>>(childCount);
			for (let j = 0; j < childCount; ++j) {
				let child = new b3ChildShape();
				b3GetCompoundChildInto(compound, j, child, new Uint16Array(1));
				children[j] = child;
				childShapes[j] = new b3Shape();
				childMaterials[j] = new StaticArray<b3SurfaceMaterial>(1);
			}
			shape.compoundChildren = children;
			shape.compoundChildShapes = childShapes;
			shape.compoundChildMaterials = childMaterials;
			shape.compoundChildData = compound;
		}

		// The parent's identity, flags or materials changed since, or the children are new
		let children = shape.compoundChildren!;
		let childShapes = shape.compoundChildShapes!;
		let childMaterials = shape.compoundChildMaterials!;
		for (let j = 0; j < childCount; ++j) {
			b3SetCompoundChildShape(childShapes[j], shape, children[j], childMaterials[j]);
		}

		shape.compoundParentGeneration = shape.generation;
		shape.compoundParentFlags = shape.flags;
		shape.compoundParentMaterials = shape.materials;
		let copy = new StaticArray<b3SurfaceMaterial>(shape.materials.length);
		for (let j = 0; j < copy.length; ++j) {
			copy[j] = shape.materials[j];
		}
		shape.compoundParentMaterialCopy = copy;
	}
}
const s_contactChild = new b3ChildShape();
const s_contactChildMaterial = new Uint16Array(1);
const s_contactChildShape = new b3Shape();
const s_contactChildMaterials = new StaticArray<b3SurfaceMaterial>(1);

unsafe function b3ComputeConvexManifold(
	world: b3World,
	workerIndex: i32,
	contact: b3Contact,
	shapeA: b3Shape,
	xfA: b3WorldTransform,
	shapeB: b3Shape,
	xfB: b3WorldTransform,
): bool {
	let typeA = shapeA.type;
	let typeB = shapeB.type;

	// cache is contact.convexContact.cache

	let pointCapacity = B3_MAX_MANIFOLD_POINTS;

	let geomManifold = s_convexContactScratch[workerIndex].manifold;
	b3ZeroLocalManifold(geomManifold);

	let transformBtoA = b3InvMulWorldTransforms(xfA, xfB);

	if (typeA == b3_sphereShape) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(typeB == b3_sphereShape);
		b3CollideSpheres(geomManifold, pointCapacity, shapeA.sphere, shapeB.sphere, transformBtoA);
	} else if (typeA == b3_capsuleShape) {
		if (typeB == b3_sphereShape) {
			b3CollideCapsuleAndSphere(geomManifold, pointCapacity, shapeA.capsule, shapeB.sphere, transformBtoA);
		} else {
			if (B3_ASSERT_ENABLED) B3_ASSERT(typeB == b3_capsuleShape);
			b3CollideCapsules(geomManifold, pointCapacity, shapeA.capsule, shapeB.capsule, transformBtoA, workerIndex);
		}
	} else {
		if (B3_ASSERT_ENABLED) B3_ASSERT(typeA == b3_hullShape);

		if (typeB == b3_sphereShape) {
			b3CollideHullAndSphere(
				geomManifold,
				pointCapacity,
				shapeA.hull!,
				shapeB.sphere,
				transformBtoA,
				contact.convexContact.cache.simplexCache,
				workerIndex,
			);
		} else if (typeB == b3_capsuleShape) {
			b3CollideHullAndCapsule(
				geomManifold,
				pointCapacity,
				shapeA.hull!,
				shapeB.capsule,
				transformBtoA,
				contact.convexContact.cache.simplexCache,
				workerIndex,
			);
		} else {
			if (B3_ASSERT_ENABLED) B3_ASSERT(typeB == b3_hullShape);
			b3CollideHullsAtWidth(
				geomManifold,
				pointCapacity,
				shapeA.hull!,
				shapeB.hull!,
				transformBtoA,
				contact.convexContact.cache.satCache,
				world.simdWidth,
				workerIndex,
			);
			let taskContext = world.taskContexts[workerIndex];
			taskContext.satCallCount += 1;
			taskContext.satCacheHitCount += <i32>contact.convexContact.cache.satCache.hit;
		}
	}

	// The contact keeps its manifold storage, which b3CreateContact allocated, and clears it as C allocates it
	if (geomManifold.pointCount == 0) {
		contact.manifoldCount = 0;
		return false;
	}

	let oldPoints: FixedArray<b3ManifoldPoint, 4>; // B3_MAX_MANIFOLD_POINTS
	let oldCount = 0;

	if (contact.manifoldCount == 0) {
		if (!Parallel.inPart()) {
			if (contact.manifolds.length == 0) {
				contact.manifolds = b3AllocateManifolds(world, 1);
			}
		}
		memory.fill(changetype<usize>(contact.manifolds), 0, sizeof<b3Manifold>());
		contact.manifoldCount = 1;
	} else {
		oldCount = contact.manifolds[0].pointCount;
		for (let i = 0; i < oldCount; ++i) {
			oldPoints[i] = contact.manifolds[0].points[i];
		}
	}

	ref manifold = contact.manifolds[0];
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 < geomManifold.pointCount && geomManifold.pointCount <= B3_MAX_MANIFOLD_POINTS);
	manifold.pointCount = geomManifold.pointCount;

	let matrixA = b3MakeMatrixFromQuat(xfA.q);
	manifold.normal = b3MulMV(matrixA, geomManifold.normal);

	// Store point data in contact and copy impulses from old points. C stores every point before matching any;
	// a point's match reads only its own feature id and the old points, so one pass per point is the same.
	for (let i = 0; i < geomManifold.pointCount; ++i) {
		let source = geomManifold.points[geomManifold.pointOffset + i];

		// Contact points are computed in frame A
		let anchorA = b3MulMV(matrixA, source.point);
		let anchorB = b3Add(anchorA, b3SubPos(xfA.p, xfB.p));
		let featureId = b3MakeFeatureId(source.pair);

		let normalImpulse: f32 = 0.0;
		let persisted = false;
		for (let j = 0; j < oldCount; ++j) {
			if (featureId == oldPoints[j].featureId) {
				normalImpulse = oldPoints[j].normalImpulse;
				persisted = true;

				// claimed
				oldPoints[j].featureId = UINT32_MAX;

				break;
			}
		}

		let mp = b3ManifoldPointAddress(contact.manifolds, 0, i);
		b3StorePointAnchors(mp, anchorA, anchorB, source.separation, normalImpulse);
		b3StoreRow(mp + MP_TOTALNORMALIMPULSE, 0.0, 0.0, load<f32>(mp + MP_BASESEPARATION), reinterpret<f32>(featureId));
		store<i32>(mp + MP_TRIANGLEINDEX, B3_NULL_INDEX);
		store<bool>(mp + MP_PERSISTED, persisted);
	}

	return true;
}

unsafe function b3UpdateConvexContact(
	world: b3World,
	workerIndex: i32,
	contact: b3Contact,
	shapeA: b3Shape,
	xfA: b3WorldTransform,
	shapeB: b3Shape,
	xfB: b3WorldTransform,
	flip: bool,
): bool {
	// Compute new manifold
	let touching = b3ComputeConvexManifold(world, workerIndex, contact, shapeA, xfA, shapeB, xfB);

	if (touching == false) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(contact.manifoldCount == 0);
		return false;
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(contact.manifoldCount == 1);

	if (flip) {
		// Not flipping the feature ids because they just need to match and flipping is consistent.
		ref manifold = contact.manifolds[0];
		manifold.normal = b3Neg(manifold.normal);
		let pointCount = manifold.pointCount;
		for (let i = 0; i < pointCount; ++i) {
			let mp = b3ManifoldPointAddress(contact.manifolds, 0, i);
			let anchorA = load<b3Vec3>(mp + MP_ANCHORA);
			let anchorB = load<b3Vec3>(mp + MP_ANCHORB);
			b3StorePointAnchors(mp, anchorB, anchorA, load<f32>(mp + MP_SEPARATION), load<f32>(mp + MP_NORMALIMPULSE));
		}
	}

	let materialA = b3GetShapeMaterials(shapeA)[0];
	let materialB = b3GetShapeMaterials(shapeB)[0];

	// Keep these updated in case the values on the shapes are modified
	contact.friction = b3MixFriction(world, materialA.friction, materialA.userMaterialId, materialB.friction, materialB.userMaterialId);
	contact.restitution = b3MixRestitution(
		world,
		materialA.restitution,
		materialA.userMaterialId,
		materialB.restitution,
		materialB.userMaterialId,
	);

	if (materialA.rollingResistance > 0.0 || materialB.rollingResistance > 0.0) {
		let typeA = shapeA.type;
		let typeB = shapeB.type;

		let radiusA: f32 = 0.0;
		if (typeA == b3_sphereShape) {
			radiusA = shapeA.sphere.radius;
		} else if (typeA == b3_capsuleShape) {
			radiusA = shapeA.capsule.radius;
		} else if (typeA == b3_hullShape) {
			radiusA = 0.25 * shapeA.hull!.innerRadius;
		}

		let radiusB: f32 = 0.0;
		if (typeB == b3_sphereShape) {
			radiusB = shapeB.sphere.radius;
		} else if (typeB == b3_capsuleShape) {
			radiusB = shapeB.capsule.radius;
		} else if (typeB == b3_hullShape) {
			radiusB = 0.25 * shapeB.hull!.innerRadius;
		}

		let maxRadius = b3MaxFloat(radiusA, radiusB);
		contact.rollingResistance = b3MaxFloat(materialA.rollingResistance, materialB.rollingResistance) * maxRadius;
	} else {
		contact.rollingResistance = 0.0;
	}

	let tangentVelocityA = b3RotateVector(xfA.q, materialA.tangentVelocity);
	let tangentVelocityB = b3RotateVector(xfB.q, materialB.tangentVelocity);
	contact.tangentVelocity = b3Sub(tangentVelocityA, tangentVelocityB);

	// Parallel parts run only without a pre-solve callback
	if (!Parallel.inPart()) {
		let preSolveFcn = world.preSolveFcn;
		if (preSolveFcn != null && (contact.flags & b3_simEnablePreSolveEvents) != 0) {
			let shapeIdA = b3MakeShapeIdInternal(shapeA.id + 1, world.worldId, shapeA.generation);
			let shapeIdB = b3MakeShapeIdInternal(shapeB.id + 1, world.worldId, shapeB.generation);

			// this call assumes thread safety
			let point = b3OffsetPos(xfA.p, contact.manifolds[0].points[0].anchorA);
			let normal = contact.manifolds[0].normal;
			touching = preSolveFcn(shapeIdA, shapeIdB, point, normal);
			if (touching == false) {
				// disable contact
				contact.manifoldCount = 0;

				// Invalidate recycling cache.
				contact.flags &= ~b3_relativeTransformValid;
				return false;
			}
		}
	}

	if ((shapeA.flags & b3_enableHitEvents) != 0 || (shapeB.flags & b3_enableHitEvents) != 0) {
		contact.flags |= b3_simEnableHitEvent;
	} else {
		contact.flags &= ~b3_simEnableHitEvent;
	}

	return true;
}

// Update the contact manifold and touching status.
// Note: do not assume the shape AABBs are overlapping or are valid.
export unsafe function b3UpdateContact(
	world: b3World,
	workerIndex: i32,
	contact: b3Contact,
	shapeA: b3Shape,
	localCenterA: b3Vec3,
	xfA: b3WorldTransform,
	shapeB: b3Shape,
	localCenterB: b3Vec3,
	xfB: b3WorldTransform,
	isFast: bool,
): bool {
	let touching = false;

	if (B3_ASSERT_ENABLED) B3_ASSERT(shapeB.type != b3_compoundShape);

	if (shapeA.type == b3_compoundShape) {
		let childIndex = contact.childIndex;
		let child = s_contactChild;
		let childShapeA = s_contactChildShape;
		if (Parallel.inPart()) {
			// The main lane built these before the parallel call, b3PrepareCompoundContacts
			child = shapeA.compoundChildren![childIndex];
			childShapeA = shapeA.compoundChildShapes![childIndex];
		} else {
			b3GetCompoundChildInto(shapeA.compound!, childIndex, child, s_contactChildMaterial);
			b3SetCompoundChildShape(childShapeA, shapeA, child, s_contactChildMaterials);
		}

		if (child.type == b3_capsuleShape) {
			if (shapeB.type == b3_hullShape) {
				// Flip
				let flip = true;
				touching = b3UpdateConvexContact(world, workerIndex, contact, shapeB, xfB, childShapeA, xfA, flip);
			} else {
				let flip = false;
				touching = b3UpdateConvexContact(world, workerIndex, contact, childShapeA, xfA, shapeB, xfB, flip);
			}
		} else if (child.type == b3_hullShape) {
			let xfChild = b3MulWorldTransforms(xfA, child.transform);
			let flip = false;
			touching = b3UpdateConvexContact(world, workerIndex, contact, childShapeA, xfChild, shapeB, xfB, flip);
		} else if (child.type == b3_meshShape) {
			let xfChild = b3MulWorldTransforms(xfA, child.transform);

			touching = b3ComputeMeshManifoldsAt(
				world,
				workerIndex,
				contact,
				childShapeA,
				child.materialIndices,
				xfChild,
				shapeB,
				xfB,
				isFast,
			);

			if (touching && ((shapeA.flags & b3_enableHitEvents) != 0 || (shapeB.flags & b3_enableHitEvents) != 0)) {
				contact.flags |= b3_simEnableHitEvent;
			} else {
				contact.flags &= ~b3_simEnableHitEvent;
			}

			if (B3_ASSERT_ENABLED) B3_ASSERT((touching == true && contact.manifoldCount > 0) || (touching == false && contact.manifoldCount == 0));
		} else {
			if (B3_ASSERT_ENABLED) B3_ASSERT(child.type == b3_sphereShape);

			if (shapeB.type == b3_capsuleShape || shapeB.type == b3_hullShape) {
				// Flip
				let flip = true;
				touching = b3UpdateConvexContact(world, workerIndex, contact, shapeB, xfB, childShapeA, xfA, flip);
			} else {
				let flip = false;
				touching = b3UpdateConvexContact(world, workerIndex, contact, childShapeA, xfA, shapeB, xfB, flip);
			}
		}

		// The anchor is relative to the child origin but oriented in world space.
		// Offset the anchor to be relative to the compound origin.
		let manifoldCount = contact.manifoldCount;
		let offset = b3RotateVector(xfA.q, child.transform.p);
		for (let i = 0; i < manifoldCount; ++i) {
			let pointCount = contact.manifolds[i].pointCount;
			for (let j = 0; j < pointCount; ++j) {
				let mp = b3ManifoldPointAddress(contact.manifolds, i, j);
				let anchorA = b3Add(load<b3Vec3>(mp + MP_ANCHORA), offset);
				b3StoreRow(mp + MP_ANCHORA, anchorA.x, anchorA.y, anchorA.z, load<f32>(mp + MP_ANCHORB));
			}
		}
	} else if (shapeA.type == b3_meshShape || shapeA.type == b3_heightShape) {
		// Does this contact touch a mesh or height-field?

		// Compute mesh manifolds
		touching = b3ComputeMeshManifoldsAt(world, workerIndex, contact, shapeA, null, xfA, shapeB, xfB, isFast);

		if (touching && ((shapeA.flags & b3_enableHitEvents) != 0 || (shapeB.flags & b3_enableHitEvents) != 0)) {
			contact.flags |= b3_simEnableHitEvent;
		} else {
			contact.flags &= ~b3_simEnableHitEvent;
		}

		if (B3_ASSERT_ENABLED) B3_ASSERT((touching == true && contact.manifoldCount > 0) || (touching == false && contact.manifoldCount == 0));
	} else {
		// Convex-vs-convex
		let flip = false;
		touching = b3UpdateConvexContact(world, workerIndex, contact, shapeA, xfA, shapeB, xfB, flip);
	}

	if (touching) {
		let centerA = b3RotateVector(xfA.q, localCenterA);
		let centerB = b3RotateVector(xfB.q, localCenterB);

		// Adjust anchors to be relative to center of mass
		for (let i = 0; i < contact.manifoldCount; ++i) {
			let pointCount = contact.manifolds[i].pointCount;
			for (let j = 0; j < pointCount; ++j) {
				let mp = b3ManifoldPointAddress(contact.manifolds, i, j);
				let anchorA = b3Sub(load<b3Vec3>(mp + MP_ANCHORA), centerA);
				let anchorB = b3Sub(load<b3Vec3>(mp + MP_ANCHORB), centerB);
				b3StorePointAnchors(mp, anchorA, anchorB, load<f32>(mp + MP_SEPARATION), load<f32>(mp + MP_NORMALIMPULSE));
			}
		}

		contact.flags |= b3_simTouchingFlag;
	} else {
		contact.flags &= ~b3_simTouchingFlag;
	}

	return touching;
}
