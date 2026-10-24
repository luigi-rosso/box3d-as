// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/contact.c

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX, UINT32_MAX } from "./core";
import { B3_MAX_MANIFOLD_POINTS } from "./constants";
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
import { b3GetCompoundChild, b3GetCompoundChildInto } from "./compound";
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
import { b3ComputeMeshManifolds } from "./mesh_contact";
import {
	b3AllocateManifolds,
	b3FreeManifolds,
	b3GetWorld,
	b3World,
	b3_awakeSet,
	b3_disabledSet,
	b3_staticSet,
} from "./physics_world";
import {
	b3CopyShapeInto,
	b3GetShapeMaterials,
	b3Shape,
	b3_enableContactEvents,
	b3_enableHitEvents,
	b3_enablePreSolveEvents,
	b3_enableSpeculative,
} from "./shape";

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

	// *contact = (b3Contact){ 0 } keeping the generation
	let generation = world.contacts[contactId].generation;
	let contact = new b3Contact();
	world.contacts[contactId] = contact;
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
		let child = b3GetCompoundChild(shapeA.compound!, childIndex);
		if (child.type == b3_meshShape) {
			contact.flags |= b3_simMeshContact;
		}
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
		contact.meshContact.triangleCache = new Array<b3TriangleCache>();
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

// C's stack buffers for b3ComputeConvexManifold and the compound child of b3UpdateContact. Neither
// call nests in itself, so each has one.
const s_convexPoints = new StaticArray<b3LocalManifoldPoint>(B3_MAX_MANIFOLD_POINTS);
const s_convexManifold = new b3LocalManifold();
const s_contactChild = new b3ChildShape();
const s_contactChildMaterial = new Uint16Array(1);
const s_contactChildShape = new b3Shape();
const s_contactChildMaterials = new StaticArray<b3SurfaceMaterial>(1);

function b3ComputeConvexManifold(
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
	let pointBuffer = s_convexPoints;

	let geomManifold = s_convexManifold;
	b3ZeroLocalManifold(geomManifold);
	geomManifold.points = pointBuffer;

	let transformBtoA = b3InvMulWorldTransforms(xfA, xfB);

	if (typeA == b3_sphereShape) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(typeB == b3_sphereShape);
		b3CollideSpheres(geomManifold, pointCapacity, shapeA.sphere, shapeB.sphere, transformBtoA);
	} else if (typeA == b3_capsuleShape) {
		if (typeB == b3_sphereShape) {
			b3CollideCapsuleAndSphere(geomManifold, pointCapacity, shapeA.capsule, shapeB.sphere, transformBtoA);
		} else {
			if (B3_ASSERT_ENABLED) B3_ASSERT(typeB == b3_capsuleShape);
			b3CollideCapsules(geomManifold, pointCapacity, shapeA.capsule, shapeB.capsule, transformBtoA);
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
			);
		} else if (typeB == b3_capsuleShape) {
			b3CollideHullAndCapsule(
				geomManifold,
				pointCapacity,
				shapeA.hull!,
				shapeB.capsule,
				transformBtoA,
				contact.convexContact.cache.simplexCache,
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
			);
			let taskContext = world.taskContexts[workerIndex];
			taskContext.satCallCount += 1;
			taskContext.satCacheHitCount += <i32>contact.convexContact.cache.satCache.hit;
		}
	}

	if (geomManifold.pointCount == 0) {
		if (contact.manifoldCount > 0) {
			b3FreeManifolds(world, contact.manifolds, contact.manifoldCount);
			contact.manifolds = b3_emptyManifolds;
			contact.manifoldCount = 0;
		}

		return false;
	}

	let oldPoints: FixedArray<b3ManifoldPoint, 4>; // B3_MAX_MANIFOLD_POINTS
	let oldCount = 0;

	if (contact.manifoldCount == 0) {
		contact.manifolds = b3AllocateManifolds(world, 1);
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

	// Store point data in contact
	for (let i = 0; i < geomManifold.pointCount; ++i) {
		let source = geomManifold.points[geomManifold.pointOffset + i];
		ref target = manifold.points[i];

		// Contact points are computed in frame A
		target.anchorA = b3MulMV(matrixA, source.point);
		target.anchorB = b3Add(target.anchorA, b3SubPos(xfA.p, xfB.p));
		target.separation = source.separation;
		target.featureId = b3MakeFeatureId(source.pair);
		target.triangleIndex = B3_NULL_INDEX;
		target.normalVelocity = 0.0;
	}

	// Copy impulses from old points
	for (let i = 0; i < geomManifold.pointCount; ++i) {
		ref pt2 = manifold.points[i];
		pt2.totalNormalImpulse = 0.0;
		pt2.persisted = false;

		for (let j = 0; j < oldCount; ++j) {
			if (pt2.featureId == oldPoints[j].featureId) {
				pt2.normalImpulse = oldPoints[j].normalImpulse;
				pt2.persisted = true;

				// claimed
				oldPoints[j].featureId = UINT32_MAX;

				break;
			}
		}

		if (pt2.persisted == false) {
			pt2.normalImpulse = 0.0;
		}
	}

	return true;
}

function b3UpdateConvexContact(
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
		if (B3_ASSERT_ENABLED) B3_ASSERT(contact.manifolds.length == 0 && contact.manifoldCount == 0);
		return false;
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(contact.manifoldCount == 1);

	if (flip) {
		// Not flipping the feature ids because they just need to match and flipping is consistent.
		ref manifold = contact.manifolds[0];
		manifold.normal = b3Neg(manifold.normal);
		let pointCount = manifold.pointCount;
		for (let i = 0; i < pointCount; ++i) {
			ref mp = manifold.points[i];
			let t = mp.anchorA;
			mp.anchorA = mp.anchorB;
			mp.anchorB = t;
		}
	}

	let materialA = b3GetShapeMaterials(shapeA)[0];
	let materialB = b3GetShapeMaterials(shapeB)[0];

	// Keep these updated in case the values on the shapes are modified
	contact.friction = world.frictionCallback!(
		materialA.friction,
		materialA.userMaterialId,
		materialB.friction,
		materialB.userMaterialId,
	);
	contact.restitution = world.restitutionCallback!(
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
			b3FreeManifolds(world, contact.manifolds, contact.manifoldCount);
			contact.manifolds = b3_emptyManifolds;
			contact.manifoldCount = 0;

			// Invalidate recycling cache.
			contact.flags &= ~b3_relativeTransformValid;
			return false;
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
export function b3UpdateContact(
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
	let touching: bool;

	if (B3_ASSERT_ENABLED) B3_ASSERT(shapeB.type != b3_compoundShape);

	if (shapeA.type == b3_compoundShape) {
		let childIndex = contact.childIndex;
		let child = s_contactChild;
		b3GetCompoundChildInto(shapeA.compound!, childIndex, child, s_contactChildMaterial);

		// Temporary child shape to match existing function signatures
		let childShapeA = s_contactChildShape;
		b3CopyShapeInto(childShapeA, shapeA);

		childShapeA.type = child.type;

		// Handle child material for non-meshes.
		if (child.type != b3_meshShape) {
			let materialIndex = <i32>child.materialIndices![0];
			if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= materialIndex && materialIndex < shapeA.materialCount);
			let parentMaterials = b3GetShapeMaterials(shapeA);
			let childMaterials = s_contactChildMaterials;
			childMaterials[0] = parentMaterials[materialIndex];
			childShapeA.materials = childMaterials;
			childShapeA.materialCount = 1;
		}

		if (child.type == b3_capsuleShape) {
			childShapeA.capsule = child.capsule;
			if (shapeB.type == b3_hullShape) {
				// Flip
				let flip = true;
				touching = b3UpdateConvexContact(world, workerIndex, contact, shapeB, xfB, childShapeA, xfA, flip);
			} else {
				let flip = false;
				touching = b3UpdateConvexContact(world, workerIndex, contact, childShapeA, xfA, shapeB, xfB, flip);
			}
		} else if (child.type == b3_hullShape) {
			childShapeA.hull = child.hull;
			let xfChild = b3MulWorldTransforms(xfA, child.transform);
			let flip = false;
			touching = b3UpdateConvexContact(world, workerIndex, contact, childShapeA, xfChild, shapeB, xfB, flip);
		} else if (child.type == b3_meshShape) {
			childShapeA.mesh = child.mesh;
			let xfChild = b3MulWorldTransforms(xfA, child.transform);

			touching = b3ComputeMeshManifolds(
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

			childShapeA.sphere = child.sphere;
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
			ref manifold = contact.manifolds[i];
			let pointCount = manifold.pointCount;
			for (let j = 0; j < pointCount; ++j) {
				ref mp = manifold.points[j];
				mp.anchorA = b3Add(mp.anchorA, offset);
			}
		}
	} else if (shapeA.type == b3_meshShape || shapeA.type == b3_heightShape) {
		// Does this contact touch a mesh or height-field?

		// Compute mesh manifolds
		touching = b3ComputeMeshManifolds(world, workerIndex, contact, shapeA, null, xfA, shapeB, xfB, isFast);

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
			ref manifold = contact.manifolds[i];
			for (let j = 0; j < manifold.pointCount; ++j) {
				ref mp = manifold.points[j];
				mp.anchorA = b3Sub(mp.anchorA, centerA);
				mp.anchorB = b3Sub(mp.anchorB, centerB);
			}
		}

		contact.flags |= b3_simTouchingFlag;
	} else {
		contact.flags &= ~b3_simTouchingFlag;
	}

	return touching;
}
