// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/island.c

import { B3_ASSERT, B3_NULL_INDEX } from "./core";
import { b3AllocId, b3FreeId } from "./id_pool";
import { b3_dynamicBody } from "./types";
import { b3Contact, b3_contactTouchingFlag } from "./contact";
import { b3World, b3_awakeSet, b3_disabledSet, b3_firstSleepingSet, b3_staticSet } from "./physics_world";
import { b3WakeSolverSet } from "./solver_set";
import { b3Array_RemoveSwap } from "./container";

// Cached contact data stored in the island for fast contiguous iteration.
// Avoids touching b3Contact during union-find in b3SplitIsland.
export struct b3ContactLink {
	contactId: i32 = 0;
	bodyIdA: i32 = 0;
	bodyIdB: i32 = 0;
}

// Cached joint data stored in the island for fast contiguous iteration.
export struct b3JointLink {
	jointId: i32 = 0;
	bodyIdA: i32 = 0;
	bodyIdB: i32 = 0;
}

// Deterministic solver
//
// Collide all awake contacts
// Use bit array to emit start/stop touching events in defined order, per thread. Try using contact index, assuming contacts are
// created in a deterministic order. bit-wise OR together bit arrays and issue changes:
// - start touching: merge islands - temporary linked list - mark root island dirty - wake all - largest island is root
// - stop touching: increment constraintRemoveCount

// Persistent island for awake bodies, joints, and contacts.
// Contacts are touching.
// Contacts and joints may connect to static bodies, but static bodies are not in the island.
// https://en.wikipedia.org/wiki/Component_(graph_theory)
// https://en.wikipedia.org/wiki/Dynamic_connectivity
export class b3Island {
	// index of solver set stored in b3World
	// may be B3_NULL_INDEX
	setIndex: i32 = 0;

	// island index within set
	// may be B3_NULL_INDEX
	localIndex: i32 = 0;

	islandId: i32 = 0;

	// Keeps track of how many contacts have been removed from this island.
	// This is used to determine if an island is a candidate for splitting.
	constraintRemoveCount: i32 = 0;

	// I tried using a stack array for this but the data pointer goes out of
	// sync when the world island array grows.
	bodies: Array<i32> = new Array<i32>();

	// Contacts and joints that belong to this island. May connect to static
	// bodies not in the island.
	// Each link has the two body ids so that b3SplitIsland's union-find pass
	// never needs to touch b3Contact/b3Joint.
	contacts: Array<b3ContactLink> = new Array<b3ContactLink>();
	joints: Array<b3JointLink> = new Array<b3JointLink>();
}

// This is used to move islands across solver sets
export struct b3IslandSim {
	islandId: i32 = 0;
}

// functions from island.c follow

export function b3CreateIsland(world: b3World, setIndex: i32): b3Island {
	B3_ASSERT(setIndex == b3_awakeSet || setIndex >= b3_firstSleepingSet);

	let islandId = b3AllocId(world.islandIdPool);

	if (islandId == world.islands.length) {
		world.islands.push(new b3Island());
	} else {
		B3_ASSERT(world.islands[islandId].setIndex == B3_NULL_INDEX);
	}

	let set = world.solverSets[setIndex];

	let island = world.islands[islandId];
	island.setIndex = setIndex;
	island.localIndex = set.islandSims.length;
	island.islandId = islandId;
	island.bodies = new Array<i32>();
	island.contacts = new Array<b3ContactLink>();
	island.joints = new Array<b3JointLink>();
	island.constraintRemoveCount = 0;

	let islandSim = b3IslandSim();
	islandSim.islandId = islandId;
	set.islandSims.push(islandSim);

	return island;
}

export function b3DestroyIsland(world: b3World, islandId: i32): void {
	if (world.splitIslandId == islandId) {
		world.splitIslandId = B3_NULL_INDEX;
	}

	// assume island is empty
	let island = world.islands[islandId];
	let set = world.solverSets[island.setIndex];
	{
		let localIndex = island.localIndex;
		let lastIndex = set.islandSims.length - 1;
		B3_ASSERT(0 <= localIndex && localIndex <= lastIndex);
		let moveIslandId = set.islandSims[lastIndex].islandId;
		set.islandSims[localIndex] = set.islandSims[lastIndex];
		world.islands[moveIslandId].localIndex = localIndex;
		set.islandSims.length = lastIndex;
	}

	// Free island and id (preserve island generation)
	island.bodies = new Array<i32>();
	island.contacts = new Array<b3ContactLink>();
	island.joints = new Array<b3JointLink>();
	island.constraintRemoveCount = 0;
	island.localIndex = B3_NULL_INDEX;
	island.islandId = B3_NULL_INDEX;
	island.setIndex = B3_NULL_INDEX;

	b3FreeId(world.islandIdPool, islandId);
}

function b3MergeIslands(world: b3World, islandIdA: i32, islandIdB: i32): i32 {
	if (islandIdA == islandIdB) {
		return islandIdA;
	}

	if (islandIdA == B3_NULL_INDEX) {
		B3_ASSERT(islandIdB != B3_NULL_INDEX);
		return islandIdB;
	}

	if (islandIdB == B3_NULL_INDEX) {
		B3_ASSERT(islandIdA != B3_NULL_INDEX);
		return islandIdA;
	}

	let smallIsland: b3Island;
	let bigIsland: b3Island;
	{
		let islandA = world.islands[islandIdA];
		let islandB = world.islands[islandIdB];

		// Keep the biggest island to reduce cache misses
		if (islandA.bodies.length >= islandB.bodies.length) {
			bigIsland = islandA;
			smallIsland = islandB;
		} else {
			bigIsland = islandB;
			smallIsland = islandA;
		}
	}

	let bigIslandId = bigIsland.islandId;

	// Move bodies from smaller island to larger island
	for (let i = 0; i < smallIsland.bodies.length; ++i) {
		let bodyId = smallIsland.bodies[i];
		world.bodies[bodyId].islandId = bigIslandId;
		world.bodies[bodyId].islandIndex = bigIsland.bodies.length;
		bigIsland.bodies.push(bodyId);
	}

	// Migrate contacts from smaller island to larger island
	if (smallIsland.contacts.length > 0) {
		for (let i = 0; i < smallIsland.contacts.length; ++i) {
			let link = smallIsland.contacts[i];
			let contact = world.contacts[link.contactId];
			contact.islandId = bigIslandId;
			contact.islandIndex = bigIsland.contacts.length;
			bigIsland.contacts.push(link);
		}
	}

	// Migrate joints from smaller island to larger island
	if (smallIsland.joints.length > 0) {
		for (let i = 0; i < smallIsland.joints.length; ++i) {
			let link = smallIsland.joints[i];
			world.joints[link.jointId].islandId = bigIslandId;
			world.joints[link.jointId].islandIndex = bigIsland.joints.length;
			bigIsland.joints.push(link);
		}
	}

	// Track removed constraints
	bigIsland.constraintRemoveCount += smallIsland.constraintRemoveCount;

	b3DestroyIsland(world, smallIsland.islandId);

	return bigIslandId;
}

function b3AddContactToIsland(world: b3World, islandId: i32, contact: b3Contact): void {
	B3_ASSERT(contact.islandId == B3_NULL_INDEX);
	B3_ASSERT(contact.islandIndex == B3_NULL_INDEX);

	let island = world.islands[islandId];

	contact.islandId = islandId;
	contact.islandIndex = island.contacts.length;

	let link = b3ContactLink();
	link.contactId = contact.contactId;
	link.bodyIdA = contact.edges[0].bodyId;
	link.bodyIdB = contact.edges[1].bodyId;
	island.contacts.push(link);
}

// Link a contact into an island.
export function b3LinkContact(world: b3World, contact: b3Contact): void {
	B3_ASSERT((contact.flags & b3_contactTouchingFlag) != 0);

	let bodyIdA = contact.edges[0].bodyId;
	let bodyIdB = contact.edges[1].bodyId;

	B3_ASSERT(world.bodies[bodyIdA].setIndex != b3_disabledSet && world.bodies[bodyIdB].setIndex != b3_disabledSet);
	B3_ASSERT(world.bodies[bodyIdA].setIndex != b3_staticSet || world.bodies[bodyIdB].setIndex != b3_staticSet);

	// Wake bodyB if bodyA is awake and bodyB is sleeping
	if (world.bodies[bodyIdA].setIndex == b3_awakeSet && world.bodies[bodyIdB].setIndex >= b3_firstSleepingSet) {
		b3WakeSolverSet(world, world.bodies[bodyIdB].setIndex);
	}

	// Wake bodyA if bodyB is awake and bodyA is sleeping
	if (world.bodies[bodyIdB].setIndex == b3_awakeSet && world.bodies[bodyIdA].setIndex >= b3_firstSleepingSet) {
		b3WakeSolverSet(world, world.bodies[bodyIdA].setIndex);
	}

	let islandIdA = world.bodies[bodyIdA].islandId;
	let islandIdB = world.bodies[bodyIdB].islandId;

	// Static bodies have null island indices.
	B3_ASSERT(world.bodies[bodyIdA].setIndex != b3_staticSet || islandIdA == B3_NULL_INDEX);
	B3_ASSERT(world.bodies[bodyIdB].setIndex != b3_staticSet || islandIdB == B3_NULL_INDEX);
	B3_ASSERT(islandIdA != B3_NULL_INDEX || islandIdB != B3_NULL_INDEX);

	// Merge islands. This will destroy one of the islands.
	let finalIslandId = b3MergeIslands(world, islandIdA, islandIdB);

	// Add contact to the island that survived
	b3AddContactToIsland(world, finalIslandId, contact);
}

// This is called when a contact no longer has contact points or when a contact is destroyed.
export function b3UnlinkContact(world: b3World, contact: b3Contact): void {
	B3_ASSERT(contact.islandId != B3_NULL_INDEX);

	// remove from island
	let islandId = contact.islandId;
	let island = world.islands[islandId];

	let removeIndex = contact.islandIndex;
	B3_ASSERT(0 <= removeIndex && removeIndex < island.contacts.length);
	B3_ASSERT(island.contacts[removeIndex].contactId == contact.contactId);

	let movedIndex = b3Array_RemoveSwap(island.contacts, removeIndex);
	if (movedIndex != B3_NULL_INDEX) {
		// Fix islandIndex on the contact that was swapped into removeIndex
		let movedContact = world.contacts[island.contacts[removeIndex].contactId];
		B3_ASSERT(movedContact.islandIndex == movedIndex);
		movedContact.islandIndex = removeIndex;
	}

	contact.islandId = B3_NULL_INDEX;
	contact.islandIndex = B3_NULL_INDEX;
	island.constraintRemoveCount += 1;
}

function b3AddJointToIsland(world: b3World, islandId: i32, jointId: i32): void {
	B3_ASSERT(world.joints[jointId].islandId == B3_NULL_INDEX);
	B3_ASSERT(world.joints[jointId].islandIndex == B3_NULL_INDEX);

	let island = world.islands[islandId];

	world.joints[jointId].islandId = islandId;
	world.joints[jointId].islandIndex = island.joints.length;

	let link = b3JointLink();
	link.jointId = world.joints[jointId].jointId;
	link.bodyIdA = world.joints[jointId].edges[0].bodyId;
	link.bodyIdB = world.joints[jointId].edges[1].bodyId;
	island.joints.push(link);
}

export function b3LinkJoint(world: b3World, jointId: i32): void {
	let bodyIdA = world.joints[jointId].edges[0].bodyId;
	let bodyIdB = world.joints[jointId].edges[1].bodyId;

	B3_ASSERT(world.bodies[bodyIdA].type == b3_dynamicBody || world.bodies[bodyIdB].type == b3_dynamicBody);

	if (world.bodies[bodyIdA].setIndex == b3_awakeSet && world.bodies[bodyIdB].setIndex >= b3_firstSleepingSet) {
		b3WakeSolverSet(world, world.bodies[bodyIdB].setIndex);
	} else if (world.bodies[bodyIdB].setIndex == b3_awakeSet && world.bodies[bodyIdA].setIndex >= b3_firstSleepingSet) {
		b3WakeSolverSet(world, world.bodies[bodyIdA].setIndex);
	}

	let islandIdA = world.bodies[bodyIdA].islandId;
	let islandIdB = world.bodies[bodyIdB].islandId;

	B3_ASSERT(islandIdA != B3_NULL_INDEX || islandIdB != B3_NULL_INDEX);

	// Merge islands. This will destroy one of the islands.
	let finalIslandId = b3MergeIslands(world, islandIdA, islandIdB);

	// Add joint the island that survived
	b3AddJointToIsland(world, finalIslandId, jointId);
}

export function b3UnlinkJoint(world: b3World, jointId: i32): void {
	if (world.joints[jointId].islandId == B3_NULL_INDEX) {
		return;
	}

	// remove from island
	let islandId = world.joints[jointId].islandId;
	let island = world.islands[islandId];

	let removeIndex = world.joints[jointId].islandIndex;
	B3_ASSERT(0 <= removeIndex && removeIndex < island.joints.length);
	B3_ASSERT(island.joints[removeIndex].jointId == world.joints[jointId].jointId);

	let movedIndex = b3Array_RemoveSwap(island.joints, removeIndex);
	if (movedIndex != B3_NULL_INDEX) {
		// Fix islandIndex on the joint that was swapped into removeIndex
		let movedJointId = island.joints[removeIndex].jointId;
		B3_ASSERT(world.joints[movedJointId].islandIndex == movedIndex);
		world.joints[movedJointId].islandIndex = removeIndex;
	}

	world.joints[jointId].islandId = B3_NULL_INDEX;
	world.joints[jointId].islandIndex = B3_NULL_INDEX;
	island.constraintRemoveCount += 1;
}

// Find parent of a node. Use path halving to speed up further queries.
@inline function b3IslandFindParent(parents: StaticArray<i32>, node: i32): i32 {
	// Walk the chain of parents to find the node that is its own parent (the root)
	while (parents[node] != node) {
		let grandParent = parents[parents[node]];
		parents[node] = grandParent;
		node = grandParent;
	}

	return node;
}

// Connect the components containing node1 and node2.
// Uses rank to keep tree balanced. Tracks per-component contact and joint counts.
@inline function b3IslandUnion(
	parents: StaticArray<i32>,
	ranks: StaticArray<i32>,
	node1: i32,
	node2: i32,
	contactCounts: StaticArray<i32>,
	jointCounts: StaticArray<i32>
): void {
	let root1 = b3IslandFindParent(parents, node1);
	let root2 = b3IslandFindParent(parents, node2);
	if (root1 != root2) {
		if (ranks[root1] < ranks[root2]) {
			parents[root1] = root2;
			contactCounts[root2] += contactCounts[root1];
			jointCounts[root2] += jointCounts[root1];
		} else if (ranks[root1] > ranks[root2]) {
			parents[root2] = root1;
			contactCounts[root1] += contactCounts[root2];
			jointCounts[root1] += jointCounts[root2];
		} else {
			parents[root2] = root1;
			ranks[root1] += 1;
			contactCounts[root1] += contactCounts[root2];
			jointCounts[root1] += jointCounts[root2];
		}
	}
}

// This uses union-find.
// https://en.wikipedia.org/wiki/Disjoint-set_data_structure
export function b3SplitIsland(world: b3World, baseId: i32): void {
	let baseIsland = world.islands[baseId];
	B3_ASSERT(baseIsland.constraintRemoveCount > 0);
	B3_ASSERT(baseIsland.setIndex == b3_awakeSet);

	// The arrays are detached from the base island below, so keep them here.
	let baseBodyCount = baseIsland.bodies.length;
	let baseBodyIds = baseIsland.bodies;

	let baseContactCount = baseIsland.contacts.length;
	let baseContacts = baseIsland.contacts;

	let baseJointCount = baseIsland.joints.length;
	let baseJoints = baseIsland.joints;

	let parents = new StaticArray<i32>(baseBodyCount);
	let contactCounts = new StaticArray<i32>(baseBodyCount);
	let jointCounts = new StaticArray<i32>(baseBodyCount);
	let ranks = new StaticArray<i32>(baseBodyCount);
	for (let i = 0; i < baseBodyCount; ++i) {
		parents[i] = i;
		ranks[i] = 0;
		contactCounts[i] = 0;
		jointCounts[i] = 0;
	}

	let bodies = world.bodies;

	// Union over contacts, tracking per-component contact counts
	for (let i = 0; i < baseContactCount; ++i) {
		let bodyIdA = baseContacts[i].bodyIdA;
		let bodyIdB = baseContacts[i].bodyIdB;
		let islandIndexA = bodies[bodyIdA].islandIndex;
		let islandIndexB = bodies[bodyIdB].islandIndex;

		// Only connect non-static bodies
		if (islandIndexA != B3_NULL_INDEX && islandIndexB != B3_NULL_INDEX) {
			b3IslandUnion(parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts);
			let root = b3IslandFindParent(parents, islandIndexA);
			contactCounts[root] += 1;
		} else {
			let islandIndex = islandIndexA != B3_NULL_INDEX ? islandIndexA : islandIndexB;
			let root = b3IslandFindParent(parents, islandIndex);
			contactCounts[root] += 1;
		}
	}

	// Union over joints, tracking per-component joint counts
	for (let i = 0; i < baseJointCount; ++i) {
		let bodyIdA = baseJoints[i].bodyIdA;
		let bodyIdB = baseJoints[i].bodyIdB;
		let islandIndexA = bodies[bodyIdA].islandIndex;
		let islandIndexB = bodies[bodyIdB].islandIndex;

		// Only connect non-static bodies
		if (islandIndexA != B3_NULL_INDEX && islandIndexB != B3_NULL_INDEX) {
			b3IslandUnion(parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts);
			let root = b3IslandFindParent(parents, islandIndexA);
			jointCounts[root] += 1;
		} else {
			let islandIndex = islandIndexA != B3_NULL_INDEX ? islandIndexA : islandIndexB;
			let root = b3IslandFindParent(parents, islandIndex);
			jointCounts[root] += 1;
		}
	}

	// Flatten all parent indices and count connected components.
	let componentCount = 0;
	for (let i = 0; i < baseBodyCount; ++i) {
		parents[i] = b3IslandFindParent(parents, i);
		if (parents[i] == i) {
			componentCount += 1;
		}
	}

	// Island is still fully connected, no split needed.
	if (componentCount == 1) {
		baseIsland.constraintRemoveCount = 0;
		return;
	}

	// Detach body/contact/joint arrays from base island so b3DestroyIsland won't free them
	baseIsland.bodies = new Array<i32>();
	baseIsland.contacts = new Array<b3ContactLink>();
	baseIsland.joints = new Array<b3JointLink>();

	// Map from body index to new island index. Only set for root bodies.
	let rootMap = new StaticArray<i32>(baseBodyCount);
	for (let i = 0; i < baseBodyCount; ++i) {
		rootMap[i] = B3_NULL_INDEX;
	}

	let componentBodyCounts = new StaticArray<i32>(componentCount);
	let componentContactCounts = new StaticArray<i32>(componentCount);
	let componentJointCounts = new StaticArray<i32>(componentCount);
	let islandCount = 0;

	// Find the root body for each body and create islands as needed.
	// Extract per-component counts from the root nodes' accumulated counts.
	for (let i = 0; i < baseBodyCount; ++i) {
		let rootIndex = parents[i];
		if (rootMap[rootIndex] == B3_NULL_INDEX) {
			rootMap[rootIndex] = islandCount;
			componentBodyCounts[islandCount] = 0;
			componentContactCounts[islandCount] = contactCounts[rootIndex];
			componentJointCounts[islandCount] = jointCounts[rootIndex];
			islandCount += 1;
		}

		componentBodyCounts[rootMap[rootIndex]] += 1;
	}

	B3_ASSERT(islandCount == componentCount);

	// Map from new island index to island id
	let islandIds = new StaticArray<i32>(islandCount);

	// Create new islands. C reserves the arrays here, which has no effect on results.
	for (let i = 0; i < islandCount; ++i) {
		let newIsland = b3CreateIsland(world, b3_awakeSet);
		islandIds[i] = newIsland.islandId;
	}

	// Assign bodies to new islands
	for (let i = 0; i < baseBodyCount; ++i) {
		let bodyId = baseBodyIds[i];
		let root = b3IslandFindParent(parents, i);
		let newIslandId = islandIds[rootMap[root]];

		let newIsland = world.islands[newIslandId];

		world.bodies[bodyId].islandId = newIslandId;
		world.bodies[bodyId].islandIndex = newIsland.bodies.length;

		newIsland.bodies.push(bodyId);
	}

	// Assign contacts to the island of their bodies
	for (let i = 0; i < baseContactCount; ++i) {
		let link = baseContacts[i];
		let contact = world.contacts[link.contactId];

		// Static bodies don't have an island id.
		let islandIdA = world.bodies[link.bodyIdA].islandId;
		let targetIslandId = islandIdA != B3_NULL_INDEX ? islandIdA : world.bodies[link.bodyIdB].islandId;

		let targetIsland = world.islands[targetIslandId];
		contact.islandId = targetIslandId;
		contact.islandIndex = targetIsland.contacts.length;

		targetIsland.contacts.push(link);
	}

	// Assign joints to the island of their bodies
	for (let i = 0; i < baseJointCount; ++i) {
		let link = baseJoints[i];

		// Static bodies don't have an island id.
		let islandIdA = world.bodies[link.bodyIdA].islandId;
		let targetIslandId = islandIdA != B3_NULL_INDEX ? islandIdA : world.bodies[link.bodyIdB].islandId;

		let targetIsland = world.islands[targetIslandId];
		world.joints[link.jointId].islandId = targetIslandId;
		world.joints[link.jointId].islandIndex = targetIsland.joints.length;

		targetIsland.joints.push(link);
	}

	// Destroy the base island
	b3DestroyIsland(world, baseId);
}

// Split an island because some contacts and/or joints have been removed.
// This is called during the constraint solve while islands are not being touched. This uses union find and
// touches a lot of memory, so it can be slow.
// Note: contacts/joints connected to static bodies must belong to an island but don't affect island connectivity
// Note: static bodies are never in an island
// Note: this task interacts with some allocators without locks under the assumption that no other tasks
// are interacting with these data structures.
export function b3SplitIslandTask(world: b3World): void {
	B3_ASSERT(world.splitIslandId != B3_NULL_INDEX);

	b3SplitIsland(world, world.splitIslandId);

	world.splitIslandId = B3_NULL_INDEX;
}
