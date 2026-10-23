// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/solver_set.c

import { B3_ASSERT, B3_NULL_INDEX } from "./core";
import { b3ClearBit } from "./bitset";
import { b3AllocId, b3FreeId } from "./id_pool";
import { B3_GRAPH_COLOR_COUNT } from "./constants";
import {
	b3BodySim,
	b3BodyState,
	b3RefreshBodyContactIndices,
	b3SleepBodySimIndex,
	b3_bodyTransientFlags,
	b3_identityBodyState,
	b3_isFast,
} from "./body";
import { b3_contactTouchingFlag, b3_simMeshContact, b3_simTouchingFlag } from "./contact";
import { b3IslandSim } from "./island";
import { b3JointSim } from "./joint";
import { b3World, b3_awakeSet, b3_disabledSet, b3_firstSleepingSet } from "./physics_world";
import { B3_OVERFLOW_INDEX, b3AddContactToGraph, b3AddJointToGraph, b3RemoveJointFromGraph } from "./constraint_graph";
import { b3Array_RemoveSwap } from "./container";

// This holds solver set data. The following sets are used:
// - static set for all static bodies and joints between static bodies
// - active set for all active bodies with body states (no
// contacts or joints)
// - disabled set for disabled bodies and their joints
// - all further sets are sleeping island sets along with their contacts and joints
// The purpose of solver sets is to achieve high memory locality.
// https://www.youtube.com/watch?v=nZNd5FjSquk
export class b3SolverSet {
	// Body array. Empty for unused set.
	bodySims: Array<b3BodySim> = new Array<b3BodySim>();

	// Body state only exists for active set
	bodyStates: Array<b3BodyState> = new Array<b3BodyState>();

	// This holds sleeping/disabled joints. Empty for static/active set.
	jointSims: Array<b3JointSim> = new Array<b3JointSim>();

	// This holds all contacts for sleeping sets.
	// This holds non-touching contacts for the awake set.
	// This should be empty for the static and disabled sets.
	contactIndices: Array<i32> = new Array<i32>();

	// The awake set has an array of islands. Sleeping sets normally have a single islands. However, joints
	// created between sleeping sets causes the sets to merge, leaving them with multiple islands. These sleeping
	// islands will be naturally merged with the set is woken.
	// The static and disabled sets have no islands.
	// Islands live in the solver sets to limit the number of islands that need to be considered for sleeping.
	islandSims: Array<b3IslandSim> = new Array<b3IslandSim>();

	// Aligns with b3World::solverSetIdPool. Used to create a stable id for body/contact/joint/islands.
	setIndex: i32 = 0;
}

// functions from solver_set.c follow

// C zeroes the set in place; the port replaces the object, so references to a destroyed set go stale.
export function b3DestroySolverSet(world: b3World, setIndex: i32): void {
	b3FreeId(world.solverSetIdPool, setIndex);
	let set = new b3SolverSet();
	set.setIndex = B3_NULL_INDEX;
	world.solverSets[setIndex] = set;
}

// Wake a solver set. Does not merge islands.
// Contacts can be in several places:
// 1. non-touching contacts in the disabled set
// 2. non-touching contacts already in the awake set
// 3. touching contacts in the sleeping set
// This handles contact types 1 and 3. Type 2 doesn't need any action.
export function b3WakeSolverSet(world: b3World, setIndex: i32): void {
	B3_ASSERT(setIndex >= b3_firstSleepingSet);
	let set = world.solverSets[setIndex];
	let awakeSet = world.solverSets[b3_awakeSet];
	let disabledSet = world.solverSets[b3_disabledSet];

	let bodies = world.bodies;

	let bodyCount = set.bodySims.length;
	for (let i = 0; i < bodyCount; ++i) {
		let simSrc = set.bodySims[i];

		let bodyId = simSrc.bodyId;
		B3_ASSERT(bodies[bodyId].setIndex == setIndex);
		bodies[bodyId].setIndex = b3_awakeSet;
		bodies[bodyId].localIndex = awakeSet.bodySims.length;

		// Reset sleep timer
		bodies[bodyId].sleepTime = 0.0;

		awakeSet.bodySims.push(simSrc);

		let state = b3_identityBodyState;
		state.flags = bodies[bodyId].flags;
		awakeSet.bodyStates.push(state);

		// Move non-touching contacts from disabled set to awake set.
		let encodedBodySimIndex = bodies[bodyId].localIndex;
		let contactKey = bodies[bodyId].headContactKey;
		while (contactKey != B3_NULL_INDEX) {
			let edgeIndex = contactKey & 1;
			let contactId = contactKey >> 1;

			let contact = world.contacts[contactId];
			contactKey = contact.edges[edgeIndex].nextKey;

			if (contact.setIndex != b3_disabledSet) {
				B3_ASSERT(contact.setIndex == b3_awakeSet || contact.setIndex == setIndex);

				// If this contact is awake but not touching I need to fixup the body sim index here.
				if (contact.setIndex == b3_awakeSet) {
					B3_ASSERT(contact.colorIndex == B3_NULL_INDEX);
					if (edgeIndex == 0) {
						contact.encodedBodySimA = encodedBodySimIndex;
					} else {
						contact.encodedBodySimB = encodedBodySimIndex;
					}
				}

				continue;
			}

			let localIndex = contact.localIndex;

			B3_ASSERT(0 <= localIndex && localIndex < disabledSet.contactIndices.length);
			B3_ASSERT(disabledSet.contactIndices[localIndex] == contactId);

			B3_ASSERT((contact.flags & b3_contactTouchingFlag) == 0 && contact.manifoldCount == 0);

			contact.setIndex = b3_awakeSet;
			contact.localIndex = awakeSet.contactIndices.length;
			awakeSet.contactIndices.push(contactId);

			if (edgeIndex == 0) {
				contact.encodedBodySimA = encodedBodySimIndex;
			} else {
				contact.encodedBodySimB = encodedBodySimIndex;
			}

			let movedLocalIndex = b3Array_RemoveSwap(disabledSet.contactIndices, localIndex);
			if (movedLocalIndex != B3_NULL_INDEX) {
				// fix moved element
				let movedContactIndex = disabledSet.contactIndices[localIndex];
				let movedContact = world.contacts[movedContactIndex];
				B3_ASSERT(movedContact.localIndex == movedLocalIndex);
				movedContact.localIndex = localIndex;
			}
		}
	}

	// Transfer touching contacts from sleeping set to constraint graph.
	{
		let contactCount = set.contactIndices.length;
		for (let i = 0; i < contactCount; ++i) {
			let contactIndex = set.contactIndices[i];
			let contact = world.contacts[contactIndex];
			B3_ASSERT((contact.flags & b3_contactTouchingFlag) != 0);
			B3_ASSERT((contact.flags & b3_simTouchingFlag) != 0);
			B3_ASSERT(contact.setIndex == setIndex);
			b3AddContactToGraph(world, contact);
			contact.setIndex = b3_awakeSet;
		}
	}

	// transfer joints from sleeping set to awake set
	{
		let jointCount = set.jointSims.length;
		for (let i = 0; i < jointCount; ++i) {
			let jointSim = set.jointSims[i];
			let jointId = jointSim.jointId;
			B3_ASSERT(world.joints[jointId].setIndex == setIndex);
			b3AddJointToGraph(world, jointSim, jointId);
			world.joints[jointId].setIndex = b3_awakeSet;
		}
	}

	// transfer island from sleeping set to awake set
	// Usually a sleeping set has only one island, but it is possible
	// that joints are created between sleeping islands and they
	// are moved to the same sleeping set.
	{
		let islandCount = set.islandSims.length;
		for (let i = 0; i < islandCount; ++i) {
			let islandSrc = set.islandSims[i];
			let island = world.islands[islandSrc.islandId];
			island.setIndex = b3_awakeSet;
			island.localIndex = awakeSet.islandSims.length;
			awakeSet.islandSims.push(islandSrc);
		}
	}

	// destroy the sleeping set
	b3DestroySolverSet(world, setIndex);
}

export function b3TrySleepIsland(world: b3World, islandId: i32): void {
	let island = world.islands[islandId];
	B3_ASSERT(island.setIndex == b3_awakeSet);

	// Cannot put an island to sleep while it has a pending split and more than one body.
	if (island.constraintRemoveCount > 0 && island.bodies.length > 1) {
		return;
	}

	// island is sleeping
	// - create new sleeping solver set
	// - move island to sleeping solver set
	// - identify non-touching contacts that should move to sleeping solver set or disabled set
	// - remove old island
	// - fix island
	let sleepSetId = b3AllocId(world.solverSetIdPool);
	if (sleepSetId == world.solverSets.length) {
		let set = new b3SolverSet();
		set.setIndex = B3_NULL_INDEX;
		world.solverSets.push(set);
	}

	let sleepSet = new b3SolverSet();
	world.solverSets[sleepSetId] = sleepSet;

	let awakeSet = world.solverSets[b3_awakeSet];
	let disabledSet = world.solverSets[b3_disabledSet];
	B3_ASSERT(0 <= island.localIndex && island.localIndex < awakeSet.islandSims.length);

	sleepSet.setIndex = sleepSetId;

	// move awake bodies to sleeping set
	// this shuffles around bodies in the awake set
	{
		let bodies = world.bodies;
		for (let i = 0; i < island.bodies.length; ++i) {
			let bodyId = island.bodies[i];
			B3_ASSERT(bodies[bodyId].setIndex == b3_awakeSet);
			B3_ASSERT(bodies[bodyId].islandId == islandId);
			B3_ASSERT(bodies[bodyId].islandIndex == i);

			// Update the body move event to indicate this body fell asleep
			// It could happen the body is forced asleep before it ever moves.
			let bodyMoveIndex = bodies[bodyId].bodyMoveIndex;
			if (bodyMoveIndex != B3_NULL_INDEX) {
				// A body disabled and enabled since its last step keeps a stale index that C may write past the
				// event count, where the next step overwrites it; the port skips that write instead of trapping.
				if (bodyMoveIndex < world.bodyMoveEvents.length) {
					B3_ASSERT(world.bodyMoveEvents[bodyMoveIndex].bodyId.index1 - 1 == bodyId);
					B3_ASSERT(world.bodyMoveEvents[bodyMoveIndex].bodyId.generation == bodies[bodyId].generation);
					world.bodyMoveEvents[bodyMoveIndex].fellAsleep = true;
				}
				bodies[bodyId].bodyMoveIndex = B3_NULL_INDEX;
			}

			let awakeBodyIndex = bodies[bodyId].localIndex;

			// move body sim to sleep set
			let sleepBodyIndex = sleepSet.bodySims.length;
			sleepSet.bodySims.push(awakeSet.bodySims[awakeBodyIndex]);

			let movedIndex = b3Array_RemoveSwap(awakeSet.bodySims, awakeBodyIndex);
			if (movedIndex != B3_NULL_INDEX) {
				// fix local index on moved element
				let movedId = awakeSet.bodySims[awakeBodyIndex].bodyId;
				B3_ASSERT(bodies[movedId].localIndex == movedIndex);
				bodies[movedId].localIndex = awakeBodyIndex;

				if (bodies[movedId].islandId != islandId) {
					b3RefreshBodyContactIndices(world, movedId);
				}
			}

			// destroy state, no need to clone
			b3Array_RemoveSwap(awakeSet.bodyStates, awakeBodyIndex);

			bodies[bodyId].setIndex = sleepSetId;
			bodies[bodyId].localIndex = sleepBodyIndex;

			// Move non-touching contacts to the disabled set.
			// Non-touching contacts may exist between sleeping islands and there is no clear ownership.
			let contactKey = bodies[bodyId].headContactKey;
			while (contactKey != B3_NULL_INDEX) {
				let contactId = contactKey >> 1;
				let edgeIndex = contactKey & 1;

				let contact = world.contacts[contactId];

				B3_ASSERT(contact.setIndex == b3_awakeSet);

				contactKey = contact.edges[edgeIndex].nextKey;

				if (contact.colorIndex != B3_NULL_INDEX) {
					// contact is touching and will be moved separately
					B3_ASSERT((contact.flags & b3_contactTouchingFlag) != 0);
					continue;
				}

				if (edgeIndex == 0) {
					contact.encodedBodySimA = B3_NULL_INDEX;
				} else {
					contact.encodedBodySimB = B3_NULL_INDEX;
				}

				// the other body may still be awake, it still may go to sleep and then it will be responsible
				// for moving this contact to the disabled set.
				let otherEdgeIndex = edgeIndex ^ 1;
				let otherBodyId = contact.edges[otherEdgeIndex].bodyId;
				if (bodies[otherBodyId].setIndex == b3_awakeSet) {
					continue;
				}

				let localIndex = contact.localIndex;
				B3_ASSERT(awakeSet.contactIndices[localIndex] == contactId);

				B3_ASSERT(contact.manifoldCount == 0);
				B3_ASSERT((contact.flags & b3_contactTouchingFlag) == 0);

				// Move the non-touching contact to the disabled set.
				contact.setIndex = b3_disabledSet;

				// This is mandatory for validation to work correctly
				contact.localIndex = disabledSet.contactIndices.length;
				disabledSet.contactIndices.push(contact.contactId);

				let movedLocalIndex = b3Array_RemoveSwap(awakeSet.contactIndices, localIndex);
				if (movedLocalIndex != B3_NULL_INDEX) {
					// fix moved element
					let movedContactIndex = awakeSet.contactIndices[localIndex];
					let movedContact = world.contacts[movedContactIndex];
					B3_ASSERT(movedContact.localIndex == movedLocalIndex);
					movedContact.localIndex = localIndex;
				}
			}
		}
	}

	// move touching contacts to sleeping set
	// this shuffles contacts in the awake set
	{
		for (let i = 0; i < island.contacts.length; ++i) {
			let contactId = island.contacts[i].contactId;
			let contact = world.contacts[contactId];
			B3_ASSERT(contact.setIndex == b3_awakeSet);
			B3_ASSERT(contact.islandId == islandId);
			B3_ASSERT(contact.islandIndex == i);
			let colorIndex = contact.colorIndex;
			B3_ASSERT(0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT);

			let color = world.constraintGraph.colors[colorIndex];

			// Remove bodies from graph coloring associated with this constraint
			if (colorIndex != B3_OVERFLOW_INDEX) {
				// might clear a bit for a static body, but this has no effect
				b3ClearBit(color.bodySet, <u32>contact.edges[0].bodyId);
				b3ClearBit(color.bodySet, <u32>contact.edges[1].bodyId);
			}

			let sleepContactIndex = sleepSet.contactIndices.length;
			sleepSet.contactIndices.push(contactId);

			let localIndex = contact.localIndex;
			if ((contact.flags & b3_simMeshContact) != 0 || colorIndex == B3_OVERFLOW_INDEX) {
				let movedLocalIndex = b3Array_RemoveSwap(color.contacts, localIndex);
				if (movedLocalIndex != B3_NULL_INDEX) {
					// fix moved element
					let movedContactId = color.contacts[localIndex].contactId;
					let movedContact = world.contacts[movedContactId];
					B3_ASSERT(movedContact.localIndex == movedLocalIndex);
					movedContact.localIndex = localIndex;
				}
			} else {
				let movedLocalIndex = b3Array_RemoveSwap(color.convexContacts, localIndex);
				if (movedLocalIndex != B3_NULL_INDEX) {
					// fix moved element
					let movedContactId = color.convexContacts[localIndex];
					let movedContact = world.contacts[movedContactId];
					B3_ASSERT(movedContact.localIndex == movedLocalIndex);
					movedContact.localIndex = localIndex;
				}
			}

			contact.setIndex = sleepSetId;
			contact.colorIndex = B3_NULL_INDEX;
			contact.localIndex = sleepContactIndex;
			contact.encodedBodySimA = b3SleepBodySimIndex(contact.encodedBodySimA);
			contact.encodedBodySimB = b3SleepBodySimIndex(contact.encodedBodySimB);
		}
	}

	// move joints
	// this shuffles joints in the awake set
	{
		for (let i = 0; i < island.joints.length; ++i) {
			let jointId = island.joints[i].jointId;
			B3_ASSERT(world.joints[jointId].setIndex == b3_awakeSet);
			B3_ASSERT(world.joints[jointId].islandId == islandId);
			B3_ASSERT(world.joints[jointId].islandIndex == i);
			let colorIndex = world.joints[jointId].colorIndex;
			let localIndex = world.joints[jointId].localIndex;

			B3_ASSERT(0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT);

			let color = world.constraintGraph.colors[colorIndex];

			let awakeJointSim = color.jointSims[localIndex];

			if (colorIndex != B3_OVERFLOW_INDEX) {
				// might clear a bit for a static body, but this has no effect
				b3ClearBit(color.bodySet, <u32>world.joints[jointId].edges[0].bodyId);
				b3ClearBit(color.bodySet, <u32>world.joints[jointId].edges[1].bodyId);
			}

			let sleepJointIndex = sleepSet.jointSims.length;
			sleepSet.jointSims.push(awakeJointSim);

			let movedIndex = b3Array_RemoveSwap(color.jointSims, localIndex);
			if (movedIndex != B3_NULL_INDEX) {
				// fix moved element
				let movedId = color.jointSims[localIndex].jointId;
				B3_ASSERT(world.joints[movedId].localIndex == movedIndex);
				world.joints[movedId].localIndex = localIndex;
			}

			world.joints[jointId].setIndex = sleepSetId;
			world.joints[jointId].colorIndex = B3_NULL_INDEX;
			world.joints[jointId].localIndex = sleepJointIndex;
		}
	}

	// move island struct
	{
		B3_ASSERT(island.setIndex == b3_awakeSet);

		let islandIndex = island.localIndex;
		let sleepIsland = b3IslandSim();
		sleepIsland.islandId = islandId;
		sleepSet.islandSims.push(sleepIsland);

		let movedIslandIndex = b3Array_RemoveSwap(awakeSet.islandSims, islandIndex);
		if (movedIslandIndex != B3_NULL_INDEX) {
			// fix index on moved element
			let movedIslandId = awakeSet.islandSims[islandIndex].islandId;
			let movedIsland = world.islands[movedIslandId];
			B3_ASSERT(movedIsland.localIndex == movedIslandIndex);
			movedIsland.localIndex = islandIndex;
		}

		island.setIndex = sleepSetId;
		island.localIndex = 0;
	}

	if (world.splitIslandId == islandId) {
		world.splitIslandId = B3_NULL_INDEX;
	}
}

// This is called when joints are created between sets. I want to allow the sets
// to continue sleeping if both are asleep. Otherwise one set is waked.
// Islands will get merge when the set is woke.
export function b3MergeSolverSets(world: b3World, setId1: i32, setId2: i32): void {
	B3_ASSERT(setId1 >= b3_firstSleepingSet);
	B3_ASSERT(setId2 >= b3_firstSleepingSet);
	let set1 = world.solverSets[setId1];
	let set2 = world.solverSets[setId2];

	// Move the fewest number of bodies
	if (set1.bodySims.length < set2.bodySims.length) {
		let tempSet = set1;
		set1 = set2;
		set2 = tempSet;

		let tempId = setId1;
		setId1 = setId2;
		setId2 = tempId;
	}

	// transfer bodies
	{
		let bodies = world.bodies;
		let bodyCount = set2.bodySims.length;
		for (let i = 0; i < bodyCount; ++i) {
			let simSrc = set2.bodySims[i];

			let bodyId = simSrc.bodyId;
			B3_ASSERT(bodies[bodyId].setIndex == setId2);
			bodies[bodyId].setIndex = setId1;
			bodies[bodyId].localIndex = set1.bodySims.length;

			set1.bodySims.push(simSrc);
		}
	}

	// transfer contacts
	{
		let contactCount = set2.contactIndices.length;
		for (let i = 0; i < contactCount; ++i) {
			let contactIndex = set2.contactIndices[i];
			let contact = world.contacts[contactIndex];
			B3_ASSERT(contact.setIndex == setId2);
			contact.setIndex = setId1;
			contact.localIndex = set1.contactIndices.length;
			set1.contactIndices.push(contactIndex);
		}
	}

	// transfer joints
	{
		let jointCount = set2.jointSims.length;
		for (let i = 0; i < jointCount; ++i) {
			let jointSrc = set2.jointSims[i];

			let jointId = jointSrc.jointId;
			B3_ASSERT(world.joints[jointId].setIndex == setId2);
			world.joints[jointId].setIndex = setId1;
			world.joints[jointId].localIndex = set1.jointSims.length;

			set1.jointSims.push(jointSrc);
		}
	}

	// transfer islands
	{
		let islandCount = set2.islandSims.length;
		for (let i = 0; i < islandCount; ++i) {
			let islandSrc = set2.islandSims[i];
			let islandId = islandSrc.islandId;

			let island = world.islands[islandId];
			island.setIndex = setId1;
			island.localIndex = set1.islandSims.length;

			set1.islandSims.push(islandSrc);
		}
	}

	// destroy the merged set
	// Warning: need to be careful not to destroy things that got transferred, like triangle caches.
	b3DestroySolverSet(world, setId2);
}

export function b3TransferBody(world: b3World, targetSet: b3SolverSet, sourceSet: b3SolverSet, bodyId: i32): void {
	B3_ASSERT(world.bodies[bodyId].headContactKey == B3_NULL_INDEX);

	if (targetSet == sourceSet) {
		return;
	}

	let sourceIndex = world.bodies[bodyId].localIndex;

	let targetIndex = targetSet.bodySims.length;
	targetSet.bodySims.push(sourceSet.bodySims[sourceIndex]);

	// Clear transient body flags
	world.bodies[bodyId].flags &= ~b3_bodyTransientFlags;
	targetSet.bodySims[targetIndex].flags &= ~(b3_isFast | b3_bodyTransientFlags);

	// Remove body sim from solver set that owns it
	let movedIndex = b3Array_RemoveSwap(sourceSet.bodySims, sourceIndex);
	if (movedIndex != B3_NULL_INDEX) {
		// Fix moved body index
		let movedId = sourceSet.bodySims[sourceIndex].bodyId;
		B3_ASSERT(world.bodies[movedId].localIndex == movedIndex);
		world.bodies[movedId].localIndex = sourceIndex;
		b3RefreshBodyContactIndices(world, movedId);
	}

	if (sourceSet.setIndex == b3_awakeSet) {
		b3Array_RemoveSwap(sourceSet.bodyStates, sourceIndex);
	} else if (targetSet.setIndex == b3_awakeSet) {
		let state = b3_identityBodyState;
		state.flags = world.bodies[bodyId].flags;
		targetSet.bodyStates.push(state);
	}

	world.bodies[bodyId].setIndex = targetSet.setIndex;
	world.bodies[bodyId].localIndex = targetIndex;
}

export function b3TransferJoint(world: b3World, targetSet: b3SolverSet, sourceSet: b3SolverSet, jointId: i32): void {
	if (targetSet == sourceSet) {
		return;
	}

	let localIndex = world.joints[jointId].localIndex;
	let colorIndex = world.joints[jointId].colorIndex;

	// Retrieve source.
	let sourceSim: b3JointSim;
	if (sourceSet.setIndex == b3_awakeSet) {
		B3_ASSERT(0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT);
		let color = world.constraintGraph.colors[colorIndex];

		sourceSim = color.jointSims[localIndex];
	} else {
		B3_ASSERT(colorIndex == B3_NULL_INDEX);
		sourceSim = sourceSet.jointSims[localIndex];
	}

	// Create target and move. Fix joint.
	if (targetSet.setIndex == b3_awakeSet) {
		b3AddJointToGraph(world, sourceSim, jointId);
		world.joints[jointId].setIndex = b3_awakeSet;
	} else {
		world.joints[jointId].setIndex = targetSet.setIndex;
		world.joints[jointId].localIndex = targetSet.jointSims.length;
		world.joints[jointId].colorIndex = B3_NULL_INDEX;

		targetSet.jointSims.push(sourceSim);
	}

	// Destroy source.
	if (sourceSet.setIndex == b3_awakeSet) {
		b3RemoveJointFromGraph(
			world,
			world.joints[jointId].edges[0].bodyId,
			world.joints[jointId].edges[1].bodyId,
			colorIndex,
			localIndex
		);
	} else {
		let movedIndex = b3Array_RemoveSwap(sourceSet.jointSims, localIndex);
		if (movedIndex != B3_NULL_INDEX) {
			// fix swapped element
			let movedId = sourceSet.jointSims[localIndex].jointId;
			world.joints[movedId].localIndex = localIndex;
		}
	}
}
