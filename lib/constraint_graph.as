// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/constraint_graph.c

import {
	b3BitSet,
	b3ClearBit,
	b3CreateBitSet,
	b3DestroyBitSet,
	b3GetBit,
	b3SetBitCountAndClear,
	b3SetBitGrow,
} from "./bitset";
import { B3_ASSERT, B3_NULL_INDEX, UINT16_MAX } from "./core";
import { B3_GRAPH_COLOR_COUNT } from "./constants";
import { b3MaxInt } from "./math_functions";
import { b3BodyType, b3_dynamicBody } from "./types";
import { b3EncodeBodySimIndex } from "./body";
import { b3Contact, b3ContactSpec, b3_contactTouchingFlag, b3_simMeshContact } from "./contact";
import { b3World, b3_awakeSet } from "./physics_world";
import { b3Array_RemoveSwap } from "./container";
import {
	b3ContactConstraint,
	b3ContactConstraintWide,
	b3ContactManifoldWide,
	b3ManifoldConstraint,
} from "./contact_solver";
import { b3JointSim } from "./joint";

// This holds constraints that cannot fit the graph color limit. This happens when a single dynamic body
// is touching many other bodies.
export const B3_OVERFLOW_INDEX: i32 = B3_GRAPH_COLOR_COUNT - 1;

// This keeps constraints involving two dynamic bodies at a lower solver priority than constraints
// involving a dynamic and static bodies. This reduces tunneling due to push through.
export const B3_DYNAMIC_COLOR_COUNT: i32 = B3_GRAPH_COLOR_COUNT - 4;

// todo optimize mesh contact constraints
// They can be lumped with convex contacts and I can use the bitset event to re-link if the manifold count increases
// Could also create a group for two wide manifolds and use bitset event
// This could create ping-pong jitter so may need to pin to high water mark or introduce hysteresis somehow
//
// Dirk has the idea to do graph coloring based on manifolds. This suggests mesh contact will have manifolds
// in multiple graph colors. So each manifold with have a color and local index.
// Some concerns about this:
// - manifolds don't have a strong identity, would this affect stability/jitter?
// - this creates a lot of static graph colors and can overflow

const b3_emptyWideConstraints = new StaticArray<b3ContactConstraintWide>(0);
const b3_emptyWideManifolds = new StaticArray<b3ContactManifoldWide>(0);
const b3_emptyManifoldConstraints = new StaticArray<b3ManifoldConstraint>(0);
const b3_emptyContactConstraints = new StaticArray<b3ContactConstraint>(0);

export class b3GraphColor {
	// This bitset is indexed by bodyId so this is over-sized to encompass static bodies
	// however I never traverse these bits or use the bit count for anything
	// This bitset is unused on the overflow color.
	bodySet: b3BitSet = new b3BitSet();

	// cache friendly arrays
	jointSims: Array<b3JointSim> = new Array<b3JointSim>();

	convexContacts: Array<i32> = new Array<i32>();
	contacts: Array<b3ContactSpec> = new Array<b3ContactSpec>();

	// These are used for convex contacts. C points at this color's slice; here the arrays
	// are the step context's, and the slice begins at wideConstraintStart.
	// wideManifolds[i] belongs to wideConstraints[i].
	wideConstraints: StaticArray<b3ContactConstraintWide> = b3_emptyWideConstraints;
	wideManifolds: StaticArray<b3ContactManifoldWide> = b3_emptyWideManifolds;
	wideConstraintStart: i32 = 0;
	wideConstraintCount: i32 = 0;

	// Mesh constraints are the step context's arrays from wideMeshConstraintStart. The starts view
	// begins at this color's first group and holds indices into wideMeshManifolds.
	wideMeshConstraints: StaticArray<b3ContactConstraintWide> = b3_emptyWideConstraints;
	wideMeshManifolds: StaticArray<b3ContactManifoldWide> = b3_emptyWideManifolds;
	wideMeshConstraintStart: i32 = 0;
	wideMeshManifoldStarts: Int32Array = new Int32Array(0);
	wideMeshConstraintCount: i32 = 0;

	// These are used for mesh and overflow contacts
	manifoldConstraints: StaticArray<b3ManifoldConstraint> = b3_emptyManifoldConstraints;
	contactConstraints: StaticArray<b3ContactConstraint> = b3_emptyContactConstraints;
}

export class b3ConstraintGraph {
	// including overflow at the end
	colors: StaticArray<b3GraphColor> = new StaticArray<b3GraphColor>(B3_GRAPH_COLOR_COUNT);

	constructor() {
		for (let i = 0; i < B3_GRAPH_COLOR_COUNT; ++i) {
			this.colors[i] = new b3GraphColor();
		}
	}
}

// functions from constraint_graph.c follow

// Solver using graph coloring. Islands are only used for sleep.
// High-Performance Physical Simulations on Next-Generation Architecture with Many Cores
// http://web.eecs.umich.edu/~msmelyan/papers/physsim_onmanycore_itj.pdf

// Kinematic bodies have to be treated like dynamic bodies in graph coloring. Unlike static bodies, we cannot use a dummy solver
// body for kinematic bodies. We cannot access a kinematic body from multiple threads efficiently because the SIMD solver body
// scatter would write to the same kinematic body from multiple threads. Even if these writes don't modify the body, they will
// cause horrible cache stalls. To make this feasible I would need a way to block these writes.

// This is used for debugging by making all constraints be assigned to overflow.
const B3_FORCE_OVERFLOW: i32 = 0;

export function b3CreateGraph(graph: b3ConstraintGraph, bodyCapacity: i32): void {
	for (let i = 0; i < B3_GRAPH_COLOR_COUNT; ++i) {
		graph.colors[i] = new b3GraphColor();
	}

	bodyCapacity = b3MaxInt(bodyCapacity, 8);

	// Initialize graph color bit set.
	// No bitset for overflow color.
	for (let i = 0; i < B3_OVERFLOW_INDEX; ++i) {
		let color = graph.colors[i];
		color.bodySet = b3CreateBitSet(<u32>bodyCapacity);
		b3SetBitCountAndClear(color.bodySet, <u32>bodyCapacity);
	}
}

export function b3DestroyGraph(graph: b3ConstraintGraph): void {
	for (let i = 0; i < B3_GRAPH_COLOR_COUNT; ++i) {
		let color = graph.colors[i];

		// The bit set should never be used on the overflow color
		B3_ASSERT(i != B3_OVERFLOW_INDEX || color.bodySet.bits.length == 0);

		b3DestroyBitSet(color.bodySet);

		color.convexContacts = new Array<i32>();
		color.contacts = new Array<b3ContactSpec>();
		color.jointSims = new Array<b3JointSim>();
	}
}

// Contacts are always created as non-touching. They get cloned into the constraint
// graph once they are found to be touching.
export function b3AddContactToGraph(world: b3World, contact: b3Contact): void {
	B3_ASSERT(contact.manifoldCount > 0);
	B3_ASSERT((contact.flags & b3_contactTouchingFlag) != 0);

	let graph = world.constraintGraph;
	let colorIndex = B3_OVERFLOW_INDEX;

	let bodyIdA = contact.edges[0].bodyId;
	let bodyIdB = contact.edges[1].bodyId;
	let bodyA = world.bodies[bodyIdA];
	let bodyB = world.bodies[bodyIdB];

	let typeA = bodyA.type;
	let typeB = bodyB.type;
	B3_ASSERT(typeA == b3_dynamicBody || typeB == b3_dynamicBody);

	if (B3_FORCE_OVERFLOW == 0) {
		if (typeA == b3_dynamicBody && typeB == b3_dynamicBody) {
			// Dynamic constraint colors cannot encroach on colors reserved for static constraints
			for (let i = 0; i < B3_DYNAMIC_COLOR_COUNT; ++i) {
				let color = graph.colors[i];
				if (b3GetBit(color.bodySet, <u32>bodyIdA) || b3GetBit(color.bodySet, <u32>bodyIdB)) {
					continue;
				}

				b3SetBitGrow(color.bodySet, <u32>bodyIdA);
				b3SetBitGrow(color.bodySet, <u32>bodyIdB);
				colorIndex = i;
				break;
			}
		} else if (typeA == b3_dynamicBody) {
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for (let i = B3_OVERFLOW_INDEX - 1; i >= 1; --i) {
				let color = graph.colors[i];
				if (b3GetBit(color.bodySet, <u32>bodyIdA)) {
					continue;
				}

				b3SetBitGrow(color.bodySet, <u32>bodyIdA);
				colorIndex = i;
				break;
			}
		} else if (typeB == b3_dynamicBody) {
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for (let i = B3_OVERFLOW_INDEX - 1; i >= 1; --i) {
				let color = graph.colors[i];
				if (b3GetBit(color.bodySet, <u32>bodyIdB)) {
					continue;
				}

				b3SetBitGrow(color.bodySet, <u32>bodyIdB);
				colorIndex = i;
				break;
			}
		}
	}

	let isScalar = (contact.flags & b3_simMeshContact) != 0 || colorIndex == B3_OVERFLOW_INDEX;

	let color = graph.colors[colorIndex];
	contact.colorIndex = colorIndex;
	contact.localIndex = isScalar ? color.contacts.length : color.convexContacts.length;
	contact.encodedBodySimA = b3EncodeBodySimIndex(bodyA);
	contact.encodedBodySimB = b3EncodeBodySimIndex(bodyB);

	if (isScalar) {
		B3_ASSERT(contact.manifoldCount < UINT16_MAX);
		let spec = b3ContactSpec();
		spec.contactId = contact.contactId;
		spec.manifoldStart = 0;
		spec.manifoldCount = <u16>contact.manifoldCount;
		color.contacts.push(spec);
	} else {
		color.convexContacts.push(contact.contactId);
	}
}

export function b3RemoveContactFromGraph(
	world: b3World,
	bodyIdA: i32,
	bodyIdB: i32,
	colorIndex: i32,
	localIndex: i32,
	meshContact: bool
): void {
	let graph = world.constraintGraph;

	B3_ASSERT(0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT);
	let color = graph.colors[colorIndex];

	if (colorIndex != B3_OVERFLOW_INDEX) {
		// This might clear a bit for a static body, but this has no effect
		b3ClearBit(color.bodySet, <u32>bodyIdA);
		b3ClearBit(color.bodySet, <u32>bodyIdB);
	}

	if (meshContact || colorIndex == B3_OVERFLOW_INDEX) {
		let movedIndex = b3Array_RemoveSwap(color.contacts, localIndex);
		if (movedIndex != B3_NULL_INDEX) {
			// Fix index on swapped contact
			let movedContactId = color.contacts[localIndex].contactId;
			let movedContact = world.contacts[movedContactId];
			B3_ASSERT(movedContact.setIndex == b3_awakeSet);
			B3_ASSERT(movedContact.colorIndex == colorIndex);
			B3_ASSERT(movedContact.localIndex == movedIndex);
			movedContact.localIndex = localIndex;
		}
	} else {
		let movedIndex = b3Array_RemoveSwap(color.convexContacts, localIndex);
		if (movedIndex != B3_NULL_INDEX) {
			// Fix index on swapped contact
			let movedContactId = color.convexContacts[localIndex];
			let movedContact = world.contacts[movedContactId];
			B3_ASSERT(movedContact.setIndex == b3_awakeSet);
			B3_ASSERT(movedContact.colorIndex == colorIndex);
			B3_ASSERT(movedContact.localIndex == movedIndex);
			B3_ASSERT((movedContact.flags & b3_simMeshContact) == 0);
			movedContact.localIndex = localIndex;
		}
	}
}

function b3AssignJointColor(
	graph: b3ConstraintGraph,
	bodyIdA: i32,
	bodyIdB: i32,
	typeA: b3BodyType,
	typeB: b3BodyType
): i32 {
	B3_ASSERT(typeA == b3_dynamicBody || typeB == b3_dynamicBody);

	if (B3_FORCE_OVERFLOW == 0) {
		if (typeA == b3_dynamicBody && typeB == b3_dynamicBody) {
			// Dynamic constraint colors cannot encroach on colors reserved for static constraints
			for (let i = 0; i < B3_DYNAMIC_COLOR_COUNT; ++i) {
				let color = graph.colors[i];
				if (b3GetBit(color.bodySet, <u32>bodyIdA) || b3GetBit(color.bodySet, <u32>bodyIdB)) {
					continue;
				}

				b3SetBitGrow(color.bodySet, <u32>bodyIdA);
				b3SetBitGrow(color.bodySet, <u32>bodyIdB);
				return i;
			}
		} else if (typeA == b3_dynamicBody) {
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for (let i = B3_OVERFLOW_INDEX - 1; i >= 1; --i) {
				let color = graph.colors[i];
				if (b3GetBit(color.bodySet, <u32>bodyIdA)) {
					continue;
				}

				b3SetBitGrow(color.bodySet, <u32>bodyIdA);
				return i;
			}
		} else if (typeB == b3_dynamicBody) {
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for (let i = B3_OVERFLOW_INDEX - 1; i >= 1; --i) {
				let color = graph.colors[i];
				if (b3GetBit(color.bodySet, <u32>bodyIdB)) {
					continue;
				}

				b3SetBitGrow(color.bodySet, <u32>bodyIdB);
				return i;
			}
		}
	}

	return B3_OVERFLOW_INDEX;
}

// Pushes a zeroed joint sim into the joint's graph color and returns it.
export function b3CreateJointInGraph(world: b3World, jointId: i32): b3JointSim {
	let graph = world.constraintGraph;

	let bodyIdA = world.joints[jointId].edges[0].bodyId;
	let bodyIdB = world.joints[jointId].edges[1].bodyId;
	let typeA = world.bodies[bodyIdA].type;
	let typeB = world.bodies[bodyIdB].type;

	let colorIndex = b3AssignJointColor(graph, bodyIdA, bodyIdB, typeA, typeB);

	let jointSim = new b3JointSim();
	graph.colors[colorIndex].jointSims.push(jointSim);

	world.joints[jointId].colorIndex = colorIndex;
	world.joints[jointId].localIndex = graph.colors[colorIndex].jointSims.length - 1;
	return jointSim;
}

// C copies the sim into the new slot and the caller then removes the source, so the port moves the reference.
export function b3AddJointToGraph(world: b3World, jointSim: b3JointSim, jointId: i32): void {
	b3CreateJointInGraph(world, jointId);
	let joint = world.joints[jointId];
	world.constraintGraph.colors[joint.colorIndex].jointSims[joint.localIndex] = jointSim;
}

export function b3RemoveJointFromGraph(world: b3World, bodyIdA: i32, bodyIdB: i32, colorIndex: i32, localIndex: i32): void {
	let graph = world.constraintGraph;

	B3_ASSERT(0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT);
	let color = graph.colors[colorIndex];

	if (colorIndex != B3_OVERFLOW_INDEX) {
		// May clear static bodies, no effect
		b3ClearBit(color.bodySet, <u32>bodyIdA);
		b3ClearBit(color.bodySet, <u32>bodyIdB);
	}

	let movedIndex = b3Array_RemoveSwap(color.jointSims, localIndex);
	if (movedIndex != B3_NULL_INDEX) {
		// Fix moved joint
		let movedJointSim = color.jointSims[localIndex];
		let movedId = movedJointSim.jointId;
		B3_ASSERT(world.joints[movedId].setIndex == b3_awakeSet);
		B3_ASSERT(world.joints[movedId].colorIndex == colorIndex);
		B3_ASSERT(world.joints[movedId].localIndex == movedIndex);
		world.joints[movedId].localIndex = localIndex;
	}
}
