// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/constraint_graph.c

import { b3BitSet } from "./bitset";
import { B3_GRAPH_COLOR_COUNT } from "./constants";
import { b3ContactSpec } from "./contact";
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
