// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/island.c

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
