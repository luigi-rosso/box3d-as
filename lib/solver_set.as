// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/solver_set.c

import { b3BodySim, b3BodyState } from "./body";
import { b3IslandSim } from "./island";
import { b3JointSim } from "./joint";

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
