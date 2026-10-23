// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/compound.c

import { b3CompoundData } from "./collision";
import { b3TreeView } from "./dynamic_tree";

export function b3GetCompoundTreeView(compound: b3CompoundData): b3TreeView {
	let view = new b3TreeView(compound.nodes, null);
	return view;
}

// functions from compound.c follow
