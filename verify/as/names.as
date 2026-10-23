// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/name_cache.as, mirrors verify/cref/names.c

import { vtag, vi32, vu32, vbool, vend, vseed, vrand32, vrandint } from "./common";
import { b3AddName, b3CreateNameCache, b3DestroyNameCache, b3FindName, b3Hash32 } from "../../lib/name_cache";

export { frameBoundary } from "./common";

function rname(): string {
	let length = vrandint(0, 24);
	let s = "";
	for (let i = 0; i < length; ++i) {
		let kind = vrandint(0, 8);
		let c: i32;
		if (kind == 0) {
			c = vrandint(0x80, 0x800);
		} else if (kind == 1) {
			c = vrandint(0x800, 0xd800);
		} else {
			c = vrandint(0x20, 0x7f);
		}
		s += String.fromCharCode(c);
	}
	return s;
}

// C's strlen of the UTF-8 name
function byteLength(s: string): i32 {
	return String.UTF8.byteLength(s);
}

export function runAll(): void {
	vseed(0x6e616d6573);

	let cache = b3CreateNameCache();
	let ids = new Array<u32>();

	for (let i = 0; i < 3000; ++i) {
		let name = rname();
		let bytes = Uint8Array.wrap(String.UTF8.encode(name));

		vtag("hash");
		vi32(bytes.length);
		vu32(b3Hash32(bytes, bytes.length));
		vend();

		let id = b3AddName(cache, name);
		if (ids.length < 256) {
			ids.push(id);
		}

		vtag("add");
		vu32(id);
		let found = b3FindName(cache, id);
		vi32(found == null ? -1 : byteLength(found));
		vbool(found != null && found == name);
		vend();

		if ((vrand32() & 7) == 0) {
			vtag("again");
			vu32(b3AddName(cache, name));
			vend();
		}
	}

	for (let i = 0; i < 500; ++i) {
		let id = (vrand32() & 1) != 0 ? ids[vrandint(0, ids.length)] : vrand32();
		let found = b3FindName(cache, id);
		vtag("find");
		vu32(id);
		vi32(found == null ? -1 : byteLength(found));
		vend();
	}

	b3DestroyNameCache(cache);
}
