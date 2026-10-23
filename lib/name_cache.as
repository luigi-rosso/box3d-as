// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/name_cache.c

import { b3Log } from "./core";

export const B3_NULL_NAME: u32 = 0;

// Replaces the entry array plus verstable map; recording and snapshots, the only iterators, are skipped.
export type b3NameCache = Map<u32, string>;

const FNV32_OFFSET_BASIS: u32 = 0x811c9dc5;
const FNV32_PRIME: u32 = 0x01000193;

// This is designed to hash small strings and while avoiding collisions
// for a 32-bit hash.
// Note: changing this will break recordings
export function b3Hash32(data: Uint8Array, length: i32): u32 {
	// FNV-1a
	let h = FNV32_OFFSET_BASIS;
	for (let i = 0; i < length; ++i) {
		h ^= <u32>data[i];
		h *= FNV32_PRIME;
	}

	// Murmur3 finalizer to help with short strings
	h ^= h >> 16;
	h *= 0x85ebca6b;
	h ^= h >> 13;
	h *= 0xc2b2ae35;
	h ^= h >> 16;
	return h;
}

export function b3CreateNameCache(): b3NameCache {
	return new Map<u32, string>();
}

export function b3DestroyNameCache(cache: b3NameCache): void {
	cache.clear();
}

// C hashes the UTF-8 bytes of the C string.
export function b3AddName(cache: b3NameCache, name: string): u32 {
	if (name.length == 0) {
		return B3_NULL_NAME;
	}

	let bytes = Uint8Array.wrap(String.UTF8.encode(name));
	let id = b3Hash32(bytes, bytes.length);

	id = id == 0 ? 1 : id;

	if (cache.has(id)) {
		if (cache.get(id) != name) {
			// Different name, same hash
			b3Log("Hash collision on " + name);
		}
		return id;
	}

	cache.set(id, name);
	return id;
}

export function b3FindName(cache: b3NameCache, id: u32): string? {
	if (cache.has(id)) {
		return cache.get(id);
	}

	return null;
}

export function b3FindNameWithDefault(cache: b3NameCache, id: u32, def: string): string {
	let name = b3FindName(cache, id);
	return name == null ? def : name;
}
