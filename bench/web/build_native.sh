#!/bin/sh
# Builds out/bin/native_simd and native_scalar: the runner's twin against the native NEON and scalar Box3D builds
set -e
here="$(cd "$(dirname "$0")" && pwd)"
BOX3D=${BOX3D:-$HOME/Projects/box3d}
scenes="$here/../../verify/scenes"
mkdir -p "$here/out/bin"
for flavor in release scalar; do
	name=native_scalar
	[ "$flavor" = release ] && name=native_simd
	cc -std=c17 -O3 -DNDEBUG -ffp-contract=off -Wall -Wextra -I "$BOX3D/include" -I "$scenes" "$here/c/native.c" \
		"$here/c/driver.c" "$scenes/scenes.c" "$BOX3D/build/$flavor/src/libbox3d.a" -lpthread -lm -o "$here/out/bin/$name"
done
