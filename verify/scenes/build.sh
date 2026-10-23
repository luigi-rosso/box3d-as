#!/bin/sh
# Builds traj against the scalar parity build and the SIMD build of Box3D
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
BOX3D=${BOX3D:-$HOME/Projects/box3d}
mkdir -p "$HERE/bin"
for flavor in scalar release; do
	cmake --build "$BOX3D/build/$flavor" --target box3d > /dev/null
	name=$flavor
	[ "$flavor" = release ] && name=simd
	cc -std=c17 -O2 -DNDEBUG -ffp-contract=off -Wall -Wextra -I "$BOX3D/include" \
		"$HERE/traj.c" "$HERE/scenes.c" "$BOX3D/build/$flavor/src/libbox3d.a" -lpthread -lm \
		-o "$HERE/bin/traj_$name"
done
