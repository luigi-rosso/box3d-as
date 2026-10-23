#!/bin/sh
# Builds bench against the SIMD and scalar builds of Box3D
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
BOX3D=${BOX3D:-$HOME/Projects/box3d}
SCENES="$HERE/../../verify/scenes"
mkdir -p "$HERE/bin"
for flavor in scalar release; do
	cmake --build "$BOX3D/build/$flavor" > /dev/null
	name=$flavor
	[ "$flavor" = release ] && name=simd
	cc -std=c17 -O3 -DNDEBUG -ffp-contract=off -Wall -Wextra -I "$BOX3D/include" -I "$SCENES" \
		"$HERE/bench.c" "$SCENES/scenes.c" "$BOX3D/build/$flavor/src/libbox3d.a" -lpthread -lm \
		-o "$HERE/bin/bench_$name"
done
