#!/bin/sh
# Records every scene with the scalar and SIMD builds at 1, 2, 4 and 8 workers and compares each
# recording to scalar at 1 worker. Set KEEP_ALL=1 to keep every recording, otherwise only the scalar
# 1 worker reference of the large scenes is kept.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
OUT="$HERE/out"
mkdir -p "$OUT"
"$HERE/build.sh"

SCENES="pyramid:300 large_pyramid:200 card_house:300 chains:300 capsule_mesh:300 heightfield:400 bullets:150 mover:300 many_bodies:100"
WORKERS="1 2 4 8"

for entry in $SCENES; do
	scene=${entry%%:*}
	steps=${entry##*:}
	for flavor in scalar simd; do
		for w in $WORKERS; do
			"$HERE/bin/traj_$flavor" "$scene" "$steps" "$w" "$OUT/${scene}_${flavor}_w$w.bin" > /dev/null
		done
	done
	node "$HERE/determinism.mjs" "$OUT" "$scene" $WORKERS
	if [ "${KEEP_ALL:-0}" != 1 ] && [ "$scene" = large_pyramid -o "$scene" = many_bodies ]; then
		find "$OUT" -name "${scene}_*.bin" ! -name "${scene}_scalar_w1.bin" -delete
	fi
done
