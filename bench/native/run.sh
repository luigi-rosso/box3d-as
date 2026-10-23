#!/bin/sh
# Native Box3D step timings for the port comparison. Records uptime before every run because the
# machine may be shared; compare min over repeats. Writes out/run_<timestamp>.log and prints a table.
#   REPEATS=5 WORKERS="1 2 4 8" ./run.sh
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
BOX3D=${BOX3D:-$HOME/Projects/box3d}
REPEATS=${REPEATS:-5}
WORKERS=${WORKERS:-1 2 4 8}
SCENES=${SCENES:-large_pyramid:200 many_bodies:100}
mkdir -p "$HERE/out"
"$HERE/build.sh"
LOG="$HERE/out/run_$(date +%Y%m%d_%H%M%S).log"

{
	echo "MACHINE $(sysctl -n machdep.cpu.brand_string 2>/dev/null || uname -m) cores $(sysctl -n hw.ncpu 2>/dev/null || nproc)"
	for entry in $SCENES; do
		scene=${entry%%:*}
		steps=${entry##*:}
		for build in simd scalar; do
			for w in $WORKERS; do
				echo "UPTIME $(uptime)"
				echo "BUILD $build"
				"$HERE/bin/bench_$build" "$scene" "$steps" "$w" "$REPEATS"
			done
		done
	done

	# Upstream's own tool as a sanity check; it writes a csv into the working directory
	cd "$HERE/out"
	for build in release scalar; do
		for w in $WORKERS; do
			echo "UPTIME $(uptime)"
			echo "UPSTREAM build=$build benchmark=large_pyramid workers=$w"
			"$BOX3D/build/$build/bin/benchmark" -b=large_pyramid -t=$w -w=$w -r=$REPEATS
		done
	done
} 2>&1 | tee "$LOG"

python3 "$HERE/table.py" "$LOG"
