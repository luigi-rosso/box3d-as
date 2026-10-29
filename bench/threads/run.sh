#!/bin/sh
# Threaded step timings, interleaved with native Box3D at the same worker counts. Each round runs, scene by
# scene, native SIMD at every worker count, the serial AnimaScript port, then every lane configuration at every
# lane count, recording uptime before each run since the machine may be shared; compare min over rounds.
#   ROUNDS=3 NATIVE_REPEATS=3 bench/threads/run.sh
#   CONFIGS="solve:2:2" LANE_COUNTS="2 4" SCENES="large_pyramid:200:1" bench/threads/run.sh
# A config is name:tasks:backend, tasks 1 collide, 2 solve, 3 both; backend 1 jobs, 2 workers, 3 parallel calls, whose
# part count comes from the job pool: n parts is --job-threads=n-1, one part --parallel-safe. AOT, SIMD artifact.
# Writes out/session_<timestamp>.log and prints the tables.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROUNDS=${ROUNDS:-3}
NATIVE_REPEATS=${NATIVE_REPEATS:-3}
LANE_COUNTS=${LANE_COUNTS:-1 2 4 8}
CONFIGS=${CONFIGS:-parallel:3:3 solve:2:2 solve-jobs:2:1}
# scene:steps:define, the define numbering in main.as
SCENES=${SCENES:-large_pyramid:200:1 many_bodies:100:2 mesh_drop:170:5 village:200:6}
NATIVE=$HERE/../native/bin/bench_simd
mkdir -p "$HERE/out"
"$HERE/../native/build.sh"
"$HERE/sync.sh"
LOG="$HERE/out/session_$(date +%Y%m%d_%H%M%S).log"

as_run() {
	echo "UPTIME $(uptime)"
	echo "AS config=$1 lanes=$2"
	threads="--job-threads=8 --worker-threads=8"
	lanes=$2
	if [ "$6" = 3 ]; then
		threads=--job-threads=$(($2 - 1))
		[ "$2" = 1 ] && threads=--parallel-safe
		lanes=1
	fi
	OPTIONS="SCENE=$4 STEPS=$3 LANES=$lanes TASKS=$5 BACKEND=$6" \
		"$HERE/run1.sh" 900 --bench=$(($3 + 10)) $threads |
		grep -E "script tier|BENCH RESULT|trapped|failed|gave up" | grep -v "^.*warning"
}

{
	echo "MACHINE $(sysctl -n machdep.cpu.brand_string) cores $(sysctl -n hw.ncpu)"
	round=1
	while [ $round -le "$ROUNDS" ]; do
		echo "ROUND $round"
		for entry in $SCENES; do
			scene=$(echo "$entry" | cut -d: -f1)
			steps=$(echo "$entry" | cut -d: -f2)
			define=$(echo "$entry" | cut -d: -f3)
			for w in $LANE_COUNTS; do
				echo "UPTIME $(uptime)"
				echo "BUILD simd"
				"$NATIVE" "$scene" "$steps" "$w" "$NATIVE_REPEATS" | grep "^RESULT"
			done
			as_run serial 0 "$steps" "$define" 0 2
			for config in $CONFIGS; do
				name=$(echo "$config" | cut -d: -f1)
				tasks=$(echo "$config" | cut -d: -f2)
				backend=$(echo "$config" | cut -d: -f3)
				for l in $LANE_COUNTS; do
					as_run "$name" "$l" "$steps" "$define" "$tasks" "$backend"
				done
			done
		done
		round=$((round + 1))
	done
} | tee "$LOG"

python3 "$HERE/table.py" "$LOG"
