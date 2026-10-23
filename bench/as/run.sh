#!/bin/sh
# AnimaScript step timings interleaved with the native ones, one worker, for the port comparison.
# Each round runs bench/native/run.sh (SIMD and scalar) and then the AnimaScript bench for each scene,
# recording uptime before every run, since the machine may be shared; compare min over rounds.
#   ROUNDS=3 NATIVE_REPEATS=3 bench/as/run.sh
#   WASM_SIMD=off bench/as/run.sh   the scalar AnimaScript artifact
# Writes out/session_<timestamp>.log and prints the table.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
RIVE=${RIVE:-$HOME/Projects/rive-master/packages/rive-cli/out/release/rive}
WAMRC=${RIVE_WAMRC:-$HOME/Projects/tools/wamrc}
ROUNDS=${ROUNDS:-3}
NATIVE_REPEATS=${NATIVE_REPEATS:-3}
# scene:steps:define, run on the AOT tier
AOT_SCENES=${AOT_SCENES:-large_pyramid:200:1 many_bodies:100:2 pyramid:200:0}
# run on the interpreter
INTERP_SCENES=${INTERP_SCENES:-pyramid:200:0}
CACHE=${RIVE_AOT_CACHE_DIR:-$HERE/out/aotcache}
mkdir -p "$HERE/out" "$CACHE"
LOG="$HERE/out/session_$(date +%Y%m%d_%H%M%S).log"
rsync -a --delete "$HERE/../../lib/" "$HERE/lib/"
# The scalar artifact bakes from a copy of the project with wasmSimd: off, outside it so the CLI does not
# compile the copy as part of the project. WASM_SIMD=off runs the AOT scenes on it too.
SCALAR_PROJECT=${TMPDIR:-/tmp}/box3d_bench_scalar_project
mkdir -p "$SCALAR_PROJECT"
rsync -a --delete --exclude out --exclude build "$HERE/" "$SCALAR_PROJECT/"
echo "wasmSimd: off" >> "$SCALAR_PROJECT/rive.yaml"
PROJECT=$HERE
[ "${WASM_SIMD:-on}" = off ] && PROJECT=$SCALAR_PROJECT

as_run() {
	tier=$1 scene=$2 steps=$3 define=$4
	echo "UPTIME $(uptime)"
	echo "AS tier=$tier scene=$scene"
	if [ "$tier" = aot ]; then
		cd "$PROJECT"
		RIVE_WAMRC="$WAMRC" RIVE_WASM_AOT_SYNC=o3 RIVE_AOT_CACHE_DIR="$CACHE" \
			"$RIVE" . --aot --bench=$((steps + 10)) --define=SCENE="$define" --define=STEPS="$steps" 2>&1
	else
		# WAMR's fast interpreter traps on the SIMD artifact (notes/findings.md), so it runs the scalar one
		cd "$SCALAR_PROJECT"
		env -u RIVE_WAMRC "$RIVE" . --bench=$((steps + 10)) --define=SCENE="$define" --define=STEPS="$steps" 2>&1
	fi | grep -E "script tier|BENCH RESULT|bench memory"
}

{
	echo "MACHINE $(sysctl -n machdep.cpu.brand_string) cores $(sysctl -n hw.ncpu)"
	round=1
	while [ $round -le "$ROUNDS" ]; do
		echo "ROUND $round"
		for entry in $AOT_SCENES; do
			scene=$(echo "$entry" | cut -d: -f1)
			steps=$(echo "$entry" | cut -d: -f2)
			define=$(echo "$entry" | cut -d: -f3)
			REPEATS=$NATIVE_REPEATS WORKERS=1 SCENES="$scene:$steps" "$HERE/../native/run.sh" 2>&1 |
				grep -E "^UPTIME|^BUILD|^RESULT"
			as_run aot "$scene" "$steps" "$define"
		done
		for entry in $INTERP_SCENES; do
			scene=$(echo "$entry" | cut -d: -f1)
			steps=$(echo "$entry" | cut -d: -f2)
			define=$(echo "$entry" | cut -d: -f3)
			as_run interp "$scene" "$steps" "$define"
		done
		round=$((round + 1))
	done
} | tee "$LOG"

python3 "$HERE/table.py" "$LOG"
