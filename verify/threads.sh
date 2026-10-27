#!/bin/bash
# Threaded trajectory parity: runs the trajectory scenes in rive-cli with the narrow phase and the solver stages on
# lanes, at each lane count and backend, and compares every step's state hash with the native scalar recording in
# verify/scenes/out (verify/traj.sh records them with NATIVE=1). node cannot run lanes, so this is the gate for
# threaded runs; verify/traj.sh stays the gate for the serial port.
#   verify/threads.sh                 every scene except large_pyramid, many_bodies, mesh_drop and village
#   verify/threads.sh --large         all scenes
#   verify/threads.sh chains village  only those
# LANES="1 2 4 8" BACKENDS="2 1" (2 workers, 1 jobs, 3 parallel calls) TASKS=3 (1 collide, 2 solve, 3 both) pick the
# runs; LANES=1 also runs the main lane's partition through the serialized replica path. Parallel calls take their
# part count from the job pool, so backend 3 runs PARTS="1 2 4 8" instead, 1 being --parallel-safe, plus the default
# pool, and needs the parallel-calls toolchain, as it brings in lib/parallel/. WASM_SIMD=off takes the scalar artifact.
here="$(cd "$(dirname "$0")" && pwd)"
project="$here/../bench/threads"
lanes=${LANES:-1 2 4 8}
parts=${PARTS:-1 2 4 8 default}
backends=${BACKENDS:-2 1}
tasks=${TASKS:-3}

scenes=()
large=0
for a in "$@"; do
	if [ "$a" = "--large" ]; then large=1; else scenes+=("$a"); fi
done
if [ ${#scenes[@]} -eq 0 ]; then
	scenes=(pyramid card_house chains capsule_mesh heightfield bullets mover plank_pile queries)
	[ $large -eq 1 ] && scenes+=(large_pyramid many_bodies mesh_drop village)
fi

scene_define() {
	case $1 in
		pyramid) echo 0 ;; large_pyramid) echo 1 ;; many_bodies) echo 2 ;; chains) echo 3 ;; plank_pile) echo 4 ;;
		mesh_drop) echo 5 ;; village) echo 6 ;; queries) echo 7 ;; capsule_mesh) echo 8 ;; heightfield) echo 9 ;;
		mover) echo 10 ;; card_house) echo 11 ;; bullets) echo 12 ;;
	esac
}

case " $backends " in *" 3 "*) export PARALLEL=1 ;; esac
"$project/sync.sh"
run_dir="$project"
if [ "${WASM_SIMD:-on}" = off ]; then
	# The scalar artifact bakes from a copy with wasmSimd: off, outside the project so the CLI does not compile it
	run_dir=${TMPDIR:-/tmp}/box3d_threads_scalar_$(echo "$project" | md5 | cut -c1-8)
	mkdir -p "$run_dir"
	rsync -a --delete --exclude out --exclude build "$project/" "$run_dir/"
	echo "wasmSimd: off" >> "$run_dir/rive.yaml"
fi
logs=$(mktemp -d)

results=()
failed=0
for s in "${scenes[@]}"; do
	ref="$here/scenes/out/${s}_scalar_w1.bin"
	if [ ! -f "$ref" ]; then
		echo "missing $ref; record it with NATIVE=1 verify/traj.sh $s"
		failed=$((failed + 1))
		continue
	fi
	steps=$(node -e "const b=require('fs').readFileSync('$ref');console.log(new Uint32Array(b.buffer,b.byteOffset,16)[10])")
	define=$(scene_define "$s")
	for b in $backends; do
		if [ "$b" = 3 ]; then
			for p in $parts; do
				case $p in
					1) threads=--parallel-safe ;;
					default) threads= ;;
					*) threads=--job-threads=$((p - 1)) ;;
				esac
				label="$s P$p B$b T$tasks"
				log="$logs/run.log"
				OPTIONS="SCENE=$define STEPS=$steps LANES=1 BACKEND=$b TASKS=$tasks HASHLOG=1" \
					"$run_dir/run1.sh" 900 --bench=$((steps + 10)) $threads > "$log" 2>&1
				r=$(node "$here/threads.mjs" "$ref" "$log" "$label")
				status=$?
				size=$(sed -n 's/.*BENCH parallel size=\([0-9]*\).*/\1/p' "$log" | head -1)
				if [ "$p" != default ] && [ "$size" != "$p" ]; then
					r="RESULT $label 0 fail Parallel.size() is ${size:-missing}"
					status=1
				fi
				echo "$r"
				results+=("$r")
				[ $status -eq 0 ] || failed=$((failed + 1))
			done
			continue
		fi
		for l in $lanes; do
			for replica in 0 1; do
				[ $replica -eq 1 ] && [ "$l" != 1 ] && continue
				label="$s L$l B$b T$tasks"
				[ $replica -eq 1 ] && label="$label replica"
				log="$logs/run.log"
				OPTIONS="SCENE=$define STEPS=$steps LANES=$l BACKEND=$b TASKS=$tasks HASHLOG=1 REPLICA=$replica" \
					"$run_dir/run1.sh" 900 --bench=$((steps + 10)) --job-threads=8 --worker-threads=8 > "$log" 2>&1
				r=$(node "$here/threads.mjs" "$ref" "$log" "$label")
				status=$?
				echo "$r"
				results+=("$r")
				[ $status -eq 0 ] || failed=$((failed + 1))
			done
		done
	done
done
rm -rf "$logs"

echo
printf "%-34s %6s  %-5s %s\n" run steps pass "first divergence"
for r in "${results[@]}"; do
	set -- $r
	shift
	label="$1 $2 $3 $4"
	shift 4
	if [ "$1" = replica ]; then label="$label replica"; shift; fi
	steps=$1; verdict=$2; shift 2
	printf "%-34s %6s  %-5s %s\n" "$label" "$steps" "$verdict" "$*"
done
echo "passed: $(( ${#results[@]} - failed ))  failed: $failed"
[ $failed -eq 0 ]
