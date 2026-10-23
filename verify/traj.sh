#!/bin/bash
# End to end trajectory parity: builds verify/scenes/traj.as, runs each scene and diffs it against
# the native scalar recording in verify/scenes/out (1 worker).
#   verify/traj.sh                every scene except large_pyramid and many_bodies
#   verify/traj.sh --large        all scenes
#   verify/traj.sh chains mover   only those
# RASC_FLAGS replaces -O. WASM sets the driver path. NATIVE=1 re-records the native references first.

here="$(cd "$(dirname "$0")" && pwd)"
RASC=${RASC:-$HOME/Projects/rive-master/packages/rasc/animascript/out/release/rasc}
AS_STD=${AS_STD:-$HOME/Projects/rive-master/packages/rasc/animascript/std}
RASC_FLAGS=${RASC_FLAGS--O}
out="$here/scenes/out"
mkdir -p "$out/as"
wasm=${WASM:-$out/as/traj.wasm}

scenes=()
large=0
for a in "$@"; do
	if [ "$a" = "--large" ]; then large=1; else scenes+=("$a"); fi
done
if [ ${#scenes[@]} -eq 0 ]; then
	scenes=(pyramid card_house chains capsule_mesh heightfield bullets mover)
	[ $large -eq 1 ] && scenes+=(large_pyramid many_bodies)
fi

if [ "${NATIVE:-0}" = "1" ]; then
	"$here/scenes/build.sh" || exit 2
	for s in "${scenes[@]}"; do
		steps=300
		case $s in
			large_pyramid) steps=200 ;; heightfield) steps=400 ;; bullets) steps=150 ;; many_bodies) steps=100 ;;
		esac
		"$here/scenes/bin/traj_scalar" "$s" $steps 1 "$out/${s}_scalar_w1.bin" > /dev/null || exit 2
	done
fi

if ! "$RASC" "$here/scenes/traj.as" --path "$AS_STD" --lib "$AS_STD/assembly" --runtime frame \
		$RASC_FLAGS -o "$wasm"; then
	echo "rasc failed"
	exit 2
fi

results=()
failed=0
for s in "${scenes[@]}"; do
	echo "== $s"
	log=$(node "$here/traj.mjs" "$s" --wasm="$wasm" 2>&1)
	status=$?
	echo "$log" | grep -v '^RESULT '
	r=$(echo "$log" | grep '^RESULT ' | tail -1)
	[ -z "$r" ] && r="RESULT $s 0 fail node-exited-$status"
	results+=("$r")
	[ $status -eq 0 ] || failed=$((failed + 1))
done

echo
printf "%-14s %6s  %-5s %s\n" scene steps pass "first divergence"
for r in "${results[@]}"; do
	set -- $r
	name=$2; steps=$3; verdict=$4; shift 4
	printf "%-14s %6s  %-5s %s\n" "$name" "$steps" "$verdict" "$*"
done
echo "passed: $(( ${#results[@]} - failed ))  failed: $failed"
[ $failed -eq 0 ]
