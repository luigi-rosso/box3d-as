#!/bin/sh
# Time Profiler over the AOT bench, symbolized to wasm functions by tools/aotsym.py.
#   bench/as/profile.sh [scene] [steps] [seconds]    scene 1 large_pyramid, 2 many_bodies
# The trace bundle records the environment, so it stays in OUT (default a temp dir).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
RIVE=${RIVE:-$HOME/Projects/rive-master/packages/rive-cli/out/release/rive}
WAMRC=${RIVE_WAMRC:-$HOME/Projects/tools/wamrc}
SCENE=${1:-1}
STEPS=${2:-300}
SECONDS_=${3:-15}
OUT=${OUT:-$(mktemp -d)}
CACHE=${RIVE_AOT_CACHE_DIR:-$OUT/aotcache}
mkdir -p "$OUT" "$CACHE"
rsync -a --delete "$HERE/../../lib/" "$HERE/lib/"
sed 's|"\.\./\.\./lib/|"./lib/|' "$HERE/../../verify/scenes/scenes.as" > "$HERE/scenes.as"

cd "$HERE"
RIVE_RASC_NAMES=1 RIVE_WAMRC="$WAMRC" RIVE_WASM_AOT_SYNC=o3 RIVE_AOT_CACHE_DIR="$CACHE" \
	"$RIVE" . --aot --bench=$((STEPS + 10)) --define=SCENE="$SCENE" --define=STEPS="$STEPS" > "$OUT/run.log" 2>&1 &
i=0
until grep -q "BENCH start" "$OUT/run.log" 2>/dev/null; do
	i=$((i + 1))
	[ $i -gt 600 ] && { echo "bench never started"; cat "$OUT/run.log"; exit 1; }
	sleep 0.5
done
PID=$(pgrep -n -f "bench=$((STEPS + 10)) --define=SCENE=$SCENE")
vmmap -wide "$PID" > "$OUT/vmmap.txt"
REGION=$(grep -E '^VM_ALLOCATE .*\] rwx/rwx' "$OUT/vmmap.txt" | head -1 | awk '{split($2, r, "-"); print r[1]}')
rm -rf "$OUT/profile.trace"
xctrace record --template 'Time Profiler' --attach "$PID" --time-limit "${SECONDS_}s" --output "$OUT/profile.trace" > /dev/null
wait
grep "BENCH RESULT" "$OUT/run.log"

AOT=$(ls -t "$CACHE"/*/*.aot | head -1)
WASM=${AOT%.aot}.wasm
"$WAMRC" --bounds-checks=1 --opt-level=3 --cpu=generic --cpu-features=+reserve-x18 --tune-cpu=apple-m1 \
	--format=object -o "$OUT/module.o" "$WASM" > /dev/null
"$WAMRC" --bounds-checks=1 --opt-level=3 --cpu=generic --cpu-features=+reserve-x18 --tune-cpu=apple-m1 \
	-o "$OUT/check.aot" "$WASM" > /dev/null
cmp -s "$OUT/check.aot" "$AOT" || echo "warning: wamrc flags differ from the ladder's, symbols may be off"
xctrace export --input "$OUT/profile.trace" \
	--xpath '/trace-toc/run[@number="1"]/data/table[@schema="time-profile"]' > "$OUT/time_profile.xml"
python3 "$HERE/tools/xctrace_samples.py" "$OUT/time_profile.xml" > "$OUT/samples.txt"
python3 "$HERE/tools/aotsym.py" "$WASM" "$AOT" "$OUT/module.o" "$REGION" "$OUT/samples.txt" | tee "$OUT/profile.txt"
echo "out: $OUT"
