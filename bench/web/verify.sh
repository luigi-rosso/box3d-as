#!/bin/bash
# Every scene on every implementation, each step's state hash against the native scalar recording
here="$(cd "$(dirname "$0")" && pwd)"
IMPLS=${IMPLS:-native_scalar native_simd c_scalar c_simd as_scalar as_simd}
WORKERS=${WORKERS:-1}
steps_of() {
	case $1 in
		large_pyramid) echo 200 ;; heightfield) echo 400 ;; bullets) echo 150 ;; many_bodies) echo 100 ;;
		mesh_drop | village) echo 120 ;; *) echo 300 ;;
	esac
}
failed=0
for s in ${SCENES:-pyramid large_pyramid card_house chains capsule_mesh heightfield bullets mover many_bodies plank_pile mesh_drop village queries}; do
	for i in $IMPLS; do
		for w in $WORKERS; do
			line=$(node "$here/run.mjs" "$i" "$s" "$(steps_of "$s")" --workers="$w" 2>&1 | tail -1)
			echo "$line" | python3 -c "
import json,sys
l=sys.stdin.read()
try:
    r=json.loads(l)
    print(f\"{r['scene']:14} {r['impl']:14} w{r['workers']} compared {r['compared']:4} {'pass' if r['mismatch']<0 and r['compared']>0 else 'FAIL at step '+str(r['mismatch'])} q {' '.join(r['queryHash'])}\")
except Exception:
    print('$s $i w$w ERROR', l[:300]); sys.exit(1)
" || failed=$((failed + 1))
		done
	done
done
echo "errors: $failed"
