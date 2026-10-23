#!/bin/bash
# Parity checks against the scalar C build.
#   verify/run.sh            every module, stops at the first failure
#   verify/run.sh table simd only those modules, in that order
# A module <m> is verify/cref/<m>.c, verify/as/<m>.as and verify/<m>.mjs.
# Optional verify/cref/<m>.flags and verify/as/<m>.flags add compiler arguments.
# KEEP_GOING=1 runs the rest after a failure. RASC_FLAGS replaces -O.

here="$(cd "$(dirname "$0")" && pwd)"
BOX3D=${BOX3D:-$HOME/Projects/box3d}
BOX3D_BUILD=${BOX3D_BUILD:-$BOX3D/build/scalar}
RASC=${RASC:-$HOME/Projects/rive-master/packages/rasc/animascript/out/release/rasc}
AS_STD=${AS_STD:-$HOME/Projects/rive-master/packages/rasc/animascript/std}
RASC_FLAGS=${RASC_FLAGS--O}
CC=${CC:-clang}
LIBBOX3D="$BOX3D_BUILD/src/libbox3d.a"

if [ ! -f "$LIBBOX3D" ]; then
  echo "missing $LIBBOX3D; configure $BOX3D_BUILD with BOX3D_DISABLE_SIMD=ON -DCMAKE_C_FLAGS=-ffp-contract=off and build it" >&2
  exit 2
fi

cd "$here"
mkdir -p out vectors

modules=("$@")
if [ ${#modules[@]} -eq 0 ]; then
  for c in cref/*.c; do
    m=$(basename "$c" .c)
    if [ -f "as/$m.as" ] && [ -f "$m.mjs" ]; then
      modules+=("$m")
    fi
  done
fi

passed=()
failed=()
summary=()

for m in "${modules[@]}"; do
  echo "== $m"
  cflags=()
  if [ -f "cref/$m.flags" ]; then read -r -a cflags < "cref/$m.flags"; fi
  aflags=()
  if [ -f "as/$m.flags" ]; then read -r -a aflags < "as/$m.flags"; fi

  ok=1
  if ! "$CC" -O2 -ffp-contract=off -DBOX3D_DISABLE_SIMD -DNDEBUG -std=gnu17 -w \
      -I "$BOX3D/include" -I "$BOX3D/src" -I cref "${cflags[@]}" \
      "cref/$m.c" "$LIBBOX3D" -lm -o "out/$m.cref"; then
    echo "$m: C driver failed to build"; ok=0
  elif ! "out/$m.cref" > "vectors/$m.txt"; then
    echo "$m: C driver failed"; ok=0
  elif ! "$RASC" "as/$m.as" --path "$AS_STD" --lib "$AS_STD/assembly" --runtime frame \
      $RASC_FLAGS "${aflags[@]}" -o "out/$m.wasm"; then
    echo "$m: rasc failed"; ok=0
  else
    log=$(node "$m.mjs" 2>&1)
    status=$?
    echo "$log" | grep -v '^RESULT '
    result=$(echo "$log" | grep '^RESULT ' | tail -1)
    [ -n "$result" ] && summary+=("$result")
    [ $status -eq 0 ] || ok=0
  fi

  if [ $ok -eq 1 ]; then
    passed+=("$m")
  else
    failed+=("$m")
    [ "${KEEP_GOING:-0}" = "1" ] || break
  fi
done

echo
echo "summary"
for r in "${summary[@]}"; do
  set -- $r
  printf "  %-24s %10s lines  %s mismatches\n" "$2" "$3" "$4"
done
echo "passed: ${#passed[@]}  failed: ${#failed[@]} ${failed[*]}"
[ ${#failed[@]} -eq 0 ]
