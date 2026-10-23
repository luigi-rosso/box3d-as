#!/bin/bash
# Collide-only parity for the contact driver: the scripted scene with b3Solve replaced by a no-op on
# both sides, so contacts, the broad-phase and sensors are compared without the solver in the loop.
# C: verify/cref/contact.c built with -DCONTACT_COLLIDE_ONLY. AS: a copy of lib whose b3Solve is
# renamed and replaced by an empty one. Run from anywhere: verify/contact_collide.sh

here="$(cd "$(dirname "$0")" && pwd)"
root="$(dirname "$here")"
BOX3D=${BOX3D:-$HOME/Projects/box3d}
BOX3D_BUILD=${BOX3D_BUILD:-$BOX3D/build/scalar}
RASC=${RASC:-$HOME/Projects/rive-master/packages/rasc/animascript/out/release/rasc}
AS_STD=${AS_STD:-$HOME/Projects/rive-master/packages/rasc/animascript/std}
RASC_FLAGS=${RASC_FLAGS--O}
CC=${CC:-clang}

set -e
cd "$here"
mkdir -p out vectors
work="out/contact_collide"
rm -rf "$work"
mkdir -p "$work/verify"
cp -R "$root/lib" "$work/lib"
cp -R as "$work/verify/as"

"$CC" -O2 -ffp-contract=off -DBOX3D_DISABLE_SIMD -DNDEBUG -DCONTACT_COLLIDE_ONLY -std=gnu17 -w \
  -I "$BOX3D/include" -I "$BOX3D/src" -I cref cref/contact.c "$BOX3D_BUILD/src/libbox3d.a" -lm -o out/contact_collide.cref
out/contact_collide.cref > vectors/contact_collide.txt

solver="$work/lib/solver.as"
if grep -q "export function b3Solve(" "$solver"; then
  sed -i.bak 's/export function b3Solve(/export function b3Solve_replaced(/' "$solver"
fi
printf '\nexport function b3Solve(world: b3World, stepContext: b3StepContext): void {}\n' >> "$solver"

"$RASC" "$work/verify/as/contact.as" --path "$AS_STD" --lib "$AS_STD/assembly" --runtime frame \
  $RASC_FLAGS -o out/contact_collide.wasm

WASM="$here/out/contact_collide.wasm" VECTORS="$here/vectors/contact_collide.txt" CONTACT_SCENES=4 node contact.mjs
