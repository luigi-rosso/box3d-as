#!/bin/bash
# Builds the four single threaded web modules into bench/web/out:
#   c_simd, c_scalar     native Box3D through emscripten (Release, -O3, wasm SIMD or BOX3D_DISABLE_SIMD)
#   as_simd.wasm, as_scalar.wasm   the port through rasc -O2 (release bake: binaryen -O3 post pass)
# THREADS=1 adds c_simd_mt and c_scalar_mt, the pthreads builds.
# EMSDK names the emsdk root, WASM_OPT the binaryen 133 wasm-opt that rive-cli links.
set -e
here="$(cd "$(dirname "$0")" && pwd)"
root="$here/../.."
EMSDK=${EMSDK:?set EMSDK to an emsdk root}
WASM_OPT=${WASM_OPT:?set WASM_OPT to binaryen 133 wasm-opt}
RASC=${RASC:-$HOME/Projects/rive-master/packages/rasc/animascript/out/release/rasc}
AS_STD=${AS_STD:-$HOME/Projects/rive-master/packages/rasc/animascript/std}
out="$here/out"
mkdir -p "$out"

source "$EMSDK/emsdk_env.sh" > /dev/null 2>&1

c_build() {
	local name=$1
	shift
	local dir="$out/build_$name"
	emcmake cmake -S "$here/c" -B "$dir" -G Ninja -DCMAKE_BUILD_TYPE=Release "$@" > "$dir.log" 2>&1 ||
		{ cat "$dir.log"; exit 1; }
	cmake --build "$dir" >> "$dir.log" 2>&1 || { tail -30 "$dir.log"; exit 1; }
	cp "$dir/box3d_web.js" "$out/$name.mjs"
	cp "$dir/box3d_web.wasm" "$out/box3d_web_$name.wasm"
	# The glue names its wasm after the target, so each variant points at its own copy
	sed -i '' "s/box3d_web\.wasm/box3d_web_$name.wasm/g" "$out/$name.mjs"
}

c_build c_simd -DBOX3D_DISABLE_SIMD=OFF
c_build c_scalar -DBOX3D_DISABLE_SIMD=ON
if [ "${THREADS:-0}" = "1" ]; then
	c_build c_simd_mt -DBOX3D_DISABLE_SIMD=OFF -DWEB_THREADS=ON
	c_build c_scalar_mt -DBOX3D_DISABLE_SIMD=ON -DWEB_THREADS=ON
fi

# Release bake flags: optimized, no fuel (dangerouslyFast), throws stripped, frame collector
as_build() {
	local name=$1
	shift
	RASC_WASM_OPT="$WASM_OPT" "$RASC" "$here/as/web.as" --path "$AS_STD" --lib "$AS_STD/assembly" \
		--runtime frame -O2 --dangerously-fast --no-throw-messages "$@" -o "$out/$name.wasm"
}

as_build as_scalar
as_build as_simd --enable-simd
ls -la "$out"/*.wasm
