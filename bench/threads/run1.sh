#!/bin/sh
# One headless AOT run of the threads bench with a watchdog: OPTIONS="SCENE=1 LANES=4" run1.sh <seconds> <rive args>
# OPTIONS goes into the options blob main.as reads, so runs that differ only in options share one compiled bake.
HERE=$(cd "$(dirname "$0")" && pwd)
RIVE=${RIVE:-$HOME/Projects/rive-master/packages/rive-cli/out/release/rive}
limit=$1
shift
printf '%s\n' "${OPTIONS:-}" > "$HERE/options.txt"
cd "$HERE" && RIVE_WAMRC=${RIVE_WAMRC:-$HOME/Projects/tools/wamrc} RIVE_WASM_AOT_SYNC=o3 \
	RIVE_AOT_CACHE_DIR=${RIVE_AOT_CACHE_DIR:-$HERE/out/aotcache} perl -e 'alarm shift; exec @ARGV' "$limit" "$RIVE" . --aot "$@" 2>&1
