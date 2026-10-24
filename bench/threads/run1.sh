#!/bin/sh
# One headless AOT run of the threads bench with a watchdog: run1.sh <seconds> <rive args...>
HERE=$(cd "$(dirname "$0")" && pwd)
RIVE=${RIVE:-$HOME/Projects/rive-master/packages/rive-cli/out/release/rive}
limit=$1
shift
cd "$HERE" && RIVE_WAMRC=${RIVE_WAMRC:-$HOME/Projects/tools/wamrc} RIVE_WASM_AOT_SYNC=o3 \
	RIVE_AOT_CACHE_DIR=${RIVE_AOT_CACHE_DIR:-$HERE/out/aotcache} perl -e 'alarm shift; exec @ARGV' "$limit" "$RIVE" . --aot "$@" 2>&1
