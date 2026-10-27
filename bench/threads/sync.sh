#!/bin/sh
# Copies lib/ and the scene mirror into the project, since rive-cli resolves imports only inside it. lib/parallel/,
# the parallel calls backend, comes too with PARALLEL=1; it needs the parallel-calls toolchain.
HERE=$(cd "$(dirname "$0")" && pwd)
skip="--exclude /parallel/ --delete-excluded"
[ "${PARALLEL:-0}" = 1 ] && skip=
rsync -a --delete $skip "$HERE/../../lib/" "$HERE/lib/"
mkdir -p "$HERE/scenes" && sed "s|\"../../lib/|\"../lib/|" "$HERE/../../verify/scenes/scenes.as" > "$HERE/scenes/scenes.as"
