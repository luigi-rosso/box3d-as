#!/bin/sh
# Bakes bench/threads into out/lanes/<name>.riv with the options blob holding the rest of the arguments:
#   bench/web/lanes_bake.sh lp_w4 SCENE=1 STEPS=200 LANES=4 TASKS=2 BACKEND=2 IDLEFRAMES=30
set -e
here="$(cd "$(dirname "$0")" && pwd)"
RIVE=${RIVE:-$HOME/Projects/rive-master/packages/rive-cli/out/release/rive}
name=$1
shift
project="$here/../threads"
"$project/sync.sh"
printf '%s\n' "$*" > "$project/options.txt"
mkdir -p "$here/out/lanes"
"$RIVE" "$project" --unsigned-riv --release > "$here/out/lanes/$name.bake.log" 2>&1 || { tail -20 "$here/out/lanes/$name.bake.log"; exit 1; }
cp "$project/build/box3d_threads.riv" "$here/out/lanes/$name.riv"
printf '\n' > "$project/options.txt"
