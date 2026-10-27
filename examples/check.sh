#!/bin/bash
# Runs every example through examples/tests.as with rive --test, in a project copy that holds lib/ beside
# examples/, since rive-cli resolves imports only inside the project.
#   examples/check.sh          interpreted
#   examples/check.sh --aot    compiled ahead of time, with RIVE_WAMRC
# RIVE picks the CLI; any other arguments go to it. PARALLEL=1 also copies lib/parallel/, which needs the
# parallel-calls toolchain.
here="$(cd "$(dirname "$0")" && pwd)"
RIVE=${RIVE:-$HOME/Projects/rive-master/packages/rive-cli/out/release/rive}
project=$(mktemp -d)
trap 'rm -rf "$project"' EXIT

mkdir -p "$project/examples"
skip="--exclude /parallel/"
[ "${PARALLEL:-0}" = 1 ] && skip=
rsync -a $skip "$here/../lib/" "$project/lib/"
cp "$here"/*.as "$project/examples/"
cat > "$project/rive.yaml" << 'EOF'
name: box3d_examples
scripting: wasm
EOF

perl -e 'alarm shift; exec @ARGV' 600 "$RIVE" "$project" --test "$@" 2>&1 | grep -v "warning"
exit "${PIPESTATUS[0]}"
