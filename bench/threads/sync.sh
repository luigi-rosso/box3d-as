#!/bin/sh
# Copies lib/ and the scene mirror into the project, since rive-cli resolves imports only inside it
HERE=$(cd "$(dirname "$0")" && pwd)
rsync -a --delete "$HERE/../../lib/" "$HERE/lib/"
mkdir -p "$HERE/scenes" && sed "s|\"../../lib/|\"../lib/|" "$HERE/../../verify/scenes/scenes.as" > "$HERE/scenes/scenes.as"
