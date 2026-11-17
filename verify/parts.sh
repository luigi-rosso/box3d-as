#!/bin/bash
# What verify/threads.sh cannot see of the parallel calls backend, each held on a copy of bench/threads with
# lib/solver.as edited:
#   workers  part 0 runs no block of a stage it published and the others never leave, so a step ends only if they
#            can run every block
#   syncs    part 0 asks every part to sync after 8 polls of a stage wait, not 2^24, so the requests run
#   trap     a worker traps inside a block from step 20 on, and the call has to end with an error, not hang
#   trap0    part 0 traps after it published a stage in step 20, the same
# workers and syncs must stay bit exact against the native recording at 2, 4 and 8 parts (PARTS, none of them 1), and
# syncs must have synced.
#   verify/parts.sh [scene...]    pyramid and large_pyramid by default
# A scene needs stages of more than one block, as those two have: a small one publishes none, so it makes no sync
# and reaches no trap. A hang fails at bench/threads/run1.sh's alarm. It runs both tasks on the SIMD artifact. It
# does not hold speed: workers that leave too soon, that are slow to answer a sync, or a trap that takes minutes to
# end the call, pass.
here="$(cd "$(dirname "$0")" && pwd)"
project="$here/../bench/threads"
# The scalar artifact runs from another directory, where the checks below would not find its log, and the edits
# are to the solver's task
unset WASM_SIMD TASKS
scenes=("$@")
[ ${#scenes[@]} -gt 0 ] || scenes=(pyramid large_pyramid)
"$project/sync.sh"
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

# variant <name>: the copy with that edit, refused when solver.as no longer has the lines it replaces
variant() {
	rsync -a --exclude out --exclude build "$project/" "$work/$1/"
	python3 - "$work/$1/lib/solver.as" "$1" <<'PY'
import sys
path, name = sys.argv[1:]
trap = '''if (Parallel.inPart() && %s) {
				unreachable();
			}
'''
edits = {
	# The workers never leave either, since no step can end without them
	'workers': [
		('''		b3ExecuteStage(stage, context, previousSyncIndex, syncIndex, workerIndex);

		// The span is this worker's own blocks, not the wait for the others
''', '''		if (Parallel.inPart() == false) {
			b3ExecuteStage(stage, context, previousSyncIndex, syncIndex, workerIndex);
		}

		// The span is this worker's own blocks, not the wait for the others
'''),
		('const B3_WORKER_IDLE_SLEEPS: i32 = 500;', 'const B3_WORKER_IDLE_SLEEPS: i32 = 1 << 30;'),
	],
	'syncs': [('const B3_STAGE_SYNC_POLLS: i32 = 1 << 24;', 'const B3_STAGE_SYNC_POLLS: i32 = 8;')],
	'trap': [('''			b3ExecuteBlock(stage, context, blocks[blockIndex].block, workerIndex);
''', '			' + trap % 'workerIndex != 0 && context.world.stepIndex >= 20' + '''			b3ExecuteBlock(stage, context, blocks[blockIndex].block, workerIndex);
''')],
	'trap0': [('''		b3PublishSyncBits(context, syncBits);

		let syncIndex = <i32>((syncBits >> 16) & 0xffff);
''', '''		b3PublishSyncBits(context, syncBits);
		''' + trap.replace('\t\t\t', '\t\t') % 'context.world.stepIndex == 20' + '''
		let syncIndex = <i32>((syncBits >> 16) & 0xffff);
''')],
}
source = open(path).read()
for old, new in edits[name]:
	if source.count(old) != 1:
		sys.exit(name + ': lib/solver.as no longer has\n' + old)
	source = source.replace(old, new)
open(path, 'w').write(source)
PY
}

failed=0
for v in workers syncs; do
	variant $v || exit 2
	PROJECT="$work/$v" BACKENDS=3 PARTS="${PARTS:-2 4 8}" "$here/threads.sh" "${scenes[@]}" > "$work/$v.out" || failed=$((failed + 1))
	grep "^RESULT" "$work/$v.out" | sed "s/^RESULT/$v/"
done
# The last run's log
synced=$(sed -n 's/.* parStageSyncs=\([0-9]*\).*/\1/p' "$work/syncs/build/rive.log" | tail -1)
if [ "${synced:-0}" -gt 0 ]; then
	echo "syncs made $synced stage syncs in its last run"
else
	echo "syncs fail: no stage sync was made"
	failed=$((failed + 1))
fi

for v in trap trap0; do
	variant $v || exit 2
	r=$(PROJECT="$work/$v" BACKENDS=3 PARTS=4 "$here/threads.sh" "${scenes[0]}" | grep "^RESULT")
	case "$r" in
		*trapped*) echo "$v pass: the call ended with the trap" ;;
		*) echo "$v fail: ${r#RESULT }"; failed=$((failed + 1)) ;;
	esac
done
echo "failed: $failed"
[ $failed -eq 0 ]
