#!/usr/bin/env python3
# Per function frame cells (params + locals, i32/f32 1, i64/f64 2, v128 4) and the frame offset of the last v128
# local, as WAMR 2.4.5's fast interpreter lays them out; binaryen writes v128 locals after the scalar ones. A v128
# local at offset 256 or more traps the interpreter with "invalid local type" (notes/findings.md).
#   v128cells.py module.wasm [threshold]    lists functions with v128 locals; exits 1 if any reaches 256
import sys
import os
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "aotsym.py")).read().replace("\nmain()", "\n")
g = {"__name__": "x"}; exec(src, g)
leb = g["leb"]
data = open(sys.argv[1], 'rb').read()
imported, names = g["wasm_names"](sys.argv[1])
CELLS = {0x7f: 1, 0x7d: 1, 0x7e: 2, 0x7c: 2, 0x7b: 4, 0x70: 1, 0x6f: 1}
types = []; funcs = []
i = 8
while i < len(data):
    sid = data[i]; size, i = leb(data, i + 1); end = i + size
    if sid == 1:
        n, j = leb(data, i)
        for _ in range(n):
            assert data[j] == 0x60; j += 1
            pc, j = leb(data, j); ps = data[j:j + pc]; j += pc
            rc, j = leb(data, j); rs = data[j:j + rc]; j += rc
            types.append((ps, rs))
    elif sid == 3:
        n, j = leb(data, i)
        for _ in range(n):
            t, j = leb(data, j); funcs.append(t)
    elif sid == 10:
        n, j = leb(data, i)
        rows = []
        for k in range(n):
            bsize, j = leb(data, j); bend = j + bsize
            ng, q = leb(data, j)
            ps, rs = types[funcs[k]]
            cells = sum(CELLS[t] for t in ps); scal = cells; v = 0; nloc = 0
            for _ in range(ng):
                c, q = leb(data, q); t = data[q]; q += 1
                cells += c * CELLS[t]; nloc += c
                if t == 0x7b: v += c
                else: scal += c * CELLS[t]
            # binaryen groups v128 locals after scalars: last v128 offset = scal + 4 (v - 1)
            last = scal + 4 * (v - 1) if v else -1
            rows.append((last, cells, v, nloc, names.get(imported + k, "#%d" % k)))
            j = bend
        lim = int(sys.argv[2]) if len(sys.argv) > 2 else 200
        for r in sorted(rows, reverse=True)[:40]:
            if r[2] or r[0] >= lim:
                print("last v128 offset %5d  cells %5d  v128 %4d  locals %4d  %s" % r)
        if any(r[0] >= 256 for r in rows):
            print("error: a v128 local at frame offset 256 or more traps WAMR's fast interpreter")
            sys.exit(1)
    i = end
