#!/usr/bin/env python3
# Maps sampled PCs in WAMR AOT code back to wasm function names.
#   aotsym.py <module.wasm> <module.aot> <module.o> <region_start_hex> <samples.txt> [--split]
# module.o is the same wasm through wamrc --format=object with the ladder's flags; samples.txt has
# one sample per line: weight, then the backtrace PCs leaf first, all hex. --split reports a
# function's exported entry stub (aot_func#N, which checks the native stack) apart from its body.
import struct
import subprocess
import sys
from bisect import bisect_right
from collections import Counter


def leb(data, i):
    result = shift = 0
    while True:
        b = data[i]
        i += 1
        result |= (b & 0x7F) << shift
        shift += 7
        if b < 0x80:
            return result, i


def wasm_names(path):
    data = open(path, 'rb').read()
    i = 8
    imported = 0
    names = {}
    while i < len(data):
        sid = data[i]
        size, i = leb(data, i + 1)
        end = i + size
        if sid == 2:
            count, j = leb(data, i)
            for _ in range(count):
                n, j = leb(data, j)
                j += n
                n, j = leb(data, j)
                j += n
                kind = data[j]
                j += 1
                if kind == 0:
                    _, j = leb(data, j)
                    imported += 1
                elif kind == 1:
                    j += 1
                    flags, j = leb(data, j)
                    _, j = leb(data, j)
                    if flags & 1:
                        _, j = leb(data, j)
                elif kind == 2:
                    flags, j = leb(data, j)
                    _, j = leb(data, j)
                    if flags & 1:
                        _, j = leb(data, j)
                elif kind == 3:
                    j += 2
                else:
                    _, j = leb(data, j)
                    _, j = leb(data, j)
        elif sid == 0:
            n, j = leb(data, i)
            if data[j:j + n] == b'name':
                j += n
                while j < end:
                    sub = data[j]
                    ssize, j = leb(data, j + 1)
                    send = j + ssize
                    if sub == 1:
                        count, k = leb(data, j)
                        for _ in range(count):
                            idx, k = leb(data, k)
                            n, k = leb(data, k)
                            names[idx] = data[k:k + n].decode('utf-8', 'replace')
                            k += n
                    j = send
        i = end
    return imported, names


def object_symbols(path):
    out = subprocess.run(['llvm-nm', '-n', '-S', path], capture_output=True, text=True, check=True).stdout
    syms = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[3].startswith('aot_func'):
            syms.append((int(parts[0], 16), int(parts[1], 16), parts[3]))
    return syms


def object_text(path):
    out = subprocess.run(['llvm-readelf', '-S', '-W', path], capture_output=True, text=True, check=True).stdout
    for line in out.splitlines():
        if ' .text ' in line:
            f = line.split(']')[1].split()
            off, size = int(f[3], 16), int(f[4], 16)
            return open(path, 'rb').read()[off:off + size]
    raise SystemExit('no .text')


def literal_size(aot_path, text):
    aot = open(aot_path, 'rb').read()
    probe = 0x400
    hit = aot.find(text[probe:probe + 64])
    if hit < 0:
        raise SystemExit('text not found in aot')
    code = hit - probe
    for p in range(code - 4, max(code - 65536, 0), -1):
        if struct.unpack_from('<I', aot, p)[0] == code - p - 4:
            return code - p - 4
    raise SystemExit('literal size not found')


def main():
    wasm, aot, obj, region, samples = sys.argv[1:6]
    split = len(sys.argv) > 6 and sys.argv[6] == '--split'
    imported, names = wasm_names(wasm)
    syms = object_symbols(obj)
    text = object_text(obj)
    lit = literal_size(aot, text)
    base = int(region, 16) + 4 + lit
    starts = [s[0] for s in syms]

    def name_of(pc):
        off = pc - base
        if off < 0 or off >= len(text):
            return None
        k = bisect_right(starts, off) - 1
        if k < 0:
            return None
        start, size, sym = syms[k]
        idx = int(sym.split('#')[1])
        name = names.get(imported + idx, sym)
        return name + ' [entry]' if split and not sym.startswith('aot_func_internal') else name

    self_w = Counter()
    total_w = Counter()
    script = 0
    total = 0
    for line in open(samples):
        parts = line.split()
        if not parts:
            continue
        weight = int(parts[0])
        pcs = [int(p, 16) for p in parts[1:]]
        total += weight
        frames = [name_of(pc) for pc in pcs]
        if not any(frames):
            continue
        script += weight
        leaf = frames[0] if frames[0] else 'native:' + parts[1]
        self_w[leaf] += weight
        for f in set(f for f in frames if f):
            total_w[f] += weight
    print(f'literal {lit} code base 0x{base:x} samples {total} with script frames {script}')
    for name, w in self_w.most_common(40):
        print(f'{w:7d} {100.0 * w / script:6.2f}% self {100.0 * total_w.get(name, w) / script:6.2f}% total  {name}')


main()
